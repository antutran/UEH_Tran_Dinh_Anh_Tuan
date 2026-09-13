#!/usr/bin/env python3
"""Traffic light detection node for UEH CRC 2026.

Subscribes to: /camera/image_raw
Publishes to:  /light/state  (std_msgs/String)
               Values: "RED" | "YELLOW" | "GREEN" | "NONE"

IMPORTANT: We do NOT subscribe to /traffic_lights, /traffic_light/*,
or /automobile/semaphores. Detection is purely vision-based.

Key design decisions to avoid false positives:
  1. Look only in the UPPER portion of the image (traffic light is mounted at
     ~0.32 m height, appears in the top third at close range).
  2. Require a circular/disk shape (aspect ratio near 1, circularity > 0.6).
  3. Apply temporal filtering: a state must persist for N frames before
     we publish it.
  4. Distinguish from STOP signs: STOP signs appear LOWER in the frame
     and have large rectangular shapes, not small circles.
  5. Distinguish from highway signs (green): highway signs are rectangular,
     not circular.

The traffic light has three lamps stacked vertically.
The active lamp is a sphere (r=0.017 m) with emissive Gazebo material.
In the camera it appears as a bright, saturated coloured disk.
"""

import signal
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

# HSV ranges for traffic light colours (bright emissive spheres)
LIGHT_RED_LOW1  = np.array([0,   150, 100], dtype=np.uint8)
LIGHT_RED_HIGH1 = np.array([12,  255, 255], dtype=np.uint8)
LIGHT_RED_LOW2  = np.array([168, 150, 100], dtype=np.uint8)
LIGHT_RED_HIGH2 = np.array([180, 255, 255], dtype=np.uint8)

LIGHT_YELLOW_LOW  = np.array([18,  150, 100], dtype=np.uint8)
LIGHT_YELLOW_HIGH = np.array([35,  255, 255], dtype=np.uint8)

LIGHT_GREEN_LOW  = np.array([45,  150,  80], dtype=np.uint8)
LIGHT_GREEN_HIGH = np.array([90,  255, 255], dtype=np.uint8)

# Minimum circularity to accept a blob as a traffic light lamp
MIN_CIRCULARITY = 0.55


def catch_sigterm():
    stopping = {'now': False}
    signal.signal(signal.SIGTERM, lambda *_: stopping.update(now=True))
    return stopping


class LightNode(Node):

    def __init__(self):
        super().__init__('light_node')

        self.declare_parameter('light_min_radius',      6)
        self.declare_parameter('light_max_radius',     30)
        self.declare_parameter('light_confirm_frames',  5)
        self.declare_parameter('light_roi_bottom_frac', 0.50)
        self.declare_parameter('publish_debug',        False)

        self.min_r         = int(self.get_parameter('light_min_radius').value)
        self.max_r         = int(self.get_parameter('light_max_radius').value)
        self.confirm_n     = int(self.get_parameter('light_confirm_frames').value)
        self.roi_bottom    = self.get_parameter('light_roi_bottom_frac').value
        self.publish_debug = self.get_parameter('publish_debug').value

        self.bridge      = CvBridge()
        self.latest_img  = None

        # Temporal filter: ring buffer of raw detections
        self.history = deque(maxlen=self.confirm_n)
        self.confirmed_state = 'NONE'

        self.pub_light = self.create_publisher(String, '/light/state', 10)
        if self.publish_debug:
            self.pub_debug = self.create_publisher(
                Image, '/light/debug', qos_profile_sensor_data)

        self.create_subscription(
            Image, '/camera/image_raw',
            self._on_image, qos_profile_sensor_data)

        self.create_timer(0.05, self._tick)
        self.get_logger().info('light_node ready')

    def _on_image(self, msg):
        try:
            self.latest_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'light img convert: {e}')

    def _tick(self):
        if self.latest_img is None:
            return
        raw = self._detect_raw(self.latest_img)
        self.history.append(raw)

        # Confirmed state: most common in history buffer if unanimous/majority
        if len(self.history) == self.confirm_n:
            counts = {}
            for s in self.history:
                counts[s] = counts.get(s, 0) + 1
            best = max(counts, key=counts.get)
            # Require at least half the history to agree
            if counts[best] >= self.confirm_n // 2 + 1:
                self.confirmed_state = best

        msg = String()
        msg.data = self.confirmed_state
        self.pub_light.publish(msg)

    # ---------------------------------------------------------------------- #
    def _detect_raw(self, img):
        """Return raw colour string for this frame: RED|YELLOW|GREEN|NONE."""
        h, w = img.shape[:2]

        # Only look in the upper portion (traffic light appears there)
        roi_h = int(h * self.roi_bottom)
        roi   = img[:roi_h, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        scores = {}

        for colour, ranges in [
            ('RED',    [(LIGHT_RED_LOW1, LIGHT_RED_HIGH1),
                        (LIGHT_RED_LOW2, LIGHT_RED_HIGH2)]),
            ('YELLOW', [(LIGHT_YELLOW_LOW, LIGHT_YELLOW_HIGH)]),
            ('GREEN',  [(LIGHT_GREEN_LOW,  LIGHT_GREEN_HIGH)]),
        ]:
            mask = np.zeros(roi.shape[:2], dtype=np.uint8)
            for lo, hi in ranges:
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))

            # Small morphological cleanup
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            best = 0.0
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 1:
                    continue
                peri = cv2.arcLength(cnt, True)
                if peri < 1:
                    continue

                # Radius check
                (cx, cy), radius = cv2.minEnclosingCircle(cnt)
                if not (self.min_r <= radius <= self.max_r):
                    continue

                # Circularity: 4π·area / perimeter²
                circ = 4.0 * np.pi * area / (peri * peri)
                if circ < MIN_CIRCULARITY:
                    continue

                # Area check (π·r² expected for a filled circle)
                fill = area / (np.pi * radius * radius)
                if fill < 0.45:
                    continue

                score = area * circ * fill
                best  = max(best, score)

            if best > 0:
                scores[colour] = best

        if not scores:
            return 'NONE'
        return max(scores, key=scores.get)


# ============================================================================ #
def spin(node, stopping):
    while rclpy.ok() and not stopping['now']:
        try:
            rclpy.spin_once(node, timeout_sec=0.05)
        except Exception:
            if stopping['now'] or not rclpy.ok():
                break
            raise


def main(args=None):
    rclpy.init(args=args)
    stopping = catch_sigterm()
    node = LightNode()
    try:
        spin(node, stopping)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
