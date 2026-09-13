#!/usr/bin/env python3
"""Traffic sign detection node for UEH CRC 2026.

Subscribes to: /camera/image_raw
Publishes to:  /sign/detection  (std_msgs/String)
               Values: "STOP" | "crosswalk" | "ramp" | "tunnel" | "none"

The graded track uses different sign positions, so we do NOT use coordinates.
Detection is entirely image-based.

STOP sign detection:
  - Red octagon in lower 70% of image
  - Red HSV mask → contour → check shape (area, aspect, hull convexity)
  - Confirm with white interior region (the word STOP)

Other signs (awareness only, do not trigger stops):
  - crosswalk: blue square with white pedestrian figure
  - ramp: red triangle with arch shape
  - tunnel: blue square with white arch shape
"""

import signal

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

# HSV colour ranges
RED_LOW1  = np.array([0,   120, 70],  dtype=np.uint8)
RED_HIGH1 = np.array([10,  255, 255], dtype=np.uint8)
RED_LOW2  = np.array([170, 120, 70],  dtype=np.uint8)
RED_HIGH2 = np.array([180, 255, 255], dtype=np.uint8)

BLUE_LOW  = np.array([100, 80,  80],  dtype=np.uint8)
BLUE_HIGH = np.array([135, 255, 255], dtype=np.uint8)


def catch_sigterm():
    stopping = {'now': False}
    signal.signal(signal.SIGTERM, lambda *_: stopping.update(now=True))
    return stopping


class SignNode(Node):

    def __init__(self):
        super().__init__('sign_node')

        self.declare_parameter('stop_min_area', 350)
        self.declare_parameter('publish_debug', False)

        self.stop_min_area   = int(self.get_parameter('stop_min_area').value)
        self.publish_debug   = self.get_parameter('publish_debug').value

        self.bridge     = CvBridge()
        self.latest_img = None

        self.pub_sign = self.create_publisher(String, '/sign/detection', 10)
        if self.publish_debug:
            self.pub_debug = self.create_publisher(
                Image, '/sign/debug', qos_profile_sensor_data)

        self.create_subscription(
            Image, '/camera/image_raw',
            self._on_image, qos_profile_sensor_data)

        self.create_timer(0.05, self._tick)   # 20 Hz
        self.get_logger().info('sign_node ready')

    def _on_image(self, msg):
        try:
            self.latest_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'sign img convert: {e}')

    def _tick(self):
        if self.latest_img is None:
            return
        sign = self._detect(self.latest_img)
        msg  = String()
        msg.data = sign
        self.pub_sign.publish(msg)

    # ---------------------------------------------------------------------- #
    def _detect(self, img):
        """Return the name of the most confident sign detected, or 'none'."""
        h, w = img.shape[:2]

        # Work in the lower 70% (signs are close to the ground)
        roi_y = int(h * 0.30)
        roi   = img[roi_y:, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # ---- STOP sign (red octagon) ----------------------------------------
        red1  = cv2.inRange(hsv, RED_LOW1,  RED_HIGH1)
        red2  = cv2.inRange(hsv, RED_LOW2,  RED_HIGH2)
        red   = cv2.bitwise_or(red1, red2)

        # Morphological cleanup
        k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        red = cv2.morphologyEx(red, cv2.MORPH_OPEN,  k3)
        red = cv2.morphologyEx(red, cv2.MORPH_CLOSE, k3)

        contours, _ = cv2.findContours(red, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        best_stop_score = 0.0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.stop_min_area:
                continue
            x, y, cw, ch = cv2.boundingRect(cnt)
            aspect = cw / max(ch, 1)
            if not (0.60 < aspect < 1.65):
                continue  # must be roughly square (octagon bounding box)

            # Convexity check: hull / contour area ratio (octagon ~= convex)
            hull = cv2.convexHull(cnt)
            hull_area = cv2.contourArea(hull)
            if hull_area < 1:
                continue
            convexity = area / hull_area
            if convexity < 0.70:
                continue  # not convex enough for a STOP sign

            # Check that there is a white region inside (the word STOP)
            mask_roi = np.zeros(roi.shape[:2], dtype=np.uint8)
            cv2.drawContours(mask_roi, [cnt], -1, 255, -1)
            inner = cv2.bitwise_and(
                cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY), mask_roi)
            white_frac = np.sum(inner > 180) / max(area, 1)
            if white_frac < 0.08:
                continue  # interior not white enough

            score = area * convexity * white_frac
            if score > best_stop_score:
                best_stop_score = score

        if best_stop_score > 0:
            return 'STOP'

        # ---- Crosswalk sign (blue square) -----------------------------------
        blue = cv2.inRange(hsv, BLUE_LOW, BLUE_HIGH)
        blue = cv2.morphologyEx(blue, cv2.MORPH_OPEN,  k3)
        b_contours, _ = cv2.findContours(blue, cv2.RETR_EXTERNAL,
                                         cv2.CHAIN_APPROX_SIMPLE)
        for cnt in b_contours:
            area = cv2.contourArea(cnt)
            if area < 200:
                continue
            x, y, cw, ch = cv2.boundingRect(cnt)
            aspect = cw / max(ch, 1)
            if 0.7 < aspect < 1.4:
                return 'crosswalk'

        return 'none'


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
    node = SignNode()
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
