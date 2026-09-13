#!/usr/bin/env python3
"""Pedestrian detection node for UEH CRC 2026.

Subscribes to:
  /scan                    (sensor_msgs/LaserScan)
  /camera/image_raw        (sensor_msgs/Image)

Publishes to:
  /pedestrian/blocking     (std_msgs/Bool)   True if pedestrian is in the road

Strategy:
  1. LiDAR: monitor a LEFT lateral sector (the pedestrian crosses from the
     right kerb toward the robot's left, relative to the robot heading +X).
     The pedestrian model has a collision cylinder r=0.03 m height=0.17 m;
     the robot LiDAR is at z=0.132 m so the beam intersects the cylinder.

  2. Camera: detect the purple/indigo coloured pedestrian model.
     The model is intentionally purple/indigo so it does not trigger
     traffic light or STOP sign detectors.

Both signals must agree (AND logic) before we declare blocking=True.
This avoids false positives from tunnel walls or road paint.
"""

import math
import signal
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool

# Purple/Indigo HSV range
PURPLE_LOW  = np.array([120, 60,  40], dtype=np.uint8)
PURPLE_HIGH = np.array([165, 255, 255], dtype=np.uint8)


def catch_sigterm():
    stopping = {'now': False}
    signal.signal(signal.SIGTERM, lambda *_: stopping.update(now=True))
    return stopping


class PedestrianDetectNode(Node):

    def __init__(self):
        super().__init__('ped_node')

        self.declare_parameter('ped_lidar_sector_lo',  60)    # deg left
        self.declare_parameter('ped_lidar_sector_hi', 110)    # deg left
        self.declare_parameter('ped_lidar_max_dist',   1.20)  # m
        self.declare_parameter('ped_camera_min_area',  80)    # px²
        self.declare_parameter('ped_stop_hysteresis',  0.40)  # m

        self.sector_lo   = float(self.get_parameter('ped_lidar_sector_lo').value)
        self.sector_hi   = float(self.get_parameter('ped_lidar_sector_hi').value)
        self.lidar_max   = float(self.get_parameter('ped_lidar_max_dist').value)
        self.cam_min_area = int(self.get_parameter('ped_camera_min_area').value)
        self.hysteresis  = float(self.get_parameter('ped_stop_hysteresis').value)

        self.bridge      = CvBridge()
        self.latest_img  = None
        self.latest_scan = None

        # State: once blocking, require pedestrian to clear + hysteresis
        self.currently_blocking = False

        # History buffer for camera detections (temporal filter)
        self.cam_history = deque(maxlen=4)

        self.pub_blocking = self.create_publisher(Bool, '/pedestrian/blocking', 10)

        self.create_subscription(
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            Image, '/camera/image_raw', self._on_image, qos_profile_sensor_data)

        self.create_timer(0.05, self._tick)
        self.get_logger().info('ped_node ready')

    def _on_scan(self, msg):
        self.latest_scan = msg

    def _on_image(self, msg):
        try:
            self.latest_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'ped img convert: {e}')

    def _tick(self):
        lidar_see = self._lidar_sees_pedestrian()
        cam_see   = self._camera_sees_pedestrian()

        # Require both to agree to set blocking True
        # Require both to be clear (plus hysteresis) to set blocking False
        if lidar_see and cam_see:
            self.currently_blocking = True
        elif not lidar_see and not cam_see:
            self.currently_blocking = False
        # If only one agrees: maintain previous state (hysteresis)

        msg      = Bool()
        msg.data = self.currently_blocking
        self.pub_blocking.publish(msg)

    # ---------------------------------------------------------------------- #
    def _lidar_sees_pedestrian(self):
        """True if a close object is detected in the lateral pedestrian sector."""
        if self.latest_scan is None:
            return False

        msg = self.latest_scan
        n   = len(msg.ranges)
        if n == 0:
            return False

        # Convert degree bounds to index range
        # In ROS LiDAR convention: 0° = front, 90° = left, 270° = right
        lo_idx = int(round(self.sector_lo * n / 360.0)) % n
        hi_idx = int(round(self.sector_hi * n / 360.0)) % n

        # Collect ranges in that sector
        min_r = float('inf')
        for i in range(lo_idx, hi_idx + 1):
            r = msg.ranges[i % n]
            if math.isfinite(r) and msg.range_min < r < self.lidar_max:
                min_r = min(min_r, r)

        return min_r < self.lidar_max

    # ---------------------------------------------------------------------- #
    def _camera_sees_pedestrian(self):
        """True if a purple/indigo blob of sufficient size is visible."""
        if self.latest_img is None:
            return False

        img = self.latest_img
        h   = img.shape[0]

        # Pedestrian appears in mid-to-lower image (it's on the ground)
        roi = img[h // 4:, :]

        hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, PURPLE_LOW, PURPLE_HIGH)

        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            if cv2.contourArea(cnt) >= self.cam_min_area:
                self.cam_history.append(True)
                return True

        self.cam_history.append(False)
        # Require at least 2 out of last 4 frames to have seen it
        return sum(self.cam_history) >= 2


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
    node = PedestrianDetectNode()
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
