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

# Purple/Indigo HSV range - lower bound Hue 125 to reject pure blue (Hue 100-120)
PURPLE_LOW  = np.array([125, 60,  40], dtype=np.uint8)
PURPLE_HIGH = np.array([165, 255, 255], dtype=np.uint8)


def catch_sigterm():
    stopping = {'now': False}
    signal.signal(signal.SIGTERM, lambda *_: stopping.update(now=True))
    return stopping


class PedestrianDetectNode(Node):

    def __init__(self):
        super().__init__('ped_node')

        self.declare_parameter('enable_pedestrian',    True)
        self.declare_parameter('ped_lidar_sector_lo',  60.0)  # deg left
        self.declare_parameter('ped_lidar_sector_hi', 110.0)  # deg left
        self.declare_parameter('ped_lidar_max_dist',   1.20)  # m
        self.declare_parameter('ped_lidar_min_points', 2)     # min valid returns
        self.declare_parameter('ped_camera_min_area',  100)   # px²
        self.declare_parameter('ped_cam_confirm_min',  3)     # frames required
        self.declare_parameter('ped_cam_history_len',  5)     # buffer length

        self.enable_pedestrian = bool(self.get_parameter('enable_pedestrian').value)
        self.sector_lo         = float(self.get_parameter('ped_lidar_sector_lo').value)
        self.sector_hi         = float(self.get_parameter('ped_lidar_sector_hi').value)
        self.lidar_max         = float(self.get_parameter('ped_lidar_max_dist').value)
        self.lidar_min_points  = int(self.get_parameter('ped_lidar_min_points').value)
        self.cam_min_area      = int(self.get_parameter('ped_camera_min_area').value)
        self.cam_confirm_min   = int(self.get_parameter('ped_cam_confirm_min').value)
        history_len            = int(self.get_parameter('ped_cam_history_len').value)

        self.bridge      = CvBridge()
        self.latest_img  = None
        self.latest_scan = None

        # State: detector defaults strictly to FALSE unless positive evidence
        self.currently_blocking = False

        # History buffer for camera detections (temporal filter)
        self.cam_history = deque(maxlen=history_len)

        self.pub_blocking = self.create_publisher(Bool, '/pedestrian/blocking', 10)

        self.create_subscription(
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            Image, '/camera/image_raw', self._on_image, qos_profile_sensor_data)

        self.create_timer(0.05, self._tick)
        self.get_logger().info(
            f'ped_node ready | enable={self.enable_pedestrian} | '
            f'sector=[{self.sector_lo:.1f}, {self.sector_hi:.1f}] | max_dist={self.lidar_max:.2f}m')

    def _on_scan(self, msg):
        self.latest_scan = msg

    def _on_image(self, msg):
        try:
            self.latest_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'ped img convert: {e}')

    def _tick(self):
        # When disabled by parameter, always publish False and return immediately
        if not self.enable_pedestrian:
            self.currently_blocking = False
            msg = Bool()
            msg.data = False
            self.pub_blocking.publish(msg)
            return

        lidar_see = self._lidar_sees_pedestrian()
        cam_see   = self._camera_sees_pedestrian()

        # Strict AND logic:
        # 1. Detector defaults to FALSE unless there is positive evidence.
        # 2. Camera purple blob alone MUST NOT be enough to stop the robot.
        # 3. Non-finite LiDAR values (inf/nan/no returns) evaluate to lidar_see=False.
        # 4. BOTH camera AND LiDAR confirmation are required simultaneously.
        self.currently_blocking = bool(lidar_see and cam_see)

        msg      = Bool()
        msg.data = self.currently_blocking
        self.pub_blocking.publish(msg)

    # ---------------------------------------------------------------------- #
    @staticmethod
    def _in_sector(angle_deg, lo, hi):
        """Check if angle in degrees [0, 360) falls within [lo, hi]. Handles wrap-around."""
        angle = angle_deg % 360.0
        lo_norm = lo % 360.0
        hi_norm = hi % 360.0
        if lo_norm <= hi_norm:
            return lo_norm <= angle <= hi_norm
        else:
            return angle >= lo_norm or angle <= hi_norm

    def _lidar_sees_pedestrian(self):
        """True only if positive finite returns are detected in the sector within range.

        Treats non-finite LiDAR values (inf/nan/no valid returns) as:
        'no LiDAR confirmation' -> False.
        """
        if self.latest_scan is None:
            return False

        msg = self.latest_scan
        n   = len(msg.ranges)
        if n == 0:
            return False

        min_r = float('inf')
        valid_hits = 0

        for i in range(n):
            angle_deg = (i * 360.0 / n) % 360.0
            if not self._in_sector(angle_deg, self.sector_lo, self.sector_hi):
                continue

            r = msg.ranges[i]
            # Treat non-finite LiDAR values (inf/nan) as NO LiDAR confirmation
            if not math.isfinite(r) or math.isnan(r) or math.isinf(r):
                continue

            # Must be strictly within valid sensor range bounds and detection threshold
            if msg.range_min < r < self.lidar_max:
                min_r = min(min_r, r)
                valid_hits += 1

        # Must have positive evidence: finite returns and minimum hit count
        if valid_hits < self.lidar_min_points or not math.isfinite(min_r):
            return False

        return min_r < self.lidar_max

    # ---------------------------------------------------------------------- #
    def _camera_sees_pedestrian(self):
        """True only if a purple/indigo blob is confirmed across temporal history buffer."""
        if self.latest_img is None:
            self.cam_history.append(False)
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

        frame_detected = False
        for cnt in contours:
            if cv2.contourArea(cnt) >= self.cam_min_area:
                frame_detected = True
                break

        self.cam_history.append(frame_detected)

        # Require consistent confirmation over temporal history buffer (e.g. 3 of last 5 frames)
        return sum(self.cam_history) >= self.cam_confirm_min


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
