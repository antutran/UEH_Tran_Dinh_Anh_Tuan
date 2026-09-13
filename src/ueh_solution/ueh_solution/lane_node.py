#!/usr/bin/env python3
"""Lane perception node for UEH CRC 2026.

Subscribes to: /camera/image_raw
Publishes to:  /lane/error   (std_msgs/Float32)
               /lane/debug   (sensor_msgs/Image)   -- optional visualization

Algorithm:
  1. Crop a bottom ROI from the image (roi_top_frac to 1.0).
  2. Apply CLAHE histogram equalisation to handle dark tunnel sections.
  3. Compute an adaptive threshold to extract bright road markings.
  4. Also apply an HSV white-pixel mask for daylight conditions.
  5. Combine both masks with OR.
  6. Split the ROI into left and right halves.
  7. Compute the centroid-x of white pixels in each half.
  8. Derive a lane-centre estimate; compute error = image_cx - lane_centre.
  9. Publish the error for the behavior node to consume.

Positive error → lane centre is to the LEFT of image centre → steer LEFT (w > 0).
"""

import math
import signal
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32


# HSV ranges for white lane markings (daylight)
WHITE_HSV_LOW  = np.array([0,   0, 160], dtype=np.uint8)
WHITE_HSV_HIGH = np.array([180, 55, 255], dtype=np.uint8)


def catch_sigterm():
    stopping = {'now': False}
    signal.signal(signal.SIGTERM, lambda *_: stopping.update(now=True))
    return stopping


class LaneNode(Node):

    def __init__(self):
        super().__init__('lane_node')

        # --- Parameters -------------------------------------------------------
        self.declare_parameter('roi_top_frac',     0.55)
        self.declare_parameter('clahe_clip',        2.5)
        self.declare_parameter('clahe_clip_dark',   4.5)
        self.declare_parameter('dark_threshold',    40)
        self.declare_parameter('adaptive_block',    31)
        self.declare_parameter('adaptive_c',       -10)
        self.declare_parameter('min_white_pixels',  80)
        self.declare_parameter('publish_debug',     False)
        self.declare_parameter('rate',              20.0)

        self.roi_top_frac    = self.get_parameter('roi_top_frac').value
        self.clahe_clip      = self.get_parameter('clahe_clip').value
        self.clahe_clip_dark = self.get_parameter('clahe_clip_dark').value
        self.dark_threshold  = self.get_parameter('dark_threshold').value
        self.adaptive_block  = int(self.get_parameter('adaptive_block').value)
        self.adaptive_c      = int(self.get_parameter('adaptive_c').value)
        self.min_white       = int(self.get_parameter('min_white_pixels').value)
        self.publish_debug   = self.get_parameter('publish_debug').value

        # --- State ------------------------------------------------------------
        self.bridge    = CvBridge()
        self.latest_img = None
        self.prev_error = 0.0
        self._last_log  = {}

        # --- I/O --------------------------------------------------------------
        self.pub_error = self.create_publisher(Float32, '/lane/error', 10)
        self.pub_mode  = self.create_publisher(Int32,   '/lane/dark_mode', 10)
        if self.publish_debug:
            self.pub_debug = self.create_publisher(
                Image, '/lane/debug', qos_profile_sensor_data)

        self.create_subscription(
            Image, '/camera/image_raw',
            self._on_image, qos_profile_sensor_data)

        rate = self.get_parameter('rate').value
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info('lane_node ready')

    # ---------------------------------------------------------------------- #
    def _on_image(self, msg):
        try:
            self.latest_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'image convert: {e}')

    # ---------------------------------------------------------------------- #
    def _tick(self):
        if self.latest_img is None:
            return
        img = self.latest_img
        error, dark_mode = self._process(img)

        msg = Float32()
        msg.data = float(error)
        self.pub_error.publish(msg)

        mode_msg = Int32()
        mode_msg.data = int(dark_mode)
        self.pub_mode.publish(mode_msg)

    # ---------------------------------------------------------------------- #
    def _process(self, img):
        """Return (error_pixels, dark_mode_flag).

        error_pixels > 0  → lane centre is LEFT of image centre → steer LEFT.
        """
        h, w = img.shape[:2]
        roi_y = int(h * self.roi_top_frac)
        roi   = img[roi_y:, :]          # bottom portion of the image

        # ---- Detect scene brightness ----------------------------------------
        gray_roi   = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        mean_bright = float(np.mean(gray_roi))
        dark_mode   = mean_bright < self.dark_threshold

        # ---- Build white mask -----------------------------------------------
        clip = self.clahe_clip_dark if dark_mode else self.clahe_clip
        mask = self._white_mask(roi, clip)

        # ---- Split into left / right halves ---------------------------------
        mid = w // 2
        left_mask  = mask[:, :mid]
        right_mask = mask[:, mid:]

        left_cx  = self._centroid_x(left_mask)   # None if not enough pixels
        right_cx = self._centroid_x(right_mask)  # relative to right half

        # ---- Lane centre estimate -------------------------------------------
        if left_cx is not None and right_cx is not None:
            # Both sides visible: midpoint
            lane_cx = (left_cx + (mid + right_cx)) / 2.0
        elif left_cx is not None:
            # Only left marking: assume lane width ~half image width
            lane_cx = left_cx + mid * 0.5
        elif right_cx is not None:
            # Only right marking
            lane_cx = (mid + right_cx) - mid * 0.5
        else:
            # No markings detected — hold previous error (temporal filter)
            return self.prev_error, dark_mode

        image_cx = w / 2.0
        error    = image_cx - lane_cx   # +ve → lane centre is left → turn left

        # Temporal smoothing: blend with previous
        error = 0.75 * error + 0.25 * self.prev_error
        self.prev_error = error

        # ---- Optional debug image -------------------------------------------
        if self.publish_debug and hasattr(self, 'pub_debug'):
            debug = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            cx_int = int(lane_cx)
            cv2.line(debug, (cx_int, 0), (cx_int, mask.shape[0]), (0, 255, 0), 2)
            cv2.line(debug, (w // 2, 0), (w // 2, mask.shape[0]), (0, 0, 255), 1)
            debug_msg = self.bridge.cv2_to_imgmsg(debug, 'bgr8')
            self.pub_debug.publish(debug_msg)

        return error, dark_mode

    # ---------------------------------------------------------------------- #
    def _white_mask(self, roi, clahe_clip):
        """Combine adaptive-threshold mask + HSV white mask."""
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # CLAHE equalisation
        clahe = cv2.createCLAHE(clipLimit=clahe_clip, tileGridSize=(8, 8))
        eq    = clahe.apply(gray)

        # Adaptive threshold (Gaussian) – robust to global brightness change
        block = self.adaptive_block
        if block % 2 == 0:
            block += 1
        adapt = cv2.adaptiveThreshold(
            eq, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            block,
            self.adaptive_c)

        # HSV white mask (daylight: high V, low S)
        hsv    = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        white  = cv2.inRange(hsv, WHITE_HSV_LOW, WHITE_HSV_HIGH)

        # Combine
        combined = cv2.bitwise_or(adapt, white)

        # Morphological cleanup: remove small noise, bridge tiny gaps
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN,  kernel)
        combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel)

        return combined

    # ---------------------------------------------------------------------- #
    def _centroid_x(self, mask):
        """Return x-centroid of white pixels in mask, or None if too few."""
        pts = cv2.findNonZero(mask)
        if pts is None or len(pts) < self.min_white:
            return None
        return float(np.mean(pts[:, 0, 0]))


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
    node = LaneNode()
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
