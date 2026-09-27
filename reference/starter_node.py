#!/usr/bin/env python3
"""Starter template for the UEH CRC 2026 simulation round.

Autonomous robot stack: Lane Keeping V1 with Bright Ramp Surface Fallback
and Dedicated DARK_LANE Tunnel Perception.
Perception & Control pipeline:
  Camera Image
  -> Scene Classification via Brightness Hysteresis (NORMAL vs DARK)
  -> If DARK:
       Dedicated DARK_LANE Multi-Row Perception (CLAHE, truncated ROI [0.52..0.85],
       transverse marking rejection, multi-row run consensus, near/far lookahead)
       Fallback to V1 Lane Perception if DARK_LANE invalid
  -> If NORMAL:
       Primary: White Lane Perception (Binary threshold >= 200, moments, V1 target)
       Secondary (only when primary confidence < min_confidence):
         Bright Surface Fallback (Grayscale inRange [160..230], morphology,
         connected component selection, multi-row sampling, surface target)
  -> Normalized Lateral Error
  -> Shared PD Controller
  -> Speed: Adaptive on normal road / conservative on ramp / tunnel
  -> self.drive(v, w)

Usage:
    ros2 run crc_sim starter
    ros2 run crc_sim starter --ros-args -p kp:=0.85 -p straight_speed:=0.08
"""

import math
import signal
import statistics
import time


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


# --- SIMPLE LOCAL CURVE-HOLD FIX CONSTANTS ---
SIMPLE_CURVE_DANGER_X_MIN = 1.05
SIMPLE_CURVE_DANGER_X_MAX = 1.50
SIMPLE_CURVE_DANGER_Y_MIN = -0.05
SIMPLE_CURVE_DANGER_Y_MAX = 0.25
SIMPLE_CURVE_DANGER_YAW_MIN = 0.15
SIMPLE_CURVE_DANGER_YAW_MAX = 0.50

SIMPLE_CURVE_HEALTHY_STEER_MIN = 0.03
SIMPLE_CURVE_HEALTHY_STEER_MAX = 0.25
SIMPLE_CURVE_STEERING_COLLAPSE_THRESH = 0.020

MAX_SHORT_HOLD_DISTANCE = 0.30  # m (conservative safety cap)
SIMPLE_CURVE_RELEASE_X = 1.50
SIMPLE_CURVE_RELEASE_Y = 0.25
SIMPLE_CURVE_RELEASE_YAW = 0.55
SIMPLE_CURVE_RECOVERY_STEER_MIN = 0.03
SIMPLE_CURVE_RECOVERY_FRAMES = 3

SIMPLE_CURVE_FAILSAFE_YAW_MAX = 1.90
SIMPLE_CURVE_FAILSAFE_X_MIN = 0.50
SIMPLE_CURVE_FAILSAFE_X_MAX = 2.80
SIMPLE_CURVE_FAILSAFE_Y_MIN = -0.40
SIMPLE_CURVE_FAILSAFE_Y_MAX = 1.60

# --- POST-REACQUIRE FALSE-PAIR PROTECTION CONSTANTS ---
POST_REACQUIRE_STABLE_DUAL_FRAMES = 15

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, LaserScan

try:
    import cv2
    import numpy as np
    from cv_bridge import CvBridge
    HAVE_CV = True
except ImportError:
    HAVE_CV = False


class DarkRow:
    """Individual scan row result for dedicated DARK_LANE perception."""
    __slots__ = ('row_y', 'row_center', 'left_x', 'right_x', 'lane_width', 'boundary_type', 'status', 'reject_reason')

    def __init__(self, row_y, row_center, left_x, right_x, lane_width, boundary_type, status='ACCEPTED', reject_reason='NONE'):
        self.row_y = int(row_y)
        self.row_center = float(row_center) if row_center is not None else None
        self.left_x = float(left_x) if left_x is not None else None
        self.right_x = float(right_x) if right_x is not None else None
        self.lane_width = float(lane_width) if lane_width is not None else None
        self.boundary_type = boundary_type  # 'DUAL', 'SINGLE', or 'NONE'
        self.status = status                # 'ACCEPTED', 'REJECTED', or 'PENDING'
        self.reject_reason = reject_reason  # 'NONE', 'TRANSVERSE', 'NO_CANDIDATE', 'SINGLE_DISAGREE', 'CENTER_OUTLIER', 'GEOMETRY'

    def __iter__(self):
        # 4-tuple unpacking for backward compatibility (row_y, row_center, left_x, right_x)
        return iter((self.row_y, self.row_center, self.left_x, self.right_x))

    def __getitem__(self, idx):
        return (self.row_y, self.row_center, self.left_x, self.right_x)[idx]

    def __len__(self):
        return 4

    def __repr__(self):
        c_str = f"{self.row_center:.1f}" if self.row_center is not None else 'None'
        return f'<DarkRow y={self.row_y} c={c_str} type={self.boundary_type} status={self.status} reason={self.reject_reason}>'

class SceneState(str):
    """String subclass representing scene state with backward-compatibility for 'DARK'."""
    def __eq__(self, other):
        s = str(self)
        o = str(other) if other is not None else ''
        if s == o:
            return True
        if s in ('DARK_ACTIVE', 'DARK') and o in ('DARK_ACTIVE', 'DARK'):
            return True
        return False

    def __ne__(self, other):
        return not self.__eq__(other)


class LaneTargetResult(tuple):
    """Smart tuple supporting unpacking 4 or 6 elements for backward compatibility."""
    def __iter__(self):
        try:
            import inspect, dis
            frame = inspect.currentframe().f_back
            code = frame.f_code.co_code
            lasti = frame.f_lasti
            op = code[lasti]
            arg = code[lasti + 1]
            if dis.opname[op] == 'UNPACK_SEQUENCE' and arg == 4:
                return iter(self[:4])
        except Exception:
            pass
        return super().__iter__()



# Pedestrian Crossing Risk States
CROSSING_OCCUPIED = 'OCCUPIED'
CROSSING_ENTRY_THREAT = 'ENTRY_THREAT'
CROSSING_CLEAR = 'CLEAR'
CROSSING_UNKNOWN = 'UNKNOWN'

# DARK Recovery Constants
MAX_DARK_RECOVERY_TIME = 1.0        # Max recovery duration in seconds during temporary perception dropout
MAX_DARK_RECOVERY_DISTANCE = 0.04    # Max recovery distance in meters if odometry is available
MAX_RECOVERY_W = 0.25                # Maximum angular velocity magnitude during dark recovery (rad/s)
DARK_RECOVERY_SPEED = 0.02           # Forward crawl speed during dark recovery (m/s)

# DARK Pending Grace Constants
DARK_PENDING_GRACE_SEC = 0.6         # Max confirmation grace duration in seconds at pending deadline
MAX_PENDING_READY_STALE_SEC = 0.25   # Maximum age in seconds for pending readiness evidence to be fresh


class Starter(Node):

    def __init__(self):
        super().__init__('crc_starter')

        # --- Baseline robot limits ------------------------------------------
        self.declare_parameter('max_speed', 0.12)       # m/s
        self.declare_parameter('max_turn', 1.0)         # rad/s
        self.declare_parameter('stop_distance', 0.35)   # m
        self.declare_parameter('rate', 20.0)            # Hz

        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_turn = float(self.get_parameter('max_turn').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        rate = float(self.get_parameter('rate').value)

        # --- Known-Good Lane Keeping V1 Parameters --------------------------
        self.declare_parameter('roi_start_ratio', 0.65)  # Bottom 35% of image (rows 312..480)
        self.declare_parameter('white_threshold', 200)   # Bright white lane marks (asphalt is ~23)
        self.declare_parameter('kp', 0.85)               # Proportional steering gain
        self.declare_parameter('kd', 0.30)               # Derivative damping gain
        self.declare_parameter('max_angular', 0.80)      # Max angular velocity clamp (rad/s)
        self.declare_parameter('straight_speed', 0.08)   # Forward speed on straight track (m/s)
        self.declare_parameter('medium_speed', 0.05)     # Forward speed on gentle curve (m/s)
        self.declare_parameter('corner_speed', 0.03)     # Forward speed on sharp curve (m/s)
        self.declare_parameter('min_confidence', 0.20)   # Minimum confidence threshold
        self.declare_parameter('debug_viz', False)       # Publish /camera/debug_lane image

        # --- Normal Lookahead & Curvature Feedforward Parameters -----------
        self.declare_parameter('normal_far_roi_top_ratio', 0.42)          # Top boundary of far lookahead band (row ~201)
        self.declare_parameter('normal_far_roi_bottom_ratio', 0.58)       # Bottom boundary of far lookahead band (row ~278)
        self.declare_parameter('normal_curve_gain', 0.75)                 # Curvature angular feedforward gain
        self.declare_parameter('normal_curve_min_signal_px', 15.0)        # Min curve signal to activate feedforward (px)
        self.declare_parameter('max_normal_curve_w', 0.30)                # Max angular feedforward clamp (rad/s)
        self.declare_parameter('normal_curve_alpha', 0.35)                # EMA smoothing factor for w_curve
        self.declare_parameter('normal_curve_speed_mod_px', 20.0)         # Curvature threshold for medium speed (px)
        self.declare_parameter('normal_curve_speed_strong_px', 40.0)      # Curvature threshold for corner speed (px)

        self.roi_start_ratio = float(self.get_parameter('roi_start_ratio').value)
        self.white_threshold = int(self.get_parameter('white_threshold').value)
        self.kp = float(self.get_parameter('kp').value)
        self.kd = float(self.get_parameter('kd').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.straight_speed = float(self.get_parameter('straight_speed').value)
        self.medium_speed = float(self.get_parameter('medium_speed').value)
        self.corner_speed = float(self.get_parameter('corner_speed').value)
        self.min_confidence = float(self.get_parameter('min_confidence').value)
        self.debug_viz = bool(self.get_parameter('debug_viz').value)

        self.normal_far_roi_top_ratio = float(self.get_parameter('normal_far_roi_top_ratio').value)
        self.normal_far_roi_bottom_ratio = float(self.get_parameter('normal_far_roi_bottom_ratio').value)
        self.normal_curve_gain = float(self.get_parameter('normal_curve_gain').value)
        self.normal_curve_min_signal_px = float(self.get_parameter('normal_curve_min_signal_px').value)
        self.max_normal_curve_w = float(self.get_parameter('max_normal_curve_w').value)
        self.normal_curve_alpha = float(self.get_parameter('normal_curve_alpha').value)
        self.normal_curve_speed_mod_px = float(self.get_parameter('normal_curve_speed_mod_px').value)
        self.normal_curve_speed_strong_px = float(self.get_parameter('normal_curve_speed_strong_px').value)

        # --- Bright Ramp Surface Fallback Parameters ------------------------
        self.declare_parameter('surface_min_gray', 160)             # Min gray for light-gray ramp surface
        self.declare_parameter('surface_max_gray', 230)             # Max gray for light-gray ramp surface
        self.declare_parameter('min_surface_area_ratio', 0.20)      # Min fraction of ROI area
        self.declare_parameter('min_surface_bottom_overlap', 0.25)  # Min fraction of bottom edge overlap
        self.declare_parameter('min_valid_surface_rows', 10)        # Min horizontal slices for target estimation
        self.declare_parameter('ramp_fallback_speed', 0.03)         # Conservative speed for ramp traversal (m/s)
        self.declare_parameter('ramp_enter_pitch_deg', 2.0)         # Pitch angle (deg) threshold to enter physical ramp mode
        self.declare_parameter('ramp_exit_pitch_deg', 1.2)          # Pitch angle (deg) threshold to exit physical ramp mode

        self.surface_min_gray = int(self.get_parameter('surface_min_gray').value)
        self.surface_max_gray = int(self.get_parameter('surface_max_gray').value)
        self.min_surface_area_ratio = float(self.get_parameter('min_surface_area_ratio').value)
        self.min_surface_bottom_overlap = float(self.get_parameter('min_surface_bottom_overlap').value)
        self.min_valid_surface_rows = int(self.get_parameter('min_valid_surface_rows').value)
        self.ramp_fallback_speed = float(self.get_parameter('ramp_fallback_speed').value)
        self.ramp_enter_pitch_deg = float(self.get_parameter('ramp_enter_pitch_deg').value)
        self.ramp_exit_pitch_deg = float(self.get_parameter('ramp_exit_pitch_deg').value)

        # --- Scene Classification Parameters (Forward-Looking ROI) ----------
        self.declare_parameter('scene_roi_top_ratio', 0.25)          # Top boundary for SCENE ROI (rows ~120 on 480p)
        self.declare_parameter('scene_roi_bottom_ratio', 0.70)       # Bottom boundary for SCENE ROI (rows ~336 on 480p)
        self.declare_parameter('dark_pixel_threshold', 20)           # Pixels with gray < threshold are considered dark
        self.declare_parameter('dark_ratio_enter', 0.28)             # Dark pixel ratio to consider frame as DARK candidate
        self.declare_parameter('dark_median_enter', 23.0)            # Supporting median threshold for DARK candidate
        self.declare_parameter('dark_enter_frames', 2)               # Consecutive candidate frames to enter DARK mode
        self.declare_parameter('dark_ratio_exit', 0.20)              # Dark pixel ratio ceiling to allow NORMAL exit
        self.declare_parameter('dark_median_exit', 27.0)             # Median brightness floor to allow NORMAL exit
        self.declare_parameter('dark_exit_frames', 8)                # Consecutive exit frames to return to NORMAL mode

        # --- Dark Tunnel Dedicated Lane Keeping Parameters (UNCHANGED) ------
        self.declare_parameter('dark_enter_threshold', 23.0)         # Backward-compatible threshold alias
        self.declare_parameter('dark_exit_threshold', 27.0)          # Backward-compatible threshold alias
        self.declare_parameter('dark_roi_top_ratio', 0.52)           # Top boundary for DARK ROI (fraction of height)
        self.declare_parameter('dark_roi_bottom_ratio', 0.85)        # Bottom boundary for DARK ROI (fraction of height)
        self.declare_parameter('dark_clahe_clip', 2.0)               # Mild CLAHE clip limit for dim road
        self.declare_parameter('dark_white_threshold', 180)          # Bright lane marking threshold in DARK mode
        self.declare_parameter('dark_scan_rows', 7)                  # Number of horizontal scan rows in DARK ROI
        self.declare_parameter('max_transverse_row_ratio', 0.55)     # Max bright fraction across row before transverse rejection
        self.declare_parameter('min_dark_valid_rows', 2)             # Min valid row centers required for valid target
        self.declare_parameter('near_weight', 0.4)                   # Lookahead near center weight
        self.declare_parameter('far_weight', 0.6)                    # Lookahead far center weight
        self.declare_parameter('dark_target_alpha', 0.35)            # EMA smoothing alpha for dark target
        self.declare_parameter('dark_lane_speed', 0.04)              # Conservative straight forward speed in tunnel (m/s)
        self.declare_parameter('dark_adaptive_band_delta_px', 18)   # Adaptive scan search half-band delta (px)

        self.scene_roi_top_ratio = float(self.get_parameter('scene_roi_top_ratio').value)
        self.scene_roi_bottom_ratio = float(self.get_parameter('scene_roi_bottom_ratio').value)
        self.dark_pixel_threshold = int(self.get_parameter('dark_pixel_threshold').value)
        self.dark_ratio_enter = float(self.get_parameter('dark_ratio_enter').value)
        self.dark_median_enter = float(self.get_parameter('dark_median_enter').value)
        self.dark_enter_frames = int(self.get_parameter('dark_enter_frames').value)
        self.dark_ratio_exit = float(self.get_parameter('dark_ratio_exit').value)
        self.dark_median_exit = float(self.get_parameter('dark_median_exit').value)
        self.dark_exit_frames = int(self.get_parameter('dark_exit_frames').value)
        self.dark_enter_threshold = float(self.get_parameter('dark_enter_threshold').value)
        self.dark_exit_threshold = float(self.get_parameter('dark_exit_threshold').value)
        self.dark_roi_top_ratio = float(self.get_parameter('dark_roi_top_ratio').value)
        self.dark_roi_bottom_ratio = float(self.get_parameter('dark_roi_bottom_ratio').value)
        self.dark_clahe_clip = float(self.get_parameter('dark_clahe_clip').value)
        self.dark_white_threshold = int(self.get_parameter('dark_white_threshold').value)
        self.dark_scan_rows = int(self.get_parameter('dark_scan_rows').value)
        self.max_transverse_row_ratio = float(self.get_parameter('max_transverse_row_ratio').value)
        self.min_dark_valid_rows = int(self.get_parameter('min_dark_valid_rows').value)
        self.near_weight = float(self.get_parameter('near_weight').value)
        self.far_weight = float(self.get_parameter('far_weight').value)
        self.dark_target_alpha = float(self.get_parameter('dark_target_alpha').value)
        self.dark_lane_speed = float(self.get_parameter('dark_lane_speed').value)
        self.dark_adaptive_band_delta_px = int(self.get_parameter('dark_adaptive_band_delta_px').value)

        # --- Dark Tunnel Robustness & Stabilization Parameters --------------
        self.declare_parameter('dark_hold_frames', 6)                # Max frames to hold target if DARK_LANE lost
        self.declare_parameter('dark_hold_speed', 0.02)              # Crawl speed during target hold (m/s)
        self.declare_parameter('max_dark_recovery_time', MAX_DARK_RECOVERY_TIME)
        self.declare_parameter('max_dark_recovery_dist', MAX_DARK_RECOVERY_DISTANCE)
        self.declare_parameter('max_dark_recovery_w', MAX_RECOVERY_W)
        self.declare_parameter('dark_recovery_speed', DARK_RECOVERY_SPEED)
        self.declare_parameter('max_dark_row_center_deviation_px', 60.0)  # Max row center deviation from consensus median
        self.declare_parameter('max_dark_adjacent_center_delta_px', 55.0)  # Max center shift between adjacent rows (far->near)
        self.declare_parameter('max_dark_near_far_disagreement_px', 70.0) # Max disagreement between near and far lookahead centers
        self.declare_parameter('dark_entry_stabilize_frames', 10)    # Initial DARK frames preferring far/mid centers
        self.declare_parameter('dual_row_weight', 1.0)               # Confidence weight for 2-boundary rows
        self.declare_parameter('single_row_weight', 0.35)            # Confidence weight for synthesized 1-boundary rows

        self.dark_hold_frames = int(self.get_parameter('dark_hold_frames').value)
        self.dark_hold_speed = float(self.get_parameter('dark_hold_speed').value)
        self.max_dark_recovery_time = float(self.get_parameter('max_dark_recovery_time').value)
        self.max_dark_recovery_dist = float(self.get_parameter('max_dark_recovery_dist').value)
        self.max_dark_recovery_w = float(self.get_parameter('max_dark_recovery_w').value)
        self.dark_recovery_speed = float(self.get_parameter('dark_recovery_speed').value)
        self.max_dark_row_center_deviation_px = float(self.get_parameter('max_dark_row_center_deviation_px').value)
        self.max_dark_adjacent_center_delta_px = float(self.get_parameter('max_dark_adjacent_center_delta_px').value)
        self.max_dark_near_far_disagreement_px = float(self.get_parameter('max_dark_near_far_disagreement_px').value)
        self.dark_entry_stabilize_frames = int(self.get_parameter('dark_entry_stabilize_frames').value)
        self.dual_row_weight = float(self.get_parameter('dual_row_weight').value)
        self.single_row_weight = float(self.get_parameter('single_row_weight').value)

        # Dark Transition / Handoff Parameters & State
        self.declare_parameter('dark_ready_min_rows', 2)             # Min accepted rows for DARK_LANE readiness
        self.declare_parameter('dark_ready_frames', 2)               # Consecutive frames required for handoff
        self.declare_parameter('dark_pending_speed', 0.02)           # Bridge crawling speed during handoff (m/s)
        self.declare_parameter('dark_pending_max_frames', 40)        # Max frames allowed in DARK_PENDING before safe stop
        self.declare_parameter('dark_pending_grace_sec', DARK_PENDING_GRACE_SEC) # Max confirmation grace duration (s)

        self.dark_ready_min_rows = int(self.get_parameter('dark_ready_min_rows').value)
        self.dark_ready_frames = int(self.get_parameter('dark_ready_frames').value)
        self.dark_pending_speed = float(self.get_parameter('dark_pending_speed').value)
        self.dark_pending_max_frames = int(self.get_parameter('dark_pending_max_frames').value)
        self.dark_pending_grace_sec = float(self.get_parameter('dark_pending_grace_sec').value)

        # --- Dark Curve Feedforward & History Decay Parameters -------------
        self.declare_parameter('dark_expected_center_hist_weight', 0.35) # Soft decay weight toward image center for stale history
        self.declare_parameter('dark_curve_gain', 1.35)                   # Direct angular feedforward gain for dark curve
        self.declare_parameter('max_dark_curve_w', 0.25)                  # Max clamp for curve feedforward term (rad/s)
        self.declare_parameter('dark_max_w', 0.40)                        # Max clamp for total dark angular command (rad/s)
        self.declare_parameter('dark_curve_min_span_px', 40.0)            # Min vertical span between far and near dual rows (px)
        self.declare_parameter('dark_curve_min_dual_rows', 3)             # Min trusted dual rows to activate curve feedforward
        self.declare_parameter('dark_curve_alpha', 0.60)                  # EMA smoothing alpha for dark curve feedforward

        self.dark_expected_center_hist_weight = float(self.get_parameter('dark_expected_center_hist_weight').value)
        self.dark_curve_gain = float(self.get_parameter('dark_curve_gain').value)
        self.max_dark_curve_w = float(self.get_parameter('max_dark_curve_w').value)
        self.dark_max_w = float(self.get_parameter('dark_max_w').value)
        self.dark_curve_min_span_px = float(self.get_parameter('dark_curve_min_span_px').value)
        self.dark_curve_min_dual_rows = int(self.get_parameter('dark_curve_min_dual_rows').value)
        self.dark_curve_alpha = float(self.get_parameter('dark_curve_alpha').value)

        self.prev_dark_w_curve = 0.0
        self.last_dark_curve_signal = 0.0
        self.last_dark_w_curve = 0.0
        self.last_dark_lateral_center = None

        # Dark Scene Hysteresis & Target Smoothing State
        self.scene_state = SceneState('NORMAL')                      # 'NORMAL', 'DARK_PENDING', or 'DARK_ACTIVE'
        self.dark_enter_count = 0
        self.dark_exit_count = 0
        self.last_stable_normal_target_x = None                      # Saved anchor target from reliable LANE / SURFACE
        self.dark_pending_count = 0                                  # Frames spent in DARK_PENDING
        self.dark_ready_count = 0                                    # Consecutive frames candidate is ready
        self.dark_pending_grace_active = False
        self.dark_pending_grace_start_time = None
        self.dark_pending_last_ready_stamp = None
        self.dark_pending_last_ready_frame = None
        self.dark_pending_last_ready_rows = 0
        self.prev_dark_target_x = None
        self.last_valid_dark_target_x = None
        self.last_good_dark_target = None
        self.last_good_dark_w = 0.0
        self.last_good_dark_stamp = None
        self.last_good_dark_odom_x = None
        self.last_good_dark_odom_y = None
        self.dark_recovery_active = False
        self.dark_recovery_expired_latched = False
        self.dark_hold_count = 0
        self.dark_entry_stabilize_count = 0
        self.last_dark_rows = []
        self.scene_mean = 0.0
        self.scene_median = 0.0
        self.scene_p25 = 0.0
        self.dark_pixel_ratio = 0.0
        self.perception_mode = 'NONE'                                # 'LANE', 'SURFACE', 'DARK_PENDING', 'DARK_LANE', or 'NONE'

        # --- Front Obstacle Safety (Cluster Detection) Parameters -----------
        self.declare_parameter('front_sector_deg', 20.0)             # Frontal sector (+/- deg)
        self.declare_parameter('clear_distance', 0.42)               # Release hysteresis threshold (m)
        self.declare_parameter('min_obstacle_cluster_rays', 5)       # Min consecutive rays for valid candidate
        self.declare_parameter('obstacle_confirm_scans', 2)          # Required consecutive scans to confirm stop
        self.declare_parameter('emergency_sector_deg', 5.0)          # Central emergency sector (+/- deg)
        self.declare_parameter('emergency_distance', 0.18)           # Emergency stop threshold (m)
        self.declare_parameter('emergency_min_cluster_rays', 3)      # Min central rays for emergency stop

        self.front_sector_deg = float(self.get_parameter('front_sector_deg').value)
        self.clear_distance = float(self.get_parameter('clear_distance').value)
        self.min_obstacle_cluster_rays = int(self.get_parameter('min_obstacle_cluster_rays').value)
        self.obstacle_confirm_scans = int(self.get_parameter('obstacle_confirm_scans').value)
        self.emergency_sector_deg = float(self.get_parameter('emergency_sector_deg').value)
        self.declare_parameter('min_obstacle_width_m', 0.025)        # Min lateral cluster width for normal obstacle (m)
        self.declare_parameter('emergency_min_width_m', 0.020)       # Min lateral cluster width for emergency stop (m)

        self.emergency_distance = float(self.get_parameter('emergency_distance').value)
        self.emergency_min_cluster_rays = int(self.get_parameter('emergency_min_cluster_rays').value)
        self.min_obstacle_width_m = float(self.get_parameter('min_obstacle_width_m').value)
        self.emergency_min_width_m = float(self.get_parameter('emergency_min_width_m').value)

        # Physical Robot Footprint (Laser Frame) for Self-Return Masking
        # Laser frame base_scan origin relative to base_link: (-0.064, 0, 0.122)
        # Waffle chassis STL mesh X extent in laser frame: [-0.1328 .. +0.1378] m
        # Y extent in laser frame: [-0.1394 .. +0.1394] m (wheels at +/- 0.144m, outer edge 0.153m)
        self.declare_parameter('footprint_x_min', -0.140)            # Laser frame rear limit (m)
        self.declare_parameter('footprint_x_max', 0.142)             # Laser frame front limit (m: waffle mesh 0.1378 + 4.2mm margin)
        self.declare_parameter('footprint_y_min', -0.155)            # Laser frame right limit (m: includes right wheel)
        self.declare_parameter('footprint_y_max', 0.155)             # Laser frame left limit (m: includes left wheel)

        self.footprint_x_min = float(self.get_parameter('footprint_x_min').value)
        self.footprint_x_max = float(self.get_parameter('footprint_x_max').value)
        self.footprint_y_min = float(self.get_parameter('footprint_y_min').value)
        self.footprint_y_max = float(self.get_parameter('footprint_y_max').value)

        # Obstacle Safety State
        self.obstacle_confirm_count = 0
        self.obstacle_stop_active = False
        self._scan_seq = 0
        self._last_processed_scan_seq = -1
        self._last_processed_scan = None

        # --- Pedestrian Zebra Crossing Parameters ---------------------------
        self.declare_parameter('zebra_roi_top_ratio', 0.50)          # Top boundary for zebra ROI (rows ~240 on 480p)
        self.declare_parameter('zebra_roi_bottom_ratio', 0.85)       # Bottom boundary for zebra ROI (rows ~408 on 480p)
        self.declare_parameter('zebra_roi_left_ratio', 0.25)         # Left boundary for central roadway window (cols ~160 on 640p)
        self.declare_parameter('zebra_roi_right_ratio', 0.75)        # Right boundary for central roadway window (cols ~480 on 640p)
        self.declare_parameter('zebra_white_threshold', 150)         # White pixel threshold for zebra stripes
        self.declare_parameter('zebra_min_row_bright_ratio', 0.22)   # Min bright pixel fraction per row to consider stripe row
        self.declare_parameter('zebra_min_bands', 3)                 # Min separated transverse bands required
        self.declare_parameter('zebra_confirm_frames', 2)            # Consecutive frames to confirm zebra crossing ahead
        self.declare_parameter('zebra_exit_frames', 4)               # Consecutive frames without zebra to exit crossing mode
        self.declare_parameter('crossing_approach_speed', 0.03)      # Speed cap during CROSSING_APPROACH (m/s)
        self.declare_parameter('crossing_pass_speed', 0.04)          # Speed cap during CROSSING_PASS (m/s)
        self.declare_parameter('crossing_pass_min_distance_m', 0.30) # Min relative distance to clear crossing (m)
        # CORE/EDGE zone split - replaces old Patch C x-coordinate heuristic
        self.declare_parameter('crossing_core_half_width_m', 0.20)         # CORE: |y| <= this => STOP without motion evidence
        self.declare_parameter('crossing_edge_motion_min_total_m', 0.05)   # EDGE: min accumulated inward displacement (m)
        self.declare_parameter('crossing_edge_motion_min_steps', 2)        # EDGE: min NEW scans with positive inward delta

        # Crossing-Specific LiDAR Roadway Corridor Parameters
        self.declare_parameter('crossing_detect_min_x_m', 0.15)      # Min forward distance ahead of chassis (m)
        self.declare_parameter('crossing_detect_max_x_m', 1.20)      # Max forward lookahead in crossing corridor (m)
        self.declare_parameter('crossing_half_width_m', 0.42)        # Roadway half-width: covers road (|y| <= 0.42m), rejects kerb/scenery (|y| >= 0.50m)
        self.declare_parameter('crossing_clear_hold_s', 0.8)         # Temporal debounce: consecutive seconds crossing must stay clear before PASS
        self.declare_parameter('crossing_min_cluster_rays', 2)       # Min contiguous LiDAR rays for pedestrian candidate
        self.declare_parameter('crossing_min_cluster_width_m', 0.010)# Min lateral width for pedestrian cluster (m)
        self.declare_parameter('crossing_max_range_jump_m', 0.15)    # Max range jump (m) between adjacent rays in same cluster
        self.declare_parameter('crossing_confirm_scans', 2)          # Scans to confirm road occupancy
        self.declare_parameter('crossing_clear_scans', 5)            # Consecutive clear scans to confirm road is clear
        self.declare_parameter('crossing_entry_half_width_m', 0.38)  # Entry watch zone outer boundary: 0.24 < |y| <= 0.38m
        self.declare_parameter('crossing_track_max_dx_m', 0.15)      # Max dx (m) to match same candidate across scans
        self.declare_parameter('crossing_track_max_dy_m', 0.15)      # Max dy (m) to match same candidate across scans
        self.declare_parameter('crossing_entry_min_inward_delta_m', 0.015) # Min inward movement (|y_prev| - |y_curr|) per scan
        self.declare_parameter('crossing_entry_confirm_scans', 2)    # Consecutive scans of inward motion to confirm ENTRY_THREAT
        self.declare_parameter('crossing_scan_stale_timeout_s', 0.50)  # Max age (s) before scan is treated as UNKNOWN
        self.declare_parameter('crossing_image_stale_timeout_s', 0.50) # Max age (s) before image is treated as stale

        self.zebra_roi_top_ratio = float(self.get_parameter('zebra_roi_top_ratio').value)
        self.zebra_roi_bottom_ratio = float(self.get_parameter('zebra_roi_bottom_ratio').value)
        self.zebra_roi_left_ratio = float(self.get_parameter('zebra_roi_left_ratio').value)
        self.zebra_roi_right_ratio = float(self.get_parameter('zebra_roi_right_ratio').value)
        self.zebra_white_threshold = int(self.get_parameter('zebra_white_threshold').value)
        self.zebra_min_row_bright_ratio = float(self.get_parameter('zebra_min_row_bright_ratio').value)
        self.zebra_min_bands = int(self.get_parameter('zebra_min_bands').value)
        self.zebra_confirm_frames = int(self.get_parameter('zebra_confirm_frames').value)
        self.zebra_exit_frames = int(self.get_parameter('zebra_exit_frames').value)
        self.crossing_approach_speed = float(self.get_parameter('crossing_approach_speed').value)
        self.crossing_pass_speed = float(self.get_parameter('crossing_pass_speed').value)
        self.crossing_pass_min_distance_m = float(self.get_parameter('crossing_pass_min_distance_m').value)
        self.crossing_core_half_width_m = float(self.get_parameter('crossing_core_half_width_m').value)
        self.crossing_edge_motion_min_total_m = float(self.get_parameter('crossing_edge_motion_min_total_m').value)
        self.crossing_edge_motion_min_steps = int(self.get_parameter('crossing_edge_motion_min_steps').value)

        self.crossing_detect_min_x_m = float(self.get_parameter('crossing_detect_min_x_m').value)
        self.crossing_detect_max_x_m = float(self.get_parameter('crossing_detect_max_x_m').value)
        self.crossing_half_width_m = float(self.get_parameter('crossing_half_width_m').value)
        self.crossing_clear_hold_s = float(self.get_parameter('crossing_clear_hold_s').value)
        self.crossing_min_cluster_rays = int(self.get_parameter('crossing_min_cluster_rays').value)
        self.crossing_min_cluster_width_m = float(self.get_parameter('crossing_min_cluster_width_m').value)
        self.crossing_max_range_jump_m = float(self.get_parameter('crossing_max_range_jump_m').value)
        self.crossing_confirm_scans = int(self.get_parameter('crossing_confirm_scans').value)
        self.crossing_clear_scans = int(self.get_parameter('crossing_clear_scans').value)
        self.crossing_entry_half_width_m = float(self.get_parameter('crossing_entry_half_width_m').value)
        self.crossing_track_max_dx_m = float(self.get_parameter('crossing_track_max_dx_m').value)
        self.crossing_track_max_dy_m = float(self.get_parameter('crossing_track_max_dy_m').value)
        self.crossing_entry_min_inward_delta_m = float(self.get_parameter('crossing_entry_min_inward_delta_m').value)
        self.crossing_entry_confirm_scans = int(self.get_parameter('crossing_entry_confirm_scans').value)
        self.crossing_scan_stale_timeout_s = float(self.get_parameter('crossing_scan_stale_timeout_s').value)
        self.crossing_image_stale_timeout_s = float(self.get_parameter('crossing_image_stale_timeout_s').value)

        # Crossing State Machine Tracking
        self.crossing_state = 'IDLE'                                 # 'IDLE', 'APPROACH', 'WAIT', 'PASS'
        self.zebra_detect_count = 0
        self.zebra_lost_count = 0
        self.crossing_occupied = False
        self.crossing_detect_count = 0
        self.crossing_clear_count = 0
        self.crossing_clear_since = None
        self._last_crossing_occ_log_time = 0.0
        self._last_occ_logged_state = None
        self._last_zebra_detect_log_time = 0.0
        self.crossing_persistent_ray_count = 0
        self.last_zebra_detected = False
        self.last_zebra_bands = 0
        self.last_zebra_score = 0.0
        self._last_crossing_log_time = 0.0
        self._last_crossing_wait_log_time = 0.0
        self._last_crossing_reject_lateral_time = 0.0
        self._last_crossing_reject_single_time = 0.0
        self._last_crossing_state = 'IDLE'
        self.has_odom = False
        self.crossing_pass_start_x = None
        self.crossing_pass_start_y = None
        self._last_crossing_pass_log_time = 0.0

        # Crossing Sensor Consumption & Caching Bookkeeping
        self._crossing_processed_scan_seq = -1
        self._last_crossing_scan = None
        self._cached_crossing_occ = None
        self._latest_scan_stamp = 0.0
        self._latest_scan_received_time = 0.0
        self._last_valid_scan_stamp = 0.0

        self._image_seq = 0
        self._crossing_processed_image_seq = -1
        self._last_crossing_image = None
        self._cached_zebra_result = (False, 0, 0.0, None)
        self._latest_image_stamp = 0.0

        self._last_crossing_scan_log_time = 0.0
        self._last_crossing_scan_skip_log_time = 0.0
        self._last_crossing_unknown_log_time = 0.0
        self._last_crossing_cluster_log_time = 0.0
        self._last_zebra_pattern_log_time = 0.0

        # Temporal Candidate Tracking & Entry Threat State
        self.crossing_tracked_candidate = None   # dict with previous_x, previous_y, current_x, current_y, scan_seq, timestamp, matched_scans
        self.crossing_entry_confirm_count = 0
        self.crossing_stop_reason = 'NONE'       # 'ROAD_OCCUPIED', 'ENTRY_MOVING', or 'NONE'
        self._last_crossing_track_log_time = 0.0
        self._last_crossing_entry_log_time = 0.0
        self._last_crossing_threat_log_time = 0.0
        self._last_crossing_kerb_log_time = 0.0
        self._last_crossing_status_log_time = 0.0

        # EDGE motion tracker (replaces Patch C) - tracks inward movement of edge-zone clusters
        # Reset when: crossing resets to IDLE, cluster association fails, scan discontinuity, timeout
        self._edge_track = {
            'prev_scan_seq': -1,    # scan_seq of previous observation
            'prev_x': None,         # previous cluster centroid x
            'prev_y': None,         # previous cluster centroid y
            'curr_x': None,         # current cluster centroid x
            'curr_y': None,         # current cluster centroid y
            'start_abs_y': None,    # abs(y) at the start of current inward streak
            'motion_steps': 0,      # consecutive scans in current inward streak
        }
        self._last_crossing_object_log_time = 0.0
        self._last_speed_arb_log_time = 0.0

        # --- Lane Tracking State --------------------------------------------
        self.prev_error = 0.0
        self.half_lane_px = 160.0       # Initial estimated half-lane width in ROI (pixels)

        # --- Post-Tunnel Assist State Machine -------------------------------
        self.post_tunnel_assist_armed = False
        # --- DIAGNOSTIC ONLY: post-tunnel reference pose and trace flag ---
        self.post_tunnel_ref_x = None
        self.post_tunnel_ref_y = None
        self.post_tunnel_ref_yaw = None
        self.post_tunnel_ref_time = None
        self.post_tunnel_trace_active = False
        self._post_tunnel_ref_latched = False
        self._post_tunnel_ref_logged = False
        self._ctrl_frame_seq = 0
        self._diag_c1 = 0
        self._diag_c2 = 0
        self._diag_c3 = 0
        self._diag_c4 = 0
        self._diag_c5 = 0
        self._diag_c6 = 0
        self._diag_c7 = 0
        # ------------------------------------------------------------------
        self.post_tunnel_normal_ready = False
        self.post_tunnel_curve_active = False
        self.post_tunnel_guard_active = False
        self.post_tunnel_assist_consumed = False
        self.post_tunnel_ready_frames = 0
        self.post_tunnel_curve_confirm_count = 0
        self.post_tunnel_curve_decay_count = 0
        self.post_tunnel_curve_dir = 0
        self.post_tunnel_guard_stable_frames = 0
        self.trusted_post_tunnel_lane_width = 350.0

        # --- Simple Local Curve-Hold State Machine ---
        self.simple_curve_state = 'IDLE'
        self.simple_curve_history = []
        self.simple_curve_held_steering = 0.0
        self.simple_curve_hold_start_x = None
        self.simple_curve_hold_start_y = None
        self.simple_curve_recovery_frames = 0
        self.simple_curve_x_local = 0.0
        self.simple_curve_y_local = 0.0
        self.simple_curve_rel_yaw = 0.0
        self.post_tunnel_junction_risk_seen = False
        self.prev_normal_w_curve = 0.0
        self.last_normal_curve_signal = 0.0
        self.last_normal_w_curve = 0.0
        self._last_post_tunnel_width_reject_log_time = 0.0
        self._last_post_tunnel_single_boundary_log_time = 0.0
        self._last_post_tunnel_junction_risk_log_time = 0.0
        self._post_tunnel_curve_logged = False

        # Post-tunnel local hybrid curve boost
        self.post_tunnel_curve_boost_active = False
        self.post_tunnel_curve_candidate_dir = 0
        self.post_tunnel_curve_candidate_count = 0
        self.post_tunnel_boost_confirm_frames = 3
        self.post_tunnel_curve_dropout_frames = 0
        self.post_tunnel_curve_exit_stable_frames = 0
        self.post_tunnel_curve_exit_frames = 10
        self.post_tunnel_boost_min_signal_px = 35.0
        self._last_post_tunnel_candidate_log_time = 0.0
        self._last_post_tunnel_hold_log_time = 0.0
        self._post_tunnel_boost_logged = False
        self.post_tunnel_curve_boost_consumed = False

        # Post-curve reacquisition state machine
        self.post_curve_reacquire_active = False
        self.post_curve_reacquire_stable_count = 0
        self.post_curve_reacquire_last_target = None

        # Post-Reacquire Protect state (handoff safety after REACQUIRE_COMPLETE)
        self.POST_REACQUIRE_STABLE_DUAL_FRAMES = POST_REACQUIRE_STABLE_DUAL_FRAMES
        self.post_reacquire_protect_active = False
        self.post_reacquire_reference_target = None
        self.post_reacquire_protected_target = None
        self.post_reacquire_dual_streak = 0
        self.post_reacquire_single_streak = 0
        self.post_reacquire_trusted_width = None
        self.post_reacquire_trusted_right = None
        self.post_reacquire_trusted_target = None
        self.post_reacquire_right_missing_count = 0
        self._last_post_reacquire_protect_log_time = 0.0
        self._last_post_reacquire_false_pair_log_time = 0.0
        self.right_cont_start_x = None
        self.right_cont_start_y = None
        self.RIGHT_CONT_MIN_DISTANCE = 4.5
        self._last_right_cont_track_log_time = 0.0

        # Temporal Sanity Guard for Post-Tunnel Curve Apex
        self.last_trusted_near_left = None
        self.last_trusted_near_right = None
        self.last_trusted_near_target = None
        self.last_trusted_near_width = None
        self.apex_guard_hold_count = 0
        self.apex_quarantine_active = False
        self.apex_recovery_streak = 0
        self._last_apex_guard_diag = None
        self._last_apex_guard_log_time = 0.0
        self._apex_guard_was_triggered = False
        self.curve_containment_scale = 1.0
        self._last_curve_containment_log_time = 0.0
        self._last_post_curve_reacquire_log_time = 0.0

        # Latest sensor data. All of these stay None until the first message.
        self.image = None               # BGR image, 480x640x3
        self.scan = None                # sensor_msgs/LaserScan
        self.x = self.y = self.yaw = 0.0
        self.pitch_deg = 0.0            # IMU pitch angle for passive diagnostics only
        self.ramp_slope_active = False  # Physical slope active state with hysteresis
        self._last_log_time = 0.0
        self._last_log = {}

        self.bridge = CvBridge() if HAVE_CV else None
        if not HAVE_CV:
            self.get_logger().warn(
                'cv_bridge not found, self.image will stay None. '
                'apt install ros-humble-cv-bridge python3-opencv')

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_debug_img = self.create_publisher(Image, '/camera/debug_lane', qos_profile_sensor_data)

        self.create_subscription(Image, '/camera/image_raw',
                                 self.on_image, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan',
                                 self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)

        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            f'Lane Keeping V1 + Ramp Fallback + DARK_LANE ready | rate={rate:.0f} Hz | '
            f'speeds=({self.straight_speed}, {self.medium_speed}, {self.corner_speed}) m/s | '
            f'dark_speed={self.dark_lane_speed} m/s | fallback_speed={self.ramp_fallback_speed} m/s | '
            f'kp={self.kp} kd={self.kd} max_angular={self.max_angular}')

    # --- sensor callbacks ---------------------------------------------------

    def _get_current_time_sec(self):
        try:
            now_msg = self.get_clock().now()
            now_sec = float(now_msg.nanoseconds) * 1e-9
            if now_sec > 0.0:
                return now_sec
        except Exception:
            pass
        return time.time()

    def _reset_dark_pending_grace(self):
        self.dark_pending_grace_active = False
        self.dark_pending_grace_start_time = None
        self.dark_pending_last_ready_stamp = None
        self.dark_pending_last_ready_frame = None
        self.dark_pending_last_ready_rows = 0

    def _extract_stamp_sec(self, header_stamp):
        if header_stamp is not None:
            sec = getattr(header_stamp, 'sec', 0)
            nanosec = getattr(header_stamp, 'nanosec', 0)
            if sec > 0 or nanosec > 0:
                return float(sec) + float(nanosec) * 1e-9
        return self._get_current_time_sec()

    def _compute_scan_age(self, target_scan, now_sec):
        """Compute scan age in seconds ensuring compatible clock domains.

        Handles:
          - Both now_sec and scan_stamp in same domain (both sim time < 1e8 or both epoch >= 1e8).
          - Incompatible domains (e.g. Gazebo sim time in header.stamp vs node clock in epoch/system time):
            uses receive time recorded in on_scan().
          - Mock / zero timestamps (sec=0, nanosec=0): uses receive time if available, else 0.0.
        """
        if target_scan is None:
            return float('inf')

        stamp_msg = getattr(target_scan, 'header', None) and target_scan.header.stamp
        sec = getattr(stamp_msg, 'sec', 0) if stamp_msg is not None else 0
        nanosec = getattr(stamp_msg, 'nanosec', 0) if stamp_msg is not None else 0
        has_stamp = (sec > 0 or nanosec > 0)

        if has_stamp:
            scan_stamp = float(sec) + float(nanosec) * 1e-9
            same_domain = (now_sec >= 1e8 and scan_stamp >= 1e8) or (now_sec < 1e8 and scan_stamp < 1e8)
            if same_domain:
                return max(0.0, now_sec - scan_stamp)
            elif self._latest_scan_received_time > 0.0 and target_scan is self.scan:
                return max(0.0, now_sec - self._latest_scan_received_time)
            else:
                return 0.0

        if self._latest_scan_received_time > 0.0 and target_scan is self.scan:
            return max(0.0, now_sec - self._latest_scan_received_time)

        return 0.0

    def on_image(self, msg):
        if self.bridge is None:
            return
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self._image_seq += 1
            self._latest_image_stamp = self._extract_stamp_sec(getattr(msg, 'header', None) and msg.header.stamp)
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')

    def on_scan(self, msg):
        self.scan = msg
        self._scan_seq += 1
        self._latest_scan_received_time = self._get_current_time_sec()
        self._latest_scan_stamp = self._extract_stamp_sec(getattr(msg, 'header', None) and msg.header.stamp)

    def on_odom(self, msg):
        self.has_odom = True
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.x = p.x
        self.y = p.y
        siny_cosp = 2.0 * (o.w * o.z + o.x * o.y)
        cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def on_imu(self, msg: Imu):
        """Passive IMU callback: extracts pitch angle for diagnostic logging."""
        o = msg.orientation
        sinp = 2.0 * (o.w * o.y - o.z * o.x)
        if abs(sinp) >= 1.0:
            pitch_rad = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch_rad = math.asin(sinp)
        self.pitch_deg = math.degrees(pitch_rad)

    # --- helpers ------------------------------------------------------------

    def range_at(self, angle_deg, width_deg=10.0):
        if self.scan is None:
            return float('inf')
        msg = self.scan
        amin = msg.angle_min
        inc = msg.angle_increment
        nranges = len(msg.ranges)

        target_rad = math.radians(angle_deg)
        half_w = math.radians(width_deg / 2.0)
        start_rad = target_rad - half_w
        end_rad = target_rad + half_w

        def a2i(a):
            return int((a - amin) / inc)

        i_start = max(0, min(nranges - 1, a2i(start_rad)))
        i_end = max(0, min(nranges - 1, a2i(end_rad)))
        if i_start > i_end:
            i_start, i_end = i_end, i_start

        window = [
            msg.ranges[i] for i in range(i_start, i_end + 1)
            if msg.range_min < msg.ranges[i] < msg.range_max
        ]
        return min(window) if window else float('inf')

    def drive(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        if getattr(self, 'post_tunnel_trace_active', False):
            _fseq = getattr(self, '_ctrl_frame_seq', 0)
            self.get_logger().info(
                f'[CMD_VEL_ASSIGN] frame={_fseq} linear.x={msg.linear.x:.4f} angular.z={msg.angular.z:.4f}'
            )
        self.pub_cmd.publish(msg)

    def stop(self):
        self.pub_cmd.publish(Twist())

    def log_drive_lane(self, left_x, right_x, target_x, conf, err, v, w, front,
                       scene=None, median=None, dark_ratio=None):
        scene = scene or getattr(self, 'scene_state', 'NORMAL')
        median = median if median is not None else getattr(self, 'scene_median', 0.0)
        dark_ratio = dark_ratio if dark_ratio is not None else getattr(self, 'dark_pixel_ratio', 0.0)
        """Throttled drive logging ~1 Hz for normal LANE perception mode."""
        now = time.time()
        if now - self._last_log_time < 1.0:
            return
        self._last_log_time = now

        l_str = f'{left_x:6.1f}' if left_x is not None else '  None'
        r_str = f'{right_x:6.1f}' if right_x is not None else '  None'
        t_str = f'{target_x:6.1f}' if target_x is not None else '  None'
        f_str = f'{front:5.2f}m' if math.isfinite(front) else '  inf'

        self.get_logger().info(
            f'DRIVE scene={scene} perception=LANE median={median:5.1f} dark_ratio={dark_ratio:.3f} '
            f'left_x={l_str} right_x={r_str} target_x={t_str} '
            f'confidence={conf:.2f} error={err:+.2f} '
            f'v={v:.2f} w={w:+.2f} front_distance={f_str}')

    def log_drive_surface(self, coverage, target_x, valid_rows, err, v, w, front,
                          scene=None, median=None, dark_ratio=None):
        scene = scene or getattr(self, 'scene_state', 'NORMAL')
        median = median if median is not None else getattr(self, 'scene_median', 0.0)
        dark_ratio = dark_ratio if dark_ratio is not None else getattr(self, 'dark_pixel_ratio', 0.0)
        """Throttled drive logging ~1 Hz for bright SURFACE perception mode."""
        now = time.time()
        if now - self._last_log_time < 1.0:
            return
        self._last_log_time = now

        t_str = f'{target_x:6.1f}' if target_x is not None else '  None'
        f_str = f'{front:5.2f}m' if math.isfinite(front) else '  inf'

        self.get_logger().info(
            f'DRIVE scene={scene} perception=SURFACE median={median:5.1f} dark_ratio={dark_ratio:.3f} '
            f'coverage={coverage:.2f} target_x={t_str} '
            f'valid_rows={valid_rows:2d} error={err:+.2f} '
            f'v={v:.2f} w={w:+.2f} front_distance={f_str}')

    def check_physical_slope(self) -> bool:
        """Evaluates whether the robot is physically on a ramp/slope using IMU pitch with hysteresis."""
        current_pitch = abs(getattr(self, 'pitch_deg', 0.0))
        enter_thresh = getattr(self, 'ramp_enter_pitch_deg', 2.0)
        exit_thresh = getattr(self, 'ramp_exit_pitch_deg', 1.2)
        if not getattr(self, 'ramp_slope_active', False):
            if current_pitch >= enter_thresh:
                self.ramp_slope_active = True
        else:
            if current_pitch < exit_thresh:
                self.ramp_slope_active = False
        return self.ramp_slope_active

    def log_surface_speed_arbitration(self, pitch, slope_confirmed, target_x, error,
                                      normal_v, final_v, reason):
        """Throttled logging for SURFACE speed arbitration (~1 Hz)."""
        now = time.time()
        if now - self._last_log.get('surface_speed_arb', 0.0) < 1.0:
            return
        self._last_log['surface_speed_arb'] = now
        slope_str = 'YES' if slope_confirmed else 'NO'
        t_str = f'{target_x:.1f}' if target_x is not None else 'None'
        if slope_confirmed:
            self.get_logger().info(
                f'SURFACE_SPEED_ARB pitch={pitch:+.1f} slope_confirmed={slope_str} '
                f'target={t_str} final_v={final_v:.2f} reason={reason}')
        else:
            self.get_logger().info(
                f'SURFACE_SPEED_ARB pitch={pitch:+.1f} slope_confirmed={slope_str} '
                f'target={t_str} error={error:+.2f} normal_v={normal_v:.2f} '
                f'final_v={final_v:.2f} reason={reason}')

    def log_dark_row_reject(self, row_y, center, median, reason):
        """Throttled logging for rejected DARK scan row (~1 Hz)."""
        now = time.time()
        if now - self._last_log.get('dark_row_reject', 0.0) >= 1.0:
            self._last_log['dark_row_reject'] = now
            c_str = f'{center:.1f}' if center is not None else 'None'
            m_str = f'{median:.1f}' if median is not None else 'None'
            self.get_logger().info(
                f'DARK_ROW_REJECT row={row_y} center={c_str} median={m_str} reason={reason}')

    def log_dark_geometry_disagree(self, near, far, median, using):
        """Throttled logging for near/far geometry disagreement (~1 Hz)."""
        now = time.time()
        if now - self._last_log.get('dark_disagree', 0.0) >= 1.0:
            self._last_log['dark_disagree'] = now
            self.get_logger().info(
                f'DARK_GEOMETRY_DISAGREE near={near:.1f} far={far:.1f} median={median:.1f} using={using}')

    def log_dark_hold(self, frame, total, target, v):
        """Throttled logging for DARK_HOLD mode (~1 Hz)."""
        now = time.time()
        if now - self._last_log.get('dark_hold', 0.0) >= 1.0:
            self._last_log['dark_hold'] = now
            t_str = f'{target:.1f}' if target is not None else 'None'
            self.get_logger().info(
                f'DARK_HOLD frame={frame}/{total} target={t_str} v={v:.2f}')

    def log_drive_dark_lane(self, brightness, valid_rows, near_center, far_center,
                            target_x, err, v, w, front, skipped_transverse_count=0,
                            scene=None, median=None, dark_ratio=None, perception_mode='DARK_LANE'):
        scene = scene or getattr(self, 'scene_state', 'DARK')
        median = median if median is not None else (brightness if brightness != 0.0 else getattr(self, 'scene_median', 0.0))
        dark_ratio = dark_ratio if dark_ratio is not None else getattr(self, 'dark_pixel_ratio', 0.0)
        """Throttled drive logging ~1 Hz for dedicated DARK_LANE / DARK_HOLD perception mode."""
        now = time.time()
        if now - self._last_log_time < 1.0:
            return
        self._last_log_time = now

        med_val = median if median is not None else brightness
        t_str = f'{target_x:6.1f}' if target_x is not None else '  None'
        nc_str = f'{near_center:6.1f}' if near_center is not None else '  None'
        fc_str = f'{far_center:6.1f}' if far_center is not None else '  None'
        f_str = f'{front:5.2f}m' if math.isfinite(front) else '  inf'

        dual_count = sum(1 for r in getattr(self, 'last_dark_rows', [])
                         if getattr(r, 'status', '') == 'ACCEPTED' and getattr(r, 'boundary_type', '') == 'DUAL')
        single_count = sum(1 for r in getattr(self, 'last_dark_rows', [])
                          if getattr(r, 'status', '') == 'ACCEPTED' and getattr(r, 'boundary_type', '') == 'SINGLE')
        rows_total = len(getattr(self, 'last_dark_rows', [])) or self.dark_scan_rows

        self.get_logger().info(
            f'{perception_mode} rows_total={rows_total} rows_accepted={valid_rows:2d} dual={dual_count} single={single_count} '
            f'median={med_val:5.1f} near={nc_str} far={fc_str} target={t_str} error={err:+.2f} '
            f'v={v:.2f} w={w:+.2f} front_distance={f_str} '
            f'transverse_rows_skipped={skipped_transverse_count}')

        dual_rows_info = [
            f'(y={r.row_y},l={r.left_x:.0f},r={r.right_x:.0f},c={r.row_center:.1f})'
            for r in getattr(self, 'last_dark_rows', [])
            if getattr(r, 'status', '') == 'ACCEPTED' and getattr(r, 'boundary_type', '') == 'DUAL'
            and getattr(r, 'left_x', None) is not None and getattr(r, 'right_x', None) is not None
        ]
        if dual_rows_info:
            ox = getattr(self, 'x', 0.0) or 0.0
            oy = getattr(self, 'y', 0.0) or 0.0
            raw_t = getattr(self, 'dark_raw_target', 0.0) or 0.0
            pitch_val = getattr(self, 'pitch_deg', 0.0) or 0.0
            self.get_logger().info(
                f'DARK_DUAL_DETAIL odom=({ox:.2f},{oy:.2f}) target={t_str.strip()} raw={raw_t:.1f} '
                f'v={v:.2f} w={w:+.2f} pitch={pitch_val:+.1f} ' + ' '.join(dual_rows_info)
            )

    def log_drive(self, left_x, right_x, target_x, conf, err, v, w, front):
        """Backward-compatible logging delegating to log_drive_lane."""
        self.log_drive_lane(left_x, right_x, target_x, conf, err, v, w, front)

    def log_stop(self, reason, front=float('inf'), conf=0.0, cluster=None,
                 perception=None, valid_rows=0, brightness=0.0,
                 scene=None, dark_ratio=None, **kwargs):
        scene = scene or getattr(self, 'scene_state', 'NORMAL')
        dark_ratio = dark_ratio if dark_ratio is not None else getattr(self, 'dark_pixel_ratio', 0.0)
        """Throttled stop logging ~1 Hz reporting only actual reason."""
        now = time.time()
        if now - self._last_log_time < 1.0:
            return
        self._last_log_time = now

        f_str = f'{front:5.2f}m' if math.isfinite(front) else '  inf'
        if reason == 'NO_IMAGE':
            self.get_logger().info(f'STOP reason=NO_IMAGE front_distance={f_str}')
        elif reason == 'DARK_NOT_READY':
            self.get_logger().info(
                f'STOP reason=DARK_NOT_READY scene={scene} perception=NONE '
                f'pending_frames={getattr(self, "dark_pending_count", 0)}/{getattr(self, "dark_pending_max_frames", 12)} '
                f'front_distance={f_str}')
        elif reason == 'LOW_CONFIDENCE':
            if perception == 'DARK_LANE':
                self.get_logger().info(
                    f'STOP reason=LOW_CONFIDENCE scene={scene} perception=DARK_LANE '
                    f'valid_rows={valid_rows:2d} median={brightness:5.1f} dark_ratio={dark_ratio:.3f} front_distance={f_str}')
            else:
                self.get_logger().info(
                    f'STOP reason=LOW_CONFIDENCE scene={scene} perception=NONE '
                    f'confidence={conf:.2f} front_distance={f_str}')
        elif reason == 'FRONT_OBSTACLE':
            pitch_str = f' imu_pitch_deg={self.pitch_deg:+.1f}' if hasattr(self, 'pitch_deg') else ''
            if cluster:
                w_str = f' cluster_width={cluster.get("lateral_width_m", 0.0):.3f}m' if "lateral_width_m" in cluster else ''
                self.get_logger().info(
                    f'STOP reason=FRONT_OBSTACLE front_distance={f_str} '
                    f'cluster_rays={cluster["count"]} cluster_median={cluster["median_range"]:.2f}m '
                    f'cluster_min={cluster["min_range"]:.2f}m{w_str} '
                    f'angle_start={cluster["start_angle"]:.1f} angle_end={cluster["end_angle"]:.1f}{pitch_str}')
            else:
                self.get_logger().info(f'STOP reason=FRONT_OBSTACLE front_distance={f_str}{pitch_str}')
        else:
            self.get_logger().info(f'STOP reason={reason}')

    def log_crossing_transition(self, old_state, new_state, reason=None):
        reason_str = f' reason={reason}' if reason else ''
        self.get_logger().info(f'CROSSING_TRANSITION {old_state}->{new_state}{reason_str}')
        display_old = 'STOP' if old_state == 'WAIT' else old_state
        display_new = 'STOP' if new_state == 'WAIT' else new_state
        self.get_logger().info(f'CROSSING_STATE: {display_old} -> {display_new}{reason_str}')

    def log_crossing_occupancy(self, point_count, cluster_count, nearest_x, y_span, occupied):
        now = self._get_current_time_sec()
        state_changed = (getattr(self, '_last_occ_logged_state', None) != occupied)
        time_elapsed = (now - getattr(self, '_last_crossing_occ_log_time', 0.0) >= 1.0)
        if state_changed or (occupied and time_elapsed):
            self._last_crossing_occ_log_time = now
            self._last_occ_logged_state = occupied
            occ_str = 'true' if occupied else 'false'
            nx_str = f'{nearest_x:.2f}' if (nearest_x is not None and math.isfinite(nearest_x)) else 'inf'
            ys_str = f'{y_span:.2f}' if (y_span is not None and math.isfinite(y_span)) else '0.00'
            self.get_logger().info(
                f'CROSSING_OCCUPANCY point_count={point_count} cluster_count={cluster_count} '
                f'nearest_x={nx_str} y_span={ys_str} occupied={occ_str}'
            )

    def log_crossing_clear_hold(self, duration, required):
        self.get_logger().info(
            f'CROSSING_CLEAR_HOLD duration={duration:.2f}s required={required:.2f}s'
        )

    def log_crossing_zebra_detected(self, bands, score):
        now = self._get_current_time_sec()
        if now - getattr(self, '_last_zebra_detect_log_time', 0.0) >= 1.0:
            self._last_zebra_detect_log_time = now
            self.get_logger().info(f'CROSSING_ZEBRA_DETECTED bands={bands} score={score:.2f}')

    def log_crossing_track(self, x, y, prev_x, prev_y):
        now = time.time()
        if now - self._last_crossing_track_log_time >= 1.0:
            self._last_crossing_track_log_time = now
            px_str = f'{prev_x:.2f}' if prev_x is not None else 'None'
            py_str = f'{prev_y:.2f}' if prev_y is not None else 'None'
            self.get_logger().info(
                f'CROSSING_TRACK x={x:.2f} y={y:.2f} prev_x={px_str} prev_y={py_str}'
            )

    def log_crossing_entry(self, inward_delta, confirm, total):
        now = time.time()
        if now - self._last_crossing_entry_log_time >= 1.0:
            self._last_crossing_entry_log_time = now
            self.get_logger().info(
                f'CROSSING_ENTRY inward_delta={inward_delta:.3f} confirm={confirm}/{total}'
            )

    def log_crossing_entry_threat(self, x, y, confirm, total):
        now = time.time()
        if now - self._last_crossing_threat_log_time >= 1.0:
            self._last_crossing_threat_log_time = now
            self.get_logger().info(
                f'CROSSING_ENTRY_THREAT x={x:.2f} y={y:.2f} confirm={confirm}/{total}'
            )

    def log_crossing_hard_occupied(self, rays, x, y, r_min=None, confirm=None, total=None, scan_seq=None):
        now = self._get_current_time_sec()
        if now - self._last_crossing_log_time >= 1.0:
            self._last_crossing_log_time = now
            extra = ''
            if r_min is not None:
                extra += f' range={r_min:.2f}'
            if scan_seq is not None:
                extra += f' scan_seq={scan_seq}'
            if confirm is not None and total is not None:
                extra += f' confirm={confirm}/{total}'
            self.get_logger().info(
                f'CROSSING_HARD_OCCUPIED rays={rays} x={x:.2f} y={y:.2f}{extra}'
            )

    def log_crossing_kerb_stationary(self, x, y):
        now = time.time()
        if now - self._last_crossing_kerb_log_time >= 1.0:
            self._last_crossing_kerb_log_time = now
            self.get_logger().info(f'CROSSING_KERB_STATIONARY x={x:.2f} y={y:.2f}')

    def log_crossing_pass(self, distance, min_distance):
        now = time.time()
        if now - self._last_crossing_pass_log_time >= 1.0:
            self._last_crossing_pass_log_time = now
            self.get_logger().info(
                f'CROSSING_PASS distance={distance:.2f}/{min_distance:.2f}'
            )

    def log_crossing_scan(self, seq, hit=False, nearest_x=float('inf'), nearest_y=float('inf'),
                          occupied_count=0, clear_count=0, status='CLEAR', **kwargs):
        """Diagnostic log for every NEW LiDAR scan while crossing is active."""
        if isinstance(hit, bool):
            hit_str = 'true' if hit else 'false'
            nx_str = f'{nearest_x:.2f}' if (nearest_x is not None and math.isfinite(nearest_x)) else 'inf'
            ny_str = f'{nearest_y:.2f}' if (nearest_y is not None and math.isfinite(nearest_y)) else 'inf'
            self.get_logger().info(
                f'CROSSING_SCAN seq={seq} hit={hit_str} nearest_x={nx_str} nearest_y={ny_str} '
                f'occupied_count={occupied_count} clear_count={clear_count} status={status}'
            )
        else:
            self.get_logger().info(f'CROSSING_SCAN seq={seq} status={hit}')

    def log_crossing_exit(self, reason='ZEBRA_PASSED'):
        self.get_logger().info(f'CROSSING_EXIT reason={reason}')

    def log_crossing_scan_skip(self, seq, reason='ALREADY_PROCESSED'):
        now = self._get_current_time_sec()
        if now - self._last_crossing_scan_skip_log_time >= 2.0:
            self._last_crossing_scan_skip_log_time = now
            self.get_logger().debug(f'CROSSING_SCAN_SKIP seq={seq} reason={reason}')

    def log_crossing_unknown(self, reason, age=None):
        now = self._get_current_time_sec()
        if now - self._last_crossing_unknown_log_time >= 1.0:
            self._last_crossing_unknown_log_time = now
            age_str = f' age={age:.3f}' if age is not None else ''
            self.get_logger().info(f'CROSSING_UNKNOWN reason={reason}{age_str}')

    def log_zebra_pattern(self, bands, gaps, spacing_mean, spacing_cv,
                          height_mean, height_cv, coverage, score, accepted, reject_reason):
        now = self._get_current_time_sec()
        if now - self._last_zebra_pattern_log_time >= 1.0:
            self._last_zebra_pattern_log_time = now
            acc_str = 'true' if accepted else 'false'
            self.get_logger().info(
                f'ZEBRA_PATTERN bands={bands} gaps={gaps} spacing_mean={spacing_mean:.2f} '
                f'spacing_cv={spacing_cv:.2f} height_mean={height_mean:.2f} height_cv={height_cv:.2f} '
                f'coverage={coverage:.2f} score={score:.2f} accepted={acc_str} reject_reason={reject_reason}'
            )

    def log_crossing_cluster(self, scan_seq, raw_corridor_points, cluster_count,
                             best_cluster_rays, best_cluster_width,
                             nearest_x, nearest_y, accepted, reject_reason):
        now = self._get_current_time_sec()
        if now - self._last_crossing_cluster_log_time >= 1.0 or accepted:
            self._last_crossing_cluster_log_time = now
            acc_str = 'true' if accepted else 'false'
            nx_str = f'{nearest_x:.2f}' if math.isfinite(nearest_x) else 'inf'
            ny_str = f'{nearest_y:.2f}' if math.isfinite(nearest_y) else 'inf'
            self.get_logger().info(
                f'CROSSING_CLUSTER scan_seq={scan_seq} raw_corridor_points={raw_corridor_points} '
                f'cluster_count={cluster_count} best_cluster_rays={best_cluster_rays} '
                f'best_cluster_width={best_cluster_width:.3f} nearest_x={nx_str} nearest_y={ny_str} '
                f'accepted={acc_str} reject_reason={reject_reason}'
            )

    def log_crossing_clear(self, clear_count, clear_total, scan_seq=None):
        seq_str = f' scan_seq={scan_seq}' if scan_seq is not None else ''
        self.get_logger().info(f'CROSSING_CLEAR{seq_str} clear={clear_count}/{clear_total}')

    def log_crossing_zebra(self, bands, score, confirm, total):
        now = time.time()
        if now - self._last_crossing_log_time >= 1.0:
            self._last_crossing_log_time = now
            self.get_logger().info(
                f'CROSSING_ZEBRA bands={bands} score={score:.2f} confirmed={confirm}/{total}'
            )

    def log_crossing_occupied(self, rays, r_min, x, y, confirm, total):
        self.log_crossing_hard_occupied(rays, x, y, r_min=r_min, confirm=confirm, total=total)

    def log_crossing_wait(self, occupied, clear_count, clear_total, front=float('inf')):
        now = time.time()
        if now - self._last_crossing_wait_log_time >= 1.0:
            self._last_crossing_wait_log_time = now
            occ_str = 'true' if occupied else 'false'
            self.get_logger().info(
                f'CROSSING_WAIT occupied={occ_str} clear={clear_count}/{clear_total}'
            )

    def log_crossing_reject_lateral(self, r_min, x, y):
        now = time.time()
        if now - self._last_crossing_reject_lateral_time >= 1.0:
            self._last_crossing_reject_lateral_time = now
            self.get_logger().info(
                f'CROSSING_REJECT_LATERAL range={r_min:.2f} x={x:.2f} y={y:.2f} half_width={self.crossing_half_width_m:.2f}'
            )

    def log_crossing_reject_single(self, r_min, x, y):
        now = time.time()
        if now - self._last_crossing_reject_single_time >= 1.0:
            self._last_crossing_reject_single_time = now
            self.get_logger().info(
                f'CROSSING_REJECT_SINGLE range={r_min:.2f} x={x:.2f} y={y:.2f}'
            )

    def log_crossing_status(self, context, lidar_status, occupied_count, clear_count):
        """Rate-limited (1 Hz) high-level crossing diagnostic log."""
        now = self._get_current_time_sec()
        if now - self._last_crossing_status_log_time >= 1.0:
            self._last_crossing_status_log_time = now
            self.get_logger().info(
                f'CROSSING context={context} lidar={lidar_status} '
                f'occupied_count={occupied_count} clear_count={clear_count}'
            )

    def log_crossing_stop(self, reason='ROAD_OCCUPIED'):
        self.get_logger().info(f'CROSSING_STOP reason={reason}')

    def log_crossing_release(self, reason='ROAD_CLEAR'):
        self.get_logger().info(f'CROSSING_RELEASE reason={reason}')

    @property
    def crossing_occupied_confirm_count(self):
        return self.crossing_detect_count

    @crossing_occupied_confirm_count.setter
    def crossing_occupied_confirm_count(self, val):
        self.crossing_detect_count = val

    # --- control loop -------------------------------------------------------

    def tick(self):
        try:
            self.control()
        except Exception as e:
            self.get_logger().error(f'control() raised: {e}', throttle_duration_sec=2.0)
            self.stop()

    def compute_scene_metrics(self, img):
        """Calculate darkness and brightness metrics within forward-looking scene ROI.

        Dedicated Scene ROI: [scene_roi_top_ratio .. scene_roi_bottom_ratio]
        For 480p: rows 120..336. Looks farther ahead to detect tunnel entrance early.
        """
        if img is None:
            return 0.0, 0.0, 0.0, 0.0

        h, w = img.shape[:2]
        y_top = int(h * self.scene_roi_top_ratio)
        y_bot = int(h * self.scene_roi_bottom_ratio)
        scene_roi = img[y_top:y_bot, 0:w]

        if len(scene_roi.shape) == 3:
            scene_gray = cv2.cvtColor(scene_roi, cv2.COLOR_BGR2GRAY)
        else:
            scene_gray = scene_roi

        scene_mean = float(np.mean(scene_gray))
        scene_median = float(np.median(scene_gray))
        scene_p25 = float(np.percentile(scene_gray, 25))
        dark_pixel_ratio = float(np.sum(scene_gray < self.dark_pixel_threshold)) / float(scene_gray.size)

        return scene_mean, scene_median, scene_p25, dark_pixel_ratio

    def update_scene_state(self, img_or_gray):
        """Update scene classification (NORMAL vs DARK) with temporal hysteresis.

        Forward-looking Scene ROI:
          [scene_roi_top_ratio .. scene_roi_bottom_ratio] (rows 120..336 on 480p).

        Combined darkness measurement:
          - scene_mean, scene_median, scene_p25
          - dark_pixel_ratio = pixels(gray < dark_pixel_threshold) / total_roi_pixels

        NORMAL -> DARK Entry Rule:
          Frame candidate if:
            dark_pixel_ratio >= dark_ratio_enter (0.28)
            OR (scene_median <= dark_median_enter (23.0) AND scene_p25 <= dark_pixel_threshold (20.0))
          Requires dark_enter_frames (2) consecutive candidate frames to enter DARK.

        DARK -> NORMAL Exit Hysteresis:
          Frame candidate only if BOTH:
            dark_pixel_ratio < dark_ratio_exit (0.20)
            AND scene_median > dark_median_exit (27.0)
          Requires dark_exit_frames (8) consecutive exit frames before returning to NORMAL.
          Any dark frame resets the exit counter, preventing light flicker from exiting DARK.
        """
        if img_or_gray is None:
            return self.scene_state, 0.0, 0.0, 0.0, 0.0

        scene_mean, scene_median, scene_p25, dark_pixel_ratio = self.compute_scene_metrics(img_or_gray)
        self.scene_mean = scene_mean
        self.scene_median = scene_median
        self.scene_p25 = scene_p25
        self.dark_pixel_ratio = dark_pixel_ratio

        if self.scene_state == 'NORMAL':
            is_dark_candidate = (
                dark_pixel_ratio >= self.dark_ratio_enter or
                (scene_median <= self.dark_median_enter and scene_p25 <= float(self.dark_pixel_threshold))
            )
            if is_dark_candidate:
                self.dark_enter_count += 1
                if self.dark_enter_count >= self.dark_enter_frames:
                    self.scene_state = SceneState('DARK_PENDING')
                    self.neutralize_crossing_context('DARK_ENTRY_PENDING')
                    self.dark_exit_count = 0
                    self.dark_pending_count = 0
                    self.dark_ready_count = 0
                    self._reset_dark_pending_grace()
                    self.prev_dark_target_x = None
                    self.last_valid_dark_target_x = None
                    self.last_good_dark_target = None
                    self.last_good_dark_w = 0.0
                    self.last_good_dark_stamp = None
                    self.last_good_dark_odom_x = None
                    self.last_good_dark_odom_y = None
                    self.dark_recovery_active = False
                    self.dark_recovery_expired_latched = False
                    self.dark_hold_count = 0
                    self.dark_entry_stabilize_count = 0
                    anchor = self.last_stable_normal_target_x if self.last_stable_normal_target_x is not None else (float(img_or_gray.shape[1]) / 2.0 if hasattr(img_or_gray, 'shape') else 320.0)
                    self.last_stable_normal_target_x = anchor
                    self.get_logger().info(
                        f'DARK_TRANSITION NORMAL->PENDING anchor_target={anchor:.1f} '
                        f'forward_dark_ratio={dark_pixel_ratio:.3f}'
                    )
            else:
                self.dark_enter_count = 0
        elif self.scene_state == 'DARK_PENDING':
            is_exit_candidate = (
                dark_pixel_ratio < self.dark_ratio_exit and
                scene_median > self.dark_median_exit
            )
            if is_exit_candidate:
                self.dark_exit_count += 1
                if self.dark_exit_count >= self.dark_exit_frames:
                    self.scene_state = SceneState('NORMAL')
                    self.dark_enter_count = 0
                    self.dark_pending_count = 0
                    self.dark_ready_count = 0
                    self.dark_exit_count = 0
                    self._reset_dark_pending_grace()
                    self.get_logger().info(
                        f'DARK_TRANSITION PENDING->NORMAL median={scene_median:.1f} '
                        f'p25={scene_p25:.1f} dark_ratio={dark_pixel_ratio:.3f}'
                    )
            else:
                self.dark_exit_count = 0
        else:  # self.scene_state in ('DARK_ACTIVE', 'DARK')
            is_exit_candidate = (
                dark_pixel_ratio < self.dark_ratio_exit and
                scene_median > self.dark_median_exit
            )
            if is_exit_candidate:
                self.dark_exit_count += 1
                if self.dark_exit_count >= self.dark_exit_frames:
                    self.scene_state = SceneState('NORMAL')
                    if not getattr(self, 'post_tunnel_assist_consumed', False):
                        self.post_tunnel_assist_armed = True
                        # --- DIAGNOSTIC: Capture reference pose at tunnel exit ---
                        if not getattr(self, '_post_tunnel_ref_latched', False):
                            self._post_tunnel_ref_latched = True
                            self.post_tunnel_ref_x = getattr(self, 'x', 0.0)
                        self.post_tunnel_ref_y = getattr(self, 'y', 0.0)
                        self.post_tunnel_ref_yaw = getattr(self, 'yaw', 0.0)
                        import time as _diag_time
                        self.post_tunnel_ref_time = _diag_time.time()
                        self.post_tunnel_trace_active = True
                        self._post_tunnel_ref_logged = False
                        # ---------------------------------------------------------
                        self.post_tunnel_ready_frames = 0
                        self.post_tunnel_junction_risk_seen = False
                        self.post_tunnel_curve_boost_active = False
                        self.post_tunnel_curve_candidate_count = 0
                        self.post_tunnel_curve_dropout_frames = 0
                        self.post_tunnel_curve_exit_stable_frames = 0
                        self.post_tunnel_curve_boost_consumed = False
                        self.post_curve_reacquire_active = False
                        self.post_curve_reacquire_stable_count = 0
                        self.post_curve_reacquire_last_target = None
                        self.post_reacquire_protect_active = False
                        self.post_reacquire_reference_target = None
                        self.post_reacquire_protected_target = None
                        self.post_reacquire_dual_streak = 0
                        self.post_reacquire_single_streak = 0
                        self.post_reacquire_trusted_width = None
                        self.post_reacquire_trusted_right = None
                        self.post_reacquire_trusted_target = None
                        self.post_reacquire_right_missing_count = 0
                        self.right_cont_start_x = None
                        self.right_cont_start_y = None
                        self.curve_containment_scale = 1.0
                        self.last_trusted_near_left = None
                        self.last_trusted_near_right = None
                        self.last_trusted_near_target = None
                        self.last_trusted_near_width = None
                        self.apex_guard_hold_count = 0
                        self.apex_quarantine_active = False
                        self.apex_recovery_streak = 0
                        self._last_apex_guard_diag = None
                        self._apex_guard_was_triggered = False
                        self.simple_curve_state = 'IDLE'
                        self.simple_curve_history.clear()
                        self.simple_curve_held_steering = 0.0
                        self.simple_curve_hold_start_x = None
                        self.simple_curve_hold_start_y = None
                        self.simple_curve_recovery_frames = 0
                        self.get_logger().info('POST_TUNNEL_ASSIST_ARMED')
                    self.dark_enter_count = 0
                    self.dark_pending_count = 0
                    self.dark_ready_count = 0
                    self.dark_exit_count = 0
                    self._reset_dark_pending_grace()
                    self.last_valid_dark_target_x = None
                    self.last_good_dark_target = None
                    self.last_good_dark_w = 0.0
                    self.last_good_dark_stamp = None
                    self.last_good_dark_odom_x = None
                    self.last_good_dark_odom_y = None
                    self.dark_recovery_active = False
                    self.dark_hold_count = 0
                    self.dark_entry_stabilize_count = 0
                    self.prev_dark_target_x = None
                    self.get_logger().info(
                        f'SCENE_TRANSITION DARK->NORMAL median={scene_median:.1f} '
                        f'p25={scene_p25:.1f} dark_ratio={dark_pixel_ratio:.3f}'
                    )
            else:
                self.dark_exit_count = 0

        return self.scene_state, scene_mean, scene_median, scene_p25, dark_pixel_ratio

    def _choose_dark_lane_pair(self, runs, r, expected_center, expected_width, max_center_dev=None):
        """Robust distinct candidate-pair selection for dark lane perception.

        Evaluates candidate pairs from multiple bright runs using:
          1. Distinct pair enumeration (lx < rx, never the same physical run).
          2. Width plausibility with perspective awareness (rejects narrow half-lane pairs).
          3. Center consistency with expected/trusted lane center.
          4. Neighbor DUAL consistency.
        """
        if len(runs) < 2:
            return None, None

        if max_center_dev is None:
            max_center_dev = getattr(self, 'max_dark_row_center_deviation_px', 60.0)

        # Perspective-aware width limits
        min_width = max(180.0, 0.60 * expected_width)
        max_width = min(630.0, max(540.0, 1.50 * expected_width))

        plausible_pairs = []
        for i in range(len(runs)):
            for j in range(i + 1, len(runs)):
                run_l = runs[i]
                run_r = runs[j]
                lx = float(run_l[0])
                rx = float(run_r[0])
                if lx >= rx:
                    continue

                pair_w = rx - lx
                pair_c = (lx + rx) / 2.0
                center_err = abs(pair_c - expected_center)

                # Reject obviously narrow pairs (lane boundary + center dash)
                if pair_w < min_width or pair_w > max_width:
                    continue

                # Reject pairs with large center error
                if center_err > max_center_dev:
                    continue

                # Scoring:
                # 1. Center consistency (primary)
                # 2. Perspective width consistency (secondary)
                # 3. Straddle penalty if pair does not straddle expected center
                straddle_bonus = 0.0 if (lx < expected_center and rx > expected_center) else 50.0
                cost = center_err + 0.10 * abs(pair_w - expected_width) + straddle_bonus
                plausible_pairs.append((cost, run_l, run_r, pair_c, pair_w))

        if not plausible_pairs:
            # Preserve simple two-run behavior if two clean outer runs exist
            if len(runs) == 2:
                run_l, run_r = runs[0], runs[1]
                lx, rx = float(run_l[0]), float(run_r[0])
                if lx < rx and (rx - lx) >= 180.0 and abs(((lx + rx) / 2.0) - expected_center) <= max_center_dev:
                    return run_l, run_r
            return None, None

        plausible_pairs.sort(key=lambda x: x[0])
        best = plausible_pairs[0]
        return best[1], best[2]

    def find_dark_lane_target(self, img):
        """Dedicated DARK_LANE perception pipeline for dim tunnel traversal.

        Robust Multi-Stage Pipeline:
          1. Preprocess: Mild CLAHE and crop dedicated DARK ROI [dark_roi_top_ratio..dark_roi_bottom_ratio].
          2. Binary threshold and morphological opening.
          3. Horizontal scan across dark_scan_rows:
             - Group consecutive bright pixels into runs (min 5 px, max 120 px).
             - Reject rows where bright pixels or any single run exceeds max_transverse_row_ratio (reason: TRANSVERSE).
             - Identify candidate boundaries left (lx) and right (rx).
             - Record boundary_type as DUAL (if both lx and rx present) or SINGLE (if only one present).
             - Retain individual row results: row_y, left_x, right_x, row_center, lane_width, boundary_type.
          4. DUAL vs SINGLE Consensus:
             - DUAL rows have higher trust (weight 1.0 vs 0.35).
             - If valid DUAL rows exist, compute dual consensus median.
             - Reject SINGLE rows that strongly disagree with DUAL consensus (reason: SINGLE_DISAGREE).
          5. Robust Row Consensus:
             - Compute median_center = median(row_centers) from all candidate rows.
             - Reject any row deviating > max_dark_row_center_deviation_px (60 px) (reason: CENTER_OUTLIER).
          6. Far-to-Near Continuity:
             - Step from farther rows toward nearer rows.
             - Reject rows where center shifts > max_dark_adjacent_center_delta_px (55 px) from previous accepted row (reason: GEOMETRY).
          7. Minimum Valid Rows:
             - If accepted_rows < min_dark_valid_rows (3): invalid target -> return None.
          8. Tunnel Entry Stabilization & Near/Far Lookahead Disagreement Protection:
             - If in initial dark_entry_stabilize_frames (10 frames): prefer far/mid accepted rows.
             - In steady state: check abs(near_center - far_center) <= max_dark_near_far_disagreement_px (70 px).
             - If <= 70 px: compute lookahead target = near_weight * near_center + far_weight * far_center.
             - If > 70 px: use robust_median_center and log DARK_GEOMETRY_DISAGREE.
          9. Temporal EMA Smoothing:
             - Applied strictly AFTER outlier rejection: filtered = alpha * raw + (1 - alpha) * prev.
        """
        if img is None or not HAVE_CV:
            return None, None, None, None, [], [], None, None

        h, w = img.shape[:2]
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Mild CLAHE
        clahe = cv2.createCLAHE(clipLimit=self.dark_clahe_clip, tileGridSize=(8, 8))
        enhanced = clahe.apply(gray)

        # Crop DARK ROI
        y_top = int(h * self.dark_roi_top_ratio)
        y_bot = int(h * self.dark_roi_bottom_ratio)
        if y_bot <= y_top:
            return None, None, None, None, [], [], None, None

        dark_roi = enhanced[y_top:y_bot, :]
        h_dark_roi, w_dark_roi = dark_roi.shape[:2]

        # Binary threshold & morphological cleanup
        _, dark_thresh = cv2.threshold(dark_roi, self.dark_white_threshold, 255, cv2.THRESH_BINARY)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        dark_clean = cv2.morphologyEx(dark_thresh, cv2.MORPH_OPEN, kernel)

        # Scan rows
        scan_indices = np.linspace(5, h_dark_roi - 6, self.dark_scan_rows, dtype=int)
        min_run_len = 5
        max_transverse_px = int(w_dark_roi * self.max_transverse_row_ratio)
        image_center_x = float(w_dark_roi) / 2.0

        all_rows = []
        skipped_transverse = []

        # Pass 1: Extract runs and detect transverse rows across all scan indices
        row_runs = {}
        cand_dual_rows = []

        for r in scan_indices:
            row = dark_clean[r, :]
            total_bright = int(np.sum(row > 0))
            if total_bright > max_transverse_px:
                row_runs[r] = (None, 'TRANSVERSE')
                continue

            diffs = np.diff(np.pad((row > 0).astype(np.int32), (1, 1), 'constant'))
            starts = np.where(diffs == 1)[0]
            ends = np.where(diffs == -1)[0]

            runs = []
            row_contaminated = False
            for s, e in zip(starts, ends):
                length = e - s
                if length > max_transverse_px:
                    row_contaminated = True
                    break
                if min_run_len <= length <= 120:
                    c = (s + e - 1) / 2.0
                    runs.append((c, s, e, length))

            if row_contaminated or not runs:
                reason = 'TRANSVERSE' if row_contaminated else 'NO_CANDIDATE'
                row_runs[r] = (None, reason)
            else:
                row_runs[r] = (runs, 'OK')
                # Enumerate candidate outer pairs across runs (including multi-run rows)
                if len(runs) >= 2:
                    nom_w = 260.0 + 1.8 * float(r)
                    best_pair = None
                    best_cost = 1e9
                    for i in range(len(runs)):
                        for j in range(i + 1, len(runs)):
                            c0, c1 = float(runs[i][0]), float(runs[j][0])
                            if c0 < c1:
                                pw = c1 - c0
                                if 180.0 <= pw <= 620.0:
                                    pc = (c0 + c1) / 2.0
                                    cost = abs(pw - nom_w)
                                    if cost < best_cost:
                                        best_cost = cost
                                        best_pair = (pc, pw, r)
                    if best_pair is not None:
                        cand_dual_rows.append(best_pair)

        # Cross-row consensus: evaluate candidates against frame consensus median
        frame_dual_centers = []
        frame_dual_widths = []
        if cand_dual_rows:
            cand_median_c = float(np.median([p[0] for p in cand_dual_rows]))
            for pc, pw, r in cand_dual_rows:
                if abs(pc - cand_median_c) <= self.max_dark_row_center_deviation_px:
                    frame_dual_centers.append(pc)
                    frame_dual_widths.append((r, pw))

        # Expected Center Hierarchy:
        # 1. Fresh CURRENT-FRAME trusted DUAL consensus (Authority #1)
        # 2. Decayed historical prior toward image center (Authority #2)
        # 3. Image center fallback (Authority #3)
        if frame_dual_centers:
            expected_center = float(np.median(frame_dual_centers))
        elif self.last_valid_dark_target_x is not None or self.last_good_dark_target is not None or self.prev_dark_target_x is not None:
            prior = self.last_valid_dark_target_x
            if prior is None:
                prior = self.last_good_dark_target
            if prior is None:
                prior = self.prev_dark_target_x
            expected_center = image_center_x + self.dark_expected_center_hist_weight * (float(prior) - image_center_x)
        else:
            expected_center = image_center_x

        # Determine reference width for perspective scaling
        ref_r = 29
        ref_w = max(380.0, 2.0 * self.half_lane_px)
        if frame_dual_widths:
            frame_dual_widths.sort(key=lambda x: x[0])
            ref_r, ref_w = frame_dual_widths[len(frame_dual_widths) // 2]

        # Pass 2: Select boundaries and construct DarkRow for each scan index
        for r in scan_indices:
            runs, status = row_runs[r]
            if runs is None:
                if status == 'TRANSVERSE':
                    skipped_transverse.append(r)
                all_rows.append(DarkRow(r, None, None, None, None, 'NONE', 'REJECTED', status))
                continue

            # Compute perspective expected width for row r
            expected_w = ref_w + 5.0 * (r - ref_r)
            expected_w = max(200.0, min(float(w_dark_roi - 10), expected_w))

            left_run = None
            right_run = None

            if len(runs) >= 2:
                left_run, right_run = self._choose_dark_lane_pair(runs, r, expected_center, expected_w)

            # If no pair was chosen from multi-run or if single-run, preserve single-candidate logic
            if left_run is None and right_run is None:
                left_candidates = [run for run in runs if run[0] < expected_center + 30.0]
                right_candidates = [run for run in runs if run[0] > expected_center - 30.0]

                cand_l = left_candidates[-1] if left_candidates else None
                cand_r = right_candidates[0] if right_candidates else None

                if cand_l is not None and cand_r is not None and cand_l == cand_r:
                    if cand_l[0] < expected_center:
                        cand_r = None
                    else:
                        cand_l = None

                # Disallow resurrection of rejected multi-run pairs into false DUAL
                if len(runs) >= 2 and cand_l is not None and cand_r is not None:
                    cand_l = None
                    cand_r = None

                left_run = cand_l
                right_run = cand_r

            lx = float(left_run[0]) if left_run is not None else None
            rx = float(right_run[0]) if right_run is not None else None

            if lx is not None and rx is not None:
                row_center = (lx + rx) / 2.0
                lane_width = rx - lx
                boundary_type = 'DUAL'
                measured_half = lane_width / 2.0
                if 40.0 < measured_half < 250.0:
                    self.half_lane_px = 0.95 * self.half_lane_px + 0.05 * measured_half
                all_rows.append(DarkRow(r, row_center, lx, rx, lane_width, boundary_type, 'PENDING', 'NONE'))
            elif lx is not None:
                row_center = lx + self.half_lane_px
                lane_width = 2.0 * self.half_lane_px
                boundary_type = 'SINGLE'
                all_rows.append(DarkRow(r, row_center, lx, rx, lane_width, boundary_type, 'PENDING', 'NONE'))
            elif rx is not None:
                row_center = rx - self.half_lane_px
                lane_width = 2.0 * self.half_lane_px
                boundary_type = 'SINGLE'
                all_rows.append(DarkRow(r, row_center, lx, rx, lane_width, boundary_type, 'PENDING', 'NONE'))
            else:
                all_rows.append(DarkRow(r, None, None, None, None, 'NONE', 'REJECTED', 'NO_CANDIDATE'))

        candidate_rows = [row for row in all_rows if row.status == 'PENDING']
        if not candidate_rows:
            self.last_dark_rows = all_rows
            return None, None, None, None, [], skipped_transverse, dark_roi, dark_clean

        # Stage 2: DUAL boundaries have higher trust (Section 4)
        dual_rows = [row for row in candidate_rows if row.boundary_type == 'DUAL']
        if dual_rows:
            dual_consensus = float(np.median([r.row_center for r in dual_rows]))
            for row in candidate_rows:
                if row.boundary_type == 'SINGLE':
                    if abs(row.row_center - dual_consensus) > self.max_dark_row_center_deviation_px:
                        row.status = 'REJECTED'
                        row.reject_reason = 'SINGLE_DISAGREE'
                        self.log_dark_row_reject(row.row_y, row.row_center, dual_consensus, 'SINGLE_DISAGREE')

        # Stage 3: Robust row consensus (Section 3)
        active_candidates = [row for row in candidate_rows if row.status == 'PENDING']
        if active_candidates:
            median_center = float(np.median([r.row_center for r in active_candidates]))
            for row in active_candidates:
                if abs(row.row_center - median_center) > self.max_dark_row_center_deviation_px:
                    row.status = 'REJECTED'
                    row.reject_reason = 'CENTER_OUTLIER'
                    self.log_dark_row_reject(row.row_y, row.row_center, median_center, 'CENTER_OUTLIER')

        # Stage 4: Far-to-near continuity (Section 5)
        active_candidates = [row for row in candidate_rows if row.status == 'PENDING']
        accepted_rows = []
        for row in active_candidates:
            if not accepted_rows:
                row.status = 'ACCEPTED'
                accepted_rows.append(row)
            else:
                prev_center = accepted_rows[-1].row_center
                if abs(row.row_center - prev_center) <= self.max_dark_adjacent_center_delta_px:
                    row.status = 'ACCEPTED'
                    accepted_rows.append(row)
                else:
                    row.status = 'REJECTED'
                    row.reject_reason = 'GEOMETRY'
                    self.log_dark_row_reject(row.row_y, row.row_center, prev_center, 'GEOMETRY')

        # Stage 4b: Adaptive Scan-Row Densification (Underconstrained DARK frame recovery)
        # Trigger criteria:
        # 1. accepted_rows < min_dark_valid_rows (frame needs more valid rows)
        # 2. At least one credible normal accepted row exists (len(accepted_rows) >= 1)
        # 3. Current frame scene is DARK
        if (len(accepted_rows) < self.min_dark_valid_rows and
                len(accepted_rows) >= 1 and
                getattr(self, 'scene_state', 'DARK') in ('DARK', 'DARK_ACTIVE', 'DARK_PENDING')):

            acc_y = accepted_rows[0].row_y
            delta = self.dark_adaptive_band_delta_px
            band_top = max(12, acc_y - delta)
            if skipped_transverse:
                first_transverse = min(skipped_transverse)
                band_bot = min(first_transverse - 8, h_dark_roi - 10)
                raw_candidates = list(np.linspace(band_top, band_bot, 6, dtype=int))
            else:
                band_bot = min(acc_y + delta, h_dark_roi - 10)
                raw_candidates = [acc_y - delta, acc_y - 12, acc_y - 9, acc_y + 9, acc_y + 15, acc_y + delta]

            if band_bot - band_top >= 15:
                # Generate 3-5 supplemental rows with minimum 6px separation from normal rows and each other
                supp_indices = []
                for cand in raw_candidates:
                    if cand < 12 or cand > band_bot:
                        continue
                    if any(abs(cand - nr) < 6 for nr in scan_indices):
                        continue
                    if any(abs(cand - sr) < 6 for sr in supp_indices):
                        continue
                    supp_indices.append(int(cand))

                adaptive_rows = []
                for r in supp_indices:
                    row = dark_clean[r, :]
                    total_bright = int(np.sum(row > 0))
                    if total_bright > max_transverse_px:
                        continue

                    diffs = np.diff(np.pad((row > 0).astype(np.int32), (1, 1), 'constant'))
                    starts = np.where(diffs == 1)[0]
                    ends = np.where(diffs == -1)[0]

                    runs = []
                    row_contaminated = False
                    for s, e in zip(starts, ends):
                        length = e - s
                        if length > max_transverse_px:
                            row_contaminated = True
                            break
                        if min_run_len <= length <= 120:
                            c = (s + e - 1) / 2.0
                            runs.append((c, s, e, length))

                    if row_contaminated or not runs:
                        continue

                    expected_w = ref_w + 5.0 * (r - ref_r)
                    expected_w = max(200.0, min(float(w_dark_roi - 10), expected_w))

                    left_run = None
                    right_run = None
                    if len(runs) >= 2:
                        left_run, right_run = self._choose_dark_lane_pair(runs, r, expected_center, expected_w)

                    if left_run is None and right_run is None:
                        left_candidates = [run for run in runs if run[0] < expected_center + 30.0]
                        right_candidates = [run for run in runs if run[0] > expected_center - 30.0]

                        cand_l = left_candidates[-1] if left_candidates else None
                        cand_r = right_candidates[0] if right_candidates else None

                        if cand_l is not None and cand_r is not None and cand_l == cand_r:
                            if cand_l[0] < expected_center:
                                cand_r = None
                            else:
                                cand_l = None

                        if len(runs) >= 2 and cand_l is not None and cand_r is not None:
                            cand_l = None
                            cand_r = None

                        left_run = cand_l
                        right_run = cand_r

                    lx = float(left_run[0]) if left_run is not None else None
                    rx = float(right_run[0]) if right_run is not None else None

                    if lx is not None and rx is not None:
                        row_center = (lx + rx) / 2.0
                        lane_width = rx - lx
                        boundary_type = 'DUAL'
                        adaptive_rows.append(DarkRow(r, row_center, lx, rx, lane_width, boundary_type, 'PENDING', 'NONE'))
                    elif lx is not None:
                        row_center = lx + self.half_lane_px
                        lane_width = 2.0 * self.half_lane_px
                        boundary_type = 'SINGLE'
                        adaptive_rows.append(DarkRow(r, row_center, lx, rx, lane_width, boundary_type, 'PENDING', 'NONE'))
                    elif rx is not None:
                        row_center = rx - self.half_lane_px
                        lane_width = 2.0 * self.half_lane_px
                        boundary_type = 'SINGLE'
                        adaptive_rows.append(DarkRow(r, row_center, lx, rx, lane_width, boundary_type, 'PENDING', 'NONE'))

                if adaptive_rows:
                    combined = [r for r in all_rows if r.status in ('PENDING', 'ACCEPTED')] + adaptive_rows
                    combined.sort(key=lambda x: x.row_y)

                    # Deduplicate rows too close in Y (min 5 px)
                    deduped = []
                    for row in combined:
                        if not deduped or abs(row.row_y - deduped[-1].row_y) >= 5:
                            row.status = 'PENDING'
                            deduped.append(row)

                    # Same Stage 2: DUAL vs SINGLE consensus
                    dual_rows = [row for row in deduped if row.boundary_type == 'DUAL']
                    if dual_rows:
                        dual_consensus = float(np.median([r.row_center for r in dual_rows]))
                        for row in deduped:
                            if row.boundary_type == 'SINGLE':
                                if abs(row.row_center - dual_consensus) > self.max_dark_row_center_deviation_px:
                                    row.status = 'REJECTED'
                                    row.reject_reason = 'SINGLE_DISAGREE'

                    # Same Stage 3: Robust consensus
                    active_candidates = [row for row in deduped if row.status == 'PENDING']
                    if active_candidates:
                        median_center = float(np.median([r.row_center for r in active_candidates]))
                        for row in active_candidates:
                            if abs(row.row_center - median_center) > self.max_dark_row_center_deviation_px:
                                row.status = 'REJECTED'
                                row.reject_reason = 'CENTER_OUTLIER'

                    # Same Stage 4: Far-to-near continuity
                    active_candidates = [row for row in deduped if row.status == 'PENDING']
                    new_accepted = []
                    for row in active_candidates:
                        if not new_accepted:
                            row.status = 'ACCEPTED'
                            new_accepted.append(row)
                        else:
                            prev_center = new_accepted[-1].row_center
                            if abs(row.row_center - prev_center) <= self.max_dark_adjacent_center_delta_px:
                                row.status = 'ACCEPTED'
                                new_accepted.append(row)
                            else:
                                row.status = 'REJECTED'
                                row.reject_reason = 'GEOMETRY'
                    accepted_rows = new_accepted
                    all_rows_combined = all_rows + [r for r in adaptive_rows if r not in all_rows]
                    all_rows = all_rows_combined

        self.last_dark_rows = all_rows

        if len(accepted_rows) < self.min_dark_valid_rows:
            return None, None, None, None, accepted_rows, skipped_transverse, dark_roi, dark_clean

        # Stage 5: Target synthesis with Entry Stabilization & Near/Far Lookahead Protection (Sections 6, 7, 8)
        accepted_centers = [r.row_center for r in accepted_rows]
        robust_median_center = float(np.median(accepted_centers))

        mid = len(accepted_rows) // 2
        far_rows = accepted_rows[:mid] if mid > 0 else accepted_rows
        near_rows = accepted_rows[mid:] if mid > 0 else accepted_rows

        far_center = float(np.median([r.row_center for r in far_rows]))
        near_center = float(np.median([r.row_center for r in near_rows]))

        # Tunnel Entry Stabilization (Section 6)
        is_entry_stabilize = (self.dark_entry_stabilize_count <= self.dark_entry_stabilize_frames)
        far_mid_rows = [r for r in accepted_rows if r.row_y <= h_dark_roi * 0.70]
        if not far_mid_rows and len(accepted_rows) >= 2:
            far_mid_rows = accepted_rows[:max(1, int(round(len(accepted_rows) * 0.67)))]

        if is_entry_stabilize:
            if len(far_mid_rows) >= 2:
                raw_target_x = float(np.median([r.row_center for r in far_mid_rows]))
                using_str = 'FAR_MID'
            else:
                raw_target_x = robust_median_center
                using_str = 'MEDIAN'
            if abs(near_center - far_center) > self.max_dark_near_far_disagreement_px:
                self.log_dark_geometry_disagree(near_center, far_center, robust_median_center, using_str)
        else:
            # Steady-state: Lateral centering strictly uses near / near-mid rows
            near_mid_rows = [r for r in accepted_rows if r.row_y >= h_dark_roi * 0.60]
            lateral_center = float(np.median([r.row_center for r in near_mid_rows])) if near_mid_rows else near_center
            raw_target_x = lateral_center
            self.last_dark_lateral_center = lateral_center

        # Camera-driven curve feedforward computation
        dual_accepted = [r for r in accepted_rows if r.boundary_type == 'DUAL']
        dual_accepted.sort(key=lambda r: r.row_y)

        curve_signal = 0.0
        raw_w_curve = 0.0

        if len(dual_accepted) >= self.dark_curve_min_dual_rows:
            dual_span = float(dual_accepted[-1].row_y - dual_accepted[0].row_y)
            if dual_span >= self.dark_curve_min_span_px:
                k = len(dual_accepted)
                dual_far = dual_accepted[:max(1, k // 2)]
                dual_near = dual_accepted[k - max(1, k // 2):]
                d_far_c = float(np.median([r.row_center for r in dual_far]))
                d_near_c = float(np.median([r.row_center for r in dual_near]))
                cand_curve_signal = d_far_c - d_near_c

                # Curve Coherence Gate: verify smooth progression without gross geometry kink
                is_coherent = True
                min_c = min(d_far_c, d_near_c) - 35.0
                max_c = max(d_far_c, d_near_c) + 35.0
                for r in dual_accepted:
                    if not (min_c <= r.row_center <= max_c):
                        is_coherent = False
                        break

                if is_coherent:
                    curve_signal = cand_curve_signal
                    curve_norm = curve_signal / image_center_x
                    raw_w_curve = self.dark_curve_gain * curve_norm
                    raw_w_curve = float(max(-self.max_dark_curve_w, min(self.max_dark_curve_w, raw_w_curve)))

        # EMA filter on curve feedforward to avoid steering steps
        if abs(raw_w_curve) > 1e-4:
            w_curve = self.dark_curve_alpha * raw_w_curve + (1.0 - self.dark_curve_alpha) * self.prev_dark_w_curve
            self.prev_dark_w_curve = w_curve
        else:
            w_curve = 0.0
            self.prev_dark_w_curve = 0.5 * self.prev_dark_w_curve

        self.last_dark_curve_signal = curve_signal
        self.last_dark_w_curve = w_curve

        # Stage 6: Temporal EMA smoothing (Section 10)
        if self.prev_dark_target_x is None:
            filtered_target_x = raw_target_x
        else:
            filtered_target_x = (
                self.dark_target_alpha * raw_target_x
                + (1.0 - self.dark_target_alpha) * self.prev_dark_target_x
            )
        self.prev_dark_target_x = filtered_target_x
        # Trusted history update: only update if accepted rows contain trusted DUAL evidence
        if any(r.boundary_type == 'DUAL' for r in accepted_rows):
            self.last_valid_dark_target_x = filtered_target_x

        return (
            filtered_target_x, raw_target_x, near_center, far_center,
            accepted_rows, skipped_transverse, dark_roi, dark_clean
        )

    def find_lane_target(self, img):
        """Estimate lane center target X and confidence from camera image.

        Pipeline (Unchanged V1 Baseline):
          1. Crop bottom region of interest (ROI_START_RATIO .. 1.0)
          2. Convert to grayscale and binary threshold for white lane marks
          3. Morphological opening (3x3 rect kernel) to suppress noise
          4. Compute spatial moments for left half and right half
          5. Synthesize lane target:
             - Both lines seen: midpoint between left and right lines
             - Only left seen: left_x + half_lane_px
             - Only right seen: right_x - half_lane_px
             - Single cluster: overall centroid fallback
        """
        if img is None or not HAVE_CV:
            return LaneTargetResult((None, None, None, 0.0, None, None))

        h, w = img.shape[:2]
        roi_y = int(h * self.roi_start_ratio)
        roi = img[roi_y:h, 0:w]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, self.white_threshold, 255, cv2.THRESH_BINARY)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        clean = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

        center_x = w / 2.0
        mid_col = int(round(center_x))
        left_half = clean[:, :mid_col]
        right_half = clean[:, mid_col:]

        min_pixels = 50.0

        m_left = cv2.moments(left_half)
        m_right = cv2.moments(right_half)

        has_left = m_left['m00'] > min_pixels
        has_right = m_right['m00'] > min_pixels

        left_x = (m_left['m10'] / m_left['m00']) if has_left else None
        right_x = (mid_col + m_right['m10'] / m_right['m00']) if has_right else None

        # Scoped Junction False-Pair Protection:
        # Active ONLY during post-tunnel curve assist or post-curve guard episode
        is_post_tunnel_protect = (
            getattr(self, 'post_tunnel_curve_active', False) or
            getattr(self, 'post_tunnel_guard_active', False)
        ) and not getattr(self, 'post_tunnel_assist_consumed', False)

        if has_left and has_right:
            cand_target = (left_x + right_x) / 2.0
            measured_dist = right_x - left_x

            if is_post_tunnel_protect:
                trusted_w = getattr(self, 'trusted_post_tunnel_lane_width', 350.0)
                width_ratio = measured_dist / trusted_w if trusted_w > 0 else 1.0
                prev_ref = getattr(self, 'last_stable_normal_target_x', center_x)
                if prev_ref is None:
                    prev_ref = center_x

                is_narrow_false_pair = (measured_dist < 220.0 or width_ratio < 0.70)

                if is_narrow_false_pair:
                    now_sec = self._get_current_time_sec()
                    if now_sec - getattr(self, '_last_post_tunnel_width_reject_log_time', 0.0) > 0.5:
                        self._last_post_tunnel_width_reject_log_time = now_sec
                        self.get_logger().info(
                            f'POST_TUNNEL_WIDTH_REJECT measured={measured_dist:.1f} trusted={trusted_w:.1f} ratio={width_ratio:.2f}'
                        )
                    if getattr(self, 'post_tunnel_guard_active', False) and not getattr(self, 'post_tunnel_curve_active', False):
                        if not getattr(self, 'post_tunnel_junction_risk_seen', False):
                            self.post_tunnel_junction_risk_seen = True
                            if now_sec - getattr(self, '_last_post_tunnel_junction_risk_log_time', 0.0) > 0.5:
                                self._last_post_tunnel_junction_risk_log_time = now_sec
                                self.get_logger().info(
                                    f'POST_TUNNEL_JUNCTION_RISK reason=NARROW_WIDTH measured_width={measured_dist:.1f} trusted_width={trusted_w:.1f}'
                                )
                    trusted_half = trusted_w / 2.0
                    cand_l = left_x + trusted_half
                    cand_r = right_x - trusted_half
                    diff_l = abs(cand_l - prev_ref)
                    diff_r = abs(cand_r - prev_ref)
                    chosen = cand_l if diff_l < diff_r else cand_r
                    if abs(chosen - prev_ref) > 75.0:
                        lane_target_x = prev_ref
                    else:
                        lane_target_x = chosen
                    confidence = 0.6
                else:
                    # Apex NEAR Temporal Sanity Guard
                    in_curve_context = (
                        getattr(self, 'post_tunnel_curve_active', False) and
                        getattr(self, 'post_tunnel_curve_dir', 0) != 0
                    )
                    has_recent_trusted = (
                        getattr(self, 'last_trusted_near_target', None) is not None and
                        getattr(self, 'last_trusted_near_left', None) is not None and
                        getattr(self, 'last_trusted_near_right', None) is not None
                    )

                    is_suspicious = False
                    target_jump = 0.0
                    left_jump = 0.0
                    right_jump = 0.0
                    cand_w_baseline = 0.0
                    curve_dir = getattr(self, 'post_tunnel_curve_dir', 0)

                    # Capture snapshot of trusted reference before any update
                    ref_target = getattr(self, 'last_trusted_near_target', cand_target)
                    ref_left = getattr(self, 'last_trusted_near_left', left_x)
                    ref_right = getattr(self, 'last_trusted_near_right', right_x)
                    ref_width = getattr(self, 'last_trusted_near_width', measured_dist)

                    if in_curve_context and has_recent_trusted:
                        target_jump = cand_target - ref_target
                        left_jump = left_x - ref_left
                        right_jump = right_x - ref_right

                        # 1. Plausible pair width (broadly plausible, not already rejected)
                        is_plausible_width = (220.0 <= measured_dist <= 450.0) and (0.70 <= width_ratio <= 1.35)

                        # 2. Both boundaries shift substantially in the same direction as target jump
                        target_jump_thresh = 0.15 * trusted_w
                        boundary_jump_thresh = 0.10 * trusted_w

                        same_dir_jump = (
                            (abs(target_jump) > target_jump_thresh) and
                            (abs(left_jump) > boundary_jump_thresh) and
                            (abs(right_jump) > boundary_jump_thresh) and
                            (target_jump * left_jump > 0) and
                            (target_jump * right_jump > 0)
                        )

                        # 3. Discontinuity causes a strong counter-steer opposing confirmed curve
                        cand_err = (cand_target - center_x) / center_x
                        cand_deriv = cand_err - getattr(self, 'prev_error', 0.0)
                        cand_w_baseline = -(self.kp * cand_err + self.kd * cand_deriv)
                        strong_opposing_counter_steer = (cand_w_baseline * curve_dir > 0.10)

                        if is_plausible_width and same_dir_jump and strong_opposing_counter_steer:
                            is_suspicious = True

                    cand_l = left_x
                    cand_r = right_x
                    cand_tx = cand_target
                    cand_w = measured_dist

                    # State Machine: NORMAL_NEAR vs APEX_QUARANTINE
                    quarantine_active = getattr(self, 'apex_quarantine_active', False)

                    if not quarantine_active:
                        if is_suspicious:
                            self.apex_quarantine_active = True
                            self.apex_recovery_streak = 0
                            self.apex_guard_hold_count = 0
                            in_quarantine = True
                        else:
                            in_quarantine = False
                    else:
                        # Quarantine active: test for coherent recovery
                        cand_coherent = (
                            is_plausible_width and
                            (abs(target_jump) <= target_jump_thresh) and
                            not same_dir_jump and
                            not (cand_w_baseline * curve_dir > 0.08)
                        )
                        if cand_coherent:
                            self.apex_recovery_streak = getattr(self, 'apex_recovery_streak', 0) + 1
                            if self.apex_recovery_streak >= 2:
                                # Confirmed recovery streak: exit quarantine
                                self.apex_quarantine_active = False
                                self.apex_recovery_streak = 0
                                self.apex_guard_hold_count = 0
                                in_quarantine = False
                            else:
                                in_quarantine = True
                        else:
                            self.apex_recovery_streak = 0
                            in_quarantine = True

                    max_hold_frames = 3

                    if in_quarantine:
                        temporal_guard_triggered = True
                        hold_count = getattr(self, 'apex_guard_hold_count', 0)

                        if hold_count < max_hold_frames:
                            # Initial short trusted continuity hold
                            self.apex_guard_hold_count = hold_count + 1
                            lane_target_x = ref_target
                            left_x = ref_left
                            right_x = ref_right
                            confidence = 0.8
                        else:
                            # Direct hold budget expired: DO NOT accept suspicious candidate.
                            # Use safe controller fallback without updating or trusting candidate.
                            self.apex_guard_hold_count = hold_count + 1
                            trusted_half = trusted_w / 2.0
                            left_coherent = (abs(left_jump) <= boundary_jump_thresh)
                            right_coherent = (abs(right_jump) <= boundary_jump_thresh)

                            if left_coherent and not right_coherent:
                                lane_target_x = cand_l + trusted_half
                                left_x = cand_l
                                right_x = None
                            elif right_coherent and not left_coherent:
                                lane_target_x = cand_r - trusted_half
                                left_x = None
                                right_x = cand_r
                            else:
                                # Bounded continuity: decay gracefully toward image center
                                decay_steps = self.apex_guard_hold_count - max_hold_frames
                                decay_factor = max(0.0, 0.8 ** decay_steps)
                                lane_target_x = center_x + decay_factor * (ref_target - center_x)
                                left_x = None
                                right_x = None
                            confidence = 0.6
                    else:
                        temporal_guard_triggered = False
                        self.apex_guard_hold_count = 0
                        self.apex_quarantine_active = False
                        self.apex_recovery_streak = 0

                        lane_target_x = cand_target
                        confidence = 1.0
                        if 250.0 <= measured_dist <= 450.0:
                            self.half_lane_px = 0.9 * self.half_lane_px + 0.1 * (measured_dist / 2.0)
                            self.trusted_post_tunnel_lane_width = 0.95 * self.trusted_post_tunnel_lane_width + 0.05 * measured_dist

                        # Trusted geometry rule: ONLY coherent, accepted in-lane geometry refreshes trusted state.
                        cand_opposes_curve = (cand_w_baseline * curve_dir > 0.05) if in_curve_context else False
                        is_coherent_reference = (280.0 <= measured_dist <= 420.0) and not cand_opposes_curve

                        if is_coherent_reference or not in_curve_context or not has_recent_trusted:
                            self.last_trusted_near_left = left_x
                            self.last_trusted_near_right = right_x
                            self.last_trusted_near_target = cand_target
                            self.last_trusted_near_width = measured_dist
                            self._diag_trusted_updated = True
                            if not in_curve_context:
                                self._diag_trusted_update_reason = "ACCEPTED_OUTSIDE_CURVE"
                            elif not has_recent_trusted:
                                self._diag_trusted_update_reason = "ACCEPTED_INITIAL_TRUSTED"
                            else:
                                self._diag_trusted_update_reason = "ACCEPTED_COHERENT_PAIR"
                        else:
                            self._diag_trusted_updated = False
                            if measured_dist < 280.0 or measured_dist > 420.0:
                                self._diag_trusted_update_reason = f"REJECTED_WIDTH_OUT_OF_BOUNDS_{measured_dist:.1f}"
                            elif cand_opposes_curve:
                                self._diag_trusted_update_reason = f"REJECTED_OPPOSES_CURVE_cand_w={cand_w_baseline:+.3f}"
                            else:
                                self._diag_trusted_update_reason = "REJECTED_OTHER"

                    if in_curve_context:
                        self._last_apex_guard_diag = {
                            'in_curve': True,
                            'triggered': temporal_guard_triggered,
                            'quarantine_active': getattr(self, 'apex_quarantine_active', False),
                            'recovery_streak': getattr(self, 'apex_recovery_streak', 0),
                            'hold_count': self.apex_guard_hold_count,
                            'cand_target': cand_tx,
                            'cand_left': cand_l,
                            'cand_right': cand_r,
                            'cand_width': cand_w,
                            'trusted_target': ref_target,
                            'trusted_left': ref_left,
                            'trusted_right': ref_right,
                            'trusted_width': ref_width,
                            'target_jump': target_jump,
                            'left_jump': left_jump,
                            'right_jump': right_jump,
                            'cand_w': cand_w_baseline,
                            'curve_dir': 'LEFT' if curve_dir == -1 else ('RIGHT' if curve_dir == 1 else 'NONE'),
                        }
            else:
                lane_target_x = cand_target
                if 80.0 < measured_dist < 450.0 and not getattr(self, 'post_reacquire_protect_active', False):
                    self.half_lane_px = 0.9 * self.half_lane_px + 0.1 * (measured_dist / 2.0)
                confidence = 1.0
        elif has_left and not has_right:
            if is_post_tunnel_protect:
                trusted_w = getattr(self, 'trusted_post_tunnel_lane_width', 350.0)
                trusted_half = trusted_w / 2.0
                lane_target_x = left_x + trusted_half
                confidence = 0.6
                now_sec = self._get_current_time_sec()
                if getattr(self, 'post_tunnel_guard_active', False) and not getattr(self, 'post_tunnel_curve_active', False):
                    if not getattr(self, 'post_tunnel_junction_risk_seen', False):
                        self.post_tunnel_junction_risk_seen = True
                        if now_sec - getattr(self, '_last_post_tunnel_junction_risk_log_time', 0.0) > 0.5:
                            self._last_post_tunnel_junction_risk_log_time = now_sec
                            self.get_logger().info(
                                f'POST_TUNNEL_JUNCTION_RISK reason=SINGLE_LEFT measured_width=None trusted_width={trusted_w:.1f}'
                            )
                if now_sec - getattr(self, '_last_post_tunnel_single_boundary_log_time', 0.0) > 0.5:
                    self._last_post_tunnel_single_boundary_log_time = now_sec
                    self.get_logger().info(
                        f'POST_TUNNEL_SINGLE_BOUNDARY side=LEFT boundary={left_x:.1f} '
                        f'half_lane={self.half_lane_px:.1f} trusted_half={trusted_half:.1f} target={lane_target_x:.1f}'
                    )
            else:
                lane_target_x = left_x + self.half_lane_px
                confidence = 0.6
        elif has_right and not has_left:
            if is_post_tunnel_protect:
                trusted_w = getattr(self, 'trusted_post_tunnel_lane_width', 350.0)
                trusted_half = trusted_w / 2.0
                lane_target_x = right_x - trusted_half
                confidence = 0.6
                now_sec = self._get_current_time_sec()
                if getattr(self, 'post_tunnel_guard_active', False) and not getattr(self, 'post_tunnel_curve_active', False):
                    if not getattr(self, 'post_tunnel_junction_risk_seen', False):
                        self.post_tunnel_junction_risk_seen = True
                        if now_sec - getattr(self, '_last_post_tunnel_junction_risk_log_time', 0.0) > 0.5:
                            self._last_post_tunnel_junction_risk_log_time = now_sec
                            self.get_logger().info(
                                f'POST_TUNNEL_JUNCTION_RISK reason=SINGLE_RIGHT measured_width=None trusted_width={trusted_w:.1f}'
                            )
                if now_sec - getattr(self, '_last_post_tunnel_single_boundary_log_time', 0.0) > 0.5:
                    self._last_post_tunnel_single_boundary_log_time = now_sec
                    self.get_logger().info(
                        f'POST_TUNNEL_SINGLE_BOUNDARY side=RIGHT boundary={right_x:.1f} '
                        f'half_lane={self.half_lane_px:.1f} trusted_half={trusted_half:.1f} target={lane_target_x:.1f}'
                    )
            else:
                lane_target_x = right_x - self.half_lane_px
                confidence = 0.6
        else:
            m_all = cv2.moments(clean)
            if m_all['m00'] > min_pixels:
                lane_target_x = m_all['m10'] / m_all['m00']
                confidence = 0.3
            else:
                lane_target_x = None
                confidence = 0.0

        # --- DIAGNOSTIC TELEMETRY CAPTURE (ZERO CONTROL CHANGE) ---
        self._diag_near_lx = left_x
        self._diag_near_rx = right_x
        self._diag_near_cx = ((left_x + right_x) / 2.0) if (left_x is not None and right_x is not None) else None
        self._diag_near_w = float(right_x - left_x) if (left_x is not None and right_x is not None) else None
        self._diag_near_tx = lane_target_x
        self._diag_near_conf = confidence
        self._diag_boundaries = 'DUAL' if (left_x is not None and right_x is not None) else ('LEFT_ONLY' if left_x is not None else ('RIGHT_ONLY' if right_x is not None else 'NONE'))
        if has_left and has_right:
            if is_post_tunnel_protect and is_narrow_false_pair:
                self._diag_target_source = 'TRUSTED_FALLBACK_NARROW'
            elif (getattr(self, '_last_apex_guard_diag', None) or {}).get('quarantine_active', False):
                self._diag_target_source = 'APEX_GUARD_QUARANTINE'
                self._diag_trusted_updated = False
                self._diag_trusted_update_reason = 'REJECTED_APEX_GUARD_QUARANTINE'
            elif (getattr(self, '_last_apex_guard_diag', None) or {}).get('triggered', False):
                self._diag_target_source = 'APEX_GUARD_HOLD'
                self._diag_trusted_updated = False
                self._diag_trusted_update_reason = 'REJECTED_APEX_GUARD_HOLD'
            elif is_post_tunnel_protect and cand_opposes_curve:
                self._diag_target_source = 'DUAL_OPPOSES_CURVE'
            else:
                self._diag_target_source = 'DUAL_BOUNDARY'
        elif has_left and not has_right:
            self._diag_target_source = 'SINGLE_LEFT_FALLBACK' if is_post_tunnel_protect else 'SINGLE_LEFT'
            self._diag_trusted_updated = False
            self._diag_trusted_update_reason = 'REJECTED_SINGLE_LEFT'
        elif has_right and not has_left:
            self._diag_target_source = 'SINGLE_RIGHT_FALLBACK' if is_post_tunnel_protect else 'SINGLE_RIGHT'
            self._diag_trusted_updated = False
            self._diag_trusted_update_reason = 'REJECTED_SINGLE_RIGHT'
        else:
            self._diag_target_source = 'FALLBACK_MOMENTS' if (lane_target_x is not None) else 'NONE'
            self._diag_trusted_updated = False
            self._diag_trusted_update_reason = 'REJECTED_NO_BOUNDARIES'

        return LaneTargetResult((left_x, right_x, lane_target_x, confidence, roi, clean))

    def find_normal_far_target(self, img, near_center=None):
        """Estimate far lane center from higher lookahead region in NORMAL mode.

        Advisory only: used solely for curvature preview / feedforward.
        Never replaces near lane detector as primary guidance.

        Returns:
            far_left_x (float or None): Left boundary in far ROI
            far_right_x (float or None): Right boundary in far ROI
            far_center (float or None): Estimated lane center in far ROI
            far_conf (float): Confidence in [0.0, 1.0]
            far_width (float or None): Measured width between boundaries in far ROI
        """
        if img is None or not HAVE_CV:
            return None, None, None, 0.0, None

        h, w = img.shape[:2]
        ref_c = near_center if near_center is not None else (w / 2.0)
        y1 = int(h * self.normal_far_roi_top_ratio)
        y2 = int(h * self.normal_far_roi_bottom_ratio)
        if y2 <= y1 or y1 < 0 or y2 > h:
            return None, None, None, 0.0, None

        far_roi = img[y1:y2, 0:w]
        gray = cv2.cvtColor(far_roi, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, self.white_threshold, 255, cv2.THRESH_BINARY)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        clean = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

        # Column profile analysis for robust lane marking clustering
        col_sums = np.sum(clean > 0, axis=0)
        peaks = np.where(col_sums > 2)[0]

        if len(peaks) > 0:
            diffs = np.diff(peaks)
            splits = np.where(diffs > 25)[0]  # Minimum gap between distinct lane marks
            clusters = np.split(peaks, splits + 1)
            valid_clusters = [float(np.mean(c)) for c in clusters if len(c) >= 3]
        else:
            valid_clusters = []

        far_left_x = None
        far_right_x = None
        far_center = None
        far_conf = 0.0
        far_width = None

        if len(valid_clusters) >= 2:
            best_pair = None
            best_dist = 999.0
            for i in range(len(valid_clusters)):
                for j in range(i + 1, len(valid_clusters)):
                    cw = valid_clusters[j] - valid_clusters[i]
                    if 50.0 <= cw <= 260.0:
                        cand_c = (valid_clusters[i] + valid_clusters[j]) / 2.0
                        dist = abs(cand_c - ref_c)
                        if dist <= 120.0 and dist < best_dist:
                            best_dist = dist
                            best_pair = (valid_clusters[i], valid_clusters[j], cw, cand_c)
            if best_pair is not None:
                far_left_x, far_right_x, far_width, far_center = best_pair
                far_conf = 1.0
        else:
            # Fallback to dual moments (strictly requiring both left and right halves)
            center_x = w / 2.0
            mid_col = int(round(center_x))
            m_left = cv2.moments(clean[:, :mid_col])
            m_right = cv2.moments(clean[:, mid_col:])
            min_pixels = 15.0
            has_l = m_left['m00'] > min_pixels
            has_r = m_right['m00'] > min_pixels
            if has_l and has_r:
                fl = m_left['m10'] / m_left['m00']
                fr = mid_col + m_right['m10'] / m_right['m00']
                cw = fr - fl
                if 50.0 <= cw <= 260.0:
                    cand_c = (fl + fr) / 2.0
                    if abs(cand_c - ref_c) <= 120.0:
                        far_left_x = fl
                        far_right_x = fr
                        far_width = cw
                        far_center = cand_c
                        far_conf = 0.8

        # Diagnostic reason extraction
        if far_center is not None and far_conf >= 0.7:
            self._diag_far_reason = f"VALID_FAR(cw={far_width:.1f})"
        elif len(valid_clusters) >= 2:
            self._diag_far_reason = f"NO_CLUSTER_PAIR_IN_BOUNDS(clusters={[round(c, 1) for c in valid_clusters]})"
        elif len(peaks) == 0:
            self._diag_far_reason = "ROI_NO_WHITE_PIXELS"
        elif len(valid_clusters) == 0:
            self._diag_far_reason = f"NO_VALID_CLUSTERS(peaks={len(peaks)})"
        elif len(valid_clusters) == 1:
            self._diag_far_reason = f"SINGLE_CLUSTER_ONLY(x={valid_clusters[0]:.1f})"
        else:
            self._diag_far_reason = "FAR_INVALID_OTHER"

        return far_left_x, far_right_x, far_center, far_conf, far_width

    def select_right_lane_contour(self, clean_mask, last_trusted_r, trusted_width, center_x):
        """Identify individual right-boundary contour candidates during RIGHT_CONT.

        Filters out tiny/noisy contours and branch/junction markings by selecting
        the candidate that best matches temporal continuity (closest to last_trusted_r)
        with an outer-right preference for similarly close candidates.
        """
        if clean_mask is None or not HAVE_CV:
            return None

        h, w = clean_mask.shape[:2]
        mid_col = int(round(center_x))
        x_offset = max(0, mid_col - 20)
        right_sub = clean_mask[:, x_offset:w]

        contours, _ = cv2.findContours(right_sub, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        candidates = []
        min_pixels = 40.0

        for c in contours:
            area = cv2.contourArea(c)
            if area < min_pixels:
                continue
            m = cv2.moments(c)
            if m['m00'] <= 0:
                continue
            cand_cx = x_offset + (m['m10'] / m['m00'])
            candidates.append((cand_cx, area))

        if not candidates:
            return None

        max_r_jump = 0.15 * trusted_width

        if last_trusted_r is None:
            candidates.sort(key=lambda item: item[0], reverse=True)
            return candidates[0][0]

        valid = []
        for cand_cx, area in candidates:
            jump = abs(cand_cx - last_trusted_r)
            if jump <= max_r_jump:
                valid.append((cand_cx, jump, area))

        if not valid:
            return None

        min_jump = min(item[1] for item in valid)
        close_band = 15.0
        similarly_close = [item for item in valid if item[1] <= min_jump + close_band]
        similarly_close.sort(key=lambda item: item[0], reverse=True)
        return similarly_close[0][0]

    def check_same_lane_coherence(self, lx, rx, near_center, near_conf, far_lx, far_rx, far_center, far_conf, far_width):
        """Verify whether FAR ROI geometry connects to the current NEAR lane.

        Advisory / verification gate: requires same-lane geometric continuity,
        plausible perspective width progression, and sufficient curve strength.
        Returns:
            is_coherent (bool): True if FAR geometry belongs to the same lane
            cand_dir (int): -1 for LEFT physical curve, +1 for RIGHT physical curve, 0 otherwise
            reason (str): Diagnostic explanation
        """
        if near_center is None or near_conf < self.min_confidence:
            return False, 0, 'NEAR_PERCEPTION_INVALID'

        if far_center is None or far_conf < 0.70 or far_lx is None or far_rx is None or far_width is None:
            return False, 0, 'FAR_PERCEPTION_INVALID'

        curve_signal = float(far_center - near_center)
        if abs(curve_signal) < self.post_tunnel_boost_min_signal_px:
            return False, 0, 'SIGNAL_BELOW_BOOST_THRESH'

        cand_dir = -1 if curve_signal < 0 else 1

        # Far width bounds
        if not (50.0 <= far_width <= 260.0):
            return False, cand_dir, 'FAR_WIDTH_OUT_OF_BOUNDS'

        # Perspective width progression: distant lane must appear narrower than near lane
        trusted_w = getattr(self, 'trusted_post_tunnel_lane_width', 350.0)
        if lx is not None and rx is not None:
            near_width = float(rx - lx)
            if near_width > 0 and far_width >= near_width * 0.90:
                return False, cand_dir, 'FAR_WIDTH_EXCEEDS_PERSPECTIVE'
        elif far_width >= trusted_w * 0.90:
            return False, cand_dir, 'FAR_WIDTH_EXCEEDS_TRUSTED'

        # Boundary integrity: far_lx < far_center < far_rx
        if not (far_lx < far_center < far_rx):
            return False, cand_dir, 'FAR_BOUNDARIES_INVERTED'

        # Boundary continuation: same-lane geometric envelope
        if cand_dir == -1:
            # Physical LEFT curve: road curves left ahead
            # 1. Left boundary cannot jump right across current lane
            if far_lx >= near_center + 20.0:
                return False, cand_dir, 'FAR_LEFT_JUMPS_RIGHT'
            # 2. Right boundary cannot flare absurdly right or cross left of lane center
            if rx is not None and far_rx > rx + 40.0:
                return False, cand_dir, 'FAR_RIGHT_EXPANDS_RIGHT'
            if far_rx <= near_center - 60.0:
                return False, cand_dir, 'FAR_RIGHT_CROSSES_LEFT'
            # 3. Adjacent lane rejection: far left boundary near near right boundary
            if rx is not None and abs(far_lx - rx) < 50.0:
                return False, cand_dir, 'ADJACENT_RIGHT_LANE'
            if lx is not None and abs(far_rx - lx) < 50.0:
                return False, cand_dir, 'ADJACENT_LEFT_LANE'
        else:
            # Physical RIGHT curve: road curves right ahead
            if far_rx <= near_center - 20.0:
                return False, cand_dir, 'FAR_RIGHT_JUMPS_LEFT'
            if lx is not None and far_lx < lx - 40.0:
                return False, cand_dir, 'FAR_LEFT_EXPANDS_LEFT'
            if far_lx >= near_center + 60.0:
                return False, cand_dir, 'FAR_LEFT_CROSSES_RIGHT'
            if rx is not None and abs(far_lx - rx) < 50.0:
                return False, cand_dir, 'ADJACENT_RIGHT_LANE'
            if lx is not None and abs(far_rx - lx) < 50.0:
                return False, cand_dir, 'ADJACENT_LEFT_LANE'

        return True, cand_dir, 'SAME_LANE_COHERENT'

    def find_surface_target(self, roi):
        """Secondary bright-surface perception fallback for ramp / elevated road.

        Pipeline:
          1. Check ROI validity.
          2. Binary inRange threshold: [surface_min_gray .. surface_max_gray].
          3. Morphological opening and closing (5x5 rect kernel) to suppress noise.
          4. Connected component analysis to find dominant drivable region:
             - area_ratio >= min_surface_area_ratio
             - bottom_overlap_ratio >= min_surface_bottom_overlap
             - touches bottom-center of ROI (0.30*w .. 0.70*w)
             - score = 0.6 * area_ratio + 0.4 * bottom_overlap_ratio
          5. Sample horizontal rows (step of 4 rows) across selected component:
             - row width > 20 px: row_center = (left_px + right_px) / 2.0
          6. If valid_rows >= min_valid_surface_rows:
             - surface_target_x = weighted average favoring lower rows.
             - returns surface_target_x, coverage, valid_rows, surface_mask, row_endpoints
          7. Else:
             - returns None, 0.0, 0, None, None
        """
        if roi is None or not HAVE_CV:
            return None, 0.0, 0, None, None

        h_roi, w_roi = roi.shape[:2]
        total_pixels = float(h_roi * w_roi)
        if total_pixels <= 0.0:
            return None, 0.0, 0, None, None

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        mask = cv2.inRange(gray, self.surface_min_gray, self.surface_max_gray)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, kernel)

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask_clean)
        if num_labels <= 1:
            return None, 0.0, 0, mask_clean, None

        best_label = None
        best_score = -1.0

        for lbl in range(1, num_labels):
            area = stats[lbl, cv2.CC_STAT_AREA]
            area_ratio = area / total_pixels
            if area_ratio < self.min_surface_area_ratio:
                continue

            # Overlap on bottom row of ROI
            bottom_row_matches = np.sum(labels[h_roi - 1, :] == lbl)
            bottom_overlap_ratio = bottom_row_matches / float(w_roi)
            if bottom_overlap_ratio < self.min_surface_bottom_overlap:
                continue

            # Must touch bottom-center corridor [0.30*w .. 0.70*w]
            center_slice = labels[h_roi - 1, int(w_roi * 0.30):int(w_roi * 0.70)]
            if not np.any(center_slice == lbl):
                continue

            score = area_ratio * 0.6 + bottom_overlap_ratio * 0.4
            if score > best_score:
                best_score = score
                best_label = lbl

        if best_label is None:
            return None, 0.0, 0, mask_clean, None

        surface_mask = (labels == best_label).astype(np.uint8) * 255
        coverage = stats[best_label, cv2.CC_STAT_AREA] / total_pixels

        row_centers = []
        row_endpoints = []
        step = 4
        for r in range(0, h_roi, step):
            row_pixels = np.where(surface_mask[r, :] > 0)[0]
            if len(row_pixels) > 20:
                left_px = float(row_pixels[0])
                right_px = float(row_pixels[-1])
                row_c = (left_px + right_px) / 2.0
                row_centers.append((r, row_c))
                row_endpoints.append((r, left_px, right_px))

        valid_rows = len(row_centers)
        if valid_rows < self.min_valid_surface_rows:
            return None, coverage, valid_rows, surface_mask, row_endpoints

        # Weighted average favoring lower rows (closer to the robot)
        weights = [r + h_roi * 0.5 for r, _ in row_centers]
        centers = [c for _, c in row_centers]
        surface_target_x = float(np.average(centers, weights=weights))

        return surface_target_x, coverage, valid_rows, surface_mask, row_endpoints

    def detect_zebra_crossing(self, img):
        """Visual zebra crossing detector using a forward/bottom ROI restricted
        to the central roadway portion. Calculates bright-pixel fraction per row,
        groups consecutive bright rows into horizontal bands, and requires multiple
        separated horizontal white bands (with dark asphalt gaps).
        Rejects uniform bright surfaces (e.g. ramps) that do not have multiple
        separated bands.

        Returns:
            zebra_detected (bool): True if multiple separated bands detected in candidate frame
            band_count (int): Number of detected horizontal white bands
            score (float): Normalized detection score [0.0 .. 1.0]
            debug_info (tuple or None): (r_top, r_bot, c_left, c_right, bands)
        """
        if img is None:
            return False, 0, 0.0, None

        h, w = img.shape[:2]
        r_top = int(h * self.zebra_roi_top_ratio)
        r_bot = int(h * self.zebra_roi_bottom_ratio)
        c_left = int(w * self.zebra_roi_left_ratio)
        c_right = int(w * self.zebra_roi_right_ratio)

        if r_bot <= r_top or c_right <= c_left:
            return False, 0, 0.0, None

        zebra_roi_bgr = img[r_top:r_bot, c_left:c_right]
        if zebra_roi_bgr.size == 0:
            return False, 0, 0.0, None

        gray = cv2.cvtColor(zebra_roi_bgr, cv2.COLOR_BGR2GRAY)
        roi_h, roi_w = gray.shape

        # Threshold bright pixels (white stripes on asphalt)
        bright_mask = (gray >= self.zebra_white_threshold).astype(np.uint8)

        # Fraction of bright pixels per row
        row_bright_counts = np.count_nonzero(bright_mask, axis=1)
        row_bright_fractions = row_bright_counts / float(roi_w)
        is_bright_row = row_bright_fractions >= self.zebra_min_row_bright_ratio

        # Group consecutive bright rows into horizontal bands
        # A valid zebra stripe band has height >= 2 px and <= 35% of roi_h.
        # Separated bands require dark asphalt gaps of >= 2 rows.
        bands = []
        dark_gaps = []
        current_band_start = None
        current_gap_start = None
        max_band_h = max(3, int(roi_h * 0.35))

        for r in range(roi_h):
            if is_bright_row[r]:
                if current_gap_start is not None:
                    gap_len = r - current_gap_start
                    if gap_len >= 2:
                        dark_gaps.append((current_gap_start, r - 1, gap_len))
                    current_gap_start = None
                if current_band_start is None:
                    current_band_start = r
            else:
                if current_band_start is not None:
                    band_len = r - current_band_start
                    if 2 <= band_len <= max_band_h:
                        bands.append((current_band_start, r - 1, band_len))
                    current_band_start = None
                if current_gap_start is None:
                    current_gap_start = r

        if current_band_start is not None:
            band_len = roi_h - current_band_start
            if 2 <= band_len <= max_band_h:
                bands.append((current_band_start, roi_h - 1, band_len))

        band_count = len(bands)
        band_heights = [b[2] for b in bands]
        band_centers_y = [(b[0] + b[1]) / 2.0 for b in bands]

        # Calculate inter-band dark gaps specifically between consecutive accepted bands
        inter_band_gaps = []
        for i in range(band_count - 1):
            inter_gap = bands[i + 1][0] - bands[i][1] - 1
            inter_band_gaps.append(inter_gap)
        valid_inter_gaps = [g for g in inter_band_gaps if g >= 2]
        gap_count = len(valid_inter_gaps)

        # Spacings between band centers
        spacings = [band_centers_y[i + 1] - band_centers_y[i] for i in range(band_count - 1)]
        spacing_mean = float(np.mean(spacings)) if spacings else 0.0
        spacing_std = float(np.std(spacings)) if len(spacings) > 1 else 0.0
        spacing_cv = (spacing_std / spacing_mean) if spacing_mean > 0.0 else 0.0

        # Band heights consistency
        height_mean = float(np.mean(band_heights)) if band_heights else 0.0
        height_std = float(np.std(band_heights)) if len(band_heights) > 1 else 0.0
        height_cv = (height_std / height_mean) if height_mean > 0.0 else 0.0

        # Horizontal coverage across ROI width for each band
        band_coverages = []
        for b in bands:
            b_slice = bright_mask[b[0]:b[1] + 1, :]
            col_proj = np.any(b_slice, axis=0)
            cov = float(np.count_nonzero(col_proj)) / float(roi_w)
            band_coverages.append(cov)
        avg_coverage = float(np.mean(band_coverages)) if band_coverages else 0.0
        min_coverage = float(min(band_coverages)) if band_coverages else 0.0

        # Pattern Quality Validation
        zebra_detected = False
        reject_reason = 'NONE'

        if band_count < self.zebra_min_bands:
            reject_reason = 'TOO_FEW_BANDS'
        elif gap_count < (band_count - 1):
            reject_reason = 'TOO_FEW_GAPS'
        elif spacings and (spacing_cv > 0.45 or (max(spacings) / max(min(spacings), 1e-3)) > 3.0):
            reject_reason = 'BAD_SPACING'
        elif band_heights and (height_cv > 0.50 or (max(band_heights) / max(min(band_heights), 1)) > 3.5):
            reject_reason = 'BAD_HEIGHT_PATTERN'
        elif min_coverage < 0.20 or avg_coverage < 0.25:
            reject_reason = 'LOW_COVERAGE'
        else:
            zebra_detected = True
            reject_reason = 'ACCEPTED'

        # If not detected via transverse bands, check longitudinal periodic stripes across rows
        # (for real-world/simulator zebra crossings with stripes along the lane)
        if not zebra_detected:
            valid_rows = 0
            for r in range(roi_h):
                if not is_bright_row[r]:
                    continue
                row = gray[r, :]
                b_row = (row >= self.zebra_white_threshold).astype(np.uint8)
                stripes = []
                cur_s = None
                for c in range(roi_w):
                    if b_row[c]:
                        if cur_s is None:
                            cur_s = c
                    else:
                        if cur_s is not None:
                            w_s = c - cur_s
                            if w_s >= 8:
                                stripes.append((cur_s, c - 1, w_s))
                            cur_s = None
                if cur_s is not None:
                    w_s = roi_w - cur_s
                    if w_s >= 8:
                        stripes.append((cur_s, roi_w - 1, w_s))
                if len(stripes) >= 4:
                    widths = [s[2] for s in stripes]
                    centers = [(s[0] + s[1]) / 2.0 for s in stripes]
                    s_spacings = [centers[i + 1] - centers[i] for i in range(len(centers) - 1)]
                    s_mean = np.mean(s_spacings)
                    s_std = np.std(s_spacings) if len(s_spacings) > 1 else 0
                    s_cv = s_std / s_mean if s_mean > 0 else 1.0
                    if np.mean(widths) >= 15 and s_cv <= 0.35:
                        valid_rows += 1

            if valid_rows >= 8:
                zebra_detected = True
                band_count = 4
                reject_reason = f'ACCEPTED_LONGITUDINAL (rows={valid_rows})'

        score = min(1.0, float(band_count) / 4.0) if zebra_detected else 0.0

        if band_count >= 2 or zebra_detected:
            self.log_zebra_pattern(
                bands=band_count,
                gaps=gap_count,
                spacing_mean=spacing_mean,
                spacing_cv=spacing_cv,
                height_mean=height_mean,
                height_cv=height_cv,
                coverage=avg_coverage,
                score=score,
                accepted=zebra_detected,
                reject_reason=reject_reason
            )

        if zebra_detected:
            self.log_crossing_zebra_detected(band_count, score)

        debug_info = (r_top, r_bot, c_left, c_right, bands)
        return zebra_detected, band_count, score, debug_info

    def neutralize_crossing_context(self, reason='DARK_ENTRY'):
        """Safely neutralize and reset all pedestrian crossing state.
        Ensures no crossing context leaks into the dark tunnel while keeping generic obstacle safety untouched.
        """
        old_state = getattr(self, 'crossing_state', 'IDLE')
        self.crossing_state = 'IDLE'
        self._last_crossing_state = 'IDLE'
        self.zebra_detect_count = 0
        self.zebra_lost_count = 0
        self.crossing_occupied = False
        self.crossing_detect_count = 0
        self.crossing_clear_count = 0
        self.crossing_clear_since = None
        self.crossing_persistent_ray_count = 0
        self.last_zebra_detected = False
        self.last_zebra_bands = 0
        self.last_zebra_score = 0.0
        self.crossing_pass_start_x = None
        self.crossing_pass_start_y = None
        self.crossing_stop_reason = 'NONE'
        self._cached_zebra_result = (False, 0, 0.0, None)
        self._cached_crossing_occ = None
        self._last_crossing_image = None
        self._crossing_processed_image_seq = -1
        self._crossing_processed_scan_seq = -1
        self._last_crossing_scan = None
        self.reset_crossing_tracking()
        if old_state != 'IDLE':
            self.get_logger().info(
                f'CROSSING_NEUTRALIZED reason={reason} old_state={old_state} -> IDLE'
            )

    def reset_crossing_tracking(self):
        """Reset candidate tracking and edge motion tracker."""
        self.crossing_tracked_candidate = None
        self.crossing_entry_confirm_count = 0
        self._reset_edge_track()

    def track_crossing_entry_candidate(self, entry_candidate):
        """Deprecated/simplified: returns False, 0.0 (no entry tracking)."""
        return False, 0.0

    def _reset_edge_track(self):
        """Reset EDGE motion tracker state."""
        self._edge_track = {
            'prev_scan_seq': -1,
            'prev_x': None,
            'prev_y': None,
            'curr_x': None,
            'curr_y': None,
            'start_abs_y': None,    # abs(y) when current inward streak began
            'motion_steps': 0,      # consecutive scans in current inward streak
        }

    def _classify_cluster_zone(self, cluster_x, cluster_y):
        """Classify a valid LiDAR cluster by lateral zone.

        Returns:
            zone (str): 'CORE' if |y| <= crossing_core_half_width_m, else 'EDGE'
        """
        if abs(cluster_y) <= self.crossing_core_half_width_m:
            return 'CORE'
        return 'EDGE'

    def _update_edge_motion_tracker(self, cluster_x, cluster_y, scan_seq, is_new_scan):
        """Update EDGE motion tracker with new cluster observation.

        Uses a continuous inward streak / net inward displacement approach to
        prevent static LiDAR jitter from accumulating false MOVING_INTO_ROAD
        classifications.  The old positive-only lifetime accumulator
        (inward_accum) has been replaced:

          - start_abs_y: abs(y) at the beginning of the current inward streak.
          - motion_steps: consecutive new scans belonging to the current streak.
          - Net inward displacement = start_abs_y - current abs(y).

        Outward movement beyond EDGE_MOTION_EPSILON resets the entire streak so
        previous inward history cannot be reused.  Jitter within epsilon is
        silently absorbed (no step credit, no streak reset).

        Only updates on genuinely new LiDAR scans (is_new_scan=True).
        Performs centroid-proximity association to prevent false motion from
        switching between unrelated objects.

        Returns:
            classification (str): 'MOVING_INTO_ROAD', 'STATIC_EDGE', or 'MOVING_OUTWARD'
        """
        # Small epsilon to absorb LiDAR measurement noise without creating false
        # outward resets or false inward credits.  Reuses crossing_entry_min_inward_delta_m
        # which is already calibrated against real scan data (default 0.015 m).
        EDGE_MOTION_EPSILON = self.crossing_entry_min_inward_delta_m

        et = self._edge_track

        if not is_new_scan:
            # Duplicate scan: return current classification without updating state
            motion_steps = et['motion_steps']
            start_abs_y = et['start_abs_y']
            if (motion_steps >= self.crossing_edge_motion_min_steps
                    and start_abs_y is not None):
                net_inward = start_abs_y - abs(cluster_y)
                if net_inward >= self.crossing_edge_motion_min_total_m:
                    return 'MOVING_INTO_ROAD'
            return 'STATIC_EDGE'

        # Check cluster association: must be within gate of previous centroid
        if et['prev_x'] is not None and et['prev_y'] is not None:
            dx = abs(cluster_x - et['prev_x'])
            dy = abs(cluster_y - et['prev_y'])
            if dx > self.crossing_track_max_dx_m or dy > self.crossing_track_max_dy_m:
                # Association failed - reset tracker, start fresh with this cluster
                self._reset_edge_track()
                et = self._edge_track

        prev_y = et['prev_y']
        curr_abs_y = abs(cluster_y)
        inward_delta = 0.0  # kept for MOVING_OUTWARD return path

        if prev_y is not None:
            prev_abs_y = abs(prev_y)
            inward_delta = prev_abs_y - curr_abs_y

            if inward_delta > EDGE_MOTION_EPSILON:
                # Meaningful inward step -- extend or start streak
                if et['start_abs_y'] is None:
                    # Streak begins: anchor at the previous position
                    et['start_abs_y'] = prev_abs_y
                et['motion_steps'] += 1

            elif curr_abs_y > prev_abs_y + EDGE_MOTION_EPSILON:
                # Meaningful outward movement -- discard entire streak
                et['start_abs_y'] = None
                et['motion_steps'] = 0

            # else: within epsilon -- neither credit nor reset (jitter absorption)

        # Update tracker
        et['prev_scan_seq'] = scan_seq
        et['prev_x'] = cluster_x
        et['prev_y'] = cluster_y
        et['curr_x'] = cluster_x
        et['curr_y'] = cluster_y

        motion_steps = et['motion_steps']
        start_abs_y = et['start_abs_y']

        if (motion_steps >= self.crossing_edge_motion_min_steps
                and start_abs_y is not None):
            net_inward = start_abs_y - curr_abs_y
            if net_inward >= self.crossing_edge_motion_min_total_m:
                return 'MOVING_INTO_ROAD'

        # Determine if moving outward: prev existed and inward_delta was negative
        if prev_y is not None and inward_delta < 0.0:
            return 'MOVING_OUTWARD'

        return 'STATIC_EDGE'

    def _classify_crossing_cluster(self, cluster_x, cluster_y, scan_seq, is_new_scan):
        """Full CORE/EDGE occupancy classification for a valid crossing cluster.

        Returns:
            (zone, classification, occupied_candidate):
            zone: 'CORE' | 'EDGE'
            classification: 'CORE_OCCUPIED' | 'MOVING_INTO_ROAD' | 'STATIC_EDGE' | 'MOVING_OUTWARD'
            occupied_candidate (bool): True if this cluster constitutes an occupancy candidate
        """
        zone = self._classify_cluster_zone(cluster_x, cluster_y)

        if zone == 'CORE':
            # CORE: conservative - any valid cluster is an occupancy candidate
            # Reset edge tracker since object is now in CORE, not EDGE
            self._reset_edge_track()
            return 'CORE', 'CORE_OCCUPIED', True

        # EDGE zone: need motion evidence
        edge_class = self._update_edge_motion_tracker(cluster_x, cluster_y, scan_seq, is_new_scan)
        occupied = (edge_class == 'MOVING_INTO_ROAD')
        return 'EDGE', edge_class, occupied

    def log_crossing_object_diag(self, scan_seq, state, zone, classification,
                                  x, y, prev_y, inward_delta, inward_accum,
                                  motion_steps, occupied_candidate):
        """Rate-limited diagnostic for crossing object zone classification."""
        now = time.time()
        if now - self._last_crossing_object_log_time < 0.5:
            return
        self._last_crossing_object_log_time = now
        prev_y_str = f'{prev_y:.3f}' if prev_y is not None else 'None'
        self.get_logger().info(
            f'CROSSING_OBJECT scan_seq={scan_seq} state={state} '
            f'zone={zone} class={classification} '
            f'x={x:.3f} y={y:.3f} prev_y={prev_y_str} '
            f'inward_delta={inward_delta:.4f} inward_accum={inward_accum:.4f} '
            f'motion_steps={motion_steps} occupied_candidate={occupied_candidate}'
        )

    def detect_crossing_occupancy(self, scan=None, is_new_scan=None):
        """Simplified LiDAR crossing perception strictly enforcing UEH CRC 2026 requirement:
        - Check road corridor: 0.15 m <= x <= 1.20 m and |y| <= crossing_half_width_m.
        - Valid LiDAR return inside this road corridor counts as a hit.
        - Physical self-footprint returns are filtered out.
        - Exact single-consumption of new LiDAR scans via _crossing_processed_scan_seq.
        - Reuses cached perception result if called repeatedly on the same scan.
        - Stale or missing scan yields UNKNOWN (UNKNOWN != CLEAR).
        """
        target_scan = scan if scan is not None else self.scan

        # Determine if this observation is from a genuinely new scan
        if is_new_scan is None:
            if target_scan is not None and self._last_crossing_scan is not None and target_scan is not self._last_crossing_scan:
                is_new_scan = True
                if self._scan_seq == self._crossing_processed_scan_seq:
                    self._scan_seq += 1
            else:
                is_new_scan = (self._scan_seq != self._crossing_processed_scan_seq or self._cached_crossing_occ is None)

        now_sec = self._get_current_time_sec()
        scan_age = self._compute_scan_age(target_scan, now_sec)

        # If duplicate scan: reuse cached perception result without recomputing or altering state
        if not is_new_scan and self._cached_crossing_occ is not None:
            if scan_age > self.crossing_scan_stale_timeout_s:
                self.log_crossing_unknown('STALE_SCAN', age=scan_age)
                result = {
                    'status': CROSSING_UNKNOWN,
                    'occupied': False,
                    'hit': False,
                    'cluster': None,
                    'corridor_rays_count': 0,
                    'min_range': float('inf'),
                    'x': float('inf'),
                    'y': float('inf'),
                    'entry_candidate': None,
                    'entry_threat': False,
                    'inward_delta': 0.0,
                    'is_clear': False,
                    'is_unknown': True,
                    'unknown_reason': 'STALE_SCAN',
                    'is_new_scan': False
                }
                return result

            self.log_crossing_scan_skip(self._scan_seq, reason='ALREADY_PROCESSED')
            cached = dict(self._cached_crossing_occ)
            cached['is_new_scan'] = False
            return cached

        # Check for missing or empty scan data
        if target_scan is None or not getattr(target_scan, 'ranges', None):
            self.log_crossing_unknown('NO_SCAN')
            result = {
                'status': CROSSING_UNKNOWN,
                'occupied': False,
                'hit': False,
                'cluster': None,
                'corridor_rays_count': 0,
                'min_range': float('inf'),
                'x': float('inf'),
                'y': float('inf'),
                'cluster_x': float('inf'),
                'cluster_y': float('inf'),
                'entry_candidate': None,
                'entry_threat': False,
                'inward_delta': 0.0,
                'is_clear': False,
                'is_unknown': True,
                'unknown_reason': 'NO_SCAN',
                'is_new_scan': is_new_scan
            }
            self._crossing_processed_scan_seq = self._scan_seq
            self._last_crossing_scan = target_scan
            self._cached_crossing_occ = result
            return result

        scan_stamp = self._extract_stamp_sec(getattr(target_scan, 'header', None) and target_scan.header.stamp)

        # Check for backward clock jump / simulation reset
        if self._last_valid_scan_stamp > 0.0 and scan_stamp < (self._last_valid_scan_stamp - 1.0):
            self.reset_crossing_tracking()
            self._last_valid_scan_stamp = scan_stamp
            self.log_crossing_unknown('TIME_RESET')
            result = {
                'status': CROSSING_UNKNOWN,
                'occupied': False,
                'hit': False,
                'cluster': None,
                'corridor_rays_count': 0,
                'min_range': float('inf'),
                'x': float('inf'),
                'y': float('inf'),
                'cluster_x': float('inf'),
                'cluster_y': float('inf'),
                'entry_candidate': None,
                'entry_threat': False,
                'inward_delta': 0.0,
                'is_clear': False,
                'is_unknown': True,
                'unknown_reason': 'TIME_RESET',
                'is_new_scan': is_new_scan
            }
            self._crossing_processed_scan_seq = self._scan_seq
            self._last_crossing_scan = target_scan
            self._cached_crossing_occ = result
            return result

        self._last_valid_scan_stamp = scan_stamp

        # Check for genuinely stale scan data on new scan
        if scan_age > self.crossing_scan_stale_timeout_s:
            self.log_crossing_unknown('STALE_SCAN', age=scan_age)
            result = {
                'status': CROSSING_UNKNOWN,
                'occupied': False,
                'hit': False,
                'cluster': None,
                'corridor_rays_count': 0,
                'min_range': float('inf'),
                'x': float('inf'),
                'y': float('inf'),
                'cluster_x': float('inf'),
                'cluster_y': float('inf'),
                'entry_candidate': None,
                'entry_threat': False,
                'inward_delta': 0.0,
                'is_clear': False,
                'is_unknown': True,
                'unknown_reason': 'STALE_SCAN',
                'is_new_scan': is_new_scan
            }
            self._crossing_processed_scan_seq = self._scan_seq
            self._last_crossing_scan = target_scan
            self._cached_crossing_occ = result
            return result

        n = len(target_scan.ranges)
        angle_min = float(target_scan.angle_min)
        angle_inc = float(target_scan.angle_increment)
        r_min = float(target_scan.range_min)
        r_max = float(target_scan.range_max)

        in_corridor_rays = []
        outside_rays = []

        for i in range(n):
            angle_rad = angle_min + i * angle_inc
            norm_rad = math.atan2(math.sin(angle_rad), math.cos(angle_rad))
            deg = math.degrees(norm_rad)

            # Forward hemisphere check
            if abs(deg) > 90.0:
                continue

            r = target_scan.ranges[i]
            if not (math.isfinite(r) and (r_min - 1e-4) <= r <= (r_max + 1e-4)):
                continue

            x = float(r * math.cos(norm_rad))
            y = float(r * math.sin(norm_rad))

            # 1. Physical self-footprint mask check
            if (self.footprint_x_min <= x <= self.footprint_x_max and
                    self.footprint_y_min <= y <= self.footprint_y_max):
                continue

            # 2. Road corridor check: 0.15 <= x <= 1.20 and |y| <= crossing_half_width_m
            if self.crossing_detect_min_x_m <= x <= self.crossing_detect_max_x_m:
                ray_data = {
                    'index': i,
                    'angle': deg,
                    'val': float(r),
                    'x': x,
                    'y': y
                }
                if abs(y) <= self.crossing_half_width_m:
                    in_corridor_rays.append(ray_data)
                else:
                    outside_rays.append(ray_data)

        # Group in-corridor rays into contiguous clusters along scan-index order
        clusters = []
        if in_corridor_rays:
            current_cluster = [in_corridor_rays[0]]
            for r_idx in range(1, len(in_corridor_rays)):
                prev_p = in_corridor_rays[r_idx - 1]
                curr_p = in_corridor_rays[r_idx]
                is_adjacent_index = (1 <= curr_p['index'] - prev_p['index'] <= 2)
                is_coherent_range = abs(curr_p['val'] - prev_p['val']) <= self.crossing_max_range_jump_m
                if is_adjacent_index and is_coherent_range:
                    current_cluster.append(curr_p)
                else:
                    clusters.append(current_cluster)
                    current_cluster = [curr_p]
            clusters.append(current_cluster)

        # Evaluate valid clusters (minimum rays and lateral width span)
        valid_clusters = []
        is_pitched_up = (getattr(self, 'pitch_deg', 0.0) > 0.3 or getattr(self, 'ramp_slope_active', False))
        for c in clusters:
            c_rays = len(c)
            c_ys = [p['y'] for p in c]
            c_xs = [p['x'] for p in c]
            c_width = max(c_ys) - min(c_ys)
            # Filter ramp surface hits:
            # 1. On ramp pitch (imu_pitch_deg > 0.3 or ramp_slope_active), reject narrow <=2 ray ground returns
            if is_pitched_up and c_rays <= 2:
                continue
            # 2. Require physical lateral width (pedestrians have lateral width >= 10 mm)
            #    Do not allow longitudinal slope span (hypot dx, dy) to bypass lateral width
            if c_rays >= self.crossing_min_cluster_rays and c_width >= self.crossing_min_cluster_width_m:
                valid_clusters.append({
                    'rays': c,
                    'count': c_rays,
                    'width': c_width,
                    'min_range': min(p['val'] for p in c),
                    'nearest_p': min(c, key=lambda p: p['val'])
                })

        raw_points = len(in_corridor_rays)
        point_count = raw_points
        cluster_count = len(clusters)
        valid_cluster_count = len(valid_clusters)

        road_occupied = False
        reject_reason = 'NO_POINTS'
        best_cluster_rays = 0
        best_cluster_width = 0.0
        _zone = 'NONE'
        _cluster_class = 'NONE'

        if raw_points == 0:
            reject_reason = 'NO_POINTS'
        elif raw_points == 1:
            reject_reason = 'SINGLETON_ONLY'
        elif valid_cluster_count == 0:
            reject_reason = 'NO_VALID_CLUSTER'
            best_c = max(clusters, key=len)
            best_cluster_rays = len(best_c)
            best_c_ys = [p['y'] for p in best_c]
            best_cluster_width = max(best_c_ys) - min(best_c_ys)
            # No valid cluster => reset edge tracker (cluster lost)
            self._reset_edge_track()
        else:
            reject_reason = 'VALID_CLUSTER'
            best_vc = min(valid_clusters, key=lambda vc: vc['min_range'])
            best_cluster_rays = best_vc['count']
            best_cluster_width = best_vc['width']
            # Compute cluster centroid from best valid cluster (nearest)
            _bvc_rays = best_vc['rays']
            _cx = sum(p['x'] for p in _bvc_rays) / len(_bvc_rays)
            _cy = sum(p['y'] for p in _bvc_rays) / len(_bvc_rays)
            # CORE/EDGE zone classification (replaces Patch C)
            _zone, _cluster_class, _occupied_candidate = self._classify_crossing_cluster(
                _cx, _cy, self._scan_seq, is_new_scan
            )
            road_occupied = _occupied_candidate
            reject_reason = f'VALID_CLUSTER zone={_zone} class={_cluster_class}'
            # Diagnostic log
            _et = self._edge_track
            _prev_y = _et.get('prev_y') if _zone == 'EDGE' else None
            _inward_delta = (abs(_prev_y) - abs(_cy)) if (_prev_y is not None and _zone == 'EDGE') else 0.0
            _start_abs_y = _et.get('start_abs_y')
            _net_inward = (_start_abs_y - abs(_cy)) if (_start_abs_y is not None and _zone == 'EDGE') else 0.0
            self.log_crossing_object_diag(
                scan_seq=self._scan_seq,
                state=self.crossing_state,
                zone=_zone,
                classification=_cluster_class,
                x=_cx, y=_cy,
                prev_y=_prev_y,
                inward_delta=_inward_delta,
                inward_accum=_net_inward,
                motion_steps=_et.get('motion_steps', 0),
                occupied_candidate=_occupied_candidate,
            )

        hit = road_occupied

        if hit:
            all_valid_rays = [p for vc in valid_clusters for p in vc['rays']]
            best_ray = min(all_valid_rays, key=lambda p: p['val'])
            min_range = best_ray['val']
            nearest_x = best_ray['x']
            nearest_y = best_ray['y']
            status = CROSSING_OCCUPIED
            ys = [r['y'] for r in all_valid_rays]
            xs = [r['x'] for r in all_valid_rays]
            y_span = max(ys) - min(ys)
            cluster_info = {
                'count': len(all_valid_rays),
                'cluster_count': valid_cluster_count,
                'min_range': min_range,
                'median_range': min_range,
                'lateral_width_m': y_span,
                'min_x': min(xs),
                'max_x': max(xs),
                'median_x': nearest_x,
                'min_y': min(ys),
                'max_y': max(ys),
                'median_y': nearest_y,
                'rays': all_valid_rays
            }
        else:
            min_range = float('inf')
            nearest_x = float('inf')
            nearest_y = float('inf')
            y_span = 0.0
            status = CROSSING_CLEAR
            cluster_info = None

            # Lateral reject logging for objects beyond roadway corridor
            if outside_rays:
                closest_outside = min(outside_rays, key=lambda c: abs(c['y']))
                if abs(closest_outside['y']) <= 0.60:
                    self.log_crossing_reject_lateral(
                        closest_outside['val'],
                        closest_outside['x'],
                        closest_outside['y']
                    )

        # Log CROSSING_CLUSTER diagnostic (rate-limited)
        self.log_crossing_cluster(
            scan_seq=self._scan_seq,
            raw_corridor_points=raw_points,
            cluster_count=cluster_count,
            best_cluster_rays=best_cluster_rays,
            best_cluster_width=best_cluster_width,
            nearest_x=nearest_x,
            nearest_y=nearest_y,
            accepted=road_occupied,
            reject_reason=reject_reason
        )

        # Log occupancy
        self.log_crossing_occupancy(
            point_count=raw_points,
            cluster_count=cluster_count,
            nearest_x=nearest_x,
            y_span=y_span,
            occupied=road_occupied
        )

        result = {
            'status': status,
            'occupied': road_occupied,
            'hit': hit,
            'cluster': cluster_info,
            'corridor_rays_count': point_count,
            'cluster_count': cluster_count,
            'min_range': min_range,
            'x': nearest_x,
            'y': nearest_y,
            'cluster_x': _cx if valid_cluster_count > 0 else float('inf'),
            'cluster_y': _cy if valid_cluster_count > 0 else float('inf'),
            'y_span': y_span,
            'entry_candidate': None,
            'entry_threat': False,
            'inward_delta': 0.0,
            'is_clear': (status == CROSSING_CLEAR),
            'is_unknown': False,
            'unknown_reason': 'NONE',
            'is_new_scan': True,
            'scan_stamp': scan_stamp,
            'zone': _zone,
            'cluster_class': _cluster_class,
        }

        self._crossing_processed_scan_seq = self._scan_seq
        self._last_crossing_scan = target_scan
        self._cached_crossing_occ = result
        return result

    def update_crossing_state(self, zebra_detected, is_occupied=False, crossing_occ=None,
                              is_entry_threat=False, is_new_scan=None, is_new_image=None):
        """Simplified crossing state manager strictly implementing UEH CRC 2026 requirement:
        IDLE
         -> confirmed zebra (>= zebra_confirm_frames) -> APPROACH

        APPROACH
         -> pedestrian confirmed in roadway (>= crossing_confirm_scans new scans) -> WAIT (reason=ROAD_OCCUPIED)
         -> zebra absent (>= zebra_exit_frames) -> PASS (reason=ZEBRA_LEFT_CAMERA)

        WAIT
         -> road clear for >= crossing_clear_scans new scans -> PASS (reason=ROAD_CLEAR)
         -> pedestrian remains on road -> stay in WAIT (v=0, w=0)

        PASS
         -> road re-occupied (>= crossing_confirm_scans new scans) -> WAIT (reason=ROAD_OCCUPIED)
         -> cleared zebra area (displacement >= crossing_pass_min_distance_m and zebra absent) -> IDLE (reason=ZEBRA_PASSED)
        """
        old_state = self.crossing_state

        if is_new_image is None:
            is_new_image = True

        if is_new_scan is None:
            if crossing_occ is not None:
                is_new_scan = crossing_occ.get('is_new_scan', True)
            else:
                is_new_scan = True

        # Update visual zebra detection confirmation counters
        if is_new_image:
            if zebra_detected:
                self.zebra_detect_count += 1
                self.zebra_lost_count = 0
                self.log_crossing_zebra(
                    self.last_zebra_bands, self.last_zebra_score,
                    self.zebra_detect_count, self.zebra_confirm_frames
                )
            else:
                self.zebra_lost_count += 1
                self.zebra_detect_count = 0

        # Determine explicit risk status
        hit = False
        nearest_x = float('inf')
        nearest_y = float('inf')
        if crossing_occ is not None:
            status = crossing_occ.get('status')
            hit = crossing_occ.get('hit', False) or crossing_occ.get('occupied', False)
            nearest_x = crossing_occ.get('x', float('inf'))
            nearest_y = crossing_occ.get('y', float('inf'))
            if not status:
                status = CROSSING_OCCUPIED if (hit or is_occupied) else CROSSING_CLEAR
        else:
            hit = is_occupied
            status = CROSSING_OCCUPIED if is_occupied else CROSSING_CLEAR
            if is_occupied:
                self.crossing_detect_count = max(self.crossing_detect_count, self.crossing_confirm_scans)

        now_sec = self._get_current_time_sec()
        if crossing_occ is not None and crossing_occ.get('scan_stamp', 0.0) > 0.0:
            now_sec = crossing_occ['scan_stamp']

        # Update roadway occupancy and clearance counters ONLY on genuinely new LiDAR scans
        if is_new_scan:
            if status == CROSSING_OCCUPIED:
                self.crossing_detect_count += 1
                self.crossing_clear_count = 0
                self.crossing_clear_since = None
                if crossing_occ and crossing_occ.get('cluster'):
                    c = crossing_occ['cluster']
                    self.log_crossing_hard_occupied(
                        c['count'], crossing_occ['x'], crossing_occ['y'],
                        r_min=crossing_occ.get('min_range'),
                        confirm=self.crossing_detect_count,
                        total=self.crossing_confirm_scans,
                        scan_seq=self._scan_seq
                    )
            elif status == CROSSING_CLEAR:
                self.crossing_detect_count = 0
                self.crossing_clear_count += 1
                if self.crossing_clear_since is None:
                    self.crossing_clear_since = now_sec
                self.log_crossing_clear(self.crossing_clear_count, self.crossing_clear_scans, scan_seq=self._scan_seq)
            elif status == CROSSING_UNKNOWN:
                # UNKNOWN: safety principle UNKNOWN != CLEAR; do NOT increment clear_count or start clear_since
                pass

        # Temporal debounce check
        clear_duration = (now_sec - self.crossing_clear_since) if self.crossing_clear_since is not None else 0.0
        road_occupied_confirmed = (self.crossing_detect_count >= self.crossing_confirm_scans)
        road_clear_confirmed = (self.crossing_clear_count >= self.crossing_clear_scans and
                                (self.crossing_clear_hold_s <= 0.0 or clear_duration >= self.crossing_clear_hold_s))

        # Rate-limited CROSSING status logging
        crossing_context_str = 'ACTIVE' if (self.crossing_state != 'IDLE' or zebra_detected or self.zebra_detect_count > 0) else 'INACTIVE'
        self.log_crossing_status(crossing_context_str, status, self.crossing_detect_count, self.crossing_clear_count)

        # Diagnostic per-NEW-scan logging while crossing context is active
        if crossing_context_str == 'ACTIVE' and is_new_scan:
            self.log_crossing_scan(
                self._scan_seq, hit=hit, nearest_x=nearest_x, nearest_y=nearest_y,
                occupied_count=self.crossing_detect_count, clear_count=self.crossing_clear_count,
                status=status
            )

        # State transitions
        transition_reason = None
        if self.crossing_state == 'IDLE':
            if self.zebra_detect_count >= self.zebra_confirm_frames:
                self.crossing_state = 'APPROACH'
                self.crossing_clear_count = 0
                self.crossing_clear_since = now_sec
                self._reset_edge_track()  # Reset EDGE tracker for new crossing event
                transition_reason = 'ZEBRA_CONFIRMED'
                if road_occupied_confirmed:
                    self.crossing_state = 'WAIT'
                    self.crossing_clear_count = 0
                    self.crossing_clear_since = None
                    self.crossing_stop_reason = 'ROAD_OCCUPIED'
                    transition_reason = 'ROAD_OCCUPIED'
                    self.log_crossing_stop('ROAD_OCCUPIED')

        elif self.crossing_state == 'APPROACH':
            if road_occupied_confirmed:
                self.crossing_state = 'WAIT'
                self.crossing_clear_count = 0
                self.crossing_clear_since = None
                self.crossing_stop_reason = 'ROAD_OCCUPIED'
                transition_reason = 'ROAD_OCCUPIED'
                self.log_crossing_stop('ROAD_OCCUPIED')
            elif self.zebra_lost_count >= self.zebra_exit_frames:
                if road_clear_confirmed:
                    self.crossing_state = 'PASS'
                    transition_reason = 'ROAD_CLEAR_CONFIRMED'
                    self.log_crossing_clear_hold(clear_duration, self.crossing_clear_hold_s)
                    if getattr(self, 'has_odom', False):
                        if self.crossing_pass_start_x is None:
                            self.crossing_pass_start_x = self.x
                            self.crossing_pass_start_y = self.y
                    else:
                        self.crossing_pass_start_x = None
                        self.crossing_pass_start_y = None

        elif self.crossing_state in ('STOP', 'WAIT'):
            if status == CROSSING_OCCUPIED:
                self.crossing_clear_count = 0
                self.crossing_clear_since = None
                self.crossing_stop_reason = 'ROAD_OCCUPIED'
            elif status == CROSSING_CLEAR:
                if road_clear_confirmed:
                    self.crossing_state = 'PASS'
                    self.crossing_stop_reason = 'NONE'
                    self._reset_edge_track()  # Reset EDGE tracker when crossing clears
                    transition_reason = 'ROAD_CLEAR'
                    self.log_crossing_clear_hold(clear_duration, self.crossing_clear_hold_s)
                    self.log_crossing_release('ROAD_CLEAR')
                    if getattr(self, 'has_odom', False):
                        self.crossing_pass_start_x = self.x
                        self.crossing_pass_start_y = self.y
                    else:
                        self.crossing_pass_start_x = None
                        self.crossing_pass_start_y = None
            elif status == CROSSING_UNKNOWN:
                # UNKNOWN does not release STOP
                pass

        elif self.crossing_state == 'PASS':
            # Initialize PASS start pose if not yet recorded
            if self.crossing_pass_start_x is None and getattr(self, 'has_odom', False):
                self.crossing_pass_start_x = self.x
                self.crossing_pass_start_y = self.y

            pass_distance = 0.0
            has_valid_odom = False
            if ((getattr(self, 'has_odom', False) or self.crossing_pass_start_x is not None)
                    and self.crossing_pass_start_x is not None
                    and self.crossing_pass_start_y is not None):
                has_valid_odom = True
                pass_distance = math.hypot(self.x - self.crossing_pass_start_x, self.y - self.crossing_pass_start_y)
                self.log_crossing_pass(pass_distance, self.crossing_pass_min_distance_m)

            # Check if crossing has been cleared
            is_cleared = False
            if has_valid_odom and pass_distance >= self.crossing_pass_min_distance_m:
                if (self.zebra_lost_count >= self.zebra_exit_frames and
                        not road_occupied_confirmed and not is_occupied and
                        status != CROSSING_OCCUPIED):
                    is_cleared = True
            elif (getattr(self, 'pitch_deg', 0.0) > 1.0 or getattr(self, 'ramp_slope_active', False)) and self.zebra_lost_count >= self.zebra_exit_frames:
                # Entering physical ramp after zebra left camera => crossing completed
                is_cleared = True

            if is_cleared:
                self.crossing_state = 'IDLE'
                self.crossing_pass_start_x = None
                self.crossing_pass_start_y = None
                self._reset_edge_track()  # Reset EDGE tracker on full crossing completion
                transition_reason = 'ZEBRA_PASSED'
                self.log_crossing_exit('ZEBRA_PASSED')
            else:
                # CORE/EDGE zone-based PASS->WAIT re-entry guard (replaces Patch C x-coordinate heuristic)
                #
                # CORE_OCCUPIED: pedestrian is in road center => PASS->WAIT after normal temporal confirmation
                # MOVING_INTO_ROAD: pedestrian moving from edge toward road center => PASS->WAIT
                # STATIC_EDGE: static pole on road edge => MUST NOT trigger PASS->WAIT
                # MOVING_OUTWARD: object moving away from road => MUST NOT trigger PASS->WAIT
                _pass_road_occ = False
                if road_occupied_confirmed or (is_occupied and self.crossing_detect_count >= self.crossing_confirm_scans):
                    if crossing_occ is not None:
                        _cluster_class = crossing_occ.get('cluster_class', 'NONE')
                        _zone = crossing_occ.get('zone', 'NONE')
                        # Accept re-entry only for CORE or confirmed inward-moving EDGE object
                        if _cluster_class in ('CORE_OCCUPIED', 'MOVING_INTO_ROAD'):
                            _pass_road_occ = True
                        else:
                            # STATIC_EDGE or MOVING_OUTWARD: reject re-entry, reset detect count
                            self.crossing_detect_count = 0
                    else:
                        _pass_road_occ = True  # trusted caller, no scan context
                if _pass_road_occ:
                    self.crossing_state = 'WAIT'
                    self.crossing_clear_count = 0
                    self.crossing_clear_since = None
                    self.crossing_stop_reason = 'ROAD_OCCUPIED'
                    transition_reason = 'ROAD_OCCUPIED'
                    self.log_crossing_stop('ROAD_OCCUPIED')

        # Log state transition
        if self.crossing_state != old_state:
            self.log_crossing_transition(old_state, self.crossing_state, reason=transition_reason)
            self._last_crossing_state = self.crossing_state

    def _publish_debug_viz(self, roi, mask, left_x, right_x, target_x, center_x,
                           perception_mode='LANE', conf=0.0, coverage=0.0,
                           valid_rows=0, row_endpoints=None,
                           brightness=0.0, dark_rows=None, skipped_transverse=None,
                           near_center=None, far_center=None,
                           dark_target=None, filtered_target=None, err=0.0,
                           scene='NORMAL', scene_mean=0.0, scene_median=0.0,
                           scene_p25=0.0, dark_ratio=0.0,
                           crossing_state='IDLE', crossing_occupied=False,
                           zebra_detected=False, zebra_bands=0):
        """Construct and publish debug visualization on /camera/debug_lane."""
        if not self.debug_viz or self.bridge is None or roi is None:
            return
        if self.pub_debug_img.get_subscription_count() == 0:
            return
        try:
            h_roi, w_roi = roi.shape[:2]
            if mask is not None:
                mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            else:
                mask_bgr = np.zeros_like(roi)

            viz = np.hstack([roi.copy(), mask_bgr])

            for offset_x in [0, w_roi]:
                # Image center line (yellow)
                cx = int(round(center_x + offset_x))
                cv2.line(viz, (cx, 0), (cx, h_roi), (0, 255, 255), 1)

                if perception_mode in ('DARK_LANE', 'DARK_HOLD'):
                    # Draw skipped transverse indicators
                    if skipped_transverse:
                        for sr in skipped_transverse:
                            cv2.line(viz, (offset_x, sr), (offset_x + w_roi, sr), (255, 0, 255), 1)

                    # Draw all scan row candidates (ACCEPTED vs REJECTED)
                    rows_to_draw = getattr(self, 'last_dark_rows', []) or dark_rows or []
                    for item in rows_to_draw:
                        r = getattr(item, 'row_y', item[0] if isinstance(item, (tuple, list)) else None)
                        rc = getattr(item, 'row_center', item[1] if isinstance(item, (tuple, list)) else None)
                        lx = getattr(item, 'left_x', item[2] if isinstance(item, (tuple, list)) and len(item) > 2 else None)
                        rx = getattr(item, 'right_x', item[3] if isinstance(item, (tuple, list)) and len(item) > 3 else None)
                        status = getattr(item, 'status', 'ACCEPTED')
                        reason = getattr(item, 'reject_reason', 'NONE')

                        if r is not None:
                            if lx is not None:
                                cv2.circle(viz, (int(round(lx + offset_x)), r), 3, (255, 0, 0), -1)
                            if rx is not None:
                                cv2.circle(viz, (int(round(rx + offset_x)), r), 3, (0, 0, 255), -1)
                            if rc is not None:
                                color = (0, 255, 0) if status == 'ACCEPTED' else (0, 0, 255)
                                cv2.circle(viz, (int(round(rc + offset_x)), r), 4, color, -1)
                                if status == 'REJECTED' and reason != 'NONE':
                                    cv2.putText(viz, reason[:4], (int(round(rc + offset_x)) + 6, r + 3),
                                                cv2.FONT_HERSHEY_SIMPLEX, 0.28, (0, 0, 255), 1, cv2.LINE_AA)

                    if near_center is not None:
                        nc_x = int(round(near_center + offset_x))
                        cv2.circle(viz, (nc_x, int(h_roi * 0.75)), 5, (0, 200, 200), -1)

                    if far_center is not None:
                        fc_x = int(round(far_center + offset_x))
                        cv2.circle(viz, (fc_x, int(h_roi * 0.25)), 5, (200, 0, 200), -1)
                elif perception_mode == 'LANE':
                    # Left detected boundary (blue)
                    if left_x is not None:
                        lx = int(round(left_x + offset_x))
                        cv2.circle(viz, (lx, h_roi // 2), 5, (255, 0, 0), -1)
                        cv2.line(viz, (lx, 0), (lx, h_roi), (255, 0, 0), 1)

                    # Right detected boundary (red)
                    if right_x is not None:
                        rx = int(round(right_x + offset_x))
                        cv2.circle(viz, (rx, h_roi // 2), 5, (0, 0, 255), -1)
                        cv2.line(viz, (rx, 0), (rx, h_roi), (0, 0, 255), 1)

                elif perception_mode == 'SURFACE' and row_endpoints:
                    # Draw sampled surface boundaries
                    for r, l_px, r_px in row_endpoints:
                        cv2.circle(viz, (int(round(l_px + offset_x)), r), 2, (255, 100, 0), -1)
                        cv2.circle(viz, (int(round(r_px + offset_x)), r), 2, (0, 100, 255), -1)

                # Target X (green)
                if target_x is not None:
                    tx = int(round(target_x + offset_x))
                    cv2.circle(viz, (tx, h_roi // 2), 6, (0, 255, 0), -1)
                    cv2.line(viz, (tx, 0), (tx, h_roi), (0, 255, 0), 2)

            # Draw semi-transparent dark overlay banner for text readability
            overlay_bg = viz.copy()
            banner_h = 108 if (scene == 'DARK' or perception_mode in ('DARK_LANE', 'DARK_HOLD')) else 92
            cv2.rectangle(overlay_bg, (5, 5), (630, banner_h), (0, 0, 0), -1)
            cv2.addWeighted(overlay_bg, 0.65, viz, 0.35, 0, viz)

            # Multi-line debug overlay
            mode_color = (0, 255, 255) if scene in ('DARK_ACTIVE', 'DARK', 'DARK_PENDING') else (0, 255, 0)
            if crossing_state != 'IDLE':
                occ_tag = ' [PED_OCCUPIED]' if crossing_occupied else ''
                line1 = f'SCENE: {scene} | {perception_mode} | CROSSING: {crossing_state}{occ_tag}'
            else:
                line1 = f'SCENE_STATE: {scene} | PERCEPTION: {perception_mode}'
            cv2.putText(viz, line1, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.50, mode_color, 2, cv2.LINE_AA)

            t_str = f'{target_x:.1f}' if target_x is not None else 'None'

            if scene == 'DARK_PENDING':
                ready_rows = valid_rows
                ready_count = getattr(self, 'dark_ready_count', 0)
                pending_count = getattr(self, 'dark_pending_count', 0)
                anchor_target = getattr(self, 'last_stable_normal_target_x', 320.0)
                anchor_str = f'{anchor_target:.1f}' if anchor_target is not None else 'None'
                cand_str = f'{dark_target:.1f}' if dark_target is not None else 'None'

                line2 = f'pending={pending_count}/{self.dark_pending_max_frames} dark_ready_rows={ready_rows} dark_ready_count={ready_count}/{self.dark_ready_frames}'
                cv2.putText(viz, line2, (10, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (220, 220, 220), 1, cv2.LINE_AA)

                line3 = f'anchor_target={anchor_str} dark_candidate_target={cand_str}'
                cv2.putText(viz, line3, (10, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 255, 255), 1, cv2.LINE_AA)

                line4 = f'error={err:+.2f} forward_dark_ratio={dark_ratio:.3f} median={scene_median:.1f}'
                cv2.putText(viz, line4, (10, 74), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200, 200, 200), 1, cv2.LINE_AA)

            elif scene in ('DARK_ACTIVE', 'DARK') or perception_mode in ('DARK_LANE', 'DARK_HOLD'):
                all_c = [f'{r.row_center:.0f}' for r in getattr(self, 'last_dark_rows', []) if getattr(r, 'row_center', None) is not None]
                acc_c = [f'{r.row_center:.0f}' for r in getattr(self, 'last_dark_rows', []) if getattr(r, 'status', '') == 'ACCEPTED' and getattr(r, 'row_center', None) is not None]
                dual_c = sum(1 for r in getattr(self, 'last_dark_rows', []) if getattr(r, 'status', '') == 'ACCEPTED' and getattr(r, 'boundary_type', '') == 'DUAL')
                single_c = sum(1 for r in getattr(self, 'last_dark_rows', []) if getattr(r, 'status', '') == 'ACCEPTED' and getattr(r, 'boundary_type', '') == 'SINGLE')
                nc_str = f'{near_center:.1f}' if near_center is not None else 'None'
                fc_str = f'{far_center:.1f}' if far_center is not None else 'None'
                raw_t_str = f'{dark_target:.1f}' if dark_target is not None else t_str

                line2 = f'all_centers=[{", ".join(all_c)}]  accepted=[{", ".join(acc_c)}]'
                cv2.putText(viz, line2, (10, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (220, 220, 220), 1, cv2.LINE_AA)

                line3 = f'dual={dual_c} single={single_c} med={scene_median:.1f} near={nc_str} far={fc_str}'
                cv2.putText(viz, line3, (10, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200, 200, 200), 1, cv2.LINE_AA)

                line4 = f'raw_target={raw_t_str} filtered={t_str} entry_stabilize={self.dark_entry_stabilize_count}/{self.dark_entry_stabilize_frames}'
                cv2.putText(viz, line4, (10, 74), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 255, 255), 1, cv2.LINE_AA)

                line5 = f'error={err:+.2f} enter={self.dark_enter_count}/{self.dark_enter_frames} exit={self.dark_exit_count}/{self.dark_exit_frames}'
                cv2.putText(viz, line5, (10, 92), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (220, 220, 220), 1, cv2.LINE_AA)
            else:
                med_val = scene_median if scene_median != 0.0 else brightness
                line2 = f'mean={scene_mean:.1f} median={med_val:.1f} p25={scene_p25:.1f} dark_ratio={dark_ratio:.2f}'
                cv2.putText(viz, line2, (10, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (220, 220, 220), 1, cv2.LINE_AA)

                line3 = f'enter={self.dark_enter_count}/{self.dark_enter_frames} exit={self.dark_exit_count}/{self.dark_exit_frames}'
                cv2.putText(viz, line3, (10, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (200, 200, 200), 1, cv2.LINE_AA)

                if perception_mode == 'LANE':
                    lx_str = f'{left_x:.1f}' if left_x is not None else 'None'
                    rx_str = f'{right_x:.1f}' if right_x is not None else 'None'
                    line4 = f'left={lx_str} right={rx_str} target={t_str} conf={conf:.2f} err={err:+.2f}'
                    detail_color = (0, 255, 0)
                elif perception_mode == 'SURFACE':
                    line4 = f'cov={coverage:.2f} rows={valid_rows} target={t_str} err={err:+.2f}'
                    detail_color = (0, 200, 255)
                elif perception_mode == 'CROSSING_WAIT':
                    occ_str = 'true' if crossing_occupied else 'false'
                    line4 = f'CROSSING_WAIT occ={occ_str} clear={getattr(self, "crossing_clear_count", 0)}/{getattr(self, "crossing_clear_scans", 5)}'
                    detail_color = (0, 0, 255)
                else:
                    line4 = 'target=None err=0.0'
                    detail_color = (0, 0, 255)
                cv2.putText(viz, line4, (10, 82), cv2.FONT_HERSHEY_SIMPLEX, 0.44, detail_color, 1, cv2.LINE_AA)

            msg = self.bridge.cv2_to_imgmsg(viz, 'bgr8')
            self.pub_debug_img.publish(msg)
        except Exception:
            pass

    @staticmethod
    def _compute_median(vals):
        if not vals:
            return float('inf')
        s = sorted(vals)
        mid = len(s) // 2
        if len(s) % 2 == 1:
            return float(s[mid])
        return float((s[mid - 1] + s[mid]) / 2.0)

    def _analyze_cluster(self, c):
        """Compute physical metrics for a contiguous ray cluster in robot-relative coordinates."""
        c_ranges = [r['val'] for r in c]
        xs = [r['val'] * math.cos(r['angle_rad']) for r in c]
        ys = [r['val'] * math.sin(r['angle_rad']) for r in c]
        lat_width = float(max(ys) - min(ys)) if ys else 0.0
        chord_width = float(math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])) if xs else 0.0
        return {
            'count': len(c),
            'min_range': min(c_ranges),
            'median_range': self._compute_median(c_ranges),
            'start_angle': c[0]['angle'],
            'end_angle': c[-1]['angle'],
            'lateral_width_m': lat_width,
            'chord_width_m': chord_width,
            'rays': c
        }

    def detect_front_obstacles(self, scan=None, is_new_scan=None):
        """Analyze front sector for obstacles using contiguous ray clustering,
        physical cluster width validation, temporal confirmation, clearance
        hysteresis, and emergency central protection.

        Returns:
            dict with 'stop', 'reason', 'front_distance', 'cluster', 'confirm_count', 'outliers'
        """
        target_scan = scan if scan is not None else self.scan
        if target_scan is None or not target_scan.ranges:
            return {
                'stop': False,
                'reason': 'NO_SCAN',
                'front_distance': float('inf'),
                'cluster': None,
                'confirm_count': 0,
                'outliers': [],
                'self_returns': []
            }

        if is_new_scan is None:
            is_new_scan = (target_scan is not self._last_processed_scan or
                           self._scan_seq != self._last_processed_scan_seq)

        n = len(target_scan.ranges)
        angle_min = float(target_scan.angle_min)
        angle_inc = float(target_scan.angle_increment)
        r_min = float(target_scan.range_min)
        r_max = float(target_scan.range_max)

        # 1. Extract rays within front sector [-front_sector_deg, +front_sector_deg]
        front_rays = []
        self_return_rays = []
        front_half_deg = self.front_sector_deg
        for i in range(n):
            angle_rad = angle_min + i * angle_inc
            # Normalize angle to [-pi, +pi]
            norm_rad = math.atan2(math.sin(angle_rad), math.cos(angle_rad))
            deg = math.degrees(norm_rad)

            if -front_half_deg <= deg <= front_half_deg:
                r = target_scan.ranges[i]
                is_valid = math.isfinite(r) and ((r_min - 1e-4) <= r <= (r_max + 1e-4))
                val = float(r) if is_valid else float('inf')

                # Physical self-footprint mask check
                if is_valid:
                    x = val * math.cos(norm_rad)
                    y = val * math.sin(norm_rad)
                    if (self.footprint_x_min <= x <= self.footprint_x_max and
                            self.footprint_y_min <= y <= self.footprint_y_max):
                        self_return_rays.append({'angle': deg, 'angle_rad': norm_rad, 'val': val, 'index': i, 'x': x, 'y': y})
                        continue

                front_rays.append({'angle': deg, 'angle_rad': norm_rad, 'val': val, 'index': i, 'valid': is_valid})

        outliers = []
        if self_return_rays:
            self.log_self_return(self_return_rays)
            self_ret_info = self._analyze_cluster(self_return_rays)
            self_ret_info['reason'] = 'SELF_RETURN'
            outliers.append(self_ret_info)

        if not front_rays:
            return {
                'stop': False,
                'reason': 'CLEAR' if self_return_rays else 'NO_RAYS',
                'front_distance': float('inf'),
                'cluster': None,
                'confirm_count': 0,
                'outliers': outliers,
                'self_returns': self_return_rays
            }

        # Sort rays monotonically by angle from -front_half_deg to +front_half_deg
        front_rays.sort(key=lambda x: x['angle'])

        # 2. Emergency Rule: Narrow central sector [-emergency_sector_deg, +emergency_sector_deg]
        emergency_rays = [r for r in front_rays if -self.emergency_sector_deg <= r['angle'] <= self.emergency_sector_deg]
        emergency_clusters = []
        curr_em_run = []
        for r in emergency_rays:
            if r['val'] < self.emergency_distance:
                curr_em_run.append(r)
            else:
                if curr_em_run:
                    emergency_clusters.append(curr_em_run)
                    curr_em_run = []
        if curr_em_run:
            emergency_clusters.append(curr_em_run)

        emergency_stop_cluster = None
        for em_c in emergency_clusters:
            em_info = self._analyze_cluster(em_c)
            if (em_info['count'] >= self.emergency_min_cluster_rays and
                    em_info['lateral_width_m'] >= self.emergency_min_width_m):
                emergency_stop_cluster = em_info
                break

        if emergency_stop_cluster is not None:
            if is_new_scan:
                self._last_processed_scan = target_scan
                self._last_processed_scan_seq = self._scan_seq
            self.obstacle_stop_active = True
            self.obstacle_confirm_count = self.obstacle_confirm_scans
            return {
                'stop': True,
                'reason': 'EMERGENCY',
                'front_distance': emergency_stop_cluster['median_range'],
                'cluster': emergency_stop_cluster,
                'confirm_count': self.obstacle_confirm_count,
                'outliers': [],
                'self_returns': self_return_rays
            }

        # 3. Standard Contiguous Cluster Detection with Hysteresis & Physical Width
        active_thresh = self.clear_distance if self.obstacle_stop_active else self.stop_distance

        clusters = []
        curr_run = []
        for r in front_rays:
            if r['val'] < active_thresh:
                curr_run.append(r)
            else:
                if curr_run:
                    clusters.append(curr_run)
                    curr_run = []
        if curr_run:
            clusters.append(curr_run)

        valid_clusters = []
        for c in clusters:
            c_info = self._analyze_cluster(c)
            if c_info['count'] < self.min_obstacle_cluster_rays:
                c_info['reason'] = 'TOO_FEW_RAYS'
                outliers.append(c_info)
            elif c_info['lateral_width_m'] < self.min_obstacle_width_m:
                c_info['reason'] = 'TOO_NARROW'
                outliers.append(c_info)
            else:
                valid_clusters.append(c_info)

        # Collect indices of outlier rays
        outlier_indices = set()
        for o in outliers:
            for r in o['rays']:
                outlier_indices.add(r['index'])

        # 4. Handle Detection Result
        if not valid_clusters:
            if is_new_scan:
                self._last_processed_scan = target_scan
                self._last_processed_scan_seq = self._scan_seq
                self.obstacle_confirm_count = 0
            self.obstacle_stop_active = False

            # Robust front distance: exclude outlier rays so transient glitches don't report as front obstacle
            non_outlier_vals = [r['val'] for r in front_rays if r['valid'] and r['index'] not in outlier_indices]
            general_front = self._compute_median(non_outlier_vals) if non_outlier_vals else float('inf')

            if outliers:
                narrow_outliers = [o for o in outliers if o.get('reason') == 'TOO_NARROW']
                if narrow_outliers:
                    self.log_outlier(narrow_outliers[0])
                else:
                    self.log_outlier(min(outliers, key=lambda o: o['min_range']))

            return {
                'stop': False,
                'reason': 'CLEAR',
                'front_distance': general_front,
                'cluster': None,
                'confirm_count': 0,
                'outliers': outliers,
                'self_returns': self_return_rays
            }

        # Candidate clusters present: pick closest by median range
        best_cluster = min(valid_clusters, key=lambda c: c['median_range'])

        if is_new_scan:
            self._last_processed_scan = target_scan
            self._last_processed_scan_seq = self._scan_seq
            self.obstacle_confirm_count += 1

        if self.obstacle_confirm_count >= self.obstacle_confirm_scans:
            self.obstacle_stop_active = True
            return {
                'stop': True,
                'reason': 'CONFIRMED',
                'front_distance': best_cluster['median_range'],
                'cluster': best_cluster,
                'confirm_count': self.obstacle_confirm_count,
                'outliers': outliers,
                'self_returns': self_return_rays
            }
        else:
            self.log_candidate(best_cluster, self.obstacle_confirm_count, self.obstacle_confirm_scans)
            return {
                'stop': False,
                'reason': 'CANDIDATE',
                'front_distance': best_cluster['median_range'],
                'cluster': best_cluster,
                'confirm_count': self.obstacle_confirm_count,
                'outliers': outliers,
                'self_returns': self_return_rays
            }

    def log_self_return(self, rays):
        """Throttled diagnostic logging for LiDAR self-footprint returns (~1 Hz)."""
        now = time.time()
        if now - self._last_log.get('self_return', 0.0) >= 1.0:
            self._last_log['self_return'] = now
            xs = [r['x'] for r in rays]
            ys = [r['y'] for r in rays]
            min_x, max_x = min(xs), max(xs)
            min_y, max_y = min(ys), max(ys)
            pitch_str = f' imu_pitch_deg={self.pitch_deg:+.1f}' if hasattr(self, 'pitch_deg') else ''
            self.get_logger().info(
                f'LIDAR_SELF_RETURN rays={len(rays)} '
                f'x_range=[{min_x:.3f}..{max_x:.3f}] y_range=[{min_y:.3f}..{max_y:.3f}]{pitch_str}'
            )

    def log_outlier(self, outlier, rays=None):
        """Throttled logging for isolated or narrow LiDAR outliers (~1 Hz)."""
        now = time.time()
        if now - self._last_log.get('outlier', 0.0) >= 1.0:
            self._last_log['outlier'] = now
            pitch_str = f' imu_pitch_deg={self.pitch_deg:+.1f}' if hasattr(self, 'pitch_deg') else ''
            if isinstance(outlier, dict):
                if outlier.get('reason') == 'TOO_NARROW':
                    self.get_logger().info(
                        f'LIDAR_OUTLIER reason=TOO_NARROW rays={outlier["count"]} '
                        f'median={outlier["median_range"]:.2f} min={outlier["min_range"]:.2f} '
                        f'width={outlier["lateral_width_m"]:.3f} '
                        f'angle_start={outlier["start_angle"]:.0f} angle_end={outlier["end_angle"]:.0f}{pitch_str}')
                else:
                    self.get_logger().info(
                        f'LIDAR_OUTLIER min={outlier["min_range"]:.2f} rays={outlier["count"]} ignored{pitch_str}')
            else:
                self.get_logger().info(f'LIDAR_OUTLIER min={outlier:.2f} rays={rays} ignored{pitch_str}')

    def log_candidate(self, cluster, confirm, total):
        """Throttled logging for candidate obstacle cluster (~1 Hz)."""
        now = time.time()
        if now - self._last_log.get('candidate', 0.0) >= 1.0:
            self._last_log['candidate'] = now
            w = cluster.get('lateral_width_m', 0.0)
            pitch_str = f' imu_pitch_deg={self.pitch_deg:+.1f}' if hasattr(self, 'pitch_deg') else ''
            self.get_logger().info(
                f'OBSTACLE_CANDIDATE rays={cluster["count"]} median={cluster["median_range"]:.2f}m '
                f'min={cluster["min_range"]:.2f}m width={w:.3f}m confirm={confirm}/{total}{pitch_str}')

    def correlate_obstacle_with_crossing(self, obs, crossing_occ):
        """Correlates generic obstacle cluster with crossing perception.

        Determines whether a generic obstacle detection corresponds to a known
        STATIC_EDGE or MOVING_OUTWARD pedestrian/pole identified by the crossing classifier.

        Safety Requirements (Phases 5, 6, 11):
        - Classification must be STATIC_EDGE or MOVING_OUTWARD.
        - Crossing data must be fresh (current scan_seq or diff <= 1, age <= 0.5s).
        - Robot-frame spatial association:
            abs(generic_x - crossing_x) <= association_dx (0.20 m)
            abs(generic_y - crossing_y) <= association_dy (0.20 m)
            same lateral side: generic_y * crossing_y >= -1e-4
        - If correlation does not match with high certainty: return False (generic obstacle wins).
        """
        if getattr(self, 'scene_state', 'NORMAL') in ('DARK_PENDING', 'DARK_ACTIVE', 'DARK'):
            # Pedestrian crossing semantics are OFF inside dark tunnel; generic front obstacle safety stays ON
            return False, None, None

        if obs is None or not obs.get('stop', False):
            return False, None, None

        gen_cluster = obs.get('cluster')
        if not gen_cluster or 'rays' not in gen_cluster:
            return False, None, None

        rays = gen_cluster['rays']
        if not rays:
            return False, None, None

        # Compute generic obstacle centroid in robot frame
        gx = sum(r['val'] * math.cos(r['angle_rad']) for r in rays) / len(rays)
        gy = sum(r['val'] * math.sin(r['angle_rad']) for r in rays) / len(rays)

        if crossing_occ is None:
            return False, gx, gy

        crossing_class = crossing_occ.get('cluster_class', 'NONE')
        if crossing_class not in ('STATIC_EDGE', 'MOVING_OUTWARD'):
            return False, gx, gy

        # Freshness / Stale check (Phase 11)
        now_sec = self._get_current_time_sec()
        scan_stamp = crossing_occ.get('scan_stamp', 0.0)
        if scan_stamp > 0.0 and (now_sec - scan_stamp) > self.crossing_scan_stale_timeout_s:
            return False, gx, gy

        # Check scan_seq freshness
        crossing_scan_seq = getattr(self, '_crossing_processed_scan_seq', self._scan_seq)
        if abs(self._scan_seq - crossing_scan_seq) > 1:
            return False, gx, gy

        cx = crossing_occ.get('cluster_x', float('inf'))
        cy = crossing_occ.get('cluster_y', float('inf'))
        if not (math.isfinite(cx) and math.isfinite(cy)):
            cx = self._edge_track.get('curr_x')
            cy = self._edge_track.get('curr_y')
            if cx is None or cy is None:
                return False, gx, gy

        # Spatial correlation gate (Phase 6)
        dx = abs(gx - cx)
        dy = abs(gy - cy)
        same_side = (gy * cy >= -1e-4)

        ASSOCIATION_DX = 0.20  # meters
        ASSOCIATION_DY = 0.20  # meters

        if dx <= ASSOCIATION_DX and dy <= ASSOCIATION_DY and same_side:
            return True, gx, gy

        return False, gx, gy

    def log_speed_arbitration(self, state, base_v, crossing_class, crossing_scan_seq,
                              generic_obstacle, generic_x, generic_y,
                              matched_static_edge, final_v, reason):
        """Rate-limited (1 Hz) diagnostic log for velocity arbitration layer (Phase 12)."""
        now = time.time()
        if now - self._last_speed_arb_log_time < 1.0:
            return
        self._last_speed_arb_log_time = now
        gx_str = f'{generic_x:.2f}' if generic_x is not None else 'None'
        gy_str = f'{generic_y:.2f}' if generic_y is not None else 'None'
        self.get_logger().info(
            f'SPEED_ARB state={state} base_v={base_v:.2f} '
            f'crossing_class={crossing_class} crossing_scan_seq={crossing_scan_seq} '
            f'generic_obstacle={generic_obstacle} generic_x={gx_str} generic_y={gy_str} '
            f'matched_static_edge={matched_static_edge} final_v={final_v:.2f} '
            f'reason={reason}'
        )

    def apply_speed_constraints(self, base_speed, crossing_state=None):
        """Lightweight velocity arbitration / speed constraint manager (Autoware-inspired).
        Applies safety and behavioral speed caps on top of the base speed determined by
        lane / SURFACE / DARK perception.
        Steering target is never modified by this method.

        Architecture extension point:
        Future traffic light / sign constraints can be chained here:
          - red light: return 0.0 (speed limit 0)
          - yellow light: conditional deceleration / stop
          - green light: return base_speed (no stop constraint)
        without modifying lane detection or PD steering controllers.
        """
        state = crossing_state if crossing_state is not None else self.crossing_state

        if state in ('STOP', 'WAIT'):
            return 0.0
        # Phase 3 & 8: Normal lane speed is maintained in APPROACH and PASS when road is clear
        return base_speed

    def log_forensic_telemetry(self, now_sec, scene, perception_mode, target_x, confidence, error, speed, steering, front, obs):
        import json, os, cv2, numpy as np
        is_problem_interval = (
            getattr(self, 'post_tunnel_trace_active', False) or
            getattr(self, 'post_tunnel_normal_ready', False) or
            getattr(self, 'post_tunnel_curve_boost_active', False) or
            getattr(self, 'post_tunnel_curve_active', False) or
            getattr(self, 'post_tunnel_guard_active', False) or
            getattr(self, 'post_curve_reacquire_active', False) or
            getattr(self, 'post_reacquire_protect_active', False) or
            (getattr(self, 'post_tunnel_assist_armed', False) and scene == 'NORMAL')
        )
        last_t = getattr(self, '_last_forensic_diag_time', 0.0)
        if not is_problem_interval and (now_sec - last_t < 0.20):
            return
        self._last_forensic_diag_time = now_sec

        image_center_x = self.image.shape[1] / 2.0 if self.image is not None else 320.0
        target_err_px = (target_x - image_center_x) if target_x is not None else 0.0

        cur_x = getattr(self, 'x', 0.0)
        cur_y = getattr(self, 'y', 0.0)
        cur_yaw = getattr(self, 'yaw', 0.0)
        prev_t = getattr(self, '_prev_diag_odom_t', None)
        prev_yaw = getattr(self, '_prev_diag_odom_yaw', None)
        dt = (now_sec - prev_t) if (prev_t is not None and (now_sec - prev_t) > 1e-4) else 0.05
        if prev_yaw is not None:
            dyaw = cur_yaw - prev_yaw
            while dyaw > 3.141592653589793:
                dyaw -= 2.0 * 3.141592653589793
            while dyaw < -3.141592653589793:
                dyaw += 2.0 * 3.141592653589793
            yaw_rate = dyaw / dt
        else:
            dyaw = 0.0
            yaw_rate = 0.0
        self._prev_diag_odom_t = now_sec
        self._prev_diag_odom_yaw = cur_yaw

        curv_actual = (yaw_rate / speed) if (speed is not None and abs(speed) > 0.01) else 0.0

        apex_diag = getattr(self, '_last_apex_guard_diag', None) or {}

        far_valid = getattr(self, '_diag_far_valid', False)
        curve_signal = getattr(self, '_diag_curve_signal', 0.0)
        has_valid_boost = getattr(self, '_diag_has_valid_boost_signal', False)
        detailed_far_reason = getattr(self, '_diag_far_reason', 'NONE')
        if not far_valid:
            far_reason = f"FAR_INVALID({detailed_far_reason})"
        elif abs(curve_signal) < self.normal_curve_min_signal_px:
            far_reason = f"SIGNAL_BELOW_MIN({curve_signal:+.1f}px)"
        elif not has_valid_boost:
            far_reason = f"BOOST_SIGNAL_REJECTED({detailed_far_reason})"
        else:
            far_reason = "VALID"

        # --- Visual Curve-Completion Multi-Row Analysis ---
        vis_near_cx = None
        vis_mid_cx = None
        vis_far_cx = None
        vis_bend_mid_near = None
        vis_bend_far_mid = None
        road_still_visually_curving = False

        if self.image is not None and HAVE_CV and is_problem_interval:
            try:
                gray_all = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
                _, thresh_all = cv2.threshold(gray_all, self.white_threshold, 255, cv2.THRESH_BINARY)
                
                # Near band: rows 380..440
                near_pix = np.where(thresh_all[380:440, :] > 0)
                if len(near_pix[1]) > 10:
                    vis_near_cx = float(np.mean(near_pix[1]))
                    
                # Mid band: rows 280..340
                mid_pix = np.where(thresh_all[280:340, :] > 0)
                if len(mid_pix[1]) > 10:
                    vis_mid_cx = float(np.mean(mid_pix[1]))

                # Far band: rows 200..260
                far_pix = np.where(thresh_all[200:260, :] > 0)
                if len(far_pix[1]) > 10:
                    vis_far_cx = float(np.mean(far_pix[1]))

                if vis_near_cx is not None and vis_mid_cx is not None:
                    vis_bend_mid_near = round(vis_mid_cx - vis_near_cx, 1)
                if vis_mid_cx is not None and vis_far_cx is not None:
                    vis_bend_far_mid = round(vis_far_cx - vis_mid_cx, 1)

                # Road is visually curving left if mid band is displaced left relative to near band,
                # or if far band continues to lead leftwards
                if vis_bend_mid_near is not None and vis_bend_mid_near < -12.0:
                    road_still_visually_curving = True
                elif vis_bend_far_mid is not None and vis_bend_far_mid < -10.0:
                    road_still_visually_curving = True
            except Exception:
                pass

        # Periodic frame snapshot saving for visual ground-truth
        frame_seq = getattr(self, '_diag_frame_seq', 0) + 1
        self._diag_frame_seq = frame_seq
        if is_problem_interval and (frame_seq % 5 == 0) and self.image is not None:
            try:
                frame_dir = '/ws/scripts/reports/curve_frames'
                os.makedirs(frame_dir, exist_ok=True)
                cv2.imwrite(f"{frame_dir}/frame_{frame_seq:04d}_{now_sec:.2f}.png", self.image)
            except Exception:
                pass

        entry = {
            't': round(now_sec, 3),
            'frame_seq': frame_seq,
            'scene': str(scene),
            'state': str(getattr(self, 'scene_state', 'NORMAL')),
            'armed': bool(getattr(self, 'post_tunnel_assist_armed', False)),
            'normal_ready': bool(getattr(self, 'post_tunnel_normal_ready', False)),
            'curve_active': bool(getattr(self, 'post_tunnel_curve_active', False)),
            'guard_active': bool(getattr(self, 'post_tunnel_guard_active', False)),
            'boost_active': bool(getattr(self, 'post_tunnel_curve_boost_active', False)),
            'reacquire_active': bool(getattr(self, 'post_curve_reacquire_active', False)),
            'assist_consumed': bool(getattr(self, 'post_tunnel_assist_consumed', False)),
            'dropout_frames': getattr(self, '_diag_dropout_frames', 0),
            'curve_exit_stable_cnt': getattr(self, 'post_tunnel_curve_exit_stable_frames', 0),
            'reacq_stable_cnt': getattr(self, 'post_curve_reacquire_stable_count', 0),
            'post_reacquire_protect': bool(getattr(self, 'post_reacquire_protect_active', False)),
            'post_reacquire_ref': round(getattr(self, 'post_reacquire_reference_target', 0.0), 1) if getattr(self, 'post_reacquire_reference_target', None) is not None else None,
            'post_reacquire_protected': round(getattr(self, 'post_reacquire_protected_target', 0.0), 1) if getattr(self, 'post_reacquire_protected_target', None) is not None else None,
            'post_reacquire_dual_streak': getattr(self, 'post_reacquire_dual_streak', 0),
            'post_reacquire_single_streak': getattr(self, 'post_reacquire_single_streak', 0),
            'post_reacquire_trusted_w': round(getattr(self, 'post_reacquire_trusted_width', 0.0), 1) if getattr(self, 'post_reacquire_trusted_width', None) is not None else None,
            
            'near_lx': round(getattr(self, '_diag_near_lx', 0.0), 1) if getattr(self, '_diag_near_lx', None) is not None else None,
            'near_rx': round(getattr(self, '_diag_near_rx', 0.0), 1) if getattr(self, '_diag_near_rx', None) is not None else None,
            'near_cx': round(getattr(self, '_diag_near_cx', 0.0), 1) if getattr(self, '_diag_near_cx', None) is not None else None,
            'near_w': round(getattr(self, '_diag_near_w', 0.0), 1) if getattr(self, '_diag_near_w', None) is not None else None,
            'near_tx': round(getattr(self, '_diag_near_tx', 0.0), 1) if getattr(self, '_diag_near_tx', None) is not None else None,
            'near_conf': round(getattr(self, '_diag_near_conf', 0.0), 2),
            'boundaries': getattr(self, '_diag_boundaries', 'NONE'),
            'target_source': getattr(self, '_diag_target_source', 'OTHER'),
            'error_px': round(target_err_px, 1),

            'trusted_near_l': round(getattr(self, 'last_trusted_near_left', 0.0), 1) if getattr(self, 'last_trusted_near_left', None) is not None else None,
            'trusted_near_r': round(getattr(self, 'last_trusted_near_right', 0.0), 1) if getattr(self, 'last_trusted_near_right', None) is not None else None,
            'trusted_near_tx': round(getattr(self, 'last_trusted_near_target', 0.0), 1) if getattr(self, 'last_trusted_near_target', None) is not None else None,
            'trusted_near_w': round(getattr(self, 'last_trusted_near_width', 0.0), 1) if getattr(self, 'last_trusted_near_width', None) is not None else None,
            'trusted_w_fsm': round(getattr(self, 'trusted_post_tunnel_lane_width', 350.0), 1),
            'trusted_updated': getattr(self, '_diag_trusted_updated', False),
            'trusted_update_reason': getattr(self, '_diag_trusted_update_reason', 'NONE'),

            'far_valid': far_valid,
            'far_lx': round(getattr(self, '_diag_far_lx', 0.0), 1) if getattr(self, '_diag_far_lx', None) is not None else None,
            'far_rx': round(getattr(self, '_diag_far_rx', 0.0), 1) if getattr(self, '_diag_far_rx', None) is not None else None,
            'far_cx': round(getattr(self, '_diag_far_cx', 0.0), 1) if getattr(self, '_diag_far_cx', None) is not None else None,
            'far_w': round(getattr(self, '_diag_far_w', 0.0), 1) if getattr(self, '_diag_far_w', None) is not None else None,
            'far_conf': round(getattr(self, '_diag_far_conf', 0.0), 2),
            'curve_signal': round(curve_signal, 1),
            'curve_dir': getattr(self, '_diag_same_lane_dir', 0),
            'same_lane': bool(getattr(self, '_diag_same_lane', False)),
            'same_lane_reason': str(getattr(self, '_diag_same_lane_reason', 'NONE')),
            'far_reason': far_reason,
            'detailed_far_reason': detailed_far_reason,
            'has_valid_boost': has_valid_boost,
            'raw_w_curve': round(getattr(self, '_diag_raw_w_curve', 0.0) or 0.0, 4),

            'w_baseline': round(getattr(self, '_diag_w_baseline', 0.0), 4),
            'w_boost_raw': round(getattr(self, '_diag_w_boost_raw', 0.0), 4),
            'boost_scale': round(getattr(self, '_diag_boost_scale', 1.0), 3),
            'w_boost_eff': round(getattr(self, '_diag_w_boost_eff', 0.0), 4),
            'w_cmd': round(getattr(self, '_diag_w_cmd', 0.0), 4),
            'w_final': round(steering, 4),
            'v_final': round(speed, 3),

            'apex_guard_triggered': bool(apex_diag.get('triggered', False)),
            'apex_quarantine_active': bool(apex_diag.get('quarantine_active', False)),
            'apex_recovery_streak': apex_diag.get('recovery_streak', 0),
            'apex_guard_hold_count': apex_diag.get('hold_count', 0),
            'apex_guard_cand_tx': round(apex_diag.get('cand_target', 0.0), 1) if apex_diag.get('cand_target') is not None else None,
            'apex_guard_cand_w': round(apex_diag.get('cand_width', 0.0), 1) if apex_diag.get('cand_width') is not None else None,
            'apex_guard_jump': round(apex_diag.get('target_jump', 0.0), 1) if apex_diag.get('target_jump') is not None else None,

            'odom_x': round(cur_x, 4),
            'odom_y': round(cur_y, 4),
            'odom_yaw': round(cur_yaw, 4),
            'delta_yaw': round(dyaw, 4),
            'yaw_rate': round(yaw_rate, 4),
            'curv_actual': round(curv_actual, 4),

            'vis_near_cx': vis_near_cx,
            'vis_mid_cx': vis_mid_cx,
            'vis_far_cx': vis_far_cx,
            'vis_bend_mid_near': vis_bend_mid_near,
            'vis_bend_far_mid': vis_bend_far_mid,
            'road_still_visually_curving': road_still_visually_curving
        }

        log_path = '/ws/scripts/reports/forensic_telemetry.jsonl'
        try:
            with open(log_path, 'a') as f:
                f.write(json.dumps(entry) + chr(10))
        except Exception:
            pass

        # ---- DIAGNOSTIC TRACE: [POST_TUNNEL_REF] and [POST_TUNNEL_TRACE] ----
        if getattr(self, 'post_tunnel_trace_active', False):
            import math as _dmath

            # Emit [POST_TUNNEL_REF] exactly once at start of trace
            if not getattr(self, '_post_tunnel_ref_logged', False):
                self._post_tunnel_ref_logged = True
                _rx0   = getattr(self, 'post_tunnel_ref_x',   0.0) or 0.0
                _ry0   = getattr(self, 'post_tunnel_ref_y',   0.0) or 0.0
                _ryaw0 = getattr(self, 'post_tunnel_ref_yaw', 0.0) or 0.0
                _fsm0  = str(getattr(self, 'scene_state', 'UNKNOWN'))
                self.get_logger().info(
                    '[POST_TUNNEL_REF] '
                    f't={now_sec:.3f} '
                    f'x={_rx0:.4f} y={_ry0:.4f} yaw={_ryaw0:.4f} '
                    f'fsm={_fsm0}'
                )

            # Compute route-relative metrics
            _cx   = cur_x
            _cy   = cur_y
            _cyaw = cur_yaw
            _rx0  = getattr(self, 'post_tunnel_ref_x',   0.0) or 0.0
            _ry0  = getattr(self, 'post_tunnel_ref_y',   0.0) or 0.0
            _ryaw0= getattr(self, 'post_tunnel_ref_yaw', 0.0) or 0.0
            _dist = _dmath.hypot(_cx - _rx0, _cy - _ry0)
            _raw_ryw = _cyaw - _ryaw0
            while _raw_ryw >  _dmath.pi: _raw_ryw -= 2.0 * _dmath.pi
            while _raw_ryw < -_dmath.pi: _raw_ryw += 2.0 * _dmath.pi
            _rel_yaw = _raw_ryw

            _fseq       = getattr(self, '_ctrl_frame_seq', 0)
            _w_pd       = round(entry.get('w_baseline', 0.0), 4)
            _w_braw     = round(entry.get('w_boost_raw', 0.0), 4)
            _bscale     = round(entry.get('boost_scale', 1.0), 3)
            _w_beff     = round(entry.get('w_boost_eff', 0.0), 4)
            _w_cmd      = round(entry.get('w_cmd', _w_pd), 4)
            _w_final    = round(entry.get('w_final', 0.0), 4)
            _v_cmd      = round(entry.get('v_final', 0.0), 4)
            _yaw_rate   = round(entry.get('yaw_rate', 0.0), 4)
            _far_conf   = round(entry.get('far_conf', 0.0), 2)
            _boost_on   = int(bool(getattr(self, 'post_tunnel_curve_boost_active', False)))
            _exit_cnt   = getattr(self, 'post_tunnel_curve_exit_stable_frames', 0)
            _dropout    = getattr(self, 'post_tunnel_curve_dropout_frames', 0)
            _csig       = round(entry.get('curve_signal', 0.0), 1)
            _cont       = round(getattr(self, 'curve_containment_scale', 1.0), 3)
            _c1         = getattr(self, '_diag_c1', 0)
            _c2         = getattr(self, '_diag_c2', 0)
            _c3         = getattr(self, '_diag_c3', 0)
            _c4         = getattr(self, '_diag_c4', 0)
            _c5         = getattr(self, '_diag_c5', 0)
            _c6         = getattr(self, '_diag_c6', 0)
            _c7         = getattr(self, '_diag_c7', 0)
            _atrig      = int(bool(entry.get('apex_guard_triggered', False)))
            _aquar      = int(bool(entry.get('apex_quarantine_active', False)))

            self.get_logger().info(
                '[POST_TUNNEL_TRACE] '
                f'frame={_fseq} t={now_sec:.3f} '
                f'state={str(getattr(self, "scene_state", "?"))} '
                f'x={_cx:.4f} y={_cy:.4f} yaw={_cyaw:.4f} '
                f'dist={_dist:.3f} rel_yaw={_rel_yaw:.4f} '
                f'mode={perception_mode} '
                f'tgt={round(target_x,1) if target_x is not None else None} '
                f'err={round(error,4)} '
                f'boost={_boost_on} sig={_csig} far_conf={_far_conf} drop={_dropout} '
                f'exit_cnt={_exit_cnt} c1={_c1} c2={_c2} c3={_c3} c4={_c4} c5={_c5} c6={_c6} c7={_c7} '
                f'w_pd={_w_pd} w_braw={_w_braw} cont={_cont} '
                f'w_beff={_w_beff} w_cmd={_w_cmd} w_fin={_w_final} '
                f'v_cmd={_v_cmd} yaw_rate={_yaw_rate} '
                f'apex={_atrig} quar={_aquar}'
            )

            # Auto-deactivate when robot passes the entire evaluation section
            if _dist > 25.0:
                self.post_tunnel_trace_active = False
                self.get_logger().info('[POST_TUNNEL_TRACE_END] Section complete (dist > 25m)')
        # -------------------------------------------------------------------

    def control(self):
        """Called at rate Hz (default 20 Hz).

        Perception Priority:
          1. LiDAR front obstacle safety -> stop if blocked.
          2. Camera image availability -> stop safely if image is None.
          3. Scene detection (NORMAL vs DARK) via forward-looking ROI [0.25..0.70] + darkness hysteresis.
          4. If scene == DARK:
             - Run primary DARK_LANE multi-row perception.
             - If valid: perception=DARK_LANE, conservative speed 0.04 m/s (0.03 in curves).
               V1 confidence=1.00 CANNOT override DARK_LANE.
             - If invalid: optional fallback to existing V1 lane perception.
             - If neither valid: stop safely (LOW_CONFIDENCE).
          5. If scene == NORMAL:
             - Run existing V1 lane perception unchanged.
             - If V1 confidence < min_confidence: run existing SURFACE fallback unchanged.
             - If neither valid: stop safely (LOW_CONFIDENCE).
          6. Compute normalized lateral error: (target_x - center_x) / center_x.
          7. Shared PD steering controller: w = -(kp * error + kd * derivative).
          8. Command self.drive(v, w).
        """
        # --- DIAGNOSTIC: increment per-control-cycle frame counter ---
        self._ctrl_frame_seq = getattr(self, '_ctrl_frame_seq', 0) + 1
        # -----------------------------------------------------------------
        # 1. LiDAR front collision safety guard and crossing occupancy
        crossing_occ = None
        obs = None
        matched_static_edge = False
        generic_x = None
        generic_y = None

        if self.scan is not None:
            obs = self.detect_front_obstacles()
            front = obs['front_distance']
            is_dark_now = getattr(self, 'scene_state', 'NORMAL') in ('DARK_PENDING', 'DARK_ACTIVE', 'DARK')
            crossing_occ = None if is_dark_now else self.detect_crossing_occupancy(self.scan)
            if obs['stop']:
                matched_static_edge, generic_x, generic_y = (
                    (False, None, None) if is_dark_now else self.correlate_obstacle_with_crossing(obs, crossing_occ)
                )
                if not matched_static_edge:
                    # True front obstacle: maintain safety stop
                    self.perception_mode = 'NONE'
                    self.stop()
                    self.log_stop('FRONT_OBSTACLE', front=front, cluster=obs['cluster'])
                    self.log_speed_arbitration(
                        state=self.crossing_state, base_v=0.0,
                        crossing_class=crossing_occ.get('cluster_class', 'NONE') if crossing_occ else 'NONE',
                        crossing_scan_seq=getattr(self, '_crossing_processed_scan_seq', -1),
                        generic_obstacle=True, generic_x=generic_x, generic_y=generic_y,
                        matched_static_edge=False, final_v=0.0, reason='TRUE_FRONT_OBSTACLE'
                    )
                    return
        else:
            front = self.range_at(0, width_deg=30)
            if front < self.stop_distance:
                self.perception_mode = 'NONE'
                self.stop()
                self.log_stop('FRONT_OBSTACLE', front=front)
                self.log_speed_arbitration(
                    state=self.crossing_state, base_v=0.0,
                    crossing_class='NONE',
                    crossing_scan_seq=-1,
                    generic_obstacle=True, generic_x=None, generic_y=None,
                    matched_static_edge=False, final_v=0.0, reason='TRUE_FRONT_OBSTACLE'
                )
                return

        # 2. Camera image availability guard
        if self.image is None:
            self.perception_mode = 'NONE'
            self.stop()
            self.log_stop('NO_IMAGE', front=front)
            return

        image_center_x = self.image.shape[1] / 2.0

        # Scene classification via forward-looking camera ROI + darkness hysteresis
        scene, s_mean, s_median, s_p25, dark_ratio = self.update_scene_state(self.image)
        is_dark_scene = (scene in ('DARK_PENDING', 'DARK_ACTIVE', 'DARK') or
                         self.scene_state in ('DARK_PENDING', 'DARK_ACTIVE', 'DARK'))

        if is_dark_scene:
            zebra_detected = False
            zebra_bands = 0
            zebra_score = 0.0
            zebra_viz = None
            self.last_zebra_detected = False
            self.last_zebra_bands = 0
            self.last_zebra_score = 0.0
            self._cached_zebra_result = (False, 0, 0.0, None)
            if self.crossing_state != 'IDLE':
                self.neutralize_crossing_context('DARK_SCENE_GUARD')
            crossing_active = False
            is_occ = False
            fsm_crossing_occ = None
            self.reset_crossing_tracking()
            self._cached_crossing_occ = None
            self.crossing_occupied = False
        else:
            # 2b. Visual Zebra Detection & Crossing Corridor Occupancy
            is_new_image = (self._image_seq != self._crossing_processed_image_seq or
                            (self.image is not None and self.image is not self._last_crossing_image))
            if is_new_image:
                zebra_detected, zebra_bands, zebra_score, zebra_viz = self.detect_zebra_crossing(self.image)
                self.last_zebra_detected = zebra_detected
                self.last_zebra_bands = zebra_bands
                self.last_zebra_score = zebra_score
                self._cached_zebra_result = (zebra_detected, zebra_bands, zebra_score, zebra_viz)
                self._crossing_processed_image_seq = self._image_seq
                self._last_crossing_image = self.image
            else:
                zebra_detected, zebra_bands, zebra_score, zebra_viz = self._cached_zebra_result

            crossing_active = (self.crossing_state != 'IDLE' or zebra_detected or self.zebra_detect_count > 0)
            if crossing_active:
                is_occ = crossing_occ['occupied'] if crossing_occ is not None else False
                fsm_crossing_occ = crossing_occ
            else:
                is_occ = False
                fsm_crossing_occ = None
                self.reset_crossing_tracking()
                self._cached_crossing_occ = None
            self.crossing_occupied = is_occ

            is_new_scan = crossing_occ.get('is_new_scan', True) if crossing_occ is not None else False
            self.update_crossing_state(zebra_detected, is_occ, fsm_crossing_occ,
                                       is_new_scan=is_new_scan, is_new_image=is_new_image)

            # 2c. Pedestrian Crossing Stop Gate
            road_occupied = (self.crossing_state in ('STOP', 'WAIT') or (self.crossing_detect_count >= self.crossing_confirm_scans))
            if crossing_active and (self.crossing_state in ('STOP', 'WAIT') or road_occupied):
                self.perception_mode = 'CROSSING_STOP'
                self.stop()
                self.log_crossing_wait(is_occ, self.crossing_clear_count, self.crossing_clear_scans, front=front)
                self.log_speed_arbitration(
                    state=self.crossing_state, base_v=0.0,
                    crossing_class=crossing_occ.get('cluster_class', 'NONE') if crossing_occ else 'NONE',
                    crossing_scan_seq=getattr(self, '_crossing_processed_scan_seq', -1),
                    generic_obstacle=(obs is not None and obs.get('stop', False)),
                    generic_x=generic_x, generic_y=generic_y,
                    matched_static_edge=matched_static_edge, final_v=0.0, reason='CROSSING_STOP'
                )
                if self.debug_viz and self.bridge is not None:
                    lx, rx, v1_tx, v1_conf, v1_roi, v1_clean = self.find_lane_target(self.image)
                    self._publish_debug_viz(
                        v1_roi, v1_clean, lx, rx, v1_tx, image_center_x,
                        perception_mode='CROSSING_WAIT', conf=v1_conf,
                        crossing_state=self.crossing_state, crossing_occupied=is_occ,
                        zebra_detected=zebra_detected, zebra_bands=zebra_bands
                    )
                return

        # 3. Main perception dispatch (scene already determined above)

        perception_mode = 'NONE'
        target_x = None
        speed = 0.0
        confidence = 0.0
        coverage = 0.0
        valid_rows = 0
        left_x = right_x = None
        near_center = far_center = None
        dark_raw_target = None
        dark_valid_rows = []
        skipped_transverse = []
        roi = clean = None
        surface_clean = None
        row_endpoints = None
        recovery_w = 0.0

        if scene == 'DARK_PENDING':
            self.dark_pending_count += 1
            now_sec = self._get_current_time_sec()

            if self.dark_pending_count > self.dark_pending_max_frames:
                if not self.dark_pending_grace_active:
                    # Check eligibility for bounded grace at deadline
                    is_eligible = (
                        self.dark_ready_count == (self.dark_ready_frames - 1) and
                        self.dark_pending_last_ready_frame == (self.dark_pending_count - 1) and
                        self.dark_pending_last_ready_stamp is not None and
                        (now_sec - self.dark_pending_last_ready_stamp) <= MAX_PENDING_READY_STALE_SEC
                    )
                    if is_eligible:
                        self.dark_pending_grace_active = True
                        self.dark_pending_grace_start_time = now_sec
                        self.get_logger().info(
                            f'DARK_PENDING_GRACE_START age=0.00 '
                            f'ready_rows={getattr(self, "dark_pending_last_ready_rows", 0)} '
                            f'ready_count={self.dark_ready_count}/{self.dark_ready_frames} '
                            f'grace_limit={self.dark_pending_grace_sec:.2f}'
                        )
                    else:
                        self.perception_mode = 'NONE'
                        self.stop()
                        self.log_stop('DARK_NOT_READY', front=front, conf=0.0,
                                      perception='DARK_PENDING', valid_rows=0,
                                      brightness=s_median, dark_ratio=dark_ratio)
                        self._reset_dark_pending_grace()
                        return

                if self.dark_pending_grace_active:
                    grace_age = now_sec - self.dark_pending_grace_start_time
                    if grace_age > self.dark_pending_grace_sec:
                        self.get_logger().info(
                            f'DARK_PENDING_GRACE_EXPIRED age={grace_age:.2f} '
                            f'ready_rows={getattr(self, "dark_pending_last_ready_rows", 0)} '
                            f'ready_count={self.dark_ready_count}/{self.dark_ready_frames}'
                        )
                        self.perception_mode = 'NONE'
                        self.stop()
                        self.log_stop('DARK_NOT_READY', front=front, conf=0.0,
                                      perception='DARK_PENDING', valid_rows=0,
                                      brightness=s_median, dark_ratio=dark_ratio)
                        self._reset_dark_pending_grace()
                        return

            (
                d_target_x, d_raw_target_x, d_near_c, d_far_c,
                d_valid_rows, d_skipped, d_roi, d_clean
            ) = self.find_dark_lane_target(self.image)

            roi = d_roi
            clean = d_clean
            dark_valid_rows = d_valid_rows
            skipped_transverse = d_skipped
            valid_rows = len(d_valid_rows)
            near_center = d_near_c
            far_center = d_far_c
            dark_raw_target = d_raw_target_x
            self.dark_raw_target = d_raw_target_x

            # Readiness evaluation:
            # Require at least dark_ready_min_rows (2) accepted rows.
            # Prefer readiness where at least one accepted row is DUAL, if DUAL information exists.
            has_dual_candidates = any(getattr(r, 'boundary_type', '') == 'DUAL' for r in getattr(self, 'last_dark_rows', []))
            dual_accepted = sum(1 for r in d_valid_rows if getattr(r, 'boundary_type', '') == 'DUAL')

            if has_dual_candidates:
                candidate_ready = (valid_rows >= self.dark_ready_min_rows and d_target_x is not None and dual_accepted >= 1)
            else:
                candidate_ready = (valid_rows >= self.dark_ready_min_rows and d_target_x is not None)

            if candidate_ready:
                self.dark_ready_count += 1
                self.dark_pending_last_ready_stamp = now_sec
                self.dark_pending_last_ready_frame = self.dark_pending_count
                self.dark_pending_last_ready_rows = valid_rows
            else:
                self.dark_ready_count = 0

            cand_t_str = f'{d_target_x:.1f}' if d_target_x is not None else 'None'
            if self.dark_pending_grace_active:
                grace_age = now_sec - self.dark_pending_grace_start_time
                latest_valid_str = 'true' if candidate_ready else 'false'
                self.get_logger().info(
                    f'DARK_PENDING_GRACE age={grace_age:.2f} '
                    f'ready_rows={valid_rows} '
                    f'ready_count={self.dark_ready_count}/{self.dark_ready_frames} '
                    f'latest_valid={latest_valid_str}'
                )
            else:
                anchor_val = self.last_stable_normal_target_x if self.last_stable_normal_target_x is not None else 320.0
                self.get_logger().info(
                    f'DARK_PENDING frame={self.dark_pending_count}/{self.dark_pending_max_frames} '
                    f'ready_rows={valid_rows} ready_count={self.dark_ready_count}/{self.dark_ready_frames} '
                    f'anchor_target={anchor_val:.1f}'
                )

            if self.dark_ready_count >= self.dark_ready_frames:
                if self.dark_pending_grace_active:
                    grace_age = now_sec - self.dark_pending_grace_start_time
                    self.get_logger().info(
                        f'DARK_PENDING_GRACE_CONFIRMED age={grace_age:.2f} '
                        f'ready_count={self.dark_ready_count}/{self.dark_ready_frames}'
                    )
                    self._reset_dark_pending_grace()
                # Transition PENDING -> ACTIVE
                self.scene_state = SceneState('DARK_ACTIVE')
                self.neutralize_crossing_context('DARK_ACTIVE_CONFIRMED')
                perception_mode = 'DARK_LANE'
                target_x = d_target_x
                if any(r.boundary_type == 'DUAL' for r in dark_valid_rows):
                    self.last_valid_dark_target_x = target_x
                    self.last_good_dark_target = target_x
                self.last_good_dark_stamp = now_sec
                if getattr(self, 'has_odom', False):
                    self.last_good_dark_odom_x = self.x
                    self.last_good_dark_odom_y = self.y
                self.dark_recovery_active = False
                self.dark_hold_count = 0
                confidence = float(valid_rows) / float(self.dark_scan_rows)
                abs_error = abs((target_x - image_center_x) / image_center_x)
                speed = self.dark_lane_speed if abs_error < 0.15 else self.corner_speed
                self.get_logger().info(
                    f'DARK_TRANSITION PENDING->ACTIVE rows={valid_rows} target={target_x:.1f}'
                )
            else:
                # Hold anchor target at pending crawling speed
                perception_mode = 'DARK_PENDING'
                target_x = self.last_stable_normal_target_x
                confidence = 0.6
                speed = self.dark_pending_speed  # 0.02 m/s

        elif scene in ('DARK_ACTIVE', 'DARK'):
            if self.dark_entry_stabilize_count < self.dark_entry_stabilize_frames:
                self.dark_entry_stabilize_count += 1

            (
                d_target_x, d_raw_target_x, d_near_c, d_far_c,
                d_valid_rows, d_skipped, d_roi, d_clean
            ) = self.find_dark_lane_target(self.image)

            roi = d_roi
            clean = d_clean
            dark_valid_rows = d_valid_rows
            skipped_transverse = d_skipped
            valid_rows = len(d_valid_rows)
            near_center = d_near_c
            far_center = d_far_c
            dark_raw_target = d_raw_target_x
            now_sec = self._get_current_time_sec()

            if d_target_x is not None:
                if self.dark_recovery_active:
                    reacquire_age = (now_sec - self.last_good_dark_stamp) if self.last_good_dark_stamp is not None else 0.0
                    self.get_logger().info(
                        f'DARK_REACQUIRED age={reacquire_age:.3f} rows={valid_rows} target={d_target_x:.1f}'
                    )
                    self.dark_recovery_active = False
                self.dark_recovery_expired_latched = False

                perception_mode = 'DARK_LANE'
                target_x = d_target_x
                self.last_valid_dark_target_x = target_x
                self.last_good_dark_target = target_x
                self.last_good_dark_stamp = now_sec
                if getattr(self, 'has_odom', False):
                    self.last_good_dark_odom_x = self.x
                    self.last_good_dark_odom_y = self.y
                self.dark_hold_count = 0
                confidence = float(valid_rows) / float(self.dark_scan_rows)
                abs_error = abs((target_x - image_center_x) / image_center_x)
                speed = self.dark_lane_speed if abs_error < 0.15 else self.corner_speed
            else:
                has_trusted = (self.last_good_dark_target is not None or self.last_valid_dark_target_x is not None)
                if has_trusted and self.last_good_dark_target is None:
                    self.last_good_dark_target = self.last_valid_dark_target_x
                if has_trusted and self.last_good_dark_stamp is None:
                    self.last_good_dark_stamp = now_sec

                if not has_trusted:
                    self.perception_mode = 'NONE'
                    self.stop()
                    self.log_stop('LOW_CONFIDENCE', front=front, conf=0.0,
                                  perception='DARK_LANE', valid_rows=valid_rows,
                                  brightness=s_median)
                    return

                age = max(0.0, now_sec - self.last_good_dark_stamp)
                has_odom_pos = (
                    getattr(self, 'has_odom', False) and
                    self.last_good_dark_odom_x is not None and
                    self.last_good_dark_odom_y is not None
                )
                dist = (
                    math.hypot(self.x - self.last_good_dark_odom_x, self.y - self.last_good_dark_odom_y)
                    if has_odom_pos else 0.0
                )

                is_expired = False
                expire_reason = ''
                if age > self.max_dark_recovery_time:
                    is_expired = True
                    expire_reason = 'TIME_EXPIRED'
                elif has_odom_pos and dist > self.max_dark_recovery_dist:
                    is_expired = True
                    expire_reason = 'DISTANCE_EXPIRED'

                if is_expired:
                    self.dark_recovery_active = False
                    if not getattr(self, 'dark_recovery_expired_latched', False):
                        self.dark_recovery_expired_latched = True
                        dist_str = f' distance={dist:.3f}' if has_odom_pos else ''
                        self.get_logger().info(
                            f'DARK_RECOVERY_EXPIRED age={age:.3f}{dist_str} reason={expire_reason}'
                        )
                    self.perception_mode = 'NONE'
                    self.stop()
                    self.log_stop('LOW_CONFIDENCE', front=front, conf=0.0,
                                  perception='DARK_LANE', valid_rows=valid_rows,
                                  brightness=s_median)
                    return

                self.dark_recovery_active = True
                self.dark_hold_count += 1
                perception_mode = 'DARK_HOLD'
                target_x = self.last_good_dark_target
                confidence = 0.5
                speed = self.dark_recovery_speed

                decay = max(0.1, 1.0 - 0.9 * (age / self.max_dark_recovery_time))
                recovery_w = self.last_good_dark_w * decay
                recovery_w = float(max(-self.max_dark_recovery_w, min(self.max_dark_recovery_w, recovery_w)))

                pitch = getattr(self, 'pitch_deg', 0.0)
                reason_str = 'DROPOUT' if valid_rows == 0 else f'LOW_ROWS_{valid_rows}'
                self.get_logger().info(
                    f'DARK_RECOVERY age={age:.3f} rows={valid_rows} last_target={target_x:.1f} '
                    f'v={speed:.2f} w={recovery_w:+.2f} pitch={pitch:+.1f} reason={reason_str}'
                )
        else:
            # scene == 'NORMAL': Unchanged V1 Baseline + SURFACE Fallback
            lx, rx, v1_tx, v1_conf, v1_roi, v1_clean = self.find_lane_target(self.image)
            left_x = lx
            right_x = rx
            roi = v1_roi
            clean = v1_clean
            confidence = v1_conf

            # -----------------------------------------------------------------
            # Scoped Post-Tunnel Curve Assist State Machine
            # -----------------------------------------------------------------
            curve_signal = 0.0
            raw_w_curve = 0.0
            curve_speed = None

            if getattr(self, 'post_tunnel_assist_armed', False) and not getattr(self, 'post_tunnel_assist_consumed', False):
                # State 2 Check: Stable NORMAL Reacquisition
                if not getattr(self, 'post_tunnel_normal_ready', False):
                    is_stable_normal = (
                        v1_conf >= self.min_confidence and v1_tx is not None and
                        lx is not None and rx is not None and
                        (250.0 <= (rx - lx) <= 450.0) and
                        (abs(v1_tx - image_center_x) < 70.0) and
                        not getattr(self, 'dark_recovery_active', False) and
                        getattr(self, 'crossing_state', 'IDLE') == 'IDLE'
                    )
                    if is_stable_normal:
                        self.post_tunnel_ready_frames += 1
                        measured_w = float(rx - lx)
                        self.trusted_post_tunnel_lane_width = 0.9 * self.trusted_post_tunnel_lane_width + 0.1 * measured_w
                        if self.post_tunnel_ready_frames >= 10:
                            self.post_tunnel_normal_ready = True
                            self.get_logger().info('POST_TUNNEL_NORMAL_READY')
                    else:
                        self.post_tunnel_ready_frames = 0

                # State 3 / State 4: Advisory FAR lookahead and Hybrid Curve Boost
                if getattr(self, 'post_tunnel_normal_ready', False):
                    far_lx, far_rx, far_center, far_conf, far_width = self.find_normal_far_target(self.image, near_center=v1_tx)
                    now_sec = self._get_current_time_sec()

                    self._diag_far_lx = far_lx
                    self._diag_far_rx = far_rx
                    self._diag_far_cx = far_center
                    self._diag_far_conf = far_conf
                    self._diag_far_w = far_width
                    self._diag_far_valid = bool(far_center is not None and far_conf >= 0.7)

                    # FAR ROI is strictly advisory: never replaces near target
                    if v1_tx is not None and far_center is not None and far_conf >= 0.7:
                        curve_signal = float(far_center - v1_tx)
                        self.last_normal_curve_signal = curve_signal

                        is_same_lane, cand_dir, reason = self.check_same_lane_coherence(
                            lx, rx, v1_tx, v1_conf, far_lx, far_rx, far_center, far_conf, far_width
                        )
                        self._diag_curve_signal = curve_signal
                        self._diag_same_lane = is_same_lane
                        self._diag_same_lane_dir = cand_dir
                        self._diag_same_lane_reason = reason

                        if not self.post_tunnel_curve_boost_active and not getattr(self, 'post_curve_reacquire_active', False) and not getattr(self, 'post_tunnel_curve_boost_consumed', False):
                            if is_same_lane:
                                if self.post_tunnel_curve_candidate_count == 0:
                                    self.post_tunnel_curve_candidate_dir = cand_dir
                                    self.post_tunnel_curve_candidate_count = 1
                                elif cand_dir == self.post_tunnel_curve_candidate_dir:
                                    self.post_tunnel_curve_candidate_count += 1
                                else:
                                    self.post_tunnel_curve_candidate_dir = cand_dir
                                    self.post_tunnel_curve_candidate_count = 1

                                if now_sec - getattr(self, '_last_post_tunnel_candidate_log_time', 0.0) > 0.25:
                                    self._last_post_tunnel_candidate_log_time = now_sec
                                    dir_str = 'LEFT' if cand_dir == -1 else 'RIGHT'
                                    self.get_logger().info(
                                        f'POST_TUNNEL_CURVE_CANDIDATE dir={dir_str} same_lane=True '
                                        f'count={self.post_tunnel_curve_candidate_count}/{self.post_tunnel_boost_confirm_frames} '
                                        f'near={v1_tx:.1f} far={far_center:.1f} signal={curve_signal:+.1f}'
                                    )

                                if self.post_tunnel_curve_candidate_count >= self.post_tunnel_boost_confirm_frames:
                                    self.post_tunnel_curve_boost_active = True
                                    self.post_tunnel_curve_active = True
                                    self.post_tunnel_guard_active = True
                                    self.post_tunnel_curve_dir = self.post_tunnel_curve_candidate_dir
                                    self.post_tunnel_curve_dropout_frames = 0
                                    self.post_tunnel_curve_exit_stable_frames = 0
                                    dir_str = 'LEFT' if self.post_tunnel_curve_dir == -1 else 'RIGHT'
                                    self.get_logger().info(
                                        f'POST_TUNNEL_CURVE_BOOST_ON dir={dir_str} near={v1_tx:.1f} far={far_center:.1f} '
                                        f'signal={curve_signal:+.1f}'
                                    )
                            else:
                                self.post_tunnel_curve_candidate_count = 0
                    else:
                        if not self.post_tunnel_curve_boost_active:
                            self.post_tunnel_curve_candidate_count = 0

                    if self.post_tunnel_curve_boost_active:
                        has_valid_boost_signal = (
                            v1_tx is not None and far_center is not None and far_conf >= 0.7 and
                            abs(curve_signal) >= self.normal_curve_min_signal_px
                        )

                        if has_valid_boost_signal:
                            cur_dir = -1 if curve_signal < 0 else 1
                            if cur_dir == getattr(self, 'post_tunnel_curve_dir', 0):
                                self.post_tunnel_curve_dropout_frames = 0
                                curve_norm = curve_signal / image_center_x
                                # ROS convention: LEFT physical curve (curve_signal < 0) -> raw_w_curve > 0
                                raw_w_curve = - self.normal_curve_gain * curve_norm
                                raw_w_curve = float(max(-self.max_normal_curve_w, min(self.max_normal_curve_w, raw_w_curve)))

                                abs_csig = abs(curve_signal)
                                if abs_csig >= self.normal_curve_speed_strong_px:
                                    curve_speed = 0.04
                                elif abs_csig >= self.normal_curve_speed_mod_px:
                                    curve_speed = self.medium_speed
                            else:
                                raw_w_curve = None
                                self.post_tunnel_curve_dropout_frames += 1
                        else:
                            raw_w_curve = None
                            self.post_tunnel_curve_dropout_frames += 1

                        # Bounded smooth dropout handling
                        if raw_w_curve is not None and abs(raw_w_curve) > 1e-4:
                            w_curve = self.normal_curve_alpha * raw_w_curve + (1.0 - self.normal_curve_alpha) * self.prev_normal_w_curve
                            w_curve = float(max(-self.max_normal_curve_w, min(self.max_normal_curve_w, w_curve)))
                            self.prev_normal_w_curve = w_curve
                        else:
                            w_curve = 0.85 * self.prev_normal_w_curve
                            if getattr(self, 'post_tunnel_curve_dir', 0) == -1:
                                w_curve = max(0.0, w_curve)
                            elif getattr(self, 'post_tunnel_curve_dir', 0) == 1:
                                w_curve = min(0.0, w_curve)
                            if abs(w_curve) < 1e-4:
                                w_curve = 0.0
                            self.prev_normal_w_curve = w_curve

                            if now_sec - getattr(self, '_last_post_tunnel_hold_log_time', 0.0) > 1.0:
                                self._last_post_tunnel_hold_log_time = now_sec
                                self.get_logger().info(
                                    f'POST_TUNNEL_CURVE_BOOST_HOLD curve_signal={curve_signal:+.1f} '
                                    f'near_target={v1_tx:.1f} w_curve={w_curve:+.3f} dropout={self.post_tunnel_curve_dropout_frames}'
                                )

                        if self.post_tunnel_curve_dropout_frames > 15:
                            w_curve = 0.5 * w_curve
                            self.prev_normal_w_curve = w_curve
                        self._diag_has_valid_boost_signal = bool(has_valid_boost_signal)
                        self._diag_raw_w_curve = raw_w_curve
                        self._diag_dropout_frames = getattr(self, 'post_tunnel_curve_dropout_frames', 0)
                        self._diag_w_curve_decayed = w_curve

                        # Curve Exit Verification: All 7 cues must be simultaneously verified
                        c1_weak_curve = (abs(curve_signal) < self.normal_curve_min_signal_px)
                        c2_near_valid = (v1_tx is not None)
                        c3_conf_high = (v1_conf >= self.min_confidence)
                        c4_dual_boundaries = (lx is not None and rx is not None)
                        trusted_w = getattr(self, 'trusted_post_tunnel_lane_width', 350.0)
                        measured_w = float(rx - lx) if (lx is not None and rx is not None) else None
                        width_ratio = (measured_w / trusted_w) if (measured_w is not None and trusted_w > 0) else None
                        c5_plausible_width = (
                            measured_w is not None and (280.0 <= measured_w <= 420.0) and
                            (width_ratio is not None and 0.75 <= width_ratio <= 1.25)
                        )
                        c6_target_centered = (v1_tx is not None and abs(v1_tx - image_center_x) < 50.0)
                        cur_err = (v1_tx - image_center_x) / image_center_x if v1_tx is not None else 0.0
                        cur_deriv = cur_err - getattr(self, 'prev_error', 0.0)
                        cur_w_lat = -(self.kp * cur_err + self.kd * cur_deriv)
                        c7_w_lateral_small = (abs(cur_w_lat) < 0.08)
                        self._diag_c1 = int(c1_weak_curve)
                        self._diag_c2 = int(c2_near_valid)
                        self._diag_c3 = int(c3_conf_high)
                        self._diag_c4 = int(c4_dual_boundaries)
                        self._diag_c5 = int(c5_plausible_width)
                        self._diag_c6 = int(c6_target_centered)
                        self._diag_c7 = int(c7_w_lateral_small)

                        # PATH A: existing dual-boundary exit (100% unchanged)
                        exit_candidate_path_a = (
                            c1_weak_curve and c2_near_valid and c3_conf_high and
                            c4_dual_boundaries and c5_plausible_width and
                            c6_target_centered and c7_w_lateral_small
                        )

                        # PATH B: conservative single-boundary end-of-curve exit (rx temporarily missing)
                        # - rx is None and lx is not None (rx temporarily missing, lx valid/stable)
                        # - curve signal has weakened using existing criterion (c1_weak_curve)
                        # - lx remains valid/stable (c2_near_valid, c3_conf_high, c6_target_centered)
                        # - steering demand small / returning toward neutral (c7_w_lateral_small and w_curve decaying)
                        # - heading / relative-yaw evidence: robot is at end of curve
                        cur_yaw = getattr(self, 'yaw', 0.0)
                        _ryaw0 = getattr(self, 'post_tunnel_ref_yaw', None)
                        cur_rel_yaw = normalize_angle(cur_yaw - _ryaw0) if _ryaw0 is not None else 0.0
                        prev_yaw = getattr(self, '_prev_diag_odom_yaw', None)
                        dyaw = normalize_angle(cur_yaw - prev_yaw) if prev_yaw is not None else 0.0
                        curve_dir = getattr(self, 'post_tunnel_curve_dir', -1)
                        heading_end_of_curve = (
                            (_ryaw0 is None or (cur_rel_yaw * (-curve_dir) > 0.15) or abs(dyaw) < 0.04)
                        )
                        w_curve_small = (abs(w_curve) < 0.05)

                        exit_candidate_path_b = (
                            (rx is None and lx is not None) and
                            c1_weak_curve and
                            c2_near_valid and
                            c3_conf_high and
                            c6_target_centered and
                            c7_w_lateral_small and
                            w_curve_small and
                            heading_end_of_curve
                        )

                        is_curve_exit_candidate = exit_candidate_path_a or exit_candidate_path_b

                        if is_curve_exit_candidate:
                            self.post_tunnel_curve_exit_stable_frames += 1
                            if self.post_tunnel_curve_exit_stable_frames >= self.post_tunnel_curve_exit_frames:
                                if exit_candidate_path_b and not exit_candidate_path_a:
                                    self.get_logger().info(
                                        f'[RIGHT_CONT] SINGLE_EXIT_FALLBACK rel_yaw={cur_rel_yaw:.3f} '
                                        f'w_lat={cur_w_lat:+.3f} w_curve={w_curve:+.3f}'
                                    )
                                self.post_tunnel_curve_boost_active = False
                                self.post_tunnel_curve_active = False
                                self.post_tunnel_curve_exit_stable_frames = 0
                                self.post_tunnel_curve_candidate_count = 0
                                self.post_tunnel_curve_boost_consumed = True
                                self.curve_containment_scale = 1.0
                                self.last_trusted_near_left = None
                                self.last_trusted_near_right = None
                                self.last_trusted_near_target = None
                                self.last_trusted_near_width = None
                                self.apex_guard_hold_count = 0
                                self.apex_quarantine_active = False
                                self.apex_recovery_streak = 0
                                self._last_apex_guard_diag = None
                                self.get_logger().info('POST_TUNNEL_CURVE_BOOST_OFF')
                                self.post_curve_reacquire_active = False
                                self.post_curve_reacquire_stable_count = 0
                                self.post_curve_reacquire_last_target = v1_tx

                                # Activate post-curve right-lane continuity immediately
                                self.post_reacquire_protect_active = True
                                _tw = getattr(self, 'trusted_post_tunnel_lane_width', 320.0)
                                if _tw is None or _tw < 250.0 or _tw > 420.0:
                                    _tw = 320.0
                                self.post_reacquire_trusted_width = _tw
                                self.post_reacquire_trusted_right = rx if rx is not None else ((lx + _tw) if lx is not None else None)
                                _init_target = (rx - (_tw / 2.0)) if rx is not None else ((lx + (_tw / 2.0)) if lx is not None else (v1_tx if v1_tx is not None else image_center_x))
                                self.post_reacquire_protected_target = _init_target
                                self.post_reacquire_reference_target = _init_target
                                self.post_reacquire_trusted_target = _init_target
                                self.post_reacquire_dual_streak = 0
                                self.post_reacquire_single_streak = 0
                                self.post_reacquire_right_missing_count = 0 if rx is not None else 1
                                self.right_cont_start_x = getattr(self, 'x', 0.0)
                                self.right_cont_start_y = getattr(self, 'y', 0.0)
                                self.get_logger().info(
                                    f'[RIGHT_CONT] ENTER ref_target={_init_target:.1f} '
                                    f'trusted_width={_tw:.1f} trusted_right={self.post_reacquire_trusted_right}'
                                )
                        else:
                            self.post_tunnel_curve_exit_stable_frames = 0

                    # Post-Curve Guard: Remain active across junction until junction-risk is seen and recovered
                    if getattr(self, 'post_tunnel_guard_active', False) and not self.post_tunnel_curve_boost_active:
                        trusted_w = getattr(self, 'trusted_post_tunnel_lane_width', 350.0)
                        measured_w = float(rx - lx) if (lx is not None and rx is not None) else None
                        width_ratio = (measured_w / trusted_w) if (measured_w is not None and trusted_w > 0) else None

                        # Check for junction-risk event
                        risk_reason = None
                        if lx is not None and rx is None:
                            risk_reason = 'SINGLE_LEFT'
                        elif rx is not None and lx is None:
                            risk_reason = 'SINGLE_RIGHT'
                        elif lx is None and rx is None:
                            risk_reason = 'NO_BOUNDARIES'
                        elif measured_w is not None:
                            if measured_w < 220.0 or (width_ratio is not None and width_ratio < 0.70):
                                risk_reason = 'NARROW_WIDTH'
                            elif (width_ratio is not None and width_ratio > 1.35) or measured_w > 450.0:
                                risk_reason = 'WIDE_WIDTH'
                            elif v1_tx is not None and abs(v1_tx - image_center_x) > 100.0:
                                risk_reason = 'IMPLAUSIBLE_TARGET'

                        if risk_reason and not getattr(self, 'post_tunnel_junction_risk_seen', False):
                            self.post_tunnel_junction_risk_seen = True
                            now_sec = self._get_current_time_sec()
                            if now_sec - getattr(self, '_last_post_tunnel_junction_risk_log_time', 0.0) > 0.5:
                                self._last_post_tunnel_junction_risk_log_time = now_sec
                                mw_str = f'{measured_w:.1f}' if measured_w is not None else 'None'
                                self.get_logger().info(
                                    f'POST_TUNNEL_JUNCTION_RISK reason={risk_reason} measured_width={mw_str} trusted_width={trusted_w:.1f}'
                                )

                        is_guard_stable = (
                            getattr(self, 'post_tunnel_junction_risk_seen', False) and
                            risk_reason is None and
                            v1_conf >= self.min_confidence and v1_tx is not None and
                            lx is not None and rx is not None and
                            measured_w is not None and (280.0 <= measured_w <= 420.0) and
                            (width_ratio is not None and 0.75 <= width_ratio <= 1.25) and
                            (abs(v1_tx - image_center_x) < 50.0) and
                            abs(curve_signal) < self.normal_curve_min_signal_px
                        )
                        if is_guard_stable:
                            self.post_tunnel_guard_stable_frames += 1
                            if self.post_tunnel_guard_stable_frames >= 15:
                                self.post_tunnel_guard_active = False
                                self.post_tunnel_assist_consumed = True
                                self.get_logger().info('POST_TUNNEL_GUARD_COMPLETE')
                                self.get_logger().info('POST_TUNNEL_ASSIST_CONSUMED')
                        else:
                            self.post_tunnel_guard_stable_frames = 0

            # Normal curve feedforward: active strictly under confirmed post_tunnel_curve_boost_active
            if getattr(self, 'post_tunnel_curve_boost_active', False):
                self.last_normal_w_curve = w_curve
            else:
                w_curve = 0.0
                self.prev_normal_w_curve = 0.0
                self.last_normal_w_curve = 0.0

            if v1_conf >= self.min_confidence and v1_tx is not None:
                perception_mode = 'LANE'
                target_x = v1_tx
                self.last_stable_normal_target_x = v1_tx
                abs_error = abs((target_x - image_center_x) / image_center_x)
                if abs_error < 0.15:
                    speed = self.straight_speed
                elif abs_error < 0.35:
                    speed = self.medium_speed
                else:
                    speed = self.corner_speed

                # Early curve speed applies strictly when post_tunnel_curve_boost_active
                if getattr(self, 'post_tunnel_curve_boost_active', False) and curve_speed is not None and curve_speed < speed:
                    speed = curve_speed

                # Post-Curve Reacquisition Speed Cap (Part B)
                if getattr(self, 'post_curve_reacquire_active', False) and speed > self.medium_speed:
                    speed = self.medium_speed

                # Post-Curve Lane Reacquisition State Machine (Part B)
                if getattr(self, 'post_curve_reacquire_active', False):
                    c1_scene_normal = (scene == 'NORMAL')
                    c2_near_valid = (v1_tx is not None)
                    c3_conf_high = (v1_conf >= 0.8)
                    c4_dual_boundaries = (lx is not None and rx is not None)
                    trusted_w = getattr(self, 'trusted_post_tunnel_lane_width', 350.0)
                    measured_w = float(rx - lx) if (lx is not None and rx is not None) else None
                    width_ratio = (measured_w / trusted_w) if (measured_w is not None and trusted_w > 0) else None
                    c5_plausible_width = (measured_w is not None and 280.0 <= measured_w <= 420.0)
                    c6_consistent_width = (width_ratio is not None and 0.75 <= width_ratio <= 1.25)
                    c7_target_centered = (v1_tx is not None and abs(v1_tx - image_center_x) < 50.0)

                    target_delta = abs(v1_tx - self.post_curve_reacquire_last_target) if (v1_tx is not None and self.post_curve_reacquire_last_target is not None) else 0.0
                    c8_target_variation_small = (target_delta < 15.0)

                    cur_err = (v1_tx - image_center_x) / image_center_x if v1_tx is not None else 0.0
                    cur_deriv = cur_err - getattr(self, 'prev_error', 0.0)
                    cur_w_baseline = -(self.kp * cur_err + self.kd * cur_deriv)
                    c9_w_baseline_modest = (abs(cur_w_baseline) < 0.08)

                    c10_no_junction_risk = (measured_w is not None and not (measured_w < 220.0 or (width_ratio is not None and width_ratio < 0.70)))

                    last_exit_track = getattr(self, 'last_stable_normal_target_x', image_center_x)
                    if last_exit_track is None:
                        last_exit_track = image_center_x
                    c11_temporal_continuous = (v1_tx is not None and abs(v1_tx - last_exit_track) < 40.0)

                    is_reacquire_stable = (
                        c1_scene_normal and c2_near_valid and c3_conf_high and
                        c4_dual_boundaries and c5_plausible_width and c6_consistent_width and
                        c7_target_centered and c8_target_variation_small and
                        c9_w_baseline_modest and c10_no_junction_risk and c11_temporal_continuous
                    )

                    if is_reacquire_stable:
                        self.post_curve_reacquire_stable_count += 1
                        if self.post_curve_reacquire_stable_count >= 10:
                            self.post_curve_reacquire_active = False
                            self.post_curve_reacquire_stable_count = 0
                            self.get_logger().info('POST_CURVE_REACQUIRE_COMPLETE')
                            # Activate POST_REACQUIRE_PROTECT handoff safety
                            self.post_reacquire_protect_active = True
                            self.post_reacquire_reference_target = v1_tx
                            self.post_reacquire_protected_target = v1_tx
                            self.post_reacquire_dual_streak = 0
                            self.post_reacquire_single_streak = 0
                            self.post_reacquire_trusted_width = measured_w if measured_w is not None else 320.0
                            self.post_reacquire_trusted_right = rx if rx is not None else ((lx + self.post_reacquire_trusted_width) if lx is not None else None)
                            self.post_reacquire_trusted_target = v1_tx
                            self.right_cont_start_x = getattr(self, 'x', 0.0)
                            self.right_cont_start_y = getattr(self, 'y', 0.0)
                            self.get_logger().info(
                                f'POST_REACQUIRE_PROTECT_ON ref_target={v1_tx:.1f} '
                                f'trusted_width={self.post_reacquire_trusted_width:.1f} trusted_right={rx}'
                            )
                    else:
                        self.post_curve_reacquire_stable_count = 0

                    if v1_tx is not None:
                        self.post_curve_reacquire_last_target = v1_tx

                    # Throttled diagnostic logging (~0.5s)
                    if getattr(self, 'post_curve_reacquire_active', False):
                        now_sec = self._get_current_time_sec()
                        if now_sec - getattr(self, '_last_post_curve_reacquire_log_time', 0.0) > 0.5:
                            self._last_post_curve_reacquire_log_time = now_sec
                            mw_str = f'{measured_w:.1f}' if measured_w is not None else 'None'
                            tw_str = f'{trusted_w:.1f}'
                            tx_str = f'{v1_tx:.1f}' if v1_tx is not None else 'None'
                            self.get_logger().info(
                                f'POST_CURVE_REACQUIRE target={tx_str} confidence={v1_conf:.2f} '
                                f'measured_width={mw_str} trusted_width={tw_str} target_delta={target_delta:.1f} '
                                f'w_baseline={cur_w_baseline:+.3f} stable_count={self.post_curve_reacquire_stable_count}/10 speed={speed:.2f}'
                            )

                # ========== POST_REACQUIRE_PROTECT handoff safety ==========
                if getattr(self, 'post_reacquire_protect_active', False):
                    _pr_trusted_w = getattr(self, 'post_reacquire_trusted_width', None)
                    if _pr_trusted_w is None or _pr_trusted_w <= 0:
                        _pr_trusted_w = getattr(self, 'trusted_post_tunnel_lane_width', 320.0)
                    if _pr_trusted_w > 420.0 or _pr_trusted_w < 250.0:
                        _pr_trusted_w = 320.0

                    # Local narrow false-pair threshold (Section 4):
                    # narrow_threshold = max(280.0, 0.85 * trusted_lane_width)
                    _narrow_threshold = max(280.0, 0.85 * _pr_trusted_w)

                    _pr_jump_threshold = 0.15 * _pr_trusted_w
                    _pr_max_step = 0.05 * _pr_trusted_w
                    _pr_ref = self.post_reacquire_reference_target
                    _pr_prot = self.post_reacquire_protected_target
                    _pr_exited = False

                    last_trusted_r = getattr(self, 'post_reacquire_trusted_right', None)

                    # FIX 3: Select individual right contour rather than whole-half moment
                    rx_selected = self.select_right_lane_contour(v1_clean, last_trusted_r, _pr_trusted_w, image_center_x)

                    right_coherent = (rx_selected is not None) and (
                        (last_trusted_r is None) or (abs(rx_selected - last_trusted_r) <= _pr_jump_threshold)
                    )

                    if right_coherent:
                        rx = rx_selected
                        target_from_right = rx - (_pr_trusted_w / 2.0)
                        self.post_reacquire_trusted_right = rx
                        self.half_lane_px = _pr_trusted_w / 2.0
                        now_sec = self._get_current_time_sec()
                        if (getattr(self, 'post_reacquire_right_missing_count', 0) > 0 or
                            now_sec - getattr(self, '_last_right_cont_track_log_time', 0.0) > 1.0):
                            self._last_right_cont_track_log_time = now_sec
                            self.get_logger().info(f'[RIGHT_CONT] TRACK_CANDIDATE rx={rx:.1f} trusted={last_trusted_r}')
                        self.post_reacquire_right_missing_count = 0

                        # Check for valid, stable dual-lane pair to return to normal dual-lane following
                        if lx is not None:
                            _measured_w = float(rx - lx)
                            _cand_dual_target = (lx + rx) / 2.0
                            _width_ratio = _measured_w / _pr_trusted_w
                            is_valid_dual_pair = (
                                (280.0 <= _measured_w <= 420.0) and
                                (0.75 <= _width_ratio <= 1.25) and
                                (abs(_cand_dual_target - target_from_right) <= _pr_jump_threshold) and
                                (_pr_prot is None or abs(_cand_dual_target - _pr_prot) <= _pr_jump_threshold)
                            )
                            if is_valid_dual_pair:
                                self.post_reacquire_dual_streak += 1
                                self.post_reacquire_trusted_width = 0.95 * _pr_trusted_w + 0.05 * _measured_w

                                # FIX 2: Relative distance check before release
                                _cur_x = getattr(self, 'x', 0.0)
                                _cur_y = getattr(self, 'y', 0.0)
                                _s_x = getattr(self, 'right_cont_start_x', None)
                                _s_y = getattr(self, 'right_cont_start_y', None)
                                right_cont_dist = math.hypot(_cur_x - _s_x, _cur_y - _s_y) if (_s_x is not None and _s_y is not None) else 0.0

                                is_streak_ok = (self.post_reacquire_dual_streak >= getattr(self, 'POST_REACQUIRE_STABLE_DUAL_FRAMES', 15))
                                is_dist_ok = (right_cont_dist >= getattr(self, 'RIGHT_CONT_MIN_DISTANCE', 4.5))

                                if is_streak_ok and is_dist_ok:
                                    _pr_exited = True
                                    self.post_reacquire_protect_active = False
                                    self.get_logger().info(
                                        f'[RIGHT_CONT] RELEASE distance={right_cont_dist:.2f} '
                                        f'streak={self.post_reacquire_dual_streak} '
                                        f'measured_w={_measured_w:.1f} trusted_w={self.post_reacquire_trusted_width:.1f} '
                                        f'target={_cand_dual_target:.1f}'
                                    )
                            else:
                                self.post_reacquire_dual_streak = 0
                        else:
                            self.post_reacquire_dual_streak = 0

                        chosen_target = _cand_dual_target if _pr_exited else target_from_right
                        self.post_reacquire_protected_target = chosen_target
                        target_x = chosen_target
                        v1_tx = chosen_target
                        self.last_stable_normal_target_x = chosen_target

                    else:
                        # Right boundary disappeared briefly or failed continuity
                        self.post_reacquire_dual_streak = 0
                        self.post_reacquire_right_missing_count = getattr(self, 'post_reacquire_right_missing_count', 0) + 1
                        _small_limit = 6

                        if self.post_reacquire_right_missing_count == 1:
                            _last_tgt_str = f'{_pr_prot:.1f}' if _pr_prot is not None else '0.0'
                            self.get_logger().info(
                                f'[RIGHT_CONT] SHORT_FALLBACK missing_cnt=1 last_target={_last_tgt_str}'
                            )

                        if self.post_reacquire_right_missing_count <= _small_limit:
                            fallback_target = _pr_prot if _pr_prot is not None else (_pr_ref if _pr_ref is not None else image_center_x)
                        else:
                            fallback_target = image_center_x

                        self.post_reacquire_protected_target = fallback_target
                        target_x = fallback_target
                        v1_tx = fallback_target
                        self.last_stable_normal_target_x = fallback_target

                    # Speed cap while protect is active
                    if not _pr_exited and getattr(self, 'post_reacquire_protect_active', False):
                        if speed > self.medium_speed:
                            speed = self.medium_speed

                    # Throttled diagnostic logging (~0.5s)
                    if getattr(self, 'post_reacquire_protect_active', False) or _pr_exited:
                        _pr_now = self._get_current_time_sec()
                        if _pr_now - getattr(self, '_last_post_reacquire_protect_log_time', 0.0) > 0.5 or _pr_exited:
                            self._last_post_reacquire_protect_log_time = _pr_now
                            _pr_prot_str = f'{self.post_reacquire_protected_target:.1f}' if self.post_reacquire_protected_target is not None else 'None'
                            _pr_ref_str = f'{self.post_reacquire_reference_target:.1f}' if self.post_reacquire_reference_target is not None else 'None'
                            _pr_src = 'DUAL' if (lx is not None and rx is not None) else ('SINGLE_LEFT' if lx is not None else ('SINGLE_RIGHT' if rx is not None else 'NONE'))
                            self.get_logger().info(
                                f'POST_REACQUIRE_PROTECT source={_pr_src} '
                                f'ref={_pr_ref_str} protected={_pr_prot_str} '
                                f'raw_target={v1_tx:.1f} '
                                f'dual_streak={self.post_reacquire_dual_streak} '
                                f'single_streak={self.post_reacquire_single_streak} '
                                f'speed={speed:.2f}'
                            )

                if getattr(self, 'post_tunnel_curve_boost_active', False) and not getattr(self, '_post_tunnel_boost_logged', False):
                    self._post_tunnel_boost_logged = True
                    self.get_logger().info(
                        f'POST_TUNNEL_CURVE_BOOST_ACTIVE near={v1_tx:.1f} far={far_center:.1f} '
                        f'curve_signal={curve_signal:+.1f} w_curve={w_curve:+.3f} speed={speed:.2f}'
                    )
                elif not getattr(self, 'post_tunnel_curve_boost_active', False):
                    self._post_tunnel_boost_logged = False
            else:
                # Secondary Bright-Surface Fallback (Ramp)
                s_tx, s_cov, s_rows, s_clean, s_endpoints = self.find_surface_target(v1_roi)
                if s_tx is not None:
                    perception_mode = 'SURFACE'
                    target_x = s_tx
                    self.last_stable_normal_target_x = s_tx
                    coverage = s_cov
                    valid_rows = s_rows
                    surface_clean = s_clean
                    row_endpoints = s_endpoints

                    # Separate SURFACE target steering from RAMP slow speed authority
                    pitch = getattr(self, 'pitch_deg', 0.0)
                    slope_confirmed = self.check_physical_slope()
                    s_err = (target_x - image_center_x) / image_center_x
                    abs_s_err = abs(s_err)

                    if abs_s_err < 0.15:
                        norm_v = self.straight_speed
                    elif abs_s_err < 0.35:
                        norm_v = self.medium_speed
                    else:
                        norm_v = self.corner_speed

                    if slope_confirmed:
                        speed = self.ramp_fallback_speed
                        surf_reason = 'CONFIRMED_RAMP_SLOW_SPEED'
                    else:
                        speed = norm_v
                        surf_reason = 'FLAT_SURFACE_USE_NORMAL_SPEED'

                    self.log_surface_speed_arbitration(
                        pitch=pitch,
                        slope_confirmed=slope_confirmed,
                        target_x=target_x,
                        error=s_err,
                        normal_v=norm_v,
                        final_v=speed,
                        reason=surf_reason
                    )
                else:
                    self.perception_mode = 'NONE'
                    self.stop()
                    self.log_stop('LOW_CONFIDENCE', front=front, conf=v1_conf)
                    return

        self.perception_mode = perception_mode

        # Velocity Arbitration: apply safety / behavioral speed constraints
        base_speed = speed
        speed = self.apply_speed_constraints(speed, self.crossing_state)

        # Diagnostic logging for speed arbitration (Phase 12)
        if matched_static_edge:
            arb_reason = 'STATIC_EDGE_SUPPRESSED'
        elif perception_mode == 'SURFACE' and getattr(self, 'ramp_slope_active', False):
            arb_reason = 'LANE_CONSTRAINT'
        elif speed < self.straight_speed:
            arb_reason = 'CURVATURE_CONSTRAINT'
        else:
            arb_reason = 'NORMAL'

        self.log_speed_arbitration(
            state=self.crossing_state, base_v=base_speed,
            crossing_class=crossing_occ.get('cluster_class', 'NONE') if crossing_occ else 'NONE',
            crossing_scan_seq=getattr(self, '_crossing_processed_scan_seq', -1),
            generic_obstacle=(obs is not None and obs.get('stop', False)),
            generic_x=generic_x, generic_y=generic_y,
            matched_static_edge=matched_static_edge, final_v=speed, reason=arb_reason
        )

        # Normalized lateral error [-1.0 .. +1.0]
        error = (target_x - image_center_x) / image_center_x

        # Optional debug visualization (does not alter control)
        if self.debug_viz and self.bridge is not None:
            viz_mask = clean if perception_mode != 'SURFACE' else surface_clean
            self._publish_debug_viz(
                roi, viz_mask, left_x, right_x, target_x, image_center_x,
                perception_mode=perception_mode, conf=confidence,
                coverage=coverage, valid_rows=valid_rows, row_endpoints=row_endpoints,
                brightness=s_median, dark_rows=dark_valid_rows,
                skipped_transverse=skipped_transverse,
                near_center=near_center, far_center=far_center,
                dark_target=dark_raw_target, filtered_target=target_x, err=error,
                scene=scene, scene_mean=s_mean, scene_median=s_median,
                scene_p25=s_p25, dark_ratio=dark_ratio,
                crossing_state=self.crossing_state, crossing_occupied=self.crossing_occupied,
                zebra_detected=self.last_zebra_detected, zebra_bands=self.last_zebra_bands)

        # Shared PD Steering Controller
        derivative = error - self.prev_error
        self.prev_error = error

        if perception_mode in ('DARK_HOLD', 'DARK_RECOVERY'):
            steering = recovery_w
        elif perception_mode == 'DARK_LANE':
            w_lateral = -(self.kp * error + self.kd * derivative)
            w_curve = getattr(self, 'last_dark_w_curve', 0.0)
            w_cmd = w_lateral + w_curve
            steering = float(max(-self.dark_max_w, min(self.dark_max_w, w_cmd)))
        elif perception_mode == 'LANE' and getattr(self, 'post_tunnel_curve_boost_active', False):
            w_baseline = -(self.kp * error + self.kd * derivative)
            w_boost = getattr(self, 'last_normal_w_curve', 0.0)

            # PART A: Smooth Boost Attenuation for Lane Containment
            if w_baseline * w_boost >= 0:
                raw_scale = 1.0
            else:
                # Sign conflict: NEAR centering opposes curve boost
                # Only trust NEAR when geometry is good
                trusted_w = getattr(self, 'trusted_post_tunnel_lane_width', 350.0)
                measured_w = float(right_x - left_x) if (left_x is not None and right_x is not None) else None
                width_ratio = (measured_w / trusted_w) if (measured_w is not None and trusted_w > 0) else None

                near_trusted = (
                    target_x is not None and
                    confidence >= 0.8 and
                    left_x is not None and right_x is not None and
                    measured_w is not None and (220.0 <= measured_w <= 450.0) and
                    (width_ratio is not None and 0.70 <= width_ratio <= 1.35) and
                    abs(target_x - image_center_x) < 160.0
                )

                if near_trusted:
                    opp_w = abs(w_baseline)
                    min_scale = 0.35
                    if opp_w <= 0.03:
                        raw_scale = 1.0
                    elif opp_w < 0.15:
                        raw_scale = 1.0 - (1.0 - min_scale) * ((opp_w - 0.03) / (0.15 - 0.03))
                    else:
                        raw_scale = min_scale
                else:
                    raw_scale = 1.0

            # EMA smoothing to eliminate any single-frame jumps
            self.curve_containment_scale = 0.5 * raw_scale + 0.5 * getattr(self, 'curve_containment_scale', 1.0)
            ema_scale = self.curve_containment_scale
            boost_scale = ema_scale

            # Authority Floor Protection: Prevent containment from extinguishing active curve steering
            cur_far_cx = locals().get('far_center', getattr(self, '_diag_far_cx', None))
            cur_far_conf = locals().get('far_conf', getattr(self, '_diag_far_conf', 0.0))
            cur_csig = locals().get('curve_signal', getattr(self, '_diag_curve_signal', 0.0))
            cur_dropout = getattr(self, 'post_tunnel_curve_dropout_frames', 0)
            cur_exit_cnt = getattr(self, 'post_tunnel_curve_exit_stable_frames', 0)

            protect_active = (
                getattr(self, 'post_tunnel_curve_boost_active', False) and
                (w_baseline * w_boost < 0.0) and
                (cur_far_cx is not None) and
                (cur_far_conf >= 0.7) and
                (abs(cur_csig) >= self.normal_curve_min_signal_px) and
                (cur_dropout == 0) and
                (cur_exit_cnt == 0) and
                (abs(w_boost) > 1e-4)
            )

            if protect_active:
                authority_floor = 0.03
                required_scale = (abs(w_baseline) + authority_floor) / abs(w_boost)
                required_scale = max(0.0, min(1.0, required_scale))
                protected_scale = max(boost_scale, required_scale)
                protected_scale = max(0.0, min(1.0, protected_scale))

                if protected_scale > boost_scale:
                    w_cmd_unprotected = w_baseline + w_boost * boost_scale
                    boost_scale = protected_scale
                    w_cmd_protected = w_baseline + w_boost * boost_scale
                    _fseq = getattr(self, '_ctrl_frame_seq', 0)
                    self.get_logger().info(
                        f'[CURVE_AUTHORITY_PROTECT] frame={_fseq} '
                        f'w_baseline={w_baseline:+.4f} w_boost={w_boost:+.4f} '
                        f'ema_containment_scale={ema_scale:.3f} required_scale={required_scale:.3f} '
                        f'effective_scale={boost_scale:.3f} '
                        f'w_cmd_without_protection={w_cmd_unprotected:+.4f} '
                        f'w_cmd_with_protection={w_cmd_protected:+.4f} '
                        f'curve_signal={cur_csig:+.1f} far_confidence={cur_far_conf:.2f} '
                        f'far_dropout={cur_dropout} exit_stable_frames={cur_exit_cnt}'
                    )

            w_boost_effective = w_boost * boost_scale
            w_cmd = w_baseline + w_boost_effective
            steering = float(max(-self.max_angular, min(self.max_angular, w_cmd)))

            # Throttled diagnostic logging (~0.5s)
            now_sec = self._get_current_time_sec()
            if now_sec - getattr(self, '_last_curve_containment_log_time', 0.0) > 0.5:
                self._last_curve_containment_log_time = now_sec
                self.get_logger().info(
                    f'CURVE_LANE_CONTAINMENT near_error={error:+.3f} near_conf={confidence:.2f} '
                    f'w_baseline={w_baseline:+.3f} w_boost={w_boost:+.3f} boost_scale={boost_scale:.2f} '
                    f'w_boost_effective={w_boost_effective:+.3f} w_final={steering:+.3f}'
                )

            # Apex NEAR Temporal Sanity Guard Diagnostic Logging
            diag = getattr(self, '_last_apex_guard_diag', None)
            if diag and diag.get('in_curve', False):
                is_rej = diag.get('triggered', False)
                last_log_t = getattr(self, '_last_apex_guard_log_time', 0.0)
                should_log = is_rej or getattr(self, '_apex_guard_was_triggered', False) or (now_sec - last_log_t > 0.5)
                if should_log:
                    self._last_apex_guard_log_time = now_sec
                    self._apex_guard_was_triggered = is_rej
                    rej_val = 1 if is_rej else 0
                    t_prev = f'{diag["trusted_target"]:.1f}' if diag.get("trusted_target") is not None else "None"
                    t_now = f'{diag["cand_target"]:.1f}' if diag.get("cand_target") is not None else "None"
                    t_jump = f'{diag["target_jump"]:+.1f}' if diag.get("target_jump") is not None else "+0.0"
                    l_jump = f'{diag["left_jump"]:+.1f}' if diag.get("left_jump") is not None else "+0.0"
                    r_jump = f'{diag["right_jump"]:+.1f}' if diag.get("right_jump") is not None else "+0.0"
                    c_l = f'{diag["cand_left"]:.1f}' if diag.get("cand_left") is not None else "None"
                    c_r = f'{diag["cand_right"]:.1f}' if diag.get("cand_right") is not None else "None"
                    c_w = f'{diag["cand_width"]:.1f}' if diag.get("cand_width") is not None else "None"
                    c_w_base = f'{diag["cand_w"]:+.3f}' if diag.get("cand_w") is not None else "+0.000"
                    q_act = 1 if diag.get("quarantine_active", False) else 0
                    r_str = diag.get("recovery_streak", 0)
                    self.get_logger().info(
                        f'[APEX_NEAR_GUARD] POST_TUNNEL_CURVE reject={rej_val} quar={q_act} rec={r_str} '
                        f'target_prev={t_prev} target_now={t_now} '
                        f'jump={t_jump} left_jump={l_jump} right_jump={r_jump} '
                        f'cand_l={c_l} cand_r={c_r} cand_w={c_w} '
                        f'w_candidate={c_w_base} dir={diag.get("curve_dir", "NONE")} '
                        f'hold={diag.get("hold_count", 0)} final_w={steering:+.3f}'
                    )
        else:
            w_baseline = -(self.kp * error + self.kd * derivative)
            steering = float(max(-self.max_angular, min(self.max_angular, w_baseline)))

        if perception_mode == 'DARK_LANE':
            self.last_good_dark_w = steering
            pitch = getattr(self, 'pitch_deg', 0.0)
            w_lat = -(self.kp * error + self.kd * derivative)
            w_c = getattr(self, 'last_dark_w_curve', 0.0)
            c_sig = getattr(self, 'last_dark_curve_signal', 0.0)
            self.get_logger().info(
                f'DARK_LANE_VALID rows={valid_rows} target={target_x:.1f} v={speed:.2f} '
                f'w_lat={w_lat:+.2f} w_curve={w_c:+.2f} (csig={c_sig:+.1f}) w={steering:+.2f} pitch={pitch:+.1f}'
            )

        # Record decomposition terms
        self._diag_w_baseline = locals().get('w_baseline', locals().get('w_lateral', 0.0))
        self._diag_w_boost_raw = getattr(self, 'last_normal_w_curve', 0.0) if getattr(self, 'post_tunnel_curve_boost_active', False) else 0.0
        self._diag_boost_scale = getattr(self, 'curve_containment_scale', 1.0) if getattr(self, 'post_tunnel_curve_boost_active', False) else 1.0
        self._diag_w_boost_eff = (getattr(self, 'last_normal_w_curve', 0.0) * getattr(self, 'curve_containment_scale', 1.0)) if getattr(self, 'post_tunnel_curve_boost_active', False) else 0.0
        self._diag_w_cmd = locals().get('w_cmd', steering)

        # =================================================================
        # SIMPLE LOCAL CURVE-HOLD FIX (POST-TUNNEL LEFT CURVE ONLY)
        # =================================================================
        normal_steering = steering
        _fseq = getattr(self, '_ctrl_frame_seq', 0)

        # Reset state machine when outside NORMAL mode (e.g. entering or inside tunnel)
        if str(getattr(self, 'scene_state', 'NORMAL')) != 'NORMAL':
            self.simple_curve_state = 'IDLE'
            self.simple_curve_history.clear()
            self.simple_curve_held_steering = 0.0
            self.simple_curve_hold_start_x = None
            self.simple_curve_hold_start_y = None
            self.simple_curve_recovery_frames = 0

        _rx0 = getattr(self, 'post_tunnel_ref_x', None)
        _ry0 = getattr(self, 'post_tunnel_ref_y', None)
        _ryaw0 = getattr(self, 'post_tunnel_ref_yaw', None)

        _has_valid_ref = (
            getattr(self, 'has_odom', False) and
            (_rx0 is not None) and
            (_ry0 is not None) and
            (_ryaw0 is not None) and
            getattr(self, 'x', None) is not None and
            getattr(self, 'y', None) is not None and
            getattr(self, 'yaw', None) is not None
        )

        _post_tunnel_context_valid = (
            _has_valid_ref and
            getattr(self, 'post_tunnel_assist_armed', False) and
            (str(getattr(self, 'scene_state', 'NORMAL')) == 'NORMAL')
        )

        if _has_valid_ref:
            dx = self.x - _rx0
            dy = self.y - _ry0
            x_local = math.cos(_ryaw0) * dx + math.sin(_ryaw0) * dy
            y_local = -math.sin(_ryaw0) * dx + math.cos(_ryaw0) * dy
            relative_yaw = normalize_angle(self.yaw - _ryaw0)
        else:
            x_local = 0.0
            y_local = 0.0
            relative_yaw = 0.0

        self.simple_curve_x_local = x_local
        self.simple_curve_y_local = y_local
        self.simple_curve_rel_yaw = relative_yaw

        # Landmark / artificial curve-hold bypass:
        # Zero steering authority for traffic signs, horizontal markings, or forced turn overrides.
        # Normal lane follower keeps 100% continuous steering ownership through the curve.
        self.simple_curve_state = 'DONE'
        steering = normal_steering

        # Final angular clamp
        steering = float(max(-self.max_angular, min(self.max_angular, steering)))
        self._diag_w_final = steering
        self._diag_v_final = speed

        # Per-cycle [STEER_STAGE] trace for post-tunnel section
        if getattr(self, 'post_tunnel_trace_active', False):
            _fseq_stg = getattr(self, '_ctrl_frame_seq', 0)
            _wb = getattr(self, '_diag_w_baseline', 0.0)
            _wbr = getattr(self, '_diag_w_boost_raw', 0.0)
            _wbe = getattr(self, '_diag_w_boost_eff', 0.0)
            _wcm = getattr(self, '_diag_w_cmd', 0.0)
            _wfin = getattr(self, '_diag_w_final', 0.0)
            self.get_logger().info(
                f'[STEER_STAGE] frame={_fseq_stg} '
                f'stage=PD_BASELINE before=N/A after={_wb:+.4f} changed=1'
            )
            self.get_logger().info(
                f'[STEER_STAGE] frame={_fseq_stg} '
                f'stage=BOOST_RAW before={_wb:+.4f} after={_wbr:+.4f} changed={int(abs(_wbr) > 0.001)}'
            )
            self.get_logger().info(
                f'[STEER_STAGE] frame={_fseq_stg} '
                f'stage=BOOST_CONTAINMENT before={_wbr:+.4f} after={_wbe:+.4f} '
                f'changed={int(abs(_wbr - _wbe) > 0.001)}'
            )
            self.get_logger().info(
                f'[STEER_STAGE] frame={_fseq_stg} '
                f'stage=W_CMD before={_wbe:+.4f} after={_wcm:+.4f} changed=1'
            )
            self.get_logger().info(
                f'[STEER_STAGE] frame={_fseq_stg} '
                f'stage=CLAMP_MAX_ANGULAR before={_wcm:+.4f} after={normal_steering:+.4f} '
                f'changed={int(abs(_wcm - normal_steering) > 0.001)}'
            )
            if self.simple_curve_state == 'HOLD_CURVE':
                self.get_logger().info(
                    f'[STEER_STAGE] frame={_fseq_stg} '
                    f'stage=SIMPLE_CURVE_{self.simple_curve_state} before={normal_steering:+.4f} after={_wfin:+.4f} '
                    f'changed={int(abs(_wfin - normal_steering) > 0.001)}'
                )
            self.get_logger().info(
                f'[STEER_STAGE] frame={_fseq_stg} '
                f'stage=CMD_VEL_PUBLISH before={_wfin:+.4f} after={_wfin:+.4f} changed=0'
            )

        # Command actuator
        self.drive(speed, steering)

        # Log frame telemetry
        now_sec = self._get_current_time_sec()
        self.log_forensic_telemetry(now_sec, scene, perception_mode, target_x, confidence, error, speed, steering, front, obs)

        # Throttled diagnostic logging (~1 Hz)
        if perception_mode in ('DARK_LANE', 'DARK_HOLD', 'DARK_PENDING'):
            self.log_drive_dark_lane(
                s_median, valid_rows, near_center, far_center,
                target_x, error, speed, steering, front,
                skipped_transverse_count=len(skipped_transverse),
                scene=scene, median=s_median, dark_ratio=dark_ratio,
                perception_mode=perception_mode)
        elif perception_mode == 'LANE':
            self.log_drive_lane(
                left_x, right_x, target_x, confidence, error, speed, steering, front,
                scene=scene, median=s_median, dark_ratio=dark_ratio)
        else:
            self.log_drive_surface(
                coverage, target_x, valid_rows, error, speed, steering, front,
                scene=scene, median=s_median, dark_ratio=dark_ratio)


def catch_sigterm():
    """Turn SIGTERM into a flag instead of letting rclpy tear down the context.

    Call this before building the node. rclpy's own handler invalidates the
    context on SIGTERM, which breaks clean shutdown when running inside
    run_demo.sh or ros2 launch.
    """
    stopping = [False]

    def _handler(signum, frame):
        stopping[0] = True

    try:
        signal.signal(signal.SIGTERM, _handler)
    except (ValueError, AttributeError):
        pass
    return stopping


def spin(node, stopping):
    """Spin the node at 50 Hz until Ctrl-C, SIGTERM, or rclpy shutdown."""
    rate = node.create_rate(50.0)
    try:
        while rclpy.ok() and not stopping[0]:
            rclpy.spin_once(node, timeout_sec=0.02)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()


def main(args=None):
    stopping = catch_sigterm()
    rclpy.init(args=args)
    node = Starter()
    try:
        spin(node, stopping)
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
