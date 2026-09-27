#!/usr/bin/env python3
"""Lane following and autonomous driving node for UEH CRC 2026.

Thuật toán bám DUY NHẤT VẠCH PHẢI (Pure Right-Line Follower):
1. CHỈ QUAN TÂM DUY NHẤT VẠCH PHẢI (Right Line Only):
   - Hoàn toàn bỏ qua mọi vạch bên trái (vạch tim đường, vạch đứt, làn ngược chiều, ngã rẽ trái).
   - Vùng quét chỉ lấy nửa bên phải khung hình (x từ 280px -> 635px).
   - Giữ vạch mép phải luôn nằm ở vị trí chuẩn (target_right_x = 495px trên khung hình 640px).
     + Nếu vạch phải lệch > 495px: Xe đang lệch sang trái -> Bẻ lái sang phải để ôm vạch.
     + Nếu vạch phải lệch < 495px: Xe đang quá sát mép phải -> Bẻ lái sang trái để giữ khoảng cách an toàn.
2. NẾU MẤT VẠCH PHẢI -> TIẾP TỤC ĐI THẲNG (Straight Cruise):
   - Khi đi qua các khoảng trống/mất vạch: Giữ thẳng tuyệt đối bánh lái (steer = 0.0) với tốc độ ổn định
     cho đến khi camera nhận lại được vạch phải.
3. BỘ ĐIỀU KHIỂN CHỐNG LẮC LƯ (Anti-Wobble):
   - Vùng chết Deadzone (|error| <= 7px): Xe đi thẳng mượt mà, triệt tiêu hiện tượng "rắn bò".
   - Lọc thông thấp góc lái (Low-pass filter) và giới hạn gia tốc quay (Slew rate limiter),
     giúp 2 bánh xe chuyển hướng cực kỳ êm ái.
4. TÍCH HỢP AN TOÀN LIDAR & TRỰC QUAN HÓA CAMERA THEO THỜI GIAN THỰC.
"""

import math
import os
import signal
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan

try:
    from cv_bridge import CvBridge
    HAVE_CV = True
except ImportError:
    HAVE_CV = False

try:
    from gazebo_msgs.msg import ModelStates
    HAVE_GAZEBO_MSGS = True
except ImportError:
    HAVE_GAZEBO_MSGS = False


class Starter(Node):

    def __init__(self):
        super().__init__('crc_starter')

        # Thông số vận tốc và an toàn
        self.declare_parameter('max_speed', 0.14)        # m/s (tốc độ tối đa khi đi thẳng)
        self.declare_parameter('min_speed', 0.06)        # m/s (tốc độ tối thiểu khi cua gắt)
        self.declare_parameter('straight_speed', 0.12)   # m/s (tốc độ khi mất vạch đi thẳng)
        self.declare_parameter('max_turn', 0.9)          # rad/s (giới hạn tốc độ quay)
        self.declare_parameter('stop_distance', 0.28)    # m (khoảng cách phanh an toàn cách cản trước ~8cm)
        self.declare_parameter('rate', 20.0)             # Hz (tần số điều khiển)

        # Thông số bám DUY NHẤT VẠCH PHẢI
        self.declare_parameter('target_right_x', 492.0)  # pixel (dịch xe sang trái thêm ~3cm theo yêu cầu)
        self.declare_parameter('right_search_min', 330)  # Chỉ quét x >= 330px (loại bỏ 100% vạch tim đường và làn ngược chiều)
        self.declare_parameter('right_search_max', 638)  # Đến sát mép phải ảnh (bắt trọn rìa dốc cầu)
        self.declare_parameter('kp', 0.0030)             # Hệ số tỉ lệ P
        self.declare_parameter('kd', 0.0008)             # Hệ số vi phân D
        self.declare_parameter('deadzone', 5.0)          # pixel (vùng chết triệt tiêu rung lắc)
        self.declare_parameter('max_steer_step', 0.07)   # rad/s mỗi chu kỳ (giới hạn gia tốc bẻ lái)
        self.declare_parameter('roi_top', 0.68)          # Quét từ 68% chiều cao ảnh (nhìn gần mặt đường dưới hầm)
        self.declare_parameter('roi_bottom', 0.94)       # Quét đến 94% chiều cao ảnh (sát mặt đường trước bánh xe)

        # Cấu hình hiển thị cửa sổ Camera
        has_display = bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))
        self.declare_parameter('show_view', has_display)

        self.max_speed = float(self.get_parameter('max_speed').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.straight_speed = float(self.get_parameter('straight_speed').value)
        self.max_turn = float(self.get_parameter('max_turn').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.rate = float(self.get_parameter('rate').value)

        self.target_right_x = float(self.get_parameter('target_right_x').value)
        self.right_search_min = int(self.get_parameter('right_search_min').value)
        self.right_search_max = int(self.get_parameter('right_search_max').value)

        # Cấu hình bám làn trái (khi né xe hoặc chuyển làn)
        self.declare_parameter('target_left_x', 148.0)          # Mốc chuẩn vạch trái (px)
        self.declare_parameter('left_search_min', 10)           # Vùng quét vạch trái (px)
        self.declare_parameter('left_search_max', 310)          # Vùng quét vạch trái (px)
        self.declare_parameter('lane_follow_mode', 'RIGHT')     # 'RIGHT' (bám làn phải) hoặc 'LEFT' (bám làn trái)

        self.target_left_x = float(self.get_parameter('target_left_x').value)
        self.left_search_min = int(self.get_parameter('left_search_min').value)
        self.left_search_max = int(self.get_parameter('left_search_max').value)
        self.lane_follow_mode = str(self.get_parameter('lane_follow_mode').value)

        self.kp = float(self.get_parameter('kp').value)
        self.kd = float(self.get_parameter('kd').value)
        self.deadzone = float(self.get_parameter('deadzone').value)
        self.max_steer_step = float(self.get_parameter('max_steer_step').value)
        self.roi_top = float(self.get_parameter('roi_top').value)
        self.roi_bottom = float(self.get_parameter('roi_bottom').value)
        self.show_view = bool(self.get_parameter('show_view').value)

        # Biến trạng thái điều khiển & bộ lọc
        self.prev_error = 0.0
        self.d_error_filtered = 0.0
        self.last_valid_steer = 0.0
        self.smooth_right_x = None
        self.smooth_left_x = None

        # Logic rẽ sau hầm (khi đường line cam bị kéo lệch hết sang phải, thực hiện 1 lần duy nhất)
        self.post_tunnel_state = 'IDLE'       # 'IDLE' -> 'STRAIGHT' -> 'TURN_RIGHT' -> 'DONE'
        self.post_tunnel_maneuver_done = False
        self.post_tunnel_trigger_count = 0
        self.post_tunnel_timer_start = 0.0
        self.post_tunnel_turn_start_yaw = 0.0
        self.post_tunnel_straight_time = 3.0     # Thời gian đi thẳng (giây)
        self.post_tunnel_turn_target_deg = 58.0  # Góc cua phải (độ)
        self.post_tunnel_trigger_x = 590.0       # Ngưỡng chạm x bên phải (px) - càng nhỏ càng kích hoạt sớm/sát hơn

        # Logic vượt xe né xe dừng trên cao tốc (parked_robot màu xanh, thực hiện 1 lần duy nhất)
        self.overtake_state = 'IDLE'             # 'IDLE' -> 'OVERTAKE_STEER_LEFT' -> 'OVERTAKE_DIAG_LEFT' -> 'OVERTAKE_STRAIGHTEN_LEFT' -> 'OVERTAKE_FOLLOW_LEFT' -> 'OVERTAKE_STEER_RIGHT' -> 'OVERTAKE_DIAG_RIGHT' -> 'OVERTAKE_STRAIGHTEN_RIGHT' -> 'DONE'
        self.overtake_done = False
        self.overtake_timer_start = 0.0
        self.overtake_trigger_dist = 0.70        # Khoảng cách phát hiện xe dừng phía trước (m)
        self.overtake_steer_time = 0.90          # Thời gian bẻ lái sang trái để chuyển làn (giây)
        self.overtake_steer_right_time = 1.30    # Thời gian rẽ phải sau khi bám lane trái để về lại làn phải (giây, tăng thêm theo yêu cầu)
        self.overtake_diag_time = 2.50           # Thời gian chạy chéo sang làn (giây)
        self.overtake_left_follow_time = 3.0    # Thời gian bám vạch trái một lát trước khi về lại làn phải (giây)
        self.overtake_speed_turn = 0.12          # Vận tốc khi bẻ lái (m/s)
        self.overtake_speed_straight = 0.14      # Vận tốc khi chạy thẳng/chéo (m/s)
        self.overtake_turn_rate = 0.85           # Tốc độ quay bẻ lái (rad/s)

        # Trạng thái theo dõi vị trí hầm: TRƯỚC HẦM CHỈ DÒ LANE, SAU HẦM MỚI ÁP DỤNG CÁC LOGIC THỦ CÔNG
        self.entered_tunnel = False
        self.has_passed_tunnel = False

        # Cảm biến
        self.image = None       # BGR image (480x640x3)
        self.scan = None        # sensor_msgs/LaserScan
        self.x = self.y = self.yaw = 0.0
        self._last_log = {}

        self.bridge = CvBridge() if HAVE_CV else None
        if not HAVE_CV:
            self.get_logger().warn(
                'cv_bridge not found! Cần cài đặt ros-humble-cv-bridge python3-opencv')

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_lane_debug = self.create_publisher(Image, '/camera/lane_debug', 10)

        self.create_subscription(Image, '/camera/image_raw',
                                 self.on_image, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan',
                                 self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        if HAVE_GAZEBO_MSGS:
            self.create_subscription(ModelStates, '/model_states', self.on_model_states, 10)

        self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(
            f'Pure Right-Line Follower ready | Target X={self.target_right_x}px | Search=[{self.right_search_min}..{self.right_search_max}px]')

    # --- Sensor Callbacks ---

    def on_image(self, msg):
        if self.bridge is None:
            return
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'Image conversion failed: {e}')

    def on_scan(self, msg):
        self.scan = msg

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x, self.y = p.x, p.y
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self.y >= 1.20:
            self.has_passed_tunnel = True

    def on_model_states(self, msg):
        if 'waffle' in msg.name:
            idx = msg.name.index('waffle')
            p = msg.pose[idx].position
            q = msg.pose[idx].orientation
            self.x, self.y = p.x, p.y
            self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            if self.y >= 1.20:
                self.has_passed_tunnel = True

    # --- Helper Functions ---

    def range_at(self, angle_deg, width_deg=18.0, min_dist=0.20):
        if self.scan is None or not self.scan.ranges:
            return float('inf')

        n = len(self.scan.ranges)
        half = int(round(width_deg / 2.0))
        centre = int(round(angle_deg)) % 360
        valid_ranges = []
        for d in range(-half, half + 1):
            idx = int((centre + d) % 360 * n / 360)
            r = self.scan.ranges[idx]
            # Bỏ qua các điểm < 0.20m (nhiễu phản xạ khung xe, cọc đỡ Waffle hoặc mặt dốc sát cản trước khi dốc chúi mũi)
            if math.isfinite(r) and r >= min_dist:
                valid_ranges.append(r)

        if len(valid_ranges) < 3:
            return float('inf')

        # Xác nhận vật cản bằng tối thiểu 3 tia quét để triệt tiêu hoàn toàn tia nhiễu đơn lẻ
        valid_ranges.sort()
        return valid_ranges[2]

    def drive(self, v, w):
        msg = Twist()
        msg.linear.x = float(max(-self.max_speed, min(self.max_speed, v)))
        msg.angular.z = float(max(-self.max_turn, min(self.max_turn, w)))
        self.pub_cmd.publish(msg)

    def stop(self):
        self.pub_cmd.publish(Twist())

    def log_every(self, seconds, text):
        now = time.time()
        if now - self._last_log.get(text[:20], 0.0) >= seconds:
            self._last_log[text[:20]] = now
            self.get_logger().info(text)

    def tick(self):
        try:
            self.control()
        except Exception as e:
            self.get_logger().error(f'control() raised: {e}')
            self.stop()

    # --- Thuật toán Xử lý ảnh: BÁM VẠCH PHẢI / BÁM RÌA DỐC CẦU / BÁM VẠCH TRÁI ---

    def detect_lines(self, img):
        if img is None:
            return None, None, None, (0, 0), 'NONE'

        h, w, _ = img.shape
        y1 = int(h * self.roi_top)
        y2 = int(h * self.roi_bottom)
        roi = img[y1:y2, :]

        # 1. Chuyển sang không gian màu HSV để lọc màu:
        # - Vạch kẻ trắng và mặt dốc cầu xám ĐỀU LÀ MÀU ĐƠN SẮC/PHI MÀU (Achromatic: Saturation S <= 40).
        # - Chướng ngại vật người đi bộ màu tím/indigo có độ bão hòa màu rất cao (S > 150).
        # -> Loại bỏ 100% người đi bộ màu tím, biển báo xanh/đỏ, đèn tín hiệu bằng điều kiện (S <= 40).
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        s_channel = hsv[:, :, 1]
        v_channel = hsv[:, :, 2]

        is_achromatic = (s_channel <= 40)
        # Nền đường nhựa tối có V < 50. Mặt dốc cầu xám có V >= 70. Vạch trắng có V >= 180.
        mask = np.zeros((roi.shape[0], roi.shape[1]), dtype=np.uint8)
        mask[is_achromatic & (v_channel >= 70)] = 255

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # 2. Quét đa tầng cả 2 bên (Vạch phải & Vạch trái)
        num_slices = 8
        slice_h = max(1, (y2 - y1) // num_slices)
        valid_right_centers = []
        valid_left_centers = []
        feature_types = []

        for i in range(num_slices):
            sy1 = i * slice_h
            sy2 = min(y2 - y1, (i + 1) * slice_h)
            strip = mask[sy1:sy2, :]

            # --- Quét nửa phải ---
            strip_r = np.zeros_like(strip)
            strip_r[:, self.right_search_min:self.right_search_max] = strip[:, self.right_search_min:self.right_search_max]
            pts_rx = np.where(strip_r > 0)[1]
            if len(pts_rx) >= 8:
                cols_r = sorted(pts_rx)
                clusters_r = []
                curr = [cols_r[0]]
                for c in cols_r[1:]:
                    if c - curr[-1] > 15:
                        clusters_r.append(curr)
                        curr = [c]
                    else:
                        curr.append(c)
                clusters_r.append(curr)

                valid_cl_r = [cl for cl in clusters_r if len(cl) >= 6]
                if valid_cl_r:
                    rightmost = max(valid_cl_r, key=lambda cl: np.mean(cl))
                    xs, xe = rightmost[0], rightmost[-1]
                    cw = xe - xs
                    if cw < 50:
                        target_rx = (xs + xe) / 2.0
                        feature_types.append('LINE')
                    else:
                        target_rx = float(xe)
                        feature_types.append('RAMP_EDGE')
                    valid_right_centers.append(target_rx)

            # --- Quét nửa trái ---
            strip_l = np.zeros_like(strip)
            strip_l[:, self.left_search_min:self.left_search_max] = strip[:, self.left_search_min:self.left_search_max]
            pts_lx = np.where(strip_l > 0)[1]
            if len(pts_lx) >= 8:
                cols_l = sorted(pts_lx)
                clusters_l = []
                curr = [cols_l[0]]
                for c in cols_l[1:]:
                    if c - curr[-1] > 15:
                        clusters_l.append(curr)
                        curr = [c]
                    else:
                        curr.append(c)
                clusters_l.append(curr)

                valid_cl_l = [cl for cl in clusters_l if len(cl) >= 6]
                if valid_cl_l:
                    # Chọn cụm sát mép trái nhất (vạch biên ngoài bên trái của làn trái)
                    leftmost = min(valid_cl_l, key=lambda cl: np.mean(cl))
                    xs, xe = leftmost[0], leftmost[-1]
                    target_lx = (xs + xe) / 2.0
                    valid_left_centers.append(target_lx)

        # Xử lý kết quả vạch phải
        detected_type = 'LINE'
        if len(valid_right_centers) >= 3:
            det_rx = float(np.median(valid_right_centers))
            ramp_votes = sum(1 for t in feature_types if t == 'RAMP_EDGE')
            if ramp_votes >= len(feature_types) / 2:
                detected_type = 'RAMP_EDGE'

            if self.smooth_right_x is None:
                self.smooth_right_x = det_rx
            else:
                self.smooth_right_x = 0.65 * det_rx + 0.35 * self.smooth_right_x
            right_x = self.smooth_right_x
        else:
            self.smooth_right_x = None
            right_x = None

        # Xử lý kết quả vạch trái
        if len(valid_left_centers) >= 3:
            det_lx = float(np.median(valid_left_centers))
            if self.smooth_left_x is None:
                self.smooth_left_x = det_lx
            else:
                self.smooth_left_x = 0.65 * det_lx + 0.35 * self.smooth_left_x
            left_x = self.smooth_left_x
        else:
            self.smooth_left_x = None
            left_x = None

        return left_x, right_x, mask, (y1, y2), detected_type

    def detect_right_line(self, img):
        left_x, right_x, mask, roi_y, detected_type = self.detect_lines(img)
        return right_x, mask, roi_y, detected_type

    def render_debug_frame(self, img, left_x, right_x, mask, roi_y, status_text, speed, steer, error, feature_type='LINE'):
        if img is None:
            return None

        h, w, _ = img.shape
        debug = img.copy()
        y1, y2 = roi_y
        mid_y = (y1 + y2) // 2

        if self.lane_follow_mode == 'LEFT':
            tgt_x = int(self.target_left_x)

            # 1. Khung ROI quét vạch trái
            cv2.rectangle(debug, (self.left_search_min, y1), (self.left_search_max, y2), (255, 255, 0), 2)
            cv2.putText(debug, f'Vung Quet Vach Trai [x: {self.left_search_min}..{self.left_search_max}]',
                        (self.left_search_min + 5, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 0), 1)

            # 2. Mốc chuẩn vạch trái
            cv2.line(debug, (tgt_x, y1 - 10), (tgt_x, y2 + 10), (255, 255, 0), 2)
            cv2.putText(debug, f'Moc Trai ({tgt_x}px)', (tgt_x - 45, y2 + 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

            # 3. Vẽ vạch trái phát hiện được
            if left_x is not None:
                lx = int(left_x)
                cv2.line(debug, (lx, y1), (lx, y2), (255, 200, 0), 3)
                cv2.circle(debug, (lx, mid_y), 8, (255, 200, 0), -1)
                cv2.putText(debug, f'Vach Trai ({lx}px)', (min(w - 175, lx - 55), y1 - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 200, 0), 1)
                cv2.arrowedLine(debug, (tgt_x, mid_y), (lx, mid_y), (255, 255, 0), 2, tipLength=0.2)

            # Nếu thấy cả vạch tim đường bên phải
            if right_x is not None:
                rx = int(right_x)
                cv2.line(debug, (rx, y1), (rx, y2), (0, 165, 255), 2)
                cv2.putText(debug, f'Vach Tim ({rx}px)', (rx - 40, y2 + 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 165, 255), 1)

        else:
            tgt_x = int(self.target_right_x)

            # 1. Vẽ khung ROI bên phải
            cv2.rectangle(debug, (self.right_search_min, y1), (self.right_search_max, y2), (0, 255, 0), 2)
            cv2.putText(debug, f'Vung Quet Vach/Ria Phai [x: {self.right_search_min}..{self.right_search_max}]',
                        (self.right_search_min + 5, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 0), 1)

            # 2. Làm mờ toàn bộ bên trái
            overlay_left = debug.copy()
            cv2.rectangle(overlay_left, (0, y1), (self.right_search_min, y2), (30, 30, 30), -1)
            cv2.addWeighted(overlay_left, 0.5, debug, 0.5, 0, debug)
            cv2.putText(debug, 'Ben trai & Nguoc chieu: BO QUA HOAN TOAN', (10, mid_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (160, 160, 160), 1)

            # 3. Vẽ MỐC CHUẨN MONG MUỐN & NGƯỠNG CHẠM KÍCH HOẠT
            cv2.line(debug, (tgt_x, y1 - 10), (tgt_x, y2 + 10), (0, 255, 255), 2)
            cv2.putText(debug, f'Moc ({tgt_x}px)', (tgt_x - 45, y2 + 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

            trig_x = int(self.post_tunnel_trigger_x)
            if self.has_passed_tunnel and not self.post_tunnel_maneuver_done:
                cv2.line(debug, (trig_x, y1 - 8), (trig_x, y2 + 8), (0, 0, 255), 2)
                cv2.putText(debug, f'Nguong Cham ({trig_x}px)', (trig_x - 85, y1 - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 0, 255), 1)
            elif not self.has_passed_tunnel:
                cv2.putText(debug, 'Truoc & Trong ham: Chi do lane', (self.right_search_min + 5, y1 - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (160, 160, 160), 1)

            # 4. Vẽ VẠCH PHẢI hoặc RÌA PHẢI DỐC CẦU
            if right_x is not None:
                rx = int(right_x)
                if feature_type == 'RAMP_EDGE':
                    line_color = (0, 140, 255)
                    label = f'Ria Doc Xam-Den ({rx}px)'
                else:
                    line_color = (0, 0, 255)
                    label = f'Vach Phai ({rx}px)'

                cv2.line(debug, (rx, y1), (rx, y2), line_color, 3)
                cv2.circle(debug, (rx, mid_y), 8, line_color, -1)
                cv2.putText(debug, label, (min(w - 175, rx - 55), y1 - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, line_color, 1)

                cv2.arrowedLine(debug, (tgt_x, mid_y), (rx, mid_y), (0, 255, 255), 2, tipLength=0.2)

        # Hiển thị trạng thái đặc biệt sau hầm hoặc mất mục tiêu
        if status_text.startswith('QUA HAM: DI THANG'):
            cv2.arrowedLine(debug, (w // 2, y2), (w // 2, y1), (0, 255, 0), 3, tipLength=0.25)
            cv2.putText(debug, status_text, (w // 2 - 130, mid_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 0), 2)
        elif status_text.startswith('QUA HAM: CUA PHAI'):
            cv2.arrowedLine(debug, (w // 2 - 40, mid_y), (w // 2 + 60, mid_y), (0, 165, 255), 3, tipLength=0.25)
            cv2.putText(debug, status_text, (w // 2 - 160, mid_y - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 165, 255), 2)
            cv2.putText(debug, 'KHOA DO LINE (XONG 60 DO MOI NHAN LAI)', (w // 2 - 180, mid_y + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 200, 255), 1)
        elif status_text.startswith('VUOT XE:'):
            if 'TRAI' in status_text:
                cv2.arrowedLine(debug, (w // 2 + 70, mid_y), (w // 2 - 70, mid_y), (0, 255, 255), 3, tipLength=0.25)
            elif 'PHAI' in status_text:
                cv2.arrowedLine(debug, (w // 2 - 70, mid_y), (w // 2 + 70, mid_y), (0, 255, 255), 3, tipLength=0.25)
            else:
                cv2.arrowedLine(debug, (w // 2 - 50, y2), (w // 2 - 50, y1), (0, 255, 255), 3, tipLength=0.25)
            cv2.putText(debug, status_text, (w // 2 - 170, mid_y - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 255), 2)
            cv2.putText(debug, 'NE XE DUNG - DANG CHAY LAN TRAI', (w // 2 - 160, mid_y + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1)
        elif (self.lane_follow_mode == 'RIGHT' and right_x is None) or (self.lane_follow_mode == 'LEFT' and left_x is None and right_x is None):
            cv2.arrowedLine(debug, (w // 2, y2), (w // 2, y1), (0, 255, 0), 3, tipLength=0.25)
            cv2.putText(debug, 'MAT MUC TIEU -> GIU LAI DI THANG', (w // 2 - 135, mid_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 5. Thanh thông số HUD phía trên
        overlay = debug.copy()
        cv2.rectangle(overlay, (0, 0), (w, 55), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.75, debug, 0.25, 0, debug)

        if status_text.startswith('VUOT XE:'):
            mode_name = 'NE XE DUNG (SANG LAN TRAI)'
        elif status_text.startswith('QUA HAM: CUA PHAI'):
            mode_name = 'KHOA DO LINE (DANG CUA 60 DO)'
        elif self.lane_follow_mode == 'LEFT':
            mode_name = 'BAM LANE TRAI'
        elif feature_type == 'RAMP_EDGE':
            mode_name = 'BAM RIA DOC CAU'
        else:
            mode_name = 'BAM VACH PHAI'

        tgt_disp = int(self.target_left_x) if self.lane_follow_mode == 'LEFT' else int(self.target_right_x)
        side_disp = 'Trai' if self.lane_follow_mode == 'LEFT' else 'Phai'
        cv2.putText(debug, f'Trang thai: {status_text} | Nhan dien: {mode_name}', (12, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1)
        cv2.putText(debug, f'Lech: {error:+.1f}px | Lai: {steer:+.2f} rad/s | Toc do: {speed:.2f} m/s | Moc {side_disp}: {tgt_disp}px',
                    (12, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)

        # 6. Khung nhỏ PiP Mask
        if mask is not None:
            pip_w, pip_h = 140, int(140 * (y2 - y1) / w)
            mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            pip = cv2.resize(mask_bgr, (pip_w, pip_h))
            px1, py1 = w - pip_w - 10, 8
            cv2.rectangle(debug, (px1 - 2, py1 - 2), (px1 + pip_w + 2, py1 + pip_h + 2), (0, 255, 255), 2)
            debug[py1:py1 + pip_h, px1:px1 + pip_w] = pip
            cv2.putText(debug, 'Mask Nhi phan', (px1, py1 + pip_h + 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 255), 1)

        return debug

    # ------------------------------------------------------------------------
    # VÒNG LẶP ĐIỀU KHIỂN CHÍNH
    # ------------------------------------------------------------------------

    def control(self):
        # 1. Kiểm tra an toàn bằng LiDAR phía trước (góc 18 độ tránh chạm thành hầm)
        front_obstacle = self.range_at(0, width_deg=18.0)
        is_blocked = front_obstacle < self.stop_distance

        # Xác nhận đã ra khỏi hầm: Xe bắt buộc phải lên nửa trên sa bàn (y >= 1.20m).
        # Toàn bộ khu vực trước hầm và trong hầm luôn có y <= 0.0m (cửa ra hầm tại y = 0.0m).
        if self.y >= 1.20:
            self.has_passed_tunnel = True

        # 2. Xử lý ảnh: BÁM VẠCH PHẢI / BÁM RÌA DỐC CẦU / BÁM VẠCH TRÁI
        left_x, right_x, mask, roi_y, feature_type = self.detect_lines(self.image)

        speed = 0.0
        steer = 0.0
        error_display = 0.0
        if not self.has_passed_tunnel:
            status_text = 'BAM RIA DOC CAU' if feature_type == 'RAMP_EDGE' else 'DO LANE TRUOC & TRONG HAM'
        else:
            status_text = 'BAM RIA DOC CAU' if feature_type == 'RAMP_EDGE' else ('BAM LANE TRAI' if self.lane_follow_mode == 'LEFT' else 'BAM VACH PHAI')

        # TOÀN BỘ CÁC LOGIC THỦ CÔNG CHỈ ĐƯỢC PHÉP KÍCH HOẠT SAU KHI ĐÃ ĐI QUA HẦM:
        if self.has_passed_tunnel:
            # Logic 1: Ngã ba sau hầm (khi vạch cam bị kéo lệch sang phải qua ngưỡng chạm 590px -> đi thẳng 3s -> ôm cua 60 độ)
            if not self.post_tunnel_maneuver_done and self.post_tunnel_state == 'IDLE':
                raw_err = (right_x - self.target_right_x) if right_x is not None else 0.0
                trigger_x = self.post_tunnel_trigger_x
                line_pulled_extreme = (right_x is not None) and (right_x >= trigger_x or raw_err >= (trigger_x - self.target_right_x))

                if line_pulled_extreme:
                    self.post_tunnel_trigger_count += 1
                    if self.post_tunnel_trigger_count >= 2:  # Đạt 2 frame (~0.1s) là kích hoạt ngay
                        self.post_tunnel_state = 'STRAIGHT'
                        self.post_tunnel_timer_start = time.time()
                        self.get_logger().info(
                            f'[KICH HOAT] Duong line cam cham nguong (rx={right_x:.1f}px >= {trigger_x:.0f}px, err={raw_err:+.1f}px) '
                            f'-> DI THANG {self.post_tunnel_straight_time:.1f}s -> CUA {self.post_tunnel_turn_target_deg:.0f} DO')
                else:
                    self.post_tunnel_trigger_count = 0

            # Logic 2: Vượt xe né xe dừng trên cao tốc
            # Chỉ kích hoạt sau khi đã ra khỏi hầm VÀ (đã xong ngã ba HOẶC đã ở trên đường cao tốc x < 0.5)
            is_highway = self.has_passed_tunnel and (self.post_tunnel_maneuver_done or self.x < 0.5)
            if not self.overtake_done and self.overtake_state == 'IDLE' and is_highway:
                if 0.20 <= front_obstacle <= self.overtake_trigger_dist:
                    self.overtake_state = 'OVERTAKE_STEER_LEFT'
                    self.overtake_timer_start = time.time()
                    self.get_logger().info(
                        f'[VUOT XE] Phat hien xe dung phia truoc o {front_obstacle:.2f}m <= {self.overtake_trigger_dist:.2f}m '
                        f'-> KICH HOAT NE XE QUA LAN TRAI!')

        # 0. Ưu tiên cao nhất: Chuỗi động tác VƯỢT XE NÉ XE DỪNG TRÊN CAO TỐC
        if self.overtake_state not in ('IDLE', 'DONE'):
            elapsed = time.time() - self.overtake_timer_start
            v_t = self.overtake_speed_turn
            v_s = self.overtake_speed_straight
            w_r = self.overtake_turn_rate

            if self.overtake_state == 'OVERTAKE_STEER_LEFT':
                if elapsed < self.overtake_steer_time:
                    speed = v_t
                    steer = w_r   # Bẻ lái sang trái (+w)
                    status_text = f'VUOT XE: BE LAI TRAI ({self.overtake_steer_time - elapsed:.1f}s)'
                    self.drive(speed, steer)
                else:
                    self.overtake_state = 'OVERTAKE_DIAG_LEFT'
                    self.overtake_timer_start = time.time()
                    self.drive(v_s, 0.0)
                    self.get_logger().info('[VUOT XE] Chuyen sang chay cheo sang lan trai...')

            elif self.overtake_state == 'OVERTAKE_DIAG_LEFT':
                if elapsed < self.overtake_diag_time:
                    speed = v_s
                    steer = 0.0   # Chạy chéo thẳng sang làn trái
                    status_text = f'VUOT XE: CHAY CHEO SANG TRAI ({self.overtake_diag_time - elapsed:.1f}s)'
                    self.drive(speed, steer)
                else:
                    self.overtake_state = 'OVERTAKE_STRAIGHTEN_LEFT'
                    self.overtake_timer_start = time.time()
                    self.drive(v_t, -w_r)
                    self.get_logger().info('[VUOT XE] Tra thang lai song song lan trai...')

            elif self.overtake_state == 'OVERTAKE_STRAIGHTEN_LEFT':
                if elapsed < self.overtake_steer_time:
                    speed = v_t
                    steer = -w_r  # Bẻ lái sang phải để trả thẳng song song trục đường (-w)
                    status_text = f'VUOT XE: TRA LAI SONG SONG ({self.overtake_steer_time - elapsed:.1f}s)'
                    self.drive(speed, steer)
                else:
                    self.overtake_state = 'OVERTAKE_FOLLOW_LEFT'
                    self.overtake_timer_start = time.time()
                    self.lane_follow_mode = 'RIGHT'
                    self.smooth_left_x = None
                    self.smooth_right_x = None
                    self.last_valid_steer = 0.0
                    self.get_logger().info(
                        f'[VUOT XE] Da vao lan trai -> KHOA DO LANE, DI THANG trong {self.overtake_left_follow_time:.1f}s qua mat xe dung...')

            elif self.overtake_state == 'OVERTAKE_FOLLOW_LEFT':
                if elapsed < self.overtake_left_follow_time:
                    # Đi thẳng trên làn trái, KHÔNG DÒ LANE theo yêu cầu
                    speed = v_s
                    steer = 0.0
                    self.last_valid_steer = 0.0
                    status_text = f'VUOT XE: DI THANG LAN TRAI ({self.overtake_left_follow_time - elapsed:.1f}s)'
                    self.drive(speed, 0.0)
                else:
                    # ĐÃ ĐI THẲNG QUA MẶT XE DỪNG -> ĐÁNH LÁI VỀ PHÍA VẠCH PHẢI!
                    self.overtake_state = 'OVERTAKE_STEER_RIGHT'
                    self.overtake_timer_start = time.time()
                    self.lane_follow_mode = 'RIGHT'
                    self.smooth_left_x = None
                    self.smooth_right_x = None
                    self.last_valid_steer = 0.0
                    self.drive(v_t, -w_r)
                    self.get_logger().info(
                        f'[VUOT XE] Da di thang lan trai {self.overtake_left_follow_time:.1f}s qua mat xe dung '
                        f'-> BAT DAU DANH LAI VE PHIA VACH PHAI ({self.overtake_steer_right_time:.1f}s)!')

            elif self.overtake_state == 'OVERTAKE_STEER_RIGHT':
                if elapsed < self.overtake_steer_right_time:
                    speed = v_t
                    steer = -w_r  # Bẻ lái sang phải để chuyển về làn ban đầu (-w)
                    status_text = f'VUOT XE: BE LAI PHAI VE LAN ({self.overtake_steer_right_time - elapsed:.1f}s)'
                    self.drive(speed, steer)
                else:
                    self.overtake_state = 'OVERTAKE_DIAG_RIGHT'
                    self.overtake_timer_start = time.time()
                    self.drive(v_s, 0.0)
                    self.get_logger().info('[VUOT XE] Chay cheo tro ve lan phai...')

            elif self.overtake_state == 'OVERTAKE_DIAG_RIGHT':
                if elapsed < self.overtake_diag_time:
                    speed = v_s
                    steer = 0.0   # Chạy chéo thẳng về làn phải
                    status_text = f'VUOT XE: CHAY CHEO VE PHAI ({self.overtake_diag_time - elapsed:.1f}s)'
                    self.drive(speed, steer)
                else:
                    self.overtake_state = 'OVERTAKE_STRAIGHTEN_RIGHT'
                    self.overtake_timer_start = time.time()
                    self.drive(v_t, w_r)
                    self.get_logger().info('[VUOT XE] Tra thang lai song song lan phai...')

            elif self.overtake_state == 'OVERTAKE_STRAIGHTEN_RIGHT':
                if elapsed < self.overtake_steer_right_time:
                    speed = v_t
                    steer = w_r   # Bẻ lái sang trái để trả thẳng song song vạch (+w)
                    status_text = f'VUOT XE: TRA LAI VE LAN PHAI ({self.overtake_steer_right_time - elapsed:.1f}s)'
                    self.drive(speed, steer)
                else:
                    self.overtake_state = 'DONE'
                    self.overtake_done = True
                    self.lane_follow_mode = 'RIGHT'
                    self.smooth_left_x = None
                    self.smooth_right_x = None
                    self.last_valid_steer = 0.0
                    self.prev_error = 0.0
                    self.d_error_filtered = 0.0
                    self.drive(self.straight_speed, 0.0)
                    self.get_logger().info(
                        '[VUOT XE] HOAN TAT VUOT XE! Da tro ve lan phai an toan -> TIEP TUC BAM LANE PHAI.')

        # 1. Ưu tiên phanh dừng nếu có vật cản ngoài chuỗi vượt xe
        elif is_blocked:
            self.stop()
            self.last_valid_steer = 0.0
            status_text = f'DUNG XE (Vat can {front_obstacle:.2f}m)'
            self.log_every(1.5, f'[CANH BAO] Vat can o {front_obstacle:.2f}m -> PHANH DUNG')

        # 2. Logic sau hầm - Giai đoạn 1: Tiếp tục đi thẳng trong 2 giây (tăng thêm 1s)
        elif self.post_tunnel_state == 'STRAIGHT':
            elapsed = time.time() - self.post_tunnel_timer_start
            straight_duration = self.post_tunnel_straight_time
            if elapsed < straight_duration:
                speed = self.straight_speed
                steer = 0.0
                self.last_valid_steer = 0.0
                status_text = f'QUA HAM: DI THANG ({straight_duration - elapsed:.1f}s)'
                self.drive(speed, steer)
                self.log_every(0.5, f'[QUA HAM] Tiep tuc di thang... con {straight_duration - elapsed:.1f}s')
            else:
                self.post_tunnel_state = 'TURN_RIGHT'
                self.post_tunnel_timer_start = time.time()
                self.post_tunnel_turn_start_yaw = self.yaw
                target_deg = self.post_tunnel_turn_target_deg
                self.get_logger().info(f'[QUA HAM] Het {straight_duration:.1f}s di thang -> Bat dau CUA VONG CUNG {target_deg:.0f} DO SANG PHAI (KHOA DO LINE)')
                speed = 0.08
                steer = -0.70
                status_text = f'QUA HAM: CUA PHAI {target_deg:.0f} DO (0/{target_deg:.0f} do)'
                self.drive(speed, steer)

        # 3. Logic sau hầm - Giai đoạn 2: Cua vòng cung sang phải 60 độ (KHÔNG nhận tín hiệu dò line)
        elif self.post_tunnel_state == 'TURN_RIGHT':
            elapsed = time.time() - self.post_tunnel_timer_start
            speed = 0.08
            steer = -0.70
            target_deg = self.post_tunnel_turn_target_deg

            # Tính góc đã quay sang phải (chuẩn hóa độ lệch yaw)
            delta_yaw = (self.yaw - self.post_tunnel_turn_start_yaw + math.pi) % (2.0 * math.pi) - math.pi
            turn_rad = -delta_yaw   # Quay phải: yaw giảm -> turn_rad dương
            turn_deg = math.degrees(turn_rad)

            status_text = f'QUA HAM: CUA PHAI {target_deg:.0f} DO ({max(0.0, turn_deg):.0f}/{target_deg:.0f} do)'
            self.drive(speed, steer)
            self.log_every(0.5, f'[QUA HAM] Dang om cua {target_deg:.0f} do (khoa do line)... goc quay: {turn_deg:.1f}/{target_deg:.0f} do ({elapsed:.1f}s)')

            max_turn_time = math.radians(target_deg) / 0.70 + 0.15
            turn_finished = (turn_deg >= (target_deg - 2.0)) or (elapsed >= max_turn_time)

            if turn_finished:
                self.post_tunnel_state = 'DONE'
                self.post_tunnel_maneuver_done = True
                self.smooth_right_x = None
                self.last_valid_steer = 0.0
                self.prev_error = 0.0
                self.d_error_filtered = 0.0
                self.get_logger().info(
                    f'[QUA HAM] Da om cua xong goc {target_deg:.0f} do ({turn_deg:.1f} do, {elapsed:.1f}s) '
                    '-> BAT DAU NHAN LAI TIN HIEU DO LANE!')

        # 4. Chế độ bám lane bình thường (Làn phải hoặc Làn trái)
        else:
            effective_error = None

            if self.lane_follow_mode == 'LEFT':
                status_text = 'BAM LANE TRAI'
                if left_x is not None and right_x is not None:
                    # Cả 2 vạch đều thấy: bám tâm làn (giữa vạch trái và vạch tim đường)
                    lane_center = (left_x + right_x) / 2.0
                    effective_error = lane_center - 320.0
                    status_text = f'BAM TAM LANE TRAI ({left_x:.0f}..{right_x:.0f}px)'
                elif left_x is not None:
                    # Chỉ thấy vạch trái: bám theo mốc target_left_x
                    effective_error = left_x - self.target_left_x
                    status_text = f'BAM VACH TRAI ({left_x:.0f}px)'
                elif right_x is not None:
                    # Chỉ thấy vạch tim đường bên phải: bám theo mốc target_right_x
                    effective_error = right_x - self.target_right_x
                    status_text = f'BAM VACH TIM ({right_x:.0f}px)'
            else:
                status_text = 'BAM RIA DOC CAU' if feature_type == 'RAMP_EDGE' else 'BAM VACH PHAI'
                if right_x is not None:
                    effective_error = right_x - self.target_right_x

            if effective_error is not None:
                raw_error = effective_error
                error_display = raw_error

                # Vùng chết Deadzone
                if abs(raw_error) <= self.deadzone:
                    error = 0.0
                else:
                    error = raw_error - math.copysign(self.deadzone, raw_error)

                # Lọc vi phân D
                dt = 1.0 / self.rate
                raw_de = (error - self.prev_error) / dt if dt > 0 else 0.0
                self.d_error_filtered = 0.60 * raw_de + 0.40 * self.d_error_filtered
                self.prev_error = error

                # Bộ điều khiển PD êm ái
                target_steer = - (self.kp * error + self.kd * self.d_error_filtered)

                # Bộ lọc tay lái mượt (35% góc mới + 65% góc cũ)
                filtered_steer = 0.35 * target_steer + 0.65 * self.last_valid_steer

                # Giới hạn gia tốc quay (Slew Rate Limiter)
                steer_diff = filtered_steer - self.last_valid_steer
                if abs(steer_diff) > self.max_steer_step:
                    filtered_steer = self.last_valid_steer + math.copysign(self.max_steer_step, steer_diff)

                steer = filtered_steer
                self.last_valid_steer = steer

                # Giảm tốc thích nghi khi cua gắt
                turn_ratio = min(1.0, abs(raw_error) / 90.0)
                speed = self.max_speed - (self.max_speed - self.min_speed) * turn_ratio

                self.drive(speed, steer)
                mode_desc = 'Lane trai' if self.lane_follow_mode == 'LEFT' else ('Ria doc' if feature_type == 'RAMP_EDGE' else 'Vach phai')
                self.log_every(2.0, f'Bam {mode_desc}: err={raw_error:+.1f}px | steer={steer:+.2f} rad/s | speed={speed:.2f} m/s')

            else:
                # MẤT MỤC TIÊU -> TIẾP TỤC ĐI THẲNG ỔN ĐỊNH
                status_text = 'MAT MUC TIEU -> DI THANG'
                speed = self.straight_speed
                steer = 0.0
                self.prev_error = 0.0
                self.d_error_filtered = 0.0
                self.last_valid_steer = 0.0
                self.smooth_right_x = None
                self.smooth_left_x = None
                self.drive(speed, 0.0)
                self.log_every(2.0, 'Mat muc tieu -> Giu goc lai 0.0, tiep tuc di thang...')

        # 3. Tạo khung hình trực quan & Hiển thị
        if self.image is not None and roi_y is not None:
            debug_frame = self.render_debug_frame(
                self.image, left_x, right_x, mask, roi_y, status_text, speed, steer, error_display, feature_type)

            # Phát ra ROS topic /camera/lane_debug
            if self.bridge is not None and debug_frame is not None:
                try:
                    debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, 'bgr8')
                    self.pub_lane_debug.publish(debug_msg)
                except Exception:
                    pass

            # Hiển thị trực tiếp nếu có giao diện
            if self.show_view and debug_frame is not None:
                try:
                    cv2.imshow('UEH CRC 2026 - Camera Do Lan (Lane Tracking)', debug_frame)
                    cv2.waitKey(1)
                except Exception as e:
                    self.get_logger().warn(f'Khong the hien thi cua so OpenCV: {e}')
                    self.show_view = False


def catch_sigterm():
    stopping = {'now': False}
    signal.signal(signal.SIGTERM, lambda *_: stopping.update(now=True))
    return stopping


def spin(node, stopping):
    while rclpy.ok() and not stopping['now']:
        try:
            rclpy.spin_once(node, timeout_sec=0.1)
        except Exception:
            if stopping['now'] or not rclpy.ok():
                break
            raise


def main(args=None):
    rclpy.init(args=args)
    stopping = catch_sigterm()
    node = Starter()
    try:
        spin(node, stopping)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
