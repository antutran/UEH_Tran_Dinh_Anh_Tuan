#!/usr/bin/env python3
"""Starter template for the UEH CRC 2026 simulation round.

There is no driving logic here. The node just collects sensor data, calls
control() 20 times a second, and forwards whatever you ask for to /cmd_vel.
Write your code in control(), at the bottom of the file.

    ros2 run crc_sim starter
    ros2 run crc_sim starter --ros-args -p max_speed:=0.15

As shipped it drives forward and stops when the LiDAR sees something close.
That is a smoke test, not a solution.
"""

import math
import signal
import time

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


class Starter(Node):

    def __init__(self):
        super().__init__('crc_starter')

        self.declare_parameter('max_speed', 0.12)       # m/s
        self.declare_parameter('max_turn', 1.0)         # rad/s
        self.declare_parameter('stop_distance', 0.35)   # m
        self.declare_parameter('rate', 20.0)            # Hz

        self.max_speed = self.get_parameter('max_speed').value
        self.max_turn = self.get_parameter('max_turn').value
        self.stop_distance = self.get_parameter('stop_distance').value
        rate = self.get_parameter('rate').value

        # Latest sensor data. All of these stay None until the first message.
        self.image = None       # BGR image, 480x640x3
        self.scan = None        # sensor_msgs/LaserScan
        self.x = self.y = self.yaw = 0.0
        self._last_log = {}

        self.bridge = CvBridge() if HAVE_CV else None
        if not HAVE_CV:
            self.get_logger().warn(
                'cv_bridge not found, self.image will stay None. '
                'apt install ros-humble-cv-bridge python3-opencv')

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Image, '/camera/image_raw',
                                 self.on_image, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan',
                                 self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)

        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            f'ready | max_speed={self.max_speed} m/s | {rate:.0f} Hz')

    # --- sensor callbacks ---------------------------------------------------

    def on_image(self, msg):
        if self.bridge is None:
            return
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'image conversion failed: {e}')

    def on_scan(self, msg):
        self.scan = msg

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x, self.y = p.x, p.y
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    # --- helpers ------------------------------------------------------------

    def range_at(self, angle_deg, width_deg=10.0):
        """Closest LiDAR return within +/- width_deg of angle_deg.

        0 is straight ahead, 90 is left, -90 is right. Returns inf if there is
        no data yet.
        """
        if self.scan is None or not self.scan.ranges:
            return float('inf')

        n = len(self.scan.ranges)
        best = float('inf')
        half = int(round(width_deg / 2.0))
        centre = int(round(angle_deg)) % 360
        for d in range(-half, half + 1):
            r = self.scan.ranges[int((centre + d) % 360 * n / 360)]
            if math.isfinite(r) and r > self.scan.range_min:
                best = min(best, r)
        return best

    def drive(self, v, w):
        """v in m/s forward, w in rad/s counter-clockwise. Both get clamped."""
        msg = Twist()
        msg.linear.x = float(max(-self.max_speed, min(self.max_speed, v)))
        msg.angular.z = float(max(-self.max_turn, min(self.max_turn, w)))
        self.pub_cmd.publish(msg)

    def stop(self):
        self.pub_cmd.publish(Twist())

    def log_every(self, seconds, text):
        """Rate-limited logging, so a message inside control() cannot spam."""
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

    # ------------------------------------------------------------------------
    # WRITE YOUR CODE BELOW
    # ------------------------------------------------------------------------

    def control(self):
        """Called at `rate` Hz.

        Read from: self.image, self.scan, self.range_at(deg), self.x/y/yaw
        Write with: self.drive(v, w), self.stop()
        """
        front = self.range_at(0, width_deg=30)

        if front < self.stop_distance:
            self.stop()
            self.log_every(2.0, f'obstacle at {front:.2f} m, stopping')
            return

        self.drive(self.max_speed, 0.0)

        # Where to go from here:
        #   Lane following - take the bottom third of self.image, threshold it,
        #   find the centroid of the white pixels, and steer on the error
        #   between that centroid and the image centre.
        #
        #   Obstacles - compare range_at(-30) against range_at(30) to pick a
        #   side before you commit to a turn.
        #
        #   Lights and signs - detect them in self.image. Subscribing to
        #   /traffic_lights during the graded run is not allowed.


def catch_sigterm():
    """Turn SIGTERM into a flag instead of letting rclpy tear down the context.

    Call this before building the node. rclpy's own handler invalidates the
    context, and if the signal lands mid-construction the constructor dies
    with "rcl node's context is invalid".
    """
    stopping = {'now': False}
    signal.signal(signal.SIGTERM, lambda *_: stopping.update(now=True))
    return stopping


def spin(node, stopping):
    while rclpy.ok() and not stopping['now']:
        try:
            rclpy.spin_once(node, timeout_sec=0.1)
        except Exception:
            # Shutting down mid-callback is normal; anything else is not.
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
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
