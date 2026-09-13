#!/usr/bin/env python3
"""Data logger node for UEH CRC 2026.

Subscribes to:
  /odom                  – x, y, v, w
  /cmd_vel               – commanded v, w
  /lane/error            – lane error in pixels
  /obstacle/front_dist   – LiDAR front distance
  /sign/detection        – detected sign
  /light/state           – detected light
  /pedestrian/blocking   – pedestrian status

Writes a CSV log to log_dir/run_<timestamp>.csv
Used to generate analysis plots without fabricating results.

Does NOT subscribe to any forbidden topics.
"""

import csv
import math
import os
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


def catch_sigterm():
    stopping = {'now': False}
    signal.signal(signal.SIGTERM, lambda *_: stopping.update(now=True))
    return stopping


class DataLogger(Node):

    def __init__(self):
        super().__init__('data_logger')

        self.declare_parameter('log_enabled', True)
        self.declare_parameter('log_dir',     '/tmp/crc_logs')
        self.declare_parameter('log_rate_hz', 10.0)

        self.enabled  = self.get_parameter('log_enabled').value
        log_dir       = self.get_parameter('log_dir').value
        rate          = float(self.get_parameter('log_rate_hz').value)

        # Latest values
        self.odom_x   = 0.0
        self.odom_y   = 0.0
        self.odom_v   = 0.0
        self.odom_w   = 0.0
        self.cmd_v    = 0.0
        self.cmd_w    = 0.0
        self.lane_err = 0.0
        self.front_d  = float('inf')
        self.sign     = 'none'
        self.light    = 'NONE'
        self.ped      = False
        self.distance = 0.0
        self._prev_xy = None
        self.t0       = time.time()

        if self.enabled:
            os.makedirs(log_dir, exist_ok=True)
            ts   = time.strftime('%Y%m%d_%H%M%S')
            path = os.path.join(log_dir, f'run_{ts}.csv')
            self._file = open(path, 'w', newline='')
            self._csv  = csv.writer(self._file)
            self._csv.writerow([
                'time_s', 'odom_x', 'odom_y', 'distance_m',
                'odom_v', 'odom_w', 'cmd_v', 'cmd_w',
                'lane_error_px', 'front_dist_m',
                'sign', 'light', 'pedestrian'])
            self.get_logger().info(f'data_logger writing to {path}')
        else:
            self._file = None
            self._csv  = None

        # Subscriptions (all permitted topics)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(Twist,    '/cmd_vel', self._on_cmd, 10)
        self.create_subscription(
            Float32, '/lane/error',
            lambda m: setattr(self, 'lane_err', m.data), 10)
        self.create_subscription(
            Float32, '/obstacle/front_dist',
            lambda m: setattr(self, 'front_d', m.data), 10)
        self.create_subscription(
            String, '/sign/detection',
            lambda m: setattr(self, 'sign', m.data), 10)
        self.create_subscription(
            String, '/light/state',
            lambda m: setattr(self, 'light', m.data), 10)
        self.create_subscription(
            Bool, '/pedestrian/blocking',
            lambda m: setattr(self, 'ped', m.data), 10)

        self.create_timer(1.0 / rate, self._tick)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        v = msg.twist.twist
        x, y = p.x, p.y
        if self._prev_xy is not None:
            dx = x - self._prev_xy[0]
            dy = y - self._prev_xy[1]
            self.distance += math.hypot(dx, dy)
        self._prev_xy = (x, y)
        self.odom_x = x
        self.odom_y = y
        self.odom_v = v.linear.x
        self.odom_w = v.angular.z

    def _on_cmd(self, msg):
        self.cmd_v = msg.linear.x
        self.cmd_w = msg.angular.z

    def _tick(self):
        if not self.enabled or self._csv is None:
            return
        t = time.time() - self.t0
        self._csv.writerow([
            f'{t:.2f}',
            f'{self.odom_x:.4f}',  f'{self.odom_y:.4f}',
            f'{self.distance:.4f}',
            f'{self.odom_v:.4f}',  f'{self.odom_w:.4f}',
            f'{self.cmd_v:.4f}',   f'{self.cmd_w:.4f}',
            f'{self.lane_err:.2f}',
            f'{self.front_d:.3f}',
            self.sign, self.light,
            int(self.ped)])
        self._file.flush()

    def destroy_node(self):
        if self._file is not None:
            try:
                self._file.close()
            except Exception:
                pass
        super().destroy_node()


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
    node = DataLogger()
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
