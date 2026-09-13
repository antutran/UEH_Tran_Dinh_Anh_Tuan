#!/usr/bin/env python3
"""LiDAR safety node for UEH CRC 2026.

Subscribes to: /scan
Publishes to:
  /obstacle/front_dist   (std_msgs/Float32)  – closest distance in front arc
  /obstacle/status       (std_msgs/Int32)     – 0=clear, 1=slow, 2=stop

Does NOT subscribe to any forbidden topics.
Does NOT hard-code track coordinates.
"""

import math
import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Int32

STATUS_CLEAR = 0
STATUS_SLOW  = 1
STATUS_STOP  = 2


def catch_sigterm():
    stopping = {'now': False}
    signal.signal(signal.SIGTERM, lambda *_: stopping.update(now=True))
    return stopping


class LidarNode(Node):

    def __init__(self):
        super().__init__('lidar_node')

        self.declare_parameter('emergency_stop_dist', 0.25)
        self.declare_parameter('slow_down_dist',      0.55)
        self.declare_parameter('front_arc_deg',       25.0)

        self.estop_dist  = self.get_parameter('emergency_stop_dist').value
        self.slow_dist   = self.get_parameter('slow_down_dist').value
        self.arc_half    = self.get_parameter('front_arc_deg').value / 2.0

        self.scan = None

        self.pub_dist   = self.create_publisher(Float32, '/obstacle/front_dist', 10)
        self.pub_status = self.create_publisher(Int32,   '/obstacle/status', 10)

        self.create_subscription(
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)

        self.create_timer(0.05, self._tick)   # 20 Hz
        self.get_logger().info('lidar_node ready')

    def _on_scan(self, msg):
        self.scan = msg

    def _front_distance(self):
        """Return minimum LiDAR range in the front arc (±front_arc_deg)."""
        if self.scan is None or not self.scan.ranges:
            return float('inf')

        msg  = self.scan
        n    = len(msg.ranges)
        best = float('inf')

        half_steps = int(round(self.arc_half * n / 360.0))
        for i in range(-half_steps, half_steps + 1):
            idx = i % n
            r   = msg.ranges[idx]
            if math.isfinite(r) and msg.range_min < r < msg.range_max:
                best = min(best, r)

        return best

    def _tick(self):
        d = self._front_distance()

        dist_msg = Float32()
        dist_msg.data = float(d)
        self.pub_dist.publish(dist_msg)

        status_msg = Int32()
        if d <= self.estop_dist:
            status_msg.data = STATUS_STOP
        elif d <= self.slow_dist:
            status_msg.data = STATUS_SLOW
        else:
            status_msg.data = STATUS_CLEAR
        self.pub_status.publish(status_msg)


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
    node = LidarNode()
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
