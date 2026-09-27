#!/usr/bin/env python3
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetEntityState

rclpy.init()
node = Node('test_overtake')
pub = node.create_publisher(Twist, '/cmd_vel', 10)
cli = node.create_client(GetEntityState, '/get_entity_state')

def get_pose():
    if not cli.wait_for_service(timeout_sec=2.0):
        return None
    req = GetEntityState.Request()
    req.name = 'waffle'
    req.reference_frame = 'world'
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
    if fut.done() and fut.result() is not None:
        p = fut.result().state.pose.position
        return p.x, p.y
    return None

p0 = get_pose()
print(f'Start pose: {p0}')

def drive_step(v, w, duration):
    t_end = time.time() + duration
    msg = Twist()
    msg.linear.x = float(v)
    msg.angular.z = float(w)
    while time.time() < t_end:
        pub.publish(msg)
        time.sleep(0.05)

print('--- Phase 1: Shift Left ---')
drive_step(0.10, +0.70, 0.70)
drive_step(0.12,  0.00, 1.00)
drive_step(0.10, -0.70, 0.70)
p1 = get_pose()
print(f'Pose after shift left: {p1}')

print('--- Phase 2: Pass straight in left lane ---')
drive_step(0.12, 0.00, 1.60)
p2 = get_pose()
print(f'Pose after passing: {p2}')

print('--- Phase 3: Shift Right ---')
drive_step(0.10, -0.70, 0.70)
drive_step(0.12,  0.00, 0.90)
drive_step(0.10, +0.70, 0.70)
p3 = get_pose()
print(f'Pose after shift right: {p3}')

pub.publish(Twist())
node.destroy_node()
rclpy.shutdown()
