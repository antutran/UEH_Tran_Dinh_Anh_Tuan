#!/usr/bin/env python3
"""
Script dịch chuyển xe Waffle trong Gazebo ra vị trí giữa vòng cung sau hầm (hoặc tọa độ tùy chọn).
Mặc định:
  - Tọa độ giữa vòng cung: x = 4.00 m (ở giữa lane ngoài của khúc cua), y = 1.68 m
  - Góc quay: yaw = 180.0 độ (hướng Tây -X, chạy dọc theo làn cong)
"""

import math
import sys
import time
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Twist

def main():
    # Nhận tham số tùy chọn: x, y, yaw_deg
    target_x = float(sys.argv[1]) if len(sys.argv) > 1 else 4.00
    target_y = float(sys.argv[2]) if len(sys.argv) > 2 else 1.68
    target_yaw_deg = float(sys.argv[3]) if len(sys.argv) > 3 else 180.0
    target_z = 0.015

    yaw_rad = math.radians(target_yaw_deg)
    qz = math.sin(yaw_rad / 2.0)
    qw = math.cos(yaw_rad / 2.0)

    rclpy.init()
    node = Node('teleport_robot')
    
    print('--------------------------------------------------------------')
    print('  DANG DICH CHUYEN XE...')
    print(f'  - Muc tieu: x = {target_x:.2f} m, y = {target_y:.2f} m (Giua vong cung sau ham)')
    print(f'  - Huong: yaw = {target_yaw_deg:.1f} do (Huong Tay -X)')
    print('--------------------------------------------------------------')

    # 1. Triệt tiêu vận tốc dư thừa cũ (nếu xe đang chạy)
    cmd_pub = node.create_publisher(Twist, '/cmd_vel', 10)
    stop_msg = Twist()
    for _ in range(5):
        cmd_pub.publish(stop_msg)
        time.sleep(0.01)

    # 2. Gọi service /set_entity_state của Gazebo
    cli = node.create_client(SetEntityState, '/set_entity_state')
    if not cli.wait_for_service(timeout_sec=5.0):
        print('[LOI] Khong ket noi duoc voi Gazebo (/set_entity_state chua bat)!')
        print('      Hay chac chan ban da chay "1_start_gazebo.bat".')
        node.destroy_node()
        rclpy.shutdown()
        return

    req = SetEntityState.Request()
    req.state.name = 'waffle'
    req.state.reference_frame = 'world'
    
    req.state.pose.position.x = target_x
    req.state.pose.position.y = target_y
    req.state.pose.position.z = target_z
    req.state.pose.orientation.x = 0.0
    req.state.pose.orientation.y = 0.0
    req.state.pose.orientation.z = qz
    req.state.pose.orientation.w = qw

    req.state.twist.linear.x = 0.0
    req.state.twist.linear.y = 0.0
    req.state.twist.linear.z = 0.0
    req.state.twist.angular.x = 0.0
    req.state.twist.angular.y = 0.0
    req.state.twist.angular.z = 0.0

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

    if future.done() and future.result() is not None and future.result().success:
        print('==============================================================')
        print('  [THANH CONG] XE DA DUOC DICH CHUYEN RA GIUA VONG CUNG!')
        print(f'  * Toa do thuc te: X={target_x:.2f} m, Y={target_y:.2f} m (Lane ngoai)')
        print(f'  * Huong xe: {target_yaw_deg:.0f} do (Huong Tay -X)')
        print('  * Ban co the quan sat xe tren Gazebo hoac tiep tuc chay starter_node.')
        print('==============================================================')
    else:
        print('[LOI] Gazebo tu choi lenh dich chuyen hoac timeout!')

    # Dừng xe lần nữa cho ổn định
    for _ in range(3):
        cmd_pub.publish(stop_msg)
        time.sleep(0.01)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
