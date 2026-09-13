#!/usr/bin/env python3
"""
UEH CRC 2026 – Contestant solution launch file.

Usage (with simulator already running):
    ros2 launch ueh_solution run.launch.py

Optional arguments:
    log_dir:=/tmp/crc_logs        where to save the data CSV
    log_enabled:=true             enable/disable data logging
    publish_debug:=false          publish debug images (/lane/debug etc.)

This file starts ONLY contestant nodes. It assumes:
    ros2 launch crc_sim sim.launch.py
has already been started in another terminal (or automatically by a wrapper).

NODE GRAPH:
    camera → lane_node     → /lane/error → behavior_node → /cmd_vel
    camera → sign_node     → /sign/detection
    camera → light_node    → /light/state
    camera + scan → ped_node → /pedestrian/blocking
    scan   → lidar_node    → /obstacle/front_dist, /obstacle/status
    behavior_node subscribes: /lane/error /obstacle/* /sign/* /light/* /pedestrian/*
    data_logger  subscribes: /odom /cmd_vel /lane/error /obstacle/front_dist ...
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('ueh_solution')
    params_file = os.path.join(pkg, 'config', 'params.yaml')

    log_dir_arg           = DeclareLaunchArgument('log_dir',           default_value='/tmp/crc_logs')
    log_enabled_arg       = DeclareLaunchArgument('log_enabled',       default_value='true')
    debug_arg             = DeclareLaunchArgument('publish_debug',     default_value='false')
    test_mode_arg         = DeclareLaunchArgument('test_mode',         default_value='',
                                                  description='Test mode: "" (normal competition) or "lane_only"')
    enable_pedestrian_arg = DeclareLaunchArgument('enable_pedestrian', default_value='true',
                                                  description='Enable pedestrian detection (true/false)')

    common_params = [
        params_file,
        {'use_sim_time': True},
    ]

    lane_node = Node(
        package='ueh_solution',
        executable='lane_node',
        name='lane_node',
        output='screen',
        parameters=common_params + [
            {'publish_debug': LaunchConfiguration('publish_debug')},
        ],
    )

    lidar_node = Node(
        package='ueh_solution',
        executable='lidar_node',
        name='lidar_node',
        output='screen',
        parameters=common_params,
    )

    sign_node = Node(
        package='ueh_solution',
        executable='sign_node',
        name='sign_node',
        output='screen',
        parameters=common_params + [
            {'publish_debug': LaunchConfiguration('publish_debug')},
        ],
    )

    light_node = Node(
        package='ueh_solution',
        executable='light_node',
        name='light_node',
        output='screen',
        parameters=common_params + [
            {'publish_debug': LaunchConfiguration('publish_debug')},
        ],
    )

    ped_node = Node(
        package='ueh_solution',
        executable='ped_node',
        name='ped_node',
        output='screen',
        parameters=common_params + [
            {'enable_pedestrian': LaunchConfiguration('enable_pedestrian')},
        ],
    )

    behavior_node = Node(
        package='ueh_solution',
        executable='behavior_node',
        name='behavior_node',
        output='screen',
        parameters=common_params + [
            {'test_mode':         LaunchConfiguration('test_mode')},
            {'enable_pedestrian': LaunchConfiguration('enable_pedestrian')},
        ],
    )

    data_logger = Node(
        package='ueh_solution',
        executable='data_logger',
        name='data_logger',
        output='screen',
        parameters=common_params + [
            {'log_dir':     LaunchConfiguration('log_dir')},
            {'log_enabled': LaunchConfiguration('log_enabled')},
        ],
    )

    return LaunchDescription([
        log_dir_arg,
        log_enabled_arg,
        debug_arg,
        test_mode_arg,
        enable_pedestrian_arg,
        lane_node,
        lidar_node,
        sign_node,
        light_node,
        ped_node,
        behavior_node,
        data_logger,
    ])
