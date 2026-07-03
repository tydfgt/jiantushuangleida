#!/usr/bin/env python3
"""
独立版双雷达 + IMU + 时间同步 协同启动
用法: ros2 launch $(pwd)/launch/sensors_launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    lidar0_port = LaunchConfiguration('lidar0_port', default='/dev/ttyUSB0')
    lidar1_port = LaunchConfiguration('lidar1_port', default='/dev/ttyUSB1')
    imu_port    = LaunchConfiguration('imu_port', default='/dev/ttyCH341USB0')

    return LaunchDescription([
        DeclareLaunchArgument('lidar0_port', default_value=lidar0_port),
        DeclareLaunchArgument('lidar1_port', default_value=lidar1_port),
        DeclareLaunchArgument('imu_port', default_value=imu_port),

        # 雷达0 → /scan_0
        Node(package='sllidar_ros2', executable='sllidar_node',
             name='sllidar_node_0', output='screen',
             parameters=[{'channel_type': 'serial', 'serial_port': lidar0_port,
                          'serial_baudrate': 460800, 'frame_id': 'laser_0',
                          'inverted': False, 'angle_compensate': True,
                          'scan_mode': 'Standard'}],
             remappings=[('/scan', '/scan_0')]),

        # 雷达1 → /scan_1
        Node(package='sllidar_ros2', executable='sllidar_node',
             name='sllidar_node_1', output='screen',
             parameters=[{'channel_type': 'serial', 'serial_port': lidar1_port,
                          'serial_baudrate': 460800, 'frame_id': 'laser_1',
                          'inverted': False, 'angle_compensate': True,
                          'scan_mode': 'Standard'}],
             remappings=[('/scan', '/scan_1')]),

        # IMU → /imu/data_raw
        Node(package='wit_ros2_imu', executable='wit_ros2_imu',
             name='imu', output='screen',
             parameters=[{'port': imu_port, 'baud': 9600}]),

        # 时间同步 → /sync/*
        Node(package='wit_ros2_imu', executable='time_sync',
             name='time_sync', output='screen',
             parameters=[{'sync_slop': 0.05}]),
    ])
