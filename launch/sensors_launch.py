#!/usr/bin/env python3
"""
协同启动文件：同时启动 RPLidar C1 激光雷达 + 10轴 IMU 惯导模块
用法:
  ros2 launch launch sensors_launch.py
  ros2 launch launch sensors_launch.py lidar_port:=/dev/ttyUSB1 imu_port:=/dev/ttyCH341USB0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ========== RPLidar C1 参数 ==========
    lidar_port = LaunchConfiguration('lidar_port', default='/dev/ttyUSB1')
    lidar_baudrate = LaunchConfiguration('lidar_baudrate', default='460800')
    lidar_frame_id = LaunchConfiguration('lidar_frame_id', default='laser')

    # ========== 10-axis IMU 参数 ==========
    imu_port = LaunchConfiguration('imu_port', default='/dev/ttyCH341USB0')
    imu_baudrate = LaunchConfiguration('imu_baudrate', default='9600')
    imu_frame_id = LaunchConfiguration('imu_frame_id', default='imu_link')

    return LaunchDescription([
        # ---- 雷达参数声明 ----
        DeclareLaunchArgument(
            'lidar_port', default_value=lidar_port,
            description='RPLidar C1 串口设备'),

        DeclareLaunchArgument(
            'lidar_baudrate', default_value=lidar_baudrate,
            description='RPLidar C1 波特率'),

        DeclareLaunchArgument(
            'lidar_frame_id', default_value=lidar_frame_id,
            description='RPLidar 坐标系 ID'),

        # ---- IMU 参数声明 ----
        DeclareLaunchArgument(
            'imu_port', default_value=imu_port,
            description='10轴 IMU 串口设备'),

        DeclareLaunchArgument(
            'imu_baudrate', default_value=imu_baudrate,
            description='10轴 IMU 波特率'),

        DeclareLaunchArgument(
            'imu_frame_id', default_value=imu_frame_id,
            description='IMU 坐标系 ID'),

        # ========== RPLidar C1 节点 ==========
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[{
                'serial_port': lidar_port,
                'serial_baudrate': lidar_baudrate,
                'frame_id': lidar_frame_id,
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }],
        ),

        # ========== 10-axis IMU 节点 ==========
        Node(
            package='wit_ros2_imu',
            executable='wit_ros2_imu',
            name='imu',
            output='screen',
            parameters=[{
                'port': imu_port,
                'baud': imu_baudrate,
            }],
        ),
    ])
