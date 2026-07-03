#!/usr/bin/env python3
"""
双雷达 + IMU 协同启动文件
  雷达0: /dev/ttyUSB0 → topic /scan_0 → frame laser_0
  雷达1: /dev/ttyUSB1 → topic /scan_1 → frame laser_1
  IMU:   /dev/ttyCH341USB0 → topic /imu/data_raw → frame imu_link

用法:
  ros2 launch wit_ros2_imu sensors.launch.py
  ros2 launch wit_ros2_imu sensors.launch.py lidar0_port:=/dev/ttyUSB0 lidar1_port:=/dev/ttyUSB1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ========== 雷达0 ==========
    lidar0_port = LaunchConfiguration('lidar0_port', default='/dev/ttyUSB0')
    lidar0_frame = LaunchConfiguration('lidar0_frame', default='laser_0')

    # ========== 雷达1 ==========
    lidar1_port = LaunchConfiguration('lidar1_port', default='/dev/ttyUSB1')
    lidar1_frame = LaunchConfiguration('lidar1_frame', default='laser_1')

    # ========== IMU ==========
    imu_port = LaunchConfiguration('imu_port', default='/dev/ttyCH341USB0')

    return LaunchDescription([
        # ---- 参数声明 ----
        DeclareLaunchArgument('lidar0_port', default_value=lidar0_port),
        DeclareLaunchArgument('lidar0_frame', default_value=lidar0_frame),
        DeclareLaunchArgument('lidar1_port', default_value=lidar1_port),
        DeclareLaunchArgument('lidar1_frame', default_value=lidar1_frame),
        DeclareLaunchArgument('imu_port', default_value=imu_port),

        # ========== 雷达0 ==========
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node_0',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': lidar0_port,
                'serial_baudrate': 460800,
                'frame_id': lidar0_frame,
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }],
            remappings=[('/scan', '/scan_0')],
        ),

        # ========== 雷达1 ==========
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node_1',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': lidar1_port,
                'serial_baudrate': 460800,
                'frame_id': lidar1_frame,
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }],
            remappings=[('/scan', '/scan_1')],
        ),

        # ========== IMU ==========
        Node(
            package='wit_ros2_imu',
            executable='wit_ros2_imu',
            name='imu',
            output='screen',
            parameters=[{'port': imu_port, 'baud': 9600}],
        ),
    ])
