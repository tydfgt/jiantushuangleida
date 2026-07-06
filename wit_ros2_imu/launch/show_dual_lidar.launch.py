#!/usr/bin/env python3
"""
双雷达 + IMU RViz 可视化启动文件（wit_ros2_imu 包版本）
  - 发布 laser_1 → laser_0 和 imu_link → laser_0 的静态 TF 变换
  - 启动 RViz2 并加载含双雷达+IMU的配置文件

用法:
  ros2 launch wit_ros2_imu show_dual_lidar.launch.py
  ros2 launch wit_ros2_imu show_dual_lidar.launch.py x:=0.3 y:=0.0 yaw:=1.57
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    x = LaunchConfiguration('x', default='0.0')
    y = LaunchConfiguration('y', default='0.0')
    z = LaunchConfiguration('z', default='0.0')
    yaw = LaunchConfiguration('yaw', default='0.0')

    return LaunchDescription([
        DeclareLaunchArgument('x', default_value=x,
                              description='雷达1 相对雷达0 X偏移(m)'),
        DeclareLaunchArgument('y', default_value=y,
                              description='雷达1 相对雷达0 Y偏移(m)'),
        DeclareLaunchArgument('z', default_value=z,
                              description='雷达1 相对雷达0 Z偏移(m)'),
        DeclareLaunchArgument('yaw', default_value=yaw,
                              description='雷达1 相对雷达0 偏航角(rad)'),

        # 静态 TF: laser_1 → laser_0（让两个雷达在同一坐标系下可见）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar1_to_lidar0_tf',
            arguments=[x, y, z, yaw, '0.0', '0.0', 'laser_0', 'laser_1'],
        ),

        # 静态 TF: imu_link → laser_0（让 IMU 在雷达坐标系下可视化）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_to_lidar0_tf',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0',
                       'laser_0', 'imu_link'],
        ),

        # RViz2 加载双雷达+IMU配置
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('wit_ros2_imu'),
                'rviz', 'dual_lidar.rviz'
            ])],
        ),
    ])
