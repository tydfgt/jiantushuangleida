#!/usr/bin/env python3
"""
传感器时间同步节点
  订阅: /scan_0, /scan_1, /imu/data_raw
  发布: /sync/scan_0, /sync/scan_1, /sync/imu (时间对齐后的数据)
  发布: /sync/status (同步状态: delay, drift)

原理: 使用 ApproximateTimeSynchronizer 按最近时间戳匹配多传感器数据。
      当所有传感器数据的时间戳在允许窗口内，同时转发出去。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import Float64MultiArray
from message_filters import ApproximateTimeSynchronizer, Subscriber


class SensorTimeSync(Node):
    def __init__(self):
        super().__init__('sensor_time_sync')

        # 同步窗口 (秒) — 传感器数据时间戳在此窗口内视为"同一时刻"
        self.declare_parameter('sync_slop', 0.05)  # 50ms
        sync_slop = self.get_parameter('sync_slop').value

        # 订阅器
        self.scan0_sub = Subscriber(self, LaserScan, '/scan_0')
        self.scan1_sub = Subscriber(self, LaserScan, '/scan_1')
        self.imu_sub = Subscriber(self, Imu, '/imu/data_raw')

        # 时间同步器 — 按最近时间戳对齐
        self.sync = ApproximateTimeSynchronizer(
            [self.scan0_sub, self.scan1_sub, self.imu_sub],
            queue_size=30,
            slop=sync_slop,
        )
        self.sync.registerCallback(self.sync_callback)

        # 发布器 — 时间对齐后的数据
        self.scan0_pub = self.create_publisher(LaserScan, '/sync/scan_0', 10)
        self.scan1_pub = self.create_publisher(LaserScan, '/sync/scan_1', 10)
        self.imu_pub = self.create_publisher(Imu, '/sync/imu', 10)
        self.status_pub = self.create_publisher(
            Float64MultiArray, '/sync/status', 10
        )

        self.get_logger().info(
            f'Time sync ready (slop={sync_slop}s) → /sync/*'
        )

    def sync_callback(self, scan0: LaserScan, scan1: LaserScan, imu: Imu):
        """当三传感器数据在 sync_slop 窗口内匹配时触发"""
        # 计算时间偏差
        t0 = scan0.header.stamp
        t1 = scan1.header.stamp
        ti = imu.header.stamp

        scan0_to_imu = (t0.sec - ti.sec) + (t0.nanosec - ti.nanosec) * 1e-9
        scan1_to_imu = (t1.sec - ti.sec) + (t1.nanosec - ti.nanosec) * 1e-9
        scan0_to_scan1 = (t0.sec - t1.sec) + (t0.nanosec - t1.nanosec) * 1e-9

        # 发布对齐后的数据
        self.scan0_pub.publish(scan0)
        self.scan1_pub.publish(scan1)
        self.imu_pub.publish(imu)

        # 发布同步状态 (延迟/漂移指标)
        status = Float64MultiArray()
        status.data = [scan0_to_imu, scan1_to_imu, scan0_to_scan1]
        self.status_pub.publish(status)


def main():
    rclpy.init()
    node = SensorTimeSync()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
