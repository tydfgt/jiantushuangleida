# jiantushuangleida — Jetson Orin Nano 双雷达 + IMU ROS2 传感器平台

[![Platform](https://img.shields.io/badge/Platform-Jetson%20Orin%20Nano-green)](https://www.nvidia.com/jetson)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

基于 **NVIDIA Jetson Orin Nano Super** 的多传感器 ROS2 Humble 平台，一键启动双 RPLidar C1 激光雷达 + 10轴 IMU。

---

## 🚀 快速开始

```bash
# 1. 安装依赖
sudo apt purge brltty          # 必做：卸载盲文驱动
sudo apt install python3-pip python3-serial
pip3 install pyserial

# 2. 安装 CH340 驱动（如果内核不带 ch341）
git clone https://github.com/WCHSoftGroup/ch341ser_linux.git /tmp/ch341
cd /tmp/ch341/driver && make && sudo make install

# 3. 安装 udev 规则（固定串口名）
sudo cp sllidar_ros2/scripts/rplidar.rules /etc/udev/rules.d/
sudo cp wit_ros2_imu/imu_usb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

# 4. 构建
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 5. 启动所有传感器
source install/setup.bash
ros2 launch wit_ros2_imu sensors.launch.py

# 6. 启动 RViz 可视化（需有桌面环境）
source install/setup.bash
ros2 launch wit_ros2_imu show_dual_lidar.launch.py
```

启动后话题一览：

| 话题 | 传感器 | 频率 | 消息类型 |
|------|--------|------|----------|
| `/scan_0` | RPLidar C1 #0 | 10 Hz | `sensor_msgs/LaserScan` |
| `/scan_1` | RPLidar C1 #1 | 10 Hz | `sensor_msgs/LaserScan` |
| `/imu/data_raw` | 10轴 IMU | ~10 Hz | `sensor_msgs/Imu` |

---

## 🎨 RViz 可视化

项目预置了 RViz2 配置文件，可一键查看双雷达点云 + IMU 姿态：

```bash
# 终端 1：启动传感器
ros2 launch wit_ros2_imu sensors.launch.py

# 终端 2：启动可视化
ros2 launch wit_ros2_imu show_dual_lidar.launch.py

# 自定义雷达安装位置（x y z yaw 单位：米/弧度）
ros2 launch wit_ros2_imu show_dual_lidar.launch.py x:=0.3 y:=0.0 yaw:=1.57
```

RViz 显示说明：

| 显示项 | 颜色 | 话题 | 帧 |
|--------|------|------|-----|
| LaserScan_0 | 🔴 红色 | `/scan_0` | `laser_0` |
| LaserScan_1 | 🟢 绿色 | `/scan_1` | `laser_1` → `laser_0` |
| IMU (Axes) | 🟡 坐标轴 | `/imu/data_raw` | `imu_link` → `laser_0` |

> 💡 如遇到 `could not connect to display` 错误，说明当前为 SSH 无桌面环境，请在 Jetson 本地桌面或使用 `ssh -X` 转发后运行。

---

## 📦 项目结构

```
jiantu/
├── sllidar_ros2/          # RPLidar ROS2 驱动 (Slamtec 官方)
│   ├── launch/            #   sllidar_c1_launch.py 等
│   ├── sdk/               #   RPLidar SDK v2.1.0
│   └── src/               #   sllidar_node C++ 源码
├── wit_ros2_imu/          # 10轴 IMU ROS2 驱动 (WIT)
│   ├── launch/
│   │   ├── rviz_and_imu.launch.py   # 单独 IMU 启动
│   │   ├── sensors.launch.py        # ★ 双雷达+IMU 协同启动
│   │   └── show_dual_lidar.launch.py # ★ 双雷达+IMU RViz 可视化
│   ├── rviz/
│   │   └── dual_lidar.rviz          # ★ RViz 预置配置
│   ├── wit_ros2_imu/      #   imu 节点 Python 源码
│   ├── imu_usb.rules      #   CH340 udev 规则
│   └── bind_usb.sh        #   USB 绑定脚本
├── launch/                # 独立 launch 文件
│   ├── sensors_launch.py  #   独立版传感器协同启动
│   └── show_dual_lidar.launch.py  # 独立版 RViz 可视化
├── rviz/
│   └── dual_lidar.rviz    #  独立版 RViz 配置
├── 10-axis_IMU_Module/    # IMU 原始资料 & 协议文档
│   ├── 3.Basic application/
│   ├── 5.Communication protocol/
│   └── ...
├── 机器状况详细报告.md      # 机器完整配置报告
└── Jetson_Orin_Nano_双传感器_ROS2_配置指南.md  # CSDN 教程文章
```

---

## 🔧 串口设备对照

| 串口 | 芯片 | 传感器 | udev 别名 |
|------|------|--------|-----------|
| `/dev/ttyUSB0` | CP210x | RPLidar C1 #0 | `/dev/rplidar` |
| `/dev/ttyUSB1` | CP210x | RPLidar C1 #1 | — |
| `/dev/ttyCH341USB0` | CH340 | 10轴 IMU | `/dev/imu_usb` |

验证：
```bash
ls -la /dev/ttyUSB* /dev/ttyCH34* /dev/rplidar /dev/serial/by-id/
```

---

## 🐛 常见问题

| 症状 | 解决 |
|------|------|
| CH340 不出现 `/dev/ttyCH341*` | `sudo apt purge brltty` 后重插 |
| IMU 报 `multiple access on port` | `sudo fuser -k /dev/ttyCH341USB0` |
| 雷达报 `80008004` / 超时 | `sudo modprobe -r cp210x && sudo modprobe cp210x` |
| 构建报 `serial_baudrate` 类型错误 | 参数值用整数 `460800` 而非字符串 |

---

## 📚 文档

- [机器状况详细报告](./机器状况详细报告.md) — 硬件/系统/环境完整报告
- [交接文档](./交接文档.md) — 面向 AI 编程的详细项目说明
- [配置指南](./Jetson_Orin_Nano_双传感器_ROS2_配置指南.md) — CSDN 发布版教程

---

## 🖥️ 硬件配置

| 项目 | 规格 |
|------|------|
| 开发板 | NVIDIA Jetson Orin Nano Super (P3767-0005) |
| SoC | Tegra234, 6核 Cortex-A78AE |
| 内存 | 8GB 统一内存 + 8GB SSD swap |
| 系统 | Ubuntu 22.04, L4T 36.5.0, JetPack 6 |
| 雷达1 | SLAMTEC RPLidar C1 |
| 雷达2 | SLAMTEC RPLidar C1 |
| IMU | WIT 10-axis (加速度+角速度+姿态+磁力计) |

---

## 📄 License

MIT
