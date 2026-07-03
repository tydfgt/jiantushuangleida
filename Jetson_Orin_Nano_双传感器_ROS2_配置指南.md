# Jetson Orin Nano 配置 RPLidar C1(×2) + 10轴IMU 多传感器 ROS2 Humble 全流程

> 本文记录在 NVIDIA Jetson Orin Nano (L4T 36.5.0) 上从零配置两个 SLAMTEC RPLidar C1 激光雷达和一个 WIT 10轴 IMU 惯导模块的完整过程，包含 CH340 驱动编译、ROS2 驱动包构建、udev 规则配置、以及一键协同启动双雷达+IMU。

---

## 1. 设备环境

| 项目 | 规格 |
|------|------|
| **开发板** | NVIDIA Jetson Orin Nano Developer Kit (Super) |
| **系统** | Ubuntu 22.04 + L4T 36.5.0 (JetPack 6) |
| **内核** | 5.15.185-tegra |
| **ROS2** | Humble |
| **激光雷达** | SLAMTEC RPLidar C1 ×2 (USB/CP210x) |
| **IMU** | WIT 10-axis 惯导模块 (USB/CH340) |

---

## 2. 插上设备，先看串口

```bash
lsusb
# Bus 001 Device 005: ID 10c4:ea60 Silicon Labs CP210x UART Bridge  ← 雷达0
# Bus 001 Device 006: ID 10c4:ea60 Silicon Labs CP210x UART Bridge  ← 雷达1
# Bus 001 Device 004: ID 1a86:7523 QinHeng Electronics CH340         ← IMU

ls /dev/ttyUSB*   # ttyUSB0 ttyUSB1 (两个雷达)
# CH340 不出现 → 见第3节
```

CP210x（雷达）自动识别了，CH340（IMU）却没出现——Jetson tegra 内核不带 `ch341.ko`。

---

## 3. 解决 CH340 驱动（踩坑记录)

### 3.1 编译 WCH 官方驱动

> ⚠️ 不要用内核自带的 `ch341.c`，用 WCH 官方的 `ch341ser_linux` 更稳定，设备名为 `/dev/ttyCH341USB0`。

```bash
git clone https://github.com/WCHSoftGroup/ch341ser_linux.git
cd ch341ser_linux/driver
make
sudo make install   # 安装到 /lib/modules/.../usb/serial/ + depmod -a
```

### 3.2 杀手：brltty

驱动装好了，设备还是不出来？查 dmesg：

```
usb 1-2.1: usbfs: interface 0 claimed by ch341 while 'brltty' sets config #1
ch341-uart ttyUSB2: ch341-uart converter now disconnected
```

**`brltty`** 是盲文显示驱动，会抢占所有 USB 串口！必须卸载：

```bash
sudo apt purge brltty
```

卸载后重新插拔 IMU，`/dev/ttyCH341USB0` 出现了。

---

## 4. udev 规则（固定设备名）

### 4.1 RPLidar C1（双雷达）

两个相同型号的雷达，需要用序列号区分：

```bash
# 查看每个雷达的序列号
udevadm info -a -n /dev/ttyUSB0 | grep serial
# ATTRS{serial}=="b6079120cf6ff011a3ba90301045c30f"  → 雷达0
udevadm info -a -n /dev/ttyUSB1 | grep serial
# ATTRS{serial}=="dc1ff222d36ff011ba5d95301045c30f"  → 雷达1
```

创建按序列号绑定的 udev 规则：

```bash
# /etc/udev/rules.d/rplidar.rules
# 雷达0 — S/N: b6079120
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
  ATTRS{serial}=="b6079120cf6ff011a3ba90301045c30f", MODE:="0777", SYMLINK+="rplidar0"
# 雷达1 — S/N: dc1ff222
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
  ATTRS{serial}=="dc1ff222d36ff011ba5d95301045c30f", MODE:="0777", SYMLINK+="rplidar1"
```

```bash
sudo cp rplidar.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -la /dev/rplidar*  # rplidar0 → ttyUSB0, rplidar1 → ttyUSB1
```

> 💡 也可直接用 `/dev/serial/by-id/` 路径，天然唯一稳定。

### 4.2 10轴 IMU

```bash
# /etc/udev/rules.d/imu_usb.rules
KERNEL=="ttyCH341USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", SYMLINK+="imu_usb"
```

> 注意：WCH 驱动生成的是 `ttyCH341USB*` 而不是 `ttyUSB*`！

---

## 5. RPLidar C1 配置

### 5.1 克隆并构建 sllidar_ros2

```bash
cd ~/jiantu
git clone https://github.com/Slamtec/sllidar_ros2.git
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select sllidar_ros2
```

### 5.2 测试

```bash
# 健康检查
python3 -c "
import serial
s = serial.Serial('/dev/rplidar', 460800, timeout=1)
s.write(b'\xA5\x52')
import time; time.sleep(0.5)
print(s.read(64).hex())  # a55a...000000 = OK
s.close()
"
```

### 5.3 启动

```bash
source /opt/ros/humble/setup.bash
source ~/jiantu/install/setup.bash
ros2 launch sllidar_ros2 sllidar_c1_launch.py serial_port:=/dev/rplidar

# 看到: SLLidar health status : OK.
#       scan mode: Standard, sample rate: 5 Khz, scan frequency:10.0 Hz
```

---

## 6. 10轴 IMU 配置

### 6.1 通信协议

11 字节数据帧，`0x55` 帧头，校验和：

```
55 XX D0 D1 D2 D3 D4 D5 D6 D7 CK
  ↑                      ↑
帧头(0x55)              校验和 (前10字节累加取低8位)
数据类型: 0x51=加速度  0x52=角速度  0x53=姿态角  0x54=磁力计
```

### 6.2 构建 wit_ros2_imu

ROS2 驱动包需要小修改：串口改为 `/dev/ttyCH341USB0`，udev 规则改为 CH340 的 VID/PID。

```bash
cd ~/jiantu
# 解压 wit_ros2_imu.zip
unzip wit_ros2_imu.zip
# 修改 wit_ros2_imu/wit_ros2_imu/wit_ros2_imu.py:
#   port_name 参数生效 + 默认端口 → /dev/ttyCH341USB0
# 修改 imu_usb.rules:
#   KERNEL=="ttyCH341USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523"...

source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select wit_ros2_imu
```

### 6.3 启动

```bash
source ~/jiantu/install/setup.bash
ros2 launch wit_ros2_imu rviz_and_imu.launch.py

# 看到: Serial port opened successfully...
```

### 6.4 验证

```bash
ros2 topic echo /imu/data_raw --once
```

输出示例：

```yaml
orientation:
  x: -0.004
  y: -0.002
  z: 0.992
  w: 0.125
angular_velocity:
  x: 0.0
  y: 0.0
  z: 0.0
linear_acceleration:
  z: 0.00478   # 静止时接近 0
```

---

## 7. 多传感器协同 Launch 文件

双雷达 + IMU 一键启动。关键点：
- 两个雷达节点需**不同的 node name**，否则冲突
- 话题需 **remap** 避免两个雷达都发 `/scan`
- `serial_baudrate` 在 ROS2 Humble 中必须传**整数**，不能传字符串

```python
# wit_ros2_imu/launch/sensors.launch.py
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

        # 雷达0 → /scan_0, frame laser_0
        Node(
            package='sllidar_ros2', executable='sllidar_node',
            name='sllidar_node_0', output='screen',
            parameters=[{'channel_type': 'serial',
                         'serial_port': lidar0_port,
                         'serial_baudrate': 460800,   # 整数！
                         'frame_id': 'laser_0',
                         'inverted': False,
                         'angle_compensate': True,
                         'scan_mode': 'Standard'}],
            remappings=[('/scan', '/scan_0')],
        ),

        # 雷达1 → /scan_1, frame laser_1
        Node(
            package='sllidar_ros2', executable='sllidar_node',
            name='sllidar_node_1', output='screen',
            parameters=[{'channel_type': 'serial',
                         'serial_port': lidar1_port,
                         'serial_baudrate': 460800,   # 整数！
                         'frame_id': 'laser_1',
                         'inverted': False,
                         'angle_compensate': True,
                         'scan_mode': 'Standard'}],
            remappings=[('/scan', '/scan_1')],
        ),

        # IMU → /imu/data_raw
        Node(
            package='wit_ros2_imu', executable='wit_ros2_imu',
            name='imu', output='screen',
            parameters=[{'port': imu_port, 'baud': 9600}],
        ),
    ])
```

**一键启动：**

```bash
ros2 launch wit_ros2_imu sensors.launch.py

# 或指定端口
ros2 launch wit_ros2_imu sensors.launch.py \
    lidar0_port:=/dev/ttyUSB0 lidar1_port:=/dev/ttyUSB2
```

输出：

```
[sllidar_node_0] SLLidar S/N: D606...   health status : OK.  ← 雷达0 ✅
[sllidar_node_1] SLLidar S/N: F674...   health status : OK.  ← 雷达1 ✅
[imu]            Serial port opened successfully...           ← IMU  ✅
```

---

## 8. 最终话题一览

```bash
$ ros2 topic list
/imu/data_raw      # IMU: sensor_msgs/Imu (~100Hz, frame: imu_link)
/scan_0            # 雷达0: sensor_msgs/LaserScan (10Hz, 360°, frame: laser_0)
/scan_1            # 雷达1: sensor_msgs/LaserScan (10Hz, 360°, frame: laser_1)
/parameter_events  # 系统
/rosout            # 系统
```

---

## 9. 踩坑总结

| 坑 | 原因 | 解决 |
|----|------|------|
| CH340 不识别 | Jetson 内核无 `ch341.ko` | WCH 官方驱动 `ch341ser_linux` |
| 驱动装好设备还是消失 | `brltty` 抢占串口 | `sudo apt purge brltty` |
| udev 规则不生效 | WCH 驱动生成 `ttyCH341USB*` 而非 `ttyUSB*` | 规则匹配 `ttyCH341USB*` |
| IMU 节点报端口被占用 | 旧进程未退出 | `sudo fuser -k /dev/ttyCH341USB0` |
| launch 文件报 `node_executable` | ROS2 Humble 用 `executable` | 改为 `executable` |
| `serial_baudrate` 类型错误 | ROS2 Humble 参数类型校验严格 | 传整数 `460800` 不要传字符串 `'460800'` |
| 双雷达同时启动话题冲突 | 两个节点默认发同一个 `/scan` | 不同 node name + remap 到 `/scan_0` `/scan_1` |
| 同型号雷达区分不了 | 串口号可能变化 | 用 ATTRS{serial} 创建 udev 规则或直接用 by-id |

---

## 10. 参考链接

- [Slamtec sllidar_ros2](https://github.com/Slamtec/sllidar_ros2)
- [WCH CH341 官方 Linux 驱动](https://github.com/WCHSoftGroup/ch341ser_linux)
- [brltty CH340 问题参考](https://blog.insmtr.com/2025/04/24/2025.4.24%20CH340/)

---

> **硬件快照**: NVIDIA Jetson Orin Nano Super | L4T 36.5.0 | ROS2 Humble | RPLidar C1×2 (CP210x) + 10轴 IMU (CH340)
> **最后更新**: 2026-07-03
> **项目路径**: `~/jiantu/`
