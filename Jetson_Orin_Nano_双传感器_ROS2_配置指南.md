# Jetson Orin Nano 配置 RPLidar C1(×2) + 10轴IMU 多传感器 ROS2 Humble 全流程

> 本文记录在 NVIDIA Jetson Orin Nano (L4T 36.5.0) 上从零配置两个 SLAMTEC RPLidar C1 激光雷达和一个 WIT 10轴 IMU 惯导模块的完整过程。涵盖 CH340 驱动编译、brltty 冲突解决、udev 规则、ROS2 驱动包构建、双雷达话题隔离、传感器时间同步、RViz 可视化以及一键协同启动——每一步都有可直接复制执行的命令和验证方法。
>
> 📦 项目仓库: [Gitee](https://gitee.com/tydfgt/jiantushuangleida) | [GitHub](https://github.com/tydfgt/jiantushuangleida)

---

## 目录

- [1. 设备环境](#1-设备环境)
- [2. 硬件连接与串口识别](#2-硬件连接与串口识别)
- [3. CH340 驱动编译与 brltty 冲突](#3-ch340-驱动编译与-brltty-冲突)
- [4. udev 规则](#4-udev-规则固定设备名)
- [5. RPLidar C1 配置](#5-rplidar-c1-配置)
- [6. 10轴 IMU 配置](#6-10轴-imu-配置)
- [7. 协同 Launch + 时间同步](#7-多传感器协同-launch--时间同步)
- [8. 话题总览与数据验证](#8-话题总览与数据验证)
- [9. 踩坑总结](#9-踩坑总结)
- [10. RViz 可视化双雷达+IMU](#10-rviz-可视化双雷达imu)
- [11. 参考链接](#11-参考链接)

---

## 1. 设备环境

| 项目 | 规格 | 获取命令 |
|------|------|----------|
| **开发板** | NVIDIA Jetson Orin Nano Developer Kit (Super) | `cat /sys/devices/soc0/machine` |
| **SoC** | Tegra234, 6× Cortex-A78AE + Ampere GPU | `cat /proc/cpuinfo` |
| **内存** | 8GB 统一内存 + 8GB SSD swap | `free -h` |
| **系统** | Ubuntu 22.04 | `lsb_release -a` |
| **L4T** | 36.5.0 (JetPack 6) | `cat /etc/nv_tegra_release` |
| **内核** | 5.15.185-tegra | `uname -r` |
| **ROS2** | Humble | `echo $ROS_DISTRO` |
| **激光雷达** | SLAMTEC RPLidar C1 ×2 (USB/CP210x, 460800bps) | — |
| **IMU** | WIT 10-axis (加速度+角速度+姿态+磁力计, 9600bps) | — |

### 系统架构

```
┌─────────────────────────────────────────────────┐
│              Jetson Orin Nano                     │
│  ┌──────────┐ ┌──────────┐ ┌──────────────────┐  │
│  │ RPLidar  │ │ RPLidar  │ │  10-axis IMU     │  │
│  │ C1 #0    │ │ C1 #1    │ │  (CH340)         │  │
│  │ CP210x   │ │ CP210x   │ │  9600 bps        │  │
│  └────┬─────┘ └────┬─────┘ └────────┬─────────┘  │
│       │USB         │USB             │USB          │
│  ┌────▼────────────▼───────────────▼──────────┐   │
│  │         ROS2 Humble 驱动层                  │   │
│  │  sllidar_ros2   sllidar_ros2   wit_ros2_imu │   │
│  └────┬────────────┬───────────────┬──────────┘   │
│       │/scan_0     │/scan_1        │/imu/data_raw  │
│  ┌────▼────────────▼───────────────▼──────────┐   │
│  │           time_sync 时间同步                │   │
│  └────┬────────────┬───────────────┬──────────┘   │
│       │/sync/*     │               │              │
│       ▼            ▼               ▼              │
│    SLAM / 建图 / 导航 / rosbag 录制               │
└─────────────────────────────────────────────────┘
```

---

## 2. 硬件连接与串口识别

### 2.1 物理连接

所有传感器通过 USB Hub 连接至 Jetson，建议使用**带外部供电的 USB 3.0 Hub**，两个雷达同时工作时对供电要求较高。

```
Jetson USB-A ──┬── RPLidar C1 #0 (自带 Type-C 转 USB 线)
               ├── RPLidar C1 #1 (自带 Type-C 转 USB 线)
               └── 10轴 IMU    (Type-C 数据线)
```

### 2.2 查看 USB 设备

```bash
lsusb
# Bus 001 Device 004: ID 1a86:7523 QinHeng Electronics CH340 serial converter
# Bus 001 Device 005: ID 10c4:ea60 Silicon Labs CP210x UART Bridge  ← 雷达0
# Bus 001 Device 006: ID 10c4:ea60 Silicon Labs CP210x UART Bridge  ← 雷达1
```

| USB ID | 芯片 | 设备 | 内核驱动 |
|--------|------|------|----------|
| `10c4:ea60` | Silicon Labs CP2102N | RPLidar C1 | `cp210x`（内核自带） |
| `1a86:7523` | QinHeng CH340 | 10轴 IMU | `ch341`（需手动编译） |

### 2.3 查看串口设备

```bash
ls -la /dev/ttyUSB* /dev/ttyCH34*
# /dev/ttyUSB0  ← 雷达0 (CP210x)
# /dev/ttyUSB1  ← 雷达1 (CP210x)
# CH340 没有出现 → 进入第3节
```

> 💡 也可通过 `/dev/serial/by-id/` 查看，路径中包含芯片序列号，天然唯一，适合在 launch 文件中硬编码。

```bash
ls -la /dev/serial/by-id/
# usb-Silicon_Labs_CP2102N_..._b6079120... -> ../../ttyUSB0
# usb-Silicon_Labs_CP2102N_..._dc1ff222... -> ../../ttyUSB1
```

---

## 3. CH340 驱动编译与 brltty 冲突

> ⚠️ 这是本文最容易踩坑的部分。Jetson tegra 内核 **不包含** `ch341.ko` 模块，且 Ubuntu 默认安装的 `brltty` 会抢占串口。

### 3.1 为什么 Jetson 内核没有 CH340 驱动？

```bash
# 检查内核是否带 ch341 模块
ls /lib/modules/$(uname -r)/kernel/drivers/usb/serial/ch341*
# 输出: No such file or directory  ← 确认缺失

# lsmod 也看不到
lsmod | grep ch341
# (空)
```

Jetson 的 tegra 内核是 NVIDIA 定制编译的，仅包含 Jetson 硬件必需的驱动模块。CH340 这种通用 USB 串口芯片的驱动被裁剪掉了。

### 3.2 方案选择：WCH 官方驱动 vs 内核源码

| | 内核 ch341.c | WCH ch341ser_linux |
|--|-------------|-------------------|
| 来源 | Linux 主线内核 | WCH 官方 GitHub |
| 设备节点 | `/dev/ttyUSBx` | `/dev/ttyCH341USB0` |
| 稳定性 | 对 CH340 变体兼容性一般 | 官方维护，兼容所有 CH34x 变体 |
| 与 CP210x 区分 | 容易混淆（都是 ttyUSBx） | 名称一目了然 |

**结论：用 WCH 官方驱动。** 设备名 `ttyCH341USB0` 不会和雷达的 `ttyUSB0/1` 混淆。

### 3.3 编译安装

```bash
# 确保编译工具链可用
sudo apt install -y git make gcc

# 克隆 WCH 官方驱动
git clone https://github.com/WCHSoftGroup/ch341ser_linux.git /tmp/ch341
cd /tmp/ch341/driver

# 编译（注意编译器版本警告可忽略）
make
# 输出: === The target driver file has been generated ===
#       -rw-rw-r-- ... ch341.ko

# 安装到内核模块目录
sudo make install
# 执行: cp ch341.ko → /lib/modules/.../usb/serial/
#       depmod -a（更新模块依赖）

# 验证安装
modinfo ch341 | head -5
# filename: /lib/modules/5.15.185-tegra/kernel/drivers/usb/serial/ch341.ko
# description: WCH CH341 USB serial driver
```

> 💡 `make install` 会自动执行 `depmod -a`，确保**开机自动加载**。下次重启后插入 IMU 即可自动识别。

### 3.4 brltty：看不见的串口杀手

驱动装好了，重新插拔 IMU，`/dev/ttyCH341USB0` 还是没出现？查内核日志：

```bash
dmesg | grep -i "ch34\|brltty" | tail -10
```

如果看到类似输出：

```
ch341 1-2.1:1.0: ch341-uart converter detected
usb 1-2.1: ch341-uart converter now attached to ttyUSB2
usb 1-2.1: usbfs: interface 0 claimed by ch341 while 'brltty' sets config #1
ch341-uart ttyUSB2: ch341-uart converter now disconnected  ← 被 brltty 踢掉！
```

**`brltty`** 是盲文显示终端驱动，它在系统启动时会扫描所有 USB 串口设备并尝试占用它们。即使 CH340 驱动正常工作创建了设备节点，brltty 也会立刻将其断开。

**解决方案：彻底卸载 brltty**

```bash
# 停止服务
sudo systemctl stop brltty
sudo systemctl disable brltty

# 彻底卸载（推荐）
sudo apt purge brltty

# 确认进程已消失
ps aux | grep brltty | grep -v grep
# (空输出 = 已清除)
```

卸载后重新插拔 IMU（或重启设备），`/dev/ttyCH341USB0` 应正常出现。

### 3.5 验证 CH340 驱动

```bash
# 1. 模块已加载
lsmod | grep ch341
# ch341                  xxxxx  0
# usbserial              xxxxx  3 ch341,cp210x

# 2. 设备节点存在
ls -la /dev/ttyCH341USB0
# crw-rw---- 1 root dialout 169, 0 ... /dev/ttyCH341USB0

# 3. 原始数据读取（验证 IMU 是否在发数据）
python3 -c "
import serial
s = serial.Serial('/dev/ttyCH341USB0', 9600, timeout=2)
data = s.read(33)
print('Raw hex:', data.hex())  # 期望看到 55 开头的字节流
s.close()
"
```

---

## 4. udev 规则（固定设备名）

### 4.1 为什么需要 udev？

Linux 的 `/dev/ttyUSB0` / `ttyUSB1` 编号是**按插入顺序**分配的。如果先插雷达1再插雷达0，编号可能互换。udev 规则通过芯片序列号绑定固定符号链接，保证设备名稳定。

### 4.2 单雷达（简单场景）

```bash
# /etc/udev/rules.d/rplidar.rules
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
  MODE:="0777", SYMLINK+="rplidar"
```

### 4.3 双雷达（按序列号区分）

两个雷达 VID/PID 完全相同，必须用序列号区分：

```bash
# 获取每个雷达的序列号
udevadm info -a -n /dev/ttyUSB0 | grep -i serial
# ATTRS{serial}=="b6079120cf6ff011a3ba90301045c30f"  ← 雷达0

udevadm info -a -n /dev/ttyUSB1 | grep -i serial
# ATTRS{serial}=="dc1ff222d36ff011ba5d95301045c30f"  ← 雷达1
```

```bash
# /etc/udev/rules.d/rplidar.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
  ATTRS{serial}=="b6079120cf6ff011a3ba90301045c30f", \
  MODE:="0777", SYMLINK+="rplidar0"

SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
  ATTRS{serial}=="dc1ff222d36ff011ba5d95301045c30f", \
  MODE:="0777", SYMLINK+="rplidar1"
```

> ⚠️ **注意**：如果你的雷达序列号不同，运行 `udevadm info` 获取实际值并替换。换雷达时也需更新此规则。

### 4.4 IMU udev 规则

```bash
# /etc/udev/rules.d/imu_usb.rules
KERNEL=="ttyCH341USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", \
  MODE:="0777", SYMLINK+="imu_usb"
```

> ⚠️ 关键点：WCH 驱动生成的设备名是 `ttyCH341USB*` 而非 `ttyUSB*`。网上很多教程的 udev 规则写的是 `ttyUSB*`，用在 WCH 驱动上不会生效。

### 4.5 应用规则

```bash
sudo cp rplidar.rules /etc/udev/rules.d/
sudo cp imu_usb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# 验证
ls -la /dev/rplidar* /dev/imu_usb
# lrwxrwxrwx /dev/rplidar0 -> ttyUSB0
# lrwxrwxrwx /dev/rplidar1 -> ttyUSB1
# lrwxrwxrwx /dev/imu_usb  -> ttyCH341USB0
```

---

## 5. RPLidar C1 配置

### 5.1 RPLidar C1 技术参数

| 参数 | 值 |
|------|-----|
| 测距范围 | 0.05m – 16m |
| 扫描角度 | 360° |
| 采样率 | 5000 点/秒 |
| 扫描频率 | 5.5 – 10 Hz (可调) |
| 通信接口 | UART over USB (CP2102N) |
| 波特率 | 460800 bps |
| SDK 版本 | sllidar_sdk v2.1.0 |

### 5.2 克隆并构建

```bash
cd ~/jiantu
git clone https://github.com/Slamtec/sllidar_ros2.git
source /opt/ros/humble/setup.bash

# 仅构建 sllidar_ros2 包（加快速度）
colcon build --symlink-install --packages-select sllidar_ros2
```

> `--symlink-install` 用符号链接代替拷贝，修改 launch 文件后无需重新构建，节省调试时间。

### 5.3 健康检查

在启动 ROS2 节点前，先用 Python 直连串口验证雷达硬件是否正常：

```bash
python3 -c "
import serial, time
s = serial.Serial('/dev/ttyUSB0', 460800, timeout=1)
# RPLidar 协议: A5 52 = 获取健康状态
s.write(b'\xA5\x52')
time.sleep(0.5)
resp = s.read(64)
print('Response:', resp.hex())
# 期望: a5 5a 03 00 00 00 06 00 00 00
#       └─ header ─┘ └len┘ └status=0(OK)┘
s.close()
"

# 如果返回 a55a...00 00 00 表示健康状态 0（正常）
# 如果返回 a55a... 非零值表示异常
# 如果无返回 → 检查连接和波特率
```

**RPLidar 协议头解析**:
```
A5 5A = 响应帧头
03 00 00 00 = 数据长度 (30 bits, 小端)
06 = 数据类型 (06=健康信息)
00 00 00 = 状态码 (0=正常) / 错误码 (2 bytes)
```

### 5.4 ROS2 启动与参数

```bash
source /opt/ros/humble/setup.bash
source ~/jiantu/install/setup.bash

# 基础启动
ros2 launch sllidar_ros2 sllidar_c1_launch.py serial_port:=/dev/ttyUSB0

# 自定义参数
ros2 launch sllidar_ros2 sllidar_c1_launch.py \
    serial_port:=/dev/ttyUSB0 \
    serial_baudrate:=460800 \
    frame_id:=laser \
    scan_mode:=Standard \
    inverted:=false \
    angle_compensate:=true
```

### 5.5 启动日志解读

```
SLLidar S/N: D606E0F6C1E092D8A19E9FF946AA4616   ← 序列号
Firmware Ver: 1.02                                 ← 固件版本
Hardware Rev: 18                                   ← 硬件版本
SLLidar health status : 0                          ← 健康状态 0=OK
scan mode: Standard                                ← 扫描模式
sample rate: 5 Khz                                 ← 采样率 5000点/秒
max_distance: 16.0 m                               ← 最大测距
scan frequency:10.0 Hz                             ← 扫描频率
```

> ⚠️ 常见错误码 `80008004` = 操作超时：检查串口是否被占用（`sudo fuser /dev/ttyUSB0`），或雷达是否正常供电。

### 5.6 查看扫描数据

```bash
# 话题信息
ros2 topic info /scan
# Type: sensor_msgs/msg/LaserScan

# 频率
ros2 topic hz /scan
# average rate: 10.0 Hz

# 一帧数据
ros2 topic echo /scan --once
# ranges: [2.49, 2.50, 2.50, ..., inf, 9.54, ...]
#   inf = 该角度无回波（超量程或无遮挡物）
```

---

## 6. 10轴 IMU 配置

### 6.1 通信协议详解

WIT 10轴 IMU 使用二进制协议，每秒发送约 100 帧，波特率 9600。

```
帧格式 (11 字节/帧):

Byte:  0    1    2    3    4    5    6    7    8    9    10
     ┌───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┐
     │55 │XX │D0L│D0H│D1L│D1H│D2L│D2H│D3L│D3H│CK │
     └───┴───┴───┴───┴───┴───┴───┴───┴───┴───┴───┘
     帧头  类型  ───── 6字节有效数据 ─────  校验和
     (固定)     (每2字节=1个int16, 小端)
```

**数据类型 (Byte[1])**:

| 代码 | 含义 | 数据含义 (D0-D5) | k值 |
|------|------|------------------|-----|
| `0x51` | 加速度 | AxL, AxH, AyL, AyH, AzL, AzH | k=16 |
| `0x52` | 角速度 | WxL, WxH, WyL, WyH, WzL, WzH | k=2000 |
| `0x53` | 姿态角 | RollL, RollH, PitchL, PitchH, YawL, YawH | k=180 |
| `0x54` | 磁力计 | MxL, MxH, MyL, MyH, MzL, MzH | (原始值) |

**数据转换公式**:

```python
# 加速度 (单位: g, 1g = 9.8 m/s²)
acc_x = (AxH << 8 | AxL) / 32768.0 * 16.0       # 结果单位: g
acc_x = acc_x * 9.8                                # 转换为 m/s²

# 角速度 (单位: °/s)
gyro_x = (WxH << 8 | WxL) / 32768.0 * 2000.0

# 姿态角 (单位: °)
angle_x = (RxH << 8 | RxL) / 32768.0 * 180.0
```

**校验和计算**:

```python
# 前10字节累加 (含帧头0x55) 取低8位
checksum = sum(frame[0:10]) & 0xFF
# 与 frame[10] 比较，不匹配则丢弃该帧
```

### 6.2 手动验证协议

在部署 ROS2 前，直接读串口验证协议：

```bash
python3 -c "
import serial
s = serial.Serial('/dev/ttyCH341USB0', 9600, timeout=2)
# 读3秒原始数据
import time
start = time.time()
while time.time() - start < 3:
    data = s.read(11)
    if data and data[0] == 0x55:  # 找到帧头
        print('Frame:', data.hex())
s.close()
"
# 期望输出: 5551faff0200fe079a0a4a  (加速度帧)
#           5552000000000000940a45  (角速度帧)
#           5553ccff5000d075064755  (姿态角帧)
#           55544605cbeb30fc0000d6  (磁力计帧)
```

### 6.3 驱动源码关键修改

原始 `wit_ros2_imu` 包是为标准 Linux 设计的（设备名 `/dev/imu_usb`），在 Jetson 上需要调整：

**① 串口设备名** (`wit_ros2_imu/wit_ros2_imu/wit_ros2_imu.py`):

```python
# 修改前
wt_imu = serial.Serial(port="/dev/imu_usb", ...)

# 修改后 — 使用 port_name 参数
wt_imu = serial.Serial(port=port_name, baudrate=9600, ...)

# main() 中传入实际设备
node = IMUDriverNode('/dev/ttyCH341USB0')
```

**② udev 规则** (`imu_usb.rules`):

```bash
# 修改前（原规则和 RPLidar 冲突！）
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ... SYMLINK+="imu_usb"

# 修改后
KERNEL=="ttyCH341USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", \
  MODE:="0777", SYMLINK+="imu_usb"
```

**③ Launch 文件** (`launch/rviz_and_imu.launch.py`):
- `node_executable` → `executable` (ROS2 Humble 语法变更)
- 端口参数改为 `/dev/ttyCH341USB0`

### 6.4 构建

```bash
source /opt/ros/humble/setup.bash
cd ~/jiantu
colcon build --symlink-install --packages-select wit_ros2_imu
```

### 6.5 数据验证

```bash
source ~/jiantu/install/setup.bash
ros2 topic echo /imu/data_raw --once
```

```yaml
header:
  frame_id: imu_link
orientation:           # 四元数 (由姿态角转换而来)
  z: 0.992             # 接近1 → 水平放置
  w: 0.125
angular_velocity:      # 角速度 (rad/s)
  x: 0.0               # 静止时均为0
  y: 0.0
  z: 0.0
linear_acceleration:   # 线加速度 (m/s²)
  z: 0.0048            # 静止时Z轴≈0 (重力已补偿)
```

> 💡 `orientation` 由姿态角（0x53）的欧拉角 → 四元数转换而来。`linear_acceleration` 单位为 m/s²，`angular_velocity` 单位为 rad/s。

### 6.6 IMU 频率验证

```bash
ros2 topic hz /imu/data_raw
# average rate: 105.2 Hz   ← ~100Hz, 与数据手册一致
```

---

## 7. 多传感器协同 Launch + 时间同步

### 7.1 设计要点

启动双雷达 + IMU 时面临的三个问题：

| 问题 | 后果 | 解决方案 |
|------|------|----------|
| 两个雷达节点默认同名 | ROS2 不允许重名节点 | 分别命名 `sllidar_node_0` / `sllidar_node_1` |
| 两个雷达都发 `/scan` 话题 | 数据互相覆盖 | `remappings` 为 `/scan_0` / `/scan_1` |
| `serial_baudrate` 类型错误 | 进程崩溃 | 传入整数 `460800` 而非字符串 `'460800'` |

### 7.2 完整 Launch 文件

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

        # 雷达0
        Node(package='sllidar_ros2', executable='sllidar_node',
             name='sllidar_node_0', output='screen',
             parameters=[{'channel_type': 'serial',
                          'serial_port': lidar0_port,
                          'serial_baudrate': 460800,    # ← 整数，不能加引号
                          'frame_id': 'laser_0',
                          'inverted': False,
                          'angle_compensate': True,
                          'scan_mode': 'Standard'}],
             remappings=[('/scan', '/scan_0')]),

        # 雷达1
        Node(package='sllidar_ros2', executable='sllidar_node',
             name='sllidar_node_1', output='screen',
             parameters=[{'channel_type': 'serial',
                          'serial_port': lidar1_port,
                          'serial_baudrate': 460800,
                          'frame_id': 'laser_1',
                          'inverted': False,
                          'angle_compensate': True,
                          'scan_mode': 'Standard'}],
             remappings=[('/scan', '/scan_1')]),

        # IMU
        Node(package='wit_ros2_imu', executable='wit_ros2_imu',
             name='imu', output='screen',
             parameters=[{'port': imu_port, 'baud': 9600}]),

        # 时间同步
        Node(package='wit_ros2_imu', executable='time_sync',
             name='time_sync', output='screen',
             parameters=[{'sync_slop': 0.05}]),
    ])
```

### 7.3 各参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `lidar0_port` | `/dev/ttyUSB0` | 雷达0串口 |
| `lidar1_port` | `/dev/ttyUSB1` | 雷达1串口 |
| `imu_port` | `/dev/ttyCH341USB0` | IMU串口 |
| `channel_type` | `serial` | 通信方式（固定） |
| `serial_baudrate` | `460800` | 雷达波特率，**整数类型** |
| `frame_id` | `laser_0` / `laser_1` | 用于 tf 树和 SLAM |
| `scan_mode` | `Standard` | 扫描模式（Standard/Express/Boost） |
| `inverted` | `false` | 雷达是否倒装 |
| `angle_compensate` | `true` | 角度补偿（建议开启） |
| `sync_slop` | `0.05` | 时间同步窗口（秒） |

### 7.4 一键启动

```bash
source /opt/ros/humble/setup.bash
source ~/jiantu/install/setup.bash

# 默认配置
ros2 launch wit_ros2_imu sensors.launch.py

# 自定义串口
ros2 launch wit_ros2_imu sensors.launch.py \
    lidar0_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102N_...-port0 \
    lidar1_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102N_...-port0 \
    imu_port:=/dev/ttyCH341USB0
```

成功输出：

```
[sllidar_node_0] SLLidar S/N: D606... health status : OK.  ← 雷达0 ✅
[sllidar_node_1] SLLidar S/N: F674... health status : OK.  ← 雷达1 ✅
[imu]            Serial port opened successfully...         ← IMU  ✅
[time_sync]      Time sync ready (slop=0.05s) → /sync/*     ← 同步 ✅
```

> 全部 4 个进程启动，系统负载约 **15% CPU, 400MB RAM**。

### 7.5 停止传感器

```bash
# Ctrl+C 在 launch 终端

# 或强制停止所有传感器进程
pkill -f "sllidar_node\|wit_ros2_imu\|time_sync"

# 释放被占用的串口（如遇端口冲突）
sudo fuser -k /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyCH341USB0
```

### 7.6 时间同步原理

`time_sync` 节点使用 ROS2 的 `message_filters.ApproximateTimeSynchronizer`，按最近时间戳策略匹配三传感器数据。当雷达0、雷达1、IMU 的数据时间戳都在 `sync_slop`（默认 50ms）窗口内时，将它们同时转发到 `/sync/*` 话题。

```
时间线:
  IMU:    |||  |||  |||  |||  |||  (~100Hz)
  Lidar0:     |___________|           (10Hz)
  Lidar1:      |___________|          (10Hz)

sync_slop=50ms:
  窗口 [t-25ms, t+25ms] 内所有传感器数据 → 同时输出
```

**同步状态监控**:

```bash
ros2 topic echo /sync/status
# data: [-0.031, 0.005, -0.036]
#        ↑ scan0→imu   ↑ scan1→imu   ↑ scan0→scan1
#        (秒) 负值=雷达早于IMU  正值=雷达晚于IMU

# 如果某个值超过 ±0.05，说明传感器时钟漂移较大
# 正常情况应在 ±0.04s 以内
```

---

## 8. 话题总览与数据验证

### 8.1 完整话题列表

```bash
$ ros2 topic list
```

| 话题 | 消息类型 | 频率 | frame_id | 用途 |
|------|----------|------|----------|------|
| `/scan_0` | `sensor_msgs/LaserScan` | 10 Hz | `laser_0` | 雷达0 原始数据 |
| `/scan_1` | `sensor_msgs/LaserScan` | 10 Hz | `laser_1` | 雷达1 原始数据 |
| `/imu/data_raw` | `sensor_msgs/Imu` | ~100 Hz | `imu_link` | IMU 原始数据 |
| `/sync/scan_0` | `sensor_msgs/LaserScan` | ~10 Hz | `laser_0` | 雷达0 时间对齐 |
| `/sync/scan_1` | `sensor_msgs/LaserScan` | ~10 Hz | `laser_1` | 雷达1 时间对齐 |
| `/sync/imu` | `sensor_msgs/Imu` | ~100 Hz | `imu_link` | IMU 时间对齐 |
| `/sync/status` | `Float64MultiArray` | ~10 Hz | — | 同步漂移监控 |

### 8.2 查看话题详情

```bash
# 某话题的发布/订阅关系
ros2 topic info /scan_0

# 每秒发布频率
ros2 topic hz /scan_0
# average rate: 9.980 Hz

# 消息类型定义
ros2 interface show sensor_msgs/msg/LaserScan
```

### 8.3 rosbag 录制

```bash
# 录制所有传感器原始数据
ros2 bag record -o sensor_data /scan_0 /scan_1 /imu/data_raw

# 录制时间同步后的数据
ros2 bag record -o sync_data /sync/scan_0 /sync/scan_1 /sync/imu

# 回放
ros2 bag play sensor_data
```

### 8.4 RViz2 可视化

```bash
# 在 launch 终端 Ctrl+C 停止，然后启动带 RViz 的版本
ros2 launch sllidar_ros2 view_sllidar_c1_launch.py serial_port:=/dev/ttyUSB0

# 手动启动 RViz2 并添加 LaserScan 和 IMU 显示
rviz2
# → Add → By topic → /scan_0 → LaserScan
# → Add → By topic → /imu/data_raw → Imu
# → Fixed Frame: laser_0 或 imu_link
```

---

## 9. 踩坑总结

| # | 坑 | 原因 | 解决 | 严重度 |
|---|-----|------|------|--------|
| 1 | CH340 插入无设备 | Jetson 内核无 `ch341.ko` | 编译 WCH 官方驱动 `ch341ser_linux` | 🔴 阻塞 |
| 2 | 驱动装了设备还是消失 | `brltty` 抢占串口 | `sudo apt purge brltty` | 🔴 阻塞 |
| 3 | udev 规则不生效 | WCH 驱动设备名是 `ttyCH341USB*` | 规则匹配 `KERNEL=="ttyCH341USB*"` | 🟡 踩坑 |
| 4 | IMU 报端口被占用 | 旧进程未退出或 brltty 残留 | `sudo fuser -k /dev/ttyCH341USB0` | 🟡 踩坑 |
| 5 | launch 报 `node_executable` 错误 | ROS2 Humble 用 `executable` | 全局替换为 `executable` | 🟡 踩坑 |
| 6 | `serial_baudrate` 类型错误 | ROS2 Humble C++ 参数校验严格 | 传整数 `460800` 不传字符串 `'460800'` | 🟡 踩坑 |
| 7 | 两个雷达话题互相覆盖 | 默认都发 `/scan` 且节点同名 | 不同 name + remap `/scan_0` `/scan_1` | 🟡 踩坑 |
| 8 | 换插入顺序后雷达串口号变了 | `/dev/ttyUSB*` 按顺序分配 | 用 `ATTRS{serial}` 或 `/dev/serial/by-id/` | 🟢 建议 |
| 9 | `ros2 topic echo /scan --once` 无输出 | `--once` 可能刚好错过当前帧 | 等下一帧或用 `ros2 topic hz` | 🟢 建议 |

---

## 10. RViz 可视化双雷达+IMU

### 10.1 预置配置

项目已预置 RViz2 配置文件，包含：
- 🔴 **LaserScan_0**：`/scan_0` 话题，红色点云
- 🟢 **LaserScan_1**：`/scan_1` 话题，绿色点云
- 🟡 **IMU**：`/imu/data_raw` 话题，黄色姿态箭头

### 10.2 一键启动

```bash
# 终端 1：启动传感器
cd ~/jiantu
source install/setup.bash
ros2 launch wit_ros2_imu sensors.launch.py

# 终端 2：启动 RViz 可视化
cd ~/jiantu
source install/setup.bash
ros2 launch wit_ros2_imu show_dual_lidar.launch.py
```

### 10.3 自定义雷达安装位置

如果两个雷达有实际安装偏移，可通过参数指定：

```bash
# x=0.3m 表示雷达1在雷达0右侧30cm处
ros2 launch wit_ros2_imu show_dual_lidar.launch.py x:=0.3 y:=0.0 z:=0.0 yaw:=0.0
```

### 10.4 原理说明

不同传感器有不同的 `frame_id`（`laser_0` / `laser_1` / `imu_link`），要在 RViz 同一窗口下同时显示，需要 TF 变换关联：

```
laser_0 ──(static TF)──→ laser_1
laser_0 ──(static TF)──→ imu_link
```

两个 `static_transform_publisher` 在 `show_dual_lidar.launch.py` 中自动发布。

> 💡 SSH 无桌面环境会报 `could not connect to display`，需在 Jetson 本地桌面或 `ssh -X` 下运行。

---

## 11. 参考链接

- [Slamtec sllidar_ros2](https://github.com/Slamtec/sllidar_ros2) — RPLidar ROS2 驱动
- [WCH ch341ser_linux](https://github.com/WCHSoftGroup/ch341ser_linux) — CH340 官方 Linux 驱动
- [brltty CH340 冲突问题](https://blog.insmtr.com/2025/04/24/2025.4.24%20CH340/) — 同类踩坑记录
- [ROS2 Launch 文档](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html) — launch 文件编写指南
- [message_filters 文档](http://wiki.ros.org/message_filters) — 时间同步器 API

---

> **项目仓库**: [Gitee](https://gitee.com/tydfgt/jiantushuangleida) | [GitHub](https://github.com/tydfgt/jiantushuangleida)  
> **硬件快照**: Jetson Orin Nano Super | L4T 36.5.0 | ROS2 Humble | RPLidar C1 ×2 + 10轴 IMU  
> **最后更新**: 2026-07-06
