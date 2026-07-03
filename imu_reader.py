#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
"""
Yahboom 10-axis IMU USB 数据读取工具
适用: NVIDIA Jetson Orin Nano / Raspberry Pi
波特率: 921600 (出厂默认)
"""

import serial
import sys
import time

# ===== 配置 =====
PORT = '/dev/ttyUSB0'
BAUD = 921600  # 产品默认波特率
BUF_LEN = 11

# ===== 数据存储 =====
acc = [0.0] * 3
gyro = [0.0] * 3
angle = [0.0] * 3


def get_acc(datahex):
    k_acc = 16.0
    ax = (datahex[1] << 8 | datahex[0]) / 32768.0 * k_acc
    ay = (datahex[3] << 8 | datahex[2]) / 32768.0 * k_acc
    az = (datahex[5] << 8 | datahex[4]) / 32768.0 * k_acc
    if ax >= k_acc:
        ax -= 2 * k_acc
    if ay >= k_acc:
        ay -= 2 * k_acc
    if az >= k_acc:
        az -= 2 * k_acc
    return ax, ay, az


def get_gyro(datahex):
    k_gyro = 2000.0
    gx = (datahex[1] << 8 | datahex[0]) / 32768.0 * k_gyro
    gy = (datahex[3] << 8 | datahex[2]) / 32768.0 * k_gyro
    gz = (datahex[5] << 8 | datahex[4]) / 32768.0 * k_gyro
    if gx >= k_gyro:
        gx -= 2 * k_gyro
    if gy >= k_gyro:
        gy -= 2 * k_gyro
    if gz >= k_gyro:
        gz -= 2 * k_gyro
    return gx, gy, gz


def get_angle(datahex):
    k_angle = 180.0
    ax = (datahex[1] << 8 | datahex[0]) / 32768.0 * k_angle
    ay = (datahex[3] << 8 | datahex[2]) / 32768.0 * k_angle
    az = (datahex[5] << 8 | datahex[4]) / 32768.0 * k_angle
    if ax >= k_angle:
        ax -= 2 * k_angle
    if ay >= k_angle:
        ay -= 2 * k_angle
    if az >= k_angle:
        az -= 2 * k_angle
    return ax, ay, az


def main():
    global acc, gyro, angle

    print(f"正在打开 {PORT} @ {BAUD} bps ...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1.0)
    except serial.SerialException as e:
        print(f"无法打开串口: {e}")
        sys.exit(1)

    print(f"串口已打开: {ser.is_open}")
    print("等待 IMU 数据... (按 Ctrl+C 停止)\n")

    buf = []
    count = 0

    try:
        while True:
            # 逐字节读取
            raw = ser.read(1)
            if not raw:
                continue

            byte_val = raw[0]
            buf.append(byte_val)

            # 找帧头 0x55
            if len(buf) > 1 and buf[-2] == 0x55:
                # 帧头找到，检查帧长是否为 11 字节
                if len(buf) >= 11:
                    frame = buf[-11:]

                    # 校验和
                    ck = sum(frame[:10]) & 0xFF
                    if ck != frame[10]:
                        # 校验失败，丢弃
                        buf = buf[-10:]
                        continue

                    cmd = frame[1]
                    data = frame[2:8]

                    if cmd == 0x51:  # 加速度
                        acc = list(get_acc(data))
                        print(f"[加速度]  X:{acc[0]:8.3f}g  Y:{acc[1]:8.3f}g  Z:{acc[2]:8.3f}g")
                    elif cmd == 0x52:  # 角速度
                        gyro = list(get_gyro(data))
                        print(f"[角速度]  X:{gyro[0]:8.3f}°  Y:{gyro[1]:8.3f}°  Z:{gyro[2]:8.3f}°")
                    elif cmd == 0x53:  # 姿态角
                        angle = list(get_angle(data))
                        print(f"[姿态角]  Roll:{angle[0]:8.3f}°  Pitch:{angle[1]:8.3f}°  Yaw:{angle[2]:8.3f}°")

                    count += 1
                    # 限制 buffer 大小
                    if len(buf) > 100:
                        buf = buf[-50:]

            # 限制 buffer 大小
            if len(buf) > 200:
                buf = buf[-50:]

    except KeyboardInterrupt:
        print(f"\n\n收到 {count} 条数据，程序退出。")
    finally:
        ser.close()
        print("串口已关闭。")


if __name__ == '__main__':
    main()
