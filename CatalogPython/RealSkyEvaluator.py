#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
星敏感器终极评估系统：全区真实星空模拟 + 姿态误差交叉验证
"""

import serial
import time
import numpy as np
import random
import sys
import math
import struct
import csv

# ========================= 参数配置 =========================
PORT = 'COM5'
BAUDRATE = 115200

# 读取你刚刚自己生成的 CSV
CSV_PATH = r"D:\1StarTracker\CatalogPython\tycho2_bright_stars.csv"

IMG_WIDTH = 320
IMG_HEIGHT = 240
FX = 500.0
FY = 500.0
CX = 160.0
CY = 120.0

BACKGROUND_MEAN = 100
BACKGROUND_STD = 10
FRAME_RATE = 2
SEND_INTERVAL = 1.0 / FRAME_RATE

# 合格的误差门限 (度)
ACCEPTABLE_ERROR_DEG = 0.5


# ========================= 核心数学逻辑 =========================

def load_star_catalog(csv_path):
    """从 CSV 读取星表"""
    stars_3d = []
    print(f"正在读取星表: {csv_path}")
    try:
        with open(csv_path, 'r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    ra_deg = float(row['RA_deg'])
                    dec_deg = float(row['Dec_deg'])
                    mag = float(row['VTmag'])

                    ra_rad = math.radians(ra_deg)
                    dec_rad = math.radians(dec_deg)
                    x = math.cos(dec_rad) * math.cos(ra_rad)
                    y = math.cos(dec_rad) * math.sin(ra_rad)
                    z = math.sin(dec_rad)
                    stars_3d.append((x, y, z, mag))
                except (ValueError, KeyError):
                    continue
    except Exception as e:
        print(f"读取星表失败: {e}")
        sys.exit(1)

    print(f"成功读取 {len(stars_3d)} 颗恒星！")
    return np.array(stars_3d)


def generate_rotation_matrix(z_vec):
    """生成旋转矩阵 (真值)"""
    z_vec = np.array(z_vec) / np.linalg.norm(z_vec)
    up = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(z_vec, up)) > 0.99:
        up = np.array([1.0, 0.0, 0.0])

    x_vec = np.cross(up, z_vec)
    x_vec = x_vec / np.linalg.norm(x_vec)
    y_vec = np.cross(z_vec, x_vec)

    roll = random.uniform(0, 2 * math.pi)
    cos_r, sin_r = math.cos(roll), math.sin(roll)
    x_rot = x_vec * cos_r + y_vec * sin_r
    y_rot = -x_vec * sin_r + y_vec * cos_r

    return np.vstack((x_rot, y_rot, z_vec))


def rotation_matrix_to_quaternion(R):
    """将旋转矩阵转换为四元数 (w, x, y, z)"""
    m00, m01, m02 = R[0, 0], R[0, 1], R[0, 2]
    m10, m11, m12 = R[1, 0], R[1, 1], R[1, 2]
    m20, m21, m22 = R[2, 0], R[2, 1], R[2, 2]
    tr = m00 + m11 + m22

    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2.0
        qw, qx, qy, qz = 0.25 * S, (m21 - m12) / S, (m02 - m20) / S, (m10 - m01) / S
    elif (m00 > m11) and (m00 > m22):
        S = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw, qx, qy, qz = (m21 - m12) / S, 0.25 * S, (m01 + m10) / S, (m02 + m20) / S
    elif m11 > m22:
        S = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw, qx, qy, qz = (m02 - m20) / S, (m01 + m10) / S, 0.25 * S, (m12 + m21) / S
    else:
        S = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw, qx, qy, qz = (m10 - m01) / S, (m02 + m20) / S, (m12 + m21) / S, 0.25 * S

    return np.array([qw, qx, qy, qz])


def calculate_attitude_error(q_true, q_est):
    """计算两个四元数之间的角误差 (度)"""
    # 归一化以防万一
    q_true = q_true / np.linalg.norm(q_true)
    q_est = q_est / np.linalg.norm(q_est)

    # 求点积的绝对值 (因为 q 和 -q 表示同一旋转)
    dot = np.abs(np.dot(q_true, q_est))
    dot = min(1.0, dot)  # 防止浮点数溢出超过 1

    # 计算误差角
    error_rad = 2.0 * math.acos(dot)
    return math.degrees(error_rad)


def render_star_image(stars, R):
    """渲染星空"""
    img = np.random.normal(BACKGROUND_MEAN, BACKGROUND_STD, (IMG_HEIGHT, IMG_WIDTH)).astype(np.float32)
    img = np.clip(img, 0, 65535)

    v_cam = np.dot(stars[:, 0:3], R.T)
    mags = stars[:, 3]

    stars_in_fov = 0
    for i in range(len(v_cam)):
        x, y, z = v_cam[i]
        if z > 0:
            u = FX * (x / z) + CX
            v = FY * (y / z) + CY
            if 0 <= u < IMG_WIDTH and 0 <= v < IMG_HEIGHT:
                stars_in_fov += 1
                base_brightness = 60000 * (10 ** (-0.4 * mags[i]))
                radius = 2
                u_int, v_int = int(u), int(v)
                for dy in range(-radius, radius + 1):
                    for dx in range(-radius, radius + 1):
                        ny, nx = v_int + dy, u_int + dx
                        if 0 <= nx < IMG_WIDTH and 0 <= ny < IMG_HEIGHT:
                            dist2 = (nx - u) ** 2 + (ny - v) ** 2
                            pixel_val = base_brightness * math.exp(-dist2 / 1.5)
                            img[ny, nx] = min(65535, img[ny, nx] + pixel_val)

    return np.clip(img, 0, 65535).astype(np.uint16), stars_in_fov


def frame_to_bytes(image, width, height):
    return bytes([0xAA, 0x55]) + struct.pack('<H', width) + struct.pack('<H', height) + image.tobytes()


# ========================= 主程序 =========================

def main():
    stars = load_star_catalog(CSV_PATH)

    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=0.1)
        print(f"串口 {PORT} 打开成功")
    except Exception as e:
        print(f"打开串口失败: {e}")
        sys.exit(1)

    ser.reset_input_buffer()
    frame_count = 0
    success_count = 0
    total_error = 0.0

    print(f"\n开始【真实星表评估】，帧率 {FRAME_RATE} Hz")
    print("=" * 70)

    try:
        while True:
            start_time = time.time()

            # 1. 随机找一个亮星作为中心指向，生成相机姿态
            target_star = random.choice(stars)
            R_true = generate_rotation_matrix(target_star[0:3])

            # 计算真值四元数
            q_true = rotation_matrix_to_quaternion(R_true)

            # 2. 渲染并发送图像
            img, stars_in_fov = render_star_image(stars, R_true)
            frame_bytes = frame_to_bytes(img, IMG_WIDTH, IMG_HEIGHT)
            ser.write(frame_bytes)
            frame_count += 1

            # 3. 接收单片机结果
            received = False
            start_recv = time.time()
            while time.time() - start_recv < 2.0:
                line = ser.readline()
                if not line: continue
                try:
                    line_str = line.decode('utf-8', errors='ignore').strip()
                    if line_str.startswith('ATT:'):
                        # 解析四元数 ATT: w x y z (Det:D Mat:M)
                        parts = line_str.split()
                        if len(parts) >= 5:
                            q_est = np.array([float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4])])

                            # 获取检测与匹配数
                            extra_info = line_str.split('(')[-1].replace(')', '') if '(' in line_str else ""

                            # 计算误差！
                            error_deg = calculate_attitude_error(q_true, q_est)

                            received = True
                            success_count += 1
                            total_error += error_deg
                            break
                    elif line_str.startswith('ERR:'):
                        print(f"[帧 {frame_count:3d}] 视野内星数: {stars_in_fov:2d} | ❌ 解算失败: {line_str}")
                except:
                    pass

            if received:
                # 评估打印
                mark = "✅ 优秀" if error_deg <= ACCEPTABLE_ERROR_DEG else "⚠️ 偏差较大"
                print(f"[帧 {frame_count:3d}] 视野内星数: {stars_in_fov:2d} | {extra_info}")
                print(f"      真值 Q: w={q_true[0]:.4f}, x={q_true[1]:.4f}, y={q_true[2]:.4f}, z={q_true[3]:.4f}")
                print(f"      估计 Q: w={q_est[0]:.4f},  x={q_est[1]:.4f},  y={q_est[2]:.4f},  z={q_est[3]:.4f}")
                print(f"      🎯 姿态误差: {error_deg:.4f} 度 {mark}")
                print("-" * 70)

            # 帧率控制
            elapsed = time.time() - start_time
            if elapsed < SEND_INTERVAL:
                time.sleep(SEND_INTERVAL - elapsed)

    except KeyboardInterrupt:
        print(f"\n\n================= 评估总结 =================")
        print(f"总发送测试帧: {frame_count}")
        print(f"成功解算帧数: {success_count} (成功率: {success_count / frame_count * 100:.1f}%)")
        if success_count > 0:
            print(f"平均姿态误差: {total_error / success_count:.4f} 度")
        print(f"============================================")
    finally:
        ser.close()


if __name__ == "__main__":
    main()