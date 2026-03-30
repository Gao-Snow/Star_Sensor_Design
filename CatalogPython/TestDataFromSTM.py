#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
星图闭环测试模拟器：直接读取 STM32 的 C 语言源码星表和三角形库
强制生成单片机“必认识”的星图
"""

import serial
import time
import numpy as np
import random
import sys
import math
import struct
import re

# ========================= 参数配置 =========================
PORT = 'COM5'
BAUDRATE = 115200

#  请修改为你本地 STM32 工程中这两个 .c 文件的绝对路径！
CATALOG_C_PATH = r"D:\1StarTracker\STM32\Core\Src\star_catalog_nav.c"
TRIANGLE_C_PATH = r"D:\1StarTracker\STM32\Core\Src\triangle_db.c"

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


# ========================= C源码解析逻辑 =========================

def load_c_catalog(filepath):
    """直接从 STM32 的 C 源码中解析星表"""
    stars = []
    print(f"正在解析 C 星表: {filepath}")
    try:
        with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
            content = f.read()

        # 匹配格式: { 6849303433ULL, 1, 0, 0, 191, 32768, 0, 0 }
        pattern = r'\{\s*\d+ULL\s*,\s*\d+\s*,\s*[-]?\d+\s*,\s*[-]?\d+\s*,\s*(\d+)\s*,\s*([-]?\d+)\s*,\s*([-]?\d+)\s*,\s*([-]?\d+)\s*\}'

        for match in re.finditer(pattern, content):
            mag = float(match.group(1)) / 100.0
            x = float(match.group(2)) / 32768.0
            y = float(match.group(3)) / 32768.0
            z = float(match.group(4)) / 32768.0

            # 还原单片机视角的单位向量
            norm = math.sqrt(x * x + y * y + z * z)
            if norm > 0:
                stars.append((x / norm, y / norm, z / norm, mag))

        print(f"成功从 C 源码中提取了 {len(stars)} 颗恒星！")
        return np.array(stars)
    except Exception as e:
        print(f"读取星表失败: {e}")
        sys.exit(1)


def load_c_triangles(filepath):
    """直接从 STM32 的 C 源码中解析三角形数据库"""
    triangles = []
    print(f"正在解析 C 三角形库: {filepath}")
    try:
        with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
            content = f.read()

        # 匹配格式: { 391290001987610383ULL, { 0, 2, 5 } }
        pattern = r'\{\s*\d+ULL\s*,\s*\{\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*\}\s*\}'

        for match in re.finditer(pattern, content):
            triangles.append((int(match.group(1)), int(match.group(2)), int(match.group(3))))

        print(f"成功从 C 源码中提取了 {len(triangles)} 个三角形！")
        return triangles
    except Exception as e:
        print(f"读取三角形库失败: {e}")
        sys.exit(1)


# ========================= 核心投影逻辑 =========================

def generate_rotation_matrix(z_vec):
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


def render_star_image(stars, R):
    img = np.random.normal(BACKGROUND_MEAN, BACKGROUND_STD, (IMG_HEIGHT, IMG_WIDTH)).astype(np.float32)
    img = np.clip(img, 0, 65535)

    vectors = stars[:, 0:3]
    mags = stars[:, 3]
    v_cam = np.dot(vectors, R.T)

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
    # 1. 直接读取单片机的星表和数据库
    stars = load_c_catalog(CATALOG_C_PATH)
    triangles = load_c_triangles(TRIANGLE_C_PATH)

    # 2. 筛选出可以完美放入我们 35度 FOV 视野的合法三角形
    valid_triangles = []
    for t in triangles:
        # 防止越界
        if t[0] < len(stars) and t[1] < len(stars) and t[2] < len(stars):
            v1, v2, v3 = stars[t[0]][:3], stars[t[1]][:3], stars[t[2]][:3]
            # 计算三角形的三个边角(度)
            a = math.degrees(math.acos(np.clip(np.dot(v1, v2), -1, 1)))
            b = math.degrees(math.acos(np.clip(np.dot(v2, v3), -1, 1)))
            c = math.degrees(math.acos(np.clip(np.dot(v1, v3), -1, 1)))

            # 如果最大内角跨度小于 25 度，它就能完美被 320x240(约35度视场)拍下
            if max(a, b, c) < 25.0:
                valid_triangles.append(t)

    print(f"筛选出符合 FOV 视场大小的极佳三角形组合: {len(valid_triangles)} 个")

    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=0.1)
        print(f"串口 {PORT} 打开成功")
    except Exception as e:
        print(f"打开串口失败: {e}")
        sys.exit(1)

    ser.reset_input_buffer()
    ser.reset_output_buffer()

    frame_count = 0
    success_count = 0

    print(f"\n开始发送闭环强制图像，帧率 {FRAME_RATE} Hz")
    print("-" * 60)

    try:
        while True:
            start_time = time.time()

            # --- 闭环核心逻辑 ---
            # 随机挑选一个“单片机绝对认识”的三角形
            target_tri = random.choice(valid_triangles)
            v1, v2, v3 = stars[target_tri[0]][:3], stars[target_tri[1]][:3], stars[target_tri[2]][:3]

            # 将虚拟相机的正中心，强制对准这个三角形的几何中心！
            z_vec = (v1 + v2 + v3) / 3.0
            R = generate_rotation_matrix(z_vec)

            # 渲染图像
            img, stars_in_fov = render_star_image(stars, R)

            # 发送图像
            frame_bytes = frame_to_bytes(img, IMG_WIDTH, IMG_HEIGHT)
            ser.write(frame_bytes)
            frame_count += 1

            # 接收逻辑
            received = False
            q_info = ""
            start_recv = time.time()
            while time.time() - start_recv < 0.4:
                line = ser.readline()
                if not line:
                    continue
                try:
                    line_str = line.decode('utf-8', errors='ignore').strip()
                    if line_str.startswith('ATT:'):
                        q_info = line_str
                        received = True
                        success_count += 1
                        break
                    elif line_str.startswith('ERR:'):
                        print(f"      >> STM32 反馈: {line_str}")
                    elif line_str:
                        print(f"      >> 原始信息: {line_str}")
                except:
                    pass

            status = "✓" if received else "✗"
            print(f"[帧 {frame_count:3d}] 视野内星数: {stars_in_fov:2d} | 接收状态: {status}")
            if received:
                print(f"      🎉解算成功: {q_info}")

            # 严格帧率控制
            elapsed = time.time() - start_time
            if elapsed < SEND_INTERVAL:
                time.sleep(SEND_INTERVAL - elapsed)

    except KeyboardInterrupt:
        print(f"\n\n用户中断")
        print(f"总发送 {frame_count} 帧，成功解算 {success_count} 帧")
    finally:
        ser.close()


if __name__ == "__main__":
    main()