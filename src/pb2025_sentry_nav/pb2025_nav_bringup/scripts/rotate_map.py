#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
地图旋转工具 - 用于对齐 small_gicp 定位后的点云与栅格地图
=============================================================

功能说明:
---------
当 small_gicp_relocalization 定位后，rviz 中实时检测的点云与栅格地图/代价地图
存在角度偏差时，使用此脚本旋转栅格地图（PGM 文件）并更新 YAML 配置文件。

旋转原理:
---------
- PGM 地图: 绕图像中心旋转指定角度
- YAML 配置: 更新 origin 位置和角度 (yaw)

使用方法:
---------
1. 旋转地图（默认顺时针90度，即 -90度）:
   python rotate_map.py pb2025_nav_bringup/map/simulation/rmuc_2026_1.yaml --angle -90

2. 旋转180度:
   python rotate_map.py pb2025_nav_bringup/map/simulation/rmuc_2026_1.yaml --angle 180

3. 指定输出路径:
   python rotate_map.py input_map.yaml --angle -90 -o output_map.yaml

4. 使用最近邻插值（保持栅格值不变）:
   python rotate_map.py input_map.yaml --angle 90 --nearest

参数说明:
---------
- angle: 旋转角度（度），正值为逆时针，负值为顺时针
  * 常见值: -90 (顺时针90°), 90 (逆时针90°), 180 (翻转)
- output: 输出文件路径（可选，默认在原文件名后加 _rotated）
- nearest: 使用最近邻插值（推荐用于栅格地图，避免产生中间值）

注意事项:
---------
1. 旋转后地图尺寸会变化（变为能容纳原图的最小矩形）
2. 新的 origin 会自动计算，保持地图的物理位置正确
3. 建议备份原始地图文件后再操作
4. 旋转后需要在 nav2_params.yaml 中更新地图路径

作者: Assistant
日期: 2026-05-01
"""

import os
import sys
import argparse
import math
import numpy as np
from PIL import Image
from typing import Tuple, List
import yaml


def read_pgm(filename: str) -> Tuple[np.ndarray, dict]:
    """
    读取 PGM 文件，返回图像数组和元数据
    
    支持二进制 PGM (P5) 格式
    """
    with open(filename, 'rb') as f:
        # 读取头部
        header = f.readline().strip()
        if header == b'P5':
            # 二进制 PGM
            pass
        elif header == b'P2':
            # ASCII PGM - 这里我们只处理二进制格式
            raise ValueError("不支持 ASCII PGM (P2) 格式，请转换为二进制 PGM (P5)")
        else:
            raise ValueError(f"不支持的 PGM 格式: {header}")
        
        # 跳过注释行
        while True:
            line = f.readline()
            if not line.startswith(b'#'):
                break
        
        # 读取宽度和高度
        width, height = map(int, line.split())
        max_val = int(f.readline().strip())
        
        # 读取图像数据
        image_data = np.frombuffer(f.read(), dtype=np.uint8)
        
        # 如果 max_val > 255，需要读取为 16-bit
        if max_val > 255:
            f.seek(0)
            # 重新读取
            # ... 简化处理，假设 max_val <= 255
            pass
        
        image = image_data.reshape((height, width))
        
        metadata = {
            'width': width,
            'height': height,
            'max_val': max_val,
            'mode': 'P5'
        }
        
        return image, metadata


def write_pgm(filename: str, image: np.ndarray, max_val: int = 255):
    """
    写入二进制 PGM (P5) 文件
    """
    height, width = image.shape
    with open(filename, 'wb') as f:
        f.write(b'P5\n')
        f.write(f'{width} {height}\n'.encode())
        f.write(f'{max_val}\n'.encode())
        f.write(image.astype(np.uint8).tobytes())


def rotate_image(image: np.ndarray, angle_deg: float, 
                 fill_value: int = 205,  # 未知区域默认值 (Nav2 中 205 表示未知)
                 order: int = 0) -> Tuple[np.ndarray, Tuple[float, float]]:
    """
    旋转图像，返回旋转后的图像和偏移量
    
    参数:
    -----
    image: 输入图像数组
    angle_deg: 旋转角度（度），正值为逆时针
    fill_value: 填充值（旋转后空白区域的值）
    order: 插值顺序 (0=最近邻, 1=线性, 3=三次样条)
    
    返回:
    -----
    rotated_image: 旋转后的图像
    offset: (dx, dy) 原图中心相对于新图中心的偏移
    """
    from scipy import ndimage
    
    height, width = image.shape
    angle_rad = math.radians(angle_deg)
    
    # 计算旋转后的图像尺寸
    # 新尺寸 = 原图对角线长度，确保能容纳旋转后的图像
    diag = math.sqrt(width**2 + height**2)
    new_width = int(math.ceil(abs(width * math.cos(angle_rad)) + abs(height * math.sin(angle_rad))))
    new_height = int(math.ceil(abs(width * math.sin(angle_rad)) + abs(height * math.cos(angle_rad))))
    
    # 确保尺寸为奇数或偶数的一致性，避免半像素偏移
    # 这里我们保持中心对齐
    
    # 计算中心点
    cx, cy = width / 2.0, height / 2.0
    new_cx, new_cy = new_width / 2.0, new_height / 2.0
    
    # 计算偏移：将原图中心移到新图中心需要的平移
    # 旋转矩阵 (逆时针)
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    
    # 构建变换矩阵 (仿射变换)
    # 步骤: 1. 平移原图中心到原点 2. 旋转 3. 平移新图中心
    # 组合后的变换: 
    # x' = cos*a * (x-cx) - sin*a * (y-cy) + new_cx
    # y' = sin*a * (x-cx) + cos*a * (y-cy) + new_cy
    
    # 对于 ndimage.affine_transform，我们需要逆变换
    # 即给定输出坐标 (x', y')，求输入坐标 (x, y)
    # x = cos*a * (x'-new_cx) + sin*a * (y'-new_cy) + cx
    # y = -sin*a * (x'-new_cx) + cos*a * (y'-new_cy) + cy
    
    transform_matrix = np.array([
        [cos_a, sin_a, cx - cos_a * new_cx - sin_a * new_cy],
        [-sin_a, cos_a, cy + sin_a * new_cx - cos_a * new_cy]
    ])
    
    # 执行仿射变换
    rotated = ndimage.affine_transform(
        image,
        transform_matrix,
        output_shape=(new_height, new_width),
        order=order,  # 0=最近邻, 1=线性
        mode='constant',
        cval=fill_value,
        prefilter=False  # 对于 order=0 不需要预过滤
    )
    
    # 计算原图中心在新图中的位置（应该是 new_cx, new_cy）
    offset = (new_cx - cx, new_cy - cy)
    
    return rotated, offset


def rotate_map(yaml_file: str, angle_deg: float, 
               output_yaml: str = None,
               use_nearest: bool = True,
               unknown_value: int = 205):
    """
    旋转地图（PGM + YAML）
    
    参数:
    -----
    yaml_file: 输入 YAML 文件路径
    angle_deg: 旋转角度（度）
    output_yaml: 输出 YAML 文件路径（可选）
    use_nearest: 是否使用最近邻插值
    unknown_value: 未知区域的填充值
    """
    # 读取 YAML 配置
    with open(yaml_file, 'r') as f:
        map_config = yaml.safe_load(f)
    
    # 获取 PGM 文件路径
    yaml_dir = os.path.dirname(os.path.abspath(yaml_file))
    pgm_filename = map_config['image']
    if not os.path.isabs(pgm_filename):
        pgm_path = os.path.join(yaml_dir, pgm_filename)
    else:
        pgm_path = pgm_filename
    
    if not os.path.exists(pgm_path):
        raise FileNotFoundError(f"找不到 PGM 文件: {pgm_path}")
    
    # 读取 PGM
    print(f"读取地图: {pgm_path}")
    image, metadata = read_pgm(pgm_path)
    height, width = image.shape
    print(f"  原始尺寸: {width} x {height} 像素")
    
    # 获取原始参数
    resolution = map_config['resolution']
    origin = map_config['origin']  # [x, y, yaw]
    original_yaw = origin[2] if len(origin) > 2 else 0.0
    
    print(f"  分辨率: {resolution} m/像素")
    print(f"  原始原点: ({origin[0]:.3f}, {origin[1]:.3f}, {math.degrees(original_yaw):.1f}°)")
    
    # 旋转图像
    print(f"\n旋转角度: {angle_deg}°")
    order = 0 if use_nearest else 1
    rotated_image, offset = rotate_image(image, angle_deg, fill_value=unknown_value, order=order)
    new_height, new_width = rotated_image.shape
    print(f"  新尺寸: {new_width} x {new_height} 像素")
    
    # 计算新的 origin
    # 原图左下角在地图坐标系中的位置
    # 原图: origin[0], origin[1] 是左下角
    # 旋转后，左下角位置会变化
    
    angle_rad = math.radians(angle_deg)
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    
    # 原图中心在地图坐标系中的位置
    cx_map = origin[0] + (width * resolution) / 2.0
    cy_map = origin[1] + (height * resolution) / 2.0
    
    # 新图左下角在地图坐标系中的位置
    # 新图中心 = 原图中心（地图坐标系中不变）
    new_origin_x = cx_map - (new_width * resolution) / 2.0
    new_origin_y = cy_map - (new_height * resolution) / 2.0
    
    # 新的 yaw 角度
    new_yaw = original_yaw + angle_rad
    # 归一化到 [-pi, pi]
    new_yaw = (new_yaw + math.pi) % (2 * math.pi) - math.pi
    
    print(f"  新原点: ({new_origin_x:.3f}, {new_origin_y:.3f}, {math.degrees(new_yaw):.1f}°)")
    
    # 生成输出文件名
    if output_yaml is None:
        base_name = os.path.splitext(yaml_file)[0]
        output_yaml = f"{base_name}_rotated{angle_deg:.0f}.yaml"
    
    output_dir = os.path.dirname(os.path.abspath(output_yaml))
    base_name = os.path.splitext(os.path.basename(output_yaml))[0]
    output_pgm = os.path.join(output_dir, f"{base_name}.pgm")
    
    # 保存旋转后的 PGM
    print(f"\n保存旋转后的地图...")
    write_pgm(output_pgm, rotated_image)
    print(f"  PGM: {output_pgm}")
    
    # 更新 YAML 配置
    new_config = dict(map_config)
    new_config['image'] = os.path.basename(output_pgm)
    new_config['origin'] = [
        float(new_origin_x),
        float(new_origin_y),
        float(new_yaw)
    ]
    
    # 保存 YAML
    with open(output_yaml, 'w') as f:
        # 手动格式化 YAML，保持与原始格式一致
        f.write(f"image: {new_config['image']}\n")
        f.write(f"mode: {new_config.get('mode', 'trinary')}\n")
        f.write(f"resolution: {new_config['resolution']}\n")
        f.write(f"origin: [{new_config['origin'][0]:.6f}, {new_config['origin'][1]:.6f}, {new_config['origin'][2]:.6f}]\n")
        f.write(f"negate: {new_config.get('negate', 0)}\n")
        f.write(f"occupied_thresh: {new_config.get('occupied_thresh', 0.65)}\n")
        f.write(f"free_thresh: {new_config.get('free_thresh', 0.25)}\n")
    
    print(f"  YAML: {output_yaml}")
    
    # 打印总结
    print("\n" + "="*60)
    print("旋转完成!")
    print("="*60)
    print(f"输出文件:")
    print(f"  PGM: {output_pgm}")
    print(f"  YAML: {output_yaml}")
    print(f"\n新地图参数:")
    print(f"  尺寸: {new_width} x {new_height} 像素")
    print(f"  原点: ({new_origin_x:.3f}, {new_origin_y:.3f}, {math.degrees(new_yaw):.1f}°)")
    print(f"  分辨率: {resolution} m/像素")
    print(f"\n使用说明:")
    print(f"  1. 在 nav2_params.yaml 中更新地图路径为: {output_yaml}")
    print(f"  2. 重新启动导航节点以加载新地图")
    print(f"  3. 在 RViz 中检查点云与地图是否对齐")
    print("="*60)
    
    return output_yaml, output_pgm


def main():
    parser = argparse.ArgumentParser(
        description="旋转 ROS2 导航栅格地图 (PGM + YAML)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 顺时针旋转90度（最常用，用于对齐Gazebo坐标系）
  python rotate_map.py map.yaml --angle -90
  
  # 逆时针旋转90度
  python rotate_map.py map.yaml --angle 90
  
  # 旋转180度
  python rotate_map.py map.yaml --angle 180
  
  # 指定输出路径
  python rotate_map.py map.yaml --angle -90 -o rotated_map.yaml
  
  # 使用线性插值（默认使用最近邻）
  python rotate_map.py map.yaml --angle 90 --no-nearest
        """
    )
    
    parser.add_argument('yaml_file', help='输入 YAML 地图文件路径')
    parser.add_argument('--angle', type=float, required=True,
                        help='旋转角度（度），正值为逆时针，负值为顺时针')
    parser.add_argument('-o', '--output', type=str, default=None,
                        help='输出 YAML 文件路径（可选）')
    parser.add_argument('--nearest', action='store_true', default=True,
                        help='使用最近邻插值（默认）')
    parser.add_argument('--no-nearest', action='store_true',
                        help='使用线性插值而非最近邻')
    parser.add_argument('--unknown', type=int, default=205,
                        help='未知区域填充值（默认: 205）')
    
    args = parser.parse_args()
    
    # 检查输入文件
    if not os.path.exists(args.yaml_file):
        print(f"错误: 找不到文件: {args.yaml_file}")
        sys.exit(1)
    
    # 确定插值方法
    use_nearest = args.nearest and not args.no_nearest
    
    try:
        rotate_map(
            yaml_file=args.yaml_file,
            angle_deg=args.angle,
            output_yaml=args.output,
            use_nearest=use_nearest,
            unknown_value=args.unknown
        )
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
