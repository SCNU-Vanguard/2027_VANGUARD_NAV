#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PCD点云地图旋转工具
==================

功能说明:
---------
旋转PCD点云地图文件，用于对齐 small_gicp_relocalization 的先验地图。

使用方法:
---------
1. 旋转PCD文件（默认90度）:
   python rotate_pcd.py input.pcd --angle -90 -o output.pcd

2. 指定旋转中心:
   python rotate_pcd.py input.pcd --angle 180 --center 0 0 0.5

3. 同时应用平移:
   python rotate_pcd.py input.pcd --angle -90 --translate 1.0 0.0 0.0

参数说明:
---------
- angle: 旋转角度（度），正值为逆时针，负值为顺时针
- center: 旋转中心点 (x, y, z)，默认为点云中心
- translate: 旋转后的平移量 (x, y, z)
- output: 输出文件路径

注意事项:
---------
1. 仅支持ASCII格式的PCD文件
2. 旋转后点云坐标系会改变
3. 建议先备份原始文件

作者: Assistant
日期: 2026-05-01
"""

import os
import sys
import argparse
import math
import numpy as np
from typing import Tuple, Optional


def read_pcd_ascii(filename: str) -> Tuple[np.ndarray, str]:
    """
    读取ASCII格式的PCD文件
    
    返回:
        points: Nx3的点云数组
        header: PCD文件头部信息（用于写入）
    """
    with open(filename, 'r') as f:
        lines = f.readlines()
    
    # 解析头部
    header_lines = []
    data_start = 0
    has_intensity = False
    
    for i, line in enumerate(lines):
        header_lines.append(line)
        if line.strip().startswith('DATA'):
            data_start = i + 1
            break
        if line.strip().startswith('FIELDS'):
            fields = line.strip().split()[1:]
            if 'intensity' in fields:
                has_intensity = True
    
    header = ''.join(header_lines)
    
    # 读取点数据
    points = []
    intensities = []
    
    for line in lines[data_start:]:
        line = line.strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) >= 3:
            x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
            points.append([x, y, z])
            if has_intensity and len(parts) >= 4:
                intensities.append(float(parts[3]))
    
    points = np.array(points)
    intensities = np.array(intensities) if intensities else None
    
    return points, intensities, header


def write_pcd_ascii(filename: str, points: np.ndarray, 
                    intensities: Optional[np.ndarray], header: str):
    """
    写入ASCII格式的PCD文件
    """
    # 更新头部中的点数
    header_lines = header.split('\n')
    new_header_lines = []
    
    for line in header_lines:
        if line.strip().startswith('WIDTH'):
            new_header_lines.append(f'WIDTH {len(points)}')
        elif line.strip().startswith('POINTS'):
            new_header_lines.append(f'POINTS {len(points)}')
        else:
            new_header_lines.append(line)
    
    header = '\n'.join(new_header_lines)
    
    with open(filename, 'w') as f:
        f.write(header)
        
        for i, point in enumerate(points):
            if intensities is not None and i < len(intensities):
                f.write(f'{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} {intensities[i]:.6f}\n')
            else:
                f.write(f'{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n')


def rotate_points(points: np.ndarray, angle_deg: float, 
                  center: Optional[np.ndarray] = None,
                  translation: Optional[np.ndarray] = None) -> np.ndarray:
    """
    旋转点云
    
    参数:
    -----
    points: Nx3的点云数组
    angle_deg: 旋转角度（度），绕Z轴旋转
    center: 旋转中心 (3,)，默认为点云中心
    translation: 旋转后的平移量 (3,)
    
    返回:
    -----
    rotated_points: 旋转后的点云
    """
    angle_rad = math.radians(angle_deg)
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    
    # 旋转矩阵（绕Z轴）
    R = np.array([
        [cos_a, -sin_a, 0],
        [sin_a, cos_a, 0],
        [0, 0, 1]
    ])
    
    # 确定旋转中心
    if center is None:
        center = np.mean(points, axis=0)
    else:
        center = np.array(center)
    
    # 平移到原点 -> 旋转 -> 平移回去
    rotated = (points - center) @ R.T + center
    
    # 应用额外平移
    if translation is not None:
        rotated = rotated + np.array(translation)
    
    return rotated


def main():
    parser = argparse.ArgumentParser(
        description='旋转PCD点云地图',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
示例:
  # 顺时针旋转90度
  python rotate_pcd.py input.pcd --angle -90
  
  # 旋转180度并指定输出
  python rotate_pcd.py input.pcd --angle 180 -o output.pcd
  
  # 以指定点为中心旋转
  python rotate_pcd.py input.pcd --angle -90 --center 0 0 0.5
  
  # 旋转后平移
  python rotate_pcd.py input.pcd --angle 90 --translate 1.0 0.0 0.0
        '''
    )
    
    parser.add_argument('input', help='输入PCD文件路径')
    parser.add_argument('--angle', type=float, required=True,
                        help='旋转角度（度），正值为逆时针')
    parser.add_argument('-o', '--output', type=str, default=None,
                        help='输出PCD文件路径')
    parser.add_argument('--center', type=float, nargs=3, default=None,
                        metavar=('X', 'Y', 'Z'),
                        help='旋转中心点 (x y z)，默认为点云中心')
    parser.add_argument('--translate', type=float, nargs=3, default=None,
                        metavar=('X', 'Y', 'Z'),
                        help='旋转后的平移量 (x y z)')
    
    args = parser.parse_args()
    
    # 检查输入文件
    if not os.path.exists(args.input):
        print(f"错误: 找不到文件: {args.input}")
        sys.exit(1)
    
    # 确定输出文件名
    if args.output is None:
        base, ext = os.path.splitext(args.input)
        args.output = f"{base}_rotated{args.angle:.0f}{ext}"
    
    print(f"读取PCD文件: {args.input}")
    points, intensities, header = read_pcd_ascii(args.input)
    print(f"  点数: {len(points)}")
    
    # 计算点云边界
    min_bounds = np.min(points, axis=0)
    max_bounds = np.max(points, axis=0)
    print(f"  原始边界:")
    print(f"    X: [{min_bounds[0]:.3f}, {max_bounds[0]:.3f}]")
    print(f"    Y: [{min_bounds[1]:.3f}, {max_bounds[1]:.3f}]")
    print(f"    Z: [{min_bounds[2]:.3f}, {max_bounds[2]:.3f}]")
    
    # 旋转点云
    print(f"\n旋转角度: {args.angle}°")
    
    center = np.array(args.center) if args.center else None
    translation = np.array(args.translate) if args.translate else None
    
    if center is not None:
        print(f"  旋转中心: ({center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f})")
    else:
        print(f"  旋转中心: 点云中心 ({np.mean(points, axis=0)})")
    
    if translation is not None:
        print(f"  平移量: ({translation[0]:.3f}, {translation[1]:.3f}, {translation[2]:.3f})")
    
    rotated_points = rotate_points(points, args.angle, center, translation)
    
    # 计算新边界
    new_min = np.min(rotated_points, axis=0)
    new_max = np.max(rotated_points, axis=0)
    print(f"\n旋转后边界:")
    print(f"    X: [{new_min[0]:.3f}, {new_max[0]:.3f}]")
    print(f"    Y: [{new_min[1]:.3f}, {new_max[1]:.3f}]")
    print(f"    Z: [{new_min[2]:.3f}, {new_max[2]:.3f}]")
    
    # 保存
    print(f"\n保存到: {args.output}")
    write_pcd_ascii(args.output, rotated_points, intensities, header)
    
    print("\n" + "="*60)
    print("旋转完成!")
    print("="*60)
    print(f"输出文件: {args.output}")
    print(f"\n使用说明:")
    print(f"  1. 在 small_gicp_relocalization 参数中更新 prior_pcd_file")
    print(f"  2. 重新启动定位节点以加载新地图")
    print("="*60)


if __name__ == '__main__':
    main()
