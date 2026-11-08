#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
STL to PCD Converter for Robot Relocalization
==============================================

将 STL 格式的 3D 模型文件转换为 PCD 格式的点云文件，
作为 small_gicp_relocalization 的先验地图。

适配传感器: Livox Mid-360 混合固态激光雷达
- 测距精度: 2 cm
- 角精度: < 0.15°
- 点频: 200,000 点/秒 (10Hz 扫描频率下，单帧约 20,000 点)
- 视场角: 水平 360°，垂直 -7° 至 52°
- 盲区: 0.1 m

作者: Assistant
日期: 2026-04-30
"""

import os
import sys
import struct
import math
import argparse
import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass
from datetime import datetime

# =============================================================================
# 配置参数 - 用户可修改这些参数
# =============================================================================

@dataclass
class Config:
    """配置类 - 包含所有可调参数"""
    
    # --- 输入输出配置 ---
    input_stl_file: str = ""  # STL 文件路径
    output_pcd_file: str = ""  # PCD 输出路径
    
    # --- 模型缩放配置 ---
    # 如果模型单位是 mm，设置为 0.001 转换为米
    # 如果模型单位已经是米，设置为 1.0
    scale_factor: float = 1.0
    
    # --- 采样参数 ---
    # 目标点间距 (米)，建议 0.02m (2cm) 以匹配 Livox Mid-360 的测距精度
    target_point_spacing: float = 0.02
    
    # 采样方法: "uniform" (均匀采样), "poisson" (泊松圆盘采样)
    sampling_method: str = "poisson"
    
    # 泊松圆盘采样的最小距离系数 (相对于 target_point_spacing)
    poisson_min_dist_factor: float = 0.9
    
    # 泊松圆盘采样的最大尝试次数
    poisson_max_attempts: int = 30
    
    # --- 传感器配置 ---
    # 传感器安装位置 (在地图坐标系中的位置，单位: 米)
    # 对于仿真环境，通常地图原点在场地中心
    sensor_position: Tuple[float, float, float] = (0.0, 0.0, 0.5)
    
    # 传感器盲区 (米)，小于此距离的点将被过滤
    sensor_blind_zone: float = 0.1
    
    # --- 点云过滤配置 ---
    # 是否过滤地面以下的点 (z < 0)
    filter_below_ground: bool = True
    ground_level: float = 0.0
    
    # 是否过滤过高的点 (超过传感器垂直视场上限)
    filter_above_sensor_max: bool = False
    sensor_max_height: float = 10.0
    
    # --- PCD 文件配置 ---
    # PCD 文件格式: "ascii" 或 "binary"
    pcd_format: str = "binary"
    
    # 是否添加 intensity 字段 (small_gicp 不使用，但某些应用需要)
    add_intensity: bool = False
    
    # --- 调试配置 ---
    verbose: bool = True


# =============================================================================
# STL 文件解析器
# =============================================================================

class STLParser:
    """STL 文件解析器，支持二进制和 ASCII 格式"""
    
    def __init__(self, filepath: str):
        self.filepath = filepath
        self.triangles: List[np.ndarray] = []  # 每个三角形是 3x3 的顶点数组
        self.normals: List[np.ndarray] = []
        
    def parse(self) -> bool:
        """解析 STL 文件"""
        if not os.path.exists(self.filepath):
            print(f"错误: 文件不存在: {self.filepath}")
            return False
        
        with open(self.filepath, 'rb') as f:
            header = f.read(80)
            f.seek(0)
            
            # 判断是二进制还是 ASCII 格式
            if self._is_binary(header, f):
                return self._parse_binary(f)
            else:
                return self._parse_ascii(f)
    
    def _is_binary(self, header: bytes, f) -> bool:
        """判断是否为二进制 STL 文件"""
        try:
            f.seek(80)  # 跳过头部
            num_triangles = struct.unpack('<I', f.read(4))[0]
            expected_size = 80 + 4 + num_triangles * 50  # 每个三角形 50 字节
            actual_size = os.path.getsize(self.filepath)
            f.seek(0)
            return expected_size == actual_size
        except:
            return False
    
    def _parse_binary(self, f) -> bool:
        """解析二进制 STL 文件"""
        try:
            f.seek(80)  # 跳过头部
            num_triangles = struct.unpack('<I', f.read(4))[0]
            
            print(f"解析二进制 STL: 发现 {num_triangles} 个三角形")
            
            for _ in range(num_triangles):
                # 读取法向量 (12 字节)
                normal = struct.unpack('<3f', f.read(12))
                self.normals.append(np.array(normal))
                
                # 读取三个顶点 (36 字节)
                vertices = []
                for _ in range(3):
                    vertex = struct.unpack('<3f', f.read(12))
                    vertices.append(vertex)
                self.triangles.append(np.array(vertices))
                
                # 跳过属性字节计数 (2 字节)
                f.read(2)
            
            return True
        except Exception as e:
            print(f"解析二进制 STL 失败: {e}")
            return False
    
    def _parse_ascii(self, f) -> bool:
        """解析 ASCII STL 文件"""
        try:
            content = f.read().decode('utf-8', errors='ignore')
            lines = content.split('\n')
            
            current_triangle = []
            current_normal = None
            
            for line in lines:
                line = line.strip().lower()
                
                if line.startswith('facet normal'):
                    # 解析法向量
                    parts = line.split()
                    if len(parts) >= 6:
                        current_normal = np.array([float(parts[2]), float(parts[3]), float(parts[4])])
                
                elif line.startswith('vertex'):
                    # 解析顶点
                    parts = line.split()
                    if len(parts) >= 4:
                        vertex = [float(parts[1]), float(parts[2]), float(parts[3])]
                        current_triangle.append(vertex)
                
                elif line.startswith('endfacet'):
                    # 完成一个三角形
                    if len(current_triangle) == 3:
                        self.triangles.append(np.array(current_triangle))
                        if current_normal is not None:
                            self.normals.append(current_normal)
                        else:
                            # 计算法向量
                            v0 = np.array(current_triangle[0])
                            v1 = np.array(current_triangle[1])
                            v2 = np.array(current_triangle[2])
                            normal = np.cross(v1 - v0, v2 - v0)
                            normal = normal / (np.linalg.norm(normal) + 1e-10)
                            self.normals.append(normal)
                    current_triangle = []
                    current_normal = None
            
            print(f"解析 ASCII STL: 发现 {len(self.triangles)} 个三角形")
            return True
            
        except Exception as e:
            print(f"解析 ASCII STL 失败: {e}")
            return False
    
    def get_bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        """获取模型的边界框"""
        if not self.triangles:
            return np.zeros(3), np.zeros(3)
        
        all_vertices = np.vstack(self.triangles)
        min_bounds = np.min(all_vertices, axis=0)
        max_bounds = np.max(all_vertices, axis=0)
        return min_bounds, max_bounds
    
    def calculate_surface_area(self) -> float:
        """计算模型的总表面积"""
        total_area = 0.0
        for triangle in self.triangles:
            v0, v1, v2 = triangle
            # 使用叉积计算三角形面积
            area = 0.5 * np.linalg.norm(np.cross(v1 - v0, v2 - v0))
            total_area += area
        return total_area


# =============================================================================
# 点云采样器
# =============================================================================

class PointCloudSampler:
    """点云采样器，支持均匀采样和泊松圆盘采样"""
    
    def __init__(self, config: Config):
        self.config = config
        
    def sample(self, triangles: List[np.ndarray]) -> np.ndarray:
        """对三角形网格进行采样"""
        if self.config.sampling_method == "poisson":
            return self._poisson_disk_sampling(triangles)
        else:
            return self._uniform_sampling(triangles)
    
    def _uniform_sampling(self, triangles: List[np.ndarray]) -> np.ndarray:
        """均匀采样 - 基于面积加权"""
        points = []
        spacing = self.config.target_point_spacing
        
        for triangle in triangles:
            v0, v1, v2 = triangle
            
            # 计算三角形面积
            area = 0.5 * np.linalg.norm(np.cross(v1 - v0, v2 - v0))
            if area < 1e-10:
                continue
            
            # 根据面积计算采样点数
            # 等边三角形边长为 a 时，面积 = sqrt(3)/4 * a^2
            # 如果点间距为 d，则每个点占据面积约 sqrt(3)/2 * d^2
            point_area = 0.5 * spacing * spacing * math.sqrt(3)
            num_points = max(1, int(area / point_area))
            
            # 在三角形内随机采样
            for _ in range(num_points):
                # 使用重心坐标在三角形内采样
                r1 = np.random.random()
                r2 = np.random.random()
                if r1 + r2 > 1:
                    r1 = 1 - r1
                    r2 = 1 - r2
                
                point = (1 - r1 - r2) * v0 + r1 * v1 + r2 * v2
                points.append(point)
        
        return np.array(points) if points else np.zeros((0, 3))
    
    def _poisson_disk_sampling(self, triangles: List[np.ndarray]) -> np.ndarray:
        """泊松圆盘采样 - 使用体素网格降采样实现"""
        # 首先进行均匀采样
        points = self._uniform_sampling(triangles)
        
        if len(points) == 0:
            return np.zeros((0, 3))
        
        # 使用体素网格进行降采样（泊松圆盘的近似实现）
        min_dist = self.config.target_point_spacing * self.config.poisson_min_dist_factor
        voxel_size = min_dist
        
        # 计算每个点所属的体素索引
        min_bounds = np.min(points, axis=0)
        voxel_indices = np.floor((points - min_bounds) / voxel_size).astype(np.int32)
        
        # 使用字典按体素分组，每个体素保留一个点（取中心）
        voxel_dict = {}
        for i, voxel_idx in enumerate(voxel_indices):
            key = (voxel_idx[0], voxel_idx[1], voxel_idx[2])
            if key not in voxel_dict:
                voxel_dict[key] = []
            voxel_dict[key].append(i)
        
        # 从每个体素中选择一个点（取该体素内点的平均值）
        selected_indices = []
        for indices in voxel_dict.values():
            # 选择体素内点的中心位置
            selected_indices.append(indices[len(indices) // 2])
        
        selected_points = points[selected_indices]
        
        return selected_points


# =============================================================================
# 点云处理器
# =============================================================================

class PointCloudProcessor:
    """点云处理器 - 过滤和变换点云"""
    
    def __init__(self, config: Config):
        self.config = config
    
    def process(self, points: np.ndarray) -> np.ndarray:
        """处理点云"""
        if len(points) == 0:
            return points
        
        # 应用缩放
        points = points * self.config.scale_factor
        
        # 过滤点
        points = self._filter_points(points)
        
        return points
    
    def _filter_points(self, points: np.ndarray) -> np.ndarray:
        """根据配置过滤点"""
        mask = np.ones(len(points), dtype=bool)
        
        # 过滤盲区内的点
        if self.config.sensor_blind_zone > 0:
            sensor_pos = np.array(self.config.sensor_position)
            distances = np.linalg.norm(points - sensor_pos, axis=1)
            mask &= distances >= self.config.sensor_blind_zone
        
        # 过滤地面以下的点
        if self.config.filter_below_ground:
            mask &= points[:, 2] >= self.config.ground_level
        
        # 过滤过高的点
        if self.config.filter_above_sensor_max:
            mask &= points[:, 2] <= self.config.sensor_max_height
        
        return points[mask]


# =============================================================================
# PCD 文件写入器
# =============================================================================

class PCDWriter:
    """PCD 文件写入器 - 生成 small_gicp_relocalization 兼容的 PCD 文件"""
    
    def __init__(self, config: Config):
        self.config = config
    
    def write(self, points: np.ndarray, filepath: str) -> bool:
        """写入 PCD 文件"""
        if len(points) == 0:
            print("错误: 没有点可以写入")
            return False
        
        try:
            if self.config.pcd_format == "ascii":
                return self._write_ascii(points, filepath)
            else:
                return self._write_binary(points, filepath)
        except Exception as e:
            print(f"写入 PCD 文件失败: {e}")
            return False
    
    def _write_ascii(self, points: np.ndarray, filepath: str) -> bool:
        """写入 ASCII 格式 PCD 文件"""
        num_points = len(points)
        has_intensity = self.config.add_intensity
        
        with open(filepath, 'w') as f:
            # 写入 PCD 头部
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write("VERSION 0.7\n")
            
            if has_intensity:
                f.write("FIELDS x y z intensity\n")
                f.write("SIZE 4 4 4 4\n")
                f.write("TYPE F F F F\n")
                f.write("COUNT 1 1 1 1\n")
            else:
                f.write("FIELDS x y z\n")
                f.write("SIZE 4 4 4\n")
                f.write("TYPE F F F\n")
                f.write("COUNT 1 1 1\n")
            
            f.write(f"WIDTH {num_points}\n")
            f.write("HEIGHT 1\n")
            f.write(f"POINTS {num_points}\n")
            f.write("DATA ascii\n")
            
            # 写入点数据
            for point in points:
                if has_intensity:
                    # 默认 intensity 为 1.0
                    f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} 1.0\n")
                else:
                    f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n")
        
        return True
    
    def _write_binary(self, points: np.ndarray, filepath: str) -> bool:
        """写入二进制格式 PCD 文件"""
        num_points = len(points)
        has_intensity = self.config.add_intensity
        
        with open(filepath, 'wb') as f:
            # 写入 PCD 头部
            header = "# .PCD v0.7 - Point Cloud Data file format\n"
            header += "VERSION 0.7\n"
            
            if has_intensity:
                header += "FIELDS x y z intensity\n"
                header += "SIZE 4 4 4 4\n"
                header += "TYPE F F F F\n"
                header += "COUNT 1 1 1 1\n"
            else:
                header += "FIELDS x y z\n"
                header += "SIZE 4 4 4\n"
                header += "TYPE F F F\n"
                header += "COUNT 1 1 1\n"
            
            header += f"WIDTH {num_points}\n"
            header += "HEIGHT 1\n"
            header += f"POINTS {num_points}\n"
            header += "DATA binary\n"
            
            f.write(header.encode('ascii'))
            
            # 写入二进制点数据
            if has_intensity:
                # 创建包含 intensity 的数组
                intensity = np.ones((num_points, 1), dtype=np.float32)
                data = np.hstack([points.astype(np.float32), intensity])
            else:
                data = points.astype(np.float32)
            
            f.write(data.tobytes())
        
        return True


# =============================================================================
# 主转换器类
# =============================================================================

class STLtoPCDConverter:
    """STL 到 PCD 转换器主类"""
    
    def __init__(self, config: Config):
        self.config = config
        self.parser = STLParser(config.input_stl_file)
        self.sampler = PointCloudSampler(config)
        self.processor = PointCloudProcessor(config)
        self.writer = PCDWriter(config)
    
    def convert(self) -> bool:
        """执行转换"""
        print("=" * 70)
        print("STL 到 PCD 转换器")
        print("适配 small_gicp_relocalization 先验地图生成")
        print("=" * 70)
        print()
        
        # 1. 解析 STL 文件
        print("[1/5] 解析 STL 文件...")
        if not self.parser.parse():
            return False
        
        min_bounds, max_bounds = self.parser.get_bounds()
        surface_area = self.parser.calculate_surface_area()
        
        print(f"  三角形数量: {len(self.parser.triangles)}")
        print(f"  原始模型边界:")
        print(f"    X: [{min_bounds[0]:.3f}, {max_bounds[0]:.3f}]")
        print(f"    Y: [{min_bounds[1]:.3f}, {max_bounds[1]:.3f}]")
        print(f"    Z: [{min_bounds[2]:.3f}, {max_bounds[2]:.3f}]")
        print(f"  原始模型尺寸:")
        print(f"    X: {max_bounds[0] - min_bounds[0]:.3f}")
        print(f"    Y: {max_bounds[1] - min_bounds[1]:.3f}")
        print(f"    Z: {max_bounds[2] - min_bounds[2]:.3f}")
        print(f"  总表面积: {surface_area:.3f} (原始单位)")
        print()
        
        # 2. 智能采样
        print("[2/5] 智能采样...")
        print(f"  采样方法: {self.config.sampling_method}")
        print(f"  目标点间距: {self.config.target_point_spacing} m")
        
        # 计算理论采样点数
        point_area = 0.5 * self.config.target_point_spacing ** 2 * math.sqrt(3)
        estimated_points = int(surface_area * (self.config.scale_factor ** 2) / point_area)
        print(f"  理论采样点数: ~{estimated_points}")
        
        # 如果理论采样点数超过 450 万，自动调整点间距以控制在 300-400 万
        max_target_points = 4000000
        min_target_points = 3000000
        if estimated_points > max_target_points:
            adjusted_spacing = math.sqrt(
                surface_area * self.config.scale_factor ** 2 /
                (max_target_points * 0.5 * math.sqrt(3))
            )
            print(f"  警告: 理论采样点数过多，自动调整点间距为 {adjusted_spacing:.4f} m")
            self.config.target_point_spacing = adjusted_spacing
            self.sampler = PointCloudSampler(self.config)
            point_area = 0.5 * self.config.target_point_spacing ** 2 * math.sqrt(3)
            estimated_points = int(surface_area * (self.config.scale_factor ** 2) / point_area)
            print(f"  调整后理论采样点数: ~{estimated_points}")
        elif estimated_points < min_target_points:
            adjusted_spacing = math.sqrt(
                surface_area * self.config.scale_factor ** 2 /
                (max_target_points * 0.5 * math.sqrt(3))
            )
            print(f"  提示: 理论采样点数较少，自动调整点间距为 {adjusted_spacing:.4f} m")
            self.config.target_point_spacing = adjusted_spacing
            self.sampler = PointCloudSampler(self.config)
            point_area = 0.5 * self.config.target_point_spacing ** 2 * math.sqrt(3)
            estimated_points = int(surface_area * (self.config.scale_factor ** 2) / point_area)
            print(f"  调整后理论采样点数: ~{estimated_points}")
        
        points = self.sampler.sample(self.parser.triangles)
        print(f"  实际采样点数: {len(points)}")
        print()
        
        # 3. 处理点云
        print("[3/5] 处理点云...")
        print(f"  缩放因子: {self.config.scale_factor}")
        print(f"  传感器位置: {self.config.sensor_position}")
        print(f"  盲区过滤: {self.config.sensor_blind_zone} m")
        
        points_before = len(points)
        points = self.processor.process(points)
        points_after = len(points)
        
        print(f"  过滤前点数: {points_before}")
        print(f"  过滤后点数: {points_after}")
        print(f"  过滤点数: {points_before - points_after}")
        print()
        
        if len(points) == 0:
            print("错误: 处理后没有点剩余")
            return False
        
        # 4. 计算统计信息
        print("[4/5] 计算统计信息...")
        min_bounds_proc = np.min(points, axis=0)
        max_bounds_proc = np.max(points, axis=0)
        dimensions = max_bounds_proc - min_bounds_proc
        volume = dimensions[0] * dimensions[1] * dimensions[2]
        density = len(points) / volume if volume > 0 else 0
        
        print(f"  缩放后模型边界:")
        print(f"    X: [{min_bounds_proc[0]:.3f}, {max_bounds_proc[0]:.3f}]")
        print(f"    Y: [{min_bounds_proc[1]:.3f}, {max_bounds_proc[1]:.3f}]")
        print(f"    Z: [{min_bounds_proc[2]:.3f}, {max_bounds_proc[2]:.3f}]")
        print(f"  缩放后模型尺寸:")
        print(f"    X: {dimensions[0]:.3f} m")
        print(f"    Y: {dimensions[1]:.3f} m")
        print(f"    Z: {dimensions[2]:.3f} m")
        print(f"  生成总点数: {len(points)}")
        print(f"  点云密度: {density:.2f} 点/m³")
        print(f"  平均点间距: ~{self.config.target_point_spacing} m")
        print()
        
        # 5. 写入 PCD 文件
        print("[5/5] 写入 PCD 文件...")
        print(f"  输出路径: {self.config.output_pcd_file}")
        print(f"  文件格式: {self.config.pcd_format}")
        
        # 确保输出目录存在
        output_dir = os.path.dirname(self.config.output_pcd_file)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir)
        
        if not self.writer.write(points, self.config.output_pcd_file):
            return False
        
        file_size = os.path.getsize(self.config.output_pcd_file)
        print(f"  文件大小: {file_size / 1024 / 1024:.2f} MB")
        print()
        
        print("=" * 70)
        print("转换完成!")
        print("=" * 70)
        
        return True


# =============================================================================
# 命令行接口
# =============================================================================

def create_default_config() -> Config:
    """创建默认配置"""
    return Config()


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description="STL to PCD Converter for small_gicp_relocalization",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 基本用法
  python stl_to_pcd_converter.py input.stl output.pcd
  
  # 指定采样方法和点间距
  python stl_to_pcd_converter.py input.stl output.pcd --method poisson --spacing 0.02
  
  # 处理 mm 单位的模型并指定传感器位置
  python stl_to_pcd_converter.py input.stl output.pcd --scale 0.001 --sensor-pos "0,0,0.5"
  
  # 使用 ASCII 格式输出
  python stl_to_pcd_converter.py input.stl output.pcd --format ascii
        """
    )
    
    parser.add_argument("input", help="输入 STL 文件路径")
    parser.add_argument("output", help="输出 PCD 文件路径")
    parser.add_argument("--scale", type=float, default=1.0, help="缩放因子 (默认: 1.0)")
    parser.add_argument("--spacing", type=float, default=0.02, help="目标点间距，单位米 (默认: 0.02)")
    parser.add_argument("--method", choices=["uniform", "poisson"], default="poisson",
                        help="采样方法 (默认: poisson)")
    parser.add_argument("--sensor-pos", type=str, default="0,0,0.5",
                        help="传感器位置，格式为 x,y,z (默认: 0,0,0.5)")
    parser.add_argument("--blind-zone", type=float, default=0.1, help="传感器盲区，单位米 (默认: 0.1)")
    parser.add_argument("--format", choices=["ascii", "binary"], default="binary",
                        help="PCD 文件格式 (默认: binary)")
    parser.add_argument("--no-ground-filter", action="store_true",
                        help="不过滤地面以下的点")
    parser.add_argument("--add-intensity", action="store_true",
                        help="添加 intensity 字段")
    
    args = parser.parse_args()
    
    # 创建配置
    config = Config()
    config.input_stl_file = args.input
    config.output_pcd_file = args.output
    config.scale_factor = args.scale
    config.target_point_spacing = args.spacing
    config.sampling_method = args.method
    config.sensor_blind_zone = args.blind_zone
    config.pcd_format = args.format
    config.filter_below_ground = not args.no_ground_filter
    config.add_intensity = args.add_intensity
    
    # 解析传感器位置
    try:
        pos = [float(x.strip()) for x in args.sensor_pos.split(",")]
        if len(pos) == 3:
            config.sensor_position = tuple(pos)
        else:
            print("警告: 传感器位置格式错误，使用默认值 (0,0,0.5)")
    except:
        print("警告: 传感器位置格式错误，使用默认值 (0,0,0.5)")
    
    # 执行转换
    converter = STLtoPCDConverter(config)
    success = converter.convert()
    
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()