#!/usr/bin/env python3
"""检查栅格地图（pgm/yaml）与先验点云（pcd/bundle）是否在同一 map 坐标系。

用法:
    python3 check_map_alignment.py --map-dir <含 pgm+yaml 的目录> \
        --pcd <先验点云>

输出：
  - 栅格图占用格的世界坐标范围
  - 点云在 map 系下的范围
  - 格点搜索得到的最佳 (dx, dy, dyaw) 平移/旋转补偿量
"""

import argparse
import os
import struct

import numpy as np
from scipy.spatial import cKDTree


def load_pgm(path):
    raw = open(path, 'rb').read()
    # P5 头：magic, 可能有注释, 宽 高, 最大值
    tokens = []
    i = 0
    while len(tokens) < 4:
        while i < len(raw) and raw[i:i + 1].isspace():
            i += 1
        if raw[i:i + 1] == b'#':
            while i < len(raw) and raw[i:i + 1] != b'\n':
                i += 1
            continue
        j = i
        while j < len(raw) and not raw[j:j + 1].isspace():
            j += 1
        tokens.append(raw[i:j])
        i = j
    i += 1  # 跳过一个空白
    w, h = int(tokens[1]), int(tokens[2])
    img = np.frombuffer(raw[i:i + w * h], dtype=np.uint8).reshape(h, w)
    return img


def load_yaml_simple(path):
    vals = {}
    for line in open(path):
        if ':' in line and not line.strip().startswith('#'):
            k, v = line.split(':', 1)
            vals[k.strip()] = v.strip()
    origin = vals['origin'].strip('[]').split(',')
    return {
        'resolution': float(vals['resolution']),
        'origin': [float(o) for o in origin],
        'negate': int(vals.get('negate', 0)),
        'occupied_thresh': float(vals.get('occupied_thresh', 0.65)),
    }


def load_pcd_xyz(path):
    raw = open(path, 'rb').read()
    i = raw.find(b'DATA ')
    j = raw.find(b'\n', i) + 1
    hdr = {}
    for line in raw[:j].decode(errors='ignore').splitlines():
        p = line.split()
        if p:
            hdr[p[0]] = p[1:]
    names = hdr['FIELDS']
    n = int(hdr['POINTS'][0])
    nf = len(names)
    body = raw[j:]
    arr = np.frombuffer(body[:n * nf * 4], dtype=np.float32).reshape(n, nf)
    return arr[:, [names.index(a) for a in ('x', 'y', 'z')]].astype(np.float64)


def pgm_to_world(occ_index, meta, shape):
    """像素 (row, col) -> 世界坐标（栅格中心）。"""
    res = meta['resolution']
    rows, cols = shape
    r, c = occ_index[:, 0], occ_index[:, 1]
    x = meta['origin'][0] + (c + 0.5) * res
    y = meta['origin'][1] + (rows - 1 - r + 0.5) * res
    return np.column_stack([x, y])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map-dir', required=True)
    ap.add_argument('--yaml', default=None,
                    help='显式指定 map 的 yaml；目录里有多张图时必填')
    ap.add_argument('--pcd', required=True)
    ap.add_argument('--z-min', type=float, default=0.35,
                    help='只取这个高度以上的点当墙体（排除地面）')
    ap.add_argument('--z-max', type=float, default=3.0)
    args = ap.parse_args()

    yaml_path = args.yaml
    if yaml_path is None:
        for f in sorted(os.listdir(args.map_dir)):
            if f.endswith('.yaml'):
                yaml_path = os.path.join(args.map_dir, f)
                break
    meta = load_yaml_simple(yaml_path)
    image_name = None
    for l in open(yaml_path):
        if l.startswith('image'):
            image_name = l.split(':', 1)[1].strip()
    pgm_path = os.path.join(args.map_dir, image_name)
    img = load_pgm(pgm_path)

    occ_frac = (255.0 - img.astype(np.float64)) / 255.0
    if meta['negate']:
        occ_frac = 1.0 - occ_frac
    occ_idx = np.argwhere(occ_frac > meta['occupied_thresh'])
    occ_xy = pgm_to_world(occ_idx, meta, img.shape)

    print('=== 栅格地图 ===')
    print(f'  尺寸 {img.shape[1]}x{img.shape[0]}  分辨率 {meta["resolution"]}  '
          f'origin {meta["origin"][:2]}')
    print(f'  范围 x[{meta["origin"][0]:.2f},{meta["origin"][0] + img.shape[1] * meta["resolution"]:.2f}] '
          f'y[{meta["origin"][1]:.2f},{meta["origin"][1] + img.shape[0] * meta["resolution"]:.2f}]')
    print(f'  占用格 {len(occ_xy)}')
    print(f'  占用区域 x[{occ_xy[:, 0].min():.2f},{occ_xy[:, 0].max():.2f}] '
          f'y[{occ_xy[:, 1].min():.2f},{occ_xy[:, 1].max():.2f}]')

    pts = load_pcd_xyz(args.pcd)
    print('\n=== 先验点云 ===')
    print(f'  总点数 {len(pts)}')
    print(f'  全范围 x[{pts[:, 0].min():.2f},{pts[:, 0].max():.2f}] '
          f'y[{pts[:, 1].min():.2f},{pts[:, 1].max():.2f}] '
          f'z[{pts[:, 2].min():.2f},{pts[:, 2].max():.2f}]')
    wall = pts[(pts[:, 2] > args.z_min) & (pts[:, 2] < args.z_max)]
    print(f'  墙体点(z {args.z_min}~{args.z_max}) {len(wall)}')

    # 只保留落在栅格范围附近的墙点，避免远处边缘干扰
    pad = 2.0
    x0 = meta['origin'][0] - pad
    y0 = meta['origin'][1] - pad
    x1 = meta['origin'][0] + img.shape[1] * meta['resolution'] + pad
    y1 = meta['origin'][1] + img.shape[0] * meta['resolution'] + pad
    m = (wall[:, 0] > x0) & (wall[:, 0] < x1) & (wall[:, 1] > y0) & (wall[:, 1] < y1)
    wall = wall[m]
    if len(wall) > 60000:
        wall = wall[np.random.default_rng(0).choice(len(wall), 60000, replace=False)]
    print(f'  参与比对的墙点 {len(wall)}')

    tree = cKDTree(occ_xy)

    def score(dx, dy, yaw_deg):
        a = np.radians(yaw_deg)
        c, s = np.cos(a), np.sin(a)
        R = np.array([[c, -s], [s, c]])
        p = wall[:, :2] @ R.T + np.array([dx, dy])
        d, _ = tree.query(p, k=1)
        return float(np.median(d)), float(np.mean(d < 0.25))

    print('\n=== 对齐搜索（把点云整体平移/旋转去贴栅格）===')
    base_med, base_hit = score(0.0, 0.0, 0.0)
    print(f'  不补偿: 中位距离 {base_med:.3f} m, 命中率(0.25m) {base_hit:.3f}')

    best = (base_med, 0.0, 0.0, 0.0)
    for yaw in (0.0, 90.0, 180.0, -90.0):
        # 粗搜
        for dx in np.arange(-3.0, 3.01, 0.25):
            for dy in np.arange(-3.0, 3.01, 0.25):
                med, _ = score(dx, dy, yaw)
                if med < best[0]:
                    best = (med, dx, dy, yaw)
        # 仅对当前最优 yaw 精搜
        if best[3] == yaw:
            cx, cy = best[1], best[2]
            for dx in np.arange(cx - 0.25, cx + 0.26, 0.05):
                for dy in np.arange(cy - 0.25, cy + 0.26, 0.05):
                    med, _ = score(dx, dy, yaw)
                    if med < best[0]:
                        best = (med, dx, dy, yaw)

    med, dx, dy, yaw = best
    _, hit = score(dx, dy, yaw)
    print(f'  最优补偿: dx={dx:+.2f} dy={dy:+.2f} dyaw={yaw:+.0f}° '
          f'→ 中位 {med:.3f} m, 命中率 {hit:.3f}')

    if abs(dx) < 0.06 and abs(dy) < 0.06 and yaw == 0.0:
        print('\n结论: 点云与栅格图已对齐（残余 < 6 cm），不需要补偿。')
    else:
        print(f'\n结论: 点云相对栅格图偏了 (dx={dx:+.2f}, dy={dy:+.2f}, dyaw={yaw:+.0f}°)。')
        print('      修正办法：把先验点云按 (-dx, -dy, -dyaw) 变换后重新生成 bundle，')
        print('      或在生成前调整点云的平移量。')


if __name__ == '__main__':
    main()
