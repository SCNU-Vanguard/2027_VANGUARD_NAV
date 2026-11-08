#!/usr/bin/env python3
"""用二维互相关求「先验点云 → 栅格地图」的平移偏差。

比最近邻中位数稳：先把两边栅格化成同一分辨率的二值占据图，
再用 FFT 做互相关，峰值位置就是点云需要整体平移的量。

用法:
    python3 align_pcd_to_pgm.py \
        --yaml  .../map/simulation/rmuc_2026.yaml \
        --pcd   .../pcd/simulation/rmuc_2026_mapframe.pcd
"""

import argparse
import os

import numpy as np
from scipy.signal import fftconvolve

from check_map_alignment import load_pgm, load_yaml_simple, load_pcd_xyz


def rasterize(xy, x0, y0, res, shape):
    cols = np.round((xy[:, 0] - x0) / res).astype(np.int64)
    rows = np.round((xy[:, 1] - y0) / res).astype(np.int64)
    ok = (cols >= 0) & (cols < shape[1]) & (rows >= 0) & (rows < shape[0])
    img = np.zeros(shape, dtype=np.float32)
    img[rows[ok], cols[ok]] = 1.0
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--yaml', required=True)
    ap.add_argument('--pcd', required=True)
    ap.add_argument('--res', type=float, default=0.05)
    ap.add_argument('--z-min', type=float, default=0.35)
    ap.add_argument('--z-max', type=float, default=2.0,
                    help='上限压到 2 m：只取低处墙体，避免高地/装饰件参与')
    ap.add_argument('--max-shift', type=float, default=3.0)
    args = ap.parse_args()

    meta = load_yaml_simple(args.yaml)
    image = None
    for line in open(args.yaml):
        if line.startswith('image'):
            image = line.split(':', 1)[1].strip()
    img = load_pgm(os.path.join(os.path.dirname(args.yaml), image))
    occ = ((255.0 - img.astype(np.float64)) / 255.0 > meta['occupied_thresh'])

    res = args.res
    pts = load_pcd_xyz(args.pcd)
    wall = pts[(pts[:, 2] > args.z_min) & (pts[:, 2] < args.z_max)]
    print(f'墙点(z {args.z_min}~{args.z_max} m): {len(wall)}')

    # 公共栅格：把两边都放进来，四周留 --max-shift 的余量
    pad = args.max_shift + 1.0
    x0 = min(meta['origin'][0], wall[:, 0].min()) - pad
    y0 = min(meta['origin'][1], wall[:, 1].min()) - pad
    x1 = max(meta['origin'][0] + img.shape[1] * meta['resolution'],
             wall[:, 0].max()) + pad
    y1 = max(meta['origin'][1] + img.shape[0] * meta['resolution'],
             wall[:, 1].max()) + pad
    W = int(round((x1 - x0) / res)) + 1
    H = int(round((y1 - y0) / res)) + 1

    # 栅格图占用格 -> 公共栅格
    oi = np.argwhere(occ)
    ox = meta['origin'][0] + (oi[:, 1] + 0.5) * meta['resolution']
    oy = meta['origin'][1] + (img.shape[0] - 1 - oi[:, 0] + 0.5) * meta['resolution']
    A = rasterize(np.column_stack([ox, oy]), x0, y0, res, (H, W))
    B = rasterize(wall[:, :2], x0, y0, res, (H, W))
    print(f'公共栅格 {W}x{H} @ {res} m，栅格图占用 {int(A.sum())}，点云墙点 {int(B.sum())}')

    C = fftconvolve(A, B[::-1, ::-1], mode='full')
    peak = np.unravel_index(np.argmax(C), C.shape)
    # 峰位置 -> 位移：C[i,j] 对应 d = (i-(H-1), j-(W-1))
    dy_cells = peak[0] - (H - 1)
    dx_cells = peak[1] - (W - 1)
    dx, dy = dx_cells * res, dy_cells * res

    print(f'\n互相关峰值: 点云需平移 dx={dx:+.2f} m, dy={dy:+.2f} m')
    print(f'  峰值重叠度 {C[peak] / max(1.0, B.sum()):.3f}')

    def overlap(ddx, ddy):
        Bs = np.roll(np.roll(B, int(round(ddy / res)), axis=0),
                     int(round(ddx / res)), axis=1)
        # 只统计 A 有占用的区域，看 B 覆盖了多少
        hit = float((A * Bs).sum() / max(1.0, A.sum()))
        return hit

    print('\n=== 验证：栅格占用被点云覆盖的比例 ===')
    for name, (ddx, ddy) in [
        ('当前（不补偿）', (0.0, 0.0)),
        (f'补偿后 ({dx:+.2f},{dy:+.2f})', (dx, dy)),
        ('反方向', (-dx, -dy)),
    ]:
        print(f'  {name:<28s} 覆盖率 {overlap(ddx, ddy):.3f}')

    print('\n建议：把先验点云整体平移 '
          f'({dx:+.2f}, {dy:+.2f}) 后重新生成 bundle。')


if __name__ == '__main__':
    main()
