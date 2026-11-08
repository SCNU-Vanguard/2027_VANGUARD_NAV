#!/usr/bin/env python3
"""把栅格图与先验点云叠成一张 PNG，用于肉眼判断是否对齐。

背景：栅格图（白=空闲，黑=占用，灰=未知）
前景：点云低处墙体（红色），可用 --dx/--dy 施加平移做对比
"""

import argparse
import os

import numpy as np

from check_map_alignment import load_pgm, load_yaml_simple, load_pcd_xyz


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--yaml', required=True)
    ap.add_argument('--pcd', required=True)
    ap.add_argument('--dx', type=float, default=0.0)
    ap.add_argument('--dy', type=float, default=0.0)
    ap.add_argument('--out', required=True)
    ap.add_argument('--scale', type=int, default=4)
    ap.add_argument('--z-min', type=float, default=0.35)
    ap.add_argument('--z-max', type=float, default=2.0)
    args = ap.parse_args()

    meta = load_yaml_simple(args.yaml)
    image = None
    for line in open(args.yaml):
        if line.startswith('image'):
            image = line.split(':', 1)[1].strip()
    img = load_pgm(os.path.join(os.path.dirname(args.yaml), image))
    h, w = img.shape
    res = meta['resolution']

    rgb = np.stack([img, img, img], axis=-1).astype(np.uint8)

    pts = load_pcd_xyz(args.pcd)
    wall = pts[(pts[:, 2] > args.z_min) & (pts[:, 2] < args.z_max)][:, :2]
    wall = wall + np.array([args.dx, args.dy])

    cols = np.round((wall[:, 0] - meta['origin'][0]) / res - 0.5).astype(np.int64)
    rows = np.round((h - 1) - ((wall[:, 1] - meta['origin'][1]) / res - 0.5)).astype(np.int64)
    ok = (cols >= 0) & (cols < w) & (rows >= 0) & (rows < h)
    rgb[rows[ok], cols[ok]] = [255, 40, 40]

    if args.scale > 1:
        rgb = rgb.repeat(args.scale, axis=0).repeat(args.scale, axis=1)
    # 写 PNG（PIL 不可用时回退到 ppm）
    try:
        from PIL import Image
        Image.fromarray(rgb).save(args.out)
        print(f'已写出 {args.out}  ({rgb.shape[1]}x{rgb.shape[0]})')
    except ImportError:
        out = args.out.rsplit('.', 1)[0] + '.ppm'
        with open(out, 'wb') as f:
            f.write(f'P6\n{rgb.shape[1]} {rgb.shape[0]}\n255\n'.encode())
            f.write(rgb.tobytes())
        print(f'PIL 不可用，已写出 {out}')


if __name__ == '__main__':
    main()
