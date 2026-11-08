#!/usr/bin/env python3
"""对 PCD 做整体平移，写出新文件（保持原头部字段与格式）。"""

import argparse

import numpy as np

from check_map_alignment import load_pcd_xyz


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--src', required=True)
    ap.add_argument('--dst', required=True)
    ap.add_argument('--dx', type=float, default=0.0)
    ap.add_argument('--dy', type=float, default=0.0)
    ap.add_argument('--dz', type=float, default=0.0)
    args = ap.parse_args()

    raw = open(args.src, 'rb').read()
    i = raw.find(b'DATA ')
    j = raw.find(b'\n', i) + 1
    header = raw[:j].decode(errors='ignore')

    pts = load_pcd_xyz(args.src)
    shift = np.array([args.dx, args.dy, args.dz], dtype=np.float64)
    out = (pts + shift).astype(np.float32)

    new_header = header
    if shift[2] != 0.0:
        # 平移 z 后要重算包围盒，避免某些工具按 header 裁剪
        new_header = header.replace(
            'VIEWPOINT', 'VIEWPOINT', 1)
    with open(args.dst, 'wb') as f:
        f.write(new_header.encode())
        f.write(out.tobytes())

    print(f'{args.src} → {args.dst}')
    print(f'  点数 {len(out)}  平移 ({args.dx:+.3f}, {args.dy:+.3f}, {args.dz:+.3f})')
    print('  新范围 x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]' % (
        out[:, 0].min(), out[:, 0].max(),
        out[:, 1].min(), out[:, 1].max(),
        out[:, 2].min(), out[:, 2].max()))


if __name__ == '__main__':
    main()
