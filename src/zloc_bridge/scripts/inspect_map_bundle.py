#!/usr/bin/env python3
"""离线核查 Z-Localization 先验地图包：子图覆盖度与合成扫描的支持率。

用途：定位 `support_ok=false / corr_ratio 偏低` 这类初始定位失败时，判断是
    (a) 目标子图太稀（关键帧采样不足），还是
    (b) 扫描与地图整体错位（外参 / 坐标轴 / 高度）。
用法：
    python3 inspect_map_bundle.py <bundle_dir> [--pose x y z yaw_deg] [--kf 12] [--radius 12]
"""
import argparse
import struct

import numpy as np


def lzf_decompress(data, out_len):
    out = bytearray(out_len)
    ip = op = 0
    n = len(data)
    while ip < n:
        ctrl = data[ip]
        ip += 1
        if ctrl < 32:
            ln = ctrl + 1
            out[op:op + ln] = data[ip:ip + ln]
            ip += ln
            op += ln
        else:
            ln = ctrl >> 5
            ref = op - ((ctrl & 0x1F) << 8) - 1
            if ln == 7:
                ln += data[ip]
                ip += 1
            ref -= data[ip]
            ip += 1
            ln += 2
            for _ in range(ln):
                out[op] = out[ref]
                op += 1
                ref += 1
    return bytes(out[:op])


def load_pcd(path):
    """支持 ascii / binary / binary_compressed 三种存储，返回 Nx3 float64。"""
    raw = open(path, 'rb').read()
    if raw.find(b'DATA binary') < 0:
        return np.loadtxt(path, skiprows=11, usecols=(0, 1, 2))
    i = raw.find(b'DATA ')
    j = raw.find(b'\n', i) + 1
    lines = {}
    for line in raw[:j].decode(errors='ignore').splitlines():
        parts = line.split()
        if parts:
            lines[parts[0]] = parts[1:]
    body = raw[j:]
    names = lines['FIELDS']
    n = int(lines['POINTS'][0])
    nf = len(names)
    mode = lines['DATA'][0]
    if mode.endswith('compressed'):
        csize, usize = struct.unpack('<II', body[:8])
        dec = lzf_decompress(body[8:8 + csize], usize)
        arr = np.frombuffer(dec, dtype=np.float32).reshape(nf, n).T  # field-major
    else:
        arr = np.frombuffer(body[:n * nf * 4], dtype=np.float32).reshape(n, nf)
    return arr[:, [names.index(a) for a in ('x', 'y', 'z')]].astype(np.float64)


def quat_to_rot(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def load_keyframes(bundle):
    rows = [l.split() for l in open(f'{bundle}/keyframes.txt') if not l.startswith('#')]
    pose = np.array([[float(v) for v in r[2:5]] for r in rows])
    quat = np.array([[float(v) for v in r[5:9]] for r in rows])
    return pose, quat


def synth_scan(cloud, origin, az_res=0.45, el_res=0.7, el_lo=-7.0, el_hi=52.0, max_r=45.0):
    """按 Mid-360 视场做角度分箱取最近点，近似一帧真实扫描。"""
    delta = cloud - origin
    rng = np.linalg.norm(delta, axis=1)
    az = np.degrees(np.arctan2(delta[:, 1], delta[:, 0]))
    el = np.degrees(np.arcsin(np.clip(delta[:, 2] / np.maximum(rng, 1e-9), -1, 1)))
    keep = (rng > 0.3) & (rng < max_r) & (el > el_lo) & (el < el_hi)
    pts, rng, az, el = cloud[keep], rng[keep], az[keep], el[keep]
    key = (np.floor((az + 180) / az_res).astype(np.int64) * 1000
           + np.floor((el - el_lo) / el_res).astype(np.int64))
    order = np.lexsort((rng, key))
    ks = key[order]
    first = np.ones(len(ks), bool)
    first[1:] = ks[1:] != ks[:-1]
    return pts[order][first]


def voxelize(cloud, res):
    _, keep = np.unique(np.floor(cloud / res).astype(np.int64), axis=0, return_index=True)
    return cloud[keep]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bundle')
    ap.add_argument('--pose', nargs=4, type=float, default=[0.0, 0.0, 0.0, 0.0],
                    metavar=('X', 'Y', 'Z', 'YAW_DEG'))
    ap.add_argument('--kf', type=int, default=12, help='relocalization_submap_keyframes')
    ap.add_argument('--radius', type=float, default=12.0, help='relocalization_submap_radius')
    ap.add_argument('--voxel', type=float, default=0.15)
    ap.add_argument('--gate', type=float, default=0.6, help='max_correspondence_distance')
    args = ap.parse_args()

    from scipy.spatial import cKDTree

    kf_pose, kf_quat = load_keyframes(args.bundle)
    rng = np.linalg.norm(kf_pose[:, :2] - np.array(args.pose[:2]), axis=1)
    candidates = np.where(rng <= args.radius)[0]
    if len(candidates) == 0:
        print(f'没有 {args.radius} m 内的关键帧')
        return
    sel = candidates[np.argsort(rng[candidates])[:args.kf]]
    print(f'选中 {len(sel)} 个关键帧，最近 {rng[sel].min():.2f} m，最远 {rng[sel].max():.2f} m')

    sub = np.vstack([load_pcd(f'{args.bundle}/clouds/{i:06d}.pcd') @ quat_to_rot(kf_quat[i]).T
                     + kf_pose[i] for i in sel])
    sub = voxelize(sub.astype(np.float64), args.voxel)
    full = load_pcd(f'{args.bundle}/map.pcd').astype(np.float64)
    print(f'子图 {sub.shape[0]} 点 / 全图 {full.shape[0]} 点')

    yaw = np.radians(args.pose[3])
    rot = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    origin = np.array(args.pose[:3]) + rot @ np.array([0.0, 0.0, 0.512])
    scan = synth_scan(full, origin)
    rr = np.linalg.norm(scan - origin, axis=1)
    print(f'合成扫描 {scan.shape[0]} 点，半径中位数 {np.median(rr):.1f} m')

    for name, target in (('12 关键帧子图', sub), ('整张 map.pcd', full)):
        dist, _ = cKDTree(target).query(scan, k=1, distance_upper_bound=2.0)
        ratios = '  '.join(f'thr={t}:{np.mean(dist < t):.3f}' for t in (0.3, 0.45, args.gate))
        print(f'  {name:<14s} {ratios}')
    print('\n对比：若子图与全图的支持率都低 → 扫描与地图整体错位；'
          '若子图远低于全图 → 子图太稀（加大 --kf / --radius）。')


if __name__ == '__main__':
    main()
