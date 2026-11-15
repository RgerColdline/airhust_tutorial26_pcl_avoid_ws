#!/usr/bin/env python3
"""生成 data/pillar_frame.pcd —— 模拟一帧机载激光雷达看到的"一排柱子"。

为什么要有这个东西：
    仿真的算例必须在 Gazebo 里跑，起一次要等半天。
    这个脚本用最朴素的方法模拟了一遍雷达：从原点往四面八方打射线，
    打到柱子 / 地面 / 远处的墙就把那个点记下来。
    所以出来的点云和真雷达一样，是"只能看到朝向自己那一面"的稀疏点。

输出：
    data/pillar_frame.pcd   点云（PointXYZI，机体系：x 前、y 左、z 上）
    data/pillar_gt.txt      真值（每根柱子的 x y radius），只用来对答案

布局：
    雷达在原点，飞机悬停在离地 1m，所以地面在 z = -1。
    前方 x ≈ 4m 处立着 4 根柱子，柱子之间有 3 个缝：
    只有一个缝宽到能飞过去，另外两个比机身窄。

用法：
    python3 data/make_pillar_frame.py            # 用默认种子
    python3 data/make_pillar_frame.py --seed 7   # 换一组柱子
"""

import argparse
import struct
from pathlib import Path

import numpy as np

# ---- 场景参数 -------------------------------------------------------------
SENSOR_Z = 0.0
GROUND_Z = -1.0
PILLAR_X = 4.0
PILLAR_Z0, PILLAR_Z1 = -1.0, 1.0
WALL_X = 12.0

# 雷达视场：水平 360°，垂直 -20° ~ +45°
FOV_V_MIN = np.deg2rad(-20.0)
FOV_V_MAX = np.deg2rad(45.0)

N_RAYS = 40000
RANGE_NOISE = 0.01  # 1cm 测距噪声

# ---- 柱子布局 -------------------------------------------------------------
# 和 spawn_pillars.py 保持一致：4 根柱子、3 个缝，只有一个缝能过。
PILLAR_COUNT = 4
PASSABLE_GAP = (1.9, 2.3)  # 中心距
BLOCKED_GAP = (0.6, 0.9)   # 中心距
PILLAR_RADIUS = (0.15, 0.25)


def make_layout(rng):
    """随机生成一排柱子的布局，返回 [(x, y, radius), ...]。"""
    n = PILLAR_COUNT
    pass_idx = int(rng.integers(0, n - 1))
    gaps = []
    for i in range(n - 1):
        if i == pass_idx:
            gaps.append(rng.uniform(*PASSABLE_GAP))
        else:
            gaps.append(rng.uniform(*BLOCKED_GAP))

    ys = [0.0]
    for gap in gaps:
        ys.append(ys[-1] + gap)
    shift = rng.uniform(-0.4, 0.4) - (ys[0] + ys[-1]) / 2.0
    ys = [y + shift for y in ys]

    pillars = []
    for i in range(n):
        pillars.append(
            (
                PILLAR_X + rng.uniform(-0.1, 0.1),
                ys[i],
                rng.uniform(*PILLAR_RADIUS),
            )
        )
    return pillars


def raycast(origin, dirs, pillars):
    """返回每条射线打到的东西：距离 t、命中类型。没打中记为 -1。"""
    ox, oy, oz = origin
    dx, dy, dz = dirs[:, 0], dirs[:, 1], dirs[:, 2]
    n = len(dx)

    best_t = np.full(n, np.inf)
    best_kind = np.full(n, -1, dtype=np.int32)  # 0=柱 1=地 2=墙

    # 圆柱：解 (ox+t dx - cx)^2 + (oy+t dy - cy)^2 = r^2
    for cx, cy, r in pillars:
        a = dx * dx + dy * dy
        b = 2.0 * ((ox - cx) * dx + (oy - cy) * dy)
        c = (ox - cx) ** 2 + (oy - cy) ** 2 - r * r
        disc = b * b - 4 * a * c
        ok = disc > 0
        sq = np.sqrt(np.where(ok, disc, 0.0))
        t = np.where(ok, (-b - sq) / (2 * a), np.inf)
        z_hit = oz + t * dz
        ok &= (t > 0) & (z_hit > PILLAR_Z0) & (z_hit < PILLAR_Z1)
        better = ok & (t < best_t)
        best_t[better] = t[better]
        best_kind[better] = 0

    # 地面
    down = dz < -1e-6
    t = np.where(down, (GROUND_Z - oz) / dz, np.inf)
    hit = down & (t > 0) & (np.abs(ox + t * dx) < 15) & (np.abs(oy + t * dy) < 15)
    better = hit & (t < best_t)
    best_t[better] = t[better]
    best_kind[better] = 1

    # 远处的墙（给点背景杂物）
    fwd = dx > 1e-6
    t = np.where(fwd, (WALL_X - ox) / dx, np.inf)
    hit = fwd & (t > 0) & (oz + t * dz > GROUND_Z)
    better = hit & (t < best_t)
    best_t[better] = t[better]
    best_kind[better] = 2

    return best_t, best_kind


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=4)
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    pillars = make_layout(rng)

    # 射线方向：Livox 是非重复扫描，这里用均匀随机凑一个近似
    az = rng.uniform(0.0, 2 * np.pi, N_RAYS)
    el = rng.uniform(FOV_V_MIN, FOV_V_MAX, N_RAYS)
    dirs = np.stack(
        [np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)], axis=1
    )

    t, kind = raycast((0.0, 0.0, SENSOR_Z), dirs, pillars)
    hit = np.isfinite(t) & (kind >= 0)
    hit &= rng.random(N_RAYS) > 0.02  # 2% 的点随机丢掉

    t = t[hit] + rng.normal(0.0, RANGE_NOISE, hit.sum())
    p = dirs[hit] * t[:, None]
    kind = kind[hit]

    intensity = np.select(
        [kind == 0, kind == 1, kind == 2], [120.0, 40.0, 70.0], default=0.0
    ) + rng.normal(0.0, 5.0, len(kind))

    n = len(p)
    out = Path(__file__).resolve().parent / "pillar_frame.pcd"
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    )
    blob = bytearray()
    for (x, y, z), i in zip(p, intensity):
        blob += struct.pack("<ffff", x, y, z, i)
    with open(out, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(blob)

    gt = Path(__file__).resolve().parent / "pillar_gt.txt"
    lines = ["# 真值：x y radius（机体系，雷达在原点）",
             "# 只给 offline_test 对答案用，你的代码不许读它"]
    for x, y, r in pillars:
        lines.append(f"{x:.6f} {y:.6f} {r:.6f}")
    gt.write_text("\n".join(lines) + "\n")

    print(f"seed={args.seed}")
    for i, (x, y, r) in enumerate(pillars):
        print(f"柱子 {i}: ({x:.2f}, {y:.2f}) r={r:.3f}")
    print(f"写了 {out}（{n} 点）和 {gt}")


if __name__ == "__main__":
    main()
