"""生成关节角查找表 workspace_lut.json + workspace_grid.json。

扫描 θ1×θ2（±60° 步长 2°），每个点用 MuJoCo solve_fk_newton 解出 6 个关节角
（都是相对父级的角度，度），连同碰撞/可达状态存成 JSON。
网页加载后拖滑块时查表 + 双线性插值。

# ============ 关键：避免缠绕解 / 错误分支 ============
闭环方程组有无穷多解（连杆可绕 Y 轴转任意整圈，sin/cos 周期性让 anchor 重合）。
牛顿迭代只收敛到"离初值最近"的解，初值决定一切。
旧版每个点都从 x0=0 冷启动 → 少数点落到缠绕解（t3=29917°）或错误分支（t3=+227°）。

本版策略（连续追踪 + 多初值 + 连续性选择）：
  1. BFS：从 (0,0) 种子（全 0 初值的正确解）出发，向四邻扩展
  2. 每个新点用 {已解邻居解, 0 初值} 多个候选各解一次
  3. 在所有收敛解里，选与已解邻居关节角差最小的（连续性优先，阻止缠绕）
  4. 缠绕解硬过滤：任意被动角 |.|>WRAP_TOL（默认 360°）判无效

输出：
  WebPreviewer/workspace_lut.json   （关节角表）
  WebPreviewer/workspace_grid.json  （0/1/2 可达性地图，供网页画图）
  mujoco_leg/workspace_grid.json    （同上，供 plot_workspace.py）
"""
import json
import os
from collections import deque
import numpy as np
import mujoco
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fk_newton import solve_fk_newton, joint_qadr

os.chdir('/home/mg/AIMAO/F3DMaojoco')
MODEL = 'mujoco_leg/leg.xml'
OUT_LUT = 'WebPreviewer/workspace_lut.json'
OUT_GRID_WEB = 'WebPreviewer/workspace_grid.json'
OUT_GRID_MJ = 'mujoco_leg/workspace_grid.json'

RANGE = 180
STEP = 2
WRAP_TOL_DEG = 360.0      # 单关节角超出此值的解视为缠绕，丢弃
NEIGHBOR_MAXJUMP_DEG = 45.0  # 与邻居关节角差超过此值视为跳变，降低连续性分（软惩罚，非硬过滤）

m = mujoco.MjModel.from_xml_path(MODEL)
d = mujoco.MjData(m)

angles = list(range(-RANGE, RANGE + 1, STEP))
N = len(angles)
idx = {a: i for i, a in enumerate(angles)}  # 角度值 → 序号


def jq(name):
    return np.degrees(d.qpos[joint_qadr(m, name)])


def has_collision():
    """当前 qpos 是否碰撞（任意 contact 穿透 > 0.01mm）。"""
    mujoco.mj_forward(m, d)
    return any(d.contact[k].dist < -1e-5 for k in range(d.ncon))


def passive_vector(x):
    """把 solve_fk_newton 返回的 x=[t6,t7,t4,t3]（弧度）转成 4 关节角度向量（度）。
    顺序与 JOINT_KEYS 一致：t3,t4,t6,t7。"""
    return np.array([
        np.degrees(x[3]),  # t3
        np.degrees(x[2]),  # t4
        np.degrees(x[0]),  # t6
        np.degrees(x[1]),  # t7
    ])


JOINT_KEYS = ['t3', 't4', 't6', 't7']  # 被动关节，连续性比较用这个顺序


def try_solve(t1, t2, x0):
    """尝试求解，返回 (cell_dict | None)。cell_dict 含关节角(度)+碰撞+可达。"""
    x, err, it = solve_fk_newton(m, d, t1, t2, x0=np.array(x0, dtype=float))
    if err > 1e-4:
        return None
    pv = passive_vector(x)
    # 硬过滤：缠绕解
    if np.any(np.abs(pv) > WRAP_TOL_DEG):
        return None
    col = has_collision()
    return {
        "reachable": True,
        "collision": col,
        "x": x.copy(),                 # 弧度初值，供邻居继承
        "passive_deg": pv,             # [t3,t4,t6,t7] 度
        "t1": round(float(t1), 3),
        "t2": round(float(t2), 3),
        "t3": round(float(pv[0]), 3),
        "t4": round(float(pv[1]), 3),
        "t6": round(float(pv[2]), 3),
        "t7": round(float(pv[3]), 3),
    }


def solve_best(t1, t2, seed_x0_list):
    """对 (t1,t2) 用多个候选初值各解一次，返回 (cell, score)。
    score 越小越连续（与种子初值的关节角差）。无解返回 (None, inf)。"""
    candidates = []
    for seed in seed_x0_list:
        c = try_solve(t1, t2, seed)
        if c is None:
            continue
        # 连续性分：与种子初值（已解邻居）的关节角差（度，L2）
        # seed 是弧度，转度比较
        seed_deg = passive_vector(seed)
        diff = np.linalg.norm(c["passive_deg"] - seed_deg)
        # 软惩罚：跳变大的解大幅加分（降低优先级），但保留作 fallback
        if diff > NEIGHBOR_MAXJUMP_DEG:
            diff += 1000.0  # 不直接淘汰，但排到最后
        candidates.append((c, diff))
    if not candidates:
        return None, float('inf')
    candidates.sort(key=lambda kv: kv[1])
    return candidates[0]


def neighbor_diff(c, nc):
    """两格点被动关节角向量(度)的 L2 距离。"""
    return np.linalg.norm(c['passive_deg'] - nc['passive_deg'])


def refine_continuity(solved, rounds=3):
    """全局连续性平滑：消除 BFS 后残留的相邻格点关节角跳变。

    大范围（如 ±180°）扫描时，不同区域可能锁到不同分支（四连杆多构型），
    边界处产生跳变。本函数以"距原点近者优先"为原则：
      1. 标记稳定点（与所有邻居跳变<阈值）
      2. 对不稳定点，收集稳定邻居解作候选初值，各解一次
      3. 选与稳定邻居最连续的解（直接比较 neighbor_diff）
      4. 多轮迭代，从中心向外收敛
    """
    for rnd in range(rounds):
        # 标记稳定点（与所有已解邻居跳变<阈值）
        stable_set = set()
        for (i, j), c in solved.items():
            if not c.get('reachable'):
                continue
            is_stable = True
            for di, dj in [(-1,0),(1,0),(0,-1),(0,1)]:
                ni, nj = i+di, j+dj
                if (ni, nj) not in solved or not solved[(ni,nj)].get('reachable'):
                    continue
                if neighbor_diff(c, solved[(ni,nj)]) > NEIGHBOR_MAXJUMP_DEG:
                    is_stable = False
                    break
            if is_stable:
                stable_set.add((i, j))

        if not stable_set:
            break

        # 不稳定点按距原点近→远排序
        unstable = []
        for (i, j), c in solved.items():
            if not c.get('reachable') or (i, j) in stable_set:
                continue
            has_jump = False
            for di, dj in [(-1,0),(1,0),(0,-1),(0,1)]:
                ni, nj = i+di, j+dj
                if (ni, nj) in solved and solved[(ni,nj)].get('reachable'):
                    if neighbor_diff(c, solved[(ni,nj)]) > NEIGHBOR_MAXJUMP_DEG:
                        has_jump = True
                        break
            if has_jump:
                d2 = angles[i]**2 + angles[j]**2
                unstable.append((d2, (i, j)))
        unstable.sort()  # 距原点近者优先

        if not unstable:
            break

        fixed = 0
        for _, (i, j) in unstable:
            seeds = []
            stable_neighbors = []
            for di, dj in [(-1,0),(1,0),(0,-1),(0,1),(1,1),(1,-1),(-1,1),(-1,-1)]:
                ni, nj = i+di, j+dj
                if (ni, nj) in stable_set:
                    seeds.append(solved[(ni, nj)]['x'])
                    stable_neighbors.append(solved[(ni, nj)])
            if not seeds:
                continue
            t1, t2 = angles[i], angles[j]
            best_cell = None
            best_cont = float('inf')
            for seed in seeds:
                c = try_solve(t1, t2, seed)
                if c is None:
                    continue
                # 连续性 = 与所有稳定邻居的最大关节角差
                cont = max(neighbor_diff(c, sn) for sn in stable_neighbors)
                if cont < best_cont:
                    best_cont = cont
                    best_cell = c
            if best_cell is not None and best_cont < NEIGHBOR_MAXJUMP_DEG:
                solved[(i, j)] = best_cell
                stable_set.add((i, j))
                fixed += 1
        if fixed == 0:
            break
        print(f"  平滑轮 {rnd+1}: 修复 {fixed} 个跳变点（稳定集 {len(stable_set)}）")


# ============ BFS 连续追踪 ============
print(f"模型: {m.nbody} bodies, {m.njnt} joints, {m.neq} equality")
print(f"扫描范围 ±{RANGE}° 步长 {STEP}° → {N}×{N} = {N*N} 格点")
print(f"缠绕阈值 ±{WRAP_TOL_DEG}°，邻居跳变阈值 {NEIGHBOR_MAXJUMP_DEG}°\n")

# solved[(i,j)] = cell_dict（i=θ1 序号, j=θ2 序号）
solved = {}
# 用 0 初值解种子点 (0,0)
seed_x0 = np.zeros(4)
seed = try_solve(0, 0, seed_x0)
if seed is None:
    raise RuntimeError("(0,0) 种子点不可解，无法启动 BFS！")
solved[(idx[0], idx[0])] = seed
print(f"种子 (0,0): t3={seed['t3']:+.1f} t4={seed['t4']:+.1f} t6={seed['t6']:+.1f} t7={seed['t7']:+.1f} ✓")

# BFS：从种子四邻扩展
queue = deque([(idx[0], idx[0])])
neighbors4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]
unsolved_neighbors = []  # 记录 BFS 后仍不可解的点（后续用 0 初值兜底重试）

while queue:
    ci, cj = queue.popleft()
    cur_x0 = solved[(ci, cj)]["x"]
    for di, dj in neighbors4:
        ni, nj = ci + di, cj + dj
        if not (0 <= ni < N and 0 <= nj < N):
            continue
        if (ni, nj) in solved:
            continue
        t1, t2 = angles[ni], angles[nj]
        # 候选初值：当前邻居解 + 零初值（兜底）
        seeds = [cur_x0, np.zeros(4)]
        cell, score = solve_best(t1, t2, seeds)
        if cell is not None:
            solved[(ni, nj)] = cell
            queue.append((ni, nj))
        else:
            unsolved_neighbors.append((ni, nj))

print(f"BFS 第一轮: 解出 {len(solved)}/{N*N}，未解 {len(unsolved_neighbors)}")

# ============ 兜底：BFS 漏掉的点用 0 初值重试 ============
# 这些点通常是孤岛（四周都不可解），但仍可能本身可解
for (ni, nj) in unsolved_neighbors:
    if (ni, nj) in solved:
        continue
    t1, t2 = angles[ni], angles[nj]
    cell, score = solve_best(t1, t2, [np.zeros(4)])
    if cell is not None:
        solved[(ni, nj)] = cell

print(f"兜底后: 解出 {len(solved)}/{N*N}")

# ============ 全局连续性平滑 ============
# BFS 是局部连续（邻居初值），大范围扫描时不同区域锁不同分支，
# 在距原点远的边缘产生跳变。这里从中心向外做多轮平滑，消除跳变。
print("连续性平滑中...")
refine_continuity(solved, rounds=3)
print()

# ============ 组装输出 ============
cells = {}
grid = np.zeros((N, N), dtype=int)  # [i_θ1, j_θ2]：0 黑/不可达 1 绿/可达 2 红/碰撞
wrap_filtered = 0
for (i, j), c in solved.items():
    key = f"{angles[i]},{angles[j]}"
    cell_out = {
        "reachable": True,
        "collision": c["collision"],
        "t1": c["t1"], "t2": c["t2"],
        "t3": c["t3"], "t4": c["t4"], "t6": c["t6"], "t7": c["t7"],
    }
    cells[key] = cell_out
    grid[i, j] = 2 if c["collision"] else 1

# 补全不可达点
for i in range(N):
    for j in range(N):
        key = f"{angles[i]},{angles[j]}"
        if key not in cells:
            cells[key] = {"reachable": False, "collision": False}

# ============ 连通性分析 ============
# 从原点 (0,0) 对绿色可达格点 flood fill，标记连通区域。
# 不连通的可达格点 → 3（黄色孤岛）：数学上可达但拖不到（被碰撞/不可达区隔开）。
# grid 值：0 黑(不可达) 1 绿(可达·连通) 2 红(碰撞) 3 黄(可达·孤岛)
i0 = idx[0]  # 原点 θ1=0 序号
connected = np.zeros((N, N), dtype=bool)
if grid[i0, i0] == 1:
    from collections import deque as _deque
    fq = _deque([(i0, i0)])
    connected[i0, i0] = True
    while fq:
        ci, cj = fq.popleft()
        for di, dj in [(-1,0),(1,0),(0,-1),(0,1)]:
            ni, nj = ci+di, cj+dj
            if 0 <= ni < N and 0 <= nj < N and not connected[ni, nj] and grid[ni, nj] == 1:
                connected[ni, nj] = True
                fq.append((ni, nj))
isolated_count = int(((grid == 1) & (~connected)).sum())
grid[(grid == 1) & (~connected)] = 3  # 孤岛 → 黄
print(f"连通性: 孤岛可达区 {isolated_count} 格点（标黄），连通绿色 {(grid==1).sum()} 格点")

# 回填 connected 到 cells（孤岛 connected=false，用于网页禁止拖入）
for i in range(N):
    for j in range(N):
        key = f"{angles[i]},{angles[j]}"
        if key in cells and cells[key].get("reachable"):
            cells[key]["connected"] = bool(connected[i, j])

out_lut = {
    "_meta": {
        "description": "关节角查找表（MuJoCo solve_fk_newton + BFS 连续追踪）。key='θ1,θ2'（度），值为相对父级的关节角（度）。",
        "method": "BFS continuous tracking from (0,0) seed + multi-initial-value + continuity selection. Avoids winding/wrong-branch solutions.",
        "range": RANGE,
        "step": STEP,
        "wrap_tol_deg": WRAP_TOL_DEG,
        "joints": {
            "t1": "膝盖动力发生器（相对世界）",
            "t2": "大腿刚体（相对世界）",
            "t3": "小腿（相对大腿）",
            "t4": "膝盖传动2（相对膝盖转动）",
            "t6": "膝盖传动1（相对膝盖动力发生器）",
            "t7": "膝盖转动（相对膝盖传动1）",
        }
    },
    "angles": angles,
    "cells": cells,
}

out_grid = {
    "angles": angles,
    "range": RANGE,
    "step": STEP,
    "grid": grid.tolist(),
}

with open(OUT_LUT, 'w', encoding='utf-8') as f:
    json.dump(out_lut, f, ensure_ascii=False)
with open(OUT_GRID_WEB, 'w', encoding='utf-8') as f:
    json.dump(out_grid, f, ensure_ascii=False)
with open(OUT_GRID_MJ, 'w', encoding='utf-8') as f:
    json.dump(out_grid, f, ensure_ascii=False)

# ============ 统计 + 自检 ============
reachable = sum(1 for c in cells.values() if c.get("reachable"))
collision = sum(1 for c in cells.values() if c.get("collision"))
total = len(cells)

# 自检：是否有残留缠绕解（被动角超阈值）
residual_wrap = 0
max_passive = 0.0
for c in cells.values():
    if not c.get("reachable"):
        continue
    for k in JOINT_KEYS:
        v = abs(c[k])
        max_passive = max(max_passive, v)
        if v > WRAP_TOL_DEG:
            residual_wrap += 1

# 自检：相邻格点关节角跳变（连续性）
jump_count = 0
jump_worst = 0.0
for i in range(N):
    for j in range(N):
        key = f"{angles[i]},{angles[j]}"
        c = cells[key]
        if not c.get("reachable"):
            continue
        for di, dj in [(1, 0), (0, 1)]:
            ni, nj = i + di, j + dj
            if not (0 <= ni < N and 0 <= nj < N):
                continue
            nkey = f"{angles[ni]},{angles[nj]}"
            nc = cells[nkey]
            if not nc.get("reachable"):
                continue
            diff = max(abs(c[k] - nc[k]) for k in JOINT_KEYS)
            if diff > NEIGHBOR_MAXJUMP_DEG:
                jump_count += 1
                jump_worst = max(jump_worst, diff)

print(f"=== 输出 ===")
print(f"  {OUT_LUT}")
print(f"  {OUT_GRID_WEB}")
print(f"  {OUT_GRID_MJ}")
print(f"\n=== 统计 ===")
print(f"  总格点: {total}")
print(f"  可达: {reachable} ({reachable/total*100:.1f}%)")
print(f"  碰撞: {collision} ({collision/total*100:.1f}%)")
print(f"  不可达: {total - reachable} ({(total-reachable)/total*100:.1f}%)")
print(f"\n=== 自检 ===")
print(f"  残留缠绕解（被动角>{WRAP_TOL_DEG}°）: {residual_wrap}  {'✓' if residual_wrap == 0 else '✗ 仍有缠绕！'}")
print(f"  被动角最大值: {max_passive:+.1f}°")
print(f"  相邻跳变（>{NEIGHBOR_MAXJUMP_DEG}°）: {jump_count} 处，最大 {jump_worst:.1f}°")
