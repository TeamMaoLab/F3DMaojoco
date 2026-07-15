"""生成关节角查找表 workspace_lut.json。

扫描 θ1×θ2（±60° 步长 2°），每个点用 MuJoCo solve_fk_newton 解出 6 个关节角
（都是相对父级的角度，度），连同碰撞/可达状态存成 JSON。
网页加载后拖滑块时查表 + 双线性插值。

输出：WebPreviewer/workspace_lut.json
"""
import json
import os
import numpy as np
import mujoco
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fk_newton import solve_fk_newton, joint_qadr

os.chdir('/home/mg/AIMAO/F3DMaojoco')
MODEL = 'mujoco_leg/leg.xml'
OUTPUT = 'WebPreviewer/workspace_lut.json'

RANGE = 60
STEP = 2

m = mujoco.MjModel.from_xml_path(MODEL)
d = mujoco.MjData(m)

angles = list(range(-RANGE, RANGE + 1, STEP))
cells = {}

def jq(name):
    return np.degrees(d.qpos[joint_qadr(m, name)])

for t2 in angles:
    for t1 in angles:
        x, err, it = solve_fk_newton(m, d, t1, t2, x0=np.zeros(4))
        key = f"{t1},{t2}"
        if err > 1e-4:
            cells[key] = {"reachable": False, "collision": False}
            continue
        mujoco.mj_forward(m, d)
        col = any(d.contact[k].dist < -1e-5 for k in range(d.ncon))
        cells[key] = {
            "reachable": True,
            "collision": col,
            # 关节角（度），相对父级
            "t1": round(float(jq("j_旋转1")), 3),
            "t2": round(float(jq("j_旋转2")), 3),
            "t3": round(float(jq("j_旋转3")), 3),
            "t4": round(float(jq("j_旋转4")), 3),
            "t6": round(float(jq("j_旋转6")), 3),
            "t7": round(float(jq("j_旋转7")), 3),
        }

out = {
    "_meta": {
        "description": "关节角查找表（MuJoCo solve_fk_newton 求解）。key='θ1,θ2'（度），值为相对父级的关节角（度）。",
        "range": RANGE,
        "step": STEP,
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

with open(OUTPUT, 'w', encoding='utf-8') as f:
    json.dump(out, f, ensure_ascii=False)

reachable = sum(1 for c in cells.values() if c.get("reachable"))
collision = sum(1 for c in cells.values() if c.get("collision"))
total = len(cells)
print(f"生成 {OUTPUT}: {total} 格点")
print(f"  可达: {reachable} ({reachable/total*100:.0f}%)")
print(f"  碰撞: {collision} ({collision/total*100:.0f}%)")
print(f"  不可达: {total - reachable} ({(total-reachable)/total*100:.0f}%)")
