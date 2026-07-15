"""生成碰撞配置 collision_config.json。

输出两类碰撞体：
1. 大腿碰撞体：从 COL_GROUP.stl 聚类提取（用户在 Fusion 里画的圆柱）
2. 关节碰撞体：每个旋转点中心一个直径6mm圆柱（自动生成）

策略：
- 大腿圆柱：聚类 COL_GROUP.stl，每个连通块拟合圆柱（PCA 主轴 + 到轴距离）
- 关节圆柱：7 个旋转点，每个画一个 r=3mm L=6mm 的短圆柱（沿 Y 轴，代表销轴）
- Y 偏移：统一取 -3（零件厚度中点，避免每个零件 Y 不一致）

输出 JSON 格式（给 WebPreviewer + MuJoCo 转换器共用）：
{
  "thigh_cylinders": [
    {"name": "thigh_0", "radius": 10.0, "from": [x,y,z], "to": [x,y,z], "body": "thigh_rigid"},
    ...
  ],
  "joint_cylinders": [
    {"name": "joint_J1", "radius": 3.0, "from": [...], "to": [...], "body": "<关节所在body>"},
    ...
  ]
}
"""
import struct
import json
import numpy as np
import os
from scipy.spatial import cKDTree

os.chdir('/home/mg/AIMAO/F3DMaojoco')
EXPORT_DIR = 'exports/e3'
COL_STL = f'{EXPORT_DIR}/stl_files/COL_GROUP_COL_GROUP.stl'
POS_JSON = f'{EXPORT_DIR}/component_positions.json'
OUTPUT = 'WebPreviewer/collision_config.json'

# 关节世界坐标（XZ 平面，来自 component_positions.json）
# [name, (x,z), 所属body名]
JOINTS = [
    ('J1', (-10.9, 14.4), 'knee_driver'),      # 膝盖舵机轴
    ('J2', (0, 0), 'thigh_rigid'),              # 大腿舵机轴（也在 world/机架）
    ('J3', (25, -43.3), 'thigh_rigid'),         # 大腿-小腿
    ('J4', (12.12, 7), 'knee_rotor'),           # 膝盖转动-传动2
    ('J5', (37.12, -36.3), 'knee_link2'),       # 传动2-小腿
    ('J6', (-21.29, 8.4), 'knee_driver'),       # 动力发生器-传动1
    ('J7', (-12.12, -7), 'knee_link1'),         # 传动1-转动
]

# 统一 Y 偏移（零件厚度中点）
Y_MID = -3.0


def load_stl(path):
    with open(path, 'rb') as f:
        f.read(80)
        n = struct.unpack('<I', f.read(4))[0]
        data = f.read(n * 50)
    verts = []
    for i in range(n):
        rec = data[i*50:(i+1)*50]
        for j in range(3):
            verts.append(struct.unpack('<fff', rec[12+j*12:12+j*12+12]))
    return np.array(verts)


def cluster_points(v, threshold=1.5):
    """连通性聚类。"""
    tree = cKDTree(v)
    visited = np.zeros(len(v), dtype=bool)
    clusters = []
    for i in range(len(v)):
        if visited[i]:
            continue
        queue = [i]; visited[i] = True; cluster = []
        while queue:
            idx = queue.pop(); cluster.append(idx)
            nbrs = tree.query_ball_point(v[idx], threshold)
            for nb in nbrs:
                if not visited[nb]:
                    visited[nb] = True; queue.append(nb)
        clusters.append(np.array(cluster))
    return clusters


def fit_cylinder(vv):
    """PCA 拟合圆柱：主轴=最大方差方向，半径=到轴最大距离。"""
    center = vv.mean(axis=0)
    cov = np.cov((vv - center).T)
    eigval, eigvec = np.linalg.eigh(cov)
    axis = eigvec[:, -1]  # 最大特征值方向 = 圆柱轴
    proj = (vv - center) @ axis
    pmin, pmax = proj.min(), proj.max()
    length = pmax - pmin
    perp = (vv - center) - np.outer(proj, axis)
    radius = np.linalg.norm(perp, axis=1).max()
    end1 = center + axis * pmin
    end2 = center + axis * pmax
    return {
        'radius': float(radius),
        'length': float(length),
        'axis': axis.tolist(),
        'from': end1.tolist(),
        'to': end2.tolist(),
        'center': center.tolist(),
        'nverts': len(vv),
    }


def extract_thigh_cylinders():
    """从 COL_GROUP.stl 提取圆柱。

    一个物理圆柱的 STL 三角化可能把 Y 厚度方向拆成多个连通块
    （Y=-1 顶面 + Y=-5 底面 + Y=-3 中间），所以要先聚类再合并：
    半径相近 + 轴方向相近 + 端点投影到轴上重叠 → 合并为一个圆柱。
    """
    v = load_stl(COL_STL)
    clusters = cluster_points(v, threshold=1.5)
    raw = []
    for cl in clusters:
        if len(cl) < 30:
            continue
        cyl = fit_cylinder(v[cl])
        cyl['nverts'] = len(cl)
        raw.append(cyl)

    # 合并：把所有顶点重新按"轴方向 + 半径"归类
    # 简单办法：对每个圆柱，看它的轴方向。如果几个圆柱轴平行 + 半径接近 + XZ 中心接近 → 合并
    merged = []
    used = [False] * len(raw)
    for i in range(len(raw)):
        if used[i]:
            continue
        group = [i]
        used[i] = True
        ci = raw[i]
        for j in range(i+1, len(raw)):
            if used[j]:
                continue
            cj = raw[j]
            # 轴方向夹角 < 15°
            cos_a = abs(np.dot(ci['axis'], cj['axis']))
            if cos_a < 0.966:
                continue
            # 半径差 < 20%
            if abs(ci['radius'] - cj['radius']) / max(ci['radius'], cj['radius']) > 0.2:
                continue
            # XZ 中心距离 < 5mm（允许 Y 不同）
            cxi, czj = np.array(ci['center']), np.array(cj['center'])
            cxi_xz = np.array([cxi[0], cxi[2]])
            czj_xz = np.array([czj[0], czj[2]])
            if np.linalg.norm(cxi_xz - czj_xz) > 8:
                continue
            group.append(j)
            used[j] = True
        # 合并 group 里所有圆柱的顶点，重新拟合
        all_verts = []
        for gi in group:
            # 找回原始顶点：重新聚类太麻烦，直接用合并后的几何平均
            pass
        # 取 group 第一个为代表，但 Y 用中点
        rep = dict(raw[group[0]])
        if len(group) > 1:
            ys = [raw[gi]['center'][1] for gi in group]
            rep_y = (min(ys) + max(ys)) / 2
            rep['from'][1] = rep_y
            rep['to'][1] = rep_y
            rep['center'][1] = rep_y
            rep['nverts'] = sum(raw[gi]['nverts'] for gi in group)
            rep['merged_from'] = len(group)
        merged.append(rep)
    merged.sort(key=lambda c: -c['nverts'])
    return merged


def make_joint_cylinders():
    """每个旋转点一个 r=3mm L=6mm 圆柱（沿 Y 轴）。"""
    result = []
    for name, (x, z), body in JOINTS:
        # 沿 Y 轴的短圆柱，长度6mm，中心在 Y_MID
        result.append({
            'name': f'joint_{name}',
            'radius': 3.0,
            'from': [x, Y_MID - 3.0, z],
            'to': [x, Y_MID + 3.0, z],
            'body': body,
            'axis_y': True,  # 标记沿 Y 轴
        })
    return result


def main():
    print(f'=== 提取大腿圆柱 ({COL_STL}) ===')
    thigh = extract_thigh_cylinders()
    print(f'检测到 {len(thigh)} 个候选圆柱:')
    for i, c in enumerate(thigh):
        print(f'  [{i}] r={c["radius"]:.1f}mm L={c["length"]:.1f}mm '
              f'from=({c["from"][0]:.1f},{c["from"][1]:.1f},{c["from"][2]:.1f}) '
              f'to=({c["to"][0]:.1f},{c["to"][1]:.1f},{c["to"][2]:.1f}) '
              f'({c["nverts"]}顶点)')

    # 给每个标 body=thigh_rigid（COL_GROUP 挂在大腿下）
    for c in thigh:
        c['name'] = f"thigh_{thigh.index(c)}"
        c['body'] = 'thigh_rigid'

    joints = make_joint_cylinders()
    print(f'\n=== 关节圆柱（7个，每个 r=3mm L=6mm 沿Y轴）===')
    for j in joints:
        print(f'  {j["name"]} @ ({j["from"][0]:.1f},{j["from"][2]:.1f}) body={j["body"]}')

    config = {
        '_meta': {
            'description': '碰撞体配置。大腿圆柱从 COL_GROUP.stl 自动提取（可在网页微调），关节圆柱自动生成。',
            'units': 'mm',
            'y_mid': Y_MID,
        },
        'thigh_cylinders': thigh,
        'joint_cylinders': joints,
    }
    with open(OUTPUT, 'w', encoding='utf-8') as f:
        json.dump(config, f, ensure_ascii=False, indent=2)
    print(f'\n已写入 {OUTPUT}')


if __name__ == '__main__':
    main()
