"""分析每个零件 STL，自动生成碰撞胶囊参数。

每个运动零件由两个关节点定义（杆的两端）。策略：
1. 读 STL 顶点（世界坐标，因为 transform≈identity）
2. 计算零件的"杆方向"（两端关节点连线）和"粗细"（点到杆轴的最大距离）
3. 输出胶囊参数：在 body 局部坐标里，从关节A到关节B的胶囊，半径=粗细

输出可以直接粘进 leg.xml。
"""
import struct
import numpy as np
import os

os.chdir('/home/mg/AIMAO/F3DMaojoco')

# 每个零件：[stl文件, 关节A世界坐标(X,Z), 关节B世界坐标(X,Z), body名, body原点世界(X,Z)]
# 关节坐标来自 component_positions.json
PARTS = [
    # [stl, JA(x,z), JB(x,z), body名, body原点(x,z)]
    # 膝盖链
    ('膝盖动力发生器_膝盖动力发生器.stl', (-10.9, 14.4), (-21.29, 8.4), 'knee_driver', (-10.9, 14.4)),
    ('膝盖传动1_膝盖传动1.stl',          (-21.29, 8.4), (-12.12, -7), 'knee_link1', (-21.29, 8.4)),
    ('膝盖转动_膝盖转动.stl',            (-12.12, -7), (12.12, 7), 'knee_rotor', (-12.12, -7)),
    ('膝盖传动2_膝盖传动2.stl',          (12.12, 7), (37.12, -36.3), 'knee_link2', (12.12, 7)),
    # 腿链
    ('大腿主动力发生器_大腿主动力发生器.stl', (0, 0), (25, -43.3), 'thigh_rigid', (0, 0)),
    ('小腿_小腿.stl',                    (25, -43.3), (37.12, -36.3), 'shin', (25, -43.3)),
]


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


def analyze_part(stl, JA, JB, body_origin, trim_mm=4.0):
    """分析零件，返回胶囊参数（body 局部坐标）。

    胶囊方向 = JA→JB，半径 = 顶点到杆轴的最大距离（XZ平面）。
    两端各裁掉 trim_mm（避开关节销孔区域，防止和相邻/闭环体误碰）。
    """
    verts = load_stl(f'exports/export1/stl_files/{stl}')
    # 杆轴方向（XZ 2D）
    axis = np.array([JB[0] - JA[0], JB[1] - JA[1]])
    axis_len = np.linalg.norm(axis)
    axis_dir = axis / axis_len
    # 每个顶点到杆轴的垂直距离（XZ 平面）
    dists = []
    for v in verts:
        p = np.array([v[0], v[2]])  # XZ
        ap = p - np.array(JA)
        proj = np.dot(ap, axis_dir)
        if proj < 0 or proj > axis_len:
            continue
        perp = ap - proj * axis_dir
        dists.append(np.linalg.norm(perp))
    radius = max(2.0, np.percentile(dists, 75)) if dists else 3.0
    # 两端裁剪：JA 沿轴方向 +trim，JB 沿轴方向 -trim
    trim = min(trim_mm, axis_len / 3)  # 不超过杆长 1/3
    JA_t = (JA[0] + axis_dir[0] * trim, JA[1] + axis_dir[1] * trim)
    JB_t = (JB[0] - axis_dir[0] * trim, JB[1] - axis_dir[1] * trim)
    # 转 body 局部坐标（减去 body 原点）
    JA_local = (JA_t[0] - body_origin[0], JA_t[1] - body_origin[1])
    JB_local = (JB_t[0] - body_origin[0], JB_t[1] - body_origin[1])
    return JA_local, JB_local, radius, axis_len


print(f"{'零件':<30} {'杆长(mm)':<10} {'胶囊半径(mm)':<12} {'JA局部':<18} {'JB局部':<18}")
print('-' * 100)
capsules = []
for stl, JA, JB, body, origin in PARTS:
    JA_l, JB_l, radius, length = analyze_part(stl, JA, JB, origin)
    print(f"{body:<30} {length:<10.1f} {radius:<12.2f} ({JA_l[0]:+.2f},0,{JA_l[1]:+.2f})   ({JB_l[0]:+.2f},0,{JB_l[1]:+.2f})")
    capsules.append((body, JA_l, JB_l, radius, length))

# 生成 XML 片段
print('\n=== leg.xml 碰撞胶囊片段（贴在每个 body 的 mesh geom 后）===')
print('<!-- 放在 default 外或 body 内，group=3 用于碰撞，不渲染 -->')
last_body = None
for body, JA_l, JB_l, radius, length in capsules:
    if body != last_body:
        print(f'\n      <!-- {body} 的碰撞胶囊 -->')
        last_body = body
    print(f'      <geom type="capsule" group="3" contype="1" conaffinity="1" '
          f'fromto="{JA_l[0]:.4f} 0 {JA_l[1]:.4f} {JB_l[0]:.4f} 0 {JB_l[1]:.4f}" '
          f'size="{radius/1000:.4f}" rgba="1 0 0 0.3"/>')
