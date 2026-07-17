#!/usr/bin/env python3
"""四足 MuJoCo 模型生成器

从已验证的单腿运动学树 (leg_kinematic.json) + 四足装配世界变换 (parts_world.json)
参数化生成完整四足 MuJoCo 模型。复用单腿 leg.xml 的 equality 闭环 + actuator 配置。

产物:
  --mode kin  → 固定 base + 零重力 + equality 闭环（对齐 quad_leg_viewer.html 验证几何）
  --mode dyn  → 浮动 base (freejoint) + 重力 + 地面 + 足端接触 + sensor（RL 训练用）

镜像规则（沿用 quad_leg_viewer.html / docs/quad_mirror_kinematics.md）:
  - 镜像腿 (FL/RL) 腿根 body 加绕 X 转 180° (quat = 0 1 0 0)
  - 镜像腿 joint_anchor_mm 的 Z 取负
  - 镜像腿用镜像 STL（顶点 Z 已被 Fusion 反射）

用法:
  python3 scripts/gen_quad_mujoco.py --mode kin
  python3 scripts/gen_quad_mujoco.py --mode dyn
"""
import argparse
import json
import os
from pathlib import Path

# ============================================================
# 常量：4 腿配置（与 quad_leg_viewer.html LEGS 完全一致）
# ============================================================
# full_path 子串：viewer findLegParts 用的 legOcc
LEGS = [
    dict(key='FR', root_mm=[0, -47, 0],   mirror=False, occ='舵机组 v8:1'),
    dict(key='RR', root_mm=[115, -47, 0], mirror=False, occ='舵机组 v8:2'),
    dict(key='FL', root_mm=[0, 47, 0],    mirror=True,  occ='舵机组 v8(镜像) (1):1'),
    dict(key='RL', root_mm=[115, 47, 0],  mirror=True,  occ='舵机组 v8(镜像) (1):2'),
]

# 单腿 body 树的中文零件名映射（沿用 quad_leg_viewer.html cnMap + thighParts）
CN_MAP = {
    'knee_driver':  '膝盖动力发生器',
    'knee_link1':   '膝盖传动1',
    'knee_rotor':   '膝盖转动',
    'knee_link2':   '膝盖传动2',
    'shin':         '小腿',
}
THIGH_PARTS = ['大腿主动力发生器', '小腿保持架', '髋关节保持架', '大腿盖板']

# 机身零件（固定，非镜像版本）
CHASSIS_COMPONENTS = ['零部件5', '假电池 v1', '假电路板 v1']

# 闭环约束（沿用 leg.xml 的 anchor，单位 m，相对 body1 局部系）
# loop_J2: knee_rotor 上的 J2' anchor ↔ thigh_rigid 上的 J2 点
# loop_J5: knee_link2 上的 J5 anchor ↔ shin 上的 J5 点
# 注意：这两个 anchor 是"相对 body1 局部坐标系"的，对镜像腿需要把 Z 取负
LOOP_J2_ANCHOR_MM = [12.12, 0, 7.0]     # 在 knee_rotor 局部
LOOP_J5_ANCHOR_MM = [25.0, 0, -43.3]    # 在 knee_link2 局部 (= shin 原点相对 thigh_rigid)

# collider（沿用 leg.xml 的 capsule，单位 m）
# 这些是 thigh_rigid 的 4 个碰撞胶囊 + shin 的碰撞胶囊 + 膝盖传动链的
COLLIDERS = {
    'knee_driver': [
        ('c_kd',  'cylinder', '-0.01039 0.0005 -0.006 -0.01039 -0.0065 -0.006', 0.003),
        ('c_kd1', 'cylinder', '0 -0.005 0 0 0.005 0', 0.005),
    ],
    'knee_rotor': [
        ('c_kr', 'cylinder', '0 0.0005 0 0 -0.0065 0', 0.003),
    ],
    'knee_link2': [
        ('c_kl2', 'cylinder', '0 0.0005 0 0 -0.0065 0', 0.003),
    ],
    'thigh_rigid': [
        ('c_th0', 'cylinder', '0.01204 -0.001 -0.00714 0.01204 -0.005 -0.00714', 0.002),
        ('c_th1', 'cylinder', '0 -0.001 0 0 -0.005 0', 0.00999),
        ('c_th2', 'cylinder', '0.02296 -0.001 -0.02943 0.02296 -0.005 -0.02943', 0.002),
        ('c_th3', 'cylinder', '0.0086 -0.001 -0.01105 0.0086 -0.005 -0.01105', 0.002),
    ],
    'shin': [
        ('c_shin', 'cylinder', '0.01212 0.0005 0.007 0.01212 -0.0065 0.007', 0.003),
    ],
}
# 父子 collider 对（强制检测，覆盖 MuJoCo 默认排除）
THIGH_SHIN_PAIRS = ['c_th0', 'c_th1', 'c_th2', 'c_th3']

# ============================================================
# 真实质量分配（kg，实测数据 2026-07-17）
# ============================================================
# 实测：3D打印件 ~90g，舵机 14g×8=112g，电路板+电池 40g，总重 ~242g
# 拆分：
#   机身打印件 ~30g（主骨架）+ 电路板电池 40g = base 70g
#   每腿打印件 15g（(90-30)/4），拆到 6 个活动 body：
#     thigh_rigid(含4零件，最重) 7g, shin 3g, knee_driver 2g,
#     knee_link1/link2/rotor 各 1g
#   舵机 14g/个
MASS_KG = {
    'base_body': 0.070,       # 机身（打印件30g + 电池电路板40g）
    'chassis_part': 0.005,    # 电池/电路板子 body（已算入 base，这里给小值避免重复）
    'hip': 0.0005,            # 腿根虚拟 body（几乎无质量）
    'thigh_rigid': 0.007,     # 大腿刚体（4零件，最重）
    'shin': 0.003,            # 小腿
    'knee_driver': 0.002,     # 膝盖动力发生器
    'knee_link1': 0.001,      # 膝盖传动1
    'knee_rotor': 0.001,      # 膝盖转动
    'knee_link2': 0.001,      # 膝盖传动2
    'servo': 0.014,           # 舵机壳（实测 14g）
}

# 站立姿态舵机角（度，从 viewer presetStandup 提取，作为 dyn 版 qpos 初值）
# viewer 里站立是 t1=0, t2=0 的附近，但落地需要腿下压。这里先给一个能让腿伸直的角
STANDUP_T1_DEG = 0.0   # 膝盖舵机
STANDUP_T2_DEG = 0.0   # 大腿舵机


# ============================================================
# 工具函数
# ============================================================
def mm_to_m(v):
    """[x,y,z] mm → 'x y z' m（保留6位小数）"""
    return f'{v[0]*0.001:.6g} {v[1]*0.001:.6g} {v[2]*0.001:.6g}'


def read_stl_aabb_world(stl_path, world_t_mm, world_rot_3x3):
    """读 STL 二进制文件，把顶点（component 局部 mm）变换到世界坐标，返回 AABB。

    返回 (center_mm, size_mm) —— 世界坐标系的轴对齐包围盒中心和尺寸。
    用于计算机身碰撞 box 的位置和大小。
    """
    from struct import unpack
    import numpy as np
    R = np.array(world_rot_3x3)
    t = np.array(world_t_mm)
    xs, ys, zs = [], [], []
    with open(stl_path, 'rb') as f:
        f.read(80)  # header
        n_tris = unpack('<I', f.read(4))[0]
        for _ in range(n_tris):
            f.read(12)  # normal
            for _ in range(3):
                vx, vy, vz = unpack('<fff', f.read(12))
                # component 局部 → 世界
                wx, wy, wz = R @ np.array([vx, vy, vz]) + t
                xs.append(wx); ys.append(wy); zs.append(wz)
            f.read(2)  # attr
    lo = np.array([min(xs), min(ys), min(zs)])
    hi = np.array([max(xs), max(ys), max(zs)])
    center = (lo + hi) / 2
    size = hi - lo
    return center, size


def strip_mirror_suffix(component_name):
    """去掉镜像/版本后缀，返回基础中文名。
    '膝盖传动1(镜像) (2)' → '膝盖传动1'
    'servo_mg90s v2' → 'servo_mg90s v2'（无后缀）
    """
    import re
    base = re.sub(r'\(镜像\).*$', '', component_name)
    base = re.sub(r'\(2\).*$', '', base)
    base = re.sub(r'\(3\).*$', '', base)
    return base.strip()


def find_leg_parts(parts, leg):
    """从 parts_world.json 找属于指定腿的所有零件（沿用 viewer findLegParts 规则）"""
    return [p for p in parts if leg['occ'] in p['full_path']]


def find_part_by_cn(leg_parts, cn):
    """在一条腿的零件里按中文名（去后缀）精确匹配"""
    for p in leg_parts:
        if strip_mirror_suffix(p['component']) == cn:
            return p
    return None


def mirror_anchor(anchor_mm, mirror):
    """镜像腿把 anchor 的 Z 取负（复用 viewer 规则）"""
    if mirror:
        return [anchor_mm[0], anchor_mm[1], -anchor_mm[2]]
    return list(anchor_mm)


# ============================================================
# XML 生成（用字符串拼接，避免引入 lxml 依赖）
# ============================================================
class XmlBuilder:
    """简易 XML 缩进拼接器"""

    def __init__(self, indent='  '):
        self.lines = []
        self.depth = 0
        self.indent_unit = indent

    def line(self, text):
        self.lines.append(self.indent_unit * self.depth + text)

    def open(self, tag, attrs=''):
        if attrs:
            self.line(f'<{tag} {attrs}>')
        else:
            self.line(f'<{tag}>')
        self.depth += 1

    def close(self, tag):
        self.depth -= 1
        self.line(f'</{tag}>')

    def leaf(self, tag, attrs):
        self.line(f'<{tag} {attrs}/>')

    def comment(self, text):
        self.line(f'<!-- {text} -->')

    def blank(self):
        self.lines.append('')

    def render(self):
        return '\n'.join(self.lines) + '\n'


# ============================================================
# asset 生成：mesh asset（原版 + 镜像版）
# ============================================================
def collect_mesh_assets(parts):
    """收集所有需要的 mesh asset。
    返回 {mesh_name: stl_filename}。
    原版腿 (FR/RR) 共用一套 mesh（名字不加前缀），镜像腿 (FL/RL) 共用一套镜像 mesh（加 _mir 后缀）。
    机身 mesh 单独命名。
    """
    assets = {}
    # 机身 mesh
    for p in parts:
        cn_base = strip_mirror_suffix(p['component'])
        if cn_base in CHASSIS_COMPONENTS and '镜像' not in p['component']:
            assets[cn_base] = os.path.basename(p['stl_file'])
    # 单腿零件的原版 mesh（从 FR 腿取，occ='舵机组 v8:1'）
    fr_parts = find_leg_parts(parts, LEGS[0])
    for body_name, cn in CN_MAP.items():
        p = find_part_by_cn(fr_parts, cn)
        if p:
            assets[body_name] = os.path.basename(p['stl_file'])
    for cn in THIGH_PARTS:
        p = find_part_by_cn(fr_parts, cn)
        if p:
            # thigh 的 4 个 mesh 用中文名（避免与 body 名冲突）
            assets[f'thigh_{cn}'] = os.path.basename(p['stl_file'])
    # 镜像腿 mesh（从 FL 腿取，加 _mir 后缀）
    fl_parts = find_leg_parts(parts, LEGS[2])
    for body_name, cn in CN_MAP.items():
        p = find_part_by_cn(fl_parts, cn)
        if p:
            assets[f'{body_name}_mir'] = os.path.basename(p['stl_file'])
    for cn in THIGH_PARTS:
        p = find_part_by_cn(fl_parts, cn)
        if p:
            assets[f'thigh_{cn}_mir'] = os.path.basename(p['stl_file'])

    # 舵机 mesh：8 个独立 asset（每条腿 2 个舵机，STL 各不同）
    # mesh 名 = {leg}_servo_{N}，N 是 occurrence 后缀（2 或 3）
    for leg in LEGS:
        leg_parts = find_leg_parts(parts, leg)
        for p in leg_parts:
            if 'servo' in p['component'].lower():
                n = p['occurrence'].split(':')[-1]
                assets[f'{leg["key"]}_servo_{n}'] = os.path.basename(p['stl_file'])

    return assets


def render_assets(xb, assets, stl_dir_rel):
    """渲染 <asset> 块"""
    xb.open('asset')
    # 按 mesh 名排序输出（稳定）
    for name in sorted(assets.keys()):
        stl_file = assets[name]
        path = f'{stl_dir_rel}/{stl_file}'
        xb.leaf('mesh', f'name="{name}" file="{path}" scale="0.001 0.001 0.001"')
    xb.close('asset')


# ============================================================
# 单腿 body 树生成
# ============================================================
def render_leg_body_tree(xb, leg, kinematic, assets, mode):
    """渲染一条腿的 body 树（挂在腿根 body 下）。

    腿根 body（hip）: pos = leg.root_mm (m), mirror 加 quat=0 1 0 0
    腿根下挂：膝盖传动链 + 腿链（两条 parent=null 的子树）
    舵机壳：固定在腿根，不跟关节动
    """
    mirror = leg['mirror']
    leg_key = leg['key']
    suffix = '_mir' if mirror else ''

    # ----- 构建带 anchor 的 body 列表（镜像腿 Z 取负）-----
    bodies = []
    for b in kinematic['bodies']:
        anchor = mirror_anchor(b['joint_anchor_mm'], mirror)
        bodies.append({
            'name': b['name'],
            'parent': b['parent'],
            'joint': b['joint'],
            'anchor': anchor,
            'pos_mm': b['pos_mm'],
        })

    # 计算 body 相对父 anchor 的偏移（沿用 viewer buildLeg 算法）
    # parent=None 时偏移 = anchor（相对腿根）
    # 否则偏移 = (本anchor - 父anchor)
    body_by_name = {b['name']: b for b in bodies}
    for b in bodies:
        if b['parent']:
            pa = body_by_name[b['parent']]['anchor']
            b['rel_offset_mm'] = [b['anchor'][i] - pa[i] for i in range(3)]
        else:
            b['rel_offset_mm'] = list(b['anchor'])

    # ----- 找该腿的零件（用于 mesh 引用）-----
    parts = kinematic['_parts']  # 已注入
    leg_parts = find_leg_parts(parts, leg)

    # ----- 腿根 body（hip）-----
    # 腿根是一个虚拟 body，不挂 mesh（mesh 直接挂到子 body），只承载 4 条腿链的根
    hip_attrs = f'name="{leg_key}_hip" pos="{mm_to_m(leg["root_mm"])}"'
    if mirror:
        hip_attrs += ' quat="0 1 0 0"'  # 绕 X 转 180° = diag(1,-1,-1)
    # 腿根 body 给一个小惯量（虚拟 body，几乎无质量）
    xb.open('body', hip_attrs)
    xb.leaf('inertial', f'pos="0 0 0" mass="{MASS_KG["hip"]}" diaginertia="1e-8 1e-8 1e-8"')

    # ----- 舵机壳在 render_worldbody 里统一处理（8 个固定件，世界坐标系）-----
    # 这里不再生成舵机 body，避免重复

    # ----- 膝盖传动链 + 腿链 -----
    # 两条 parent=None 的子树都挂在 hip body 下
    # 先按拓扑排序（父在前），然后渲染
    # MuJoCo 要求父 body 在子 body 之前出现，且子 body 物理上嵌套在父 body 标签内
    roots = [b for b in bodies if b['parent'] is None]
    for root_b in roots:
        render_body_subtree(xb, root_b, body_by_name, leg_key, suffix, leg_parts, mode)

    xb.close('body')  # hip


def render_body_subtree(xb, b, body_by_name, leg_key, suffix, leg_parts, mode):
    """递归渲染一个 body 及其子树"""
    body_name = f'{leg_key}_{b["name"]}'
    # body pos = 相对父 anchor 的偏移（已计算好 rel_offset_mm）
    pos_str = mm_to_m(b['rel_offset_mm'])
    xb.open('body', f'name="{body_name}" pos="{pos_str}"')

    # 惯量（真实质量，见 MASS_KG；惯量用质量×特征长度²估算）
    mass = MASS_KG.get(b['name'], 0.001)
    # diaginertia 粗估：mass × (10mm)² 量级（细长杆件）
    diag = mass * 1e-4
    xb.leaf('inertial', f'pos="0 0 0" mass="{mass}" diaginertia="{diag:.2e} {diag:.2e} {diag:.2e}"')

    # 关节（继承 default 的 damping/armature，不重复写）
    # 关节名沿用 leg.xml 的 j_旋转N 命名，加腿前缀防冲突
    joint_name_map = {'t1': 'j_旋转1', 't2': 'j_旋转2', 't3': 'j_旋转3',
                      't4': 'j_旋转4', 't6': 'j_旋转6', 't7': 'j_旋转7'}
    jname = f'{leg_key}_{joint_name_map[b["joint"]]}'
    # 主动关节（t1/t2）加舵机限位 ±90°，被动关节不限位（equality 约束决定）
    if b['joint'] in ('t1', 't2'):
        xb.leaf('joint', f'name="{jname}" pos="0 0 0" axis="0 1 0" range="-1.5708 1.5708"')
    else:
        xb.leaf('joint', f'name="{jname}" pos="0 0 0" axis="0 1 0"')

    # 可视化 mesh（STL 顶点相对腿根，body 原点在关节点，mesh pos = -anchor 拉回腿根系）
    # 但 body 原点是相对父的偏移，已经累乘了父变换。mesh 在 body 局部系下需要补偿"本 body 累积偏移"
    # viewer 做法：mesh.position = -b.anchor（本 body 关节点相对腿根）
    # 这里 anchor 是镜像后的值
    neg_anchor = [-b['anchor'][0], -b['anchor'][1], -b['anchor'][2]]
    mesh_pos_str = mm_to_m(neg_anchor)

    if b['name'] == 'thigh_rigid':
        # 大腿刚体：4 个零件
        thigh_rgba = {'大腿主动力发生器': '0.9 0.2 0.6 1'}
        for cn in THIGH_PARTS:
            mesh_name = f'thigh_{cn}{suffix}'
            rgba = thigh_rgba.get(cn, '0.6 0.6 0.6 1')
            xb.leaf('geom', f'rgba="{rgba}" mesh="{mesh_name}" pos="{mesh_pos_str}" '
                            f'contype="0" conaffinity="0" group="2"')
    elif b['name'] in CN_MAP:
        mesh_name = f'{b["name"]}{suffix}'
        # 各 body 用不同颜色（沿用 leg.xml）
        rgba_map = {
            'knee_driver': '0.9 0.5 0.2 1', 'knee_link1': '0.2 0.9 0.4 1',
            'knee_rotor': '0.2 0.4 0.9 1', 'knee_link2': '0.9 0.9 0.2 1',
            'shin': '0.2 0.7 0.9 1',
        }
        rgba = rgba_map.get(b['name'], '0.6 0.6 0.7 1')
        xb.leaf('geom', f'rgba="{rgba}" mesh="{mesh_name}" pos="{mesh_pos_str}" '
                        f'contype="0" conaffinity="0" group="2"')

    # collider（沿用 leg.xml 的 capsule，仅 kin 模式用于 LUT 验证的碰撞检测）
    # dyn 模式不生成腿内部 collider（避免初始自相交 + 它们在四足关节范围内从不触发）
    if mode == 'kin' and b['name'] in COLLIDERS:
        for cname, ctype, fromto, size in COLLIDERS[b['name']]:
            xb.leaf('geom', f'name="{leg_key}_{cname}" class="collider" '
                            f'type="{ctype}" fromto="{fromto}" size="{size}"')

    # dyn 模式：thigh_rigid 加地面碰撞 capsule（和 shin_ground 对称的做法）
    # 大腿本体从 thigh 原点(J2) 延伸到 J3 方向 [25, 0, -43.3]
    # 镜像腿 Z 取负（hip 的 quat=0 1 0 0 翻转）
    if mode == 'dyn' and b['name'] == 'thigh_rigid':
        th_z = 43.3 if (suffix == '_mir') else -43.3
        xb.leaf('geom', f'name="{leg_key}_thigh_ground" type="capsule" '
                        f'fromto="0 0 0 0.025 0 {th_z*0.001:.4g}" size="0.006" '
                        f'rgba="0.9 0.2 0.6 0.3" contype="1" conaffinity="1" group="3" '
                        f'friction="0.8 0.02 0.01"')

    # J5 site（knee_link2 和 shin 上有，用于闭环约束可视化）
    if b['name'] == 'knee_link2':
        anchor = b['anchor']
        # J5 site 在 knee_link2 局部 = LOOP_J5_ANCHOR_MM 相对 knee_link2 原点的偏移
        # knee_link2 原点 = J4 = [12.12, 0, 7]（原版），J5 = [25, 0, -43.3]
        # 但 leg.xml 的 site pos="0.025 0 -0.0433" 是相对 knee_link2 body 原点
        # 镜像腿同样需要 Z 取负
        site_z = 0.0433 if (suffix == '_mir') else -0.0433
        xb.leaf('site', f'name="{leg_key}_s_J5_link2" pos="0.025 0 {site_z:.4f}" '
                        f'size="0.0015" rgba="1 0 0 1"')
    elif b['name'] == 'shin':
        # shin 上 J5 site = LOOP_J2_ANCHOR_MM 相对 shin 原点（shin 原点 = J3）
        site_z = -0.007 if (suffix == '_mir') else 0.007
        xb.leaf('site', f'name="{leg_key}_s_J5_shin" pos="0.01212 0 {site_z:.4f}" '
                        f'size="0.0015" rgba="1 0 0 1"')
    elif b['name'] == 'thigh_rigid':
        xb.leaf('site', f'name="{leg_key}_s_J2_thigh" pos="0 0 0" '
                        f'size="0.0015" rgba="0 1 0 1"')

    # dyn 模式：shin 末端加足端接触 geom（sphere）+ 小腿地面碰撞 capsule
    # 足端真实位置：站立(0,0)时足端相对腿根 = [-7.22, 0, -79.10]（来自 foot_lut）
    # shin body 原点（J3）相对腿根 = [25, 0, -43.3]
    # 所以足端相对 shin 原点 = [-32.2, 0, -35.8]（原版）；镜像腿 Z 取负
    if mode == 'dyn' and b['name'] == 'shin':
        foot_local = [-32.2, 0, -35.8] if not (suffix == '_mir') else [-32.2, 0, 35.8]
        # 足端 sphere（group 1，碰地面/机身）
        # 足端 sphere（碰地面/机身）
        xb.leaf('geom', f'name="{leg_key}_foot" type="sphere" '
                        f'pos="{foot_local[0]*0.001:.4g} 0 {foot_local[2]*0.001:.4g}" size="0.004" '
                        f'rgba="0 1 0 1" contype="1" conaffinity="1" group="3" friction="0.8 0.02 0.01"')
        xb.leaf('site', f'name="{leg_key}_foot_site" '
                        f'pos="{foot_local[0]*0.001:.4g} 0 {foot_local[2]*0.001:.4g}" '
                        f'size="0.003" rgba="0 1 0 1"')
        # 小腿地面碰撞 capsule（碰地面，倒地时小腿蹭地不穿地）
        fz = foot_local[2] * 0.001
        xb.leaf('geom', f'name="{leg_key}_shin_ground" type="capsule" '
                        f'fromto="0 0 0 {foot_local[0]*0.001:.4g} 0 {fz:.4g}" size="0.003" '
                        f'rgba="0.2 0.7 0.9 0.3" contype="1" conaffinity="1" group="3" '
                        f'friction="0.8 0.02 0.01"')

    # 递归子 body
    children = [bb for bb in body_by_name.values() if bb['parent'] == b['name']]
    for child in children:
        render_body_subtree(xb, child, body_by_name, leg_key, suffix, leg_parts, mode)

    xb.close('body')


# ============================================================
# equality / actuator / contact / sensor 生成
# ============================================================
def render_equality(xb, mode):
    """4 腿 × 2 闭环 = 8 条 connect 约束"""
    # dyn 模式用更软的 solref（timeconst 放大 10x），减少闭环预应力震荡
    if mode == 'dyn':
        solref = '0.01 1'   # 软：timeconst=10ms
        solimp = '0.99 0.999 0.001'
    else:
        solref = '0.001 1'  # 硬（沿用 leg.xml，纯 FK 用）
        solimp = '0.9999 0.99999 0.00001'
    xb.comment(f'闭环约束：4 腿 × 2 条 = 8 条 connect（solref={solref} {mode}模式）')
    xb.open('equality')
    for leg in LEGS:
        lk = leg['key']
        mirror = leg['mirror']
        # anchor 在 body1 局部系。镜像腿 body 被 quat=0 1 0 0 翻转，anchor 的 Z 要取负
        j2_z = -7.0 if mirror else 7.0
        xb.leaf('connect', f'name="{lk}_loop_J2" body1="{lk}_knee_rotor" '
                           f'body2="{lk}_thigh_rigid" anchor="0.01212 0 {j2_z*0.001:.6g}" '
                           f'solref="{solref}" solimp="{solimp}"')
        j5_z = 43.3 if mirror else -43.3
        xb.leaf('connect', f'name="{lk}_loop_J5" body1="{lk}_knee_link2" '
                           f'body2="{lk}_shin" anchor="0.025 0 {j5_z*0.001:.6g}" '
                           f'solref="{solref}" solimp="{solimp}"')
    xb.close('equality')


def render_contact(xb, mode):
    """强制父子 body 之间的碰撞检测（仅 kin 模式有 collider pair）"""
    if mode == 'dyn':
        # dyn 模式无腿内部 collider，足端/小腿/机身↔地面 自动检测（contype=1）
        return
    xb.comment('强制父子 body 碰撞检测（thigh↔shin 防过屈，覆盖默认排除）')
    xb.open('contact')
    for leg in LEGS:
        lk = leg['key']
        for cn in THIGH_SHIN_PAIRS:
            xb.leaf('pair', f'geom1="{lk}_{cn}" geom2="{lk}_c_shin"')
    xb.close('contact')


def render_actuator(xb, mode):
    """8 个 position servo（4 腿 × 2 舵机）"""
    # PD 增益：kin 模式沿用 leg.xml 的强 PD（kp=800，纯 FK 用）；
    # dyn 模式降低到 kp=80（1/10），避免轻机器人（~0.6kg）被舵机响应弹飞
    if mode == 'dyn':
        kp, kv, fmax = 80, 5, 5
        xb.comment(f'8 舵机 position actuator（dyn: kp={kp} kv={kv} forcerange±{fmax}，'
                   f'适配轻机器人避免弹飞）')
    else:
        kp, kv, fmax = 800, 20, 30
        xb.comment(f'8 舵机 position actuator（kin: kp={kp} kv={kv}，沿用 leg.xml）')
    xb.open('default')
    xb.leaf('position', f'kp="{kp}" kv="{kv}"')
    xb.close('default')
    xb.open('actuator')
    for leg in LEGS:
        lk = leg['key']
        xb.leaf('position', f'name="{lk}_servo_knee" joint="{lk}_j_旋转1" '
                            f'kp="{kp}" kv="{kv}" forcerange="-{fmax} {fmax}"')
        xb.leaf('position', f'name="{lk}_servo_thigh" joint="{lk}_j_旋转2" '
                            f'kp="{kp}" kv="{kv}" forcerange="-{fmax} {fmax}"')
    xb.close('actuator')


def render_sensors(xb):
    """dyn 模式传感器：IMU + 关节位置 + 足端接触力"""
    xb.comment('传感器：IMU + 8 关节位置 + 4 足端接触力')
    xb.open('sensor')
    # IMU（base body 的线加速度 + 角速度）
    xb.leaf('framepos',     f'name="base_pos" objtype="body" objname="base"')
    xb.leaf('framequat',    f'name="base_quat" objtype="body" objname="base"')
    xb.leaf('framelinvel',  f'name="base_linvel" objtype="body" objname="base"')
    xb.leaf('frameangvel',  f'name="base_angvel" objtype="body" objname="base"')
    xb.leaf('framelinacc',  f'name="base_linacc" objtype="body" objname="base"')
    xb.leaf('frameangacc',  f'name="base_angacc" objtype="body" objname="base"')
    # 8 关节位置
    for leg in LEGS:
        lk = leg['key']
        for jn in ['j_旋转1', 'j_旋转2']:
            xb.leaf('jointpos', f'name="{lk}_{jn}_pos" joint="{lk}_{jn}"')
            xb.leaf('jointvel', f'name="{lk}_{jn}_vel" joint="{lk}_{jn}"')
    # 4 足端接触力
    for leg in LEGS:
        lk = leg['key']
        xb.leaf('touch', f'name="{lk}_foot_touch" site="{lk}_foot_site"')
    xb.close('sensor')


# ============================================================
# worldbody 生成
# ============================================================
def render_worldbody(xb, parts, kinematic, mode, parts_path):
    """渲染 worldbody：地面 + 机身 base + 4 腿"""
    xb.open('worldbody')
    # 光源
    xb.leaf('light', 'pos="0 -0.5 0.3" dir="0 0.95 -0.3" diffuse="0.8 0.8 0.8"')
    xb.leaf('light', 'pos="0 0.5 0.3" dir="0 -0.95 -0.3" diffuse="0.4 0.4 0.4"')

    if mode == 'dyn':
        # 地面（plane，足够大）—— 必须显式设 contype/conaffinity，否则被 default mesh 的 0 覆盖
        # solref 调软，避免初始穿透弹射（timeconst=20ms）
        xb.leaf('geom', 'name="floor" type="plane" pos="0 0 0" size="3 3 0.1" '
                        'rgba="0.5 0.5 0.5 1" friction="0.8 0.02 0.01" '
                        'contype="1" conaffinity="1" solref="0.02 1"')

    # ---- 机身 base body ----
    # base 原点 = 零部件5 世界原点 [0,0,0]，单位阵
    base_attrs = 'name="base" pos="0 0 0"'
    if mode == 'dyn':
        # 浮动 base：加 freejoint（6 自由度）
        # base 初始 z：足端相对腿根 z=-79mm，腿根 z=0，所以足端 z=-79mm
        # 足端 sphere 半径 4mm，要悬空需 base z > 79+4 = 83mm。取 90mm 留余量
        base_attrs = 'name="base" pos="0 0 0.090"'
    xb.open('body', base_attrs)
    if mode == 'dyn':
        xb.leaf('freejoint', 'name="root_joint"')
    # base 惯量（实测：机身打印件30g + 电池电路板40g = 70g）
    xb.leaf('inertial', f'pos="0 0 0" mass="{MASS_KG["base_body"]}" diaginertia="6e-5 6e-5 6e-5"')
    # 机身 mesh + 碰撞 box（dyn 模式才需要碰撞，防倒地穿地）
    chassis_aabb = None  # 累积所有机身零件的世界 AABB
    for p in parts:
        cn_base = strip_mirror_suffix(p['component'])
        if cn_base in CHASSIS_COMPONENTS and '镜像' not in p['component']:
            wt = p['world_t_mm']
            wr = p['world_rot']
            # 累积世界 AABB（用于碰撞 box）
            try:
                stl_path = os.path.join(os.path.dirname(parts_path), p['stl_file'])
                c, s = read_stl_aabb_world(stl_path, wt, wr)
                if chassis_aabb is None:
                    chassis_aabb = [c.copy() - s/2, c.copy() + s/2]  # [lo, hi]
                else:
                    chassis_aabb[0] = np.minimum(chassis_aabb[0], c - s/2)
                    chassis_aabb[1] = np.maximum(chassis_aabb[1], c + s/2)
            except Exception:
                pass
            if cn_base == '零部件5':
                xb.leaf('geom', f'rgba="0.6 0.6 0.7 1" mesh="{cn_base}" '
                                f'contype="0" conaffinity="0" group="2"')
            else:
                quat = rot3x3_to_quat(wr)
                xb.open('body', f'name="chassis_{cn_base}" pos="{mm_to_m(wt)}" quat="{quat}"')
                xb.leaf('inertial', f'pos="0 0 0" mass="{MASS_KG["chassis_part"]}" diaginertia="2e-6 2e-6 2e-6"')
                xb.leaf('geom', f'rgba="0.6 0.6 0.7 1" mesh="{cn_base}" '
                                f'contype="0" conaffinity="0" group="2"')
                xb.close('body')

    # 机身碰撞 box（dyn 模式）：用所有机身零件的合并 AABB 做一个 box
    # 这样机器人倒地时机身能碰地（防穿地），RL 才能学到"别摔"
    if mode == 'dyn' and chassis_aabb is not None:
        import numpy as np
        lo, hi = chassis_aabb
        center = (lo + hi) / 2
        size = hi - lo
        # box 挂在 base body 下，base 的 pos 已含抬高（[0,0,0.090]）
        # AABB 是 base 原点坐标系（parts_world 的世界系，base z=0）
        # 所以 box 相对 base 的位置 = center（不减 base_z，base pos 已处理）
        box_rel = [center[0], center[1], center[2]]
        # box 的 size 用 half-extent（MuJoCo box 的 size 是半边长）
        half = [max(s/2, 1.0) for s in size]  # 至少 1mm，避免退化
        xb.comment(f'机身碰撞 box（合并 AABB，base 原点系）：{lo.round(1)} ~ {hi.round(1)} mm')
        xb.leaf('geom', f'name="chassis_collider" type="box" '
                        f'pos="{mm_to_m(box_rel)}" '
                        f'size="{half[0]*0.001:.4g} {half[1]*0.001:.4g} {half[2]*0.001:.4g}" '
                        f'rgba="0.3 0.6 0.3 0.3" contype="1" conaffinity="1" group="3"')

    # ---- 4 条腿挂在 base 下 ----
    for leg in LEGS:
        xb.comment(f'===== {leg["key"]} 腿（{"镜像" if leg["mirror"] else "原版"}）=====')
        render_leg_body_tree(xb, leg, kinematic, None, mode)

    # ---- 8 个舵机壳（固定件，世界坐标系，直接挂 base 下）----
    # 沿用 quad_leg_viewer.html：舵机不随关节动，用 parts_world 的 world_t_mm + world_rot 摆放
    # 每条腿 2 个舵机：servo_mg90s v2:2（大腿舵机）+ servo_mg90s v2:3（膝盖舵机）
    # STL 不同、朝向不同，必须逐个用真实 world 变换
    xb.comment('===== 8 个舵机壳（固定件，世界坐标摆放，不随关节动）=====')
    for leg in LEGS:
        leg_parts = find_leg_parts(parts, leg)
        servos = [p for p in leg_parts if 'servo' in p['component'].lower()]
        for p in servos:
            n = p['occurrence'].split(':')[-1]
            mesh_name = f'{leg["key"]}_servo_{n}'
            wt = p['world_t_mm']
            wr = p['world_rot']
            quat = rot3x3_to_quat(wr)
            xb.open('body', f'name="{leg["key"]}_servo_{n}" pos="{mm_to_m(wt)}" quat="{quat}"')
            xb.leaf('inertial', f'pos="0 0 0" mass="{MASS_KG["servo"]}" diaginertia="4e-6 4e-6 4e-6"')
            xb.leaf('geom', f'rgba="0.3 0.3 0.3 1" mesh="{mesh_name}" '
                            f'contype="0" conaffinity="0" group="2"')
            xb.close('body')

    xb.close('body')  # base
    xb.close('worldbody')


def rot3x3_to_quat(R):
    """3x3 旋转矩阵 → MuJoCo quat (w,x,y,z) 字符串"""
    import math
    t = R[0][0] + R[1][1] + R[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2][1] - R[1][2]) / s
        y = (R[0][2] - R[2][0]) / s
        z = (R[1][0] - R[0][1]) / s
    elif R[0][0] > R[1][1] and R[0][0] > R[2][2]:
        s = math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2
        w = (R[2][1] - R[1][2]) / s
        x = 0.25 * s
        y = (R[0][1] + R[1][0]) / s
        z = (R[0][2] + R[2][0]) / s
    elif R[1][1] > R[2][2]:
        s = math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2
        w = (R[0][2] - R[2][0]) / s
        x = (R[0][1] + R[1][0]) / s
        y = 0.25 * s
        z = (R[1][2] + R[2][1]) / s
    else:
        s = math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2
        w = (R[1][0] - R[0][1]) / s
        x = (R[0][2] + R[2][0]) / s
        y = (R[1][2] + R[2][1]) / s
        z = 0.25 * s
    return f'{w:.6g} {x:.6g} {y:.6g} {z:.6g}'


# ============================================================
# 主生成函数
# ============================================================
def generate(parts_path, kinematic_path, out_path, mode, stl_dir_rel):
    with open(parts_path) as f:
        parts = json.load(f)['parts']
    with open(kinematic_path) as f:
        kinematic = json.load(f)
    kinematic['_parts'] = parts  # 注入，便于 render_leg_body_tree 访问

    assets = collect_mesh_assets(parts)

    xb = XmlBuilder()

    # 文件头注释
    xb.line('<?xml version="1.0" encoding="UTF-8"?>')
    xb.comment('四足舵机机器人 MuJoCo 模型（自动生成，请勿手改；改 scripts/gen_quad_mujoco.py）')
    xb.comment(f'模式: {mode}  | 生成自 leg_kinematic.json + parts_world.json')
    xb.comment('运动学：4 腿 × (2 主动舵机 + 4 被动关节 + 2 闭环 connect 约束)')
    xb.comment('镜像腿 (FL/RL): 腿根 quat=0 1 0 0 (绕X转180°) + anchor Z 取负 + 镜像 STL')
    xb.blank()

    xb.open('mujoco', 'model="servo_quadruped"')
    xb.blank()

    # compiler + option
    if mode == 'dyn':
        # dyn: implicitfast 积分器（闭环机构更稳定）+ 重力
        # 注：闭环 equality + freejoint 在纯 Euler 下会震荡，implicitfast 显著改善
        xb.leaf('compiler', 'angle="radian" coordinate="local" autolimits="true"')
        xb.leaf('option', 'timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast" iterations="50"')
    else:
        xb.leaf('compiler', 'angle="radian" coordinate="local" autolimits="true"')
        xb.leaf('option', 'timestep="0.002" gravity="0 0 0"')
    xb.blank()

    # default
    xb.open('default')
    # dyn 模式关节阻尼加大（闭环稳定性），kin 模式沿用 leg.xml 的 0.5
    joint_damping = '2.0' if mode == 'dyn' else '0.5'
    xb.leaf('joint', f'type="hinge" axis="0 1 0" damping="{joint_damping}" armature="0.0001"')
    xb.leaf('geom', 'type="mesh" rgba="0.6 0.6 0.7 1" contype="0" conaffinity="0" group="2"')
    xb.open('default', 'class="collider"')
    xb.leaf('geom', 'type="capsule" condim="3" contype="1" conaffinity="1" group="3" '
                    'rgba="1 0.4 0.1 1"')
    xb.close('default')
    xb.close('default')
    xb.blank()

    # asset
    render_assets(xb, assets, stl_dir_rel)
    xb.blank()

    # worldbody
    render_worldbody(xb, parts, kinematic, mode, parts_path)
    xb.blank()

    # equality
    render_equality(xb, mode)
    xb.blank()

    # contact
    render_contact(xb, mode)
    xb.blank()

    # actuator
    render_actuator(xb, mode)
    xb.blank()

    # sensor (仅 dyn)
    if mode == 'dyn':
        render_sensors(xb)
        xb.blank()

    xb.close('mujoco')

    # 写文件
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write(xb.render())
    print(f'✓ 生成 {out_path}  (mode={mode}, assets={len(assets)}, legs={len(LEGS)})')
    return out_path


# ============================================================
# CLI
# ============================================================
def main():
    ap = argparse.ArgumentParser(description='四足 MuJoCo 模型生成器')
    ap.add_argument('--mode', choices=['kin', 'dyn'], default='kin',
                    help='kin=运动学调试版(固定base零重力) / dyn=RL版(浮动base重力地面)')
    ap.add_argument('--kinematic', default='exports/quad_v4/leg_kinematic.json')
    ap.add_argument('--parts', default='exports/quad_v4/parts_world.json')
    ap.add_argument('--out', default=None,
                    help='输出 XML 路径（默认 mujoco_quad/quad_{mode}.xml）')
    ap.add_argument('--stl-dir-rel', default='../exports/quad_v4/stl_files',
                    help='XML 内 STL 相对路径（相对 XML 文件所在目录）')
    args = ap.parse_args()

    out = args.out or f'mujoco_quad/quad_{args.mode}.xml'
    generate(args.parts, args.kinematic, out, args.mode, args.stl_dir_rel)


if __name__ == '__main__':
    main()
