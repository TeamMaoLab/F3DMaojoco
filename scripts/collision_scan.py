"""碰撞检测：扫描 θ1×θ2 工作空间，找出哪些角度组合会发生零件自碰撞。

每个格点：
1. 牛顿 FK 求解被动关节（闭环闭合）
2. mj_forward 让 MuJoCo 更新 geom 世界坐标 + 跑碰撞检测
3. 读 d.ncon（接触点数）判断是否碰撞
4. 三态分类：不可达(黑) / 可达无碰撞(绿) / 可达有碰撞(红)

输出：碰撞边界图 + 哪些零件对在哪些角度碰撞。
"""
import sys
import os
import numpy as np
import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fk_newton import solve_fk_newton, joint_qadr

MODEL_PATH = "mujoco_leg/leg.xml"


def load():
    m = mujoco.MjModel.from_xml_path(MODEL_PATH)
    d = mujoco.MjData(m)
    return m, d


def has_collision(m, d):
    """检测当前位姿是否有零件碰撞。返回 (bool 碰撞, [(body1,body2,penetration)...])。"""
    mujoco.mj_forward(m, d)
    n = d.ncon
    if n == 0:
        return False, []
    contacts = []
    for i in range(n):
        con = d.contact[i]
        # dist < 0 表示穿透
        if con.dist < -1e-5:  # 容差 0.01mm
            b1 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[con.geom1])
            b2 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[con.geom2])
            contacts.append((b1, b2, con.dist * 1000))  # mm
    return len(contacts) > 0, contacts


def scan(m, d, range_deg=60, step=5):
    """扫描工作空间。每个格点从零初值独立求解（物理初始位姿）。
    连续追踪会跳到非物理分支，导致伪碰撞；独立求解保证每个解都从 home 出发。"""
    print(f"\n=== 碰撞扫描 ±{range_deg}° 步长 {step}°（每点独立从 home 求解）===")
    grid = {}  # (t1,t2) -> 'green'|'red'|'black'
    stats = {"green": 0, "red": 0, "black": 0}
    collision_pairs = {}  # (t1,t2) -> [pairs]

    for t2 in range(-range_deg, range_deg + 1, step):
        row = ""
        for t1 in range(-range_deg, range_deg + 1, step):
            # 从 home（零初值）独立求解，避免分支漂移
            x, err, it = solve_fk_newton(m, d, t1, t2, x0=np.zeros(4))
            if err > 1e-4:
                grid[(t1, t2)] = "black"
                stats["black"] += 1
                row += "·"
                continue
            col, contacts = has_collision(m, d)
            if col:
                grid[(t1, t2)] = "red"
                stats["red"] += 1
                row += "✗"
                collision_pairs[(t1, t2)] = set((c[0], c[1]) for c in contacts)
            else:
                grid[(t1, t2)] = "green"
                stats["green"] += 1
                row += "█"
        print(f"  θ2={t2:+3d}: {row}")

    total = sum(stats.values())
    print(f"\n汇总: 可达无碰撞={stats['green']} 可达有碰撞={stats['red']} 不可达={stats['black']} / 共{total}")
    print(f"碰撞率（可达区域中）: {stats['red']/(stats['green']+stats['red'])*100:.0f}%")

    # 哪些零件对在碰撞
    all_pairs = set()
    for pairs in collision_pairs.values():
        all_pairs.update(pairs)
    if all_pairs:
        print(f"\n碰撞零件对（共 {len(all_pairs)} 对）:")
        for p in sorted(all_pairs):
            count = sum(1 for ps in collision_pairs.values() if p in ps)
            print(f"  {p[0]} ↔ {p[1]}  (出现 {count} 个格点)")

    return grid, stats


def find_limits(m, d, range_deg=80, step=1):
    """沿 4 个主方向找碰撞限位边界（θ1 和 θ2 各正负方向）。
    每个角度从 home 独立求解，保证物理分支正确。"""
    print(f"\n=== 碰撞限位边界（沿主轴，步长 {step}°，每点独立从 home 求解）===")
    directions = [
        ("θ1 正向（θ2=0）", lambda v: (v, 0), range(0, range_deg + 1, step)),
        ("θ1 负向（θ2=0）", lambda v: (v, 0), range(0, -range_deg - 1, -step)),
        ("θ2 正向（θ1=0）", lambda v: (0, v), range(0, range_deg + 1, step)),
        ("θ2 负向（θ1=0）", lambda v: (0, v), range(0, -range_deg - 1, -step)),
    ]
    for name, set_angles, val_range in directions:
        limit = None
        for v in val_range:
            t1, t2 = set_angles(v)
            x, err, it = solve_fk_newton(m, d, t1, t2, x0=np.zeros(4))
            if err > 1e-4:
                print(f"  {name}: 不可达 @ {v:+d}°（闭环不闭合）")
                break
            col, contacts = has_collision(m, d)
            if col:
                limit = v
                pairs = set((c[0], c[1]) for c in contacts)
                print(f"  {name}: 碰撞限位 @ {v:+d}°  碰撞: {pairs}")
                break
        if limit is None:
            print(f"  {name}: ±{range_deg}° 内无碰撞")


def main():
    print(f"MuJoCo {mujoco.__version__}")
    m, d = load()
    print(f"模型: {m.nbody} bodies, {m.njnt} joints, {m.neq} equality, {m.ngeom} geoms")

    # 先验证原点无碰撞
    solve_fk_newton(m, d, 0, 0)
    col, contacts = has_collision(m, d)
    print(f"\n原点 (0,0) 碰撞检测: {col}  contacts={contacts}")

    # 限位边界
    find_limits(m, d, range_deg=80, step=1)

    # 工作空间扫描
    scan(m, d, range_deg=60, step=5)


if __name__ == "__main__":
    main()
