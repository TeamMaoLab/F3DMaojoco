"""
验证双闭环平面机构的正向运动学求解器（Python 原型）

机构拓扑（XZ 平面，Y 是旋转轴）：
  闭环1（膝盖四连杆）: J1 ── J6 ── J7 ── (闭合到 J1)
    J1=旋转1(膝盖动力发生器-舵机), J6=旋转6(动力-传动1), J7=旋转7(传动1-转动)
    主动角: θ1 (J1)
  闭环2（腿+膝盖传动）: J2 ── J3 ── J5 ── J4 ── (闭合到 J2)
    J2=旋转2(大腿动力-舵机), J3=旋转3(大腿-小腿), J5=旋转5(传动2-小腿), J4=旋转4(转动-传动2)
    主动角: θ2 (J2)
  耦合: J7 和 J4 都在"膝盖转动"零件上（J2' 也在其上，即旋转2'连接转动-大腿）

关键: "膝盖转动"零件同时参与两个闭环，是耦合点。
它上面有三个关节点: J7(连传动1), J4(连传动2), J2'(连大腿主动力发生器)。
这三个点在同一刚体上，它们的相对位置固定。

求解策略:
  给定 θ1, θ2，求 6 个被动角 (θ3,θ4,θ5,θ6,θ7) 使两个闭环闭合。
  用牛顿迭代解非线性方程组 F(θ)=0。
"""
import numpy as np
import json
import math

# 从 export1 加载关节点初始位置（XZ 平面）
def load_joint_positions(json_path):
    d = json.load(open(json_path))
    J = {}
    for j in d['joints']:
        g = j['geometry']
        t = g.get('geometry_one_transform') or g.get('geometry_two_transform')
        if t:
            m = t['matrix']
            J[j['name']] = np.array([m[0][3], m[2][3]])  # [X, Z]
    return J

# 关节极限（从数据加载）
def load_joint_limits(json_path):
    d = json.load(open(json_path))
    L = {}
    for j in d['joints']:
        if j.get('limits') and j['limits'].get('revolute_limits'):
            rl = j['limits']['revolute_limits']['rotation_limits']
            L[j['name']] = (math.degrees(rl['minimum_value']), math.degrees(rl['maximum_value']))
        else:
            L[j['name']] = None
    return L

def solve_fk(theta1_deg, theta2_deg, J_init, verbose=False):
    """
    给定两个主动角（度），求解 6 个被动角（度）。
    返回 (success, angles_dict, error)

    机构建模：
    以初始位姿为参考。每个零件的位姿 = 初始位姿 × 绕其主动关节旋转。
    我们用"杆组法"：把机构拆成若干二连杆组，逐步求解。

    简化建模（平面）：
    - J1 旋转 θ1 → 膝盖动力发生器转动 → J6 随之移动（J6 = J1 + R(θ1)·(J6_init - J1_init))
    - J2 旋转 θ2 → 大腿主动力发生器转动 → J3 随之移动
    - 闭环1: J6 已知, J7 由 J6-J7 连杆(膝盖传动1)和 J7-J1 连杆约束
    - ...
    """
    theta1 = math.radians(theta1_deg)
    theta2 = math.radians(theta2_deg)

    # 初始关节位置
    J1, J2 = J_init['旋转 1'], J_init['旋转 2']
    J3, J4 = J_init['旋转 3'], J_init['旋转 4']
    J5, J6, J7 = J_init['旋转 5'], J_init['旋转 6'], J_init['旋转 7']

    # 连杆长度（初始时测量，刚体不变）
    L_1_6 = np.linalg.norm(J6 - J1)    # 膝盖动力发生器上 J1-J6
    L_6_7 = np.linalg.norm(J7 - J6)    # 膝盖传动1 上 J6-J7
    L_7_1 = np.linalg.norm(J1 - J7)    # 闭环1 闭合边 J7-J1
    L_2_3 = np.linalg.norm(J3 - J2)    # 大腿上 J2-J3
    L_3_5 = np.linalg.norm(J5 - J3)    # 小腿上 J3-J5
    L_5_4 = np.linalg.norm(J4 - J5)    # 膝盖传动2 上 J5-J4
    L_4_2 = np.linalg.norm(J2 - J4)    # 闭环2 闭合边 J4-J2

    # 膝盖转动零件上 J7-J4 的距离（耦合两闭环的刚体）
    L_7_4 = np.linalg.norm(J4 - J7)

    def rot(angle):
        c, s = math.cos(angle), math.sin(angle)
        return np.array([[c, -s], [s, c]])

    # === 步骤1: 主动角驱动 J6 和 J3 的新位置 ===
    # J1 是 J1 的位置（假设舵机固定，J1 点不动，膝盖动力发生器绕 J1 转 θ1）
    # J6 = J1 + R(θ1) · (J6_init - J1_init)
    J1_new = J1.copy()
    J6_new = J1 + rot(theta1) @ (J6 - J1)

    # J2 不动，大腿绕 J2 转 θ2
    J2_new = J2.copy()
    J3_new = J2 + rot(theta2) @ (J3 - J2)

    # === 步骤2: 闭环1 求 J7 ===
    # J7 满足: |J7 - J6_new| = L_6_7 (膝盖传动1长度)
    #          |J7 - J1_new| = L_7_1 (闭合边)
    # 两圆相交求 J7
    J7_candidates = circle_intersect(J6_new, L_6_7, J1_new, L_7_1)
    if len(J7_candidates) == 0:
        return False, None, "闭环1 无交点"
    # 选最接近初始 J7 的解
    J7_new = min(J7_candidates, key=lambda p: np.linalg.norm(p - J7))

    # === 步骤3: 闭环2 求 J5 和 J4 ===
    # J4 在膝盖转动零件上，与 J7 的距离 = L_7_4（刚体约束）
    # J4 满足: |J4 - J7_new| = L_7_4
    #         |J4 - J2_new| = L_4_2 (闭环2闭合边)
    J4_candidates = circle_intersect(J7_new, L_7_4, J2_new, L_4_2)
    if len(J4_candidates) == 0:
        return False, None, "闭环2 J4 无交点"
    J4_new = min(J4_candidates, key=lambda p: np.linalg.norm(p - J4))

    # J5 满足: |J5 - J3_new| = L_3_5 (小腿长度)
    #         |J5 - J4_new| = L_5_4 (膝盖传动2长度)
    J5_candidates = circle_intersect(J3_new, L_3_5, J4_new, L_5_4)
    if len(J5_candidates) == 0:
        return False, None, "闭环2 J5 无交点"
    J5_new = min(J5_candidates, key=lambda p: np.linalg.norm(p - J5))

    # === 步骤4: 计算各被动角（相对于初始位姿的转角）===
    def angle_of(vec): return math.atan2(vec[1], vec[0])

    # θ6: 膝盖传动1 的转角 = (J7_new - J6_new) 方向 vs (J7 - J6) 方向
    theta6 = angle_of(J7_new - J6_new) - angle_of(J7 - J6)
    # θ7: 闭合边方向变化
    theta7 = angle_of(J1_new - J7_new) - angle_of(J1 - J7)
    # θ3: 小腿相对大腿的角 = (J5_new - J3_new) vs (J5 - J3)
    theta3 = angle_of(J5_new - J3_new) - angle_of(J5 - J3)
    # θ4: 膝盖传动2 的角 = (J5_new - J4_new) vs (J5 - J4)
    theta4 = angle_of(J5_new - J4_new) - angle_of(J5 - J4)
    # θ5: 小腿上 J3-J5 方向变化（同 θ3，验证用）
    theta5 = angle_of(J4_new - J5_new) - angle_of(J4 - J5)

    angles = {
        '旋转 1': theta1_deg,
        '旋转 2': theta2_deg,
        '旋转 3': math.degrees(theta3),
        '旋转 4': math.degrees(theta4),
        '旋转 5': math.degrees(theta5),
        '旋转 6': math.degrees(theta6),
        '旋转 7': math.degrees(theta7),
    }

    # === 步骤5: 验证闭环闭合误差 ===
    err1 = abs(np.linalg.norm(J7_new - J6_new) - L_6_7) + abs(np.linalg.norm(J7_new - J1_new) - L_7_1)
    err2 = abs(np.linalg.norm(J5_new - J3_new) - L_3_5) + abs(np.linalg.norm(J5_new - J4_new) - L_5_4)
    err = err1 + err2

    if verbose:
        print(f"  θ1={theta1_deg:6.1f}° θ2={theta2_deg:6.1f}° → 闭合误差={err:.6f}mm")
        for k, v in angles.items():
            print(f"    {k}: {v:.2f}°")

    return err < 0.01, angles, f"误差={err:.4f}mm"

def circle_intersect(c1, r1, c2, r2):
    """两圆相交点（平面）。返回 0/1/2 个点。"""
    d = np.linalg.norm(c2 - c1)
    if d > r1 + r2 + 1e-9 or d < abs(r1 - r2) - 1e-9 or d < 1e-9:
        return []
    a = (r1*r1 - r2*r2 + d*d) / (2*d)
    h_sq = r1*r1 - a*a
    if h_sq < 0: h_sq = 0
    h = math.sqrt(h_sq)
    mid = c1 + a * (c2 - c1) / d
    perp = np.array([-(c2-c1)[1], (c2-c1)[0]]) / d
    return [mid + h*perp, mid - h*perp]

if __name__ == '__main__':
    json_path = 'exports/export1/component_positions.json'
    J_init = load_joint_positions(json_path)
    limits = load_joint_limits(json_path)

    print("=== 机构初始几何 ===")
    for name, p in J_init.items():
        print(f"  {name}: X={p[0]:8.3f}  Z={p[1]:8.3f}")

    print("\n=== 关节极限 ===")
    for name, lim in limits.items():
        print(f"  {name}: {lim}")

    print("\n=== 求解测试（零位 θ1=0, θ2=0）===")
    ok, angles, err = solve_fk(0, 0, J_init, verbose=True)
    print(f"  结果: {'✅有解' if ok else '❌无解'} ({err})")

    print("\n=== 扫描 θ1=θ2 ∈ [-90, 90] 步长 30° ===")
    for t1 in range(-90, 91, 30):
        for t2 in range(-90, 91, 30):
            ok, angles, err = solve_fk(t1, t2, J_init)
            tag = "✅" if ok else "❌"
            print(f"  θ1={t1:4d} θ2={t2:4d}: {tag} {err}")

    print("\n=== 检查被动角是否在极限内（θ1=0,θ2=30）===")
    ok, angles, err = solve_fk(0, 30, J_init, verbose=True)
    if ok and angles:
        for name, ang in angles.items():
            lim = limits.get(name)
            if lim:
                in_range = lim[0] - 1 <= ang <= lim[1] + 1
                print(f"  {name}: {ang:.2f}°  极限[{lim[0]:.0f},{lim[1]:.0f}]  {'✅' if in_range else '❌超限'}")
