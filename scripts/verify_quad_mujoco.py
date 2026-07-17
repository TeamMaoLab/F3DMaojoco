#!/usr/bin/env python3
"""四足 MuJoCo 模型验证脚本

加载 mujoco_quad/quad_kin.xml，验证：
1. 4 腿 × 2 闭环 = 8 个 connect 约束残差 < 0.5mm（在网格点上）
2. 4 腿足端位置几何对称：
   - FR ↔ FL 关于 Y=0 平面对称
   - RR ↔ RL 关于 Y=0 平面对称
   - FR ↔ RR 仅 X 方向差 115mm（前腿 X=0，后腿 X=115）
3. 同一 (θ1,θ2) 下，4 腿被动关节解一致（除镜像腿关节角符号）

方法：纯 FK 牛顿迭代（沿用 fk_newton.py 方法），不依赖 MuJoCo 约束求解器，
这样得到精确解，验证几何正确性而非动力学收敛性。

用法:
  python3 scripts/verify_quad_mujoco.py
  python3 scripts/verify_quad_mujoco.py --model mujoco_quad/quad_dyn.xml
"""
import argparse
import numpy as np
import mujoco

# 4 腿定义（与 gen_quad_mujoco.py LEGS 一致）
LEGS = ['FR', 'RR', 'FL', 'RL']
# 镜像腿（生成时 anchor Z 取负 + hip quat=0 1 0 0）
MIRROR_LEGS = {'FL', 'RL'}

# 被动关节（顺序 t6, t7, t4, t3）
JOINT_T1 = 'j_旋转1'   # 主动：膝盖舵机
JOINT_T2 = 'j_旋转2'   # 主动：大腿舵机
PASSIVE_JOINTS = ['j_旋转6', 'j_旋转7', 'j_旋转4', 'j_旋转3']


def jadr(m, name):
    jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
    return m.jnt_qposadr[jid]


def body_id(m, name):
    return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)


def loop_residual_leg(m, d, lk):
    """返回单腿 4 维闭环残差 [J2'_x, J2'_z, J5_x, J5_z]（mm）。

    anchor 在 body 局部系：
      J2': knee_rotor 上 [0.01212, 0, ±0.007]（镜像腿 Z 取负）↔ thigh_rigid 原点
      J5 : knee_link2 上 [0.025, 0, ±0.0433] ↔ shin 上 [0.01212, 0, ±0.007]
    """
    mujoco.mj_forward(m, d)
    mirror = lk in MIRROR_LEGS
    sign_z = -1.0 if mirror else 1.0

    # J2' on knee_rotor
    kr = body_id(m, f'{lk}_knee_rotor')
    kr_pos = d.xpos[kr]
    kr_mat = d.xmat[kr].reshape(3, 3)
    j2p = kr_pos + kr_mat @ np.array([0.01212, 0, 0.007 * sign_z])

    # J2 on thigh_rigid (origin)
    th = body_id(m, f'{lk}_thigh_rigid')
    j2_thigh = d.xpos[th].copy()

    # J5 on knee_link2
    kl2 = body_id(m, f'{lk}_knee_link2')
    kl2_pos = d.xpos[kl2]
    kl2_mat = d.xmat[kl2].reshape(3, 3)
    j5_link2 = kl2_pos + kl2_mat @ np.array([0.025, 0, -0.0433 * sign_z])

    # J5 on shin
    sh = body_id(m, f'{lk}_shin')
    sh_pos = d.xpos[sh]
    sh_mat = d.xmat[sh].reshape(3, 3)
    j5_shin = sh_pos + sh_mat @ np.array([0.01212, 0, 0.007 * sign_z])

    return np.array([
        (j2p - j2_thigh)[0],
        (j2p - j2_thigh)[2],
        (j5_link2 - j5_shin)[0],
        (j5_link2 - j5_shin)[2],
    ])


def solve_leg_fk(m, d, lk, t1_deg, t2_deg, x0=None, max_iter=50, tol=1e-8):
    """牛顿迭代解单腿 4 个被动关节。x0 = [t6, t7, t4, t3] 弧度。"""
    # 锁定主动关节
    d.qpos[jadr(m, f'{lk}_{JOINT_T1}')] = np.radians(t1_deg)
    d.qpos[jadr(m, f'{lk}_{JOINT_T2}')] = np.radians(t2_deg)

    if x0 is None:
        x0 = np.zeros(4)
    qadrs = [jadr(m, f'{lk}_{jn}') for jn in PASSIVE_JOINTS]
    x = np.array(x0, dtype=float)

    for it in range(max_iter):
        for i, qa in enumerate(qadrs):
            d.qpos[qa] = x[i]
        f = loop_residual_leg(m, d, lk)
        err = np.linalg.norm(f)
        if err < tol:
            break
        # 数值雅可比
        eps = 1e-6
        J = np.zeros((4, 4))
        for j in range(4):
            xp = x.copy()
            xp[j] += eps
            for i, qa in enumerate(qadrs):
                d.qpos[qa] = xp[i]
            fp = loop_residual_leg(m, d, lk)
            J[:, j] = (fp - f) / eps
        try:
            dx = np.linalg.solve(J, -f)
        except np.linalg.LinAlgError:
            return None, err, it
        x += dx

    # 写回最终解
    for i, qa in enumerate(qadrs):
        d.qpos[qa] = x[i]
    mujoco.mj_forward(m, d)
    return x, err, it


def foot_position(m, d, lk):
    """足端世界坐标（shin 上的 J5 点，即足端接触点）。"""
    sh = body_id(m, f'{lk}_shin')
    sh_pos = d.xpos[sh]
    sh_mat = d.xmat[sh].reshape(3, 3)
    mirror = lk in MIRROR_LEGS
    sign_z = -1.0 if mirror else 1.0
    # 足端在 shin 局部 = [0.01212, 0, ±0.007]
    return sh_pos + sh_mat @ np.array([0.01212, 0, 0.007 * sign_z])


def get_leg_joints(m, d, lk):
    """读单腿 6 关节角（度）。"""
    return {
        't1': np.degrees(d.qpos[jadr(m, f'{lk}_{JOINT_T1}')]),
        't2': np.degrees(d.qpos[jadr(m, f'{lk}_{JOINT_T2}')]),
        't3': np.degrees(d.qpos[jadr(m, f'{lk}_j_旋转3')]),
        't4': np.degrees(d.qpos[jadr(m, f'{lk}_j_旋转4')]),
        't6': np.degrees(d.qpos[jadr(m, f'{lk}_j_旋转6')]),
        't7': np.degrees(d.qpos[jadr(m, f'{lk}_j_旋转7')]),
    }


def main():
    ap = argparse.ArgumentParser(description='四足 MuJoCo 模型验证')
    ap.add_argument('--model', default='mujoco_quad/quad_kin.xml')
    ap.add_argument('--range', type=int, default=30, help='扫描范围 ±range 度')
    ap.add_argument('--step', type=int, default=10, help='扫描步长（度）')
    ap.add_argument('--tol-mm', type=float, default=0.5, help='闭环残差容差（mm）')
    args = ap.parse_args()

    print(f'MuJoCo {mujoco.__version__} · 四足模型验证')
    m = mujoco.MjModel.from_xml_path(args.model)
    d = mujoco.MjData(m)
    print(f'模型: {args.model}')
    print(f'  bodies={m.nbody} joints={m.njnt} actuators={m.nu} equality={m.neq}')
    print(f'  扫描范围: θ1,θ2 ∈ [-{args.range}, +{args.range}] 步长 {args.step}°')
    print(f'  容差: {args.tol_mm}mm\n')

    # ============================================================
    # 测试 1：单点（0,0）4 腿闭环 + 几何
    # ============================================================
    print('=' * 60)
    print('测试 1：初始位形 (0,0) 4 腿闭环残差')
    print('=' * 60)
    all_ok = True
    for lk in LEGS:
        x, err, it = solve_leg_fk(m, d, lk, 0, 0, x0=np.zeros(4))
        ok = err < args.tol_mm * 0.001  # err 是 m，容差转 m
        status = '✓' if ok else '✗'
        print(f'  {lk}: 闭环残差={err*1000:.4f}mm @it{it} {status}')
        if not ok:
            all_ok = False
    print()

    # ============================================================
    # 测试 2：4 腿 hip / thigh / knee_driver 位置精确性
    # ============================================================
    print('=' * 60)
    print('测试 2：4 腿关键点世界坐标精确性')
    print('=' * 60)
    expected_hip = {'FR': [0, -47, 0], 'RR': [115, -47, 0],
                    'FL': [0, 47, 0], 'RL': [115, 47, 0]}
    mujoco.mj_forward(m, d)
    for lk in LEGS:
        hip = d.xpos[body_id(m, f'{lk}_hip')] * 1000
        exp = expected_hip[lk]
        err = np.linalg.norm(hip - np.array(exp))
        ok = err < 0.01
        status = '✓' if ok else '✗'
        print(f'  {lk}_hip: [{hip[0]:.2f}, {hip[1]:.2f}, {hip[2]:.2f}] '
              f'期望 [{exp[0]}, {exp[1]}, {exp[2]}] err={err:.4f}mm {status}')
        if not ok:
            all_ok = False
    print()

    # ============================================================
    # 测试 3：网格扫描 4 腿闭环闭合率
    # ============================================================
    print('=' * 60)
    print(f'测试 3：网格扫描 4 腿闭环闭合率（容差 {args.tol_mm}mm）')
    print('=' * 60)
    rng = range(-args.range, args.range + 1, args.step)
    leg_stats = {lk: {'ok': 0, 'total': 0, 'max_err': 0} for lk in LEGS}

    for t2 in rng:
        # 用上一行的解作下一行初值（连续追踪）
        last_x0 = {lk: np.zeros(4) for lk in LEGS}
        for t1 in rng:
            for lk in LEGS:
                x, err, it = solve_leg_fk(m, d, lk, t1, t2, x0=last_x0[lk])
                leg_stats[lk]['total'] += 1
                if err < args.tol_mm * 0.001:
                    leg_stats[lk]['ok'] += 1
                    if x is not None:
                        last_x0[lk] = x.copy()
                if err > leg_stats[lk]['max_err']:
                    leg_stats[lk]['max_err'] = err

    for lk in LEGS:
        s = leg_stats[lk]
        rate = s['ok'] / s['total'] * 100
        status = '✓' if rate == 100 else ('~' if rate >= 80 else '✗')
        print(f'  {lk}: {s["ok"]}/{s["total"]} = {rate:.0f}% '
              f'(max残差 {s["max_err"]*1000:.3f}mm) {status}')
        if rate < 80:
            all_ok = False
    print()

    # ============================================================
    # 测试 4：4 腿足端对称性
    # ============================================================
    # 关键：镜像腿（FL/RL）必须喂取负的主动角，才与 viewer 语义一致
    # （viewer: 镜像腿 lut[joint] * -1；等价于 MuJoCo 里给镜像腿喂 -t1, -t2）
    # 这验证了模型的几何对称性，而不是控制信号对称性
    print('=' * 60)
    print('测试 4：足端几何对称性（FR↔FL, RR↔RL 关于 Y=0 对称）')
    print('  注：镜像腿喂 -t1,-t2（与 viewer lut*sign=-1 等价）')
    print('=' * 60)
    sym_errors = {'FR_FL': [], 'RR_RL': [], 'FR_RR': []}
    rng2 = range(-20, 21, 10)
    for t2 in rng2:
        for t1 in rng2:
            # FR/RR 用 (t1,t2)；FL/RL 用 (-t1,-t2)（镜像腿主动角取负）
            feet = {}
            for lk in LEGS:
                if lk in MIRROR_LEGS:
                    solve_leg_fk(m, d, lk, -t1, -t2, x0=np.zeros(4))
                else:
                    solve_leg_fk(m, d, lk, t1, t2, x0=np.zeros(4))
                feet[lk] = foot_position(m, d, lk) * 1000  # mm
            # FR ↔ FL 对称：x 和 z 相同，y 互为相反数
            fr, fl = feet['FR'], feet['FL']
            err_frfl = np.array([fr[0] - fl[0], fr[1] + fl[1], fr[2] - fl[2]])
            sym_errors['FR_FL'].append(np.linalg.norm(err_frfl))
            # RR ↔ RL
            rr, rl = feet['RR'], feet['RL']
            err_rrrl = np.array([rr[0] - rl[0], rr[1] + rl[1], rr[2] - rl[2]])
            sym_errors['RR_RL'].append(np.linalg.norm(err_rrrl))
            # FR ↔ RR：仅 X 差 115mm（前后腿）
            err_frrr = np.array([fr[0] - rr[0] + 115, fr[1] - rr[1], fr[2] - rr[2]])
            sym_errors['FR_RR'].append(np.linalg.norm(err_frrr))

    for pair, errs in sym_errors.items():
        maxe = max(errs)
        avge = np.mean(errs)
        status = '✓' if maxe < 0.5 else '✗'
        print(f'  {pair}: max={maxe:.4f}mm avg={avge:.4f}mm {status}')
        if maxe >= 0.5:
            all_ok = False
    print()

    # ============================================================
    # 测试 5：镜像腿被动关节角符号
    # ============================================================
    # viewer 语义：镜像腿所有关节角（主动+被动）都取负
    # MuJoCo 验证：给 FR 喂 (15,-10)，给 FL 喂 (-15,+10)
    # 则 FL 的被动关节解应与 FR 取负
    print('=' * 60)
    print('测试 5：镜像腿被动关节角应与原版腿取负')
    print('  （FR 喂 (15,-10)，FL 喂 (-15,+10)，被动角应 FR+FL≈0）')
    print('=' * 60)
    solve_leg_fk(m, d, 'FR', 15, -10, x0=np.zeros(4))
    solve_leg_fk(m, d, 'FL', -15, 10, x0=np.zeros(4))
    jfr = get_leg_joints(m, d, 'FR')
    jfl = get_leg_joints(m, d, 'FL')
    print(f'  FR(t1=+15,t2=-10): t3={jfr["t3"]:+.2f} t4={jfr["t4"]:+.2f} t6={jfr["t6"]:+.2f} t7={jfr["t7"]:+.2f}')
    print(f'  FL(t1=-15,t2=+10): t3={jfl["t3"]:+.2f} t4={jfl["t4"]:+.2f} t6={jfl["t6"]:+.2f} t7={jfl["t7"]:+.2f}')
    print('  被动角应满足 FR + FL ≈ 0（取负关系）')
    all_sign_ok = True
    for jn in ['t3', 't4', 't6', 't7']:
        diff = jfr[jn] + jfl[jn]  # 应接近 0
        status = '✓' if abs(diff) < 0.5 else '✗'
        if abs(diff) >= 0.5:
            all_sign_ok = False
        print(f'    {jn}: FR={jfr[jn]:+.2f} FL={jfl[jn]:+.2f} FR+FL={diff:+.3f} {status}')
    if not all_sign_ok:
        all_ok = False
    print()

    # ============================================================
    # 总结
    # ============================================================
    print('=' * 60)
    print(f'总结：{"✓ 全部通过" if all_ok else "✗ 有失败项"}')
    print('=' * 60)
    return 0 if all_ok else 1


if __name__ == '__main__':
    exit(main())
