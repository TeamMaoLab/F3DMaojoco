"""验证 MuJoCo 单腿模型 + 双闭环约束。

用法：
    cd /home/mg/AIMAO/F3DMaojoco
    uv run python scripts/verify_mujoco_leg.py

做什么：
1. 加载 mujoco_leg/leg.xml，检查编译无错
2. 设置 θ1/θ2 目标，跑 mj_step 让 PD 收敛 + equality 约束闭环
3. 输出关键点坐标（J2', J5）+ 约束误差，肉眼判断闭环是否闭合
4. 扫描 θ1×θ2 工作空间，输出哪些点闭环能闭合（对照网页版）
"""
import sys
import numpy as np
import mujoco

MODEL_PATH = "mujoco_leg/leg.xml"

# 关节名 → qpos/adr 映射
JOINT_NAMES = ["j_旋转1", "j_旋转6", "j_旋转7", "j_旋转4", "j_旋转2", "j_旋转3"]


def load():
    m = mujoco.MjModel.from_xml_path(MODEL_PATH)
    d = mujoco.MjData(m)
    return m, d


def set_targets(m, d, theta1_deg, theta2_deg):
    """设置两个伺服目标角（度→弧度）。"""
    t1 = np.radians(theta1_deg)
    t2 = np.radians(theta2_deg)
    mujoco.mj_setConst(m, d)  # 确保 const 数据就绪
    # 找 actuator
    a1 = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "servo_knee")
    a2 = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "servo_thigh")
    d.ctrl[a1] = t1
    d.ctrl[a2] = t2


def settle(m, d, max_steps=30000):
    """跑 mj_step 直到 qvel 收敛（PD 平衡 + 约束闭合）。"""
    for i in range(max_steps):
        mujoco.mj_step(m, d)
        if i % 500 == 0 and i > 500:
            vel = np.linalg.norm(d.qvel)
            if vel < 1e-4:
                break


def site_world(d, name):
    sid = mujoco.mj_name2id(d.model, mujoco.mjtObj.mjOBJ_SITE, name)
    return d.site_xpos[sid].copy()


def body_world(d, name):
    bid = mujoco.mj_name2id(d.model, mujoco.mjtObj.mjOBJ_BODY, name)
    return d.xpos[bid].copy()


def joint_q(d, joint_name):
    jid = mujoco.mj_name2id(d.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    qadr = d.model.jnt_qposadr[jid]
    return d.qpos[qadr]


def report(m, d, theta1_deg, theta2_deg):
    """报告关键点 + 约束误差。"""
    # 关键点世界坐标
    j2_thigh = site_world(d, "s_J2_thigh")
    j5_shin = site_world(d, "s_J5_shin")
    j5_link2 = site_world(d, "s_J5_link2")
    # 膝盖转动 body 原点（=J7）
    knee_rotor_pos = body_world(d, "knee_rotor")
    # J2' 在膝盖转动 body 上的世界坐标 = body_xpos + body_quat · anchor
    # 更直接：闭环约束的 efc_force 里有约束残差，但 MuJoCo 不直接暴露 anchor 距离
    # 我们用"膝盖转动的 J2' 点"世界坐标手动算：
    #   J2' anchor in knee_rotor frame = (0.01212, 0, 0.007)
    kr_xpos = d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "knee_rotor")]
    kr_xmat = d.xmat[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "knee_rotor")].reshape(3, 3)
    j2prime_world = kr_xpos + kr_xmat @ np.array([0.01212, 0, 0.007])

    # 两个闭环的距离误差
    err_j2 = np.linalg.norm(j2prime_world - j2_thigh)
    err_j5 = np.linalg.norm(j5_link2 - j5_shin)

    # 关节角
    t1 = np.degrees(joint_q(d, "j_旋转1"))
    t2 = np.degrees(joint_q(d, "j_旋转2"))
    t3 = np.degrees(joint_q(d, "j_旋转3"))
    t6 = np.degrees(joint_q(d, "j_旋转6"))
    t7 = np.degrees(joint_q(d, "j_旋转7"))
    t4 = np.degrees(joint_q(d, "j_旋转4"))

    print(f"  目标 θ1={theta1_deg:+6.1f}° θ2={theta2_deg:+6.1f}°")
    print(f"  实际 θ1={t1:+7.2f}° θ2={t2:+7.2f}° | 被动 θ3={t3:+7.2f}° θ6={t6:+7.2f}° θ7={t7:+7.2f}° θ4={t4:+7.2f}°")
    print(f"  闭环误差: J2'={err_j2*1000:.4f}mm  J5={err_j5*1000:.4f}mm")
    # 小腿末端（J5）世界坐标
    print(f"  J5 世界坐标: ({j5_shin[0]*1000:+8.2f}, {j5_shin[2]*1000:+8.2f}) mm (X,Z)")
    return err_j2, err_j5, t1, t2, t3


def scan(m, d, range_deg=30, step=10):
    """扫描 θ1×θ2 工作空间。"""
    print(f"\n=== 工作空间扫描 ±{range_deg}° 步长 {step}° ===")
    angles = range(-range_deg, range_deg + 1, step)
    ok_count = 0
    total = 0
    for t2 in angles:
        for t1 in angles:
            # 每次重置（从初始位姿出发，避免上一个姿态的残留）
            mujoco.mj_resetData(m, d)
            set_targets(m, d, t1, t2)
            settle(m, d, max_steps=6000)
            # 算约束误差
            j5_shin = site_world(d, "s_J5_shin")
            j5_link2 = site_world(d, "s_J5_link2")
            err_j5 = np.linalg.norm(j5_link2 - j5_shin)
            # J2' 误差
            kr_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "knee_rotor")
            kr_xpos = d.xpos[kr_id]
            kr_xmat = d.xmat[kr_id].reshape(3, 3)
            j2p = kr_xpos + kr_xmat @ np.array([0.01212, 0, 0.007])
            j2_thigh = site_world(d, "s_J2_thigh")
            err_j2 = np.linalg.norm(j2p - j2_thigh)
            total += 1
            ok = (err_j2 < 0.0005 and err_j5 < 0.0005)  # 0.5mm 容差
            if ok:
                ok_count += 1
            mark = "✓" if ok else "✗"
            print(f"  θ1={t1:+4d} θ2={t2:+4d}: J2'={err_j2*1000:6.2f}mm J5={err_j5*1000:6.2f}mm {mark}")
    print(f"\n闭合率: {ok_count}/{total} = {ok_count/total*100:.0f}%")


def main():
    print(f"MuJoCo {mujoco.__version__}")
    m, d = load()
    print(f"模型加载成功: {m.nbody} bodies, {m.njnt} joints, {m.neq} equality constraints, {m.nu} actuators")
    print(f"  关节: {[mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(m.njnt)]}")

    # 单点测试：θ1=0, θ2=20°
    print("\n=== 单点测试: θ1=0° θ2=20° ===")
    mujoco.mj_resetData(m, d)
    set_targets(m, d, 0, 20)
    settle(m, d)
    report(m, d, 0, 20)

    # 负方向（之前 JS 版本不稳定的地方）
    print("\n=== 单点测试: θ1=0° θ2=-20° ===")
    mujoco.mj_resetData(m, d)
    set_targets(m, d, 0, -20)
    settle(m, d)
    report(m, d, 0, -20)

    # 大角度
    print("\n=== 单点测试: θ1=20° θ2=20° ===")
    mujoco.mj_resetData(m, d)
    set_targets(m, d, 20, 20)
    settle(m, d)
    report(m, d, 20, 20)

    # 工作空间扫描
    scan(m, d, range_deg=30, step=10)


if __name__ == "__main__":
    main()
