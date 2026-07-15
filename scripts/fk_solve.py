"""MuJoCo 单腿 FK 求解：伺服角作为运动学输入（直接锁定 qpos），被动关节靠约束松弛。

策略：
- θ1/θ2 直接写进 qpos，每步强制保持（qvel=0），相当于伺服轴"绝对硬"
- 4 个被动关节自由运动
- equality/connect 约束把两个闭环点拉到一起
- 跑 mj_step 让被动关节收敛到约束满足的位置

这样避免了 PD vs 约束力打架的问题——伺服是"位置源"，不是"力源"。
"""
import numpy as np
import mujoco

MODEL_PATH = "mujoco_leg/leg.xml"


def load():
    m = mujoco.MjModel.from_xml_path(MODEL_PATH)
    d = mujoco.MjData(m)
    return m, d


def joint_adr(m, name):
    jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
    return m.jnt_qposadr[jid], m.jnt_dofadr[jid]


def solve_fk(m, d, theta1_deg, theta2_deg, max_steps=20000):
    """给定 θ1/θ2（度），求解所有被动关节，返回关节角 + 闭环误差。"""
    mujoco.mj_resetData(m, d)
    t1 = np.radians(theta1_deg)
    t2 = np.radians(theta2_deg)

    a1_q, a1_d = joint_adr(m, "j_旋转1")
    a2_q, a2_d = joint_adr(m, "j_旋转2")

    # 锁定伺服：清 ctrl（不用 actuator），每步强制 qpos/qvel
    d.ctrl[:] = 0

    for i in range(max_steps):
        # 强制伺服到目标（运动学输入）
        d.qpos[a1_q] = t1
        d.qpos[a2_q] = t2
        d.qvel[a1_d] = 0
        d.qvel[a2_d] = 0
        mujoco.mj_step(m, d)
        # 步进后再锁（因为 mj_step 会积分 qpos）
        d.qpos[a1_q] = t1
        d.qpos[a2_q] = t2
        d.qvel[a1_d] = 0
        d.qvel[a2_d] = 0
        if i % 500 == 0 and i > 1000:
            # 被动关节速度收敛？
            passive_vel = np.linalg.norm(d.qvel)
            if passive_vel < 1e-4:
                break

    return _collect(m, d, theta1_deg, theta2_deg)


def _collect(m, d, theta1_deg, theta2_deg):
    def jq(name):
        q, _ = joint_adr(m, name)
        return np.degrees(d.qpos[q])

    def site(name):
        sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, name)
        return d.site_xpos[sid].copy()

    j2_thigh = site("s_J2_thigh")
    j5_shin = site("s_J5_shin")
    j5_link2 = site("s_J5_link2")
    # J2' 世界坐标（膝盖转动 body 上的 anchor）
    kr_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "knee_rotor")
    kr_xpos = d.xpos[kr_id]
    kr_xmat = d.xmat[kr_id].reshape(3, 3)
    j2p = kr_xpos + kr_xmat @ np.array([0.01212, 0, 0.007])

    err_j2 = np.linalg.norm(j2p - j2_thigh)
    err_j5 = np.linalg.norm(j5_link2 - j5_shin)

    return {
        "t1": jq("j_旋转1"), "t2": jq("j_旋转2"),
        "t3": jq("j_旋转3"), "t4": jq("j_旋转4"),
        "t6": jq("j_旋转6"), "t7": jq("j_旋转7"),
        "err_j2_mm": err_j2 * 1000,
        "err_j5_mm": err_j5 * 1000,
        "j5_world_xz": (j5_shin[0] * 1000, j5_shin[2] * 1000),
    }


def main():
    print(f"MuJoCo {mujoco.__version__}")
    m, d = load()
    print(f"模型: {m.nbody} bodies, {m.njnt} joints, {m.neq} equality\n")

    # 单点测试
    for t1, t2 in [(0, 0), (0, 20), (0, -20), (20, 20), (20, -20), (-20, 20), (30, 30)]:
        r = solve_fk(m, d, t1, t2)
        ok = "✓" if (r["err_j2_mm"] < 0.5 and r["err_j5_mm"] < 0.5) else "✗"
        print(f"θ1={t1:+4d} θ2={t2:+4d} → 实际 t1={r['t1']:+6.1f} t2={r['t2']:+6.1f} | "
              f"t3={r['t3']:+7.1f} t6={r['t6']:+7.1f} t7={r['t7']:+7.1f} t4={r['t4']:+7.1f} | "
              f"J2'={r['err_j2_mm']:5.2f}mm J5={r['err_j5_mm']:5.2f}mm {ok}")

    # 工作空间扫描
    print("\n=== 工作空间扫描 ±30° 步长 10° ===")
    ok_count = 0
    total = 0
    for t2 in range(-30, 31, 10):
        line = ""
        for t1 in range(-30, 31, 10):
            r = solve_fk(m, d, t1, t2, max_steps=8000)
            ok = r["err_j2_mm"] < 0.5 and r["err_j5_mm"] < 0.5
            total += 1
            if ok:
                ok_count += 1
            line += "█" if ok else "·"
        print(f"  θ2={t2:+3d}: {line}")
    print(f"\n闭合率: {ok_count}/{total} = {ok_count/total*100:.0f}%")


if __name__ == "__main__":
    main()
