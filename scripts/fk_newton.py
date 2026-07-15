"""MuJoCo 单腿 FK 求解（纯运动学，牛顿迭代）。

不依赖 MuJoCo 的动力学/约束求解器，而是：
1. θ1/θ2 锁定（直接设 qpos）
2. mj_forward 算出所有 body 的世界坐标
3. 读约束残差（两个 anchor 的世界距离）
4. 用数值雅可比（对被动关节扰动）+ 牛顿迭代解出被动关节
5. 这样得到的是精确解，不受 PD/约束软硬度影响

这本质上是把 WebPreviewer 的 solveFKCoupled 用 MuJoCo 的 FK 引擎重写。
"""
import numpy as np
import mujoco

MODEL_PATH = "mujoco_leg/leg.xml"

# 4 个被动关节（顺序：t6, t7, t4, t3）
PASSIVE_JOINTS = ["j_旋转6", "j_旋转7", "j_旋转4", "j_旋转3"]
SERVO_JOINTS = ["j_旋转1", "j_旋转2"]


def load():
    m = mujoco.MjModel.from_xml_path(MODEL_PATH)
    d = mujoco.MjData(m)
    return m, d


def joint_qadr(m, name):
    jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
    return m.jnt_qposadr[jid]


def set_servos(m, d, t1_deg, t2_deg):
    d.qpos[joint_qadr(m, "j_旋转1")] = np.radians(t1_deg)
    d.qpos[joint_qadr(m, "j_旋转2")] = np.radians(t2_deg)


def constraint_residual(m, d):
    """返回 4 维约束残差：[J2'_x, J2'_z, J5_x, J5_z] 相对目标点。

    J2' 应该到 (0, 0) (J2 位置，大腿上)
    J5 应该在膝盖传动2 和小腿上的两个点重合
    """
    mujoco.mj_forward(m, d)  # 保证 xpos/xmat 最新

    # J2' 世界坐标（膝盖转动 body 上的 anchor）
    kr_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "knee_rotor")
    kr_xpos = d.xpos[kr_id]
    kr_xmat = d.xmat[kr_id].reshape(3, 3)
    j2p = kr_xpos + kr_xmat @ np.array([0.01212, 0, 0.007])

    # J5 在膝盖传动2 上
    kl2_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "knee_link2")
    kl2_xpos = d.xpos[kl2_id]
    kl2_xmat = d.xmat[kl2_id].reshape(3, 3)
    j5_link2 = kl2_xpos + kl2_xmat @ np.array([0.025, 0, -0.0433])

    # J5 在小腿上
    shin_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "shin")
    shin_xpos = d.xpos[shin_id]
    shin_xmat = d.xmat[shin_id].reshape(3, 3)
    j5_shin = shin_xpos + shin_xmat @ np.array([0.01212, 0, 0.007])

    # J2 在大腿上（就是大腿 body 原点）
    thigh_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "thigh_rigid")
    j2_thigh = d.xpos[thigh_id].copy()

    return np.array([
        (j2p - j2_thigh)[0],  # J2' x 残差
        (j2p - j2_thigh)[2],  # J2' z 残差
        (j5_link2 - j5_shin)[0],  # J5 x 残差
        (j5_link2 - j5_shin)[2],  # J5 z 残差
    ])


def solve_fk_newton(m, d, t1_deg, t2_deg, x0=None, max_iter=50, tol=1e-8):
    """牛顿迭代解 4 个被动关节。x0 = [t6, t7, t4, t3] 弧度初始猜测。"""
    set_servos(m, d, t1_deg, t2_deg)
    if x0 is None:
        x0 = np.zeros(4)

    qadrs = [joint_qadr(m, n) for n in PASSIVE_JOINTS]
    x = np.array(x0, dtype=float)

    for it in range(max_iter):
        for i, qa in enumerate(qadrs):
            d.qpos[qa] = x[i]
        f = constraint_residual(m, d)
        err = np.linalg.norm(f)
        if err < tol:
            break
        # 数值雅可比 4x4
        eps = 1e-6
        J = np.zeros((4, 4))
        for j in range(4):
            xp = x.copy()
            xp[j] += eps
            for i, qa in enumerate(qadrs):
                d.qpos[qa] = xp[i]
            fp = constraint_residual(m, d)
            J[:, j] = (fp - f) / eps
        # 牛顿步：J dx = -f
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


def get_state(m, d):
    """读取所有关节角（度）+ 关键点世界坐标。"""
    def jq(n):
        return np.degrees(d.qpos[joint_qadr(m, n)])
    return {
        "t1": jq("j_旋转1"), "t2": jq("j_旋转2"),
        "t3": jq("j_旋转3"), "t4": jq("j_旋转4"),
        "t6": jq("j_旋转6"), "t7": jq("j_旋转7"),
    }


def main():
    print(f"MuJoCo {mujoco.__version__} · 纯运动学牛顿迭代")
    m, d = load()
    print(f"模型: {m.nbody} bodies, {m.njnt} joints, {m.neq} equality\n")

    # 单点测试（从不同初值试，看是否找到多个分支）
    test_cases = [
        (0, 0, None),
        (0, 20, None),
        (0, -20, None),
        (20, 20, None),
        (20, -20, None),
        (-20, 20, None),
        (30, 30, None),
        (0, 40, None),
    ]
    print("=== 单点测试（从 0 初值）===")
    for t1, t2, _ in test_cases:
        x, err, it = solve_fk_newton(m, d, t1, t2, x0=np.zeros(4))
        s = get_state(m, d)
        ok = "✓" if err < 1e-4 else "✗"
        print(f"θ1={t1:+4d} θ2={t2:+4d}: 收敛 err={err*1000:.5f}mm @it{it} | "
              f"t3={s['t3']:+7.1f} t6={s['t6']:+7.1f} t7={s['t7']:+7.1f} t4={s['t4']:+7.1f} {ok}")

    # 测试分支：θ1=0 θ2=20 从不同初值，看是否找到 shear vs follow 解
    print("\n=== 分支测试：θ1=0 θ2=20，不同初值 ===")
    for t3_init in [-180, -90, 0, 90, 180]:
        # t6,t7,t4,t3 顺序
        x0 = np.array([0, 0, 0, np.radians(t3_init)])
        x, err, it = solve_fk_newton(m, d, 0, 20, x0=x0)
        s = get_state(m, d)
        ok = "✓" if err < 1e-4 else "✗"
        print(f"  t3初值={t3_init:+4d}°: err={err*1000:.5f}mm | "
              f"t3={s['t3']:+7.1f} t6={s['t6']:+7.1f} t7={s['t7']:+7.1f} t4={s['t4']:+7.1f} {ok}")

    # 工作空间扫描（连续追踪：从相邻解作初值）
    print("\n=== 工作空间扫描 ±30° 步长 5°（连续追踪）===")
    last_x0 = np.zeros(4)
    ok_count = 0
    total = 0
    grid = {}
    for t2 in range(-30, 31, 5):
        for t1 in range(-30, 31, 5):
            x, err, it = solve_fk_newton(m, d, t1, t2, x0=last_x0)
            ok = err < 1e-4
            total += 1
            if ok:
                ok_count += 1
                last_x0 = x.copy()  # 用成功解作下一个初值
            grid[(t1, t2)] = ok
        print(f"  θ2={t2:+3d}: {''.join('█' if grid[(t1,t2)] else '·' for t1 in range(-30,31,5))}")
    print(f"\n闭合率: {ok_count}/{total} = {ok_count/total*100:.0f}%")


if __name__ == "__main__":
    main()
