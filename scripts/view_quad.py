#!/usr/bin/env python3
"""四足 MuJoCo 模型查看器

一键启动 MuJoCo viewer，加载四足模型并保持站立。
机器人会自动维持站立姿态（PD 控制器 ctrl=0），可自由旋转视角观察。

用法:
  python3 scripts/view_quad.py              # dyn 版（浮动 base + 重力 + 地面）
  python3 scripts/view_quad.py --kin        # kin 版（固定 base + 零重力）
  python3 scripts/view_quad.py --walk       # dyn 版 + trot 步态（腿会动起来）
"""
import argparse
import time
import numpy as np
import mujoco
import mujoco.viewer


def init_standing(m, d, settle_steps=3000):
    """初始化到真正站立姿态（足端触地，base 落到稳态高度）。

    流程：
      1. base 抬高到 z=120mm（足端悬空）
      2. 无重力 settle：让 equality 闭环约束收敛
      3. 恢复重力，继续跑：机器人自然下落，足端触地后稳定在站立高度
      4. 清零速度，得到干净的站立初态
    """
    d.qpos[:] = 0
    d.qpos[2] = 0.120   # base z = 120mm（足端离地，避免初始穿透弹射）
    d.qpos[6] = 1.0     # qw = 1（单位四元数）
    d.ctrl[:] = 0       # 8 舵机目标 = 0（站立位）

    # 1. 无重力 settle（equality 收敛）
    saved_gravity = m.opt.gravity.copy()
    m.opt.gravity[:] = 0
    for _ in range(2000):
        mujoco.mj_step(m, d)

    # 2. 恢复重力，下落到站立稳态（足端触地）
    m.opt.gravity[:] = saved_gravity
    for _ in range(settle_steps):
        d.ctrl[:] = 0
        mujoco.mj_step(m, d)

    # 3. 清零残余速度（干净的站立初态）
    d.qvel[:] = 0


def walk_gait(t, freq=1.5, amp=0.2, ramp=1.0):
    """trot 步态：对角腿交替抬落，返回 8 舵机控制指令（弧度）。

    正确 trot 的关键：对角两组腿（FR+RL / FL+RR）相位差 π，
    一组抬腿时另一组必须保持站立位（提供支撑）。
    用「抬腿高度」驱动：每条腿在自己的摆动相把膝盖舵机收回（抬脚），
    撑地相膝盖舵机回到站立位 0（踩地）。

    参数:
      freq: 步频 (Hz)
      amp: 膝盖抬起幅度 (弧度)，0.2 ≈ 11°，抬脚离地
      ramp: 启动过渡 (秒)，避免突变冲击
    """
    scale = min(1.0, t / ramp) if ramp > 0 else 1.0
    phase = 2 * np.pi * freq * t  # 连续相位

    # 每条腿的摆动函数：用 max(0, sin) 让"抬腿"只发生在半周期，
    # 另半周期为 0（踩地站立位），保证对角两组腿总有一组踩地
    def swing(ph):
        # 抬腿高度，范围 [0, amp]，sin 正半周才抬
        return amp * scale * max(0, np.sin(ph))

    # 对角分组：FR/RL 相位 0，FL/RR 相位 π
    knee_lift_FR = swing(phase)
    knee_lift_RL = swing(phase)          # FR+RL 同相
    knee_lift_FL = swing(phase + np.pi)  # FL+RR 反相
    knee_lift_RR = swing(phase + np.pi)

    # 膝盖舵机（j_旋转1）：抬腿时收膝（正值收膝，让脚抬起）
    # 大腿舵机（j_旋转2）：保持 0（不前后摆），只做原地踏步
    # 镜像腿（FL/RL）舵机指令取负
    fr_knee, fr_thigh = knee_lift_FR, 0
    rr_knee, rr_thigh = knee_lift_RR, 0
    fl_knee, fl_thigh = -knee_lift_FL, 0   # 镜像取负
    rl_knee, rl_thigh = -knee_lift_RL, 0

    # ctrl 顺序: FR_knee, FR_thigh, RR_knee, RR_thigh, FL_knee, FL_thigh, RL_knee, RL_thigh
    return np.array([
        fr_knee, fr_thigh,
        rr_knee, rr_thigh,
        fl_knee, fl_thigh,
        rl_knee, rl_thigh,
    ])


def main():
    ap = argparse.ArgumentParser(description='四足 MuJoCo 模型查看器')
    ap.add_argument('--kin', action='store_true', help='用 kin 版（固定 base + 零重力）')
    ap.add_argument('--walk', action='store_true', help='dyn 版 + trot 步态（腿会动）')
    ap.add_argument('--model', default=None, help='自定义模型路径')
    args = ap.parse_args()

    if args.model:
        path = args.model
    elif args.kin:
        path = 'mujoco_quad/quad_kin.xml'
    else:
        path = 'mujoco_quad/quad_dyn.xml'

    m = mujoco.MjModel.from_xml_path(path)
    d = mujoco.MjData(m)
    init_standing(m, d)

    mode = 'kin' if args.kin else ('walk' if args.walk else 'stand')
    print(f'模型: {path}')
    print(f'模式: {mode}（{m.nbody} bodies, {m.njnt} joints, {m.nu} actuators）')
    print(f'操作: 鼠标拖动旋转 / 滚轮缩放 / 右键平移 / 空格暂停 / 关窗口退出')
    print()

    with mujoco.viewer.launch_passive(m, d) as viewer:
        t0 = time.time()
        while viewer.is_running():
            if args.walk and not args.kin:
                t = time.time() - t0
                d.ctrl[:] = walk_gait(t)
            else:
                d.ctrl[:] = 0
            mujoco.mj_step(m, d)
            viewer.sync()
            time.sleep(0.01)


if __name__ == '__main__':
    main()
