#!/usr/bin/env python3
"""用训练好的 RL 策略驱动 MuJoCo viewer 看机器人表现

加载 PPO checkpoint，实时控制机器人，看它学会了什么。
比边训边看更稳定（不会因为 GLFW 跨进程冲突 segfault）。

用法:
  uv run python3 scripts/play_quad.py saved/quad_ppo_stand_final.zip
  uv run python3 scripts/play_quad.py saved/quad_ppo_stand_final.zip --deterministic
  uv run python3 scripts/play_quad.py saved/quad_ppo_stand_50000_steps.zip  # 看中间 checkpoint

操作:
  viewer 窗口: 鼠标拖动旋转视角
  终端会实时打印 reward / base z / 是否摔倒
  关 viewer 窗口退出
"""
import argparse
import os
import sys
import time
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from quad_env import QuadEnv


def main():
    ap = argparse.ArgumentParser(description='RL 策略 viewer 演示')
    ap.add_argument('model', help='PPO 模型路径（.zip）')
    ap.add_argument('--task', default='stand', choices=['stand', 'march'])
    ap.add_argument('--deterministic', action='store_true', default=True,
                    help='确定性策略（默认，评估用）')
    ap.add_argument('--episodes', type=int, default=0, help='跑几轮（0=无限）')
    ap.add_argument('--perturb', type=float, default=0.0,
                    help='随机扰动幅度（rad，测试鲁棒性，如 0.3）')
    args = ap.parse_args()

    if not os.path.exists(args.model):
        print(f'模型不存在: {args.model}')
        print('先训练: uv run python3 scripts/train_quad.py --timesteps 50000')
        return

    import mujoco
    import mujoco.viewer
    from stable_baselines3 import PPO

    # 加载模型
    print(f'加载模型: {args.model}')
    model = PPO.load(args.model, device='cpu')

    # 创建环境（直接用 MuJoCo data，不走 Gym 接口，方便 viewer）
    m = mujoco.MjModel.from_xml_path('mujoco_quad/quad_dyn.xml')
    d = mujoco.MjData(m)

    # 复用 QuadEnv 的索引计算
    env = QuadEnv(task=args.task)
    base_id = env.base_id
    servo_qposadr = env.servo_qposadr
    mirror_mask = env.mirror_mask
    target_height = env.target_height
    foot_site_ids = env.foot_site_ids

    def init_standing():
        d.qpos[:] = 0
        d.qpos[2] = 0.120
        d.qpos[6] = 1.0
        d.ctrl[:] = 0
        d.qvel[:] = 0
        saved = m.opt.gravity.copy()
        m.opt.gravity[:] = 0
        for _ in range(1000):
            mujoco.mj_step(m, d)
        m.opt.gravity[:] = saved
        for _ in range(500):
            d.ctrl[:] = 0
            mujoco.mj_step(m, d)
        d.qvel[:] = 0

    def get_obs():
        base_angvel = d.qvel[3:6].copy()
        base_linvel = d.qvel[0:3].copy()
        base_mat = d.xmat[base_id].reshape(3, 3)
        grav_proj = base_mat.T @ np.array([0, 0, -9.81]) / 9.81
        joint_pos = d.qpos[servo_qposadr].copy() / 1.5708
        joint_vel = d.qvel[env.servo_dofadr].copy() / 10.0
        foot = np.zeros(4, dtype=np.float32)
        for k, sid in enumerate(foot_site_ids):
            foot[k] = 1.0 if d.site_xpos[sid][2] < 0.008 else 0.0
        last_act = getattr(get_obs, '_last_act', np.zeros(8, dtype=np.float32))
        obs = np.concatenate([
            base_angvel / 2.0, grav_proj, base_linvel, joint_pos,
            joint_vel, foot, last_act,
        ]).astype(np.float32)
        return obs

    def check_fallen():
        """摔倒判定：用 base Z 轴朝向（避免欧拉角 gimbal lock）"""
        base_z = d.qpos[2]
        # base 局部 Z 轴在世界系的方向（xmat 第 3 列）
        base_mat = d.xmat[base_id].reshape(3, 3)
        z_axis_world = base_mat[:, 2]  # base Z 在世界系
        # 正立时 z_axis_world ≈ [0,0,1]，倾倒时 z 分量变小
        upright = z_axis_world[2]  # cos(tilt)
        # upright < cos(45°)=0.707 表示倾斜超过 45°
        if base_z < 0.045 or upright < 0.707:
            return True
        return False

    print(f'\n=== RL 策略演示 ===')
    print(f'任务: {args.task}, 扰动: ±{args.perturb} rad' if args.perturb else '')
    print(f'关 viewer 窗口退出\n')

    action_scale = env.action_scale
    ep = 0
    with mujoco.viewer.launch_passive(m, d) as viewer:
        while viewer.is_running():
            # 每个 episode 开始
            if args.episodes and ep >= args.episodes:
                break
            ep += 1
            init_standing()
            # 随机扰动
            if args.perturb > 0:
                angle = np.random.uniform(-args.perturb, args.perturb)
                axis = np.random.randn(3)
                axis /= np.linalg.norm(axis) + 1e-9
                d.qpos[3:7] = np.concatenate([[np.cos(angle/2)], axis * np.sin(angle/2)])

            get_obs._last_act = np.zeros(8, dtype=np.float32)
            step = 0
            total_r = 0.0
            t0 = time.time()
            print(f'[Episode {ep}] 开始，初始扰动 {args.perturb} rad' if args.perturb else f'[Episode {ep}] 开始')

            while viewer.is_running():
                obs = get_obs()
                action, _ = model.predict(obs, deterministic=args.deterministic)
                action = np.clip(action, -1, 1)
                # 映射到舵机目标（镜像腿取负）
                target = action_scale * action * mirror_mask
                d.ctrl[:] = target
                get_obs._last_act = action.astype(np.float32)

                mujoco.mj_step(m, d)
                viewer.sync()
                step += 1

                # 简化 reward 显示
                z = d.qpos[2]
                total_r += 1.0 + np.exp(-abs(z - target_height) * 50) * 2.0

                # 每 200 步打印一次状态
                if step % 200 == 0:
                    fallen = check_fallen()
                    status = '⚠摔倒' if fallen else '✓稳定'
                    print(f'  step {step:4d}: base_z={z*1000:6.1f}mm reward≈{total_r:.0f} {status}')

                if check_fallen() or step > 3000:
                    elapsed = time.time() - t0
                    print(f'  → 结束: {step}步, reward≈{total_r:.0f}, '
                          f'{elapsed:.1f}s, base_z={d.qpos[2]*1000:.0f}mm\n')
                    break
                time.sleep(0.005)  # 给 viewer 时间渲染

    print(f'完成 {ep} 个 episode')


if __name__ == '__main__':
    main()
