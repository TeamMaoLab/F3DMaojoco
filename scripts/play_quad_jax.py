#!/usr/bin/env python3
"""用训练好的 Brax PPO 策略驱动 MuJoCo viewer（GPU 策略 + CPU 渲染）

加载 Brax PPO 训练的模型，在 CPU MuJoCo viewer 里实时演示。
策略在 GPU（JAX），渲染在 CPU（MuJoCo viewer），每步桥接。

用法:
  uv run python3 scripts/play_quad_jax.py saved_jax/quad_jax_stand.pkl
  uv run python3 scripts/play_quad_jax.py saved_jax/quad_jax_stand.pkl --episodes 3
"""
import argparse
import os
import sys
import time
import pickle

os.environ['XLA_PYTHON_CLIENT_PREALLOCATE'] = 'false'
os.environ['XLA_PYTHON_CLIENT_MEM_FRACTION'] = '0.20'

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import jax
import jax.numpy as jnp
import mujoco
import mujoco.viewer


def main():
    ap = argparse.ArgumentParser(description='Brax PPO 策略 viewer 演示')
    ap.add_argument('model', help='模型路径（.pkl）')
    ap.add_argument('--episodes', type=int, default=0, help='跑几轮（0=无限）')
    args = ap.parse_args()

    if not os.path.exists(args.model):
        print(f'模型不存在: {args.model}')
        return

    # 加载模型
    with open(args.model, 'rb') as f:
        saved = pickle.load(f)
    params = saved['params']  # (normalizer, policy, value)
    task = saved.get('task', 'stand')
    print(f'加载模型: {args.model} (task={task})')

    # 重建 Brax inference 函数
    from brax.training.agents.ppo import networks as ppo_networks
    networks_obj = ppo_networks.make_ppo_networks(
        observation_size=37, action_size=8,
        policy_hidden_layer_sizes=(256, 256), value_hidden_layer_sizes=(256, 256),
        init_noise_std=0.2,
    )
    make_policy = ppo_networks.make_inference_fn(networks_obj)
    policy_fn = make_policy(params=params, deterministic=True)

    # CPU MuJoCo 环境（用于 viewer）
    from quad_env_jax import QuadEnvJax, ACTION_SCALE, MIRROR_MASK, TARGET_HEIGHT
    env = QuadEnvJax(task=task)

    # 用 CPU MuJoCo 直接渲染（不走 JAX 环境，但复用索引和 obs 计算）
    m = mujoco.MjModel.from_xml_path('mujoco_quad/quad_dyn.xml')
    d = mujoco.MjData(m)

    # 站立初始化
    def init_standing():
        d.qpos[:] = np.array(env.home_qpos)
        d.qvel[:] = 0
        d.ctrl[:] = 0
        mujoco.mj_forward(m, d)

    # 从 CPU MjData 构造 obs（和 quad_env_jax._get_obs 一致）
    def get_obs(last_action):
        base_id = env._base_idx
        base_angvel = d.qvel[3:6] / 2.0
        base_mat = d.xmat[base_id].reshape(3, 3)
        grav_proj = base_mat.T @ np.array([0.0, 0.0, -1.0])
        base_linvel = d.qvel[0:3]
        joint_pos = d.qpos[np.array(env.servo_qposadr)] / 1.5708
        joint_vel = d.qvel[np.array(env.servo_dofadr)] / 10.0
        foot_contact = np.zeros(4, dtype=np.float32)
        for k, sid in enumerate(env.foot_site_ids):
            foot_contact[k] = 1.0 if d.site_xpos[sid][2] < 0.008 else 0.0
        return np.concatenate([
            base_angvel, grav_proj, base_linvel,
            joint_pos, joint_vel, foot_contact, last_action
        ]).astype(np.float32)

    def check_fallen():
        base_z = d.qpos[2]
        upright = d.xmat[env._base_idx].reshape(3, 3)[2, 2]
        return base_z < 0.045 or upright < 0.707

    print(f'\n=== Brax PPO 策略演示 ===')
    print(f'关 viewer 窗口退出\n')

    ep = 0
    rng = jax.random.PRNGKey(42)
    with mujoco.viewer.launch_passive(m, d) as viewer:
        while viewer.is_running():
            if args.episodes and ep >= args.episodes:
                break
            ep += 1
            init_standing()
            last_action = np.zeros(8, dtype=np.float32)
            step = 0
            t0 = time.time()
            print(f'[Episode {ep}] 开始')

            while viewer.is_running():
                # GPU 策略算 action
                obs = jnp.array(get_obs(last_action))
                rng, k = jax.random.split(rng)
                action, _ = policy_fn(obs, k)
                action_np = np.clip(np.array(action), -1, 1)

                # 写入 CPU MuJoCo（镜像腿取负）
                ctrl = ACTION_SCALE * action_np * np.array(MIRROR_MASK)
                d.ctrl[:] = ctrl
                last_action = action_np.astype(np.float32)

                mujoco.mj_step(m, d)
                viewer.sync()
                step += 1

                if step % 200 == 0:
                    z = d.qpos[2]
                    fallen = check_fallen()
                    status = '⚠摔倒' if fallen else '✓稳定'
                    print(f'  step {step:4d}: base_z={z*1000:6.1f}mm {status}')

                if check_fallen() or step > 3000:
                    elapsed = time.time() - t0
                    print(f'  → 结束: {step}步, {elapsed:.1f}s, base_z={d.qpos[2]*1000:.0f}mm\n')
                    break
                time.sleep(0.005)

    print(f'完成 {ep} 个 episode')


if __name__ == '__main__':
    main()
