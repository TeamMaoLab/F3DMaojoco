#!/usr/bin/env python3
"""四足舵机机器人 RL 训练脚本（PPO + Stable-Baselines3）

用 PPO 训练四足站立平衡，带 TensorBoard 监控 + checkpoint 保存。

用法:
  uv run python3 scripts/train_quad.py                 # 训练站立（默认）
  uv run python3 scripts/train_quad.py --task march    # 训练原地踏步
  uv run python3 scripts/train_quad.py --timesteps 500000  # 自定义步数
  uv run python3 scripts/train_quad.py --eval saved/quad_ppo_stand.zip  # 评估

训练日志:
  tensorboard --logdir saved/tb  → 浏览器打开 http://localhost:6006
"""
import argparse
import os
import sys
import time
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from quad_env import QuadEnv


def make_env(task, seed=0):
    """环境工厂（SB3 VecEnv 需要）"""
    def _init():
        env = QuadEnv(task=task)
        env.reset(seed=seed)
        return env
    return _init


def train(args):
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor
    from stable_baselines3.common.callbacks import CheckpointCallback, BaseCallback

    # ===== 多进程环境（加速训练）=====
    n_envs = args.n_envs
    env_fns = [make_env(args.task, seed=i) for i in range(n_envs)]
    vec_env = SubprocVecEnv(env_fns)
    # VecMonitor 记录 episode reward/length（tensorboard 可用时才写文件）
    try:
        import tensorboard  # noqa
        vec_env = VecMonitor(vec_env, filename=os.path.join(args.tb_dir, 'monitor'))
    except ImportError:
        vec_env = VecMonitor(vec_env)
        print('（未装 tensorboard，跳过日志文件。pip install tensorboard 后可用）')

    print(f'=== PPO 训练 ===')
    print(f'任务: {args.task}')
    print(f'环境数: {n_envs}（并行）')
    print(f'总步数: {args.timesteps}（约 {args.timesteps * 0.002:.0f}s 仿真时间）')
    print(f'输出: {args.save_dir}/')

    # ===== PPO 超参数（针对轻机器人和闭环机构调过）=====
    model = PPO(
        policy='MlpPolicy',
        env=vec_env,
        # 网络结构
        policy_kwargs=dict(
            net_arch=dict(pi=[256, 256], vf=[256, 256]),
            # 关键：初始 std 小（0.2），让策略从"小动作"开始探索
            # 默认 std=1.0 会让初始 action 太大把机器人搞倒
            log_std_init=np.log(0.2),
        ),
        learning_rate=3e-4,
        n_steps=4096,           # 加大 rollout（每环境收集更多步，episode 覆盖更全）
        batch_size=128,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.0,           # 关闭熵奖励（不需要更多探索，已知 action=0 是好的）
        vf_coef=0.5,
        max_grad_norm=0.5,
        verbose=1,
        tensorboard_log=args.tb_dir if args.tb_dir else None,
        device='cpu',
    )

    # ===== Checkpoint 回调（定期保存）=====
    checkpoint_cb = CheckpointCallback(
        save_freq=max(args.timesteps // 20, 10000),  # 存 20 次
        save_path=args.save_dir,
        name_prefix=f'quad_ppo_{args.task}',
        save_replay_buffer=False,
    )

    # ===== 训练时长回调（打印进度）=====
    class ProgressCallback(BaseCallback):
        def __init__(self, total):
            super().__init__()
            self.total = total
            self.t0 = time.time()

        def _on_step(self):
            if self.num_timesteps % 10000 == 0:
                elapsed = time.time() - self.t0
                rate = self.num_timesteps / elapsed if elapsed > 0 else 0
                pct = self.num_timesteps / self.total * 100
                print(f'  [{pct:5.1f}%] {self.num_timesteps}/{self.total} 步 '
                      f'({rate:.0f} 步/秒, {elapsed:.0f}s)', flush=True)
            return True

    # ===== 开始训练 =====
    t0 = time.time()
    model.learn(
        total_timesteps=args.timesteps,
        callback=[checkpoint_cb, ProgressCallback(args.timesteps)],
    )
    elapsed = time.time() - t0
    print(f'\n=== 训练完成 ({elapsed:.0f}s, {args.timesteps/elapsed:.0f} 步/秒) ===')

    # 保存最终模型
    final_path = os.path.join(args.save_dir, f'quad_ppo_{args.task}_final.zip')
    model.save(final_path)
    print(f'最终模型: {final_path}')
    vec_env.close()


def evaluate(args):
    """加载模型评估（看训练效果）"""
    from stable_baselines3 import PPO

    env = QuadEnv(task=args.task, max_steps=2000, render_mode=None)
    model = PPO.load(args.eval, env=env)

    print(f'=== 评估 {args.eval} ===')
    n_episodes = 5
    rewards = []
    for ep in range(n_episodes):
        obs, info = env.reset(seed=ep)
        total_r = 0
        steps = 0
        while True:
            action, _ = model.predict(obs, deterministic=True)
            obs, r, term, trunc, info = env.step(action)
            total_r += r
            steps += 1
            if term or trunc:
                break
        rewards.append(total_r)
        reason = info.get('episode', {}).get('trunc', info.get('trunc', '?'))
        print(f'  episode {ep}: reward={total_r:.1f}, steps={steps}, '
              f'base_z={env.data.qpos[2]*1000:.0f}mm, end={reason}')
    print(f'\n平均 reward: {np.mean(rewards):.1f} ± {np.std(rewards):.1f}')
    env.close()


def main():
    ap = argparse.ArgumentParser(description='四足 RL 训练')
    ap.add_argument('--task', choices=['stand', 'march'], default='stand')
    ap.add_argument('--timesteps', type=int, default=500000)
    ap.add_argument('--n-envs', type=int, default=8)
    ap.add_argument('--save-dir', default='saved')
    ap.add_argument('--tb-dir', default='saved/tb')
    ap.add_argument('--eval', default=None, help='评估模式：加载模型路径')
    args = ap.parse_args()

    os.makedirs(args.save_dir, exist_ok=True)
    if args.tb_dir:
        os.makedirs(args.tb_dir, exist_ok=True)

    if args.eval:
        evaluate(args)
    else:
        train(args)


if __name__ == '__main__':
    main()
