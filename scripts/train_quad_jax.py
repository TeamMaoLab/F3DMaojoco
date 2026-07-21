#!/usr/bin/env python3
"""四足 RL 训练（GPU 版，Brax PPO + MJX）

用 Brax PPO 在 MJX 上训练四足站立/踏步，1024 环境并行。
jax 0.6.0 + brax 0.14 + cuda12，GPU 直跑。

用法:
  uv run python3 scripts/train_quad_jax.py                       # stand, 50M 步
  uv run python3 scripts/train_quad_jax.py --task march
  uv run python3 scripts/train_quad_jax.py --timesteps 5000000
"""
import argparse
import os
import sys
import time
import pickle

os.environ['XLA_PYTHON_CLIENT_PREALLOCATE'] = 'false'
os.environ['XLA_PYTHON_CLIENT_MEM_FRACTION'] = '0.40'

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import jax
import jax.numpy as jnp
import numpy as np


def train(args):
    from brax.training.agents.ppo.train import train as ppo_train
    from brax.training.agents.ppo import networks as ppo_networks
    from quad_env_jax import QuadEnvJax

    print(f'=== Brax PPO GPU 训练（jax {jax.__version__}）===')
    print(f'任务: {args.task}, GPU: {jax.devices()}')
    print(f'总步数: {args.timesteps:,}, 并行环境: {args.num_envs}')
    print()

    # 网络工厂：照搬 SB3 的 [256,256] + init_noise_std=0.1（更小，防乱动）
    def network_factory(obs_size, action_size, preprocess_observations_fn):
        return ppo_networks.make_ppo_networks(
            observation_size=obs_size,
            action_size=action_size,
            preprocess_observations_fn=preprocess_observations_fn,
            policy_hidden_layer_sizes=(256, 256),
            value_hidden_layer_sizes=(256, 256),
            init_noise_std=0.1,  # 小 std，防初始乱动把机器人弹飞
        )

    # PPO 超参
    inference_fn, params, metrics = ppo_train(
        environment=QuadEnvJax(task=args.task),
        num_timesteps=args.timesteps,
        num_envs=args.num_envs,
        episode_length=1000,
        action_repeat=1,
        learning_rate=3e-4,
        entropy_cost=0.01,       # 加 entropy（让 std 不爆炸，SB3 默认也有）
        discounting=0.99,
        gae_lambda=0.95,
        clipping_epsilon=0.2,
        vf_loss_coefficient=0.5,
        batch_size=args.num_envs * 20,
        num_minibatches=32,
        num_updates_per_batch=4,
        unroll_length=20,
        normalize_observations=True,
        reward_scaling=1.0,
        max_grad_norm=0.5,
        network_factory=network_factory,
        seed=args.seed,
        num_evals=args.num_evals,
    )

    print(f'\n=== 训练完成 ===')
    if isinstance(metrics, dict):
        for k, v in metrics.items():
            v_arr = np.array(v)
            print(f'  {k}: {v_arr}')

    # 保存模型（params + inference_fn 的信息）
    os.makedirs(args.save_dir, exist_ok=True)
    save_path = os.path.join(args.save_dir, f'quad_jax_{args.task}.pkl')
    def to_numpy(x):
        return np.array(x)
    save_data = {
        'params': jax.tree_util.tree_map(to_numpy, params),
        'task': args.task,
        'jax_version': jax.__version__,
    }
    with open(save_path, 'wb') as f:
        pickle.dump(save_data, f)
    print(f'模型保存: {save_path}')
    print(f'inference_fn: {inference_fn}')

    return inference_fn, params


def main():
    ap = argparse.ArgumentParser(description='四足 RL GPU 训练（Brax PPO + MJX）')
    ap.add_argument('--task', choices=['stand', 'march'], default='stand')
    ap.add_argument('--timesteps', type=int, default=50_000_000)
    ap.add_argument('--num-envs', type=int, default=1024)
    ap.add_argument('--num-evals', type=int, default=10)
    ap.add_argument('--save-dir', default='saved_jax')
    ap.add_argument('--seed', type=int, default=0)
    args = ap.parse_args()
    train(args)


if __name__ == '__main__':
    main()
