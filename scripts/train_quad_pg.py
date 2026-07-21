"""训练我们的四足舵机机器人（Playground Joystick 任务）。

用 brax.training.agents.ppo.train + 我们的 QuadJoystick env。
照搬 Go1JoystickFlatTerrain 的 PPO 超参，缩规模到 5M 步 + 1024 envs。

用法:
  uv run python3 scripts/train_quad_pg.py                       # 默认 5M 步
  uv run python3 scripts/train_quad_pg.py --num_timesteps=20000000  # 20M
  uv run python3 scripts/train_quad_pg.py --num_envs=512        # OOM 时减半

训练完自动出 mp4（quad_rollout.mp4）。
"""
import os
# JAX 内存配置（必须在 import jax 之前）
os.environ["XLA_PYTHON_CLIENT_PREALLOCATE"] = "false"
os.environ["XLA_PYTHON_CLIENT_MEM_FRACTION"] = "0.40"
os.environ["MUJOCO_GL"] = "egl"
os.environ["XLA_FLAGS"] = os.environ.get("XLA_FLAGS", "") + " --xla_gpu_triton_gemm_any=True"

import sys
import time
import functools
import argparse
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import jax
import jax.numpy as jp
import numpy as np
import mediapy as media

from brax.training.agents.ppo import networks as ppo_networks
from brax.training.agents.ppo import train as ppo

from mujoco_playground import wrapper
from quad_pg_env import QuadJoystick


# ----------------------------------------------------------------------
# CLI 参数
# ----------------------------------------------------------------------
def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--num_timesteps", type=int, default=5_000_000)
    p.add_argument("--num_envs", type=int, default=1024)
    p.add_argument("--num_evals", type=int, default=5)
    p.add_argument("--episode_length", type=int, default=1000)
    p.add_argument("--unroll_length", type=int, default=20,
                   help="PPO unroll 长度（Go1=20，复杂 env 可降到 5/10）")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument(
        "--ckpt_dir",
        type=str,
        default=f"logs/quad_pg/{datetime.now().strftime('%Y%m%d-%H%M%S')}",
    )
    p.add_argument(
        "--no_render",
        action="store_true",
        help="训练后不渲染 mp4（节省时间）",
    )
    return p.parse_args()


# ----------------------------------------------------------------------
# 进度回调（每个 eval 打印一次）
# ----------------------------------------------------------------------
times: list[float] = []


def make_progress_fn(num_evals, num_timesteps):
    def progress(step, metrics):
        if step == 0:
            times.append(time.time())
        else:
            times.append(time.time())
        elapsed = times[-1] - times[0] if len(times) > 1 else 0.0
        # metrics 里有 episode 平均 reward 等
        ep_reward = metrics.get("eval/episode_reward", None)
        ep_len = metrics.get("eval/episode_length", None)
        reward_str = f"{float(ep_reward):.3f}" if ep_reward is not None else "?"
        len_str = f"{float(ep_len):.0f}" if ep_len is not None else "?"
        print(
            f"  step {step:>10,d} / {num_timesteps:,d}  "
            f"({100*step/num_timesteps:5.1f}%)  "
            f"reward={reward_str}  episode_length={len_str}  "
            f"elapsed={elapsed:.0f}s"
        )
    return progress


# ----------------------------------------------------------------------
# 主训练流程
# ----------------------------------------------------------------------
def main():
    args = parse_args()
    print("=" * 70)
    print("四足舵机 Joystick 训练（Playground + Brax PPO）")
    print("=" * 70)
    print(f"  num_timesteps  = {args.num_timesteps:,}")
    print(f"  num_envs       = {args.num_envs}")
    print(f"  num_evals      = {args.num_evals}")
    print(f"  episode_length = {args.episode_length}")
    print(f"  ckpt_dir       = {args.ckpt_dir}")
    print(f"  devices        = {jax.devices()}")
    print()

    # ----- 1. 创建 env（裸 env，ppo.train 内部 wrap） -----
    print("[1/3] 创建环境...")
    env = QuadJoystick()
    print(f"  action_size = {env.action_size}")
    print(f"  obs keys = {list(jax.eval_shape(env.reset, jax.random.PRNGKey(0)).obs.keys())}")

    # ----- 2. 构建 train_fn（照搬 Go1 PPO 超参） -----
    print("\n[2/3] 构建 PPO train_fn...")
    # PPO 超参（完全照搬 Go1JoystickFlatTerrain）
    ppo_params = dict(
        num_timesteps=args.num_timesteps,
        episode_length=args.episode_length,
        num_envs=args.num_envs,
        num_evals=args.num_evals,
        reward_scaling=1.0,
        action_repeat=1,
        unroll_length=args.unroll_length,
        num_minibatches=32,
        num_updates_per_batch=4,
        discounting=0.97,
        learning_rate=3e-4,
        entropy_cost=1e-2,
        batch_size=256,
        max_grad_norm=1.0,
        num_resets_per_eval=10,
        normalize_observations=True,
        clipping_epsilon=0.3,
        seed=args.seed,
    )
    # 网络（Go1 同款，512-256-128）
    network_factory = functools.partial(
        ppo_networks.make_ppo_networks,
        policy_hidden_layer_sizes=(512, 256, 128),
        value_hidden_layer_sizes=(512, 256, 128),
        policy_obs_key="state",
        value_obs_key="privileged_state",
    )

    train_fn = functools.partial(
        ppo.train,
        **ppo_params,
        network_factory=network_factory,
        wrap_env_fn=wrapper.wrap_for_brax_training,
        save_checkpoint_path=args.ckpt_dir,
        restore_checkpoint_path=None,
    )
    print(f"  PPO params = {ppo_params}")

    # ----- 3. 训练 -----
    print("\n[3/3] 开始训练（JIT 编译需 1-2 分钟）...")
    progress = make_progress_fn(args.num_evals, args.num_timesteps)
    t0 = time.time()
    make_inference_fn, params, metrics = train_fn(
        environment=env,
        progress_fn=progress,
        policy_params_fn=lambda *a: None,
    )
    train_time = time.time() - t0
    print(f"\n✅ 训练完成！用时 {train_time:.0f}s ({train_time/60:.1f}min)")
    print(f"  最终 eval reward = {float(metrics.get('eval/episode_reward', 0)):.3f}")

    # ----- 4. 渲染 rollout mp4 -----
    if not args.no_render:
        print("\n[4/4] 渲染 rollout mp4...")
        try:
            render_rollout(env, make_inference_fn, params, args)
        except Exception as e:
            print(f"  ⚠️ 渲染失败: {e}")
            import traceback
            traceback.print_exc()

    print(f"\ncheckpoint: {args.ckpt_dir}")
    print("done.")


# ----------------------------------------------------------------------
# 渲染一个 episode 并存 mp4
# ----------------------------------------------------------------------
def render_rollout(env, make_inference_fn, params, args, length=None):
    """加载训练好的策略，跑 1 个 episode，渲染成 mp4。"""
    n_steps = length or args.episode_length

    # 推理函数（deterministic）
    inference_fn = make_inference_fn(params, deterministic=True)
    jit_inference_fn = jax.jit(inference_fn)

    # 包装 env（同训练时的 wrap）
    infer_env = QuadJoystick()  # 新建避免状态污染
    wrapped_env = wrapper.wrap_for_brax_training(
        infer_env,
        episode_length=args.episode_length,
        action_repeat=1,
    )

    rng = jax.random.PRNGKey(args.seed + 100)
    print("  JIT 编译 reset...")
    reset_state = jax.jit(wrapped_env.reset)(rng)

    # 构造只含 qpos/qvel/time/ctrl 的轻量轨迹（env.render 只需要这些）
    # 参考 train_jax_ppo.py 的做法
    empty_data = reset_state.data.__class__(
        **{k: None for k in reset_state.data.__annotations__}
    )
    empty_traj = reset_state.__class__(
        **{k: None for k in reset_state.__annotations__}
    )
    empty_traj = empty_traj.replace(data=empty_data)

    def step(carry, _):
        state, rng = carry
        rng, act_rng = jax.random.split(rng)
        act, _ = jit_inference_fn(state.obs, act_rng)
        state = wrapped_env.step(state, act)
        traj_data = empty_traj.tree_replace({
            "data.qpos": state.data.qpos,
            "data.qvel": state.data.qvel,
            "data.time": state.data.time,
            "data.ctrl": state.data.ctrl,
            "data.xpos": state.data.xpos,
            "data.xquat": state.data.xquat,
        })
        return (state, rng), traj_data

    print(f"  JIT 编译 rollout（{n_steps} 步）...")
    (_, _), traj = jax.lax.scan(step, (reset_state, rng), None, length=n_steps)
    print(f"  episode reward = {float(traj.reward.sum()):.2f}")
    print(f"  episode length = {n_steps}")

    # 转成 list of State，env.render 接受 List[State]
    states_list = [
        jax.tree_util.tree_map(lambda x, i=i: x[i], traj) for i in range(n_steps)
    ]

    print("  渲染帧...")
    frames = env.render(states_list, height=240, width=320)
    frames = [np.array(f) for f in frames if f is not None]

    out_path = "quad_rollout.mp4"
    if frames:
        render_every = 2
        fps = 1.0 / env.dt / render_every
        media.write_video(out_path, frames[::render_every], fps=fps)
        print(f"  ✅ 已保存 {out_path} ({len(frames[::render_every])} 帧, {fps:.0f} fps)")
    else:
        print("  ⚠️ 无帧可渲染")


if __name__ == "__main__":
    main()
