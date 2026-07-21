"""验证脚本：quad_pg_env 能 reset/step，机器人不摔，obs 形状对。"""
import sys
sys.path.insert(0, "scripts")

import jax
import jax.numpy as jp
from quad_pg_env import QuadJoystick


def main():
    env = QuadJoystick()
    print(f"action_size = {env.action_size}")
    print(f"dt = {env.dt}, sim_dt = {env.sim_dt}, n_substeps = {env.n_substeps}")
    print(f"init_q shape = {env._init_q.shape}")
    print(f"default_pose = {env._default_pose}")

    # jit 加速（mjx 必须 jit 否则每步重新编译巨慢）
    rng = jax.random.PRNGKey(0)
    print("\n[JIT compiling reset + step...]")
    reset_fn = jax.jit(env.reset)
    step_fn = jax.jit(env.step)

    state = reset_fn(rng)
    print(f'obs["state"] shape = {state.obs["state"].shape}  (expect (36,))')
    print(f'obs["privileged_state"] shape = {state.obs["privileged_state"].shape}')
    print(f"initial reward = {state.reward}, done = {state.done}")

    print("\n=== 跑 50 步（action=0，应该保持站立）===")
    base_z0 = float(state.data.qpos[2])
    print(f"step 0: base_z = {base_z0*1000:.1f} mm")

    action = jp.zeros(env.action_size)
    for i in range(50):
        state = step_fn(state, action)
        if (i + 1) % 10 == 0:
            base_z = float(state.data.qpos[2])
            # MJX xmat 是 [nbody, 3, 3]，body 的 (2,2) 元素 = z 轴世界分量
            upright = float(state.data.xmat[env._torso_body_id, 2, 2])
            print(
                f"step {i+1}: reward={float(state.reward):+.4f}, "
                f"base_z={base_z*1000:.1f}mm, upright={upright:.3f}, "
                f"done={bool(state.done)}"
            )

    print("\n=== 测 vmap 4 个并行 env（验证 batch 兼容）===")
    vmap_reset = jax.jit(jax.vmap(env.reset))
    vmap_step = jax.jit(jax.vmap(env.step))
    keys = jax.random.split(jax.random.PRNGKey(42), 4)
    states = vmap_reset(keys)
    print(f"vmapped obs['state'] shape = {states.obs['state'].shape}  (expect (4, 36))")
    print(f"vmapped reward shape = {states.reward.shape}  (expect (4,))")
    actions = jp.zeros((4, env.action_size))
    states = vmap_step(states, actions)
    print(f"after vmap step: reward = {states.reward}")
    print("\n✅ 验证通过" if not bool(states.done.any()) else "\n⚠️ 有 env done")


if __name__ == "__main__":
    main()
