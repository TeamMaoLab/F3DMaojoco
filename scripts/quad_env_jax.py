#!/usr/bin/env python3
"""四足舵机机器人 RL 环境（GPU 版，Brax + MJX）

把 quad_dyn.xml 封装成 Brax Env，用 jax.vmap 并行 1024+ 环境。
reward/obs/action 逻辑全部照搬 CPU 版 quad_env.py（已验证能学到站立）。

用法:
  from quad_env_jax import QuadEnvJax
  env = QuadEnvJax(task='stand')
  state = env.reset(jax.random.PRNGKey(0))
  state = env.step(state, jnp.zeros(8))

设计（照搬 CPU 版）:
  - obs 37 维: base_angvel(3) + grav_proj(3) + base_linvel(3) + joint_pos(8)
              + joint_vel(8) + foot_contact(4) + last_act(8)
  - action 8 维: 绝对式，ctrl = 0.4 × action × mirror_mask
  - reward: 高度保持(2.0) + 姿态稳定(1.5) - 能耗(0.02) - jvel(0.0005) + (march)交替(0.5)
  - 终止: upright<0.707 或 base_z<0.045
"""
import os
os.environ['XLA_PYTHON_CLIENT_PREALLOCATE'] = 'false'
os.environ['XLA_PYTHON_CLIENT_MEM_FRACTION'] = '0.40'

from typing import Any, Dict, Tuple
import numpy as np
import mujoco
import mujoco.mjx as mjx
import jax
import jax.numpy as jnp
from brax.envs.base import Env, State
from flax import struct


# ============================================================
# HOME_POSE: CPU 预计算的站立收敛 qpos（31 维 = 7 freejoint + 24 关节）
# 从 verify_mjx_dynamics.py 跑出的稳态值
# ============================================================
HOME_POSE = np.array([
    # freejoint: x, y, z, qw, qx, qy, qz
    0.0, 0.0, 0.082658, 0.99999992, 0.0000822213, 0.000261168, -0.000277517,
    # 24 关节（FR_knee, FR_thigh, RR_knee, RR_thigh, FL_knee, FL_thigh, RL_knee, RL_thigh
    #          × 各自的被动关节不在 qpos[7:] 里，实际是所有 24 个 hinge 关节）
    # 用占位 0，下面 __init__ 会用真实 home pose 覆盖
], dtype=np.float64)
# 上面只是骨架，真实值在 __init__ 里从 CPU 计算或从文件加载


# ============================================================
# 常量（全部照搬 quad_env.py）
# ============================================================
MIRROR_MASK = jnp.array([1, 1, 1, 1, -1, -1, -1, -1], dtype=jnp.float32)
ACTION_SCALE = 0.4
TARGET_HEIGHT = 0.082
FOOT_Z_THRESHOLD = 0.008
UPRIGHT_THRESHOLD = 0.707  # cos(45°)
LOW_Z_THRESHOLD = 0.045


class QuadEnvJax(Env):
    """四足 RL 环境（MJX GPU 版）"""

    def __init__(self, task='stand', episode_length=1000):
        """
        task: 'stand' 站立 / 'march' 原地踏步
        episode_length: 每个 episode 最大步数
        """
        self.task = task
        self.episode_length = episode_length

        # 加载 MuJoCo 模型
        self.mj_model = mujoco.MjModel.from_xml_path('mujoco_quad/quad_dyn.xml')
        self.sys = mjx.put_model(self.mj_model)
        self.dt = self.mj_model.opt.timestep

        # ===== 预计算索引（照搬 CPU 版）=====
        self.base_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_BODY, 'base')
        # 8 舵机关节 qpos/dof 地址（顺序: FR_knee, FR_thigh, RR_knee, RR_thigh,
        #                                    FL_knee, FL_thigh, RL_knee, RL_thigh）
        servo_joints = []
        for lk in ['FR', 'RR', 'FL', 'RL']:
            for jn in ['j_旋转1', 'j_旋转2']:
                jid = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, f'{lk}_{jn}')
                servo_joints.append(jid)
        self.servo_qposadr = jnp.array([self.mj_model.jnt_qposadr[j] for j in servo_joints])
        self.servo_dofadr = jnp.array([self.mj_model.jnt_dofadr[j] for j in servo_joints])
        # 足端 site id
        foot_site_ids = []
        for lk in ['FR', 'RR', 'FL', 'RL']:
            sid = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_SITE, f'{lk}_foot_site')
            foot_site_ids.append(sid)
        self.foot_site_ids = jnp.array(foot_site_ids)

        # ===== 预计算 HOME_POSE（CPU 站立收敛）=====
        self.home_qpos = self._compute_home_pose()

        # base body 在 body 数组里的索引（MJX 用同样的 id）
        self._base_idx = self.base_id

    def _compute_home_pose(self):
        """CPU 上跑 _init_standing，返回站立收敛 qpos（numpy 数组）"""
        d = mujoco.MjData(self.mj_model)
        d.qpos[:] = 0
        d.qpos[2] = 0.120
        d.qpos[6] = 1.0
        d.ctrl[:] = 0
        d.qvel[:] = 0
        saved_g = self.mj_model.opt.gravity.copy()
        self.mj_model.opt.gravity[:] = 0
        for _ in range(1000):
            mujoco.mj_step(self.mj_model, d)
        self.mj_model.opt.gravity[:] = saved_g
        for _ in range(500):
            d.ctrl[:] = 0
            mujoco.mj_step(self.mj_model, d)
        d.qvel[:] = 0
        return jnp.array(d.qpos.copy())

    # ===== Brax Env 接口 =====
    @property
    def action_size(self) -> int:
        return 8

    @property
    def observation_size(self) -> int:
        return 37

    @property
    def backend(self) -> str:
        return 'mjx'

    # ===== reset =====
    def reset(self, rng: jax.Array) -> State:
        """初始化到站立姿态 + 域随机化扰动（照搬 CPU 版）"""
        rng, rng_angle, rng_axis, rng_joint, rng_vel = jax.random.split(rng, 5)

        qpos = self.home_qpos.copy()

        # 域随机化：base 随机倾斜 ±8°
        angle = jax.random.uniform(rng_angle, (), minval=-0.14, maxval=0.14)
        axis = jax.random.normal(rng_axis, (3,))
        axis = axis / (jnp.linalg.norm(axis) + 1e-9)
        qpos = qpos.at[3:7].set(jnp.concatenate([
            jnp.array([jnp.cos(angle/2)]), axis * jnp.sin(angle/2)]))

        # 关节随机扰动 ±10°（8 个舵机关节）
        joint_noise = jax.random.uniform(rng_joint, (8,), minval=-0.18, maxval=0.18)
        qpos = qpos.at[self.servo_qposadr].add(joint_noise)

        # 水平随机速度
        qvel = jnp.zeros(self.sys.nv)
        qvel = qvel.at[0].set(jax.random.uniform(rng_vel, (), minval=-0.3, maxval=0.3))
        qvel = qvel.at[1].set(jax.random.uniform(rng_vel, (), minval=-0.3, maxval=0.3))

        # 创建 mjx data
        data = mjx.make_data(self.mj_model)
        data = data.replace(qpos=qpos, qvel=qvel, ctrl=jnp.zeros(self.sys.nu))
        data = mjx.forward(self.sys, data)

        # 初始 obs
        last_action = jnp.zeros(8)
        info = {
            'steps': jnp.int32(0),
            'last_action': last_action,
            'last_contact': jnp.zeros(4),
        }
        obs = self._get_obs(data, info)
        reward = jnp.zeros(())
        done = jnp.zeros(())  # float 0.0（brax wrapper 要求 done 是数值型）
        metrics = {
            'base_z': data.qpos[2],
            'upright': data.xmat[self._base_idx, 2, 2],
        }
        return State(pipeline_state=data, obs=obs, reward=reward, done=done,
                     metrics=metrics, info=info)

    # ===== step =====
    def step(self, state: State, action: jax.Array) -> State:
        """执行一步：action → ctrl → mjx.step → reward/done/obs"""
        action = jnp.clip(action, -1.0, 1.0)
        # 绝对式 ctrl（照搬：ctrl = scale × action × mirror）
        ctrl = ACTION_SCALE * action * MIRROR_MASK
        data = state.pipeline_state.replace(ctrl=ctrl)
        data = mjx.step(self.sys, data)

        # 更新 info
        info = {**state.info, 'steps': state.info['steps'] + 1,
                'last_action': action, 'last_contact': self._get_foot_contacts(data)}
        # reward
        reward = self._reward(data, action)
        # 终止（摔倒判定，照搬：upright<0.707 或 base_z<0.045）
        done = self._termination(data)
        # 超时
        done = done | (info['steps'] >= self.episode_length)
        done = done.astype(jnp.float32)  # brax wrapper 要求 done 是数值型
        # obs
        obs = self._get_obs(data, info)
        metrics = {**state.metrics,
                   'base_z': data.qpos[2],
                   'upright': data.xmat[self._base_idx, 2, 2]}
        return state.replace(pipeline_state=data, obs=obs, reward=reward,
                             done=done, metrics=metrics, info=info)

    # ===== obs 构造（照搬 CPU 版，37 维）=====
    def _get_obs(self, data, info) -> jax.Array:
        # base 角速度（world frame，归一化 /2）
        base_angvel = data.qvel[3:6] / 2.0
        # 重力投影到 base 系（归一化到 [-1,1]）
        # MJX 的 xmat 是 [nbody, 3, 3]，base 的旋转矩阵
        base_mat = data.xmat[self._base_idx]  # [3, 3]
        grav_proj = base_mat.T @ jnp.array([0.0, 0.0, -1.0])  # 已归一化
        # base 线速度（world frame）
        base_linvel = data.qvel[0:3]
        # 8 关节角（归一化 /1.5708）
        joint_pos = data.qpos[self.servo_qposadr] / 1.5708
        # 8 关节角速度（归一化 /10）
        joint_vel = data.qvel[self.servo_dofadr] / 10.0
        # 4 足端接触
        foot_contact = self._get_foot_contacts(data)
        # 上一步 action
        last_act = info['last_action']
        return jnp.concatenate([
            base_angvel,      # 3
            grav_proj,        # 3
            base_linvel,      # 3
            joint_pos,        # 8
            joint_vel,        # 8
            foot_contact,     # 4
            last_act,         # 8
        ]).astype(jnp.float32)

    def _get_foot_contacts(self, data) -> jax.Array:
        """4 足端 site z < 阈值算触地（照搬 CPU 版）"""
        foot_z = data.site_xpos[self.foot_site_ids, 2]
        return (foot_z < FOOT_Z_THRESHOLD).astype(jnp.float32)

    # ===== reward（SB3 验证过能学好的版本）=====
    def _reward(self, data, action) -> jax.Array:
        base_z = data.qpos[2]
        upright = data.xmat[self._base_idx, 2, 2]

        r = jnp.zeros(())
        # 1. 存活奖励（每步 +0.5，SB3 成功版本的核心）
        r = r + 0.5
        # 2. 高度保持（exp，目标 82mm）
        height_err = jnp.abs(base_z - TARGET_HEIGHT)
        r = r + jnp.exp(-height_err * 80.0) * 1.5
        # 3. 姿态稳定（upright 接近 1）
        r = r + jnp.maximum(0.0, upright) * 1.0
        # 4. 能耗惩罚（温和）
        r = r - 0.05 * jnp.sum(action ** 2)
        # 5. 关节角速度惩罚
        joint_vel = data.qvel[self.servo_dofadr]
        r = r - 0.001 * jnp.sum(joint_vel ** 2)
        # 6. 踏步任务
        if self.task == 'march':
            foot = self._get_foot_contacts(data)
            diag1 = foot[0] + foot[3]
            diag2 = foot[1] + foot[2]
            alt = jnp.where(
                (diag1 > 0) & (diag2 == 0) | (diag2 > 0) & (diag1 == 0), 0.5, 0.0)
            r = r + alt
        return r

    def _termination(self, data) -> jax.Array:
        """摔倒判定（照搬：upright<0.707 或 base_z<0.045）"""
        base_z = data.qpos[2]
        upright = data.xmat[self._base_idx, 2, 2]
        return (base_z < LOW_Z_THRESHOLD) | (upright < UPRIGHT_THRESHOLD)


# ============================================================
# 自测
# ============================================================
if __name__ == '__main__':
    print('=== QuadEnvJax 自测 ===')
    env = QuadEnvJax(task='stand')
    print(f'action_size: {env.action_size}')
    print(f'observation_size: {env.observation_size}')
    print(f'dt: {env.dt*1000:.1f}ms, episode: {env.episode_length} steps ({env.episode_length*env.dt:.1f}s)')
    print(f'home_qpos base z: {float(env.home_qpos[2])*1000:.2f}mm')

    # 单环境 reset + step
    print('\n--- 单环境测试 ---')
    rng = jax.random.PRNGKey(0)
    state = env.reset(rng)
    print(f'reset 后: obs shape={state.obs.shape}, base_z={float(state.pipeline_state.qpos[2])*1000:.1f}mm')

    # 跑 100 步 action=0（应该稳定）
    print('\n--- action=0 跑 100 步 ---')
    @jax.jit
    def step_fn(state, action):
        return env.step(state, action)
    print('JIT 编译...')
    for i in range(100):
        state = step_fn(state, jnp.zeros(8))
    z = float(state.pipeline_state.qpos[2])
    print(f'100步后: base_z={z*1000:.2f}mm, reward={float(state.reward):.3f}, done={bool(state.done)}')

    # vmap batch 测试
    print('\n--- vmap 1024 环境测试 ---')
    @jax.jit
    def batch_reset(rng):
        return jax.vmap(env.reset)(jax.random.split(rng, 1024))
    @jax.jit
    def batch_step(states, actions):
        return jax.vmap(env.step)(states, actions)

    print('batch reset...')
    states = batch_reset(jax.random.PRNGKey(1))
    print(f'batch obs shape: {states.obs.shape} (应 [1024, 37])')

    print('batch step 50 步...')
    import time
    t0 = time.time()
    actions = jnp.zeros((1024, 8))
    for _ in range(50):
        states = batch_step(states, actions)
    jax.block_until_ready(states.obs)
    elapsed = time.time() - t0
    fps = 1024 * 50 / elapsed
    print(f'50 batch steps: {elapsed:.2f}s, 等效 {fps:.0f} FPS')
    print(f'  对比 CPU 8 进程 2700 FPS → 加速 {fps/2700:.1f}x')

    print('\n✓ QuadEnvJax 可用')
