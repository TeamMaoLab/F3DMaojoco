#!/usr/bin/env python3
"""四足舵机机器人 RL 训练环境（Gymnasium + MuJoCo）

把 quad_dyn.xml 封装成 Gym 环境，用 PPO 训练站立/踏步平衡。
镜像腿 action 在环境层自动取负（FL/RL），降低策略学习难度。

用法:
  from quad_env import QuadEnv
  env = QuadEnv(task='stand')   # 'stand' 站立平衡 / 'march' 原地踏步

设计要点:
  - obs 37 维: base 角速度(3) + 重力投影(3) + 关节角(8) + 关节角速度(8)
              + 足端接触(4) + 上步action(8) + base线速度(3)
  - action 8 维: 8 舵机目标角增量（-1~1，映射到 ±action_scale rad）
  - reward: 存活 + 高度保持 + 姿态稳定 + 能耗惩罚 + (踏步任务)足端交替
  - 终止条件: 摔倒（base z < 50mm 或 pitch/roll > 45°）或超时
"""
import numpy as np
import mujoco
import gymnasium as gym
from gymnasium import spaces


class QuadEnv(gym.Env):
    """四足 RL 环境（单进程 MuJoCo 仿真）"""

    metadata = {'render_modes': ['human']}

    def __init__(self, task='stand', max_steps=1000, action_scale=0.4,
                 render_mode=None):
        """
        task: 'stand' 站立平衡 / 'march' 原地踏步（加足端交替奖励）
        max_steps: 每个 episode 最大步数（@5ms = 5s）
        action_scale: action[-1,1] 每步 ctrl 增量幅度（rad），0.05≈2.9°/步
        """
        super().__init__()
        self.task = task
        self.max_steps = max_steps
        self.action_scale = action_scale
        self.render_mode = render_mode

        # MuJoCo 模型
        self.model = mujoco.MjModel.from_xml_path('mujoco_quad/quad_dyn.xml')
        self.data = mujoco.MjData(self.model)
        # 缩短 episode 时间步（5ms × 1000 = 5s 每 episode）
        self.dt = self.model.opt.timestep

        # ===== 预计算索引（加速 step）=====
        # base body id
        self.base_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'base')
        # 8 舵机关节 qpos/qvel 地址（顺序: FR_knee, FR_thigh, RR_knee, RR_thigh,
        #                                    FL_knee, FL_thigh, RL_knee, RL_thigh）
        self.servo_joints = []
        for lk in ['FR', 'RR', 'FL', 'RL']:
            for jn in ['j_旋转1', 'j_旋转2']:
                jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f'{lk}_{jn}')
                self.servo_joints.append(jid)
        self.servo_qposadr = np.array([self.model.jnt_qposadr[j] for j in self.servo_joints])
        self.servo_dofadr = np.array([self.model.jnt_dofadr[j] for j in self.servo_joints])
        # 镜像腿标志（FL/RL 的 action 取负）
        self.mirror_mask = np.array([1, 1, 1, 1, -1, -1, -1, -1], dtype=np.float32)
        # 足端 site id（4 个）
        self.foot_site_ids = []
        for lk in ['FR', 'RR', 'FL', 'RL']:
            sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, f'{lk}_foot_site')
            self.foot_site_ids.append(sid)
        # 期望站立高度（base z）
        self.target_height = 0.082  # 82mm

        # ===== Gym 空间 =====
        # obs 37 维
        obs_dim = 37
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32)
        # action 8 维（-1~1）
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(8,), dtype=np.float32)

        # 状态
        self.step_count = 0
        self.last_action = np.zeros(8, dtype=np.float32)
        self.viewer = None

        # episode 统计
        self.episode_reward = 0.0
        self.episode_len = 0

    def _init_standing(self):
        """初始化到站立姿态（无重力 settle 后加重力下落）"""
        self.data.qpos[:] = 0
        self.data.qpos[2] = 0.120   # base z 抬高（足端悬空）
        self.data.qpos[6] = 1.0     # qw = 1（单位四元数）
        self.data.ctrl[:] = 0
        self.data.qvel[:] = 0
        # 无重力 settle（equality 收敛）
        saved_g = self.model.opt.gravity.copy()
        self.model.opt.gravity[:] = 0
        for _ in range(1000):
            mujoco.mj_step(self.model, self.data)
        # 恢复重力，下落到站立稳态
        self.model.opt.gravity[:] = saved_g
        for _ in range(500):
            self.data.ctrl[:] = 0
            mujoco.mj_step(self.model, self.data)
        self.data.qvel[:] = 0

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self._init_standing()
        # domain randomization：初始姿态扰动（让策略有事可做——学会恢复平衡）
        if options is None or options.get('deterministic', False) is False:
            # base 随机倾斜 ±8°（需要策略主动纠偏才能站稳）
            angle = np.random.uniform(-0.14, 0.14)  # ±8°
            axis = np.random.randn(3)
            axis /= np.linalg.norm(axis) + 1e-9
            self.data.qpos[3:7] = np.concatenate([
                [np.cos(angle/2)], axis * np.sin(angle/2)])
            # 关节角随机扰动 ±10°（腿被打乱，需要恢复）
            for adr in self.servo_qposadr:
                self.data.qpos[adr] += np.random.uniform(-0.18, 0.18)
            # base 水平随机速度（推一下）
            self.data.qvel[0] = np.random.uniform(-0.3, 0.3)
            self.data.qvel[1] = np.random.uniform(-0.3, 0.3)
        self.step_count = 0
        self.last_action = np.zeros(8, dtype=np.float32)
        self._prev_action = None  # action 平滑惩罚用
        self.episode_reward = 0.0
        self.episode_len = 0
        return self._get_obs(), self._get_info()

    def _get_obs(self):
        """构造 37 维观测"""
        # base 角速度（3）
        base_angvel = self.data.qvel[3:6].copy()
        # base 线速度（3）—— 用 qvel 前 3 个（world frame）
        base_linvel = self.data.qvel[0:3].copy()
        # 重力投影到 base 系（3）—— 反映机身倾斜
        # 重力 [0,0,-9.81] 在 base 系下的表示 = R^T · g
        base_mat = self.data.xmat[self.base_id].reshape(3, 3)
        grav_proj = base_mat.T @ np.array([0, 0, -9.81])
        grav_proj = grav_proj / 9.81  # 归一化到 [-1,1]
        # 8 关节角（归一化到 [-1,1]，除以 90°=1.57）
        joint_pos = self.data.qpos[self.servo_qposadr].copy() / 1.5708
        # 8 关节角速度（归一化）
        joint_vel = self.data.qvel[self.servo_dofadr].copy() / 10.0
        # 4 足端接触（0 或 1）
        foot_contact = self._get_foot_contacts()
        # 上一步 action
        last_act = self.last_action.copy()

        obs = np.concatenate([
            base_angvel / 2.0,        # 3
            grav_proj,                # 3
            base_linvel / 1.0,        # 3
            joint_pos,                # 8
            joint_vel,                # 8
            foot_contact,             # 4
            last_act,                 # 8
        ]).astype(np.float32)
        return obs

    def _get_foot_contacts(self):
        """4 足端是否触地（基于接触力阈值）"""
        contacts = np.zeros(4, dtype=np.float32)
        mujoco.mj_forward(self.model, self.data)
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            g1 = c.geom1
            g2 = c.geom2
            # 检查是否是足端↔地面
            for k, sid in enumerate(self.foot_site_ids):
                foot_geom = self.model.site_bodyid[sid]
                # 足端 geom 在 shin body 上，名为 {leg}_foot
                # 简化：用接触力大小判断
                pass
        # 简化：用足端 site 的 z 坐标判断（< 5mm 算触地）
        for k, sid in enumerate(self.foot_site_ids):
            fz = self.data.site_xpos[sid][2]
            contacts[k] = 1.0 if fz < 0.008 else 0.0
        return contacts

    def _get_info(self):
        return {
            'base_z': self.data.qpos[2],
            'step': self.step_count,
        }

    def step(self, action):
        action = np.clip(action, -1.0, 1.0).astype(np.float64)
        # 绝对式 action：ctrl = action_scale × action × mirror
        # 站立位 ctrl=0，策略输出 0 = 保持站立
        # 纠偏靠策略输出小的非零值响应扰动
        target_rad = self.action_scale * action * self.mirror_mask
        self.data.ctrl[:] = target_rad
        self.last_action = action.astype(np.float32)

        # 仿真一步
        mujoco.mj_step(self.model, self.data)
        self.step_count += 1
        self.episode_len += 1

        # ===== 计算 reward =====
        obs = self._get_obs()
        reward = self._compute_reward()
        self.episode_reward += reward

        # ===== 终止条件 =====
        terminated, trunc_reason = self._check_termination()
        truncated = self.step_count >= self.max_steps
        if truncated:
            trunc_reason = 'timeout'

        info = self._get_info()
        if terminated or truncated:
            info['episode'] = {
                'r': self.episode_reward,
                'l': self.episode_len,
                'trunc': trunc_reason,
            }

        return obs, reward, terminated, truncated, info

    def _compute_reward(self):
        """reward 设计（站立平衡，抵抗扰动恢复）。

        核心逻辑：机器人被扰动后，奖励它恢复到站立姿态（高度准+不倾斜）。
        温和惩罚 action（鼓励小动作，但不禁止纠偏）。
        """
        base_z = self.data.qpos[2]
        base_mat = self.data.xmat[self.base_id].reshape(3, 3)
        upright = base_mat[2, 2]

        r = 0.0
        # 1. 高度保持（接近 82mm）—— 主奖励，用线性 reward（梯度更清晰）
        height_err = abs(base_z - self.target_height)
        # 线性：z=82mm 时 1.0，z=70mm 时 0.5，z=60mm 时 0
        r += max(0, 1.0 - height_err / 0.025) * 2.0

        # 2. 姿态稳定（upright 接近 1）
        r += max(0, upright) * 1.5

        # 3. 温和能耗惩罚（绝对式：鼓励小 action，但允许纠偏）
        r -= 0.02 * np.sum(self.last_action ** 2)

        # 4. 关节角速度惩罚（防甩）
        joint_vel = self.data.qvel[self.servo_dofadr]
        r -= 0.0005 * np.sum(joint_vel ** 2)

        # 5. 踏步任务
        if self.task == 'march':
            foot = self._get_foot_contacts()
            diag1 = foot[0] + foot[3]
            diag2 = foot[1] + foot[2]
            if (diag1 > 0 and diag2 == 0) or (diag2 > 0 and diag1 == 0):
                r += 0.5

        return float(r)

    def _check_termination(self):
        """摔倒判定：用 base Z 轴朝向（避免欧拉角 gimbal lock 歧义）"""
        base_z = self.data.qpos[2]
        # base 局部 Z 轴在世界系的方向（xmat 第 3 列）
        base_mat = self.data.xmat[self.base_id].reshape(3, 3)
        upright = base_mat[2, 2]  # base Z 轴在世界 Z 方向的分量 = cos(tilt)
        # upright < cos(45°) 表示倾斜超过 45°
        if base_z < 0.045:
            return True, 'fallen_low'
        if upright < 0.707:
            return True, 'fallen_tilt'
        if not np.all(np.isfinite(self.data.qpos)):
            return True, 'nan'
        return False, ''

    def render(self):
        if self.render_mode == 'human':
            if self.viewer is None:
                import mujoco.viewer
                self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.viewer.sync()

    def close(self):
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None


# ============================================================
# 快速自测
# ============================================================
if __name__ == '__main__':
    print('=== QuadEnv 自测 ===')
    env = QuadEnv(task='stand')
    print(f'obs_space: {env.observation_space.shape}')
    print(f'action_space: {env.action_space.shape}')
    print(f'dt: {env.dt*1000:.1f}ms, max_steps: {env.max_steps} ({env.max_steps*env.dt:.1f}s/episode)')

    obs, info = env.reset()
    print(f'\nreset 后 obs: shape={obs.shape}, range=[{obs.min():.2f}, {obs.max():.2f}]')
    print(f'  base_z={info["base_z"]*1000:.1f}mm')

    # 跑 100 步随机 action，看 reward 和终止
    print('\n=== 随机 action 100 步 ===')
    total_r = 0
    for i in range(100):
        a = env.action_space.sample() * 0.3  # 小幅随机
        obs, r, term, trunc, info = env.step(a)
        total_r += r
        if term or trunc:
            print(f'  step {i}: 终止 term={term} trunc={trunc} info={info.get("episode", info)}')
            break
    print(f'  100步总 reward: {total_r:.2f}, 平均 {total_r/(i+1):.3f}/步')
    print(f'  最终 base_z: {env.data.qpos[2]*1000:.1f}mm')
    print('\n✓ 环境可用' if not term else '⚠ 环境可用但容易摔（正常，随机 action）')
