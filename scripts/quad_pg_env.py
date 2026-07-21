"""Quadruped servo robot Joystick task for MuJoCo Playground.

照搬 Go1JoystickFlatTerrain 的 reward / obs / info 设计（DeepMind 调好的），
适配我们的 8-DOF 舵机四足。

关键设计（"4 连杆求解前置到模型外"）：
- 用 quad_lite.xml（完整 40 body + 0 equality + 24 actuator）
- 8 个主动舵机 actuator（policy 直接控制）
- 16 个被动关节 actuator（kp=2000 锁定到查表值）
- env.step 里：policy 输出 8 舵机角 → jax 查 workspace_lut 得 16 被动角 → 拼 24 维 ctrl
- 这样 MJX 编译只需 17 秒（vs 完整 equality 模型 20+ 分钟）

差异 vs Go1：
- N_SERVO=8（不是 12），default_pose=zeros(8)
- ROOT_BODY='base'（不是 'trunk'），foot site 名是 FR_foot_site 等
- impl='jax'（不用 warp）
- 关掉 perturbation（舵机太弱）
- PD gains 用 XML 里的 kp=80 kv=5（已验证 dyn 站立稳定）
- command 幅度砍 1/5（舵机慢）
"""

from typing import Any, Dict, Optional, Union
from pathlib import Path

import jax
import jax.numpy as jp
from ml_collections import config_dict
import mujoco
from mujoco import mjx
import numpy as np

from mujoco_playground._src import mjx_env
from passive_lut import PassiveLUT


# ----------------------------------------------------------------------
# 路径与常量
# ----------------------------------------------------------------------
XML_PATH = "mujoco_quad/quad_lite.xml"   # 改：用 lite 模型（无 equality）
N_SERVO = 8                              # 8 个主动舵机（policy 输出维度）
N_PASSIVE = 16                           # 16 个被动锁定 actuator
N_CTRL = N_SERVO + N_PASSIVE             # 总 ctrl 维度 = 24

FEET_SITES = ["FR_foot_site", "RR_foot_site", "FL_foot_site", "RL_foot_site"]
FEET_GEOMS = ["FR_foot", "RR_foot", "FL_foot", "RL_foot"]
FEET_POS_SENSOR = ["FR_pos", "RR_pos", "FL_pos", "RL_pos"]
ROOT_BODY = "base"

# IMU sensor 名（同 Go1）
UPVECTOR_SENSOR = "upvector"
GLOBAL_LINVEL_SENSOR = "global_linvel"
GLOBAL_ANGVEL_SENSOR = "global_angvel"
LOCAL_LINVEL_SENSOR = "local_linvel"
ACCELEROMETER_SENSOR = "accelerometer"
GYRO_SENSOR = "gyro"


# ----------------------------------------------------------------------
# 默认配置（照搬 Go1，砍 command 幅度，关 perturbation）
# ----------------------------------------------------------------------
def default_config() -> config_dict.ConfigDict:
  return config_dict.create(
      ctrl_dt=0.02,           # 50Hz 控制
      sim_dt=0.002,           # 500Hz 物理（与 quad_dyn.xml timestep 一致）
      episode_length=1000,
      Kp=80.0,                # 我们 XML 里的 PD（Go1 是 35）
      Kd=5.0,
      action_repeat=1,
      action_scale=0.5,       # 同 Go1
      history_len=1,
      soft_joint_pos_limit_factor=0.95,
      noise_config=config_dict.create(
          level=1.0,
          scales=config_dict.create(
              joint_pos=0.03,
              joint_vel=1.5,
              gyro=0.2,
              gravity=0.05,
              linvel=0.1,
          ),
      ),
      reward_config=config_dict.create(
          scales=config_dict.create(
              # Tracking
              tracking_lin_vel=1.0,
              tracking_ang_vel=0.5,
              # Base
              lin_vel_z=-0.5,
              ang_vel_xy=-0.05,
              orientation=-5.0,
              # Other
              dof_pos_limits=-1.0,
              pose=0.5,
              termination=-1.0,
              stand_still=-1.0,
              # Regularization
              torques=-0.0002,
              action_rate=-0.01,
              energy=-0.001,
              # Feet
              feet_clearance=-2.0,
              feet_height=-0.2,
              feet_slip=-0.1,
              feet_air_time=0.1,
          ),
          tracking_sigma=0.25,
          max_foot_height=0.04,   # 我们足端抬起 4cm（Go1 是 10cm，我们腿短）
      ),
      command_config=config_dict.create(
          # 砍 1/5（舵机慢）：Go1 是 [1.5, 0.8, 1.2]
          a=[0.3, 0.2, 0.8],
          b=[0.9, 0.25, 0.5],
      ),
      impl="jax",              # 不用 warp（equality 在 jax 已验证）
      naconmax=4 * 8192,
      njmax=40,
  )


# ----------------------------------------------------------------------
# QuadJoystick env
# ----------------------------------------------------------------------
class QuadJoystick(mjx_env.MjxEnv):
  """我们的四足舵机，跟 Go1 同款 joystick 任务。

  reward / obs / info 设计完全照搬 Go1JoystickFlatTerrain。
  """

  def __init__(
      self,
      config: config_dict.ConfigDict = default_config(),
      config_overrides: Optional[Dict[str, Union[str, int, list[Any]]]] = None,
  ):
    super().__init__(config, config_overrides)

    # 加载模型
    self._mj_model = mujoco.MjModel.from_xml_path(XML_PATH)
    self._mj_model.opt.timestep = self._config.sim_dt
    self._mj_model.opt.ccd_iterations = 20

    # PD gains：前 8 个舵机用 Kp/Kd（默认 80/5），后 16 个锁定用更大 kp（XML 里已设 2000）
    # 这里只覆盖前 8 个
    self._mj_model.dof_damping[6:6+N_SERVO] = self._config.Kd
    self._mj_model.actuator_gainprm[:N_SERVO, 0] = self._config.Kp
    self._mj_model.actuator_biasprm[:N_SERVO, 1] = -self._config.Kp

    self._mjx_model = mjx.put_model(self._mj_model, impl=self._config.impl)
    self._xml_path = XML_PATH
    self._imu_site_id = self._mj_model.site("imu").id

    # 接触 sensor id（4 足端 × floor_found，XML 里名是 FR_floor_found 等）
    self._feet_floor_found_sensor = [
        self._mj_model.sensor(f"{g.replace('_foot', '')}_floor_found").id
        for g in FEET_GEOMS
    ]

    # jax LUT 查表器（8 舵机角 → 16 被动关节角）
    self._lut = PassiveLUT()
    # jit 化的查表函数（vmap 友好）
    self._lut_lookup = jax.jit(self._lut.lookup)

    self._post_init()

  def _post_init(self) -> None:
    # keyframe home（站立位：base z=90mm + 全 0 关节）
    self._init_q = jp.array(self._mj_model.keyframe("home").qpos)
    # default_pose = qpos[7:7+N_SERVO]（只看 8 主动舵机，站立位全 0）
    self._default_pose = jp.array(
        self._mj_model.keyframe("home").qpos[7 : 7 + N_SERVO]
    )

    # 关节限位（[1:] 跳过 freejoint）
    self._lowers, self._uppers = self.mj_model.jnt_range[1:].T
    self._soft_lowers = self._lowers * self._config.soft_joint_pos_limit_factor
    self._soft_uppers = self._uppers * self._config.soft_joint_pos_limit_factor

    self._torso_body_id = self._mj_model.body(ROOT_BODY).id
    self._torso_mass = self._mj_model.body_subtreemass[self._torso_body_id]

    self._feet_site_id = np.array(
        [self._mj_model.site(name).id for name in FEET_SITES]
    )
    self._floor_geom_id = self._mj_model.geom("floor").id
    self._feet_geom_id = np.array(
        [self._mj_model.geom(name).id for name in FEET_GEOMS]
    )

    # 足端 linvel sensor 的地址映射（feet_slip reward 用）
    # XML 里 sensor 名是 FR_global_linvel 等（不是 FR_foot_global_linvel）
    foot_linvel_sensor_adr = []
    for leg_key in ["FR", "RR", "FL", "RL"]:
      sensor_name = f"{leg_key}_global_linvel"
      sensor_id = self._mj_model.sensor(sensor_name).id
      sensor_adr = self._mj_model.sensor_adr[sensor_id]
      sensor_dim = self._mj_model.sensor_dim[sensor_id]
      foot_linvel_sensor_adr.append(
          list(range(sensor_adr, sensor_adr + sensor_dim))
      )
    self._foot_linvel_sensor_adr = jp.array(foot_linvel_sensor_adr)

    self._cmd_a = jp.array(self._config.command_config.a)
    self._cmd_b = jp.array(self._config.command_config.b)

  # ----------------------------------------------------------------
  # Sensor 读取（照搬 Go1 base.py）
  # ----------------------------------------------------------------
  def get_upvector(self, data: mjx.Data) -> jax.Array:
    return mjx_env.get_sensor_data(self.mj_model, data, UPVECTOR_SENSOR)

  def get_gravity(self, data: mjx.Data) -> jax.Array:
    return data.site_xmat[self._imu_site_id].T @ jp.array([0, 0, -1])

  def get_global_linvel(self, data: mjx.Data) -> jax.Array:
    return mjx_env.get_sensor_data(
        self.mj_model, data, GLOBAL_LINVEL_SENSOR
    )

  def get_global_angvel(self, data: mjx.Data) -> jax.Array:
    return mjx_env.get_sensor_data(
        self.mj_model, data, GLOBAL_ANGVEL_SENSOR
    )

  def get_local_linvel(self, data: mjx.Data) -> jax.Array:
    return mjx_env.get_sensor_data(
        self.mj_model, data, LOCAL_LINVEL_SENSOR
    )

  def get_accelerometer(self, data: mjx.Data) -> jax.Array:
    return mjx_env.get_sensor_data(
        self.mj_model, data, ACCELEROMETER_SENSOR
    )

  def get_gyro(self, data: mjx.Data) -> jax.Array:
    return mjx_env.get_sensor_data(self.mj_model, data, GYRO_SENSOR)

  # ----------------------------------------------------------------
  # Accessors（MjxEnv 抽象方法）
  # ----------------------------------------------------------------
  @property
  def xml_path(self) -> str:
    return self._xml_path

  @property
  def action_size(self) -> int:
    # policy 只输出 8 个舵机角（不是 24 维 ctrl）
    return N_SERVO

  @property
  def mj_model(self) -> mujoco.MjModel:
    return self._mj_model

  @property
  def mjx_model(self) -> mjx.Model:
    return self._mjx_model

  # ----------------------------------------------------------------
  # reset
  # ----------------------------------------------------------------
  def reset(self, rng: jax.Array) -> mjx_env.State:
    qpos = self._init_q
    qvel = jp.zeros(self.mjx_model.nv)

    # x/y 加 ±0.5m 扰动，yaw 随机（同 Go1，增加 reset 多样性）
    rng, key = jax.random.split(rng)
    dxy = jax.random.uniform(key, (2,), minval=-0.5, maxval=0.5)
    qpos = qpos.at[0:2].set(qpos[0:2] + dxy)
    rng, key = jax.random.split(rng)
    yaw = jax.random.uniform(key, (1,), minval=-3.14, maxval=3.14)
    # yaw 转 quat 乘到当前 quat 上
    cy, sy = jp.cos(yaw[0] / 2), jp.sin(yaw[0] / 2)
    yaw_quat = jp.array([cy, 0, 0, sy])
    new_quat = _quat_mul(yaw_quat, qpos[3:7])
    qpos = qpos.at[3:7].set(new_quat)

    # qvel[0:6] 加 ±0.5 扰动
    rng, key = jax.random.split(rng)
    qvel = qvel.at[0:6].set(
        jax.random.uniform(key, (6,), minval=-0.5, maxval=0.5)
    )

    # 站立位 ctrl: 8 舵机=0 + 16 被动查表=0（站立位查表得全 0）
    init_servos = qpos[7 : 7 + N_SERVO]
    init_passive = self._lut_lookup(init_servos)  # (16,)
    init_ctrl = jp.concatenate([init_servos, init_passive])

    data = mjx_env.make_data(
        self.mj_model,
        qpos=qpos,
        qvel=qvel,
        ctrl=init_ctrl,
        impl=self.mjx_model.impl.value,
        naconmax=self._config.naconmax,
        njmax=self._config.njmax,
    )
    data = mjx.forward(self.mjx_model, data)

    # 采样初始 command
    rng, key1, key2 = jax.random.split(rng, 3)
    time_until_next_cmd = jax.random.exponential(key1) * 5.0
    steps_until_next_cmd = jp.round(time_until_next_cmd / self.dt).astype(
        jp.int32
    )
    cmd = jax.random.uniform(
        key2, shape=(3,), minval=-self._cmd_a, maxval=self._cmd_a
    )

    info = {
        "rng": rng,
        "command": cmd,
        "steps_until_next_cmd": steps_until_next_cmd,
        "last_act": jp.zeros(N_SERVO),
        "last_last_act": jp.zeros(N_SERVO),
        "feet_air_time": jp.zeros(4),
        "last_contact": jp.zeros(4, dtype=bool),
        "swing_peak": jp.zeros(4),
    }

    metrics = {}
    for k in self._config.reward_config.scales.keys():
      metrics[f"reward/{k}"] = jp.zeros(())
    metrics["swing_peak"] = jp.zeros(())

    obs = self._get_obs(data, info)
    reward, done = jp.zeros(2)
    return mjx_env.State(data, obs, reward, done, metrics, info)

  # ----------------------------------------------------------------
  # step
  # ----------------------------------------------------------------
  def step(self, state: mjx_env.State, action: jax.Array) -> mjx_env.State:
    # 1. policy 输出 8 舵机目标角（绝对位置 + action_scale）
    servo_targets = self._default_pose + action * self._config.action_scale
    # 2. 查表得 16 被动关节目标角（jax 双线性插值，含镜像取负）
    passive_targets = self._lut_lookup(servo_targets)  # (16,)
    # 3. 拼成 24 维 ctrl（8 舵机 + 16 锁定）
    ctrl = jp.concatenate([servo_targets, passive_targets])
    # 4. 物理仿真
    data = mjx_env.step(
        self.mjx_model, state.data, ctrl, self.n_substeps
    )

    # 4 足接触（floor_found sensor > 0 算触地）
    contact = jp.array([
        data.sensordata[self._mj_model.sensor_adr[sid]] > 0
        for sid in self._feet_floor_found_sensor
    ])
    contact_filt = contact | state.info["last_contact"]
    first_contact = (state.info["feet_air_time"] > 0.0) * contact_filt
    state.info["feet_air_time"] += self.dt
    p_f = data.site_xpos[self._feet_site_id]
    p_fz = p_f[..., -1]
    state.info["swing_peak"] = jp.maximum(state.info["swing_peak"], p_fz)

    obs = self._get_obs(data, state.info)
    done = self._get_termination(data)

    rewards = self._get_reward(
        data, action, state.info, state.metrics, done, first_contact, contact
    )
    rewards = {
        k: v * self._config.reward_config.scales[k] for k, v in rewards.items()
    }
    reward = jp.clip(jp.sum(jp.array(list(rewards.values()))) * self.dt, 0.0, 10000.0)

    state.info["last_last_act"] = state.info["last_act"]
    state.info["last_act"] = action
    state.info["steps_until_next_cmd"] -= 1
    state.info["rng"], key1, key2 = jax.random.split(state.info["rng"], 3)
    state.info["command"] = jp.where(
        state.info["steps_until_next_cmd"] <= 0,
        self.sample_command(key1, state.info["command"]),
        state.info["command"],
    )
    state.info["steps_until_next_cmd"] = jp.where(
        done | (state.info["steps_until_next_cmd"] <= 0),
        jp.round(jax.random.exponential(key2) * 5.0 / self.dt).astype(jp.int32),
        state.info["steps_until_next_cmd"],
    )
    state.info["feet_air_time"] *= ~contact
    state.info["last_contact"] = contact
    state.info["swing_peak"] *= ~contact
    for k, v in rewards.items():
      state.metrics[f"reward/{k}"] = v
    state.metrics["swing_peak"] = jp.mean(state.info["swing_peak"])

    done = done.astype(reward.dtype)
    state = state.replace(data=data, obs=obs, reward=reward, done=done)
    return state

  def sample_command(self, rng: jax.Array, x_k: jax.Array) -> jax.Array:
    """照搬 Go1：随机更新 command（b 是保留概率）。"""
    rng, y_rng, w_rng, z_rng = jax.random.split(rng, 4)
    y_k = jax.random.uniform(
        y_rng, shape=(3,), minval=-self._cmd_a, maxval=self._cmd_a
    )
    z_k = jax.random.bernoulli(z_rng, self._cmd_b, shape=(3,))
    w_k = jax.random.bernoulli(w_rng, 0.5, shape=(3,))
    x_kp1 = x_k - w_k * (x_k - y_k * z_k)
    return x_kp1

  # ----------------------------------------------------------------
  # termination（照搬 Go1：upvector[-1] < 0 即倒地）
  # ----------------------------------------------------------------
  def _get_termination(self, data: mjx.Data) -> jax.Array:
    fall_termination = self.get_upvector(data)[-1] < 0.0
    return fall_termination

  # ----------------------------------------------------------------
  # obs（照搬 Go1，nu=8 → state 是 42 维，privileged 是 ~85）
  # ----------------------------------------------------------------
  def _get_obs(
      self, data: mjx.Data, info: dict[str, Any]
  ) -> Dict[str, jax.Array]:
    gyro = self.get_gyro(data)
    info["rng"], noise_rng = jax.random.split(info["rng"])
    noisy_gyro = self._add_noise(gyro, noise_rng, self._config.noise_config.scales.gyro)

    gravity = self.get_gravity(data)
    info["rng"], noise_rng = jax.random.split(info["rng"])
    noisy_gravity = self._add_noise(gravity, noise_rng, self._config.noise_config.scales.gravity)

    joint_angles = data.qpos[7 : 7 + N_SERVO]  # 只取 8 主动关节
    info["rng"], noise_rng = jax.random.split(info["rng"])
    noisy_joint_angles = self._add_noise(
        joint_angles, noise_rng, self._config.noise_config.scales.joint_pos
    )

    joint_vel = data.qvel[6 : 6 + N_SERVO]
    info["rng"], noise_rng = jax.random.split(info["rng"])
    noisy_joint_vel = self._add_noise(
        joint_vel, noise_rng, self._config.noise_config.scales.joint_vel
    )

    linvel = self.get_local_linvel(data)
    info["rng"], noise_rng = jax.random.split(info["rng"])
    noisy_linvel = self._add_noise(linvel, noise_rng, self._config.noise_config.scales.linvel)

    # state: policy 输入（3+3+3+8+8+8+3 = 42 维；Go1 是 48 因 nu=12）
    state = jp.hstack([
        noisy_linvel,                       # 3
        noisy_gyro,                         # 3
        noisy_gravity,                      # 3
        noisy_joint_angles - self._default_pose,  # 8
        noisy_joint_vel,                    # 8
        info["last_act"],                   # 8
        info["command"],                    # 3
    ])

    # privileged: value 网络输入（额外加 ground-truth + 接触 + 足速）
    accelerometer = self.get_accelerometer(data)
    angvel = self.get_global_angvel(data)
    feet_vel = data.sensordata[self._foot_linvel_sensor_adr].ravel()

    privileged_state = jp.hstack([
        state,
        gyro,                               # 3
        accelerometer,                      # 3
        gravity,                            # 3
        linvel,                             # 3
        angvel,                             # 3
        joint_angles - self._default_pose,  # 8
        joint_vel,                          # 8
        data.actuator_force[:N_SERVO],      # 8（只取舵机力矩）
        info["last_contact"],               # 4
        feet_vel,                           # 12
        info["feet_air_time"],              # 4
    ])

    return {
        "state": state,
        "privileged_state": privileged_state,
    }

  def _add_noise(self, x, noise_rng, scale):
    level = self._config.noise_config.level
    return x + (2 * jax.random.uniform(noise_rng, shape=x.shape) - 1) * level * scale

  # ----------------------------------------------------------------
  # reward（完全照搬 Go1）
  # ----------------------------------------------------------------
  def _get_reward(
      self,
      data: mjx.Data,
      action: jax.Array,
      info: dict[str, Any],
      metrics: dict[str, Any],
      done: jax.Array,
      first_contact: jax.Array,
      contact: jax.Array,
  ) -> dict[str, jax.Array]:
    del metrics
    return {
        "tracking_lin_vel": self._reward_tracking_lin_vel(
            info["command"], self.get_local_linvel(data)
        ),
        "tracking_ang_vel": self._reward_tracking_ang_vel(
            info["command"], self.get_gyro(data)
        ),
        "lin_vel_z": self._cost_lin_vel_z(self.get_global_linvel(data)),
        "ang_vel_xy": self._cost_ang_vel_xy(self.get_global_angvel(data)),
        "orientation": self._cost_orientation(self.get_upvector(data)),
        "stand_still": self._cost_stand_still(info["command"], data.qpos[7 : 7 + N_SERVO]),
        "termination": self._cost_termination(done),
        "pose": self._reward_pose(data.qpos[7 : 7 + N_SERVO]),
        "torques": self._cost_torques(data.actuator_force[:N_SERVO]),
        "action_rate": self._cost_action_rate(
            action, info["last_act"], info["last_last_act"]
        ),
        "energy": self._cost_energy(data.qvel[6:], data.actuator_force[:N_SERVO]),
        "feet_slip": self._cost_feet_slip(data, contact, info),
        "feet_clearance": self._cost_feet_clearance(data),
        "feet_height": self._cost_feet_height(
            info["swing_peak"], first_contact, info
        ),
        "feet_air_time": self._reward_feet_air_time(
            info["feet_air_time"], first_contact, info["command"]
        ),
        "dof_pos_limits": self._cost_joint_pos_limits(
            data.qpos[7 : 7 + N_SERVO]
        ),
    }

  # Tracking rewards
  def _reward_tracking_lin_vel(self, commands, local_vel):
    lin_vel_error = jp.sum(jp.square(commands[:2] - local_vel[:2]))
    return jp.exp(-lin_vel_error / self._config.reward_config.tracking_sigma)

  def _reward_tracking_ang_vel(self, commands, ang_vel):
    ang_vel_error = jp.square(commands[2] - ang_vel[2])
    return jp.exp(-ang_vel_error / self._config.reward_config.tracking_sigma)

  # Base-related
  def _cost_lin_vel_z(self, global_linvel):
    return jp.square(global_linvel[2])

  def _cost_ang_vel_xy(self, global_angvel):
    return jp.sum(jp.square(global_angvel[:2]))

  def _cost_orientation(self, torso_zaxis):
    return jp.sum(jp.square(torso_zaxis[:2]))

  # Energy
  def _cost_torques(self, torques):
    return jp.sqrt(jp.sum(jp.square(torques))) + jp.sum(jp.abs(torques))

  def _cost_energy(self, qvel_full, qfrc_actuator):
    # qvel_full 是 qvel[6:]（含 8 主动 + 16 被动 = 24 维）
    # 只用主动关节那部分（前 8 维），与 actuator_force 对齐
    qvel = qvel_full[: N_SERVO]
    return jp.sum(jp.abs(qvel) * jp.abs(qfrc_actuator))

  def _cost_action_rate(self, act, last_act, last_last_act):
    del last_last_act
    return jp.sum(jp.square(act - last_act))

  # Other
  def _reward_pose(self, qpos):
    weight = jp.array([1.0, 1.0, 0.1] * 4)[: N_SERVO]  # 适配 nu=8
    return jp.exp(-jp.sum(jp.square(qpos - self._default_pose) * weight))

  def _cost_stand_still(self, commands, qpos):
    cmd_norm = jp.linalg.norm(commands)
    return jp.sum(jp.abs(qpos - self._default_pose)) * (cmd_norm < 0.01)

  def _cost_termination(self, done):
    return done

  def _cost_joint_pos_limits(self, qpos):
    out_of_limits = -jp.clip(qpos - self._soft_lowers[: N_SERVO], None, 0.0)
    out_of_limits += jp.clip(qpos - self._soft_uppers[: N_SERVO], 0.0, None)
    return jp.sum(out_of_limits)

  # Feet
  def _cost_feet_slip(self, data, contact, info):
    cmd_norm = jp.linalg.norm(info["command"])
    feet_vel = data.sensordata[self._foot_linvel_sensor_adr]
    vel_xy = feet_vel[..., :2]
    vel_xy_norm_sq = jp.sum(jp.square(vel_xy), axis=-1)
    return jp.sum(vel_xy_norm_sq * contact) * (cmd_norm > 0.01)

  def _cost_feet_clearance(self, data):
    feet_vel = data.sensordata[self._foot_linvel_sensor_adr]
    vel_xy = feet_vel[..., :2]
    vel_norm = jp.sqrt(jp.linalg.norm(vel_xy, axis=-1))
    foot_pos = data.site_xpos[self._feet_site_id]
    foot_z = foot_pos[..., -1]
    delta = jp.abs(foot_z - self._config.reward_config.max_foot_height)
    return jp.sum(delta * vel_norm)

  def _cost_feet_height(self, swing_peak, first_contact, info):
    cmd_norm = jp.linalg.norm(info["command"])
    error = swing_peak / self._config.reward_config.max_foot_height - 1.0
    return jp.sum(jp.square(error) * first_contact) * (cmd_norm > 0.01)

  def _reward_feet_air_time(self, air_time, first_contact, commands):
    cmd_norm = jp.linalg.norm(commands)
    rew_air_time = jp.sum((air_time - 0.1) * first_contact)
    rew_air_time *= cmd_norm > 0.01
    return rew_air_time


# ----------------------------------------------------------------------
# 四元数乘法（reset 的 yaw 扰动用；不依赖 mjx.math 避免版本差异）
# ----------------------------------------------------------------------
def _quat_mul(a, b):
  """a, b = (w, x, y, z)。返回 a ⊗ b。"""
  w1, x1, y1, z1 = a
  w2, x2, y2, z2 = b
  return jp.array([
      w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
      w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
      w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
      w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
  ])
