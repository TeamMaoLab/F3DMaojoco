# 四足舵机机器人 MuJoCo 模型

从 Fusion 360 设计 + 已验证的单腿运动学（`mujoco_leg/leg.xml`）参数化生成的四足 MuJoCo 模型，可直接用于 RL 训练。

## 文件

| 文件 | 说明 |
|---|---|
| `quad_kin.xml` | **运动学调试版**：固定 base + 零重力 + 硬 equality 闭环。用于验证几何正确性（对齐 `quad_leg_viewer.html`）。 |
| `quad_dyn.xml` | **RL 训练版**：浮动 base (freejoint) + 重力 + 地面 + 足端接触 + 传感器。站立稳定，可直接接入 RL 框架。 |
| `../scripts/gen_quad_mujoco.py` | 模型生成器（从 `leg_kinematic.json` + `parts_world.json` 参数化生成）。 |
| `../scripts/verify_quad_mujoco.py` | 验证脚本（闭环残差 + 4 腿对称性，全部通过）。 |

## 重新生成

模型改了（Fusion 重新导出数据）后，一键重新生成：

```bash
python3 scripts/gen_quad_mujoco.py --mode kin   # 运动学版
python3 scripts/gen_quad_mujoco.py --mode dyn   # RL 版
python3 scripts/verify_quad_mujoco.py           # 验证几何
```

数据源：
- `exports/quad_v4/leg_kinematic.json` — 单腿 6 body 运动学树（joint_anchor_mm）
- `exports/quad_v4/parts_world.json` — 49 零件世界变换 + STL 路径
- `exports/quad_v4/stl_files/` — STL 网格（component 局部坐标，mm）

## 模型结构

### Body 树（40 bodies）

```
world
└── base (freejoint, 6 DOF)                        ← RL 的 root
    ├── chassis_假电池 / chassis_假电路板            ← 固定装饰
    ├── FR_hip (pos=[0,-47,0]mm)                    ← 右前腿根
    │   ├── FR_servo_housing_0/1                   ← 舵机壳（固定）
    │   ├── FR_knee_driver (j_旋转1, 主动)          ← 膝盖舵机
    │   │   └── FR_knee_link1 (j_旋转6) → knee_rotor (j_旋转7) → knee_link2 (j_旋转4)
    │   └── FR_thigh_rigid (j_旋转2, 主动)          ← 大腿舵机
    │       └── FR_shin (j_旋转3) + foot sphere     ← 足端接触
    ├── RR_hip (pos=[115,-47,0]mm)                  ← 右后腿（结构同 FR）
    ├── FL_hip (pos=[0,47,0]mm, quat=0 1 0 0)       ← 左前腿（镜像）
    └── RL_hip (pos=[115,47,0]mm, quat=0 1 0 0)     ← 左后腿（镜像）
```

### 关节（25 joints = 1 freejoint + 24 hinge）

每条腿 6 个 hinge（全 Y 轴，XZ 平面机构）：

| 关节 | 类型 | body | 说明 |
|---|---|---|---|
| `j_旋转1` (t1) | **主动** | knee_driver | 膝盖舵机，range ±90° |
| `j_旋转2` (t2) | **主动** | thigh_rigid | 大腿舵机，range ±90° |
| `j_旋转3` (t3) | 被动 | shin | equality 解 |
| `j_旋转4` (t4) | 被动 | knee_link2 | equality 解 |
| `j_旋转6` (t6) | 被动 | knee_link1 | equality 解 |
| `j_旋转7` (t7) | 被动 | knee_rotor | equality 解 |

### 闭环约束（8 条 equality connect）

每条腿 2 条双闭环（沿用 `leg.xml`）：

```
loop_J2: knee_rotor 上的 J2' anchor ↔ thigh_rigid 原点 (J2)
loop_J5: knee_link2 上的 J5 anchor  ↔ shin 原点
```

4 腿 × 2 = 8 条。镜像腿（FL/RL）的 anchor Z 取负（见"镜像规则"）。

## RL 接入

### 动作空间（8 维）

| 索引 | 名字 | 含义 | 单位 | range |
|---|---|---|---|---|
| ctrl[0] | FR_servo_knee | 右前膝盖舵机 | rad | [-1.57, 1.57] |
| ctrl[1] | FR_servo_thigh | 右前大腿舵机 | rad | [-1.57, 1.57] |
| ctrl[2] | RR_servo_knee | 右后膝盖舵机 | rad | [-1.57, 1.57] |
| ctrl[3] | RR_servo_thigh | 右后大腿舵机 | rad | [-1.57, 1.57] |
| ctrl[4] | FL_servo_knee | 左前膝盖舵机 | rad | [-1.57, 1.57] |
| ctrl[5] | FL_servo_thigh | 左前大腿舵机 | rad | [-1.57, 1.57] |
| ctrl[6] | RL_servo_knee | 左后膝盖舵机 | rad | [-1.57, 1.57] |
| ctrl[7] | RL_servo_thigh | 左后大腿舵机 | rad | [-1.57, 1.57] |

Actuator 是 `<position kp="800" kv="20" forcerange="-30 30">`，舵机力矩上限 ±30 N·m。

### 观测空间（26 维 sensor + base 位姿）

| 类别 | 维度 | 来源 |
|---|---|---|
| base 位姿 | 3+4 | `base_pos`, `base_quat` |
| base 速度 | 3+3 | `base_linvel`, `base_angvel` |
| base 加速度 | 3+3 | `base_linacc`, `base_angacc` |
| 8 关节位置 | 8 | `{leg}_{j_旋转1/2}_pos` |
| 8 关节速度 | 8 | `{leg}_{j_旋转1/2}_vel` |
| 4 足端接触 | 4 | `{leg}_foot_touch` |

典型 RL 观测 = root 线速度(3) + 角速度(3) + 重力投影(3) + 关节角(8) + 关节角速度(8) + 足端接触(4) + 上一步动作(8) = 37 维。

### 镜像腿控制（重要！）

**镜像腿（FL/RL）的舵机指令需要取负**，才能产生与原版腿（FR/RR）左右对称的运动：

```python
# 错误（机器人会扭曲）：
d.ctrl[:] = [t1, t2, t1, t2, t1, t2, t1, t2]  # 4 腿同输入

# 正确（镜像腿取负）：
d.ctrl[:] = [t1, t2,  t1, t2,  -t1, -t2, -t1, -t2]
#             FR         RR          FL          RL
```

原因：镜像腿的 hip body 带 `quat="0 1 0 0"`（绕 X 转 180°），在翻转坐标系里绕 +Y 转 θ，世界看是绕 -Y 转 θ。要让世界运动对称，必须喂 -θ。详见 `docs/quad_mirror_kinematics.md`。

**RL 实践**：两种处理方式
1. **策略网络输出对称指令，在环境层对镜像腿取负**（推荐）：`action[4:] = -action[4:]`。降低学习难度。
2. **策略网络自己学会不对称**：直接输出 8 维，让网络学习镜像关系。更通用但训练慢。

### sim2real 舵机角度映射

硬件是 **180° 角度控制舵机**，输入范围 0°~180°，初始（站立）状态 = **90°**（中位）。
MuJoCo 关节角 0 = 站立位，范围 [-90°, +90°]，转换关系：

```
舵机输入(度) = MuJoCo关节角(度) + 90
```

| MuJoCo 关节角 | 舵机输入 | 含义 |
|---|---|---|
| -90° | 0° | 舵机一端极限 |
| 0° | 90° | 站立中位 |
| +90° | 180° | 舵机另一端极限 |

```python
# MuJoCo 关节角（rad）→ 硬件舵机输入（度）
def mujoco_to_servo(joint_rad):
    return np.degrees(joint_rad) + 90

# 硬件舵机输入（度）→ MuJoCo 关节角（rad）
def servo_to_mujoco(servo_deg):
    return np.radians(servo_deg - 90)
```

注：joint range 已在 XML 设为 ±90°（`range="-1.5708 1.5708"`），与 180° 舵机硬件限制一致。

### 初始化（站立位）

```python
import mujoco, numpy as np
m = mujoco.MjModel.from_xml_path('mujoco_quad/quad_dyn.xml')
d = mujoco.MjData(m)
d.qpos[:] = 0
d.qpos[2] = 0.090   # base z = 90mm（足端悬空，避免初始穿透）
d.qpos[6] = 1.0     # qw = 1（单位四元数）
d.ctrl[:] = 0       # 8 舵机目标 = 0（站立位）
# 先无重力让 equality 收敛，再加重力
m.opt.gravity[:] = 0
for _ in range(2000):
    mujoco.mj_step(m, d)
m.opt.gravity[:] = [0, 0, -9.81]
d.qvel[:] = 0
# 现在机器人稳定站立在 base z ≈ 82mm
```

## 关键参数

| 参数 | kin 版 | dyn 版 | 说明 |
|---|---|---|---|
| 重力 | 0 | -9.81 m/s² | dyn 启用 |
| base | 固定 | freejoint (6 DOF) | dyn 浮动 |
| integrator | Euler | implicitfast | dyn 闭环稳定 |
| 关节阻尼 | 0.5 | 2.0 | dyn 抗震 |
| equality solref | 0.001 1 (硬) | 0.01 1 (软) | dyn 减少预应力 |
| PD 增益 kp/kv | 800/20 | 80/5 | dyn 适配轻机器人（避免弹飞，见下） |
| 舵机力矩上限 | ±30 N·m | ±5 N·m | dyn 适配轻机器人 |
| 地面 | 无 | plane (friction 0.8) | dyn 足端接触 |
| 足端 | 无 | sphere r=4mm | dyn 接触点 |
| 机身质量 | - | 0.070 kg | 实测：打印件30g+电池电路板40g |
| 总质量 | - | 0.254 kg | 实测 ~242g（打印90g+舵机112g+电路40g） |

## 验证结果（`verify_quad_mujoco.py`）

```
测试 1：初始位形 4 腿闭环残差           ✓ 0.000mm
测试 2：4 腿 hip 世界坐标精确性          ✓ 0.000mm
测试 3：网格扫描 4 腿闭环闭合率（49点）  ✓ 100%
测试 4：足端几何对称性 FR↔FL, RR↔RL     ✓ 0.000mm
测试 5：镜像腿被动关节角取负关系         ✓ FR+FL=0.000
```

所有验证误差 = 0.000mm，模型几何与 `quad_leg_viewer.html` 完全一致。

## 已知限制 / 后续工作

1. **惯量估算**：diaginertia 用 mass×特征长度² 粗估，非真实值。可从 Fusion physicalProperties 补导出精确惯量。
2. **dyn 版站立姿态不完美**：4 足端接触力分布不均，因为足端位置和惯量精度有限。RL 训练时策略网络会自适应。
3. **腿内部 collider 仅 kin 版有**：dyn 版用 thigh_ground/shin_ground capsule（沿 J2→J3→足端）做地面碰撞，腿内部防过屈靠 joint range（±90°）。
4. **MuJoCo 闭环动力学**：8 条 equality + freejoint 在动力学下有轻微震荡（qvel ~10），用 implicitfast + 软 solref + 关节阻尼已大幅改善。

## 相关文件

- `mujoco_leg/leg.xml` — 单腿 MuJoCo 基准（已验证，闭环定义的权威来源）
- `scripts/fk_newton.py` — 单腿 FK 牛顿迭代求解器（纯运动学）
- `WebPreviewer/quad_leg_viewer.html` — 4 腿 LUT 分控 viewer（运动学正确性基准）
- `docs/quad_mirror_kinematics.md` — 镜像腿运动学推导（3 个坑全记录）
- `exports/quad_v4/` — 四足数据（leg_kinematic.json + parts_world.json + STL）
