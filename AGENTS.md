# F3DMaojoco 项目状态（2026-07-18）

让舵机四足机器人从 Fusion 360 设计 → 浏览器 3D 复原 → MuJoCo RL 模型。
当前进度：**MuJoCo Playground + Brax PPO 管线搭好，卡在 PPO 编译阶段**。

## 当前进展（最新在前）

### ⚠️ 进行中：Playground + Brax PPO 训练（2026-07-18）
照搬 Go1JoystickFlatTerrain 的全套设计，用 MuJoCo Playground + Brax PPO 训练我们的四足。**核心方案落地，但 PPO 训练编译未完成**。

**已突破的核心技术：「4 连杆求解前置到模型外」**
- 问题：完整 quad_playground.xml（40 body + 8 equality）在 MJX 上编译爆炸（20+ 分钟，XLA 展不开 equality 雅可比）
- 方案：删 equality，给 16 个被动关节加 position actuator（kp=2000 锁定），env.step 里用 jax 查 workspace_lut 算被动关节目标
- 产物：`mujoco_quad/quad_lite.xml`（完整 40 body + 0 equality + 24 actuator）+ `scripts/passive_lut.py`（jax 双线性插值，0.17s/vmap 256）
- 效果：**mjx.step 编译 17 秒**（vs 完整模型 20+ 分钟，70× 快），env.step 编译 19 秒，站立稳定（base_z=82.9mm）

**已就位但未跑通的训练管线**：
- env 类 `scripts/quad_pg_env.py`（照搬 Go1 joystick.py，含 15 个 reward 项）
- 训练脚本 `scripts/train_quad_pg.py`（Brax PPO + Go1 超参）
- env reset/step/vmap 全部验证通过

**⚠️ 卡住的问题：PPO 训练 100+ 秒没出 step 0**
- 实测数据：jax LUT 查表 0.17s，env.step（含查表+reward）19s，**这些都不慢**
- PPO 内部（loss + Adam 反向传播图）编译 100+ 秒未完成
- Go1 同款代码能跑通，说明我们的 env 触发了什么特殊情况（待定位）
- 嫌疑点：sensor 读取、reward 里某个 jax 操作、或 unroll+vmap+grad 组合
- 下次接续：在 brax.training.agents.ppo.train 里加 timing，定位是哪个阶段卡住

### ✅ 已完成：MuJoCo Playground 环境安装（2026-07-18）
- jax 0.6.0 + cuda12 + brax 0.14.2 + mujoco-mjx 3.10.0 + playground 0.2.0
- Cartpole 50K 步训练验证管线通（reward 309→999，60 秒）
- 参考实现：Go1JoystickFlatTerrain（全套 reward/obs/config，照搬）
- Brax PPO 超参（Go1）：num_envs=8192, unroll=20, lr=3e-4, network=(512,256,128)

### ✅ 已完成：四足 MuJoCo 模型生成（2026-07-17）
从单腿 leg.xml + 四足装配数据，参数化生成完整 MuJoCo 四足模型（RL 可用）：
- **两个版本**：`quad_kin.xml`（固定 base 零重力，几何验证用）+ `quad_dyn.xml`（浮动 base 重力地面，RL 训练用）
- **几何完全正确**：8 条 equality 闭环残差 0.000mm，4 腿足端对称性 0.000mm，镜像腿被动关节取负关系 FR+FL=0.000
- **dyn 版站立稳定**：base z 恒定 82.6mm，qvel→0，4 足端接触，20s 不塌
- **真实质量分配**：实测数据（打印件90g+舵机14g×8+电路电池40g=242g），总重 254g
- **机身+腿碰撞体**：机身box + 大腿/小腿capsule + 足端sphere，倒地防穿地
- **关键技术**：implicitfast 积分器 + 软 equality solref(0.01) + 关节阻尼 2.0 + PD kp=80 解决闭环动力学震荡
- **镜像腿处理**：hip body 加 `quat=0 1 0 0`（绕 X 翻 180°）+ anchor Z 取负；RL 控制时镜像腿舵机指令需取负
- 生成器：`scripts/gen_quad_mujoco.py`，验证：`scripts/verify_quad_mujoco.py`，文档：`mujoco_quad/README.md`

### ✅ 已完成：4 腿分控 LUT 驱动 + 镜像运动学（2026-07-17）
在静态装配基础上加关节运动，4 条腿独立控制：
- **核心突破**：把单腿 LUT（θ1,θ2 → 6 关节角）扩展到 4 腿，每条腿独立 (t1,t2) 状态
- **镜像腿三层处理**（详见 docs/quad_mirror_kinematics.md，每个坑都踩过）：
  - 腿根 Group 加 `Rx(π)`（180°绕X）
  - 关节角取负（`Rx(π)·Ry(θ)·Rx(π)⁻¹ = Ry(-θ)`，数值验证误差 0）
  - joint_anchor 的 Z 取负（不然关节跑到头顶飞天）
- **舵机简化**：固定件直接世界变换挂 scene，不再算 rootRotInv 局部旋转
- **UI**：全局滑块联动 4 腿 + 4 张分控卡片（每腿独立 θ1/θ2）+ 预设动作（站立/对角小步/侧倾/挥手）
- 查看：`http://localhost:8766/WebPreviewer/quad_leg_viewer.html`

### ✅ 已完成：坐标系骨架反推（2026-07-17）
从 parts_world.json（只含叶子零件世界变换）反推出 13 个中间节点（机身根/舵机组/ONE-LEG/大腿部分）的世界变换：
- 方法：中间节点世界变换 = 它所有后代叶子世界位置的众数
- 算出所有 62 节点的局部变换 `L = W_parent⁻¹ · W_self`
- 自顶向下累乘验证：62 节点全部误差 0.000000（数学恒等式）
- 产物：`exports/quad_v4/nodes_world.json`，查看：`quad_frames_viewer.html`

### ✅ 已完成：四足位置对齐 + 导出封装（2026-07-16）
从 Fusion「2dof动力腿 v4」文档导出 49 个零件，在 Three.js 里按世界坐标复原：
- **核心方法**：STL 顶点是 component 局部坐标，加载时用 `occurrence.transform2`（世界累乘变换）做旋转+平移
- 4 条腿正确分布在四角：FR/RR 在 Y=-47，FL/RL 在 Y=+47，前后 X=0/115
- 镜像腿（FL/RL）旋转 `[1,-1,-1]`（180°绕X）正确传递给子树，配合镜像 STL 实现左右对称
- **导出封装**：`/export_assembly` 端点一键产出 parts_world.json + STL（visibility 自动过滤废弃件，COL_ 标碰撞体）
- 查看：`http://localhost:8766/WebPreviewer/quad_stl_viewer.html`

### ✅ 基础设施（可复用）
- **F3DRemoteControl add-in**：HTTP 远程控制 Fusion（/ping /exec /export /model 等），CustomEvent 主线程调度
- **F3DMaojocoWin 导出插件**：STL + BRep 精确网格导出（inf3d/ + common/）
- **单腿 FK 求解器**（scripts/fk_newton.py）：纯运动学牛顿迭代解 4 被动角 + 闭环约束，第一阶段成果，四足可复用

### ⏳ 待做
- RL 训练环境封装（Gym/IsaacGym 接入，reward 设计，镜像腿 action 取负处理）
- 更多步态预设（walk/bound/原地踏步等）
- 关节轴/足端轨迹可视化
- 真实惯量补导出（当前 mass/diaginertia 是估算值）

## 架构全景
```
Fusion 360（Python 3.14 主线程，文档「2dof动力腿 v4」）
├── F3DRemoteControl add-in（常驻，HTTP 远程控制）
│   ├── /exec 注入任意代码到主线程（Fusion API 必须主线程）
│   └── /export_assembly?dir=... 一键导出（STL+世界变换，occurrence-centric）
│
├── scripts/dump_frames_stable.py   ← 用 /exec 注入，调试用：导出全树世界变换
│
└── exports/quad_v4/                ← 当前数据（四足）
    ├── parts_world.json            ← 49 个零件 STL + 世界变换（网页摆放用，含 is_collider）
    └── stl_files/                  ← 49 个 STL（component 局部坐标）

WSL / 浏览器
└── curl http://127.0.0.1:9099/...  ← 远程控制 Fusion
└── WebPreviewer/quad_stl_viewer.html  ← 四足 STL 装配查看（当前主线）
└── WebPreviewer/{index.html,app.js,viewer.js,...}  ← 单腿 LUT 渲染（第一阶段，保留）
```

## 数据流水线（四足，当前主线）
```
Fusion 文档
  │  curl /export_assembly?dir=...（一键）
  │  visibility 过滤 + COL_ 识别 + transform2 世界变换 + STL 导出
  ▼
exports/quad_v4/parts_world.json  ← 每个零件的 world_t_mm + world_rot + stl_file + is_collider
  │
  ▼
quad_stl_viewer.html
  mesh.position = world_t_mm        （累乘父亲变换后的世界位置）
  mesh.rotation = world_rot (3x3)   （累乘父亲变换后的世界旋转）
```

## 关键技术点（详见 docs/）

### Fusion 坐标系三层结构（核心，详见 docs/fusion_coordinate_system.md）
- **Occurrence**（实例）→ transform2 决定"零件挂哪、怎么转"
- **Component**（组件）→ STL 顶点用这个的坐标系（mm，局部坐标）
- **BRepBody**（实体）→ 共享 component 坐标系（一个 component 多实体合并成 1 个 STL）

### occurrence 命名陷阱
- 复制组件导致零件同名（`servo_mg90s v2:2` 出现 4 次）
- **必须用 full_path 唯一标识**，不用 occurrence 名

### 镜像 occurrence（详见 docs/quad_v4_alignment.md）
- 镜像腿根旋转 `[1,-1,-1]`（180°绕X，det=+1 合法旋转）会传递整棵子树
- 镜像 STL 顶点 = 原版 Z 取负（diag(1,1,-1) 反射）
- 两层叠加最终只翻 Y（左右镜像）

### 坐标系约定
- X=前后（前腿 X=0，后腿 X=115），Y=左右（右-47/左+47），Z=上下（Z 朝上）
- 单位 mm，Three.js 场景直接用 mm，camera.up = (0,0,1)

## 文件结构
```
scripts/
├── dump_frames_stable.py    # ★ 导出全树世界变换（四足当前）
├── export_all_stls.py       # ★ 导出所有零件 STL（四足当前）
├── fk_newton.py             # 单腿 FK 牛顿迭代求解器（第一阶段）
├── fk_solve.py              # 单腿 FK（伺服角输入）
├── gen_workspace_lut.py     # 单腿解空间 LUT 生成
├── gen_quad_mujoco.py       # ★ 四足 MuJoCo 模型生成器（kin + dyn 双版本）
├── verify_quad_mujoco.py    # ★ 四足 MuJoCo 几何验证（闭环残差 + 对称性）
├── gen_geometry_json.py     # STL→geometry.json 序列化（依赖已删的 export1）
├── leg_viewer.py            # 单腿 MuJoCo 交互查看器
├── plot_workspace.py        # 解空间地图 PNG
├── verify_fk_solver.py      # FK 求解器验证（依赖已删的 export1）
├── verify_mujoco_leg.py     # 单腿 MuJoCo 模型验证
├── quad_pg_env.py           # ★ Playground env（照搬 Go1 joystick，含 jax LUT 查表）
├── passive_lut.py           # ★ jax 双线性插值查 workspace_lut（8 舵机→16 被动关节）
├── train_quad_pg.py         # ★ Brax PPO 训练（照搬 Go1 超参，--unroll_length 可调）
├── render_quad_view.py      # 渲染机器人多角度 mp4（已出 quad_preview.mp4）
├── viewer_interactive.py    # MuJoCo 交互 viewer（WSL GUI 限制未跑通）
├── _verify_lite.py          # quad_lite 综合验证（几何+站立+编译速度）
└── _verify_quad_pg.py       # env reset/step 验证

exports/quad_v4/             # ★ 四足当前数据
├── parts_world.json         # 零件 STL + 世界变换（49 叶子，网页摆放用）
├── nodes_world.json         # ★ 62 节点坐标系骨架（含中间节点反推 + 局部变换）
├── leg_kinematic.json       # 单腿运动学树（6 body + joint_anchor_mm）
├── parts_manifest.json      # STL 文件名 + 层级
└── stl_files/               # 59 个 STL

WebPreviewer/
├── quad_leg_viewer.html     # ★ 四足 LUT 分控 viewer（当前主线，4 腿独立运动 + 预设步态）
├── quad_frames_viewer.html  # 坐标系骨架可视化（62 节点三轴 + 叠加 STL）
├── quad_stl_viewer.html     # 四足 STL 静态装配（已验证位置对齐）
├── index.html + app.js + viewer.js  # 单腿 LUT 渲染（第一阶段）
├── workspace_lut.json       # 单腿 LUT（32761 格，θ1,θ2 → 6 关节角）
├── geometry-loader.js       # geometry.json 解码库（零依赖）
└── vendor/                  # three.min.js + STLLoader + OrbitControls

mujoco_leg/leg.xml           # 单腿 MuJoCo 模型（基准，依赖已删的 export1）
mujoco_quad/                 # ★ 四足 MuJoCo 模型（自动生成）
├── quad_kin.xml             # 运动学调试版（固定 base + 零重力，几何验证用）
├── quad_dyn.xml             # RL 训练版（浮动 base + 重力 + 地面 + 传感器，含 8 equality）
├── quad_playground.xml      # 完整版 + Go1 同款 18 sensor（Playground env 用，仍含 equality）
├── quad_lite.xml            # ★ RL 训练版（无 equality + 16 被动 actuator 锁定，编译 17s）
└── README.md                # RL 接入说明（动作/观测空间 + 镜像腿控制 + 初始化）
F3DRemoteControl/            # HTTP 远程控制 add-in
F3DMaojocoWin/               # Win 版导出插件（STL + BRep）
F3DMaojocoScripts/           # Mac 版导出插件（相对导入）

docs/
├── quad_mirror_kinematics.md       # ★ 镜像腿运动学（取负+anchor Z 取负+数学推导，3 个坑全记录）
├── quad_v4_alignment.md            # 四足位置对齐经验（核心方法+5个坑）
├── quad_v4_tree.md                 # 四足坐标系树结构
├── fusion_coordinate_system.md     # Fusion 坐标系原理
├── fusion_to_web_workflow.md       # 单腿全流程经验（14条）
├── brep_mesh_and_remote_control.md # BRep+远程控制（8个坑）
└── 通用轨道相机设计说明书.md
```

## 运行方式

### 网页查看
```bash
python3 -m http.server 8766
# 四足装配：http://localhost:8766/WebPreviewer/quad_stl_viewer.html
# 单腿 LUT：http://localhost:8766/WebPreviewer/index.html
```

### 远程控制 Fusion（add-in 先在 Fusion Add-Ins 里 Run）
```bash
curl http://127.0.0.1:9099/ping                                        # 连通
curl http://127.0.0.1:9099/reload                                      # 热重载（只清 inf3d/common；改 add-in 主模块要 Stop→Run）
curl -G http://127.0.0.1:9099/exec --data-urlencode "code@scripts/dump_frames_stable.py"   # 调试：导出全树变换
```

### 重新导出四足数据（模型改了之后，一键）
```bash
curl -G http://127.0.0.1:9099/export_assembly \
  --data-urlencode 'dir=\\wsl.localhost\Ubuntu-24.04\home\mg\AIMAO\F3DMaojoco\exports\quad_v4'
# 一次产出 parts_world.json + 所有 STL（visibility 自动过滤，COL_ 标碰撞体）
# 然后刷新 http://localhost:8766/WebPreviewer/quad_stl_viewer.html
```

## 关键经验（详见 docs/）
- **零件 STL = component 局部坐标**，加载时用 transform2 做世界旋转+平移即可复原
- **用 full_path 唯一标识 occurrence**，复制组件会重名
- Fusion API 非线程安全：必须 CustomEvent 主线程调度（/exec 已封装）
- Fusion Python 3.14：模块缓存进程级隔离，热重载要倒序删+清__dict__+invalidate_caches
- 闭环无穷多解：初值决定分支，连续追踪是关键（单腿 FK 经验）
- `transform2` translation 是 cm，要 ×10 转 mm；旋转部分无量纲
