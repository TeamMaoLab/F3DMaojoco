# F3DMaojoco 项目状态（2026-07-16）

让舵机四足机器人从 Fusion 360 设计 → 浏览器 3D 复原。
当前进度：**四足位置对齐已完成**，51 个零件按累乘父亲变换在网页正确摆放。

## 当前进展

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
- 腿内 ONE-LEG 机构的关节角（当前零件按默认姿态叠在一起，还没摆成腿形）
- MuJoCo 四足模型（运动学树 + 闭环约束 + 驱动）
- 浏览器交互（关节拖动 / LUT 驱动）

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
├── gen_geometry_json.py     # STL→geometry.json 序列化（依赖已删的 export1）
├── leg_viewer.py            # 单腿 MuJoCo 交互查看器
├── plot_workspace.py        # 解空间地图 PNG
├── verify_fk_solver.py      # FK 求解器验证（依赖已删的 export1）
└── verify_mujoco_leg.py     # 单腿 MuJoCo 模型验证

exports/quad_v4/             # ★ 四足当前数据
├── parts_world.json         # 零件 STL + 世界变换（网页用）
├── parts_manifest.json      # STL 文件名 + 层级
└── stl_files/               # 59 个 STL

WebPreviewer/
├── quad_stl_viewer.html     # ★ 四足 STL 装配查看（当前主线）
├── index.html + app.js + viewer.js  # 单腿 LUT 渲染（第一阶段）
├── geometry-loader.js       # geometry.json 解码库（零依赖）
└── vendor/                  # three.min.js + STLLoader + OrbitControls

mujoco_leg/leg.xml           # 单腿 MuJoCo 模型（基准，依赖已删的 export1）
F3DRemoteControl/            # HTTP 远程控制 add-in
F3DMaojocoWin/               # Win 版导出插件（STL + BRep）
F3DMaojocoScripts/           # Mac 版导出插件（相对导入）

docs/
├── quad_v4_alignment.md            # ★ 四足位置对齐经验（核心方法+5个坑）
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
