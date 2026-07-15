# F3DMaojoco 项目状态（2026-07-15）

## 项目目标
让一个舵机四足单腿模型在浏览器里动起来。模型从 Fusion 360 导出，含 2 个主动关节（连舵机）+ 6 个被动关节 + 2 条闭环约束。
**已达成**：Fusion 设计 → MuJoCo 离线求解解空间 → 浏览器 LUT 驱动交互渲染，全链路打通。

## 架构全景
```
Fusion 360（Python 3.14 主线程）
├── F3DRemoteControl add-in（常驻，HTTP 远程控制）
│   ├── /ping /reload /modules（纯 Python）
│   └── /exec /export /model /brep_stats（Fusion API，走主线程队列）
├── F3DMaojocoWin 导出脚本（手动 Run）
│   └── inf3d/ + common/（绝对导入）
│       ├── component_collector（+ BRep 曲面参数提取）
│       ├── stl_exporter（STL 网格）
│       ├── brep_mesh_exporter（BRep 精确网格，body 局部坐标）
│       └── fusion_export_manager（主流程编排）
│
└── exports/export1/
    ├── component_positions.json（零件+关节+BRep曲面参数+碰撞体）
    ├── stl_files/（STL 网格，fallback）
    └── brep_geometry.json（BRep 精确网格，网页优先加载）

WSL 命令行 / 浏览器
└── curl http://127.0.0.1:9099/...（远程控制 Fusion）
└── WebPreviewer/（Three.js 渲染，加载优先级 brep > stl）
└── scripts/（MuJoCo FK 求解 + LUT 生成 + 几何序列化）
```

## 已完成的工作

### 第一阶段：MuJoCo 求解 + 网页 LUT 驱动
- **纯运动学牛顿迭代 FK**（fk_newton.py）：不用 MuJoCo 动力学/约束求解器，只用 mj_forward 当 FK 引擎，自己解闭环约束方程组（4被动角 + 2闭环anchor重合）
- **BFS 连续追踪**（gen_workspace_lut.py）：解决闭环无穷多解的缠绕问题——从(0,0)种子 BFS 扩展 + 多初值 + 连续性选择，±180° 范围 32761 格点零缠绕
- **全局连续性平滑**（refine_continuity）：消除大范围分支跳变（4095→474处）
- **连通性分析**：flood fill 标记与原点连通的可达区，孤岛标黄拖不进
- **碰撞检测**：Fusion COL_ 圆柱体导出 + MuJoCo contact pair 强制父子 body 碰撞
- **网页**：LUT 双线性插值驱动 3D 腿 + 解空间四色地图（绿连通/黄孤岛/红碰撞/黑不可达）+ 连续拖动追踪（stepTowards 挡回红区）

### 第二阶段：STL→JSON 序列化
- **geometry.json**（gen_geometry_json.py）：STL 顶点去重(83%) + base64(Float32/Int16) + Uint16 indices，5.2MB→1.65MB
- **geometry-loader.js**：零依赖解码库，跨项目复用

### 第三阶段：BRep 精确网格 + 远程控制
- **BRep 精确网格**（brep_mesh_exporter.py）：TriangleMeshCalculator + setQuality(15=VeryHigh)，圆柱面圆滑，21298顶点/756KB
- **F3DRemoteControl add-in**：HTTP 远程控制 Fusion（热重载/exec/导出/查询）
- **线程安全**：CustomEvent 主线程调度（后台线程接请求→队列→主线程执行 Fusion API）
- **坐标系修复**：BRep 用 body 局部坐标（和 STL 一致，让 viewer 应用 world_transform 转一次）

## 关键技术点

### MuJoCo FK 求解（scripts/fk_newton.py）
- 只用 mj_forward 算 body 世界坐标，不用约束求解器
- 牛顿迭代解 4 被动角：数值雅可比(4×4) + J·dx=−f，残差 < 1e-8mm
- **初值决定分支**：闭环三角方程有无穷多解，连续追踪(邻居解作初值)是关键

### BRep 网格化（F3DMaojocoWin/inf3d/brep_mesh_exporter.py）
- body.meshManager.createMeshCalculator() → setQuality(15) → calculate()
- **body 局部坐标**（cm→mm），不应用 world_transform（viewer 会转）
- API 属性名：setQuality/surfaceTolerance/maxNormalDeviation/maxSideLength（非 angleTolerance）

### 远程控制线程安全（F3DRemoteControl/）
- CustomEvent：app.registerCustomEvent + CustomEventHandler.notify（主线程）
- 后台线程 fireCustomEvent 通知 + Event.wait 阻塞等结果
- 纯 Python 端点直接执行，Fusion API 端点走队列

### 渲染（WebPreviewer/）
- 加载优先级：brep_geometry.json > geometry.json > STL fetch
- viewer.js：mesh.matrix = world_transform（matrixAutoUpdate=false），applyJointRotations 按树深度累积旋转
- 组件按运动学树刚体配色（与 MuJoCo 对齐）

## 文件结构
```
F3DMaojocoScripts/          # Mac 版导出插件（相对导入，原始开发）
F3DMaojocoWin/F3DMaojocoWin/ # Win 版导出插件（绝对导入，含 manifest）
F3DRemoteControl/F3DRemoteControl/ # 远程控制 add-in（HTTP + CustomEvent）
mujoco_leg/leg.xml          # MuJoCo 模型（运动学树+闭环约束+碰撞体）
scripts/
├── fk_newton.py            # 纯运动学牛顿迭代 FK 求解器
├── gen_workspace_lut.py    # LUT 生成（BFS+平滑+连通性）
├── gen_geometry_json.py    # STL→geometry.json 序列化
├── plot_workspace.py       # 解空间地图 PNG
└── leg_viewer.py           # MuJoCo 交互查看器（验证用）
WebPreviewer/
├── app.js                  # 主逻辑（LUT插值/拖动/解空间地图）
├── viewer.js               # Three.js 渲染（配色/材质/坐标轴）
├── geometry-loader.js      # geometry.json 解码库（零依赖）
├── brep_geometry.json      # BRep 精确网格（生成物）
└── workspace_lut.json      # 关节角查找表（生成物）
docs/
├── fusion_to_web_workflow.md           # 全流程经验总结（14条可复用经验）
└── brep_mesh_and_remote_control.md     # BRep+远程控制探索（8个坑+架构图）
```

## 运行方式

### 网页预览
```bash
python3 -m http.server 8766
# 浏览器 http://localhost:8766/WebPreviewer/index.html
```

### 远程控制 Fusion（add-in 先在 Fusion Add-Ins 里 Run）
```bash
curl http://127.0.0.1:9099/ping                              # 连通
curl http://127.0.0.1:9099/reload                           # 热重载（不用重启Fusion）
curl http://127.0.0.1:9099/model                             # 查装配体结构
curl http://127.0.0.1:9099/brep_stats                        # BRep曲面统计
curl -G http://127.0.0.1:9099/export --data-urlencode 'dir=路径'  # 导出
```

### 重新生成解空间 LUT
```bash
.venv/bin/python scripts/gen_workspace_lut.py   # ~1.5min，输出到 WebPreviewer/
```

## 已知问题 / 待办
1. F3DMaojocoScripts(Mac) 和 F3DMaojocoWin(Win) 两份代码需手动同步（改一个要 cp 到另一个 + 改导入风格）
2. NURBS 面（39%）仍用网格，未探索控制点导出
3. 解空间边缘（距原点≥88°）有 474 处分支跳变（物理真实的多构型，非 bug）
4. geometry-loader.js 的 `/exec` 复杂代码避免 URL 特殊字符（用临时文件 + exec(open())）

## 关键经验（详见 docs/）
- 闭环无穷多解：初值决定分支，连续追踪是关键（不是调参问题）
- Fusion API：objectType 带命名空间前缀（adsk::fusion::xxx），用 endswith/模糊匹配
- Fusion Python 3.14：模块缓存进程级隔离，热重载要倒序删+清__dict__+invalidate_caches
- Fusion API 非线程安全：必须 CustomEvent 主线程调度
- STL/BRep 顶点都是 body 局部坐标，viewer 应用 world_transform 转一次
