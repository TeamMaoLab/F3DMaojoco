# WebPreviewer - 导出物预览器

基于 Three.js 的纯前端 3D 预览器，用于查看 F3DMaojocoScripts 从 Fusion 360 导出的装配体数据（`component_positions.json` + `stl_files/`）。**零后端、零依赖、双击即用**，可放 GitHub Pages 分享。

这是 VistaQuickViewer（pyvista）的浏览器版替代方案，目的是摆脱 Python 环境依赖、方便分享给他人、跨设备查看。

## 快速使用

### 方式一：双击打开（推荐，零依赖）

1. 双击 `index.html`，用浏览器打开
2. 点击「📂 选择导出目录」
3. 选中一个导出目录（如 `exports/export1/`，需包含 `component_positions.json` 和 `stl_files/` 子目录）
4. 模型即加载渲染

> 浏览器会一次性读取整个目录的文件。`component_positions.json` 解析后，按其中的 `stl_file` 相对路径从已选文件中匹配 STL，**不发起网络请求**，绕过本地文件 CORS 限制。

### 方式二：本地静态服务器（备选）

如果某些浏览器对 `webkitdirectory` 子文件访问有限制，可起本地服务器：

```bash
# 在项目根目录
python3 -m http.server 8000
# 浏览器访问 http://localhost:8000/WebPreviewer/index.html
```

此时仍需通过「选择导出目录」加载文件（预览器不通过 fetch 读取本地文件）。

## 功能

- **零件渲染**：按 `world_transform.matrix`（4x4 行优先，毫米）变换每个 STL，浮点噪声自动清理
- **关节标注**：红色球 + 名称标签，可切换显示/隐藏；无运动限制的关节标 ⚠
- **坐标轴**：XYZ 红绿蓝线（50mm），可切换
- **轨道相机**：左键旋转、右键平移、滚轮缩放、双击适配视角
- **信息面板**：场景摘要（零件/关节数、单位、加载状态）+ 零件列表 + 关节列表（含类型与角度范围）
- **背景切换**：深色 / 白色 / 灰色
- **STL 复用**：同一 STL 被多个 occurrence 引用时（如舵机出现多次）复用 geometry，仅克隆并应用不同矩阵

## 数据格式

预览器读取的导出目录结构：

```
<导出目录>/
├── component_positions.json    # 主数据（meta + components[] + joints[]）
└── stl_files/
    ├── 零件A_零件A.stl
    └── ...
```

`component_positions.json` 的关键字段：

- `meta`：`geometry_unit`（单位，通常 millimeters）、`count_components`、`count_joints`、`format_version`
- `components[i]`：`occurrence_name`（显示名）、`stl_file`（相对路径）、`world_transform.matrix`（4x4 行优先，translation 在 `[i][3]`）
- `joints[i]`：`name`、`joint_type`、`geometry.geometry_one_transform.matrix`（关节位置）、`limits.revolute_limits.rotation_limits`（角度范围）

详见 `F3DMaojocoScripts/common/data_types.py` 顶部的 schema 文档。

## 技术栈

- **Three.js r128**（UMD 全局 build）—— 通过 CDN（unpkg）加载，无需安装
- **STLLoader / OrbitControls**（同 r128）—— 同时支持二进制和 ASCII STL
- 原生 JS，无框架、无构建步骤

> Three.js 默认从 `https://unpkg.com/three@0.128.0` 加载。如需离线使用，可下载到本地 `vendor/`（index.html 会自动降级到本地）：
> ```bash
> cd WebPreviewer
> mkdir -p vendor/examples/js/loaders vendor/examples/js/controls
> curl -sL -o vendor/three.min.js https://unpkg.com/three@0.128.0/build/three.min.js
> curl -sL -o vendor/examples/js/loaders/STLLoader.js https://unpkg.com/three@0.128.0/examples/js/loaders/STLLoader.js
> curl -sL -o vendor/examples/js/controls/OrbitControls.js https://unpkg.com/three@0.128.0/examples/js/controls/OrbitControls.js
> ```
> `vendor/` 已在 `.gitignore` 中，不会提交到版本控制。

## 文件结构

```
WebPreviewer/
├── index.html          # 主页面（UI + 内联 CSS + CDN 加载逻辑）
├── app.js              # 主逻辑（文件加载、JSON 解析、STL 匹配、UI 交互）
├── viewer.js           # 3D 渲染封装（ExportViewer 类）
└── README.md           # 本文档
```
（`vendor/` 为可选的离线副本，按需下载，不纳入版本控制）

## 浏览器兼容性

- Chrome / Edge 86+（webkitdirectory 支持）
- Firefox 119+（webkitdirectory 已标准化）
- Safari 13.1+（webkitdirectory 支持，但中文文件名路径匹配可能有差异）

## 已知限制

- 预览器只读「导出物」，不读 MuJoCo XML（那是后续计划）
- 不做物理仿真，纯几何预览
- 关节只显示位置，不显示旋转轴方向（后续可加）
