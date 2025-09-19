# F3DMaojoco

F3D 到 MuJoCo 的转换工具，支持将 FreeCAD 中的 3D 模型转换为 MuJoCo 仿真环境。

## 项目包含

1. **ODogExample** - 8自由度四足机器狗控制平台
   - 基于 PySide6 + MuJoCo 的GUI应用
   - 支持姿态编辑、动作序列编辑、实时控制
   - 详细的开发文档：[ODogExample 开发任务纲要](ODogExample/docs/README.md)

2. **通用轨道相机系统** - 专业级3D相机控制
   - 设计文档：[通用轨道相机设计说明书](docs/通用轨道相机设计说明书.md)

## 快速开始

### 运行ODogExample

#### 环境要求
- Python 3.12+ (推荐)
- uv (包管理器)

#### 安装和运行

1. **安装uv包管理器**：
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

2. **进入项目目录**：
```bash
cd ODogExample
```

3. **安装Python和依赖**：
```bash
# 使用uv安装Python 3.12
uv python install 3.12

# 同步项目依赖
uv sync
```

4. **运行应用程序**：
```bash
# 使用uv运行
uv run -m ODogExample.gui.app_entry
```

#### 一键启动（推荐）
如果已安装uv，直接在项目根目录运行：
```bash
cd ODogExample && uv run -m ODogExample.gui.app_entry
```

详细开发文档请查看：[ODogExample 开发任务纲要](ODogExample/docs/README.md)

## 项目结构

```
F3DMaojoco/
├── ODogExample/          # 8自由度四足机器狗控制平台
│   ├── core/            # 核心功能模块
│   ├── gui/             # GUI界面模块  
│   ├── data/            # 数据文件
│   └── docs/            # 详细开发文档
├── docs/                # 通用文档
│   ├── README.md        # 项目纲要
│   └── 通用轨道相机设计说明书.md
├── MaojocoConverter/    # 核心转换器
├── F3DMaojocoScripts/   # 脚本工具
├── VistaQuickViewer/    # 快速查看器
└── tmp-output/          # 临时输出文件
```

## 许可证

参见 [LICENSE](LICENSE) 文件。