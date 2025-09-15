# VistaQuickViewer

VistaQuickViewer 是一个基于 PyVista 的 3D 可视化工具，专门用于快速查看和验证 F3DMaojocoScripts 导出的装配体数据。它提供了直观的 3D 交互界面来检查零部件位置、关节约束和整体装配结构。

## 功能特性

- **基于 PyVista**: 使用强大的 PyVista (VTK) 库进行专业级 3D 可视化
- **智能数据加载**: 自动解析 F3DMaojocoScripts 导出的 JSON 和 STL 数据
- **交互式 3D 查看**: 支持完整的 3D 相机控制（旋转、缩放、平移）
- **关节可视化**: 红色球体标记关节位置，支持交互式显示/隐藏切换
- **坐标轴显示**: 彩色坐标轴和原点标记，便于理解空间方向
- **容错处理**: 当 STL 文件缺失时自动显示占位符几何体
- **多种输出模式**: 支持交互式窗口查看和离线截图保存
- **命令行接口**: 提供完整的命令行工具，支持批量处理

## 系统要求

- Python 3.8+
- PyVista >= 0.38.0
- NumPy >= 1.21.0
- F3DMaojocoScripts (必需，用于数据类型定义和几何计算)

## 安装依赖

```bash
pip install pyvista numpy
```

## 快速开始

### 命令行使用

VistaQuickViewer 提供了功能完整的命令行接口：

```bash
# 基本使用 - 打开交互式查看器
python -m VistaQuickViewer output/

# 保存截图而不显示窗口
python -m VistaQuickViewer output/ --save --output scene.png

# 设置黑色背景并初始显示关节
python -m VistaQuickViewer output/ --bg black --show-joints

# 启用调试模式
python -m VistaQuickViewer output/ --debug
```

### 编程接口

#### 最简单的使用方式

```python
from VistaQuickViewer import quick_view

# 一行代码完成所有操作
viewer = quick_view("output/", show=True)
```

#### 完整控制流程

```python
import logging
from VistaQuickViewer import VistaQuickViewer

# 设置日志
logging.basicConfig(level=logging.INFO)

# 创建查看器实例
viewer = VistaQuickViewer("output/")

# 加载数据
if viewer.load_data():
    print("✓ 数据加载成功")
    
    # 创建 3D 场景
    if viewer.create_scene():
        print("✓ 场景创建成功")
        
        # 显示交互式窗口
        viewer.show()
        
        # 或者保存截图
        # viewer.save_view("assembly_view.png")
```

#### 高级用法

```python
from VistaQuickViewer import VistaQuickViewer
import logging

logger = logging.getLogger(__name__)

# 创建查看器并自定义设置
viewer = VistaQuickViewer("/path/to/export", logger)

# 自定义外观
viewer.set_background_color('black')
viewer.show_joints = True  # 初始显示关节

# 加载和显示
if viewer.load_data() and viewer.create_scene():
    # 获取详细场景信息
    summary = viewer.get_scene_summary()
    print(f"场景统计:")
    print(f"  - 零部件: {summary['component_count']} 个")
    print(f"  - 关节: {summary['joint_count']} 个")
    print(f"  - STL 文件: {summary['stl_file_count']} 个")
    
    # 显示场景
    viewer.show()
```

## 模块架构

```
VistaQuickViewer/
├── __init__.py           # 模块初始化和 API 导出
├── __main__.py           # 命令行接口和主函数
├── viewer.py             # 核心 3D 查看器类 (VistaQuickViewer)
├── data_loader.py        # 数据加载器 (ExportDataLoader)
└── README.md             # 本文档
```

## 核心组件详解

### VistaQuickViewer 类

主要的 3D 查看器类，提供完整的可视化功能：

**核心方法：**
- `load_data()`: 加载并解析导出数据
- `create_scene()`: 构建 3D 场景，包括零部件、关节和坐标轴
- `show()`: 显示交互式 3D 窗口
- `save_view(filepath)`: 保存当前视图为图片文件
- `set_background_color(color)`: 设置场景背景颜色
- `get_scene_summary()`: 获取场景统计信息

**关键特性：**
- 自动应用世界坐标变换矩阵
- 智能相机定位，适应不同大小的装配体
- 交互式关节显示控制（Show Joints 复选框）
- 完整的错误处理和日志记录

### ExportDataLoader 类

数据加载和验证模块：

**核心功能：**
- 解析 `component_positions.json` 文件
- 加载和验证 STL 文件
- 提供零部件和关节的查询接口
- 计算场景边界和统计信息
- 支持数据重新加载和缓存

**数据验证：**
- 检查导出数据完整性
- 验证 STL 文件存在性
- 提供详细的错误诊断信息

## 支持的数据格式

VistaQuickViewer 专门支持 F3DMaojocoScripts 的标准输出格式：

### 必需文件
- **`component_positions.json`**: 主要的数据文件，包含零部件和关节数据
- **`stl_files/`**: STL 几何文件目录

### 数据结构
- **ExportData**: 完整的导出数据容器
- **ComponentInfo**: 零部件信息（名称、变换、STL 路径等）
- **JointInfo**: 关节信息（类型、连接关系、几何变换等）

### 坐标系统
- **单位**: 毫米（与 Fusion 360 保持一致）
- **坐标系**: 右手坐标系
- **变换**: 4x4 变换矩阵，支持平移、旋转和缩放

## 交互控制

### 3D 视图控制
- **左键拖拽**: 旋转视角
- **右键拖拽**: 平移视图
- **鼠标滚轮**: 缩放视图
- **右键菜单**: 访问 PyVista 高级功能

### 特殊控件
- **Show Joints 复选框**: 实时切换关节显示/隐藏
- **坐标轴**: 彩色编码（X=红色, Y=绿色, Z=蓝色）
- **原点标记**: 黄色球体标记坐标原点

## 命令行参数详解

```bash
python -m VistaQuickViewer [OPTIONS] [EXPORT_DIR]

位置参数:
  export_dir              导出目录路径 (默认: output)

显示选项:
  --save                  保存截图而不显示交互窗口
  --output FILE           截图保存路径 (默认: scene_screenshot.png)
  --bg COLOR              背景颜色 (默认: white)
  --no-show               不显示交互窗口
  --show-joints           初始显示关节（默认隐藏）

调试选项:
  --debug                 启用调试模式
  --log-level LEVEL       日志级别 (DEBUG|INFO|WARNING|ERROR)

示例:
  python -m VistaQuickViewer output/ --save --output my_scene.png
  python -m VistaQuickViewer /path/to/data/ --bg black --show-joints --debug
```

## 故障排除

### 常见问题

1. **PyVista 导入错误**
   ```bash
   # 确保安装了正确版本的 PyVista
   pip install --upgrade pyvista
   ```

2. **数据加载失败**
   ```bash
   # 检查目录结构
   ls output/
   # 应该包含:
   # - component_positions.json
   # - stl_files/ 目录
   ```

3. **STL 文件显示问题**
   ```bash
   # 启用调试查看详细信息
   python -m VistaQuickViewer output/ --debug
   ```

### 调试技巧

```python
import logging
# 设置详细日志
logging.basicConfig(level=logging.DEBUG)

# 或者使用命令行
python -m VistaQuickViewer output/ --log-level DEBUG
```

## 开发信息

### 技术架构
- **3D 渲染**: PyVista (VTK) - 专业科学可视化库
- **数值计算**: NumPy - 高性能数组运算
- **数据类型**: F3DMaojocoScripts.common - 共享几何库
- **变换处理**: 自定义 4x4 矩阵变换引擎

### 关键算法
- **坐标变换**: 精确的 4x4 矩阵变换，支持浮点误差清理
- **相机定位**: 基于场景边界的智能相机算法
- **关节位置**: 基于几何变换的世界坐标计算
- **容错处理**: STL 文件缺失时的占位符系统

### 依赖关系
```
VistaQuickViewer
├── F3DMaojocoScripts.common (必需)
│   ├── geometry_math.py (几何计算)
│   └── data_types.py (数据类型)
├── PyVista (3D 可视化)
└── NumPy (数值计算)
```

## 性能优化

### 大型装配体处理
- 自动优化渲染性能
- 支持简化的几何表示
- 智能内存管理

### 内存使用
- STL 文件按需加载
- 数据缓存机制
- 及时资源释放

## 版本信息

- **当前版本**: 0.1.0
- **兼容 Python**: 3.8+
- **支持平台**: Windows, macOS, Linux

## 许可证

本项目遵循与 F3DMaojoco 相同的许可证条款。

## 贡献指南

欢迎提交问题报告和功能请求。贡献代码前请：

1. 确保代码符合项目风格
2. 添加适当的测试用例
3. 更新相关文档
4. 遵循提交信息规范