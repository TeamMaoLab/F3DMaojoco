# VistaQuickViewer

VistaQuickViewer 是一个基于 PyVista 的 3D 可视化模块，用于快速查看 F3DMaojocoScripts 的导出结果。它提供了简洁的 API 来加载和显示从 Fusion 360 导出的装配体和关节数据。

## 功能特性

- **基于 PyVista**: 使用强大的 PyVista 库进行 3D 可视化
- **数据加载**: 自动解析 F3DMaojocoScripts 的导出结果
- **交互式查看**: 支持鼠标控制相机旋转、缩放和平移
- **STL 支持**: 自动加载和显示 STL 文件
- **关节可视化**: 红色球体表示关节位置
- **占位符几何**: 当 STL 文件不可用时显示占位符
- **截图功能**: 支持保存当前视图为图片

## 系统要求

- Python 3.8+
- PyVista >= 0.38.0
- NumPy >= 1.21.0
- F3DMaojocoScripts (用于数据类型定义)

## 安装依赖

```bash
pip install pyvista numpy
```

## 使用方法

### 作为子模块使用

VistaQuickViewer 设计为子模块，可以直接导入和使用：

```python
from VistaQuickViewer import VistaQuickViewer, quick_view

# 快速查看
viewer = quick_view("/path/to/export", show=True)

# 或者手动创建
viewer = VistaQuickViewer("/path/to/export")
if viewer.load_data() and viewer.create_scene():
    viewer.show()
```

### 基本示例

```python
import logging
from VistaQuickViewer import VistaQuickViewer

# 设置日志
logging.basicConfig(level=logging.INFO)

# 创建查看器
viewer = VistaQuickViewer("../output")

# 加载数据
if viewer.load_data():
    print("数据加载成功")
    
    # 创建场景
    if viewer.create_scene():
        print("场景创建成功")
        
        # 显示场景
        viewer.show()
        
        # 或者保存截图
        # viewer.save_view("scene.png")
```

### 高级用法

```python
from VistaQuickViewer import VistaQuickViewer
import logging

logger = logging.getLogger(__name__)

# 创建查看器
viewer = VistaQuickViewer("/path/to/export", logger)

# 自定义设置
viewer.set_background_color('black')

# 加载和显示
if viewer.load_data() and viewer.create_scene():
    # 获取场景信息
    summary = viewer.get_scene_summary()
    print(f"场景包含 {summary['component_count']} 个零部件")
    
    # 显示或保存
    viewer.show()  # 交互式窗口
    # 或
    viewer.save_view("output.png")  # 保存截图
```

## 模块结构

```
VistaQuickViewer/
├── __init__.py           # 模块初始化和导出
├── viewer.py             # 主要的 3D 查看器类
├── data_loader.py        # 数据加载器
├── example.py           # 使用示例
├── requirements.txt     # 依赖包列表
└── README.md           # 说明文档
```

## 核心组件

### VistaQuickViewer 类

主要的 3D 查看器类，提供：

- `load_data()`: 加载导出数据
- `create_scene()`: 创建 3D 场景
- `show()`: 显示交互式窗口
- `save_view()`: 保存截图
- `set_background_color()`: 设置背景颜色
- `get_scene_summary()`: 获取场景信息

### ExportDataLoader 类

数据加载器，负责：

- 解析 JSON 数据文件
- 加载 STL 文件
- 验证数据完整性
- 提供数据访问接口

## 支持的数据格式

VistaQuickViewer 支持 F3DMaojocoScripts 导出的标准数据格式：

- **JSON 数据文件**: `component_positions_*.json`
- **STL 文件**: `stl_files/` 目录下的 STL 文件
- **元数据**: 导出时间、单位、统计信息等

## 交互控制

在交互式窗口中：

- **左键拖拽**: 旋转视角
- **右键拖拽**: 平移视图
- **滚轮**: 缩放视图
- **右键菜单**: 访问 PyVista 的各种功能

## 开发信息

### 技术栈

- **3D 可视化**: PyVista (基于 VTK)
- **数值计算**: NumPy
- **数据类型**: F3DMaojocoScripts/common 模块
- **数据格式**: JSON

### 依赖关系

- 直接导入 `F3DMaojocoScripts.common` 模块
- 使用标准的 Python 3.8+ 库
- 需要 PyVista 和 NumPy 支持

## 故障排除

### 常见问题

1. **导入错误**: 确保已安装 PyVista 和 NumPy
2. **数据加载失败**: 检查导出目录是否包含有效的数据文件
3. **STL 文件找不到**: 确保 `stl_files/` 目录存在且包含 STL 文件

### 调试方法

```python
import logging
logging.basicConfig(level=logging.DEBUG)

# 然后运行你的代码
```

## 示例脚本

运行提供的示例脚本：

```bash
python example.py
```

这将演示如何使用 VistaQuickViewer 的基本功能。

## 版本历史

### v0.1.0

- 初始版本发布
- 基于 PyVista 的 3D 可视化
- 支持 F3DMaojocoScripts 数据格式
- 提供简洁的 API 接口

## 许可证

本项目遵循与 F3DMaojoco 相同的许可证。

## 贡献

欢迎提交问题和功能请求。如果您想贡献代码，请遵循项目的编码规范。

## 联系方式

如有问题或建议，请通过项目仓库联系我们。