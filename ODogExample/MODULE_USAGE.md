# ODogExample 模块化使用指南

## 🎯 概述

ODogExample 现在支持多种运行方式，既可以直接运行脚本文件，也可以作为 Python 包进行模块化运行。

## 🚀 启动方式

### 1. 模块化运行（推荐）

```bash
# 从项目根目录运行
python -m ODogExample.gui.app_entry
python -m ODogExample.gui.app_main
```

### 2. 直接运行

```bash
# 从子目录运行
cd ODogExample
python gui/app_entry.py
python gui/app_main.py
```

### 3. 编程式启动

```python
from ODogExample import main
main()
```

### 4. 组件导入

```python
from ODogExample.gui.app_main import MainApplication
from ODogExample.core import RobotModel
from ODogExample.gui.viewer_widget import MuJoCoViewerWidget
```

## 📦 包结构

```
F3DMaojoco/
├── __init__.py              # 主包初始化
└── ODogExample/            # ODogExample 子包
    ├── __init__.py          # 包初始化，导出主要接口
    ├── core/                # 核心模块
    ├── gui/                 # GUI 模块
    └── demo_module_usage.py # 使用演示
```

## 🔧 实现细节

### 路径处理
- 修复了模型文件路径问题，确保模块化运行时能正确找到模型文件
- 使用 `os.path` 动态计算文件路径

### 信号管理
- 简化了信号管理器，让 Qt 自动处理信号连接的生命周期
- 避免了手动断开连接时的错误

### MuJoCo API 兼容性
- 修复了相机系统中的 API 兼容性问题
- 使用正确的常量值避免运行时错误

## ✅ 测试验证

所有运行方式都经过测试验证：
- ✅ 模块化运行成功
- ✅ 直接运行成功
- ✅ 组件导入成功
- ✅ 模型加载成功
- ✅ GUI 功能正常

## 🎮 使用建议

1. **开发时**: 使用 `python gui/app_main.py` 直接运行，便于调试
2. **部署时**: 使用 `python -m ODogExample.gui.app_entry` 模块化运行
3. **集成时**: 使用组件导入方式，在其他项目中复用功能

## 🔮 未来扩展

模块化架构支持以下扩展：
- 作为包发布到 PyPI
- 集成到其他项目中
- 创建命令行工具
- 开发插件系统