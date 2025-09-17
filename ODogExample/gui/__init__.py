"""
ODogExample GUI模块

包含主应用、3D查看器、控制面板等图形界面组件。

## 模块结构

### 核心组件
- `app_main.py` - 主应用窗口
- `app_entry.py` - 应用入口和启动逻辑  
- `app_signals.py` - 信号处理和事件管理

### 3D渲染组件
- `viewer_widget.py` - 查看器组件 (重构后的简化版本)
- `camera_system.py` - 相机系统 (OrbitCamera, InputHandler)
- `mujoco_renderer.py` - MuJoCo渲染器

### 控制面板组件
- `control_panel.py` - 主控制面板 (重构后的简化版本)
- `joint_controls.py` - 关节控制组件 (JointControlWidget, LegControlGroup)
- `global_controls.py` - 全局控制组件

### 兼容性包装器
- `main_app.py` - 原有主应用的兼容性包装器
- `control_panels.py` - 原有控制面板的兼容性包装器

## 使用方式

### 新的模块化方式 (推荐)
```python
from gui.app_entry import run_application
run_application()
```

### 传统的向后兼容方式
```python
from gui.main_app import MainApplication
# 或者直接运行
python gui/main_app.py
```

### 组件化使用
```python
from gui.viewer_widget import MuJoCoViewerWidget
from gui.control_panel import create_control_panel
from gui.camera_system import OrbitCamera
```

## 迁移指南

1. **直接替换**: 原有的 `main_app.py` 和 `control_panels.py` 保持兼容
2. **渐进式迁移**: 可以逐步使用新的模块化组件
3. **新功能开发**: 建议使用新的模块化结构
"""

# 核心应用组件
from .app_main import MainApplication
from .app_entry import run_application, main
from .app_signals import SignalManager, ApplicationSignals

# 3D渲染组件
from .viewer_widget import MuJoCoViewerWidget, create_test_viewer
from .camera_system import OrbitCamera, InputHandler, create_orbit_camera, create_input_handler
from .mujoco_renderer import MuJoCoRenderer

# 控制面板组件
from .control_panel import ControlPanel, create_control_panel
from .joint_controls import JointControlWidget, LegControlGroup
from .global_controls import (
    GlobalControlGroup, PrecisionControlGroup, 
    CameraControlGroup, PoseControlGroup
)

# 兼容性导出
from .main_app import main as legacy_main
from .control_panels import create_control_panel as legacy_create_control_panel

__version__ = "0.2.0"  # 更新版本号以反映模块化重构

__all__ = [
    # 核心应用
    'MainApplication', 'run_application', 'main', 
    'SignalManager', 'ApplicationSignals',
    
    # 3D渲染
    'MuJoCoViewerWidget', 'create_test_viewer',
    'OrbitCamera', 'InputHandler', 'create_orbit_camera', 'create_input_handler',
    'MuJoCoRenderer',
    
    # 控制面板
    'ControlPanel', 'create_control_panel',
    'JointControlWidget', 'LegControlGroup',
    'GlobalControlGroup', 'PrecisionControlGroup', 
    'CameraControlGroup', 'PoseControlGroup',
    
    # 兼容性
    'legacy_main', 'legacy_create_control_panel'
]