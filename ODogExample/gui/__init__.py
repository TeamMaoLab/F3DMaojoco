"""
ODogExample GUI模块

包含主应用、3D查看器、控制面板等图形界面组件。

## 模块结构

### 核心组件
- `app_main.py` - 主应用窗口
- `app_entry.py` - 应用入口和启动逻辑
- `app_signals.py` - 信号处理和事件管理

### 3D渲染组件
- `viewer_widget.py` - 查看器组件
- `camera_system.py` - 相机系统 (OrbitCamera, InputHandler)
- `mujoco_renderer.py` - MuJoCo渲染器

### 控制面板组件
- `control_panel.py` - 主控制面板
- `joint_controls.py` - 关节控制组件 (JointControlWidget, LegControlGroup)
- `global_controls.py` - 全局控制组件
- `pose_manager.py` - 姿态管理 (PoseManager，持久化到 data/poses.json)
- `motion_editor_tab.py` - 动作序列编辑器 Tab

## 使用方式

```python
from gui.app_entry import run_application
run_application()
```

### 组件化使用
```python
from gui.viewer_widget import MuJoCoViewerWidget
from gui.control_panel import create_control_panel
from gui.camera_system import OrbitCamera
```
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

__version__ = "0.3.0"

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
]
