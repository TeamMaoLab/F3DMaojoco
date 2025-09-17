"""
ODogExample GUI模块 - 控制面板 (兼容性包装器)

为了保持向后兼容性，原有的 control_panels.py 现在作为新模块的包装器。
"""

from .control_panel import create_control_panel, ControlPanel

# 为了保持向后兼容性，导出原有的类和函数
JointControlWidget = None  # 已移动到 joint_controls.py
LegControlGroup = None      # 已移动到 joint_controls.py

# 保持原有的 create_control_panel 函数接口
def create_control_panel(robot_model=None):
    """创建控制面板实例 - 重定向到新实现"""
    from .control_panel import create_control_panel as new_create_control_panel
    return new_create_control_panel(robot_model)


if __name__ == "__main__":
    # 测试脚本 - 重定向到新实现
    from .control_panel import main
    main()