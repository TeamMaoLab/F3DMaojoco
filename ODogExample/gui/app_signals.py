"""
ODogExample GUI模块 - 应用信号处理

提供主应用的信号处理和事件管理。
"""

from typing import Dict, Any
from PySide6.QtCore import QObject, Signal


class ApplicationSignals(QObject):
    """应用信号处理器"""
    
    # 关节控制信号
    jointAngleChanged = Signal(str, float)  # 关节名称, 角度值
    allJointsZero = Signal()                # 所有关节归零
    allJointsReset = Signal()               # 所有关节重置
    
    # 姿态管理信号
    poseSaved = Signal(str, dict)           # 姿态名称, 姿态数据
    poseLoaded = Signal(str, dict)          # 姿态名称, 姿态数据
    
    # 相机控制信号
    cameraTrackingToggled = Signal(bool)   # 相机跟踪开关
    cameraRefocus = Signal()               # 重新聚焦
    
    # 应用状态信号
    applicationStarted = Signal()           # 应用启动
    applicationClosing = Signal()           # 应用关闭
    modelLoaded = Signal(bool)              # 模型加载状态
    
    def __init__(self, parent=None):
        super().__init__(parent)
        print("🔗 应用信号处理器初始化完成")


class SignalManager:
    """信号管理器 - 统一管理所有信号的连接"""
    
    def __init__(self):
        self.signals = ApplicationSignals()
        self.connections = []
        print("🔗 信号管理器初始化完成")
    
    def connect_joint_control_signals(self, control_panel, viewer_widget):
        """连接关节控制信号"""
        # 控制面板到查看器的信号连接
        control_panel.jointAngleChanged.connect(
            lambda name, angle: viewer_widget.update() if viewer_widget else None
        )
        
        # 应用层信号处理
        control_panel.allJointsZero.connect(
            lambda: print("🔄 应用层响应：所有关节已归零")
        )
        
        control_panel.allJointsReset.connect(
            lambda: print("🔙 应用层响应：所有关节已重置")
        )
    
    def connect_camera_control_signals(self, control_panel, viewer_widget):
        """连接相机控制信号"""
        # 访问嵌套在控制面板中的相机控制按钮
        camera_control = control_panel.camera_control
        camera_control.tracking_btn_ref.toggled.connect(
            viewer_widget.toggle_camera_tracking if viewer_widget else lambda x: None
        )
        
        camera_control.refocus_btn_ref.clicked.connect(
            viewer_widget.refocus_camera if viewer_widget else lambda: None
        )
    
    def connect_pose_signals(self, control_panel):
        """连接姿态管理信号"""
        control_panel.poseSaved.connect(
            lambda name, data: print(f"💾 应用层响应：姿态 {name} 已保存")
        )
    
    def connect_viewer_signals(self, viewer_widget):
        """连接查看器信号"""
        # 这里可以添加查看器到应用层的信号连接
        pass
    
    def disconnect_all(self):
        """断开所有信号连接"""
        print("🔌 信号连接已断开（Qt会自动处理）")
    
    def get_signal_summary(self) -> Dict[str, int]:
        """获取信号连接摘要"""
        return {
            'total_connections': len(self.connections),
            'connection_types': '自动连接管理'
        }