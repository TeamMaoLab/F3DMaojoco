"""
ODogExample GUI模块 - Tab页控制面板

将原有控制面板和动作编辑器整合为Tab页形式。
"""

import sys
import os
from typing import Optional
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QTabWidget, QFrame
)
from PySide6.QtCore import Signal

try:
    from ..core.robot_model import RobotModel
    from .control_panel import ControlPanel
    from .motion_editor_tab import MotionSequenceTabWidget
except ImportError:
    # 如果相对导入失败，尝试绝对导入
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from core.robot_model import RobotModel
    from gui.control_panel import ControlPanel
    from gui.motion_editor_tab import MotionSequenceTabWidget


class TabbedControlPanel(QWidget):
    """Tab页控制面板"""
    
    # 信号定义 - 从原有控制面板继承
    jointAngleChanged = Signal(str, float)  # 关节角度改变
    allJointsZero = Signal()                # 所有关节归零
    allJointsReset = Signal()               # 所有关节重置
    poseSaved = Signal(str, dict)           # 姿态保存
    
    # 信号定义 - 从动作编辑器继承
    sequenceSelected = Signal(str)      # 选中动作序列
    sequenceCreated = Signal(str)       # 创建动作序列
    sequenceDeleted = Signal(str)      # 删除动作序列
    keyframeSelected = Signal(int)     # 选中关键帧
    keyframeAdded = Signal(int)        # 添加关键帧
    keyframeDeleted = Signal(int)       # 删除关键帧
    playbackStarted = Signal()         # 播放开始
    playbackPaused = Signal()          # 播放暂停
    playbackStopped = Signal()         # 播放停止
    playbackProgress = Signal(float)   # 播放进度
    
    def __init__(self, robot_model: Optional[RobotModel] = None, parent=None):
        super().__init__(parent)
        self.robot_model = robot_model
        self.init_ui()
        self.setup_connections()
        
    def init_ui(self):
        """初始化UI"""
        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        
        # 创建Tab页
        self.tab_widget = QTabWidget()
        
        # 第一个Tab页：原有控制面板（关节调整 + 姿态管理）
        self.control_panel = ControlPanel(self.robot_model)
        self.tab_widget.addTab(self.control_panel, "🎮 关节与姿态控制")
        
        # 第二个Tab页：动作编辑器
        self.motion_editor = MotionSequenceTabWidget()
        self.tab_widget.addTab(self.motion_editor, "🎬 动作编辑器")
        
        # 设置Tab样式
        self.tab_widget.setStyleSheet("""
            QTabWidget::pane {
                border: 1px solid #ddd;
                background: white;
                border-radius: 4px;
            }
            
            QTabWidget::tab-bar {
                left: 5px;
            }
            
            QTabBar::tab {
                background: #f8f9fa;
                border: 1px solid #ddd;
                border-bottom: none;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
                padding: 8px 16px;
                margin-right: 2px;
                font-weight: bold;
            }
            
            QTabBar::tab:selected {
                background: white;
                border-color: #2196F3;
                border-bottom: 1px solid white;
            }
            
            QTabBar::tab:hover {
                background: #e9ecef;
            }
            
            QTabBar::tab:selected:hover {
                background: white;
            }
        """)
        
        layout.addWidget(self.tab_widget)
        self.setLayout(layout)
    
    def setup_connections(self):
        """设置信号连接"""
        # 连接原有控制面板信号
        self.control_panel.jointAngleChanged.connect(self.on_joint_angle_changed)
        self.control_panel.allJointsZero.connect(self.on_all_joints_zero)
        self.control_panel.allJointsReset.connect(self.on_all_joints_reset)
        self.control_panel.poseSaved.connect(self.on_pose_saved)
        
        # 连接动作编辑器信号
        self.motion_editor.sequenceSelected.connect(self.on_sequence_selected)
        self.motion_editor.sequenceCreated.connect(self.on_sequence_created)
        self.motion_editor.sequenceDeleted.connect(self.on_sequence_deleted)
        self.motion_editor.keyframeSelected.connect(self.on_keyframe_selected)
        self.motion_editor.keyframeAdded.connect(self.on_keyframe_added)
        self.motion_editor.keyframeDeleted.connect(self.on_keyframe_deleted)
        self.motion_editor.playbackStarted.connect(self.on_playback_started)
        self.motion_editor.playbackPaused.connect(self.on_playback_paused)
        self.motion_editor.playbackStopped.connect(self.on_playback_stopped)
        self.motion_editor.playbackProgress.connect(self.on_playback_progress)
    
    # 原有控制面板信号处理
    def on_joint_angle_changed(self, joint_name: str, angle: float):
        """关节角度改变处理"""
        self.jointAngleChanged.emit(joint_name, angle)
    
    def on_all_joints_zero(self):
        """所有关节归零处理"""
        self.allJointsZero.emit()
    
    def on_all_joints_reset(self):
        """所有关节重置处理"""
        self.allJointsReset.emit()
    
    def on_pose_saved(self, pose_name: str, pose_data: dict):
        """姿态保存处理"""
        self.poseSaved.emit(pose_name, pose_data)
    
    # 动作编辑器信号处理
    def on_sequence_selected(self, seq_name: str):
        """序列选中处理"""
        self.sequenceSelected.emit(seq_name)
    
    def on_sequence_created(self, seq_name: str):
        """序列创建处理"""
        self.sequenceCreated.emit(seq_name)
    
    def on_sequence_deleted(self, seq_name: str):
        """序列删除处理"""
        self.sequenceDeleted.emit(seq_name)
    
    def on_keyframe_selected(self, keyframe_index: int):
        """关键帧选中处理"""
        self.keyframeSelected.emit(keyframe_index)
    
    def on_keyframe_added(self, keyframe_index: int):
        """关键帧添加处理"""
        self.keyframeAdded.emit(keyframe_index)
    
    def on_keyframe_deleted(self, keyframe_index: int):
        """关键帧删除处理"""
        self.keyframeDeleted.emit(keyframe_index)
    
    def on_playback_started(self):
        """播放开始处理"""
        self.playbackStarted.emit()
    
    def on_playback_paused(self):
        """播放暂停处理"""
        self.playbackPaused.emit()
    
    def on_playback_stopped(self):
        """播放停止处理"""
        self.playbackStopped.emit()
    
    def on_playback_progress(self, progress: float):
        """播放进度处理"""
        self.playbackProgress.emit(progress)
    
    # 公共方法
    def set_robot_model(self, robot_model: RobotModel):
        """设置机器人模型"""
        self.robot_model = robot_model
        self.control_panel.set_robot_model(robot_model)
    
    def get_current_pose(self):
        """获取当前姿态"""
        return self.control_panel.get_current_pose()
    
    def set_pose(self, pose_data):
        """设置姿态"""
        self.control_panel.set_pose(pose_data)
    
    def set_active_tab(self, tab_index: int):
        """设置活动的Tab页"""
        if 0 <= tab_index < self.tab_widget.count():
            self.tab_widget.setCurrentIndex(tab_index)
    
    def get_active_tab(self) -> int:
        """获取当前活动的Tab页索引"""
        return self.tab_widget.currentIndex()
    
    def update_current_pose(self, pose_data):
        """更新当前姿态数据"""
        if hasattr(self.control_panel, 'update_current_pose'):
            self.control_panel.update_current_pose(pose_data)


def create_tabbed_control_panel(robot_model: Optional[RobotModel] = None) -> TabbedControlPanel:
    """
    创建Tab页控制面板实例
    
    Args:
        robot_model: 机器人模型，可选
        
    Returns:
        TabbedControlPanel: Tab页控制面板实例
    """
    return TabbedControlPanel(robot_model)


if __name__ == "__main__":
    """测试脚本"""
    from PySide6.QtWidgets import QApplication
    
    app = QApplication(sys.argv)
    
    # 创建Tab页控制面板
    panel = create_tabbed_control_panel()
    panel.setWindowTitle("ODogExample Tab页控制面板测试")
    panel.resize(800, 800)
    
    # 测试信号连接
    panel.jointAngleChanged.connect(lambda name, angle: print(f"关节角度: {name} = {angle}"))
    panel.allJointsZero.connect(lambda: print("所有关节归零"))
    panel.allJointsReset.connect(lambda: print("所有关节重置"))
    panel.poseSaved.connect(lambda name, data: print(f"姿态保存: {name}"))
    
    panel.sequenceSelected.connect(lambda name: print(f"序列选中: {name}"))
    panel.keyframeSelected.connect(lambda idx: print(f"关键帧选中: 第{idx+1}帧"))
    panel.playbackStarted.connect(lambda: print("播放开始"))
    panel.playbackStopped.connect(lambda: print("播放停止"))
    
    panel.show()
    print("🎛️ Tab页控制面板测试启动成功！")
    
    sys.exit(app.exec())