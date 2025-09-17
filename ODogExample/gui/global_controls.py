"""
ODogExample GUI模块 - 全局控制组件

提供全局控制、相机控制、姿态控制等功能面板。
"""

import math
import sys
import os
from typing import Dict
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, 
    QPushButton, QLabel
)
from PySide6.QtCore import Signal

try:
    from ..core.robot_model import RobotModel
    from ..core.joint_mapping import JointMapping
except ImportError:
    # 如果相对导入失败，尝试绝对导入
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from core.robot_model import RobotModel
    from core.joint_mapping import JointMapping


class GlobalControlGroup(QGroupBox):
    """全局控制组"""
    
    # 信号定义
    allJointsZero = Signal()                # 所有关节归零
    allJointsReset = Signal()               # 所有关节重置
    syncToRobot = Signal()                  # 同步到机器人
    
    def __init__(self, parent=None):
        super().__init__("🌐 全局控制", parent)
        self.init_ui()
    
    def init_ui(self):
        """初始化UI"""
        layout = QHBoxLayout()
        
        # 全局归零
        zero_all_btn = QPushButton("🔄 全部归零")
        zero_all_btn.setStyleSheet("QPushButton { background-color: #ff5722; color: white; font-weight: bold; padding: 8px; }")
        zero_all_btn.clicked.connect(self.zero_all_joints)
        
        # 全局重置
        reset_all_btn = QPushButton("🔙 全部重置")
        reset_all_btn.setStyleSheet("QPushButton { background-color: #2196f3; color: white; font-weight: bold; padding: 8px; }")
        reset_all_btn.clicked.connect(self.reset_all_joints)
        
        # 同步到机器人
        sync_btn = QPushButton("🤖 同步到机器人")
        sync_btn.setStyleSheet("QPushButton { background-color: #4caf50; color: white; font-weight: bold; padding: 8px; }")
        sync_btn.clicked.connect(self.sync_to_robot)
        
        layout.addWidget(zero_all_btn)
        layout.addWidget(reset_all_btn)
        layout.addWidget(sync_btn)
        
        self.setLayout(layout)
    
    def zero_all_joints(self):
        """所有关节归零"""
        print("🔄 所有关节归零")
        self.allJointsZero.emit()
    
    def reset_all_joints(self):
        """所有关节重置"""
        print("🔙 所有关节重置")
        self.allJointsReset.emit()
    
    def sync_to_robot(self):
        """同步到机器人"""
        print("🤖 同步到机器人")
        self.syncToRobot.emit()


class PrecisionControlGroup(QGroupBox):
    """精细控制组"""
    
    # 信号定义
    globalPrecisionToggled = Signal(bool)  # 全局精细控制开关
    
    def __init__(self, parent=None):
        super().__init__("🎛️ 精细控制", parent)
        self.init_ui()
    
    def init_ui(self):
        """初始化UI"""
        layout = QVBoxLayout()
        
        # 全局精细控制开关
        precision_layout = QHBoxLayout()
        
        self.global_precision_btn = QPushButton("📏 切换所有关节精细控制")
        self.global_precision_btn.setCheckable(True)
        self.global_precision_btn.setChecked(False)
        self.global_precision_btn.setStyleSheet("""
            QPushButton {
                background-color: #f0f0f0;
                border: 1px solid #ccc;
                padding: 8px;
                font-weight: bold;
            }
            QPushButton:checked {
                background-color: #2196f3;
                color: white;
            }
        """)
        self.global_precision_btn.toggled.connect(self.toggle_global_precision)
        
        precision_layout.addWidget(self.global_precision_btn)
        layout.addLayout(precision_layout)
        
        # 说明文字
        info_label = QLabel("• 普通模式：滑块快速调节\n• 精细模式：数值精确调节（0.1°精度）")
        info_label.setStyleSheet("color: #666; font-size: 10px;")
        layout.addWidget(info_label)
        
        self.setLayout(layout)
    
    def toggle_global_precision(self, checked: bool):
        """切换全局精细控制"""
        print(f"🎛️ 全局精细控制: {'开启' if checked else '关闭'}")
        self.globalPrecisionToggled.emit(checked)


class CameraControlGroup(QGroupBox):
    """相机控制组"""
    
    # 信号定义
    cameraTrackingToggled = Signal(bool)  # 相机跟踪开关
    cameraRefocus = Signal()               # 重新聚焦
    
    def __init__(self, parent=None):
        super().__init__("📷 相机控制", parent)
        self.camera_status_label = None
        self.tracking_btn_ref = None
        self.refocus_btn_ref = None
        self.init_ui()
    
    def init_ui(self):
        """初始化UI"""
        layout = QVBoxLayout()
        
        # 相机跟踪状态
        self.camera_status_label = QLabel("🎯 相机跟踪: 开启")
        self.camera_status_label.setStyleSheet("background-color: #e8f5e8; padding: 5px; border: 1px solid #4caf50; color: #2e7d32;")
        layout.addWidget(self.camera_status_label)
        
        # 相机控制按钮
        button_layout = QHBoxLayout()
        
        self.tracking_btn = QPushButton("🔄 切换跟踪")
        self.tracking_btn.setCheckable(True)
        self.tracking_btn.setChecked(True)
        self.tracking_btn.setStyleSheet("""
            QPushButton {
                background-color: #4caf50;
                color: white;
                font-weight: bold;
                padding: 8px;
                border: none;
                border-radius: 4px;
            }
            QPushButton:checked {
                background-color: #2196f3;
            }
        """)
        
        self.refocus_btn = QPushButton("🎯 重新聚焦")
        self.refocus_btn.setStyleSheet("""
            QPushButton {
                background-color: #ff9800;
                color: white;
                font-weight: bold;
                padding: 8px;
                border: none;
                border-radius: 4px;
            }
        """)
        
        button_layout.addWidget(self.tracking_btn)
        button_layout.addWidget(self.refocus_btn)
        layout.addLayout(button_layout)
        
        # 说明文字
        info_label = QLabel("• T键: 切换跟踪模式\n• L键: 重新聚焦机器人\n• 跟踪关闭时相机固定")
        info_label.setStyleSheet("color: #666; font-size: 10px; background-color: #f9f9f9; padding: 5px; border: 1px solid #eee;")
        layout.addWidget(info_label)
        
        self.setLayout(layout)
        
        # 存储按钮引用以便外部访问
        self.tracking_btn_ref = self.tracking_btn
        self.refocus_btn_ref = self.refocus_btn
        
        # 连接信号
        self.tracking_btn_ref.toggled.connect(self.on_camera_tracking_toggled)
        self.refocus_btn_ref.clicked.connect(self.on_camera_refocus)
    
    def on_camera_tracking_toggled(self, checked: bool):
        """相机跟踪开关切换"""
        status = "🎯 开启" if checked else "🔒 关闭"
        
        # 更新状态标签
        if self.camera_status_label:
            if checked:
                self.camera_status_label.setText(f"相机跟踪: {status}")
                self.camera_status_label.setStyleSheet("background-color: #e8f5e8; padding: 5px; border: 1px solid #4caf50; color: #2e7d32;")
            else:
                self.camera_status_label.setText(f"相机跟踪: {status}")
                self.camera_status_label.setStyleSheet("background-color: #ffebee; padding: 5px; border: 1px solid #f44336; color: #c62828;")
        
        print(f"{status} 相机跟踪")
        self.cameraTrackingToggled.emit(checked)
    
    def on_camera_refocus(self):
        """重新聚焦相机到机器人"""
        print("🎯 重新聚焦到机器人位置")
        self.cameraRefocus.emit()


class PoseControlGroup(QGroupBox):
    """姿态控制组"""
    
    # 信号定义
    poseSaved = Signal(str, dict)  # 姿态保存
    poseLoaded = Signal()          # 姿态加载
    
    def __init__(self, parent=None):
        super().__init__("🎯 姿态操作", parent)
        self.pose_info_label = None
        self.current_pose = {}
        self.init_ui()
    
    def init_ui(self):
        """初始化UI"""
        layout = QVBoxLayout()
        
        # 当前姿态信息
        self.pose_info_label = QLabel("当前姿态: 默认姿态")
        self.pose_info_label.setStyleSheet("background-color: #f5f5f5; padding: 5px; border: 1px solid #ddd;")
        layout.addWidget(self.pose_info_label)
        
        # 姿态操作按钮
        button_layout = QHBoxLayout()
        
        save_pose_btn = QPushButton("💾 保存姿态")
        save_pose_btn.clicked.connect(self.save_current_pose)
        
        load_pose_btn = QPushButton("📁 加载姿态")
        load_pose_btn.clicked.connect(self.load_pose_dialog)
        
        button_layout.addWidget(save_pose_btn)
        button_layout.addWidget(load_pose_btn)
        layout.addLayout(button_layout)
        
        self.setLayout(layout)
    
    def update_pose_info(self, pose_data: Dict[str, float]):
        """更新姿态信息显示"""
        self.current_pose = pose_data
        
        non_zero_angles = {name: math.degrees(angle) 
                          for name, angle in pose_data.items() 
                          if abs(angle) > 0.01}
        
        if non_zero_angles:
            angle_info = ", ".join([f"{name}: {angle:.1f}°" 
                                  for name, angle in non_zero_angles.items()])
            self.pose_info_label.setText(f"当前姿态: {angle_info}")
        else:
            self.pose_info_label.setText("当前姿态: 默认姿态")
    
    def save_current_pose(self):
        """保存当前姿态"""
        pose_name = f"pose_{len(self.current_pose)}"
        pose_data = {
            'name': pose_name,
            'joint_angles': self.current_pose.copy(),
            'timestamp': 'current'
        }
        self.poseSaved.emit(pose_name, pose_data)
        print(f"💾 姿态已保存: {pose_name}")
    
    def load_pose_dialog(self):
        """加载姿态对话框（占位）"""
        print("📁 姿态加载功能待实现")
        self.poseLoaded.emit()