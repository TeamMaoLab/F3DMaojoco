"""
ODogExample GUI模块 - 全局控制组件

提供全局控制、相机控制、姿态控制等功能面板。
"""

import math
import sys
import os
from typing import Dict, List, Optional
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, 
    QPushButton, QLabel, QListWidget, QListWidgetItem,
    QMessageBox, QFrame
)
from PySide6.QtCore import Signal, Qt
from PySide6.QtGui import QFont

try:
    from ..core.robot_model import RobotModel
    from ..core.joint_mapping import JointMapping
    from .pose_save_dialog import show_save_pose_dialog
    from ..pose_manager import get_pose_manager
except ImportError:
    # 如果相对导入失败，尝试绝对导入
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from core.robot_model import RobotModel
    from core.joint_mapping import JointMapping
    from gui.pose_save_dialog import show_save_pose_dialog
    from gui.pose_manager import get_pose_manager


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
    poseLoaded = Signal(dict)      # 姿态加载
    poseDeleted = Signal(str)      # 姿态删除
    
    def __init__(self, parent=None):
        super().__init__("🎯 姿态操作", parent)
        self.current_pose = {}
        self.pose_manager = get_pose_manager()
        self.pose_list_widget = None
        self.init_ui()
        self.load_pose_list()
        
        print("🎯 姿态控制组初始化完成")
    
    def init_ui(self):
        """初始化UI"""
        layout = QVBoxLayout()
        layout.setSpacing(8)
        
        # 操作按钮
        button_layout = QHBoxLayout()
        button_layout.setSpacing(8)
        
        self.save_btn = QPushButton("💾 保存当前")
        self.save_btn.setMinimumHeight(30)
        self.save_btn.clicked.connect(self.save_current_pose)
        
        self.load_btn = QPushButton("📁 加载选中")
        self.load_btn.setMinimumHeight(30)
        self.load_btn.clicked.connect(self.load_selected_pose)
        
        self.delete_btn = QPushButton("🗑️ 删除选中")
        self.delete_btn.setMinimumHeight(30)
        self.delete_btn.clicked.connect(self.delete_selected_pose)
        
        self.refresh_btn = QPushButton("🔄 刷新")
        self.refresh_btn.setMinimumHeight(30)
        self.refresh_btn.clicked.connect(self.refresh_pose_list)
        
        button_layout.addWidget(self.save_btn)
        button_layout.addWidget(self.load_btn)
        button_layout.addWidget(self.delete_btn)
        button_layout.addWidget(self.refresh_btn)
        
        layout.addLayout(button_layout)
        
        # 分隔线
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        line.setStyleSheet("background-color: #ccc; margin: 5px 0;")
        layout.addWidget(line)
        
        # 姿态列表
        list_label = QLabel("已保存的姿态:")
        list_label.setStyleSheet("font-weight: bold; color: #333;")
        layout.addWidget(list_label)
        
        self.pose_list_widget = QListWidget()
        self.pose_list_widget.setMinimumHeight(150)
        self.pose_list_widget.setMaximumHeight(200)
        
        # 设置列表样式
        self.pose_list_widget.setStyleSheet("""
            QListWidget {
                border: 1px solid #ddd;
                border-radius: 4px;
                background-color: #f9f9f9;
                padding: 5px;
            }
            QListWidget::item {
                padding: 8px;
                border-bottom: 1px solid #eee;
            }
            QListWidget::item:selected {
                background-color: #e3f2fd;
                color: #1976d2;
            }
            QListWidget::item:hover {
                background-color: #f5f5f5;
            }
        """)
        
        # 双击加载
        self.pose_list_widget.itemDoubleClicked.connect(self.on_item_double_clicked)
        
        layout.addWidget(self.pose_list_widget)
        
        # 当前姿态信息
        info_layout = QHBoxLayout()
        info_label = QLabel("当前状态:")
        info_label.setStyleSheet("font-weight: bold; color: #333;")
        
        self.status_label = QLabel("就绪")
        self.status_label.setStyleSheet("color: #666; font-size: 12px;")
        
        info_layout.addWidget(info_label)
        info_layout.addWidget(self.status_label)
        info_layout.addStretch()
        
        layout.addLayout(info_layout)
        
        self.setLayout(layout)
    
    def load_pose_list(self):
        """加载姿态列表"""
        try:
            poses = self.pose_manager.get_all_poses()
            self.pose_list_widget.clear()
            
            for pose_name, pose_data in poses.items():
                item_text = f"{pose_name}"
                if pose_data.description:
                    item_text += f" - {pose_data.description}"
                
                item = QListWidgetItem(item_text)
                item.setData(Qt.UserRole, pose_name)
                
                # 添加标签信息
                if pose_data.tags:
                    tag_text = f"[{', '.join(pose_data.tags)}]"
                    item.setToolTip(f"{pose_name}\n描述: {pose_data.description}\n标签: {tag_text}")
                
                self.pose_list_widget.addItem(item)
            
            self.update_status(f"加载了 {len(poses)} 个姿态")
            
        except Exception as e:
            self.update_status(f"加载姿态列表失败: {e}")
            print(f"❌ 加载姿态列表失败: {e}")
    
    def save_current_pose(self):
        """保存当前姿态"""
        if not self.current_pose:
            QMessageBox.warning(self, "警告", "没有当前姿态数据可保存！")
            return
        
        # 获取已存在的姿态名称
        existing_names = list(self.pose_manager.get_pose_names())
        
        # 显示保存对话框
        result = show_save_pose_dialog(self.current_pose, existing_names, self)
        
        if result:
            # 保存成功，刷新列表
            self.load_pose_list()
            self.update_status(f"已保存姿态: {result['name']}")
            
            # 发送信号
            self.poseSaved.emit(result['name'], result)
    
    def load_selected_pose(self):
        """加载选中的姿态"""
        current_item = self.pose_list_widget.currentItem()
        if not current_item:
            QMessageBox.information(self, "提示", "请先选择要加载的姿态！")
            return
        
        pose_name = current_item.data(Qt.UserRole)
        self.load_pose_by_name(pose_name)
    
    def load_pose_by_name(self, pose_name: str):
        """根据名称加载姿态"""
        try:
            joint_angles = self.pose_manager.load_pose(pose_name)
            if joint_angles:
                self.update_status(f"已加载姿态: {pose_name}")
                
                # 发送信号
                pose_info = {
                    'name': pose_name,
                    'joint_angles': joint_angles
                }
                self.poseLoaded.emit(pose_info)
            else:
                QMessageBox.warning(self, "加载失败", f"无法加载姿态: {pose_name}")
                
        except Exception as e:
            QMessageBox.critical(self, "错误", f"加载姿态时发生错误: {e}")
    
    def delete_selected_pose(self):
        """删除选中的姿态"""
        current_item = self.pose_list_widget.currentItem()
        if not current_item:
            QMessageBox.information(self, "提示", "请先选择要删除的姿态！")
            return
        
        pose_name = current_item.data(Qt.UserRole)
        
        # 确认删除
        reply = QMessageBox.question(
            self, "确认删除", 
            f"确定要删除姿态 '{pose_name}' 吗？\n此操作不可恢复！",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            try:
                success = self.pose_manager.delete_pose(pose_name)
                if success:
                    self.load_pose_list()
                    self.update_status(f"已删除姿态: {pose_name}")
                    
                    # 发送信号
                    self.poseDeleted.emit(pose_name)
                else:
                    QMessageBox.warning(self, "删除失败", f"无法删除姿态: {pose_name}")
                    
            except Exception as e:
                QMessageBox.critical(self, "错误", f"删除姿态时发生错误: {e}")
    
    def on_item_double_clicked(self, item):
        """双击列表项处理"""
        pose_name = item.data(Qt.UserRole)
        self.load_pose_by_name(pose_name)
    
    def refresh_pose_list(self):
        """刷新姿态列表"""
        self.load_pose_list()
        self.update_status("姿态列表已刷新")
    
    def update_current_pose(self, pose_data: Dict[str, float]):
        """更新当前姿态数据"""
        self.current_pose = pose_data.copy()
        
        # 更新状态显示
        non_zero_count = sum(1 for angle in pose_data.values() if abs(angle) > 0.01)
        self.update_status(f"当前姿态: {non_zero_count} 个非零关节")
    
    def update_status(self, message: str):
        """更新状态显示"""
        if self.status_label:
            self.status_label.setText(message)
    
    def get_selected_pose_name(self) -> Optional[str]:
        """获取当前选中的姿态名称"""
        current_item = self.pose_list_widget.currentItem()
        if current_item:
            return current_item.data(Qt.UserRole)
        return None
    
    def set_buttons_enabled(self, enabled: bool):
        """设置按钮启用状态"""
        self.save_btn.setEnabled(enabled)
        self.load_btn.setEnabled(enabled)
        self.delete_btn.setEnabled(enabled)
        self.refresh_btn.setEnabled(enabled)