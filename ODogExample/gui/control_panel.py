"""
ODogExample GUI模块 - 控制面板主组件

整合所有控制组件，提供统一的控制面板接口。
"""

import sys
import os
import math
from typing import Dict, Optional, Any
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QGridLayout, QHBoxLayout,
    QLabel, QSizePolicy, QPushButton, QFrame
)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont

try:
    from ..core.robot_model import RobotModel
    from ..core.joint_mapping import JointMapping
    from .joint_controls import JointControlWidget, LegControlGroup
    from .global_controls import (
        GlobalControlGroup, PrecisionControlGroup, 
        CameraControlGroup, PoseControlGroup
    )
    from .pose_and_motion_control import PoseAndMotionControlGroup
except ImportError:
    # 如果相对导入失败，尝试绝对导入
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from core.robot_model import RobotModel
    from core.joint_mapping import JointMapping
    from gui.joint_controls import JointControlWidget, LegControlGroup
    from gui.global_controls import (
        GlobalControlGroup, PrecisionControlGroup, 
        CameraControlGroup, PoseControlGroup
    )


class ControlPanel(QWidget):
    """执行器控制主面板"""
    
    # 信号定义
    jointAngleChanged = Signal(str, float)  # 关节角度改变
    allJointsZero = Signal()                # 所有关节归零
    allJointsReset = Signal()               # 所有关节重置
    poseSaved = Signal(str, dict)           # 姿态保存
    
    def __init__(self, robot_model: Optional[RobotModel] = None, parent=None):
        super().__init__(parent)
        self.robot_model = robot_model
        self.joint_mapping = JointMapping()
        self.leg_groups: Dict[str, LegControlGroup] = {}
        
        # 当前姿态数据
        self.current_pose = self.joint_mapping.get_default_pose()
        
        # 控制组件引用
        self.global_control = None
        self.precision_control = None
        self.camera_control = None
        self.pose_and_motion_control = None
        
        # 设置尺寸策略，防止过度扩展
        self.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        self.setMaximumWidth(400)  # 限制最大宽度
        
        self.init_ui()
        self.setup_connections()
        
        print("🎛️  控制面板初始化完成")
    
    def init_ui(self):
        """初始化UI"""
        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(8)
        
        # 顶部控制按钮区域（两排）
        top_controls_layout = QVBoxLayout()
        top_controls_layout.setSpacing(5)
        
        # 第一排按钮：归零 重置 同步 重新聚焦
        row1_layout = QHBoxLayout()
        row1_layout.setSpacing(10)
        
        zero_all_btn = QPushButton("🔄 归零")
        zero_all_btn.setMinimumHeight(35)
        zero_all_btn.clicked.connect(self.zero_all_joints)
        
        reset_all_btn = QPushButton("🔙 重置")
        reset_all_btn.setMinimumHeight(35)
        reset_all_btn.clicked.connect(self.reset_all_joints)
        
        sync_btn = QPushButton("🤖 同步")
        sync_btn.setMinimumHeight(35)
        sync_btn.clicked.connect(self.sync_to_robot)
        
        refocus_btn = QPushButton("🎯 追焦")
        refocus_btn.setMinimumHeight(35)
        refocus_btn.clicked.connect(self.refocus_camera)
        
        row1_layout.addWidget(zero_all_btn)
        row1_layout.addWidget(reset_all_btn)
        row1_layout.addWidget(sync_btn)
        row1_layout.addWidget(refocus_btn)
        row1_layout.addStretch()
        
        # 第二排按钮：精细控制Switch 相机追踪Switch
        row2_layout = QHBoxLayout()
        row2_layout.setSpacing(10)
        
        # 精细控制开关
        precision_btn = QPushButton("📏 精细控制")
        precision_btn.setMinimumHeight(30)
        precision_btn.setMaximumHeight(30)
        precision_btn.setCheckable(True)
        precision_btn.setChecked(False)
        precision_btn.clicked.connect(self.toggle_global_precision)
        # 设置开关样式
        precision_btn.setStyleSheet("""
            QPushButton {
                background-color: #f0f0f0;
                border: 2px solid #ccc;
                border-radius: 15px;
                padding: 4px 12px;
                font-weight: bold;
                min-width: 100px;
                font-size: 12px;
            }
            QPushButton:checked {
                background-color: #4CAF50;
                color: white;
                border-color: #45a049;
            }
            QPushButton:hover {
                border-color: #888;
            }
            QPushButton:checked:hover {
                border-color: #45a049;
            }
        """)
        
        # 相机追踪开关
        tracking_btn = QPushButton("📷 相机追踪")
        tracking_btn.setMinimumHeight(30)
        tracking_btn.setMaximumHeight(30)
        tracking_btn.setCheckable(True)
        tracking_btn.setChecked(False)
        tracking_btn.clicked.connect(self.toggle_camera_tracking)
        # 设置开关样式
        tracking_btn.setStyleSheet("""
            QPushButton {
                background-color: #f0f0f0;
                border: 2px solid #ccc;
                border-radius: 15px;
                padding: 4px 12px;
                font-weight: bold;
                min-width: 100px;
                font-size: 12px;
            }
            QPushButton:checked {
                background-color: #2196F3;
                color: white;
                border-color: #1976D2;
            }
            QPushButton:hover {
                border-color: #888;
            }
            QPushButton:checked:hover {
                border-color: #1976D2;
            }
        """)
        
        row2_layout.addWidget(precision_btn)
        row2_layout.addWidget(tracking_btn)
        row2_layout.addStretch()
        
        top_controls_layout.addLayout(row1_layout)
        top_controls_layout.addLayout(row2_layout)
        
        # 添加分隔线
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        line.setStyleSheet("background-color: #ccc; margin: 5px 0;")
        
        main_layout.addLayout(top_controls_layout)
        main_layout.addWidget(line)
        
        # 全局控制区域（隐藏，因为功能已经移到顶部）
        self.global_control = GlobalControlGroup()
        self.global_control.hide()
        main_layout.addWidget(self.global_control)
        
        # 腿部控制网格
        legs_layout = QGridLayout()
        legs_layout.setSpacing(10)
        
        # 创建腿部控制组
        leg_positions = [
            ('左前腿', 'left_front'),
            ('右前腿', 'right_front'),
            ('左后腿', 'left_back'),
            ('右后腿', 'right_back')
        ]
        
        for i, (display_name, leg_key) in enumerate(leg_positions):
            row = i // 2
            col = i % 2
            
            joint_names = self.joint_mapping.get_leg_joints(leg_key)
            leg_group = LegControlGroup(display_name, joint_names, self.joint_mapping)
            self.leg_groups[leg_key] = leg_group
            
            legs_layout.addWidget(leg_group, row, col)
        
        main_layout.addLayout(legs_layout)
        
        # 精细控制选项（隐藏，因为功能已经移到顶部）
        self.precision_control = PrecisionControlGroup()
        self.precision_control.hide()
        main_layout.addWidget(self.precision_control)
          
        # 相机控制区域（隐藏，因为功能已经移到顶部）
        self.camera_control = CameraControlGroup()
        self.camera_control.hide()
        main_layout.addWidget(self.camera_control)
          
        # 姿态操作区域 - 从global_controls导入
        self.pose_control = PoseControlGroup()
        main_layout.addWidget(self.pose_control)
        
        main_layout.addStretch()
        self.setLayout(main_layout)
    
    def setup_connections(self):
        """设置信号连接"""
        # 连接所有关节控制组件的信号
        for leg_group in self.leg_groups.values():
            for joint_widget in leg_group.joint_widgets.values():
                joint_widget.valueChanged.connect(self.on_joint_angle_changed)
        
        # 连接全局控制信号
        if self.global_control:
            self.global_control.allJointsZero.connect(self.zero_all_joints)
            self.global_control.allJointsReset.connect(self.reset_all_joints)
            self.global_control.syncToRobot.connect(self.sync_to_robot)
        
        # 连接精细控制信号
        if self.precision_control:
            self.precision_control.globalPrecisionToggled.connect(self.toggle_global_precision)
        
        # 连接相机控制信号
        if self.camera_control:
            # 这些信号会由外部连接到查看器
            pass
        
        # 连接姿态控制信号
        if self.pose_control:
            self.pose_control.poseSaved.connect(self.on_pose_saved)
            self.pose_control.poseLoaded.connect(self.on_pose_loaded)
            self.pose_control.poseDeleted.connect(self.on_pose_deleted)
    
    def on_joint_angle_changed(self, joint_name: str, angle: float):
        """关节角度改变处理"""
        # 更新当前姿态
        self.current_pose[joint_name] = angle
        
        # 同步到机器人模型
        if self.robot_model:
            self.robot_model.set_joint_angle(joint_name, angle)
        
        # 更新姿态信息
        if self.pose_control:
            self.pose_control.update_current_pose(self.current_pose)
        
        # 发送信号
        self.jointAngleChanged.emit(joint_name, angle)
    
    def toggle_global_precision(self, checked: bool):
        """切换所有关节的精细控制模式"""
        print(f"🎛️ 全局精细控制: {'开启' if checked else '关闭'}")
        
        for leg_group in self.leg_groups.values():
            for joint_widget in leg_group.joint_widgets.values():
                joint_widget.precision_btn.setChecked(checked)
    
    def zero_all_joints(self):
        """所有关节归零"""
        print("🔄 所有关节归零")
        # 直接遍历所有关节控制组件
        for leg_group in self.leg_groups.values():
            for joint_widget in leg_group.joint_widgets.values():
                joint_widget.set_zero()
        self.allJointsZero.emit()
    
    def reset_all_joints(self):
        """所有关节重置 - 重新加载模型并重新开始模拟"""
        print("🔙 重置模拟 - 重新加载模型")
        
        # 重新加载模型
        if self.robot_model:
            success = self.robot_model.reload_model()
            if success:
                # 重置所有关节控制器到默认值
                for leg_group in self.leg_groups.values():
                    for joint_widget in leg_group.joint_widgets.values():
                        joint_widget.reset_to_default()
                
                # 重置当前姿态数据
                self.current_pose = self.joint_mapping.get_default_pose()
                
                # 更新姿态信息显示
                if self.pose_control:
                    self.pose_control.update_status(f"重置完成: {len(self.current_pose)} 个关节")
                
                print("✅ 模拟重置完成")
            else:
                print("❌ 模拟重置失败")
        else:
            print("⚠️  没有机器人模型，只重置关节控制器")
            # 如果没有模型，只重置关节控制器
            for leg_group in self.leg_groups.values():
                for joint_widget in leg_group.joint_widgets.values():
                    joint_widget.reset_to_default()
            
            # 重置当前姿态数据
            self.current_pose = self.joint_mapping.get_default_pose()
            
            # 更新姿态信息显示
            if self.pose_control:
                self.pose_control.update_status(f"重置完成: {len(self.current_pose)} 个关节")
        
        self.allJointsReset.emit()
    
    def sync_to_robot(self):
        """同步到机器人"""
        if not self.robot_model:
            print("⚠️  没有机器人模型可同步")
            return
        
        print("🤖 同步关节角度到机器人")
        for joint_name, angle in self.current_pose.items():
            success = self.robot_model.set_joint_angle(joint_name, angle)
            if success:
                print(f"  ✅ {joint_name}: {math.degrees(angle):.1f}°")
            else:
                print(f"  ❌ {joint_name}: 同步失败")
    
    def on_pose_saved(self, pose_name: str, pose_data: dict):
        """姿态保存处理"""
        print(f"💾 姿态已保存: {pose_name}")
        # 转发信号
        self.poseSaved.emit(pose_name, pose_data)
    
    def on_pose_loaded(self, pose_info: dict):
        """姿态加载处理"""
        pose_name = pose_info.get('name', '未知姿态')
        joint_angles = pose_info.get('joint_angles', {})
        
        print(f"📁 加载姿态: {pose_name}")
        
        # 应用姿态到关节控制器
        self.set_pose(joint_angles)
        
        # 更新姿态信息显示
        if self.pose_control:
            self.pose_control.update_current_pose(joint_angles)
    
    def on_pose_deleted(self, pose_name: str):
        """姿态删除处理"""
        print(f"🗑️ 姿态已删除: {pose_name}")
    
        
    def set_robot_model(self, robot_model: RobotModel):
        """设置机器人模型"""
        self.robot_model = robot_model
        print(f"🤖 控制面板已连接到机器人模型")
    
    def get_current_pose(self) -> Dict[str, float]:
        """获取当前姿态"""
        return self.current_pose.copy()
    
    def set_pose(self, pose_data: Dict[str, float]):
        """设置姿态"""
        # 更新当前姿态数据
        self.current_pose.update(pose_data)
        
        # 使用平滑过渡应用到机器人模型
        if self.robot_model:
            print(f"🎯 开始平滑过渡到姿态: {len(pose_data)} 个关节")
            self.robot_model.set_joint_angles(pose_data, smooth=True)
        
        # 更新UI控件显示
        for leg_group in self.leg_groups.values():
            leg_group.set_joint_angles(pose_data)
        
        print(f"🎯 姿态已设置: {len(pose_data)} 个关节")
    
    @property
    def tracking_btn_ref(self):
        """获取相机跟踪按钮引用"""
        return self.camera_control.tracking_btn_ref if self.camera_control else None
    
    @property
    def refocus_btn_ref(self):
        """获取重新聚焦按钮引用"""
        return self.camera_control.refocus_btn_ref if self.camera_control else None
    
    def toggle_global_precision(self, checked: bool):
        """切换全局精细控制"""
        print(f"🎛️ 全局精细控制: {'开启' if checked else '关闭'}")
        
        for leg_group in self.leg_groups.values():
            for joint_widget in leg_group.joint_widgets.values():
                joint_widget.precision_btn.setChecked(checked)
    
    def toggle_camera_tracking(self, checked: bool):
        """切换相机追踪"""
        if self.camera_control:
            # 找到相机追踪按钮并切换状态
            if hasattr(self.camera_control, 'tracking_btn_ref'):
                self.camera_control.tracking_btn_ref.setChecked(checked)
        print(f"📷 相机追踪: {'开启' if checked else '关闭'}")
    
    def refocus_camera(self):
        """重新聚焦相机"""
        if self.camera_control:
            # 触发重新聚焦
            if hasattr(self.camera_control, 'refocus_btn_ref'):
                self.camera_control.refocus_btn_ref.click()
        print("🎯 重新聚焦相机")


def create_control_panel(robot_model: Optional[RobotModel] = None) -> ControlPanel:
    """
    创建控制面板实例
    
    Args:
        robot_model: 机器人模型，可选
        
    Returns:
        ControlPanel: 控制面板实例
    """
    return ControlPanel(robot_model)


if __name__ == "__main__":
    """测试脚本"""
    from PySide6.QtWidgets import QApplication
    
    print("🎛️  ODogExample 控制面板测试")
    print("=" * 40)
    
    app = QApplication(sys.argv)
    
    # 创建控制面板
    panel = create_control_panel()
    panel.setWindowTitle("ODogExample 关节控制面板测试")
    panel.resize(800, 600)
    
    # 测试信号连接
    panel.jointAngleChanged.connect(lambda name, angle: 
        print(f"📐 关节角度变化: {name} = {math.degrees(angle):.1f}°"))
    
    panel.allJointsZero.connect(lambda: print("🔄 所有关节归零信号"))
    panel.allJointsReset.connect(lambda: print("🔙 所有关节重置信号"))
    
    panel.show()
    print("🎉 控制面板测试启动成功！")
    
    sys.exit(app.exec())