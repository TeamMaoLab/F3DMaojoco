"""
ODogExample GUI模块 - 控制面板主组件

整合所有控制组件，提供统一的控制面板接口。
"""

import sys
import os
import math
from typing import Dict, Optional, Any
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QGridLayout, 
    QLabel, QSizePolicy
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
        self.pose_control = None
        
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
        
        # 标题
        title_label = QLabel("🎛️  8自由度关节控制")
        title_font = QFont("Arial", 14, QFont.Bold)
        title_label.setFont(title_font)
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("color: #333; padding: 10px;")
        main_layout.addWidget(title_label)
        
        # 全局控制区域
        self.global_control = GlobalControlGroup()
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
        
        # 精细控制选项
        self.precision_control = PrecisionControlGroup()
        main_layout.addWidget(self.precision_control)
          
        # 相机控制区域
        self.camera_control = CameraControlGroup()
        main_layout.addWidget(self.camera_control)
          
        # 姿态操作区域
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
    
    def on_joint_angle_changed(self, joint_name: str, angle: float):
        """关节角度改变处理"""
        # 更新当前姿态
        self.current_pose[joint_name] = angle
        
        # 同步到机器人模型
        if self.robot_model:
            self.robot_model.set_joint_angle(joint_name, angle)
        
        # 更新姿态信息
        if self.pose_control:
            self.pose_control.update_pose_info(self.current_pose)
        
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
        """所有关节重置"""
        print("🔙 所有关节重置")
        # 直接遍历所有关节控制组件
        for leg_group in self.leg_groups.values():
            for joint_widget in leg_group.joint_widgets.values():
                joint_widget.reset_to_default()
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
    
    def on_pose_loaded(self):
        """姿态加载处理"""
        print("📁 姿态加载功能待实现")
    
    def set_robot_model(self, robot_model: RobotModel):
        """设置机器人模型"""
        self.robot_model = robot_model
        print(f"🤖 控制面板已连接到机器人模型")
    
    def get_current_pose(self) -> Dict[str, float]:
        """获取当前姿态"""
        return self.current_pose.copy()
    
    def set_pose(self, pose_data: Dict[str, float]):
        """设置姿态"""
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