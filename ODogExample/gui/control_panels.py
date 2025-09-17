"""
ODogExample GUI模块 - 执行器控制面板

提供8自由度关节的实时控制界面，包括滑块控制、对称编辑、快速操作等功能。
"""

import sys
import os
import math
import numpy as np
from typing import Dict, List, Optional, Any, Tuple
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, 
    QLabel, QSlider, QPushButton, QGroupBox, QCheckBox,
    QSpinBox, QDoubleSpinBox, QFrame, QSizePolicy
)
from PySide6.QtCore import Qt, Signal, QTimer
from PySide6.QtGui import QFont, QPalette, QColor

try:
    from ..core.robot_model import RobotModel
    from ..core.joint_mapping import JointMapping
except ImportError:
    # 如果相对导入失败，尝试绝对导入
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from core.robot_model import RobotModel
    from core.joint_mapping import JointMapping


class JointControlWidget(QWidget):
    """单个关节控制组件"""
    
    valueChanged = Signal(str, float)  # 关节名称, 角度值
    
    def __init__(self, joint_name: str, joint_info: Dict, parent=None):
        super().__init__(parent)
        self.joint_name = joint_name
        self.joint_info = joint_info
        self.current_angle = 0.0
        
        self.init_ui()
        self.setup_connections()
    
    def init_ui(self):
        """初始化UI"""
        layout = QVBoxLayout()
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(3)
        
        # 关节名称、数值显示和精细控制按钮
        header_layout = QHBoxLayout()
        header_layout.setSpacing(5)
        
        # 关节名称标签
        name_label = QLabel(self.joint_info['description'])
        name_font = QFont("Arial", 9, QFont.Bold)
        name_label.setFont(name_font)
        name_label.setAlignment(Qt.AlignLeft)
        header_layout.addWidget(name_label)
        
        # 数值显示（放在标题后面）
        self.angle_label = QLabel("0.0°")
        self.angle_label.setStyleSheet("""
            QLabel {
                background-color: #f8f9fa;
                border: 1px solid #dee2e6;
                border-radius: 3px;
                padding: 2px 6px;
                font-size: 9px;
                font-weight: bold;
                color: #495057;
                min-width: 45px;
            }
        """)
        header_layout.addWidget(self.angle_label)
        
        header_layout.addStretch()  # 推送到右侧
        
        # 精细控制切换按钮
        self.precision_btn = QPushButton("📏")
        self.precision_btn.setMaximumWidth(28)
        self.precision_btn.setMaximumHeight(20)
        self.precision_btn.setCheckable(True)
        self.precision_btn.setChecked(False)
        self.precision_btn.setToolTip("切换精细控制模式")
        self.precision_btn.setStyleSheet("""
            QPushButton {
                background-color: #f8f9fa;
                border: 1px solid #dee2e6;
                border-radius: 3px;
                font-size: 10px;
                padding: 2px;
            }
            QPushButton:checked {
                background-color: #007bff;
                color: white;
                border-color: #007bff;
            }
        """)
        self.precision_btn.toggled.connect(self.toggle_precision_mode)
        header_layout.addWidget(self.precision_btn)
        
        layout.addLayout(header_layout)
        
        # 默认控制：角度滑块
        self.angle_slider = QSlider(Qt.Horizontal)
        self.angle_slider.setRange(-90, 90)  # ±90度
        self.angle_slider.setValue(0)
        self.angle_slider.setTickPosition(QSlider.TicksBelow)
        self.angle_slider.setTickInterval(30)
        layout.addWidget(self.angle_slider)
        
        # 精细控制：数值输入框（默认隐藏）
        self.precision_widget = QWidget()
        precision_layout = QVBoxLayout(self.precision_widget)
        precision_layout.setContentsMargins(0, 0, 0, 0)
        precision_layout.setSpacing(3)
        
        # 数值输入框
        input_layout = QHBoxLayout()
        input_layout.setSpacing(5)
        
        self.angle_input = QDoubleSpinBox()
        self.angle_input.setRange(-90, 90)
        self.angle_input.setValue(0)
        self.angle_input.setSuffix("°")
        self.angle_input.setSingleStep(0.1)  # 精确到0.1度
        self.angle_input.setDecimals(1)
        self.angle_input.setStyleSheet("""
            QDoubleSpinBox {
                border: 1px solid #dee2e6;
                border-radius: 3px;
                padding: 2px;
                font-size: 9px;
            }
        """)
        input_layout.addWidget(self.angle_input)
        
        # 精确控制快捷按钮
        precise_zero_btn = QPushButton("0°")
        precise_zero_btn.setMaximumWidth(30)
        precise_zero_btn.setMaximumHeight(22)
        precise_zero_btn.setStyleSheet("""
            QPushButton {
                background-color: #e9ecef;
                border: 1px solid #dee2e6;
                border-radius: 3px;
                font-size: 8px;
                padding: 1px;
            }
            QPushButton:hover {
                background-color: #dee2e6;
            }
        """)
        precise_zero_btn.clicked.connect(lambda: self.angle_input.setValue(0.0))
        
        precise_45_btn = QPushButton("45°")
        precise_45_btn.setMaximumWidth(30)
        precise_45_btn.setMaximumHeight(22)
        precise_45_btn.setStyleSheet(precise_zero_btn.styleSheet())
        precise_45_btn.clicked.connect(lambda: self.angle_input.setValue(45.0))
        
        precise_neg45_btn = QPushButton("-45°")
        precise_neg45_btn.setMaximumWidth(32)
        precise_neg45_btn.setMaximumHeight(22)
        precise_neg45_btn.setStyleSheet(precise_zero_btn.styleSheet())
        precise_neg45_btn.clicked.connect(lambda: self.angle_input.setValue(-45.0))
        
        input_layout.addWidget(precise_zero_btn)
        input_layout.addWidget(precise_45_btn)
        input_layout.addWidget(precise_neg45_btn)
        input_layout.addStretch()
        
        precision_layout.addLayout(input_layout)
        
        layout.addWidget(self.precision_widget)
        self.precision_widget.hide()  # 默认隐藏精细控制
        
        # 常用快捷操作按钮
        button_layout = QHBoxLayout()
        button_layout.setSpacing(5)
        
        zero_btn = QPushButton("归零")
        zero_btn.setMaximumWidth(45)
        zero_btn.setMaximumHeight(22)
        zero_btn.setStyleSheet("""
            QPushButton {
                background-color: #28a745;
                color: white;
                border: none;
                border-radius: 3px;
                font-size: 8px;
                font-weight: bold;
                padding: 2px;
            }
            QPushButton:hover {
                background-color: #218838;
            }
        """)
        zero_btn.clicked.connect(self.set_zero)
        
        reset_btn = QPushButton("重置")
        reset_btn.setMaximumWidth(45)
        reset_btn.setMaximumHeight(22)
        reset_btn.setStyleSheet("""
            QPushButton {
                background-color: #6c757d;
                color: white;
                border: none;
                border-radius: 3px;
                font-size: 8px;
                font-weight: bold;
                padding: 2px;
            }
            QPushButton:hover {
                background-color: #5a6268;
            }
        """)
        reset_btn.clicked.connect(self.reset_to_default)
        
        button_layout.addStretch()
        button_layout.addWidget(zero_btn)
        button_layout.addWidget(reset_btn)
        layout.addLayout(button_layout)
        
        self.setLayout(layout)
    
    def setup_connections(self):
        """设置信号连接"""
        self.angle_slider.valueChanged.connect(self.on_slider_changed)
        self.angle_input.valueChanged.connect(self.on_input_changed)
    
    def toggle_precision_mode(self, checked: bool):
        """切换精细控制模式"""
        if checked:
            # 切换到精细控制：隐藏滑块，显示数值输入
            self.angle_slider.hide()
            self.precision_widget.show()
            # 同步当前数值
            current_deg = math.degrees(self.current_angle)
            self.angle_input.setValue(current_deg)
        else:
            # 切换到普通控制：显示滑块，隐藏数值输入
            self.angle_slider.show()
            self.precision_widget.hide()
            # 同步当前数值
            current_deg = math.degrees(self.current_angle)
            self.angle_slider.setValue(int(current_deg))
    
    def on_slider_changed(self, value):
        """滑块值改变"""
        angle_deg = float(value)
        angle_rad = math.radians(angle_deg)
        self.set_angle(angle_rad)
    
    def on_input_changed(self, value):
        """输入框值改变"""
        angle_deg = float(value)
        angle_rad = math.radians(angle_deg)
        self.set_angle(angle_rad)
    
    def set_angle(self, angle_rad: float, emit_signal: bool = True):
        """设置关节角度"""
        # 验证角度范围
        is_valid, clamped_angle = self._validate_angle(angle_rad)
        
        if not is_valid:
            print(f"⚠️  {self.joint_name}: 角度 {math.degrees(angle_rad):.1f}° 超出范围，已限制为 {math.degrees(clamped_angle):.1f}°")
        
        self.current_angle = clamped_angle
        angle_deg = math.degrees(clamped_angle)
        
        # 更新角度显示
        self.angle_label.setText(f"{angle_deg:.1f}°")
        
        # 根据当前模式更新对应的控件
        if self.precision_btn.isChecked():
            # 精细控制模式：更新数值输入框
            self.angle_input.blockSignals(True)
            self.angle_input.setValue(angle_deg)
            self.angle_input.blockSignals(False)
        else:
            # 普通控制模式：更新滑块
            self.angle_slider.blockSignals(True)
            self.angle_slider.setValue(int(angle_deg))
            self.angle_slider.blockSignals(False)
        
        # 发送信号（可选）
        if emit_signal:
            self.valueChanged.emit(self.joint_name, clamped_angle)
    
    def _validate_angle(self, angle_rad: float) -> Tuple[bool, float]:
        """验证角度范围"""
        min_angle, max_angle = self.joint_info['movement_range']
        clamped_angle = max(min_angle, min(max_angle, angle_rad))
        is_valid = (min_angle <= angle_rad <= max_angle)
        return is_valid, clamped_angle
    
    def set_zero(self):
        """归零"""
        self.set_angle(0.0)
    
    def reset_to_default(self):
        """重置为默认值"""
        self.set_angle(self.joint_info['default_angle'])
    
    def get_current_angle(self) -> float:
        """获取当前角度"""
        return self.current_angle


class LegControlGroup(QWidget):
    """腿部控制组"""
    
    def __init__(self, leg_name: str, joint_names: List[str], joint_mapping: JointMapping, parent=None):
        super().__init__(parent)
        self.leg_name = leg_name
        self.joint_names = joint_names
        self.joint_mapping = joint_mapping
        self.joint_widgets: Dict[str, JointControlWidget] = {}
        
        self.init_ui()
    
    def init_ui(self):
        """初始化UI"""
        layout = QVBoxLayout()
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(5)
        
        # 腿部标题
        title_label = QLabel(self.leg_name)
        title_font = QFont("Arial", 10, QFont.Bold)
        title_label.setFont(title_font)
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("""
            QLabel {
                background-color: #e3f2fd;
                color: #1976d2;
                border: 1px solid #bbdefb;
                border-radius: 4px;
                padding: 4px 8px;
                font-weight: bold;
            }
        """)
        layout.addWidget(title_label)
        
        # 分隔线
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        layout.addWidget(line)
        
        # 关节控制组件
        for joint_name in self.joint_names:
            joint_info = self.joint_mapping.get_joint_info(joint_name)
            if joint_info:
                joint_widget = JointControlWidget(joint_name, joint_info)
                self.joint_widgets[joint_name] = joint_widget
                layout.addWidget(joint_widget)
        
        # 移除了腿部快捷操作按钮，统一使用全局控制
        
        layout.addStretch()
        self.setLayout(layout)
    
        
    def set_joint_angles(self, angles: Dict[str, float]):
        """设置关节角度"""
        for joint_name, angle in angles.items():
            if joint_name in self.joint_widgets:
                self.joint_widgets[joint_name].set_angle(angle)
    
    def get_joint_angles(self) -> Dict[str, float]:
        """获取关节角度"""
        return {name: widget.get_current_angle() 
                for name, widget in self.joint_widgets.items()}


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
        global_group = self.create_global_control_group()
        main_layout.addWidget(global_group)
        
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
        precision_group = self.create_precision_control_group()
        main_layout.addWidget(precision_group)
        
        # 对称控制选项
        symmetry_group = self.create_symmetry_control_group()
        main_layout.addWidget(symmetry_group)
        
        # 姿态操作区域
        pose_group = self.create_pose_control_group()
        main_layout.addWidget(pose_group)
        
        main_layout.addStretch()
        self.setLayout(main_layout)
    
    def create_global_control_group(self) -> QGroupBox:
        """创建全局控制组"""
        group = QGroupBox("🌐 全局控制")
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
        
        group.setLayout(layout)
        return group
    
    def create_precision_control_group(self) -> QGroupBox:
        """创建精细控制组"""
        group = QGroupBox("🎛️ 精细控制")
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
        
        group.setLayout(layout)
        return group
    
    def create_symmetry_control_group(self) -> QGroupBox:
        """创建对称控制组"""
        group = QGroupBox("🔄 对称控制")
        layout = QVBoxLayout()
        
        # 对称编辑开关
        self.symmetry_checkbox = QCheckBox("启用对称编辑")
        self.symmetry_checkbox.setToolTip("编辑一个关节时，自动同步编辑对称关节")
        layout.addWidget(self.symmetry_checkbox)
        
        # 对称模式说明
        info_label = QLabel("• 左前腿 ↔ 右前腿\n• 左后腿 ↔ 右后腿\n• 髋关节 ↔ 髋关节\n• 膝关节 ↔ 膝关节")
        info_label.setStyleSheet("color: #666; font-size: 10px;")
        layout.addWidget(info_label)
        
        group.setLayout(layout)
        return group
    
    def create_pose_control_group(self) -> QGroupBox:
        """创建姿态控制组"""
        group = QGroupBox("🎯 姿态操作")
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
        
        group.setLayout(layout)
        return group
    
    def setup_connections(self):
        """设置信号连接"""
        # 连接所有关节控制组件的信号
        for leg_group in self.leg_groups.values():
            for joint_widget in leg_group.joint_widgets.values():
                joint_widget.valueChanged.connect(self.on_joint_angle_changed)
    
    def on_joint_angle_changed(self, joint_name: str, angle: float):
        """关节角度改变处理"""
        # 更新当前姿态
        self.current_pose[joint_name] = angle
        
        # 同步到机器人模型
        if self.robot_model:
            self.robot_model.set_joint_angle(joint_name, angle)
        
        # 对称编辑（避免递归调用）
        if self.symmetry_checkbox.isChecked():
            symmetric_joint = self.joint_mapping.get_symmetric_joint(joint_name)
            if symmetric_joint and symmetric_joint in self.current_pose:
                # 设置对称关节角度（不发送信号，避免重复触发）
                for leg_group in self.leg_groups.values():
                    if symmetric_joint in leg_group.joint_widgets:
                        leg_group.joint_widgets[symmetric_joint].set_angle(angle, emit_signal=False)
                        break
        
        # 更新姿态信息
        self.update_pose_info()
        
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
    
    def update_pose_info(self):
        """更新姿态信息显示"""
        non_zero_angles = {name: math.degrees(angle) 
                          for name, angle in self.current_pose.items() 
                          if abs(angle) > 0.01}
        
        if non_zero_angles:
            angle_info = ", ".join([f"{name}: {angle:.1f}°" 
                                  for name, angle in non_zero_angles.items()])
            self.pose_info_label.setText(f"当前姿态: {angle_info}")
        else:
            self.pose_info_label.setText("当前姿态: 默认姿态")
    
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