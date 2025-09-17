"""
ODogExample GUI模块 - 关节控制组件

提供单个关节和腿部控制的UI组件，包括滑块控制、精细控制等功能。
"""

import sys
import os
import math
from typing import Dict, List, Optional, Any, Tuple
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, 
    QLabel, QSlider, QPushButton, QFrame,
    QDoubleSpinBox
)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont

try:
    from ..core.joint_mapping import JointMapping
except ImportError:
    # 如果相对导入失败，尝试绝对导入
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
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