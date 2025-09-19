"""
ODogExample GUI模块 - 保存姿态对话框

提供保存姿态时的简化输入界面，只需要输入姿态名称。
"""

import math
from typing import Dict, List, Optional
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, 
    QPushButton, QDialogButtonBox, QMessageBox, QFrame
)
from PySide6.QtCore import Qt, Signal

from .pose_manager import get_pose_manager


class PoseSaveDialog(QDialog):
    """保存姿态对话框"""
    
    def __init__(self, current_pose_data: Dict[str, float], 
                 existing_names: List[str] = None, parent=None):
        """
        初始化保存姿态对话框
        
        Args:
            current_pose_data: 当前姿态的关节数据
            existing_names: 已存在的姿态名称列表
            parent: 父窗口
        """
        super().__init__(parent)
        self.current_pose_data = current_pose_data
        self.existing_names = existing_names or []
        self.pose_manager = get_pose_manager()
        
        self.setWindowTitle("💾 保存姿态")
        self.setModal(True)
        self.setFixedSize(300, 150)
        
        self.init_ui()
        self.load_current_data()
        
        print("💾 保存姿态对话框已创建")
    
    def init_ui(self):
        """初始化UI"""
        layout = QVBoxLayout()
        layout.setSpacing(15)
        layout.setContentsMargins(20, 20, 20, 20)
        
        # 姿态名称
        name_layout = QHBoxLayout()
        name_label = QLabel("姿态名称:")
        name_label.setStyleSheet("font-weight: bold;")
        
        self.name_edit = QLineEdit()
        self.name_edit.setPlaceholderText("请输入姿态名称...")
        
        name_layout.addWidget(name_label)
        name_layout.addWidget(self.name_edit)
        layout.addLayout(name_layout)
        
        # 按钮
        button_box = QDialogButtonBox(
            QDialogButtonBox.Save | QDialogButtonBox.Cancel
        )
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        
        # 自定义按钮文本和样式
        save_btn = button_box.button(QDialogButtonBox.Save)
        save_btn.setText("保存")
        save_btn.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                font-weight: bold;
                padding: 8px 16px;
                border: none;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
        """)
        
        cancel_btn = button_box.button(QDialogButtonBox.Cancel)
        cancel_btn.setText("取消")
        
        layout.addWidget(button_box)
        
        self.setLayout(layout)
        
        # 设置名称输入框焦点
        self.name_edit.setFocus()
    
    def load_current_data(self):
        """加载当前数据"""
        # 这里可以预填一些默认值
        if self.current_pose_data:
            # 检查是否为默认姿态
            all_zero = all(abs(angle) < 0.01 for angle in self.current_pose_data.values())
            if all_zero:
                self.name_edit.setText("默认姿态")
    
    def get_pose_data(self) -> Dict:
        """
        获取对话框中的姿态数据
        
        Returns:
            Dict: 包含姿态信息的字典
        """
        name = self.name_edit.text().strip()
        
        return {
            'name': name,
            'description': '',
            'tags': [],
            'joint_angles': self.current_pose_data.copy()
        }
    
    def validate_input(self) -> bool:
        """
        验证输入数据
        
        Returns:
            bool: 验证是否通过
        """
        name = self.name_edit.text().strip()
        
        # 检查名称是否为空
        if not name:
            QMessageBox.warning(self, "输入错误", "请输入姿态名称！")
            self.name_edit.setFocus()
            return False
        
        # 检查名称是否已存在
        if name in self.existing_names:
            reply = QMessageBox.question(
                self, "名称重复", 
                f"姿态名称 '{name}' 已存在，是否覆盖？",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No
            )
            if reply == QMessageBox.No:
                self.name_edit.setFocus()
                self.name_edit.selectAll()
                return False
        
        return True
    
    def accept(self):
        """确认保存"""
        if not self.validate_input():
            return
        
        pose_data = self.get_pose_data()
        
        print(f"💾 准备保存姿态: {pose_data['name']}")
        print(f"💾 关节角度数据: {pose_data['joint_angles']}")
        print(f"💾 数据类型: {type(pose_data['joint_angles'])}")
        print(f"💾 数据长度: {len(pose_data['joint_angles']) if pose_data['joint_angles'] else 0}")
        
        # 验证数据不为空
        if not pose_data['joint_angles'] or len(pose_data['joint_angles']) == 0:
            QMessageBox.critical(self, "数据错误", 
                               f"关节角度数据为空或无效！\n数据类型: {type(pose_data['joint_angles'])}\n数据内容: {pose_data['joint_angles']}")
            return
        
        # 保存到姿态管理器
        success = self.pose_manager.save_pose(
            pose_data['name'],
            pose_data['joint_angles'],
            pose_data['description'],
            pose_data['tags']
        )
        
        if success:
            QMessageBox.information(self, "保存成功", 
                                   f"姿态 '{pose_data['name']}' 保存成功！")
            super().accept()
        else:
            QMessageBox.critical(self, "保存失败", 
                               f"姿态保存失败！\n\n可能的原因:\n1. 姿态名称已存在\n2. 关节角度数据无效\n3. 文件权限问题\n\n请检查后重试。")
    
    def keyPressEvent(self, event):
        """键盘事件处理"""
        if event.key() == Qt.Key_Enter or event.key() == Qt.Key_Return:
            self.accept()
        elif event.key() == Qt.Key_Escape:
            self.reject()
        else:
            super().keyPressEvent(event)


def show_save_pose_dialog(current_pose_data: Dict[str, float], 
                         existing_names: List[str] = None, 
                         parent=None) -> Optional[Dict]:
    """
    显示保存姿态对话框
    
    Args:
        current_pose_data: 当前姿态的关节数据
        existing_names: 已存在的姿态名称列表
        parent: 父窗口
        
    Returns:
        Optional[Dict]: 保存的姿态数据，取消返回None
    """
    dialog = PoseSaveDialog(current_pose_data, existing_names, parent)
    
    if dialog.exec() == QDialog.Accepted:
        return dialog.get_pose_data()
    
    return None


if __name__ == "__main__":
    """测试脚本"""
    from PySide6.QtWidgets import QApplication
    
    print("💾 保存姿态对话框测试")
    print("=" * 40)
    
    app = QApplication(sys.argv)
    
    # 测试数据
    test_pose_data = {
        'xuan_zhuan_1': 0.0,
        'xuan_zhuan_2': 0.0,
        'xuan_zhuan_3': 0.0,
        'xuan_zhuan_4': 0.0,
        'tui_1': -0.5,
        'tui_2': -0.5,
        'tui_3': -0.5,
        'tui_4': -0.5
    }
    
    existing_names = ['默认姿态', '站立姿态', '趴下姿态']
    
    # 显示对话框
    result = show_save_pose_dialog(test_pose_data, existing_names)
    
    if result:
        print("✅ 姿态保存成功:")
        print(f"  名称: {result['name']}")
        print(f"  描述: {result['description']}")
        print(f"  标签: {result['tags']}")
        print(f"  关节数量: {len(result['joint_angles'])}")
    else:
        print("❌ 用户取消了保存")
    
    print("🎉 保存姿态对话框测试完成！")