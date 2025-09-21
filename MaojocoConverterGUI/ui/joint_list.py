"""
关节列表组件 - 用于模型预览界面

从component_positions.json解析关节信息，提供关节选择功能。
选中关节后，3D预览图中关联的两个实体变为特殊颜色，取消选择恢复原色。
"""

from typing import List, Dict, Any, Optional, Set, Tuple
from pathlib import Path
from dataclasses import dataclass
import json

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
    QListWidget, QListWidgetItem, QGroupBox,
    QSplitter, QTextEdit, QFrame, QPushButton,
    QTabWidget, QComboBox
)
from PySide6.QtCore import Signal, Qt
from PySide6.QtGui import QFont

from ..utils.logger import logger
from ..core.domain_types import JointInfo, JointConnection, JointGeometry, JointType


@dataclass
class JointDisplayInfo:
    """关节显示信息"""
    name: str
    joint_type: JointType
    connection: JointConnection
    geometry: JointGeometry
    is_suppressed: bool
    is_light_bulb_on: bool
    component_one: str  # 关联的第一个组件名称
    component_two: str  # 关联的第二个组件名称
    
    @classmethod
    def from_json(cls, data: Dict[str, Any]) -> 'JointDisplayInfo':
        """从JSON数据创建关节显示信息"""
        # 解析连接信息
        connection_data = data.get('connection', {})
        connection = JointConnection.from_dict(connection_data)
        
        # 解析几何信息
        geometry_data = data.get('geometry', {})
        geometry = JointGeometry.from_dict(geometry_data)
        
        # 解析关节类型
        joint_type_str = data.get('joint_type', 'rigid')
        try:
            joint_type = JointType(joint_type_str)
        except ValueError:
            joint_type = JointType.RIGID
            logger.warning(f"未知的关节类型: {joint_type_str}，默认为RIGID")
        
        return cls(
            name=data.get('name', ''),
            joint_type=joint_type,
            connection=connection,
            geometry=geometry,
            is_suppressed=data.get('is_suppressed', False),
            is_light_bulb_on=data.get('is_light_bulb_on', True),
            component_one=connection.occurrence_one_component or connection.occurrence_one_name or '',
            component_two=connection.occurrence_two_component or connection.occurrence_two_name or ''
        )


class JointListWidget(QWidget):
    """关节列表组件"""
    
    # 信号定义
    joint_selected = Signal(str, bool)  # 关节选择状态变化 (关节名, 是否选中)
    joint_hovered = Signal(str)         # 鼠标悬停在关节上
    components_highlight = Signal(object, str)  # 组件高亮请求 (组件名列表, 颜色)
    
    def __init__(self, parent=None):
        """初始化关节列表组件"""
        super().__init__(parent)
        self._joints: List[JointDisplayInfo] = []
        self._selected_joints: Set[str] = set()
        self._current_project: Optional[Path] = None
        self._filter_type: Optional[JointType] = None
        self._setup_ui()
    
    def _setup_ui(self):
        """设置UI界面"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)
        
        # 标题区域
        title_layout = QHBoxLayout()
        
        title_label = QLabel("关节列表")
        title_font = QFont()
        title_font.setBold(True)
        title_label.setFont(title_font)
        title_label.setStyleSheet("font-size: 14px; color: #333;")
        title_layout.addWidget(title_label)
        
        title_layout.addStretch()
        
        # 清除选中按钮
        clear_button = QPushButton("清除选中")
        clear_button.setMaximumWidth(80)
        clear_button.setStyleSheet("""
            QPushButton {
                background-color: #f8f9fa;
                border: 1px solid #dee2e6;
                border-radius: 4px;
                padding: 4px 8px;
                font-size: 12px;
            }
            QPushButton:hover {
                background-color: #e9ecef;
            }
            QPushButton:pressed {
                background-color: #dee2e6;
            }
        """)
        clear_button.clicked.connect(self._on_clear_button_clicked)
        title_layout.addWidget(clear_button)
        
        layout.addLayout(title_layout)
        
        # 过滤器区域
        filter_layout = QHBoxLayout()
        filter_label = QLabel("关节类型:")
        filter_layout.addWidget(filter_label)
        
        self.filter_combo = QComboBox()
        self.filter_combo.addItem("全部", None)
        for joint_type in JointType:
            self.filter_combo.addItem(self._get_joint_type_display_name(joint_type), joint_type)
        self.filter_combo.currentIndexChanged.connect(self._on_filter_changed)
        filter_layout.addWidget(self.filter_combo)
        
        filter_layout.addStretch()
        layout.addLayout(filter_layout)
        
        # 创建分割器
        splitter = QSplitter(Qt.Vertical)
        layout.addWidget(splitter)
        
        # 关节列表
        self.joint_list = QListWidget()
        self.joint_list.setSelectionMode(QListWidget.MultiSelection)
        self.joint_list.setAlternatingRowColors(True)
        
        # 设置选中样式 - 橙色背景表示选中
        self.joint_list.setStyleSheet("""
            QListWidget::item:selected {
                background-color: #ff8c00;
                color: #333333;
                border: 2px solid #ff6300;
                font-weight: bold;
            }
            QListWidget::item:hover {
                background-color: #fff5cc;
            }
        """)
        
        # 连接信号
        self.joint_list.itemClicked.connect(self._on_item_clicked)
        self.joint_list.itemEntered.connect(self._on_item_entered)
        
        splitter.addWidget(self.joint_list)
        
        # 关节详情面板
        details_group = QGroupBox("关节详情")
        details_layout = QVBoxLayout()
        
        self.details_text = QTextEdit()
        self.details_text.setReadOnly(True)
        self.details_text.setMaximumHeight(200)
        self.details_text.setStyleSheet("""
            QTextEdit {
                background-color: #f8f9fa;
                border: 1px solid #dee2e6;
                border-radius: 4px;
                padding: 8px;
                font-size: 12px;
            }
        """)
        
        details_layout.addWidget(self.details_text)
        details_group.setLayout(details_layout)
        splitter.addWidget(details_group)
        
        # 设置分割器比例
        splitter.setStretchFactor(0, 7)  # 列表占70%
        splitter.setStretchFactor(1, 3)  # 详情占30%
        
        # 初始状态
        self._show_empty_state()
    
    def _get_joint_type_display_name(self, joint_type: JointType) -> str:
        """获取关节类型的显示名称"""
        type_names = {
            JointType.RIGID: "刚性",
            JointType.REVOLUTE: "旋转副",
            JointType.SLIDER: "滑动副",
            JointType.CYLINDRICAL: "圆柱副",
            JointType.PIN_SLOT: "销槽副",
            JointType.PLANAR: "平面副",
            JointType.BALL: "球面副",
            JointType.INFERRED: "推断副"
        }
        return type_names.get(joint_type, joint_type.value)
    
    def _show_empty_state(self):
        """显示空状态"""
        self.joint_list.clear()
        empty_item = QListWidgetItem("请先加载项目数据")
        empty_item.setFlags(Qt.NoItemFlags)
        self.joint_list.addItem(empty_item)
        
        self.details_text.setText("未选择关节")
    
    def load_project_data(self, project_path: Path) -> bool:
        """加载项目数据
        
        Args:
            project_path: 项目路径
            
        Returns:
            bool: 是否加载成功
        """
        try:
            # 查找component_positions.json文件
            json_file = project_path / "component_positions.json"
            
            if not json_file.exists():
                logger.error(f"未找到component_positions.json文件: {json_file}")
                self._show_empty_state()
                return False
            
            # 读取JSON文件
            with open(json_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            # 解析关节信息
            joints_data = data.get('joints', [])
            self._joints = [JointDisplayInfo.from_json(joint) for joint in joints_data]
            
            # 清空选择
            self._selected_joints.clear()
            
            # 填充列表
            self._populate_list()
            
            self._current_project = project_path
            logger.success(f"成功加载 {len(self._joints)} 个关节")
            return True
            
        except Exception as e:
            logger.error(f"加载项目数据失败: {e}")
            self._show_empty_state()
            return False
    
    def _populate_list(self):
        """填充关节列表"""
        self.joint_list.clear()
        
        if not self._joints:
            empty_item = QListWidgetItem("没有找到关节数据")
            empty_item.setFlags(Qt.NoItemFlags)
            self.joint_list.addItem(empty_item)
            return
        
        # 根据过滤器筛选关节
        filtered_joints = self._joints
        if self._filter_type:
            filtered_joints = [joint for joint in self._joints if joint.joint_type == self._filter_type]
        
        # 按关节名称排序
        sorted_joints = sorted(filtered_joints, key=lambda x: x.name)
        
        for joint in sorted_joints:
            item = QListWidgetItem(f"{joint.name} ({self._get_joint_type_display_name(joint.joint_type)})")
            item.setData(Qt.UserRole, joint.name)  # 存储关节名
            
            # 设置工具提示
            tooltip = f"""
关节: {joint.name}
类型: {self._get_joint_type_display_name(joint.joint_type)}
组件1: {joint.component_one}
组件2: {joint.component_two}
状态: {'抑制' if joint.is_suppressed else '激活'}
显示: {'开启' if joint.is_light_bulb_on else '关闭'}
            """.strip()
            item.setToolTip(tooltip)
            
            self.joint_list.addItem(item)
    
    def _on_item_clicked(self, item: QListWidgetItem):
        """点击事件处理 - 实现多选toggle状态"""
        joint_name = item.data(Qt.UserRole)
        
        if not joint_name:
            return
        
        # 查找关节信息
        joint = next((j for j in self._joints if j.name == joint_name), None)
        if not joint:
            return
        
        # 检查当前状态
        was_selected = joint_name in self._selected_joints
        
        if was_selected:
            # 如果已选中，则取消选中
            self._selected_joints.discard(joint_name)
            item.setSelected(False)  # 清除UI选择状态
            self.joint_selected.emit(joint_name, False)
            # 清除高亮
            self.components_highlight.emit([joint.component_one, joint.component_two], 'gray')
        else:
            # 如果未选中，则添加选中
            self._selected_joints.add(joint_name)
            item.setSelected(True)  # 设置UI选择状态
            self.joint_selected.emit(joint_name, True)
            # 高亮关联的组件
            self.components_highlight.emit([joint.component_one, joint.component_two], 'orange')
        
        # 更新详情显示
        self._update_details_display()
    
    def _on_clear_button_clicked(self):
        """清除选中按钮点击处理"""
        if self._selected_joints:
            logger.info(f"清除选中按钮被点击，清除 {len(self._selected_joints)} 个选中关节")
            self.clear_selection()
    
    def _on_filter_changed(self, index: int):
        """过滤器改变处理"""
        filter_data = self.filter_combo.itemData(index)
        self._filter_type = filter_data
        self._populate_list()
        logger.info(f"关节过滤器设置为: {self._get_joint_type_display_name(filter_data) if filter_data else '全部'}")
    
    def _update_details_display(self):
        """更新详情显示"""
        selected_count = len(self._selected_joints)
        
        if selected_count == 0:
            self.details_text.setText("未选择关节")
        elif selected_count == 1:
            # 显示单个关节的详细信息
            joint_name = next(iter(self._selected_joints))
            joint = next((j for j in self._joints if j.name == joint_name), None)
            
            if joint:
                details = f"""关节名称: {joint.name}
关节类型: {self._get_joint_type_display_name(joint.joint_type)}
组件1: {joint.component_one}
组件2: {joint.component_two}
状态: {'抑制' if joint.is_suppressed else '激活'}
显示: {'开启' if joint.is_light_bulb_on else '关闭'}

连接信息:
  第一个零部件: {joint.connection.occurrence_one_name or '无'}
  第二个零部件: {joint.connection.occurrence_two_name or '无'}"""
                
                self.details_text.setText(details)
            else:
                self.details_text.setText("未找到关节详细信息")
        else:
            # 显示多选摘要信息
            self.details_text.setText(f"已选择 {selected_count} 个关节")
    
    def _on_item_entered(self, item: QListWidgetItem):
        """鼠标悬停处理"""
        joint_name = item.data(Qt.UserRole)
        if joint_name:
            self.joint_hovered.emit(joint_name)
    
    def clear_selection(self):
        """清除所有选择"""
        # 获取当前选中的关节列表
        previously_selected = self._selected_joints.copy()
        
        if previously_selected:
            logger.info(f"清除 {len(previously_selected)} 个选中关节的选择状态")
            
            # 清除内部状态
            self._selected_joints.clear()
            
            # 清除UI选择状态
            self.joint_list.clearSelection()
            
            # 更新详情显示
            self.details_text.setText("未选择关节")
            
            # 为每个选中的关节发送取消选中信号
            for joint_name in previously_selected:
                self.joint_selected.emit(joint_name, False)
                # 清除高亮
                joint = next((j for j in self._joints if j.name == joint_name), None)
                if joint:
                    self.components_highlight.emit([joint.component_one, joint.component_two], 'gray')
        else:
            logger.info("没有选中的关节需要清除")
            # 确保UI状态一致
            self.joint_list.clearSelection()
            self.details_text.setText("未选择关节")
    
    def get_selected_joints(self) -> Set[str]:
        """获取选中的关节名称集合"""
        return self._selected_joints.copy()
    
    def get_joint_by_name(self, name: str) -> Optional[JointDisplayInfo]:
        """根据名称获取关节信息"""
        return next((j for j in self._joints if j.name == name), None)
    
    def get_all_joints(self) -> List[JointDisplayInfo]:
        """获取所有关节信息"""
        return self._joints.copy()
    
    def clear_data(self):
        """清除数据"""
        self._joints.clear()
        self._selected_joints.clear()
        self._current_project = None
        self._filter_type = None
        self.filter_combo.setCurrentIndex(0)
        self._show_empty_state()


class ModelViewTabWidget(QWidget):
    """模型视图选项卡组件 - 包含实体列表和关节列表"""
    
    # 信号定义
    component_selected = Signal(str, bool)  # 组件选择状态变化
    joint_selected = Signal(str, bool)     # 关节选择状态变化
    components_highlight = Signal(object, str)  # 组件高亮请求 (使用object避免类型问题)
    tab_changed = Signal(int)  # 选项卡切换信号
    
    def __init__(self, parent=None):
        """初始化模型视图选项卡组件"""
        super().__init__(parent)
        self._setup_ui()
    
    def _setup_ui(self):
        """设置UI界面"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        
        # 创建选项卡
        self.tab_widget = QTabWidget()
        
        # 创建实体列表选项卡
        from .component_list import ComponentListWidget
        self.component_list = ComponentListWidget()
        self.tab_widget.addTab(self.component_list, "实体列表")
        
        # 创建关节列表选项卡
        self.joint_list = JointListWidget()
        self.tab_widget.addTab(self.joint_list, "关节列表")
        
        # 连接信号
        self.component_list.component_selected.connect(self.component_selected)
        self.joint_list.joint_selected.connect(self.joint_selected)
        self.joint_list.components_highlight.connect(self.components_highlight)
        
        # 连接选项卡切换信号
        self.tab_widget.currentChanged.connect(self._on_tab_changed)
        
        layout.addWidget(self.tab_widget)
    
    def load_project_data(self, project_path: Path) -> bool:
        """加载项目数据"""
        success = True
        success &= self.component_list.load_project_data(project_path)
        success &= self.joint_list.load_project_data(project_path)
        return success
    
    def _on_tab_changed(self, index: int):
        """选项卡切换处理
        
        Args:
            index: 新选中的选项卡索引
        """
        try:
            # 获取切换前的选项卡
            prev_tab = self.tab_widget.widget(index - 1) if index > 0 else None
            # 获取切换后的选项卡
            current_tab = self.tab_widget.widget(index)
            
            logger.info(f"选项卡切换: {index}, 当前选项卡: {type(current_tab).__name__}")
            
            # 清空所有选择状态和渲染
            self._clear_all_selections_and_rendering()
            
            # 发送选项卡切换信号
            self.tab_changed.emit(index)
            
        except Exception as e:
            logger.error(f"选项卡切换处理失败: {e}")
    
    def _clear_all_selections_and_rendering(self):
        """清空所有选择状态和渲染"""
        try:
            # 清空实体列表选择并重置颜色（会自动发送取消选中信号）
            if hasattr(self.component_list, 'get_selected_components'):
                selected_components = self.component_list.get_selected_components()
                if selected_components:
                    logger.info(f"切换选项卡时清空 {len(selected_components)} 个实体选择")
                    # 利用clear_selection方法，它会自动发送取消选中信号
                    self.component_list.clear_selection()
            
            # 清空关节列表选择并重置颜色（会自动发送取消选中信号）
            if hasattr(self.joint_list, 'get_selected_joints'):
                selected_joints = self.joint_list.get_selected_joints()
                if selected_joints:
                    logger.info(f"切换选项卡时清空 {len(selected_joints)} 个关节选择")
                    # 利用clear_selection方法，它会自动发送取消选中信号和重置颜色
                    self.joint_list.clear_selection()
                    
        except Exception as e:
            logger.error(f"清空选择状态和渲染失败: {e}")
    
    def _clear_all_selections(self):
        """清空所有选择状态（不重置渲染）"""
        try:
            # 清空实体列表选择
            if hasattr(self.component_list, 'get_selected_components'):
                selected_components = self.component_list.get_selected_components()
                if selected_components:
                    logger.info(f"切换选项卡时清空 {len(selected_components)} 个实体选择")
                    self.component_list.clear_selection()
            
            # 清空关节列表选择
            if hasattr(self.joint_list, 'get_selected_joints'):
                selected_joints = self.joint_list.get_selected_joints()
                if selected_joints:
                    logger.info(f"切换选项卡时清空 {len(selected_joints)} 个关节选择")
                    self.joint_list.clear_selection()
                    
        except Exception as e:
            logger.error(f"清空选择状态失败: {e}")
    
    def clear_selection(self):
        """清除当前选项卡的所有选择"""
        current_tab = self.tab_widget.currentWidget()
        if current_tab == self.component_list:
            self.component_list.clear_selection()
        elif current_tab == self.joint_list:
            self.joint_list.clear_selection()
    
    def clear_data(self):
        """清除数据"""
        self.component_list.clear_data()
        self.joint_list.clear_data()