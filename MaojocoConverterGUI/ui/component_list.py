"""
实体列表组件 - 用于模型预览界面

从component_positions.json解析组件信息，提供实体选择功能。
选中实体后，3D预览图中对应部分变为黄色，取消选择恢复灰色。
"""

from typing import List, Dict, Any, Optional, Set
from pathlib import Path
from dataclasses import dataclass
import json

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
    QListWidget, QListWidgetItem, QGroupBox,
    QSplitter, QTextEdit, QFrame, QPushButton
)
from PySide6.QtCore import Signal, Qt
from PySide6.QtGui import QFont

from ..utils.logger import logger


@dataclass
class ComponentInfo:
    """组件信息"""
    name: str
    occurrence_name: str
    component_id: int
    stl_file: str
    position: List[float]  # [x, y, z]
    has_children: bool
    bodies_count: int
    
    @classmethod
    def from_json(cls, data: Dict[str, Any]) -> 'ComponentInfo':
        """从JSON数据创建组件信息"""
        # 提取位置信息从world_transform.matrix
        world_transform = data.get('world_transform', {})
        matrix = world_transform.get('matrix', [])
        position = [0.0, 0.0, 0.0]
        
        if len(matrix) >= 4:
            position = [matrix[0][3], matrix[1][3], matrix[2][3]]
        
        return cls(
            name=data.get('name', ''),
            occurrence_name=data.get('occurrence_name', ''),
            component_id=data.get('component_id', 0),
            stl_file=data.get('stl_file', ''),
            position=position,
            has_children=data.get('has_children', False),
            bodies_count=data.get('bodies_count', 0)
        )


class ComponentListWidget(QWidget):
    """实体列表组件"""
    
    # 信号定义
    component_selected = Signal(str, bool)  # 组件选择状态变化 (组件名, 是否选中)
    component_hovered = Signal(str)         # 鼠标悬停在组件上
    
    def __init__(self, parent=None):
        """初始化实体列表组件"""
        super().__init__(parent)
        self._components: List[ComponentInfo] = []
        self._selected_components: Set[str] = set()
        self._current_project: Optional[Path] = None
        self._setup_ui()
    
    def _setup_ui(self):
        """设置UI界面"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)
        
        # 标题区域
        title_layout = QHBoxLayout()
        
        title_label = QLabel("实体列表")
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
        
        # 创建分割器
        splitter = QSplitter(Qt.Vertical)
        layout.addWidget(splitter)
        
        # 组件列表
        self.component_list = QListWidget()
        self.component_list.setSelectionMode(QListWidget.MultiSelection)
        self.component_list.setAlternatingRowColors(True)
        
        # 设置选中样式 - 黄色背景表示选中
        self.component_list.setStyleSheet("""
            QListWidget::item:selected {
                background-color: #ffd700;
                color: #333333;
                border: 2px solid #ffb300;
                font-weight: bold;
            }
            QListWidget::item:hover {
                background-color: #fff5cc;
            }
        """)
        
        # 连接信号
        self.component_list.itemClicked.connect(self._on_item_clicked)
        self.component_list.itemEntered.connect(self._on_item_entered)
        
        splitter.addWidget(self.component_list)
        
        # 组件详情面板
        details_group = QGroupBox("组件详情")
        details_layout = QVBoxLayout()
        
        self.details_text = QTextEdit()
        self.details_text.setReadOnly(True)
        self.details_text.setMaximumHeight(150)
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
    
    def _show_empty_state(self):
        """显示空状态"""
        self.component_list.clear()
        empty_item = QListWidgetItem("请先加载项目数据")
        empty_item.setFlags(Qt.NoItemFlags)
        self.component_list.addItem(empty_item)
        
        self.details_text.setText("未选择组件")
    
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
            
            # 解析组件信息
            components_data = data.get('components', [])
            self._components = [ComponentInfo.from_json(comp) for comp in components_data]
            
            # 清空选择
            self._selected_components.clear()
            
            # 填充列表
            self._populate_list()
            
            self._current_project = project_path
            logger.success(f"成功加载 {len(self._components)} 个组件")
            return True
            
        except Exception as e:
            logger.error(f"加载项目数据失败: {e}")
            self._show_empty_state()
            return False
    
    def find_matching_model_name(self, component_name: str, model_names: List[str] = None) -> str:
        """查找匹配的模型名称
        
        Args:
            component_name: 组件名称
            model_names: 可选的模型名称列表，用于模糊匹配
            
        Returns:
            str: 匹配的模型名称，如未找到返回原名称
        """
        # 如果没有提供模型名称列表，直接返回原名称
        if not model_names:
            return component_name
        
        # 精确匹配
        if component_name in model_names:
            return component_name
        
        # 模糊匹配：处理常见的不一致情况
        fuzzy_matches = []
        for model_name in model_names:
            # 去除特殊字符后匹配
            clean_component = ''.join(c for c in component_name if c.isalnum())
            clean_model = ''.join(c for c in model_name if c.isalnum())
            
            if clean_component == clean_model:
                fuzzy_matches.append(model_name)
                continue
            
            # 部分匹配（至少3个字符相同）
            if len(clean_component) >= 3 and len(clean_model) >= 3:
                if clean_component in clean_model or clean_model in clean_component:
                    fuzzy_matches.append(model_name)
        
        # 如果找到模糊匹配，返回第一个
        if fuzzy_matches:
            logger.info(f"组件名称模糊匹配: {component_name} -> {fuzzy_matches[0]}")
            return fuzzy_matches[0]
        
        # 如果都没有匹配，返回原名称
        logger.warning(f"未找到模型名称匹配: {component_name}")
        return component_name
    
    def get_component_model_names(self) -> List[str]:
        """获取所有组件对应的模型名称
        
        Returns:
            List[str]: 模型名称列表
        """
        return [comp.name for comp in self._components]
    
    def _populate_list(self):
        """填充组件列表"""
        self.component_list.clear()
        
        if not self._components:
            empty_item = QListWidgetItem("没有找到组件数据")
            empty_item.setFlags(Qt.NoItemFlags)
            self.component_list.addItem(empty_item)
            return
        
        # 按组件名称排序
        sorted_components = sorted(self._components, key=lambda x: x.name)
        
        for component in sorted_components:
            item = QListWidgetItem(component.name)
            item.setData(Qt.UserRole, component.name)  # 存储组件名
            
            # 设置工具提示
            tooltip = f"""
组件: {component.name}
实例: {component.occurrence_name}
ID: {component.component_id}
STL文件: {component.stl_file}
位置: ({component.position[0]:.2f}, {component.position[1]:.2f}, {component.position[2]:.2f})
实体数: {component.bodies_count}
            """.strip()
            item.setToolTip(tooltip)
            
            self.component_list.addItem(item)
    
    def _on_item_clicked(self, item: QListWidgetItem):
        """点击事件处理 - 实现多选toggle状态"""
        component_name = item.data(Qt.UserRole)
        
        if not component_name:
            return
        
        # 查找组件信息
        component = next((c for c in self._components if c.name == component_name), None)
        if not component:
            return
        
        # 检查当前状态
        was_selected = component_name in self._selected_components
        
        if was_selected:
            # 如果已选中，则取消选中
            self._selected_components.discard(component_name)
            item.setSelected(False)  # 清除UI选择状态
            self.component_selected.emit(component_name, False)
        else:
            # 如果未选中，则添加选中
            self._selected_components.add(component_name)
            item.setSelected(True)  # 设置UI选择状态
            self.component_selected.emit(component_name, True)
        
        # 更新详情显示
        self._update_details_display()
    
    def _on_clear_button_clicked(self):
        """清除选中按钮点击处理"""
        if self._selected_components:
            logger.info(f"清除选中按钮被点击，清除 {len(self._selected_components)} 个选中组件")
            self.clear_selection()
    
    def _update_details_display(self):
        """更新详情显示"""
        selected_count = len(self._selected_components)
        
        if selected_count == 0:
            self.details_text.setText("未选择组件")
        elif selected_count == 1:
            # 显示单个组件的详细信息
            component_name = next(iter(self._selected_components))
            component = next((c for c in self._components if c.name == component_name), None)
            
            if component:
                details = f"""组件名称: {component.name}
实例名称: {component.occurrence_name}
组件ID: {component.component_id}
STL文件: {component.stl_file}
位置: X={component.position[0]:.3f}, Y={component.position[1]:.3f}, Z={component.position[2]:.3f}
实体数量: {component.bodies_count}
子组件: {'是' if component.has_children else '否'}"""
                
                self.details_text.setText(details)
            else:
                self.details_text.setText("未找到组件详细信息")
        else:
            # 显示多选摘要信息
            self.details_text.setText(f"已选择 {selected_count} 个组件")
    
    def _clear_all_selections(self):
        """清除所有选择状态"""
        # 获取当前选中的组件
        previously_selected = self._selected_components.copy()
        
        # 清除选择集合
        self._selected_components.clear()
        
        # 清除UI选择状态
        for i in range(self.component_list.count()):
            item = self.component_list.item(i)
            item.setSelected(False)
        
        # 为之前选中的组件发送取消选中信号
        for component_name in previously_selected:
            self.component_selected.emit(component_name, False)
        
        # 更新详情显示
        self._update_details_display()
    
    def _on_item_entered(self, item: QListWidgetItem):
        """鼠标悬停处理"""
        component_name = item.data(Qt.UserRole)
        if component_name:
            self.component_hovered.emit(component_name)
    
    def get_selected_components(self) -> Set[str]:
        """获取选中的组件名称集合"""
        return self._selected_components.copy()
    
    def set_component_selection(self, component_name: str, selected: bool):
        """设置组件选择状态
        
        Args:
            component_name: 组件名称
            selected: 是否选中
        """
        changed = False
        
        if selected:
            if component_name not in self._selected_components:
                self._selected_components.add(component_name)
                changed = True
        else:
            if component_name in self._selected_components:
                self._selected_components.discard(component_name)
                changed = True
        
        # 更新列表项的选中状态
        for i in range(self.component_list.count()):
            item = self.component_list.item(i)
            if item.data(Qt.UserRole) == component_name:
                item.setSelected(selected)
                break
        
        # 如果状态有变化，更新详情显示
        if changed:
            self._update_details_display()
            
        # 发送信号
        if changed:
            self.component_selected.emit(component_name, selected)
    
    def clear_selection(self):
        """清除所有选择"""
        # 获取当前选中的组件列表
        previously_selected = self._selected_components.copy()
        
        if previously_selected:
            logger.info(f"清除 {len(previously_selected)} 个选中组件的选择状态")
            
            # 清除内部状态
            self._selected_components.clear()
            
            # 清除UI选择状态
            self.component_list.clearSelection()
            
            # 更新详情显示
            self.details_text.setText("未选择组件")
            
            # 为每个选中的组件发送取消选中信号
            for component_name in previously_selected:
                self.component_selected.emit(component_name, False)
        else:
            logger.info("没有选中的组件需要清除")
            # 确保UI状态一致
            self.component_list.clearSelection()
            self.details_text.setText("未选择组件")
    
    def get_component_by_name(self, name: str) -> Optional[ComponentInfo]:
        """根据名称获取组件信息"""
        return next((c for c in self._components if c.name == name), None)
    
    def get_all_components(self) -> List[ComponentInfo]:
        """获取所有组件信息"""
        return self._components.copy()
    
    def clear_data(self):
        """清除数据"""
        self._components.clear()
        self._selected_components.clear()
        self._current_project = None
        self._show_empty_state()