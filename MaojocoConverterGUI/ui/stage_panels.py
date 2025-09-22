"""
简化的阶段面板 - 业务逻辑委托给Core服务

重构原有的stage_panels.py，简化UI组件，将业务逻辑委托给Core服务。
保持阶段管理功能，但去掉过度复杂的抽象。
"""

from typing import Optional, Dict, Any, List, Union
from pathlib import Path
from dataclasses import dataclass

# 标准库
import sys
from pathlib import Path
from typing import List, Optional, Dict, Any, Union

# 第三方库
import pyvista as pv
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
    QPushButton, QFileDialog, QTableWidget, 
    QTableWidgetItem, QHeaderView, QProgressBar,
    QGroupBox, QFormLayout, QLineEdit, QTextEdit
)
from PySide6.QtCore import Signal, Qt

# 本地模块
from ..utils.logger import logger
from ..core.project_workflow_manager import ProjectWorkflowManager
from ..core.domain_types import LoadResult
from ..core.project_workflow_manager import WorkflowState
from ..core.service_interfaces import LoadMode
from ..core.stl_model_manager import ModelData
from .ui_workflow_adapter import UIWorkflowAdapter
from .component_list import ComponentListWidget
from .joint_list import ModelViewTabWidget
from .relationship_panel import RelationshipAnalysisPanel


@dataclass
class LoadingProgress:
    """加载进度信息"""
    current: int
    total: int
    message: str
    percentage: float = 0.0


class InitializationPanel(QWidget):
    """初始化面板 - 简化版本"""
    
    # 信号定义
    project_selected = Signal(Path)          # 项目选择完成
    project_validated = Signal(bool, str)    # 项目验证结果
    loading_started = Signal()               # 开始加载
    
    def __init__(self, workflow_manager: ProjectWorkflowManager, 
                 workflow_adapter: UIWorkflowAdapter, parent=None):
        """初始化面板
        
        Args:
            workflow_manager: 工作流管理器
            workflow_adapter: 工作流适配器
            parent: 父组件
        """
        super().__init__(parent)
        self._workflow_manager = workflow_manager
        self._workflow_adapter = workflow_adapter
        self._current_project: Optional[Path] = None
        self._setup_ui()
    
    def _setup_ui(self):
        """设置UI界面"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)
        
        # 使用说明文案
        welcome_label = QLabel(
            "<b>使用说明</b><br><br>"
            "请选择 <b>F3DMaojocoScripts</b> 插件导出的数据目录<br><br>"
            "<b>目录结构示例：</b><br>"
            "• 组件位置数据：component_positions.json<br>"
            "• STL模型：stl_files/ 目录<br>"
            "• 导出日志：f3d_export.log<br>"
            "• 导出说明：export_description.md<br><br>"
            "系统将自动处理坐标变换并生成MuJoCo模板目录"
        )
        welcome_label.setWordWrap(True)
        welcome_label.setStyleSheet("""
            QLabel {
                color: #333;
                font-size: 13px;
                line-height: 1.6;
                padding: 20px;
                background-color: #f8f9fa;
                border-radius: 8px;
                border: 1px solid #e9ecef;
            }
        """)
        welcome_label.setTextFormat(Qt.RichText)
        welcome_label.setOpenExternalLinks(False)
        layout.addWidget(welcome_label)
        
        # 目录选择
        dir_select_layout = QHBoxLayout()
        
        self.path_edit = QLineEdit()
        self.path_edit.setPlaceholderText("点击浏览选择项目目录...")
        self.path_edit.setEnabled(False)
        
        browse_btn = QPushButton("浏览目录")
        browse_btn.setMinimumWidth(100)
        browse_btn.clicked.connect(self._browse_directory)
        
        dir_select_layout.addWidget(self.path_edit, 1)
        dir_select_layout.addWidget(browse_btn)
        layout.addLayout(dir_select_layout)
        
        # 状态显示
        self.status_label = QLabel("")
        self.status_label.setStyleSheet("color: #666; font-size: 12px;")
        layout.addWidget(self.status_label)
        
        layout.addStretch()
    
    def _browse_directory(self):
        """浏览目录"""
        directory = QFileDialog.getExistingDirectory(
            self, "选择包含导出数据的目录"
        )
        
        if directory:
            self._current_project = Path(directory)
            self.path_edit.setText(str(self._current_project))
            self.project_selected.emit(self._current_project)
            
            # 立即验证项目
            self._validate_project()
    
    def _validate_project(self):
        """验证项目结构"""
        if not self._current_project:
            return
        
        try:
            errors = self._workflow_manager._data_loading_service.validate_project_structure(self._current_project)
            
            if errors:
                error_msg = f"发现 {len(errors)} 个问题：\n" + "\n".join(errors)
                self.status_label.setText("项目验证失败")
                self.status_label.setStyleSheet("color: #d32f2f; font-size: 12px;")
                self.project_validated.emit(False, error_msg)
            else:
                self.status_label.setText("项目验证通过，正在自动加载...")
                self.status_label.setStyleSheet("color: #388e3c; font-size: 12px;")
                self.project_validated.emit(True, "项目结构验证通过")
                
                # 自动开始加载项目
                self._load_project()
                
        except Exception as e:
            error_msg = f"验证项目时发生错误：{str(e)}"
            self.status_label.setText("验证失败")
            self.status_label.setStyleSheet("color: #d32f2f; font-size: 12px;")
            self.project_validated.emit(False, error_msg)
    
        
    def _load_project(self):
        """加载项目"""
        if not self._current_project:
            return
        
        self.loading_started.emit()
        
        try:
            # 使用工作流管理器加载项目
            result = self._workflow_manager.load_project(self._current_project)
            
            if result.success:
                logger.success(f"项目加载成功：{result.message}")
                self.status_label.setText("项目加载成功")
                self.status_label.setStyleSheet("color: #388e3c; font-size: 12px;")
                
                # 通知父组件加载完成
                try:
                    parent = self.parent()
                    if parent and hasattr(parent, 'on_project_loaded'):
                        parent_method = getattr(parent, 'on_project_loaded')
                        if callable(parent_method):
                            parent_method(result)
                except Exception as e:
                    logger.warning(f"通知父组件项目加载完成失败: {e}")
                    
            else:
                logger.error(f"项目加载失败：{result.message}")
                self.status_label.setText("项目加载失败")
                self.status_label.setStyleSheet("color: #d32f2f; font-size: 12px;")
                
        except Exception as e:
            logger.error(f"加载项目时发生错误：{e}")
            self.status_label.setText("加载失败")
            self.status_label.setStyleSheet("color: #d32f2f; font-size: 12px;")


class DataLoadingPanel(QWidget):
    """模型预览面板 - 简化版本"""
    
    def __init__(self, workflow_manager: ProjectWorkflowManager, parent=None):
        """初始化模型预览面板
        
        Args:
            workflow_manager: 工作流管理器
            parent: 父组件
        """
        super().__init__(parent)
        self._workflow_manager = workflow_manager
        self._current_result: Optional[LoadResult] = None
        self._component_list: Optional[ComponentListWidget] = None
        self._setup_ui()
    
    def _setup_ui(self):
        """设置UI界面"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)
        
        # 实体列表组件 - 使用选项卡形式（实体列表 + 关节列表）
        self._model_view_tab_widget = ModelViewTabWidget()
        layout.addWidget(self._model_view_tab_widget)
        
        layout.addStretch()
        
        # 组件选择信号将由父容器连接
    
    def update_load_result(self, result: LoadResult):
        """更新加载结果显示
        
        Args:
            result: 加载结果
        """
        self._current_result = result
        
        if result.success:
            # 加载实体列表数据
            if result.project_info and result.project_info.project_directory:
                project_path = Path(result.project_info.project_directory)
                self._model_view_tab_widget.load_project_data(project_path)
            
        else:
            # 清空实体列表
            self._model_view_tab_widget.clear_data()
    
    def _update_models_table(self, models: List[Union[ModelData, Any]]):
        """更新模型表格
        
        Args:
            models: 模型列表 (ModelData or STLModel)
        """
        self.models_table.setRowCount(len(models))
        
        for row, model in enumerate(models):
            # 模型名称
            name_item = QTableWidgetItem(model.name)
            name_item.setFlags(name_item.flags() & ~Qt.ItemIsEditable)
            self.models_table.setItem(row, 0, name_item)
            
            # 顶点数 - 支持ModelData和STLModel两种类型
            vertex_count = (
                model.mesh.n_points 
                if hasattr(model, 'mesh') and hasattr(model.mesh, 'n_points')
                else model.vertex_count 
                if hasattr(model, 'vertex_count')
                else 0
            )
            
            vertex_item = QTableWidgetItem(str(vertex_count))
            vertex_item.setFlags(vertex_item.flags() & ~Qt.ItemIsEditable)
            self.models_table.setItem(row, 1, vertex_item)
            
            # 面数 - 支持ModelData和STLModel两种类型
            face_count = (
                model.mesh.n_faces 
                if hasattr(model, 'mesh') and hasattr(model.mesh, 'n_faces')
                else model.face_count 
                if hasattr(model, 'face_count')
                else 0
            )
                
            face_item = QTableWidgetItem(str(face_count))
            face_item.setFlags(face_item.flags() & ~Qt.ItemIsEditable)
            self.models_table.setItem(row, 2, face_item)
            
            # 状态
            is_transformed = getattr(model, 'is_transformed', False)
            status_text = "已变换" if is_transformed else "原始"
            status_item = QTableWidgetItem(status_text)
            status_item.setFlags(status_item.flags() & ~Qt.ItemIsEditable)
            self.models_table.setItem(row, 3, status_item)


class SimpleStagePanel(QWidget):
    """简单阶段面板 - 用于其他阶段"""
    
    def __init__(self, stage_name: str, display_name: str, parent=None):
        """初始化简单阶段面板
        
        Args:
            stage_name: 阶段名称
            display_name: 显示名称
            parent: 父组件
        """
        super().__init__(parent)
        self.stage_name = stage_name
        self.display_name = display_name
        self._setup_ui()
    
    def _setup_ui(self):
        """设置UI界面"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)
        
        # 阶段标题
        title_label = QLabel(f"{self.display_name}")
        title_label.setStyleSheet("font-size: 16px; font-weight: bold; color: #333;")
        layout.addWidget(title_label)
        
        # 说明文字
        info_text = QLabel(f"这是{self.display_name}阶段。\n\n此阶段的具体功能正在开发中...")
        info_text.setWordWrap(True)
        info_text.setStyleSheet("color: #666; font-size: 12px;")
        layout.addWidget(info_text)
        
        # 占位内容
        placeholder = QTextEdit()
        placeholder.setPlaceholderText(f"{self.display_name}阶段的配置选项将在这里显示...")
        placeholder.setEnabled(False)
        layout.addWidget(placeholder)
        
        # 执行按钮
        self.execute_button = QPushButton(f"执行{self.display_name}")
        self.execute_button.setMinimumHeight(40)
        self.execute_button.clicked.connect(self._on_execute_clicked)
        layout.addWidget(self.execute_button)
        
        layout.addStretch()
    
    def _on_execute_clicked(self):
        """执行按钮点击处理"""
        logger.info(f"执行阶段：{self.display_name}")
        # 这里可以添加具体的执行逻辑


class StagePanelsContainer(QWidget):
    """阶段面板容器 - 管理所有阶段面板"""
    
    # 信号定义
    stage_changed = Signal(str)    # 阶段切换信号
    project_loaded = Signal(object) # 项目加载完成信号
    component_selected = Signal(str, bool)  # 组件选择信号
    joint_selected = Signal(str, bool)     # 关节选择信号
    components_highlight = Signal(object, str)  # 组件高亮信号
    
    def __init__(self, workflow_manager: ProjectWorkflowManager, 
                 workflow_adapter: UIWorkflowAdapter, parent=None):
        """初始化阶段面板容器
        
        Args:
            workflow_manager: 工作流管理器
            workflow_adapter: 工作流适配器
            parent: 父组件
        """
        super().__init__(parent)
        self._workflow_manager = workflow_manager
        self._workflow_adapter = workflow_adapter
        self._current_stage = "initialization"
        self._panels = {}
        self._setup_ui()
        
        # 连接工作流适配器信号
        self._workflow_adapter.workflow_state_changed.connect(self._on_workflow_state_changed)
        self._workflow_adapter.project_loaded.connect(self._on_project_loaded)
        self._workflow_adapter.components_updated.connect(self._on_components_updated)
        self._workflow_adapter.joints_updated.connect(self._on_joints_updated)
        self._workflow_adapter.error_occurred.connect(self._on_error_occurred)
        self._workflow_adapter.info_message.connect(self._on_info_message)
    
    def _setup_ui(self):
        """设置UI界面"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        
        # 创建阶段面板
        self._panels['initialization'] = InitializationPanel(
            self._workflow_manager, self._workflow_adapter
        )
        self._panels['data_loading'] = DataLoadingPanel(self._workflow_manager)
        
        # 连接组件选择信号
        self._panels['data_loading']._model_view_tab_widget.component_selected.connect(
            self.component_selected
        )
        
        # 连接关节选择信号
        self._panels['data_loading']._model_view_tab_widget.joint_selected.connect(
            self.joint_selected
        )
        
        # 连接组件高亮信号
        self._panels['data_loading']._model_view_tab_widget.components_highlight.connect(
            self.components_highlight
        )
        
        # 连接选项卡切换信号
        self._panels['data_loading']._model_view_tab_widget.tab_changed.connect(
            self._on_tab_changed
        )
        
        # Create relationship analysis panel (full functionality)
        self._panels['relationship_analysis'] = RelationshipAnalysisPanel(
            self._workflow_manager, self._workflow_adapter
        )
        
        # Connect relationship analysis panel signals
        relationship_panel = self._panels['relationship_analysis']
        relationship_panel.component_selected.connect(
            lambda name: self.component_selected.emit(name, True)
        )
        relationship_panel.joint_selected.connect(
            lambda name: self.joint_selected.emit(name, True)
        )
        
        # Create other simple panels
        other_stages = [
            ("unit_conversion", "单位转换"),
            ("model_generation", "模型生成"),
            ("actuator_generation", "执行器生成")
        ]
        
        for stage_name, display_name in other_stages:
            self._panels[stage_name] = SimpleStagePanel(stage_name, display_name)
        
        # 添加所有面板到布局（初始隐藏）
        for panel in self._panels.values():
            panel.setVisible(False)
            layout.addWidget(panel)
        
        # 默认显示初始化面板
        self.switch_to_stage('initialization')
        
        logger.success("阶段面板容器初始化完成")
    
    def switch_to_stage(self, stage_name: str):
        """切换到指定阶段
        
        Args:
            stage_name: 阶段名称
        """
        if stage_name not in self._panels:
            logger.error(f"未知阶段：{stage_name}")
            return
        
        # 隐藏所有面板
        for panel in self._panels.values():
            panel.setVisible(False)
        
        # 显示目标面板
        target_panel = self._panels[stage_name]
        target_panel.setVisible(True)
        
        self._current_stage = stage_name
        self.stage_changed.emit(stage_name)
        
        logger.info(f"切换到阶段：{stage_name}")
    
    def get_current_stage(self) -> str:
        """获取当前阶段
        
        Returns:
            str: 当前阶段名称
        """
        return self._current_stage
    
    def on_project_loaded(self, result: LoadResult, quick_start: bool = False):
        """项目加载完成处理
        
        Args:
            result: 加载结果
            quick_start: 是否为快速启动模式
        """
        logger.info(f"项目加载完成，切换到数据加载阶段")
        
        # 更新数据加载面板
        data_panel = self._panels['data_loading']
        data_panel.update_load_result(result)
        
        # 更新关系分析面板
        if result.success and result.project_info:
            relationship_panel = self._panels['relationship_analysis']
            project_path = Path(result.project_info.project_directory)
            relationship_panel.load_project_data(project_path, result, quick_start)
        
        # 切换到数据加载面板
        self.switch_to_stage('data_loading')
        
        # 发送项目加载完成信号
        self.project_loaded.emit(result)
    
    def _on_error_occurred(self, error_msg: str):
        """错误处理
        
        Args:
            error_msg: 错误信息
        """
        logger.error(f"工作流错误：{error_msg}")
    
    def _on_workflow_state_changed(self, state: WorkflowState):
        """工作流状态变化处理
        
        Args:
            state: 工作流状态
        """
        logger.info(f"工作流状态变化: {state.current_stage} - {state.status.value}")
        self._current_stage = state.current_stage
        
        # 更新当前激活的面板
        self.switch_to_stage(state.current_stage)
    
    def _on_project_loaded(self, result: LoadResult):
        """项目加载完成处理
        
        Args:
            result: 加载结果
        """
        logger.info(f"项目加载完成: {result.message}")
        
        if result.success:
            # 切换到数据加载阶段
            self.switch_to_stage('data_loading')
            
            # 更新数据加载面板的数据
            try:
                if 'data_loading' in self._panels:
                    data_panel = self._panels['data_loading']
                    if hasattr(data_panel, 'on_project_loaded'):
                        data_panel.on_project_loaded(result)
            except Exception as e:
                logger.warning(f"更新数据加载面板失败: {e}")
            
            # 发送项目加载完成信号
            self.project_loaded.emit(result)
    
    def _on_components_updated(self, components: list):
        """组件列表更新处理
        
        Args:
            components: 组件列表
        """
        logger.info(f"组件列表更新: {len(components)} 个组件")
        
        # 更新数据加载面板的组件列表
        try:
            if 'data_loading' in self._panels:
                data_panel = self._panels['data_loading']
                if hasattr(data_panel, '_model_view_tab_widget'):
                    data_panel._model_view_tab_widget.update_components(components)
        except Exception as e:
            logger.warning(f"更新组件列表失败: {e}")
    
    def _on_joints_updated(self, joints: list):
        """关节列表更新处理
        
        Args:
            joints: 关节列表
        """
        logger.info(f"关节列表更新: {len(joints)} 个关节")
        
        # 更新数据加载面板的关节列表
        try:
            if 'data_loading' in self._panels:
                data_panel = self._panels['data_loading']
                if hasattr(data_panel, '_model_view_tab_widget'):
                    data_panel._model_view_tab_widget.update_joints(joints)
        except Exception as e:
            logger.warning(f"更新关节列表失败: {e}")
    
    def _on_info_message(self, message: str):
        """信息消息处理
        
        Args:
            message: 信息消息
        """
        logger.info(f"工作流信息: {message}")
    
    def _on_tab_changed(self, index: int):
        """选项卡切换处理
        
        Args:
            index: 选项卡索引
        """
        try:
            logger.info(f"阶段面板容器检测到选项卡切换: {index}")
            # 可以在这里添加额外的逻辑，比如更新状态等
        except Exception as e:
            logger.error(f"处理选项卡切换失败: {e}")