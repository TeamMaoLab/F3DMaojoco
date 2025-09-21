"""
简化的主窗口 - 直接调用Core服务

重构原有的main_window.py，去除过度复杂的中间层，实现UI层直接调用Core服务。
遵循2层架构：GUI层 → Core层。
"""

# 标准库
from typing import Optional

# 第三方库
from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QLabel, QPushButton, QFrame, QSplitter
)
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont

# 本地模块
from ..utils.logger import logger
from ..core.transform_service import TransformService, LoadResult
from ..core.project_data_service import ProjectDataService
from .stage_panels import StagePanelsContainer
from .visualization_widget import VisualizationWidget


class MainWindowSimplified(QMainWindow):
    """简化的主窗口
    
    直接调用Core服务，去除不必要的中间层。
    实现清晰的2层架构：GUI层 → Core层。
    """
    
    def __init__(self):
        """初始化主窗口"""
        super().__init__()
        
        # 初始化Core服务
        self._transform_service = TransformService()
        self._data_service = ProjectDataService()
        
        # 快速启动结果
        self.quick_start_result: Optional[LoadResult] = None
        
        self._setup_ui()
        self._setup_window()
        self._connect_signals()
        
        logger.success("简化主窗口初始化完成")
    
    def _setup_ui(self):
        """设置UI布局"""
        logger.info("初始化简化主窗口UI")
        
        # 创建中央窗口部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 创建主布局
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)
        
        # 创建左右分割器
        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter)
        
        # 左侧3D视图
        left_panel = self._create_left_panel()
        splitter.addWidget(left_panel)
        
        # 右侧配置面板
        right_panel = self._create_right_panel()
        splitter.addWidget(right_panel)
        
        # 设置分割器比例
        splitter.setStretchFactor(0, 6)  # 左侧3D视图
        splitter.setStretchFactor(1, 4)  # 右侧配置面板
        
        # 设置面板尺寸
        right_panel.setFixedWidth(500)
        left_panel.setMinimumWidth(400)
        
        logger.success("简化主窗口UI初始化完成")
    
    def _create_left_panel(self) -> QWidget:
        """创建左侧3D可视化面板"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        
        # 简化的3D可视化组件（包含自己的控制栏）
        self.viz_widget = VisualizationWidget(self._transform_service)
        layout.addWidget(self.viz_widget)
        
        return panel
    
    def _create_right_panel(self) -> QWidget:
        """创建右侧配置面板"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)
        
        # 标题
        self.right_panel_title = QLabel("配置面板")
        self.right_panel_title.setFont(QFont("Arial", 12, QFont.Bold))
        layout.addWidget(self.right_panel_title)
        
        # 分隔线
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        layout.addWidget(line)
        
        # 简化的阶段面板容器
        self.stage_panels = StagePanelsContainer(
            self._transform_service, self._data_service
        )
        layout.addWidget(self.stage_panels)
        
        # 添加弹簧
        layout.addStretch()
        
        return panel
    
    def _connect_signals(self):
        """连接信号"""
        # 阶段切换信号
        self.stage_panels.stage_changed.connect(self._on_stage_changed)
        
        # 项目加载完成信号
        self.stage_panels.project_loaded.connect(self._on_project_loaded)
        
        # 组件选择信号
        self.stage_panels.component_selected.connect(self._on_component_selected)
        
        # 关节选择信号
        if hasattr(self.stage_panels, 'joint_selected'):
            self.stage_panels.joint_selected.connect(self._on_joint_selected)
        
        # 组件高亮信号（用于关节选择时的实体高亮）
        if hasattr(self.stage_panels, 'components_highlight'):
            self.stage_panels.components_highlight.connect(self._on_components_highlight)
    
    def _on_stage_changed(self, stage_name: str):
        """阶段切换处理"""
        logger.info(f"阶段切换: {stage_name}")
        
        # 更新右侧面板标题
        stage_info = {
            "initialization": {"order": "1/6", "name": "初始化"},
            "data_loading": {"order": "2/6", "name": "模型预览"},
            "relationship_analysis": {"order": "3/6", "name": "关系分析"},
            "unit_conversion": {"order": "4/6", "name": "单位转换"},
            "model_generation": {"order": "5/6", "name": "模型生成"},
            "actuator_generation": {"order": "6/6", "name": "执行器生成"}
        }
        
        if stage_name in stage_info:
            info = stage_info[stage_name]
            title_text = f"配置面板-「{info['order']}」{info['name']}"
            self.right_panel_title.setText(title_text)
        
        # 控制3D视图提示文字的显示
        if stage_name == "initialization":
            # 在初始化阶段显示提示文字
            self.viz_widget.show_initial_text()
        else:
            # 在其他阶段隐藏提示文字
            self.viz_widget.hide_initial_text()
    
    def _on_project_loaded(self, result: LoadResult):
        """项目加载完成处理"""
        logger.info(f"项目加载完成: {result.message}")
        
        # 在3D视图中显示加载的模型
        if result.success and result.models:
            self.viz_widget.display_models_from_result(result)
    
    def _setup_window(self):
        """设置窗口属性"""
        logger.info("设置简化主窗口属性")
        
        # 窗口标题
        self.setWindowTitle("MaojocoConverter GUI (简化版)")
        
        # 窗口大小
        self.resize(1200, 800)
        
        # 检查是否有快速启动结果
        if self.quick_start_result:
            logger.info("检测到快速启动结果，直接跳转到模型预览阶段")
            # 延迟处理以确保UI完全初始化
            from PySide6.QtCore import QTimer
            QTimer.singleShot(100, self._process_quick_start)
        
        logger.info("简化主窗口属性设置完成")
    
    def get_transform_service(self) -> TransformService:
        """获取TransformService实例"""
        return self._transform_service
    
    def get_data_service(self) -> ProjectDataService:
        """获取ProjectDataService实例"""
        return self._data_service
    
    def _on_component_selected(self, component_name: str, selected: bool):
        """组件选择处理
        
        Args:
            component_name: 组件名称
            selected: 是否选中
        """
        try:
            # 获取实际的模型名称列表进行匹配
            model_names = self.viz_widget.get_model_names()
            
            # 获取数据加载面板
            data_panel = self.stage_panels._panels.get('data_loading')
            if data_panel and hasattr(data_panel, '_model_view_tab_widget'):
                component_list = data_panel._model_view_tab_widget.component_list
                if component_list:
                    # 使用模糊匹配查找正确的模型名称
                    matched_model_name = component_list.find_matching_model_name(
                        component_name, model_names
                    )
                else:
                    # 如果component_list为None，直接使用原始名称
                    matched_model_name = component_name
                
                # 设置模型颜色
                color = 'yellow' if selected else 'gray'
                self.viz_widget.set_model_color(matched_model_name, color)
                
                logger.info(f"组件 {component_name} -> 模型 {matched_model_name} 选择状态: {selected}, 颜色设置为: {color}")
            else:
                # 直接使用原始组件名称
                color = 'yellow' if selected else 'gray'
                self.viz_widget.set_model_color(component_name, color)
                logger.info(f"组件 {component_name} 选择状态: {selected}, 颜色设置为: {color}")
            
        except Exception as e:
            logger.error(f"处理组件选择失败: {e}")
    
    def _on_joint_selected(self, joint_name: str, selected: bool):
        """关节选择处理
        
        Args:
            joint_name: 关节名称
            selected: 是否选中
        """
        try:
            logger.info(f"关节 {joint_name} 选择状态: {selected}")
            # 这里可以添加关节选择的具体逻辑
            # 比如高亮关节的连接点、显示关节信息等
        except Exception as e:
            logger.error(f"处理关节选择失败: {e}")
    
    def _on_components_highlight(self, component_names, color: str):
        """组件高亮处理
        
        Args:
            component_names: 组件名称列表
            color: 高亮颜色
        """
        try:
            # 获取实际的模型名称列表
            model_names = self.viz_widget.get_model_names()
            
            # 获取数据加载面板
            data_panel = self.stage_panels._panels.get('data_loading')
            if data_panel and hasattr(data_panel, '_model_view_tab_widget'):
                component_list = data_panel._model_view_tab_widget.component_list
                
                # 为每个组件名称设置颜色
                for component_name in component_names:
                    if component_name:  # 确保组件名称不为空
                        if component_list:
                            # 使用模糊匹配查找正确的模型名称
                            matched_model_name = component_list.find_matching_model_name(
                                component_name, model_names
                            )
                        else:
                            # 如果component_list为None，直接使用原始名称
                            matched_model_name = component_name
                        
                        # 设置模型颜色
                        self.viz_widget.set_model_color(matched_model_name, color)
                        logger.info(f"关节关联组件 {component_name} -> 模型 {matched_model_name} 高亮为: {color}")
            else:
                # 直接使用原始组件名称
                for component_name in component_names:
                    if component_name:
                        self.viz_widget.set_model_color(component_name, color)
                        logger.info(f"关节关联组件 {component_name} 高亮为: {color}")
                        
        except Exception as e:
            logger.error(f"处理组件高亮失败: {e}")
    
    def _process_quick_start(self):
        """处理快速启动"""
        if self.quick_start_result:
            logger.info("处理快速启动：跳过初始化，直接进入模型预览")
            
            # 直接跳转到模型预览阶段
            self.stage_panels.on_project_loaded(self.quick_start_result)
            
            # 在3D视图中显示模型
            self.viz_widget.display_models_from_result(self.quick_start_result)
            
            # 清除快速启动结果
            self.quick_start_result = None
            
            logger.info("快速启动处理完成")
    
    def closeEvent(self, event):
        """窗口关闭事件处理"""
        logger.info("正在关闭简化主窗口...")
        
        # 清理资源
        if self.viz_widget:
            self.viz_widget.cleanup()
        
        if self._data_service:
            self._data_service.stop_loading()
        
        # 清理TransformService数据
        if self._transform_service:
            self._transform_service.clear_data()
        
        super().closeEvent(event)
        logger.info("简化主窗口已安全关闭")


class ApplicationSimplified:
    """简化的应用程序类"""
    
    def __init__(self):
        """初始化应用程序"""
        self.main_window: Optional[MainWindowSimplified] = None
        self._setup_application()
        
    def _setup_application(self):
        """设置应用程序"""
        logger.info("初始化简化应用程序")
        
        # 创建简化主窗口
        self.main_window = MainWindowSimplified()
        
        logger.success("简化应用程序初始化完成")
    
    def run(self):
        """运行应用程序"""
        logger.info("启动简化应用程序")
        
        if self.main_window:
            self.main_window.show()
        else:
            logger.error("简化主窗口未初始化")
            raise RuntimeError("简化主窗口未初始化")
    
    def get_main_window(self) -> MainWindowSimplified:
        """获取主窗口实例"""
        if not self.main_window:
            raise RuntimeError("简化主窗口未初始化")
        return self.main_window


def create_simplified_application() -> ApplicationSimplified:
    """创建简化应用程序实例"""
    logger.info("创建简化应用程序实例")
    return ApplicationSimplified()