"""
MaojocoConverter GUI - 基础框架搭建

第一阶段：创建主窗口和基础布局
"""

from pathlib import Path
from typing import Optional

import pyvista as pv
from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QLabel, QPushButton, QFrame, QSplitter
)
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont

from utils.logger import logger


class MainWindow(QMainWindow):
    """主窗口类
    
    提供应用程序的主界面布局和基础功能。
    """
    
    def __init__(self) -> None:
        super().__init__()
        self._setup_ui()
        self._setup_window()
        
    def _setup_ui(self) -> None:
        """设置UI布局"""
        logger.info("初始化主窗口UI")
        
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
        
        # 设置分割器比例和最小尺寸
        splitter.setStretchFactor(0, 7)
        splitter.setStretchFactor(1, 3)
        
        # 设置右侧面板固定宽度为400像素
        right_panel.setFixedWidth(400)
        
        # 设置左侧面板最小宽度为400像素
        left_panel.setMinimumWidth(400)
        
        logger.success("主窗口UI初始化完成")
    
    def _create_left_panel(self) -> QWidget:
        """创建左侧3D可视化面板"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)
        
        # 标题
        title = QLabel("3D可视化窗口")
        title.setFont(QFont("Arial", 12, QFont.Bold))
        layout.addWidget(title)
        
        # 分隔线
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        layout.addWidget(line)
        
        # 3D视图组件
        from .visualization_widget import VisualizationWidget
        self.viz_widget = VisualizationWidget()
        layout.addWidget(self.viz_widget)
        
        # 控制按钮
        control_layout = QHBoxLayout()
        
        reset_button = QPushButton("重置视图")
        zoom_button = QPushButton("缩放适配")
        
        control_layout.addWidget(reset_button)
        control_layout.addWidget(zoom_button)
        control_layout.addStretch()
        
        # 连接信号槽
        reset_button.clicked.connect(self.viz_widget.reset_view)
        zoom_button.clicked.connect(self.viz_widget.fit_to_screen)
        
        layout.addLayout(control_layout)
        
        return panel
    
    def _create_right_panel(self) -> QWidget:
        """创建右侧配置面板"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)
        
        # 标题 - 将由StageManager更新
        self.right_panel_title = QLabel("配置面板")
        self.right_panel_title.setFont(QFont("Arial", 12, QFont.Bold))
        layout.addWidget(self.right_panel_title)
        
        # 分隔线
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        layout.addWidget(line)
        
        # 阶段配置面板
        from .stage_panels import StageManager
        self.stage_manager = StageManager()
        layout.addWidget(self.stage_manager)
        
        # 连接信号
        self.stage_manager.data_loaded.connect(self._on_data_loaded)
        self.stage_manager.stage_changed.connect(self._on_stage_changed)
        
        # 手动触发一次标题更新以确保初始显示正确
        current_stage = self.stage_manager.get_current_stage()
        if current_stage:
            self._update_right_panel_title(current_stage)
        
        # 添加弹簧
        layout.addStretch()
        
        return panel
    
    def _on_data_loaded(self, data: dict) -> None:
        """数据加载完成处理"""
        logger.info(f"数据加载完成: {data}")
        
        # 如果有数据加载阶段面板，传递输入目录
        data_loading_panel = self.stage_manager.stages.get("data_loading")
        if data_loading_panel and hasattr(data_loading_panel, 'set_input_directory'):
            from pathlib import Path
            input_dir = Path(data.get('input_directory', ''))
            data_loading_panel.set_input_directory(input_dir)
            
    def _on_stage_changed(self, stage_name: str) -> None:
        """阶段切换处理"""
        logger.info(f"阶段切换: {stage_name}")
        
        # 更新右侧面板标题
        self._update_right_panel_title(stage_name)
        
        # 连接数据加载阶段的预览信号
        if stage_name == "data_loading":
            data_loading_panel = self.stage_manager.stages.get("data_loading")
            if data_loading_panel and hasattr(data_loading_panel, 'preview_requested'):
                # 断开之前的连接（避免重复连接）
                try:
                    data_loading_panel.preview_requested.disconnect()
                except:
                    pass
                data_loading_panel.preview_requested.connect(self._on_preview_requested)
                
    def _on_preview_requested(self, file_paths: list) -> None:
        """处理3D预览请求"""
        logger.info(f"收到3D预览请求: {len(file_paths)} 个文件")
        
        if hasattr(self, 'viz_widget') and self.viz_widget:
            # 转换路径格式
            from pathlib import Path
            stl_paths = [Path(fp) if isinstance(fp, str) else fp for fp in file_paths]
            
            # 加载到3D视图
            success = self.viz_widget.load_multiple_stl_models(stl_paths)
            if success:
                logger.success("3D预览加载成功")
            else:
                logger.error("3D预览加载失败")
        else:
            logger.error("3D可视化组件未初始化")
            
    def _update_right_panel_title(self, stage_name: str) -> None:
        """更新右侧面板标题显示当前阶段信息"""
        stage_info = {
            "initialization": {"order": "1/6", "name": "初始化"},
            "data_loading": {"order": "2/6", "name": "数据加载"},
            "relationship_analysis": {"order": "3/6", "name": "关系分析"},
            "unit_conversion": {"order": "4/6", "name": "单位转换"},
            "model_generation": {"order": "5/6", "name": "模型生成"},
            "actuator_generation": {"order": "6/6", "name": "执行器生成"}
        }
        
        if stage_name in stage_info:
            info = stage_info[stage_name]
            title_text = f"配置面板-「{info['order']}」{info['name']}"
            if hasattr(self, 'right_panel_title'):
                self.right_panel_title.setText(title_text)
            
    def _setup_window(self) -> None:
        """设置窗口属性"""
        logger.info("设置主窗口属性")
        
        # 窗口标题
        self.setWindowTitle("MaojocoConverter GUI")
        
        # 窗口大小
        self.resize(1200, 800)
        
        # 窗口图标（如果有的话）
        # self.setWindowIcon(QIcon("resources/icon.png"))
        
        logger.info(f"主窗口设置完成 - 大小: {self.size()}")


class Application:
    """应用程序类
    
    管理应用程序的生命周期和主要组件。
    """
    
    def __init__(self) -> None:
        self.main_window: Optional[MainWindow] = None
        self._setup_application()
        
    def _setup_application(self) -> None:
        """设置应用程序"""
        logger.info("初始化MaojocoConverter GUI应用程序")
        
        # 创建主窗口
        self.main_window = MainWindow()
        
        logger.success("应用程序初始化完成")
    
    def run(self) -> None:
        """运行应用程序"""
        logger.info("启动应用程序")
        
        if self.main_window:
            self.main_window.show()
        else:
            logger.error("主窗口未初始化")
            raise RuntimeError("主窗口未初始化")
    
    def get_main_window(self) -> MainWindow:
        """获取主窗口实例"""
        if not self.main_window:
            raise RuntimeError("主窗口未初始化")
        return self.main_window


def create_application() -> Application:
    """创建应用程序实例
    
    Returns:
        Application: 应用程序实例
    """
    logger.info("创建MaojocoConverter GUI应用程序实例")
    return Application()