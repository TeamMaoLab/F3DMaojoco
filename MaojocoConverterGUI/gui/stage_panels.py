"""
阶段配置面板接口

定义各个转换阶段的配置面板接口和基础实现。
"""

from abc import ABC, abstractmethod
from typing import Dict, Any, Optional, List
from pathlib import Path
import datetime

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
    QPushButton, QFrame, QScrollArea, QFileDialog,
    QLineEdit, QGroupBox, QFormLayout, QTableWidget, 
    QTableWidgetItem, QHeaderView, QProgressBar, 
    QCheckBox, QSpinBox, QTextEdit, QSplitter
)
from PySide6.QtCore import Signal, Qt, QTimer
from PySide6.QtGui import QFont, QColor

from utils.logger import logger
from core.transform_service import TransformService, LoadMode
from .async_data_loader import AsyncDataManager


class StageConfig:
    """阶段配置数据类"""
    
    def __init__(self, stage_name: str, display_name: str) -> None:
        """初始化阶段配置
        
        Args:
            stage_name: 阶段名称
            display_name: 显示名称
        """
        self.stage_name = stage_name
        self.display_name = display_name
        self.parameters: Dict[str, Any] = {}
        self.is_enabled = True
        self.is_completed = False
        
    def set_parameter(self, key: str, value: Any) -> None:
        """设置参数
        
        Args:
            key: 参数键
            value: 参数值
        """
        self.parameters[key] = value
        
    def get_parameter(self, key: str, default: Any = None) -> Any:
        """获取参数
        
        Args:
            key: 参数键
            default: 默认值
            
        Returns:
            Any: 参数值
        """
        return self.parameters.get(key, default)
        
    def validate(self) -> List[str]:
        """验证配置参数
        
        Returns:
            List[str]: 错误信息列表
        """
        return []


class StagePanel(QWidget):
    """阶段配置面板基类"""
    
    # 信号定义
    config_changed = Signal(str, object)  # 配置变更信号
    stage_completed = Signal(str)  # 阶段完成信号
    stage_error = Signal(str, str)  # 阶段错误信号
    
    def __init__(self, stage_name: str, display_name: str, parent: Optional[QWidget] = None) -> None:
        """初始化阶段面板
        
        Args:
            stage_name: 阶段名称
            display_name: 显示名称
            parent: 父窗口部件
        """
        super().__init__(parent)
        self.stage_name = stage_name
        self.display_name = display_name
        self.config = StageConfig(stage_name, display_name)
        self._setup_ui()
        
    def _setup_ui(self) -> None:
        """设置UI界面"""
        logger.info(f"初始化阶段面板: {self.display_name}")
        
        # 创建主布局
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)
        
        # 配置内容区域
        self.content_layout = QVBoxLayout()
        main_layout.addLayout(self.content_layout)
        
        # 操作按钮区域
        button_layout = QHBoxLayout()
        
        self.execute_button = QPushButton(f"执行{self.display_name}")
        self.execute_button.setMinimumHeight(35)
        self.execute_button.clicked.connect(self._on_execute_clicked)
        
        button_layout.addWidget(self.execute_button)
        button_layout.addStretch()
        
        main_layout.addLayout(button_layout)
        
        # 添加弹簧
        main_layout.addStretch()
        
        logger.success(f"阶段面板初始化完成: {self.display_name}")
        
    @abstractmethod
    def _create_config_widgets(self) -> None:
        """创建配置控件（子类实现）"""
        pass
        
    @abstractmethod
    def _execute_stage(self) -> bool:
        """执行阶段逻辑（子类实现）
        
        Returns:
            bool: 是否执行成功
        """
        pass
        
    def _on_execute_clicked(self) -> None:
        """执行按钮点击处理"""
        logger.info(f"开始执行阶段: {self.display_name}")
        
        # 验证配置
        errors = self.config.validate()
        if errors:
            error_msg = f"配置验证失败:\\n" + "\\n".join(errors)
            logger.error(error_msg)
            self.stage_error.emit(self.stage_name, error_msg)
            return
            
        # 执行阶段
        try:
            success = self._execute_stage()
            if success:
                self.config.is_completed = True
                self.stage_completed.emit(self.stage_name)
                logger.success(f"阶段执行完成: {self.display_name}")
            else:
                self.stage_error.emit(self.stage_name, f"阶段执行失败: {self.display_name}")
                
        except Exception as e:
            error_msg = f"阶段执行异常: {e}"
            logger.error(error_msg)
            self.stage_error.emit(self.stage_name, error_msg)
            
    def get_config(self) -> StageConfig:
        """获取阶段配置
        
        Returns:
            StageConfig: 阶段配置
        """
        return self.config
        
    def set_config(self, config: StageConfig) -> None:
        """设置阶段配置
        
        Args:
            config: 阶段配置
        """
        self.config = config
        self._update_ui_from_config()
        
    def _update_ui_from_config(self) -> None:
        """根据配置更新UI（子类实现）"""
        pass
    
    def cleanup(self) -> None:
        """清理资源（子类可根据需要重写）"""
        pass


class InitializationPanel(StagePanel):
    """初始化阶段配置面板"""
    
    # 新增信号
    directory_loaded = Signal(dict)  # 目录加载完成信号
    
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__("initialization", "初始化", parent)
        self.input_directory: Optional[Path] = None
        self.output_directory: Optional[Path] = None
        self._create_config_widgets()
        
    def _create_config_widgets(self) -> None:
        """创建配置控件"""
        
        # 输入目录选择
        input_group = QGroupBox("输入目录")
        input_layout = QVBoxLayout()
        input_layout.setSpacing(10)
        
        # 添加说明文字
        info_label = QLabel("请选择包含导出数据的目录，输出目录将自动创建")
        info_label.setWordWrap(True)
        info_label.setStyleSheet("color: #666; font-size: 12px; padding: 5px;")
        input_layout.addWidget(info_label)
        
        # 输入目录选择行
        dir_select_layout = QHBoxLayout()
        dir_select_layout.setSpacing(10)
        
        self.input_path_edit = QLineEdit()
        self.input_path_edit.setPlaceholderText("点击浏览选择输入目录...")
        self.input_path_edit.setEnabled(False)
        
        input_browse_btn = QPushButton("浏览目录")
        input_browse_btn.setMinimumWidth(100)
        input_browse_btn.clicked.connect(self._browse_input_directory)
        
        dir_select_layout.addWidget(self.input_path_edit, 1)  # 占据更多空间
        dir_select_layout.addWidget(input_browse_btn)
        input_layout.addLayout(dir_select_layout)
        
        input_group.setLayout(input_layout)
        
        # 添加到布局（只显示输入目录，输出目录自动配置）
        self.content_layout.addWidget(input_group)
        
        # 隐藏执行按钮（自动跳转到下一阶段）
        self.execute_button.setVisible(False)
        
        # 添加一些说明信息
        info_group = QGroupBox("说明")
        info_layout = QVBoxLayout()
        
        info_text = QLabel(
            "• 输出目录将自动设置为: [输入目录]/mujoco_template\n"
            "• 选择目录后系统会自动验证必要文件\n"
            "• 验证通过后将自动进入下一阶段"
        )
        info_text.setWordWrap(True)
        info_text.setStyleSheet("color: #555; font-size: 11px; padding: 10px; background-color: #f9f9f9; border-radius: 5px;")
        info_layout.addWidget(info_text)
        
        info_group.setLayout(info_layout)
        self.content_layout.addWidget(info_group)
        
    def _browse_input_directory(self) -> None:
        """浏览输入目录"""
        try:
            # 使用静态方法，避免线程问题
            directory = QFileDialog.getExistingDirectory(
                None,  # 使用 None 而不是 self，避免父窗口问题
                "选择包含导出数据的目录",
                str(self.input_directory) if self.input_directory else str(Path.home()),
                QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks
            )
            
            if directory:
                self.input_directory = Path(directory)
                self.input_path_edit.setText(str(self.input_directory))
                
                # 自动设置输出目录
                default_output = self.input_directory / "mujoco_template"
                self.output_directory = default_output
                
                # 保存到配置
                self.config.set_parameter("input_directory", str(self.input_directory))
                self.config.set_parameter("output_directory", str(self.output_directory))
                
                # 自动执行初始化阶段
                self._on_execute_clicked()
                
        except Exception as e:
            logger.error(f"文件选择器错误: {e}")
            self.stage_error.emit(self.stage_name, f"文件选择器错误: {e}")
            
    def _browse_output_directory(self) -> None:
        """浏览输出目录"""
        directory = QFileDialog.getExistingDirectory(
            self, 
            "选择输出目录",
            str(self.output_directory) if self.output_directory else ""
        )
        
        if directory:
            self.output_directory = Path(directory)
            self.output_path_edit.setText(str(self.output_directory))
            self.config.set_parameter("output_directory", str(self.output_directory))
            
    def _execute_stage(self) -> bool:
        """执行初始化阶段"""
        logger.info("开始执行初始化阶段")
        
        if not self.input_directory:
            self.stage_error.emit(self.stage_name, "请先选择输入目录")
            return False
            
        try:
            # 验证输入目录
            if not self.input_directory.exists():
                self.stage_error.emit(self.stage_name, f"输入目录不存在: {self.input_directory}")
                return False
                
            # 检查必要文件
            required_files = ['component_positions.json', 'export_description.md']
            missing_files = []
            
            for file_name in required_files:
                file_path = self.input_directory / file_name
                if not file_path.exists():
                    missing_files.append(file_name)
                    
            if missing_files:
                error_msg = f"缺少必要文件:\\n" + "\\n".join(missing_files)
                self.stage_error.emit(self.stage_name, error_msg)
                return False
                
            # 创建输出目录
            if not self.output_directory.exists():
                self.output_directory.mkdir(parents=True, exist_ok=True)
                logger.info(f"创建输出目录: {self.output_directory}")
            
            # 发送加载完成信号
            load_data = {
                "input_directory": str(self.input_directory),
                "output_directory": str(self.output_directory),
                "required_files": required_files,
                "status": "success"
            }
            self.directory_loaded.emit(load_data)
            
            logger.success("初始化阶段执行完成")
            return True
            
        except Exception as e:
            error_msg = f"初始化失败: {e}"
            logger.error(error_msg)
            self.stage_error.emit(self.stage_name, error_msg)
            return False


class DataLoadingPanel(StagePanel):
    """数据加载阶段配置面板"""
    
    # 新增信号
    stl_files_loaded = Signal(list)  # STL文件加载完成信号
    preview_requested = Signal(list)  # 3D预览请求信号
    
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__("data_loading", "数据加载", parent)
        self.input_directory: Optional[Path] = None
        self.stl_files: List[Path] = []
        self.project_info = None
        self._transform_service: Optional[TransformService] = None
        self._async_manager = None
        self._is_loading = False
        self._create_config_widgets()
        
    def _create_config_widgets(self) -> None:
        """创建配置控件"""
        
        # 创建主分割器
        main_splitter = QSplitter(Qt.Vertical)
        
        # 上部：项目信息区域
        top_widget = QWidget()
        top_layout = QVBoxLayout(top_widget)
        
        # 项目基本信息
        project_group = QGroupBox("项目信息")
        project_layout = QVBoxLayout()
        
        self.project_info_label = QLabel("等待扫描项目结构...")
        self.project_info_label.setWordWrap(True)
        self.project_info_label.setStyleSheet("color: #666; font-size: 12px; padding: 10px; background-color: #f9f9f9; border-radius: 5px;")
        project_layout.addWidget(self.project_info_label)
        project_group.setLayout(project_layout)
        
        # 状态信息
        status_group = QGroupBox("加载状态")
        status_layout = QVBoxLayout()
        
        self.status_label = QLabel("准备就绪")
        self.status_label.setStyleSheet("color: blue; font-weight: bold; padding: 5px;")
        status_layout.addWidget(self.status_label)
        
        # 总体进度条
        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)
        status_layout.addWidget(self.progress_bar)
        
        status_group.setLayout(status_layout)
        
        top_layout.addWidget(project_group)
        top_layout.addWidget(status_group)
        
        # 中部：详细日志
        log_group = QGroupBox("详细日志")
        log_layout = QVBoxLayout()
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(150)
        self.log_text.setStyleSheet("font-family: monospace; font-size: 10px; background-color: #f5f5f5;")
        log_layout.addWidget(self.log_text)
        
        log_group.setLayout(log_layout)
        
        # 下部：STL文件列表
        files_group = QGroupBox("STL文件列表")
        files_layout = QVBoxLayout()
        
        self.files_table = QTableWidget()
        self.files_table.setColumnCount(4)
        self.files_table.setHorizontalHeaderLabels(["文件名", "大小 (KB)", "状态", "进度"])
        self.files_table.horizontalHeader().setStretchLastSection(True)
        self.files_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.files_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.files_table.setMaximumHeight(200)
        
        files_layout.addWidget(self.files_table)
        files_group.setLayout(files_layout)
        
        # 操作按钮区域
        button_widget = QWidget()
        button_layout = QHBoxLayout(button_widget)
        
        self.cancel_button = QPushButton("取消加载")
        self.cancel_button.clicked.connect(self._cancel_loading)
        self.cancel_button.setEnabled(False)
        
        self.retry_button = QPushButton("重试")
        self.retry_button.clicked.connect(self._retry_loading)
        self.retry_button.setEnabled(False)
        
        self.preview_button = QPushButton("3D预览")
        self.preview_button.clicked.connect(self._preview_3d)
        self.preview_button.setEnabled(False)
        
        button_layout.addWidget(self.cancel_button)
        button_layout.addWidget(self.retry_button)
        button_layout.addStretch()
        button_layout.addWidget(self.preview_button)
        
        # 添加到主分割器
        main_splitter.addWidget(top_widget)
        main_splitter.addWidget(log_group)
        main_splitter.addWidget(files_group)
        main_splitter.addWidget(button_widget)
        
        # 设置分割器比例
        main_splitter.setSizes([200, 150, 200, 60])
        
        self.content_layout.addWidget(main_splitter)
        
        # 修改执行按钮文本和状态
        self.execute_button.setText("完成数据加载")
        self.execute_button.setEnabled(False)  # 自动加载期间禁用
        
        # 初始化异步管理器
        self._async_manager = AsyncDataManager()
            
    def set_transform_service(self, transform_service: TransformService) -> None:
        """设置TransformService
        
        Args:
            transform_service: 变换服务实例
        """
        self._transform_service = transform_service
        
    def set_input_directory(self, directory: Path) -> None:
        """设置输入目录并开始自动加载"""
        self.input_directory = directory
        self.status_label.setText(f"输入目录: {directory.name}")
        self._add_log(f"设置输入目录: {directory}")
        
        # 自动开始加载
        if self._async_manager:
            self._start_async_loading()
        else:
            self._add_log("警告：异步管理器不可用，请手动操作")
            
    def _start_async_loading(self) -> None:
        """开始异步加载"""
        if not self.input_directory or not self._async_manager:
            return
            
        self._is_loading = True
        self._reset_ui()
        
        self._add_log("开始自动数据加载...")
        self.status_label.setText("正在扫描项目结构...")
        
        # 显示进度条
        self.progress_bar.setVisible(True)
        self.progress_bar.setValue(0)
        
        # 启用取消按钮
        self.cancel_button.setEnabled(True)
        
        # 开始异步加载
        self._async_manager.start_data_loading(
            self.input_directory,
            progress_callback=self._on_progress_updated,
            scan_complete_callback=self._on_scan_completed,
            loading_complete_callback=self._on_loading_completed,
            error_callback=self._on_error_occurred
        )
        
    def _on_progress_updated(self, progress: int, message: str) -> None:
        """进度更新回调"""
        self.progress_bar.setValue(progress)
        self.status_label.setText(message)
        self._add_log(f"[{progress:3d}%] {message}")
        
    def _on_scan_completed(self, json_data: dict, project_info) -> None:
        """扫描完成回调"""
        self.project_info = project_info
        
        # 更新项目信息显示
        info_text = f"""项目信息：
• 导出时间: {project_info.export_time}
• 几何单位: {project_info.geometry_unit}
• 位置单位: {project_info.position_unit}
• 组件数量: {project_info.component_count}
• 关节数量: {project_info.joint_count}
• 格式版本: {project_info.format_version}"""
        
        self.project_info_label.setText(info_text)
        self._add_log(f"项目扫描完成: {project_info.component_count} 个组件, {project_info.joint_count} 个关节")
        
    def _on_loading_completed(self, successful_files: list, failed_files: list) -> None:
        """加载完成回调"""
        self._is_loading = False
        self.progress_bar.setValue(100)
        
        # 更新文件表格
        self._update_files_table(successful_files, failed_files)
        
        # 加载STL文件到3D视图
        if successful_files:
            self._add_log(f"开始加载 {len(successful_files)} 个STL文件到3D视图...")
            self.preview_requested.emit(successful_files)
            
        # 更新UI状态
        self.status_label.setText("加载完成！")
        self.cancel_button.setEnabled(False)
        self.preview_button.setEnabled(len(successful_files) > 0)
        self.execute_button.setEnabled(True)
        
        # 显示结果摘要
        summary = f"加载完成！成功: {len(successful_files)}, 失败: {len(failed_files)}"
        self._add_log(summary)
        
        if failed_files:
            self.retry_button.setEnabled(True)
            self._add_log(f"失败的文件: {[(f[0].name, f[1]) for f in failed_files]}")
            
    def _on_error_occurred(self, error_msg: str) -> None:
        """错误发生回调"""
        self._is_loading = False
        self.status_label.setText("加载失败")
        self.status_label.setStyleSheet("color: red; font-weight: bold; padding: 5px;")
        self._add_log(f"错误: {error_msg}")
        
        self.cancel_button.setEnabled(False)
        self.retry_button.setEnabled(True)
        self.execute_button.setEnabled(True)
        
    def _cancel_loading(self) -> None:
        """取消加载"""
        if self._async_manager:
            self._async_manager.stop_loading()
        self._is_loading = False
        self.status_label.setText("加载已取消")
        self._add_log("用户取消了加载操作")
        
        self.cancel_button.setEnabled(False)
        self.retry_button.setEnabled(True)
        
    def _retry_loading(self) -> None:
        """重试加载"""
        self._start_async_loading()
        self.retry_button.setEnabled(False)
        
    def _reset_ui(self) -> None:
        """重置UI状态"""
        self.status_label.setText("准备就绪")
        self.status_label.setStyleSheet("color: blue; font-weight: bold; padding: 5px;")
        self.progress_bar.setValue(0)
        self.files_table.setRowCount(0)
        self.log_text.clear()
        self.cancel_button.setEnabled(False)
        self.retry_button.setEnabled(False)
        self.preview_button.setEnabled(False)
        
    def _update_files_table(self, successful_files: list, failed_files: list) -> None:
        """更新文件表格"""
        self.files_table.setRowCount(len(successful_files) + len(failed_files))
        
        row = 0
        # 添加成功文件
        for file_path in successful_files:
            # 文件名
            name_item = QTableWidgetItem(file_path.name)
            self.files_table.setItem(row, 0, name_item)
            
            # 文件大小
            try:
                size_kb = file_path.stat().st_size / 1024
                size_item = QTableWidgetItem(f"{size_kb:.1f}")
            except:
                size_item = QTableWidgetItem("N/A")
            self.files_table.setItem(row, 1, size_item)
            
            # 状态
            status_item = QTableWidgetItem("成功")
            status_item.setForeground(QColor(0, 128, 0))  # 绿色
            self.files_table.setItem(row, 2, status_item)
            
            # 进度
            progress_item = QTableWidgetItem("✓")
            progress_item.setTextAlignment(Qt.AlignCenter)
            self.files_table.setItem(row, 3, progress_item)
            
            row += 1
            
        # 添加失败文件
        for file_path, error_msg in failed_files:
            # 文件名
            name_item = QTableWidgetItem(file_path.name)
            self.files_table.setItem(row, 0, name_item)
            
            # 文件大小
            size_item = QTableWidgetItem("N/A")
            self.files_table.setItem(row, 1, size_item)
            
            # 状态
            status_item = QTableWidgetItem("失败")
            status_item.setForeground(QColor(255, 0, 0))  # 红色
            self.files_table.setItem(row, 2, status_item)
            
            # 错误信息
            error_item = QTableWidgetItem(error_msg[:20] + "..." if len(error_msg) > 20 else error_msg)
            error_item.setForeground(QColor(255, 0, 0))
            self.files_table.setItem(row, 3, error_item)
            
            row += 1
            
    def _add_log(self, message: str) -> None:
        """添加日志消息"""
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}\n"
        
        self.log_text.append(log_entry)
        # 滚动到底部
        scrollbar = self.log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
        
    def _preview_3d(self) -> None:
        """3D预览功能"""
        if self.input_directory:
            # 查找所有成功的STL文件
            stl_files = []
            for row in range(self.files_table.rowCount()):
                status_item = self.files_table.item(row, 2)
                if status_item and status_item.text() == "成功":
                    file_name = self.files_table.item(row, 0).text()
                    file_path = self.input_directory / "stl_files" / file_name
                    if file_path.exists():
                        stl_files.append(file_path)
                        
            if stl_files:
                self.preview_requested.emit(stl_files)
            else:
                self._add_log("没有可预览的有效STL文件")
                
    def _execute_stage(self) -> bool:
        """执行数据加载阶段"""
        logger.info("开始执行数据加载阶段")
        
        if not self.input_directory:
            self.stage_error.emit(self.stage_name, "输入目录未设置")
            return False
            
        # 检查是否有成功加载的文件
        successful_count = 0
        for row in range(self.files_table.rowCount()):
            status_item = self.files_table.item(row, 2)
            if status_item and status_item.text() == "成功":
                successful_count += 1
                
        if successful_count == 0:
            self.stage_error.emit(self.stage_name, "没有成功加载任何STL文件")
            return False
            
        # 保存配置
        self.config.set_parameter("loaded_files_count", successful_count)
        self.config.set_parameter("input_directory", str(self.input_directory))
        if self.project_info:
            self.config.set_parameter("project_info", {
                "component_count": self.project_info.component_count,
                "joint_count": self.project_info.joint_count,
                "export_time": self.project_info.export_time
            })
        
        logger.success(f"数据加载阶段完成，共加载 {successful_count} 个文件")
        return True
    
    def cleanup(self) -> None:
        """清理异步管理器资源"""
        if self._async_manager:
            logger.info("清理数据加载面板的异步管理器...")
            self._async_manager.stop_loading()


class RelationshipAnalysisPanel(StagePanel):
    """关系分析阶段配置面板"""
    
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__("relationship_analysis", "关系分析", parent)
        self._create_config_widgets()
        
    def _create_config_widgets(self) -> None:
        """创建配置控件"""
        # 这里可以添加具体的配置控件
        pass
        
    def _execute_stage(self) -> bool:
        """执行关系分析阶段"""
        # 这里实现具体的关系分析逻辑
        return True


class UnitConversionPanel(StagePanel):
    """单位转换阶段配置面板"""
    
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__("unit_conversion", "单位转换", parent)
        self._create_config_widgets()
        
    def _create_config_widgets(self) -> None:
        """创建配置控件"""
        # 这里可以添加具体的配置控件
        pass
        
    def _execute_stage(self) -> bool:
        """执行单位转换阶段"""
        # 这里实现具体的单位转换逻辑
        return True


class ModelGenerationPanel(StagePanel):
    """模型生成阶段配置面板"""
    
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__("model_generation", "模型生成", parent)
        self._create_config_widgets()
        
    def _create_config_widgets(self) -> None:
        """创建配置控件"""
        # 这里可以添加具体的配置控件
        pass
        
    def _execute_stage(self) -> bool:
        """执行模型生成阶段"""
        # 这里实现具体的模型生成逻辑
        return True


class ActuatorGenerationPanel(StagePanel):
    """执行器生成阶段配置面板"""
    
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__("actuator_generation", "执行器生成", parent)
        self._create_config_widgets()
        
    def _create_config_widgets(self) -> None:
        """创建配置控件"""
        # 这里可以添加具体的配置控件
        pass
        
    def _execute_stage(self) -> bool:
        """执行执行器生成阶段"""
        # 这里实现具体的执行器生成逻辑
        return True


class StageManager(QWidget):
    """阶段管理器"""
    
    # 信号定义
    all_stages_completed = Signal()  # 所有阶段完成信号
    stage_execution_started = Signal(str)  # 阶段执行开始信号
    stage_changed = Signal(str)  # 阶段切换信号
    data_loaded = Signal(dict)  # 数据加载完成信号
    
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        """初始化阶段管理器"""
        super().__init__(parent)
        self.stages: Dict[str, StagePanel] = {}
        self.current_stage: Optional[str] = None
        self.stage_widgets: Dict[str, QWidget] = {}
        self._setup_ui()
        
    def _setup_ui(self) -> None:
        """设置UI界面"""
        logger.info("初始化阶段管理器")
        
        # 创建主布局
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        
        # 创建当前阶段显示区域
        self.current_stage_widget = QWidget()
        self.current_stage_layout = QVBoxLayout(self.current_stage_widget)
        self.current_stage_layout.setContentsMargins(10, 10, 10, 10)
        self.current_stage_layout.setSpacing(10)
        
        main_layout.addWidget(self.current_stage_widget)
        
        # 创建各阶段面板（初始隐藏）
        stage_classes = [
            InitializationPanel,
            DataLoadingPanel,
            RelationshipAnalysisPanel,
            UnitConversionPanel,
            ModelGenerationPanel,
            ActuatorGenerationPanel
        ]
        
        for stage_class in stage_classes:
            stage_panel = stage_class()
            stage_panel.setVisible(False)  # 初始隐藏
            self.stages[stage_panel.stage_name] = stage_panel
            self.stage_widgets[stage_panel.stage_name] = stage_panel
            
            # 连接信号
            stage_panel.stage_completed.connect(self._on_stage_completed)
            stage_panel.stage_error.connect(self._on_stage_error)
            
            # 如果是初始化面板，设置特殊信号连接
            if stage_panel.stage_name == "initialization":
                stage_panel.directory_loaded.connect(self._on_directory_loaded)
                    
            main_layout.addWidget(stage_panel)
        
        # 默认显示初始化阶段
        self.switch_to_stage("initialization")
        
        # 触发初始阶段切换信号以确保标题正确显示
        self.stage_changed.emit("initialization")
        
        logger.success("阶段管理器初始化完成")
        
    def switch_to_stage(self, stage_name: str) -> None:
        """切换到指定阶段
        
        Args:
            stage_name: 阶段名称
        """
        if stage_name not in self.stages:
            logger.error(f"未知阶段: {stage_name}")
            return
            
        # 隐藏所有阶段面板
        for stage_widget in self.stage_widgets.values():
            stage_widget.setVisible(False)
            
        # 显示当前阶段面板
        current_stage_widget = self.stage_widgets[stage_name]
        current_stage_widget.setVisible(True)
        
        self.current_stage = stage_name
        self.stage_changed.emit(stage_name)
        
        logger.info(f"切换到阶段: {stage_name}")
        
    def _on_stage_completed(self, stage_name: str) -> None:
        """阶段完成处理"""
        logger.info(f"阶段完成: {stage_name}")
        
        # 检查是否所有阶段都完成
        if all(stage.config.is_completed for stage in self.stages.values()):
            self.all_stages_completed.emit()
            
    def _on_stage_error(self, stage_name: str, error_msg: str) -> None:
        """阶段错误处理"""
        logger.error(f"阶段错误: {stage_name} - {error_msg}")
        
    def _on_directory_loaded(self, data: dict) -> None:
        """目录加载完成处理"""
        logger.info(f"目录加载完成: {data.get('directory', 'Unknown')}")
        self.data_loaded.emit(data)
        
        # 自动切换到数据加载阶段
        self.switch_to_stage("data_loading")
        
    def execute_all_stages(self) -> None:
        """执行所有阶段"""
        logger.info("开始执行所有阶段")
        
        # 重置所有阶段状态
        for stage in self.stages.values():
            stage.config.is_completed = False
            
        # 从当前阶段开始执行
        if self.current_stage:
            self.stage_execution_started.emit(self.current_stage)
                
    def get_stage_config(self, stage_name: str) -> Optional[StageConfig]:
        """获取阶段配置
        
        Args:
            stage_name: 阶段名称
            
        Returns:
            Optional[StageConfig]: 阶段配置
        """
        return self.stages.get(stage_name, {}).get_config() if stage_name in self.stages else None
        
    def set_stage_config(self, stage_name: str, config: StageConfig) -> None:
        """设置阶段配置
        
        Args:
            stage_name: 阶段名称
            config: 阶段配置
        """
        if stage_name in self.stages:
            self.stages[stage_name].set_config(config)
            
    def get_current_stage(self) -> Optional[str]:
        """获取当前阶段
        
        Returns:
            Optional[str]: 当前阶段名称
        """
        return self.current_stage