"""
阶段配置面板接口

定义各个转换阶段的配置面板接口和基础实现。
"""

from abc import ABC, abstractmethod
from typing import Dict, Any, Optional, List
from pathlib import Path

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
    QPushButton, QFrame, QScrollArea, QFileDialog,
    QLineEdit, QGroupBox, QFormLayout, QTableWidget, 
    QTableWidgetItem, QHeaderView, QProgressBar, 
    QCheckBox, QSpinBox
)
from PySide6.QtCore import Signal, Qt
from PySide6.QtGui import QFont

from utils.logger import logger


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
        input_layout = QHBoxLayout()
        
        self.input_path_edit = QLineEdit()
        self.input_path_edit.setPlaceholderText("选择包含导出数据的目录...")
        self.input_path_edit.setEnabled(False)
        
        input_browse_btn = QPushButton("浏览...")
        input_browse_btn.clicked.connect(self._browse_input_directory)
        
        input_layout.addWidget(self.input_path_edit)
        input_layout.addWidget(input_browse_btn)
        input_group.setLayout(input_layout)
        
        # 输出目录选择
        output_group = QGroupBox("输出目录")
        output_layout = QHBoxLayout()
        
        self.output_path_edit = QLineEdit()
        self.output_path_edit.setPlaceholderText("自动设置为输入目录下的mujoco_template")
        self.output_path_edit.setEnabled(False)
        
        output_browse_btn = QPushButton("浏览...")
        output_browse_btn.clicked.connect(self._browse_output_directory)
        
        output_layout.addWidget(self.output_path_edit)
        output_layout.addWidget(output_browse_btn)
        output_group.setLayout(output_layout)
        
        # 添加到布局
        self.content_layout.addWidget(input_group)
        self.content_layout.addWidget(output_group)
        
        # 隐藏执行按钮（自动跳转到下一阶段）
        self.execute_button.setVisible(False)
        
    def _browse_input_directory(self) -> None:
        """浏览输入目录"""
        directory = QFileDialog.getExistingDirectory(
            self, 
            "选择包含导出数据的目录",
            str(self.input_directory) if self.input_directory else ""
        )
        
        if directory:
            self.input_directory = Path(directory)
            self.input_path_edit.setText(str(self.input_directory))
            
            # 自动设置输出目录
            default_output = self.input_directory / "mujoco_template"
            self.output_directory = default_output
            self.output_path_edit.setText(str(default_output))
            
            # 保存到配置
            self.config.set_parameter("input_directory", str(self.input_directory))
            self.config.set_parameter("output_directory", str(self.output_directory))
            
            # 自动执行初始化阶段
            self._on_execute_clicked()
            
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
        self._create_config_widgets()
        
    def _create_config_widgets(self) -> None:
        """创建配置控件"""
        
        # 数据加载信息
        info_group = QGroupBox("数据加载信息")
        info_layout = QVBoxLayout()
        
        self.info_label = QLabel("等待初始化阶段完成...")
        self.info_label.setStyleSheet("color: gray; padding: 10px;")
        info_layout.addWidget(self.info_label)
        info_group.setLayout(info_layout)
        
        # STL文件表格
        files_group = QGroupBox("STL文件列表")
        files_layout = QVBoxLayout()
        
        self.files_table = QTableWidget()
        self.files_table.setColumnCount(4)
        self.files_table.setHorizontalHeaderLabels(["文件名", "大小 (KB)", "状态", "操作"])
        self.files_table.horizontalHeader().setStretchLastSection(True)
        self.files_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.files_table.setEditTriggers(QTableWidget.NoEditTriggers)
        
        files_layout.addWidget(self.files_table)
        files_group.setLayout(files_layout)
        
        # 加载选项
        options_group = QGroupBox("加载选项")
        options_layout = QVBoxLayout()
        
        # 过滤选项
        filter_layout = QHBoxLayout()
        self.filter_hidden = QCheckBox("过滤隐藏组件")
        self.filter_hidden.setChecked(True)
        self.min_bodies_spin = QSpinBox()
        self.min_bodies_spin.setRange(0, 1000)
        self.min_bodies_spin.setValue(1)
        self.min_bodies_spin.setPrefix("最小实体数: ")
        
        filter_layout.addWidget(self.filter_hidden)
        filter_layout.addWidget(self.min_bodies_spin)
        filter_layout.addStretch()
        
        # 加载进度
        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)
        
        options_layout.addLayout(filter_layout)
        options_layout.addWidget(self.progress_bar)
        options_group.setLayout(options_layout)
        
        # 操作按钮
        button_layout = QHBoxLayout()
        
        self.scan_button = QPushButton("扫描STL文件")
        self.scan_button.clicked.connect(self._scan_stl_files)
        
        self.load_button = QPushButton("加载选中文件")
        self.load_button.clicked.connect(self._load_selected_files)
        self.load_button.setEnabled(False)
        
        self.preview_button = QPushButton("3D预览")
        self.preview_button.clicked.connect(self._preview_3d)
        self.preview_button.setEnabled(False)
        
        button_layout.addWidget(self.scan_button)
        button_layout.addWidget(self.load_button)
        button_layout.addWidget(self.preview_button)
        
        # 添加到布局
        self.content_layout.addWidget(info_group)
        self.content_layout.addWidget(files_group)
        self.content_layout.addWidget(options_group)
        self.content_layout.addLayout(button_layout)
        
        # 修改执行按钮文本
        self.execute_button.setText("完成数据加载")
        
    def set_input_directory(self, directory: Path) -> None:
        """设置输入目录
        
        Args:
            directory: 输入目录路径
        """
        self.input_directory = directory
        self.info_label.setText(f"输入目录: {directory.name}")
        self.info_label.setStyleSheet("color: blue; padding: 10px;")
        
    def _scan_stl_files(self) -> None:
        """扫描STL文件"""
        if not self.input_directory:
            self.stage_error.emit(self.stage_name, "输入目录未设置")
            return
            
        logger.info(f"开始扫描STL文件: {self.input_directory}")
        
        # 查找所有STL文件
        stl_patterns = ["*.stl", "*.STL"]
        self.stl_files = []
        
        for pattern in stl_patterns:
            self.stl_files.extend(self.input_directory.rglob(pattern))
            
        # 更新表格
        self.files_table.setRowCount(len(self.stl_files))
        
        for i, stl_file in enumerate(self.stl_files):
            # 文件名
            name_item = QTableWidgetItem(stl_file.name)
            self.files_table.setItem(i, 0, name_item)
            
            # 文件大小
            try:
                size_kb = stl_file.stat().st_size / 1024
                size_item = QTableWidgetItem(f"{size_kb:.1f}")
            except:
                size_item = QTableWidgetItem("N/A")
            self.files_table.setItem(i, 1, size_item)
            
            # 状态
            status_item = QTableWidgetItem("未加载")
            from PySide6.QtGui import QColor
            status_item.setForeground(QColor(100, 100, 100))  # 灰色
            self.files_table.setItem(i, 2, status_item)
            
            # 操作按钮
            btn_widget = QWidget()
            btn_layout = QHBoxLayout(btn_widget)
            btn_layout.setContentsMargins(0, 0, 0, 0)
            
            preview_btn = QPushButton("预览")
            preview_btn.clicked.connect(lambda checked, file_path=stl_file: self._preview_single_file(file_path))
            
            btn_layout.addWidget(preview_btn)
            self.files_table.setCellWidget(i, 3, btn_widget)
            
        # 更新按钮状态
        if self.stl_files:
            self.load_button.setEnabled(True)
            self.preview_button.setEnabled(True)
            self.info_label.setText(f"找到 {len(self.stl_files)} 个STL文件")
        else:
            self.info_label.setText("未找到STL文件")
            self.info_label.setStyleSheet("color: orange; padding: 10px;")
            
        logger.info(f"扫描完成，找到 {len(self.stl_files)} 个STL文件")
        
    def _load_selected_files(self) -> None:
        """加载选中的文件"""
        selected_rows = set()
        for item in self.files_table.selectedItems():
            selected_rows.add(item.row())
            
        if not selected_rows:
            self.stage_error.emit(self.stage_name, "请先选择要加载的文件")
            return
            
        logger.info(f"开始加载 {len(selected_rows)} 个STL文件")
        
        # 显示进度条
        self.progress_bar.setMaximum(len(selected_rows))
        self.progress_bar.setValue(0)
        self.progress_bar.setVisible(True)
        
        loaded_files = []
        
        for i, row in enumerate(selected_rows):
            if row < len(self.stl_files):
                stl_file = self.stl_files[row]
                
                try:
                    # 更新状态
                    status_item = self.files_table.item(row, 2)
                    status_item.setText("加载中...")
                    from PySide6.QtGui import QColor
                    status_item.setForeground(QColor(0, 0, 255))  # 蓝色
                    
                    # 这里可以添加实际的STL文件验证逻辑
                    # 暂时假设所有文件都有效
                    status_item.setText("已加载")
                    status_item.setForeground(QColor(0, 128, 0))  # 绿色
                    
                    loaded_files.append(stl_file)
                    
                except Exception as e:
                    status_item.setText(f"失败: {str(e)[:20]}")
                    from PySide6.QtGui import QColor
                    status_item.setForeground(QColor(255, 0, 0))  # 红色
                    
                self.progress_bar.setValue(i + 1)
                
        self.progress_bar.setVisible(False)
        
        if loaded_files:
            self.stl_files_loaded.emit(loaded_files)
            logger.success(f"成功加载 {len(loaded_files)} 个STL文件")
        else:
            logger.warning("没有成功加载任何STL文件")
            
    def _preview_3d(self) -> None:
        """3D预览所有已加载文件"""
        loaded_files = []
        
        for row in range(self.files_table.rowCount()):
            status_item = self.files_table.item(row, 2)
            if status_item and status_item.text() == "已加载":
                loaded_files.append(self.stl_files[row])
                
        if loaded_files:
            self.preview_requested.emit(loaded_files)
        else:
            self.stage_error.emit(self.stage_name, "没有已加载的文件可以预览")
            
    def _preview_single_file(self, file_path: Path) -> None:
        """预览单个文件"""
        self.preview_requested.emit([file_path])
        
    def _execute_stage(self) -> bool:
        """执行数据加载阶段"""
        logger.info("开始执行数据加载阶段")
        
        if not self.input_directory:
            self.stage_error.emit(self.stage_name, "输入目录未设置")
            return False
            
        # 检查是否有已加载的文件
        loaded_count = 0
        for row in range(self.files_table.rowCount()):
            status_item = self.files_table.item(row, 2)
            if status_item and status_item.text() == "已加载":
                loaded_count += 1
                
        if loaded_count == 0:
            self.stage_error.emit(self.stage_name, "请先加载至少一个STL文件")
            return False
            
        # 保存配置
        self.config.set_parameter("loaded_files_count", loaded_count)
        self.config.set_parameter("filter_hidden", self.filter_hidden.isChecked())
        self.config.set_parameter("min_bodies", self.min_bodies_spin.value())
        
        logger.success(f"数据加载阶段完成，共加载 {loaded_count} 个文件")
        return True


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
                if hasattr(stage_panel, 'directory_loaded'):
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