"""
项目数据服务

整合数据加载和管理逻辑，从 async_data_loader.py 迁移核心功能。
提供同步和异步的项目数据加载、验证和扫描功能。
"""

import json
from typing import List, Optional, Callable, Dict, Any
from pathlib import Path
from dataclasses import dataclass
from PySide6.QtCore import QObject, Signal, QThread, QTimer

from core.domain_types import ExportData, ProjectInfo, ComponentInfo, create_default_metadata
from utils.logger import logger


@dataclass
class LoadingProgress:
    """加载进度信息"""
    current: int
    total: int
    message: str
    percentage: float = 0.0


class DataLoaderWorker(QObject):
    """数据加载工作线程"""
    
    # 信号定义
    progress_updated = Signal(int, str)        # 进度百分比, 状态信息
    scan_completed = Signal(dict, ProjectInfo)  # 扫描完成信号, 项目信息
    loading_started = Signal(int)               # 开始加载信号, 文件总数
    file_loading = Signal(int, str, str)       # 文件加载进度, 文件名, 状态
    loading_completed = Signal(list, list)      # 加载完成信号, 成功列表, 失败列表
    error_occurred = Signal(str)                # 错误信号
    
    def __init__(self, input_directory: Path) -> None:
        """初始化数据加载器
        
        Args:
            input_directory: 输入目录路径
        """
        super().__init__()
        self.input_directory = input_directory
        self._is_running = True
        self._thread = None
        
    def set_thread(self, thread: QThread) -> None:
        """设置工作线程"""
        self._thread = thread
        self.moveToThread(thread)
        
    def start_loading(self) -> None:
        """开始加载数据"""
        if self._thread:
            QTimer.singleShot(0, self._load_data)
    
    def stop_loading(self) -> None:
        """停止加载"""
        self._is_running = False
        
    def _load_data(self) -> None:
        """执行数据加载的主逻辑"""
        try:
            # 步骤1: 扫描和解析JSON文件
            if not self._is_running:
                return
                
            self.progress_updated.emit(10, "正在扫描项目结构...")
            
            json_data, project_info = self._scan_project_structure()
            if not json_data:
                self.error_occurred("无法找到或解析项目数据文件")
                return
                
            self.scan_completed.emit(json_data, project_info)
            
            # 步骤2: 查找STL文件
            if not self._is_running:
                return
                
            self.progress_updated.emit(20, "正在查找STL文件...")
            stl_files = self._find_stl_files(json_data)
            
            if not stl_files:
                self.error_occurred("未找到任何STL文件")
                return
                
            # 步骤3: 验证文件完整性
            if not self._is_running:
                return
                
            self.progress_updated.emit(30, "正在验证文件完整性...")
            valid_files, missing_files = self._validate_files(stl_files)
            
            if missing_files:
                self.progress_updated.emit(
                    35, 
                    f"发现 {len(missing_files)} 个缺失文件，将加载 {len(valid_files)} 个有效文件"
                )
                
            # 步骤4: 开始批量加载STL文件
            if not self._is_running:
                return
                
            self.loading_started.emit(len(valid_files))
            
            successful_files = []
            failed_files = []
            
            for i, stl_file in enumerate(valid_files):
                if not self._is_running:
                    break
                    
                progress = 30 + int((i + 1) / len(valid_files) * 65)
                file_name = stl_file.name
                
                self.file_loading.emit(i + 1, file_name, "加载中...")
                
                try:
                    # 这里只是模拟文件加载，实际的STL加载在主线程进行
                    # 因为PyVista的渲染操作必须在主线程执行
                    successful_files.append(stl_file)
                    self.file_loading.emit(i + 1, file_name, "成功")
                    
                except Exception as e:
                    failed_files.append((stl_file, str(e)))
                    self.file_loading.emit(i + 1, file_name, f"失败: {str(e)[:30]}")
                    
                self.progress_updated.emit(progress, f"加载进度: {i + 1}/{len(valid_files)}")
                
            # 步骤5: 完成
            if self._is_running:
                self.progress_updated.emit(100, "加载完成！")
                self.loading_completed.emit(successful_files, failed_files)
            else:
                self.progress_updated.emit(0, "加载已取消")
                
        except Exception as e:
            self.error_occurred(f"数据加载失败: {str(e)}")
            
    def _scan_project_structure(self) -> tuple[Optional[dict], Optional[ProjectInfo]]:
        """扫描项目结构，解析JSON文件"""
        try:
            # 查找component_positions.json文件
            json_file = self.input_directory / "component_positions.json"
            if not json_file.exists():
                return None, None
                
            with open(json_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
                
            # 解析项目信息
            meta = data.get('meta', {})
            project_info = ProjectInfo(
                project_directory=str(self.input_directory),
                export_time=meta.get('export_time', 'Unknown'),
                geometry_unit=meta.get('geometry_unit', 'millimeters'),
                position_unit=meta.get('position_unit', 'millimeters'),
                component_count=meta.get('count_components', 0),
                joint_count=meta.get('count_joints', 0),
                format_version=meta.get('format_version', '1.0'),
                has_transform_data='components' in data and len(data['components']) > 0
            )
            
            return data, project_info
            
        except Exception as e:
            logger.error(f"解析项目文件失败: {str(e)}")
            return None, None
            
    def _find_stl_files(self, json_data: dict) -> List[Path]:
        """从JSON数据中提取STL文件路径"""
        stl_files = []
        components = json_data.get('components', [])
        
        for component in components:
            stl_path = component.get('stl_file')
            if stl_path:
                full_path = self.input_directory / stl_path
                if full_path.exists():
                    stl_files.append(full_path)
                else:
                    logger.warning(f"STL文件不存在: {full_path}")
                    
        return stl_files
        
    def _validate_files(self, stl_files: List[Path]) -> tuple[List[Path], List[Path]]:
        """验证STL文件完整性"""
        valid_files = []
        missing_files = []
        
        for stl_file in stl_files:
            if stl_file.exists() and stl_file.stat().st_size > 0:
                valid_files.append(stl_file)
            else:
                missing_files.append(stl_file)
                
        return valid_files, missing_files


class ProjectDataService(QObject):
    """项目数据服务 - 提供同步和异步的数据加载功能"""
    
    # 信号定义
    progress_updated = Signal(int, str)        # 进度更新
    scan_completed = Signal(dict, ProjectInfo)  # 扫描完成
    loading_started = Signal(int)               # 开始加载
    file_loading = Signal(int, str, str)       # 文件加载进度
    loading_completed = Signal(list, list)      # 加载完成
    error_occurred = Signal(str)                # 错误发生
    
    def __init__(self):
        """初始化项目数据服务"""
        super().__init__()
        self._current_project: Optional[Path] = None
        self._loaded_data: Optional[ExportData] = None
        self._worker = None
        self._thread = None
        self._is_cleaning_up = False
        
    def scan_project(self, directory: Path) -> ProjectInfo:
        """同步扫描项目结构
        
        Args:
            directory: 项目目录路径
            
        Returns:
            ProjectInfo: 项目信息
        """
        try:
            logger.info(f"扫描项目: {directory}")
            
            # 查找component_positions.json文件
            json_file = directory / "component_positions.json"
            if not json_file.exists():
                raise FileNotFoundError(f"未找到项目数据文件: {json_file}")
                
            with open(json_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
                
            # 解析项目信息
            meta = data.get('meta', {})
            project_info = ProjectInfo(
                project_directory=str(directory),
                export_time=meta.get('export_time', 'Unknown'),
                geometry_unit=meta.get('geometry_unit', 'millimeters'),
                position_unit=meta.get('position_unit', 'millimeters'),
                component_count=meta.get('count_components', 0),
                joint_count=meta.get('count_joints', 0),
                format_version=meta.get('format_version', '1.0'),
                has_transform_data='components' in data and len(data['components']) > 0
            )
            
            logger.success(f"项目扫描成功: {project_info.component_count} 个组件")
            return project_info
            
        except Exception as e:
            logger.error(f"扫描项目失败: {e}")
            raise
    
    def load_project_data(self, directory: Path) -> ExportData:
        """同步加载项目数据
        
        Args:
            directory: 项目目录路径
            
        Returns:
            ExportData: 导出数据
        """
        try:
            logger.info(f"加载项目数据: {directory}")
            
            # 扫描项目
            project_info = self.scan_project(directory)
            
            # 加载JSON数据
            json_file = directory / "component_positions.json"
            with open(json_file, 'r', encoding='utf-8') as f:
                json_data = json.load(f)
            
            # 构建ExportData
            components = self._load_components(json_data)
            joints = self._load_joints(json_data)
            
            export_data = ExportData(
                meta=create_default_metadata(
                    component_count=len(components),
                    joint_count=len(joints)
                ),
                components=components,
                joints=joints
            )
            
            self._current_project = directory
            self._loaded_data = export_data
            
            logger.success(f"项目数据加载成功: {len(components)} 个组件, {len(joints)} 个关节")
            return export_data
            
        except Exception as e:
            logger.error(f"加载项目数据失败: {e}")
            raise
    
    def validate_project_structure(self, directory: Path) -> List[str]:
        """验证项目结构
        
        Args:
            directory: 项目目录路径
            
        Returns:
            List[str]: 错误信息列表（空列表表示无错误）
        """
        errors = []
        
        try:
            # 检查必需文件
            json_file = directory / "component_positions.json"
            if not json_file.exists():
                errors.append(f"缺少项目数据文件: component_positions.json")
                return errors
            
            # 解析JSON验证基本结构
            with open(json_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            # 验证必需字段
            if 'meta' not in data:
                errors.append("缺少元数据信息")
            
            if 'components' not in data:
                errors.append("缺少组件信息")
            else:
                components = data['components']
                for i, component in enumerate(components):
                    if not isinstance(component, dict):
                        errors.append(f"组件 {i} 格式错误")
                        continue
                    
                    if 'stl_file' not in component:
                        errors.append(f"组件 {component.get('name', i)} 缺少STL文件路径")
                        continue
                    
                    # 检查STL文件是否存在
                    stl_path = directory / component['stl_file']
                    if not stl_path.exists():
                        errors.append(f"STL文件不存在: {component['stl_file']}")
            
            # 验证joints字段（如果存在）
            if 'joints' in data:
                joints = data['joints']
                for i, joint in enumerate(joints):
                    if not isinstance(joint, dict):
                        errors.append(f"关节 {i} 格式错误")
                        continue
                    
                    if 'name' not in joint:
                        errors.append(f"关节 {i} 缺少名称")
            
            logger.info(f"项目结构验证完成: 发现 {len(errors)} 个问题")
            
        except Exception as e:
            errors.append(f"验证项目结构时发生错误: {str(e)}")
            logger.error(f"验证项目结构失败: {e}")
        
        return errors
    
    def start_async_loading(self, directory: Path) -> None:
        """开始异步数据加载
        
        Args:
            directory: 项目目录路径
        """
        # 停止现有的加载
        self.stop_loading()
        
        # 创建工作线程和工作对象
        self._thread = QThread()
        self._worker = DataLoaderWorker(directory)
        self._worker.set_thread(self._thread)
        
        # 连接信号
        self._worker.progress_updated.connect(self.progress_updated)
        self._worker.scan_completed.connect(self.scan_completed)
        self._worker.loading_started.connect(self.loading_started)
        self._worker.file_loading.connect(self.file_loading)
        self._worker.loading_completed.connect(self.loading_completed)
        self._worker.error_occurred.connect(self.error_occurred)
        
        # 连接线程完成信号
        self._thread.finished.connect(self._cleanup_thread)
        
        # 启动线程
        self._thread.started.connect(self._worker.start_loading)
        self._thread.start()
        
        logger.info(f"开始异步加载项目: {directory}")
    
    def stop_loading(self) -> None:
        """停止数据加载"""
        if self._worker:
            self._worker.stop_loading()
        self._cleanup_thread()
        
    def get_current_project(self) -> Optional[Path]:
        """获取当前项目目录"""
        return self._current_project
        
    def get_loaded_data(self) -> Optional[ExportData]:
        """获取已加载的数据"""
        return self._loaded_data
    
    def _load_components(self, json_data: dict) -> List[ComponentInfo]:
        """从JSON数据加载组件信息"""
        components = []
        
        for component_data in json_data.get('components', []):
            try:
                # 转换为强类型组件信息
                component = ComponentInfo(
                    name=component_data.get('name', ''),
                    occurrence_name=component_data.get('occurrence_name', ''),
                    full_path_name=component_data.get('full_path_name', ''),
                    component_id=component_data.get('component_id', 0),
                    stl_file=component_data.get('stl_file'),
                    bodies_count=component_data.get('bodies_count', 0),
                    has_children=component_data.get('has_children', False),
                    world_transform=None  # 暂时设为None，后续可根据需要添加
                )
                components.append(component)
                
            except Exception as e:
                logger.warning(f"加载组件信息失败: {e}")
                continue
        
        return components
    
    def _load_joints(self, json_data: dict) -> List:
        """从JSON数据加载关节数据"""
        # 暂时返回空列表，后续可根据domain_types中的JointInfo进行实现
        return json_data.get('joints', [])
    
    def _cleanup_thread(self) -> None:
        """清理线程资源"""
        if self._is_cleaning_up:
            return
            
        self._is_cleaning_up = True
        
        if self._thread:
            if self._thread.isRunning():
                self._thread.quit()
                self._thread.wait(1000)  # 等待最多1秒
            
            if self._thread.isFinished():
                self._thread.deleteLater()
            self._thread = None
        self._worker = None
        
        self._is_cleaning_up = False
        
        logger.info("异步加载线程清理完成")