"""
异步数据加载器

使用PySide6的QThread和信号槽机制实现异步数据加载，
避免阻塞UI线程，提供进度反馈和错误处理。
"""

import json
from typing import Dict, List, Any, Optional
from pathlib import Path
from dataclasses import dataclass
from PySide6.QtCore import QObject, Signal, QThread, QTimer


@dataclass
class ProjectInfo:
    """项目信息数据类"""
    export_time: str
    geometry_unit: str
    position_unit: str
    component_count: int
    joint_count: int
    format_version: str


@dataclass
class ComponentInfo:
    """组件信息数据类"""
    name: str
    occurrence_name: str
    component_id: int
    stl_file: str
    bodies_count: int
    has_children: bool
    world_transform: List[List[float]]


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
                export_time=meta.get('export_time', 'Unknown'),
                geometry_unit=meta.get('geometry_unit', 'Unknown'),
                position_unit=meta.get('position_unit', 'Unknown'),
                component_count=meta.get('count_components', 0),
                joint_count=meta.get('count_joints', 0),
                format_version=meta.get('format_version', 'Unknown')
            )
            
            return data, project_info
            
        except Exception as e:
            self.error_occurred(f"解析项目文件失败: {str(e)}")
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
                    # 记录缺失文件
                    pass
                    
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


class AsyncDataManager:
    """异步数据管理器"""
    
    def __init__(self) -> None:
        """初始化异步数据管理器"""
        self._worker = None
        self._thread = None
        self._is_cleaning_up = False
        
    def start_data_loading(self, input_directory: Path, 
                          progress_callback=None,
                          scan_complete_callback=None,
                          loading_complete_callback=None,
                          error_callback=None) -> None:
        """开始异步数据加载
        
        Args:
            input_directory: 输入目录
            progress_callback: 进度回调函数
            scan_complete_callback: 扫描完成回调函数
            loading_complete_callback: 加载完成回调函数
            error_callback: 错误回调函数
        """
        # 停止现有的加载
        self.stop_loading()
        
        # 创建工作线程和工作对象
        self._thread = QThread()
        self._worker = DataLoaderWorker(input_directory)
        self._worker.set_thread(self._thread)
        
        # 连接信号
        if progress_callback:
            self._worker.progress_updated.connect(progress_callback)
        if scan_complete_callback:
            self._worker.scan_completed.connect(scan_complete_callback)
        if loading_complete_callback:
            self._worker.loading_completed.connect(loading_complete_callback)
        if error_callback:
            self._worker.error_occurred.connect(error_callback)
            
        # 连接线程完成信号
        self._thread.finished.connect(self._cleanup_thread)
        
        # 启动线程
        self._thread.started.connect(self._worker.start_loading)
        self._thread.start()
        
    def stop_loading(self) -> None:
        """停止数据加载"""
        if self._worker:
            self._worker.stop_loading()
        self._cleanup_thread()
        
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