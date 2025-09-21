"""
核心服务接口定义

定义MaojocoConverter GUI核心业务逻辑的抽象接口，实现依赖倒置原则。
基于现有功能分析，定义四个核心服务接口：

1. IDataLoadingService - 负责项目数据加载和预处理
2. IVisualizationService - 负责3D模型可视化和交互
3. IStageManagementService - 负责工作流程阶段管理
4. ITransformService - 负责坐标变换和单位转换

设计原则：
- 使用抽象基类定义接口
- 支持异步操作
- 完整类型注解
- 清晰的错误处理
"""

from abc import ABC, abstractmethod
from typing import List, Optional, Dict, Any, Union, Callable
from pathlib import Path
from enum import Enum
from dataclasses import dataclass

from .domain_types import (
    Vector3D, Transform4D,
    ExportData, ProjectInfo, StageConfig, LoadResult, MeshQuality,
    CameraPosition, TransformStatistics, STLModel,
    JointType, ComponentInfo, JointInfo
)


class LoadMode(Enum):
    """加载模式枚举"""
    FULL_LOAD = "full_load"          # 完整加载所有数据
    QUICK_LOAD = "quick_load"        # 快速加载（仅基本信息）
    METADATA_ONLY = "metadata_only"  # 仅加载元数据


@dataclass
class LoadingProgress:
    """加载进度信息"""
    current_step: int                # 当前步骤
    total_steps: int                 # 总步骤数
    step_name: str                   # 当前步骤名称
    progress_percentage: float       # 进度百分比 (0-100)
    message: str                     # 进度消息
    data_loaded: Optional[ExportData] = None # 已加载数据


@dataclass
class VisualizationConfig:
    """可视化配置"""
    background_color: str = "#2E2E2E"        # 背景颜色
    edge_color: str = "#FF0000"             # 边缘颜色
    component_color: str = "#00FF00"         # 组件颜色
    joint_color: str = "#0000FF"            # 关节颜色
    opacity: float = 1.0                    # 不透明度
    show_edges: bool = True                 # 显示边缘
    show_axes: bool = True                  # 显示坐标轴
    show_grid: bool = True                   # 显示网格
    camera_position: Optional[Vector3D] = None  # 相机位置
    lighting_intensity: float = 1.0         # 光照强度


@dataclass
class StageExecutionResult:
    """阶段执行结果"""
    stage_name: str                       # 阶段名称
    success: bool                         # 执行是否成功
    execution_time: float                # 执行时间（秒）
    output_data: Optional[ExportData] = None    # 输出数据
    error_message: Optional[str] = None  # 错误信息
    warnings: List[str] = None           # 警告信息列表
    data_processed: Optional[Dict[str, Any]] = None  # 处理的数据统计

    def __post_init__(self):
        if self.warnings is None:
            self.warnings = []


class IDataLoadingService(ABC):
    """数据加载服务接口
    
    负责从项目目录加载各种数据文件，包括：
    - 零部件信息（STL文件、组件数据）
    - 关节信息（运动副数据）
    - 元数据（项目信息、配置文件）
    
    支持同步和异步加载，并提供进度回调。
    """
    
    @abstractmethod
    def load_project_directory(
        self, 
        directory_path: Path, 
        mode: LoadMode = LoadMode.FULL_LOAD,
        progress_callback: Optional[Callable[[LoadingProgress], None]] = None
    ) -> LoadResult:
        """加载项目目录
        
        Args:
            directory_path: 项目目录路径
            mode: 加载模式
            progress_callback: 进度回调函数
            
        Returns:
            LoadResult: 加载结果
            
        Raises:
            FileNotFoundError: 目录不存在
            ValueError: 数据格式错误
        """
        pass
    
    @abstractmethod
    def load_stl_files(self, stl_paths: List[Path]) -> List[STLModel]:
        """加载STL文件
        
        Args:
            stl_paths: STL文件路径列表
            
        Returns:
            List[STLModel]: STL模型对象列表
            
        Raises:
            STLLoadingError: STL加载失败
        """
        pass
    
    @abstractmethod
    def load_component_data(self, project_dir: Path) -> List[ComponentInfo]:
        """加载零部件数据
        
        Args:
            project_dir: 项目目录路径
            
        Returns:
            List[ComponentInfo]: 零部件信息列表
        """
        pass
    
    @abstractmethod
    def load_joint_data(self, project_dir: Path) -> List[JointInfo]:
        """加载关节数据
        
        Args:
            project_dir: 项目目录路径
            
        Returns:
            List[JointInfo]: 关节信息列表
        """
        pass
    
    @abstractmethod
    def load_project_metadata(self, project_dir: Path) -> ProjectInfo:
        """加载项目元数据
        
        Args:
            project_dir: 项目目录路径
            
        Returns:
            ProjectInfo: 项目信息
        """
        pass
    
    @abstractmethod
    def validate_project_structure(self, project_dir: Path) -> List[str]:
        """验证项目结构
        
        Args:
            project_dir: 项目目录路径
            
        Returns:
            List[str]: 验证错误列表（空列表表示无错误）
        """
        pass
    
    @abstractmethod
    def get_supported_file_extensions(self) -> List[str]:
        """获取支持的文件扩展名
        
        Returns:
            List[str]: 支持的文件扩展名列表
        """
        pass


class IVisualizationService(ABC):
    """可视化服务接口
    
    负责3D模型的可视化和用户交互，包括：
    - STL模型渲染和显示
    - 视角控制和相机管理
    - 拾取和选择功能
    - 可视化配置管理
    
    基于PyVista实现，提供丰富的3D交互功能。
    """
    
    @abstractmethod
    def initialize_visualization(self, parent_widget: Any) -> bool:
        """初始化可视化服务
        
        Args:
            parent_widget: 父控件
            
        Returns:
            bool: 初始化是否成功
        """
        pass
    
    @abstractmethod
    def add_stl_model(self, model_data: STLModel, name: str, transform: Optional[Transform4D] = None) -> bool:
        """添加STL模型
        
        Args:
            model_data: STL模型数据
            name: 模型名称
            transform: 变换矩阵
            
        Returns:
            bool: 添加是否成功
        """
        pass
    
    @abstractmethod
    def remove_model(self, name: str) -> bool:
        """移除模型
        
        Args:
            name: 模型名称
            
        Returns:
            bool: 移除是否成功
        """
        pass
    
    @abstractmethod
    def clear_all_models(self) -> None:
        """清除所有模型"""
        pass
    
    @abstractmethod
    def update_model_transform(self, name: str, transform: Transform4D) -> bool:
        """更新模型变换
        
        Args:
            name: 模型名称
            transform: 新的变换矩阵
            
        Returns:
            bool: 更新是否成功
        """
        pass
    
    @abstractmethod
    def set_model_visibility(self, name: str, visible: bool) -> None:
        """设置模型可见性
        
        Args:
            name: 模型名称
            visible: 是否可见
        """
        pass
    
    @abstractmethod
    def set_model_color(self, name: str, color: Union[str, List[float]]) -> None:
        """设置模型颜色
        
        Args:
            name: 模型名称
            color: 颜色（字符串或RGB值）
        """
        pass
    
    @abstractmethod
    def set_model_opacity(self, name: str, opacity: float) -> None:
        """设置模型不透明度
        
        Args:
            name: 模型名称
            opacity: 不透明度 (0-1)
        """
        pass
    
    @abstractmethod
    def fit_view_to_models(self) -> None:
        """调整视角以适应所有模型"""
        pass
    
    @abstractmethod
    def set_camera_position(self, position: Vector3D, focal_point: Vector3D, view_up: Vector3D) -> None:
        """设置相机位置
        
        Args:
            position: 相机位置
            focal_point: 焦点位置
            view_up: 向上向量
        """
        pass
    
    @abstractmethod
    def get_camera_position(self) -> CameraPosition:
        """获取相机位置
        
        Returns:
            CameraPosition: 相机位置信息
        """
        pass
    
    @abstractmethod
    def pick_model_at_position(self, x: int, y: int) -> Optional[str]:
        """在指定位置拾取模型
        
        Args:
            x: 屏幕坐标X
            y: 屏幕坐标Y
            
        Returns:
            Optional[str]: 拾取到的模型名称，无则为None
        """
        pass
    
    @abstractmethod
    def update_visualization_config(self, config: VisualizationConfig) -> None:
        """更新可视化配置
        
        Args:
            config: 新的可视化配置
        """
        pass
    
    @abstractmethod
    def take_screenshot(self, filepath: Path) -> bool:
        """截取屏幕截图
        
        Args:
            filepath: 保存路径
            
        Returns:
            bool: 截图是否成功
        """
        pass
    
    @abstractmethod
    def get_model_names(self) -> List[str]:
        """获取所有模型名称
        
        Returns:
            List[str]: 模型名称列表
        """
        pass
    
    @abstractmethod
    def is_model_loaded(self, name: str) -> bool:
        """检查模型是否已加载
        
        Args:
            name: 模型名称
            
        Returns:
            bool: 是否已加载
        """
        pass


class IStageManagementService(ABC):
    """阶段管理服务接口
    
    负责管理工作流程的各个阶段，包括：
    - 阶段定义和配置管理
    - 阶段执行控制
    - 阶段状态管理
    - 执行结果收集
    
    实现工作流程的顺序控制和错误处理。
    """
    
    @abstractmethod
    def initialize_stages(self) -> bool:
        """初始化所有阶段
        
        Returns:
            bool: 初始化是否成功
        """
        pass
    
    @abstractmethod
    def get_stage_names(self) -> List[str]:
        """获取所有阶段名称
        
        Returns:
            List[str]: 阶段名称列表
        """
        pass
    
    @abstractmethod
    def get_stage_config(self, stage_name: str) -> Optional[StageConfig]:
        """获取阶段配置
        
        Args:
            stage_name: 阶段名称
            
        Returns:
            Optional[StageConfig]: 阶段配置，不存在则返回None
        """
        pass
    
    @abstractmethod
    def set_stage_config(self, stage_name: str, config: StageConfig) -> bool:
        """设置阶段配置
        
        Args:
            stage_name: 阶段名称
            config: 阶段配置
            
        Returns:
            bool: 设置是否成功
        """
        pass
    
    @abstractmethod
    def execute_stage(
        self, 
        stage_name: str, 
        input_data: Optional[ExportData] = None,
        progress_callback: Optional[Callable[[LoadingProgress], None]] = None
    ) -> StageExecutionResult:
        """执行指定阶段
        
        Args:
            stage_name: 阶段名称
            input_data: 输入数据
            progress_callback: 进度回调函数
            
        Returns:
            StageExecutionResult: 执行结果
        """
        pass
    
    @abstractmethod
    def execute_all_stages(
        self, 
        initial_data: Optional[ExportData] = None,
        progress_callback: Optional[Callable[[LoadingProgress], None]] = None
    ) -> List[StageExecutionResult]:
        """执行所有阶段
        
        Args:
            initial_data: 初始数据
            progress_callback: 进度回调函数
            
        Returns:
            List[StageExecutionResult]: 所有阶段的执行结果
        """
        pass
    
    @abstractmethod
    def get_stage_status(self, stage_name: str) -> str:
        """获取阶段状态
        
        Args:
            stage_name: 阶段名称
            
        Returns:
            str: 阶段状态 ('not_started', 'in_progress', 'completed', 'failed')
        """
        pass
    
    @abstractmethod
    def reset_all_stages(self) -> bool:
        """重置所有阶段状态
        
        Returns:
            bool: 重置是否成功
        """
        pass
    
    @abstractmethod
    def validate_stage_dependencies(self, stage_name: str) -> List[str]:
        """验证阶段依赖关系
        
        Args:
            stage_name: 阶段名称
            
        Returns:
            List[str]: 依赖错误列表（空列表表示无错误）
        """
        pass
    
    @abstractmethod
    def get_execution_history(self) -> List[StageExecutionResult]:
        """获取执行历史
        
        Returns:
            List[StageExecutionResult]: 历史执行结果
        """
        pass


class ITransformService(ABC):
    """坐标变换服务接口
    
    负责坐标系统的变换和单位转换，包括：
    - 坐标系变换（毫米/米）
    - 4x4变换矩阵操作
    - 单位转换（厘米/毫米）
    - STL坐标变换
    
    保持与现有transform_service.py的兼容性。
    """
    
    @abstractmethod
    def transform_stl_coordinates(self, input_path: Path, output_path: Path) -> bool:
        """变换STL文件坐标
        
        将STL文件从厘米坐标系转换为毫米坐标系，保持与现有功能一致。
        
        Args:
            input_path: 输入STL文件路径
            output_path: 输出STL文件路径
            
        Returns:
            bool: 变换是否成功
            
        Raises:
            FileNotFoundError: 输入文件不存在
            ValueError: 坐标变换错误
        """
        pass
    
    @abstractmethod
    def millimeters_to_meters(self, value: Union[float, Vector3D]) -> Union[float, Vector3D]:
        """毫米转米
        
        Args:
            value: 输入值（数字或向量）
            
        Returns:
            Union[float, Vector3D]: 转换后的值
        """
        pass
    
    @abstractmethod
    def meters_to_millimeters(self, value: Union[float, Vector3D]) -> Union[float, Vector3D]:
        """米转毫米
        
        Args:
            value: 输入值（数字或向量）
            
        Returns:
            Union[float, Vector3D]: 转换后的值
        """
        pass
    
    @abstractmethod
    def create_transform_matrix(
        self, 
        translation: Optional[Vector3D] = None,
        rotation: Optional[List[List[float]]] = None,
        scale: Optional[Vector3D] = None
    ) -> Transform4D:
        """创建变换矩阵
        
        Args:
            translation: 平移向量
            rotation: 3x3旋转矩阵
            scale: 缩放向量
            
        Returns:
            Transform4D: 4x4变换矩阵
        """
        pass
    
    @abstractmethod
    def multiply_transforms(self, transform1: Transform4D, transform2: Transform4D) -> Transform4D:
        """矩阵乘法
        
        Args:
            transform1: 第一个变换矩阵
            transform2: 第二个变换矩阵
            
        Returns:
            Transform4D: 乘积矩阵
        """
        pass
    
    @abstractmethod
    def invert_transform(self, transform: Transform4D) -> Transform4D:
        """矩阵求逆
        
        Args:
            transform: 变换矩阵
            
        Returns:
            Transform4D: 逆矩阵
            
        Raises:
            ValueError: 矩阵不可逆
        """
        pass
    
    @abstractmethod
    def transform_point(self, point: Vector3D, transform: Transform4D) -> Vector3D:
        """变换点
        
        Args:
            point: 输入点
            transform: 变换矩阵
            
        Returns:
            Vector3D: 变换后的点
        """
        pass
    
    @abstractmethod
    def extract_translation(self, transform: Transform4D) -> Vector3D:
        """提取平移分量
        
        Args:
            transform: 变换矩阵
            
        Returns:
            Vector3D: 平移向量
        """
        pass
    
    @abstractmethod
    def extract_rotation(self, transform: Transform4D) -> List[List[float]]:
        """提取旋转分量
        
        Args:
            transform: 变换矩阵
            
        Returns:
            List[List[float]]: 3x3旋转矩阵
        """
        pass
    
    @abstractmethod
    def get_transform_units_info(self) -> str:
        """获取变换单位信息
        
        Returns:
            str: 单位信息描述
        """
        pass


class ServiceInitializationError(Exception):
    """服务初始化异常"""
    pass


class ServiceExecutionError(Exception):
    """服务执行异常"""
    pass


class ServiceValidationError(Exception):
    """服务验证异常"""
    pass