"""
项目工作流管理器

ProjectWorkflowManager 是整个 MaojocoConverter GUI 的核心控制中心，
负责统一管理数据处理、工作流推进和状态管理。

主要职责：
1. 统一管理所有核心服务
2. 控制工作流程的各个阶段
3. 管理项目状态和上下文数据
4. 提供事件驱动的信号机制

设计原则：
- 单一职责原则：作为唯一的工作流和数据中心
- 服务聚合模式：整合所有核心服务功能
- 事件驱动架构：通过信号机制实现松耦合
- 状态统一管理：避免状态分散在多个服务中
"""

from typing import Optional, Dict, Any, List
from pathlib import Path
from dataclasses import dataclass, field
from enum import Enum
import time
from concurrent.futures import ThreadPoolExecutor, Future

from PySide6.QtCore import QObject, Signal

from .domain_types import (
    ExportData, ProjectInfo, LoadResult, STLModel, 
    StageConfig,
    Vector3D, Transform4D,
    ProjectContext, Body4DCoordinates, JointGlobalCoordinates,
    KinematicTree, KinematicBody, KinematicJoint, KinematicNode,
    ConvertedData
)
from .service_interfaces import (
    IDataLoadingService, IVisualizationService, 
    IStageManagementService, ITransformService,
    LoadMode, VisualizationConfig, StageExecutionResult, LoadingProgress,
    JointType, ComponentInfo, JointInfo, RelativeTransform
)
from .transform_service import TransformService
from .project_data_service import ProjectDataService
from .stl_model_manager import STLModelManager


class WorkflowState(Enum):
    """工作流状态枚举"""
    IDLE = "idle"                    # 空闲状态
    LOADING = "loading"              # 加载中
    PROCESSING = "processing"        # 处理中
    COMPLETED = "completed"          # 已完成
    ERROR = "error"                  # 错误状态
    PAUSED = "paused"                # 暂停状态


@dataclass
class WorkflowState:
    """工作流状态容器
    
    统一管理整个项目的状态和数据，替代分散在各个服务中的状态。
    包含工作流各阶段所需的全部上下文数据。
    """
    # 项目上下文数据
    context: ProjectContext = field(default_factory=ProjectContext)
    
    # 项目信息
    project_info: Optional[ProjectInfo] = None
    
    # 阶段配置和状态
    stage_configs: Dict[str, StageConfig] = field(default_factory=dict)
    stage_results: Dict[str, StageExecutionResult] = field(default_factory=dict)
    
    # 工作流状态
    current_workflow_state: WorkflowState = WorkflowState.IDLE
    current_stage: Optional[str] = None
    execution_history: List[StageExecutionResult] = field(default_factory=list)
    
    # 时间戳
    start_time: Optional[float] = None
    end_time: Optional[float] = None
    
    def reset(self):
        """重置项目状态"""
        self.context.reset()
        self.project_info = None
        self.stage_configs.clear()
        self.stage_results.clear()
        self.current_workflow_state = WorkflowState.IDLE
        self.current_stage = None
        self.execution_history.clear()
        self.start_time = None
        self.end_time = None
    
    @property
    def project_directory(self) -> Optional[Path]:
        """获取项目目录"""
        return self.context.project_directory
    
    @project_directory.setter
    def project_directory(self, value: Optional[Path]):
        """设置项目目录"""
        self.context.project_directory = value
    
    @property
    def loaded_models(self) -> List[STLModel]:
        """获取已加载的模型"""
        return self.context.loaded_models
    
    @property
    def raw_export_data(self) -> Optional[ExportData]:
        """获取原始导出数据"""
        return self.context.export_data
    
    @raw_export_data.setter
    def raw_export_data(self, value: Optional[ExportData]):
        """设置原始导出数据"""
        self.context.export_data = value
    
    @property
    def body_4d_coordinates(self) -> Dict[str, Body4DCoordinates]:
        """获取刚体4D坐标"""
        return self.context.body_4d_coordinates
    
    @property
    def joint_global_coordinates(self) -> Dict[str, JointGlobalCoordinates]:
        """获取关节全局坐标"""
        return self.context.joint_global_coordinates
    
    @property
    def kinematic_tree(self) -> Optional[KinematicTree]:
        """获取运动学树"""
        return self.context.kinematic_tree
    
    @property
    def converted_data(self) -> Optional[ConvertedData]:
        """获取转换后的数据"""
        return self.context.converted_data
    
    @property
    def xml_content(self) -> Optional[str]:
        """获取XML内容"""
        return self.context.xml_content
    
    def get_processing_summary(self) -> str:
        """获取处理摘要"""
        return self.context.get_summary()


class ProjectWorkflowManager(QObject):
    """项目工作流管理器
    
    整个 MaojocoConverter GUI 的核心控制中心，统一管理所有业务逻辑。
    继承自 QObject 以支持 Qt 信号机制。
    """
    
    # 数据加载信号
    project_loaded = Signal(LoadResult)
    loading_progress = Signal(LoadingProgress)
    loading_started = Signal()
    loading_finished = Signal()
    
    # 阶段执行信号
    stage_started = Signal(str)  # stage_name
    stage_progress = Signal(str, int, str)  # stage_name, progress, message
    stage_completed = Signal(StageExecutionResult)
    stage_failed = Signal(str, str)  # stage_name, error_message
    
    # 工作流信号
    workflow_started = Signal()
    workflow_completed = Signal()
    workflow_paused = Signal()
    workflow_resumed = Signal()
    workflow_failed = Signal(str)
    
    # 状态变更信号
    status_changed = Signal(str)  # status_message
    error_occurred = Signal(str)  # error_message
    warning_occurred = Signal(str)  # warning_message
    
    # 数据变更信号
    data_updated = Signal(str, Any)  # data_type, data
    visualization_updated = Signal()
    
    def __init__(self):
        """初始化工作流管理器"""
        super().__init__()
        
        # 项目状态
        self._state = WorkflowState()
        
        # 核心服务实例
        self._data_loading_service: Optional[IDataLoadingService] = None
        self._visualization_service: Optional[IVisualizationService] = None
        self._stage_management_service: Optional[IStageManagementService] = None
        self._transform_service: Optional[ITransformService] = None
        
        # 线程池用于异步操作
        self._executor = ThreadPoolExecutor(max_workers=2)
        self._current_operation: Optional[Future] = None
        
        # 阶段定义
        self._stage_names = [
            'initialization',
            'data_loading', 
            'relationship_analysis',
            'unit_conversion',
            'model_generation',
            'actuator_generation'
        ]
        
        # 初始化服务
        self._initialize_services()
        self._initialize_stages()
        
    def _initialize_services(self):
        """初始化核心服务"""
        try:
            # 初始化数据加载服务
            self._data_loading_service = ProjectDataService()
            
            # 初始化变换服务
            self._transform_service = TransformService()
            
            # 初始化STL模型管理器
            self._stl_model_manager = STLModelManager()
            
            self.status_changed.emit("核心服务初始化完成")
            
        except Exception as e:
            self.error_occurred.emit(f"服务初始化失败: {str(e)}")
            raise
    
    def _initialize_stages(self):
        """初始化工作流阶段"""
        stage_definitions = {
            'initialization': ('初始化阶段', '项目选择和验证'),
            'data_loading': ('模型预览阶段', 'STL模型加载和显示'),
            'relationship_analysis': ('关系分析阶段', '关节关系分析'),
            'unit_conversion': ('单位转换阶段', '坐标系转换'),
            'model_generation': ('模型生成阶段', 'MuJoCo模型生成'),
            'actuator_generation': ('执行器生成阶段', '执行器配置')
        }
        
        for stage_name, (display_name, description) in stage_definitions.items():
            config = StageConfig(stage_name, display_name)
            config.set_parameter('description', description)
            self._state.stage_configs[stage_name] = config
    
    def get_state(self) -> WorkflowState:
        """获取当前项目状态"""
        return self._state
    
    def get_context(self) -> ProjectContext:
        """获取项目上下文数据"""
        return self._state.context
    
    def get_workflow_state(self) -> WorkflowState:
        """获取工作流状态"""
        return self._state.current_workflow_state
    
    def is_busy(self) -> bool:
        """检查是否正在执行操作"""
        return (self._state.current_workflow_state in [WorkflowState.LOADING, WorkflowState.PROCESSING] 
                and self._current_operation is not None 
                and not self._current_operation.done())
    
    def set_visualization_service(self, service: IVisualizationService):
        """设置可视化服务"""
        self._visualization_service = service
        self.status_changed.emit("可视化服务已设置")
    
    def set_data_loading_service(self, service: IDataLoadingService):
        """设置数据加载服务"""
        self._data_loading_service = service
        self.status_changed.emit("数据加载服务已设置")
    
    def set_stage_management_service(self, service: IStageManagementService):
        """设置阶段管理服务"""
        self._stage_management_service = service
        self.status_changed.emit("阶段管理服务已设置")
    
    def set_transform_service(self, service: ITransformService):
        """设置变换服务"""
        self._transform_service = service
        self.status_changed.emit("变换服务已设置")
    
    def set_stage_config(self, stage_name: str, config: Dict[str, Any]):
        """设置阶段配置"""
        if stage_name in self._state.stage_configs:
            stage_config = self._state.stage_configs[stage_name]
            for key, value in config.items():
                stage_config.set_parameter(key, value)
            self.status_changed.emit(f"阶段 {stage_name} 配置已更新")
        else:
            self.error_occurred.emit(f"未知阶段: {stage_name}")
    
    def load_project(self, directory_path: Path, mode: LoadMode = LoadMode.FULL_LOAD) -> LoadResult:
        """加载项目
        
        Args:
            directory_path: 项目目录路径
            mode: 加载模式
            
        Returns:
            LoadResult: 加载结果
        """
        if self.is_busy():
            return LoadResult(
                success=False,
                models=[],
                message="系统正在执行其他操作，请稍后再试"
            )
        
        # 更新状态
        self._state.reset()
        self._state.project_directory = directory_path
        self._state.current_workflow_state = WorkflowState.LOADING
        self._state.start_time = time.time()
        
        self.loading_started.emit()
        self.status_changed.emit(f"开始加载项目: {directory_path}")
        
        try:
            # 验证项目结构
            if self._data_loading_service:
                validation_errors = self._data_loading_service.validate_project_structure(directory_path)
                if validation_errors:
                    error_msg = f"项目结构验证失败: {'; '.join(validation_errors)}"
                    self.error_occurred.emit(error_msg)
                    return LoadResult(
                        success=False,
                        models=[],
                        message=error_msg
                    )
            
            # 加载项目数据
            if self._data_loading_service:
                result = self._data_loading_service.load_project_directory(directory_path, mode)
                
                if result.success:
                    # 更新状态
                    self._state.raw_export_data = result.project_info
                    self._state.loaded_models = result.models
                    self._state.project_info = result.project_info
                    
                    # 更新上下文数据
                    self._state.context.export_data = result.project_info
                    self._state.context.loaded_models = result.models
                    
                    # 发送信号
                    self.project_loaded.emit(result)
                    self.status_changed.emit("项目加载成功")
                    
                    # 更新可视化
                    self._update_visualization_with_loaded_models()
                    
                else:
                    self._state.current_workflow_state = WorkflowState.ERROR
                    self.error_occurred.emit(f"项目加载失败: {result.message}")
                
                return result
            
        except Exception as e:
            error_msg = f"加载项目时发生异常: {str(e)}"
            self._state.current_workflow_state = WorkflowState.ERROR
            self.error_occurred.emit(error_msg)
            return LoadResult(
                success=False,
                models=[],
                message=error_msg
            )
        
        finally:
            self._state.end_time = time.time()
            self.loading_finished.emit()
            self._state.current_workflow_state = WorkflowState.IDLE
    
    def _update_visualization_with_loaded_models(self):
        """使用加载的模型更新可视化"""
        if not self._visualization_service or not self._state.loaded_models:
            return
        
        try:
            # 清除现有模型
            self._visualization_service.clear_all_models()
            
            # 添加新模型
            for model in self._state.loaded_models:
                transform = model.transform_matrix if model.is_transformed else None
                self._visualization_service.add_stl_model(model, model.name, transform)
            
            # 调整视角
            self._visualization_service.fit_view_to_models()
            
            # 发送信号
            self.visualization_updated.emit()
            
        except Exception as e:
            self.error_occurred.emit(f"更新可视化失败: {str(e)}")
    
    def execute_stage(self, stage_name: str) -> StageExecutionResult:
        """执行指定阶段
        
        Args:
            stage_name: 阶段名称
            
        Returns:
            StageExecutionResult: 执行结果
        """
        if stage_name not in self._stage_names:
            return StageExecutionResult(
                stage_name=stage_name,
                success=False,
                execution_time=0.0,
                error_message=f"未知的阶段: {stage_name}"
            )
        
        if self.is_busy():
            return StageExecutionResult(
                stage_name=stage_name,
                success=False,
                execution_time=0.0,
                error_message="系统正在执行其他操作"
            )
        
        # 更新状态
        self._state.current_stage = stage_name
        self._state.current_workflow_state = WorkflowState.PROCESSING
        
        self.stage_started.emit(stage_name)
        self.status_changed.emit(f"开始执行阶段: {stage_name}")
        
        start_time = time.time()
        
        try:
            # 验证前置条件
            dependency_errors = self._validate_stage_dependencies(stage_name)
            if dependency_errors:
                error_msg = f"阶段依赖验证失败: {'; '.join(dependency_errors)}"
                self.stage_failed.emit(stage_name, error_msg)
                return StageExecutionResult(
                    stage_name=stage_name,
                    success=False,
                    execution_time=0.0,
                    error_message=error_msg
                )
            
            # 执行阶段逻辑
            result = self._execute_stage_logic(stage_name)
            
            # 保存结果
            self._state.stage_results[stage_name] = result
            self._state.execution_history.append(result)
            
            # 更新阶段配置
            if stage_name in self._state.stage_configs:
                self._state.stage_configs[stage_name].is_completed = result.success
            
            # 发送信号
            self.stage_completed.emit(result)
            
            if result.success:
                self.status_changed.emit(f"阶段 {stage_name} 执行成功")
            else:
                self.stage_failed.emit(stage_name, result.error_message or "未知错误")
            
            return result
            
        except Exception as e:
            error_msg = f"执行阶段 {stage_name} 时发生异常: {str(e)}"
            result = StageExecutionResult(
                stage_name=stage_name,
                success=False,
                execution_time=time.time() - start_time,
                error_message=error_msg
            )
            
            self.stage_failed.emit(stage_name, error_msg)
            self.error_occurred.emit(error_msg)
            
            return result
        
        finally:
            self._state.current_stage = None
            self._state.current_workflow_state = WorkflowState.IDLE
    
    def _validate_stage_dependencies(self, stage_name: str) -> List[str]:
        """验证阶段依赖关系"""
        errors = []
        
        # 检查前置阶段是否完成
        stage_index = self._stage_names.index(stage_name)
        
        for i in range(stage_index):
            prev_stage = self._stage_names[i]
            if (prev_stage not in self._state.stage_results or 
                not self._state.stage_results[prev_stage].success):
                errors.append(f"前置阶段 {prev_stage} 未完成")
        
        # 检查基础数据
        if stage_name != 'initialization' and not self._state.raw_export_data:
            errors.append("缺少原始项目数据")
        
        return errors
    
    def _execute_stage_logic(self, stage_name: str) -> StageExecutionResult:
        """执行阶段的具体逻辑"""
        start_time = time.time()
        
        try:
            if stage_name == 'initialization':
                return self._execute_initialization_stage()
            elif stage_name == 'data_loading':
                return self._execute_data_loading_stage()
            elif stage_name == 'relationship_analysis':
                return self._execute_relationship_analysis_stage()
            elif stage_name == 'unit_conversion':
                return self._execute_unit_conversion_stage()
            elif stage_name == 'model_generation':
                return self._execute_model_generation_stage()
            elif stage_name == 'actuator_generation':
                return self._execute_actuator_generation_stage()
            else:
                return StageExecutionResult(
                    stage_name=stage_name,
                    success=False,
                    execution_time=0.0,
                    error_message=f"未实现阶段: {stage_name}"
                )
                
        except Exception as e:
            return StageExecutionResult(
                stage_name=stage_name,
                success=False,
                execution_time=time.time() - start_time,
                error_message=str(e)
            )
    
    def _execute_initialization_stage(self) -> StageExecutionResult:
        """执行初始化阶段"""
        start_time = time.time()
        
        # 验证项目目录
        if not self._state.project_directory or not self._state.project_directory.exists():
            error_msg = "项目目录不存在"
            self._state.context.add_validation_error(error_msg)
            return StageExecutionResult(
                stage_name='initialization',
                success=False,
                execution_time=time.time() - start_time,
                error_message=error_msg
            )
        
        # 验证项目结构
        if self._data_loading_service:
            errors = self._data_loading_service.validate_project_structure(self._state.project_directory)
            if errors:
                error_msg = f"项目结构验证失败: {'; '.join(errors)}"
                for error in errors:
                    self._state.context.add_validation_error(error)
                return StageExecutionResult(
                    stage_name='initialization',
                    success=False,
                    execution_time=time.time() - start_time,
                    error_message=error_msg
                )
        
        return StageExecutionResult(
            stage_name='initialization',
            success=True,
            execution_time=time.time() - start_time
        )
    
    def _execute_data_loading_stage(self) -> StageExecutionResult:
        """执行数据加载阶段"""
        start_time = time.time()
        
        # 如果还没有加载数据，则加载
        if not self._state.loaded_models and self._state.project_directory:
            result = self.load_project(self._state.project_directory)
            if not result.success:
                error_msg = result.message or "数据加载失败"
                self._state.context.add_validation_error(error_msg)
                return StageExecutionResult(
                    stage_name='data_loading',
                    success=False,
                    execution_time=time.time() - start_time,
                    error_message=error_msg
                )
        
        # 验证加载的数据
        if not self._state.context.export_data:
            error_msg = "缺少导出数据"
            self._state.context.add_validation_error(error_msg)
            return StageExecutionResult(
                stage_name='data_loading',
                success=False,
                execution_time=time.time() - start_time,
                error_message=error_msg
            )
        
        return StageExecutionResult(
            stage_name='data_loading',
            success=True,
            execution_time=time.time() - start_time
        )
    
    def _execute_relationship_analysis_stage(self) -> StageExecutionResult:
        """执行关系分析阶段"""
        start_time = time.time()
        
        # 检查是否有关节数据
        if not self._state.context.export_data or not self._state.context.export_data.joints:
            error_msg = "缺少关节数据"
            self._state.context.add_validation_error(error_msg)
            return StageExecutionResult(
                stage_name='relationship_analysis',
                success=False,
                execution_time=time.time() - start_time,
                error_message=error_msg
            )
        
        # 处理刚体坐标
        export_data = self._state.context.export_data
        for component in export_data.components:
            if component.world_transform:
                body_coord = Body4DCoordinates(
                    name=component.name,
                    occurrence_name=component.occurrence_name,
                    full_path_name=component.full_path_name,
                    component_id=str(component.component_id),
                    transform=component.world_transform,
                    stl_file=component.stl_file,
                    bodies_count=component.bodies_count,
                    has_children=component.has_children
                )
                self._state.context.body_4d_coordinates[component.name] = body_coord
        
        # 处理关节坐标
        for joint in export_data.joints:
            if joint.geometry.geometry_one_transform:
                joint_coord = JointGlobalCoordinates(
                    position=joint.geometry.geometry_one_transform.get_translation(),
                    quaternion=joint.geometry.geometry_one_transform.to_quaternion(),
                    joint_name=joint.name,
                    joint_type=joint.joint_type
                )
                self._state.context.joint_global_coordinates[joint.name] = joint_coord
        
        # 构建运动学树（简化版本）
        self._build_kinematic_tree()
        
        return StageExecutionResult(
            stage_name='relationship_analysis',
            success=True,
            execution_time=time.time() - start_time
        )
    
    def _execute_unit_conversion_stage(self) -> StageExecutionResult:
        """执行单位转换阶段"""
        start_time = time.time()
        
        # 检查变换服务
        if not self._transform_service:
            return StageExecutionResult(
                stage_name='unit_conversion',
                success=False,
                execution_time=time.time() - start_time,
                error_message="变换服务未初始化"
            )
        
        # TODO: 实现单位转换逻辑
        # 这里可以添加毫米到米的转换、坐标系变换等
        
        return StageExecutionResult(
            stage_name='unit_conversion',
            success=True,
            execution_time=time.time() - start_time
        )
    
    def _execute_model_generation_stage(self) -> StageExecutionResult:
        """执行模型生成阶段"""
        start_time = time.time()
        
        # TODO: 实现MuJoCo模型生成逻辑
        # 这里可以添加XML生成、模型配置等
        
        return StageExecutionResult(
            stage_name='model_generation',
            success=True,
            execution_time=time.time() - start_time
        )
    
    def _execute_actuator_generation_stage(self) -> StageExecutionResult:
        """执行执行器生成阶段"""
        start_time = time.time()
        
        # TODO: 实现执行器生成逻辑
        # 这里可以添加执行器配置、控制参数设置等
        
        return StageExecutionResult(
            stage_name='actuator_generation',
            success=True,
            execution_time=time.time() - start_time
        )
    
    def execute_workflow(self) -> List[StageExecutionResult]:
        """执行完整工作流程
        
        Returns:
            List[StageExecutionResult]: 所有阶段的执行结果
        """
        if self.is_busy():
            self.error_occurred.emit("系统正在执行其他操作")
            return []
        
        self.workflow_started.emit()
        self.status_changed.emit("开始执行完整工作流程")
        
        results = []
        
        try:
            # 按顺序执行所有阶段
            for stage_name in self._stage_names:
                result = self.execute_stage(stage_name)
                results.append(result)
                
                # 如果某个阶段失败，停止执行
                if not result.success:
                    self.workflow_failed.emit(f"阶段 {stage_name} 执行失败")
                    break
            
            # 检查是否全部成功
            if all(result.success for result in results):
                self.workflow_completed.emit()
                self.status_changed.emit("工作流程执行完成")
            
            return results
            
        except Exception as e:
            self.workflow_failed.emit(f"工作流程执行异常: {str(e)}")
            self.error_occurred.emit(f"工作流程执行异常: {str(e)}")
            return results
    
    def reset_workflow(self):
        """重置工作流程"""
        self._state.reset()
        self.status_changed.emit("工作流程已重置")
        
        # 清除可视化
        if self._visualization_service:
            self._visualization_service.clear_all_models()
    
    def get_stage_names(self) -> List[str]:
        """获取所有阶段名称"""
        return self._stage_names.copy()
    
    def get_stage_status(self, stage_name: str) -> str:
        """获取阶段状态"""
        if stage_name not in self._state.stage_results:
            return 'not_started'
        
        result = self._state.stage_results[stage_name]
        return 'completed' if result.success else 'failed'
    
    def get_execution_history(self) -> List[StageExecutionResult]:
        """获取执行历史"""
        return self._state.execution_history.copy()
    
    def _build_kinematic_tree(self):
        """构建运动学树"""
        if not self._state.context.export_data:
            return
        
        export_data = self._state.context.export_data
        
        # 创建运动学树
        kinematic_tree = KinematicTree(
            roots=[],
            nodes={},
            joints={},
            bodies={},
            relative_transforms={}
        )
        
        # 创建刚体
        for component in export_data.components:
            body = KinematicBody(
                body_id=str(component.component_id),
                name=component.name,
                component_id=str(component.component_id),
                occurrence_name=component.occurrence_name,
                world_transform=component.world_transform or Transform4D.identity(),
                stl_file=component.stl_file,
                bodies_count=component.bodies_count
            )
            kinematic_tree.bodies[body.body_id] = body
        
        # 创建关节
        for joint in export_data.joints:
            if joint.connection.occurrence_one_component and joint.connection.occurrence_two_component:
                kinematic_joint = KinematicJoint(
                    joint_id=joint.name,
                    name=joint.name,
                    joint_type=joint.joint_type,
                    parent_body=joint.connection.occurrence_one_component,
                    child_body=joint.connection.occurrence_two_component,
                    position=joint.geometry.geometry_one_transform.get_translation() if joint.geometry.geometry_one_transform else Vector3D.zero(),
                    axis=None,
                    limits=None,
                    is_suppressed=joint.is_suppressed,
                    is_active=joint.is_light_bulb_on
                )
                kinematic_tree.joints[joint.name] = kinematic_joint
        
        # 构建节点关系（简化版本，假设第一个组件为根）
        if export_data.components:
            root_component = export_data.components[0]
            kinematic_tree.roots.append(str(root_component.component_id))
            
            # 为每个组件创建节点
            for i, component in enumerate(export_data.components):
                node = KinematicNode(
                    body_id=str(component.component_id),
                    parent_body=str(export_data.components[0].component_id) if i > 0 else None,
                    children=[],
                    joint=None,
                    level=0 if i == 0 else 1
                )
                kinematic_tree.nodes[str(component.component_id)] = node
        
        self._state.context.kinematic_tree = kinematic_tree
    
    def cleanup(self):
        """清理资源"""
        if self._current_operation and not self._current_operation.done():
            self._current_operation.cancel()
        
        self._executor.shutdown(wait=True)
        self.status_changed.emit("工作流管理器已清理")