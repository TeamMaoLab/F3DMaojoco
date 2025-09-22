"""
UI工作流适配器

作为UI模块与ProjectWorkflowManager之间的桥梁，提供：
1. 信号转换和适配
2. UI事件到工作流命令的转换
3. 工作流状态到UI更新的转换
4. 异步操作的协调和错误处理
"""

# 标准库
from typing import Optional, Dict, Any, List, Callable
from pathlib import Path
from dataclasses import dataclass

# 第三方库
from PySide6.QtCore import QObject, Signal, Slot

# 本地模块
from core.project_workflow_manager import ProjectWorkflowManager, WorkflowState
from core.domain_types import (
    ExportData, ComponentInfo, JointInfo, LoadResult,
    AssemblyTreeInfo, JointPairwiseRelationship,
    ProjectInfo
)
from core.service_interfaces import StageExecutionResult
from utils.logger import logger


@dataclass
class UICommand:
    """UI命令数据结构"""
    command_type: str
    parameters: Dict[str, Any]
    callback: Optional[Callable] = None


class UIWorkflowAdapter(QObject):
    """UI工作流适配器 - 连接UI和ProjectWorkflowManager的桥梁"""
    
    # 信号定义
    # 工作流状态信号
    workflow_state_changed = Signal(WorkflowState)           # 工作流状态变化
    workflow_status_changed = Signal(str)                  # 工作流状态文本变化
    stage_execution_started = Signal(str)                   # 阶段执行开始
    stage_execution_progress = Signal(str, int, str)        # 阶段执行进度
    stage_execution_completed = Signal(str, bool)           # 阶段执行完成
    stage_execution_failed = Signal(str, str)                # 阶段执行失败
    
    # 项目数据信号
    project_loaded = Signal(LoadResult)                     # 项目加载完成
    project_data_updated = Signal(ExportData)               # 项目数据更新
    components_updated = Signal(list)                       # 组件列表更新
    joints_updated = Signal(list)                           # 关节列表更新
    
    # 关系分析信号
    relationship_analysis_completed = Signal()             # 关系分析完成
    cycles_detected = Signal(list)                          # 环检测结果
    assembly_trees_generated = Signal(list)                 # 装配树生成
    joint_relationships_updated = Signal(dict)              # 关节关系更新
    
    # 错误和状态信号
    error_occurred = Signal(str)                            # 错误发生
    warning_occurred = Signal(str)                          # 警告发生
    info_message = Signal(str)                              # 信息消息
    
    # 进度信号
    operation_started = Signal(str)                         # 操作开始
    operation_progress = Signal(int, str)                   # 操作进度
    operation_completed = Signal(str, bool)                  # 操作完成
    
    def __init__(self, workflow_manager: ProjectWorkflowManager):
        """初始化UI工作流适配器
        
        Args:
            workflow_manager: ProjectWorkflowManager实例
        """
        super().__init__()
        self._workflow_manager = workflow_manager
        self._current_state: Optional[WorkflowState] = None
        
        # 连接工作流管理器的信号
        self._connect_workflow_signals()
        
        logger.info("UI工作流适配器初始化完成")
    
    def _connect_workflow_signals(self):
        """连接工作流管理器的信号"""
        # 工作流状态信号
        self._workflow_manager.state_changed.connect(self._on_workflow_state_changed)
        self._workflow_manager.stage_execution_started.connect(self._on_stage_execution_started)
        self._workflow_manager.stage_execution_progress.connect(self._on_stage_execution_progress)
        self._workflow_manager.stage_execution_completed.connect(self._on_stage_execution_completed)
        self._workflow_manager.stage_execution_failed.connect(self._on_stage_execution_failed)
        
        # 项目数据信号
        self._workflow_manager.project_loaded.connect(self._on_project_loaded)
        self._workflow_manager.project_data_updated.connect(self._on_project_data_updated)
        
        # 关系分析信号
        self._workflow_manager.relationship_analysis_completed.connect(self._on_relationship_analysis_completed)
        
        # 错误和状态信号
        self._workflow_manager.error_occurred.connect(self._on_error_occurred)
        self._workflow_manager.warning_occurred.connect(self._on_warning_occurred)
        self._workflow_manager.info_message.connect(self._on_info_message)
    
    # ===== 项目管理方法 =====
    
    @Slot(str)
    def load_project(self, project_path: str):
        """加载项目
        
        Args:
            project_path: 项目路径
        """
        try:
            self.operation_started.emit("加载项目")
            logger.info(f"UI请求加载项目: {project_path}")
            
            project_dir = Path(project_path)
            result = self._workflow_manager.load_project(project_dir)
            
            if result.success:
                self.project_loaded.emit(result)
                self.operation_completed.emit("加载项目", True)
                self.info_message.emit(f"项目加载成功: {len(result.export_data.components)} 个组件")
            else:
                self.error_occurred.emit(f"项目加载失败: {result.message}")
                self.operation_completed.emit("加载项目", False)
                
        except Exception as e:
            error_msg = f"加载项目异常: {str(e)}"
            logger.error(error_msg)
            self.error_occurred.emit(error_msg)
            self.operation_completed.emit("加载项目", False)
    
    @Slot()
    def unload_project(self):
        """卸载当前项目"""
        try:
            logger.info("UI请求卸载项目")
            self._workflow_manager.unload_project()
            self.info_message.emit("项目已卸载")
            
        except Exception as e:
            error_msg = f"卸载项目异常: {str(e)}"
            logger.error(error_msg)
            self.error_occurred.emit(error_msg)
    
    # ===== 阶段执行方法 =====
    
    @Slot(str)
    def execute_stage(self, stage_name: str):
        """执行指定阶段
        
        Args:
            stage_name: 阶段名称
        """
        try:
            logger.info(f"UI请求执行阶段: {stage_name}")
            
            # 检查阶段依赖
            if not self._workflow_manager.can_execute_stage(stage_name):
                deps = self._workflow_manager.get_stage_dependencies(stage_name)
                error_msg = f"阶段 '{stage_name}' 依赖未满足，需要先完成: {', '.join(deps)}"
                self.error_occurred.emit(error_msg)
                return
            
            # 执行阶段
            success = self._workflow_manager.execute_stage(stage_name)
            
            if not success:
                self.error_occurred.emit(f"阶段 '{stage_name}' 执行失败")
                
        except Exception as e:
            error_msg = f"执行阶段异常: {str(e)}"
            logger.error(error_msg)
            self.error_occurred.emit(error_msg)
    
    @Slot(str)
    def reset_stage(self, stage_name: str):
        """重置指定阶段
        
        Args:
            stage_name: 阶段名称
        """
        try:
            logger.info(f"UI请求重置阶段: {stage_name}")
            self._workflow_manager.reset_stage(stage_name)
            self.info_message.emit(f"阶段 '{stage_name}' 已重置")
            
        except Exception as e:
            error_msg = f"重置阶段异常: {str(e)}"
            logger.error(error_msg)
            self.error_occurred.emit(error_msg)
    
    # ===== 关系分析方法 =====
    
    @Slot()
    def analyze_relationships(self):
        """执行关系分析"""
        try:
            logger.info("UI请求执行关系分析")
            self.operation_started.emit("关系分析")
            
            success = self._workflow_manager.execute_stage("relationship_analysis")
            
            if success:
                self.operation_completed.emit("关系分析", True)
            else:
                self.error_occurred.emit("关系分析执行失败")
                self.operation_completed.emit("关系分析", False)
                
        except Exception as e:
            error_msg = f"关系分析异常: {str(e)}"
            logger.error(error_msg)
            self.error_occurred.emit(error_msg)
            self.operation_completed.emit("关系分析", False)
    
    @Slot()
    def detect_cycles(self):
        """检测环结构"""
        try:
            logger.info("UI请求检测环结构")
            cycles = self._workflow_manager.get_detected_cycles()
            self.cycles_detected.emit(cycles)
            
            if cycles:
                self.info_message.emit(f"检测到 {len(cycles)} 个环结构")
            else:
                self.info_message.emit("未检测到环结构")
                
        except Exception as e:
            error_msg = f"检测环结构异常: {str(e)}"
            logger.error(error_msg)
            self.error_occurred.emit(error_msg)
    
    @Slot(str, str)
    def break_joint_in_cycle(self, cycle_id: str, joint_name: str):
        """在环中断开指定关节
        
        Args:
            cycle_id: 环ID
            joint_name: 关节名称
        """
        try:
            logger.info(f"UI请求在环 {cycle_id} 中断开关节: {joint_name}")
            
            success = self._workflow_manager.break_joint_in_cycle(int(cycle_id), joint_name)
            
            if success:
                self.info_message.emit(f"关节 '{joint_name}' 已断开")
                # 重新检测环
                self.detect_cycles()
            else:
                self.error_occurred.emit(f"断开关节 '{joint_name}' 失败")
                
        except Exception as e:
            error_msg = f"断开关节异常: {str(e)}"
            logger.error(error_msg)
            self.error_occurred.emit(error_msg)
    
    @Slot(str)
    def set_root_node_strategy(self, strategy: str):
        """设置根节点选择策略
        
        Args:
            strategy: 策略名称 ("center", "max_degree")
        """
        try:
            logger.info(f"UI请求设置根节点策略: {strategy}")
            
            success = self._workflow_manager.set_root_node_strategy(strategy)
            
            if success:
                self.info_message.emit(f"根节点策略已设置为: {strategy}")
                # 重新生成装配树
                trees = self._workflow_manager.get_assembly_trees()
                self.assembly_trees_generated.emit(trees)
            else:
                self.error_occurred.emit(f"设置根节点策略失败: {strategy}")
                
        except Exception as e:
            error_msg = f"设置根节点策略异常: {str(e)}"
            logger.error(error_msg)
            self.error_occurred.emit(error_msg)
    
    # ===== 数据获取方法 =====
    
    def get_current_state(self) -> Optional[WorkflowState]:
        """获取当前工作流状态
        
        Returns:
            WorkflowState: 当前状态
        """
        return self._current_state
    
    def get_project_data(self) -> Optional[ExportData]:
        """获取项目数据
        
        Returns:
            ExportData: 项目数据
        """
        if self._current_state:
            return self._current_state.project_data
        return None
    
    def get_components(self) -> List[ComponentInfo]:
        """获取组件列表
        
        Returns:
            List[ComponentInfo]: 组件列表
        """
        if self._current_state and self._current_state.project_data:
            return self._current_state.project_data.components
        return []
    
    def get_joints(self) -> List[JointInfo]:
        """获取关节列表
        
        Returns:
            List[JointInfo]: 关节列表
        """
        if self._current_state and self._current_state.project_data:
            return self._current_state.project_data.joints
        return []
    
    def get_joint_relationships(self) -> Dict[str, JointPairwiseRelationship]:
        """获取关节关系
        
        Returns:
            Dict[str, JointPairwiseRelationship]: 关节关系字典
        """
        if self._current_state:
            return self._current_state.joint_pairwise_relationships
        return {}
    
    def get_assembly_trees(self) -> List[AssemblyTreeInfo]:
        """获取装配树
        
        Returns:
            List[AssemblyTreeInfo]: 装配树列表
        """
        if self._current_state:
            return self._current_state.assembly_trees
        return []
    
    def get_stage_status(self, stage_name: str) -> Optional[str]:
        """获取阶段状态
        
        Args:
            stage_name: 阶段名称
            
        Returns:
            str: 阶段状态 ("pending", "in_progress", "completed", "failed")
        """
        if self._current_state:
            return self._current_state.get_stage_status(stage_name)
        return None
    
    # ===== 信号处理方法 =====
    
    def _on_workflow_state_changed(self, state: WorkflowState):
        """处理工作流状态变化"""
        self._current_state = state
        self.workflow_state_changed.emit(state)
        
        # 更新状态文本
        status_text = f"阶段: {state.current_stage} | 状态: {state.status.value}"
        self.workflow_status_changed.emit(status_text)
        
        logger.info(f"工作流状态更新: {status_text}")
    
    def _on_stage_execution_started(self, stage_name: str):
        """处理阶段执行开始"""
        self.stage_execution_started.emit(stage_name)
        self.operation_started.emit(f"执行阶段: {stage_name}")
        logger.info(f"阶段执行开始: {stage_name}")
    
    def _on_stage_execution_progress(self, stage_name: str, progress: int, message: str):
        """处理阶段执行进度"""
        self.stage_execution_progress.emit(stage_name, progress, message)
        self.operation_progress.emit(progress, message)
    
    def _on_stage_execution_completed(self, stage_name: str, result: StageExecutionResult):
        """处理阶段执行完成"""
        success = result.success
        self.stage_execution_completed.emit(stage_name, success)
        self.operation_completed.emit(f"阶段: {stage_name}", success)
        
        if success:
            self.info_message.emit(f"阶段 '{stage_name}' 执行完成")
        else:
            self.error_occurred.emit(f"阶段 '{stage_name}' 执行失败: {result.error_message}")
    
    def _on_stage_execution_failed(self, stage_name: str, error_message: str):
        """处理阶段执行失败"""
        self.stage_execution_failed.emit(stage_name, error_message)
        self.error_occurred.emit(f"阶段 '{stage_name}' 执行失败: {error_message}")
        self.operation_completed.emit(f"阶段: {stage_name}", False)
    
    def _on_project_loaded(self, result: LoadResult):
        """处理项目加载完成"""
        self.project_loaded.emit(result)
        
        if result.success:
            self.info_message.emit(f"项目加载成功: {len(result.export_data.components)} 个组件")
        else:
            self.error_occurred.emit(f"项目加载失败: {result.message}")
    
    def _on_project_data_updated(self, project_data: ExportData):
        """处理项目数据更新"""
        self.project_data_updated.emit(project_data)
        self.components_updated.emit(project_data.components)
        self.joints_updated.emit(project_data.joints)
    
    def _on_relationship_analysis_completed(self):
        """处理关系分析完成"""
        self.relationship_analysis_completed.emit()
        
        # 发送相关数据更新信号
        if self._current_state:
            self.joint_relationships_updated.emit(self._current_state.joint_pairwise_relationships)
            self.cycles_detected.emit(self._current_state.detected_cycles)
            self.assembly_trees_generated.emit(self._current_state.assembly_trees)
    
    def _on_error_occurred(self, error_message: str):
        """处理错误发生"""
        self.error_occurred.emit(error_message)
    
    def _on_warning_occurred(self, warning_message: str):
        """处理警告发生"""
        self.warning_occurred.emit(warning_message)
    
    def _on_info_message(self, info_message: str):
        """处理信息消息"""
        self.info_message.emit(info_message)
    
    # ===== 工具方法 =====
    
    def is_project_loaded(self) -> bool:
        """检查是否有项目已加载
        
        Returns:
            bool: 是否有项目已加载
        """
        return self._current_state is not None and self._current_state.project_data is not None
    
    def get_project_info(self) -> Optional[ProjectInfo]:
        """获取项目信息
        
        Returns:
            ProjectInfo: 项目信息
        """
        if self._current_state and self._current_state.project_data:
            return self._current_state.project_data.meta
        return None
    
    def get_processing_summary(self) -> str:
        """获取处理摘要
        
        Returns:
            str: 处理摘要文本
        """
        if not self._current_state:
            return "无活动项目"
        
        status = self._current_state.get_processing_status()
        return (
            f"工作流摘要:\n"
            f"  当前阶段: {self._current_state.current_stage}\n"
            f"  整体状态: {self._current_state.status.value}\n"
            f"  组件数量: {status['component_count']}\n"
            f"  关节数量: {status['joint_count']}\n"
            f"  已完成阶段: {status['completed_stages']}/{status['total_stages']}"
        )