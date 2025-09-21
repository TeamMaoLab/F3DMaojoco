"""
工作流管理器工厂

提供ProjectWorkflowManager的创建和配置功能，
简化UI层的初始化过程。
"""

from typing import Optional, Dict, Any
from pathlib import Path

from core.project_workflow_manager import ProjectWorkflowManager
from core.service_interfaces import (
    IDataLoadingService, IVisualizationService, 
    IStageManagementService, ITransformService
)
from core.transform_service import TransformService
from core.project_data_service import ProjectDataService
from core.stl_model_manager import STLModelManager


class WorkflowManagerFactory:
    """工作流管理器工厂
    
    提供统一的ProjectWorkflowManager创建和配置接口，
    简化UI层的初始化代码。
    """
    
    @staticmethod
    def create_default_manager() -> ProjectWorkflowManager:
        """创建默认配置的工作流管理器"""
        manager = ProjectWorkflowManager()
        
        # 设置默认服务
        manager.set_data_loading_service(ProjectDataService())
        manager.set_transform_service(TransformService())
        
        return manager
    
    @staticmethod
    def create_custom_manager(
        data_loading_service: Optional[IDataLoadingService] = None,
        visualization_service: Optional[IVisualizationService] = None,
        stage_management_service: Optional[IStageManagementService] = None,
        transform_service: Optional[ITransformService] = None
    ) -> ProjectWorkflowManager:
        """创建自定义配置的工作流管理器
        
        Args:
            data_loading_service: 数据加载服务
            visualization_service: 可视化服务
            stage_management_service: 阶段管理服务
            transform_service: 变换服务
            
        Returns:
            ProjectWorkflowManager: 配置好的工作流管理器
        """
        manager = ProjectWorkflowManager()
        
        # 设置自定义服务
        if data_loading_service:
            manager.set_data_loading_service(data_loading_service)
        
        if visualization_service:
            manager.set_visualization_service(visualization_service)
        
        if stage_management_service:
            manager.set_stage_management_service(stage_management_service)
        
        if transform_service:
            manager.set_transform_service(transform_service)
        
        return manager
    
    @staticmethod
    def create_with_config(config: Dict[str, Any]) -> ProjectWorkflowManager:
        """根据配置字典创建工作流管理器
        
        Args:
            config: 配置字典，包含服务配置和参数
            
        Returns:
            ProjectWorkflowManager: 配置好的工作流管理器
        """
        manager = ProjectWorkflowManager()
        
        # 解析配置并设置服务
        if 'data_loading' in config:
            data_config = config['data_loading']
            if data_config.get('enabled', True):
                manager.set_data_loading_service(ProjectDataService())
        
        if 'transform' in config:
            transform_config = config['transform']
            if transform_config.get('enabled', True):
                manager.set_transform_service(TransformService())
        
        # 设置阶段参数
        if 'stages' in config:
            stage_configs = config['stages']
            for stage_name, stage_config in stage_configs.items():
                manager.set_stage_config(stage_name, stage_config)
        
        return manager
    
    @staticmethod
    def create_for_quick_start(project_directory: Path) -> ProjectWorkflowManager:
        """为快速启动模式创建工作流管理器
        
        Args:
            project_directory: 项目目录路径
            
        Returns:
            ProjectWorkflowManager: 预配置的工作流管理器
        """
        manager = ProjectWorkflowManager()
        
        # 设置服务
        manager.set_data_loading_service(ProjectDataService())
        manager.set_transform_service(TransformService())
        
        # 预加载项目
        if project_directory.exists():
            manager.load_project(project_directory)
        
        return manager


class WorkflowManagerConfig:
    """工作流管理器配置类
    
    提供类型安全的配置接口。
    """
    
    def __init__(self):
        """初始化配置"""
        self.enable_data_loading = True
        self.enable_visualization = True
        self.enable_stage_management = True
        self.enable_transform = True
        
        self.data_loading_config = {}
        self.visualization_config = {}
        self.stage_management_config = {}
        self.transform_config = {}
        
        self.stage_configs = {}
    
    def set_data_loading_enabled(self, enabled: bool):
        """设置数据加载服务是否启用"""
        self.enable_data_loading = enabled
        return self
    
    def set_visualization_enabled(self, enabled: bool):
        """设置可视化服务是否启用"""
        self.enable_visualization = enabled
        return self
    
    def set_stage_management_enabled(self, enabled: bool):
        """设置阶段管理服务是否启用"""
        self.enable_stage_management = enabled
        return self
    
    def set_transform_enabled(self, enabled: bool):
        """设置变换服务是否启用"""
        self.enable_transform = enabled
        return self
    
    def set_data_loading_config(self, config: Dict[str, Any]):
        """设置数据加载服务配置"""
        self.data_loading_config = config
        return self
    
    def set_visualization_config(self, config: Dict[str, Any]):
        """设置可视化服务配置"""
        self.visualization_config = config
        return self
    
    def set_stage_management_config(self, config: Dict[str, Any]):
        """设置阶段管理服务配置"""
        self.stage_management_config = config
        return self
    
    def set_transform_config(self, config: Dict[str, Any]):
        """设置变换服务配置"""
        self.transform_config = config
        return self
    
    def set_stage_config(self, stage_name: str, config: Dict[str, Any]):
        """设置特定阶段的配置"""
        self.stage_configs[stage_name] = config
        return self
    
    def to_dict(self) -> Dict[str, Any]:
        """转换为字典"""
        return {
            'data_loading': {
                'enabled': self.enable_data_loading,
                **self.data_loading_config
            },
            'visualization': {
                'enabled': self.enable_visualization,
                **self.visualization_config
            },
            'stage_management': {
                'enabled': self.enable_stage_management,
                **self.stage_management_config
            },
            'transform': {
                'enabled': self.enable_transform,
                **self.transform_config
            },
            'stages': self.stage_configs
        }
    
    @staticmethod
    def default() -> 'WorkflowManagerConfig':
        """创建默认配置"""
        return WorkflowManagerConfig()
    
    @staticmethod
    def minimal() -> 'WorkflowManagerConfig':
        """创建最小配置（仅启用必要服务）"""
        return (WorkflowManagerConfig()
                .set_visualization_enabled(False)
                .set_stage_management_enabled(False))
    
    @staticmethod
    def full_featured() -> 'WorkflowManagerConfig':
        """创建全功能配置"""
        config = WorkflowManagerConfig()
        
        # 设置详细配置
        config.set_data_loading_config({
            'cache_enabled': True,
            'async_loading': True,
            'progress_callback': True
        })
        
        config.set_visualization_config({
            'background_color': '#2E2E2E',
            'show_edges': True,
            'show_axes': True,
            'lighting_intensity': 1.0
        })
        
        config.set_stage_management_config({
            'auto_advance': True,
            'error_handling': 'strict',
            'parallel_execution': False
        })
        
        config.set_transform_config({
            'coordinate_system': 'millimeters',
            'precision': 6,
            'validation_enabled': True
        })
        
        return config


# 使用示例
def example_usage():
    """使用示例"""
    
    # 示例1: 使用工厂创建默认管理器
    manager1 = WorkflowManagerFactory.create_default_manager()
    
    # 示例2: 使用配置类创建管理器
    config = (WorkflowManagerConfig.default()
              .set_data_loading_config({'cache_enabled': True})
              .set_visualization_config({'background_color': '#FFFFFF'}))
    
    manager2 = WorkflowManagerFactory.create_with_config(config.to_dict())
    
    # 示例3: 为快速启动创建管理器
    project_dir = Path('/path/to/project')
    manager3 = WorkflowManagerFactory.create_for_quick_start(project_dir)
    
    # 示例4: 完全自定义
    manager4 = WorkflowManagerFactory.create_custom_manager(
        data_loading_service=ProjectDataService(),
        transform_service=TransformService()
    )
    
    return [manager1, manager2, manager3, manager4]