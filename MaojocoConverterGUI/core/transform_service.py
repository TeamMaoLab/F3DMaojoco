"""
变换服务

统一的服务接口，协调各个逻辑组件完成STL文件的坐标变换和加载。
"""

from typing import List, Optional
from pathlib import Path
from enum import Enum
from dataclasses import dataclass

from utils.logger import logger
from core.project_data_loader import ProjectDataLoader, ProjectMetadata
from core.coordinate_transformer import CoordinateTransformer
from core.stl_model_manager import STLModelManager, ModelData
from core.domain_types import (
    TransformStatistics, LoadResult, ProjectInfo
)


class LoadMode(Enum):
    """加载模式"""
    AUTO = "auto"  # 自动检测
    TRANSFORMED = "transformed"  # 应用坐标变换
    PLAIN = "plain"  # 普通加载


# 使用 domain_types.py 中的 LoadResult，避免重复定义


class TransformService:
    """变换服务
    
    统一的接口，协调数据加载、坐标变换和模型管理。
    """
    
    def __init__(self) -> None:
        """初始化变换服务"""
        self._data_loader = ProjectDataLoader()
        self._transformer = CoordinateTransformer()
        self._model_manager = STLModelManager()
        self._model_manager.set_data_loader(self._data_loader)
        self._current_project: Optional[Path] = None
        self._current_load_mode: LoadMode = LoadMode.AUTO
    
    def load_project(self, project_directory: Path, 
                    load_mode: LoadMode = LoadMode.AUTO) -> LoadResult:
        """加载项目并返回模型数据
        
        Args:
            project_directory: 项目目录路径
            load_mode: 加载模式
            
        Returns:
            LoadResult: 加载结果
        """
        logger.info(f"开始加载项目: {project_directory}, 模式: {load_mode.value}")
        
        try:
            # 清除之前的数据
            self.clear_data()
            
            # 加载项目数据
            if not self._data_loader.load_project(project_directory):
                return LoadResult(
                    success=False,
                    models=[],
                    message="项目数据加载失败",
                    load_mode=load_mode
                )
            
            self._current_project = project_directory
            
            # 确定实际加载模式
            actual_mode = self._determine_load_mode(load_mode)
            
            # 根据模式加载模型
            if actual_mode == LoadMode.TRANSFORMED:
                models = self._model_manager.load_transformed_models()
                message = f"成功加载 {len(models)} 个应用坐标变换的模型"
            elif actual_mode == LoadMode.PLAIN:
                models = self._model_manager.load_plain_stl_models(project_directory)
                message = f"成功加载 {len(models)} 个普通STL模型"
            else:
                # AUTO模式：根据是否有变换数据决定
                if self._data_loader.has_transform_data():
                    models = self._model_manager.load_transformed_models()
                    actual_mode = LoadMode.TRANSFORMED
                    message = f"自动检测到变换数据，成功加载 {len(models)} 个变换后的模型"
                else:
                    models = self._model_manager.load_plain_stl_models(project_directory)
                    actual_mode = LoadMode.PLAIN
                    message = f"未检测到变换数据，成功加载 {len(models)} 个普通模型"
            
            # 获取项目信息
            project_info = self._get_project_info(actual_mode)
            
            self._current_load_mode = actual_mode
            
            logger.success(message)
            return LoadResult(
                success=True,
                models=models,
                message=message,
                project_info=project_info,
                load_mode=actual_mode
            )
            
        except Exception as e:
            error_msg = f"项目加载失败: {e}"
            logger.error(error_msg)
            return LoadResult(
                success=False,
                models=[],
                message=error_msg,
                load_mode=load_mode,
                error_details=str(e)
            )
    
    def _determine_load_mode(self, requested_mode: LoadMode) -> LoadMode:
        """确定实际加载模式"""
        if requested_mode != LoadMode.AUTO:
            return requested_mode
        
        # AUTO模式：根据是否有变换数据决定
        if self._data_loader.has_transform_data():
            return LoadMode.TRANSFORMED
        else:
            return LoadMode.PLAIN
    
    def _get_project_info(self, load_mode: LoadMode) -> ProjectInfo:
        """获取项目信息
        
        Args:
            load_mode: 当前加载模式，用于记录项目状态
        """
        metadata = self._data_loader.get_metadata()
        
        if metadata:
            return ProjectInfo(
                project_directory=str(self._current_project),
                export_time=metadata.export_time,
                geometry_unit=metadata.geometry_unit,
                position_unit=metadata.position_unit,
                component_count=metadata.component_count,
                joint_count=metadata.joint_count,
                format_version="1.0",
                has_transform_data=self._data_loader.has_transform_data()
            )
        else:
            return ProjectInfo(
                project_directory=str(self._current_project),
                export_time="",
                geometry_unit="millimeters",
                position_unit="millimeters",
                component_count=0,
                joint_count=0,
                format_version="1.0",
                has_transform_data=self._data_loader.has_transform_data()
            )
    
    def reload_current_project(self) -> LoadResult:
        """重新加载当前项目"""
        if not self._current_project:
            return LoadResult(
                success=False,
                models=[],
                message="没有当前加载的项目",
                load_mode=self._current_load_mode
            )
        
        return self.load_project(self._current_project, self._current_load_mode)
    
    def get_models_by_component_names(self, names: List[str]) -> List[ModelData]:
        """根据组件名称列表获取模型
        
        Args:
            names: 组件名称列表
            
        Returns:
            List[ModelData]: 模型数据列表
        """
        models = []
        for name in names:
            model = self._model_manager.get_model_by_name(name)
            if model:
                models.append(model)
        return models
    
    def get_all_model_names(self) -> List[str]:
        """获取所有模型名称"""
        models = self._model_manager.get_all_models()
        return [model.name for model in models]
    
    def validate_current_project(self) -> List[str]:
        """验证当前项目的完整性
        
        Returns:
            List[str]: 问题列表
        """
        issues = []
        
        if not self._current_project:
            issues.append("没有加载项目")
            return issues
        
        # 验证STL文件
        missing_files = self._data_loader.validate_stl_files()
        if missing_files:
            issues.append(f"缺失STL文件: {missing_files}")
        
        # 验证模型完整性
        model_issues = self._model_manager.validate_model_integrity()
        issues.extend(model_issues)
        
        return issues
    
    def get_transform_statistics(self) -> TransformStatistics:
        """获取变换统计信息"""
        if not self._data_loader.has_transform_data():
            return TransformStatistics(
                has_transform_data=False,
                total_transforms=0,
                max_translation_distance=0.0
            )
        
        components = self._data_loader.get_components()
        transforms = []
        
        for component in components:
            if component.world_transform:
                transform = self._transformer.create_transform_from_components(
                    component.world_transform
                )
                if transform:
                    transforms.append(transform)
        
        return self._transformer.get_transform_statistics(transforms)
    
    def clear_data(self) -> None:
        """清除所有数据"""
        self._data_loader.clear()
        self._model_manager.clear_cache()
        self._current_project = None
        self._current_load_mode = LoadMode.AUTO
        logger.info("变换服务数据已清除")
    
    def get_current_project_info(self) -> Optional[ProjectInfo]:
        """获取当前项目信息"""
        if not self._current_project:
            return None
        
        return self._get_project_info(self._current_load_mode)
    
    def is_project_loaded(self) -> bool:
        """检查是否已加载项目"""
        return self._current_project is not None
    
    def get_load_mode(self) -> LoadMode:
        """获取当前加载模式"""
        return self._current_load_mode
    
    def switch_load_mode(self, new_mode: LoadMode) -> LoadResult:
        """切换加载模式
        
        Args:
            new_mode: 新的加载模式
            
        Returns:
            LoadResult: 加载结果
        """
        if not self._current_project:
            return LoadResult(
                success=False,
                models=[],
                message="没有当前加载的项目",
                load_mode=new_mode
            )
        
        logger.info(f"切换加载模式: {self._current_load_mode.value} -> {new_mode.value}")
        return self.load_project(self._current_project, new_mode)