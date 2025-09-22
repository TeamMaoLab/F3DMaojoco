"""
STL模型管理器

负责STL文件的加载、缓存和管理。
"""

from typing import List, Dict, Optional, Any, Tuple
from pathlib import Path
import pyvista as pv
import numpy as np
from dataclasses import dataclass

from utils.logger import logger
from .project_data_loader import ComponentData, ProjectDataLoader
from .coordinate_transformer import CoordinateTransformer, Transform4D


@dataclass
class ModelData:
    """模型数据"""
    mesh: pv.PolyData
    name: str
    color: str
    component_id: int
    is_transformed: bool = False
    original_bounds: Optional[Tuple[float, float, float, float, float, float]] = None


class STLModelManager:
    """STL模型管理器
    
    负责STL文件的加载、坐标变换应用和模型数据管理。
    """
    
    def __init__(self) -> None:
        """初始化STL模型管理器"""
        self._data_loader: Optional[ProjectDataLoader] = None
        self._transformer: CoordinateTransformer = CoordinateTransformer()
        self._model_cache: Dict[str, ModelData] = {}
        self._color_palette = [
            "#4A90E2", "#7ED321", "#F5A623", "#BD10E0", "#50E3C2", "#B8E986",
            "#FF6B6B", "#4ECDC4", "#45B7D1", "#96CEB4", "#FECA57", "#FF9FF3",
            "#9B59B6", "#3498DB", "#2ECC71", "#F39C12", "#E74C3C", "#1ABC9C"
        ]
    
    def set_data_loader(self, data_loader: ProjectDataLoader) -> None:
        """设置数据加载器
        
        Args:
            data_loader: 项目数据加载器
        """
        self._data_loader = data_loader
        self.clear_cache()
    
    def load_transformed_models(self) -> List[ModelData]:
        """加载应用了坐标变换的模型
        
        Returns:
            List[ModelData]: 模型数据列表
        """
        if not self._data_loader or not self._data_loader.is_loaded():
            logger.warning("数据加载器未就绪")
            return []
        
        components = self._data_loader.get_components()
        project_dir = self._data_loader.get_project_directory()
        
        if not project_dir or not project_dir.exists():
            logger.error(f"项目目录不存在: {project_dir}")
            return []
        
        models = []
        
        for i, component in enumerate(components):
            if not component.stl_file:
                continue
            
            # 从缓存中查找
            cache_key = f"{component.name}_{component.component_id}"
            if cache_key in self._model_cache:
                models.append(self._model_cache[cache_key])
                continue
            
            # 加载STL文件 (JSON中的stl_file已经包含stl_files/前缀)
            stl_path = project_dir / component.stl_file
            if not stl_path.exists():
                logger.warning(f"STL文件不存在: {stl_path}")
                continue
            
            try:
                # 加载网格
                mesh = pv.read(stl_path)
                
                # 应用坐标变换
                is_transformed = False
                if component.world_transform:
                    # 添加调试日志验证变换矩阵数据
                    logger.debug(f"组件 {component.name} 的变换矩阵: {component.world_transform}")
                    logger.debug(f"变换矩阵类型: {type(component.world_transform)}")
                    
                    transform = self._transformer.create_transform_from_components(
                        component.world_transform
                    )
                    if transform:
                        # 记录变换统计信息
                        translation = self._transformer.get_translation_vector(transform)
                        logger.debug(f"组件 {component.name} 的平移向量: {translation}")
                        
                        # 应用变换前记录原始边界
                        original_bounds = mesh.bounds
                        logger.debug(f"组件 {component.name} 变换前边界: {original_bounds}")
                        
                        points = mesh.points
                        transformed_points = self._transformer.apply_transform_to_points(
                            points, transform
                        )
                        mesh.points = transformed_points
                        is_transformed = True
                        
                        # 记录变换后边界
                        new_bounds = mesh.bounds
                        logger.debug(f"组件 {component.name} 变换后边界: {new_bounds}")
                        logger.info(f"成功应用坐标变换到组件: {component.name}, 平移: {translation}")
                    else:
                        logger.warning(f"无法为组件 {component.name} 创建变换矩阵")
                
                # 选择颜色
                color = self._color_palette[i % len(self._color_palette)]
                
                # 保存原始边界
                original_bounds = mesh.bounds
                
                # 创建模型数据
                model_data = ModelData(
                    mesh=mesh,
                    name=component.name,
                    color=color,
                    component_id=component.component_id,
                    is_transformed=is_transformed,
                    original_bounds=original_bounds
                )
                
                # 缓存模型
                self._model_cache[cache_key] = model_data
                models.append(model_data)
                
            except Exception as e:
                logger.error(f"加载模型失败 {component.name}: {e}")
                continue
        
        logger.info(f"成功加载 {len(models)} 个模型")
        return models
    
    def load_plain_stl_models(self, directory: Path) -> List[ModelData]:
        """加载普通STL模型（无坐标变换）
        
        Args:
            directory: 包含STL文件的目录
            
        Returns:
            List[ModelData]: 模型数据列表
        """
        stl_dir = directory / "stl_files"
        if not stl_dir.exists():
            logger.error(f"STL目录不存在: {stl_dir}")
            return []
        
        models = []
        stl_files = list(stl_dir.glob("*.stl"))
        
        for i, stl_path in enumerate(stl_files):
            try:
                mesh = pv.read(stl_path)
                
                model_data = ModelData(
                    mesh=mesh,
                    name=stl_path.stem,
                    color=self._color_palette[i % len(self._color_palette)],
                    component_id=i,
                    is_transformed=False,
                    original_bounds=mesh.bounds
                )
                
                models.append(model_data)
                
            except Exception as e:
                logger.error(f"加载STL文件失败 {stl_path}: {e}")
                continue
        
        logger.info(f"成功加载 {len(models)} 个普通STL模型")
        return models
    
    def get_model_by_name(self, name: str) -> Optional[ModelData]:
        """根据名称获取模型
        
        Args:
            name: 模型名称
            
        Returns:
            Optional[ModelData]: 模型数据
        """
        for model_data in self._model_cache.values():
            if model_data.name == name:
                return model_data
        return None
    
    def get_model_by_component_id(self, component_id: int) -> Optional[ModelData]:
        """根据组件ID获取模型
        
        Args:
            component_id: 组件ID
            
        Returns:
            Optional[ModelData]: 模型数据
        """
        for model_data in self._model_cache.values():
            if model_data.component_id == component_id:
                return model_data
        return None
    
    def get_all_models(self) -> List[ModelData]:
        """获取所有缓存的模型
        
        Returns:
            List[ModelData]: 模型数据列表
        """
        return list(self._model_cache.values())
    
    def clear_cache(self) -> None:
        """清除模型缓存"""
        self._model_cache.clear()
        logger.info("模型缓存已清除")
    
    def get_model_statistics(self) -> Dict[str, Any]:
        """获取模型统计信息
        
        Returns:
            Dict[str, Any]: 统计信息
        """
        models = self.get_all_models()
        
        if not models:
            return {"count": 0}
        
        total_vertices = sum(len(model.mesh.points) for model in models)
        total_faces = sum(len(model.mesh.faces) for model in models)
        transformed_count = sum(1 for model in models if model.is_transformed)
        
        return {
            "count": len(models),
            "total_vertices": total_vertices,
            "total_faces": total_faces,
            "transformed_count": transformed_count,
            "plain_count": len(models) - transformed_count
        }
    
    def validate_model_integrity(self) -> List[str]:
        """验证模型完整性
        
        Returns:
            List[str]: 问题列表
        """
        issues = []
        
        for model_data in self._model_cache.values():
            mesh = model_data.mesh
            
            # 检查网格是否为空
            if mesh.n_points == 0:
                issues.append(f"模型 {model_data.name} 没有顶点")
            
            # 检查网格是否有面
            if mesh.n_faces == 0:
                issues.append(f"模型 {model_data.name} 没有面")
            
            # 检查边界是否合理
            bounds = mesh.bounds
            if any(abs(bounds[i]) > 10000 for i in range(6)):  # 10km边界检查
                issues.append(f"模型 {model_data.name} 边界异常: {bounds}")
        
        return issues
    
    def reload_models(self) -> List[ModelData]:
        """重新加载所有模型
        
        Returns:
            List[ModelData]: 重新加载的模型列表
        """
        self.clear_cache()
        return self.load_transformed_models()