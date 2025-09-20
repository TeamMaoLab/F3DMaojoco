"""
项目数据加载器

负责加载和解析项目数据，包括component_positions.json文件。
"""

from typing import Dict, List, Any, Optional
from pathlib import Path
import json
from dataclasses import dataclass

from utils.logger import logger


@dataclass
class ComponentData:
    """组件数据"""
    name: str
    occurrence_name: str
    full_path_name: str
    component_id: int
    stl_file: Optional[str] = None
    world_transform: Optional[List[List[float]]] = None
    bodies_count: int = 0
    has_children: bool = False


@dataclass
class ProjectMetadata:
    """项目元数据"""
    export_time: str
    geometry_unit: str
    position_unit: str
    matrix_storage: str
    component_count: int
    joint_count: int
    format_version: str = "1.0"


class ProjectDataLoader:
    """项目数据加载器
    
    负责加载和解析component_positions.json文件，
    提供结构化的项目数据访问接口。
    """
    
    def __init__(self) -> None:
        """初始化项目数据加载器"""
        self._project_directory: Optional[Path] = None
        self._metadata: Optional[ProjectMetadata] = None
        self._components: List[ComponentData] = []
        self._joints: List[Dict[str, Any]] = []
        self._is_loaded: bool = False
    
    def load_project(self, project_directory: Path) -> bool:
        """加载项目数据
        
        Args:
            project_directory: 项目目录路径
            
        Returns:
            bool: 是否加载成功
        """
        logger.info(f"加载项目数据: {project_directory}")
        
        try:
            # 检查component_positions.json是否存在
            positions_file = project_directory / "component_positions.json"
            if not positions_file.exists():
                logger.info(f"未找到component_positions.json文件，将使用普通加载模式")
                self._project_directory = project_directory
                self._is_loaded = False
                return True
            
            # 加载JSON文件
            with open(positions_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            # 解析元数据
            self._parse_metadata(data.get("meta", {}))
            
            # 解析组件数据
            self._parse_components(data.get("components", []))
            
            # 解析关节数据
            self._parse_joints(data.get("joints", []))
            
            self._project_directory = project_directory
            self._is_loaded = True
            
            logger.success(f"项目数据加载成功: {len(self._components)} 个组件, {len(self._joints)} 个关节")
            return True
            
        except Exception as e:
            logger.error(f"项目数据加载失败: {e}")
            return False
    
    def _parse_metadata(self, meta_data: Dict[str, Any]) -> None:
        """解析项目元数据"""
        self._metadata = ProjectMetadata(
            export_time=meta_data.get("export_time", ""),
            geometry_unit=meta_data.get("geometry_unit", "millimeters"),
            position_unit=meta_data.get("position_unit", "millimeters"),
            matrix_storage=meta_data.get("matrix_storage", "4x4_array"),
            component_count=meta_data.get("count_components", 0),
            joint_count=meta_data.get("count_joints", 0),
            format_version=meta_data.get("format_version", "1.0")
        )
    
    def _parse_components(self, components_data: List[Dict[str, Any]]) -> None:
        """解析组件数据"""
        self._components = []
        
        for comp_data in components_data:
            component = ComponentData(
                name=comp_data.get("name", ""),
                occurrence_name=comp_data.get("occurrence_name", ""),
                full_path_name=comp_data.get("full_path_name", ""),
                component_id=comp_data.get("component_id", 0),
                stl_file=comp_data.get("stl_file"),
                world_transform=comp_data.get("world_transform", {}).get("matrix"),
                bodies_count=comp_data.get("bodies_count", 0),
                has_children=comp_data.get("has_children", False)
            )
            self._components.append(component)
    
    def _parse_joints(self, joints_data: List[Dict[str, Any]]) -> None:
        """解析关节数据"""
        self._joints = joints_data.copy()
    
    def get_components(self) -> List[ComponentData]:
        """获取所有组件数据"""
        return self._components.copy()
    
    def get_component_by_name(self, name: str) -> Optional[ComponentData]:
        """根据名称获取组件"""
        for component in self._components:
            if component.name == name:
                return component
        return None
    
    def get_joints(self) -> List[Dict[str, Any]]:
        """获取所有关节数据"""
        return self._joints.copy()
    
    def get_metadata(self) -> Optional[ProjectMetadata]:
        """获取项目元数据"""
        return self._metadata
    
    def get_stl_directory(self) -> Optional[Path]:
        """获取STL文件目录"""
        if self._project_directory:
            return self._project_directory / "stl_files"
        return None
    
    def has_transform_data(self) -> bool:
        """是否有变换数据"""
        return self._is_loaded and any(comp.world_transform for comp in self._components)
    
    def get_project_directory(self) -> Optional[Path]:
        """获取项目目录"""
        return self._project_directory
    
    def validate_stl_files(self) -> List[str]:
        """验证STL文件是否存在，返回缺失的文件列表"""
        missing_files = []
        stl_dir = self.get_stl_directory()
        
        if not stl_dir or not stl_dir.exists():
            return [comp.stl_file for comp in self._components if comp.stl_file]
        
        for component in self._components:
            if component.stl_file:
                stl_path = stl_dir / component.stl_file
                if not stl_path.exists():
                    missing_files.append(component.stl_file)
        
        return missing_files
    
    def clear(self) -> None:
        """清除加载的数据"""
        self._project_directory = None
        self._metadata = None
        self._components.clear()
        self._joints.clear()
        self._is_loaded = False
        logger.info("项目数据已清除")
    
    def is_loaded(self) -> bool:
        """检查是否已加载数据"""
        return self._is_loaded