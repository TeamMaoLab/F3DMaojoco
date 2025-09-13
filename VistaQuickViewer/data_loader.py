"""
数据加载器

用于解析和加载F3DMaojocoScripts导出的数据。
支持读取ExportResult目录中的JSON数据和STL文件。
复用F3DMaojocoScripts/common/data_types中的load_export_data函数。
"""

import os
import logging
from typing import Optional, List, Dict, Any
from pathlib import Path

# 直接导入F3DMaojocoScripts的common模块
from F3DMaojocoScripts.common.data_types import (
    ExportData, ExportMetadata, ComponentInfo, JointInfo, Transform4D,
    load_export_data
)


class ExportDataLoader:
    """导出数据加载器
    
    负责加载和解析F3DMaojocoScripts导出的数据：
    - 读取JSON格式的导出数据
    - 解析零部件和关节信息
    - 验证数据完整性
    - 提供数据访问接口
    """
    
    def __init__(self, export_dir: str, logger: Optional[logging.Logger] = None):
        """初始化数据加载器
        
        Args:
            export_dir: 导出目录路径
            logger: 可选的日志记录器
        """
        self.export_dir = Path(export_dir)
        self.logger = logger or logging.getLogger(__name__)
        
        # 数据缓存
        self._export_data: Optional[ExportData] = None
        self._stl_files: List[str] = []
        
        # 验证目录存在
        if not self.export_dir.exists():
            raise FileNotFoundError(f"导出目录不存在: {export_dir}")
    
    def load_export_data(self) -> ExportData:
        """加载导出数据
        
        Returns:
            ExportData: 加载的导出数据
            
        Raises:
            FileNotFoundError: 当数据文件不存在时
            json.JSONDecodeError: 当JSON格式错误时
        """
        if self._export_data is not None:
            return self._export_data
        
        # 查找JSON数据文件
        json_file = self.export_dir / "component_positions.json"
        if not json_file.exists():
            raise FileNotFoundError(f"未找到导出数据文件: {json_file}")
        
        self.logger.info(f"加载导出数据: {json_file}")
        
        # 使用F3DMaojocoScripts/common/data_types中的load_export_data函数
        self._export_data = load_export_data(str(json_file), self.logger)
        
        if self._export_data is None:
            raise RuntimeError(f"加载导出数据失败: {json_file}")
        
        # 验证数据完整性
        self._validate_export_data()
        
        self.logger.info(f"数据加载成功: {len(self._export_data.components)} 个零部件, {len(self._export_data.joints)} 个关节")
        return self._export_data
    
    def load_stl_files(self) -> List[str]:
        """加载STL文件列表
        
        Returns:
            List[str]: STL文件路径列表
        """
        if self._stl_files:
            return self._stl_files
        
        # 查找STL文件目录
        stl_dir = self.export_dir / "stl_files"
        if not stl_dir.exists():
            self.logger.warning("未找到STL文件目录")
            return []
        
        # 查找所有STL文件
        stl_files = list(stl_dir.glob("*.stl"))
        self._stl_files = [str(f) for f in stl_files]
        
        self.logger.info(f"找到 {len(self._stl_files)} 个STL文件")
        return self._stl_files
    
    def get_component_by_name(self, name: str) -> Optional[ComponentInfo]:
        """根据名称获取零部件信息
        
        Args:
            name: 零部件名称
            
        Returns:
            Optional[ComponentInfo]: 零部件信息，未找到时返回None
        """
        if self._export_data is None:
            self.load_export_data()
        
        for component in self._export_data.components:
            if component.name == name:
                return component
        
        return None
    
    def get_joint_by_name(self, name: str) -> Optional[JointInfo]:
        """根据名称获取关节信息
        
        Args:
            name: 关节名称
            
        Returns:
            Optional[JointInfo]: 关节信息，未找到时返回None
        """
        if self._export_data is None:
            self.load_export_data()
        
        for joint in self._export_data.joints:
            if joint.name == name:
                return joint
        
        return None
    
    def get_components_hierarchy(self) -> Dict[str, List[ComponentInfo]]:
        """获取零部件层次结构
        
        Returns:
            Dict[str, List[ComponentInfo]]: 按父组件分组的零部件字典
        """
        if self._export_data is None:
            self.load_export_data()
        
        hierarchy = {}
        
        for component in self._export_data.components:
            # 从完整路径中提取父组件
            path_parts = component.full_path_name.split('/')
            if len(path_parts) > 1:
                parent_name = path_parts[-2]
            else:
                parent_name = "root"
            
            if parent_name not in hierarchy:
                hierarchy[parent_name] = []
            
            hierarchy[parent_name].append(component)
        
        return hierarchy
    
    def get_stl_file_path(self, component: ComponentInfo) -> Optional[str]:
        """获取零部件对应的STL文件路径
        
        Args:
            component: 零部件信息
            
        Returns:
            Optional[str]: STL文件路径，未找到时返回None
        """
        if not component.stl_file:
            return None
        
        # 构建完整路径
        stl_path = self.export_dir / component.stl_file
        if stl_path.exists():
            return str(stl_path)
        
        # 尝试在stl_files目录中查找
        stl_dir = self.export_dir / "stl_files"
        if stl_dir.exists():
            stl_path = stl_dir / Path(component.stl_file).name
            if stl_path.exists():
                return str(stl_path)
        
        return None
    
    def get_export_metadata(self) -> Optional[ExportMetadata]:
        """获取导出元数据
        
        Returns:
            Optional[ExportMetadata]: 导出元数据
        """
        if self._export_data is None:
            self.load_export_data()
        
        return self._export_data.meta
    
    def get_scene_bounds(self) -> Optional[Dict[str, float]]:
        """获取场景边界
        
        Returns:
            Optional[Dict[str, float]]: 包含min_x, max_x, min_y, max_y, min_z, max_z的字典
        """
        if self._export_data is None:
            self.load_export_data()
        
        if not self._export_data.components:
            return None
        
        # 初始化边界
        bounds = {
            'min_x': float('inf'),
            'max_x': float('-inf'),
            'min_y': float('inf'),
            'max_y': float('-inf'),
            'min_z': float('inf'),
            'max_z': float('-inf')
        }
        
        # 计算所有零部件的边界
        for component in self._export_data.components:
            if component.world_transform:
                # 获取STL文件路径
                stl_path = self.get_stl_file_path(component)
                if stl_path and os.path.exists(stl_path):
                    try:
                        import pyvista as pv
                        import numpy as np
                        
                        # 加载STL文件
                        mesh = pv.read(stl_path)
                        
                        # 应用变换矩阵
                        matrix = component.world_transform.matrix
                        
                        # 清理浮点误差
                        cleaned_matrix = []
                        for row in matrix:
                            cleaned_row = []
                            for val in row:
                                if abs(val) < 1e-10:
                                    cleaned_row.append(0.0)
                                else:
                                    cleaned_row.append(val)
                            cleaned_matrix.append(cleaned_row)
                        
                        # 创建变换矩阵
                        transform_matrix = np.array([
                            [cleaned_matrix[0][0], cleaned_matrix[0][1], cleaned_matrix[0][2], cleaned_matrix[0][3]],
                            [cleaned_matrix[1][0], cleaned_matrix[1][1], cleaned_matrix[1][2], cleaned_matrix[1][3]],
                            [cleaned_matrix[2][0], cleaned_matrix[2][1], cleaned_matrix[2][2], cleaned_matrix[2][3]],
                            [0.0, 0.0, 0.0, 1.0]
                        ], dtype=np.float64)
                        
                        # 转置为列优先格式
                        transform_matrix = transform_matrix.T
                        
                        # 应用变换
                        transformed_mesh = mesh.transform(transform_matrix)
                        
                        # 更新边界
                        mesh_bounds = transformed_mesh.bounds
                        bounds['min_x'] = min(bounds['min_x'], mesh_bounds[0])
                        bounds['max_x'] = max(bounds['max_x'], mesh_bounds[1])
                        bounds['min_y'] = min(bounds['min_y'], mesh_bounds[2])
                        bounds['max_y'] = max(bounds['max_y'], mesh_bounds[3])
                        bounds['min_z'] = min(bounds['min_z'], mesh_bounds[4])
                        bounds['max_z'] = max(bounds['max_z'], mesh_bounds[5])
                        
                    except Exception as e:
                        # 如果STL文件加载失败，使用变换矩阵的位置部分
                        transform = component.world_transform
                        pos_x = transform.matrix[0][3]
                        pos_y = transform.matrix[1][3]
                        pos_z = transform.matrix[2][3]
                        
                        bounds['min_x'] = min(bounds['min_x'], pos_x - 10)
                        bounds['max_x'] = max(bounds['max_x'], pos_x + 10)
                        bounds['min_y'] = min(bounds['min_y'], pos_y - 10)
                        bounds['max_y'] = max(bounds['max_y'], pos_y + 10)
                        bounds['min_z'] = min(bounds['min_z'], pos_z - 10)
                        bounds['max_z'] = max(bounds['max_z'], pos_z + 10)
                else:
                    # 如果没有STL文件，使用变换矩阵的位置部分
                    transform = component.world_transform
                    pos_x = transform.matrix[0][3]
                    pos_y = transform.matrix[1][3]
                    pos_z = transform.matrix[2][3]
                    
                    bounds['min_x'] = min(bounds['min_x'], pos_x - 10)
                    bounds['max_x'] = max(bounds['max_x'], pos_x + 10)
                    bounds['min_y'] = min(bounds['min_y'], pos_y - 10)
                    bounds['max_y'] = max(bounds['max_y'], pos_y + 10)
                    bounds['min_z'] = min(bounds['min_z'], pos_z - 10)
                    bounds['max_z'] = max(bounds['max_z'], pos_z + 10)
        
        return bounds
    
    def reload_data(self) -> ExportData:
        """重新加载数据
        
        Returns:
            ExportData: 重新加载的导出数据
        """
        self._export_data = None
        self._stl_files = []
        return self.load_export_data()
    
    def _validate_export_data(self):
        """验证导出数据的完整性"""
        if not self._export_data:
            raise ValueError("导出数据为空")
        
        if not self._export_data.meta:
            raise ValueError("缺少导出元数据")
        
        if not self._export_data.components:
            self.logger.warning("未找到零部件数据")
        
        # 验证零部件数据
        for component in self._export_data.components:
            if not component.name:
                raise ValueError("零部件名称为空")
            if not component.component_id:
                raise ValueError(f"零部件 {component.name} 缺少ID")
        
        # 验证关节数据
        for joint in self._export_data.joints:
            if not joint.name:
                raise ValueError("关节名称为空")
            if not joint.joint_type:
                raise ValueError(f"关节 {joint.name} 缺少类型")
        
        self.logger.info("数据验证通过")
    
    def get_data_summary(self) -> Dict[str, Any]:
        """获取数据摘要
        
        Returns:
            Dict[str, Any]: 数据摘要信息
        """
        if self._export_data is None:
            self.load_export_data()
        
        summary = {
            'export_dir': str(self.export_dir),
            'component_count': len(self._export_data.components),
            'joint_count': len(self._export_data.joints),
            'stl_file_count': len(self.load_stl_files()),
            'export_time': self._export_data.meta.export_time,
            'geometry_unit': self._export_data.meta.geometry_unit,
            'position_unit': self._export_data.meta.position_unit,
            'components_with_stl': sum(1 for c in self._export_data.components if c.stl_file),
            'bounds': self.get_scene_bounds()
        }
        
        return summary


def create_data_loader(export_dir: str, logger: Optional[logging.Logger] = None) -> ExportDataLoader:
    """创建数据加载器实例
    
    Args:
        export_dir: 导出目录路径
        logger: 可选的日志记录器
        
    Returns:
        ExportDataLoader: 数据加载器实例
    """
    return ExportDataLoader(export_dir, logger)