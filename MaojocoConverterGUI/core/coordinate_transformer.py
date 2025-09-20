"""
坐标变换器

负责处理4x4变换矩阵的计算和应用。
"""

from typing import List, Optional, Tuple
import numpy as np
from dataclasses import dataclass

from utils.logger import logger


@dataclass
class Transform4D:
    """4x4变换矩阵"""
    matrix: List[List[float]]
    
    def __post_init__(self):
        """验证矩阵维度"""
        if len(self.matrix) != 4 or any(len(row) != 4 for row in self.matrix):
            raise ValueError("Transform4D requires a 4x4 matrix")
    
    def transform_points(self, points: np.ndarray) -> np.ndarray:
        """变换点云数组
        
        Args:
            points: Nx3的numpy点云数组
            
        Returns:
            np.ndarray: 变换后的Nx3数组
        """
        # 转换为齐次坐标
        ones = np.ones((points.shape[0], 1))
        homogeneous_points = np.hstack([points, ones])
        
        # 应用变换
        matrix = np.array(self.matrix)
        transformed = homogeneous_points @ matrix.T
        
        # 返回笛卡尔坐标
        return transformed[:, :3]
    
    def get_translation(self) -> Tuple[float, float, float]:
        """获取平移部分"""
        return (self.matrix[0][3], self.matrix[1][3], self.matrix[2][3])
    
    @classmethod
    def from_matrix(cls, matrix: List[List[float]]) -> "Transform4D":
        """从矩阵创建变换"""
        return cls(matrix)
    
    @classmethod
    def identity(cls) -> "Transform4D":
        """单位矩阵"""
        return cls([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])


class CoordinateTransformer:
    """坐标变换器
    
    提供4x4变换矩阵的计算和应用功能。
    """
    
    def __init__(self) -> None:
        """初始化坐标变换器"""
        self._identity_transform = Transform4D.identity()
    
    def create_transform(self, matrix_data: List[List[float]]) -> Optional[Transform4D]:
        """创建变换矩阵
        
        Args:
            matrix_data: 4x4矩阵数据
            
        Returns:
            Transform4D: 变换矩阵对象，如果数据无效则返回None
        """
        try:
            return Transform4D(matrix_data)
        except ValueError as e:
            logger.error(f"创建变换矩阵失败: {e}")
            return None
    
    def apply_transform_to_points(self, points: np.ndarray, transform: Transform4D) -> np.ndarray:
        """应用变换到点云
        
        Args:
            points: Nx3点云数组
            transform: 变换矩阵
            
        Returns:
            np.ndarray: 变换后的点云
        """
        return transform.transform_points(points)
    
    def get_translation_vector(self, transform: Transform4D) -> Tuple[float, float, float]:
        """获取平移向量
        
        Args:
            transform: 变换矩阵
            
        Returns:
            Tuple[float, float, float]: 平移向量 (x, y, z)
        """
        return transform.get_translation()
    
    def is_identity_transform(self, transform: Transform4D, tolerance: float = 1e-6) -> bool:
        """检查是否是单位变换
        
        Args:
            transform: 变换矩阵
            tolerance: 容差
            
        Returns:
            bool: 是否是单位变换
        """
        identity = self._identity_transform.matrix
        for i in range(4):
            for j in range(4):
                if abs(transform.matrix[i][j] - identity[i][j]) > tolerance:
                    return False
        return True
    
    def create_transform_from_components(self, components_data: List[List[float]]) -> Optional[Transform4D]:
        """从组件数据创建变换矩阵
        
        Args:
            components_data: 组件数据中的变换矩阵
            
        Returns:
            Transform4D: 变换矩阵对象
        """
        if not components_data or len(components_data) != 4:
            logger.error("变换矩阵数据无效：需要4x4矩阵")
            return None
        
        # 检查每行是否都有4个元素
        for i, row in enumerate(components_data):
            if len(row) != 4:
                logger.error(f"变换矩阵第{i}行数据无效：需要4个元素，实际{len(row)}个")
                return None
        
        return self.create_transform(components_data)
    
    def batch_transform_points(self, points_list: List[np.ndarray], 
                             transforms: List[Transform4D]) -> List[np.ndarray]:
        """批量变换点云
        
        Args:
            points_list: 点云列表
            transforms: 对应的变换矩阵列表
            
        Returns:
            List[np.ndarray]: 变换后的点云列表
        """
        if len(points_list) != len(transforms):
            logger.error("点云数量与变换矩阵数量不匹配")
            return points_list.copy()
        
        result = []
        for points, transform in zip(points_list, transforms):
            transformed_points = self.apply_transform_to_points(points, transform)
            result.append(transformed_points)
        
        return result
    
    def get_transform_statistics(self, transforms: List[Transform4D]) -> dict:
        """获取变换矩阵统计信息
        
        Args:
            transforms: 变换矩阵列表
            
        Returns:
            dict: 统计信息
        """
        if not transforms:
            return {}
        
        translations = [self.get_translation_vector(t) for t in transforms]
        
        # 计算平移范围
        x_coords = [t[0] for t in translations]
        y_coords = [t[1] for t in translations]
        z_coords = [t[2] for t in translations]
        
        stats = {
            "count": len(transforms),
            "translation_range": {
                "x": {"min": min(x_coords), "max": max(x_coords)},
                "y": {"min": min(y_coords), "max": max(y_coords)},
                "z": {"min": min(z_coords), "max": max(z_coords)}
            },
            "identity_count": sum(1 for t in transforms if self.is_identity_transform(t))
        }
        
        return stats