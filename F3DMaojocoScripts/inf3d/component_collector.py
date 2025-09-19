"""
零部件收集器

遍历装配体，收集所有零部件信息。
递归遍历装配树结构，提取零部件的几何信息和变换矩阵。
"""

from typing import List, Optional, Dict, Any

import adsk.core
import adsk.fusion

from .logger import log_component, log_transform
from ..common.data_types import ComponentInfo
from ..common.geometry_math import Transform4D


class ComponentCollector:
    """零部件收集器
    
    遍历装配体，只收集叶子零部件信息：
    - 递归遍历装配树结构，只处理叶子节点
    - 过滤掉不可见的零部件和occurrence
    - 提取零部件的几何信息、变换矩阵
    - 统计实体数量
    - 生成 ComponentInfo 对象
    """
    
    def __init__(self, logger):
        """初始化零部件收集器
        
        Args:
            logger: 日志记录器
        """
        self.logger = logger
        self.app = adsk.core.Application.get()
        self.design = self.app.activeProduct
        
        # 用于跟踪已处理的零部件，避免重复处理
        self._processed_components = set()
        
        # 装配树路径缓存
        self._component_paths = {}
    
    def collect_components(self) -> List[ComponentInfo]:
        """收集所有零部件信息
        
        Returns:
            List[ComponentInfo]: 零部件信息列表
        """
        self.logger.info("开始收集零部件信息")
        
        if not self.design:
            raise ValueError("没有活动的 Fusion 360 设计")
        
        # 获取根组件
        root_component = self.design.rootComponent
        if not root_component:
            raise ValueError("无法获取根组件")
        
        # 清空处理记录
        self._processed_components.clear()
        self._component_paths.clear()
        
        # 收集所有零部件
        components = []
        
        # 从根组件开始递归收集
        self._collect_from_component(root_component, components, "")
        
        self.logger.info(f"零部件收集完成，共收集 {len(components)} 个零部件")
        return components
    
    def _collect_from_component(self, component: adsk.fusion.Component, 
                               components: List[ComponentInfo], 
                               parent_path: str):
        """从指定组件递归收集零部件
        
        Args:
            component: Fusion 360 组件
            components: 零部件列表
            parent_path: 父级路径
        """
        try:
            # 获取组件的唯一标识
            component_id = id(component)
            
            # 如果已经处理过，跳过
            if component_id in self._processed_components:
                return
            
            self._processed_components.add(component_id)
            
            # 构建完整路径
            current_path = f"{parent_path}/{component.name}" if parent_path else component.name
            self._component_paths[component_id] = current_path
            
            # 记录当前处理的组件
            log_component(self.logger, component.name, "收集")
            
            # 只处理当前组件的所有 occurrence，不重复处理子组件
            # 子组件会通过 occurrence 的层级结构自动处理
            for occurrence in component.occurrences:
                self._collect_from_occurrence(occurrence, components, current_path)
            
        except Exception as e:
            self.logger.error(f"收集组件 {component.name} 时发生错误: {str(e)}")
            # 不要因为一个组件失败而中断整个收集过程
    
    def _collect_from_occurrence(self, occurrence: adsk.fusion.Occurrence,
                                components: List[ComponentInfo],
                                parent_path: str):
        """从 occurrence 收集零部件信息
        
        Args:
            occurrence: Fusion 360 occurrence
            components: 零部件列表
            parent_path: 父级路径
        """
        try:
            if not occurrence.component:
                return
            
            # 检查 occurrence 是否可见
            if not self._is_occurrence_visible(occurrence):
                self.logger.debug(f"跳过不可见的 occurrence: {occurrence.name}")
                return
            
            # 构建完整路径
            full_path = f"{parent_path}/{occurrence.name}"
            
            # 检查是否为叶子节点（没有子 occurrence）
            is_leaf = len(occurrence.childOccurrences) == 0
            
            if is_leaf:
                # 只有叶子节点才创建零部件信息
                component_info = self._create_component_info(occurrence, full_path)
                
                if component_info:
                    components.append(component_info)
                    log_component(self.logger, component_info.name, "创建叶子节点")
            else:
                # 非叶子节点，只递归处理子节点
                self.logger.debug(f"跳过非叶子节点 {occurrence.name}，有 {len(occurrence.childOccurrences)} 个子节点")
            
            # 递归处理子 occurrence
            for child_occurrence in occurrence.childOccurrences:
                self._collect_from_occurrence(child_occurrence, components, full_path)
            
        except Exception as e:
            self.logger.error(f"收集 occurrence {occurrence.name} 时发生错误: {str(e)}")
    
    def _create_component_info(self, occurrence: adsk.fusion.Occurrence,
                               full_path: str) -> Optional[ComponentInfo]:
        """创建零部件信息对象
        
        Args:
            occurrence: Fusion 360 occurrence
            full_path: 完整路径
            
        Returns:
            Optional[ComponentInfo]: 零部件信息对象
        """
        try:
            component = occurrence.component
            if not component:
                return None
            
            # 获取世界坐标系变换矩阵
            world_transform = self._get_world_transform(occurrence)
            
            # 将Fusion 360 API的厘米单位转换为毫米单位
            if world_transform:
                world_transform = self._convert_cm_to_mm(world_transform)
            
            # 统计实体数量
            bodies_count = len(component.bRepBodies)
            
            # 创建零部件信息（叶子节点，所以 has_children 总是 False）
            component_info = ComponentInfo(
                name=component.name,
                occurrence_name=occurrence.name,
                full_path_name=full_path,
                component_id=id(component),  # 使用组件的唯一ID作为整数
                bodies_count=bodies_count,
                has_children=False,  # 叶子节点没有子零部件
                world_transform=world_transform
            )
            
            # 记录变换信息（调试级别）
            if world_transform:
                log_transform(self.logger, f"{component.name}_world", world_transform)
            
            # 调试：记录装配体层次结构信息
            self.logger.debug(f"{component.name}: 装配体路径 = {full_path}")
            self.logger.debug(f"{component.name}: 有子零部件 = False")
            self.logger.debug(f"{component.name}: 实体数量 = {bodies_count}")
            
            return component_info
            
        except Exception as e:
            self.logger.error(f"创建零部件信息 {occurrence.name} 时发生错误: {str(e)}")
            return None
    
    def _get_world_transform(self, occurrence: adsk.fusion.Occurrence) -> Optional[Transform4D]:
        """获取世界坐标系变换矩阵
        
        计算 occurrence 相对于世界坐标系的完整变换。
        如果 occurrence 在装配体中，需要累加所有父级变换。
        
        Args:
            occurrence: Fusion 360 occurrence
            
        Returns:
            Optional[Transform4D]: 世界坐标系变换矩阵
        """
        try:
            # 对于世界变换，如果 occurrence 有父装配体，需要计算变换链
            # 但首先让我们尝试直接使用 occurrence.transform，看看是否已经包含了正确的世界变换
            
            # 直接使用 occurrence.transform
            transform = occurrence.transform
            if not transform:
                return None
            
            # 转换为 4x4 矩阵
            matrix_data = transform.asArray()
            
            # 重新组织为 4x4 格式
            matrix_4x4 = [
                [matrix_data[0], matrix_data[1], matrix_data[2], matrix_data[3]],
                [matrix_data[4], matrix_data[5], matrix_data[6], matrix_data[7]],
                [matrix_data[8], matrix_data[9], matrix_data[10], matrix_data[11]],
                [0.0, 0.0, 0.0, 1.0]
            ]
            
            return Transform4D(matrix_4x4)
            
        except Exception as e:
            self.logger.error(f"获取世界变换矩阵时发生错误: {str(e)}")
            return None
    
        
    def _convert_cm_to_mm(self, transform: Transform4D) -> Transform4D:
        """将厘米单位的变换矩阵转换为毫米单位
        
        Args:
            transform: 厘米单位的变换矩阵
            
        Returns:
            Transform4D: 毫米单位的变换矩阵
        """
        try:
            # 创建新的矩阵，只缩放位置分量（第4列）
            scaled_matrix = []
            for i, row in enumerate(transform.matrix):
                if i < 3:  # 前3行
                    scaled_row = row.copy()
                    scaled_row[3] = row[3] * 10.0  # 厘米转毫米
                    scaled_matrix.append(scaled_row)
                else:  # 最后一行
                    scaled_matrix.append(row.copy())
            
            return Transform4D(scaled_matrix)
            
        except Exception as e:
            self.logger.error(f"厘米转毫米转换时发生错误: {str(e)}")
            return transform
    
    def _is_occurrence_visible(self, occurrence: adsk.fusion.Occurrence) -> bool:
        """检查 occurrence 是否可见
        
        Args:
            occurrence: Fusion 360 occurrence
            
        Returns:
            bool: 如果可见返回 True，否则返回 False
        """
        try:
            # 检查 occurrence 本身的可见性
            if not occurrence.isVisible:
                return False
            
            # 检查 occurrence 的组件是否可见
            if not occurrence.component.isLightBulbOn:
                return False
            
            # 检查父级 occurrence 的可见性（如果存在）
            if occurrence.parentOccurrence:
                if not self._is_occurrence_visible(occurrence.parentOccurrence):
                    return False
            
            return True
            
        except Exception as e:
            self.logger.error(f"检查可见性时发生错误: {str(e)}")
            # 默认返回 True 以避免误过滤
            return True