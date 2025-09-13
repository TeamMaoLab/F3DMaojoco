"""
关节分析器

分析装配体中的关节约束关系。
识别和分类关节类型，提取关节连接的零部件信息。
"""

from typing import List, Optional

import adsk.core
import adsk.fusion

from .logger import log_joint, log_transform
from ..common.data_types import JointInfo, JointType, JointConnection, JointGeometry, ComponentInfo
from ..common.geometry_math import Transform4D
from .joint_limits_extractor import FusionJointLimitsExtractor


class JointAnalyzer:
    """关节分析器
    
    分析装配体中的关节约束关系：
    - 识别和分类关节类型（RIGID、REVOLUTE、SLIDER等）
    - 提取关节连接的零部件信息
    - 获取关节几何变换矩阵
    - 生成 JointInfo 对象
    """
    
    def __init__(self, logger):
        """初始化关节分析器
        
        Args:
            logger: 日志记录器
        """
        self.logger = logger
        self.app = adsk.core.Application.get()
        self.design = self.app.activeProduct
        
        # 关节类型映射
        self._joint_type_mapping = {
            adsk.fusion.JointTypes.RigidJointType: JointType.RIGID,
            adsk.fusion.JointTypes.RevoluteJointType: JointType.REVOLUTE,
            adsk.fusion.JointTypes.SliderJointType: JointType.SLIDER,
            adsk.fusion.JointTypes.CylindricalJointType: JointType.CYLINDRICAL,
            adsk.fusion.JointTypes.PinSlotJointType: JointType.PIN_SLOT,
            adsk.fusion.JointTypes.PlanarJointType: JointType.PLANAR,
            adsk.fusion.JointTypes.BallJointType: JointType.BALL
        }
        
        # 创建关节限制提取器
        self.limits_extractor = FusionJointLimitsExtractor(logger)
    
    def analyze_joints(self) -> List[JointInfo]:
        """分析所有关节
        
        Returns:
            List[JointInfo]: 关节信息列表
        """
        self.logger.info("开始分析关节信息")
        
        if not self.design:
            raise ValueError("没有活动的 Fusion 360 设计")
        
        # 获取根组件
        root_component = self.design.rootComponent
        if not root_component:
            raise ValueError("无法获取根组件")
        
        # 收集所有关节
        joints = []
        
        # 从根组件开始递归收集
        self._collect_joints_from_component(root_component, joints)
        
        self.logger.info(f"关节分析完成，共分析 {len(joints)} 个关节")
        return joints
    
    def _collect_joints_from_component(self, component: adsk.fusion.Component, 
                                      joints: List[JointInfo]):
        """从指定组件收集关节
        
        Args:
            component: Fusion 360 组件
            joints: 关节列表
        """
        try:
            # 处理当前组件的关节
            for joint in component.joints:
                joint_info = self._create_joint_info(joint)
                if joint_info:
                    joints.append(joint_info)
                    log_joint(self.logger, joint_info.name, joint_info.joint_type.value)
            
            # 递归处理子组件
            for occurrence in component.allOccurrences:
                if occurrence.component and occurrence.component != component:
                    self._collect_joints_from_component(occurrence.component, joints)
                    
        except Exception as e:
            self.logger.error(f"收集组件 {component.name} 的关节时发生错误: {str(e)}")
    
    def _create_joint_info(self, joint: adsk.fusion.Joint) -> Optional[JointInfo]:
        """创建关节信息对象
        
        Args:
            joint: Fusion 360 关节
            
        Returns:
            Optional[JointInfo]: 关节信息对象
        """
        try:
            # 获取关节类型
            joint_type = self._map_joint_type(joint.jointMotion.jointType)
            
            # 获取关节连接信息
            connection = self._get_joint_connection(joint)
            
            # 获取关节几何信息
            geometry = self._get_joint_geometry(joint)
            
            # 提取关节限制信息（仅支持旋转关节和球关节）
            limits = None
            if joint_type in [JointType.REVOLUTE, JointType.BALL]:
                limits = self.limits_extractor.extract_joint_limits(joint, joint_type)
                if limits:
                    # 验证限制信息
                    if self.limits_extractor.validate_limits(limits):
                        self.logger.info(f"成功提取关节 {joint.name} 的限制信息")
                        summary = self.limits_extractor.get_limits_summary(limits)
                        self.logger.debug(f"限制摘要: {summary}")
                    else:
                        self.logger.warning(f"关节 {joint.name} 的限制信息验证失败")
                        limits = None
                else:
                    self.logger.warning(f"未能提取关节 {joint.name} 的限制信息")
            
            # 创建关节信息
            joint_info = JointInfo(
                name=joint.name,
                joint_type=joint_type,
                connection=connection,
                geometry=geometry,
                is_suppressed=joint.isSuppressed,
                is_light_bulb_on=joint.isLightBulbOn,
                limits=limits
            )
            
            return joint_info
            
        except Exception as e:
            self.logger.error(f"创建关节信息 {joint.name} 时发生错误: {str(e)}")
            return None
    
    def _map_joint_type(self, fusion_joint_type: adsk.fusion.JointTypes) -> JointType:
        """映射 Fusion 360 关节类型到内部类型
        
        Args:
            fusion_joint_type: Fusion 360 关节类型
            
        Returns:
            JointType: 内部关节类型
        """
        return self._joint_type_mapping.get(fusion_joint_type, JointType.INFERRED)
    
    def _get_joint_connection(self, joint: adsk.fusion.Joint) -> JointConnection:
        """获取关节连接信息
        
        Args:
            joint: Fusion 360 关节
            
        Returns:
            JointConnection: 关节连接信息
        """
        try:
            # 获取几何实体
            geo_one = joint.geometryOrOriginOne
            geo_two = joint.geometryOrOriginTwo
            
            # 添加调试信息
            self.logger.debug(f"处理关节: {joint.name}")
            self.logger.debug(f"geo_one 类型: {type(geo_one)}, geo_two 类型: {type(geo_two)}")
            
            # 获取连接的 occurrence
            occurrence_one = None
            occurrence_two = None
            
            if geo_one:
                occurrence_one = self._find_occurrence_for_geometry(geo_one)
                self.logger.debug(f"geo_one 找到 occurrence: {occurrence_one.name if occurrence_one else 'None'}")
            if geo_two:
                occurrence_two = self._find_occurrence_for_geometry(geo_two)
                self.logger.debug(f"geo_two 找到 occurrence: {occurrence_two.name if occurrence_two else 'None'}")
            
            # 如果还是找不到，尝试通过关节的父级关系获取
            if not occurrence_one or not occurrence_two:
                self.logger.debug("尝试通过父级关系获取连接信息")
                # 方法：检查关节的父组件
                try:
                    parent_occurrence = joint.parentOccurrence
                    if parent_occurrence:
                        self.logger.debug(f"关节父级 occurrence: {parent_occurrence.name}")
                        # 这里可以添加更复杂的逻辑来找出关节连接的具体组件
                except AttributeError:
                    pass
            
            # 创建连接信息
            connection = JointConnection(
                occurrence_one_name=occurrence_one.name if occurrence_one else None,
                occurrence_one_full_path=self._get_occurrence_path(occurrence_one) if occurrence_one else None,
                occurrence_one_component=occurrence_one.component.name if occurrence_one and occurrence_one.component else None,
                occurrence_two_name=occurrence_two.name if occurrence_two else None,
                occurrence_two_full_path=self._get_occurrence_path(occurrence_two) if occurrence_two else None,
                occurrence_two_component=occurrence_two.component.name if occurrence_two and occurrence_two.component else None
            )
            
            # 记录连接信息
            self.logger.debug(f"关节 {joint.name} 连接信息: {connection}")
            
            return connection
            
        except Exception as e:
            self.logger.error(f"获取关节连接信息时发生错误: {str(e)}")
            return JointConnection()
    
    def _get_joint_geometry(self, joint: adsk.fusion.Joint) -> JointGeometry:
        """获取关节几何信息
        
        Args:
            joint: Fusion 360 关节
            
        Returns:
            JointGeometry: 关节几何信息
        """
        try:
            # 获取几何变换
            transform_one = None
            transform_two = None
            
            # 尝试获取第一个几何体的变换
            if joint.geometryOrOriginOne:
                transform_one = self._get_geometry_transform(joint.geometryOrOriginOne)
            
            # 尝试获取第二个几何体的变换
            if joint.geometryOrOriginTwo:
                transform_two = self._get_geometry_transform(joint.geometryOrOriginTwo)
            
            # 创建几何信息
            geometry = JointGeometry(
                geometry_one_transform=transform_one,
                geometry_two_transform=transform_two
            )
            
            return geometry
            
        except Exception as e:
            self.logger.error(f"获取关节几何信息时发生错误: {str(e)}")
            return JointGeometry()
    
    def _find_occurrence_for_geometry(self, geometry) -> Optional[adsk.fusion.Occurrence]:
        """为几何实体查找对应的 occurrence
        
        Args:
            geometry: 几何实体
            
        Returns:
            Optional[adsk.fusion.Occurrence]: 对应的 occurrence
        """
        try:
            # 尝试通过不同的方式获取组件信息
            
            # 方法1: 检查几何实体是否有 entityOne 属性 (JointGeometry)
            entity_one = None
            entity_two = None
            
            # 通过直接属性访问获取实体
            try:
                entity_one = geometry.entityOne
                entity_two = geometry.entityTwo
            except AttributeError:
                pass
            
            # 如果有实体，尝试获取组件
            if entity_one:
                component = self._get_component_from_entity(entity_one)
                if component:
                    return self._find_occurrence_for_component(component)
            
            if entity_two:
                component = self._get_component_from_entity(entity_two)
                if component:
                    return self._find_occurrence_for_component(component)
            
            # 方法2: 检查是否可以直接从几何体获取组件
            try:
                # 对于某些几何类型，可能有直接的组件引用
                body = geometry.body
                if body:
                    component = body.parentComponent
                    if component:
                        return self._find_occurrence_for_component(component)
            except AttributeError:
                pass
            
            # 如果所有方法都失败，返回 None
            return None
            
        except Exception as e:
            self.logger.error(f"查找几何实体对应的 occurrence 时发生错误: {str(e)}")
            return None
    
    def _get_component_from_entity(self, entity) -> Optional[adsk.fusion.Component]:
        """从几何实体获取对应的组件
        
        Args:
            entity: 几何实体
            
        Returns:
            Optional[adsk.fusion.Component]: 对应的组件
        """
        try:
            # 检查实体类型并获取组件
            
            # 尝试获取 body
            body = None
            try:
                body = entity.body
            except AttributeError:
                pass
            
            if body:
                # 从 body 获取组件
                try:
                    return body.parentComponent
                except AttributeError:
                    pass
            
            # 尝试直接获取组件
            try:
                return entity.parentComponent
            except AttributeError:
                pass
            
            # 检查是否是组件本身
            try:
                # 如果是组件，应该有 name 属性
                name = entity.name
                # 尝试作为组件处理
                return entity
            except AttributeError:
                pass
            
            # 检查是否是 occurrence
            try:
                # 如果是 occurrence，应该有 component 属性
                component = entity.component
                return component
            except AttributeError:
                pass
            
            return None
            
        except Exception as e:
            self.logger.error(f"从几何实体获取组件时发生错误: {str(e)}")
            return None
    
    def _find_occurrence_for_component(self, component: adsk.fusion.Component) -> Optional[adsk.fusion.Occurrence]:
        """为组件查找对应的 occurrence
        
        Args:
            component: Fusion 360 组件
            
        Returns:
            Optional[adsk.fusion.Occurrence]: 对应的 occurrence
        """
        try:
            # 在根组件中查找包含这个组件的 occurrence
            root_component = self.design.rootComponent
            
            # 遍历所有 occurrences
            for occurrence in root_component.allOccurrences:
                if occurrence.component == component:
                    return occurrence
            
            # 如果没有找到，返回 None
            return None
            
        except Exception as e:
            self.logger.error(f"查找组件对应的 occurrence 时发生错误: {str(e)}")
            return None
    
    def _get_occurrence_path(self, occurrence: adsk.fusion.Occurrence) -> Optional[str]:
        """获取 occurrence 的完整路径
        
        Args:
            occurrence: Fusion 360 occurrence
            
        Returns:
            Optional[str]: 完整路径
        """
        try:
            if not occurrence:
                return None
            
            # 如果 occurrence 没有 parentOccurrence 属性，直接返回 occurrence 名称
            # 这是在顶层组件中的 occurrence
            try:
                # 尝试检查是否有父级
                parent = occurrence.parentOccurrence
                if parent is None:
                    # 没有父级，直接返回名称
                    return occurrence.name
                else:
                    # 有父级，构建完整路径
                    path_parts = []
                    current = occurrence
                    
                    while current:
                        path_parts.append(current.name)
                        # 检查是否有父级 occurrence
                        try:
                            parent = current.parentOccurrence
                            if parent:
                                current = parent
                            else:
                                break
                        except AttributeError:
                            break
                    
                    # 反转路径并连接
                    path_parts.reverse()
                    return "/".join(path_parts)
            except AttributeError:
                # 没有 parentOccurrence 属性，直接返回名称
                return occurrence.name
            
        except Exception as e:
            self.logger.error(f"获取 occurrence 路径时发生错误: {str(e)}")
            return None
    
    def _get_geometry_transform(self, geometry) -> Optional[Transform4D]:
        """获取几何实体的变换矩阵
        
        Args:
            geometry: 几何实体
            
        Returns:
            Optional[Transform4D]: 变换矩阵（已转换为毫米单位）
        """
        try:
            # 添加调试信息
            self.logger.debug(f"获取几何实体变换矩阵，类型: {type(geometry)}")
            
            # 对于 JointGeometry，使用其内置的坐标系信息构建变换矩阵
            try:
                # 尝试获取原点和轴向信息
                origin = geometry.origin
                primary_axis = geometry.primaryAxisVector
                secondary_axis = geometry.secondaryAxisVector
                third_axis = geometry.thirdAxisVector
                
                if origin and primary_axis and secondary_axis and third_axis:
                    # 使用 JointGeometry 的坐标系信息构建变换矩阵
                    # 注意：Fusion 360 的坐标系可能与我们的期望不同
                    # 这里我们构建一个从原点和轴向导出的变换矩阵
                    
                    # 获取原点坐标（从厘米转换为毫米）
                    origin_x = origin.x * 10.0  # cm to mm
                    origin_y = origin.y * 10.0  # cm to mm
                    origin_z = origin.z * 10.0  # cm to mm
                    
                    # 获取轴向向量（旋转部分不需要单位转换）
                    px, py, pz = primary_axis.x, primary_axis.y, primary_axis.z
                    sx, sy, sz = secondary_axis.x, secondary_axis.y, secondary_axis.z
                    tx, ty, tz = third_axis.x, third_axis.y, third_axis.z
                    
                    # 构建 4x4 变换矩阵（平移部分已转换为毫米）
                    # 对于旋转关节，primaryAxisVector 是旋转轴，应该放在 Y 轴位置
                    # 矩阵格式: [secondary_axis, primary_axis, third_axis, origin]
                    matrix_4x4 = [
                        [sx, px, tx, origin_x],
                        [sy, py, ty, origin_y],
                        [sz, pz, tz, origin_z],
                        [0.0, 0.0, 0.0, 1.0]
                    ]
                    
                    self.logger.debug(f"从 JointGeometry 坐标系构建变换矩阵（已转换为毫米）: {matrix_4x4}")
                    return Transform4D(matrix_4x4)
                    
            except AttributeError:
                # 如果 JointGeometry 没有完整的坐标系信息，继续尝试其他方法
                pass
            
            # 尝试直接获取变换矩阵（适用于其他几何类型）
            try:
                transform = geometry.transform
                if transform:
                    matrix_data = transform.asArray()
                    # 转换平移部分从厘米到毫米（索引 3, 7, 11）
                    matrix_4x4 = [
                        [matrix_data[0], matrix_data[1], matrix_data[2], matrix_data[3] * 10.0],
                        [matrix_data[4], matrix_data[5], matrix_data[6], matrix_data[7] * 10.0],
                        [matrix_data[8], matrix_data[9], matrix_data[10], matrix_data[11] * 10.0],
                        [0.0, 0.0, 0.0, 1.0]
                    ]
                    self.logger.debug(f"成功获取变换矩阵（已转换为毫米）: {matrix_4x4}")
                    return Transform4D(matrix_4x4)
                else:
                    self.logger.debug("几何实体没有变换矩阵")
            except AttributeError:
                self.logger.debug("几何实体没有 transform 属性")
            
            # 如果所有方法都失败，返回单位矩阵
            self.logger.debug("无法获取变换信息，返回单位矩阵")
            return Transform4D([
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0]
            ])
            
        except Exception as e:
            self.logger.error(f"获取几何实体变换矩阵时发生错误: {str(e)}")
            return None