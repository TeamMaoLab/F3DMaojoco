"""
单位转换阶段

将 Fusion 360 的毫米单位转换为 MuJoCo 的米单位。
"""

from typing import Dict, Any, List, Optional

from ..utils.logger import logger
from ..context import MaojocoContext
from .base import ConversionPhase
from ..type_definitions import (
    Body4DCoordinates, JointGlobalCoordinates, KinematicTree, ConvertedData,
    BodyName, JointName, Body4DCoordinatesDict, JointGlobalCoordinatesDict
)
from F3DMaojocoScripts.common.geometry_math import Vector3D


class UnitConversionPhase(ConversionPhase):
    """单位转换阶段"""
    
    def __init__(self, ctx: MaojocoContext):
        super().__init__("UnitConversion", ctx)
    
    def _execute(self) -> bool:
        """执行单位转换"""
        logger.info("📐 开始单位转换")
        
        try:
            # 初始化转换数据结构
            self.ctx.converted_data = ConvertedData(
                body_coordinates={},
                joint_coordinates={},
                kinematic_tree=None,
                conversion_info={
                    'source_units': 'mm',
                    'target_units': 'm',
                    'conversion_factor': 0.001,
                    'conversion_timestamp': None  # 可以添加时间戳
                }
            )
            
            # 转换 Body 4D 坐标从毫米到米
            self._convert_body_coordinates()
            
            # 转换关节坐标从毫米到米
            self._convert_joint_coordinates()
            
            # 转换运动学树中的变换
            self._convert_kinematic_tree()
            
            logger.info("🔄 转换 Fusion 360 (mm) 到 MuJoCo (m)")
            logger.info(f"📊 转换结果:")
            logger.info(f"    - Body 坐标: {len(self.ctx.converted_data.body_coordinates)} 个")
            logger.info(f"    - 关节坐标: {len(self.ctx.converted_data.joint_coordinates)} 个")
            if self.ctx.converted_data.kinematic_tree:
                tree = self.ctx.converted_data.kinematic_tree
                logger.info(f"    - 运动学树: {len(tree.get('bodies', {}))} 个刚体")
            
            return True
            
        except Exception as e:
            logger.error(f"❌ 单位转换失败: {e}")
            import traceback
            logger.error(traceback.format_exc())
            return False
    
    def _convert_body_coordinates(self):
        """转换 Body 坐标单位"""
        logger.info("📐 转换 Body 坐标单位")
        
        converted_bodies: Body4DCoordinatesDict = {}
        
        for body_name, body_4d in self.ctx.body_4d_coordinates.items():
            # 转换变换矩阵（使用内置的单位转换方法）
            converted_transform = body_4d.transform.to_mujoco_units()
            
            # 创建转换后的 Body 数据
            converted_body = Body4DCoordinates(
                name=body_4d.name,
                occurrence_name=body_4d.occurrence_name,
                full_path_name=body_4d.full_path_name,
                component_id=body_4d.component_id,
                transform=converted_transform,
                stl_file=body_4d.stl_file,
                bodies_count=body_4d.bodies_count,
                has_children=body_4d.has_children
            )
            
            converted_bodies[body_name] = converted_body
            
            # 记录转换前后的对比（用于验证）
            original_pos = body_4d.transform.get_translation()
            converted_pos = converted_transform.get_translation()
            logger.debug(f"   📏 {body_name}: ({original_pos.x:.3f}, {original_pos.y:.3f}, {original_pos.z:.3f}) mm → ({converted_pos.x:.6f}, {converted_pos.y:.6f}, {converted_pos.z:.6f}) m")
        
        # 更新转换数据
        self.ctx.converted_data.body_coordinates = converted_bodies
        
        logger.info(f"📊 转换了 {len(converted_bodies)} 个 Body 坐标")
    
    def _convert_joint_coordinates(self):
        """转换关节坐标单位"""
        logger.info("📐 转换关节坐标单位")
        
        converted_joints: JointGlobalCoordinatesDict = {}
        
        for joint_name, joint_coord in self.ctx.joint_global_coordinates.items():
            # 转换位置从毫米到米
            converted_position = joint_coord.position.to_mujoco_units()
            
            # 四元数不需要单位转换（它是无量纲的）
            converted_quaternion = joint_coord.quaternion
            
            # 创建转换后的关节坐标
            converted_joint_coord = JointGlobalCoordinates(
                position=converted_position,
                quaternion=converted_quaternion,
                joint_name=joint_name,
                joint_type=joint_coord.joint_type
            )
            
            converted_joints[joint_name] = converted_joint_coord
            
            # 记录转换前后的对比（用于验证）
            original_pos = joint_coord.position
            logger.debug(f"   📏 {joint_name}: ({original_pos.x:.3f}, {original_pos.y:.3f}, {original_pos.z:.3f}) mm → ({converted_position.x:.6f}, {converted_position.y:.6f}, {converted_position.z:.6f}) m")
        
        # 更新转换数据
        self.ctx.converted_data.joint_coordinates = converted_joints
        
        logger.info(f"📊 转换了 {len(converted_joints)} 个关节坐标")
    
    def _convert_kinematic_tree(self):
        """转换运动学树单位"""
        logger.info("📐 转换运动学树单位")
        
        if not self.ctx.kinematic_tree:
            logger.warning("⚠️  运动学树不存在，跳过转换")
            return
        
        # 深拷贝运动学树以避免修改原始数据
        import copy
        converted_tree = copy.deepcopy(self.ctx.kinematic_tree)
        
        # 转换刚体数据
        for body_id, body_data in converted_tree['bodies'].items():
            # 转换世界变换
            converted_world_transform = body_data.world_transform.to_mujoco_units()
            body_data.world_transform = converted_world_transform
        
        # 转换关节位置
        for joint_id, joint_data in converted_tree['joints'].items():
            if joint_data.position:
                joint_data.position = joint_data.position.to_mujoco_units()
        
        # 转换相对变换
        for body_id, rel_transform in converted_tree['relative_transforms'].items():
            converted_transform = rel_transform.transform.to_mujoco_units()
            rel_transform.transform = converted_transform
        
        # 存储转换后的运动学树
        self.ctx.converted_data.kinematic_tree = converted_tree
        
        logger.info(f"📊 运动学树单位转换完成:")
        logger.info(f"    - 刚体: {len(converted_tree['bodies'])} 个")
        logger.info(f"    - 关节: {len(converted_tree['joints'])} 个")
        logger.info(f"    - 相对变换: {len(converted_tree['relative_transforms'])} 个")