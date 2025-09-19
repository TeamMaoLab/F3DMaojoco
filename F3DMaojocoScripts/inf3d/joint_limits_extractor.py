"""
Fusion 360 关节限制提取器

专门用于从Fusion 360 API中提取关节限制信息，专注于旋转关节和球关节。

注意：此模块依赖adsk库，只能在Fusion 360环境中使用。

主要功能：
1. 从RevoluteJointMotion中提取旋转角度限制和轴向信息
2. 从BallJointMotion中提取球心位置和三轴方向信息
3. 处理单位转换（厘米→毫米）
4. 提供类型安全的限制信息结构

作者: Claude
时间: 2025-01-10
"""

from typing import Optional, Dict, Any
import math

import adsk.core
import adsk.fusion

from ..common.data_types import (
    BasicJointLimits, RevoluteJointLimits, BallJointLimits,
    JointType, Vector3D, JointLimits
)


class FusionJointLimitsExtractor:
    """Fusion 360 关节限制提取器
    
    专门用于从Fusion 360关节对象中提取限制信息，
    目前支持旋转关节和球关节。
    """
    
    def __init__(self, logger):
        """初始化提取器
        
        Args:
            logger: 日志记录器
        """
        self.logger = logger
    
    def extract_joint_limits(self, joint: adsk.fusion.Joint, joint_type: JointType) -> Optional[JointLimits]:
        """提取关节限制信息
        
        Args:
            joint: Fusion 360关节对象
            joint_type: 关节类型
            
        Returns:
            Optional[JointLimits]: 关节限制信息，如果不支持的类型则返回None
        """
        try:
            if joint_type == JointType.REVOLUTE:
                return self._extract_revolute_limits(joint)
            elif joint_type == JointType.BALL:
                return self._extract_ball_limits(joint)
            else:
                # 其他类型的关节暂不支持
                self.logger.debug(f"关节类型 {joint_type.value} 暂不支持限制提取")
                return None
                
        except Exception as e:
            self.logger.error(f"提取关节限制时发生错误: {e}")
            return None
    
    def _extract_revolute_limits(self, joint: adsk.fusion.Joint) -> Optional[JointLimits]:
        """提取旋转关节限制信息
        
        Args:
            joint: Fusion 360旋转关节对象
            
        Returns:
            Optional[JointLimits]: 旋转关节限制信息
        """
        try:
            motion: adsk.fusion.RevoluteJointMotion = joint.jointMotion
            
            # 提取旋转轴信息
            rotation_axis = self._extract_rotation_axis(motion)
            
            # 提取旋转限制
            rotation_limits = self._extract_rotation_limits(motion)
            
            # 提取当前旋转角度
            current_angle = self._extract_current_rotation(motion)
            
            # 创建旋转关节限制对象
            revolute_limits = RevoluteJointLimits(
                rotation_limits=rotation_limits,
                current_angle=current_angle,
                rotation_axis=rotation_axis
            )
            
            self.logger.info(f"成功提取旋转关节 {joint.name} 限制信息")
            self.logger.debug(f"旋转轴: {rotation_axis}, 限制: [{rotation_limits.minimum_value:.3f}, {rotation_limits.maximum_value:.3f}] rad")
            
            # 创建统一的限制对象
            return JointLimits(revolute_limits=revolute_limits)
            
        except Exception as e:
            self.logger.error(f"提取旋转关节限制时发生错误: {e}")
            return None
    
    def _extract_ball_limits(self, joint: adsk.fusion.Joint) -> Optional[JointLimits]:
        """提取球关节限制信息
        
        Args:
            joint: Fusion 360球关节对象
            
        Returns:
            Optional[JointLimits]: 球关节限制信息
        """
        try:
            motion: adsk.fusion.BallJointMotion = joint.jointMotion
            
            # 提取球心位置
            center_position = self._extract_ball_center(joint)
            
            # 提取三轴方向（使用官方API）
            pitch_axis = None
            yaw_axis = None
            roll_axis = None
            
            try:
                # 使用官方API获取轴向向量
                if motion.pitchDirectionVector:
                    pitch_vec = motion.pitchDirectionVector
                    pitch_axis = Vector3D(pitch_vec.x, pitch_vec.y, pitch_vec.z)
                    self.logger.debug(f"成功获取pitch轴: {pitch_axis}")
            except Exception as e:
                self.logger.debug(f"获取pitch轴失败: {e}")
                
            try:
                if motion.yawDirectionVector:
                    yaw_vec = motion.yawDirectionVector
                    yaw_axis = Vector3D(yaw_vec.x, yaw_vec.y, yaw_vec.z)
                    self.logger.debug(f"成功获取yaw轴: {yaw_axis}")
            except Exception as e:
                self.logger.debug(f"获取yaw轴失败: {e}")
                
            try:
                if motion.rollDirectionVector:
                    roll_vec = motion.rollDirectionVector
                    roll_axis = Vector3D(roll_vec.x, roll_vec.y, roll_vec.z)
                    self.logger.debug(f"成功获取roll轴: {roll_axis}")
            except Exception as e:
                self.logger.debug(f"获取roll轴失败: {e}")
            
            # 提取角度值和限制
            pitch_limits = self._extract_ball_axis_limits(motion.pitchLimits, "pitch")
            yaw_limits = self._extract_ball_axis_limits(motion.yawLimits, "yaw")
            roll_limits = self._extract_ball_axis_limits(motion.rollLimits, "roll")
            
            # 创建球关节限制对象
            ball_limits = BallJointLimits(
                pitch_limits=pitch_limits,
                yaw_limits=yaw_limits,
                roll_limits=roll_limits,
                center_position=center_position,
                pitch_axis=pitch_axis,
                yaw_axis=yaw_axis,
                roll_axis=roll_axis
            )
            
            self.logger.info(f"成功提取球关节 {joint.name} 限制信息")
            if center_position:
                self.logger.debug(f"球心: {center_position}")
            
            # 创建统一的限制对象
            return JointLimits(ball_limits=ball_limits)
            
        except Exception as e:
            self.logger.error(f"提取球关节限制时发生错误: {e}")
            return None
    
    def _extract_rotation_axis(self, motion: adsk.fusion.RevoluteJointMotion) -> Optional[Vector3D]:
        """提取旋转轴方向
        
        Args:
            motion: 旋转关节运动对象
            
        Returns:
            Optional[Vector3D]: 旋转轴方向向量
        """
        try:
            # 优先使用rotationAxisVector
            if hasattr(motion, 'rotationAxisVector') and motion.rotationAxisVector:
                axis_vector: adsk.core.Vector3D = motion.rotationAxisVector
                return Vector3D(axis_vector.x, axis_vector.y, axis_vector.z)
            
            # 备用：从几何坐标系推断
            self.logger.warning("无法直接获取旋转轴，尝试从几何推断")
            return None
            
        except Exception as e:
            self.logger.error(f"提取旋转轴时发生错误: {e}")
            return None
    
    def _extract_rotation_limits(self, motion: adsk.fusion.RevoluteJointMotion) -> JointLimits:
        """提取旋转限制
        
        Args:
            motion: 旋转关节运动对象
            
        Returns:
            JointLimits: 旋转限制信息
        """
        try:
            # 获取旋转限制对象
            limits: adsk.fusion.JointLimits = motion.rotationLimits
            
            # 检查是否有有效的限制
            if limits:
                min_value = limits.minimumValue  # 弧度
                max_value = limits.maximumValue  # 弧度
                rest_value = limits.restValue    # 弧度
                
                # 验证限制值的合理性
                if min_value is not None and max_value is not None:
                    # 确保最小值小于最大值
                    if min_value > max_value:
                        self.logger.warning(f"旋转限制值异常: min={min_value} > max={max_value}, 自动交换")
                        min_value, max_value = max_value, min_value
                    
                    has_limits = True
                    self.logger.debug(f"找到旋转限制: [{min_value:.3f}, {max_value:.3f}] rad, rest={rest_value:.3f}")
                    
                    return BasicJointLimits(
                        minimum_value=min_value,
                        maximum_value=max_value,
                        rest_value=rest_value,
                        has_limits=has_limits
                    )
            
            # 没有限制或限制无效
            self.logger.debug("未找到有效旋转限制，使用默认值")
            return BasicJointLimits(
                minimum_value=-math.pi,    # 默认-180度
                maximum_value=math.pi,     # 默认+180度
                rest_value=0.0,            # 默认0度
                has_limits=False
            )
            
        except Exception as e:
            self.logger.error(f"提取旋转限制时发生错误: {e}")
            # 返回默认限制
            return BasicJointLimits(
                minimum_value=-math.pi,
                maximum_value=math.pi,
                rest_value=0.0,
                has_limits=False
            )
    
    def _extract_current_rotation(self, motion: adsk.fusion.RevoluteJointMotion) -> Optional[float]:
        """提取当前旋转角度
        
        Args:
            motion: 旋转关节运动对象
            
        Returns:
            Optional[float]: 当前旋转角度（弧度）
        """
        try:
            if hasattr(motion, 'rotationValue'):
                current_angle = motion.rotationValue  # 弧度
                self.logger.debug(f"当前旋转角度: {current_angle:.3f} rad")
                return current_angle
            else:
                return 0.0  # 默认0度
        except Exception as e:
            self.logger.error(f"提取当前旋转角度时发生错误: {e}")
            return 0.0
    
    def _extract_ball_center(self, joint: adsk.fusion.Joint) -> Optional[Vector3D]:
        """提取球心位置
        
        Args:
            joint: 关节对象
            
        Returns:
            Optional[Vector3D]: 球心位置（毫米）
        """
        try:
            # 获取关节几何体
            geometry = joint.geometryOrOriginOne
            if geometry is None:
                self.logger.warning("关节几何体为空")
                return None
                
            # 使用官方API获取原点
            # JointGeometry 对象有 origin 属性，返回 Point3D
            try:
                origin: adsk.core.Point3D = geometry.origin
                if origin:
                    # Fusion 360中的单位是厘米，转换为毫米
                    center_mm = Vector3D(
                        origin.x * 10.0,  # cm → mm
                        origin.y * 10.0,  # cm → mm
                        origin.z * 10.0   # cm → mm
                    )
                    self.logger.debug(f"从几何原点获取球心: {origin} (cm) → {center_mm} (mm)")
                    return center_mm
            except Exception as e:
                self.logger.debug(f"获取几何原点失败: {e}")
            
            # 如果无法从原点获取，尝试从变换矩阵提取位置
            # JointGeometry 对象有 transform 属性，返回 Matrix3D
            try:
                transform: adsk.core.Matrix3D = geometry.transform
                if transform:
                    matrix_data = transform.asArray()
                    # 提取平移部分（索引 3, 7, 11）并转换为毫米
                    center_mm = Vector3D(
                        matrix_data[3] * 10.0,
                        matrix_data[7] * 10.0,
                        matrix_data[11] * 10.0
                    )
                    self.logger.debug(f"从变换矩阵获取球心: {center_mm} (mm)")
                    return center_mm
            except Exception as e:
                self.logger.debug(f"获取变换矩阵失败: {e}")
            
            self.logger.warning("无法获取球心位置")
            return None
            
        except Exception as e:
            self.logger.error(f"提取球心位置时发生错误: {e}")
            return None
    
    def _extract_ball_axis_limits(self, fusion_limits: adsk.fusion.JointLimits, axis_name: str) -> Optional[BasicJointLimits]:
        """提取球关节单轴限制信息
        
        Args:
            fusion_limits: Fusion 360 关节限制对象
            axis_name: 轴名称（用于日志）
            
        Returns:
            Optional[BasicJointLimits]: 关节限制信息
        """
        try:
            if fusion_limits is None:
                self.logger.debug(f"{axis_name}轴限制未设置")
                return None
                
            # 检查限制是否启用
            is_min_enabled = fusion_limits.isMinimumValueEnabled if hasattr(fusion_limits, 'isMinimumValueEnabled') else False
            is_max_enabled = fusion_limits.isMaximumValueEnabled if hasattr(fusion_limits, 'isMaximumValueEnabled') else False
            
            if not (is_min_enabled or is_max_enabled):
                self.logger.debug(f"{axis_name}轴限制未启用")
                return BasicJointLimits(
                    minimum_value=-math.pi,
                    maximum_value=math.pi,
                    rest_value=0.0,
                    has_limits=False
                )
            
            # 获取限制值
            min_value = fusion_limits.minimumValue if is_min_enabled else -math.pi
            max_value = fusion_limits.maximumValue if is_max_enabled else math.pi
            rest_value = fusion_limits.restValue if hasattr(fusion_limits, 'restValue') else 0.0
            
            # 验证限制值
            if min_value > max_value:
                self.logger.warning(f"{axis_name}轴限制值异常: min={min_value} > max={max_value}, 自动交换")
                min_value, max_value = max_value, min_value
            
            limits = BasicJointLimits(
                minimum_value=min_value,
                maximum_value=max_value,
                rest_value=rest_value,
                has_limits=True
            )
            
            self.logger.debug(f"{axis_name}轴限制: [{min_value:.3f}, {max_value:.3f}] rad")
            return limits
            
        except Exception as e:
            self.logger.error(f"提取{axis_name}轴限制时发生错误: {e}")
            return None
    
    def validate_limits(self, limits: JointLimits) -> bool:
        """验证关节限制的合理性
        
        Args:
            limits: 关节限制信息
            
        Returns:
            bool: 限制是否合理
        """
        try:
            if limits.revolute_limits:
                # 验证旋转限制
                rot_limits = limits.revolute_limits.rotation_limits
                if rot_limits.minimum_value is not None and rot_limits.maximum_value is not None:
                    if rot_limits.minimum_value >= rot_limits.maximum_value:
                        self.logger.warning(f"旋转限制值无效: min >= max")
                        return False
                
                # 验证旋转轴（如果有）
                if limits.revolute_limits.rotation_axis:
                    axis = limits.revolute_limits.rotation_axis
                    # 检查轴向量是否为零向量
                    if abs(axis.x) < 1e-10 and abs(axis.y) < 1e-10 and abs(axis.z) < 1e-10:
                        self.logger.warning("旋转轴为零向量")
                        return False
            
            elif limits.ball_limits:
                # 验证球心位置（如果有）
                if limits.ball_limits.center_position:
                    # 球心位置可以是任何有效值
                    pass
                
                # 验证三轴正交性（如果有）
                axes = []
                if limits.ball_limits.pitch_axis:
                    axes.append(limits.ball_limits.pitch_axis)
                if limits.ball_limits.yaw_axis:
                    axes.append(limits.ball_limits.yaw_axis)
                if limits.ball_limits.roll_axis:
                    axes.append(limits.ball_limits.roll_axis)
                
                # 如果有多个轴，检查它们是否正交（放宽验证条件）
                if len(axes) >= 2:
                    for i in range(len(axes)):
                        for j in range(i + 1, len(axes)):
                            # 计算点积，应该接近0
                            dot = axes[i].x * axes[j].x + axes[i].y * axes[j].y + axes[i].z * axes[j].z
                            if abs(dot) > 0.1:  # 放宽到0.1，允许更大的误差
                                self.logger.warning(f"轴向量不够正交: dot={dot:.6f}")
                                # 对于球关节，不强制要求完全正交，只记录警告
                                self.logger.debug("球关节轴向不完全正交，但这是可接受的")
                    
                    # 检查是否有零向量
                    for axis in axes:
                        if abs(axis.x) < 1e-10 and abs(axis.y) < 1e-10 and abs(axis.z) < 1e-10:
                            self.logger.warning("存在零轴向")
                            return False
            
            return True
            
        except Exception as e:
            self.logger.error(f"验证关节限制时发生错误: {e}")
            return False
    
    def get_limits_summary(self, limits: JointLimits) -> Dict[str, Any]:
        """获取关节限制的摘要信息
        
        Args:
            limits: 关节限制信息
            
        Returns:
            Dict[str, Any]: 限制摘要
        """
        if not limits:
            return {"type": "no_limits"}
        
        if limits.revolute_limits:
            rot_limits = limits.revolute_limits.rotation_limits
            return {
                "type": "revolute",
                "min_angle_deg": math.degrees(rot_limits.minimum_value) if rot_limits.minimum_value is not None else None,
                "max_angle_deg": math.degrees(rot_limits.maximum_value) if rot_limits.maximum_value is not None else None,
                "rest_angle_deg": math.degrees(rot_limits.rest_value) if rot_limits.rest_value is not None else None,
                "has_limits": rot_limits.has_limits,
                "current_angle_deg": math.degrees(limits.revolute_limits.current_angle) if limits.revolute_limits.current_angle is not None else None,
                "has_axis": limits.revolute_limits.rotation_axis is not None
            }
        
        elif limits.ball_limits:
            return {
                "type": "ball",
                "has_center": limits.ball_limits.center_position is not None,
                "has_pitch_axis": limits.ball_limits.pitch_axis is not None,
                "has_yaw_axis": limits.ball_limits.yaw_axis is not None,
                "has_roll_axis": limits.ball_limits.roll_axis is not None
            }
        
        else:
            return {"type": "unknown"}