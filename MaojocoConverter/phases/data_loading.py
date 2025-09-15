"""
数据加载阶段

从 Fusion 360 导出数据中加载并提取几何信息，包括关节名称转换为拼音。
"""

from typing import Dict, Any, Optional
from pathlib import Path
import re

# 直接导入F3DMaojocoScripts的common模块
from F3DMaojocoScripts.common.data_types import (
    load_export_data, ExportData, ComponentInfo, JointInfo,
    Vector3D, Quaternion
)

from ..utils.logger import logger
from ..context import MaojocoContext
from .base import ConversionPhase
from ..type_definitions import (
    Body4DCoordinates, Body4DCoordinatesDict, JointGlobalCoordinates,
    JointGlobalCoordinatesDict, PhaseResult, PhaseStatus
)
from F3DMaojocoScripts.common.geometry_math import Transform4D

try:
    from pypinyin import pinyin, Style
    PINYIN_AVAILABLE = True
except ImportError:
    PINYIN_AVAILABLE = False
    logger.warning("⚠️  pypinyin library not found, joint names will not be converted to pinyin")


class DataLoadingPhase(ConversionPhase):
    """数据加载阶段"""
    
    def __init__(self, ctx: MaojocoContext):
        super().__init__("DataLoading", ctx)
        self._required_files = ['component_positions.json', 'export_description.md']
    
    def _convert_to_pinyin(self, text: str) -> str:
        """将中文文本转换为拼音"""
        if not PINYIN_AVAILABLE:
            return text
        
        if not text or not any('\u4e00' <= char <= '\u9fff' for char in text):
            return text
        
        try:
            # 分离中英文部分
            chinese_chars = []
            non_chinese_chars = []
            
            for char in text:
                if '\u4e00' <= char <= '\u9fff':
                    chinese_chars.append(char)
                else:
                    non_chinese_chars.append(char)
            
            # 转换中文部分为拼音
            if chinese_chars:
                chinese_text = ''.join(chinese_chars)
                pinyin_result = pinyin(chinese_text, style=Style.NORMAL, heteronym=False)
                pinyin_str = '_'.join([p[0].lower() for p in pinyin_result if p])
                
                # 处理非中文部分（清理连续空格）
                non_chinese_text = ''.join(non_chinese_chars).strip()
                non_chinese_clean = re.sub(r'\s+', '_', non_chinese_text)
                
                # 组合结果
                if non_chinese_clean:
                    result = f"{pinyin_str}_{non_chinese_clean}" if pinyin_str else non_chinese_clean
                else:
                    result = pinyin_str
                
                # 清理多余的下划线
                result = re.sub(r'_+', '_', result).strip('_')
                
                logger.info(f"🔄 关节名称转换: '{text}' -> '{result}'")
                return result
            else:
                # 如果没有中文字符，只清理空格
                result = re.sub(r'\s+', '_', text).strip('_')
                return result
                
        except Exception as e:
            logger.warning(f"⚠️  拼音转换失败 '{text}': {e}")
            return text
    
    def _execute(self) -> bool:
        """执行数据加载"""
        logger.info("📁 开始加载导出数据")
        
        try:
            # 加载导出数据
            export_data_file = self.ctx.export_dir / 'component_positions.json'
            if not export_data_file.exists():
                logger.error(f"❌ 导出数据文件不存在: {export_data_file}")
                return False
            
            # 使用共享的 load_export_data 函数
            self.ctx.raw_export_data = load_export_data(str(export_data_file))
            if not self.ctx.raw_export_data:
                logger.error("❌ 导出数据加载失败")
                return False
            
            # 转换关节名称为拼音
            self._convert_joint_names_to_pinyin()
            
            # 验证数据完整性
            if not self._validate_export_data():
                logger.error("❌ 导出数据验证失败")
                return False
            
            # 提取4D坐标信息
            self._extract_4d_coordinates()
            
            # 提取关节全局坐标
            self._extract_joint_coordinates()
            
                        
            logger.info(f"📊 数据加载成功:")
            logger.info(f"    - 零部件数量: {len(self.ctx.raw_export_data.components)}")
            logger.info(f"    - 关节数量: {len(self.ctx.raw_export_data.joints)}")
            logger.info(f"    - 4D坐标: {len(self.ctx.body_4d_coordinates)} 个")
            logger.info(f"    - 关节坐标: {len(self.ctx.joint_global_coordinates)} 个")
            
            return True
            
        except Exception as e:
            logger.error(f"❌ 数据加载失败: {e}")
            return False
    
    def _convert_joint_names_to_pinyin(self):
        """将所有关节名称转换为拼音"""
        if not self.ctx.raw_export_data or not self.ctx.raw_export_data.joints:
            return
        
        logger.info("🔄 开始转换关节名称为拼音")
        converted_count = 0
        
        for joint in self.ctx.raw_export_data.joints:
            original_name = joint.name
            pinyin_name = self._convert_to_pinyin(original_name)
            
            if original_name != pinyin_name:
                # 修改关节名称
                joint.name = pinyin_name
                converted_count += 1
                logger.info(f"   ✅ {original_name} → {pinyin_name}")
        
        logger.info(f"🔄 关节名称转换完成，共转换 {converted_count} 个关节")
    
    def _validate_export_data(self) -> bool:
        """验证导出数据的完整性"""
        if not self.ctx.raw_export_data:
            return False
        
        # 检查元数据
        if not self.ctx.raw_export_data.meta:
            logger.error("❌ 缺少导出元数据")
            return False
        
        # 检查零部件数据
        if not self.ctx.raw_export_data.components:
            logger.warning("⚠️  未找到零部件数据")
        
        # 验证零部件数据
        for component in self.ctx.raw_export_data.components:
            if not component.name:
                logger.error("❌ 零部件名称为空")
                return False
            if not component.world_transform:
                logger.warning(f"⚠️  零部件 {component.name} 缺少世界变换矩阵")
        
        # 验证关节数据
        for joint in self.ctx.raw_export_data.joints:
            if not joint.name:
                logger.error("❌ 关节名称为空")
                return False
            if not joint.joint_type:
                logger.error(f"❌ 关节 {joint.name} 缺少类型")
                return False
        
        logger.info("✅ 导出数据验证通过")
        return True
    
    def _extract_4d_coordinates(self):
        """提取Body的4D坐标表达"""
        logger.info("📐 提取Body的4D坐标表达")
        
        for component in self.ctx.raw_export_data.components:
            if component.world_transform:
                # 构建Body的4D坐标表达
                body_4d = Body4DCoordinates(
                    name=component.name,
                    occurrence_name=component.occurrence_name,
                    full_path_name=component.full_path_name,
                    component_id=component.component_id,
                    transform=component.world_transform,
                    stl_file=component.stl_file,
                    bodies_count=component.bodies_count,
                    has_children=component.has_children
                )
                
                self.ctx.body_4d_coordinates[component.name] = body_4d
        
        logger.info(f"📊 提取了 {len(self.ctx.body_4d_coordinates)} 个Body的4D坐标")
    
    def _extract_joint_coordinates(self):
        """提取Joint的全局坐标"""
        logger.info("🔗 提取Joint的全局坐标")
        
        for joint in self.ctx.raw_export_data.joints:
            # 基于关节几何信息计算全局坐标
            global_position = None
            global_quaternion = Quaternion.identity()
            
            if joint.geometry.geometry_one_transform:
                # 使用第一个几何体的变换作为关节位置
                global_position = joint.geometry.geometry_one_transform.get_translation()
                
                # 从变换矩阵中提取旋转四元数
                global_quaternion = joint.geometry.geometry_one_transform.to_quaternion()
            
            # 如果没有几何信息，尝试从连接的零部件推断
            if not global_position and joint.connection:
                # 找到连接的零部件，使用其位置作为关节位置
                connected_component = None
                for component in self.ctx.raw_export_data.components:
                    if component.name == joint.connection.occurrence_one_component:
                        connected_component = component
                        break
                
                if connected_component and connected_component.world_transform:
                    global_position = connected_component.world_transform.get_translation()
                    global_quaternion = connected_component.world_transform.to_quaternion()
            
            if global_position:
                joint_coord = JointGlobalCoordinates(
                    position=global_position,
                    quaternion=global_quaternion,
                    joint_name=joint.name,
                    joint_type=joint.joint_type
                )
                self.ctx.joint_global_coordinates[joint.name] = joint_coord
            else:
                logger.warning(f"⚠️  无法确定关节 {joint.name} 的全局坐标")
        
        logger.info(f"📊 提取了 {len(self.ctx.joint_global_coordinates)} 个Joint的全局坐标")