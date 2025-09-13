"""
导出分析器

分析导出结果，提供统计信息。
生成导出摘要，分析关节类型分布。
"""

import os
import traceback
from typing import Dict, Any, List
from collections import Counter

from .logger import log_info, log_warning
from ..common.data_types import ExportData, JointType


class ExportAnalyzer:
    """导出分析器
    
    分析导出结果，提供统计信息：
    - 生成导出摘要（零部件数量、关节数量等）
    - 分析关节类型分布
    - 检查导出完整性和潜在问题
    - 提供用户友好的统计报告
    """
    
    def __init__(self, logger):
        """初始化导出分析器
        
        Args:
            logger: 日志记录器
        """
        self.logger = logger
    
    def analyze_export(self, export_data: ExportData, stl_files: List[str]) -> Dict[str, Any]:
        """分析导出结果
        
        Args:
            export_data: 导出数据
            stl_files: STL文件列表
            
        Returns:
            Dict[str, Any]: 分析结果
        """
        try:
            self.logger.info("开始分析导出结果")
            
            analysis_result = {
                "summary": self._generate_summary(export_data, stl_files),
                "component_analysis": self._analyze_components(export_data),
                "joint_analysis": self._analyze_joints(export_data),
                "file_analysis": self._analyze_files(stl_files),
                "quality_check": self._check_quality(export_data, stl_files),
                "recommendations": self._generate_recommendations(export_data, stl_files)
            }
            
            # 记录主要发现
            self._log_analysis_summary(analysis_result)
            
            return analysis_result
            
        except Exception as e:
            log_warning(self.logger, f"分析导出结果时发生错误: {str(e)}")
            return self._get_empty_analysis_result()
    
    def _generate_summary(self, export_data: ExportData, stl_files: List[str]) -> Dict[str, Any]:
        """生成导出摘要
        
        Args:
            export_data: 导出数据
            stl_files: STL文件列表
            
        Returns:
            Dict[str, Any]: 摘要信息
        """
        return {
            "export_time": export_data.meta.export_time,
            "total_components": export_data.meta.count_components,
            "total_joints": export_data.meta.count_joints,
            "stl_files_count": len(stl_files),
            "geometry_unit": export_data.meta.geometry_unit,
            "position_unit": export_data.meta.position_unit,
            "format_version": export_data.meta.format_version
        }
    
    def _analyze_components(self, export_data: ExportData) -> Dict[str, Any]:
        """分析零部件信息
        
        Args:
            export_data: 导出数据
            
        Returns:
            Dict[str, Any]: 零部件分析结果
        """
        try:
            components = export_data.components
            
            # 基本统计
            total_components = len(components)
            components_with_bodies = sum(1 for c in components if c.bodies_count > 0)
            components_with_stl = sum(1 for c in components if c.stl_file)
            components_with_children = sum(1 for c in components if c.has_children)
            
            # 实体数量统计
            total_bodies = sum(c.bodies_count for c in components)
            avg_bodies_per_component = total_bodies / total_components if total_components > 0 else 0
            
            # 路径深度分析
            path_depths = []
            for component in components:
                depth = component.full_path_name.count('/') if component.full_path_name else 0
                path_depths.append(depth)
            
            max_depth = max(path_depths) if path_depths else 0
            avg_depth = sum(path_depths) / len(path_depths) if path_depths else 0
            
            return {
                "total_components": total_components,
                "components_with_bodies": components_with_bodies,
                "components_with_stl": components_with_stl,
                "components_with_children": components_with_children,
                "total_bodies": total_bodies,
                "avg_bodies_per_component": round(avg_bodies_per_component, 2),
                "max_assembly_depth": max_depth,
                "avg_assembly_depth": round(avg_depth, 2),
                "component_types": self._analyze_component_types(components)
            }
            
        except Exception as e:
            log_warning(self.logger, f"分析零部件信息时发生错误: {str(e)}")
            return self._get_empty_component_analysis()
    
    def _analyze_component_types(self, components: List) -> Dict[str, int]:
        """分析零部件类型分布
        
        Args:
            components: 零部件列表
            
        Returns:
            Dict[str, int]: 类型分布
        """
        try:
            # 根据零部件特征分类
            type_counts = {
                "simple_part": 0,        # 简单零件（无子部件，单个实体）
                "complex_part": 0,       # 复杂零件（无子部件，多个实体）
                "sub_assembly": 0,       # 子装配体（有子部件）
                "empty_component": 0     # 空组件（无实体）
            }
            
            for component in components:
                if component.has_children:
                    type_counts["sub_assembly"] += 1
                elif component.bodies_count == 0:
                    type_counts["empty_component"] += 1
                elif component.bodies_count == 1:
                    type_counts["simple_part"] += 1
                else:
                    type_counts["complex_part"] += 1
            
            return type_counts
            
        except Exception as e:
            log_warning(self.logger, f"分析零部件类型时发生错误: {str(e)}")
            return {}
    
    def _analyze_joints(self, export_data: ExportData) -> Dict[str, Any]:
        """分析关节信息
        
        Args:
            export_data: 导出数据
            
        Returns:
            Dict[str, Any]: 关节分析结果
        """
        try:
            joints = export_data.joints
            
            # 基本统计
            total_joints = len(joints)
            active_joints = sum(1 for j in joints if not j.is_suppressed and j.is_light_bulb_on)
            suppressed_joints = sum(1 for j in joints if j.is_suppressed)
            inactive_joints = sum(1 for j in joints if not j.is_light_bulb_on)
            
            # 关节类型分布
            joint_type_counts = Counter(joint.joint_type for joint in joints)
            joint_type_distribution = {joint_type.value: count for joint_type, count in joint_type_counts.items()}
            
            # 连接状态分析
            well_connected_joints = 0
            poorly_connected_joints = 0
            
            for joint in joints:
                connection = joint.connection
                if (connection.occurrence_one_component and connection.occurrence_two_component):
                    well_connected_joints += 1
                else:
                    poorly_connected_joints += 1
            
            return {
                "total_joints": total_joints,
                "active_joints": active_joints,
                "suppressed_joints": suppressed_joints,
                "inactive_joints": inactive_joints,
                "joint_types": joint_type_distribution,
                "well_connected_joints": well_connected_joints,
                "poorly_connected_joints": poorly_connected_joints,
                "joint_type_details": self._get_joint_type_details(joint_type_distribution)
            }
            
        except Exception as e:
            log_warning(self.logger, f"分析关节信息时发生错误: {str(e)}")
            return self._get_empty_joint_analysis()
    
    def _get_joint_type_details(self, joint_type_distribution: Dict[str, int]) -> Dict[str, str]:
        """获取关节类型详细说明
        
        Args:
            joint_type_distribution: 关节类型分布
            
        Returns:
            Dict[str, str]: 关节类型说明
        """
        type_descriptions = {
            "rigid": "刚性连接，完全固定，无相对运动",
            "revolute": "旋转副，绕单一轴线旋转运动",
            "slider": "滑动副，沿单一轴线平移运动",
            "cylindrical": "圆柱副，可同时绕轴线旋转和沿轴线平移",
            "pin_slot": "销槽副，销钉在槽内滑动，约束较复杂",
            "planar": "平面副，在平面内进行平移和旋转运动",
            "ball": "球面副，绕固定点进行三轴旋转运动",
            "inferred": "推断副，由系统自动推断的关节类型"
        }
        
        details = {}
        for joint_type, count in joint_type_distribution.items():
            description = type_descriptions.get(joint_type, "未知关节类型")
            details[joint_type] = f"{description} (数量: {count})"
        
        return details
    
    def _analyze_files(self, stl_files: List[str]) -> Dict[str, Any]:
        """分析文件信息
        
        Args:
            stl_files: STL文件列表
            
        Returns:
            Dict[str, Any]: 文件分析结果
        """
        try:
            # 基本统计
            total_files = len(stl_files)
            
            # 文件大小统计
            total_size = 0
            file_sizes = []
            
            for stl_file in stl_files:
                try:
                    if os.path.exists(stl_file):
                        size = os.path.getsize(stl_file)
                        total_size += size
                        file_sizes.append(size)
                except Exception:
                    continue
            
            avg_file_size = total_size / total_files if total_files > 0 else 0
            
            # 文件大小分布
            size_distribution = {
                "small": sum(1 for size in file_sizes if size < 1024 * 1024),  # < 1MB
                "medium": sum(1 for size in file_sizes if 1024 * 1024 <= size < 10 * 1024 * 1024),  # 1-10MB
                "large": sum(1 for size in file_sizes if size >= 10 * 1024 * 1024)  # >= 10MB
            }
            
            return {
                "total_files": total_files,
                "total_size_mb": round(total_size / (1024 * 1024), 2),
                "avg_file_size_mb": round(avg_file_size / (1024 * 1024), 2),
                "size_distribution": size_distribution,
                "missing_files": sum(1 for f in stl_files if not os.path.exists(f))
            }
            
        except Exception as e:
            log_warning(self.logger, f"分析文件信息时发生错误: {str(e)}")
            return self._get_empty_file_analysis()
    
    def _check_quality(self, export_data: ExportData, stl_files: List[str]) -> Dict[str, Any]:
        """检查导出质量
        
        Args:
            export_data: 导出数据
            stl_files: STL文件列表
            
        Returns:
            Dict[str, Any]: 质量检查结果
        """
        try:
            issues = []
            warnings = []
            
            # 检查零部件完整性
            components_without_stl = [c for c in export_data.components if not c.stl_file]
            if components_without_stl:
                issues.append(f"{len(components_without_stl)} 个零部件缺少STL文件")
            
            # 检查空零部件
            empty_components = [c for c in export_data.components if c.bodies_count == 0]
            if empty_components:
                warnings.append(f"{len(empty_components)} 个零部件没有几何实体")
            
            # 检查关节连接
            poorly_connected_joints = [
                j for j in export_data.joints 
                if not (j.connection.occurrence_one_component and j.connection.occurrence_two_component)
            ]
            if poorly_connected_joints:
                warnings.append(f"{len(poorly_connected_joints)} 个关节连接信息不完整")
            
            # 检查文件存在性
            missing_files = [f for f in stl_files if not os.path.exists(f)]
            if missing_files:
                issues.append(f"{len(missing_files)} 个STL文件缺失")
            
            return {
                "has_issues": len(issues) > 0,
                "has_warnings": len(warnings) > 0,
                "issues": issues,
                "warnings": warnings,
                "quality_score": self._calculate_quality_score(export_data, stl_files, issues, warnings)
            }
            
        except Exception as e:
            log_warning(self.logger, f"检查导出质量时发生错误: {str(e)}")
            return self._get_empty_quality_check()
    
    def _calculate_quality_score(self, export_data: ExportData, stl_files: List[str], 
                                issues: List[str], warnings: List[str]) -> float:
        """计算质量分数
        
        Args:
            export_data: 导出数据
            stl_files: STL文件列表
            issues: 问题列表
            warnings: 警告列表
            
        Returns:
            float: 质量分数 (0-100)
        """
        try:
            score = 100.0
            
            # 扣除问题分数
            score -= len(issues) * 20  # 每个问题扣20分
            score -= len(warnings) * 5  # 每个警告扣5分
            
            # 检查STL文件覆盖率
            if export_data.components:
                stl_coverage = len(stl_files) / len(export_data.components)
                if stl_coverage < 1.0:
                    score -= (1.0 - stl_coverage) * 30  # STL覆盖率不足扣分
            
            # 检查关节完整性
            if export_data.joints:
                complete_joints = sum(1 for j in export_data.joints 
                                   if (j.connection.occurrence_one_component and j.connection.occurrence_two_component))
                joint_completeness = complete_joints / len(export_data.joints)
                if joint_completeness < 1.0:
                    score -= (1.0 - joint_completeness) * 15  # 关节完整性不足扣分
            
            return max(0.0, min(100.0, score))
            
        except Exception:
            return 0.0
    
    def _generate_recommendations(self, export_data: ExportData, stl_files: List[str]) -> List[str]:
        """生成改进建议
        
        Args:
            export_data: 导出数据
            stl_files: STL文件列表
            
        Returns:
            List[str]: 建议列表
        """
        try:
            recommendations = []
            
            # 基于分析结果生成建议
            components_without_stl = [c for c in export_data.components if not c.stl_file]
            if components_without_stl:
                recommendations.append("检查缺少STL文件的零部件，可能需要重新导出")
            
            empty_components = [c for c in export_data.components if c.bodies_count == 0]
            if len(empty_components) > len(export_data.components) * 0.1:  # 超过10%
                recommendations.append("存在较多空零部件，建议检查模型完整性")
            
            if export_data.joints:
                active_joints = sum(1 for j in export_data.joints if not j.is_suppressed and j.is_light_bulb_on)
                if active_joints == 0:
                    recommendations.append("没有活动的关节，可能需要检查关节约束设置")
            
            # 检查文件大小
            total_size = sum(os.path.getsize(f) for f in stl_files if os.path.exists(f))
            if total_size > 100 * 1024 * 1024:  # 超过100MB
                recommendations.append("STL文件总大小较大，考虑优化网格质量或清理模型")
            
            return recommendations
            
        except Exception as e:
            log_warning(self.logger, f"生成改进建议时发生错误: {str(e)}")
            return []
    
    def _log_analysis_summary(self, analysis_result: Dict[str, Any]):
        """记录分析摘要
        
        Args:
            analysis_result: 分析结果
        """
        try:
            summary = analysis_result["summary"]
            component_analysis = analysis_result["component_analysis"]
            joint_analysis = analysis_result["joint_analysis"]
            quality_check = analysis_result["quality_check"]
            
            log_info(self.logger, f"导出分析完成:")
            log_info(self.logger, f"  - 零部件: {summary['total_components']} 个")
            log_info(self.logger, f"  - 关节: {summary['total_joints']} 个")
            log_info(self.logger, f"  - STL文件: {summary['stl_files_count']} 个")
            log_info(self.logger, f"  - 质量分数: {quality_check['quality_score']:.1f}/100")
            
            if quality_check["has_issues"]:
                log_info(self.logger, f"  - 发现 {len(quality_check['issues'])} 个问题需要处理")
            
            if quality_check["has_warnings"]:
                log_info(self.logger, f"  - 发现 {len(quality_check['warnings'])} 个警告")
            
        except Exception as e:
            log_warning(self.logger, f"记录分析摘要时发生错误: {str(e)}")
    
    def _get_empty_analysis_result(self) -> Dict[str, Any]:
        """获取空的分析结果"""
        return {
            "summary": {},
            "component_analysis": self._get_empty_component_analysis(),
            "joint_analysis": self._get_empty_joint_analysis(),
            "file_analysis": self._get_empty_file_analysis(),
            "quality_check": self._get_empty_quality_check(),
            "recommendations": []
        }
    
    def _get_empty_component_analysis(self) -> Dict[str, Any]:
        """获取空的零部件分析结果"""
        return {
            "total_components": 0,
            "components_with_bodies": 0,
            "components_with_stl": 0,
            "components_with_children": 0,
            "total_bodies": 0,
            "avg_bodies_per_component": 0,
            "max_assembly_depth": 0,
            "avg_assembly_depth": 0,
            "component_types": {}
        }
    
    def _get_empty_joint_analysis(self) -> Dict[str, Any]:
        """获取空的关节分析结果"""
        return {
            "total_joints": 0,
            "active_joints": 0,
            "suppressed_joints": 0,
            "inactive_joints": 0,
            "joint_types": {},
            "well_connected_joints": 0,
            "poorly_connected_joints": 0,
            "joint_type_details": {}
        }
    
    def _get_empty_file_analysis(self) -> Dict[str, Any]:
        """获取空的文件分析结果"""
        return {
            "total_files": 0,
            "total_size_mb": 0,
            "avg_file_size_mb": 0,
            "size_distribution": {"small": 0, "medium": 0, "large": 0},
            "missing_files": 0
        }
    
    def _get_empty_quality_check(self) -> Dict[str, Any]:
        """获取空的质量检查结果"""
        return {
            "has_issues": False,
            "has_warnings": False,
            "issues": [],
            "warnings": [],
            "quality_score": 0.0
        }