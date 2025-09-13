"""
STL导出器

将零部件几何导出为STL文件。
控制网格质量，批量导出多个零部件。
"""

import os
import traceback
from typing import List, Optional

import adsk.core
import adsk.fusion

from .logger import log_progress, log_component, log_error
from ..common.data_types import ComponentInfo, MeshQuality


class STLExporter:
    """STL导出器
    
    将零部件几何导出为STL文件：
    - 控制网格质量（LOW/MEDIUM/HIGH）
    - 批量导出多个零部件
    - 文件路径管理和命名
    - 导出进度跟踪
    """
    
    def __init__(self, logger, mesh_quality: MeshQuality = MeshQuality.MEDIUM):
        """初始化STL导出器
        
        Args:
            logger: 日志记录器
            mesh_quality: 网格质量
        """
        self.logger = logger
        self.mesh_quality = mesh_quality
        self.app = adsk.core.Application.get()
        self.design = self.app.activeProduct
        
        # 网格质量设置
        self._mesh_quality_settings = {
            MeshQuality.LOW: {
                'angleTolerance': 30.0,
                'aspectRatio': 10.0,
                'surfaceTolerance': 0.1,
                'maxEdgeLength': 1.0
            },
            MeshQuality.MEDIUM: {
                'angleTolerance': 15.0,
                'aspectRatio': 6.0,
                'surfaceTolerance': 0.05,
                'maxEdgeLength': 0.5
            },
            MeshQuality.HIGH: {
                'angleTolerance': 5.0,
                'aspectRatio': 3.0,
                'surfaceTolerance': 0.01,
                'maxEdgeLength': 0.1
            }
        }
    
    def export_components(self, components: List[ComponentInfo], output_dir: str) -> List[str]:
        """导出所有零部件的STL文件
        
        Args:
            components: 零部件信息列表
            output_dir: 输出目录
            
        Returns:
            List[str]: 成功导出的STL文件路径列表
        """
        self.logger.info(f"开始导出STL文件，网格质量: {self.mesh_quality.value}")
        
        # 创建STL文件目录
        stl_dir = os.path.join(output_dir, "stl_files")
        os.makedirs(stl_dir, exist_ok=True)
        
        # 导出的文件列表
        exported_files = []
        
        # 遍历所有零部件
        for i, component in enumerate(components):
            try:
                log_progress(self.logger, i + 1, len(components), f"导出 {component.name}")
                
                # 导出单个零部件
                stl_file = self._export_single_component(component, stl_dir)
                
                if stl_file:
                    exported_files.append(stl_file)
                    component.stl_file = os.path.relpath(stl_file, output_dir)
                    log_component(self.logger, component.name, "STL导出成功")
                else:
                    log_component(self.logger, component.name, "STL导出失败")
                    
            except Exception as e:
                log_error(self.logger, e, f"导出零部件 {component.name} STL")
                continue
        
        self.logger.info(f"STL导出完成，成功导出 {len(exported_files)} 个文件")
        return exported_files
    
    def _export_single_component(self, component: ComponentInfo, stl_dir: str) -> Optional[str]:
        """导出单个零部件的STL文件
        
        Args:
            component: 零部件信息
            stl_dir: STL文件目录
            
        Returns:
            Optional[str]: 导出的STL文件路径
        """
        try:
            # 生成安全的文件名
            safe_name = self._get_safe_filename(component.name)
            stl_filename = f"{safe_name}_{component.component_id}.stl"
            stl_filepath = os.path.join(stl_dir, stl_filename)
            
            # 查找对应的 Fusion 360 组件
            fusion_component = self._find_fusion_component(component)
            if not fusion_component:
                self.logger.warning(f"未找到零部件 {component.name} 对应的 Fusion 360 组件")
                return None
            
            # 创建导出管理器
            export_manager = self.design.exportManager
            
            # 创建STL导出选项
            stl_options = export_manager.createSTLExportOptions(fusion_component, stl_filepath)
            
            # 设置网格质量
            self._configure_mesh_quality(stl_options)
            
            # 执行导出
            export_manager.execute(stl_options)
            
            # 检查文件是否成功创建
            if os.path.exists(stl_filepath):
                return stl_filepath
            else:
                self.logger.warning(f"STL文件未创建: {stl_filepath}")
                return None
                
        except Exception as e:
            log_error(self.logger, e, f"导出单个零部件 {component.name}")
            return None
    
    def _find_fusion_component(self, component: ComponentInfo) -> Optional[adsk.fusion.Component]:
        """查找对应的 Fusion 360 组件
        
        Args:
            component: 零部件信息
            
        Returns:
            Optional[adsk.fusion.Component]: Fusion 360 组件
        """
        try:
            # 在根组件的所有 occurrences 中查找
            root_component = self.design.rootComponent
            
            # 遍历所有 occurrences
            for occurrence in root_component.allOccurrences:
                if occurrence.component and occurrence.component.name == component.component_id:
                    return occurrence.component
            
            # 如果在 occurrences 中没找到，尝试在所有组件中查找
            for fusion_component in self.design.allComponents:
                if fusion_component.name == component.component_id:
                    return fusion_component
            
            return None
            
        except Exception as e:
            log_error(self.logger, e, f"查找 Fusion 360 组件 {component.name}")
            return None
    
    def _configure_mesh_quality(self, stl_options: adsk.fusion.STLExportOptions):
        """配置网格质量设置
        
        Args:
            stl_options: STL导出选项
        """
        try:
            # 获取当前质量设置
            quality_settings = self._mesh_quality_settings[self.mesh_quality]
            
            # 设置网格参数
            stl_options.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
            
            # 根据质量设置调整参数
            if self.mesh_quality == MeshQuality.LOW:
                stl_options.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
            elif self.mesh_quality == MeshQuality.HIGH:
                stl_options.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
            
            # 设置其他参数
            stl_options.normalDeviation = quality_settings['angleTolerance']
            stl_options.aspectRatio = quality_settings['aspectRatio']
            stl_options.surfaceDeviation = quality_settings['surfaceTolerance']
            stl_options.maximumEdgeLength = quality_settings['maxEdgeLength']
            
        except Exception as e:
            log_error(self.logger, e, "配置网格质量设置")
            # 如果配置失败，使用默认设置
    
    def _get_safe_filename(self, name: str) -> str:
        """生成安全的文件名
        
        Args:
            name: 原始名称
            
        Returns:
            str: 安全的文件名
        """
        # 替换不安全的字符
        unsafe_chars = '<>:"/\\|?*'
        safe_name = name
        
        for char in unsafe_chars:
            safe_name = safe_name.replace(char, '_')
        
        # 限制长度
        if len(safe_name) > 100:
            safe_name = safe_name[:100]
        
        return safe_name
    
    def export_single_component_to_path(self, component: ComponentInfo, output_path: str) -> bool:
        """导出单个零部件到指定路径
        
        Args:
            component: 零部件信息
            output_path: 输出路径
            
        Returns:
            bool: 是否成功导出
        """
        try:
            # 确保输出目录存在
            output_dir = os.path.dirname(output_path)
            if output_dir:
                os.makedirs(output_dir, exist_ok=True)
            
            # 查找对应的 Fusion 360 组件
            fusion_component = self._find_fusion_component(component)
            if not fusion_component:
                self.logger.warning(f"未找到零部件 {component.name} 对应的 Fusion 360 组件")
                return False
            
            # 创建导出管理器
            export_manager = self.design.exportManager
            
            # 创建STL导出选项
            stl_options = export_manager.createSTLExportOptions(fusion_component, output_path)
            
            # 设置网格质量
            self._configure_mesh_quality(stl_options)
            
            # 执行导出
            export_manager.execute(stl_options)
            
            # 检查文件是否成功创建
            if os.path.exists(output_path):
                return True
            else:
                self.logger.warning(f"STL文件未创建: {output_path}")
                return False
                
        except Exception as e:
            log_error(self.logger, e, f"导出单个零部件 {component.name} 到 {output_path}")
            return False