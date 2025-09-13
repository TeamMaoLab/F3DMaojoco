"""
Fusion 360 导出管理器

统一管理整个导出流程，协调各个组件的工作。
遵循 Linus 设计哲学，保持简洁直接。
"""

import os
import time
from typing import Dict, Any

import adsk.core
import adsk.fusion

from .logger import log_performance_start, log_performance_end, log_progress
from .component_collector import ComponentCollector
from .joint_analyzer import JointAnalyzer
from .stl_exporter import STLExporter
from .data_serializer import DataSerializer
from .export_analyzer import ExportAnalyzer
from ..common.data_types import (
    ExportConfig, ExportData, ExportResult, MeshQuality,
    create_default_metadata
)


class FusionExportManager:
    """Fusion 360 导出管理器
    
    统一管理整个导出流程，协调各个组件的工作：
    - 初始化导出配置和日志系统
    - 协调零部件收集、关节分析、STL导出
    - 提供导出摘要和结果分析
    - 错误处理和进度管理
    """
    
    def __init__(self, mesh_quality: MeshQuality = MeshQuality.MEDIUM):
        """初始化导出管理器
        
        Args:
            mesh_quality: STL网格质量
        """
        self.mesh_quality = mesh_quality
        self.logger = None
        self.app = adsk.core.Application.get()
        self.ui = self.app.userInterface
        
        # 导出统计信息
        self._export_stats = {
            'total_components': 0,
            'total_joints': 0,
            'stl_files': [],
            'start_time': None,
            'end_time': None
        }
    
    def export_assembly(self, output_dir: str) -> ExportResult:
        """导出装配体
        
        Args:
            output_dir: 输出目录
            
        Returns:
            ExportResult: 导出结果
        """
        start_time = log_performance_start(self.logger, "装配体导出")
        self._export_stats['start_time'] = start_time
        
        try:
            # 创建输出目录
            os.makedirs(output_dir, exist_ok=True)
            
            # 创建组件实例
            component_collector = ComponentCollector(self.logger)
            joint_analyzer = JointAnalyzer(self.logger)
            stl_exporter = STLExporter(self.logger, self.mesh_quality)
            data_serializer = DataSerializer(self.logger)
            export_analyzer = ExportAnalyzer(self.logger)
            
            # 第一步：收集零部件信息
            self.logger.info("开始收集零部件信息")
            components = component_collector.collect_components()
            self._export_stats['total_components'] = len(components)
            log_progress(self.logger, len(components), len(components), "零部件收集完成")
            
            # 第二步：分析关节信息
            self.logger.info("开始分析关节信息")
            joints = joint_analyzer.analyze_joints()
            self._export_stats['total_joints'] = len(joints)
            log_progress(self.logger, len(joints), len(joints), "关节分析完成")
            
            # 第三步：导出STL文件
            self.logger.info("开始导出STL文件")
            stl_files = stl_exporter.export_components(components, output_dir)
            self._export_stats['stl_files'] = stl_files
            log_progress(self.logger, len(stl_files), len(components), "STL文件导出完成")
            
            # 第四步：创建导出数据
            self.logger.info("开始创建导出数据")
            metadata = create_default_metadata(
                component_count=len(components),
                joint_count=len(joints),
                logger=self.logger
            )
            
            export_data = ExportData(
                meta=metadata,
                components=components,
                joints=joints
            )
            
            # 第五步：序列化数据
            self.logger.info("开始序列化数据")
            data_serializer.serialize_data(export_data, output_dir)
            
            # 第六步：生成分析报告
            self.logger.info("开始生成分析报告")
            export_analyzer.analyze_export(export_data, stl_files)
            
            # 创建导出结果
            result = ExportResult(
                config=ExportConfig(
                    output_dir=output_dir,
                    mesh_quality=self.mesh_quality,
                    create_timestamp_dir=False
                ),
                data=export_data,
                output_directory=output_dir,
                stl_files=stl_files,
                execution_time=time.time() - start_time,
                success=True
            )
            
            self._export_stats['end_time'] = time.time()
            log_performance_end(self.logger, "装配体导出", start_time)
            
            self.logger.info(f"导出完成: {len(components)} 个零部件, {len(joints)} 个关节")
            return result
            
        except Exception as e:
            self.logger.error(f"导出失败: {str(e)}", exc_info=True)
            
            # 创建失败结果
            result = ExportResult(
                config=ExportConfig(
                    output_dir=output_dir,
                    mesh_quality=self.mesh_quality,
                    create_timestamp_dir=False
                ),
                data=ExportData(
                    meta=create_default_metadata(0, 0, self.logger),
                    components=[],
                    joints=[]
                ),
                output_directory=output_dir,
                stl_files=[],
                execution_time=time.time() - start_time,
                success=False,
                error_message=str(e)
            )
            
            self._export_stats['end_time'] = time.time()
            return result
    
    def get_export_summary(self) -> Dict[str, Any]:
        """获取导出摘要
        
        Returns:
            Dict[str, Any]: 导出摘要信息
        """
        execution_time = 0
        if self._export_stats['start_time'] and self._export_stats['end_time']:
            execution_time = self._export_stats['end_time'] - self._export_stats['start_time']
        
        return {
            'output_directory': getattr(self, '_output_dir', ''),
            'mesh_quality': self.mesh_quality.value,
            'total_components': self._export_stats['total_components'],
            'total_joints': self._export_stats['total_joints'],
            'stl_export': {
                'exported_files_count': len(self._export_stats['stl_files']),
                'file_list': self._export_stats['stl_files']
            },
            'execution_time': execution_time
        }
    
    def analyze_export_results(self) -> Dict[str, Any]:
        """分析导出结果
        
        Returns:
            Dict[str, Any]: 分析结果
        """
        # 这里可以添加更复杂的分析逻辑
        return {
            'components_with_bodies': self._export_stats['total_components'],
            'components_with_stl': len(self._export_stats['stl_files']),
            'components_with_children': 0,  # 需要从组件数据中统计
            'active_joints': self._export_stats['total_joints'],
            'joint_types': {
                'rigid': 0,  # 需要从关节数据中统计
                'revolute': 0,
                'slider': 0,
                'cylindrical': 0,
                'pin_slot': 0,
                'planar': 0,
                'ball': 0,
                'inferred': 0
            }
        }
    
    def set_logger(self, logger):
        """设置日志记录器
        
        Args:
            logger: 日志记录器实例
        """
        self.logger = logger