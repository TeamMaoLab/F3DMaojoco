"""
F3DMaojocoScripts - inf3d 模块

Fusion 360 特定的数据导出功能模块。
包含所有依赖 Fusion 360 API 的类和函数。
"""

from inf3d.logger import (
    setup_logger,
    log_performance_start,
    log_performance_end,
    log_progress,
    log_component,
    log_joint,
    log_transform,
    log_error,
    log_warning,
    log_debug,
    log_info,
    close_logger,
    get_logger,
    initialize_logging,
    cleanup_logging
)

from inf3d.fusion_export_manager import FusionExportManager
from inf3d.component_collector import ComponentCollector
from inf3d.joint_analyzer import JointAnalyzer
from inf3d.stl_exporter import STLExporter
from inf3d.brep_extractor import BRepExtractor
from inf3d.data_serializer import DataSerializer
from inf3d.export_analyzer import ExportAnalyzer

__all__ = [
    # 日志功能
    'setup_logger',
    'log_performance_start',
    'log_performance_end',
    'log_progress',
    'log_component',
    'log_joint',
    'log_transform',
    'log_error',
    'log_warning',
    'log_debug',
    'log_info',
    'close_logger',
    'get_logger',
    'initialize_logging',
    'cleanup_logging',

    # 核心导出类
    'FusionExportManager',
    'ComponentCollector',
    'JointAnalyzer',
    'STLExporter',
    'BRepExtractor',
    'DataSerializer',
    'ExportAnalyzer'
]