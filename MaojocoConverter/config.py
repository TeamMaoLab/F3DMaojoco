"""
MaojocoConverter 配置管理

配置工具函数，用于处理配置文件和命令行参数
"""

from pathlib import Path
from typing import Dict, Any, Optional
import json

# 导入统一的配置类型
from .type_definitions import ConversionConfig


def create_config_from_dict(config_dict: Dict[str, Any]) -> ConversionConfig:
    """从字典创建配置对象"""
    # 设置默认值
    defaults = {
        'output_format': 'xml',
        'coordinate_system': 'mujoco',
        'unit_conversion': {
            'enabled': True,
            'scale_factor': 0.001  # mm to m
        },
        'xml_generation': {
            'template': 'base_template.xml',
            'include_assets': True,
            'pretty_print': True
        },
        'debug': {
            'enabled': False,
            'save_intermediate': False,
            'log_level': 'INFO'
        },
        # 根节点选择策略
        'root_node_strategy': 'center',  # "center", "max_degree", "manual"
        'manual_root_node': None  # 当策略为 "manual" 时使用
    }
    
    # 合并默认值
    merged_config = defaults.copy()
    merged_config.update(config_dict)
    
    # 创建配置对象
    return ConversionConfig(
        output_dir=merged_config.get('output_dir', ''),
        mesh_quality=merged_config.get('mesh_quality', 'medium'),
        create_timestamp_dir=merged_config.get('create_timestamp_dir', True),
        debug_mode=merged_config.get('debug', {}).get('enabled', False),
        validate_output=merged_config.get('validate_output', True),
        cleanup_temp_files=merged_config.get('cleanup_temp_files', False),
        root_node_strategy=merged_config.get('root_node_strategy', 'center'),
        manual_root_node=merged_config.get('manual_root_node', None)
    )


def load_config_from_file(file_path: Path) -> ConversionConfig:
    """从文件加载配置"""
    with open(file_path, 'r', encoding='utf-8') as f:
        config_dict = json.load(f)
    return create_config_from_dict(config_dict)


def save_config_to_file(config: ConversionConfig, file_path: Path):
    """保存配置到文件"""
    config_dict = config.to_dict()
    with open(file_path, 'w', encoding='utf-8') as f:
        json.dump(config_dict, f, indent=2, ensure_ascii=False)