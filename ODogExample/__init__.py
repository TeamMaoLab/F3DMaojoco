"""
ODogExample - 8自由度四足机器狗开发平台

这个包包含了完整的GUI应用、机器人模型控制和仿真功能。
"""

__version__ = "1.0.0"
__author__ = "TeamMaoLab"
__description__ = "8自由度四足机器狗GUI开发平台"

# 导出主要接口
from .gui.app_entry import main
from .gui.app_main import MainApplication

__all__ = ['main', 'MainApplication']