"""
ODogExample 核心模块

包含机器人模型、关节映射、绑定管理、姿态管理等核心功能。
"""

from .robot_model import RobotModel, create_test_model

__all__ = ['RobotModel', 'create_test_model']

__version__ = "0.1.0"