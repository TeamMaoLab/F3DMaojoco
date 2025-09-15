"""
日志工具
"""

import logging
import sys
from pathlib import Path
from typing import Optional
from datetime import datetime


class Logger:
    """日志管理器"""
    
    def __init__(self, name: str = "MaojocoConverter", level: str = "INFO"):
        self.logger = logging.getLogger(name)
        self.logger.setLevel(getattr(logging, level.upper()))
        
        # 避免重复添加处理器
        if not self.logger.handlers:
            self._setup_handlers()
    
    def _setup_handlers(self):
        """设置日志处理器"""
        # 控制台处理器
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setLevel(logging.INFO)
        console_formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        console_handler.setFormatter(console_formatter)
        self.logger.addHandler(console_handler)
        
        # 文件处理器（可选）
        # file_handler = logging.FileHandler('maojoco_converter.log')
        # file_handler.setLevel(logging.DEBUG)
        # file_formatter = logging.Formatter(
        #     '%(asctime)s - %(name)s - %(levelname)s - %(funcName)s:%(lineno)d - %(message)s'
        # )
        # file_handler.setFormatter(file_formatter)
        # self.logger.addHandler(file_handler)
    
    def info(self, message: str, **kwargs):
        """记录信息日志"""
        self.logger.info(message, **kwargs)
    
    def debug(self, message: str, **kwargs):
        """记录调试日志"""
        self.logger.debug(message, **kwargs)
    
    def warning(self, message: str, **kwargs):
        """记录警告日志"""
        self.logger.warning(message, **kwargs)
    
    def error(self, message: str, **kwargs):
        """记录错误日志"""
        self.logger.error(message, **kwargs)
    
    def critical(self, message: str, **kwargs):
        """记录严重错误日志"""
        self.logger.critical(message, **kwargs)
    
    def stage(self, stage_name: str, message: str):
        """记录阶段日志"""
        self.info(f"[{stage_name}] {message}")
    
    def success(self, message: str):
        """记录成功日志"""
        self.info(f"✅ {message}")
    
    def error_msg(self, message: str):
        """记录错误消息"""
        self.error(f"❌ {message}")


# 全局日志实例
logger = Logger("MaojocoConverter")