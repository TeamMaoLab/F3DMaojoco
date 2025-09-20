"""
日志工具模块

提供统一的日志接口，集成MaojocoConverter的日志系统。
"""

from typing import Optional
import sys
from pathlib import Path

# 添加项目根目录到sys.path，用于导入MaojocoConverter的logger
project_root = Path(__file__).parent.parent.parent
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

try:
    # 导入MaojocoConverter的logger
    from MaojocoConverter.utils.logger import logger as maojoco_logger
    
    # 创建我们的logger实例
    class Logger:
        """GUI日志记录器
        
        封装MaojocoConverter的logger，提供统一的日志接口。
        """
        
        def debug(self, message: str) -> None:
            """调试信息"""
            maojoco_logger.debug(message)
        
        def info(self, message: str) -> None:
            """一般信息"""
            maojoco_logger.info(message)
        
        def warning(self, message: str) -> None:
            """警告信息"""
            maojoco_logger.warning(message)
        
        def error(self, message: str) -> None:
            """错误信息"""
            maojoco_logger.error_msg(message)
        
        def success(self, message: str) -> None:
            """成功信息"""
            maojoco_logger.success(message)
    
    # 创建全局logger实例
    logger = Logger()
    
except ImportError as e:
    print(f"⚠️  无法导入MaojocoConverter logger: {e}")
    print("💡 请确保在F3DMaojoco项目根目录下运行GUI程序")
    
    # 创建备用logger
    class FallbackLogger:
        """备用日志记录器
        
        当无法导入MaojocoConverter logger时使用。
        """
        
        def debug(self, message: str) -> None:
            print(f"DEBUG: {message}")
        
        def info(self, message: str) -> None:
            print(f"INFO: {message}")
        
        def warning(self, message: str) -> None:
            print(f"WARNING: {message}")
        
        def error(self, message: str) -> None:
            print(f"ERROR: {message}")
        
        def success(self, message: str) -> None:
            print(f"SUCCESS: {message}")
    
    logger = FallbackLogger()


__all__ = ["logger"]