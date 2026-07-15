"""
简单的日志记录模块

为Fusion 360插件提供基本的日志功能，避免过度封装。
日志文件与导出数据保存在同一目录中。
"""

import os
import logging
import time
from typing import Optional


def setup_logger(output_dir: str, log_filename: str = "f3d_export.log") -> logging.Logger:
    """设置日志记录器
    
    Args:
        output_dir: 输出目录
        log_filename: 日志文件名
        
    Returns:
        配置好的Logger实例
    """
    # 确保目录存在
    os.makedirs(output_dir, exist_ok=True)
    
    # 创建logger
    logger = logging.getLogger("F3DMaojoco")
    logger.setLevel(logging.DEBUG)
    
    # 清除现有处理器
    logger.handlers.clear()
    
    # 文件处理器
    log_path = os.path.join(output_dir, log_filename)
    file_handler = logging.FileHandler(log_path, encoding='utf-8')
    file_handler.setLevel(logging.DEBUG)
    
    # 控制台处理器（输出到Fusion 360文本命令窗口）
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    
    # 简单的格式
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    file_handler.setFormatter(formatter)
    console_handler.setFormatter(formatter)
    
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)
    
    logger.info(f"日志系统启动 - 输出目录: {output_dir}")
    return logger


def log_performance_start(logger: logging.Logger, operation: str) -> float:
    """开始性能计时
    
    Args:
        logger: Logger实例
        operation: 操作名称
        
    Returns:
        开始时间戳
    """
    logger.info(f"开始: {operation}")
    return time.time()


def log_performance_end(logger: logging.Logger, operation: str, start_time: float):
    """结束性能计时
    
    Args:
        logger: Logger实例
        operation: 操作名称
        start_time: 开始时间戳
    """
    duration = time.time() - start_time
    logger.info(f"完成: {operation} (耗时: {duration:.3f}秒)")


def log_progress(logger: logging.Logger, current: int, total: int, item_name: str = ""):
    """记录进度
    
    Args:
        logger: Logger实例
        current: 当前进度
        total: 总数
        item_name: 项目名称
    """
    if total > 0:
        percent = (current / total) * 100
        item_info = f" - {item_name}" if item_name else ""
        logger.info(f"进度: {current}/{total} ({percent:.1f}%){item_info}")


def log_component(logger: logging.Logger, component_name: str, action: str = "处理"):
    """记录零部件操作
    
    Args:
        logger: Logger实例
        component_name: 零部件名称
        action: 操作类型
    """
    logger.info(f"{action}零部件: {component_name}")


def log_joint(logger: logging.Logger, joint_name: str, joint_type: str):
    """记录关节信息
    
    Args:
        logger: Logger实例
        joint_name: 关节名称
        joint_type: 关节类型
    """
    logger.info(f"关节: {joint_name} (类型: {joint_type})")


def log_transform(logger: logging.Logger, name: str, transform_data):
    """记录变换信息（调试级别）
    
    Args:
        logger: Logger实例
        name: 变换名称
        transform_data: 变换数据
    """
    logger.debug(f"变换矩阵 - {name}: {type(transform_data).__name__}")


def log_error(logger: logging.Logger, error: Exception, context: str = ""):
    """记录错误信息
    
    Args:
        logger: Logger实例
        error: 异常对象
        context: 错误上下文
    """
    context_info = f" - {context}" if context else ""
    logger.error(f"错误{context_info}: {str(error)}", exc_info=True)


def log_warning(logger: logging.Logger, message: str):
    """记录警告信息
    
    Args:
        logger: Logger实例
        message: 警告消息
    """
    logger.warning(f"警告: {message}")


def log_debug(logger: logging.Logger, message: str):
    """记录调试信息
    
    Args:
        logger: Logger实例
        message: 调试消息
    """
    logger.debug(f"调试: {message}")


def log_info(logger: logging.Logger, message: str):
    """记录一般信息
    
    Args:
        logger: Logger实例
        message: 信息内容
    """
    logger.info(message)


def close_logger(logger: logging.Logger):
    """关闭日志记录器
    
    Args:
        logger: Logger实例
    """
    logger.info("日志系统关闭")
    for handler in logger.handlers[:]:
        handler.close()
        logger.removeHandler(handler)


# 全局logger实例
_global_logger: Optional[logging.Logger] = None


def get_logger(output_dir: str = None) -> logging.Logger:
    """获取全局logger实例
    
    Args:
        output_dir: 输出目录（首次调用时需要）
        
    Returns:
        Logger实例
    """
    global _global_logger
    
    if _global_logger is None:
        if output_dir is None:
            raise ValueError("首次调用必须提供output_dir参数")
        _global_logger = setup_logger(output_dir)
    
    return _global_logger


def initialize_logging(output_dir: str):
    """初始化全局日志系统
    
    Args:
        output_dir: 输出目录
        
    Returns:
        logging.Logger: 初始化的logger实例
    """
    global _global_logger
    _global_logger = setup_logger(output_dir)
    return _global_logger


def cleanup_logging():
    """清理全局日志系统"""
    global _global_logger
    if _global_logger:
        close_logger(_global_logger)
        _global_logger = None