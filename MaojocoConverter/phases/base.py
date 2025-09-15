"""
转换阶段基类

定义所有转换阶段的统一接口和基础功能。
"""

from abc import ABC, abstractmethod
from typing import Dict, Any, Optional
from datetime import datetime

from ..utils.logger import logger
from ..context import MaojocoContext
from ..type_definitions import PhaseResult, PhaseStatus


class ConversionPhase(ABC):
    """转换阶段基类
    
    每个转换步骤都是一个独立的 Phase 对象，负责：
    1. 处理特定的数据转换逻辑
    2. 验证输入数据
    3. 生成输出数据
    4. 记录处理过程
    """
    
    def __init__(self, name: str, ctx: MaojocoContext) -> None:
        """
        初始化转换阶段
        
        Args:
            name: 阶段名称
            ctx: 转换上下文
        """
        self.name = name
        self.ctx = ctx
        self.start_time: Optional[datetime] = None
        self.end_time: Optional[datetime] = None
        self.duration: Optional[float] = None
        self.status: PhaseStatus = PhaseStatus.PENDING
        
    def execute(self) -> bool:
        """
        执行转换阶段
        
        Returns:
            bool: 执行是否成功
        """
        try:
            logger.info(f"🔄 开始执行阶段: {self.name}")
            self.start_time = datetime.now()
            self.status = PhaseStatus.RUNNING
            
            # 验证前置条件
            if not self._validate_prerequisites():
                logger.error(f"❌ 前置条件验证失败: {self.name}")
                self.status = PhaseStatus.FAILED
                return False
            
            # 执行具体逻辑
            result = self._execute()
            
            self.end_time = datetime.now()
            self.duration = (self.end_time - self.start_time).total_seconds()
            self.status = PhaseStatus.COMPLETED if result else PhaseStatus.FAILED
            
            if result:
                logger.success(f"✅ 阶段完成: {self.name} (耗时: {self.duration:.3f}s)")
            else:
                logger.error(f"❌ 阶段失败: {self.name}")
            
            return result
            
        except Exception as e:
            logger.error(f"❌ 阶段执行异常: {self.name} - {e}")
            self.status = PhaseStatus.FAILED
            return False
    
    @abstractmethod
    def _execute(self) -> bool:
        """
        执行具体的转换逻辑
        
        Returns:
            bool: 执行是否成功
        """
        pass
    
    def _validate_prerequisites(self) -> bool:
        """
        验证前置条件
        
        Returns:
            bool: 前置条件是否满足
        """
        # 默认实现：检查必要的文件
        if hasattr(self, '_required_files'):
            for file_name in self._required_files:
                file_path = self.ctx.export_dir / file_name
                if not file_path.exists():
                    logger.error(f"❌ 必要文件不存在: {file_path}")
                    return False
        return True
    
    def get_status(self) -> PhaseResult:
        """
        获取阶段状态
        
        Returns:
            PhaseResult: 阶段状态信息
        """
        return PhaseResult(
            phase_name=self.name,
            status=self.status,
            duration=self.duration or 0.0,
            start_time=self.start_time.isoformat() if self.start_time else None,
            end_time=self.end_time.isoformat() if self.end_time else None,
            data_processed=self._get_processed_data()
        )
    
    def _get_processed_data(self) -> Optional[Dict[str, Any]]:
        """
        获取处理的数据（子类可重写）
        
        Returns:
            Optional[Dict[str, Any]]: 处理的数据统计
        """
        return None