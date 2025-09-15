"""
Phase 框架

为每个转换步骤提供统一的接口和数据封装。
"""

from typing import Dict, Any, Optional

from .base import ConversionPhase
from .data_loading import DataLoadingPhase
from .relationship_analysis import RelationshipAnalysisPhase
from .unit_conversion import UnitConversionPhase
from .model_generation import ModelGenerationPhase
from .actuator_generation import ActuatorGenerationPhase

from ..utils.logger import logger
from ..context import MaojocoContext
from ..type_definitions import PhaseResult, PhaseStatus


class PhaseManager:
    """阶段管理器"""
    
    def __init__(self, ctx: MaojocoContext):
        self.ctx = ctx
        self.phases: Dict[str, ConversionPhase] = {}
        self._initialize_phases()
    
    def _initialize_phases(self):
        """初始化所有阶段"""
        self.phases = {
            'data_loading': DataLoadingPhase(self.ctx),
            'relationship_analysis': RelationshipAnalysisPhase(self.ctx),
            'unit_conversion': UnitConversionPhase(self.ctx),
            'model_generation': ModelGenerationPhase(self.ctx),
            'actuator_generation': ActuatorGenerationPhase(self.ctx)
        }
    
    def execute_phase(self, phase_name: str) -> bool:
        """执行指定阶段"""
        if phase_name not in self.phases:
            logger.error(f"❌ 未知阶段: {phase_name}")
            return False
        
        phase = self.phases[phase_name]
        return phase.execute()
    
    def execute_all(self) -> bool:
        """执行所有阶段"""
        logger.info("🚀 开始执行所有转换阶段")
        
        all_success = True
        for _, phase in self.phases.items():
            if not phase.execute():
                all_success = False
                break
        
        if all_success:
            logger.success("✅ 所有阶段执行完成")
        else:
            logger.error("❌ 部分阶段执行失败")
        
        return all_success
    
    def get_phase_status(self, phase_name: str) -> Optional[PhaseResult]:
        """获取指定阶段状态"""
        if phase_name not in self.phases:
            return None
        
        return self.phases[phase_name].get_status()
    
    def get_all_status(self) -> Dict[str, PhaseResult]:
        """获取所有阶段状态"""
        return {name: phase.get_status() for name, phase in self.phases.items()}