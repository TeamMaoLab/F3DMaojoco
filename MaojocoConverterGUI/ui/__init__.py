"""
UI组件模块

包含简化版GUI的所有UI组件，采用2层架构：GUI层 → Core层。
"""

from .main_window import create_simplified_application, MainWindowSimplified
from .stage_panels import (
    InitializationPanel, 
    DataLoadingPanel, 
    SimpleStagePanel, 
    StagePanelsContainer
)
from .visualization_widget import VisualizationWidget

__all__ = [
    'create_simplified_application',
    'MainWindowSimplified', 
    'InitializationPanel',
    'DataLoadingPanel', 
    'SimpleStagePanel',
    'StagePanelsContainer',
    'VisualizationWidget'
]