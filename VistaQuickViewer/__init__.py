"""
VistaQuickViewer - F3DMaojoco快速查看器

用于快速查看F3DMaojocoScripts执行结果的3D可视化工具。
基于PyVista实现，可以直接调用F3DMaojocoScripts/common模块。
"""

from .viewer import VistaQuickViewer, quick_view
from .data_loader import ExportDataLoader

__version__ = "0.1.0"
__author__ = "F3DMaojoco Team"

__all__ = [
    'VistaQuickViewer',
    'quick_view',
    'ExportDataLoader'
]

# 导入主函数以便作为模块运行
from .__main__ import main

# 当作为脚本运行时，调用主函数
if __name__ == "__main__":
    main()