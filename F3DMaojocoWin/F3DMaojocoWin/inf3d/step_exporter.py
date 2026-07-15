"""
STEP导出器

将零部件几何导出为STEP文件（B-Rep精确曲面，非网格）。
与STLExporter并行运行，作为附加产物；失败不影响STL/JSON主流程。

注意：Fusion 360 的 createSTEPExportOptions 参数顺序为 (filename, geometry)，
与 createSTLExportOptions 的 (geometry, filename) 相反。
"""

import os
from typing import List, Optional

import adsk.core
import adsk.fusion

from inf3d.logger import log_progress, log_component, log_error
from common.data_types import ComponentInfo


class STEPExporter:
    """STEP导出器

    将零部件几何导出为STEP文件：
    - 逐个零部件导出（B-Rep精确曲面）
    - 文件路径管理和命名
    - 导出进度跟踪
    - 单个失败仅警告，不影响其他零部件
    """

    def __init__(self, logger):
        """初始化STEP导出器

        Args:
            logger: 日志记录器
        """
        self.logger = logger
        self.app = adsk.core.Application.get()
        self.design = self.app.activeProduct

    def export_components(self, components: List[ComponentInfo], output_dir: str) -> List[str]:
        """导出所有零部件的STEP文件

        Args:
            components: 零部件信息列表
            output_dir: 输出目录

        Returns:
            List[str]: 成功导出的STEP文件路径列表
        """
        self.logger.info("开始导出STEP文件")

        # 创建STEP文件目录
        step_dir = os.path.join(output_dir, "step_files")
        os.makedirs(step_dir, exist_ok=True)

        # 导出的文件列表
        exported_files = []

        # 遍历所有零部件
        for i, component in enumerate(components):
            try:
                # 碰撞体（COL 前缀，含 COL_ 和裸 COL）不导出 STEP
                if component.name.startswith("COL"):
                    continue

                log_progress(self.logger, i + 1, len(components), f"导出STEP {component.name}")

                # 导出单个零部件
                step_file = self._export_single_component(component, step_dir)

                if step_file:
                    exported_files.append(step_file)
                    log_component(self.logger, component.name, "STEP导出成功")
                else:
                    log_component(self.logger, component.name, "STEP导出失败")

            except Exception as e:
                # 单个失败仅警告，继续其他零部件
                log_error(self.logger, e, f"导出零部件 {component.name} STEP")
                continue

        self.logger.info(f"STEP导出完成，成功导出 {len(exported_files)} 个文件")
        return exported_files

    def _export_single_component(self, component: ComponentInfo, step_dir: str) -> Optional[str]:
        """导出单个零部件的STEP文件

        Args:
            component: 零部件信息
            step_dir: STEP文件目录

        Returns:
            Optional[str]: 导出的STEP文件路径
        """
        try:
            # 生成安全的文件名
            safe_name = self._get_safe_filename(component.name)
            step_filename = f"{safe_name}_{component.component_id}.step"
            step_filepath = os.path.join(step_dir, step_filename)

            # 查找对应的 Fusion 360 组件
            fusion_component = self._find_fusion_component(component)
            if not fusion_component:
                self.logger.warning(f"未找到零部件 {component.name} 对应的 Fusion 360 组件")
                return None

            # 创建导出管理器
            export_manager = self.design.exportManager

            # 创建STEP导出选项
            # 注意：STEP的参数顺序是 (filename, geometry)，与STL的 (geometry, filename) 相反
            step_options = export_manager.createSTEPExportOptions(step_filepath, fusion_component)

            # 执行导出
            export_manager.execute(step_options)

            # 检查文件是否成功创建
            if os.path.exists(step_filepath):
                return step_filepath
            else:
                self.logger.warning(f"STEP文件未创建: {step_filepath}")
                return None

        except Exception as e:
            log_error(self.logger, e, f"导出单个零部件STEP {component.name}")
            return None

    def _find_fusion_component(self, component: ComponentInfo) -> Optional[adsk.fusion.Component]:
        """查找对应的 Fusion 360 组件

        Args:
            component: 零部件信息

        Returns:
            Optional[adsk.fusion.Component]: Fusion 360 组件
        """
        try:
            # 在根组件的所有 occurrences 中查找
            root_component = self.design.rootComponent

            # 遍历所有 occurrences
            for occurrence in root_component.allOccurrences:
                if occurrence.component and occurrence.component.name == component.component_id:
                    return occurrence.component

            # 如果在 occurrences 中没找到，尝试在所有组件中查找
            for fusion_component in self.design.allComponents:
                if fusion_component.name == component.component_id:
                    return fusion_component

            return None

        except Exception as e:
            log_error(self.logger, e, f"查找 Fusion 360 组件 {component.name}")
            return None

    def _get_safe_filename(self, name: str) -> str:
        """生成安全的文件名

        Args:
            name: 原始名称

        Returns:
            str: 安全的文件名
        """
        # 替换不安全的字符
        unsafe_chars = '<>:"/\\|?*'
        safe_name = name

        for char in unsafe_chars:
            safe_name = safe_name.replace(char, '_')

        # 限制长度
        if len(safe_name) > 100:
            safe_name = safe_name[:100]

        return safe_name
