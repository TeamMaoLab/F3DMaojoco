"""BRep 网格导出器 — 从整个装配体生成合并的 brep_geometry.json。

遍历装配体（复用 component_collector 的 occurrence 遍历逻辑），
对每个有实体的零件用 BRepMeshExtractor 网格化，输出与 geometry.json
完全兼容的格式（positions + indices）。网页端可直接加载。

输出: <output_dir>/brep_geometry.json
{
  "_meta": {...},
  "parts": {
    "stl_files/零件名.stl": {  # key 与 component_positions.json 的 stl_file 对齐
      "positions": [...], "indices": [...]
    }
  }
}
"""
import os
import json
from typing import List

import adsk.core
import adsk.fusion

from .logger import log_progress, log_error, log_info
from .brep_mesh_extractor import BRepMeshExtractor


COLLIDER_PREFIX = "COL_"


class BRepMeshExporter:
    """遍历装配体，导出所有零件的 BRep 网格到单一 JSON。"""

    def __init__(self, logger, angle_tolerance: float = 5.0, max_edge_length: float = 0.5):
        self.logger = logger
        self.app = adsk.core.Application.get()
        self.design = self.app.activeProduct
        self._extractor = BRepMeshExtractor(
            logger, angle_tolerance=angle_tolerance, max_edge_length=max_edge_length
        )

    def export_assembly(self, output_dir: str) -> str:
        """导出整个装配体的 BRep 网格。

        Returns:
            输出文件路径，失败返回空串
        """
        try:
            if not self.design:
                log_error(self.logger, "BRep 网格导出: 无活动设计")
                return ""
            root = self.design.rootComponent
            if not root:
                return ""

            parts = {}
            processed = set()
            count = 0

            def visit(occ: adsk.fusion.Occurrence):
                nonlocal count
                comp = occ.component
                if not comp:
                    return
                # 跳过碰撞体
                if comp.name.startswith(COLLIDER_PREFIX):
                    return
                # 跳过容器（无实体）
                if comp.bRepBodies.count == 0:
                    for child in occ.childOccurrences:
                        visit(child)
                    return

                comp_key = id(comp)
                # 用 occurrence 名作 key（与 STL 文件名对齐）
                # STL 命名规则：component_name + "_" + component_name + ".stl"
                # 这里用 occurrence 的 component.name 生成 key
                stl_key = self._stl_key_for(occ)

                if stl_key and stl_key not in parts:
                    mesh = self._extractor.extract_mesh(occ)
                    if mesh['vertex_count'] > 0:
                        parts[stl_key] = {
                            'positions': mesh['positions'],
                            'indices': mesh['indices'],
                        }
                        count += 1
                        log_progress(self.logger, count, 0, f"BRep网格 {comp.name}")

                # 递归子
                for child in occ.childOccurrences:
                    visit(child)

            for occ in root.occurrences:
                visit(occ)

            # root 自己的 body（如果有）
            if root.bRepBodies.count > 0:
                # root 通常没有零件实体，略过
                pass

            out = {
                '_meta': {
                    'description': 'BRep 精确网格（Fusion MeshManager tessellate）',
                    'angle_tolerance_deg': self._extractor.angle_tolerance,
                    'max_edge_length_cm': self._extractor.max_edge_length,
                    'unit': 'mm',
                    'source': 'brep',
                },
                'parts': parts,
            }

            os.makedirs(output_dir, exist_ok=True)
            out_path = os.path.join(output_dir, 'brep_geometry.json')
            with open(out_path, 'w', encoding='utf-8') as f:
                json.dump(out, f, ensure_ascii=False, separators=(',', ':'))

            total_verts = sum(len(p['positions']) // 3 for p in parts.values())
            total_faces = sum(len(p['indices']) // 3 for p in parts.values())
            file_kb = os.path.getsize(out_path) / 1024
            log_info(self.logger, f"BRep 网格导出完成: {count} 零件, "
                     f"{total_verts} 顶点, {total_faces} 面, {file_kb:.0f}KB → {out_path}")
            return out_path

        except Exception as e:
            log_error(self.logger, f"BRep 网格导出失败: {e}")
            return ""

    def _stl_key_for(self, occ: adsk.fusion.Occurrence) -> str:
        """生成与 component_positions.json 里 stl_file 字段对齐的 key。

        STL 命名规则（见 stl_exporter）: "stl_files/" + comp_name + "_" + comp_name + ".stl"
        多个同名 occurrence 共享一个 STL，这里也共享一个 BRep 网格。
        """
        comp = occ.component
        if not comp or not comp.name:
            return ""
        name = comp.name
        return f"stl_files/{name}_{name}.stl"
