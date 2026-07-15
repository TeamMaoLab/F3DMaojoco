"""BRep 网格导出器 — 从整个装配体生成合并的 brep_geometry.json（自包含，不依赖 extractor）。

遍历装配体，对每个有实体的零件用 Fusion MeshManager 网格化，
输出与 geometry.json 兼容的格式（positions + indices）。网页端可直接加载。

精度参数通过 Fusion 的 displayMetrics / quality 设置，比 STL 默认更精细。
"""
import os
import json
import math

import adsk.core
import adsk.fusion

from .logger import log_progress, log_error, log_info

COLLIDER_PREFIX = "COL_"
MM = 10.0  # Fusion 内部 cm → mm


def _accumulate_transform(occ):
    """累加 occurrence 及祖先 transform → 世界变换矩阵。"""
    try:
        chain = []
        o = occ
        while o is not None:
            t = o.transform
            if t is not None:
                chain.append(t)
            o = o.assemblyContext
        if not chain:
            return None
        world = chain[-1].copy()
        for i in range(len(chain) - 2, -1, -1):
            world.transformBy(chain[i])
        return world
    except Exception:
        return None


class BRepMeshExporter:
    """遍历装配体，导出所有零件的 BRep 网格到单一 JSON。"""

    def __init__(self, logger, angle_tolerance=1.0, max_edge_length=0.1):
        """初始化。

        Args:
            logger: 日志记录器
            angle_tolerance: 角度容差（度，越小越精细，默认1°）
            max_edge_length: 最大边长（cm，越小越精细，默认0.1cm=1mm）
        """
        self.logger = logger
        self.app = adsk.core.Application.get()
        self.design = self.app.activeProduct
        self.angle_tolerance = angle_tolerance
        self.max_edge_length = max_edge_length
        self._diag_logged = False  # 只打一次诊断

    def export_assembly(self, output_dir: str) -> str:
        """导出整个装配体的 BRep 网格。"""
        try:
            if not self.design:
                log_error(self.logger, "BRep 网格导出: 无活动设计")
                return ""
            root = self.design.rootComponent
            if not root:
                return ""

            parts = {}
            count = 0

            def visit(occ):
                nonlocal count
                comp = occ.component
                if not comp:
                    return
                if comp.name.startswith(COLLIDER_PREFIX):
                    return
                if comp.bRepBodies.count == 0:
                    for child in occ.childOccurrences:
                        visit(child)
                    return

                stl_key = self._stl_key_for(occ)
                if stl_key and stl_key not in parts:
                    mesh_data = self._tessellate_occurrence(occ)
                    if mesh_data and mesh_data['vertex_count'] > 0:
                        parts[stl_key] = {
                            'positions': mesh_data['positions'],
                            'indices': mesh_data['indices'],
                        }
                        count += 1
                        log_progress(self.logger, count, 0,
                                     f"BRep网格 {comp.name}: {mesh_data['vertex_count']}顶点")

                for child in occ.childOccurrences:
                    visit(child)

            for occ in root.occurrences:
                visit(occ)

            out = {
                '_meta': {
                    'description': 'BRep 精确网格（Fusion MeshManager tessellate）',
                    'angle_tolerance_deg': self.angle_tolerance,
                    'max_edge_length_cm': self.max_edge_length,
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

    def _tessellate_occurrence(self, occ):
        """网格化一个 occurrence 的所有 body。返回 {positions, indices, vertex_count}。"""
        positions = []
        indices = []
        # 入口诊断：确认这个方法被执行了 + 精度参数值
        log_info(self.logger, "[BRep诊断] _tessellate_occurrence 被调用: %s, angle=%s, edge=%s" %
                 (occ.name, self.angle_tolerance, self.max_edge_length))
        try:
            comp = occ.component
            world_transform = _accumulate_transform(occ)
            vert_offset = 0

            for body_idx in range(comp.bRepBodies.count):
                body = comp.bRepBodies.item(body_idx)
                mesh = self._tessellate_body(body)
                if mesh is None:
                    continue

                coords = mesh.nodeCoordinates
                for p in coords:
                    wp = p
                    if world_transform is not None:
                        wp = p.copy()
                        wp.transformBy(world_transform)
                    positions.extend([
                        round(wp.x * MM, 4),
                        round(wp.y * MM, 4),
                        round(wp.z * MM, 4),
                    ])

                idx = mesh.nodeIndices
                for i in idx:
                    indices.append(int(i) + vert_offset)

                vert_offset += len(coords)

        except Exception as e:
            log_error(self.logger, f"网格化失败 {occ.name}: {e}")

        return {'positions': positions, 'indices': indices,
                'vertex_count': len(positions) // 3}

    def _tessellate_body(self, body):
        """用 MeshManager 网格化一个 body，设置精度参数。返回 TriangleMesh 或 None。"""
        try:
            mm = body.meshManager
            if mm is None:
                return None
            calc = mm.createMeshCalculator()
            if calc is None:
                return None

            # 诊断：只第一个零件打一次
            if not self._diag_logged:
                import sys
                self.logger.info("[PY诊断] Python解释器: " + sys.executable)
                self.logger.info("[PY诊断] Python版本: " + sys.version)
                self.logger.info("[PY诊断] MeshCalculator objectType=" + str(calc.objectType))
                # 列出 calculator 所有可用属性
                all_attrs = [a for a in dir(calc) if not a.startswith('_')]
                self.logger.info("[PY诊断] Calculator属性: " + str(all_attrs))
                # 读当前精度值（设之前）
                for a in ['angleTolerance', 'maxEdgeLength', 'surfaceTolerance', 'aspectRatio']:
                    try:
                        self.logger.info("[PY诊断]   读 %s = %s" % (a, getattr(calc, a)))
                    except Exception as e:
                        self.logger.info("[PY诊断]   读 %s 失败: %s" % (a, e))
                self._diag_logged = True

            # 设置精度：先用 setQuality 设基准（枚举 8/11/13/15，越大越精细），
            # 再手动覆盖 surfaceTolerance（cm，越小越精细）做额外控制。
            # API 文档（adsk/defs/adsk/fusion.py TriangleMeshCalculator）：
            #   surfaceTolerance: 网格偏离真实曲面的最大距离（cm）
            #   maxNormalDeviation: 相邻顶点法线最大角度（弧度，0=不限）
            #   maxSideLength: 三角形最大边长（cm，0=不限）
            #   setQuality: 用 LOD 枚举一次性调好所有参数
            try:
                # 15 = VeryHighQualityTriangleMesh（最精细枚举档）
                ok = calc.setQuality(15)
                if not self._diag_logged:
                    self.logger.info("[PY诊断] setQuality(15) 返回: " + str(ok))
            except Exception as e:
                if not self._diag_logged:
                    self.logger.info("[PY诊断] setQuality 失败: " + str(e))
            try:
                # 额外精细：surfaceTolerance 0.001cm = 0.01mm（极精细）
                calc.surfaceTolerance = 0.001
            except Exception:
                pass

            mesh = calc.calculate()
            if mesh is None:
                mesh = calc.mesh
            return mesh
        except Exception as e:
            log_error(self.logger, f"_tessellate_body 失败: {e}")
            return None

    def _stl_key_for(self, occ) -> str:
        """生成与 component_positions.json 的 stl_file 对齐的 key。"""
        comp = occ.component
        if not comp or not comp.name:
            return ""
        name = comp.name
        return f"stl_files/{name}_{name}.stl"
