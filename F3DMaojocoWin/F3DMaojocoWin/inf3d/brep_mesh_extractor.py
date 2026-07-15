"""BRep 网格化提取器 — 从 Fusion BRep 精确曲面生成三角网格。

与 STL 导出的区别：STL 用 Fusion 的导出管线（固定质量），这里用 MeshManager
直接读 BRep 的精确网格，可自定义质量（角度/边长容差），精度可控。

Fusion API 路径:
  body.meshManager → createMeshCalculator() → setQuality/手动设参 → mesh
  mesh.nodeCoordinates: Point3D[]（cm，body 局部）
  mesh.nodeIndices:     int[]（每 3 个 = 一个三角面）
  mesh.normalVectors:   Vector3D[]（每个顶点的法线）

trim 自动处理：MeshManager 生成的网格已经是 trim 后的实际面形状，
不需要手动处理 edge loop。

输出与 geometry.json 兼容：{positions: [x,y,z,...], indices: [i,j,k,...]}（mm，世界坐标）。
"""

from typing import Dict, Any, List, Optional

import adsk.core
import adsk.fusion

from inf3d.logger import log_component, log_error

MM = 10.0  # Fusion 内部 cm → mm


def _accumulate_transform(occurrence: adsk.fusion.Occurrence) -> Optional['adsk.core.Matrix3D']:
    """累加 occurrence 及祖先 transform → 世界变换矩阵。"""
    try:
        chain = []
        occ = occurrence
        while occ is not None:
            t = occ.transform
            if t is not None:
                chain.append(t)
            occ = occ.assemblyContext
        if not chain:
            return None
        world = chain[-1].copy()
        for i in range(len(chain) - 2, -1, -1):
            world.transformBy(chain[i])
        return world
    except Exception:
        return None


class BRepMeshExtractor:
    """BRep 网格化提取器：用 Fusion MeshManager 把零件网格化。

    可配置网格质量（角度容差），比 STL 导出默认值更精细。
    输出与 geometry.json 兼容的 {positions, indices}。
    """

    def __init__(self, logger, angle_tolerance: float = 1.0,
                 max_edge_length: float = 0.1):
        """初始化。

        Args:
            logger: 日志记录器
            angle_tolerance: 角度容差（度，越小越精细，默认 1°，圆柱面每段≤1°≈360段，很圆）
            max_edge_length: 最大边长（cm，越小越精细，默认 0.1cm=1mm）
        """
        self.logger = logger
        self.angle_tolerance = angle_tolerance
        self.max_edge_length = max_edge_length

    def extract_mesh(self, occurrence: adsk.fusion.Occurrence) -> Dict[str, Any]:
        """提取一个 occurrence 的 BRep 网格。

        Args:
            occurrence: Fusion occurrence（有实体的零件）

        Returns:
            {positions: [...], indices: [...], face_count, vertex_count}
            或失败返回空 {positions: [], indices: []}
        """
        result = {'positions': [], 'indices': [], 'face_count': 0, 'vertex_count': 0}
        try:
            component = occurrence.component
            if not component or component.bRepBodies.count == 0:
                return result

            world_transform = _accumulate_transform(occurrence)

            all_positions: List[float] = []
            all_indices: List[int] = []
            vert_offset = 0
            total_faces = 0

            for body_idx in range(component.bRepBodies.count):
                body = component.bRepBodies.item(body_idx)
                mesh = self._tessellate_body(body)
                if mesh is None:
                    continue

                # 读顶点（cm → mm + 世界变换）
                coords = mesh.nodeCoordinates
                for p in coords:
                    wp = p
                    if world_transform is not None:
                        wp = p.copy()
                        wp.transformBy(world_transform)
                    all_positions.extend([
                        round(wp.x * MM, 4),
                        round(wp.y * MM, 4),
                        round(wp.z * MM, 4),
                    ])

                # 读索引（三角面）
                idx = mesh.nodeIndices
                for i in idx:
                    all_indices.append(int(i) + vert_offset)

                n_verts = len(coords)
                n_faces = len(idx) // 3
                vert_offset += n_verts
                total_faces += n_faces

            result['positions'] = all_positions
            result['indices'] = all_indices
            result['face_count'] = total_faces
            result['vertex_count'] = vert_offset
            log_component(self.logger, occurrence.name,
                          f"BRep网格: {vert_offset} 顶点, {total_faces} 面")
        except Exception as e:
            log_error(self.logger, f"BRep 网格化失败 {occurrence.name}: {e}")
        return result

    def _tessellate_body(self, body: adsk.fusion.BRepBody):
        """用 MeshManager 网格化一个 body。返回 TriangleMesh 或 None。"""
        try:
            mesh_manager = body.meshManager
            if mesh_manager is None:
                return None
            calc = mesh_manager.createMeshCalculator()
            if calc is None:
                return None

            # [诊断] 打印 calculator 可用的属性/方法，确认 API 接口
            self.logger.info(f"  [BRep诊断] MeshCalculator type={calc.objectType}")
            # 尝试列出属性
            for attr_name in ['angleTolerance', 'maxEdgeLength', 'surfaceTolerance', 'aspectRatio']:
                try:
                    val = getattr(calc, attr_name)
                    self.logger.info(f"  [BRep诊断]   {attr_name} (读) = {val}")
                except Exception as e:
                    self.logger.info(f"  [BRep诊断]   {attr_name} 不可读: {e}")

            # 尝试设置质量参数（多种 API 形式）
            set_ok = []
            # 方式1: 直接属性赋值
            for attr, val in [('angleTolerance', self.angle_tolerance),
                              ('maxEdgeLength', self.max_edge_length)]:
                try:
                    setattr(calc, attr, val)
                    set_ok.append(f'{attr}={val}(属性)')
                except Exception as e:
                    self.logger.info(f"  [BRep诊断]   {attr} 属性赋值失败: {e}")
                    # 方式2: setter 方法
                    setter = 'set' + attr[0].upper() + attr[1:]
                    try:
                        getattr(calc, setter)(val)
                        set_ok.append(f'{attr}={val}(setter)')
                    except Exception as e2:
                        self.logger.info(f"  [BRep诊断]   {setter}() 也失败: {e2}")
            self.logger.info(f"  [BRep诊断] 参数设置结果: {set_ok if set_ok else '全部失败（用默认精度）'}")

            mesh = calc.calculate()
            if mesh is None:
                mesh = calc.mesh
            if mesh is not None:
                self.logger.info(f"  [BRep诊断] 网格生成: {len(mesh.nodeCoordinates)} 顶点, "
                                 f"{len(mesh.nodeIndices)//3} 面")
            return mesh
        except Exception as e:
            log_error(self.logger, f"_tessellate_body 失败: {e}")
            return None
