"""BRep 曲面提取器 — 从 Fusion 零件提取精确曲面定义。

遍历零件每个 body 的所有 BRepFace，提取面的数学曲面参数（Plane/Cylinder/Cone/
Sphere/Torus），输出与零件绑定的曲面列表。这些参数是数学精确的（不是网格近似），
可用于网页端按曲面类型重建几何，替代体积大的 STL 网格。

Fusion API: face.geometry 返回 Surface 对象，objectType 形如 'adsk::core::Cylinder'。
单位：Fusion 内部 cm，这里转 mm（×10）。
"""

from typing import List, Dict, Any, Optional

import adsk.core
import adsk.fusion

from inf3d.logger import log_component, log_error

MM = 10.0  # Fusion 内部 cm → mm


def _get_attr(obj, names):
    """从对象取属性，尝试多个可能的名字（Fusion API 版本兼容）。"""
    for n in names:
        if hasattr(obj, n):
            return getattr(obj, n)
    return None


def _p3d_to_list(p, world_transform):
    """Point3D → [x,y,z] mm，应用世界变换。"""
    if p is None:
        return None
    if world_transform is not None:
        p = p.copy()
        p.transformBy(world_transform)
    return [round(p.x * MM, 4), round(p.y * MM, 4), round(p.z * MM, 4)]


def _v3d_to_list(v, world_transform):
    """Vector3D → [x,y,z] 单位向量，应用世界变换（只旋转）。"""
    if v is None:
        return None
    if world_transform is not None:
        v = v.copy()
        v.transformBy(world_transform)
        v.normalize()
    return [round(v.x, 6), round(v.y, 6), round(v.z, 6)]


def _accumulate_transform(occurrence: adsk.fusion.Occurrence) -> Optional['adsk.core.Matrix3D']:
    """累加 occurrence 及所有祖先的 transform → 世界变换矩阵。

    occurrence.transform 是相对父级的局部变换，要转世界坐标需从根向下累乘。
    """
    try:
        chain = []
        occ = occurrence
        while occ is not None:
            t = occ.transform
            if t is not None:
                chain.append(t)
            occ = occ.assemblyContext  # 父 occurrence（顶级为 None）
        if not chain:
            return None
        # chain[0] 最深（当前），chain[-1] 最浅（根）；世界变换 = 根×...×当前
        world = chain[-1].copy()
        for i in range(len(chain) - 2, -1, -1):
            world.transformBy(chain[i])
        return world
    except Exception:
        return None


def _extract_face(face: adsk.fusion.BRepFace, world_transform) -> Optional[Dict[str, Any]]:
    """提取一个面的曲面参数。返回 dict 或 None（不支持的类型）。"""
    geo = face.geometry
    if geo is None:
        return None
    # 'adsk::core::Cylinder' → 'Cylinder'
    geo_type = geo.objectType.split('::')[-1]

    result: Dict[str, Any] = {'surface': geo_type, 'params': {}}

    if geo_type == 'Plane':
        normal = _get_attr(geo, ['normal', 'axisVector', 'axis'])
        root = _get_attr(geo, ['rootPoint', 'basePoint', 'origin'])
        result['params']['normal'] = _v3d_to_list(normal, world_transform)
        result['params']['point'] = _p3d_to_list(root, world_transform)

    elif geo_type in ('Cylinder', 'EllipticalCylinder'):
        axis = _get_attr(geo, ['axisVector', 'axis'])
        origin = _get_attr(geo, ['basePoint', 'origin'])
        radius = _get_attr(geo, ['radius'])
        result['params']['axis'] = _v3d_to_list(axis, world_transform)
        result['params']['origin'] = _p3d_to_list(origin, world_transform)
        if radius is not None:
            result['params']['radius'] = round(radius * MM, 4)

    elif geo_type in ('Cone', 'EllipticalCone'):
        axis = _get_attr(geo, ['axisVector', 'axis'])
        origin = _get_attr(geo, ['basePoint', 'origin'])
        radius = _get_attr(geo, ['radius'])
        half_angle = _get_attr(geo, ['halfAngle'])
        result['params']['axis'] = _v3d_to_list(axis, world_transform)
        result['params']['origin'] = _p3d_to_list(origin, world_transform)
        if radius is not None:
            result['params']['radius'] = round(radius * MM, 4)
        if half_angle is not None:
            result['params']['halfAngle'] = round(half_angle, 6)

    elif geo_type == 'Sphere':
        center = _get_attr(geo, ['centerPoint', 'center', 'basePoint', 'origin'])
        radius = _get_attr(geo, ['radius'])
        result['params']['center'] = _p3d_to_list(center, world_transform)
        if radius is not None:
            result['params']['radius'] = round(radius * MM, 4)

    elif geo_type == 'Torus':
        center = _get_attr(geo, ['centerPoint', 'center'])
        axis = _get_attr(geo, ['axisVector', 'axis'])
        major_r = _get_attr(geo, ['majorRadius'])
        minor_r = _get_attr(geo, ['minorRadius'])
        result['params']['center'] = _p3d_to_list(center, world_transform)
        result['params']['axis'] = _v3d_to_list(axis, world_transform)
        if major_r is not None:
            result['params']['majorRadius'] = round(major_r * MM, 4)
        if minor_r is not None:
            result['params']['minorRadius'] = round(minor_r * MM, 4)
    else:
        # 未识别（NURBS 自由曲面等），记录类型名
        result['params']['_unsupported'] = geo_type

    # 面面积（cm²）用于判断主面/小面
    try:
        result['area_mm2'] = round(face.area * MM * MM, 2)
    except Exception:
        result['area_mm2'] = 0

    return result


class BRepExtractor:
    """BRep 曲面提取器：遍历零件所有实体面，提取精确曲面参数。"""

    def __init__(self, logger):
        self.logger = logger

    def extract_surfaces(self, occurrence: adsk.fusion.Occurrence) -> List[Dict[str, Any]]:
        """提取一个 occurrence 的所有 BRep 曲面。

        Args:
            occurrence: Fusion occurrence（有实体的零件）

        Returns:
            曲面参数列表 [{surface, params, area_mm2}, ...]
        """
        surfaces: List[Dict[str, Any]] = []
        try:
            component = occurrence.component
            if not component or component.bRepBodies.count == 0:
                return surfaces

            world_transform = _accumulate_transform(occurrence)

            for body_idx in range(component.bRepBodies.count):
                body = component.bRepBodies.item(body_idx)
                for face in body.faces:
                    fd = _extract_face(face, world_transform)
                    if fd is not None:
                        surfaces.append(fd)

            log_component(self.logger, occurrence.name,
                          f"BRep: {len(surfaces)} 面")
        except Exception as e:
            log_error(self.logger, f"BRep 提取失败 {occurrence.name}: {e}")
        return surfaces
