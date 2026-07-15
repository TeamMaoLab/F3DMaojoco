"""
零部件收集器

遍历装配体，收集所有零部件信息。
递归遍历装配树结构，提取零部件的几何信息和变换矩阵。
"""

from typing import List, Optional, Dict, Any

import adsk.core
import adsk.fusion

from .logger import log_component, log_transform
from ..common.data_types import ComponentInfo, CollisionShape
from ..common.geometry_math import Transform4D

# 碰撞体命名前缀：以 COL_ 开头的 occurrence 被识别为碰撞体（不导出 STL，几何提取到父零件的 colliders）
COLLIDER_PREFIX = "COL_"


class ComponentCollector:
    """零部件收集器
    
    遍历装配体，收集所有零部件信息：
    - 递归遍历装配树结构
    - 提取零部件的几何信息、变换矩阵
    - 统计实体数量和子零部件关系
    - 生成 ComponentInfo 对象
    """
    
    def __init__(self, logger):
        """初始化零部件收集器
        
        Args:
            logger: 日志记录器
        """
        self.logger = logger
        self.app = adsk.core.Application.get()
        self.design = self.app.activeProduct
        
        # 用于跟踪已处理的零部件，避免重复处理
        self._processed_components = set()
        
        # 装配树路径缓存
        self._component_paths = {}
    
    def collect_components(self) -> List[ComponentInfo]:
        """收集所有零部件信息
        
        Returns:
            List[ComponentInfo]: 零部件信息列表
        """
        self.logger.info("开始收集零部件信息")
        
        if not self.design:
            raise ValueError("没有活动的 Fusion 360 设计")
        
        # 获取根组件
        root_component = self.design.rootComponent
        if not root_component:
            raise ValueError("无法获取根组件")
        
        # 清空处理记录
        self._processed_components.clear()
        self._component_paths.clear()
        
        # 收集所有零部件
        components = []
        
        # 从根组件开始递归收集
        self._collect_from_component(root_component, components, "")
        
        self.logger.info(f"零部件收集完成，共收集 {len(components)} 个零部件")
        return components
    
    def _collect_from_component(self, component: adsk.fusion.Component, 
                               components: List[ComponentInfo], 
                               parent_path: str):
        """从指定组件递归收集零部件
        
        Args:
            component: Fusion 360 组件
            components: 零部件列表
            parent_path: 父级路径
        """
        try:
            # 获取组件的唯一标识
            component_id = id(component)
            
            # 如果已经处理过，跳过
            if component_id in self._processed_components:
                return
            
            self._processed_components.add(component_id)
            
            # 构建完整路径
            current_path = f"{parent_path}/{component.name}" if parent_path else component.name
            self._component_paths[component_id] = current_path
            
            # 记录当前处理的组件
            log_component(self.logger, component.name, "收集")
            
            # 处理当前组件的所有 occurrence
            for occurrence in component.occurrences:
                self._collect_from_occurrence(occurrence, components, current_path)
            
            # 递归处理子组件
            for child_component in component.allOccurrences:
                if child_component.component and child_component.component != component:
                    self._collect_from_component(child_component.component, components, current_path)
            
        except Exception as e:
            self.logger.error(f"收集组件 {component.name} 时发生错误: {str(e)}")
            # 不要因为一个组件失败而中断整个收集过程
    
    def _collect_from_occurrence(self, occurrence: adsk.fusion.Occurrence,
                                components: List[ComponentInfo],
                                parent_path: str):
        """从 occurrence 收集零部件信息
        
        Args:
            occurrence: Fusion 360 occurrence
            components: 零部件列表
            parent_path: 父级路径
        """
        try:
            if not occurrence.component:
                return

            # COL 前缀的 occurrence（COL_ 和裸 COL）是碰撞体，不作为独立零件收集
            # （它的圆柱几何会被父零件的 _collect_colliders 提取）
            if occurrence.component.name.startswith("COL"):
                self.logger.debug(f"跳过碰撞体 occurrence: {occurrence.name}")
                return

            # 构建完整路径
            full_path = f"{parent_path}/{occurrence.name}"

            # 创建零部件信息
            component_info = self._create_component_info(occurrence, full_path)

            if component_info:
                # 提取挂在本 occurrence 下的 COL_ 子碰撞体
                component_info.colliders = self._collect_colliders(occurrence)
                components.append(component_info)
                log_component(self.logger, component_info.name, "创建")
                if component_info.colliders:
                    self.logger.info(f"{component_info.name}: 提取 {len(component_info.colliders)} 个碰撞体")

            # 递归处理子 occurrence
            for child_occurrence in occurrence.childOccurrences:
                self._collect_from_occurrence(child_occurrence, components, full_path)

        except Exception as e:
            self.logger.error(f"收集 occurrence {occurrence.name} 时发生错误: {str(e)}")
    
    def _create_component_info(self, occurrence: adsk.fusion.Occurrence,
                               full_path: str) -> Optional[ComponentInfo]:
        """创建零部件信息对象
        
        Args:
            occurrence: Fusion 360 occurrence
            full_path: 完整路径
            
        Returns:
            Optional[ComponentInfo]: 零部件信息对象
        """
        try:
            component = occurrence.component
            if not component:
                return None
            
            # 获取世界坐标系变换矩阵
            world_transform = self._get_world_transform(occurrence)
            
            # 将Fusion 360 API的厘米单位转换为毫米单位
            if world_transform:
                world_transform = self._convert_cm_to_mm(world_transform)
            
            # 统计实体数量
            bodies_count = len(component.bRepBodies)
            
            # 检查是否有子零部件
            has_children = len(occurrence.childOccurrences) > 0
            
            # 创建零部件信息
            component_info = ComponentInfo(
                name=component.name,
                occurrence_name=occurrence.name,
                full_path_name=full_path,
                component_id=component.name,
                bodies_count=bodies_count,
                has_children=has_children,
                world_transform=world_transform
            )
            
            # 记录变换信息（调试级别）
            if world_transform:
                log_transform(self.logger, f"{component.name}_world", world_transform)
            
            # 调试：记录装配体层次结构信息
            self.logger.debug(f"{component.name}: 装配体路径 = {full_path}")
            self.logger.debug(f"{component.name}: 有子零部件 = {has_children}")
            
            return component_info
            
        except Exception as e:
            self.logger.error(f"创建零部件信息 {occurrence.name} 时发生错误: {str(e)}")
            return None

    def _collect_colliders(self, occurrence: adsk.fusion.Occurrence) -> List[CollisionShape]:
        """递归遍历 occurrence 的子节点，提取所有 COL_ 前缀碰撞体的圆柱几何。

        COL_ 组件下可能挂多个实体（每个实体可能有多个圆柱面），
        每个圆柱面输出一个 CollisionShape。归属为当前 occurrence（父零件）。
        """
        colliders: List[CollisionShape] = []
        try:
            for child in occurrence.childOccurrences:
                if not child.component:
                    continue
                if child.component.name.startswith("COL"):
                    shapes = self._extract_cylinders(child, occurrence.component.name)
                    colliders.extend(shapes)
                # 递归更深层（COL_ 可能挂在子装配体下）
                colliders.extend(self._collect_colliders(child))
        except Exception as e:
            self.logger.error(f"提取碰撞体时发生错误: {str(e)}")
        return colliders

    def _accumulate_transform(self, occurrence: adsk.fusion.Occurrence) -> Optional['adsk.core.Matrix3D']:
        """累加 occurrence 及其所有祖先的 transform，得到世界变换矩阵。

        Fusion 的 occurrence.transform 是相对父级的局部变换。要从 body 局部坐标
        转到世界坐标，需要从根向下累乘：world = root.local × ... × parent.local × this.local。
        这里从当前 occurrence 往上收集所有局部 transform，再按从根到叶的顺序相乘。
        """
        try:
            import adsk.core
            # 收集从根到当前 occurrence 的局部变换链
            chain = []
            occ = occurrence
            while occ is not None:
                t = occ.transform
                if t is not None:
                    chain.append(t)
                occ = occ.assemblyContext  # 父 occurrence（顶级为 None）
            if not chain:
                return None
            # chain[0] 是最深的（当前），chain[-1] 是最浅的（根级）
            # 世界变换 = 根.local × ... × 当前.local，所以要反序相乘
            world = chain[-1].copy()
            for i in range(len(chain) - 2, -1, -1):
                world.transformBy(chain[i])
            return world
        except Exception as e:
            self.logger.error(f"累加 transform 失败: {str(e)}")
            return None

    def _extract_cylinders(self, collider_occ: adsk.fusion.Occurrence,
                           parent_name: str) -> List[CollisionShape]:
        """从 COL_ occurrence 提取所有圆柱几何（可能多个 body，每个 body 多个圆柱面）。

        一个 COL_ 组件下可以挂多个实体，每个实体可能有多个圆柱面。
        每个圆柱面输出一个 CollisionShape。
        Fusion API 返回 body 局部坐标 + cm 单位；这里用累加的 world transform
        把 origin/axis/endpoints 转世界，再 ×10 转 mm。
        """
        shapes: List[CollisionShape] = []
        try:
            component = collider_occ.component
            if not component or component.bRepBodies.count == 0:
                self.logger.warning(f"碰撞体 {collider_occ.name} 无实体，跳过")
                return shapes

            # 预先算好世界变换（所有圆柱共用）
            world_transform = self._accumulate_transform(collider_occ)
            if world_transform is None:
                self.logger.warning(f"碰撞体 {component.name} 无 transform，用局部坐标")

            MM = 10.0
            cyl_index = 0  # 同名组件下多个圆柱用后缀区分

            # 遍历所有 body
            for body_idx in range(component.bRepBodies.count):
                body = component.bRepBodies.item(body_idx)
                # 详细诊断：body 类型 + face 数 + 属性
                try:
                    body_type = body.objectType
                    face_count = body.faces.count
                    is_surf = getattr(body, 'isSurface', '?')
                    body_name = getattr(body, 'name', '?')
                    self.logger.info(f"  {component.name} body[{body_idx}]: type={body_type}, "
                                     f"faces={face_count}, isSurface={is_surf}, name={body_name}")
                except Exception as be:
                    self.logger.warning(f"  {component.name} body[{body_idx}] 诊断失败: {be}")

                # 遍历所有面，找圆柱面
                faces_found = 0
                for face in body.faces:
                    faces_found += 1
                    try:
                        st = face.surfaceType
                        if faces_found <= 6:  # 只打印前 6 个面避免刷屏
                            self.logger.debug(f"    face[{faces_found-1}]: surfaceType={st}")
                        if st != adsk.fusion.SurfaceTypes.CylinderSurfaceType:
                            continue
                    except Exception as fe:
                        self.logger.debug(f"    face 读取 surfaceType 失败: {fe}")
                        continue

                    try:
                        cyl = face.geometry  # adsk.core.Cylinder
                        radius_cm = cyl.radius
                        axis_local = cyl.axis
                        origin_local = cyl.origin

                        # 从圆形边提取两端点
                        endpoints_local = []
                        for edge in face.edges:
                            try:
                                geo = edge.geometry
                                if geo is None:
                                    continue
                                if getattr(geo, 'objectType', '') == 'Circle3D':
                                    endpoints_local.append(geo.center)
                            except Exception:
                                continue
                        unique_ends = self._dedup_points(endpoints_local, tol=1e-4)
                        if len(unique_ends) != 2:
                            # 用 boundingBox 兜底估算长度
                            self.logger.debug(
                                f"碰撞体 {component.name} 圆柱{cyl_index} 圆边数={len(unique_ends)}，用 bbox 估算")
                            length_cm = self._estimate_length_from_bbox(body, axis_local)
                            end1 = self._point_along(origin_local, axis_local, length_cm / 2)
                            end2 = self._point_along(origin_local, axis_local, -length_cm / 2)
                        else:
                            end1, end2 = unique_ends[0], unique_ends[1]
                            length_cm = self._dist(end1, end2)

                        # 转世界坐标
                        if world_transform is None:
                            origin_world = origin_local
                            axis_world = axis_local
                            end1_world = end1
                            end2_world = end2
                        else:
                            origin_world = origin_local.copy()
                            origin_world.transformBy(world_transform)
                            end1_world = end1.copy(); end1_world.transformBy(world_transform)
                            end2_world = end2.copy(); end2_world.transformBy(world_transform)
                            axis_world = axis_local.copy()
                            axis_world.transformBy(world_transform)
                            axis_world.normalize()

                        shape_name = component.name if cyl_index == 0 else f"{component.name}_{cyl_index}"
                        shapes.append(CollisionShape(
                            name=shape_name,
                            parent_component=parent_name,
                            shape_type="cylinder",
                            radius=round(radius_cm * MM, 4),
                            length=round(length_cm * MM, 4),
                            axis=[round(axis_world.x, 6), round(axis_world.y, 6), round(axis_world.z, 6)],
                            origin=[round(origin_world.x * MM, 4),
                                    round(origin_world.y * MM, 4),
                                    round(origin_world.z * MM, 4)],
                            endpoints=[[round(end1_world.x * MM, 4), round(end1_world.y * MM, 4), round(end1_world.z * MM, 4)],
                                       [round(end2_world.x * MM, 4), round(end2_world.y * MM, 4), round(end2_world.z * MM, 4)]],
                        ))
                        cyl_index += 1
                    except Exception as e:
                        self.logger.error(f"处理圆柱面时出错（{component.name}）: {str(e)}")
                        continue

            if not shapes:
                self.logger.warning(
                    f"碰撞体 {component.name}（{component.bRepBodies.count} 个实体）未找到圆柱面，跳过")
        except Exception as e:
            self.logger.error(f"提取圆柱 {collider_occ.name} 时发生错误: {str(e)}")
        return shapes

    @staticmethod
    def _dedup_points(points, tol=1e-4):
        """去重 3D 点（距离 < tol 视为同一点）。"""
        unique = []
        for p in points:
            is_dup = False
            for u in unique:
                if abs(p.x - u.x) < tol and abs(p.y - u.y) < tol and abs(p.z - u.z) < tol:
                    is_dup = True
                    break
            if not is_dup:
                unique.append(p)
        return unique

    @staticmethod
    def _dist(a, b):
        """两点距离（cm）。"""
        import math
        return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2 + (a.z - b.z)**2)

    @staticmethod
    def _point_along(origin, axis, distance):
        """从 origin 沿 axis 方向偏移 distance 的点。"""
        import adsk.core
        return adsk.core.Point3D.create(
            origin.x + axis.x * distance,
            origin.y + axis.y * distance,
            origin.z + axis.z * distance,
        )

    @staticmethod
    def _estimate_length_from_bbox(body, axis):
        """用 body 的 boundingBox 估算沿 axis 方向的长度（cm，近似）。"""
        bbox = body.boundingBox
        if not bbox:
            return 0.0
        # bbox 是轴对齐的，取 max(尺寸) 作为长度估计（圆柱长度是最长方向）
        dx = bbox.maxPoint.x - bbox.minPoint.x
        dy = bbox.maxPoint.y - bbox.minPoint.y
        dz = bbox.maxPoint.z - bbox.minPoint.z
        return max(dx, dy, dz)

    def _get_world_transform(self, occurrence: adsk.fusion.Occurrence) -> Optional[Transform4D]:
        """获取世界坐标系变换矩阵
        
        计算 occurrence 相对于世界坐标系的完整变换。
        如果 occurrence 在装配体中，需要累加所有父级变换。
        
        Args:
            occurrence: Fusion 360 occurrence
            
        Returns:
            Optional[Transform4D]: 世界坐标系变换矩阵
        """
        try:
            # 对于世界变换，如果 occurrence 有父装配体，需要计算变换链
            # 但首先让我们尝试直接使用 occurrence.transform，看看是否已经包含了正确的世界变换
            
            # 直接使用 occurrence.transform
            transform = occurrence.transform
            if not transform:
                return None
            
            # 转换为 4x4 矩阵
            matrix_data = transform.asArray()
            
            # 重新组织为 4x4 格式
            matrix_4x4 = [
                [matrix_data[0], matrix_data[1], matrix_data[2], matrix_data[3]],
                [matrix_data[4], matrix_data[5], matrix_data[6], matrix_data[7]],
                [matrix_data[8], matrix_data[9], matrix_data[10], matrix_data[11]],
                [0.0, 0.0, 0.0, 1.0]
            ]
            
            return Transform4D(matrix_4x4)
            
        except Exception as e:
            self.logger.error(f"获取世界变换矩阵时发生错误: {str(e)}")
            return None
    
        
    def _convert_cm_to_mm(self, transform: Transform4D) -> Transform4D:
        """将厘米单位的变换矩阵转换为毫米单位
        
        Args:
            transform: 厘米单位的变换矩阵
            
        Returns:
            Transform4D: 毫米单位的变换矩阵
        """
        try:
            # 创建新的矩阵，只缩放位置分量（第4列）
            scaled_matrix = []
            for i, row in enumerate(transform.matrix):
                if i < 3:  # 前3行
                    scaled_row = row.copy()
                    scaled_row[3] = row[3] * 10.0  # 厘米转毫米
                    scaled_matrix.append(scaled_row)
                else:  # 最后一行
                    scaled_matrix.append(row.copy())
            
            return Transform4D(scaled_matrix)
            
        except Exception as e:
            self.logger.error(f"厘米转毫米转换时发生错误: {str(e)}")
            return transform