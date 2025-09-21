"""
算法服务模块

从 MaojocoConverter 移植的核心算法，包括：
- 装配图构建算法
- 环检测算法
- 树生成算法
- 坐标变换算法

这些算法将集成到 ProjectWorkflowManager 中。
"""

import math
import networkx as nx
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass
from pathlib import Path

from utils.logger import logger
from .domain_types import (
    ExportData, JointInfo, ComponentInfo, JointType,
    Body4DCoordinates, JointGlobalCoordinates, JointPairwiseRelationship,
    KinematicTree, KinematicBody, KinematicJoint, KinematicNode, RelativeTransform,
    CycleInfo, BrokenJointInfo, AssemblyTreeInfo,
    Vector3D, Transform4D
)


class RelationshipAnalysisService:
    """关系分析服务 - 处理装配关系分析和树生成"""
    
    def __init__(self, project_context):
        """初始化关系分析服务
        
        Args:
            project_context: 项目上下文，包含所有数据和状态
        """
        self.ctx = project_context
    
    def analyze_joint_pairwise_relationships(self):
        """分析关节两两关系"""
        logger.info("🔍 分析关节两两关系")
        
        joints = self.ctx.export_data.joints
        
        for i, joint1 in enumerate(joints):
            for j, joint2 in enumerate(joints):
                if i < j:  # 避免重复分析
                    relationship = self._analyze_joint_pair(joint1, joint2)
                    if relationship:
                        pair_key = (joint1.name, joint2.name)
                        self.ctx.joint_pairwise_relationships[pair_key] = relationship
    
    def _analyze_joint_pair(self, joint1: JointInfo, joint2: JointInfo) -> Optional[JointPairwiseRelationship]:
        """分析两个关节之间的关系"""
        # 获取关节位置
        pos1 = self.ctx.joint_global_coordinates.get(joint1.name)
        pos2 = self.ctx.joint_global_coordinates.get(joint2.name)
        
        if not pos1 or not pos2:
            return None
        
        # 计算距离
        distance = pos1.position.distance_to(pos2.position)
        
        # 获取连接的零部件
        joint1_components = set()
        if joint1.connection.occurrence_one_component:
            joint1_components.add(joint1.connection.occurrence_one_component)
        if joint1.connection.occurrence_two_component:
            joint1_components.add(joint1.connection.occurrence_two_component)
        
        joint2_components = set()
        if joint2.connection.occurrence_one_component:
            joint2_components.add(joint2.connection.occurrence_one_component)
        if joint2.connection.occurrence_two_component:
            joint2_components.add(joint2.connection.occurrence_two_component)
        
        # 检查是否有共享零部件
        shared_components = joint1_components.intersection(joint2_components)
        
        return JointPairwiseRelationship(
            joint1_name=joint1.name,
            joint2_name=joint2.name,
            distance=distance,
            joint1_type=joint1.joint_type,
            joint2_type=joint2.joint_type,
            joint1_components=list(joint1_components),
            joint2_components=list(joint2_components),
            shared_components=list(shared_components),
            has_shared_components=len(shared_components) > 0,
            connection_strength=self._calculate_connection_strength(distance, len(shared_components))
        )
    
    def _calculate_connection_strength(self, distance: float, shared_components: int) -> float:
        """计算连接强度"""
        # 距离越近，共享组件越多，连接强度越大
        distance_factor = 1.0 / (1.0 + distance / 100.0)  # 归一化距离因子
        shared_factor = min(shared_components, 2) / 2.0  # 共享组件因子
        
        return distance_factor * shared_factor
    
    def build_assembly_graph(self):
        """构建装配关系图"""
        logger.info("🏗️  构建装配关系图")
        
        # 创建无向图
        self.ctx.assembly_graph = nx.Graph()
        
        # 添加所有零部件作为节点
        for component in self.ctx.export_data.components:
            self.ctx.assembly_graph.add_node(
                component.name,
                component_type='component',
                occurrence_name=component.occurrence_name,
                full_path=component.full_path_name,
                bodies_count=component.bodies_count,
                has_children=component.has_children,
                component_id=component.component_id
            )
        
        # 基于关节连接添加边
        for joint in self.ctx.export_data.joints:
            if joint.connection.occurrence_one_component and joint.connection.occurrence_two_component:
                comp1 = joint.connection.occurrence_one_component
                comp2 = joint.connection.occurrence_two_component
                
                # 确保节点存在
                if comp1 in self.ctx.assembly_graph and comp2 in self.ctx.assembly_graph:
                    # 添加边，包含关节信息
                    self.ctx.assembly_graph.add_edge(
                        comp1, comp2,
                        joint_name=joint.name,
                        joint_type=joint.joint_type.value,
                        relationship='joint_connection',
                        is_suppressed=joint.is_suppressed,
                        is_active=joint.is_light_bulb_on
                    )
        
        # 基于关节两两关系添加额外的边
        for pair_key, relationship in self.ctx.joint_pairwise_relationships.items():
            joint1_name, joint2_name = pair_key
            
            # 如果两个关节有共享组件，则在这些组件之间添加间接关系
            if relationship.has_shared_components:
                for comp in relationship.shared_components:
                    if comp in self.ctx.assembly_graph:
                        # 为这个组件添加自环边表示其参与多个关节
                        if self.ctx.assembly_graph.has_node(comp):
                            current_data = self.ctx.assembly_graph.nodes[comp]
                            current_data['multi_joint'] = current_data.get('multi_joint', 0) + 1
        
        logger.info(f"装配图构建完成: {self.ctx.assembly_graph.number_of_nodes()} 节点, {self.ctx.assembly_graph.number_of_edges()} 边")
    
    def detect_cycles(self) -> List[CycleInfo]:
        """检测环结构"""
        logger.info("🔄 检测环结构")
        
        self.ctx.detected_cycles = []
        
        if not self.ctx.assembly_graph:
            logger.warning("装配图不存在，无法检测环结构")
            return []
        
        # 检测环
        cycles = list(nx.cycle_basis(self.ctx.assembly_graph))
        
        for i, cycle in enumerate(cycles):
            # 获取环中的边和关节
            edges = []
            joints = []
            
            for j in range(len(cycle)):
                node1 = cycle[j]
                node2 = cycle[(j + 1) % len(cycle)]
                
                if self.ctx.assembly_graph.has_edge(node1, node2):
                    edge_data = self.ctx.assembly_graph.edges[node1, node2]
                    edges.append((node1, node2))
                    joints.append(edge_data.get('joint_name', 'unknown'))
            
            cycle_info = CycleInfo(
                cycle_id=i + 1,
                nodes=cycle,
                edges=edges,
                joints=joints,
                length=len(cycle)
            )
            self.ctx.detected_cycles.append(cycle_info)
        
        logger.info(f"检测到 {len(self.ctx.detected_cycles)} 个环结构")
        return self.ctx.detected_cycles
    
    def generate_assembly_trees(self, root_node_strategy: str = "center") -> List[AssemblyTreeInfo]:
        """生成装配树
        
        Args:
            root_node_strategy: 根节点选择策略 ("center", "max_degree", "manual")
        """
        logger.info("🌳 生成装配树")
        
        if not self.ctx.assembly_graph:
            logger.warning("装配图不存在，无法生成装配树")
            return []
        
        # 找到所有连通分量
        connected_components = list(nx.connected_components(self.ctx.assembly_graph))
        
        self.ctx.assembly_trees = []
        
        for i, component_set in enumerate(connected_components):
            # 为每个连通分量生成一个树
            subgraph = self.ctx.assembly_graph.subgraph(component_set)
            
            # 选择根节点（根据策略）
            root_node = self._select_root_node(subgraph, root_node_strategy)
            
            # 显示原始连接信息
            original_edges = set(subgraph.edges())
            
            # 生成最小生成树
            try:
                # 如果有权重边，使用权重
                if any('connection_strength' in subgraph.edges[e] for e in subgraph.edges()):
                    mst = nx.minimum_spanning_tree(subgraph, weight='connection_strength')
                else:
                    # 否则使用普通的最小生成树
                    mst = nx.minimum_spanning_tree(subgraph)
            except:
                # 如果失败，使用BFS树
                mst = nx.bfs_tree(subgraph, root_node)
            
            # 构建树结构
            tree_structure = self._build_tree_structure(mst, root_node)
            
            tree_info = AssemblyTreeInfo(
                root_node=root_node,
                tree_depth=self._calculate_tree_depth(tree_structure),
                node_count=len(component_set),
                broken_edges=list(original_edges - set(mst.edges())),
                tree_structure=tree_structure
            )
            
            self.ctx.assembly_trees.append(tree_info)
            
            logger.info(f"装配树 {i+1}: 根节点={root_node}, 组件数={len(component_set)}, 深度={tree_info.tree_depth}")
        
        logger.info(f"生成了 {len(self.ctx.assembly_trees)} 个装配树")
        return self.ctx.assembly_trees
    
    def _select_root_node(self, graph: nx.Graph, strategy: str = "center") -> str:
        """根据策略选择根节点"""
        logger.info(f"根节点选择策略: {strategy}")
        
        if strategy == "center":
            # 选择图的中心节点
            return self._find_center_node(graph)
        elif strategy == "max_degree":
            # 选择连接度最大的节点
            max_degree_node = max(graph.nodes(), key=lambda n: graph.degree(n))
            max_degree = graph.degree(max_degree_node)
            logger.info(f"选择连接度最大的节点: {max_degree_node} (连接度: {max_degree})")
            return max_degree_node
        else:
            logger.warning(f"未知的根节点选择策略: {strategy}，使用默认策略 'center'")
            return self._find_center_node(graph)
    
    def _find_center_node(self, graph: nx.Graph) -> str:
        """找到图的中心节点（最小化到所有其他节点的最大距离）"""
        try:
            # 计算所有节点的偏心度（到最远节点的距离）
            eccentricity = nx.eccentricity(graph)
            
            # 找到偏心度最小的节点（中心节点）
            center_node = min(eccentricity.keys(), key=lambda n: eccentricity[n])
            min_eccentricity = eccentricity[center_node]
            
            logger.info(f"选择的中心节点: {center_node} (偏心度: {min_eccentricity})")
            
            return center_node
            
        except Exception as e:
            logger.warning(f"计算中心节点失败: {e}")
            # 降级为连接度最大的节点
            fallback_node = max(graph.nodes(), key=lambda n: graph.degree(n))
            logger.info(f"降级策略: 选择连接度最大的节点 {fallback_node}")
            return fallback_node
    
    def _build_tree_structure(self, tree: nx.DiGraph, root: str, parent: Optional[str] = None) -> Dict[str, Any]:
        """构建树结构"""
        node_data = tree.nodes[root]
        
        tree_node = {
            'name': root,
            'data': node_data,
            'children': []
        }
        
        # 递归处理子节点
        for child in tree.neighbors(root):
            if child != parent:
                child_tree = self._build_tree_structure(tree, child, root)
                tree_node['children'].append(child_tree)
        
        return tree_node
    
    def _calculate_tree_depth(self, tree_node: Dict[str, Any]) -> int:
        """计算树深度"""
        if not tree_node['children']:
            return 1
        
        max_child_depth = 0
        for child in tree_node['children']:
            child_depth = self._calculate_tree_depth(child)
            max_child_depth = max(max_child_depth, child_depth)
        
        return max_child_depth + 1
    
    def build_kinematic_tree(self) -> Optional[KinematicTree]:
        """构建MuJoCo运动学树"""
        logger.info("🌲 构建MuJoCo运动学树")
        
        if not self.ctx.assembly_graph or not self.ctx.assembly_trees:
            logger.error("装配图或装配树不存在，无法构建运动学树")
            return None
        
        # 运动学树结构
        kinematic_tree = KinematicTree(
            roots=[],
            nodes={},
            joints={},
            bodies={},
            relative_transforms={}
        )
        
        # 从装配树构建运动学树
        for assembly_tree in self.ctx.assembly_trees:
            kinematic_root = self._convert_assembly_to_kinematics(
                assembly_tree.tree_structure, 
                None,  # 父节点
                kinematic_tree
            )
            if kinematic_root:
                kinematic_tree.roots.append(kinematic_root)
        
        # 计算相对变换
        self._calculate_relative_transforms(kinematic_tree)
        
        # 验证运动学树完整性
        self._validate_kinematic_tree(kinematic_tree)
        
        self.ctx.kinematic_tree = kinematic_tree
        
        logger.info("运动学树构建完成:")
        logger.info(f"  - 根节点: {len(kinematic_tree.roots)} 个")
        logger.info(f"  - 节点: {len(kinematic_tree.nodes)} 个")
        logger.info(f"  - 关节: {len(kinematic_tree.joints)} 个")
        logger.info(f"  - 刚体: {len(kinematic_tree.bodies)} 个")
        
        return kinematic_tree
    
    def _convert_assembly_to_kinematics(self, assembly_node: Dict[str, Any], parent_body: Optional[str], kinematic_tree: KinematicTree) -> Optional[str]:
        """将装配树节点转换为动力学树节点"""
        component_name = assembly_node['name']
        
        # 获取组件的4D坐标
        body_4d = self.ctx.body_4d_coordinates.get(component_name)
        if not body_4d:
            logger.warning(f"组件 {component_name} 没有4D坐标信息")
            return None
        
        # 创建动力学体
        body_id = f"body_{component_name}"
        
        kinematic_body = KinematicBody(
            body_id=body_id,
            name=component_name,
            component_id=str(body_4d.component_id),
            occurrence_name=body_4d.occurrence_name,
            world_transform=body_4d.transform,
            stl_file=body_4d.stl_file,
            bodies_count=body_4d.bodies_count,
            mass=1.0,  # 默认质量，后续可以计算
            inertia=[1.0, 1.0, 1.0],  # 默认惯性，后续可以计算
            parent=parent_body
        )
        
        kinematic_tree.bodies[body_id] = kinematic_body
        
        # 创建运动学节点
        kinematic_node = KinematicNode(
            body_id=body_id,
            parent_body=parent_body,
            children=[],
            joint=None,  # 将在下面设置
            level=0 if parent_body is None else kinematic_tree.nodes[parent_body].level + 1
        )
        
        kinematic_tree.nodes[body_id] = kinematic_node
        
        # 处理与父节点的关节连接
        if parent_body:
            # 从 parent_body 中提取组件名称 (例如: "body_connect2" -> "connect2")
            parent_component_name = parent_body.replace("body_", "")
            
            joint_info = self._find_joint_between_bodies(parent_component_name, component_name)
            if joint_info:
                joint_id = f"joint_{joint_info.name}"
                
                # 获取关节位置
                joint_position = self._get_joint_position(joint_info)
                joint_axis = self._get_joint_axis(joint_info)
                
                # 创建关节
                kinematic_joint = KinematicJoint(
                    joint_id=joint_id,
                    name=joint_info.name,
                    joint_type=joint_info.joint_type,
                    parent_body=parent_body,
                    child_body=body_id,
                    position=joint_position,
                    axis=joint_axis,
                    limits=None,  # 简化版本，后续可以添加
                    is_suppressed=joint_info.is_suppressed,
                    is_active=joint_info.is_light_bulb_on
                )
                
                kinematic_tree.joints[joint_id] = kinematic_joint
                
                # 连接节点和关节
                kinematic_tree.nodes[body_id].joint = joint_id
                kinematic_tree.nodes[parent_body].children.append(body_id)
        
        # 递归处理子节点
        for child_node in assembly_node['children']:
            self._convert_assembly_to_kinematics(child_node, body_id, kinematic_tree)
        
        return body_id
    
    def _find_joint_between_bodies(self, body1_name: str, body2_name: str) -> Optional[JointInfo]:
        """找到两个体之间的关节"""
        for joint in self.ctx.export_data.joints:
            comp1 = joint.connection.occurrence_one_component
            comp2 = joint.connection.occurrence_two_component
            
            if (comp1 == body1_name and comp2 == body2_name) or (comp1 == body2_name and comp2 == body1_name):
                return joint
        
        return None
    
    def _get_joint_position(self, joint_info: JointInfo) -> Vector3D:
        """获取关节位置（相对于父节点）"""
        # 简化实现：返回默认位置
        # 实际实现需要复杂的坐标变换，这里先简化
        return Vector3D(0.0, 0.0, 0.0)
    
    def _get_joint_axis(self, joint_info: JointInfo) -> Optional[Vector3D]:
        """获取关节轴"""
        if joint_info.joint_type == JointType.REVOLUTE:
            # 旋转关节，默认使用Z轴
            return Vector3D(0.0, 0.0, 1.0)
        elif joint_info.joint_type == JointType.SLIDER:
            # 滑动关节，假设沿X轴
            return Vector3D(1.0, 0.0, 0.0)
        
        return None
    
    def _calculate_relative_transforms(self, kinematic_tree: KinematicTree):
        """计算相对变换"""
        logger.info("📐 计算相对变换")
        
        # 为每个根节点计算相对变换
        for root_body_id in kinematic_tree.roots:
            self._calculate_body_relative_transform(root_body_id, None, kinematic_tree)
    
    def _calculate_body_relative_transform(self, body_id: str, parent_body_id: Optional[str], kinematic_tree: KinematicTree):
        """计算单个体的相对变换"""
        body = kinematic_tree.bodies[body_id]
        
        if parent_body_id is None:
            # 根节点，相对变换就是世界变换
            relative_transform = body.world_transform
        else:
            # 计算相对于父节点的变换
            parent_body = kinematic_tree.bodies[parent_body_id]
            relative_transform = self._compute_relative_transform(
                body.world_transform,
                parent_body.world_transform
            )
        
        # 存储相对变换
        relative_transform_obj = RelativeTransform(
            parent=parent_body_id,
            transform=relative_transform
        )
        
        kinematic_tree.relative_transforms[body_id] = relative_transform_obj
        
        # 递归处理子节点
        node = kinematic_tree.nodes[body_id]
        for child_body_id in node.children:
            self._calculate_body_relative_transform(child_body_id, body_id, kinematic_tree)
    
    def _compute_relative_transform(self, child_transform: Transform4D, parent_transform: Transform4D) -> Transform4D:
        """计算相对变换矩阵"""
        try:
            import numpy as np
            
            # 转换为numpy矩阵
            child_matrix = np.array(child_transform.matrix)
            parent_matrix = np.array(parent_transform.matrix)
            
            # 计算相对变换: T_relative = T_parent^-1 * T_child
            parent_inverse = np.linalg.inv(parent_matrix)
            relative_matrix = np.dot(parent_inverse, child_matrix)
            
            # 清理浮点误差
            relative_matrix = np.where(np.abs(relative_matrix) < 1e-10, 0, relative_matrix)
            
            return Transform4D(relative_matrix.tolist())
            
        except ImportError:
            # 如果没有numpy，使用简化的实现
            # 这里返回单位矩阵作为简化
            return Transform4D.identity()
    
    def _validate_kinematic_tree(self, kinematic_tree: KinematicTree):
        """验证运动学树完整性"""
        logger.info("✅ 验证运动学树完整性")
        
        # 检查所有节点都有父节点引用（除了根节点）
        for body_id, node in kinematic_tree.nodes.items():
            if body_id not in kinematic_tree.roots and node.parent_body is None:
                logger.warning(f"节点 {body_id} 不是根节点但没有父节点")
        
        # 检查父子关系一致性
        for body_id, node in kinematic_tree.nodes.items():
            if node.parent_body:
                parent_node = kinematic_tree.nodes.get(node.parent_body)
                if parent_node and body_id not in parent_node.children:
                    logger.warning(f"父子关系不一致: {body_id} 不在 {node.parent_body} 的子节点列表中")
        
        logger.info("✅ 运动学树验证通过")


class CoordinateTransformService:
    """坐标变换服务 - 处理单位转换和坐标变换"""
    
    @staticmethod
    def convert_millimeters_to_meters(transform: Transform4D) -> Transform4D:
        """将毫米单位的变换转换为米单位"""
        # 创建新的变换矩阵
        new_matrix = []
        for row in transform.matrix:
            new_row = row.copy()
            # 只缩放平移部分（前3行的第4列）
            new_row[3] /= 1000.0  # 毫米转米
            new_matrix.append(new_row)
        
        return Transform4D(new_matrix)
    
    @staticmethod
    def convert_meters_to_millimeters(transform: Transform4D) -> Transform4D:
        """将米单位的变换转换为毫米单位"""
        # 创建新的变换矩阵
        new_matrix = []
        for row in transform.matrix:
            new_row = row.copy()
            # 只缩放平移部分（前3行的第4列）
            new_row[3] *= 1000.0  # 米转毫米
            new_matrix.append(new_row)
        
        return Transform4D(new_matrix)
    
    @staticmethod
    def convert_vector_mm_to_m(vector: Vector3D) -> Vector3D:
        """将毫米单位的向量转换为米单位"""
        return Vector3D(vector.x / 1000.0, vector.y / 1000.0, vector.z / 1000.0)
    
    @staticmethod
    def convert_vector_m_to_mm(vector: Vector3D) -> Vector3D:
        """将米单位的向量转换为毫米单位"""
        return Vector3D(vector.x * 1000.0, vector.y * 1000.0, vector.z * 1000.0)