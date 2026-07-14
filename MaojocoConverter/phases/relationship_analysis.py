"""
关系分析阶段

分析装配关系，构建装配图和动力学树。
"""

# TODO(重构): type_definitions.py 定义了 KinematicTree dataclass（强类型字段），
# 但本文件构造 kinematic_tree 时用的是 dict 字面量（{'roots':[], 'nodes':{}, ...}），
# 后续读取也按 dict 访问（kinematic_tree['bodies']、.get('roots', [])），
# 导致 dataclass 形同虚设、类型系统未启用。待统一为 dataclass 构造与访问。
# 另：mass/inertia 全部硬编码为 1.0（注释"后续可以计算"未实现）；
# _extract_rotation_axis_from_matrix 依赖"旋转轴=矩阵第二列"的口头约定，耦合脆弱。
# 见 2026-07-14 架构梳理。

from typing import Dict, Any, Optional, List, Tuple
import math
import networkx as nx

# 直接导入F3DMaojocoScripts的common模块
from F3DMaojocoScripts.common.data_types import (
    JointInfo, Transform4D, JointType
)
from F3DMaojocoScripts.common.geometry_math import Vector3D

from ..utils.logger import logger
from ..context import MaojocoContext
from .base import ConversionPhase
from ..type_definitions import (
    Body4DCoordinates, JointGlobalCoordinates, JointPairwiseRelationship,
    KinematicBody, KinematicJoint, KinematicNode, RelativeTransform, KinematicTree,
    BodyName, JointName, AssemblyGraph, AssemblyTree, JointPairKey
)


class RelationshipAnalysisPhase(ConversionPhase):
    """关系分析阶段"""
    
    def __init__(self, ctx: MaojocoContext):
        super().__init__("RelationshipAnalysis", ctx)
    
    def _execute(self) -> bool:
        """执行关系分析"""
        logger.info("🔗 开始分析装配关系")
        
        try:
            # 分析关节两两关系
            self._analyze_joint_pairwise_relationships()
            
            # 构建装配关系图
            self._build_assembly_graph()
            
            # 生成装配树
            self._generate_assembly_trees()
            
            # 分析关节连接关系
            self._analyze_joint_connections()
            
            logger.info("📊 关系分析完成:")
            logger.info(f"    - 关节两两关系: {len(self.ctx.joint_pairwise_relationships)} 个")
            logger.info(f"    - 装配图节点: {self.ctx.assembly_graph.number_of_nodes()}")
            logger.info(f"    - 装配图边: {self.ctx.assembly_graph.number_of_edges()}")
            logger.info(f"    - 装配树: {len(self.ctx.assembly_trees)} 个")
            
            
            return True
            
        except Exception as e:
            logger.error(f"❌ 关系分析失败: {e}")
            return False
    
    def _analyze_joint_pairwise_relationships(self):
        """分析关节两两关系"""
        logger.info("🔍 分析关节两两关系")
        
        joints = self.ctx.raw_export_data.joints
        
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
    
    def _build_assembly_graph(self):
        """构建装配关系图"""
        logger.info("🏗️  构建装配关系图")
        
        # 创建无向图
        self.ctx.assembly_graph = nx.Graph()
        
        # 添加所有零部件作为节点
        for component in self.ctx.raw_export_data.components:
            self.ctx.assembly_graph.add_node(
                component.name,
                component_type='component',
                occurrence_name=component.occurrence_name,
                full_path=component.full_path_name,
                bodies_count=component.bodies_count,
                has_children=component.has_children
            )
        
        # 基于关节连接添加边
        for joint in self.ctx.raw_export_data.joints:
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
                        relationship='joint_connection'
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
    
    def _generate_assembly_trees(self):
        """生成装配树"""
        logger.info("🌳 生成装配树")
        
        if not self.ctx.assembly_graph:
            logger.warning("⚠️  装配图不存在，无法生成装配树")
            return
        
        # 找到所有连通分量
        connected_components = list(nx.connected_components(self.ctx.assembly_graph))
        
        # 检测环结构
        self._detect_and_log_cycles()
        
        self.ctx.assembly_trees = []
        
        for i, component_set in enumerate(connected_components):
            # 为每个连通分量生成一个树
            subgraph = self.ctx.assembly_graph.subgraph(component_set)
            
            # 选择根节点（根据配置策略）
            root_node = self._select_root_node(subgraph)
            
            # 显示原始连接信息
            original_edges = set(subgraph.edges())
            
            # 生成最小生成树（基于连接强度）
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
            
            self.ctx.assembly_trees.append({
                'root': root_node,
                'tree': tree_structure,
                'components': list(component_set),
                'depth': self._calculate_tree_depth(tree_structure)
            })
            
            # 显示断开的连接
            tree_edges = set(mst.edges())
            broken_edges = original_edges - tree_edges
            
            # 展示树结构
            logger.info(f"🌳 装配树 {i+1}:")
            logger.info(f"   📊 根节点: {root_node} (连接度: {subgraph.degree(root_node)})")
            logger.info(f"   📊 组件数: {len(component_set)}, 深度: {self._calculate_tree_depth(tree_structure)}")
            
            # 展示树结构
            logger.info("   🌳 树结构:")
            self._display_tree_structure(tree_structure, "      ")
            
            # 展示断开的连接
            if broken_edges:
                logger.info("   ❌ 断开的连接 (为了断环):")
                for edge in broken_edges:
                    edge_data = subgraph.edges[edge]
                    logger.info(f"      • {edge[0]} ↔ {edge[1]} (关节: {edge_data.get('joint_name', 'unknown')})")
            else:
                logger.info("   ✅ 无需断开连接 (无环结构)")
    
    def _detect_and_log_cycles(self):
        """检测并记录环结构"""
        import networkx as nx
        
        # 检测环
        cycles = list(nx.cycle_basis(self.ctx.assembly_graph))
        
        if cycles:
            logger.info(f"🔄 检测到 {len(cycles)} 个环结构:")
            for i, cycle in enumerate(cycles):
                logger.info(f"   🔄 环 {i+1}: {' → '.join(cycle)} → {cycle[0]}")
                
                # 显示环中的关节信息
                logger.info("   🔗 环中的关节:")
                for j in range(len(cycle)):
                    node1 = cycle[j]
                    node2 = cycle[(j + 1) % len(cycle)]
                    
                    if self.ctx.assembly_graph.has_edge(node1, node2):
                        edge_data = self.ctx.assembly_graph.edges[node1, node2]
                        logger.info(f"      • {node1} ↔ {node2}: {edge_data.get('joint_name', 'unknown')}")
        else:
            logger.info("✅ 无环结构 (已经是树结构)")
    
    def _display_tree_structure(self, node: Dict[str, Any], indent: str, is_last: bool = True):
        """递归显示树结构"""
        node_name = node['name']
        
        # 选择合适的树形符号
        if is_last:
            prefix = "└── "
            next_indent = indent + "    "
        else:
            prefix = "├── "
            next_indent = indent + "│   "
        
        logger.info(f"{indent}{prefix}{node_name}")
        
        # 显示子节点
        children = node.get('children', [])
        if children:
            for j, child in enumerate(children):
                is_last_child = (j == len(children) - 1)
                self._display_tree_structure(child, next_indent, is_last_child)
    
    def _build_tree_structure(self, tree: nx.DiGraph, root: str, parent: Optional[str] = None) -> Dict[str, Any]:
        """构建树结构"""
        node_data = tree.nodes[root]
        
        tree_node = {
            'name': root,
            'data': node_data,
            'children': []
        }
        
        # 递归处理子节点（排除父节点避免重复）
        for child in tree.neighbors(root):
            if child != parent:  # 避免回到父节点
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
    
    def _analyze_joint_connections(self):
        """分析关节连接关系"""
        logger.info("🔗 分析关节连接关系")
        
        # 为每个关节找到连接的零部件
        joint_connections = {}
        
        for joint in self.ctx.raw_export_data.joints:
            connections = []
            
            if joint.connection.occurrence_one_component:
                connections.append(joint.connection.occurrence_one_component)
            if joint.connection.occurrence_two_component:
                connections.append(joint.connection.occurrence_two_component)
            
            joint_connections[joint.name] = {
                'components': connections,
                'type': joint.joint_type.value,
                'is_suppressed': joint.is_suppressed,
                'is_active': joint.is_light_bulb_on
            }
        
        # 在上下文中存储关节连接信息
        self.ctx.joint_connections = joint_connections
        
        # 构建MuJoCo运动学树
        self._build_kinematic_tree()
    
    def _build_kinematic_tree(self):
        """构建MuJoCo运动学树"""
        logger.info("🌲 构建MuJoCo运动学树")
        
        if not self.ctx.assembly_graph:
            logger.error("❌ 装配图不存在，无法构建运动学树")
            return
        
        # 运动学树结构
        kinematic_tree: KinematicTree = {
            'roots': [],
            'nodes': {},
            'joints': {},
            'bodies': {},
            'relative_transforms': {}
        }
        
        # 从装配树构建运动学树
        for assembly_tree in self.ctx.assembly_trees:
            kinematic_root = self._convert_assembly_to_kinematics(
                assembly_tree['tree'], 
                None,  # 父节点
                kinematic_tree
            )
            if kinematic_root:
                kinematic_tree['roots'].append(kinematic_root)
        
        # 计算相对变换
        self._calculate_relative_transforms(kinematic_tree)
        
        # 验证运动学树完整性
        self._validate_kinematic_tree(kinematic_tree)
        
        self.ctx.kinematic_tree = kinematic_tree
        
        logger.info("📊 运动学树构建完成:")
        logger.info(f"    - 根节点: {len(kinematic_tree['roots'])} 个")
        logger.info(f"    - 节点: {len(kinematic_tree['nodes'])} 个")
        logger.info(f"    - 关节: {len(kinematic_tree['joints'])} 个")
        logger.info(f"    - 刚体: {len(kinematic_tree['bodies'])} 个")
        
        # 显示运动学树结构
        logger.info("🌲 运动学树结构:")
        for root in kinematic_tree['roots']:
            logger.info(f"   🌳 根节点: {root}")
            self._display_kinematic_tree_structure(root, kinematic_tree['nodes'], kinematic_tree['bodies'], kinematic_tree['joints'], "      ")
    
    def _convert_assembly_to_kinematics(self, assembly_node: Dict[str, Any], parent_body: Optional[str], kinematic_tree: KinematicTree) -> Optional[str]:
        """将装配树节点转换为动力学树节点"""
        component_name = assembly_node['name']
        
        # 获取组件的4D坐标
        body_4d = self.ctx.body_4d_coordinates.get(component_name)
        if not body_4d:
            logger.warning(f"⚠️  组件 {component_name} 没有4D坐标信息")
            return None
        
        # 创建动力学体
        body_id = f"body_{component_name}"
        
        kinematic_body = KinematicBody(
            body_id=body_id,
            name=component_name,
            component_id=body_4d.component_id,
            occurrence_name=body_4d.occurrence_name,
            world_transform=body_4d.transform,
            stl_file=body_4d.stl_file,
            bodies_count=body_4d.bodies_count,
            mass=1.0,  # 默认质量，后续可以计算
            inertia=[1.0, 1.0, 1.0],  # 默认惯性，后续可以计算
            parent=parent_body
        )
        
        kinematic_tree['bodies'][body_id] = kinematic_body
        
        # 创建运动学节点
        kinematic_node = KinematicNode(
            body_id=body_id,
            parent_body=parent_body,
            children=[],
            joint=None,  # 将在下面设置
            level=0 if parent_body is None else kinematic_tree['nodes'][parent_body].level + 1
        )
        
        kinematic_tree['nodes'][body_id] = kinematic_node
        
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
                    limits=self._get_joint_limits(joint_info),
                    is_suppressed=joint_info.is_suppressed,
                    is_active=joint_info.is_light_bulb_on
                )
                
                kinematic_tree['joints'][joint_id] = kinematic_joint
                
                # 连接节点和关节
                kinematic_tree['nodes'][body_id].joint = joint_id
                kinematic_tree['nodes'][parent_body].children.append(body_id)
        
        # 递归处理子节点（不要重复添加，因为已经在关节连接处理时添加了）
        for child_node in assembly_node['children']:
            self._convert_assembly_to_kinematics(child_node, body_id, kinematic_tree)
        
        return body_id
    
    def _find_joint_between_bodies(self, body1_name: str, body2_name: str) -> Optional[JointInfo]:
        """找到两个体之间的关节"""
        for joint in self.ctx.raw_export_data.joints:
            comp1 = joint.connection.occurrence_one_component
            comp2 = joint.connection.occurrence_two_component
            
            if (comp1 == body1_name and comp2 == body2_name) or (comp1 == body2_name and comp2 == body1_name):
                return joint
        
        return None
    
    def _get_joint_position(self, joint_info: JointInfo) -> Vector3D:
        """获取关节位置（相对于父节点）"""
        # 获取关节的全局位置
        global_joint_position = None
        
        # 优先使用关节几何信息
        if joint_info.geometry.geometry_one_transform:
            global_joint_position = joint_info.geometry.geometry_one_transform.get_translation()
            logger.debug(f"关节 {joint_info.name} 从几何变换获取位置: {global_joint_position}")
        
        # 备用：使用全局坐标
        if not global_joint_position:
            global_coord = self.ctx.joint_global_coordinates.get(joint_info.name)
            if global_coord:
                global_joint_position = global_coord.position
                logger.debug(f"关节 {joint_info.name} 从全局坐标获取位置: {global_joint_position}")
        
        if not global_joint_position:
            logger.warning(f"关节 {joint_info.name} 无法获取位置，使用默认值")
            return Vector3D(0.0, 0.0, 0.0)
        
        # 找到关节连接的父组件和子组件
        parent_component = None
        child_component = None
        
        if joint_info.connection.occurrence_one_component and joint_info.connection.occurrence_two_component:
            # 确定哪个是父组件，哪个是子组件
            # 在运动学树中，父组件是更接近根节点的
            comp1 = joint_info.connection.occurrence_one_component
            comp2 = joint_info.connection.occurrence_two_component
            
            # 检查哪个组件在运动学树中层级更浅（更接近根节点）
            parent_component, child_component = self._find_parent_child_relationship(comp1, comp2)
        
        if not parent_component or not child_component:
            logger.warning(f"关节 {joint_info.name} 无法确定父子关系，返回全局位置")
            return global_joint_position
        
        # 获取父组件的全局变换
        parent_body_4d = self.ctx.body_4d_coordinates.get(parent_component)
        if not parent_body_4d or not parent_body_4d.transform:
            logger.warning(f"关节 {joint_info.name} 父组件 {parent_component} 没有变换信息")
            return global_joint_position
        
        # 计算相对于父节点的位置
        parent_transform = parent_body_4d.transform
        
        # 将关节全局位置转换为相对于父节点的局部位置
        # 局部位置 = 父节点逆矩阵 * 全局位置
        import numpy as np
        
        # 创建齐次坐标的关节位置
        joint_pos_array = np.array([
            [global_joint_position.x],
            [global_joint_position.y], 
            [global_joint_position.z],
            [1.0]
        ])
        
        # 父节点变换矩阵
        parent_matrix = np.array(parent_transform.matrix)
        
        # 计算父节点逆矩阵
        try:
            parent_inverse = np.linalg.inv(parent_matrix)
            
            # 计算相对位置
            relative_pos_array = np.dot(parent_inverse, joint_pos_array)
            
            # 提取位置分量（前三个元素）
            relative_position = Vector3D(
                relative_pos_array[0, 0],
                relative_pos_array[1, 0], 
                relative_pos_array[2, 0]
            )
            
            logger.info(f"关节 {joint_info.name} 位置转换:")
            logger.info(f"  全局位置: ({global_joint_position.x:.6f}, {global_joint_position.y:.6f}, {global_joint_position.z:.6f})")
            logger.info(f"  相对于 {parent_component}: ({relative_position.x:.6f}, {relative_position.y:.6f}, {relative_position.z:.6f})")
            
            return relative_position
            
        except Exception as e:
            logger.error(f"计算关节 {joint_info.name} 相对位置失败: {e}")
            return global_joint_position
    
    def _find_parent_child_relationship(self, comp1: str, comp2: str) -> Tuple[Optional[str], Optional[str]]:
        """确定两个组件之间的父子关系
        
        在运动学树中，层级较浅的（更接近根节点）是父组件
        
        Args:
            comp1: 第一个组件名称
            comp2: 第二个组件名称
            
        Returns:
            Tuple[Optional[str], Optional[str]]: (父组件, 子组件)
        """
        try:
            if not self.ctx.kinematic_tree:
                logger.warning("运动学树不存在，无法确定父子关系")
                return comp1, comp2  # 默认返回
            
            # 在运动学树中查找两个组件对应的body_id
            body1_id = f"body_{comp1}"
            body2_id = f"body_{comp2}"
            
            nodes = self.ctx.kinematic_tree.get('nodes', {})
            
            node1 = nodes.get(body1_id)
            node2 = nodes.get(body2_id)
            
            if not node1 or not node2:
                logger.warning(f"无法在运动学树中找到组件: {comp1 if not node1 else comp2}")
                return comp1, comp2  # 默认返回
            
            # 比较两个节点的层级（level）
            # 层级数值越小，越接近根节点
            if node1.level < node2.level:
                # node1 层级更浅，是父节点
                logger.debug(f"父子关系确定: {comp1} (level {node1.level}) → {comp2} (level {node2.level})")
                return comp1, comp2
            elif node2.level < node1.level:
                # node2 层级更浅，是父节点
                logger.debug(f"父子关系确定: {comp2} (level {node2.level}) → {comp1} (level {node1.level})")
                return comp2, comp1
            else:
                # 同一层级，检查是否存在直接的父子关系
                if node1.parent_body == body2_id:
                    # node2 是 node1 的父节点
                    logger.debug(f"父子关系确定: {comp2} → {comp1} (直接父子)")
                    return comp2, comp1
                elif node2.parent_body == body1_id:
                    # node1 是 node2 的父节点
                    logger.debug(f"父子关系确定: {comp1} → {comp2} (直接父子)")
                    return comp1, comp2
                else:
                    # 同一层级但无直接父子关系，任意指定
                    logger.warning(f"组件 {comp1} 和 {comp2} 在同一层级且无直接父子关系，默认指定 {comp1} 为父")
                    return comp1, comp2
                    
        except Exception as e:
            logger.error(f"确定父子关系失败: {e}")
            return comp1, comp2  # 默认返回
    
    def _get_joint_axis(self, joint_info: JointInfo) -> Optional[Vector3D]:
        """获取关节轴"""
        if joint_info.joint_type == JointType.REVOLUTE:
            # 旋转关节，从几何变换矩阵中提取旋转轴
            if hasattr(joint_info, 'geometry') and joint_info.geometry:
                geom = joint_info.geometry
                if hasattr(geom, 'geometry_one_transform') and geom.geometry_one_transform:
                    # 从变换矩阵的旋转部分提取旋转轴
                    transform = geom.geometry_one_transform
                    if hasattr(transform, 'matrix'):
                        matrix = transform.matrix
                        # 对于旋转关节，旋转轴通常是变换矩阵的第三列（Z轴）
                        # 或者可以从旋转矩阵的特征向量计算
                        rotation_axis = self._extract_rotation_axis_from_matrix(matrix)
                        return rotation_axis
            
            # 如果无法从矩阵提取，使用默认的Z轴
            return Vector3D(0.0, 0.0, 1.0)
        elif joint_info.joint_type == JointType.SLIDER:
            # 滑动关节，假设沿X轴
            return Vector3D(1.0, 0.0, 0.0)
        
        return None
    
    def _extract_rotation_axis_from_matrix(self, matrix: List[List[float]]) -> Vector3D:
        """从4x4变换矩阵中提取旋转轴
        
        根据Fusion 360的joint_analyzer.py注释，变换矩阵格式为:
        [secondary_axis, primary_axis, third_axis, origin]
        
        其中primary_axis是旋转轴，被放在第二列（Y轴位置）。
        """
        try:
            # 提取3x3旋转矩阵
            rotation_matrix = [
                [matrix[0][0], matrix[0][1], matrix[0][2]],
                [matrix[1][0], matrix[1][1], matrix[1][2]],
                [matrix[2][0], matrix[2][1], matrix[2][2]]
            ]
            
            # 从第二列提取旋转轴（primary_axis，Fusion 360的旋转轴）
            fusion_axis_x = rotation_matrix[0][1]  # 第二列第一行
            fusion_axis_y = rotation_matrix[1][1]  # 第二列第二行  
            fusion_axis_z = rotation_matrix[2][1]  # 第二列第三行
            
            logger.info(f"🔍 从矩阵第二列提取Fusion 360旋转轴(primary_axis): ({fusion_axis_x:.6f}, {fusion_axis_y:.6f}, {fusion_axis_z:.6f})")
            
            # 归一化向量
            length = (fusion_axis_x**2 + fusion_axis_y**2 + fusion_axis_z**2)**0.5
            if length > 1e-10:
                fusion_axis_x /= length
                fusion_axis_y /= length
                fusion_axis_z /= length
            
            fusion_axis = (fusion_axis_x, fusion_axis_y, fusion_axis_z)
            logger.info(f"🔍 Fusion 360旋转轴: {fusion_axis}")
            
            # 直接使用Fusion 360提供的旋转轴
            result = Vector3D(fusion_axis_x, fusion_axis_y, fusion_axis_z)
            logger.info(f"🔍 MuJoCo使用提取的旋转轴: {result}")
            logger.info(f"📝 根据joint_analyzer.py注释，从第二列提取primary_axis作为旋转轴")
            
            return result
                
        except Exception as e:
            logger.error(f"❌ 从矩阵提取旋转轴失败: {e}")
            logger.error(f"   矩阵数据: {matrix}")
            # 失败时使用默认的Y轴
            logger.warning(f"🔄 使用默认Y轴作为旋转轴")
            return Vector3D(0.0, 1.0, 0.0)
    
    def _get_joint_limits(self, joint_info: JointInfo) -> Optional[Dict[str, Any]]:
        """获取关节限制"""
        if joint_info.limits:
            return {
                'has_limits': joint_info.limits.has_limits,
                'range': [joint_info.limits.minimum_value, joint_info.limits.maximum_value] if joint_info.limits.has_limits else None
            }
        
        return None
    
    def _calculate_relative_transforms(self, kinematic_tree: KinematicTree):
        """计算相对变换"""
        logger.info("📐 计算相对变换")
        
        # 为每个根节点计算相对变换
        for root_body_id in kinematic_tree['roots']:
            self._calculate_body_relative_transform(root_body_id, None, kinematic_tree)
    
    def _calculate_body_relative_transform(self, body_id: str, parent_body_id: Optional[str], kinematic_tree: KinematicTree):
        """计算单个体的相对变换"""
        body = kinematic_tree['bodies'][body_id]
        
        if parent_body_id is None:
            # 根节点，相对变换就是世界变换
            relative_transform = body.world_transform
        else:
            # 计算相对于父节点的变换
            parent_body = kinematic_tree['bodies'][parent_body_id]
            relative_transform = self._compute_relative_transform(
                body.world_transform,
                parent_body.world_transform
            )
        
        # 存储相对变换
        relative_transform_obj = RelativeTransform(
            parent=parent_body_id,
            transform=relative_transform
        )
        
        kinematic_tree['relative_transforms'][body_id] = relative_transform_obj
        
        # 递归处理子节点
        node = kinematic_tree['nodes'][body_id]
        for child_body_id in node.children:
            self._calculate_body_relative_transform(child_body_id, body_id, kinematic_tree)
    
    def _compute_relative_transform(self, child_transform: Transform4D, parent_transform: Transform4D) -> Transform4D:
        """计算相对变换矩阵"""
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
    
    def _validate_kinematic_tree(self, kinematic_tree: KinematicTree):
        """验证运动学树完整性"""
        logger.info("✅ 验证运动学树完整性")
        
        # 检查所有节点都有父节点引用（除了根节点）
        for body_id, node in kinematic_tree['nodes'].items():
            if body_id not in kinematic_tree['roots'] and node.parent_body is None:
                logger.warning(f"⚠️  节点 {body_id} 不是根节点但没有父节点")
        
        # 检查父子关系一致性
        for body_id, node in kinematic_tree['nodes'].items():
            if node.parent_body:
                parent_node = kinematic_tree['nodes'].get(node.parent_body)
                if parent_node and body_id not in parent_node.children:
                    logger.warning(f"⚠️  父子关系不一致: {body_id} 不在 {node.parent_body} 的子节点列表中")
        
        logger.info("✅ 运动学树验证通过")
    
    def _find_center_node(self, graph: nx.Graph) -> str:
        """找到图的中心节点（最小化到所有其他节点的最大距离）"""
        try:
            # 计算所有节点的偏心度（到最远节点的距离）
            eccentricity = nx.eccentricity(graph)
            
            # 找到偏心度最小的节点（中心节点）
            center_node = min(eccentricity.keys(), key=lambda n: eccentricity[n])
            min_eccentricity = eccentricity[center_node]
            
            logger.info(f"🎯 选择的中心节点: {center_node} (偏心度: {min_eccentricity})")
            
            # 显示所有候选的偏心度对比
            candidates = sorted(eccentricity.items(), key=lambda x: x[1])
            logger.info(f"📊 节点偏心度排名:")
            for i, (node, ecc) in enumerate(candidates[:3]):  # 显示前3个
                logger.info(f"   {i+1}. {node}: 偏心度 {ecc}")
            
            return center_node
            
        except Exception as e:
            logger.warning(f"⚠️  计算中心节点失败: {e}")
            # 降级为连接度最大的节点
            fallback_node = max(graph.nodes(), key=lambda n: graph.degree(n))
            logger.info(f"🔄 降级策略: 选择连接度最大的节点 {fallback_node}")
            return fallback_node
    
    def _select_root_node(self, graph: nx.Graph) -> str:
        """根据配置策略选择根节点"""
        strategy = self.ctx.config.root_node_strategy
        
        logger.info(f"🎯 根节点选择策略: {strategy}")
        
        if strategy == "manual":
            # 手动指定根节点
            manual_root = self.ctx.config.manual_root_node
            if manual_root and manual_root in graph.nodes():
                logger.info(f"📍 使用手动指定的根节点: {manual_root}")
                return manual_root
            else:
                logger.warning(f"⚠️  手动指定的根节点 '{manual_root}' 不存在，降级为其他策略")
                strategy = "center"  # 降级策略
        
        if strategy == "center":
            # 选择图的中心节点
            return self._find_center_node(graph)
        elif strategy == "max_degree":
            # 选择连接度最大的节点
            max_degree_node = max(graph.nodes(), key=lambda n: graph.degree(n))
            max_degree = graph.degree(max_degree_node)
            logger.info(f"🎯 选择连接度最大的节点: {max_degree_node} (连接度: {max_degree})")
            return max_degree_node
        else:
            logger.warning(f"⚠️  未知的根节点选择策略: {strategy}，使用默认策略 'center'")
            return self._find_center_node(graph)
    
    def _display_kinematic_tree_structure(self, body_id: str, nodes: Dict, bodies: Dict, joints: Dict, indent: str):
        """递归显示运动学树结构"""
        node = nodes.get(body_id)
        body = bodies.get(body_id)
        
        if not node or not body:
            return
        
        # 显示身体信息
        children = node.children
        is_last = len(children) == 0  # 如果没有子节点，是最后一个
        
        # 选择合适的树形符号
        if node.parent_body is None:
            # 根节点
            prefix = "🌳 "
            next_indent = indent + "    "
        else:
            # 子节点
            siblings = nodes.get(node.parent_body).children
            is_last_child = body_id == siblings[-1] if siblings else True
            
            if is_last_child:
                prefix = "└── "
                next_indent = indent + "    "
            else:
                prefix = "├── "
                next_indent = indent + "│   "
        
        logger.info(f"{indent}{prefix}{body.name} ({body_id})")
        
        # 显示关节信息
        if node.joint:
            joint = joints.get(node.joint)
            if joint:
                logger.info(f"{next_indent}   🔗 关节: {joint.name} ({joint.joint_type.value})")
                logger.info(f"{next_indent}   📍 位置: ({joint.position.x:.3f}, {joint.position.y:.3f}, {joint.position.z:.3f})")
        
        # 显示子节点
        for i, child_id in enumerate(children):
            is_last_child = (i == len(children) - 1)
            self._display_kinematic_tree_structure(child_id, nodes, bodies, joints, next_indent)