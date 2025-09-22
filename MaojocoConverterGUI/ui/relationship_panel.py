"""
关系分析面板

提供装配关系分析的可视化界面，包括环检测、根节点选择和装配树构建功能。
"""

from typing import Dict, List, Optional, Any, Set, Tuple
from pathlib import Path
from dataclasses import dataclass

import networkx as nx
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QSplitter,
    QLabel, QPushButton, QTreeWidget, QTreeWidgetItem,
    QTableWidget, QTableWidgetItem, QHeaderView,
    QGroupBox, QFormLayout, QComboBox, QSpinBox,
    QTextEdit, QMessageBox, QProgressBar, QTabWidget
)
from PySide6.QtCore import Signal, Qt, QTimer
from PySide6.QtGui import QColor, QFont

from ..utils.logger import logger
from ..core.project_workflow_manager import ProjectWorkflowManager
from ..core.domain_types import JointInfo, ComponentInfo, JointType, LoadResult, JointConnection, JointGeometry
from .ui_workflow_adapter import UIWorkflowAdapter


@dataclass
class CycleInfo:
    """环结构信息"""
    cycle_id: int
    nodes: List[str]
    edges: List[Tuple[str, str]]
    joints: List[str]
    length: int


@dataclass
class AssemblyTreeInfo:
    """装配树信息"""
    root_node: str
    tree_depth: int
    node_count: int
    broken_edges: List[Tuple[str, str]]
    tree_structure: Dict[str, Any]


class RelationshipAnalysisPanel(QWidget):
    """关系分析主面板"""
    
    # 信号定义
    root_node_changed = Signal(str)           # 根节点改变信号
    cycle_break_requested = Signal(int, str)   # 断环请求信号
    tree_structure_changed = Signal(object)   # 树结构改变信号
    component_selected = Signal(str)           # 组件选择信号
    joint_selected = Signal(str)              # 关节选择信号
    
    def __init__(self, workflow_manager: ProjectWorkflowManager, 
                 workflow_adapter: UIWorkflowAdapter, parent=None):
        """
        初始化关系分析面板
        
        Args:
            workflow_manager: 工作流管理器
            workflow_adapter: 工作流适配器
            parent: 父组件
        """
        super().__init__(parent)
        self._workflow_manager = workflow_manager
        self._workflow_adapter = workflow_adapter
        
        # 数据存储
        self._current_project: Optional[Path] = None
        self._components: List[ComponentInfo] = []
        self._joints: List[JointInfo] = []
        self._assembly_graph: Optional[nx.Graph] = None
        self._cycles: List[CycleInfo] = []
        self._assembly_trees: List[AssemblyTreeInfo] = []
        self._selected_root_node: Optional[str] = None
        
        # 快速启动模式标志
        self._quick_start_mode: bool = False
        
        # UI初始化
        self._setup_ui()
        
        # 定时器用于延迟处理
        self._analysis_timer = QTimer()
        self._analysis_timer.setSingleShot(True)
        self._analysis_timer.timeout.connect(self._perform_analysis)
    
    def _setup_ui(self):
        """设置UI界面"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)
        
        # 标题
        title_label = QLabel("关系分析")
        title_label.setStyleSheet("font-size: 18px; font-weight: bold; color: #333;")
        layout.addWidget(title_label)
        
        # 创建选项卡布局
        tab_widget = QTabWidget()
        layout.addWidget(tab_widget)
        
        # === 环检测选项卡 ===
        cycle_tab = self._create_cycle_detection_tab()
        tab_widget.addTab(cycle_tab, "环检测")
        
        # === 装配树选项卡 ===
        tree_tab = self._create_assembly_tree_tab()
        tab_widget.addTab(tree_tab, "装配树")
        
        # === 关系统计选项卡 ===
        stats_tab = self._create_relationship_stats_tab()
        tab_widget.addTab(stats_tab, "关系统计")
        
        # 控制按钮
        control_layout = QHBoxLayout()
        
        self.analyze_btn = QPushButton("分析关系")
        self.analyze_btn.setMinimumHeight(40)
        self.analyze_btn.clicked.connect(self._start_analysis)
        control_layout.addWidget(self.analyze_btn)
        
        self.auto_tree_btn = QPushButton("自动生成树")
        self.auto_tree_btn.setMinimumHeight(40)
        self.auto_tree_btn.clicked.connect(self._auto_generate_tree)
        control_layout.addWidget(self.auto_tree_btn)
        
        control_layout.addStretch()
        
        layout.addLayout(control_layout)
        
        # 分析进度条
        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)
        layout.addWidget(self.progress_bar)
        
        # 状态显示
        self.status_label = QLabel("准备就绪")
        self.status_label.setStyleSheet("color: #666; font-size: 12px;")
        layout.addWidget(self.status_label)
    
    def _create_cycle_detection_tab(self) -> QWidget:
        """创建环检测选项卡"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)
        
        # 环检测结果组
        cycle_group = QGroupBox("环检测结果")
        cycle_layout = QVBoxLayout(cycle_group)
        
        # 环统计信息
        self.cycle_stats_label = QLabel("尚未进行环检测")
        cycle_layout.addWidget(self.cycle_stats_label)
        
        # 环列表
        self.cycle_table = QTableWidget()
        self.cycle_table.setColumnCount(4)
        self.cycle_table.setHorizontalHeaderLabels(["环ID", "长度", "组件数", "关节数"])
        self.cycle_table.horizontalHeader().setStretchLastSection(True)
        self.cycle_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.cycle_table.itemSelectionChanged.connect(self._on_cycle_selected)
        cycle_layout.addWidget(self.cycle_table)
        
        layout.addWidget(cycle_group)
        
        # 环详情组
        details_group = QGroupBox("环详情")
        details_layout = QVBoxLayout(details_group)
        
        self.cycle_details_text = QTextEdit()
        self.cycle_details_text.setReadOnly(True)
        self.cycle_details_text.setMaximumHeight(200)
        details_layout.addWidget(self.cycle_details_text)
        
        layout.addWidget(details_group)
        
        # 断环控制
        break_group = QGroupBox("断环控制")
        break_layout = QHBoxLayout(break_group)
        
        self.auto_break_btn = QPushButton("自动断环")
        self.auto_break_btn.clicked.connect(self._auto_break_cycles)
        break_layout.addWidget(self.auto_break_btn)
        
        self.manual_break_btn = QPushButton("手动断环")
        self.manual_break_btn.clicked.connect(self._manual_break_cycles)
        break_layout.addWidget(self.manual_break_btn)
        
        break_layout.addStretch()
        
        layout.addWidget(break_group)
        
        return widget
    
    def _create_assembly_tree_tab(self) -> QWidget:
        """创建装配树选项卡"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)
        
        # 水平分割器
        splitter = QSplitter(Qt.Horizontal)
        
        # 左侧：根节点选择和树结构
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        
        # 根节点选择
        root_group = QGroupBox("根节点选择")
        root_layout = QFormLayout(root_group)
        
        self.root_node_combo = QComboBox()
        self.root_node_combo.currentTextChanged.connect(self._on_root_node_changed)
        root_layout.addRow("根节点:", self.root_node_combo)
        
        self.root_strategy_combo = QComboBox()
        self.root_strategy_combo.addItems(["center", "max_degree", "manual"])
        self.root_strategy_combo.currentTextChanged.connect(self._on_strategy_changed)
        root_layout.addRow("选择策略:", self.root_strategy_combo)
        
        left_layout.addWidget(root_group)
        
        # 装配树显示
        tree_group = QGroupBox("装配树结构")
        tree_layout = QVBoxLayout(tree_group)
        
        self.tree_widget = QTreeWidget()
        self.tree_widget.setHeaderLabel("装配树")
        self.tree_widget.itemClicked.connect(self._on_tree_item_clicked)
        tree_layout.addWidget(self.tree_widget)
        
        left_layout.addWidget(tree_group)
        
        splitter.addWidget(left_widget)
        
        # 右侧：树信息和断开连接
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        
        # 树信息
        info_group = QGroupBox("树信息")
        info_layout = QFormLayout(info_group)
        
        self.tree_depth_label = QLabel("-")
        self.tree_nodes_label = QLabel("-")
        self.tree_roots_label = QLabel("-")
        
        info_layout.addRow("树深度:", self.tree_depth_label)
        info_layout.addRow("节点数:", self.tree_nodes_label)
        info_layout.addRow("根节点数:", self.tree_roots_label)
        
        right_layout.addWidget(info_group)
        
        # 断开的连接
        broken_group = QGroupBox("断开的连接")
        broken_layout = QVBoxLayout(broken_group)
        
        self.broken_table = QTableWidget()
        self.broken_table.setColumnCount(3)
        self.broken_table.setHorizontalHeaderLabels(["组件1", "组件2", "关节"])
        self.broken_table.horizontalHeader().setStretchLastSection(True)
        broken_layout.addWidget(self.broken_table)
        
        right_layout.addWidget(broken_group)
        
        splitter.addWidget(right_widget)
        
        # 设置分割器比例
        splitter.setSizes([400, 300])
        
        layout.addWidget(splitter)
        
        return widget
    
    def _create_relationship_stats_tab(self) -> QWidget:
        """创建关系统计选项卡"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)
        
        # 基本统计
        basic_group = QGroupBox("基本统计")
        basic_layout = QFormLayout(basic_group)
        
        self.components_count_label = QLabel("0")
        self.joints_count_label = QLabel("0")
        self.connections_count_label = QLabel("0")
        self.graph_density_label = QLabel("0.0")
        
        basic_layout.addRow("组件数:", self.components_count_label)
        basic_layout.addRow("关节数:", self.joints_count_label)
        basic_layout.addRow("连接数:", self.connections_count_label)
        basic_layout.addRow("图密度:", self.graph_density_label)
        
        layout.addWidget(basic_group)
        
        # 关节类型统计
        joint_group = QGroupBox("关节类型统计")
        joint_layout = QVBoxLayout(joint_group)
        
        self.joint_type_table = QTableWidget()
        self.joint_type_table.setColumnCount(3)
        self.joint_type_table.setHorizontalHeaderLabels(["关节类型", "数量", "占比"])
        self.joint_type_table.horizontalHeader().setStretchLastSection(True)
        joint_layout.addWidget(self.joint_type_table)
        
        layout.addWidget(joint_group)
        
        # 连接度统计
        degree_group = QGroupBox("连接度统计")
        degree_layout = QVBoxLayout(degree_group)
        
        self.degree_stats_text = QTextEdit()
        self.degree_stats_text.setReadOnly(True)
        self.degree_stats_text.setMaximumHeight(150)
        degree_layout.addWidget(self.degree_stats_text)
        
        layout.addWidget(degree_group)
        
        return widget
    
    def load_project_data(self, project_path: Path, load_result: LoadResult, quick_start: bool = False):
        """
        加载项目数据
        
        Args:
            project_path: 项目路径
            load_result: 加载结果
            quick_start: 是否为快速启动模式
        """
        self._current_project = project_path
        self._quick_start_mode = quick_start
        
        # 从工作流管理器获取组件和关节数据
        try:
            state = self._workflow_manager.get_state()
            if state.raw_export_data:
                # 转换为ComponentInfo对象
                self._components = self._convert_to_component_info(state.raw_export_data.components)
                # 转换为JointInfo对象
                self._joints = self._convert_to_joint_info(state.raw_export_data.joints)
        except Exception as e:
            logger.error(f"获取项目数据失败: {e}")
            self._components = []
            self._joints = []
        
        # 更新UI状态
        self._update_ui_for_project_loaded()
        
        # 自动开始分析
        self._start_analysis()
    
    def _convert_to_component_info(self, raw_components: List[Any]) -> List[ComponentInfo]:
        """将原始组件数据转换为ComponentInfo对象"""
        components = []
        for raw_comp in raw_components:
            comp = ComponentInfo(
                name=getattr(raw_comp, 'name', ''),
                occurrence_name=getattr(raw_comp, 'occurrence_name', ''),
                full_path_name=getattr(raw_comp, 'full_path_name', ''),
                component_id=getattr(raw_comp, 'component_id', 0),
                stl_file=getattr(raw_comp, 'stl_file', None),
                world_transform=getattr(raw_comp, 'world_transform', None),
                bodies_count=getattr(raw_comp, 'bodies_count', 0),
                has_children=getattr(raw_comp, 'has_children', False)
            )
            components.append(comp)
        return components
    
    def _convert_to_joint_info(self, raw_joints: List[Any]) -> List[JointInfo]:
        """将原始关节数据转换为JointInfo对象"""
        joints = []
        for raw_joint in raw_joints:
            # 如果已经是JointInfo对象，直接使用
            if isinstance(raw_joint, JointInfo):
                joints.append(raw_joint)
                continue
                
            # 创建JointConnection对象
            connection = JointConnection(
                occurrence_one_name=getattr(raw_joint, 'occurrence_one_name', None),
                occurrence_one_full_path=getattr(raw_joint, 'occurrence_one_full_path', None),
                occurrence_one_component=getattr(raw_joint, 'occurrence_one_component', None),
                occurrence_two_name=getattr(raw_joint, 'occurrence_two_name', None),
                occurrence_two_full_path=getattr(raw_joint, 'occurrence_two_full_path', None),
                occurrence_two_component=getattr(raw_joint, 'occurrence_two_component', None)
            )
            
            # 创建JointGeometry对象
            geometry = JointGeometry(
                geometry_one_transform=getattr(raw_joint, 'geometry_one_transform', None),
                geometry_two_transform=getattr(raw_joint, 'geometry_two_transform', None)
            )
            
            # 安全地获取关节类型
            joint_type_value = getattr(raw_joint, 'joint_type', 'rigid')
            if isinstance(joint_type_value, JointType):
                # 如果已经是JointType枚举，直接使用
                joint_type = joint_type_value
            else:
                # 如果是字符串，转换为JointType枚举
                try:
                    joint_type = JointType(joint_type_value)
                except ValueError:
                    logger.warning(f"无效的关节类型: {joint_type_value}，使用默认值 RIGID")
                    joint_type = JointType.RIGID
            
            # 创建JointInfo对象
            joint = JointInfo(
                name=getattr(raw_joint, 'name', ''),
                joint_type=joint_type,
                connection=connection,
                geometry=geometry,
                is_suppressed=getattr(raw_joint, 'is_suppressed', False),
                is_light_bulb_on=getattr(raw_joint, 'is_light_bulb_on', True)
            )
            joints.append(joint)
        return joints
    
    def _update_ui_for_project_loaded(self):
        """更新项目加载后的UI状态"""
        if not self._components or not self._joints:
            self.status_label.setText("项目数据不完整，无法进行分析")
            self.analyze_btn.setEnabled(False)
            return
        
        self.analyze_btn.setEnabled(True)
        self.status_label.setText(f"已加载 {len(self._components)} 个组件，{len(self._joints)} 个关节")
        
        # 更新根节点选择器
        self.root_node_combo.clear()
        for comp in self._components:
            self.root_node_combo.addItem(comp.name)
        
        # 如果有配置的根节点，选择它
        if self._selected_root_node:
            index = self.root_node_combo.findText(self._selected_root_node)
            if index >= 0:
                self.root_node_combo.setCurrentIndex(index)
    
    def _start_analysis(self):
        """开始关系分析"""
        if not self._quick_start_mode and (not self._components or not self._joints):
            QMessageBox.warning(self, "警告", "请先加载项目数据")
            return
        
        self.status_label.setText("正在分析关系...")
        self.analyze_btn.setEnabled(False)
        self.progress_bar.setVisible(True)
        self.progress_bar.setRange(0, 0)  # 无限进度条
        
        # 延迟执行分析，避免阻塞UI
        self._analysis_timer.start(100)
    
    def _perform_analysis(self):
        """执行关系分析"""
        try:
            # 构建装配图
            self._build_assembly_graph()
            
            # 检测环结构
            self._detect_cycles()
            
            # 生成装配树
            self._generate_assembly_trees()
            
            # 更新统计信息
            self._update_statistics()
            
            # 更新UI显示
            self._update_all_displays()
            
            self.status_label.setText("关系分析完成")
            
        except Exception as e:
            logger.error(f"关系分析失败: {e}")
            self.status_label.setText(f"分析失败: {str(e)}")
            QMessageBox.critical(self, "错误", f"关系分析失败:\n{str(e)}")
        
        finally:
            self.analyze_btn.setEnabled(True)
            self.progress_bar.setVisible(False)
    
    def _build_assembly_graph(self):
        """构建装配关系图"""
        logger.info("构建装配关系图")
        
        # 创建无向图
        self._assembly_graph = nx.Graph()
        
        # 添加所有组件作为节点
        for comp in self._components:
            self._assembly_graph.add_node(
                comp.name,
                component_type='component',
                occurrence_name=comp.occurrence_name,
                full_path=comp.full_path_name,
                bodies_count=comp.bodies_count,
                has_children=comp.has_children,
                component_id=comp.component_id
            )
        
        # 基于关节连接添加边
        for joint in self._joints:
            if joint.connection.occurrence_one_component and joint.connection.occurrence_two_component:
                comp1 = joint.connection.occurrence_one_component
                comp2 = joint.connection.occurrence_two_component
                
                # 确保节点存在
                if comp1 in self._assembly_graph and comp2 in self._assembly_graph:
                    # 添加边，包含关节信息
                    self._assembly_graph.add_edge(
                        comp1, comp2,
                        joint_name=joint.name,
                        joint_type=joint.joint_type.value,
                        relationship='joint_connection',
                        is_suppressed=joint.is_suppressed,
                        is_active=joint.is_light_bulb_on
                    )
        
        logger.info(f"装配图构建完成: {self._assembly_graph.number_of_nodes()} 节点, {self._assembly_graph.number_of_edges()} 边")
    
    def _detect_cycles(self):
        """检测环结构"""
        logger.info("检测环结构")
        
        self._cycles = []
        
        if not self._assembly_graph:
            return
        
        # 检测环
        cycles = list(nx.cycle_basis(self._assembly_graph))
        
        for i, cycle in enumerate(cycles):
            # 获取环中的边和关节
            edges = []
            joints = []
            
            for j in range(len(cycle)):
                node1 = cycle[j]
                node2 = cycle[(j + 1) % len(cycle)]
                
                if self._assembly_graph.has_edge(node1, node2):
                    edge_data = self._assembly_graph.edges[node1, node2]
                    edges.append((node1, node2))
                    joints.append(edge_data.get('joint_name', 'unknown'))
            
            cycle_info = CycleInfo(
                cycle_id=i + 1,
                nodes=cycle,
                edges=edges,
                joints=joints,
                length=len(cycle)
            )
            self._cycles.append(cycle_info)
        
        logger.info(f"检测到 {len(self._cycles)} 个环结构")
    
    def _generate_assembly_trees(self):
        """生成装配树"""
        logger.info("生成装配树")
        
        self._assembly_trees = []
        
        if not self._assembly_graph:
            return
        
        # 找到所有连通分量
        connected_components = list(nx.connected_components(self._assembly_graph))
        
        for i, component_set in enumerate(connected_components):
            # 为每个连通分量生成一个树
            subgraph = self._assembly_graph.subgraph(component_set)
            
            # 选择根节点
            root_node = self._select_root_node(subgraph)
            
            # 显示原始连接信息
            original_edges = set(subgraph.edges())
            
            # 生成最小生成树
            try:
                mst = nx.minimum_spanning_tree(subgraph)
            except:
                # 如果失败，使用BFS树
                mst = nx.bfs_tree(subgraph, root_node)
            
            # 构建树结构
            tree_structure = self._build_tree_structure(mst, root_node)
            
            # 计算断开的连接
            tree_edges = set(mst.edges())
            broken_edges = original_edges - tree_edges
            
            tree_info = AssemblyTreeInfo(
                root_node=root_node,
                tree_depth=self._calculate_tree_depth(tree_structure),
                node_count=len(component_set),
                broken_edges=list(broken_edges),
                tree_structure=tree_structure
            )
            
            self._assembly_trees.append(tree_info)
        
        logger.info(f"生成了 {len(self._assembly_trees)} 个装配树")
    
    def _select_root_node(self, graph: nx.Graph) -> str:
        """选择根节点"""
        strategy = self.root_strategy_combo.currentText()
        
        if strategy == "manual" and self._selected_root_node:
            if self._selected_root_node in graph.nodes():
                return self._selected_root_node
        
        if strategy == "max_degree":
            # 选择连接度最大的节点
            return max(graph.nodes(), key=lambda n: graph.degree(n))
        else:  # center or default
            # 选择中心节点
            try:
                eccentricity = nx.eccentricity(graph)
                return min(eccentricity.keys(), key=lambda n: eccentricity[n])
            except:
                # 降级为连接度最大的节点
                return max(graph.nodes(), key=lambda n: graph.degree(n))
    
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
    
    def _update_statistics(self):
        """更新统计信息"""
        if not self._assembly_graph:
            return
        
        # 基本统计
        node_count = self._assembly_graph.number_of_nodes()
        edge_count = self._assembly_graph.number_of_edges()
        
        # 计算图密度
        max_edges = node_count * (node_count - 1) / 2
        density = edge_count / max_edges if max_edges > 0 else 0.0
        
        self.components_count_label.setText(str(node_count))
        self.joints_count_label.setText(str(len(self._joints)))
        self.connections_count_label.setText(str(edge_count))
        self.graph_density_label.setText(f"{density:.3f}")
        
        # 关节类型统计
        joint_type_counts = {}
        for joint in self._joints:
            joint_type = joint.joint_type.value
            joint_type_counts[joint_type] = joint_type_counts.get(joint_type, 0) + 1
        
        self.joint_type_table.setRowCount(len(joint_type_counts))
        for i, (joint_type, count) in enumerate(joint_type_counts.items()):
            self.joint_type_table.setItem(i, 0, QTableWidgetItem(joint_type))
            self.joint_type_table.setItem(i, 1, QTableWidgetItem(str(count)))
            percentage = count / len(self._joints) * 100
            self.joint_type_table.setItem(i, 2, QTableWidgetItem(f"{percentage:.1f}%"))
        
        # 连接度统计
        degrees = [self._assembly_graph.degree(n) for n in self._assembly_graph.nodes()]
        if degrees:
            avg_degree = sum(degrees) / len(degrees)
            max_degree = max(degrees)
            min_degree = min(degrees)
            
            degree_text = f"""连接度统计:
- 平均连接度: {avg_degree:.2f}
- 最大连接度: {max_degree}
- 最小连接度: {min_degree}
- 连接度分布:"""
            
            # 统计连接度分布
            degree_dist = {}
            for degree in degrees:
                degree_dist[degree] = degree_dist.get(degree, 0) + 1
            
            for degree, count in sorted(degree_dist.items()):
                degree_text += f"\n  - {degree}个连接: {count}个节点"
            
            self.degree_stats_text.setText(degree_text)
    
    def _update_all_displays(self):
        """更新所有显示内容"""
        self._update_cycle_display()
        self._update_tree_display()
        self._update_info_display()
    
    def _update_cycle_display(self):
        """更新环检测显示"""
        # 更新环统计
        cycle_text = f"检测到 {len(self._cycles)} 个环结构"
        if self._cycles:
            avg_length = sum(cycle.length for cycle in self._cycles) / len(self._cycles)
            cycle_text += f"，平均长度: {avg_length:.1f}"
        self.cycle_stats_label.setText(cycle_text)
        
        # 更新环列表
        self.cycle_table.setRowCount(len(self._cycles))
        for i, cycle in enumerate(self._cycles):
            self.cycle_table.setItem(i, 0, QTableWidgetItem(f"环{cycle.cycle_id}"))
            self.cycle_table.setItem(i, 1, QTableWidgetItem(str(cycle.length)))
            self.cycle_table.setItem(i, 2, QTableWidgetItem(str(len(cycle.nodes))))
            self.cycle_table.setItem(i, 3, QTableWidgetItem(str(len(cycle.joints))))
    
    def _update_tree_display(self):
        """更新装配树显示"""
        # 清空树
        self.tree_widget.clear()
        
        if not self._assembly_trees:
            return
        
        # 显示第一个树
        tree_info = self._assembly_trees[0]
        self._populate_tree_widget(tree_info.tree_structure, self.tree_widget)
        
        # 展开所有节点
        self.tree_widget.expandAll()
    
    def _populate_tree_widget(self, tree_node: Dict[str, Any], parent_item: QTreeWidgetItem):
        """递归填充树控件"""
        # 创建当前节点
        node_name = tree_node['name']
        node_data = tree_node['data']
        
        # 显示信息：名称 + 连接度
        display_text = f"{node_name}"
        if 'component_type' in node_data:
            display_text += f" (ID: {node_data.get('component_id', 'N/A')})"
        
        item = QTreeWidgetItem(display_text)
        item.setData(0, Qt.UserRole, node_name)  # 存储组件名
        
        # 添加到父节点
        if parent_item is self.tree_widget:
            self.tree_widget.addTopLevelItem(item)
        else:
            parent_item.addChild(item)
        
        # 递归处理子节点
        for child in tree_node['children']:
            self._populate_tree_widget(child, item)
    
    def _update_info_display(self):
        """更新信息显示"""
        if not self._assembly_trees:
            return
        
        tree_info = self._assembly_trees[0]
        
        # 更新树信息
        self.tree_depth_label.setText(str(tree_info.tree_depth))
        self.tree_nodes_label.setText(str(tree_info.node_count))
        self.tree_roots_label.setText(str(len(self._assembly_trees)))
        
        # 更新断开的连接表
        self.broken_table.setRowCount(len(tree_info.broken_edges))
        for i, (comp1, comp2) in enumerate(tree_info.broken_edges):
            self.broken_table.setItem(i, 0, QTableWidgetItem(comp1))
            self.broken_table.setItem(i, 1, QTableWidgetItem(comp2))
            
            # 获取关节名称
            if self._assembly_graph.has_edge(comp1, comp2):
                edge_data = self._assembly_graph.edges[comp1, comp2]
                joint_name = edge_data.get('joint_name', 'unknown')
                self.broken_table.setItem(i, 2, QTableWidgetItem(joint_name))
    
    def _on_cycle_selected(self):
        """环选择事件处理"""
        selected_items = self.cycle_table.selectedItems()
        if not selected_items:
            return
        
        row = selected_items[0].row()
        if row < len(self._cycles):
            cycle = self._cycles[row]
            
            # 显示环详情
            details = f"环 {cycle.cycle_id} 详情:\n"
            details += f"- 长度: {cycle.length}\n"
            details += f"- 组件: {' → '.join(cycle.nodes)} → {cycle.nodes[0]}\n"
            details += f"- 关节: {', '.join(cycle.joints)}\n\n"
            
            details += "环中的连接:\n"
            for i, (node1, node2) in enumerate(cycle.edges):
                if self._assembly_graph.has_edge(node1, node2):
                    edge_data = self._assembly_graph.edges[node1, node2]
                    joint_name = edge_data.get('joint_name', 'unknown')
                    joint_type = edge_data.get('joint_type', 'unknown')
                    details += f"  {i+1}. {node1} ↔ {node2} ({joint_name}, {joint_type})\n"
            
            self.cycle_details_text.setText(details)
    
    def _on_root_node_changed(self, node_name: str):
        """根节点改变事件处理"""
        if node_name:
            self._selected_root_node = node_name
            self.root_node_changed.emit(node_name)
            
            # 重新生成树
            self._generate_assembly_trees()
            self._update_tree_display()
            self._update_info_display()
    
    def _on_strategy_changed(self, strategy: str):
        """策略改变事件处理"""
        if strategy == "manual":
            # 手动模式，允许用户选择
            self.root_node_combo.setEnabled(True)
        else:
            # 自动模式，禁用选择
            self.root_node_combo.setEnabled(False)
            # 重新生成树
            self._generate_assembly_trees()
            self._update_tree_display()
            self._update_info_display()
    
    def _on_tree_item_clicked(self, item: QTreeWidgetItem, column: int):
        """树节点点击事件处理"""
        component_name = item.data(0, Qt.UserRole)
        if component_name:
            self.component_selected.emit(component_name)
    
    def _auto_break_cycles(self):
        """自动断环"""
        if not self._cycles:
            QMessageBox.information(self, "提示", "没有检测到环结构")
            return
        
        # 使用最小生成树自动断环
        self._generate_assembly_trees()
        self._update_tree_display()
        self._update_info_display()
        
        QMessageBox.information(self, "完成", f"已自动断开 {len(self._cycles)} 个环")
    
    def _manual_break_cycles(self):
        """手动断环"""
        QMessageBox.information(self, "提示", "手动断环功能开发中...")
    
    def _auto_generate_tree(self):
        """自动生成装配树"""
        if not self._assembly_graph:
            QMessageBox.warning(self, "警告", "请先进行关系分析")
            return
        
        self._generate_assembly_trees()
        self._update_tree_display()
        self._update_info_display()
        
        self.status_label.setText("已自动生成装配树")
    
    def clear_data(self):
        """清除数据"""
        self._current_project = None
        self._components = []
        self._joints = []
        self._assembly_graph = None
        self._cycles = []
        self._assembly_trees = []
        self._selected_root_node = None
        
        # 清空UI
        self.tree_widget.clear()
        self.cycle_table.setRowCount(0)
        self.broken_table.setRowCount(0)
        self.joint_type_table.setRowCount(0)
        self.cycle_details_text.clear()
        self.degree_stats_text.clear()
        
        # 重置状态
        self.status_label.setText("准备就绪")
        self.analyze_btn.setEnabled(False)