"""
MaojocoConverter 类型系统

为整个 MaojocoConverter 模块提供统一的类型定义，
充分利用 F3DMaojocoScripts/common 中的现成类型。
"""

from typing import Dict, List, Optional, Any, Union, Tuple
from dataclasses import dataclass
from enum import Enum
import networkx as nx

# 导入共享类型
from F3DMaojocoScripts.common.data_types import (
    ExportData, ExportMetadata, ComponentInfo, JointInfo, Transform4D,
    JointType, Vector3D, Quaternion
)
from F3DMaojocoScripts.common.geometry_math import Vector3D, Quaternion, Transform4D


class ConversionPhase(Enum):
    """转换阶段枚举"""
    INITIALIZED = "initialized"
    DATA_LOADING = "data_loading"
    RELATIONSHIP_ANALYSIS = "relationship_analysis"
    UNIT_CONVERSION = "unit_conversion"
    MODEL_GENERATION = "model_generation"
    COMPLETED = "completed"
    ERROR = "error"


class PhaseStatus(Enum):
    """阶段状态枚举"""
    PENDING = "pending"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    SKIPPED = "skipped"


@dataclass
class Body4DCoordinates:
    """Body 4D 坐标表达"""
    name: str
    occurrence_name: str
    full_path_name: str
    component_id: str
    transform: Transform4D
    stl_file: Optional[str] = None
    bodies_count: int = 1
    has_children: bool = False
    
    @property
    def position(self) -> Vector3D:
        """获取位置向量"""
        return self.transform.get_translation()
    
    @property
    def rotation(self) -> List[List[float]]:
        """获取旋转矩阵"""
        return self.transform.get_rotation_matrix()


@dataclass
class JointGlobalCoordinates:
    """关节全局坐标"""
    position: Vector3D
    quaternion: Quaternion
    joint_name: str
    joint_type: JointType


@dataclass
class JointPairwiseRelationship:
    """关节两两关系"""
    joint1_name: str
    joint2_name: str
    distance: float
    joint1_type: JointType
    joint2_type: JointType
    joint1_components: List[str]
    joint2_components: List[str]
    shared_components: List[str]
    has_shared_components: bool
    connection_strength: float


@dataclass
class KinematicBody:
    """运动学刚体"""
    body_id: str
    name: str
    component_id: str
    occurrence_name: str
    world_transform: Transform4D
    stl_file: Optional[str]
    bodies_count: int
    mass: float
    inertia: List[float]
    parent: Optional[str] = None
    original_units: str = "mm"
    converted_units: str = "mm"
    
    @property
    def position(self) -> Vector3D:
        """获取位置向量"""
        return self.world_transform.get_translation()
    
    @property
    def rotation(self) -> List[List[float]]:
        """获取旋转矩阵"""
        return self.world_transform.get_rotation_matrix()
    
    @property
    def quaternion(self) -> Quaternion:
        """获取旋转四元数"""
        return self.world_transform.to_quaternion()


@dataclass
class KinematicJoint:
    """运动学关节"""
    joint_id: str
    name: str
    joint_type: JointType
    parent_body: str
    child_body: str
    position: Vector3D
    axis: Optional[Vector3D] = None
    limits: Optional[Dict[str, Any]] = None
    is_suppressed: bool = False
    is_active: bool = True


@dataclass
class KinematicNode:
    """运动学节点"""
    body_id: str
    parent_body: Optional[str]
    children: List[str]
    joint: Optional[str]
    level: int


@dataclass
class RelativeTransform:
    """相对变换"""
    parent: Optional[str]
    transform: Transform4D
    original_units: str = "mm"
    converted_units: str = "mm"
    
    @property
    def position(self) -> Vector3D:
        """获取位置向量"""
        return self.transform.get_translation()
    
    @property
    def rotation(self) -> List[List[float]]:
        """获取旋转矩阵"""
        return self.transform.get_rotation_matrix()


@dataclass
class KinematicTree:
    """MuJoCo 运动学树"""
    roots: List[str]
    nodes: Dict[str, KinematicNode]
    joints: Dict[str, KinematicJoint]
    bodies: Dict[str, KinematicBody]
    relative_transforms: Dict[str, RelativeTransform]


@dataclass
class ConvertedData:
    """转换后的数据"""
    body_coordinates: Dict[str, Body4DCoordinates]
    joint_coordinates: Dict[str, JointGlobalCoordinates]
    kinematic_tree: Optional[KinematicTree] = None
    filename_mapping: Optional[Dict[str, str]] = None
    conversion_info: Optional[Dict[str, Any]] = None


@dataclass
class PhaseResult:
    """阶段执行结果"""
    phase_name: str
    status: PhaseStatus
    duration: float
    start_time: Optional[str] = None
    end_time: Optional[str] = None
    error_message: Optional[str] = None
    data_processed: Optional[Dict[str, Any]] = None


@dataclass
class DebugLogEntry:
    """调试日志条目"""
    timestamp: str
    event: str
    phase: str
    message: Optional[str] = None
    action: Optional[str] = None
    context: Optional[str] = None
    error_type: Optional[str] = None
    error_message: Optional[str] = None
    error_details: Optional[Dict[str, Any]] = None
    duration: Optional[float] = None
    old_phase: Optional[str] = None


@dataclass
class DebugSummary:
    """调试摘要"""
    export_dir: str
    current_phase: str
    total_time: float
    phase_durations: Dict[str, float]
    key_data_summary: Dict[str, Any]


@dataclass
class KeyDataSummary:
    """关键数据摘要"""
    body_4d_coordinates_count: int
    body_4d_coordinates_list: List[str]
    joint_global_coordinates_count: int
    joint_global_coordinates_list: List[str]
    joint_pairwise_relationships_count: int
    joint_pairwise_relationships_list: List[Tuple[str, str]]
    assembly_graph_nodes: int
    assembly_graph_edges: int
    assembly_trees_count: int
    assembly_trees_roots: List[str]
    kinematic_tree_exists: bool
    kinematic_tree_roots_count: int
    kinematic_tree_nodes_count: int
    kinematic_tree_joints_count: int


# 类型别名
BodyName = str
JointName = str
ComponentName = str
AssemblyGraph = nx.Graph
AssemblyTree = Dict[str, Any]
JointPairKey = Tuple[JointName, JointName]

# 数据结构类型
Body4DCoordinatesDict = Dict[BodyName, Body4DCoordinates]
JointGlobalCoordinatesDict = Dict[JointName, JointGlobalCoordinates]
JointPairwiseRelationshipsDict = Dict[JointPairKey, JointPairwiseRelationship]
AssemblyTreesList = List[AssemblyTree]
PhaseResultsDict = Dict[str, PhaseResult]
DebugLogList = List[DebugLogEntry]




# 配置类型
@dataclass
class ConversionConfig:
    """转换配置"""
    output_dir: str
    mesh_quality: str = "medium"
    create_timestamp_dir: bool = True
    debug_mode: bool = False
    validate_output: bool = True
    cleanup_temp_files: bool = False
    
    # 根节点选择策略
    root_node_strategy: str = "center"  # "center", "max_degree", "manual"
    manual_root_node: Optional[str] = None  # 当策略为 "manual" 时使用
    
    def to_dict(self) -> Dict[str, Any]:
        """转换为字典"""
        return {
            'output_dir': self.output_dir,
            'mesh_quality': self.mesh_quality,
            'create_timestamp_dir': self.create_timestamp_dir,
            'debug_mode': self.debug_mode,
            'validate_output': self.validate_output,
            'cleanup_temp_files': self.cleanup_temp_files,
            'root_node_strategy': self.root_node_strategy,
            'manual_root_node': self.manual_root_node
        }