"""
MaojocoContext 上下文管理类

贯穿整个转换流程，跟踪状态变化和关键数据
"""

from pathlib import Path
from typing import Optional, Dict, Any
import networkx as nx

# 直接导入F3DMaojocoScripts的common模块
from F3DMaojocoScripts.common.data_types import ExportData

from .type_definitions import ConversionConfig
from .type_definitions import (
    ConversionPhase, Body4DCoordinates, JointGlobalCoordinates, JointPairwiseRelationship,
    KinematicTree, ConvertedData,
    Body4DCoordinatesDict, JointGlobalCoordinatesDict, JointPairwiseRelationshipsDict,
    AssemblyTreesList
)


class MaojocoContext:
    """转换上下文 - 贯穿全部流程，跟踪关键数据变化
    
    MaojocoContext 在整个转换流程中维护状态和数据，各个阶段会按顺序填充和读取这些字段：
    
    阶段流程：
    1. INITIALIZED → DATA_LOADING → RELATIONSHIP_ANALYSIS → UNIT_CONVERSION → MODEL_GENERATION → COMPLETED
    
    字段生命周期：
    - raw_export_data: [DATA_LOADING] 从 Fusion 360 导出数据中加载，后续阶段只读
    - body_4d_coordinates: [DATA_LOADING] 提取的 Body 4D 坐标，包含原始变换矩阵
    - joint_global_coordinates: [DATA_LOADING] 提取的关节全局坐标和类型
    - joint_pairwise_relationships: [RELATIONSHIP_ANALYSIS] 分析得出的关节两两关系
    - assembly_graph: [RELATIONSHIP_ANALYSIS] 基于关节关系构建的装配关系图
    - assembly_trees: [RELATIONSHIP_ANALYSIS] 从装配图断开环得到的树结构
    - kinematic_tree: [RELATIONSHIP_ANALYSIS] 最终生成的 MuJoCo 运动学树结构
    - converted_data: [UNIT_CONVERSION] 单位转换后的数据，包含米制的坐标和变换
    - xml_content: [MODEL_GENERATION] 生成的 MuJoCo XML 模型内容
    
    数据流向：
    Fusion 360 导出数据 → 坐标提取 → 关系分析 → 运动学树构建 → 单位转换 → XML 生成
    """
    
    # 所有可能的阶段状态
    PHASES = [
        phase.value for phase in ConversionPhase
    ]
    
    def __init__(self, export_dir: str, config: ConversionConfig):
        self.export_dir = Path(export_dir)
        self.config = config
        
        # 阶段状态
        self.phase = ConversionPhase.INITIALIZED.value
        
        # ===== 核心数据 =====
        
        # [DATA_LOADING] 原始导出数据 - 从 Fusion 360 导出文件加载
        # 包含 components, joints, filename_mapping 等原始信息
        self.raw_export_data: Optional[ExportData] = None
        
        # [UNIT_CONVERSION] 转换结果 - 单位转换后的数据（毫米→米）
        # 包含 body_coordinates, joint_coordinates 等米制数据
        self.converted_data: Optional[ConvertedData] = None
        
        # [MODEL_GENERATION] 最终输出 - 生成的 MuJoCo XML 模型内容
        self.xml_content: Optional[str] = None
        
        # [ACTUATOR_GENERATION] 执行器输出 - 带执行器的 MuJoCo XML 模型内容
        self.actuator_xml_content: Optional[str] = None
        
        # ===== 中间处理数据 =====
        
        # [DATA_LOADING] Body 4D 坐标表达 - 从 Fusion 360 组件提取
        # 每个包含 name, occurrence_name, transform(4x4矩阵), stl_file 等
        # 注意：此时坐标仍是毫米单位，transform 是 Fusion 360 坐标系
        self.body_4d_coordinates: Body4DCoordinatesDict = {}
        
        # [DATA_LOADING] Joint 全局坐标 - 从 Fusion 360 关节提取
        # 每个包含 position, quaternion, joint_type, axis 等信息
        # 注意：此时坐标仍是毫米单位
        self.joint_global_coordinates: JointGlobalCoordinatesDict = {}
        
        # [RELATIONSHIP_ANALYSIS] Joint 两两关系 - 分析得出的关节连接关系
        # 基于 joint_global_coordinates 计算每对关节间的相对位置和连接关系
        # 用于构建装配关系图和识别装配层次
        self.joint_pairwise_relationships: JointPairwiseRelationshipsDict = {}
        
        # [RELATIONSHIP_ANALYSIS] 装配关系图 - 基于关节两两关系构建
        # 使用 NetworkX 图结构表示组件间的连接关系
        # 包含环结构，需要断开环才能生成树结构
        self.assembly_graph: Optional[nx.Graph] = None
        
        # [RELATIONSHIP_ANALYSIS] 装配树 - 从装配图断开环生成的树结构
        # 每棵树包含 root, nodes, edges，表示一个独立的装配子树
        # 用于后续生成运动学树结构
        self.assembly_trees: AssemblyTreesList = []
        
        # ===== 运动学树 - 关键输出 =====
        
        # [RELATIONSHIP_ANALYSIS] MuJoCo 运动学树结构 - 最终的层次化结构
        # 包含 roots, bodies, joints, nodes, relative_transforms
        # 这是生成 MuJoCo 模型的核心数据结构
        self.kinematic_tree: Optional[KinematicTree] = None
        
        
        
    def __str__(self) -> str:
        return f"MaojocoContext(phase={self.phase}, export_dir={self.export_dir}, bodies={len(self.body_4d_coordinates)}, joints={len(self.joint_global_coordinates)})"
    
    def __repr__(self) -> str:
        return self.__str__()