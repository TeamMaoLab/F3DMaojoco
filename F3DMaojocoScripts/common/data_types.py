"""
共享数据类型定义

为F3DMaojocoScripts和MaojocoConverter提供统一的数据结构定义。

## 数据类型树形结构

```
ExportResult (导出结果)
├── config: ExportConfig (导出配置)
│   ├── output_dir: str (输出目录)
│   ├── mesh_quality: MeshQuality (网格质量)
│   └── create_timestamp_dir: bool (创建时间戳目录)
├── data: ExportData (导出数据)
│   ├── meta: ExportMetadata (导出元数据)
│   │   ├── export_time: str (导出时间)
│   │   ├── geometry_unit: str (几何单位)
│   │   ├── position_unit: str (位置单位)
│   │   ├── matrix_storage: str (矩阵存储格式)
│   │   ├── count_components: int (零部件总数)
│   │   ├── count_joints: int (关节数量)
│   │   └── format_version: str (格式版本)
│   ├── components: List[ComponentInfo] (零部件列表)
│   │   ├── name: str (零部件名称)
│   │   ├── occurrence_name: str (装配体实例名称)
│   │   ├── full_path_name: str (装配树完整路径)
│   │   ├── component_id: int (唯一ID)
│   │   ├── stl_file: Optional[str] (STL文件路径)
│   │   ├── world_transform: Optional[Transform4D] (世界坐标系变换)
│   │   │   └── matrix: List[List[float]] (4x4变换矩阵)
│   │   ├── assembly_transform: Optional[Transform4D] (装配体相对变换)
│   │   │   └── matrix: List[List[float]] (4x4变换矩阵)
│   │   ├── bodies_count: int (实体数量)
│   │   └── has_children: bool (是否包含子零部件)
│   └── joints: List[JointInfo] (关节列表)
│       ├── name: str (关节名称)
│       ├── joint_type: JointType (关节类型)
│       │   └── value: str (rigid/revolute/slider/cylindrical/pin_slot/planar/ball/inferred)
│       ├── connection: JointConnection (连接信息)
│       │   ├── occurrence_one_name: Optional[str] (第一个零部件显示名称)
│       │   ├── occurrence_one_full_path: Optional[str] (第一个零部件完整路径)
│       │   ├── occurrence_one_component: Optional[str] (第一个零部件component名称)
│       │   ├── occurrence_two_name: Optional[str] (第二个零部件显示名称)
│       │   ├── occurrence_two_full_path: Optional[str] (第二个零部件完整路径)
│       │   └── occurrence_two_component: Optional[str] (第二个零部件component名称)
│       ├── geometry: JointGeometry (几何信息)
│       │   ├── geometry_one_transform: Optional[Transform4D] (第一个几何体变换)
│       │   │   └── matrix: List[List[float]] (4x4变换矩阵)
│       │   └── geometry_two_transform: Optional[Transform4D] (第二个几何体变换)
│       │       └── matrix: List[List[float]] (4x4变换矩阵)
│       ├── is_suppressed: bool (是否被抑制)
│       └── is_light_bulb_on: bool (是否激活显示)
├── output_directory: str (实际输出目录)
├── stl_files: List[str] (STL文件列表)
├── execution_time: float (执行时间)
├── success: bool (是否成功)
└── error_message: Optional[str] (错误信息)

## 几何类型 (来自 geometry_math 模块)
```
Vector3D (3D向量)
├── x: float
├── y: float
└── z: float

Quaternion (四元数)
├── w: float
├── x: float
├── y: float
└── z: float

Transform4D (4x4变换矩阵)
└── matrix: List[List[float]] (4x4数组)

## 几何类型 (来自 geometry_math 模块)
```
Vector3D (3D向量)
├── x: float
├── y: float
└── z: float

Quaternion (四元数)
├── w: float
├── x: float
├── y: float
└── z: float

Transform4D (4x4变换矩阵)
└── matrix: List[List[float]] (4x4数组)

详细说明请参考: docs/theory/fusion_360_matrix_formats.md
```

BoundingBox (轴对齐包围盒)
├── min_point: Vector3D
└── max_point: Vector3D
```
"""

from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any
from enum import Enum
import json
import time
import logging

# 导入几何数学模块中的类型
from .geometry_math import Vector3D, Quaternion, Transform4D


class JointType(Enum):
    """关节类型枚举
    
    这些关节类型对应Fusion 360中的不同运动副类型：
    
    - RIGID: 刚性连接，完全固定，无相对运动
    - REVOLUTE: 旋转副，绕单一轴线旋转运动
    - SLIDER: 滑动副，沿单一轴线平移运动  
    - CYLINDRICAL: 圆柱副，可同时绕轴线旋转和沿轴线平移
    - PIN_SLOT: 销槽副，销钉在槽内滑动，约束较复杂
    - PLANAR: 平面副，在平面内进行平移和旋转运动
    - BALL: 球面副，绕固定点进行三轴旋转运动
    - INFERRED: 推断副，由系统自动推断的关节类型
    """
    RIGID = "rigid"                    # 刚性连接
    REVOLUTE = "revolute"              # 旋转副
    SLIDER = "slider"                  # 滑动副
    CYLINDRICAL = "cylindrical"        # 圆柱副
    PIN_SLOT = "pin_slot"              # 销槽副
    PLANAR = "planar"                  # 平面副
    BALL = "ball"                      # 球面副
    INFERRED = "inferred"              # 推断副


class MeshQuality(Enum):
    """网格质量枚举
    
    控制STL文件导出的精细程度，影响文件大小和模型精度：
    
    - LOW: 低质量，三角面片较少，文件小，导出快，适合快速预览
    - MEDIUM: 中等质量，平衡精度和文件大小，推荐用于一般仿真
    - HIGH: 高质量，三角面片密集，文件大，导出慢，适合高精度仿真
    """
    LOW = "low"                      # 低质量
    MEDIUM = "medium"                  # 中等质量
    HIGH = "high"                     # 高质量


# 序列化工具函数
def vector3d_to_dict(vector: Vector3D) -> Dict[str, float]:
    """将Vector3D转换为字典"""
    return {"x": vector.x, "y": vector.y, "z": vector.z}


def vector3d_from_dict(data: Dict[str, float]) -> Vector3D:
    """从字典创建Vector3D"""
    return Vector3D(data["x"], data["y"], data["z"])


def quaternion_to_dict(quaternion: Quaternion) -> Dict[str, float]:
    """将Quaternion转换为字典"""
    return {"w": quaternion.w, "x": quaternion.x, "y": quaternion.y, "z": quaternion.z}


def quaternion_from_dict(data: Dict[str, float]) -> Quaternion:
    """从字典创建Quaternion"""
    return Quaternion(data["w"], data["x"], data["y"], data["z"])


def transform4d_to_dict(transform: Transform4D) -> Dict[str, Any]:
    """将Transform4D转换为字典"""
    return {"matrix": transform.matrix}


def transform4d_from_dict(data: Dict[str, Any]) -> Transform4D:
    """从字典创建Transform4D"""
    return Transform4D(data["matrix"])


@dataclass
class JointConnection:
    """关节连接信息
    
    记录关节连接的两个零部件（occurrence）的信息：
    
    - occurrence_one_name/two_name: 零部件在装配体中的显示名称
    - occurrence_one/two_full_path: 零部件在装配树中的完整路径
    - occurrence_one/two_component: 指向实际零部件（component）的名称
    
    注意：occurrence是零部件在装配体中的实例，一个component可以有多个occurrence
    """
    occurrence_one_name: Optional[str] = None        # 第一个零部件的显示名称
    occurrence_one_full_path: Optional[str] = None    # 第一个零部件的完整路径
    occurrence_one_component: Optional[str] = None    # 第一个零部件对应的component名称
    occurrence_two_name: Optional[str] = None        # 第二个零部件的显示名称
    occurrence_two_full_path: Optional[str] = None    # 第二个零部件的完整路径
    occurrence_two_component: Optional[str] = None    # 第二个零部件对应的component名称
    
    def to_dict(self) -> Dict[str, Any]:
        """转换为字典"""
        return {
            "occurrence_one_name": self.occurrence_one_name,
            "occurrence_one_full_path": self.occurrence_one_full_path,
            "occurrence_one_component": self.occurrence_one_component,
            "occurrence_two_name": self.occurrence_two_name,
            "occurrence_two_full_path": self.occurrence_two_full_path,
            "occurrence_two_component": self.occurrence_two_component
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'JointConnection':
        """从字典创建"""
        return cls(
            occurrence_one_name=data.get("occurrence_one_name"),
            occurrence_one_full_path=data.get("occurrence_one_full_path"),
            occurrence_one_component=data.get("occurrence_one_component"),
            occurrence_two_name=data.get("occurrence_two_name"),
            occurrence_two_full_path=data.get("occurrence_two_full_path"),
            occurrence_two_component=data.get("occurrence_two_component")
        )


@dataclass
class BasicJointLimits:
    """基础关节限制信息
    
    记录关节的运动范围限制，用于MuJoCo仿真中的关节约束：
    
    - minimum_value: 最小值（单位：旋转为弧度，滑动为毫米）
    - maximum_value: 最大值（单位：旋转为弧度，滑动为毫米）
    - rest_value: 静止状态值（单位：旋转为弧度，滑动为毫米）
    - has_limits: 是否有运动限制
    
    注意：Fusion 360 API中滑动限制的单位是厘米，这里统一转换为毫米
    """
    minimum_value: Optional[float] = None    # 最小值
    maximum_value: Optional[float] = None    # 最大值
    rest_value: Optional[float] = None       # 静止值
    has_limits: bool = False                 # 是否有运动限制
    
    def to_dict(self) -> Dict[str, Any]:
        """转换为字典"""
        return {
            "minimum_value": self.minimum_value,
            "maximum_value": self.maximum_value,
            "rest_value": self.rest_value,
            "has_limits": self.has_limits
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'BasicJointLimits':
        """从字典创建"""
        return cls(
            minimum_value=data.get("minimum_value"),
            maximum_value=data.get("maximum_value"),
            rest_value=data.get("rest_value"),
            has_limits=data.get("has_limits", False)
        )


@dataclass
class RevoluteJointLimits:
    """旋转关节限制信息
    
    专门用于旋转关节的限制信息：
    
    - rotation_limits: 旋转角度限制
    - current_angle: 当前旋转角度（弧度）
    - rotation_axis: 旋转轴方向向量
    """
    rotation_limits: BasicJointLimits    # 旋转限制
    current_angle: Optional[float] = None    # 当前旋转角度（弧度）
    rotation_axis: Optional[Vector3D] = None  # 旋转轴方向向量
    
    def to_dict(self) -> Dict[str, Any]:
        """转换为字典"""
        return {
            "rotation_limits": self.rotation_limits.to_dict(),
            "current_angle": self.current_angle,
            "rotation_axis": vector3d_to_dict(self.rotation_axis) if self.rotation_axis else None
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'RevoluteJointLimits':
        """从字典创建"""
        return cls(
            rotation_limits=JointLimits.from_dict(data["rotation_limits"]),
            current_angle=data.get("current_angle"),
            rotation_axis=vector3d_from_dict(data["rotation_axis"]) if data.get("rotation_axis") else None
        )


@dataclass
class BallJointLimits:
    """球关节限制信息
    
    专门用于球关节的限制信息：
    
    - pitch_limits: 俯仰角限制（可选）
    - yaw_limits: 偏航角限制（可选）
    - roll_limits: 滚转角限制（可选）
    - center_position: 球心位置
    - pitch_axis: 俯仰轴方向
    - yaw_axis: 偏航轴方向
    - roll_axis: 滚转轴方向
    
    注意：球关节通常没有角度限制，但可以记录各轴的方向信息
    """
    pitch_limits: Optional[BasicJointLimits] = None    # 俯仰角限制
    yaw_limits: Optional[BasicJointLimits] = None      # 偏航角限制
    roll_limits: Optional[BasicJointLimits] = None     # 滚转角限制
    center_position: Optional[Vector3D] = None   # 球心位置
    pitch_axis: Optional[Vector3D] = None        # 俯仰轴方向
    yaw_axis: Optional[Vector3D] = None          # 偏航轴方向
    roll_axis: Optional[Vector3D] = None         # 滚转轴方向
    
    def to_dict(self) -> Dict[str, Any]:
        """转换为字典"""
        return {
            "pitch_limits": self.pitch_limits.to_dict() if self.pitch_limits else None,
            "yaw_limits": self.yaw_limits.to_dict() if self.yaw_limits else None,
            "roll_limits": self.roll_limits.to_dict() if self.roll_limits else None,
            "center_position": vector3d_to_dict(self.center_position) if self.center_position else None,
            "pitch_axis": vector3d_to_dict(self.pitch_axis) if self.pitch_axis else None,
            "yaw_axis": vector3d_to_dict(self.yaw_axis) if self.yaw_axis else None,
            "roll_axis": vector3d_to_dict(self.roll_axis) if self.roll_axis else None
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'BallJointLimits':
        """从字典创建"""
        return cls(
            pitch_limits=BasicJointLimits.from_dict(data["pitch_limits"]) if data.get("pitch_limits") else None,
            yaw_limits=BasicJointLimits.from_dict(data["yaw_limits"]) if data.get("yaw_limits") else None,
            roll_limits=BasicJointLimits.from_dict(data["roll_limits"]) if data.get("roll_limits") else None,
            center_position=vector3d_from_dict(data["center_position"]) if data.get("center_position") else None,
            pitch_axis=vector3d_from_dict(data["pitch_axis"]) if data.get("pitch_axis") else None,
            yaw_axis=vector3d_from_dict(data["yaw_axis"]) if data.get("yaw_limits") else None,
            roll_axis=vector3d_from_dict(data["roll_axis"]) if data.get("roll_limits") else None
        )


@dataclass
class JointGeometry:
    """关节几何信息
    
    记录关节的几何变换信息，用于定义关节的位置和方向：
    
    - geometry_one_transform: 第一个几何体（通常是原点或轴线）的变换矩阵
    - geometry_two_transform: 第二个几何体（通常是运动参考）的变换矩阵
    
    这些变换矩阵定义了关节在局部坐标系中的位置和姿态，对于不同类型的关节：
    - 旋转副：定义旋转轴线的位置和方向
    - 滑动副：定义滑动轴线的位置和方向  
    - 球面副：定义球心的位置
    """
    geometry_one_transform: Optional[Transform4D] = None    # 第一个几何体的变换矩阵
    geometry_two_transform: Optional[Transform4D] = None    # 第二个几何体的变换矩阵
    
    def to_dict(self) -> Dict[str, Any]:
        """转换为字典"""
        return {
            "geometry_one_transform": transform4d_to_dict(self.geometry_one_transform) if self.geometry_one_transform else None,
            "geometry_two_transform": transform4d_to_dict(self.geometry_two_transform) if self.geometry_two_transform else None
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'JointGeometry':
        """从字典创建"""
        return cls(
            geometry_one_transform=transform4d_from_dict(data["geometry_one_transform"]) if data.get("geometry_one_transform") else None,
            geometry_two_transform=transform4d_from_dict(data["geometry_two_transform"]) if data.get("geometry_two_transform") else None
        )


@dataclass
class JointLimits:
    """关节限制信息（统一接口）
    
    根据关节类型存储不同的限制信息：
    - 旋转关节：存储旋转角度限制和轴向信息
    - 球关节：存储球心位置和三轴方向信息
    - 其他关节：预留扩展
    
    使用Union类型确保只有一种限制类型被设置
    """
    revolute_limits: Optional[RevoluteJointLimits] = None    # 旋转关节限制
    ball_limits: Optional[BallJointLimits] = None           # 球关节限制
    
    @property
    def has_limits(self) -> bool:
        """是否有任何限制信息"""
        return self.revolute_limits is not None or self.ball_limits is not None
    
    def to_dict(self) -> Dict[str, Any]:
        """转换为字典"""
        return {
            "revolute_limits": self.revolute_limits.to_dict() if self.revolute_limits else None,
            "ball_limits": self.ball_limits.to_dict() if self.ball_limits else None
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'JointLimits':
        """从字典创建"""
        return cls(
            revolute_limits=RevoluteJointLimits.from_dict(data["revolute_limits"]) if data.get("revolute_limits") else None,
            ball_limits=BallJointLimits.from_dict(data["ball_limits"]) if data.get("ball_limits") else None
        )


@dataclass
class JointInfo:
    """关节信息
    
    完整记录一个关节的所有信息，是导出的核心数据之一：
    
    - name: 关节在Fusion 360中的名称
    - joint_type: 关节类型（旋转副、滑动副等）
    - connection: 关节连接的两个零部件信息
    - geometry: 关节的几何变换信息
    - is_suppressed: 关节是否被抑制（禁用状态）
    - is_light_bulb_on: 关节是否在Fusion 360中激活显示
    - limits: 关节限制信息（根据类型存储不同数据）
    
    这些信息用于在MuJoCo中重建相应的关节约束关系
    """
    name: str                              # 关节名称
    joint_type: JointType                  # 关节类型
    connection: JointConnection            # 连接信息
    geometry: JointGeometry                # 几何信息
    is_suppressed: bool                    # 是否被抑制
    is_light_bulb_on: bool                 # 是否激活显示
    limits: Optional[JointLimits] = None   # 关节限制信息
    
    def to_dict(self) -> Dict[str, Any]:
        """转换为字典"""
        return {
            "name": self.name,
            "joint_type": self.joint_type.value,
            "connection": self.connection.to_dict(),
            "geometry": self.geometry.to_dict(),
            "is_suppressed": self.is_suppressed,
            "is_light_bulb_on": self.is_light_bulb_on,
            "limits": self.limits.to_dict() if self.limits else None
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'JointInfo':
        """从字典创建"""
        return cls(
            name=data["name"],
            joint_type=JointType(data["joint_type"]),
            connection=JointConnection.from_dict(data["connection"]),
            geometry=JointGeometry.from_dict(data["geometry"]),
            is_suppressed=data["is_suppressed"],
            is_light_bulb_on=data["is_light_bulb_on"],
            limits=JointLimits.from_dict(data["limits"]) if data.get("limits") else None
        )






@dataclass
class ComponentInfo:
    """零部件信息
    
    记录装配体中单个零部件的完整信息，是导出的核心数据之一：
    
    - name: 零部件的原始名称（component名称）
    - occurrence_name: 零部件在装配体中的实例名称
    - full_path_name: 零部件在装配树中的完整路径
    - component_id: Fusion 360分配的唯一ID
    - stl_file: 导出的STL文件路径（相对于输出目录）
    - world_transform: 零部件在世界坐标系中的变换矩阵
    - bodies_count: 零部件包含的实体数量
    - has_children: 是否包含子零部件（是否为子装配体）
    
    这些信息用于在MuJoCo中重建模型结构和位置关系
    
    详细说明请参考: docs/theory/fusion_360_matrix_formats.md
    """
    name: str                              # 零部件原始名称
    occurrence_name: str                   # 装配体实例名称
    full_path_name: str                    # 装配树完整路径
    component_id: int                      # 唯一ID
    stl_file: Optional[str] = None         # STL文件路径
    world_transform: Optional[Transform4D] = None    # 世界坐标系变换
    bodies_count: int = 0                  # 实体数量
    has_children: bool = False             # 是否包含子零部件
    
    def to_dict(self) -> Dict[str, Any]:
        """转换为字典"""
        return {
            "name": self.name,
            "occurrence_name": self.occurrence_name,
            "full_path_name": self.full_path_name,
            "component_id": self.component_id,
            "stl_file": self.stl_file,
            "world_transform": transform4d_to_dict(self.world_transform) if self.world_transform else None,
            "bodies_count": self.bodies_count,
            "has_children": self.has_children
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'ComponentInfo':
        """从字典创建"""
        return cls(
            name=data["name"],
            occurrence_name=data["occurrence_name"],
            full_path_name=data["full_path_name"],
            component_id=data["component_id"],
            stl_file=data.get("stl_file"),
            world_transform=transform4d_from_dict(data["world_transform"]) if data.get("world_transform") else None,
            bodies_count=data.get("bodies_count", 0),
            has_children=data.get("has_children", False)
        )


@dataclass
class ExportMetadata:
    """导出元数据
    
    记录导出过程的元信息，用于验证和理解导出数据：
    
    - export_time: 导出操作的时间戳
    - geometry_unit: 几何数据单位（毫米，转换后的统一标准）
    - position_unit: 位置数据单位（毫米，已从Fusion 360 API的厘米转换为毫米）
    - matrix_storage: 矩阵存储格式（4x4数组）
    - count_components: 导出的零部件总数
    - count_joints: 导出的关节总数
    - format_version: 数据格式版本
    
    这些信息帮助MaojocoConverter正确解析和处理数据
    """
    export_time: str                        # 导出时间
    geometry_unit: str                      # 几何单位
    position_unit: str                      # 位置单位
    matrix_storage: str                     # 矩阵存储格式
    count_components: int                  # 零部件总数
    count_joints: int                       # 关节总数
    format_version: str = "1.0"             # 格式版本
    
    def to_dict(self) -> Dict[str, Any]:
        """转换为字典"""
        return {
            "export_time": self.export_time,
            "geometry_unit": self.geometry_unit,
            "position_unit": self.position_unit,
            "matrix_storage": self.matrix_storage,
            "count_components": self.count_components,
            "count_joints": self.count_joints,
            "format_version": self.format_version
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'ExportMetadata':
        """从字典创建"""
        return cls(
            export_time=data["export_time"],
            geometry_unit=data["geometry_unit"],
            position_unit=data["position_unit"],
            matrix_storage=data["matrix_storage"],
            count_components=data["count_components"],
            count_joints=data["count_joints"],
            format_version=data.get("format_version", "1.0")
        )


@dataclass
class ExportData:
    """导出数据
    
    整个导出操作的顶级数据结构，包含完整的装配体信息：
    
    - meta: 导出元数据，记录导出过程的基本信息
    - components: 所有零部件信息的列表
    - joints: 所有关节信息的列表
    
    这是F3DMaojocoScripts和MaojocoConverter之间交换的主要数据格式，
    包含了重建MuJoCo模型所需的全部信息。
    """
    meta: ExportMetadata                                     # 导出元数据
    components: List[ComponentInfo] = field(default_factory=list)    # 零部件列表
    joints: List[JointInfo] = field(default_factory=list)            # 关节列表
    
    def to_dict(self) -> Dict[str, Any]:
        """转换为字典"""
        return {
            "meta": self.meta.to_dict(),
            "components": [comp.to_dict() for comp in self.components],
            "joints": [joint.to_dict() for joint in self.joints]
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'ExportData':
        """从字典创建"""
        return cls(
            meta=ExportMetadata.from_dict(data["meta"]),
            components=[ComponentInfo.from_dict(comp) for comp in data.get("components", [])],
            joints=[JointInfo.from_dict(joint) for joint in data.get("joints", [])]
        )
    
    def to_json(self, indent: int = 2) -> str:
        """转换为JSON字符串"""
        return json.dumps(self.to_dict(), indent=indent, ensure_ascii=False)
    
    @classmethod
    def from_json(cls, json_str: str) -> 'ExportData':
        """从JSON字符串创建"""
        data = json.loads(json_str)
        return cls.from_dict(data)
    
    def save_to_file(self, filepath: str):
        """保存到文件"""
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write(self.to_json())
    
    @classmethod
    def load_from_file(cls, filepath: str) -> 'ExportData':
        """从文件加载"""
        with open(filepath, 'r', encoding='utf-8') as f:
            json_str = f.read()
        return cls.from_json(json_str)


@dataclass
class ExportConfig:
    """导出配置
    
    控制导出过程的各项参数：
    
    - output_dir: 输出目录的根路径
    - mesh_quality: STL网格质量（低/中/高）
    - create_timestamp_dir: 是否创建时间戳子目录
    
    所有导出都是完整的（包含零部件和关节），保持简单直接
    """
    output_dir: str                          # 输出目录
    mesh_quality: MeshQuality = MeshQuality.MEDIUM    # 网格质量
    create_timestamp_dir: bool = True        # 创建时间戳目录
    
    def to_dict(self) -> Dict[str, Any]:
        """转换为字典"""
        return {
            "output_dir": self.output_dir,
            "mesh_quality": self.mesh_quality.value,
            "create_timestamp_dir": self.create_timestamp_dir
        }


@dataclass
class ExportResult:
    """导出结果
    
    记录导出操作的完整结果，包含执行状态和输出信息：
    
    - config: 使用的导出配置
    - data: 导出的数据内容
    - output_directory: 实际的输出目录路径
    - stl_files: 生成的STL文件列表
    - execution_time: 总执行时间（秒）
    - success: 导出是否成功
    - error_message: 错误信息（如果失败）
    
    这个结构提供了导出操作的完整反馈，可用于结果分析和错误诊断
    """
    config: ExportConfig                                     # 导出配置
    data: ExportData                                       # 导出数据
    output_directory: str                                   # 输出目录
    stl_files: List[str] = field(default_factory=list)     # STL文件列表
    execution_time: float = 0.0                            # 执行时间
    success: bool = True                                    # 是否成功
    error_message: Optional[str] = None                    # 错误信息
    
    def to_dict(self) -> Dict[str, Any]:
        """转换为字典"""
        return {
            "config": self.config.to_dict(),
            "data": self.data.to_dict(),
            "output_directory": self.output_directory,
            "stl_files": self.stl_files,
            "execution_time": self.execution_time,
            "success": self.success,
            "error_message": self.error_message
        }


def create_default_metadata(component_count: int = 0, joint_count: int = 0, logger: Optional[logging.Logger] = None) -> ExportMetadata:
    """创建默认元数据
    
    根据数据统计创建标准的导出元数据：
    
    - component_count: 零部件数量统计
    - joint_count: 关节数量统计
    - logger: 可选的日志记录器
    
    Returns:
        ExportMetadata: 创建好的元数据对象
    """
    if logger:
        logger.info(f"创建元数据: 零部件数={component_count}, 关节数={joint_count}")
    
    return ExportMetadata(
        export_time=time.strftime("%Y-%m-%d %H:%M:%S"),
        geometry_unit="millimeters",
        position_unit="millimeters",
        matrix_storage="4x4_array",
        count_components=component_count,
        count_joints=joint_count
    )


def save_export_data(data: ExportData, filepath: str, logger: Optional[logging.Logger] = None) -> bool:
    """保存导出数据到文件
    
    将ExportData对象序列化为JSON格式并保存到指定文件：
    
    - data: 要保存的导出数据对象
    - filepath: 目标文件路径
    - logger: 可选的日志记录器
    
    Returns:
        bool: 保存是否成功
    """
    try:
        if logger:
            logger.info(f"保存导出数据到: {filepath}")
        
        data.save_to_file(filepath)
        
        if logger:
            logger.info(f"导出数据保存成功")
        
        return True
        
    except Exception as e:
        if logger:
            logger.error(f"保存导出数据失败: {str(e)}")
        return False


def load_export_data(filepath: str, logger: Optional[logging.Logger] = None) -> Optional[ExportData]:
    """从文件加载导出数据
    
    从JSON文件中反序列化导出数据：
    
    - filepath: 源文件路径
    - logger: 可选的日志记录器
    
    Returns:
        Optional[ExportData]: 加载的导出数据对象，失败时返回None
    """
    try:
        if logger:
            logger.info(f"加载导出数据从: {filepath}")
        
        data = ExportData.load_from_file(filepath)
        
        if logger:
            logger.info(f"导出数据加载成功: {len(data.components)} 个零部件, {len(data.joints)} 个关节")
        
        return data
        
    except Exception as e:
        if logger:
            logger.error(f"加载导出数据失败: {str(e)}")
        return None