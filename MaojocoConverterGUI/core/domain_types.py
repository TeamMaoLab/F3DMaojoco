"""
独立类型系统

从F3DMaojocoScripts复制的独立类型系统，保持完全独立性，无外部依赖。
为MaojocoConverter GUI提供统一的数据结构定义。

## 数据类型树形结构

```
ExportData (导出数据)
├── meta: ExportMetadata (导出元数据)
│   ├── export_time: str (导出时间)
│   ├── geometry_unit: str (几何单位)
│   ├── position_unit: str (位置单位)
│   ├── matrix_storage: str (矩阵存储格式)
│   ├── count_components: int (零部件总数)
│   ├── count_joints: int (关节数量)
│   └── format_version: str (格式版本)
├── components: List[ComponentInfo] (零部件列表)
│   ├── name: str (零部件名称)
│   ├── occurrence_name: str (装配体实例名称)
│   ├── full_path_name: str (装配树完整路径)
│   ├── component_id: int (唯一ID)
│   ├── stl_file: Optional[str] (STL文件路径)
│   ├── world_transform: Optional[Transform4D] (世界坐标系变换)
│   │   └── matrix: List[List[float]] (4x4变换矩阵)
│   ├── bodies_count: int (实体数量)
│   └── has_children: bool (是否包含子零部件)
└── joints: List[JointInfo] (关节列表)
    ├── name: str (关节名称)
    ├── joint_type: JointType (关节类型)
    │   └── value: str (rigid/revolute/slider/cylindrical/pin_slot/planar/ball/inferred)
    ├── connection: JointConnection (连接信息)
    │   ├── occurrence_one_name: Optional[str] (第一个零部件显示名称)
    │   ├── occurrence_one_full_path: Optional[str] (第一个零部件完整路径)
    │   ├── occurrence_one_component: Optional[str] (第一个零部件component名称)
    │   ├── occurrence_two_name: Optional[str] (第二个零部件显示名称)
    │   ├── occurrence_two_full_path: Optional[str] (第二个零部件完整路径)
    │   └── occurrence_two_component: Optional[str] (第二个零部件component名称)
    ├── geometry: JointGeometry (几何信息)
    │   ├── geometry_one_transform: Optional[Transform4D] (第一个几何体变换)
    │   │   └── matrix: List[List[float]] (4x4变换矩阵)
    │   └── geometry_two_transform: Optional[Transform4D] (第二个几何体变换)
    │       └── matrix: List[List[float]] (4x4变换矩阵)
    ├── is_suppressed: bool (是否被抑制)
    └── is_light_bulb_on: bool (是否激活显示)

## 几何类型
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

BoundingBox (轴对齐包围盒)
├── min_point: Vector3D
└── max_point: Vector3D
```
"""

from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any
from enum import Enum
from pathlib import Path
import json
import time
import math


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


@dataclass
class Vector3D:
    """3D向量类，支持坐标系转换"""
    x: float
    y: float
    z: float
    
    def __add__(self, other: 'Vector3D') -> 'Vector3D':
        """向量加法"""
        return Vector3D(self.x + other.x, self.y + other.y, self.z + other.z)
    
    def __sub__(self, other: 'Vector3D') -> 'Vector3D':
        """向量减法"""
        return Vector3D(self.x - other.x, self.y - other.y, self.z - other.z)
    
    def __mul__(self, scalar: float) -> 'Vector3D':
        """标量乘法"""
        return Vector3D(self.x * scalar, self.y * scalar, self.z * scalar)
    
    def __rmul__(self, scalar: float) -> 'Vector3D':
        """标量乘法（右乘）"""
        return self.__mul__(scalar)
    
    def __truediv__(self, scalar: float) -> 'Vector3D':
        """标量除法"""
        return Vector3D(self.x / scalar, self.y / scalar, self.z / scalar)
    
    def dot(self, other: 'Vector3D') -> float:
        """点积"""
        return self.x * other.x + self.y * other.y + self.z * other.z
    
    def cross(self, other: 'Vector3D') -> 'Vector3D':
        """叉积"""
        return Vector3D(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x
        )
    
    def magnitude(self) -> float:
        """向量长度"""
        return math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)
    
    def normalize(self) -> 'Vector3D':
        """归一化向量"""
        mag = self.magnitude()
        if mag == 0:
            return Vector3D(0, 0, 0)
        return self / mag
    
    def distance_to(self, other: 'Vector3D') -> float:
        """到另一个点的距离"""
        return (self - other).magnitude()
    
    def to_list(self) -> List[float]:
        """转换为列表"""
        return [self.x, self.y, self.z]
    
    def to_tuple(self) -> tuple:
        """转换为元组"""
        return (self.x, self.y, self.z)
    
    @classmethod
    def from_list(cls, values: List[float]) -> 'Vector3D':
        """从列表创建向量"""
        if len(values) != 3:
            raise ValueError("Vector3D requires exactly 3 values")
        return cls(values[0], values[1], values[2])
    
    @classmethod
    def zero(cls) -> 'Vector3D':
        """零向量"""
        return cls(0, 0, 0)
    
    @classmethod
    def one(cls) -> 'Vector3D':
        """单位向量"""
        return cls(1, 1, 1)


@dataclass
class Quaternion:
    """四元数类，用于表示旋转"""
    w: float
    x: float
    y: float
    z: float
    
    def __mul__(self, other: 'Quaternion') -> 'Quaternion':
        """四元数乘法"""
        return Quaternion(
            self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
            self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w
        )
    
    def conjugate(self) -> 'Quaternion':
        """共轭四元数"""
        return Quaternion(self.w, -self.x, -self.y, -self.z)
    
    def norm_squared(self) -> float:
        """范数平方"""
        return self.w ** 2 + self.x ** 2 + self.y ** 2 + self.z ** 2
    
    def norm(self) -> float:
        """范数"""
        return math.sqrt(self.norm_squared())
    
    def normalize(self) -> 'Quaternion':
        """归一化四元数"""
        n = self.norm()
        if n == 0:
            return Quaternion(0, 0, 0, 0)
        return Quaternion(self.w / n, self.x / n, self.y / n, self.z / n)
    
    def to_rotation_matrix(self) -> List[List[float]]:
        """转换为旋转矩阵"""
        # 标准化四元数
        q = self.normalize()
        w, x, y, z = q.w, q.x, q.y, q.z
        
        # 计算旋转矩阵
        return [
            [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
        ]
    
    @classmethod
    def identity(cls) -> 'Quaternion':
        """单位四元数"""
        return cls(1, 0, 0, 0)


@dataclass
class Transform4D:
    """4x4变换矩阵类"""
    matrix: List[List[float]]
    
    def __post_init__(self):
        """验证矩阵维度"""
        if len(self.matrix) != 4 or any(len(row) != 4 for row in self.matrix):
            raise ValueError("Transform4D requires a 4x4 matrix")
    
    def transform_point(self, point: Vector3D) -> Vector3D:
        """变换点"""
        x = self.matrix[0][0] * point.x + self.matrix[0][1] * point.y + self.matrix[0][2] * point.z + self.matrix[0][3]
        y = self.matrix[1][0] * point.x + self.matrix[1][1] * point.y + self.matrix[1][2] * point.z + self.matrix[1][3]
        z = self.matrix[2][0] * point.x + self.matrix[2][1] * point.y + self.matrix[2][2] * point.z + self.matrix[2][3]
        return Vector3D(x, y, z)
    
    def get_translation(self) -> Vector3D:
        """获取平移部分"""
        return Vector3D(self.matrix[0][3], self.matrix[1][3], self.matrix[2][3])
    
    def get_rotation_matrix(self) -> List[List[float]]:
        """获取旋转部分"""
        return [row[:3] for row in self.matrix[:3]]
    
    @classmethod
    def identity(cls) -> 'Transform4D':
        """单位矩阵"""
        return cls([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])
    
    def to_quaternion(self) -> Quaternion:
        """将旋转矩阵转换为四元数"""
        # 提取3x3旋转矩阵
        rotation_matrix = self.get_rotation_matrix()
        
        # 转换为四元数（简化实现）
        trace = rotation_matrix[0][0] + rotation_matrix[1][1] + rotation_matrix[2][2]
        
        if trace > 0:
            s = 0.5 / (trace ** 0.5)
            w = 0.25 / s
            x = (rotation_matrix[2][1] - rotation_matrix[1][2]) * s
            y = (rotation_matrix[0][2] - rotation_matrix[2][0]) * s
            z = (rotation_matrix[1][0] - rotation_matrix[0][1]) * s
        else:
            if rotation_matrix[0][0] > rotation_matrix[1][1] and rotation_matrix[0][0] > rotation_matrix[2][2]:
                s = 2.0 * (1.0 + rotation_matrix[0][0] - rotation_matrix[1][1] - rotation_matrix[2][2]) ** 0.5
                w = (rotation_matrix[2][1] - rotation_matrix[1][2]) / s
                x = 0.25 * s
                y = (rotation_matrix[0][1] + rotation_matrix[1][0]) / s
                z = (rotation_matrix[0][2] + rotation_matrix[2][0]) / s
            elif rotation_matrix[1][1] > rotation_matrix[2][2]:
                s = 2.0 * (1.0 + rotation_matrix[1][1] - rotation_matrix[0][0] - rotation_matrix[2][2]) ** 0.5
                w = (rotation_matrix[0][2] - rotation_matrix[2][0]) / s
                x = (rotation_matrix[0][1] + rotation_matrix[1][0]) / s
                y = 0.25 * s
                z = (rotation_matrix[1][2] + rotation_matrix[2][1]) / s
            else:
                s = 2.0 * (1.0 + rotation_matrix[2][2] - rotation_matrix[0][0] - rotation_matrix[1][1]) ** 0.5
                w = (rotation_matrix[1][0] - rotation_matrix[0][1]) / s
                x = (rotation_matrix[0][2] + rotation_matrix[2][0]) / s
                y = (rotation_matrix[1][2] + rotation_matrix[2][1]) / s
                z = 0.25 * s
        
        return Quaternion(w, x, y, z).normalize()


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
class JointInfo:
    """关节信息
    
    完整记录一个关节的所有信息，是导出的核心数据之一：
    
    - name: 关节在Fusion 360中的名称
    - joint_type: 关节类型（旋转副、滑动副等）
    - connection: 关节连接的两个零部件信息
    - geometry: 关节的几何变换信息
    - is_suppressed: 关节是否被抑制（禁用状态）
    - is_light_bulb_on: 关节是否在Fusion 360中激活显示
    
    这些信息用于在MuJoCo中重建相应的关节约束关系
    """
    name: str                              # 关节名称
    joint_type: JointType                  # 关节类型
    connection: JointConnection            # 连接信息
    geometry: JointGeometry                # 几何信息
    is_suppressed: bool                    # 是否被抑制
    is_light_bulb_on: bool                 # 是否激活显示
    
    def to_dict(self) -> Dict[str, Any]:
        """转换为字典"""
        return {
            "name": self.name,
            "joint_type": self.joint_type.value,
            "connection": self.connection.to_dict(),
            "geometry": self.geometry.to_dict(),
            "is_suppressed": self.is_suppressed,
            "is_light_bulb_on": self.is_light_bulb_on
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
            is_light_bulb_on=data["is_light_bulb_on"]
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
class CameraPosition:
    """相机位置信息"""
    position: Vector3D
    focal_point: Vector3D
    view_up: Vector3D


@dataclass
class TransformStatistics:
    """变换统计信息"""
    has_transform_data: bool
    total_transforms: int
    max_translation_distance: float


@dataclass
class ProjectInfo:
    """项目信息
    
    记录项目的基本信息，用于GUI显示和验证：
    
    - project_directory: 项目目录路径
    - export_time: 导出时间
    - geometry_unit: 几何单位
    - position_unit: 位置单位
    - component_count: 零部件数量
    - joint_count: 关节数量
    - format_version: 格式版本
    - has_transform_data: 是否包含变换数据
    """
    project_directory: str
    export_time: str
    geometry_unit: str
    position_unit: str
    component_count: int
    joint_count: int
    format_version: str = "1.0"
    has_transform_data: bool = False


@dataclass
class StageConfig:
    """阶段配置数据类"""
    
    def __init__(self, stage_name: str, display_name: str) -> None:
        """初始化阶段配置
        
        Args:
            stage_name: 阶段名称
            display_name: 显示名称
        """
        self.stage_name = stage_name
        self.display_name = display_name
        self.parameters: Dict[str, Any] = {}
        self.is_enabled = True
        self.is_completed = False
        
    def set_parameter(self, key: str, value: Any) -> None:
        """设置参数
        
        Args:
            key: 参数键
            value: 参数值
        """
        self.parameters[key] = value
        
    def get_parameter(self, key: str, default: Any = None) -> Any:
        """获取参数
        
        Args:
            key: 参数键
            default: 默认值
            
        Returns:
            Any: 参数值
        """
        return self.parameters.get(key, default)
        
    def validate(self) -> List[str]:
        """验证配置参数
        
        Returns:
            List[str]: 错误信息列表
        """
        return []


@dataclass 
class LoadResult:
    """加载结果"""
    success: bool
    models: List['STLModel']  # 前向引用
    message: str
    project_info: Optional[ProjectInfo] = None
    load_mode: Optional[str] = None
    error_details: Optional[str] = None


def create_default_metadata(component_count: int = 0, joint_count: int = 0) -> ExportMetadata:
    """创建默认元数据
    
    根据数据统计创建标准的导出元数据：
    
    - component_count: 零部件数量统计
    - joint_count: 关节数量统计
    
    Returns:
        ExportMetadata: 创建好的元数据对象
    """
    return ExportMetadata(
        export_time=time.strftime("%Y-%m-%d %H:%M:%S"),
        geometry_unit="millimeters",
        position_unit="millimeters",
        matrix_storage="4x4_array",
        count_components=component_count,
        count_joints=joint_count
    )


def save_export_data(data: ExportData, filepath: str) -> bool:
    """保存导出数据到文件
    
    将ExportData对象序列化为JSON格式并保存到指定文件：
    
    - data: 要保存的导出数据对象
    - filepath: 目标文件路径
    
    Returns:
        bool: 保存是否成功
    """
    try:
        data.save_to_file(filepath)
        return True
    except Exception as e:
        print(f"保存导出数据失败: {str(e)}")
        return False


def load_export_data(filepath: str) -> Optional[ExportData]:
    """从文件加载导出数据
    
    从JSON文件中反序列化导出数据：
    
    - filepath: 源文件路径
    
    Returns:
        Optional[ExportData]: 加载的导出数据对象，失败时返回None
    """
    try:
        data = ExportData.load_from_file(filepath)
        return data
    except Exception as e:
        print(f"加载导出数据失败: {str(e)}")
        return None


class BoundingBox:
    """轴对齐包围盒"""
    
    def __init__(self, min_point: Vector3D, max_point: Vector3D):
        self.min_point = min_point
        self.max_point = max_point
        
        # 验证最小点确实小于最大点
        if (min_point.x > max_point.x or 
            min_point.y > max_point.y or 
            min_point.z > max_point.z):
            raise ValueError("Min point must be less than or equal to max point")
    
    def get_center(self) -> Vector3D:
        """获取中心点"""
        return Vector3D(
            (self.min_point.x + self.max_point.x) / 2.0,
            (self.min_point.y + self.max_point.y) / 2.0,
            (self.min_point.z + self.max_point.z) / 2.0
        )
    
    def get_size(self) -> Vector3D:
        """获取尺寸"""
        return Vector3D(
            self.max_point.x - self.min_point.x,
            self.max_point.y - self.min_point.y,
            self.max_point.z - self.min_point.z
        )
    
    def get_volume(self) -> float:
        """获取体积"""
        size = self.get_size()
        return size.x * size.y * size.z
    
    def contains_point(self, point: Vector3D) -> bool:
        """检查点是否在包围盒内"""
        return (self.min_point.x <= point.x <= self.max_point.x and
                self.min_point.y <= point.y <= self.max_point.y and
                self.min_point.z <= point.z <= self.max_point.z)
    
    @classmethod
    def from_points(cls, points: List[Vector3D]) -> 'BoundingBox':
        """从点列表创建包围盒"""
        if not points:
            raise ValueError("Cannot create BoundingBox from empty points list")
        
        min_x = min(point.x for point in points)
        min_y = min(point.y for point in points)
        min_z = min(point.z for point in points)
        max_x = max(point.x for point in points)
        max_y = max(point.y for point in points)
        max_z = max(point.z for point in points)
        
        return cls(
            Vector3D(min_x, min_y, min_z),
            Vector3D(max_x, max_y, max_z)
        )


@dataclass 
class STLModel:
    """STL模型数据"""
    name: str
    mesh_data: bytes  # STL文件的二进制数据
    file_path: Path
    bounding_box: BoundingBox
    vertex_count: int
    face_count: int
    is_transformed: bool = False
    transform_matrix: Optional[Transform4D] = None


@dataclass
class Body4DCoordinates:
    """Body 4D坐标表达"""
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
class KinematicJoint:
    """运动学关节"""
    joint_id: str
    name: str
    joint_type: JointType
    parent_body: str
    child_body: str
    position: Vector3D
    axis: Optional[Vector3D] = None
    limits: Optional[Dict[str, float]] = None
    is_suppressed: bool = False
    is_active: bool = True


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
    mass: float = 1.0
    inertia: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
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


@dataclass
class KinematicTree:
    """MuJoCo 运动学树"""
    roots: List[str]
    nodes: Dict[str, KinematicNode]
    joints: Dict[str, KinematicJoint]
    bodies: Dict[str, KinematicBody]
    relative_transforms: Dict[str, RelativeTransform]
    
    def get_root_bodies(self) -> List[KinematicBody]:
        """获取根刚体列表"""
        return [self.bodies[root_id] for root_id in self.roots if root_id in self.bodies]
    
    def get_body_children(self, body_id: str) -> List[KinematicBody]:
        """获取刚体的子刚体列表"""
        if body_id not in self.nodes:
            return []
        node = self.nodes[body_id]
        return [self.bodies[child_id] for child_id in node.children if child_id in self.bodies]
    
    def get_body_joint(self, body_id: str) -> Optional[KinematicJoint]:
        """获取刚体对应的关节"""
        if body_id not in self.nodes:
            return None
        node = self.nodes[body_id]
        if node.joint and node.joint in self.joints:
            return self.joints[node.joint]
        return None


@dataclass
class ConvertedData:
    """转换后的数据"""
    body_coordinates: Dict[str, Body4DCoordinates]
    joint_coordinates: Dict[str, JointGlobalCoordinates]
    kinematic_tree: Optional[KinematicTree] = None
    filename_mapping: Optional[Dict[str, str]] = None
    conversion_info: Optional[Dict[str, str]] = None
    
    def get_body_count(self) -> int:
        """获取刚体数量"""
        return len(self.body_coordinates)
    
    def get_joint_count(self) -> int:
        """获取关节数量"""
        return len(self.joint_coordinates)


@dataclass
class ProjectContext:
    """项目上下文数据
    
    为ProjectWorkflowManager提供统一的数据容器，
    包含工作流各阶段所需的上下文信息。
    
    所有字段都有明确的类型定义，避免使用Any类型。
    """
    project_directory: Optional[Path] = None
    export_data: Optional[ExportData] = None
    loaded_models: List[STLModel] = field(default_factory=list)
    
    # 处理过程中的数据
    body_4d_coordinates: Dict[str, Body4DCoordinates] = field(default_factory=dict)
    joint_global_coordinates: Dict[str, JointGlobalCoordinates] = field(default_factory=dict)
    kinematic_tree: Optional[KinematicTree] = None
    
    # 转换结果
    converted_data: Optional[ConvertedData] = None
    xml_content: Optional[str] = None
    
    # 元数据和配置
    processing_options: Dict[str, str] = field(default_factory=dict)
    validation_errors: List[str] = field(default_factory=list)
    warning_messages: List[str] = field(default_factory=list)
    
    def reset(self):
        """重置上下文数据"""
        self.project_directory = None
        self.export_data = None
        self.loaded_models.clear()
        self.body_4d_coordinates.clear()
        self.joint_global_coordinates.clear()
        self.kinematic_tree = None
        self.converted_data = None
        self.xml_content = None
        self.processing_options.clear()
        self.validation_errors.clear()
        self.warning_messages.clear()
    
    def get_model_count(self) -> int:
        """获取已加载的模型数量"""
        return len(self.loaded_models)
    
    def get_component_count(self) -> int:
        """获取组件数量"""
        if self.export_data:
            return len(self.export_data.components)
        return 0
    
    def get_joint_count(self) -> int:
        """获取关节数量"""
        if self.export_data:
            return len(self.export_data.joints)
        return 0
    
    def get_processing_status(self) -> Dict[str, int]:
        """获取处理状态统计"""
        return {
            'models_loaded': len(self.loaded_models),
            'bodies_processed': len(self.body_4d_coordinates),
            'joints_processed': len(self.joint_global_coordinates),
            'has_kinematic_tree': self.kinematic_tree is not None,
            'has_converted_data': self.converted_data is not None,
            'has_xml_output': self.xml_content is not None,
            'validation_errors': len(self.validation_errors),
            'warnings': len(self.warning_messages)
        }
    
    def add_validation_error(self, error: str):
        """添加验证错误"""
        self.validation_errors.append(error)
    
    def add_warning(self, warning: str):
        """添加警告信息"""
        self.warning_messages.append(warning)
    
    def is_valid(self) -> bool:
        """检查上下文数据是否有效"""
        return len(self.validation_errors) == 0
    
    def get_summary(self) -> str:
        """获取上下文数据摘要"""
        status = self.get_processing_status()
        return (
            f"项目上下文摘要:\n"
            f"  模型数量: {status['models_loaded']}\n"
            f"  组件数量: {self.get_component_count()}\n"
            f"  关节数量: {self.get_joint_count()}\n"
            f"  已处理刚体: {status['bodies_processed']}\n"
            f"  已处理关节: {status['joints_processed']}\n"
            f"  运动学树: {'已构建' if status['has_kinematic_tree'] else '未构建'}\n"
            f"  转换数据: {'已生成' if status['has_converted_data'] else '未生成'}\n"
            f"  XML输出: {'已生成' if status['has_xml_output'] else '未生成'}\n"
            f"  验证错误: {status['validation_errors']}\n"
            f"  警告信息: {status['warnings']}"
        )