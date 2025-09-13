"""
几何数学模块

提供F3DMaojocoScripts和MaojocoConverter共享的几何计算类和函数。
包含向量、矩阵、变换等基础几何类型和计算功能。

设计原则：
- 使用Python 3.12+标准库，无外部依赖
- 统一使用毫米单位（转换后的输出标准）
- 提供完整的类型注解
- 优化性能，避免不必要的对象创建
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class Vector3D:
    """3D向量类，支持坐标系转换"""
    x: float
    y: float
    z: float
    
    def __add__(self, other: Vector3D) -> Vector3D:
        """向量加法"""
        return Vector3D(self.x + other.x, self.y + other.y, self.z + other.z)
    
    def __sub__(self, other: Vector3D) -> Vector3D:
        """向量减法"""
        return Vector3D(self.x - other.x, self.y - other.y, self.z - other.z)
    
    def __mul__(self, scalar: float) -> Vector3D:
        """标量乘法"""
        return Vector3D(self.x * scalar, self.y * scalar, self.z * scalar)
    
    def __rmul__(self, scalar: float) -> Vector3D:
        """标量乘法（右乘）"""
        return self.__mul__(scalar)
    
    def __truediv__(self, scalar: float) -> Vector3D:
        """标量除法"""
        return Vector3D(self.x / scalar, self.y / scalar, self.z / scalar)
    
    def dot(self, other: Vector3D) -> float:
        """点积"""
        return self.x * other.x + self.y * other.y + self.z * other.z
    
    def cross(self, other: Vector3D) -> Vector3D:
        """叉积"""
        return Vector3D(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x
        )
    
    def magnitude(self) -> float:
        """向量长度"""
        return math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)
    
    def normalize(self) -> Vector3D:
        """归一化向量"""
        mag = self.magnitude()
        if mag == 0:
            return Vector3D(0, 0, 0)
        return self / mag
    
    def distance_to(self, other: Vector3D) -> float:
        """到另一个点的距离"""
        return (self - other).magnitude()
    
      
    def to_list(self) -> List[float]:
        """转换为列表"""
        return [self.x, self.y, self.z]
    
    def to_tuple(self) -> Tuple[float, float, float]:
        """转换为元组"""
        return (self.x, self.y, self.z)
    
    @classmethod
    def from_list(cls, values: List[float]) -> Vector3D:
        """从列表创建向量"""
        if len(values) != 3:
            raise ValueError("Vector3D requires exactly 3 values")
        return cls(values[0], values[1], values[2])
    
    @classmethod
    def zero(cls) -> Vector3D:
        """零向量"""
        return cls(0, 0, 0)
    
    @classmethod
    def one(cls) -> Vector3D:
        """单位向量"""
        return cls(1, 1, 1)
    
    def to_mujoco_units(self) -> Vector3D:
        """转换为MuJoCo单位（米）
        
        Fusion 360使用毫米，MuJoCo使用米。
        """
        def clean_value(value: float) -> float:
            """清理浮点精度误差"""
            scaled_value = value / 1000.0
            if abs(scaled_value) < 1e-12:  # 小于1e-12的值视为0
                return 0.0
            return round(scaled_value, 12)
        
        return Vector3D(clean_value(self.x), clean_value(self.y), clean_value(self.z))
    
    def to_fusion_units(self) -> Vector3D:
        """转换为Fusion 360单位（毫米）
        
        从米转换回毫米。
        """
        return Vector3D(self.x * 1000.0, self.y * 1000.0, self.z * 1000.0)


@dataclass
class Quaternion:
    """四元数类，用于表示旋转"""
    w: float
    x: float
    y: float
    z: float
    
    def __mul__(self, other: Quaternion) -> Quaternion:
        """四元数乘法"""
        return Quaternion(
            self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
            self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w
        )
    
    def conjugate(self) -> Quaternion:
        """共轭四元数"""
        return Quaternion(self.w, -self.x, -self.y, -self.z)
    
    def inverse(self) -> Quaternion:
        """逆四元数"""
        conj = self.conjugate()
        norm_sq = self.norm_squared()
        if norm_sq == 0:
            return Quaternion(0, 0, 0, 0)
        return Quaternion(conj.w / norm_sq, conj.x / norm_sq, conj.y / norm_sq, conj.z / norm_sq)
    
    def norm_squared(self) -> float:
        """范数平方"""
        return self.w ** 2 + self.x ** 2 + self.y ** 2 + self.z ** 2
    
    def norm(self) -> float:
        """范数"""
        return math.sqrt(self.norm_squared())
    
    def normalize(self) -> Quaternion:
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
    
    def rotate_vector(self, vector: Vector3D) -> Vector3D:
        """用四元数旋转向量"""
        # 将向量转换为四元数
        vec_quat = Quaternion(0, vector.x, vector.y, vector.z)
        
        # 执行旋转: q * v * q^(-1)
        result = self * vec_quat * self.inverse()
        
        return Vector3D(result.x, result.y, result.z)
    
    @classmethod
    def from_axis_angle(cls, axis: Vector3D, angle: float) -> Quaternion:
        """从轴角创建四元数"""
        # 归一化轴
        axis_norm = axis.normalize()
        
        # 计算半角
        half_angle = angle / 2.0
        sin_half = math.sin(half_angle)
        cos_half = math.cos(half_angle)
        
        return cls(
            cos_half,
            axis_norm.x * sin_half,
            axis_norm.y * sin_half,
            axis_norm.z * sin_half
        )
    
    @classmethod
    def from_euler_angles(cls, roll: float, pitch: float, yaw: float) -> Quaternion:
        """从欧拉角创建四元数（ZYX顺序）"""
        # 计算半角
        cr = math.cos(roll / 2.0)
        sr = math.sin(roll / 2.0)
        cp = math.cos(pitch / 2.0)
        sp = math.sin(pitch / 2.0)
        cy = math.cos(yaw / 2.0)
        sy = math.sin(yaw / 2.0)
        
        # 计算四元数分量
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return cls(w, x, y, z)
    
    @classmethod
    def identity(cls) -> Quaternion:
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
    
    def __mul__(self, other: Transform4D) -> Transform4D:
        """矩阵乘法"""
        result = [[0.0 for _ in range(4)] for _ in range(4)]
        for i in range(4):
            for j in range(4):
                for k in range(4):
                    result[i][j] += self.matrix[i][k] * other.matrix[k][j]
        return Transform4D(result)
    
    def transform_point(self, point: Vector3D) -> Vector3D:
        """变换点"""
        x = self.matrix[0][0] * point.x + self.matrix[0][1] * point.y + self.matrix[0][2] * point.z + self.matrix[0][3]
        y = self.matrix[1][0] * point.x + self.matrix[1][1] * point.y + self.matrix[1][2] * point.z + self.matrix[1][3]
        z = self.matrix[2][0] * point.x + self.matrix[2][1] * point.y + self.matrix[2][2] * point.z + self.matrix[2][3]
        return Vector3D(x, y, z)
    
    def transform_vector(self, vector: Vector3D) -> Vector3D:
        """变换向量（不包含平移）"""
        x = self.matrix[0][0] * vector.x + self.matrix[0][1] * vector.y + self.matrix[0][2] * vector.z
        y = self.matrix[1][0] * vector.x + self.matrix[1][1] * vector.y + self.matrix[1][2] * vector.z
        z = self.matrix[2][0] * vector.x + self.matrix[2][1] * vector.y + self.matrix[2][2] * vector.z
        return Vector3D(x, y, z)
    
    def inverse(self) -> Transform4D:
        """求逆矩阵"""
        # 简化实现，假设是仿射变换
        # 提取旋转和平移部分
        rotation = [row[:3] for row in self.matrix[:3]]
        translation = [row[3] for row in self.matrix[:3]]
        
        # 计算旋转矩阵的逆
        det = (rotation[0][0] * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1]) -
               rotation[0][1] * (rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0]) +
               rotation[0][2] * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0]))
        
        if abs(det) < 1e-10:
            raise ValueError("Matrix is singular")
        
        inv_det = 1.0 / det
        
        inv_rotation = [[0.0 for _ in range(3)] for _ in range(3)]
        inv_rotation[0][0] = inv_det * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1])
        inv_rotation[0][1] = inv_det * (rotation[0][2] * rotation[2][1] - rotation[0][1] * rotation[2][2])
        inv_rotation[0][2] = inv_det * (rotation[0][1] * rotation[1][2] - rotation[0][2] * rotation[1][1])
        inv_rotation[1][0] = inv_det * (rotation[1][2] * rotation[2][0] - rotation[1][0] * rotation[2][2])
        inv_rotation[1][1] = inv_det * (rotation[0][0] * rotation[2][2] - rotation[0][2] * rotation[2][0])
        inv_rotation[1][2] = inv_det * (rotation[0][2] * rotation[1][0] - rotation[0][0] * rotation[1][2])
        inv_rotation[2][0] = inv_det * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0])
        inv_rotation[2][1] = inv_det * (rotation[0][1] * rotation[2][0] - rotation[0][0] * rotation[2][1])
        inv_rotation[2][2] = inv_det * (rotation[0][0] * rotation[1][1] - rotation[0][1] * rotation[1][0])
        
        # 计算逆平移
        inv_translation = [
            -(inv_rotation[0][0] * translation[0] + inv_rotation[0][1] * translation[1] + inv_rotation[0][2] * translation[2]),
            -(inv_rotation[1][0] * translation[0] + inv_rotation[1][1] * translation[1] + inv_rotation[1][2] * translation[2]),
            -(inv_rotation[2][0] * translation[0] + inv_rotation[2][1] * translation[1] + inv_rotation[2][2] * translation[2])
        ]
        
        # 构建逆矩阵
        inv_matrix = [
            [inv_rotation[0][0], inv_rotation[0][1], inv_rotation[0][2], inv_translation[0]],
            [inv_rotation[1][0], inv_rotation[1][1], inv_rotation[1][2], inv_translation[1]],
            [inv_rotation[2][0], inv_rotation[2][1], inv_rotation[2][2], inv_translation[2]],
            [0.0, 0.0, 0.0, 1.0]
        ]
        
        return Transform4D(inv_matrix)
    
        
    def get_translation(self) -> Vector3D:
        """获取平移部分"""
        return Vector3D(self.matrix[0][3], self.matrix[1][3], self.matrix[2][3])
    
    def get_rotation_matrix(self) -> List[List[float]]:
        """获取旋转部分"""
        return [row[:3] for row in self.matrix[:3]]
    
    @classmethod
    def from_translation(cls, translation: Vector3D) -> Transform4D:
        """从平移创建变换矩阵"""
        return cls([
            [1.0, 0.0, 0.0, translation.x],
            [0.0, 1.0, 0.0, translation.y],
            [0.0, 0.0, 1.0, translation.z],
            [0.0, 0.0, 0.0, 1.0]
        ])
    
    @classmethod
    def from_rotation_matrix(cls, rotation: List[List[float]]) -> Transform4D:
        """从旋转矩阵创建变换矩阵"""
        if len(rotation) != 3 or any(len(row) != 3 for row in rotation):
            raise ValueError("Rotation matrix must be 3x3")
        
        matrix = [
            [rotation[0][0], rotation[0][1], rotation[0][2], 0.0],
            [rotation[1][0], rotation[1][1], rotation[1][2], 0.0],
            [rotation[2][0], rotation[2][1], rotation[2][2], 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ]
        return cls(matrix)
    
    @classmethod
    def from_quaternion(cls, quaternion: Quaternion) -> Transform4D:
        """从四元数创建变换矩阵"""
        rotation = quaternion.to_rotation_matrix()
        return cls.from_rotation_matrix(rotation)
    
    @classmethod
    def from_translation_rotation(cls, translation: Vector3D, rotation: Quaternion) -> Transform4D:
        """从平移和旋转创建变换矩阵"""
        rotation_matrix = rotation.to_rotation_matrix()
        return cls([
            [rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2], translation.x],
            [rotation_matrix[1][0], rotation_matrix[1][1], rotation_matrix[1][2], translation.y],
            [rotation_matrix[2][0], rotation_matrix[2][1], rotation_matrix[2][2], translation.z],
            [0.0, 0.0, 0.0, 1.0]
        ])
    
    @classmethod
    def identity(cls) -> Transform4D:
        """单位矩阵"""
        return cls([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])
    
    def to_mujoco_units(self) -> 'Transform4D':
        """转换为MuJoCo单位（米）
        
        Fusion 360使用毫米，MuJoCo使用米。
        只缩放位置分量，保持旋转部分不变。
        """
        # 创建新的矩阵，只缩放位置分量
        scaled_matrix = []
        for i, row in enumerate(self.matrix):
            if i < 3:  # 前3行
                scaled_row = row.copy()
                # 清理浮点精度误差并转换单位
                scaled_value = row[3] / 1000.0  # 毫米转米
                
                # 应用浮点容差清理
                if abs(scaled_value) < 1e-12:  # 小于1e-12的值视为0
                    scaled_row[3] = 0.0
                else:
                    # 保留12位有效数字
                    scaled_row[3] = round(scaled_value, 12)
                
                scaled_matrix.append(scaled_row)
            else:  # 最后一行
                scaled_matrix.append(row.copy())
        
        return Transform4D(scaled_matrix)
    
    def to_fusion_units(self) -> 'Transform4D':
        """转换为Fusion 360单位（毫米）
        
        从米转换回毫米。
        只缩放位置分量，保持旋转部分不变。
        """
        # 创建新的矩阵，只缩放位置分量
        scaled_matrix = []
        for i, row in enumerate(self.matrix):
            if i < 3:  # 前3行
                scaled_row = row.copy()
                scaled_row[3] = row[3] * 1000.0  # 米转毫米
                scaled_matrix.append(scaled_row)
            else:  # 最后一行
                scaled_matrix.append(row.copy())
        
        return Transform4D(scaled_matrix)
    
    def to_quaternion(self) -> Quaternion:
        """从旋转矩阵提取四元数
        
        使用标准的矩阵到四元数转换算法。
        """
        # 提取3x3旋转矩阵
        m = self.get_rotation_matrix()
        
        # 计算四元数分量
        trace = m[0][0] + m[1][1] + m[2][2]
        
        if trace > 0:
            s = math.sqrt(trace + 1.0) * 2.0
            w = 0.25 * s
            x = (m[2][1] - m[1][2]) / s
            y = (m[0][2] - m[2][0]) / s
            z = (m[1][0] - m[0][1]) / s
        elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
            s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
            w = (m[2][1] - m[1][2]) / s
            x = 0.25 * s
            y = (m[0][1] + m[1][0]) / s
            z = (m[0][2] + m[2][0]) / s
        elif m[1][1] > m[2][2]:
            s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
            w = (m[0][2] - m[2][0]) / s
            x = (m[0][1] + m[1][0]) / s
            y = 0.25 * s
            z = (m[1][2] + m[2][1]) / s
        else:
            s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
            w = (m[1][0] - m[0][1]) / s
            x = (m[0][2] + m[2][0]) / s
            y = (m[1][2] + m[2][1]) / s
            z = 0.25 * s
        
        return Quaternion(w, x, y, z)


class GeometryUtils:
    """几何工具类"""
    
    @staticmethod
    def deg_to_rad(degrees: float) -> float:
        """角度转弧度"""
        return degrees * math.pi / 180.0
    
    @staticmethod
    def rad_to_deg(radians: float) -> float:
        """弧度转角度"""
        return radians * 180.0 / math.pi
    
    @staticmethod
    def lerp_vector(a: Vector3D, b: Vector3D, t: float) -> Vector3D:
        """线性插值向量"""
        return a + (b - a) * t
    
    @staticmethod
    def lerp_quaternion(a: Quaternion, b: Quaternion, t: float) -> Quaternion:
        """球面线性插值四元数"""
        # 计算点积
        dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z
        
        # 如果点积为负，取反以确保最短路径
        if dot < 0:
            b = Quaternion(-b.w, -b.x, -b.y, -b.z)
            dot = -dot
        
        # 如果四元数非常接近，直接线性插值
        if dot > 0.9995:
            result = Quaternion(
                a.w + t * (b.w - a.w),
                a.x + t * (b.x - a.x),
                a.y + t * (b.y - a.y),
                a.z + t * (b.z - a.z)
            )
            return result.normalize()
        
        # 计算插值角度
        theta_0 = math.acos(abs(dot))
        sin_theta_0 = math.sin(theta_0)
        theta = theta_0 * t
        sin_theta = math.sin(theta)
        
        s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0
        
        return Quaternion(
            s0 * a.w + s1 * b.w,
            s0 * a.x + s1 * b.x,
            s0 * a.y + s1 * b.y,
            s0 * a.z + s1 * b.z
        )
    
    @staticmethod
    def vector_angle_between(a: Vector3D, b: Vector3D) -> float:
        """计算两个向量之间的角度"""
        cos_angle = a.dot(b) / (a.magnitude() * b.magnitude())
        # 处理数值误差
        cos_angle = max(-1.0, min(1.0, cos_angle))
        return math.acos(cos_angle)
    
    @staticmethod
    def vector_project_onto(a: Vector3D, b: Vector3D) -> Vector3D:
        """将向量a投影到向量b上"""
        b_sq = b.dot(b)
        if b_sq == 0:
            return Vector3D.zero()
        return b * (a.dot(b) / b_sq)
    
    @staticmethod
    def vector_reject_from(a: Vector3D, b: Vector3D) -> Vector3D:
        """计算向量a相对于向量b的垂直分量"""
        return a - GeometryUtils.vector_project_onto(a, b)
    
    @staticmethod
    def point_to_line_distance(point: Vector3D, line_start: Vector3D, line_end: Vector3D) -> float:
        """计算点到直线的距离"""
        line_vec = line_end - line_start
        point_vec = point - line_start
        
        if line_vec.magnitude() == 0:
            return point_vec.magnitude()
        
        # 计算投影长度
        projection_length = point_vec.dot(line_vec) / line_vec.magnitude()
        
        # 计算投影点
        projection_point = line_start + line_vec.normalize() * projection_length
        
        # 返回距离
        return point.distance_to(projection_point)
    
    @staticmethod
    def point_to_plane_distance(point: Vector3D, plane_point: Vector3D, plane_normal: Vector3D) -> float:
        """计算点到平面的距离"""
        normal = plane_normal.normalize()
        return abs((point - plane_point).dot(normal))
    
    @staticmethod
    def are_points_collinear(points: List[Vector3D], tolerance: float = 1e-6) -> bool:
        """检查点是否共线"""
        if len(points) < 3:
            return True
        
        # 使用前三个点确定直线方向
        v1 = points[1] - points[0]
        v2 = points[2] - points[0]
        
        # 计算叉积，如果为零则共线
        cross = v1.cross(v2)
        if cross.magnitude() > tolerance:
            return False
        
        # 检查其他点
        for i in range(3, len(points)):
            v = points[i] - points[0]
            cross = v1.cross(v)
            if cross.magnitude() > tolerance:
                return False
        
        return True
    
    @staticmethod
    def are_points_coplanar(points: List[Vector3D], tolerance: float = 1e-6) -> bool:
        """检查点是否共面"""
        if len(points) < 4:
            return True
        
        # 使用前四个点确定平面
        v1 = points[1] - points[0]
        v2 = points[2] - points[0]
        v3 = points[3] - points[0]
        
        # 计算标量三重积
        triple_product = v1.dot(v2.cross(v3))
        if abs(triple_product) > tolerance:
            return False
        
        # 检查其他点
        for i in range(4, len(points)):
            v = points[i] - points[0]
            triple_product = v1.dot(v2.cross(v))
            if abs(triple_product) > tolerance:
                return False
        
        return True


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
    
    def intersects(self, other: BoundingBox) -> bool:
        """检查与另一个包围盒是否相交"""
        return (self.min_point.x <= other.max_point.x and
                self.max_point.x >= other.min_point.x and
                self.min_point.y <= other.max_point.y and
                self.max_point.y >= other.min_point.y and
                self.min_point.z <= other.max_point.z and
                self.max_point.z >= other.min_point.z)
    
    def expand(self, delta: float) -> BoundingBox:
        """扩展包围盒"""
        delta_vec = Vector3D(delta, delta, delta)
        return BoundingBox(self.min_point - delta_vec, self.max_point + delta_vec)
    
    def transform(self, transform: Transform4D) -> BoundingBox:
        """变换包围盒"""
        # 变换所有8个角点
        corners = [
            Vector3D(self.min_point.x, self.min_point.y, self.min_point.z),
            Vector3D(self.min_point.x, self.min_point.y, self.max_point.z),
            Vector3D(self.min_point.x, self.max_point.y, self.min_point.z),
            Vector3D(self.min_point.x, self.max_point.y, self.max_point.z),
            Vector3D(self.max_point.x, self.min_point.y, self.min_point.z),
            Vector3D(self.max_point.x, self.min_point.y, self.max_point.z),
            Vector3D(self.max_point.x, self.max_point.y, self.min_point.z),
            Vector3D(self.max_point.x, self.max_point.y, self.max_point.z)
        ]
        
        # 变换所有角点
        transformed_corners = [transform.transform_point(corner) for corner in corners]
        
        # 计算新的包围盒
        min_x = min(corner.x for corner in transformed_corners)
        min_y = min(corner.y for corner in transformed_corners)
        min_z = min(corner.z for corner in transformed_corners)
        max_x = max(corner.x for corner in transformed_corners)
        max_y = max(corner.y for corner in transformed_corners)
        max_z = max(corner.z for corner in transformed_corners)
        
        return BoundingBox(Vector3D(min_x, min_y, min_z), Vector3D(max_x, max_y, max_z))
    
        
    @classmethod
    def from_points(cls, points: List[Vector3D]) -> BoundingBox:
        """从点列表创建包围盒"""
        if not points:
            raise ValueError("Cannot create BoundingBox from empty points list")
        
        min_x = min(point.x for point in points)
        min_y = min(point.y for point in points)
        min_z = min(point.z for point in points)
        max_x = max(point.x for point in points)
        max_y = max(point.y for point in points)
        max_z = max(point.z for point in points)
        
        return cls(Vector3D(min_x, min_y, min_z), Vector3D(max_x, max_y, max_z))
    
    @classmethod
    def from_sphere(cls, center: Vector3D, radius: float) -> BoundingBox:
        """从球体创建包围盒"""
        delta = Vector3D(radius, radius, radius)
        return cls(center - delta, center + delta)