"""
ODogExample GUI模块 - 相机系统

包含通用轨道相机控制器和输入处理器。
"""

import math
import numpy as np
from typing import Dict, Any, Optional
from PySide6.QtCore import QPointF


class OrbitCamera:
    """通用轨道相机控制器 - 基于设计说明书的完整实现"""
    
    def __init__(self):
        # 轨道参数
        self.azimuth = 90.0      # 方位角
        self.elevation = -30.0    # 仰角
        self.distance = 2.0      # 观察距离
        self.lookat = np.array([0.0, 0.0, 0.0], dtype=np.float32)  # 观察目标
        self.fovy = 45.0          # 视场角
        
        # 控制参数 - 针对小型模型优化
        self.orbit_sensitivity = 0.35    # 旋转灵敏度
        self.pan_sensitivity = 0.0015    # 平移灵敏度 - 降低以便更精细控制
        self.zoom_speed = 0.12           # 缩放速度
        self.fov_zoom_speed = 0.10       # FOV缩放速度
        
        # 范围限制
        self.min_distance = 0.001
        self.max_distance = 100.0
        self.min_fovy = 5.0
        self.max_fovy = 90.0
        
        # 性能参数
        self.last_update_time = 0
        
    def fit(self, model_stats: Dict[str, Any]):
        """基于模型统计信息自动适配视角 - 确保模型居中"""
        extent = model_stats.get('extent', 0.15)
        center = model_stats.get('center', np.array([0.0, 0.0, 0.029]))
        
        # 设置观察目标为模型中心（机器人位置）
        self.lookat[:] = center
        
        # 使用更适合观察机器人的相机参数
        self.distance = 0.150  # 15cm观察距离
        self.azimuth = 45.0    # 45度方位角
        self.elevation = -20.0  # -20度仰角，稍微俯视
        
        # 设置适合小模型的缩放范围
        self.min_distance = 0.080  # 最小距离8cm
        self.max_distance = 1.000  # 最大距离1m
        
        print(f"📷 相机适配: 距离={self.distance:.3f}m, 目标=[{self.lookat[0]:.3f}, {self.lookat[1]:.3f}, {self.lookat[2]:.3f}]")
        print(f"📏 模型尺寸: {extent:.3f}m, 角度: Az={self.azimuth:.1f}°, El={self.elevation:.1f}°")
    
    def apply_to_mjcam(self, mjcam):
        """应用到MuJoCo相机"""
        mjcam.type = 0  # mjCAMERA_FREE = 0
        mjcam.azimuth = self.azimuth
        mjcam.elevation = self.elevation
        mjcam.distance = self.distance
        mjcam.lookat[:] = self.lookat
        
        # FOV 需要通过 model.vis.global.fovy 设置
        # 这里不设置，将在渲染循环中处理
    
    def orbit(self, dx, dy):
        """球坐标轨道旋转 - 围绕机器人位置旋转"""
        # 方位角旋转（水平方向）
        old_azimuth = self.azimuth
        self.azimuth = (self.azimuth + dx * self.orbit_sensitivity) % 360.0
        
        # 仰角旋转（垂直方向）- 避免万向节锁
        old_elevation = self.elevation
        self.elevation += dy * self.orbit_sensitivity
        self.elevation = max(-89.9, min(89.9, self.elevation))
        
        # 调试信息 - 旋转操作时打印
        if abs(old_azimuth - self.azimuth) > 1.0 or abs(old_elevation - self.elevation) > 1.0:
            print(f"🔄 轨道旋转: 方位角={self.azimuth:.1f}°, 仰角={self.elevation:.1f}°")
    
    def pan(self, dx, dy, viewport_h):
        """屏幕空间平移 - 修复为符合直觉的平移方式"""
        if viewport_h <= 0:
            viewport_h = 1
        
        # 使用更适合小模型的平移灵敏度
        scale = self.distance * self.pan_sensitivity * 0.8  # 稍微降低灵敏度，更容易控制
        
        # 修复为符合直觉的平移：
        # - 鼠标上移：模型向上移动（相机视角感觉是前进）
        # - 鼠标下移：模型向下移动（相机视角感觉是后退）
        # - 鼠标左移：模型向左移动
        # - 鼠标右移：模型向右移动
        # 现在改为直接的世界坐标移动，更符合用户直觉
        
        # 计算相机在水平面的投影方向
        az = math.radians(self.azimuth)
        
        # 考虑相机方位角的坐标系转换
        cos_az = math.cos(az)
        sin_az = math.sin(az)
        
        # 更直观的平移映射：
        # - 屏幕X轴移动 -> 世界坐标系的左右移动
        # - 屏幕Y轴移动 -> 世界坐标系的前后移动（考虑相机朝向）
        world_dx = (dx * cos_az + dy * sin_az) * scale
        world_dy = (dx * sin_az - dy * cos_az) * scale
        
        # 应用平移 - 在XY平面移动，Z轴保持稳定
        old_lookat = self.lookat.copy()
        self.lookat[0] += world_dx  # X轴移动
        self.lookat[1] += world_dy  # Y轴移动
        # Z轴基本保持不变，只做微小的高度调整
        self.lookat[2] += dy * scale * 0.05  # 垂直移动的5%，减少干扰
        
        # 调试信息 - 平移操作时打印
        if np.linalg.norm(self.lookat - old_lookat) > 0.01:
            print(f"🔄 平移操作: 目标=[{self.lookat[0]:.3f}, {self.lookat[1]:.3f}, {self.lookat[2]:.3f}], dXY=[{world_dx:.3f}, {world_dy:.3f}]")
    
    def dolly(self, scroll_steps):
        """距离缩放 - 针对小型机器人优化的缩放算法"""
        # 使用更适合小型机器人的缩放速度 - 进一步减慢以便精细控制
        effective_zoom_speed = self.zoom_speed * 0.3  # 减慢缩放速度，更容易控制小模型
        
        factor = math.exp(-effective_zoom_speed * scroll_steps)
        self.distance *= factor
        
        # 应用更适合小模型的距离限制
        self.distance = max(self.min_distance, min(self.max_distance, self.distance))
        
        # 调试信息 - 添加旋转角度信息
        print(f"🔍 缩放: 距离={self.distance:.3f}m, 方位角={self.azimuth:.1f}°, 仰角={self.elevation:.1f}° (范围: {self.min_distance:.3f}m - {self.max_distance:.3f}m)")
    
    def zoom_fov(self, scroll_steps):
        """视场角缩放"""
        factor = math.exp(-self.fov_zoom_speed * scroll_steps)
        self.fovy *= factor
        self.fovy = max(self.min_fovy, min(self.max_fovy, self.fovy))
    
    def update_clip_planes(self, model):
        """更新裁剪平面 - 动态调整避免深度冲突"""
        # 直接访问MuJoCo模型属性，类型是明确的
        znear = max(1e-4, self.distance * 0.01)
        
        # 设置FOV - 使用global_属性避免Python关键字冲突
        model.vis.global_.fovy = self.fovy
        
        # 设置裁剪平面
        model.vis.map.znear = float(znear)
    
    def get_parameters(self) -> Dict[str, Any]:
        """获取相机参数"""
        # 计算相机位置
        az = math.radians(self.azimuth)
        el = math.radians(self.elevation)
        d = self.distance
        
        x = d * math.cos(el) * math.sin(az)
        y = d * math.cos(el) * math.cos(az)
        z = d * math.sin(el)
        
        position = self.lookat + np.array([x, y, z])
        
        return {
            'azimuth': self.azimuth,
            'elevation': self.elevation,
            'distance': self.distance,
            'fovy': self.fovy,
            'lookat': self.lookat.tolist(),
            'position': position.tolist()
        }


class InputHandler:
    """输入处理器 - 统一管理鼠标和键盘输入"""
    
    def __init__(self):
        self.mouse_pos = None
        self.last_mouse_pos = None
        self.mouse_buttons = {}
        self.keyboard_modifiers = {}
        self.scroll_delta = 0
        
    def handle_mouse_press(self, event):
        """处理鼠标按下"""
        button = event.button()
        self.mouse_buttons[button] = True
        self.mouse_pos = event.position()
        self.last_mouse_pos = self.mouse_pos
        
    def handle_mouse_release(self, event):
        """处理鼠标释放"""
        button = event.button()
        self.mouse_buttons[button] = False
        
    def handle_mouse_move(self, event):
        """处理鼠标移动"""
        self.last_mouse_pos = self.mouse_pos
        self.mouse_pos = event.position()
        
    def handle_scroll(self, event):
        """处理滚轮"""
        self.scroll_delta = event.angleDelta().y() / 120.0
        
    def get_mouse_delta(self) -> Optional[QPointF]:
        """获取鼠标移动增量"""
        if self.mouse_pos and self.last_mouse_pos:
            return self.mouse_pos - self.last_mouse_pos
        return None
    
    def clear_scroll(self):
        """清除滚轮状态"""
        self.scroll_delta = 0


def create_orbit_camera() -> OrbitCamera:
    """创建轨道相机实例"""
    return OrbitCamera()


def create_input_handler():
    """创建输入处理器实例"""
    return InputHandler()