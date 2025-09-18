"""
ODogExample GUI模块 - MuJoCo渲染器

处理MuJoCo模型的核心渲染逻辑和物理仿真。
"""

import sys
import os
import math
import time
import numpy as np
import mujoco
from typing import Optional, Dict, Any

try:
    from ..core.robot_model import RobotModel
    from .camera_system import OrbitCamera
except ImportError:
    # 如果相对导入失败，尝试绝对导入
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from core.robot_model import RobotModel
    from gui.camera_system import OrbitCamera


class MuJoCoRenderer:
    """MuJoCo渲染器 - 处理3D渲染和物理仿真"""
    
    def __init__(self, robot_model: Optional[RobotModel] = None):
        self.robot = robot_model
        
        # MuJoCo渲染资源
        self.scene = None
        self.mjr_context = None
        self.mjcam = None
        self.opt = None
        
        # 仿真控制
        self.is_running = False
        self.simulation_time = 0.0
        
        # FPS计数器
        self.frame_count = 0
        self.last_fps_time = 0
        self.current_fps = 0.0
        
        # 机器人位置跟踪
        self.initial_robot_position = None
        self.last_camera_update_time = 0
        self.camera_tracking_enabled = False
        
        print(f"🔧 MuJoCoRenderer 初始化完成，robot: {self.robot is not None}")
    
    def initialize_renderer(self):
        """初始化渲染资源"""
        print("🔧 initialize_renderer 被调用")
        
        if self.robot and self.robot.model:
            print("🎮 初始化MuJoCo渲染资源...")
            # 优化：减少几何体数量以提高性能
            self.scene = mujoco.MjvScene(self.robot.model, maxgeom=5000)
            self.mjr_context = mujoco.MjrContext(self.robot.model, mujoco.mjtFontScale.mjFONTSCALE_100)
            self.mjcam = mujoco.MjvCamera()
            self.opt = mujoco.MjvOption()
            
            # 初始化默认值
            mujoco.mjv_defaultCamera(self.mjcam)
            mujoco.mjv_defaultOption(self.opt)
            
            # 优化：设置渲染选项以提高性能
            self.opt.flags[mujoco.mjtVisFlag.mjVIS_CONVEXHULL] = 0  # 关闭凸包渲染
            self.opt.flags[mujoco.mjtVisFlag.mjVIS_INERTIA] = 0    # 关闭惯性椭球渲染
            
            # 重新计算模型统计信息以确保准确性
            self.robot._calculate_model_stats()
            
            # 初始化FPS计数器
            self.last_fps_time = time.time()
            
            print("🎮 MuJoCo渲染器初始化完成")
            print(f"🔍 mjcam 状态: {self.mjcam is not None}")
        else:
            print("⚠️  没有机器人模型，无法初始化渲染器")
    
    def setup_camera(self, camera: OrbitCamera):
        """设置相机参数"""
        if self.mjcam is not None and self.robot and self.robot.is_loaded():
            camera.apply_to_mjcam(self.mjcam)
            camera.update_clip_planes(self.robot.model)
    
    def update_robot_tracking(self, camera: OrbitCamera):
        """更新机器人位置跟踪"""
        if not self.camera_tracking_enabled or not self.robot or not self.robot.model:
            return
        
        robot_center = self.robot.data.xpos[1].copy()  # 获取base body的位置
        
        # 记录初始位置
        if self.initial_robot_position is None:
            self.initial_robot_position = robot_center.copy()
            self.mjcam.lookat[:] = self.initial_robot_position
            camera.lookat[:] = self.initial_robot_position
            print(f"📍 设置初始相机中心: [{self.initial_robot_position[0]:.3f}, {self.initial_robot_position[1]:.3f}, {self.initial_robot_position[2]:.3f}]")
        
        # 只有在机器人显著移动时才更新相机中心
        xy_distance = np.sqrt((robot_center[0] - self.mjcam.lookat[0])**2 + 
                             (robot_center[1] - self.mjcam.lookat[1])**2)
        z_distance = abs(robot_center[2] - self.mjcam.lookat[2])
        
        # 更保守的阈值：XY平面10cm，Z轴5cm
        current_time = time.time()
        # 至少间隔1秒才允许再次更新
        time_since_last_update = current_time - self.last_camera_update_time
        
        if (xy_distance > 0.1 or z_distance > 0.05) and time_since_last_update > 1.0:
            self.mjcam.lookat[:] = robot_center
            camera.lookat[:] = robot_center
            self.last_camera_update_time = current_time
            print(f"🔄 更新相机中心: XY={xy_distance:.3f}m, Z={z_distance:.3f}m")
    
    def render_frame(self, camera: OrbitCamera, viewport_width: int, viewport_height: int):
        """渲染一帧"""
        if not (self.robot and self.robot.model and self.mjcam is not None):
            return
        
        # 运行仿真
        if self.is_running:
            self.step_simulation()
        else:
            # 进行物理计算以实现平滑过渡
            # 保持用户设置的actuator控制信号不变
            for _ in range(15):  # 适中的物理计算步数，平衡响应速度和稳定性
                mujoco.mj_step(self.robot.model, self.robot.data)
        
        # 同步相机参数
        self.setup_camera(camera)
        
        # 更新机器人跟踪
        self.update_robot_tracking(camera)
        
        # 设置视口
        viewport = mujoco.MjrRect(0, 0, viewport_width, viewport_height)
        
        # 更新和渲染场景
        mujoco.mjv_updateScene(
            self.robot.model, self.robot.data, self.opt,
            None, self.mjcam, mujoco.mjtCatBit.mjCAT_ALL, self.scene
        )
        mujoco.mjr_render(viewport, self.scene, self.mjr_context)
        
        # 更新FPS计数
        self.update_fps()
        
        # 调试信息：每60帧打印一次
        if self.frame_count % 60 == 0:
            cam_params = camera.get_parameters()
            print(f"🎮 渲染状态: 相机位置=[{cam_params['position'][0]:.3f}, {cam_params['position'][1]:.3f}, {cam_params['position'][2]:.3f}], 距离={camera.distance:.3f}m, 方位角={camera.azimuth:.1f}°, 仰角={camera.elevation:.1f}°")
    
    def step_simulation(self, dt_target=1/60.0):
        """优化：提高物理仿真响应速度"""
        if not self.robot or not self.robot.model:
            return
        
        start = self.robot.data.time
        steps = 0
        max_steps = 60  # 减少最大步数以提高响应速度
        
        while (self.robot.data.time - start) < dt_target and steps < max_steps:
            # 直接调用mj_step以提高性能
            mujoco.mj_step(self.robot.model, self.robot.data)
            steps += 1
        
        self.simulation_time = self.robot.data.time
        
        # 调试信息：减少输出频率
        if steps > 10 and self.frame_count % 120 == 0:
            print(f"⚡ 仿真步数: {steps}, 时间: {self.simulation_time:.3f}s")
    
    def update_fps(self):
        """更新FPS计数"""
        current_time = time.time()
        
        self.frame_count += 1
        
        if current_time - self.last_fps_time >= 1.0:
            self.current_fps = self.frame_count / (current_time - self.last_fps_time)
            self.frame_count = 0
            self.last_fps_time = current_time
    
    def toggle_simulation(self):
        """切换仿真状态"""
        self.is_running = not self.is_running
        status = "🚀 运行" if self.is_running else "⏸️ 暂停"
        print(f"{status} 仿真")
    
    def toggle_camera_tracking(self):
        """切换相机跟踪模式"""
        self.camera_tracking_enabled = not self.camera_tracking_enabled
        status = "🎯 开启" if self.camera_tracking_enabled else "🔒 关闭"
        print(f"{status} 相机跟踪")
        
        if self.camera_tracking_enabled:
            # 重新开启跟踪时，重置初始位置
            self.initial_robot_position = None
            self.last_camera_update_time = 0
    
    def refocus_camera(self, camera: OrbitCamera):
        """重新聚焦到机器人当前位置"""
        if self.robot and self.robot.model:
            # 获取当前机器人位置
            robot_center = self.robot.data.xpos[1].copy()
            
            # 更新相机中心
            if self.mjcam is not None:
                self.mjcam.lookat[:] = robot_center
            camera.lookat[:] = robot_center
            self.initial_robot_position = robot_center.copy()
            self.last_camera_update_time = 0
            
            print(f"🎯 重新聚焦到机器人位置: [{robot_center[0]:.3f}, {robot_center[1]:.3f}, {robot_center[2]:.3f}]")
    
    def set_robot_model(self, robot_model: RobotModel):
        """设置机器人模型"""
        self.robot = robot_model
        
        if self.robot and self.robot.is_loaded():
            # 重置相机跟踪状态
            self.initial_robot_position = None
            self.last_camera_update_time = 0
            
            # 如果已经初始化，更新渲染资源
            if self.mjcam is not None:
                print("🔄 更新现有渲染资源...")
                self.scene = mujoco.MjvScene(self.robot.model, maxgeom=20000)
                self.mjr_context = mujoco.MjrContext(self.robot.model, mujoco.mjtFontScale.mjFONTSCALE_150)
            else:
                print("⚠️  mjcam 未初始化，等待 initializeGL")
    
    def get_performance_info(self) -> Dict[str, Any]:
        """获取性能信息"""
        return {
            'fps': self.current_fps,
            'frame_count': self.frame_count,
            'simulation_time': self.simulation_time,
            'is_running': self.is_running,
            'camera_tracking': self.camera_tracking_enabled
        }
    
    def print_camera_controls(self):
        """打印相机控制说明"""
        print("=== ODogExample 3D查看器控制说明 ===")
        print("🖱️  鼠标控制:")
        print("   左键拖动：轨道旋转（围绕机器人中心）")
        print("   右键拖动或 Shift+左键：平移模型")
        print("     • 鼠标上移：前进（模型远离）")
        print("     • 鼠标下移：后退（模型靠近）")
        print("     • 鼠标左移：向左平移")
        print("     • 鼠标右移：向右平移")
        print("   滚轮：距离缩放")
        print("   Ctrl+滚轮：FOV 视角缩放")
        print("   双击：自动适配模型")
        print("⌨️  键盘控制:")
        print("   空格：开始/暂停仿真")
        print("   R/F：重置相机视角")
        print("   C：打印相机参数")
        print("   P：打印性能信息")
        print("   T：切换相机跟踪模式")
        print("   L：重新聚焦到机器人位置")
        print("🎯 相机跟踪:")
        print("   默认开启：自动跟踪机器人位置")
        print("   按T键：关闭/开启跟踪模式")
        print("   按L键：手动重新聚焦到机器人")
        print("   跟踪关闭时：相机保持固定视角")
        print("=" * 40)