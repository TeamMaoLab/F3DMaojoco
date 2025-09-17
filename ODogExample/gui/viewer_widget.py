"""
ODogExample GUI模块 - 集成通用轨道相机的3D渲染组件

基于通用轨道相机设计说明书的完整实现，提供专业级的3D模型查看和交互功能。
"""

import sys
import math
import numpy as np
import mujoco
from typing import Optional, Tuple, Dict, Any
from PySide6.QtCore import QTimer, Qt, QPointF
from PySide6.QtOpenGLWidgets import QOpenGLWidget
from PySide6.QtGui import QSurfaceFormat
from OpenGL.GL import *

try:
    from ..core.robot_model import RobotModel
except ImportError:
    # 如果相对导入失败，尝试绝对导入
    import sys
    import os
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from core.robot_model import RobotModel


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
        mjcam.type = mujoco.mjtCamera.mjCAMERA_FREE
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
        if hasattr(model, "vis"):
            znear = max(1e-4, self.distance * 0.01)
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


class MuJoCoViewerWidget(QOpenGLWidget):
    """集成通用轨道相机的MuJoCo渲染组件"""
    
    def __init__(self, robot_model: Optional[RobotModel] = None):
        super().__init__()
        
        # 设置焦点策略
        self.setFocusPolicy(Qt.StrongFocus)

        # 机器人模型
        self.robot = robot_model
        
        # 初始化相机
        self.camera = OrbitCamera()
        
        # 交互状态
        self.last_pos = None
        self.interaction_mode = None
        
        # 机器人位置跟踪优化
        self.initial_robot_position = None
        self.last_camera_update_time = 0
        self.camera_tracking_enabled = True  # 默认开启相机跟踪
        
        # 仿真控制
        self.is_running = False
        self.simulation_time = 0.0
        
        # MuJoCo渲染资源
        self.scene = None
        self.mjr_context = None
        self.mjcam = None
        self.opt = None
        
        # 输入处理器
        self.input_handler = InputHandler()
        
        # FPS计数器
        self.frame_count = 0
        self.last_fps_time = 0
        self.current_fps = 0.0
        
        # 初始化定时器 - 优化：降低到30FPS以提高性能
        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(33)  # ~30 FPS，更好的性能
        
        print(f"🔧 MuJoCoViewerWidget 初始化完成，robot: {self.robot is not None}")
        
        # 如果有机器人模型，适配相机
        if self.robot and self.robot.is_loaded():
            print("🤖 在 __init__ 中适配相机")
            self.camera.fit(self.robot.get_model_stats())
    
    def initializeGL(self):
        """初始化OpenGL和MuJoCo资源"""
        print("🔧 initializeGL 被调用")
        
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
            
            # 启用深度测试
            glEnable(GL_DEPTH_TEST)
            
            # 重新计算模型统计信息以确保准确性
            self.robot._calculate_model_stats()
            
            # 确保相机正确适配模型
            model_stats = self.robot.get_model_stats()
            print(f"🔄 使用最新模型统计信息适配相机: 中心={model_stats['center']}, 尺寸={model_stats['extent']:.3f}m")
            
            self.camera.fit(model_stats)
            self.camera.apply_to_mjcam(self.mjcam)
            self.camera.update_clip_planes(self.robot.model)
            
            # 初始化FPS计数器
            import time
            self.last_fps_time = time.time()
            
            print("🎮 MuJoCo渲染器初始化完成")
            print(f"📷 初始相机: 距离={self.camera.distance:.3f}m, 方位={self.camera.azimuth:.1f}°, 仰角={self.camera.elevation:.1f}°")
            print(f"🔍 mjcam 状态: {self.mjcam is not None}")
        else:
            # 没有模型时的默认设置
            print("⚠️  没有机器人模型，使用默认设置")
            glClearColor(0.15, 0.0, 0.0, 1.0)
    
    def resizeGL(self, w, h):
        """窗口大小改变"""
        if self.robot and self.robot.model:
            # 防止无效的窗口大小
            if w > 0 and h > 0:
                # 确保OpenGL上下文当前
                self.makeCurrent()
                # 设置视口为整个widget区域
                glViewport(0, 0, w, h)
                # 设置投影矩阵为正交投影以确保全屏渲染
                glMatrixMode(GL_PROJECTION)
                glLoadIdentity()
                # 使用正交投影确保渲染填满整个视口
                aspect = w / h if h > 0 else 1.0
                glOrtho(-aspect, aspect, -1, 1, -1, 1)
                glMatrixMode(GL_MODELVIEW)
                glLoadIdentity()
                self.doneCurrent()
                
                print(f"📐 视口更新: {w}x{h}, 比例: {aspect:.2f}")
        else:
            # 默认视口设置
            if w > 0 and h > 0:
                self.makeCurrent()
                glViewport(0, 0, w, h)
                self.doneCurrent()
    
    def paintGL(self):
        """渲染循环"""
        if self.robot and self.robot.model and self.mjcam is not None:
            # 运行仿真
            if self.is_running:
                self.step_simulation()
            else:
                # 优化：直接进行物理计算以提高响应速度
                # 设置执行器控制信号并立即进行物理计算
                if self.robot.model.nu > 0:
                    for i in range(self.robot.model.nu):
                        actuator = self.robot.model.actuator(i)
                        joint_id = actuator.trnid[0]
                        joint_addr = self.robot.model.jnt_qposadr[joint_id]
                        current_angle = self.robot.data.qpos[joint_addr]
                        self.robot.data.ctrl[i] = current_angle
                
                # 直接进行多步物理计算以提高响应速度
                for _ in range(2):  # 减少到2步物理计算，避免震荡
                    mujoco.mj_step(self.robot.model, self.robot.data)
            
                        
            # 同步相机参数
            self.camera.apply_to_mjcam(self.mjcam)
            self.camera.update_clip_planes(self.robot.model)
            
            # 设置视口
            viewport = mujoco.MjrRect(0, 0, self.width(), self.height())
            
            # 智能相机跟踪 - 根据开关状态决定是否跟踪
            if self.camera_tracking_enabled:
                robot_center = self.robot.data.xpos[1].copy()  # 获取base body的位置
                
                # 记录初始位置
                if self.initial_robot_position is None:
                    self.initial_robot_position = robot_center.copy()
                    self.mjcam.lookat[:] = self.initial_robot_position
                    self.camera.lookat[:] = self.initial_robot_position
                    print(f"📍 设置初始相机中心: [{self.initial_robot_position[0]:.3f}, {self.initial_robot_position[1]:.3f}, {self.initial_robot_position[2]:.3f}]")
                
                # 只有在机器人显著移动时才更新相机中心
                xy_distance = np.sqrt((robot_center[0] - self.mjcam.lookat[0])**2 + 
                                     (robot_center[1] - self.mjcam.lookat[1])**2)
                z_distance = abs(robot_center[2] - self.mjcam.lookat[2])
                
                # 更保守的阈值：XY平面10cm，Z轴5cm
                import time
                current_time = time.time()
                # 至少间隔1秒才允许再次更新
                time_since_last_update = current_time - self.last_camera_update_time
                
                if (xy_distance > 0.1 or z_distance > 0.05) and time_since_last_update > 1.0:
                    self.mjcam.lookat[:] = robot_center
                    self.camera.lookat[:] = robot_center
                    self.last_camera_update_time = current_time
                    print(f"🔄 更新相机中心: XY={xy_distance:.3f}m, Z={z_distance:.3f}m")
            # 如果相机跟踪关闭，不进行任何自动更新，保持当前视角
            
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
                cam_params = self.camera.get_parameters()
                print(f"🎮 渲染状态: 相机位置=[{cam_params['position'][0]:.3f}, {cam_params['position'][1]:.3f}, {cam_params['position'][2]:.3f}], 距离={self.camera.distance:.3f}m, 方位角={self.camera.azimuth:.1f}°, 仰角={self.camera.elevation:.1f}°")
        else:
            # 没有模型时的默认渲染
            glClearColor(0.2, 0.0, 0.0, 1.0)
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            
            # 调试信息
            if self.frame_count % 60 == 0:
                print(f"⚠️  渲染状态异常: robot={self.robot is not None}, model={self.robot.model if self.robot else None}, mjcam={self.mjcam}")
    
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
        import time
        current_time = time.time()
        
        self.frame_count += 1
        
        if current_time - self.last_fps_time >= 1.0:
            self.current_fps = self.frame_count / (current_time - self.last_fps_time)
            self.frame_count = 0
            self.last_fps_time = current_time
    
    # === 鼠标事件处理 ===
    def mousePressEvent(self, event):
        """鼠标按下"""
        self.input_handler.handle_mouse_press(event)
        self._update_interaction_mode()
    
    def mouseMoveEvent(self, event):
        """鼠标移动"""
        self.input_handler.handle_mouse_move(event)
        
        mouse_delta = self.input_handler.get_mouse_delta()
        if mouse_delta is None or self.interaction_mode is None:
            return
        
        dx, dy = mouse_delta.x(), mouse_delta.y()
        
        if self.interaction_mode == 'orbit':
            self.camera.orbit(dx, dy)
        elif self.interaction_mode == 'pan':
            self.camera.pan(dx, dy, self.height())
        
        self.update()  # 触发重新渲染
    
    def mouseReleaseEvent(self, event):
        """鼠标释放"""
        self.input_handler.handle_mouse_release(event)
        self.interaction_mode = None
    
    def wheelEvent(self, event):
        """滚轮事件"""
        self.input_handler.handle_scroll(event)
        
        steps = self.input_handler.scroll_delta
        mods = event.modifiers()
        
        if mods & Qt.ControlModifier:
            self.camera.zoom_fov(steps)
        else:
            self.camera.dolly(steps)
        
        self.input_handler.clear_scroll()
        self.update()  # 触发重新渲染
    
    def mouseDoubleClickEvent(self, event):
        """双击事件 - 自动适配模型"""
        # event 参数保留用于未来的扩展功能
        if self.robot and self.robot.is_loaded():
            self.camera.fit(self.robot.get_model_stats())
            self.update()
    
    # === 键盘事件处理 ===
    def keyPressEvent(self, event):
        """键盘按下"""
        self.input_handler.keyboard_modifiers[event.modifiers()] = True
        
        # 处理具体按键
        key = event.key()
        if key == Qt.Key_Space:
            self.toggle_simulation()
        elif key == Qt.Key_R or key == Qt.Key_F:
            if self.robot and self.robot.is_loaded():
                self.camera.fit(self.robot.get_model_stats())
                self.update()
        elif key == Qt.Key_C:
            # 打印相机参数（调试用）
            params = self.camera.get_parameters()
            print(f"📷 相机参数: Az={params['azimuth']:.1f}°, El={params['elevation']:.1f}°, "
                  f"Dist={params['distance']:.2f}m, FOV={params['fovy']:.1f}°")
        elif key == Qt.Key_P:
            # 打印性能信息
            print(f"⚡ FPS: {self.current_fps:.1f}")
        elif key == Qt.Key_T:
            # 切换相机跟踪模式
            self.toggle_camera_tracking()
        elif key == Qt.Key_L:
            # 重新聚焦到机器人
            self.refocus_camera()
            
    def keyReleaseEvent(self, event):
        """键盘释放"""
        self.input_handler.keyboard_modifiers[event.modifiers()] = False
    
    def _update_interaction_mode(self):
        """更新交互模式"""
        buttons = self.input_handler.mouse_buttons
        mods = self.input_handler.keyboard_modifiers
        
        left_pressed = buttons.get(Qt.LeftButton, False)
        right_pressed = buttons.get(Qt.RightButton, False)
        shift_pressed = Qt.ShiftModifier in mods
        
        if left_pressed and not right_pressed:
            if shift_pressed:
                self.interaction_mode = 'pan'
            else:
                self.interaction_mode = 'orbit'
        elif right_pressed:
            self.interaction_mode = 'pan'
        else:
            self.interaction_mode = None
    
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
            # 重新开启跟踪时，立即更新到机器人位置
            self.refocus_camera()
    
    def refocus_camera(self):
        """重新聚焦到机器人当前位置"""
        if self.robot and self.robot.model:
            # 获取当前机器人位置
            robot_center = self.robot.data.xpos[1].copy()
            
            # 更新相机中心
            self.mjcam.lookat[:] = robot_center
            self.camera.lookat[:] = robot_center
            self.initial_robot_position = robot_center.copy()
            self.last_camera_update_time = 0
            
            print(f"🎯 重新聚焦到机器人位置: [{robot_center[0]:.3f}, {robot_center[1]:.3f}, {robot_center[2]:.3f}]")
            
            # 触发重新渲染
            self.update()
    
    def set_robot_model(self, robot_model: RobotModel):
        """设置机器人模型"""
        self.robot = robot_model
        
        if self.robot and self.robot.is_loaded():
            # 重置相机跟踪状态
            self.initial_robot_position = None
            self.last_camera_update_time = 0
            
            # 适配相机
            self.camera.fit(self.robot.get_model_stats())
            
            # 如果已经初始化，更新渲染资源
            if self.mjcam is not None:
                print("🔄 更新现有渲染资源...")
                self.scene = mujoco.MjvScene(self.robot.model, maxgeom=20000)
                self.mjr_context = mujoco.MjrContext(self.robot.model, mujoco.mjtFontScale.mjFONTSCALE_150)
                
                self.camera.apply_to_mjcam(self.mjcam)
                self.camera.update_clip_planes(self.robot.model)
            else:
                print("⚠️  mjcam 未初始化，等待 initializeGL")
    
    def showEvent(self, event):
        """窗口显示事件 - 确保OpenGL资源正确初始化"""
        super().showEvent(event)
        
        # 强制重新初始化OpenGL资源
        if self.robot and self.robot.model and self.mjcam is None:
            print("🔄 在 showEvent 中重新初始化OpenGL资源")
            self.makeCurrent()
            self.initializeGL()
            self.doneCurrent()
    
    def print_controls(self):
        """打印控制说明"""
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


def create_test_viewer():
    """创建测试用的查看器"""
    from core.robot_model import create_test_model
    
    print("🎮 创建ODogExample 3D查看器...")
    
    # 创建机器人模型
    robot = create_test_model()
    
    if robot and robot.is_loaded():
        # 创建查看器
        viewer = MuJoCoViewerWidget(robot)
        viewer.print_controls()
        
        return viewer
    else:
        print("❌ 无法创建查看器：机器人模型加载失败")
        return None


if __name__ == "__main__":
    """独立测试脚本"""
    from PySide6.QtWidgets import QApplication, QMainWindow
    
    app = QApplication(sys.argv)
    
    # 创建测试查看器
    viewer = create_test_viewer()
    
    if viewer:
        # 创建主窗口
        window = QMainWindow()
        window.setWindowTitle("ODogExample 3D查看器 - 阶段一测试")
        window.resize(1200, 800)
        window.setCentralWidget(viewer)
        
        window.show()
        print("🎉 3D查看器启动成功！")
        
        sys.exit(app.exec())
    else:
        print("❌ 测试失败")
        sys.exit(1)