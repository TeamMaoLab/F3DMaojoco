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
        
        # 控制参数
        self.orbit_sensitivity = 0.35    # 旋转灵敏度
        self.pan_sensitivity = 0.0025    # 平移灵敏度
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
        """基于模型统计信息自动适配视角"""
        extent = model_stats.get('extent', 0.047)
        center = model_stats.get('center', np.array([0.0, 0.0, 0.0]))
        
        # 设置观察目标
        self.lookat[:] = center
        
        # 计算最佳观察距离
        half_fov_rad = math.radians(self.fovy) / 2.0
        base = extent / (2.0 * math.tan(half_fov_rad) + 1e-9)
        self.distance = base * 1.2  # 留出边距
        
        # 动态调整范围限制
        self.min_distance = max(extent * 0.01, 0.0005)
        self.max_distance = extent * 20.0
        
        print(f"📷 相机适配: 距离={self.distance:.3f}m, 目标=[{center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f}]")
    
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
        """球坐标轨道旋转"""
        # 方位角旋转（水平方向）
        self.azimuth = (self.azimuth + dx * self.orbit_sensitivity) % 360.0
        
        # 仰角旋转（垂直方向）- 避免万向节锁
        self.elevation += dy * self.orbit_sensitivity
        self.elevation = max(-89.9, min(89.9, self.elevation))
    
    def pan(self, dx, dy, viewport_h):
        """屏幕空间平移"""
        if viewport_h <= 0:
            viewport_h = 1
        
        # 根据距离调整平移灵敏度
        scale = self.distance * self.pan_sensitivity
        
        # 计算相机坐标系
        az = math.radians(self.azimuth)
        el = math.radians(self.elevation)
        
        # 前方向
        forward = np.array([
            math.cos(el) * math.sin(az),
            math.cos(el) * math.cos(az),
            math.sin(el)
        ], dtype=np.float32)
        
        # 构建相机坐标系
        world_up = np.array([0, 0, 1], dtype=np.float32)
        right = np.cross(forward, world_up)
        if np.linalg.norm(right) < 1e-8:
            right = np.array([1, 0, 0], dtype=np.float32)
        right /= np.linalg.norm(right)
        
        up = np.cross(right, forward)
        up /= np.linalg.norm(up)
        
        # 应用平移
        self.lookat -= right * dx * scale
        self.lookat += up * dy * scale
    
    def dolly(self, scroll_steps):
        """距离缩放 - 指数缩放算法"""
        factor = math.exp(-self.zoom_speed * scroll_steps)
        self.distance *= factor
        self.distance = max(self.min_distance, min(self.max_distance, self.distance))
    
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
        
        # 初始化定时器
        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(16)  # ~60 FPS
        
        # 如果有机器人模型，适配相机
        if self.robot and self.robot.is_loaded():
            self.camera.fit(self.robot.get_model_stats())
    
    def initializeGL(self):
        """初始化OpenGL和MuJoCo资源"""
        if self.robot and self.robot.model:
            # 预分配渲染资源，避免每帧重新分配
            self.scene = mujoco.MjvScene(self.robot.model, maxgeom=20000)
            self.mjr_context = mujoco.MjrContext(self.robot.model, mujoco.mjtFontScale.mjFONTSCALE_150)
            self.mjcam = mujoco.MjCamera()
            self.opt = mujoco.MjvOption()
            
            # 初始化默认值
            mujoco.mjv_defaultCamera(self.mjcam)
            mujoco.mjv_defaultOption(self.opt)
            
            # 启用深度测试
            glEnable(GL_DEPTH_TEST)
            
            # 确保相机正确适配模型
            self.camera.fit(self.robot.get_model_stats())
            self.camera.apply_to_mjcam(self.mjcam)
            self.camera.update_clip_planes(self.robot.model)
            
            # 初始化FPS计数器
            import time
            self.last_fps_time = time.time()
            
            print("🎮 MuJoCo渲染器初始化完成")
            print(f"📷 初始相机: 距离={self.camera.distance:.3f}m, 方位={self.camera.azimuth:.1f}°, 仰角={self.camera.elevation:.1f}°")
        else:
            # 没有模型时的默认设置
            glClearColor(0.15, 0.0, 0.0, 1.0)
    
    def resizeGL(self, w, h):
        """窗口大小改变"""
        if self.robot and self.robot.model:
            glViewport(0, 0, w, h)
    
    def paintGL(self):
        """渲染循环"""
        if self.robot and self.robot.model and self.mjcam is not None:
            # 运行仿真
            if self.is_running:
                self.step_simulation()
            
            # 设置FOV - 暂时跳过，避免global关键字问题
            # TODO: 找到正确的MuJoCo FOV设置方式
            
            # 同步相机参数
            self.camera.apply_to_mjcam(self.mjcam)
            self.camera.update_clip_planes(self.robot.model)
            
            # 设置视口
            viewport = mujoco.MjrRect(0, 0, self.width(), self.height())
            
            # 更新和渲染场景
            mujoco.mjv_updateScene(
                self.robot.model, self.robot.data, self.opt,
                None, self.mjcam, mujoco.mjtCatBit.mjCAT_ALL, self.scene
            )
            mujoco.mjr_render(viewport, self.scene, self.mjr_context)
            
            # 更新FPS计数
            self.update_fps()
        else:
            # 没有模型时的默认渲染
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    
    def step_simulation(self, dt_target=1/60.0):
        """多步仿真保证时间同步"""
        if not self.robot or not self.robot.model:
            return
        
        start = self.robot.data.time
        steps = 0
        max_steps = 400  # 防止无限循环
        
        while (self.robot.data.time - start) < dt_target and steps < max_steps:
            self.robot.step_simulation()
            steps += 1
        
        self.simulation_time = self.robot.data.time
    
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
    
    def set_robot_model(self, robot_model: RobotModel):
        """设置机器人模型"""
        self.robot = robot_model
        
        if self.robot and self.robot.is_loaded():
            # 适配相机
            self.camera.fit(self.robot.get_model_stats())
            
            # 如果已经初始化，更新渲染资源
            if self.scene is not None:
                self.scene = mujoco.MjvScene(self.robot.model, maxgeom=20000)
                self.mjr_context = mujoco.MjrContext(self.robot.model, mujoco.mjtFontScale.mjFONTSCALE_150)
                
                self.camera.apply_to_mjcam(self.mjcam)
                self.camera.update_clip_planes(self.robot.model)
    
    def print_controls(self):
        """打印控制说明"""
        print("=== ODogExample 3D查看器控制说明 ===")
        print("🖱️  鼠标控制:")
        print("   左键拖动：轨道旋转")
        print("   右键拖动或 Shift+左键：平移")
        print("   滚轮：距离缩放")
        print("   Ctrl+滚轮：FOV 缩放")
        print("   双击：自动适配模型")
        print("⌨️  键盘控制:")
        print("   空格：开始/暂停仿真")
        print("   R/F：重置相机视角")
        print("   C：打印相机参数")
        print("   P：打印性能信息")
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