"""
ODogExample GUI模块 - 查看器组件

集成了相机系统和MuJoCo渲染器的3D查看器组件。
"""

from typing import Optional
from PySide6.QtCore import QTimer, Qt
from PySide6.QtOpenGLWidgets import QOpenGLWidget
from PySide6.QtGui import QSurfaceFormat
from OpenGL.GL import *

from ..core.robot_model import RobotModel
from .camera_system import OrbitCamera, InputHandler
from .mujoco_renderer import MuJoCoRenderer


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
        self.interaction_mode = None
        
        # 渲染器
        self.renderer = MuJoCoRenderer(robot_model)
        
        # 输入处理器
        self.input_handler = InputHandler()
        
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
        
        # 初始化渲染器
        self.renderer.initialize_renderer()
        
        if self.renderer.mjcam is not None:
            # 确保相机正确适配模型
            model_stats = self.robot.get_model_stats()
            print(f"🔄 使用最新模型统计信息适配相机: 中心={model_stats['center']}, 尺寸={model_stats['extent']:.3f}m")
            
            self.camera.fit(model_stats)
            self.renderer.setup_camera(self.camera)
            
            print(f"📷 初始相机: 距离={self.camera.distance:.3f}m, 方位={self.camera.azimuth:.1f}°, 仰角={self.camera.elevation:.1f}°")
        else:
            # 没有模型时的默认设置
            print("⚠️  没有机器人模型，使用默认设置")
            glClearColor(0.15, 0.0, 0.0, 1.0)
    
    def resizeGL(self, w, h):
        """窗口大小改变"""
        if self.renderer and self.renderer.scene is not None:
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
        if self.renderer and self.renderer.mjcam is not None:
            # 渲染帧
            self.renderer.render_frame(self.camera, self.width(), self.height())
        else:
            # 没有模型时的默认渲染
            glClearColor(0.2, 0.0, 0.0, 1.0)
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            
            # 调试信息
            if self.renderer and self.renderer.frame_count % 60 == 0:
                print(f"⚠️  渲染状态异常: robot={self.robot is not None}, model={self.robot.model if self.robot else None}, mjcam={self.renderer.mjcam}")
    
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
            self.renderer.toggle_simulation()
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
            perf_info = self.renderer.get_performance_info()
            print(f"⚡ FPS: {perf_info['fps']:.1f}")
        elif key == Qt.Key_T:
            # 切换相机跟踪模式
            self.renderer.toggle_camera_tracking()
        elif key == Qt.Key_L:
            # 重新聚焦到机器人
            self.renderer.refocus_camera(self.camera)
            
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
    
    @property
    def is_running(self):
        """获取仿真运行状态"""
        return self.renderer.is_running if self.renderer else False
    
    def toggle_simulation(self):
        """切换仿真状态"""
        if self.renderer:
            self.renderer.toggle_simulation()
    
    def toggle_camera_tracking(self):
        """切换相机跟踪模式"""
        if self.renderer:
            self.renderer.toggle_camera_tracking()
    
    def refocus_camera(self):
        """重新聚焦到机器人当前位置"""
        if self.renderer:
            self.renderer.refocus_camera(self.camera)
    
    def set_robot_model(self, robot_model: RobotModel):
        """设置机器人模型"""
        self.robot = robot_model
        if self.renderer:
            self.renderer.set_robot_model(robot_model)
        
        if self.robot and self.robot.is_loaded():
            # 重置相机跟踪状态
            if self.renderer:
                self.renderer.initial_robot_position = None
                self.renderer.last_camera_update_time = 0
            
            # 适配相机
            self.camera.fit(self.robot.get_model_stats())
            
            # 如果已经初始化，更新渲染资源
            if self.renderer and self.renderer.mjcam is not None:
                print("🔄 更新现有渲染资源...")
                self.renderer.setup_camera(self.camera)
            else:
                print("⚠️  mjcam 未初始化，等待 initializeGL")
    
    def showEvent(self, event):
        """窗口显示事件 - 确保OpenGL资源正确初始化"""
        super().showEvent(event)
        
        # 强制重新初始化OpenGL资源
        if self.robot and self.robot.model and self.renderer and self.renderer.mjcam is None:
            print("🔄 在 showEvent 中重新初始化OpenGL资源")
            self.makeCurrent()
            self.initializeGL()
            self.doneCurrent()
    
    def print_controls(self):
        """打印控制说明"""
        if self.renderer:
            self.renderer.print_camera_controls()


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