"""
ODogExample GUI模块 - 主应用窗口

提供应用程序的主窗口和界面布局管理。
"""

import sys
import os
from typing import Optional
from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QLabel, QApplication
)
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont

try:
    from ..core.robot_model import create_test_model, RobotModel
    from .viewer_widget import MuJoCoViewerWidget
    from .tabbed_control_panel import create_tabbed_control_panel
    from .app_signals import SignalManager
except ImportError:
    # 如果相对导入失败，尝试绝对导入
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from core.robot_model import create_test_model, RobotModel
    from gui.viewer_widget import MuJoCoViewerWidget
    from gui.tabbed_control_panel import create_tabbed_control_panel
    from gui.app_signals import SignalManager


class MainApplication(QMainWindow):
    """ODogExample主应用窗口"""
    
    def __init__(self):
        super().__init__()
        
        # 组件引用
        self.viewer = None
        self.control_panel = None
        self.signal_manager = SignalManager()
        
        # 窗口设置
        self._setup_window()
        
        # 创建主界面
        self._create_main_interface()
        
        # 连接信号
        self._connect_signals()
        
        # 状态栏
        self.statusBar().showMessage("准备就绪")
        
        print("🎉 ODogExample主应用启动成功！")
    
    def _setup_window(self):
        """设置窗口属性"""
        self.setWindowTitle("ODogExample - 8自由度四足机器狗开发平台")
        self.resize(1200, 1000)  # 调整窗口大小以避免挤压
        self.setMinimumSize(800, 600)  # 设置最小窗口尺寸
    
    def _create_main_interface(self):
        """创建主界面"""
        # 创建中央窗口
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 创建主布局
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(10, 10, 10, 10)
        
        # 创建3D查看器
        self.viewer = self._create_viewer()
        if self.viewer:
            main_layout.addWidget(self.viewer, stretch=2)
            
            # 创建控制面板
            self.control_panel = self._create_control_panel()
            if self.control_panel:
                main_layout.addWidget(self.control_panel, stretch=1)
            else:
                # 显示控制面板创建失败的错误信息
                error_widget = self._create_error_widget("❌ 控制面板加载失败")
                main_layout.addWidget(error_widget, stretch=1)
        else:
            # 显示查看器创建失败的错误信息
            error_widget = self._create_error_widget("❌ 无法初始化3D查看器")
            main_layout.addWidget(error_widget)
    
    def _create_viewer(self) -> Optional[MuJoCoViewerWidget]:
        """创建3D查看器"""
        try:
            # 创建测试机器人模型
            robot = create_test_model()
            if robot and robot.is_loaded():
                viewer = MuJoCoViewerWidget(robot)
                viewer.print_controls()
                return viewer
            else:
                print("❌ 无法创建机器人模型")
                return None
        except Exception as e:
            print(f"❌ 创建3D查看器失败: {e}")
            return None
    
    def _create_control_panel(self) -> Optional[QWidget]:
        """创建控制面板"""
        try:
            # 获取机器人模型
            robot = None
            if self.viewer and self.viewer.robot:
                robot = self.viewer.robot
            
            # 创建Tab页控制面板
            panel = create_tabbed_control_panel(robot)
            
            if panel:
                # 设置机器人模型到控制面板
                if robot:
                    panel.set_robot_model(robot)
                
                return panel
            else:
                print("❌ 控制面板创建失败")
                return None
                
        except Exception as e:
            print(f"❌ 创建控制面板失败: {e}")
            return None
    
    def _create_error_widget(self, error_message: str) -> QWidget:
        """创建错误显示组件"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        error_label = QLabel(error_message)
        error_label.setAlignment(Qt.AlignCenter)
        error_label.setStyleSheet("font-size: 18px; color: red; padding: 50px;")
        layout.addWidget(error_label)
        
        return widget
    
    def _connect_signals(self):
        """连接所有信号"""
        if not (self.viewer and self.control_panel):
            print("⚠️  组件不完整，跳过信号连接")
            return
        
        try:
            # 连接关节控制信号
            self.signal_manager.connect_joint_control_signals(self.control_panel, self.viewer)
            
            # 连接相机控制信号
            self.signal_manager.connect_camera_control_signals(self.control_panel, self.viewer)
            
            # 连接姿态信号
            self.signal_manager.connect_pose_signals(self.control_panel)
            
            # 连接查看器信号
            self.signal_manager.connect_viewer_signals(self.viewer)
            
            # 打印信号连接摘要
            signal_summary = self.signal_manager.get_signal_summary()
            print(f"🔗 信号连接完成: {signal_summary['total_connections']} 个连接")
            
        except Exception as e:
            print(f"❌ 信号连接失败: {e}")
    
    def _on_joint_angle_changed(self, joint_name: str, angle: float):
        """关节角度改变处理"""
        # 触发3D视图重新渲染以显示关节变化
        if self.viewer:
            self.viewer.update()
    
    def _on_all_joints_zero(self):
        """所有关节归零处理"""
        print("🔄 应用层响应：所有关节已归零")
    
    def _on_all_joints_reset(self):
        """所有关节重置处理"""
        print("🔙 应用层响应：所有关节已重置")
    
    def _on_pose_saved(self, pose_name: str, pose_data: dict):
        """姿态保存处理"""
        print(f"💾 应用层响应：姿态 {pose_name} 已保存")
        # 更新状态栏
        self.statusBar().showMessage(f"姿态已保存: {pose_name}", 3000)
    
    def keyPressEvent(self, event):
        """键盘事件处理"""
        if self.viewer:
            # 将键盘事件传递给查看器
            self.viewer.keyPressEvent(event)
        
        # 应用级别的快捷键
        if event.key() == Qt.Key_Escape:
            self.close()
        elif event.key() == Qt.Key_F11:
            # 全屏切换
            if self.isFullScreen():
                self.showNormal()
            else:
                self.showFullScreen()
    
    def closeEvent(self, event):
        """关闭事件处理"""
        print("🔄 正在关闭应用...")
        
        # 断开所有信号连接
        if self.signal_manager:
            self.signal_manager.disconnect_all()
        
        # 停止仿真
        if self.viewer and self.viewer.is_running:
            self.viewer.toggle_simulation()
        
        # 清理资源
        if self.viewer:
            self.viewer.timer.stop()
        
        print("👋 应用已关闭")
        event.accept()
    
    def get_application_info(self) -> dict:
        """获取应用信息"""
        return {
            'title': self.windowTitle(),
            'size': (self.width(), self.height()),
            'viewer_loaded': self.viewer is not None,
            'control_panel_loaded': self.control_panel is not None,
            'signal_connections': self.signal_manager.get_signal_summary() if self.signal_manager else {}
        }


def main():
    """主程序入口 - 用于直接运行app_main.py"""
    import sys
    from PySide6.QtWidgets import QApplication
    
    app = QApplication(sys.argv)
    
    # 创建主应用
    main_app = MainApplication()
    main_app.show()
    
    # 打印应用信息
    app_info = main_app.get_application_info()
    print(f"📱 应用信息:")
    print(f"  标题: {app_info['title']}")
    print(f"  尺寸: {app_info['size'][0]}x{app_info['size'][1]}")
    print(f"  查看器: {'已加载' if app_info['viewer_loaded'] else '未加载'}")
    print(f"  控制面板: {'已加载' if app_info['control_panel_loaded'] else '未加载'}")
    print(f"  信号连接: {app_info['signal_connections'].get('total_connections', 0)} 个")
    
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())