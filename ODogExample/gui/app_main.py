"""
ODogExample GUI模块 - 主应用窗口

提供应用程序的主窗口和界面布局管理。
"""

import sys
from typing import Optional
from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QLabel, QApplication, QMessageBox
)
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont

from ..core.robot_model import create_test_model, RobotModel
from .viewer_widget import MuJoCoViewerWidget
from .tabbed_control_panel import create_tabbed_control_panel
from .app_signals import SignalManager
from .pose_manager import get_pose_manager


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
            self.control_panel.cameraTrackingToggled.connect(self.viewer.toggle_camera_tracking)
            self.control_panel.cameraRefocus.connect(self.viewer.refocus_camera)
            
            # 连接姿态信号
            self.signal_manager.connect_pose_signals(self.control_panel)
            
            # 连接动作编辑器播放信号
            self._connect_motion_editor_signals()
            
            # 连接查看器信号
            self.signal_manager.connect_viewer_signals(self.viewer)
            
            # 打印信号连接摘要
            signal_summary = self.signal_manager.get_signal_summary()
            print(f"🔗 信号连接完成: {signal_summary['total_connections']} 个连接")
            
        except Exception as e:
            print(f"❌ 信号连接失败: {e}")
    
    def _connect_motion_editor_signals(self):
        """连接动作编辑器的播放信号"""
        try:
            # 检查是否有动作编辑器
            motion_editor = self.control_panel.motion_editor
            if motion_editor:
                print(f"🔍 找到动作编辑器: {motion_editor}")
                
                # 连接姿态应用信号
                motion_editor.applyPoseRequest.connect(self._on_apply_pose_request)
                print(f"🔗 已连接applyPoseRequest信号到_on_apply_pose_request方法")
                
                print("🔗 动作编辑器播放信号连接完成")
            else:
                print("⚠️  控制面板没有motion_editor属性")
        except Exception as e:
            print(f"⚠️  动作编辑器信号连接失败: {e}")
            import traceback
            traceback.print_exc()
    
    def _on_apply_pose_request(self, pose_name: str):
        """处理姿态应用请求"""
        try:
            print(f"🎯 应用姿态请求: {pose_name}")
            
            if not (self.viewer and self.viewer.robot):
                print("⚠️  机器人模型不可用")
                return
            
            # 严格从姿态管理器加载姿态数据
            try:
                pose_manager = get_pose_manager()
                joint_angles = pose_manager.load_pose(pose_name)
                
                if joint_angles:
                    print(f"📁 从姿态管理器加载姿态: {pose_name}")
                else:
                    print(f"⚠️ 姿态管理器中没有找到姿态: {pose_name}")
                    print(f"📋 可用姿态列表: {pose_manager.get_pose_names()}")
                    QMessageBox.warning(self, "姿态未找到", 
                                      f"姿态 '{pose_name}' 在姿态管理器中不存在！\n\n"
                                      f"可用姿态:\n{chr(10).join(pose_manager.get_pose_names())}")
                    return
                        
            except Exception as e:
                print(f"❌ 加载姿态失败: {e}")
                QMessageBox.critical(self, "加载失败", f"加载姿态 '{pose_name}' 失败: {e}")
                return
            
            if joint_angles:
                print(f"🤖 应用姿态 {pose_name} 到机器人: {len(joint_angles)} 个关节")
                print(f"📊 关节角度数据:")
                for joint_name, angle in joint_angles.items():
                    print(f"  - {joint_name}: {angle:.3f} rad ({angle*180/3.14159:.1f}°)")
                
                # 使用控制面板的姿态应用功能
                print("🔧 使用控制面板的set_pose方法")
                self.control_panel.set_pose(joint_angles)
                print("✅ 控制面板set_pose调用完成")
                
                # 更新状态栏
                self.statusBar().showMessage(f"已应用姿态: {pose_name}", 2000)
            else:
                print(f"⚠️  未找到姿态: {pose_name}")
                print(f"📋 可用姿态: {self._get_available_poses()}")
                
        except Exception as e:
            print(f"❌ 应用姿态失败: {e}")
            import traceback
            traceback.print_exc()
    
    def _get_fallback_pose(self, pose_name: str) -> dict:
        """获取后备姿态数据"""
        fallback_poses = {
            "站立姿态": {
                "xuan_zhuan_1": 0.0, "xuan_zhuan_2": 0.0, "xuan_zhuan_3": 0.0, "xuan_zhuan_4": 0.0,
                "xuan_zhuan_5": 0.0, "xuan_zhuan_6": 0.0, "xuan_zhuan_7": 0.0, "xuan_zhuan_8": 0.0
            },
            "趴下姿态": {
                "xuan_zhuan_1": 1.57, "xuan_zhuan_2": -1.57, "xuan_zhuan_3": 1.57, "xuan_zhuan_4": -1.57,
                "xuan_zhuan_5": 1.57, "xuan_zhuan_6": -1.57, "xuan_zhuan_7": 1.57, "xuan_zhuan_8": -1.57
            },
            "坐下姿态": {
                "xuan_zhuan_1": 0.8, "xuan_zhuan_2": -1.57, "xuan_zhuan_3": 0.8, "xuan_zhuan_4": -1.57,
                "xuan_zhuan_5": 0.8, "xuan_zhuan_6": -1.57, "xuan_zhuan_7": 0.8, "xuan_zhuan_8": -1.57
            },
            "行走姿态1": {
                "xuan_zhuan_1": 0.5, "xuan_zhuan_2": -1.0, "xuan_zhuan_3": -0.5, "xuan_zhuan_4": 1.0,
                "xuan_zhuan_5": 0.5, "xuan_zhuan_6": -1.0, "xuan_zhuan_7": -0.5, "xuan_zhuan_8": 1.0
            },
            "行走姿态2": {
                "xuan_zhuan_1": -0.5, "xuan_zhuan_2": 1.0, "xuan_zhuan_3": 0.5, "xuan_zhuan_4": -1.0,
                "xuan_zhuan_5": -0.5, "xuan_zhuan_6": 1.0, "xuan_zhuan_7": 0.5, "xuan_zhuan_8": -1.0
            },
            "趴下-抬头": {
                "xuan_zhuan_1": 1.0, "xuan_zhuan_2": -0.3, "xuan_zhuan_3": 1.0, "xuan_zhuan_4": -0.3,
                "xuan_zhuan_5": 1.0, "xuan_zhuan_6": -0.3, "xuan_zhuan_7": 1.0, "xuan_zhuan_8": -0.3
            }
        }
        return fallback_poses.get(pose_name, {})
    
    def _get_available_poses(self) -> list:
        """获取可用姿态列表"""
        try:
            pose_manager = get_pose_manager()
            return pose_manager.get_pose_names()
        except Exception:
            return list(self._get_fallback_pose("站立姿态").keys())
    
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