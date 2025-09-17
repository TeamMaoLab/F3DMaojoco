"""
ODogExample 主应用入口

提供应用程序的初始化和主窗口管理。
"""

import sys
import os
from typing import Optional
from PySide6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont

try:
    from .viewer_widget import MuJoCoViewerWidget
    from ..core.robot_model import create_test_model
except ImportError:
    # 如果相对导入失败，尝试绝对导入
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from gui.viewer_widget import MuJoCoViewerWidget
    from core.robot_model import create_test_model


class MainApplication(QMainWindow):
    """ODogExample主应用窗口"""
    
    def __init__(self):
        super().__init__()
        
        # 窗口设置
        self.setWindowTitle("ODogExample - 8自由度四足机器狗开发平台")
        self.resize(1200, 1000)  # 调整窗口大小以避免挤压
        self.setMinimumSize(800, 600)  # 设置最小窗口尺寸
        
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
            control_panel = self._create_control_panel()
            main_layout.addWidget(control_panel, stretch=1)
        else:
            # 如果查看器创建失败，显示错误信息
            error_label = QLabel("❌ 无法初始化3D查看器")
            error_label.setAlignment(Qt.AlignCenter)
            error_label.setStyleSheet("font-size: 18px; color: red; padding: 50px;")
            main_layout.addWidget(error_label)
        
        # 状态栏
        self.statusBar().showMessage("准备就绪")
        
        print("🎉 ODogExample主应用启动成功！")
    
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
            # 尝试相对导入，如果失败则使用绝对导入
            try:
                from .control_panels import create_control_panel
            except ImportError:
                import sys
                import os
                sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
                from gui.control_panels import create_control_panel
            
            # 获取机器人模型
            robot = None
            if self.viewer and self.viewer.robot:
                robot = self.viewer.robot
            
            # 创建控制面板
            panel = create_control_panel(robot)
            
            # 连接信号
            if panel:
                panel.jointAngleChanged.connect(self._on_joint_angle_changed)
                panel.allJointsZero.connect(self._on_all_joints_zero)
                panel.allJointsReset.connect(self._on_all_joints_reset)
                panel.poseSaved.connect(self._on_pose_saved)
                
                # 连接相机控制信号到查看器
                if panel.tracking_btn_ref:
                    panel.tracking_btn_ref.toggled.connect(self.viewer.toggle_camera_tracking)
                if panel.refocus_btn_ref:
                    panel.refocus_btn_ref.clicked.connect(self.viewer.refocus_camera)
            
            print("🎛️  控制面板创建成功")
            return panel
            
        except Exception as e:
            print(f"❌ 创建控制面板失败: {e}")
            # 返回简单的错误显示
            error_label = QLabel("❌ 控制面板加载失败")
            error_label.setAlignment(Qt.AlignCenter)
            error_label.setStyleSheet("color: red; padding: 20px;")
            return error_label
    
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
    
    def _create_info_panel(self) -> QWidget:
        """创建信息面板"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(10, 10, 10, 10)
        
        # 标题
        title_label = QLabel("ODogExample")
        title_font = QFont("Arial", 16, QFont.Bold)
        title_label.setFont(title_font)
        title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(title_label)
        
        # 副标题
        subtitle_label = QLabel("8自由度四足机器狗开发平台")
        subtitle_font = QFont("Arial", 10)
        subtitle_label.setFont(subtitle_font)
        subtitle_label.setAlignment(Qt.AlignCenter)
        subtitle_label.setStyleSheet("color: gray;")
        layout.addWidget(subtitle_label)
        
        # 分隔线
        line1 = QLabel("─" * 30)
        line1.setAlignment(Qt.AlignCenter)
        line1.setStyleSheet("color: #ccc;")
        layout.addWidget(line1)
        
        # 开发阶段信息
        stage_label = QLabel("🎯 当前阶段：第一阶段")
        stage_label.setStyleSheet("font-weight: bold; color: #2196F3;")
        layout.addWidget(stage_label)
        
        stage_desc = QLabel("专业级3D模型查看器")
        stage_desc.setStyleSheet("color: #666;")
        stage_desc.setWordWrap(True)
        layout.addWidget(stage_desc)
        
        # 分隔线
        line2 = QLabel("─" * 30)
        line2.setAlignment(Qt.AlignCenter)
        line2.setStyleSheet("color: #ccc;")
        layout.addWidget(line2)
        
        # 功能特性
        features_title = QLabel("✨ 已实现功能")
        features_title.setStyleSheet("font-weight: bold;")
        layout.addWidget(features_title)
        
        features = [
            "✅ MuJoCo模型加载",
            "✅ 通用轨道相机系统",
            "✅ 实时3D渲染",
            "✅ 鼠标交互控制",
            "✅ 自动视角适配",
            "✅ 相机参数优化"
        ]
        
        for feature in features:
            feature_label = QLabel(feature)
            feature_label.setStyleSheet("color: #4CAF50; font-size: 12px;")
            layout.addWidget(feature_label)
        
        # 分隔线
        line3 = QLabel("─" * 30)
        line3.setAlignment(Qt.AlignCenter)
        line3.setStyleSheet("color: #ccc;")
        layout.addWidget(line3)
        
        # 控制说明
        controls_title = QLabel("🎮 控制说明")
        controls_title.setStyleSheet("font-weight: bold;")
        layout.addWidget(controls_title)
        
        controls = [
            "🖱️ 左键拖动：轨道旋转（围绕机器人）",
            "🖱️ 右键拖动：平移模型",
            "   • 上移：前进    下移：后退",
            "   • 左移：左平移  右移：右平移",
            "🖱️ 滚轮：缩放距离",
            "🖱️ Ctrl+滚轮：调整视野",
            "⌨️ 空格：开始/暂停仿真",
            "⌨️ R/F：重置视角",
            "⌨️ T：切换相机跟踪",
            "⌨️ L：重新聚焦机器人",
            "⌨️ 双击：自动适配模型"
        ]
        
        for control in controls:
            control_label = QLabel(control)
            control_label.setStyleSheet("font-size: 11px;")
            layout.addWidget(control_label)
        
        # 添加弹性空间
        layout.addStretch()
        
        return panel
    
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
        
        # 停止仿真
        if self.viewer and self.viewer.is_running:
            self.viewer.toggle_simulation()
        
        # 清理资源
        if self.viewer:
            self.viewer.timer.stop()
        
        print("👋 应用已关闭")
        event.accept()


def main():
    """主应用入口"""
    print("🚀 启动ODogExample...")
    print("=" * 50)
    
    # 创建应用
    app = QApplication(sys.argv)
    
    # 设置应用信息
    app.setApplicationName("ODogExample")
    app.setApplicationVersion("0.1.0")
    app.setOrganizationName("TeamMaoLab")
    
    # 创建主窗口
    main_app = MainApplication()
    
    # 显示窗口
    main_app.show()
    
    # 运行应用
    sys.exit(app.exec())


if __name__ == "__main__":
    main()