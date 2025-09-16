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
        self.resize(1200, 800)
        
        # 创建中央窗口
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 创建主布局
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(10, 10, 10, 10)
        
        # 创建3D查看器
        self.viewer = self._create_viewer()
        if self.viewer:
            main_layout.addWidget(self.viewer, stretch=3)
            
            # 创建信息面板
            info_panel = self._create_info_panel()
            main_layout.addWidget(info_panel, stretch=1)
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
            "🖱️ 左键拖动：轨道旋转",
            "🖱️ 右键拖动：平移",
            "🖱️ 滚轮：缩放",
            "⌨️ 空格：开始/暂停",
            "⌨️ R/F：重置视角",
            "⌨️ 双击：自动适配"
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