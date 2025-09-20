"""
Integration Example: Enhanced Visualization Widget

This example demonstrates how to integrate the enhanced visualization widget
with outline rendering capabilities into your application.
"""

import sys
from pathlib import Path
from PySide6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QHBoxLayout, QPushButton, QComboBox, QLabel, QSlider, QCheckBox
from PySide6.QtCore import Qt

# Import the enhanced visualization widget
from enhanced_visualization_widget import EnhancedVisualizationWidget


class OutlineDemoWindow(QMainWindow):
    """演示轮廓渲染功能的主窗口"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PyVista 轮廓渲染演示")
        self.setGeometry(100, 100, 1200, 800)
        
        # 创建主部件
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        
        # 创建布局
        self.main_layout = QHBoxLayout(self.central_widget)
        
        # 创建可视化部件
        self.viz_widget = EnhancedVisualizationWidget()
        
        # 创建控制面板
        self.control_panel = self._create_control_panel()
        
        # 添加到主布局
        self.main_layout.addWidget(self.viz_widget, 3)  # 3/4 的空间给可视化
        self.main_layout.addWidget(self.control_panel, 1)  # 1/4 的空间给控制面板
        
        # 加载示例模型
        self._load_sample_model()
    
    def _create_control_panel(self):
        """创建控制面板"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # 标题
        title = QLabel("轮廓渲染控制")
        title.setStyleSheet("font-size: 16px; font-weight: bold; padding: 10px;")
        layout.addWidget(title)
        
        # 轮廓技术选择
        layout.addWidget(QLabel("轮廓技术:"))
        self.technique_combo = QComboBox()
        self.technique_combo.addItems([
            "特征边 (Feature Edges)",
            "轮廓剪影 (Contour Silhouette)",
            "深度剪影 (Depth Silhouette)",
            "表面网格 (Surface Net)"
        ])
        self.technique_combo.currentTextChanged.connect(self._on_technique_changed)
        layout.addWidget(self.technique_combo)
        
        # 启用/禁用轮廓
        self.enable_checkbox = QCheckBox("启用轮廓渲染")
        self.enable_checkbox.setChecked(True)
        self.enable_checkbox.stateChanged.connect(self._on_outline_enabled_changed)
        layout.addWidget(self.enable_checkbox)
        
        # 轮廓颜色
        layout.addWidget(QLabel("轮廓颜色:"))
        self.color_combo = QComboBox()
        self.color_combo.addItems(["black", "white", "red", "blue", "green", "yellow", "purple"])
        self.color_combo.currentTextChanged.connect(self._on_color_changed)
        layout.addWidget(self.color_combo)
        
        # 线宽滑块
        layout.addWidget(QLabel("线宽:"))
        self.width_slider = QSlider(Qt.Horizontal)
        self.width_slider.setRange(1, 10)
        self.width_slider.setValue(2)
        self.width_slider.valueChanged.connect(self._on_width_changed)
        layout.addWidget(self.width_slider)
        
        # 透明度滑块
        layout.addWidget(QLabel("透明度:"))
        self.opacity_slider = QSlider(Qt.Horizontal)
        self.opacity_slider.setRange(0, 100)
        self.opacity_slider.setValue(100)
        self.opacity_slider.valueChanged.connect(self._on_opacity_changed)
        layout.addWidget(self.opacity_slider)
        
        # 特征角度滑块（仅对特征边技术有效）
        layout.addWidget(QLabel("特征角度:"))
        self.angle_slider = QSlider(Qt.Horizontal)
        self.angle_slider.setRange(10, 90)
        self.angle_slider.setValue(30)
        self.angle_slider.valueChanged.connect(self._on_angle_changed)
        layout.addWidget(self.angle_slider)
        
        # 轮廓值滑块（仅对轮廓剪影技术有效）
        layout.addWidget(QLabel("轮廓值:"))
        self.contour_slider = QSlider(Qt.Horizontal)
        self.contour_slider.setRange(0, 100)
        self.contour_slider.setValue(50)
        self.contour_slider.valueChanged.connect(self._on_contour_changed)
        layout.addWidget(self.contour_slider)
        
        # 重置按钮
        reset_button = QPushButton("重置设置")
        reset_button.clicked.connect(self._on_reset_clicked)
        layout.addWidget(reset_button)
        
        # 添加弹簧
        layout.addStretch()
        
        # 信息标签
        info_label = QLabel("提示：加载STL文件后可以实时调整轮廓效果")
        info_label.setWordWrap(True)
        info_label.setStyleSheet("color: gray; font-size: 12px; padding: 10px;")
        layout.addWidget(info_label)
        
        return panel
    
    def _load_sample_model(self):
        """加载示例模型"""
        # 创建一个示例球体进行演示
        import pyvista as pv
        mesh = pv.Sphere(radius=1.0, theta_resolution=30, phi_resolution=30)
        
        # 保存临时文件
        temp_file = Path("sample_sphere.stl")
        mesh.save(temp_file)
        
        # 使用增强的加载方法
        self.viz_widget.load_stl_model_with_outline(temp_file, "feature_edges")
        
        # 删除临时文件
        temp_file.unlink()
    
    def _on_technique_changed(self, text):
        """处理轮廓技术改变"""
        technique_map = {
            "特征边 (Feature Edges)": "feature_edges",
            "轮廓剪影 (Contour Silhouette)": "contour_silhouette",
            "深度剪影 (Depth Silhouette)": "depth_silhouette",
            "表面网格 (Surface Net)": "surface_net_outline"
        }
        
        technique = technique_map.get(text, "feature_edges")
        self.viz_widget.set_outline_technique(technique)
    
    def _on_outline_enabled_changed(self, state):
        """处理轮廓启用/禁用"""
        enabled = state == Qt.Checked
        self.viz_widget.toggle_outline(enabled)
    
    def _on_color_changed(self, color):
        """处理颜色改变"""
        self.viz_widget.set_outline_color(color)
    
    def _on_width_changed(self, value):
        """处理线宽改变"""
        self.viz_widget.set_outline_width(value)
    
    def _on_opacity_changed(self, value):
        """处理透明度改变"""
        opacity = value / 100.0
        self.viz_widget.set_outline_opacity(opacity)
    
    def _on_angle_changed(self, value):
        """处理特征角度改变"""
        self.viz_widget.set_feature_angle(value)
    
    def _on_contour_changed(self, value):
        """处理轮廓值改变"""
        contour_value = value / 100.0
        self.viz_widget.set_contour_value(contour_value)
    
    def _on_reset_clicked(self):
        """处理重置按钮点击"""
        self.viz_widget.reset_outline_settings()
        
        # 重置控件状态
        self.technique_combo.setCurrentIndex(0)
        self.enable_checkbox.setChecked(True)
        self.color_combo.setCurrentIndex(0)
        self.width_slider.setValue(2)
        self.opacity_slider.setValue(100)
        self.angle_slider.setValue(30)
        self.contour_slider.setValue(50)


def main():
    """主函数"""
    app = QApplication(sys.argv)
    
    # 创建并显示窗口
    window = OutlineDemoWindow()
    window.show()
    
    sys.exit(app.exec())


if __name__ == "__main__":
    main()