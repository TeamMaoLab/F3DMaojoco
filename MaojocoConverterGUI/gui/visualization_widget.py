"""
PyVista可视化组件

使用pyvistaqt集成PyVista 3D渲染引擎到PySide6界面中，提供STL模型加载和显示功能。
"""

from typing import Optional, List, Any
from pathlib import Path

import pyvista as pv
from pyvistaqt import QtInteractor
from PySide6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel
from PySide6.QtCore import Signal, Qt
from PySide6.QtGui import QFont, QColor

from utils.logger import logger


class VisualizationWidget(QWidget):
    """3D可视化组件
    
    使用pyvistaqt提供PyVista渲染器的Qt集成界面，支持STL文件加载和交互式查看。
    """
    
    # 信号定义
    model_loaded = Signal(str)  # 模型加载完成信号
    view_reset = Signal()  # 视图重置信号
    error_occurred = Signal(str)  # 错误信号
    
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        """初始化可视化组件
        
        Args:
            parent: 父窗口部件
        """
        super().__init__(parent)
        self._plotter: Optional[QtInteractor] = None
        self._current_models: List[Any] = []
        self._initial_text_actor = None
        self._initial_text_label = None
        self._initial_text_timer = None
        self._setup_pyvista()
        
    def _setup_ui(self) -> None:
        """设置UI界面"""
        logger.info("初始化可视化组件UI")
        
        # 创建主布局
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        
        # 添加标题栏和控制按钮的占位符
        # PyVista Qt interactor将在这里创建
        main_layout.addStretch()
        
        logger.success("可视化组件UI初始化完成")
        
    def _setup_pyvista(self) -> None:
        """设置PyVista渲染器"""
        logger.info("初始化PyVista渲染器")
        
        try:
            # 清理现有布局和部件
            if self.layout():
                # 删除布局中的所有部件
                while self.layout().count():
                    item = self.layout().takeAt(0)
                    if item.widget():
                        item.widget().deleteLater()
                # 删除现有布局
                QWidget().setLayout(self.layout())
            
            # 创建新的布局
            layout = QVBoxLayout(self)
            layout.setContentsMargins(0, 0, 0, 0)
            layout.setSpacing(0)
            
            # 创建pyvistaqt的QtInteractor，设置合适的大小
            self._plotter = QtInteractor(self)
            self._plotter.renderer.set_background("white")
            
            # 设置plotter的大小策略，让它占据更多空间
            from PySide6.QtWidgets import QSizePolicy
            size_policy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            self._plotter.setSizePolicy(size_policy)
            
            # 设置渲染参数以改善视觉效果
            try:
                self._plotter.renderer.enable_anti_aliasing()  # 启用抗锯齿
            except AttributeError:
                logger.warning("抗锯齿功能不可用，跳过")
            self._plotter.renderer.set_background("white")  # 白色背景
            
            # 添加坐标轴，但不显示网格
            self._plotter.add_axes()
            # self._plotter.show_grid()  # 注释掉网格显示
            
            # 设置相机位置
            self._plotter.camera_position = [(2, 2, 2), (0, 0, 0), (0, 0, 1)]
            
            # 设置光照参数
            try:
                self._plotter.enable_light_kit()  # 启用光照套件
            except AttributeError:
                logger.warning("光照套件功能不可用，跳过")
            
            # 创建覆盖的提示文字标签
            self._create_overlay_text()
            
            # 将plotter添加到布局中，设置拉伸因子
            layout.addWidget(self._plotter, 1)  # 1 表示拉伸因子
                
            logger.success("PyVista渲染器初始化完成")
            
        except Exception as e:
            logger.error(f"PyVista渲染器初始化失败: {e}")
            self.error_occurred.emit(f"渲染器初始化失败: {e}")
            
    def _create_overlay_text(self) -> None:
        """创建覆盖在3D窗口上的提示文字"""
        try:
            # 创建一个透明的覆盖标签
            self._initial_text_label = QLabel("点击右侧加载目录", self)
            self._initial_text_label.setAlignment(Qt.AlignCenter)
            self._initial_text_label.setWordWrap(True)  # 允许文字换行
            self._initial_text_label.setStyleSheet("""
                QLabel {
                    background-color: rgba(255, 255, 200, 240);
                    color: #222222;
                    font-size: 32px;
                    font-weight: bold;
                    border: 3px solid #FF6B35;
                    border-radius: 20px;
                    padding: 30px 35px;
                    min-width: 500px;
                    min-height: 90px;
                }
            """)
            
            # 设置更大的字体，使用系统可用的字体
            font = QFont("Arial", 26, QFont.Bold)  # 使用Arial字体，增大字号
            # 尝试使用更好的中文字体
            available_families = font.families()
            chinese_fonts = ["PingFang SC", "Hiragino Sans GB", "Microsoft YaHei", "STHeiti", "SimHei"]
            
            for chinese_font in chinese_fonts:
                if chinese_font in available_families:
                    font.setFamily(chinese_font)
                    break
            
            self._initial_text_label.setFont(font)
            
            # 将标签提升到最上层
            self._initial_text_label.raise_()
            
            # 初始时隐藏，等布局完成后再显示
            self._initial_text_label.setVisible(False)
            
            # 延迟显示以确保布局完成
            from PySide6.QtCore import QTimer
            self._initial_text_timer = QTimer()
            self._initial_text_timer.setSingleShot(True)
            self._initial_text_timer.timeout.connect(self._position_and_show_text)
            self._initial_text_timer.start(200)  # 增加延迟时间
            
            logger.info("覆盖文字标签创建完成")
            
        except Exception as e:
            logger.error(f"创建覆盖文字失败: {e}")
    
    def _position_and_show_text(self) -> None:
        """定位并显示提示文字"""
        if self._initial_text_label and self._plotter:
            try:
                # 获取3D窗口的几何位置
                plotter_geometry = self._plotter.geometry()
                
                # 计算更大的标签尺寸以确保文字不被裁减
                label_width = 550
                label_height = 120
                
                # 计算居中位置
                x = plotter_geometry.x() + (plotter_geometry.width() - label_width) // 2
                y = plotter_geometry.y() + (plotter_geometry.height() - label_height) // 2
                
                # 确保不会超出边界
                if x < 10:
                    x = 10
                if y < 10:
                    y = 10
                
                # 设置标签位置和大小
                self._initial_text_label.setGeometry(x, y, label_width, label_height)
                self._initial_text_label.setVisible(True)
                
                logger.info(f"提示文字定位完成: 位置({x}, {y}), 尺寸({label_width}x{label_height})")
                
            except Exception as e:
                logger.error(f"文字定位失败: {e}")
                # 如果定位失败，使用安全的默认位置
                self._initial_text_label.setGeometry(20, 20, 550, 120)
                self._initial_text_label.setVisible(True)
    
    def _show_initial_text(self) -> None:
        """显示初始化提示文字"""
        if self._initial_text_label:
            self._initial_text_label.setVisible(True)
            self._initial_text_label.raise_()
    
    def _hide_initial_text(self) -> None:
        """隐藏初始化提示文字"""
        if self._initial_text_label:
            self._initial_text_label.setVisible(False)
            # 停止任何未执行的定时器
            if hasattr(self, '_initial_text_timer') and self._initial_text_timer:
                self._initial_text_timer.stop()
    
    def _add_initial_text(self) -> None:
        """添加初始化提示文字（保留兼容性）"""
        self._show_initial_text()
            
    def _remove_initial_text(self) -> None:
        """移除初始化提示文字（保留兼容性）"""
        self._hide_initial_text()
            
    def load_stl_model(self, file_path: Path) -> bool:
        """加载STL模型
        
        Args:
            file_path: STL文件路径
            
        Returns:
            bool: 是否加载成功
        """
        logger.info(f"开始加载STL模型: {file_path}")
        
        if not self._plotter:
            logger.error("PyVista渲染器未初始化")
            return False
            
        try:
            # 加载STL文件
            mesh = pv.read(file_path)
            
            # 清除之前的模型和初始化文字
            self._plotter.clear()
            self._current_models.clear()
            self._remove_initial_text()
            
            # 添加新模型 - 不显示三角形边线
            actor = self._plotter.add_mesh(
                mesh,
                color="#4A90E2",  # 专业的蓝色
                opacity=1.0,
                show_edges=False,  # 不显示三角形边线
                smooth_shading=True,  # 启用平滑着色
                lighting=True,  # 启用光照效果
                specular=0.3,  # 设置高光反射
                specular_power=20  # 设置高光强度
            )
            
            self._current_models.append(actor)
            
            # 添加清晰的轮廓线以增强模型边界定义
            try:
                # 提取特征边创建轮廓线
                outline_edges = mesh.extract_feature_edges(
                    feature_angle=30.0,
                    boundary_edges=True,
                    non_manifold_edges=True,
                    manifold_edges=False
                )
                
                if outline_edges and outline_edges.n_points > 0:
                    # 添加轮廓线，使用细线宽和适中的透明度
                    outline_actor = self._plotter.add_mesh(
                        outline_edges,
                        color="black",
                        line_width=1.5,
                        opacity=0.7,
                        render_lines_as_tubes=True
                    )
                    self._current_models.append(outline_actor)
                    logger.debug(f"为模型添加轮廓线: {file_path.name}")
                    
            except Exception as outline_error:
                logger.debug(f"轮廓线添加失败（不影响主模型）: {outline_error}")
            
            # 重置相机视图
            self._plotter.reset_camera()
            
            # 更新显示
            self._plotter.render()
            
            logger.success(f"STL模型加载成功: {file_path}")
            self.model_loaded.emit(str(file_path))
            return True
            
        except Exception as e:
            logger.error(f"STL模型加载失败: {file_path}, 错误: {e}")
            self.error_occurred.emit(f"模型加载失败: {e}")
            return False
            
    def load_multiple_stl_models(self, file_paths: List[Path]) -> bool:
        """加载多个STL模型
        
        Args:
            file_paths: STL文件路径列表
            
        Returns:
            bool: 是否全部加载成功
        """
        logger.info(f"开始加载多个STL模型，数量: {len(file_paths)}")
        
        if not self._plotter:
            logger.error("PyVista渲染器未初始化")
            return False
            
        try:
            # 清除之前的模型和初始化文字
            self._plotter.clear()
            self._current_models.clear()
            self._remove_initial_text()
            
            success_count = 0
            
            for i, file_path in enumerate(file_paths):
                try:
                    # 加载STL文件
                    mesh = pv.read(file_path)
                    
                    # 为不同模型设置专业的颜色方案
                    colors = ["#4A90E2", "#7ED321", "#F5A623", "#BD10E0", "#50E3C2", "#B8E986"]
                    color = colors[i % len(colors)]
                    
                    # 添加模型 - 不显示三角形边线
                    actor = self._plotter.add_mesh(
                        mesh,
                        color=color,
                        opacity=1.0,
                        show_edges=False,  # 不显示三角形边线
                        smooth_shading=True,  # 启用平滑着色
                        lighting=True,  # 启用光照效果
                        specular=0.3,  # 设置高光反射
                        specular_power=20  # 设置高光强度
                    )
                    
                    self._current_models.append(actor)
                    
                    # 为每个模型添加轮廓线以增强边界定义
                    try:
                        outline_edges = mesh.extract_feature_edges(
                            feature_angle=30.0,
                            boundary_edges=True,
                            non_manifold_edges=True,
                            manifold_edges=False
                        )
                        
                        if outline_edges and outline_edges.n_points > 0:
                            outline_actor = self._plotter.add_mesh(
                                outline_edges,
                                color="black",
                                line_width=1.0,
                                opacity=0.6,
                                render_lines_as_tubes=True
                            )
                            self._current_models.append(outline_actor)
                            logger.debug(f"为模型 {file_path.name} 添加轮廓线")
                            
                    except Exception as outline_error:
                        logger.debug(f"轮廓线添加失败（不影响主模型）: {outline_error}")
                    
                    success_count += 1
                    
                except Exception as e:
                    logger.warning(f"模型加载失败: {file_path}, 错误: {e}")
                    continue
            
            if success_count > 0:
                # 重置相机视图
                self._plotter.reset_camera()
                
                # 更新显示
                self._plotter.render()
                
                logger.success(f"成功加载 {success_count}/{len(file_paths)} 个STL模型")
                return True
            else:
                logger.error("所有STL模型加载失败")
                return False
                
        except Exception as e:
            logger.error(f"批量加载STL模型失败: {e}")
            self.error_occurred.emit(f"批量加载失败: {e}")
            return False
            
    def reset_view(self) -> None:
        """重置视图"""
        logger.info("重置3D视图")
        
        if self._plotter:
            self._plotter.reset_camera()
            self._plotter.render()
            self.view_reset.emit()
            
    def fit_to_screen(self) -> None:
        """适配屏幕"""
        logger.info("适配3D视图到屏幕")
        
        if self._plotter:
            self._plotter.reset_camera()
            self._plotter.render()
            
    def set_model_opacity(self, model_index: int, opacity: float) -> None:
        """设置模型透明度
        
        Args:
            model_index: 模型索引
            opacity: 透明度 (0.0-1.0)
        """
        if 0 <= model_index < len(self._current_models) and self._plotter:
            actor = self._current_models[model_index]
            actor.GetProperty().SetOpacity(opacity)
            self._plotter.render()
            
    def set_model_color(self, model_index: int, color: str) -> None:
        """设置模型颜色
        
        Args:
            model_index: 模型索引
            color: 颜色名称
        """
        if 0 <= model_index < len(self._current_models) and self._plotter:
            actor = self._current_models[model_index]
            try:
                actor.GetProperty().SetColor(pv.Color(color).float_rgb)
            except:
                # 如果颜色名称无效，使用默认颜色
                pass
            self._plotter.render()
            
    def clear_all_models(self) -> None:
        """清除所有模型"""
        logger.info("清除所有3D模型")
        
        if self._plotter:
            self._plotter.clear()
            self._current_models.clear()
            # 重新显示初始化文字
            self._add_initial_text()
            self._plotter.render()
            
    def get_plotter(self) -> Optional[QtInteractor]:
        """获取PyVista绘图器实例
        
        Returns:
            Optional[QtInteractor]: PyVista QtInteractor实例
        """
        return self._plotter
        
    def closeEvent(self, event) -> None:
        """窗口关闭事件处理"""
        logger.info("可视化组件正在关闭")
        
        if self._plotter:
            self._plotter.close()
            
        super().closeEvent(event)