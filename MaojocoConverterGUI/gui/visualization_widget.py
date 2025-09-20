"""
PyVista可视化组件

使用pyvistaqt集成PyVista 3D渲染引擎到PySide6界面中，提供STL模型加载和显示功能。
"""

from typing import Optional, List, Any
from pathlib import Path

import pyvista as pv
from pyvistaqt import QtInteractor
from PySide6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton
from PySide6.QtCore import Signal, Qt

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
            
            # 添加坐标轴和网格
            self._plotter.add_axes()
            self._plotter.show_grid()
            
            # 设置相机位置
            self._plotter.camera_position = [(2, 2, 2), (0, 0, 0), (0, 0, 1)]
            
            # 将plotter添加到布局中，设置拉伸因子
            layout.addWidget(self._plotter, 1)  # 1 表示拉伸因子
                
            logger.success("PyVista渲染器初始化完成")
            
        except Exception as e:
            logger.error(f"PyVista渲染器初始化失败: {e}")
            self.error_occurred.emit(f"渲染器初始化失败: {e}")
            
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
            
            # 清除之前的模型
            self._plotter.clear()
            self._current_models.clear()
            
            # 添加新模型
            actor = self._plotter.add_mesh(
                mesh,
                color="lightblue",
                opacity=1.0,
                show_edges=True
            )
            
            self._current_models.append(actor)
            
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
            # 清除之前的模型
            self._plotter.clear()
            self._current_models.clear()
            
            success_count = 0
            
            for i, file_path in enumerate(file_paths):
                try:
                    # 加载STL文件
                    mesh = pv.read(file_path)
                    
                    # 为不同模型设置不同颜色
                    colors = ["lightblue", "lightgreen", "lightcoral", "lightyellow", "lightpink"]
                    color = colors[i % len(colors)]
                    
                    # 添加模型
                    actor = self._plotter.add_mesh(
                        mesh,
                        color=color,
                        opacity=1.0,
                        show_edges=True
                    )
                    
                    self._current_models.append(actor)
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