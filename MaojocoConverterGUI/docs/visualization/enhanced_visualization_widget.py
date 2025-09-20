"""
Enhanced PyVista Visualization Widget with Outline Rendering

This module extends the existing VisualizationWidget with advanced outline rendering
techniques for better STL model visualization.
"""

from typing import Optional, List, Any, Dict
from pathlib import Path
import pyvista as pv
from pyvistaqt import QtInteractor

from gui.visualization_widget import VisualizationWidget
from utils.logger import logger


class EnhancedVisualizationWidget(VisualizationWidget):
    """Enhanced 3D可视化组件 with outline rendering capabilities
    
    Extends the base VisualizationWidget with advanced outline and silhouette rendering
    techniques for improved STL model visualization.
    """
    
    def __init__(self, parent: Optional[Any] = None) -> None:
        """初始化增强可视化组件
        
        Args:
            parent: 父窗口部件
        """
        super().__init__(parent)
        self._outline_actors: List[Any] = []
        self._current_outline_technique = "feature_edges"
        self._outline_settings = {
            'enabled': True,
            'technique': 'feature_edges',
            'color': 'black',
            'line_width': 2.0,
            'opacity': 1.0,
            'feature_angle': 30.0,
            'contour_value': 0.5,
            'smooth_outline': True
        }
        
    def load_stl_model_with_outline(self, file_path: Path, outline_technique: str = "feature_edges") -> bool:
        """加载STL模型并应用轮廓效果
        
        Args:
            file_path: STL文件路径
            outline_technique: 轮廓技术 ('feature_edges', 'contour_silhouette', 'depth_silhouette', 'surface_net_outline')
            
        Returns:
            bool: 是否加载成功
        """
        logger.info(f"加载STL模型并应用轮廓效果: {file_path}")
        
        if not self._plotter:
            logger.error("PyVista渲染器未初始化")
            return False
            
        try:
            # 加载STL文件
            mesh = pv.read(file_path)
            
            # 清除之前的模型和轮廓
            self._plotter.clear()
            self._current_models.clear()
            self._outline_actors.clear()
            self._remove_initial_text()
            
            # 设置轮廓技术
            self._current_outline_technique = outline_technique
            
            # 添加基础模型
            base_actor = self._plotter.add_mesh(
                mesh,
                color="#4A90E2",
                opacity=1.0,
                show_edges=False,  # 不显示三角形边线
                smooth_shading=True,
                lighting=True,
                specular=0.3,
                specular_power=20
            )
            self._current_models.append(base_actor)
            
            # 添加轮廓效果
            if self._outline_settings['enabled']:
                outline_success = self._add_outline_to_mesh(mesh, outline_technique)
                if not outline_success:
                    logger.warning(f"轮廓效果添加失败，使用基础模型显示")
            
            # 重置相机视图
            self._plotter.reset_camera()
            
            # 更新显示
            self._plotter.render()
            
            logger.success(f"STL模型加载成功（含轮廓效果）: {file_path}")
            self.model_loaded.emit(str(file_path))
            return True
            
        except Exception as e:
            logger.error(f"STL模型加载失败: {file_path}, 错误: {e}")
            self.error_occurred.emit(f"模型加载失败: {e}")
            return False
    
    def _add_outline_to_mesh(self, mesh: pv.PolyData, technique: str) -> bool:
        """为网格添加轮廓效果
        
        Args:
            mesh: 输入网格
            technique: 轮廓技术
            
        Returns:
            bool: 是否成功添加轮廓
        """
        try:
            outline_mesh = None
            
            if technique == "feature_edges":
                outline_mesh = self._create_feature_edges_outline(mesh)
            elif technique == "contour_silhouette":
                outline_mesh = self._create_contour_silhouette(mesh)
            elif technique == "depth_silhouette":
                outline_mesh = self._create_depth_silhouette(mesh)
            elif technique == "surface_net_outline":
                outline_mesh = self._create_surface_net_outline(mesh)
            
            if outline_mesh is not None and outline_mesh.n_points > 0:
                outline_actor = self._plotter.add_mesh(
                    outline_mesh,
                    color=self._outline_settings['color'],
                    line_width=self._outline_settings['line_width'],
                    opacity=self._outline_settings['opacity'],
                    render_lines_as_tubes=True,  # 渲染为管状以获得更好的视觉效果
                    smooth_shading=True
                )
                self._outline_actors.append(outline_actor)
                return True
            else:
                logger.warning(f"生成的轮廓网格为空: {technique}")
                return False
                
        except Exception as e:
            logger.error(f"添加轮廓效果失败: {e}")
            return False
    
    def _create_feature_edges_outline(self, mesh: pv.PolyData) -> Optional[pv.PolyData]:
        """创建特征边轮廓
        
        提取模型的特征边、边界边和非流形边，形成轮廓线。
        
        Args:
            mesh: 输入网格
            
        Returns:
            轮廓线网格
        """
        try:
            # 提取特征边
            edges = mesh.extract_feature_edges(
                feature_angle=self._outline_settings['feature_angle'],
                boundary_edges=True,  # 包含边界边
                non_manifold_edges=True,  # 包含非流形边
                manifold_edges=False  # 不包含流形边
            )
            
            # 应用平滑处理
            if self._outline_settings['smooth_outline'] and edges.n_points > 0:
                edges = edges.smooth(n_iter=5, relaxation_factor=0.1)
            
            return edges
            
        except Exception as e:
            logger.error(f"创建特征边轮廓失败: {e}")
            return None
    
    def _create_contour_silhouette(self, mesh: pv.PolyData) -> Optional[pv.PolyData]:
        """创建基于轮廓的剪影
        
        使用标量场轮廓提取技术创建剪影效果。
        
        Args:
            mesh: 输入网格
            
        Returns:
            剪影网格
        """
        try:
            # 计算法线
            mesh_with_normals = mesh.compute_normals(point_normals=True, cell_normals=True)
            
            # 基于视角创建标量场
            # 使用当前相机位置或默认位置
            if hasattr(self._plotter, 'camera_position'):
                camera_pos = np.array(self._plotter.camera_position[0])
            else:
                camera_pos = np.array([2, 2, 2])
            
            mesh_center = mesh_with_normals.center
            view_direction = camera_pos - mesh_center
            view_direction = view_direction / np.linalg.norm(view_direction)
            
            # 计算法线与视角方向的点积
            normals = mesh_with_normals['Normals']
            scalars = np.abs(np.dot(normals, view_direction))
            
            # 添加标量到网格
            mesh_with_normals['view_scalars'] = scalars
            
            # 提取轮廓
            contour = mesh_with_normals.contour(
                isosurfaces=[self._outline_settings['contour_value']], 
                scalars='view_scalars'
            )
            
            return contour
            
        except Exception as e:
            logger.error(f"创建轮廓剪影失败: {e}")
            return None
    
    def _create_depth_silhouette(self, mesh: pv.PolyData) -> Optional[pv.PolyData]:
        """创建基于深度的剪影
        
        基于深度信息和表面法线创建剪影轮廓。
        
        Args:
            mesh: 输入网格
            
        Returns:
            剪影网格
        """
        try:
            # 创建网格副本
            mesh_copy = mesh.copy()
            
            # 获取相机位置
            if hasattr(self._plotter, 'camera_position'):
                camera_pos = np.array(self._plotter.camera_position[0])
            else:
                camera_pos = np.array([2, 2, 2])
            
            mesh_center = mesh_copy.center
            view_direction = camera_pos - mesh_center
            view_direction = view_direction / np.linalg.norm(view_direction)
            
            # 计算法线
            mesh_copy = mesh_copy.compute_normals(point_normals=True, cell_normals=True)
            
            # 创建剪影标量场
            normals = mesh_copy['Normals']
            silhouette_scalars = np.abs(np.dot(normals, view_direction))
            
            # 添加标量到网格
            mesh_copy['silhouette_scalars'] = silhouette_scalars
            
            # 提取剪影边缘（标量值低的位置）
            silhouette_edges = mesh_copy.contour(
                isosurfaces=[0.15],  # 调整此值以获得更好的效果
                scalars='silhouette_scalars'
            )
            
            return silhouette_edges
            
        except Exception as e:
            logger.error(f"创建深度剪影失败: {e}")
            return None
    
    def _create_surface_net_outline(self, mesh: pv.PolyData) -> Optional[pv.PolyData]:
        """创建基于表面网格的平滑轮廓
        
        使用体素化和表面重建技术创建平滑轮廓。
        
        Args:
            mesh: 输入网格
            
        Returns:
            平滑轮廓网格
        """
        try:
            # 将网格转换为体积表示
            bounds = mesh.bounds
            resolution = min(50, max(20, int(mesh.volume / 1000)))  # 动态分辨率
            
            # 体素化网格
            try:
                volume = pv.voxelize(mesh, density=resolution/20)
            except:
                # 如果体素化失败，使用边界框方法
                logger.warning("体素化失败，使用边界框方法")
                return self._create_feature_edges_outline(mesh)
            
            # 从体积提取表面
            surface = volume.extract_surface()
            
            # 提取特征边
            edges = surface.extract_feature_edges(
                feature_angle=20.0,
                boundary_edges=True,
                non_manifold_edges=False,
                manifold_edges=False
            )
            
            # 应用平滑处理
            if self._outline_settings['smooth_outline'] and edges.n_points > 0:
                edges = edges.smooth(n_iter=10, relaxation_factor=0.2)
            
            return edges
            
        except Exception as e:
            logger.error(f"创建表面网格轮廓失败: {e}")
            return None
    
    def set_outline_technique(self, technique: str) -> bool:
        """设置轮廓技术
        
        Args:
            technique: 轮廓技术名称
            
        Returns:
            bool: 设置是否成功
        """
        valid_techniques = ["feature_edges", "contour_silhouette", "depth_silhouette", "surface_net_outline"]
        
        if technique not in valid_techniques:
            logger.error(f"无效的轮廓技术: {technique}")
            return False
        
        self._current_outline_technique = technique
        self._outline_settings['technique'] = technique
        
        # 如果当前有模型，重新渲染
        if self._current_models and self._plotter:
            self._refresh_model_outlines()
        
        return True
    
    def set_outline_color(self, color: str) -> None:
        """设置轮廓颜色
        
        Args:
            color: 颜色名称或十六进制值
        """
        self._outline_settings['color'] = color
        self._update_outline_appearance()
    
    def set_outline_width(self, width: float) -> None:
        """设置轮廓线宽
        
        Args:
            width: 线宽
        """
        self._outline_settings['line_width'] = width
        self._update_outline_appearance()
    
    def set_outline_opacity(self, opacity: float) -> None:
        """设置轮廓透明度
        
        Args:
            opacity: 透明度 (0.0-1.0)
        """
        self._outline_settings['opacity'] = opacity
        self._update_outline_appearance()
    
    def set_feature_angle(self, angle: float) -> None:
        """设置特征角度（仅对feature_edges技术有效）
        
        Args:
            angle: 特征角度（度）
        """
        self._outline_settings['feature_angle'] = angle
        if self._current_outline_technique == "feature_edges" and self._current_models:
            self._refresh_model_outlines()
    
    def set_contour_value(self, value: float) -> None:
        """设置轮廓值（仅对contour_silhouette技术有效）
        
        Args:
            value: 轮廓值 (0.0-1.0)
        """
        self._outline_settings['contour_value'] = value
        if self._current_outline_technique == "contour_silhouette" and self._current_models:
            self._refresh_model_outlines()
    
    def toggle_outline(self, enabled: bool) -> None:
        """切换轮廓显示
        
        Args:
            enabled: 是否启用轮廓
        """
        self._outline_settings['enabled'] = enabled
        
        if self._current_models and self._plotter:
            self._refresh_model_outlines()
    
    def _refresh_model_outlines(self) -> None:
        """刷新模型轮廓"""
        if not self._plotter or not self._current_models:
            return
        
        try:
            # 获取当前模型
            current_meshes = []
            for actor in self._current_models:
                if hasattr(actor, 'GetMapper'):
                    mapper = actor.GetMapper()
                    if hasattr(mapper, 'GetInput'):
                        current_meshes.append(mapper.GetInput())
            
            # 清除现有轮廓
            for outline_actor in self._outline_actors:
                self._plotter.remove_actor(outline_actor)
            self._outline_actors.clear()
            
            # 重新添加轮廓
            for mesh in current_meshes:
                if mesh and self._outline_settings['enabled']:
                    self._add_outline_to_mesh(mesh, self._current_outline_technique)
            
            # 更新显示
            self._plotter.render()
            
        except Exception as e:
            logger.error(f"刷新轮廓失败: {e}")
    
    def _update_outline_appearance(self) -> None:
        """更新轮廓外观"""
        try:
            for outline_actor in self._outline_actors:
                prop = outline_actor.GetProperty()
                prop.SetColor(pv.Color(self._outline_settings['color']).float_rgb)
                prop.SetLineWidth(self._outline_settings['line_width'])
                prop.SetOpacity(self._outline_settings['opacity'])
            
            if self._plotter:
                self._plotter.render()
                
        except Exception as e:
            logger.error(f"更新轮廓外观失败: {e}")
    
    def get_outline_settings(self) -> Dict:
        """获取当前轮廓设置
        
        Returns:
            轮廓设置字典
        """
        return self._outline_settings.copy()
    
    def reset_outline_settings(self) -> None:
        """重置轮廓设置为默认值"""
        self._outline_settings = {
            'enabled': True,
            'technique': 'feature_edges',
            'color': 'black',
            'line_width': 2.0,
            'opacity': 1.0,
            'feature_angle': 30.0,
            'contour_value': 0.5,
            'smooth_outline': True
        }
        
        self._current_outline_technique = "feature_edges"
        
        if self._current_models:
            self._refresh_model_outlines()