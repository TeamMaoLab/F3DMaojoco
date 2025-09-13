"""
3D查看器

基于PyVista的3D可视化工具，用于查看F3DMaojocoScripts的导出结果。
提供简单的3D场景查看和交互功能。
"""

import os
import logging
from typing import Optional, Dict, Any, List, Tuple
import numpy as np

# 直接导入F3DMaojocoScripts的common模块
from F3DMaojocoScripts.common.data_types import ComponentInfo, JointInfo, Transform4D
from F3DMaojocoScripts.common.geometry_math import Vector3D

try:
    import pyvista as pv
    PYVISTA_AVAILABLE = True
except ImportError:
    PYVISTA_AVAILABLE = False
    logging.warning("PyVista not available. 3D visualization will be disabled.")

from .data_loader import ExportDataLoader


class VistaQuickViewer:
    """VistaQuickViewer 3D查看器
    
    基于PyVista的3D可视化工具，提供：
    - 3D场景查看
    - 零部件和关节显示
    - 交互式控制
    - 多种渲染模式
    """
    
    def __init__(self, export_dir: str, logger: Optional[logging.Logger] = None):
        """初始化3D查看器
        
        Args:
            export_dir: 导出目录路径
            logger: 可选的日志记录器
        """
        self.logger = logger or logging.getLogger(__name__)
        self.export_dir = export_dir
        
        # 检查PyVista可用性
        if not PYVISTA_AVAILABLE:
            raise RuntimeError("PyVista is required for 3D visualization")
        
        # 创建数据加载器
        self.data_loader = ExportDataLoader(export_dir, self.logger)
        
        # PyVista绘图器
        self.plotter = None
        
        # 场景对象
        self.actors = {}
        self.joint_actors = []  # 单独存储关节actor以便控制显示/隐藏
        
        # 显示设置
        self.show_grid = False
        self.show_axes = True
        self.background_color = 'white'
        self.show_joints = False  # 关节显示状态
        
        self.logger.info("VistaQuickViewer初始化完成")
    
    def load_data(self) -> bool:
        """加载导出数据
        
        Returns:
            bool: 是否成功加载
        """
        try:
            # 加载导出数据
            self.export_data = self.data_loader.load_export_data()
            self.stl_files = self.data_loader.load_stl_files()
            
            self.logger.info(f"数据加载成功: {len(self.export_data.components)} 个零部件, {len(self.export_data.joints)} 个关节")
            return True
            
        except Exception as e:
            self.logger.error(f"数据加载失败: {str(e)}")
            return False
    
    def create_scene(self) -> bool:
        """创建3D场景
        
        Returns:
            bool: 是否成功创建
        """
        try:
            # 创建PyVista绘图器
            self.plotter = pv.Plotter()
            
            # 设置背景色
            self.plotter.set_background(self.background_color)
            
            # 添加网格
            if self.show_grid:
                self.plotter.show_grid()
            
            # 添加坐标轴
            if self.show_axes:
                # 创建从原点延伸的坐标轴
                axis_length = 50.0
                arrow_length = 5.0
                
                # X轴 (红色)
                x_axis = pv.Line(pointa=[0, 0, 0], pointb=[axis_length, 0, 0])
                self.plotter.add_mesh(x_axis, color='red', line_width=5)
                
                # X轴箭头
                x_arrow = pv.Arrow(
                    start=[axis_length - arrow_length, 0, 0],
                    direction=[arrow_length, 0, 0],
                    tip_radius=0.1,
                    shaft_radius=0.05,
                    tip_length=0.2
                )
                self.plotter.add_mesh(x_arrow, color='red')
                
                # Y轴 (绿色)  
                y_axis = pv.Line(pointa=[0, 0, 0], pointb=[0, axis_length, 0])
                self.plotter.add_mesh(y_axis, color='green', line_width=5)
                
                # Y轴箭头
                y_arrow = pv.Arrow(
                    start=[0, axis_length - arrow_length, 0],
                    direction=[0, arrow_length, 0],
                    tip_radius=0.1,
                    shaft_radius=0.05,
                    tip_length=0.2
                )
                self.plotter.add_mesh(y_arrow, color='green')
                
                # Z轴 (蓝色)
                z_axis = pv.Line(pointa=[0, 0, 0], pointb=[0, 0, axis_length])
                self.plotter.add_mesh(z_axis, color='blue', line_width=5)
                
                # Z轴箭头
                z_arrow = pv.Arrow(
                    start=[0, 0, axis_length - arrow_length],
                    direction=[0, 0, arrow_length],
                    tip_radius=0.1,
                    shaft_radius=0.05,
                    tip_length=0.2
                )
                self.plotter.add_mesh(z_arrow, color='blue')
                
                # 添加坐标轴标签
                self.plotter.add_point_labels(
                    [np.array([axis_length + 0.3, 0, 0]), 
                     np.array([0, axis_length + 0.3, 0]), 
                     np.array([0, 0, axis_length + 0.3])],
                    ['X', 'Y', 'Z'],
                    point_size=10,
                    font_size=16,
                    shape='rounded_rect',
                    always_visible=True,
                    text_color='white'
                )
                
                # 添加原点标记
                self.plotter.add_points(
                    np.array([[0, 0, 0]]),
                    color='yellow',
                    point_size=15,
                    render_points_as_spheres=True
                )
            
            # 添加零部件
            self._add_components()
            
            # 添加关节
            self._add_joints()
            
            # 设置相机位置
            self._setup_camera()
            
            # 添加关节控制控件
            self.add_joint_control_widget()
            
            self.logger.info("3D场景创建完成")
            return True
            
        except Exception as e:
            self.logger.error(f"场景创建失败: {str(e)}")
            return False
    
    def _add_components(self):
        """添加零部件到场景"""
        for component in self.export_data.components:
            try:
                self.logger.info(f"处理零部件: {component.name}")
                
                # 获取STL文件路径
                stl_path = self.data_loader.get_stl_file_path(component)
                
                if stl_path and os.path.exists(stl_path):
                    self.logger.info(f"加载STL文件: {stl_path}")
                    # 加载STL文件
                    mesh = pv.read(stl_path)
                    
                    # 记录原始边界
                    original_bounds = mesh.bounds
                    self.logger.info(f"原始边界: {original_bounds}")
                    
                    # 应用世界坐标变换
                    if component.world_transform:
                        self.logger.info(f"应用世界坐标变换")
                        self.logger.debug(f"世界变换矩阵: {component.world_transform.matrix}")
                        
                        # 使用世界坐标变换（保持毫米单位）
                        mesh = self._apply_transform(mesh, component.world_transform)
                    
                    # 记录变换后边界
                    transformed_bounds = mesh.bounds
                    self.logger.info(f"变换后边界: {transformed_bounds}")
                    
                    # 添加到场景
                    actor = self.plotter.add_mesh(
                        mesh,
                        color=self._generate_color(component.name),
                        opacity=0.8,
                        show_edges=False,
                        name=component.name
                    )
                    
                    self.actors[component.name] = actor
                    
                else:
                    self.logger.warning(f"未找到STL文件: {stl_path}")
                    # 如果没有STL文件，创建占位符几何体
                    self._add_placeholder_component(component)
                    
            except Exception as e:
                self.logger.error(f"添加零部件 {component.name} 失败: {str(e)}")
                import traceback
                self.logger.error(traceback.format_exc())
                # 创建占位符
                self._add_placeholder_component(component)
    
    def _add_placeholder_component(self, component: ComponentInfo):
        """添加占位符零部件"""
        try:
            # 创建立方体作为占位符
            cube = pv.Cube(center=(0, 0, 0), x_length=1.0, y_length=1.0, z_length=1.0)
            
            # 应用变换矩阵
            if component.world_transform:
                cube = self._apply_transform(cube, component.world_transform)
            
            # 添加到场景
            actor = self.plotter.add_mesh(
                cube,
                color=self._generate_color(component.name),
                opacity=0.6,
                show_edges=True,
                name=f"{component.name}_placeholder"
            )
            
            self.actors[component.name] = actor
            
        except Exception as e:
            self.logger.error(f"创建占位符零部件 {component.name} 失败: {str(e)}")
    
    def _add_joints(self):
        """添加关节到场景"""
        for joint in self.export_data.joints:
            try:
                # 获取关节的世界坐标变换
                world_transform = self._get_joint_world_transform(joint)
                
                if world_transform:
                    # 创建球体表示关节（使用毫米单位）
                    sphere = pv.Sphere(radius=2.0, center=(0, 0, 0))  # 增大半径到2mm
                    
                    # 应用世界坐标变换
                    sphere = self._apply_transform(sphere, world_transform)
                    
                    # 添加到场景
                    actor = self.plotter.add_mesh(
                        sphere,
                        color='red',
                        opacity=0.9,
                        name=f"joint_{joint.name}"
                    )
                    
                    # 设置可见性
                    actor.SetVisibility(self.show_joints)
                    
                    self.actors[f"joint_{joint.name}"] = actor
                    self.joint_actors.append(actor)  # 添加到关节列表以便控制
                    
                    # 添加关节标签
                    pos = world_transform.get_translation()
                    label_actor = self.plotter.add_point_labels(
                        [np.array([pos.x, pos.y, pos.z])],
                        [joint.name],
                        point_size=10,
                        font_size=12,
                        name=f"joint_label_{joint.name}"
                    )
                    # 设置标签可见性
                    label_actor.SetVisibility(self.show_joints)
                    self.joint_actors.append(label_actor)  # 将标签也加入关节列表
                    
                    # 记录调试信息
                    self.logger.info(f"关节 {joint.name} 世界位置: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
                else:
                    self.logger.warning(f"无法计算关节 {joint.name} 的世界坐标")
                    
            except Exception as e:
                self.logger.warning(f"添加关节 {joint.name} 失败: {str(e)}")
                import traceback
                self.logger.error(traceback.format_exc())
    
    def _get_joint_world_transform(self, joint: JointInfo) -> Optional[Transform4D]:
        """获取关节的世界坐标变换
        
        Args:
            joint: 关节信息
            
        Returns:
            Optional[Transform4D]: 世界坐标变换矩阵
        """
        try:
            # 检查连接信息
            if not joint.connection:
                self.logger.warning(f"关节 {joint.name} 缺少连接信息")
                return None
                
            # 根据你的说明，geometry transforms 应该已经是世界坐标
            # 并且关节连接点应该与零部件原点很接近
            
            # 优先使用 geometry_one_transform
            if joint.geometry.geometry_one_transform:
                # 验证：检查关节位置是否与对应零部件位置接近
                if joint.connection.occurrence_one_component:
                    component_one = self.data_loader.get_component_by_name(joint.connection.occurrence_one_component)
                    if component_one and component_one.world_transform:
                        joint_pos = joint.geometry.geometry_one_transform.get_translation()
                        component_pos = component_one.world_transform.get_translation()
                        
                        # 计算距离
                        distance = ((joint_pos.x - component_pos.x)**2 + 
                                  (joint_pos.y - component_pos.y)**2 + 
                                  (joint_pos.z - component_pos.z)**2)**0.5
                        
                        self.logger.info(f"关节 {joint.name} 与 {component_one.name} 距离: {distance:.3f}mm")
                        
                        if distance < 50.0:  # 如果距离小于50mm，认为是合理的
                            self.logger.info(f"关节 {joint.name} 使用 geometry_one_transform (已是世界坐标)")
                            return joint.geometry.geometry_one_transform
                
                # 如果验证失败，仍然使用它但给出警告
                self.logger.warning(f"关节 {joint.name} geometry_one_transform 位置验证失败，但仍使用")
                return joint.geometry.geometry_one_transform
                
            # 备用：使用 geometry_two_transform
            if joint.geometry.geometry_two_transform:
                if joint.connection.occurrence_two_component:
                    component_two = self.data_loader.get_component_by_name(joint.connection.occurrence_two_component)
                    if component_two and component_two.world_transform:
                        joint_pos = joint.geometry.geometry_two_transform.get_translation()
                        component_pos = component_two.world_transform.get_translation()
                        
                        distance = ((joint_pos.x - component_pos.x)**2 + 
                                  (joint_pos.y - component_pos.y)**2 + 
                                  (joint_pos.z - component_pos.z)**2)**0.5
                        
                        self.logger.info(f"关节 {joint.name} 与 {component_two.name} 距离: {distance:.3f}mm")
                        
                        if distance < 50.0:
                            self.logger.info(f"关节 {joint.name} 使用 geometry_two_transform (已是世界坐标)")
                            return joint.geometry.geometry_two_transform
                
                self.logger.warning(f"关节 {joint.name} geometry_two_transform 位置验证失败，但仍使用")
                return joint.geometry.geometry_two_transform
                
            self.logger.warning(f"关节 {joint.name} 没有可用的几何变换信息")
            return None
            
        except Exception as e:
            self.logger.error(f"获取关节 {joint.name} 世界坐标失败: {str(e)}")
            import traceback
            self.logger.error(traceback.format_exc())
            return None
    
    def _apply_transform(self, mesh, transform: Transform4D):
        """应用变换矩阵到网格
        
        Args:
            mesh: PyVista网格对象
            transform: 变换矩阵
            
        Returns:
            变换后的网格
        """
        try:
            # 提取变换矩阵
            matrix = transform.matrix
            
            self.logger.info(f"原始变换矩阵: {matrix}")
            
            # 清理浮点误差：接近零的值设为0
            cleaned_matrix = []
            for row in matrix:
                cleaned_row = []
                for val in row:
                    if abs(val) < 1e-10:
                        cleaned_row.append(0.0)
                    else:
                        cleaned_row.append(val)
                cleaned_matrix.append(cleaned_row)
            
            # 提取位置和旋转信息
            translation = [cleaned_matrix[0][3], cleaned_matrix[1][3], cleaned_matrix[2][3]]
            rotation_matrix = [
                [cleaned_matrix[0][0], cleaned_matrix[0][1], cleaned_matrix[0][2]],
                [cleaned_matrix[1][0], cleaned_matrix[1][1], cleaned_matrix[1][2]],
                [cleaned_matrix[2][0], cleaned_matrix[2][1], cleaned_matrix[2][2]]
            ]
            
            self.logger.info(f"位置信息: X={translation[0]:.3f}, Y={translation[1]:.3f}, Z={translation[2]:.3f}")
            self.logger.info(f"旋转矩阵: {rotation_matrix}")
            
            # 使用PyVista的transform方法，但使用正确的格式
            # 创建4x4变换矩阵（行优先格式）
            transform_4x4 = np.array([
                [cleaned_matrix[0][0], cleaned_matrix[0][1], cleaned_matrix[0][2], cleaned_matrix[0][3]],
                [cleaned_matrix[1][0], cleaned_matrix[1][1], cleaned_matrix[1][2], cleaned_matrix[1][3]],
                [cleaned_matrix[2][0], cleaned_matrix[2][1], cleaned_matrix[2][2], cleaned_matrix[2][3]],
                [0.0, 0.0, 0.0, 1.0]
            ], dtype=np.float64)
            
            self.logger.info(f"PyVista变换矩阵 (行优先): {transform_4x4}")
            
            # 应用变换
            transformed_mesh = mesh.copy()
            transformed_mesh = transformed_mesh.transform(transform_4x4, inplace=False)
            
            # 验证变换结果
            original_bounds = mesh.bounds
            transformed_bounds = transformed_mesh.bounds
            
            original_center = [(original_bounds[0] + original_bounds[1])/2,
                              (original_bounds[2] + original_bounds[3])/2,
                              (original_bounds[4] + original_bounds[5])/2]
            transformed_center = [(transformed_bounds[0] + transformed_bounds[1])/2,
                                 (transformed_bounds[2] + transformed_bounds[3])/2,
                                 (transformed_bounds[4] + transformed_bounds[5])/2]
            
            actual_translation = [transformed_center[0] - original_center[0],
                                  transformed_center[1] - original_center[1],
                                  transformed_center[2] - original_center[2]]
            
            self.logger.info(f"原始中心: {original_center}")
            self.logger.info(f"变换后中心: {transformed_center}")
            self.logger.info(f"实际平移: {actual_translation}")
            self.logger.info(f"预期平移: {translation}")
            
            # 计算误差
            error = sum(abs(actual_translation[i] - translation[i]) for i in range(3))
            self.logger.info(f"平移误差: {error:.6f}")
            
            return transformed_mesh
            
        except Exception as e:
            self.logger.error(f"应用变换矩阵失败: {str(e)}")
            self.logger.error(f"变换矩阵: {transform.matrix}")
            import traceback
            self.logger.error(traceback.format_exc())
            return mesh
    
    def _setup_camera(self):
        """设置相机位置"""
        try:
            # 获取场景边界
            bounds = self.data_loader.get_scene_bounds()
            if bounds:
                # 计算场景中心
                center_x = (bounds['min_x'] + bounds['max_x']) / 2
                center_y = (bounds['min_y'] + bounds['max_y']) / 2
                center_z = (bounds['min_z'] + bounds['max_z']) / 2
                
                # 计算场景大小
                size_x = bounds['max_x'] - bounds['min_x']
                size_y = bounds['max_y'] - bounds['min_y']
                size_z = bounds['max_z'] - bounds['min_z']
                max_size = max(size_x, size_y, size_z, 50.0)  # 考虑坐标轴长度
                
                self.logger.info(f"场景中心: ({center_x:.1f}, {center_y:.1f}, {center_z:.1f})")
                self.logger.info(f"场景大小: {size_x:.1f} x {size_y:.1f} x {size_z:.1f}")
                
                # 设置相机位置 - 等距投影视角，适合观察大型装配体
                camera_distance = max_size * 2.5
                self.plotter.camera_position = [
                    (center_x + camera_distance, center_y - camera_distance, center_z + camera_distance * 0.8),
                    (center_x, center_y, center_z),
                    (0, 0, 1)
                ]
                
                # 设置相机视角
                self.plotter.reset_camera()
                
            else:
                # 默认相机位置 - 适应更大的场景
                camera_distance = 200.0
                self.plotter.camera_position = [
                    (camera_distance, -camera_distance, camera_distance * 0.8),
                    (0, 0, 0),
                    (0, 0, 1)
                ]
                
        except Exception as e:
            self.logger.error(f"设置相机失败: {str(e)}")
    
    def _generate_color(self, name: str) -> str:
        """根据名称生成颜色
        
        Args:
            name: 对象名称
            
        Returns:
            str: 颜色名称
        """
        # 简单的哈希函数生成颜色
        colors = ['blue', 'green', 'red', 'yellow', 'cyan', 'magenta', 'orange', 'purple', 'brown', 'pink']
        hash_value = hash(name)
        color_index = abs(hash_value) % len(colors)
        return colors[color_index]
    
    def toggle_joints_visibility(self, show: bool):
        """切换关节显示状态
        
        Args:
            show: True=显示关节，False=隐藏关节
        """
        self.show_joints = show
        
        # 更新所有关节actor的可见性
        for actor in self.joint_actors:
            if hasattr(actor, 'SetVisibility'):
                actor.SetVisibility(show)
            elif hasattr(actor, 'Set'):
                actor.Set(show)
        
        # 刷新场景
        if self.plotter:
            self.plotter.render()
        
        self.logger.info(f"关节显示状态: {'显示' if show else '隐藏'}")
    
    def add_joint_control_widget(self):
        """添加关节控制交互控件"""
        if not self.plotter:
            return
        
        def joint_checkbox_callback(value):
            """关节复选框回调函数"""
            self.toggle_joints_visibility(value)
        
        # 使用 PyVista 的 checkbox widget
        self.plotter.add_checkbox_button_widget(
            callback=joint_checkbox_callback,
            value=self.show_joints,  # 初始状态
            position=(10, 10)  # 左上角位置
        )
        
        # 添加文本标签
        self.plotter.add_text(
            "Show Joints",
            position=(50, 15),
            font_size=12,
            color='black',
            shadow=True
        )
        
        self.logger.info("已添加关节控制控件")
    
    def show(self, screenshot_path: Optional[str] = None):
        """显示3D场景
        
        Args:
            screenshot_path: 可选的截图保存路径
        """
        if not self.plotter:
            self.logger.error("场景未创建")
            return
        
        try:
            if screenshot_path:
                # 保存截图（需要设置off_screen=True）
                self.plotter.off_screen = True
                self.plotter.screenshot(screenshot_path)
                self.logger.info(f"截图已保存: {screenshot_path}")
            else:
                # 显示交互式窗口
                self.plotter.show()
                
        except Exception as e:
            self.logger.error(f"显示场景失败: {str(e)}")
    
    def set_background_color(self, color: str):
        """设置背景颜色
        
        Args:
            color: 颜色名称或RGB值
        """
        self.background_color = color
        if self.plotter:
            self.plotter.set_background(color)
    
    def set_grid_visible(self, visible: bool):
        """设置网格可见性
        
        Args:
            visible: 是否可见
        """
        self.show_grid = visible
        # 注意：PyVista的网格显示需要在创建时设置
    
    def set_axes_visible(self, visible: bool):
        """设置坐标轴可见性
        
        Args:
            visible: 是否可见
        """
        self.show_axes = visible
        # 注意：PyVista的坐标轴显示需要在创建时设置
    
    def get_scene_summary(self) -> Dict[str, Any]:
        """获取场景摘要
        
        Returns:
            Dict[str, Any]: 场景摘要信息
        """
        if not hasattr(self, 'export_data'):
            return {"error": "数据未加载"}
        
        return {
            'export_dir': self.export_dir,
            'component_count': len(self.export_data.components),
            'joint_count': len(self.export_data.joints),
            'stl_file_count': len(self.stl_files) if hasattr(self, 'stl_files') else 0,
            'actors_count': len(self.actors),
            'bounds': self.data_loader.get_scene_bounds()
        }
    
    def save_view(self, filepath: str):
        """保存当前视图
        
        Args:
            filepath: 保存路径
        """
        if not self.plotter:
            self.logger.error("场景未创建")
            return
        
        try:
            # 保存为图片
            self.plotter.screenshot(filepath)
            self.logger.info(f"视图已保存: {filepath}")
            
        except Exception as e:
            self.logger.error(f"保存视图失败: {str(e)}")
    
    def close(self):
        """关闭查看器"""
        if self.plotter:
            self.plotter.close()
            self.plotter = None
        
        self.actors.clear()
        self.logger.info("VistaQuickViewer已关闭")


def create_viewer(export_dir: str, logger: Optional[logging.Logger] = None) -> VistaQuickViewer:
    """创建3D查看器实例
    
    Args:
        export_dir: 导出目录路径
        logger: 可选的日志记录器
        
    Returns:
        VistaQuickViewer: 3D查看器实例
    """
    return VistaQuickViewer(export_dir, logger)


def quick_view(export_dir: str, show: bool = True, screenshot_path: Optional[str] = None, show_joints: bool = False):
    """快速查看3D场景
    
    Args:
        export_dir: 导出目录路径
        show: 是否显示交互式窗口
        screenshot_path: 可选的截图保存路径
        show_joints: 初始是否显示关节（默认隐藏）
    """
    logger = logging.getLogger(__name__)
    
    try:
        # 创建查看器
        viewer = create_viewer(export_dir, logger)
        
        # 设置关节显示状态
        viewer.show_joints = show_joints
        
        # 加载数据
        if not viewer.load_data():
            logger.error("数据加载失败")
            return
        
        # 创建场景
        if not viewer.create_scene():
            logger.error("场景创建失败")
            return
        
        # 显示场景
        if show or screenshot_path:
            viewer.show(screenshot_path)
        
        # 打印场景摘要
        summary = viewer.get_scene_summary()
        logger.info(f"场景摘要: {summary}")
        
        return viewer
        
    except Exception as e:
        logger.error(f"快速查看失败: {str(e)}")
        return None