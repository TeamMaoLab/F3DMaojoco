"""
简化的3D可视化组件

只负责显示，业务逻辑委托给Core服务。
基于原有visualization_widget.py进行简化。
"""

# 标准库
from typing import Optional, List, Dict
from pathlib import Path

# 第三方库
import pyvista as pv
import numpy as np
from pyvistaqt import QtInteractor
from PySide6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel
from PySide6.QtCore import Signal, Qt, QTimer
from PySide6.QtGui import QFont

# 本地模块
from ..utils.logger import logger
from ..core.project_workflow_manager import ProjectWorkflowManager
from ..core.domain_types import LoadResult, STLModel, Transform4D
from ..core.stl_model_manager import ModelData
import pyvista as pv
import tempfile
from pathlib import Path


class VisualizationWidget(QWidget):
    """简化的3D可视化组件
    
    只负责STL模型的显示和交互，业务逻辑委托给Core服务。
    """
    
    # 信号定义
    model_loaded = Signal(str)      # 模型加载完成信号
    error_occurred = Signal(str)    # 错误信号
    
    def __init__(self, workflow_manager: ProjectWorkflowManager, parent=None):
        """初始化可视化组件
        
        Args:
            workflow_manager: 工作流管理器
            parent: 父组件
        """
        super().__init__(parent)
        self._workflow_manager = workflow_manager
        self._plotter: Optional[QtInteractor] = None
        self._current_models: List[ModelData] = []
        self._model_colors: Dict[str, str] = {}  # 模型颜色映射
        self._actor_map: Dict[str, object] = {}  # 模型名称到actor的映射
        self._initial_text_label: Optional[QLabel] = None
        self._initial_text_timer: Optional[QTimer] = None
        self._loaded_model_hashes: set = set()  # 用于跟踪已加载的模型，避免重复加载
        self._setup_ui()
    
    def _convert_stl_model_to_model_data(self, stl_model: STLModel) -> Optional[ModelData]:
        """将STLModel转换为ModelData
        
        Args:
            stl_model: STL模型数据
            
        Returns:
            ModelData: 可视化模型数据，转换失败返回None
        """
        try:
            # 从二进制数据创建临时文件
            with tempfile.NamedTemporaryFile(suffix='.stl', delete=False) as temp_file:
                temp_file.write(stl_model.mesh_data)
                temp_file_path = Path(temp_file.name)
            
            # 使用PyVista读取STL文件
            mesh = pv.read(temp_file_path)
            
            # 清理临时文件
            temp_file_path.unlink()
            
            # 应用坐标变换（关键修复）
            if stl_model.is_transformed and stl_model.transform_matrix:
                logger.info(f"应用坐标变换到模型: {stl_model.name}")
                mesh = self._apply_transform_to_mesh(mesh, stl_model.transform_matrix)
            
            # 创建ModelData对象
            model_data = ModelData(
                mesh=mesh,
                name=stl_model.name,
                color='gray',  # 默认颜色
                component_id=0,  # 使用默认ID
                is_transformed=stl_model.is_transformed,
                original_bounds=mesh.bounds if hasattr(mesh, 'bounds') else None
            )
            
            logger.info(f"成功转换STL模型: {stl_model.name} -> {mesh.n_points}个顶点, {mesh.n_cells}个面, 变换状态: {stl_model.is_transformed}")
            return model_data
            
        except Exception as e:
            logger.error(f"转换STL模型失败: {stl_model.name}, 错误: {e}")
            # 清理临时文件（如果存在）
            if 'temp_file_path' in locals():
                try:
                    temp_file_path.unlink()
                except:
                    pass
            return None
    
    def _apply_transform_to_mesh(self, mesh: pv.PolyData, transform_matrix: Transform4D) -> pv.PolyData:
        """应用坐标变换到网格
        
        Args:
            mesh: 原始网格
            transform_matrix: 变换矩阵
            
        Returns:
            pv.PolyData: 变换后的网格
        """
        try:
            if not transform_matrix or not hasattr(transform_matrix, 'matrix'):
                logger.warning(f"变换矩阵无效，跳过变换应用")
                return mesh
            
            # 获取变换矩阵数据
            matrix_data = transform_matrix.matrix
            if not matrix_data or len(matrix_data) != 4 or any(len(row) != 4 for row in matrix_data):
                logger.warning(f"变换矩阵格式无效，跳过变换应用")
                return mesh
            
            # 记录变换前的边界
            original_bounds = mesh.bounds
            logger.debug(f"应用变换前边界: {original_bounds}")
            
            # 添加调试日志验证变换矩阵数据
            logger.info(f"变换矩阵数据: {matrix_data}")
            logger.info(f"变换矩阵类型: {type(matrix_data)}")
            
            # 提取平移向量用于调试
            translation = [matrix_data[0][3], matrix_data[1][3], matrix_data[2][3]]
            logger.info(f"平移向量: {translation}")
            
            # 提取旋转矩阵用于调试
            rotation_matrix = [row[:3] for row in matrix_data[:3]]
            logger.info(f"旋转矩阵: {rotation_matrix}")
            
            # 提取平移和旋转部分
            # Transform4D矩阵格式:
            # [ [R00, R01, R02, Tx],
            #   [R10, R11, R12, Ty], 
            #   [R20, R21, R22, Tz],
            #   [0,   0,   0,   1] ]
            
            # 获取所有点
            points = mesh.points
            if len(points) == 0:
                logger.warning(f"网格没有顶点数据，跳过变换")
                return mesh
            
            # 应用变换：对于每个点 p' = R * p + T
            transformed_points = []
            for point in points:
                # 确保点是3D坐标
                if len(point) >= 3:
                    x, y, z = point[0], point[1], point[2]
                    
                    # 应用旋转和平移
                    new_x = matrix_data[0][0] * x + matrix_data[0][1] * y + matrix_data[0][2] * z + matrix_data[0][3]
                    new_y = matrix_data[1][0] * x + matrix_data[1][1] * y + matrix_data[1][2] * z + matrix_data[1][3]
                    new_z = matrix_data[2][0] * x + matrix_data[2][1] * y + matrix_data[2][2] * z + matrix_data[2][3]
                    
                    transformed_points.append([new_x, new_y, new_z])
                else:
                    # 保持原始点不变
                    transformed_points.append(point)
            
            # 创建新的网格
            transformed_mesh = mesh.copy()
            transformed_mesh.points = np.array(transformed_points)
            
            # 记录变换后的边界
            new_bounds = transformed_mesh.bounds
            logger.debug(f"应用变换后边界: {new_bounds}")
            
            # 计算变换统计信息
            translation = [matrix_data[0][3], matrix_data[1][3], matrix_data[2][3]]
            logger.info(f"成功应用坐标变换到网格: 平移={translation}, 顶点数={len(points)}")
            
            return transformed_mesh
            
        except Exception as e:
            logger.error(f"应用坐标变换失败: {e}")
            return mesh
    
    def _setup_ui(self):
        """设置UI界面"""
        logger.info("初始化简化可视化组件")
        
        # 创建主布局
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        
        # 创建控制栏
        control_bar = self._create_control_bar()
        main_layout.addLayout(control_bar)
        
        # 创建PyVista渲染器
        self._setup_pyvista_renderer(main_layout)
        
        # 创建初始化提示文字
        self._create_initial_text()
        
        logger.success("简化可视化组件初始化完成")
    
    def _create_control_bar(self) -> QHBoxLayout:
        """创建控制栏
        
        Returns:
            QHBoxLayout: 控制栏布局
        """
        control_layout = QHBoxLayout()
        
        # 标题标签
        title_label = QLabel("3D 模型视图")
        title_font = QFont()
        title_font.setBold(True)
        title_label.setFont(title_font)
        control_layout.addWidget(title_label)
        
        control_layout.addStretch()
        
        # 控制按钮
        reset_view_btn = QPushButton("重置视角")
        reset_view_btn.clicked.connect(self._reset_view)
        control_layout.addWidget(reset_view_btn)
        
        fit_view_btn = QPushButton("适应视图")
        fit_view_btn.clicked.connect(self._fit_view_to_models)
        control_layout.addWidget(fit_view_btn)
        
        return control_layout
    
    def _setup_pyvista_renderer(self, parent_layout: QVBoxLayout):
        """设置PyVista渲染器
        
        Args:
            parent_layout: 父布局
        """
        try:
            # 创建QtInteractor
            self._plotter = QtInteractor(self)
            
            # 设置渲染器属性
            self._plotter.renderer.set_background("white")
            self._plotter.renderer.enable_anti_aliasing()
            
            # 添加坐标轴
            self._plotter.add_axes()
            
            # 设置初始相机位置
            self._plotter.camera_position = [(2, 2, 2), (0, 0, 0), (0, 0, 1)]
            
            # 添加到布局
            parent_layout.addWidget(self._plotter)
            
            logger.success("PyVista渲染器初始化完成")
            
        except Exception as e:
            logger.error(f"PyVista渲染器初始化失败: {e}")
            # 创建错误显示
            error_label = QLabel(f"3D渲染器初始化失败: {e}")
            error_label.setStyleSheet("color: red; padding: 20px;")
            parent_layout.addWidget(error_label)
    
    def display_models_from_result(self, result: LoadResult):
        """从加载结果显示模型
        
        Args:
            result: 加载结果
        """
        if not result.success:
            logger.error(f"无法显示模型：{result.message}")
            self.error_occurred.emit(f"加载失败：{result.message}")
            return
        
        # 生成结果哈希值，检查是否已经加载过相同的结果
        result_hash = self._generate_result_hash(result)
        if result_hash in self._loaded_model_hashes:
            logger.info(f"检测到重复加载，跳过: {result_hash}")
            return
        
        logger.info(f"显示 {len(result.models)} 个模型")
        
        # 检查模型类型并转换
        models_to_display = []
        for model in result.models:
            if isinstance(model, ModelData):
                # 已经是ModelData类型，直接使用
                models_to_display.append(model)
            elif hasattr(model, 'mesh_data') and hasattr(model, 'name'):
                # 这是STLModel类型，需要转换
                converted_model = self._convert_stl_model_to_model_data(model)
                if converted_model:
                    models_to_display.append(converted_model)
            else:
                logger.warning(f"未知模型类型: {type(model)}, 跳过")
        
        logger.info(f"成功转换 {len(models_to_display)} 个模型用于显示")
        
        # 记录已加载的结果
        self._loaded_model_hashes.add(result_hash)
        
        self.display_models(models_to_display)
    
    def _generate_result_hash(self, result: LoadResult) -> str:
        """生成加载结果的哈希值，用于检测重复加载
        
        Args:
            result: 加载结果
            
        Returns:
            str: 哈希值
        """
        import hashlib
        
        # 基于模型数量和名称生成哈希
        model_names = [model.name for model in result.models if hasattr(model, 'name')]
        model_names_str = ",".join(sorted(model_names))
        
        # 添加时间戳避免哈希碰撞
        content = f"{len(result.models)}:{model_names_str}:{result.message}"
        
        return hashlib.md5(content.encode()).hexdigest()
    
    def display_models(self, models: List[ModelData]):
        """显示模型数据列表
        
        Args:
            models: 模型数据列表
        """
        try:
            logger.info(f"开始显示 {len(models)} 个模型，首先隐藏初始化文字")
            
            # 隐藏初始化提示文字
            self.hide_initial_text()
            
            # 清除现有模型
            if self._plotter:
                self._plotter.clear()
                self._plotter.add_axes()  # 重新添加坐标轴
            
            if not models:
                logger.warning("没有模型需要显示")
                return
            
            # 显示新模型
            for i, model in enumerate(models):
                try:
                    logger.info(f"添加模型到场景: {model.name}")
                    self._add_model_to_scene(model, i)
                except Exception as e:
                    logger.error(f"添加模型 {model.name} 失败: {e}")
                    continue
            
            # 保存当前模型列表
            self._current_models = models
            
            # 初始化颜色映射和actor映射
            self._model_colors = {model.name: 'gray' for model in models}
            self._actor_map.clear()  # 清理actor映射
            
            # 调整视图
            self._fit_view_to_models()
            
            logger.success(f"成功显示 {len(models)} 个模型")
            self.model_loaded.emit(f"已加载 {len(models)} 个模型")
            
            # 额外确保初始化文字隐藏
            if self._initial_text_label and self._initial_text_label.isVisible():
                logger.warning("模型显示后初始化文字仍然可见，再次隐藏")
                self._initial_text_label.setVisible(False)
            
        except Exception as e:
            logger.error(f"显示模型时发生错误: {e}")
            self.error_occurred.emit(f"显示模型失败: {e}")
    
    def _add_model_to_scene(self, model: ModelData, index: int):
        """添加单个模型到场景
        
        Args:
            model: 模型数据
            index: 模型索引（用于颜色分配）
        """
        if not self._plotter:
            logger.error("渲染器未初始化")
            return
        
        try:
            # 直接使用模型中的mesh对象
            mesh = model.mesh
            
            # 获取模型颜色，默认为灰色
            color = self._model_colors.get(model.name, 'gray')
            
            actor = self._plotter.add_mesh(
                mesh, 
                color=color,
                show_edges=False,  # 不显示三角形边线
                smooth_shading=True,  # 启用平滑着色
                lighting=True,  # 启用光照效果
                specular=0.3,  # 设置高光反射
                specular_power=20,  # 设置高光强度
                opacity=0.5,  # 50%透明度
                name=model.name
            )
            
            # 保存actor映射
            self._actor_map[model.name] = actor
            
            # 提取特征边创建轮廓线，增强边界定义
            try:
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
                        line_width=1.0,
                        opacity=0.8,  # 轮廓线使用稍高透明度以便看清
                        render_lines_as_tubes=True,
                        name=f"{model.name}_outline"
                    )
                    logger.debug(f"为模型 {model.name} 添加轮廓线")
                    
            except Exception as outline_error:
                logger.debug(f"轮廓线添加失败（不影响主模型）: {outline_error}")
            
            logger.info(f"添加模型到场景: {model.name}")
                    
        except Exception as e:
            logger.error(f"添加模型 {model.name} 失败: {e}")
            raise
    
      
    def _reset_view(self):
        """重置视角"""
        if not self._plotter:
            return
        
        try:
            # 先重置相机以适应所有模型
            self._plotter.reset_camera()
            
            # 然后设置一个合适的视角位置，确保能看清整个模型
            bounds = self._plotter.renderer.bounds
            if bounds:
                # 计算模型的大致尺寸
                x_range = bounds[1] - bounds[0]
                y_range = bounds[3] - bounds[2] 
                z_range = bounds[5] - bounds[4]
                max_range = max(x_range, y_range, z_range)
                
                # 根据模型大小调整相机距离
                distance = max_range * 2.5  # 确保足够的距离看到整个模型
                
                # 设置相机位置：从右上方俯视
                camera_pos = [distance, distance, distance * 0.8]
                focal_point = [(bounds[0] + bounds[1]) / 2, (bounds[2] + bounds[3]) / 2, (bounds[4] + bounds[5]) / 2]
                view_up = [0, 0, 1]
                
                self._plotter.camera_position = [camera_pos, focal_point, view_up]
            
            self._plotter.render()
            logger.info("视角已重置")
        except Exception as e:
            logger.error(f"重置视角失败: {e}")
            # 如果计算失败，使用默认位置
            try:
                self._plotter.camera_position = [(5, 5, 3), (0, 0, 0), (0, 0, 1)]
                self._plotter.render()
            except:
                pass
    
    def _fit_view_to_models(self):
        """调整视角以适应所有模型"""
        if not self._plotter or not self._current_models:
            return
        
        try:
            # 重置相机以适应所有模型
            self._plotter.reset_camera()
            
            # 获取模型边界，确保相机位置合适
            bounds = self._plotter.renderer.bounds
            if bounds:
                # 计算模型的大致尺寸
                x_range = bounds[1] - bounds[0]
                y_range = bounds[3] - bounds[2] 
                z_range = bounds[5] - bounds[4]
                max_range = max(x_range, y_range, z_range)
                
                # 根据模型大小调整相机距离
                distance = max_range * 2.0  # 适应视图时使用稍近的距离
                
                # 设置相机位置：从右上方俯视
                camera_pos = [distance, distance, distance * 0.8]
                focal_point = [(bounds[0] + bounds[1]) / 2, (bounds[2] + bounds[3]) / 2, (bounds[4] + bounds[5]) / 2]
                view_up = [0, 0, 1]
                
                self._plotter.camera_position = [camera_pos, focal_point, view_up]
            
            self._plotter.render()
            logger.info("视角已调整到适应所有模型")
        except Exception as e:
            logger.error(f"调整视角失败: {e}")
    
    def _create_initial_text(self) -> None:
        """创建初始化提示文字"""
        try:
            # 创建一个透明的覆盖标签
            self._initial_text_label = QLabel("欢迎使用 MaojocoConverter GUI\n\n请在右侧选择项目目录开始", self)
            self._initial_text_label.setAlignment(Qt.AlignCenter)
            self._initial_text_label.setWordWrap(True)  # 允许文字换行
            self._initial_text_label.setStyleSheet("""
                QLabel {
                    background-color: rgba(255, 255, 200, 240);
                    color: #222222;
                    font-size: 28px;
                    font-weight: bold;
                    border: 3px solid #FF6B35;
                    border-radius: 20px;
                    padding: 30px 35px;
                    min-width: 500px;
                    min-height: 120px;
                }
            """)
            
            # 设置更好的字体
            font = QFont("Arial", 24, QFont.Bold)
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
            self._initial_text_timer = QTimer()
            self._initial_text_timer.setSingleShot(True)
            self._initial_text_timer.timeout.connect(self._position_and_show_text)
            self._initial_text_timer.start(300)  # 延迟显示
            
            logger.info("初始化提示文字标签创建完成")
            
        except Exception as e:
            logger.error(f"创建初始化提示文字失败: {e}")
    
    def _position_and_show_text(self) -> None:
        """定位并显示提示文字"""
        if self._initial_text_label and self._plotter:
            try:
                # 获取3D窗口的几何位置
                plotter_geometry = self._plotter.geometry()
                
                # 计算标签尺寸
                label_width = 550
                label_height = 150
                
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
                
                logger.info(f"初始化提示文字定位完成: 位置({x}, {y}), 尺寸({label_width}x{label_height})")
                
            except Exception as e:
                logger.error(f"初始化提示文字定位失败: {e}")
                # 如果定位失败，使用安全的默认位置
                self._initial_text_label.setGeometry(20, 20, 550, 150)
                self._initial_text_label.setVisible(True)
    
    def show_initial_text(self) -> None:
        """显示初始化提示文字"""
        if self._initial_text_label:
            self._initial_text_label.setVisible(True)
            self._initial_text_label.raise_()
    
    def hide_initial_text(self) -> None:
        """隐藏初始化提示文字"""
        if self._initial_text_label:
            self._initial_text_label.setVisible(False)
            logger.info("初始化提示文字已隐藏")
            # 停止任何未执行的定时器
            if self._initial_text_timer:
                self._initial_text_timer.stop()
                logger.info("初始化文字定时器已停止")
        else:
            logger.warning("尝试隐藏初始化提示文字，但标签不存在")
    
    def set_model_visibility(self, model_name: str, visible: bool):
        """设置模型可见性
        
        Args:
            model_name: 模型名称
            visible: 是否可见
        """
        if not self._plotter:
            return
        
        try:
            actor = self._plotter.get_actor(model_name)
            if actor:
                actor.SetVisibility(visible)
                self._plotter.render()
                logger.info(f"模型 {model_name} 可见性设置为: {visible}")
            else:
                logger.warning(f"未找到模型: {model_name}")
        except Exception as e:
            logger.error(f"设置模型可见性失败: {e}")
    
    def get_model_names(self) -> List[str]:
        """获取所有模型名称
        
        Returns:
            List[str]: 模型名称列表
        """
        return [model.name for model in self._current_models]
    
    def set_model_color(self, model_name: str, color: str):
        """设置模型颜色
        
        Args:
            model_name: 模型名称
            color: 颜色名称
        """
        if not self._plotter:
            logger.warning("渲染器未初始化")
            return
        
        try:
            # 更新颜色映射
            self._model_colors[model_name] = color
            
            # 首先尝试从actor映射中快速查找
            target_actor = self._actor_map.get(model_name)
            
            if target_actor:
                # 直接修改actor属性
                self._modify_actor_color(target_actor, color)
                logger.debug(f"通过映射快速修改模型 {model_name} 颜色成功")
            else:
                logger.debug(f"actor映射中未找到 {model_name}，尝试深度查找")
                # 使用更深入的查找方式
                self._find_and_modify_actor_deep(model_name, color)
                
        except Exception as e:
            logger.error(f"设置模型颜色失败: {e}")
            # 使用重建方式作为最后的后备方案
            self._rebuild_model_actor(model_name, color)
    
    def _find_and_modify_actor_deep(self, model_name: str, color: str) -> None:
        """深度查找并修改actor颜色
        
        Args:
            model_name: 模型名称
            color: 颜色名称
        """
        try:
            # 获取所有actors
            actors = self._plotter.renderer.actors
            target_actor = None
            
            # 尝试多种查找方式
            for actor in actors:
                # 方法1: 检查actor的_name属性
                actor_name = getattr(actor, '_name', None)
                if actor_name == model_name:
                    target_actor = actor
                    break
                
                # 方法2: 检查mapper的输入数据名称
                try:
                    mapper = actor.GetMapper()
                    if mapper and hasattr(mapper, 'GetInput'):
                        input_data = mapper.GetInput()
                        if input_data:
                            input_name = getattr(input_data, '_name', None)
                            if input_name == model_name:
                                target_actor = actor
                                break
                except:
                    continue
            
            if target_actor:
                self._modify_actor_color(target_actor, color)
                logger.debug(f"深度查找成功找到模型actor: {model_name}")
            else:
                logger.warning(f"深度查找未找到模型actor: {model_name}，使用重建方式")
                self._rebuild_model_actor(model_name, color)
                
        except Exception as e:
            logger.error(f"深度查找修改actor颜色失败: {e}")
            self._rebuild_model_actor(model_name, color)
    
    def _modify_actor_color(self, actor, color: str) -> None:
        """修改actor颜色
        
        Args:
            actor: PyVista actor对象
            color: 颜色名称
        """
        try:
            # 获取actor的属性对象
            prop = actor.GetProperty()
            # 设置颜色
            prop.SetColor(pv.Color(color).float_rgb)
            
            # 重新渲染
            self._plotter.render()
            logger.debug(f"成功修改actor颜色为: {color}")
            
        except Exception as e:
            logger.warning(f"直接修改actor颜色失败: {e}")
            raise
    
    def _update_model_actor_color(self, model_name: str, color: str):
        """更新模型actor颜色（已弃用，使用set_model_color代替）"""
        # 委托给set_model_color方法
        self.set_model_color(model_name, color)
    
    def _rebuild_model_actor(self, model_name: str, color: str):
        """重建模型actor（作为颜色修改的后备方案）
        
        Args:
            model_name: 模型名称
            color: 颜色名称
        """
        try:
            # 查找对应的模型数据
            model = next((m for m in self._current_models if m.name == model_name), None)
            if not model:
                logger.warning(f"未找到模型数据: {model_name}")
                return
            
            # QtInteractor没有get_actor方法，直接清除所有actors重新添加
            # 这是最后的后备方案
            self._plotter.clear()
            self._plotter.add_axes()
            
            # 清理actor映射，将在重建过程中重新填充
            self._actor_map.clear()
            
            # 重新添加所有模型，但将目标模型改为新颜色
            for current_model in self._current_models:
                model_color = color if current_model.name == model_name else 'gray'
                
                # 添加主模型
                actor = self._plotter.add_mesh(
                    current_model.mesh,
                    color=model_color,
                    show_edges=False,  # 不显示三角形边线
                    smooth_shading=True,  # 启用平滑着色
                    lighting=True,  # 启用光照效果
                    specular=0.3,  # 设置高光反射
                    specular_power=20,  # 设置高光强度
                    opacity=0.5,  # 50%透明度
                    name=current_model.name
                )
                
                # 更新actor映射
                self._actor_map[current_model.name] = actor
                
                # 提取特征边创建轮廓线，增强边界定义
                try:
                    outline_edges = current_model.mesh.extract_feature_edges(
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
                            line_width=1.0,
                            opacity=0.8,  # 轮廓线使用稍高透明度以便看清
                            render_lines_as_tubes=True,
                            name=f"{current_model.name}_outline"
                        )
                        logger.debug(f"为模型 {current_model.name} 添加轮廓线")
                        
                except Exception as outline_error:
                    logger.debug(f"轮廓线添加失败（不影响主模型）: {outline_error}")
            
            # 重新渲染
            self._plotter.render()
            logger.info(f"成功重建所有模型actor，{model_name} 颜色为 {color}")
            
        except Exception as e:
            logger.error(f"重建模型 {model_name} actor失败: {e}")
    
    def reset_all_model_colors(self):
        """重置所有模型颜色为灰色"""
        logger.info("重置所有模型颜色为灰色")
        for model_name in self._model_colors.keys():
            self.set_model_color(model_name, 'gray')
    
    def get_model_colors(self) -> Dict[str, str]:
        """获取当前模型颜色映射
        
        Returns:
            Dict[str, str]: 模型名称到颜色的映射
        """
        return self._model_colors.copy()
    
    def clear_all_models(self) -> None:
        """清除所有模型"""
        try:
            if self._plotter:
                self._plotter.clear()
                self._plotter.add_axes()  # 重新添加坐标轴
            self._current_models.clear()
            self._model_colors.clear()
            self._actor_map.clear()
            self._loaded_model_hashes.clear()  # 清除已加载模型记录
            logger.info("已清除所有模型")
        except Exception as e:
            logger.error(f"清除模型失败: {e}")
    
    def get_visualization_service(self):
        """获取可视化服务接口
        
        让VisualizationWidget能够作为IVisualizationService被ProjectWorkflowManager使用
        
        Returns:
            self: 返回自身作为可视化服务
        """
        return self
    
    def cleanup(self):
        """清理资源"""
        try:
            if self._plotter:
                self._plotter.close()
                self._plotter = None
            logger.info("可视化组件资源清理完成")
        except Exception as e:
            logger.error(f"清理可视化组件资源失败: {e}")