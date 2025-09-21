# ProjectWorkflowManager UI集成指南

本指南详细说明如何将ProjectWorkflowManager集成到现有的MaojocoConverterGUI UI模块中。

## 集成策略

### 1. 渐进式集成方法

我们采用**渐进式集成**策略，确保现有功能不受影响的同时逐步引入新的工作流管理。

#### 阶段1：基础集成（最小影响）
- 在现有MainWindow中引入ProjectWorkflowManager
- 保持现有UI组件和功能不变
- 建立信号连接和数据同步

#### 阶段2：功能迁移
- 将业务逻辑逐步迁移到ProjectWorkflowManager
- 简化UI组件，移除重复逻辑
- 统一状态管理

#### 阶段3：优化完善
- 利用工作流管理器增强功能
- 优化用户体验和错误处理
- 完善测试和文档

## 具体集成步骤

### 步骤1：修改现有MainWindow

```python
# 在 gui/main_window.py 中添加ProjectWorkflowManager支持

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # 现有初始化逻辑
        self._transform_service = TransformService()
        self._data_service = ProjectDataService()
        self.quick_start_result: Optional[LoadResult] = None
        
        # 新增：工作流管理器
        self.workflow_manager = WorkflowManagerFactory.create_default_manager()
        self._setup_workflow_manager()
        
        self._setup_ui()
        self._connect_signals()
        
    def _setup_workflow_manager(self):
        """设置工作流管理器"""
        # 配置服务
        self.workflow_manager.set_data_loading_service(self._data_service)
        self.workflow_manager.set_transform_service(self._transform_service)
        
        # 如果有可视化服务，也可以设置
        if hasattr(self, 'viz_widget'):
            self.workflow_manager.set_visualization_service(self.viz_widget)
    
    def _connect_signals(self):
        """连接信号（保持现有连接 + 新增工作流信号）"""
        # 现有信号连接
        # ... 原有的信号连接代码 ...
        
        # 新增：工作流管理器信号连接
        self._connect_workflow_signals()
    
    def _connect_workflow_signals(self):
        """连接工作流管理器信号"""
        self.workflow_manager.project_loaded.connect(self._on_workflow_project_loaded)
        self.workflow_manager.stage_completed.connect(self._on_workflow_stage_completed)
        self.workflow_manager.error_occurred.connect(self._on_workflow_error)
```

### 步骤2：保持现有功能兼容

```python
class MainWindow(QMainWindow):
    # ... 其他代码 ...
    
    def load_project_quick_start(self, directory_path: Path) -> LoadResult:
        """快速启动项目加载（保持兼容）"""
        # 现有逻辑
        result = self._transform_service.load_project_directory(directory_path)
        
        # 新增：同步到工作流管理器
        self.workflow_manager.load_project(directory_path)
        
        return result
    
    def on_preview_requested(self, component_names: List[str]):
        """预览请求处理（保持兼容）"""
        # 现有逻辑
        if hasattr(self, 'viz_widget'):
            self.viz_widget.highlight_components(component_names)
        
        # 新增：通知工作流管理器
        self.workflow_manager.get_context().processing_options['selected_components'] = component_names
```

### 步骤3：逐步迁移业务逻辑

```python
class MainWindow(QMainWindow):
    # ... 其他代码 ...
    
    def execute_current_stage(self):
        """执行当前阶段（迁移到工作流管理器）"""
        # 获取当前阶段
        current_stage = self.stage_panels.get_current_stage()
        
        if current_stage:
            # 使用工作流管理器执行
            result = self.workflow_manager.execute_stage(current_stage)
            
            # 更新UI状态
            if result.success:
                self.status_bar.showMessage(f"阶段 {current_stage} 执行成功")
            else:
                QMessageBox.warning(self, "执行失败", result.error_message)
```

### 步骤4：简化StagePanels

```python
# 在 ui/stage_panels.py 中简化业务逻辑

class StagePanelsContainer(QWidget):
    def __init__(self, workflow_manager: ProjectWorkflowManager):
        super().__init__()
        self.workflow_manager = workflow_manager
        
        # 移除直接的服务依赖，通过工作流管理器统一管理
        # self.transform_service = transform_service  # 移除
        # self.data_service = data_service          # 移除
        
        self._setup_ui()
        self._connect_signals()
    
    def load_project_data(self, project_path: Path, load_result: LoadResult):
        """加载项目数据（简化版）"""
        # 现有UI更新逻辑
        self.data_loading_panel.update_load_result(load_result)
        
        # 新增：通过工作流管理器获取状态
        context = self.workflow_manager.get_context()
        self.update_stage_status(context)
    
    def execute_stage(self, stage_name: str):
        """执行阶段（委托给工作流管理器）"""
        return self.workflow_manager.execute_stage(stage_name)
```

## UI组件适配

### 1. ComponentList适配

```python
class ComponentList(QWidget):
    def __init__(self, workflow_manager: ProjectWorkflowManager):
        super().__init__()
        self.workflow_manager = workflow_manager
        self._setup_ui()
        self._connect_signals()
    
    def update_component_list(self, components: List[ComponentInfo]):
        """更新组件列表（使用工作流管理器数据）"""
        context = self.workflow_manager.get_context()
        
        # 从上下文获取组件数据
        if context.export_data:
            self._refresh_list(context.export_data.components)
    
    def on_component_selected(self, component_name: str, selected: bool):
        """组件选择处理"""
        # 更新上下文
        context = self.workflow_manager.get_context()
        if 'selected_components' not in context.processing_options:
            context.processing_options['selected_components'] = []
        
        if selected:
            if component_name not in context.processing_options['selected_components']:
                context.processing_options['selected_components'].append(component_name)
        else:
            if component_name in context.processing_options['selected_components']:
                context.processing_options['selected_components'].remove(component_name)
        
        # 发出信号
        self.component_selected.emit(component_name, selected)
```

### 2. VisualizationWidget适配

```python
class VisualizationWidget(QWidget):
    def __init__(self, workflow_manager: ProjectWorkflowManager):
        super().__init__()
        self.workflow_manager = workflow_manager
        self._setup_ui()
        self._connect_signals()
    
    def load_models_from_context(self):
        """从上下文加载模型"""
        context = self.workflow_manager.get_context()
        
        # 清除现有模型
        self.clear_all_models()
        
        # 加载新模型
        for model in context.loaded_models:
            self.add_stl_model(model, model.name)
        
        # 调整视角
        self.fit_view_to_models()
    
    def _connect_signals(self):
        """连接信号"""
        self.workflow_manager.project_loaded.connect(self.load_models_from_context)
        self.workflow_manager.visualization_updated.connect(self.refresh_view)
```

## 数据同步策略

### 1. 双向数据同步

```python
class MainWindow(QMainWindow):
    def __init__(self):
        # ... 初始化代码 ...
        
        # 设置数据同步定时器
        self.sync_timer = QTimer()
        self.sync_timer.timeout.connect(self._sync_ui_state)
        self.sync_timer.start(1000)  # 每秒同步一次
    
    def _sync_ui_state(self):
        """同步UI状态"""
        workflow_state = self.workflow_manager.get_state()
        
        # 同步阶段面板状态
        if hasattr(self, 'stage_panels'):
            self.stage_panels.sync_with_workflow_state(workflow_state)
        
        # 同步组件列表
        if hasattr(self, 'component_list'):
            self.component_list.sync_with_context(workflow_state.context)
        
        # 同步状态栏
        self.status_bar.showMessage(
            f"状态: {workflow_state.current_workflow_state.value}"
        )
```

### 2. 事件驱动更新

```python
class MainWindow(QMainWindow):
    def _connect_workflow_signals(self):
        """连接工作流信号"""
        # 数据加载完成
        self.workflow_manager.project_loaded.connect(self._on_project_loaded)
        
        # 阶段状态变化
        self.workflow_manager.stage_started.connect(self._on_stage_started)
        self.workflow_manager.stage_completed.connect(self._on_stage_completed)
        
        # 错误处理
        self.workflow_manager.error_occurred.connect(self._on_error_occurred)
        
        # 状态变更
        self.workflow_manager.status_changed.connect(self._on_status_changed)
    
    def _on_project_loaded(self, result: LoadResult):
        """项目加载完成处理"""
        # 更新各个UI组件
        self.stage_panels.on_project_loaded(result)
        self.component_list.update_from_result(result)
        self.visualization_widget.load_models(result.models)
        
        # 更新状态
        self.status_bar.showMessage(f"项目加载成功: {len(result.models)} 个模型")
```

## 错误处理和用户体验

### 1. 统一错误处理

```python
class MainWindow(QMainWindow):
    def _on_error_occurred(self, error_message: str):
        """统一错误处理"""
        # 显示错误对话框
        QMessageBox.critical(self, "错误", error_message)
        
        # 更新状态栏
        self.status_bar.showMessage(f"错误: {error_message}")
        
        # 记录错误日志
        logger.error(f"工作流错误: {error_message}")
        
        # 更新UI状态
        self._update_error_state()
    
    def _update_error_state(self):
        """更新错误状态UI"""
        workflow_state = self.workflow_manager.get_state()
        
        if workflow_state.current_workflow_state == WorkflowState.ERROR:
            # 显示错误状态
            self.error_indicator.show()
            self.execute_workflow_btn.setEnabled(False)
        else:
            # 隐藏错误状态
            self.error_indicator.hide()
            self.execute_workflow_btn.setEnabled(True)
```

### 2. 进度显示

```python
class MainWindow(QMainWindow):
    def _on_loading_progress(self, progress):
        """显示加载进度"""
        if hasattr(self, 'progress_bar'):
            self.progress_bar.setVisible(True)
            self.progress_bar.setValue(int(progress.progress_percentage))
            self.progress_bar.setFormat(f"加载中: {progress.message}")
    
    def _on_stage_started(self, stage_name: str):
        """阶段开始"""
        if hasattr(self, 'progress_bar'):
            self.progress_bar.setVisible(True)
            self.progress_bar.setRange(0, 0)  # 不确定进度
            self.progress_bar.setFormat(f"执行阶段: {stage_name}")
    
    def _on_stage_completed(self, result: StageExecutionResult):
        """阶段完成"""
        if hasattr(self, 'progress_bar'):
            self.progress_bar.setVisible(False)
```

## 测试策略

### 1. 单元测试

```python
import unittest
from unittest.mock import Mock, patch

class TestUIIntegration(unittest.TestCase):
    def setUp(self):
        self.workflow_manager = Mock(spec=ProjectWorkflowManager)
        self.main_window = MainWindow()
    
    def test_workflow_manager_integration(self):
        """测试工作流管理器集成"""
        # 模拟项目加载
        mock_result = Mock(spec=LoadResult)
        mock_result.success = True
        mock_result.models = []
        
        # 触发信号
        self.main_window.workflow_manager.project_loaded.emit(mock_result)
        
        # 验证UI更新
        self.assertEqual(self.main_window.status_bar.currentMessage(), "项目加载成功: 0 个模型")
    
    def test_error_handling(self):
        """测试错误处理"""
        # 模拟错误
        error_message = "测试错误"
        self.main_window.workflow_manager.error_occurred.emit(error_message)
        
        # 验证错误处理（这里需要根据实际实现调整）
        # self.assert_show_error_dialog(error_message)
```

### 2. 集成测试

```python
class TestWorkflowUIIntegration(unittest.TestCase):
    def test_complete_workflow(self):
        """测试完整工作流程"""
        app = QApplication([])
        
        window = WorkflowAwareMainWindow()
        window.show()
        
        # 模拟项目加载
        test_project = Path("/path/to/test/project")
        window.workflow_manager.load_project(test_project)
        
        # 执行工作流程
        results = window.workflow_manager.execute_workflow()
        
        # 验证结果
        self.assertTrue(all(result.success for result in results))
        
        app.quit()
```

## 部署和回滚策略

### 1. 特性开关

```python
class Config:
    """配置类"""
    ENABLE_WORKFLOW_MANAGER = True  # 特性开关
    WORKFLOW_MANAGER_MODE = "wrapper"  # 集成模式

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        
        if Config.ENABLE_WORKFLOW_MANAGER:
            self.workflow_manager = WorkflowManagerFactory.create_default_manager()
            self._setup_workflow_manager()
        else:
            # 使用传统方式
            self._transform_service = TransformService()
            self._data_service = ProjectDataService()
```

### 2. 回滚机制

```python
class MainWindow(QMainWindow):
    def __init__(self, use_workflow_manager=True):
        super().__init__()
        
        self.use_workflow_manager = use_workflow_manager
        
        if use_workflow_manager:
            self._init_with_workflow_manager()
        else:
            self._init_traditional()
    
    def _init_traditional(self):
        """传统初始化（回滚用）"""
        self._transform_service = TransformService()
        self._data_service = ProjectDataService()
        self._setup_ui_traditional()
```

## 总结

通过以上集成方案，我们可以：

1. **保持兼容性**：现有功能不受影响，可以逐步迁移
2. **提升可维护性**：统一的工作流管理和状态管理
3. **增强扩展性**：基于工作流管理器可以轻松添加新功能
4. **改善用户体验**：统一的错误处理和进度显示
5. **降低风险**：渐进式集成，可以随时回滚

这个集成方案充分考虑了现有项目的复杂性和团队的开发效率，确保在引入新架构的同时保持稳定性和兼容性。