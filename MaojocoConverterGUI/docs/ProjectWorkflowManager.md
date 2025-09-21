# ProjectWorkflowManager - 项目工作流管理器

## 概述

`ProjectWorkflowManager` 是 MaojocoConverter GUI 项目的核心控制中心，提供了统一的工作流管理和数据处理功能。它整合了原有的多个独立服务，实现了单一入口点的项目管理模式。

## 核心特性

### 1. 统一的服务管理
- **数据加载服务**: 整合 `IDataLoadingService` 功能
- **可视化服务**: 整合 `IVisualizationService` 功能
- **阶段管理服务**: 整合 `IStageManagementService` 功能
- **变换服务**: 整合 `ITransformService` 功能

### 2. 强类型系统
- **完全类型安全**: 所有字段都有明确的类型定义，避免使用 `Any` 类型
- **ProjectContext**: 统一的项目上下文数据容器
- **数据结构**: `Body4DCoordinates`, `JointGlobalCoordinates`, `KinematicTree` 等
- **类型检查**: 编译时类型安全，减少运行时错误

### 3. 完整的状态管理
- **WorkflowState**: 统一的状态容器
- **工作流状态**: IDLE, LOADING, PROCESSING, COMPLETED, ERROR, PAUSED
- **阶段状态追踪**: 记录每个阶段的执行结果
- **历史记录**: 保存完整的执行历史

### 4. 事件驱动架构
- **Qt信号机制**: 提供丰富的事件通知
- **实时状态更新**: UI层可以实时响应状态变化
- **错误处理**: 统一的错误和警告处理机制

### 5. 灵活的配置系统
- **工厂模式**: 多种创建和配置方式
- **运行时配置**: 支持动态修改阶段配置
- **可扩展设计**: 易于添加新功能和阶段

## 类型系统

ProjectWorkflowManager 使用完全强类型的系统，避免使用 `Any` 类型：

### 核心数据类型

```python
@dataclass
class ProjectContext:
    """项目上下文数据"""
    project_directory: Optional[Path] = None
    export_data: Optional[ExportData] = None
    loaded_models: List[STLModel] = field(default_factory=list)
    body_4d_coordinates: Dict[str, Body4DCoordinates] = field(default_factory=dict)
    joint_global_coordinates: Dict[str, JointGlobalCoordinates] = field(default_factory=dict)
    kinematic_tree: Optional[KinematicTree] = None
    converted_data: Optional[ConvertedData] = None
    xml_content: Optional[str] = None
    processing_options: Dict[str, str] = field(default_factory=dict)
    validation_errors: List[str] = field(default_factory=list)
    warning_messages: List[str] = field(default_factory=list)
```

### 运动学数据类型

```python
@dataclass
class Body4DCoordinates:
    """刚体4D坐标"""
    name: str
    occurrence_name: str
    transform: Transform4D
    stl_file: Optional[str] = None
    bodies_count: int = 1

@dataclass
class JointGlobalCoordinates:
    """关节全局坐标"""
    position: Vector3D
    quaternion: Quaternion
    joint_name: str
    joint_type: JointType

@dataclass
class KinematicTree:
    """运动学树"""
    roots: List[str]
    nodes: Dict[str, KinematicNode]
    joints: Dict[str, KinematicJoint]
    bodies: Dict[str, KinematicBody]
    relative_transforms: Dict[str, RelativeTransform]
```

### 类型安全优势

1. **编译时检查**: 减少运行时类型错误
2. **IDE支持**: 更好的代码补全和重构支持
3. **文档化**: 类型定义本身就是文档
4. **可维护性**: 明确的数据结构便于理解和维护

## 架构设计

```
UI Layer
    ↓
ProjectWorkflowManager (核心控制中心)
    ↓
Core Services (整合后的服务层)
    ↓
Data Layer (数据处理和存储)
```

### 核心类结构

```python
class ProjectWorkflowManager(QObject):
    """项目工作流管理器"""
    
    # 主要组件
    - ProjectState: 状态管理
    - Service聚合: 服务整合
    - Workflow控制: 阶段推进
    - Signal机制: 事件通知
```

## 使用方法

### 1. 基本使用

```python
from core.project_workflow_manager import ProjectWorkflowManager
from core.workflow_manager_factory import WorkflowManagerFactory
from pathlib import Path

# 创建工作流管理器
manager = WorkflowManagerFactory.create_default_manager()

# 加载项目
result = manager.load_project(Path("/path/to/project"))

# 执行单个阶段
result = manager.execute_stage("data_loading")

# 执行完整工作流程
results = manager.execute_workflow()

# 获取状态
state = manager.get_state()
workflow_state = manager.get_workflow_state()
```

### 2. 事件处理

```python
# 连接信号
manager.project_loaded.connect(self._on_project_loaded)
manager.stage_completed.connect(self._on_stage_completed)
manager.workflow_completed.connect(self._on_workflow_completed)
manager.error_occurred.connect(self._on_error_occurred)

# 信号处理函数
def _on_project_loaded(self, result: LoadResult):
    if result.success:
        print(f"项目加载成功: {len(result.models)} 个模型")
    else:
        print(f"项目加载失败: {result.message}")
```

### 3. 自定义配置

```python
from core.workflow_manager_factory import WorkflowManagerConfig

# 创建自定义配置
config = (WorkflowManagerConfig.default()
          .set_data_loading_config({'cache_enabled': True})
          .set_visualization_config({'background_color': '#FFFFFF'})
          .set_stage_config('data_loading', {'auto_validate': True}))

# 使用配置创建管理器
manager = WorkflowManagerFactory.create_with_config(config.to_dict())
```

### 4. UI集成示例

```python
from PySide6.QtWidgets import QMainWindow
from core.project_workflow_manager import ProjectWorkflowManager

class MyMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # 创建工作流管理器
        self.workflow_manager = WorkflowWorkflowManager()
        
        # 连接信号
        self.workflow_manager.status_changed.connect(self.update_status)
        self.workflow_manager.project_loaded.connect(self.on_project_loaded)
        
        # 设置UI
        self.setup_ui()
    
    def load_project(self, path):
        result = self.workflow_manager.load_project(path)
        # UI自动更新，无需手动处理
```

## 阶段定义

ProjectWorkflowManager 预定义了6个工作流程阶段：

1. **initialization** - 初始化阶段
   - 项目选择和验证
   - 目录结构检查

2. **data_loading** - 数据加载阶段
   - STL模型加载
   - 项目数据解析

3. **relationship_analysis** - 关系分析阶段
   - 关节关系分析
   - 运动学树构建

4. **unit_conversion** - 单位转换阶段
   - 坐标系转换
   - 单位标准化

5. **model_generation** - 模型生成阶段
   - MuJoCo模型生成
   - XML配置生成

6. **actuator_generation** - 执行器生成阶段
   - 执行器配置
   - 控制参数设置

## 信号系统

ProjectWorkflowManager 提供了丰富的Qt信号：

### 数据加载信号
- `project_loaded(LoadResult)` - 项目加载完成
- `loading_progress(LoadingProgress)` - 加载进度更新
- `loading_started()` - 开始加载
- `loading_finished()` - 加载完成

### 阶段执行信号
- `stage_started(str)` - 阶段开始
- `stage_progress(str, int, str)` - 阶段进度
- `stage_completed(StageExecutionResult)` - 阶段完成
- `stage_failed(str, str)` - 阶段失败

### 工作流信号
- `workflow_started()` - 工作流开始
- `workflow_completed()` - 工作流完成
- `workflow_paused()` - 工作流暂停
- `workflow_resumed()` - 工作流恢复
- `workflow_failed(str)` - 工作流失败

### 状态信号
- `status_changed(str)` - 状态变更
- `error_occurred(str)` - 错误发生
- `warning_occurred(str)` - 警告发生
- `data_updated(str, Any)` - 数据更新
- `visualization_updated()` - 可视化更新

## 迁移指南

### 从多服务模式迁移

**原有模式：**
```python
# 需要管理多个服务
self.transform_service = TransformService()
self.data_service = ProjectDataService()
self.viz_service = VisualizationWidget()

# 分散的调用
result = self.data_service.load_project(path)
models = self.transform_service.load_models(result.models)
for model in models:
    self.viz_service.add_stl_model(model, model.name)
```

**新模式：**
```python
# 统一的管理器
self.workflow_manager = WorkflowManagerFactory.create_default_manager()

# 单一调用
result = self.workflow_manager.load_project(path)
# 可视化自动更新
```

### 集成步骤

1. **替换服务初始化**
   ```python
   # 原有
   self.transform_service = TransformService()
   self.data_service = ProjectDataService()
   
   # 新
   self.workflow_manager = WorkflowManagerFactory.create_default_manager()
   ```

2. **更新信号连接**
   ```python
   # 原有
   self.data_service.project_loaded.connect(self.on_loaded)
   
   # 新
   self.workflow_manager.project_loaded.connect(self.on_loaded)
   ```

3. **简化方法调用**
   ```python
   # 原有
   result = self.data_service.load_project(path)
   self.process_result(result)
   
   # 新
   result = self.workflow_manager.load_project(path)
   # 处理逻辑内化，无需手动处理
   ```

## 最佳实践

### 1. 状态管理
- 使用 `get_state()` 获取完整状态
- 通过信号监听状态变化
- 避免直接修改内部状态

### 2. 错误处理
- 监听 `error_occurred` 信号
- 检查 `StageExecutionResult.success`
- 提供用户友好的错误信息

### 3. 性能优化
- 使用异步操作避免UI阻塞
- 合理配置服务参数
- 及时清理不需要的资源

### 4. 扩展开发
- 继承 `ProjectWorkflowManager` 添加自定义功能
- 使用工厂模式管理不同的配置
- 保持向后兼容性

## 文件结构

```
core/
├── project_workflow_manager.py      # 主要实现
├── workflow_manager_factory.py      # 工厂类
├── domain_types.py                  # 类型定义
└── service_interfaces.py            # 服务接口

examples/
├── workflow_manager_example.py     # 基础使用示例
└── integration_example.py          # 集成示例
```

## 故障排除

### 常见问题

1. **服务初始化失败**
   - 检查依赖项是否完整
   - 确认服务配置正确

2. **项目加载失败**
   - 验证项目目录结构
   - 检查文件权限

3. **阶段执行失败**
   - 查看错误信息
   - 检查前置条件

4. **UI更新问题**
   - 确认信号连接正确
   - 检查线程安全

### 调试方法

1. **启用详细日志**
   ```python
   import logging
   logging.basicConfig(level=logging.DEBUG)
   ```

2. **使用状态检查**
   ```python
   print(f"工作流状态: {manager.get_workflow_state()}")
   print(f"当前阶段: {manager.get_state().current_stage}")
   ```

3. **检查执行历史**
   ```python
   for result in manager.get_execution_history():
       print(f"{result.stage_name}: {result.success}")
   ```

## 总结

ProjectWorkflowManager 通过统一的服务管理、完整的状态控制和事件驱动的架构，显著提升了 MaojocoConverter GUI 的可维护性和扩展性。它为后续的功能迭代提供了坚实的基础，同时简化了UI层的开发复杂度。

通过采用单一入口点的设计模式，开发者可以更容易地理解系统架构、调试问题和添加新功能。这种设计特别适合需要长期维护和持续发展的复杂项目。