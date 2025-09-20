# MaojocoConverter GUI 开发规范和原则

## 🎯 项目概述

MaojocoConverter GUI 是一个基于 PySide6 + PyVista 的图形界面，为 MaojocoConverter 提供可视化的配置和转换流程控制。

## 📜 开发原则和规矩

### 1. 代码风格原则

#### 1.1 强类型优先
```python
# ✅ 正确：明确的类型注解
from typing import List, Optional, Dict, Any
from pathlib import Path

def load_stl_files(file_paths: List[Path]) -> List[pv.PolyData]:
    """加载STL文件并返回网格数据"""
    pass

# ❌ 避免：运行时类型检查
def load_stl_files(file_paths):
    if not isinstance(file_paths, list):
        raise TypeError("Expected list")
    # 不要这样做！
```

#### 1.2 模块级导入
```python
# ✅ 正确：模块顶部导入
from PySide6.QtWidgets import QMainWindow, QVBoxLayout
from PySide6.QtCore import Qt, Signal
import pyvista as pv

class MainWindow(QMainWindow):
    pass

# ❌ 避免：运行时导入
def some_function():
    import pyvista as pv  # 不要这样做！
```

#### 1.3 禁用动态属性检查
```python
# ✅ 正确：明确检查接口或使用协议
from typing import Protocol

class HasRenderable(Protocol):
    def render(self) -> None:
        pass

def process_component(obj: HasRenderable) -> None:
    obj.render()

# ❌ 避免：hasattr/getattr/isinstance 滥用
def process_component(obj):
    if hasattr(obj, 'render') and callable(getattr(obj, 'render')):
        obj.render()  # 不要这样做！
```

### 2. 架构设计原则

#### 2.1 依赖注入优于全局状态
```python
# ✅ 正确：通过构造函数注入依赖
class STLViewer:
    def __init__(self, renderer: pv.Renderer, config: Config):
        self.renderer = renderer
        self.config = config

# ❌ 避免：全局变量或单例模式
class STLViewer:
    def __init__(self):
        self.renderer = get_global_renderer()  # 不要这样做！
```

#### 2.2 数据驱动设计
```python
# ✅ 正确：配置数据驱动行为
@dataclass
class VisualizationConfig:
    opacity_in_cycle: float = 0.9
    opacity_out_cycle: float = 0.2
    color_in_cycle: str = "red"
    color_out_cycle: str = "gray"

class CycleVisualizer:
    def __init__(self, config: VisualizationConfig):
        self.config = config
```

#### 2.3 明确的错误处理
```python
# ✅ 正确：自定义异常类型
class STLLoadingError(Exception):
    """STL文件加载失败"""
    pass

class CycleDetectionError(Exception):
    """环检测失败"""
    pass

def load_stl(file_path: Path) -> pv.PolyData:
    try:
        return pv.read(file_path)
    except Exception as e:
        raise STLLoadingError(f"无法加载 {file_path}: {e}")
```

### 3. 日志规范

#### 3.1 统一使用logger
```python
# ✅ 正确：使用统一的logger
from ..utils.logger import logger

class STLViewer:
    def load_model(self, file_path: Path) -> None:
        logger.info(f"开始加载模型: {file_path}")
        try:
            # 加载逻辑
            logger.success(f"模型加载成功: {file_path}")
        except Exception as e:
            logger.error(f"模型加载失败: {file_path}, 错误: {e}")

# ❌ 避免：使用print或其他日志方式
class STLViewer:
    def load_model(self, file_path: Path) -> None:
        print(f"Loading model: {file_path}")  # 不要这样做！
```

#### 3.2 日志级别使用规范
```python
# 不同级别的使用场景
logger.debug("详细的调试信息")      # 开发调试
logger.info("一般信息")           # 流程记录
logger.warning("警告信息")        # 可恢复的问题
logger.error("错误信息")          # 需要注意的错误
logger.success("成功信息")        # 操作成功
```

### 4. 导入规范

#### 4.1 标准导入顺序
```python
# 1. 标准库
import sys
from pathlib import Path
from typing import List, Optional

# 2. 第三方库
import pyvista as pv
from PySide6.QtWidgets import QMainWindow
from PySide6.QtCore import Qt, Signal

# 3. 本地模块
from ..core.stl_loader import STLLoader
from ..utils.logger import logger
from ..utils.geometry_utils import GeometryUtils
```

#### 4.2 禁用条件导入
```python
# ✅ 正确：直接导入，依赖环境管理
import pyvista as pv
from PySide6.QtWidgets import QApplication

# ❌ 避免：try-except包裹导入
try:
    import pyvista as pv
except ImportError:
    print("请安装pyvista")
    sys.exit(1)  # 不要这样做！
```

### 5. 文件组织规范

#### 5.1 单一职责原则
```python
# ✅ 正确：每个文件专注一个功能
# stl_loader.py - 专门处理STL文件加载
# cycle_visualizer.py - 专门处理环检测可视化
# link_breaker.py - 专门处理链接断开交互

# ❌ 避免：大杂烩文件
# utils.py - 包含所有功能  # 不要这样做！
```

#### 5.2 清晰的模块边界
```python
# ✅ 正确：明确的API边界
# core/ - 核心业务逻辑
# gui/ - UI组件
# utils/ - 纯工具函数

# 每个模块只暴露必要的公共接口
__all__ = ["STLLoader", "STLLoadingError"]
```

### 6. 文档规范

#### 6.1 完整的类型注解
```python
@dataclass
class ComponentInfo:
    """组件信息
    
    Attributes:
        name: 组件名称
        file_path: STL文件路径
        position: 世界坐标系位置
        is_in_cycle: 是否在环内
    """
    name: str
    file_path: Path
    position: Vector3D
    is_in_cycle: bool
    opacity: float = 1.0
```

#### 6.2 详细的函数文档
```python
def visualize_cycles(
    self, 
    cycles: List[CycleInfo], 
    components: List[ComponentInfo]
) -> None:
    """可视化环检测结果
    
    根据环检测结果，为不同状态的组件设置不同的可视化属性：
    - 环内组件：低透明度，红色高亮
    - 环外组件：高透明度，灰色淡化
    
    Args:
        cycles: 检测到的环信息列表
        components: 所有组件的信息列表
        
    Raises:
        CycleDetectionError: 环检测失败时抛出
        VisualizationError: 可视化失败时抛出
    """
```

### 7. 测试规范

#### 7.1 类型安全的测试
```python
# ✅ 正确：基于类型和行为的测试
def test_stl_loader_with_valid_file():
    """测试有效STL文件的加载"""
    loader = STLLoader()
    result = loader.load(Path("test.stl"))
    
    assert isinstance(result, pv.PolyData)
    assert result.n_points > 0
    assert result.n_faces > 0

# ❌ 避免：依赖实现的测试
def test_stl_loader_uses_pyvista():
    """不关心内部实现，只关心行为"""
    pass
```

## 🔧 开发环境

### 环境管理
- 使用 uv 管理项目依赖和环境
- 如需运行代码，请使用项目虚拟环境：`.venv/bin/python`

### 运行方式
```bash
# 在项目根目录下执行
.venv/bin/python MaojocoConverterGUI/main.py
```

## 🚀 开发目标

### 主要目标
1. **可视化配置**: 通过图形界面配置每个转换步骤
2. **实时预览**: 配置变更时立即显示效果
3. **交互式调试**: 支持环检测可视化和链接断开操作
4. **流程控制**: 逐步执行转换流程

### 技术要求
- 使用 PySide6 + PyVista 构建3D可视化界面
- 强类型设计，编译时类型检查
- 模块化架构，清晰的依赖关系
- 无历史包袱，采用现代Python开发实践

## 📋 开发流程

1. **分析现有流程**: 理解 MaojocoConverter 的每个步骤
2. **识别可配置点**: 找出可以参数化的控制点
3. **设计UI界面**: 基于可配置点设计用户界面
4. **实现可视化**: 为每个步骤提供实时预览
5. **集成测试**: 确保GUI与核心功能的无缝集成