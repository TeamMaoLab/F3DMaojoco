# MaojocoConverter 流程分析和可配置点识别

## 📋 整体流程概览

MaojocoConverter 将 Fusion 360 导出的数据转换为 MuJoCo 模型，包含以下6个主要阶段：

```
Fusion 360 导出数据 → 数据加载 → 关系分析 → 单位转换 → 模型生成 → 执行器生成 → MuJoCo 模型
```

## 🔍 详细流程分析

### 阶段1: 初始化 (Initialize)

#### 当前实现
```python
# 验证导出目录
export_path = Path(export_dir)
if not export_path.exists():
    return ConversionResult(False, f"导出目录不存在: {export_dir}")

# 创建 MaojocoContext 和 PhaseManager
self.ctx = MaojocoContext(str(export_path), self.config)
self.phase_manager = PhaseManager(self.ctx)

# 检查必要文件
required_files = ['component_positions.json', 'export_description.md']
```

#### 可配置点
1. **输入目录选择** - 用户选择包含导出数据的目录
2. **文件验证规则** - 可配置需要检查的文件列表
3. **输出目录设置** - 可设置自定义输出路径
4. **调试模式开关** - 控制日志详细程度

#### UI设计要点
- [ ] 文件夹选择器（输入/输出目录）
- [ ] 文件验证状态显示
- [ ] 调试模式开关
- [ ] 路径预览和验证

---

### 阶段2: 数据加载 (Data Loading)

#### 当前实现
```python
# 加载 Fusion 360 导出数据
export_data = load_export_data(self.ctx.export_dir)

# 提取组件和关节信息
for component in export_data.components:
    body_coord = Body4DCoordinates(
        name=component.name,
        occurrence_name=component.occurrence_name,
        full_path_name=component.full_path_name,
        component_id=component.component_id,
        transform=Transform4D(component.world_transform.matrix),
        stl_file=component.stl_file,
        bodies_count=component.bodies_count,
        has_children=component.has_children
    )
```

#### 可配置点
1. **数据过滤规则** - 根据组件名称、类型、大小等过滤
2. **坐标系转换** - 可配置不同的坐标系转换规则
3. **组件重命名** - 批量重命名规则（中文转拼音等）
4. **STL文件路径映射** - 自定义STL文件路径
5. **数据预览** - 实时预览加载的组件数量和属性

#### UI设计要点
- [ ] 组件列表表格（名称、路径、实体数量等）
- [ ] 过滤条件设置面板
- [ ] 坐标系转换选项
- [ ] 3D预览窗口（显示STL模型）
- [ ] 中文转拼音开关

---

### 阶段3: 关系分析 (Relationship Analysis)

#### 当前实现
```python
# 分析关节两两关系
self._analyze_joint_pairwise_relationships()

# 构建装配关系图
self._build_assembly_graph()

# 生成装配树
self._generate_assembly_trees()

# 分析关节连接关系
self._analyze_joint_connections()
```

#### 可配置点
1. **环检测算法** - 不同的环检测策略和参数
2. **根节点选择策略** - center/max_degree/manual
3. **距离阈值** - 关节间距离判断阈值
4. **连接强度计算** - 不同的连接强度计算方法
5. **装配图构建规则** - 可配置的图构建参数

#### 🔥 核心UI功能：环检测可视化
- [ ] 环检测结果3D可视化
- [ ] 环内/环外组件透明度控制
- [ ] 交互式链接断开功能
- [ ] 根节点手动选择界面
- [ ] 连接强度调节滑块

---

### 阶段4: 单位转换 (Unit Conversion)

#### 当前实现
```python
# 转换单位 (mm to m)
for body_name, body_coord in self.ctx.body_coordinates.items():
    converted_transform = body_coord.transform.scale(0.001)
    # 更新转换后的数据
```

#### 可配置点
1. **单位转换比例** - 可配置的缩放因子
2. **坐标系对齐** - 不同的坐标系对齐方式
3. **旋转矩阵处理** - 可配置的旋转矩阵转换规则
4. **批量变换应用** - 选择性应用变换

#### UI设计要点
- [ ] 单位转换比例输入框
- [ ] 坐标系对齐选项
- [ ] 变换预览（原始 vs 转换后）
- [ ] 批量操作开关

---

### 阶段5: 模型生成 (Model Generation)

#### 当前实现
```python
# 生成 MuJoCo XML 模型
xml_content = self._generate_xml_content()

# 生成基础模型文件
model_file = self.ctx.export_dir / "model.xml"
with open(model_file, 'w', encoding='utf-8') as f:
    f.write(xml_content)
```

#### 可配置点
1. **XML模板选择** - 不同的MuJoCo模型模板
2. **模型参数设置** - 质量、惯性、摩擦系数等
3. **几何体设置** - 碰撞体、可视体设置
4. **关节参数** - 位置限制、阻尼、刚度等
5. **输出格式** - 不同的XML格式和结构

#### UI设计要点
- [ ] XML模板选择器
- [ ] 模型参数表格编辑
- [ ] 实时XML预览
- [ ] 参数验证和提示
- [ ] 模板自定义编辑器

---

### 阶段6: 执行器生成 (Actuator Generation) [可选]

#### 当前实现
```python
# 生成执行器配置
for joint in self.ctx.kinematic_tree.joints.values():
    if joint.joint_type in [JointType.REVOLUTE, JointType.SLIDER]:
        actuator_config = {
            'name': f"{joint.name}_actuator",
            'joint': joint.name,
            'type': 'motor',
            'ctrlrange': [-1, 1],
            'forcerange': [-100, 100]
        }
```

#### 可配置点
1. **执行器类型** - motor/position/velocity/force
2. **控制范围** - ctrlrange, forcerange设置
3. **PID参数** - 比例、积分、微分增益
4. **关节过滤** - 选择哪些关节添加执行器
5. **控制模式** - 位置控制、速度控制、力控制

#### UI设计要点
- [ ] 执行器类型选择
- [ ] 关节列表和多选框
- [ ] 参数范围滑块
- [ ] 实时参数预览
- [ **] 控制模式选择

---

## 🎯 核心UI功能设计

### 1. 主界面布局

```
┌─────────────────────────────────────────────────────────────┐
│ 🚀 MaojocoConverter GUI                                    │
├─────────────────────────────────────────────────────────────┤
│ ┌─────────────┐ ┌─────────────────────────────────────────┐ │
│ │   配置面板   │ │           3D可视化窗口                  │ │
│ │             │ │                                         │ │
│ │ 📁 输入目录  │ │  🔍 环检测可视化                        │ │
│ │ 📁 输出目录  │ │  - 环内组件(不透明)                     │ │
│ │             │ │  - 环外组件(透明)                       │ │
│ │ ⚙️ 流程配置  │ │                                         │ │
│ │ ☐ 数据加载  │ │  🖱️ 点击链接断开                        │ │
│ │ ☐ 关系分析  │ │                                         │ │
│ │ ☐ 单位转换  │ │  [缩放] [旋转] [重置]                  │ │
│ │ ☐ 模型生成  │ │                                         │ │
│ │ ☐ 执行器生成│ │                                         │ │
│ │             │ └─────────────────────────────────────────┘ │
│ │ [🚀 开始]   │                                           │
│ └─────────────┘                                           │
│                                                             │
│ ┌─────────────────────────────────────────────────────────┐ │
│ │                    进度和日志面板                       │ │
│ │                                                        │ │
│ │ 📊 阶段进度: [████████████████████████] 100%           │ │
│ │                                                        │ │
│ │ 📋 处理统计:                                            │ │
│ │ 组件数量: 24    关节数量: 15    环数量: 3              │ │
│ │                                                        │ │
│ │ 💡 操作提示: 点击3D视图中的链接来断开环                │ │
│ └─────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

### 2. 配置参数树形结构

```python
{
    "input": {
        "data_directory": "/path/to/export/data",
        "required_files": ["component_positions.json", "export_description.md"],
        "stl_directory": "stl_files"
    },
    "data_loading": {
        "filter_rules": {
            "name_patterns": [],
            "min_bodies": 1,
            "exclude_hidden": true
        },
        "coordinate_system": {
            "convert_to_pinyin": true,
            "name_mapping": {}
        }
    },
    "relationship_analysis": {
        "cycle_detection": {
            "algorithm": "networkx_simple_cycles",
            "max_cycle_length": 10,
            "distance_threshold": 0.1
        },
        "root_selection": {
            "strategy": "center",  # center, max_degree, manual
            "manual_root": null
        },
        "connection_analysis": {
            "distance_threshold": 0.05,
            "angle_threshold": 30.0
        }
    },
    "unit_conversion": {
        "scale_factor": 0.001,
        "coordinate_alignment": "mujoco",
        "apply_to_all": true
    },
    "model_generation": {
        "template": "base_template.xml",
        "physics": {
            "gravity": [0, 0, -9.81],
            "timestep": 0.002,
            "density": 1000
        },
        "geometry": {
            "convex_hull": true,
            "simplify_mesh": true
        }
    },
    "actuator_generation": {
        "enabled": true,
        "joint_filter": {
            "include_types": ["revolute", "slider"],
            "exclude_names": []
        },
        "actuator_type": "motor",
        "control_range": [-1, 1],
        "force_range": [-100, 100]
    }
}
```

### 3. 交互式环检测可视化

#### 环检测算法配置
- **算法选择**: NetworkX简单环检测、DFS算法、Tarjan算法
- **参数调节**: 最大环长度、距离阈值、连接强度
- **实时预览**: 参数调整时立即更新可视化

#### 可视化控制
- **透明度调节**: 环内组件 0.1-1.0，环外组件 0.1-1.0
- **颜色编码**: 不同环使用不同颜色
- **链接高亮**: 鼠标悬停时高亮显示链接
- **选择模式**: 点击选择要断开的链接

#### 交互操作
- **链接断开**: 点击链接弹出断开确认对话框
- **根节点选择**: 手动选择装配树的根节点
- **缩放控制**: 滚轮缩放，右键拖拽旋转
- **重置视图**: 一键重置到初始视角

## 🚀 开发计划

### 阶段1: 基础框架 (Week 1-2)
1. 创建主窗口和基础布局
2. 实现配置管理器
3. 集成PyVista 3D渲染器
4. 建立与MaojocoConverter的接口

### 阶段2: 数据加载和可视化 (Week 2-3)
1. STL文件加载和显示
2. 组件列表和过滤功能
3. 3D预览和基础交互
4. 配置面板实现

### 阶段3: 关系分析和环检测 (Week 3-4)
1. 环检测算法集成
2. 环检测可视化核心功能
3. 交互式链接断开
4. 根节点选择功能

### 阶段4: 完善和优化 (Week 4-5)
1. 模型生成参数配置
2. 执行器生成配置
3. 用户体验优化
4. 测试和调试

## 💡 技术亮点

1. **实时可视化**: 配置变更时立即显示效果
2. **交互式调试**: 支持手动干预环检测结果
3. **参数化配置**: 所有转换步骤都可配置
4. **强类型设计**: 编译时类型检查，运行时安全
5. **模块化架构**: 清晰的代码组织和依赖关系