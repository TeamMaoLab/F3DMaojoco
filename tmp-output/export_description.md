# F3DMaojoco 导出描述

## 基本信息
- **导出时间**: 2025-09-13 09:33:11
- **格式版本**: 1.0
- **几何单位**: millimeters
- **位置单位**: millimeters

## 数据统计
- **零部件总数**: 0
- **关节数量**: 0

## 文件结构
```
export_output/
├── component_positions.json      # 主要数据文件（ExportData）
├── stl_files/                    # STL几何文件目录
│   ├── component_1.stl
│   ├── component_2.stl
│   └── ...
├── export_description.md         # 本描述文件
├── f3d_export.log                # 执行日志
└── backup/                       # 备份目录（带时间戳）
    ├── component_positions_*.json
    └── export_description_*.md
```

## 数据格式说明

### ExportData 结构
ExportData 是整个工具链的核心数据结构，包含：

- **meta**: 导出元数据（时间、单位、版本等）
- **components**: 所有零部件的完整信息列表
- **joints**: 所有关节的完整信息列表

### 零部件信息
每个零部件包含：
- **name**: 零部件原始名称
- **occurrence_name**: 装配体实例名称
- **full_path_name**: 装配树完整路径
- **component_id**: 唯一标识符
- **stl_file**: STL文件路径
- **world_transform**: 世界坐标系变换矩阵
- **bodies_count**: 实体数量
- **has_children**: 是否包含子零部件

### 关节信息
每个关节包含：
- **name**: 关节名称
- **joint_type**: 关节类型（rigid/revolute/slider/cylindrical/pin_slot/planar/ball/inferred）
- **connection**: 关节连接的两个零部件信息
- **geometry**: 关节几何变换信息
- **is_suppressed**: 是否被抑制
- **is_light_bulb_on**: 是否激活显示

## 使用说明

### 在 MaojocoConverter 中使用
```python
from F3DMaojocoScripts.common.data_types import load_export_data

# 加载导出数据
export_data = load_export_data("component_positions.json")

# 处理数据...
```

### 工具链流程
```
Fusion 360 → F3DMaojocoScripts → ExportData → MaojocoConverter → MuJoCo
```

## 注意事项
- 所有几何数据使用毫米单位（转换后的统一标准）
- 位置信息使用毫米单位（已从Fusion 360 API的厘米转换为毫米）
- 变换矩阵为4x4数组格式
- 关节类型遵循Fusion 360定义

---
由 F3DMaojocoScripts 自动生成
