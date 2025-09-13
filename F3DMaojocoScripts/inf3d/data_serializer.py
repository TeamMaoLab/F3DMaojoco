"""
数据序列化器

将收集的数据序列化为JSON格式。
创建ExportData和ExportMetadata，生成JSON文件。
"""

import os
import json
import traceback
from datetime import datetime
from typing import Dict, Any

from .logger import log_progress, log_info, log_error
from ..common.data_types import ExportData, ExportMetadata, save_export_data


class DataSerializer:
    """数据序列化器
    
    将收集的数据序列化为JSON格式：
    - 创建 ExportData 和 ExportMetadata
    - 序列化组件和关节数据
    - 生成JSON文件并保存
    - 数据验证和错误检查
    """
    
    def __init__(self, logger):
        """初始化数据序列化器
        
        Args:
            logger: 日志记录器
        """
        self.logger = logger
    
    def serialize_data(self, export_data: ExportData, output_dir: str):
        """序列化导出数据
        
        Args:
            export_data: 导出数据
            output_dir: 输出目录
        """
        try:
            self.logger.info("开始序列化导出数据")
            
            # 确保输出目录存在
            os.makedirs(output_dir, exist_ok=True)
            
            # 保存主要的导出数据文件（无时间戳）
            main_file = os.path.join(output_dir, "component_positions.json")
            self._save_main_data(export_data, main_file)
            
            # 生成导出描述文件（无时间戳）
            desc_file = os.path.join(output_dir, "export_description.md")
            self._generate_description_file(export_data, desc_file)
            
            log_info(self.logger, f"数据序列化完成，文件保存在: {output_dir}")
            
        except Exception as e:
            log_error(self.logger, e, "数据序列化")
            raise
    
    def _save_main_data(self, export_data: ExportData, filepath: str):
        """保存主要的导出数据
        
        Args:
            export_data: 导出数据
            filepath: 文件路径
        """
        try:
            # 使用工具函数保存数据
            success = save_export_data(export_data, filepath, self.logger)
            
            if success:
                log_info(self.logger, f"主要数据文件保存成功: {os.path.basename(filepath)}")
            else:
                raise Exception(f"保存主要数据文件失败: {filepath}")
                
        except Exception as e:
            log_error(self.logger, e, f"保存主要数据文件 {filepath}")
            raise
    
      
    def _generate_description_file(self, export_data: ExportData, filepath: str):
        """生成导出描述文件
        
        Args:
            export_data: 导出数据
            filepath: 文件路径
        """
        try:
            # 创建描述内容
            description = self._create_description_content(export_data)
            
            # 保存文件
            with open(filepath, 'w', encoding='utf-8') as f:
                f.write(description)
            
            log_info(self.logger, f"导出描述文件保存成功: {os.path.basename(filepath)}")
            
        except Exception as e:
            log_error(self.logger, e, f"生成导出描述文件 {filepath}")
            # 描述文件失败不应该中断整个流程
    
    def _create_description_content(self, export_data: ExportData) -> str:
        """创建导出描述内容
        
        Args:
            export_data: 导出数据
            
        Returns:
            str: 描述内容
        """
        try:
            # 基本统计信息
            content = f"""# F3DMaojoco 导出描述

## 基本信息
- **导出时间**: {export_data.meta.export_time}
- **格式版本**: {export_data.meta.format_version}
- **几何单位**: {export_data.meta.geometry_unit}
- **位置单位**: {export_data.meta.position_unit}

## 数据统计
- **零部件总数**: {export_data.meta.count_components}
- **关节数量**: {export_data.meta.count_joints}

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
"""
            
            return content
            
        except Exception as e:
            log_error(self.logger, e, "创建描述内容")
            return f"# F3DMaojoco 导出描述\n\n生成描述文件时发生错误: {str(e)}\n"
    
    def validate_export_data(self, export_data: ExportData) -> Dict[str, Any]:
        """验证导出数据的完整性
        
        Args:
            export_data: 导出数据
            
        Returns:
            Dict[str, Any]: 验证结果
        """
        try:
            validation_result = {
                "is_valid": True,
                "warnings": [],
                "errors": []
            }
            
            # 检查元数据
            if not export_data.meta:
                validation_result["errors"].append("缺少导出元数据")
                validation_result["is_valid"] = False
            
            # 检查零部件数据
            if not export_data.components:
                validation_result["warnings"].append("没有零部件数据")
            
            # 检查零部件完整性
            for i, component in enumerate(export_data.components):
                if not component.name:
                    validation_result["errors"].append(f"零部件 {i} 缺少名称")
                    validation_result["is_valid"] = False
                
                if component.component_id <= 0:
                    validation_result["errors"].append(f"零部件 {component.name} 无效的ID")
                    validation_result["is_valid"] = False
            
            # 检查关节数据
            if not export_data.joints:
                validation_result["warnings"].append("没有关节数据")
            
            # 检查关节完整性
            for i, joint in enumerate(export_data.joints):
                if not joint.name:
                    validation_result["errors"].append(f"关节 {i} 缺少名称")
                    validation_result["is_valid"] = False
                
                if not joint.joint_type:
                    validation_result["errors"].append(f"关节 {joint.name} 缺少类型")
                    validation_result["is_valid"] = False
            
            return validation_result
            
        except Exception as e:
            log_error(self.logger, e, "验证导出数据")
            return {
                "is_valid": False,
                "warnings": [],
                "errors": [f"验证过程中发生错误: {str(e)}"]
            }