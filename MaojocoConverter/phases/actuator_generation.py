"""
执行器生成阶段

为 MuJoCo 模型生成执行器配置，支持位置执行器（position actuator）。
"""

# TODO(重构): 本阶段的 XML 生成辅助函数（_add_body_to_xml / _add_joint_to_xml /
# _convert_joint_type / _get_mesh_name / _add_assets_to_xml / _add_lighting_to_xml）
# 与 model_generation.py 中几乎逐行重复（本文件仅多了 freejoint / ground / 执行器）。
# 待抽取公共基类消除重复。见 2026-07-14 架构梳理。

from typing import Optional
from pathlib import Path
import xml.etree.ElementTree as ET
from xml.dom import minidom

from ..utils.logger import logger
from ..context import MaojocoContext
from .base import ConversionPhase
from ..type_definitions import (
    KinematicBody, KinematicJoint, KinematicNode, KinematicTree
)
from F3DMaojocoScripts.common.data_types import JointType


class ActuatorGenerationPhase(ConversionPhase):
    """执行器生成阶段"""
    
    def __init__(self, ctx: MaojocoContext):
        super().__init__("ActuatorGeneration", ctx)
    
    def _execute(self) -> bool:
        """执行执行器生成"""
        logger.info("⚡ 开始生成执行器配置")
        
        try:
            # 检查转换后的数据
            if not self.ctx.converted_data or not self.ctx.converted_data.kinematic_tree:
                logger.error("❌ 转换后的数据不存在，无法生成执行器")
                raise RuntimeError("转换后的数据不存在")
            
            # 生成带执行器的 MuJoCo XML
            self._generate_mujoco_xml_with_actuators()
            
            # 保存 XML 文件
            self._save_actuator_xml_file()
            
            logger.info("⚡ 生成执行器配置文件")
            
            return True
            
        except Exception as e:
            logger.error(f"❌ 执行器生成失败: {e}")
            import traceback
            logger.error(traceback.format_exc())
            return False
    
    def _generate_mujoco_xml_with_actuators(self):
        """生成带执行器的 MuJoCo XML 内容"""
        logger.info("⚡ 生成带执行器的 MuJoCo XML 内容")
        
        if not self.ctx.converted_data or not self.ctx.converted_data.kinematic_tree:
            logger.error("❌ 转换后的运动学树不存在，无法生成执行器")
            raise RuntimeError("转换后的运动学树不存在")
        
        kinematic_tree = self.ctx.converted_data.kinematic_tree
        
        # 创建根元素
        root = ET.Element("mujoco", {
            "model": "converted_model_with_actuators"
        })
        
        # 添加选项配置
        option = ET.SubElement(root, "option", {
            "gravity": "0 0 -9.81",  # 标准重力加速度，Z轴向下
            "timestep": "0.001"      # 仿真时间步长
        })
        
        # 添加世界实体
        worldbody = ET.SubElement(root, "worldbody")
        
        # 添加地面
        self._add_ground_to_xml(worldbody)
        
        # 为每个根节点添加到世界实体
        for root_body_id in kinematic_tree['roots']:
            self._add_body_to_xml(worldbody, root_body_id, None, kinematic_tree)
        
        # 添加资产（网格文件引用）
        self._add_assets_to_xml(root)
        
        # 添加光照配置
        self._add_lighting_to_xml(root)
        
        # 添加执行器配置
        self._add_position_actuators_to_xml(root)
        
        # 生成 XML 字符串
        xml_str = ET.tostring(root, encoding='unicode')
        
        # 美化 XML 格式
        dom = minidom.parseString(xml_str)
        pretty_xml = dom.toprettyxml(indent="  ")
        
        self.ctx.actuator_xml_content = pretty_xml
        
        logger.info("⚡ 带执行器的 MuJoCo XML 生成完成")
        
        # 显示执行器统计信息
        actuator_count = len(kinematic_tree.get('joints', {}))
        logger.info("📊 执行器统计信息:")
        logger.info(f"    - 关节数量: {actuator_count}")
        logger.info(f"    - 位置执行器: {actuator_count}")
        
        # 显示执行器列表
        logger.info("📋 执行器列表:")
        for joint_id, joint in kinematic_tree.get('joints', {}).items():
            logger.info(f"    - {joint_id}: {joint.name} ({joint.joint_type.value})")
    
    def _add_body_to_xml(self, parent_xml: ET.Element, body_id: str, _joint_id: Optional[str], kinematic_tree: KinematicTree):
        """添加刚体到 XML"""
        if body_id not in kinematic_tree['bodies']:
            logger.warning(f"⚠️  刚体 {body_id} 不存在，跳过")
            return
        
        body_data: KinematicBody = kinematic_tree['bodies'][body_id]
        
        # 确定使用相对坐标还是绝对坐标
        relative_transforms = kinematic_tree.get('relative_transforms', {})
        
        if body_id in relative_transforms:
            # 使用相对变换
            rel_transform = relative_transforms[body_id]
            if hasattr(rel_transform, 'transform'):
                # 新格式：RelativeTransform 对象
                rel_pos = rel_transform.transform.get_translation()
                rel_quat = rel_transform.transform.to_quaternion()
            else:
                # 旧格式：字典
                transform = rel_transform.get('transform')
                if hasattr(transform, 'get_translation'):
                    rel_pos = transform.get_translation()
                    rel_quat = transform.to_quaternion()
                else:
                    # 回退到绝对坐标
                    rel_pos = body_data.position
                    rel_quat = body_data.quaternion
        else:
            # 根节点使用绝对坐标
            rel_pos = body_data.position
            rel_quat = body_data.quaternion
        
        # 创建刚体元素
        body_attrs = {
            "name": body_data.name,
            "pos": f"{rel_pos.x:.6f} {rel_pos.y:.6f} {rel_pos.z:.6f}",
            "quat": f"{rel_quat.w:.6f} {rel_quat.x:.6f} {rel_quat.y:.6f} {rel_quat.z:.6f}"
        }
        
        body_elem = ET.SubElement(parent_xml, "body", body_attrs)
        
        # 添加惯性属性
        ET.SubElement(body_elem, "inertial", {
            "pos": "0 0 0",
            "mass": str(body_data.mass),
            "diaginertia": f"{body_data.inertia[0]} {body_data.inertia[1]} {body_data.inertia[2]}"
        })
        
        # 添加几何体
        if body_data.stl_file:
            # 为模型添加鲜艳的颜色，与深绿色地面形成对比
            model_color = self._get_model_color(body_data.name)
            ET.SubElement(body_elem, "geom", {
                "type": "mesh",
                "mesh": self._get_mesh_name(body_data.stl_file),
                "pos": "0 0 0",
                "rgba": model_color
            })
        
        # 检查是否为根节点且没有关节 - 如果是，添加 free joint 使其受重力影响
        node_data: KinematicNode = kinematic_tree['nodes'][body_id]
        is_root = body_id in kinematic_tree['roots']
        
        if is_root and not node_data.joint:
            # 为根节点添加 free joint
            logger.info(f"🔓 为根节点 {body_data.name} 添加 free joint")
            ET.SubElement(body_elem, "freejoint", {
                "name": f"{body_data.name}_free"
            })
        elif node_data.joint:
            # 添加普通关节
            joint_data: KinematicJoint = kinematic_tree['joints'][node_data.joint]
            self._add_joint_to_xml(body_elem, joint_data)
        
        # 递归添加子刚体
        for child_body_id in node_data.children:
            self._add_body_to_xml(body_elem, child_body_id, node_data.joint, kinematic_tree)
    
    def _add_joint_to_xml(self, body_xml: ET.Element, joint_data: KinematicJoint):
        """添加关节到 XML"""
        joint_type = self._convert_joint_type(joint_data.joint_type)
        
        # 关节名称已在数据加载阶段转换为拼音，直接使用
        joint_name = joint_data.name
        
        joint_attrs = {
            "name": joint_name,
            "type": joint_type,
            "pos": f"{joint_data.position.x:.6f} {joint_data.position.y:.6f} {joint_data.position.z:.6f}"
        }
        
        # 添加关节轴
        if joint_data.axis:
            axis = joint_data.axis
            joint_attrs["axis"] = f"{axis.x:.6f} {axis.y:.6f} {axis.z:.6f}"
        
        joint_elem = ET.SubElement(body_xml, "joint", joint_attrs)
        
        # 添加关节限制
        if joint_data.limits and joint_data.limits.get('has_limits'):
            limits = joint_data.limits['range']
            ET.SubElement(joint_elem, "range", {
                "min": str(limits[0]),
                "max": str(limits[1])
            })
    
    def _convert_joint_type(self, fusion_type: JointType) -> str:
        """转换 Fusion 360 关节类型到 MuJoCo 关节类型"""
        type_mapping = {
            JointType.REVOLUTE: 'hinge',
            JointType.SLIDER: 'slide',
            JointType.CYLINDRICAL: 'slide',
            JointType.PLANAR: 'slide',
            JointType.BALL: 'ball',
            JointType.RIGID: 'free'
        }
        return type_mapping.get(fusion_type, 'free')
    
    def _get_mesh_name(self, stl_file: str) -> str:
        """从 STL 文件路径获取网格名称"""
        # 提取文件名（不含扩展名）
        path = Path(stl_file)
        return path.stem
    
    def _add_assets_to_xml(self, root: ET.Element):
        """添加资产引用"""
        assets = ET.SubElement(root, "asset")
        
        if not self.ctx.converted_data or not self.ctx.converted_data.kinematic_tree:
            return
        
        kinematic_tree = self.ctx.converted_data.kinematic_tree
        
        # 添加所有 STL 文件作为网格
        stl_files = set()
        for body_data in kinematic_tree['bodies'].values():
            if body_data.stl_file:
                stl_files.add(body_data.stl_file)
        
        for stl_file in stl_files:
            mesh_name = self._get_mesh_name(stl_file)
            ET.SubElement(assets, "mesh", {
                "name": mesh_name,
                "file": stl_file,
                "scale": "0.001 0.001 0.001"
            })
    
    def _get_model_color(self, body_name: str) -> str:
        """返回统一的模型颜色"""
        return "0.7 0.7 0.7 1"  # 浅灰色模型
    
    def _add_ground_to_xml(self, worldbody: ET.Element):
        """添加地面到XML"""
        logger.info("🌍 添加地面")
        
        # 创建地面几何体
        ground_geom = ET.SubElement(worldbody, "geom", {
            "name": "ground",
            "type": "plane",
            "size": "10 10 0.1",  # 10m x 10m 地面，厚度 0.1m
            "pos": "0 0 0",        # 地面位置在原点
            "rgba": "0.2 0.3 0.2 1",  # 深绿色地面
            "friction": "1.0 0.005 0.0001",  # 摩擦系数
            "condim": "3"  # 3D 接触维度
        })
        
        logger.info("   🌍 地面已添加 (10m x 10m, 深绿色)")
    
    def _add_lighting_to_xml(self, root: ET.Element):
        """添加光照配置到XML"""
        logger.info("💡 添加光照配置")
        
        # 创建可视化选项
        visual = ET.SubElement(root, "visual")
        
        # 全局光照设置（仅保留支持的基本属性）
        ET.SubElement(visual, "global", {
            "offwidth": "2048",               # 阴影贴图宽度
            "offheight": "2048"               # 阴影贴图高度
        })
        
        # 添加头灯（MuJoCo支持的主要光源）
        ET.SubElement(visual, "headlight", {
            "ambient": "0.2 0.2 0.2",        # 环境光分量 (RGB)
            "diffuse": "0.8 0.8 0.8",        # 漫反射光分量 (RGB)
            "specular": "0.3 0.3 0.3",       # 镜面反射分量 (RGB)
        })
    
    def _add_position_actuators_to_xml(self, root: ET.Element):
        """添加位置执行器到XML"""
        logger.info("⚡ 添加位置执行器配置")
        
        if not self.ctx.converted_data or not self.ctx.converted_data.kinematic_tree:
            return
        
        kinematic_tree = self.ctx.converted_data.kinematic_tree
        
        # 创建执行器元素
        actuator = ET.SubElement(root, "actuator")
        
        # 为每个关节创建位置执行器
        for joint_id, joint_data in kinematic_tree['joints'].items():
            # 只为有意义的关节类型创建执行器
            if joint_data.joint_type in [JointType.REVOLUTE, JointType.SLIDER]:
                actuator_name = f"{joint_data.name}_actuator"
                
                # 位置执行器配置
                actuator_attrs = {
                    "name": actuator_name,
                    "joint": joint_data.name,  # 关联到关节
                    "kp": "100.0",             # 比例增益 (可配置)
                    "kv": "10.0",              # 速度增益 (可配置)
                    "forcelimited": "true",    # 限制力
                    "forcerange": "-100.0 100.0",  # 力范围 (可配置)
                    "ctrlrange": "-3.14 3.14"  # 控制范围 (±180度，适合旋转关节)
                }
                
                ET.SubElement(actuator, "position", actuator_attrs)
                
                logger.info(f"   ⚡ 添加执行器: {actuator_name} → {joint_data.name}")
            else:
                logger.info(f"   ⚠️  跳过关节 {joint_data.name}: 不支持的执行器类型 {joint_data.joint_type.value}")
        
        logger.info(f"⚡ 位置执行器配置完成")
    
    def _save_actuator_xml_file(self):
        """保存带执行器的 XML 文件"""
        if not self.ctx.actuator_xml_content:
            logger.error("❌ 执行器 XML 内容不存在")
            return
        
        xml_file = self.ctx.export_dir / "model-actuator-position.xml"
        
        try:
            with open(xml_file, 'w', encoding='utf-8') as f:
                f.write(self.ctx.actuator_xml_content)
            
            logger.info(f"⚡ 执行器 XML 文件已保存到: {xml_file}")
            
            # 生成执行器配置说明
            self._generate_actuator_readme()
            
            # 生成交互式查看器
            self._generate_interactive_viewer()
                        
        except Exception as e:
            logger.error(f"❌ 保存执行器 XML 文件失败: {e}")
            raise
    
    def _generate_actuator_readme(self):
        """生成执行器配置说明文件"""
        readme_content = """# MuJoCo 执行器配置说明

## 概述
本目录包含两个 MuJoCo 模型文件：
- `model.xml` - 基础模型文件，包含几何体、关节和光照配置
- `model-actuator-position.xml` - 带位置执行器的模型文件

## 执行器配置
每个可动关节都配置了位置执行器（position actuator），用于控制关节位置。

### 执行器类型
- **类型**: motor with position control（带位置控制的电机）
- **控制模式**: 位置控制（通过 kp 和 kv 参数）
- **增益配置**:
  - 比例增益 (kp): 100.0
  - 速度增益 (kv): 10.0
  - 力限制: ±100.0 N

### 执行器命名规则
执行器名称格式: `{关节名称}_actuator`
例如: `xuan_zhuan_1_actuator` 控制关节 `xuan_zhuan_1`

## 使用方法

### 1. 加载模型
```python
import mujoco

# 加载带执行器的模型
model = mujoco.MjModel.from_xml_path("model-actuator-position.xml")
data = mujoco.MjData(model)
```

### 2. 位置控制
```python
import numpy as np

# 设置目标位置
target_positions = np.array([0.5, -0.3, 0.8, ...])  # 根据关节数量调整
data.ctrl = target_positions

# 步进仿真
mujoco.mj_step(model, data)
```

### 3. 获取关节状态
```python
# 获取当前位置
current_positions = data.qpos

# 获取当前速度
current_velocities = data.qvel

# 获取执行器作用力
actuator_forces = data.qfrc_applied
```

## 配置参数说明

### 执行器参数
- `kp`: 比例增益，控制位置响应强度
- `kv`: 速度增益，提供阻尼
- `forcelimited`: 是否限制执行器输出力
- `forcerange`: 执行器输出力范围

### 自定义配置
可以根据需要修改 `model-actuator-position.xml` 中的执行器参数：
```xml
<motor name="joint_name_actuator" 
       joint="joint_name" 
       kp="200.0"        # 增加响应速度
       kv="20.0"         # 增加阻尼
       forcelimited="true" 
       forcerange="-200.0 200.0"  # 增大力范围
/>
```

## 注意事项
1. 执行器控制需要通过 `data.ctrl` 数组设置目标位置
2. 执行器的数量必须与关节数量匹配
3. 力限制应根据实际应用场景调整
4. 过高的增益可能导致系统不稳定
"""
        
        readme_file = self.ctx.export_dir / "actuator_readme.md"
        
        try:
            with open(readme_file, 'w', encoding='utf-8') as f:
                f.write(readme_content)
            
            logger.info(f"📋 执行器说明文件已生成: {readme_file}")
            
        except Exception as e:
            logger.error(f"❌ 生成执行器说明文件失败: {e}")
            # 不抛出异常，因为这不是致命错误
    
    def _generate_interactive_viewer(self):
        """生成交互式查看器脚本"""
        viewer_content = '''#!/usr/bin/env python3
"""
MuJoCo 简单查看器

简单的 MuJoCo 查看器，用于可视化带执行器的模型。
"""

import mujoco
import mujoco.viewer
import numpy as np
import sys
from pathlib import Path

def main():
    """主函数"""
    # 获取脚本所在目录
    script_dir = Path(__file__).parent
    xml_file = script_dir / "model-actuator-position.xml"
    
    if not xml_file.exists():
        print(f"❌ 未找到XML文件: {xml_file}")
        print("💡 请确保在包含 model-actuator-position.xml 的目录中运行此脚本")
        return 1
    
    try:
        # 加载模型
        model = mujoco.MjModel.from_xml_path(str(xml_file))
        data = mujoco.MjData(model)
        
        print(f"✅ 模型加载成功")
        print(f"📊 执行器数量: {model.nu}")
        print(f"📊 关节数量: {model.njnt}")
        print(f"📊 自由度: {model.nv}")
        
        # 显示执行器信息
        print(f"📋 执行器列表:")
        for i in range(model.nu):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
            print(f"   {i}: {name} -> {joint_name}")
        
        # 启动查看器
        print("\\n🎮 启动 MuJoCo 查看器...")
        print("💡 使用鼠标控制视角，按 ESC 退出")
        
        with mujoco.viewer.launch(model, data) as viewer:
            while viewer.is_running():
                # 步进仿真
                mujoco.mj_step(model, data)
                
                # 同步查看器
                viewer.sync()
        
        print("\\n👋 查看器已关闭")
        return 0
        
    except Exception as e:
        print(f"❌ 运行失败: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    sys.exit(main())
'''
        
        viewer_file = self.ctx.export_dir / "interact_viewer.py"
        
        try:
            with open(viewer_file, 'w', encoding='utf-8') as f:
                f.write(viewer_content)
            
            # 设置可执行权限 (Unix系统)
            try:
                import stat
                viewer_file.chmod(stat.S_IRWXU | stat.S_IRGRP | stat.S_IROTH)
            except:
                pass  # 在Windows系统上忽略权限设置
            
            logger.info(f"🎮 交互式查看器已生成: {viewer_file}")
            logger.info("💡 运行命令: python interact_viewer.py")
            
        except Exception as e:
            logger.error(f"❌ 生成交互式查看器失败: {e}")
            # 不抛出异常，因为这不是致命错误