"""
模型生成阶段

生成 MuJoCo XML 模型文件。
"""

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


class ModelGenerationPhase(ConversionPhase):
    """模型生成阶段"""
    
    def __init__(self, ctx: MaojocoContext):
        super().__init__("ModelGeneration", ctx)
    
    def _execute(self) -> bool:
        """执行模型生成"""
        logger.info("🏗️  开始生成 MuJoCo 模型")
        
        try:
            # 检查转换后的数据
            if not self.ctx.converted_data or not self.ctx.converted_data.kinematic_tree:
                logger.error("❌ 转换后的数据不存在，无法生成模型")
                raise RuntimeError("转换后的数据不存在")
            
            # 生成 MuJoCo XML
            self._generate_mujoco_xml()
            
            # 保存 XML 文件
            self._save_xml_file()
            
            logger.info("📝 生成 XML 模型文件")
            
            return True
            
        except Exception as e:
            logger.error(f"❌ 模型生成失败: {e}")
            import traceback
            logger.error(traceback.format_exc())
            return False
    
    def _generate_mujoco_xml(self):
        """生成 MuJoCo XML 内容"""
        logger.info("📝 生成 MuJoCo XML 内容")
        
        if not self.ctx.converted_data or not self.ctx.converted_data.kinematic_tree:
            logger.error("❌ 转换后的运动学树不存在，无法生成模型")
            raise RuntimeError("转换后的运动学树不存在")
        
        kinematic_tree = self.ctx.converted_data.kinematic_tree
        
        # 创建根元素
        root = ET.Element("mujoco", {
            "model": "converted_model"
        })
        
        # 添加世界实体
        worldbody = ET.SubElement(root, "worldbody")
        
        # 为每个根节点添加到世界实体
        for root_body_id in kinematic_tree['roots']:
            self._add_body_to_xml(worldbody, root_body_id, None, kinematic_tree)
        
        # 添加资产（网格文件引用）
        self._add_assets_to_xml(root)
        
        # 添加光照配置
        self._add_lighting_to_xml(root)
        
        # 生成 XML 字符串
        xml_str = ET.tostring(root, encoding='unicode')
        
        # 美化 XML 格式
        dom = minidom.parseString(xml_str)
        pretty_xml = dom.toprettyxml(indent="  ")
        
        self.ctx.xml_content = pretty_xml
        
        logger.info("📝 MuJoCo XML 生成完成")
        
        # 显示模型统计信息
        if self.ctx.converted_data and self.ctx.converted_data.kinematic_tree:
            kinematic_tree = self.ctx.converted_data.kinematic_tree
            logger.info("📊 模型统计信息:")
            logger.info(f"    - 刚体数量: {len(kinematic_tree.get('bodies', {}))}")
            logger.info(f"    - 关节数量: {len(kinematic_tree.get('joints', {}))}")
            logger.info(f"    - 根节点数量: {len(kinematic_tree.get('roots', []))}")
            
            # 统计网格文件
            stl_files = set()
            for body_data in kinematic_tree.get('bodies', {}).values():
                if body_data.stl_file:
                    stl_files.add(body_data.stl_file)
            logger.info(f"    - 网格文件数量: {len(stl_files)}")
            
            # 显示刚体列表
            logger.info("📋 刚体列表:")
            for body_id, body in kinematic_tree.get('bodies', {}).items():
                logger.info(f"    - {body_id}: {body.name}")
            
            # 显示关节列表
            logger.info("📋 关节列表:")
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
            ET.SubElement(body_elem, "geom", {
                "type": "mesh",
                "mesh": self._get_mesh_name(body_data.stl_file),
                "pos": "0 0 0"
            })
        
        # 添加关节
        node_data: KinematicNode = kinematic_tree['nodes'][body_id]
        if node_data.joint:
            joint_data: KinematicJoint = kinematic_tree['joints'][node_data.joint]
            self._add_joint_to_xml(body_elem, joint_data)
        
        # 递归添加子刚体
        for child_body_id in node_data.children:
            self._add_body_to_xml(body_elem, child_body_id, node_data.joint, kinematic_tree)
    
    def _add_joint_to_xml(self, body_xml: ET.Element, joint_data: KinematicJoint):
        """添加关节到 XML"""
        joint_type = self._convert_joint_type(joint_data.joint_type)
        
        # 刚性连接不需要关节元素
        if joint_type is None:
            logger.debug(f"刚性连接 {joint_data.name} 跳过关节元素")
            return
        
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
            if joint_data.limits.get('type') == 'ball':
                # 球关节 - MuJoCo不支持球关节的range限制，跳过
                logger.debug(f"球关节 {joint_name} 不需要range限制")
                pass
            else:
                # 旋转关节 - 添加单轴限制
                if joint_data.limits.get('range'):
                    limits = joint_data.limits['range']
                    ET.SubElement(joint_elem, "range", {
                        "min": str(limits[0]),
                        "max": str(limits[1])
                    })
    
    def _convert_joint_type(self, fusion_type: JointType) -> Optional[str]:
        """转换 Fusion 360 关节类型到 MuJoCo 关节类型"""
        type_mapping = {
            JointType.REVOLUTE: 'hinge',
            JointType.SLIDER: 'slide',
            JointType.CYLINDRICAL: 'slide',
            JointType.PLANAR: 'slide',
            JointType.BALL: 'ball',
            JointType.RIGID: None  # 刚性连接不创建关节元素
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
    
    def _save_xml_file(self):
        """保存 XML 文件"""
        if not self.ctx.xml_content:
            logger.error("❌ XML 内容不存在")
            return
        
        xml_file = self.ctx.export_dir / "model.xml"
        
        try:
            with open(xml_file, 'w', encoding='utf-8') as f:
                f.write(self.ctx.xml_content)
            
            logger.info(f"📄 XML 文件已保存到: {xml_file}")
            
            # 生成 MuJoCo 查看器
            self._generate_viewer_file()
                        
        except Exception as e:
            logger.error(f"❌ 保存 XML 文件失败: {e}")
            raise
    
    def _generate_viewer_file(self):
        """生成 MuJoCo 查看器脚本"""
        viewer_template = """#!/usr/bin/env python3
\"\"\"
MuJoCo 查看器启动脚本
用于验证Fusion 360导出的STL文件位置是否正确
\"\"\"

import mujoco
import mujoco.viewer
import os
import json
import numpy as np
import xml.etree.ElementTree as ET
import sys
from pathlib import Path

def load_original_positions():
    \"\"\"加载原始的JSON位置数据用于对比\"\"\"
    script_dir = Path(__file__).parent
    json_path = script_dir / "component_positions.json"
    if json_path.exists():
        try:
            with open(json_path, 'r', encoding='utf-8') as f:
                return json.load(f)
        except Exception as e:
            print(f"⚠️  加载原始位置数据失败: {e}")
    return None

def parse_xml_positions(xml_file):
    \"\"\"从XML文件中解析body的位置和旋转\"\"\"
    try:
        tree = ET.parse(xml_file)
        root = tree.getroot()
        
        positions = {}
        quaternions = {}
        
        # 查找所有body元素
        for body in root.findall('.//body'):
            body_name = body.get('name')
            pos_str = body.get('pos')
            quat_str = body.get('quat')
            
            if body_name and pos_str:
                # 解析位置
                pos = [float(x) for x in pos_str.split()]
                positions[body_name] = np.array(pos)
                
                # 解析四元数
                if quat_str:
                    quat = [float(x) for x in quat_str.split()]
                    quaternions[body_name] = np.array(quat)
        
        return positions, quaternions
    except Exception as e:
        print(f"⚠️  解析XML位置失败: {e}")
        return {}, {}

def load_fusion_positions(original_data):
    \"\"\"从原始Fusion 360数据中提取位置信息\"\"\"
    if not original_data or 'components' not in original_data:
        return {}
    
    fusion_positions = {}
    for component in original_data['components']:
        name = component.get('name')
        transform = component.get('world_transform', {}).get('matrix', [])
        
        if name and transform and len(transform) >= 4:
            # 提取位置向量 (最后一列的前3个元素)
            position = [transform[0][3], transform[1][3], transform[2][3]]
            # 转换为米制单位
            fusion_positions[name] = np.array(position) / 1000.0
    
    return fusion_positions

def compare_positions(fusion_pos, xml_pos, mujoco_pos, name):
    \"\"\"比较不同来源的位置数据\"\"\"
    print(f"🔧 {name}")
    
    # 显示Fusion 360原始位置 (米)
    if name in fusion_pos:
        print(f"   Fusion位置: ({fusion_pos[name][0]:.6f}, {fusion_pos[name][1]:.6f}, {fusion_pos[name][2]:.6f})")
    
    # 显示XML中的位置
    if name in xml_pos:
        print(f"   XML位置:   ({xml_pos[name][0]:.6f}, {xml_pos[name][1]:.6f}, {xml_pos[name][2]:.6f})")
    
    # 显示MuJoCo中的位置
    print(f"   MuJoCo位置: ({mujoco_pos[0]:.6f}, {mujoco_pos[1]:.6f}, {mujoco_pos[2]:.6f})")
    
    # 比较位置差异
    if name in fusion_pos and name in xml_pos:
        diff_fusion_xml = np.linalg.norm(fusion_pos[name] - xml_pos[name])
        print(f"   Fusion-XML差异: {diff_fusion_xml:.6f}m")
    
    if name in xml_pos:
        diff_xml_mujoco = np.linalg.norm(xml_pos[name] - mujoco_pos)
        if diff_xml_mujoco < 1e-6:
            print(f"   ✅ XML-MuJoCo匹配 (差异: {diff_xml_mujoco:.2e}m)")
        else:
            print(f"   ⚠️  XML-MuJoCo差异 (差异: {diff_xml_mujoco:.6f}m)")

def main():
    # 获取当前脚本所在目录
    script_dir = Path(__file__).parent
    xml_file = script_dir / "model.xml"
    
    if not xml_file.exists():
        print(f"❌ 未找到XML文件: {xml_file}")
        print("💡 请确保在包含model.xml的目录中运行此脚本")
        return
    
    try:
        # 解析XML中的位置信息
        xml_positions, xml_quaternions = parse_xml_positions(xml_file)
        
        # 加载原始位置数据
        original_data = load_original_positions()
        fusion_positions = load_fusion_positions(original_data)
        
        # 加载模型
        model = mujoco.MjModel.from_xml_path(str(xml_file))
        data = mujoco.MjData(model)
        
        # 初始化body位置
        print("📊 初始化body位置...")
        initialized_count = 0
        
        for i in range(1, model.nbody):  # 跳过worldbody
            body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
            if body_name and body_name in xml_positions:
                # 找到对应的joint
                joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{body_name}_joint")
                if joint_id >= 0:
                    # Free joint有7个DOF (3 pos + 4 quat)
                    joint_dofadr = model.jnt_dofadr[joint_id]
                    if joint_dofadr >= 0:
                        # 设置位置
                        data.qpos[joint_dofadr:joint_dofadr+3] = xml_positions[body_name]
                        # 设置四元数
                        if body_name in xml_quaternions:
                            data.qpos[joint_dofadr+3:joint_dofadr+7] = xml_quaternions[body_name]
                        
                        print(f"   {body_name}: pos={{xml_positions[body_name]}}, quat={{xml_quaternions.get(body_name, 'N/A')}}")
                        initialized_count += 1
        
        # 前向动力学计算（无重力）
        model.opt.gravity[2] = 0  # 关闭重力
        mujoco.mj_forward(model, data)
        
        print(f"✅ 模型加载成功")
        print(f"  组件数量: {{model.nbody - 1}}")
        print(f"  自由度: {{model.nv}}")
        print(f"  关节数量: {{model.njnt}}")
        print(f"  几何体: {{model.ngeom}}")
        print(f"  已初始化: {{initialized_count}}/{{model.nbody - 1}} 个body")
        print("  (重力已关闭，仅验证初始位置)")
        print()
        
        # 打印组件位置对比信息
        print("📋 组件位置对比:")
        print("=" * 80)
        
        for i in range(1, model.nbody):
            body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
            if body_name:
                body_pos = data.xpos[i]
                body_quat = data.xquat[i]
                
                compare_positions(fusion_positions, xml_positions, body_pos, body_name)
                
                # 显示四元数信息
                if body_name in xml_quaternions:
                    xml_quat = xml_quaternions[body_name]
                    quat_diff = np.linalg.norm(body_quat - xml_quat)
                    print(f"   四元数差异: {quat_diff:.6f}")
                
                print()
        
        print("=" * 80)
        print("🎮 启动交互式查看器...")
        print("💡 操作提示:")
        print("  - 鼠标左键拖拽: 旋转视角")
        print("  - 鼠标右键拖拽: 平移视角")
        print("  - 滚轮: 缩放")
        print("  - 空格键: 暂停/继续仿真")
        print("  - 关闭窗口退出程序")
        print()
        print("📊 位置验证说明:")
        print("  - Fusion位置: 从Fusion 360导出的原始位置(米)")
        print("  - XML位置: model.xml文件中定义的位置")
        print("  - MuJoCo位置: MuJoCo引擎中的实际位置")
        print("  - 理想情况下，这三个位置应该完全匹配")
        
        # 启动查看器
        mujoco.viewer.launch(model, data)
        
    except Exception as e:
        print(f"❌ 运行失败: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
"""
        
        viewer_file = self.ctx.export_dir / "viewer.py"
        
        try:
            with open(viewer_file, 'w', encoding='utf-8') as f:
                f.write(viewer_template)
            
            # 设置执行权限
            import os
            os.chmod(viewer_file, 0o755)
            
            logger.info(f"📋 MuJoCo 查看器已生成: {viewer_file}")
            logger.info("💡 运行命令: python viewer.py")
            
        except Exception as e:
            logger.error(f"❌ 生成查看器文件失败: {e}")
            # 不抛出异常，因为这不是致命错误
    
    def _add_lighting_to_xml(self, root: ET.Element):
        """添加光照配置到XML
        
        为MuJoCo模型添加标准的光照设置，使用MuJoCo支持的元素
        """
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
        
        logger.info("💡 光照配置完成:")
        logger.info("  - 全局阴影贴图: 2048x2048")
        logger.info("  - 头灯环境光: 0.2")
        logger.info("  - 头灯漫反射: 0.8")
        logger.info("  - 头灯镜面反射: 0.3")