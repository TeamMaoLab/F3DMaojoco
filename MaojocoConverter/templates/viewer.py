#!/usr/bin/env python3
"""
MuJoCo 查看器启动脚本
用于验证Fusion 360导出的STL文件位置是否正确
"""

import mujoco
import mujoco.viewer
import os
import json
import numpy as np
import xml.etree.ElementTree as ET
import sys
from pathlib import Path

def load_original_positions():
    """加载原始的JSON位置数据用于对比"""
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
    """从XML文件中解析body的位置和旋转"""
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
    """从原始Fusion 360数据中提取位置信息"""
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
    """比较不同来源的位置数据"""
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
                        
                        print(f"   {body_name}: pos={xml_positions[body_name]}, quat={xml_quaternions.get(body_name, 'N/A')}")
                        initialized_count += 1
        
        # 前向动力学计算（无重力）
        model.opt.gravity[2] = 0  # 关闭重力
        mujoco.mj_forward(model, data)
        
        print(f"✅ 模型加载成功")
        print(f"  组件数量: {model.nbody - 1}")
        print(f"  自由度: {model.nv}")
        print(f"  关节数量: {model.njnt}")
        print(f"  几何体: {model.ngeom}")
        print(f"  已初始化: {initialized_count}/{model.nbody - 1} 个body")
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