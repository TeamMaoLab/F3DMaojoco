"""
ODogExample核心模块 - 8自由度关节定义和映射

根据model.xml分析，定义和管理四足机器人的8个关节映射关系。
"""

import mujoco
import numpy as np
from typing import Dict, List, Optional, Tuple
import json
import os


class JointMapping:
    """8自由度关节映射管理器"""
    
    def __init__(self):
        """初始化关节映射"""
        # 基于model.xml分析的8个关节定义
        self.joint_definitions = {
            'xuan_zhuan_1': {
                'joint_id': 0,
                'joint_name': 'xuan_zhuan_1',
                'actuator_id': 0,
                'body_name': 'lfu',  # 左前腿上body
                'leg_position': 'left_front',
                'joint_type': 'hip',  # 髋关节
                'description': '左前腿髋关节',
                'movement_range': [-1.57, 1.57],  # ±90度
                'default_angle': 0.0,
                'control_gain': 1.0
            },
            'xuan_zhuan_2': {
                'joint_id': 1,
                'joint_name': 'xuan_zhuan_2',
                'actuator_id': 1,
                'body_name': 'lfd',  # 左前腿下body
                'leg_position': 'left_front',
                'joint_type': 'knee',  # 膝关节
                'description': '左前腿膝关节',
                'movement_range': [-1.57, 1.57],  # ±90度
                'default_angle': 0.0,
                'control_gain': 1.0
            },
            'xuan_zhuan_3': {
                'joint_id': 2,
                'joint_name': 'xuan_zhuan_3',
                'actuator_id': 2,
                'body_name': 'lbu',  # 左后腿上body
                'leg_position': 'left_back',
                'joint_type': 'hip',  # 髋关节
                'description': '左后腿髋关节',
                'movement_range': [-1.57, 1.57],  # ±90度
                'default_angle': 0.0,
                'control_gain': 1.0
            },
            'xuan_zhuan_4': {
                'joint_id': 3,
                'joint_name': 'xuan_zhuan_4',
                'actuator_id': 3,
                'body_name': 'lbd',  # 左后腿下body
                'leg_position': 'left_back',
                'joint_type': 'knee',  # 膝关节
                'description': '左后腿膝关节',
                'movement_range': [-1.57, 1.57],  # ±90度
                'default_angle': 0.0,
                'control_gain': 1.0
            },
            'xuan_zhuan_5': {
                'joint_id': 4,
                'joint_name': 'xuan_zhuan_5',
                'actuator_id': 4,
                'body_name': 'rbu',  # 右后腿上body
                'leg_position': 'right_back',
                'joint_type': 'hip',  # 髋关节
                'description': '右后腿髋关节',
                'movement_range': [-1.57, 1.57],  # ±90度
                'default_angle': 0.0,
                'control_gain': 1.0
            },
            'xuan_zhuan_6': {
                'joint_id': 5,
                'joint_name': 'xuan_zhuan_6',
                'actuator_id': 5,
                'body_name': 'rbd',  # 右后腿下body
                'leg_position': 'right_back',
                'joint_type': 'knee',  # 膝关节
                'description': '右后腿膝关节',
                'movement_range': [-1.57, 1.57],  # ±90度
                'default_angle': 0.0,
                'control_gain': 1.0
            },
            'xuan_zhuan_7': {
                'joint_id': 6,
                'joint_name': 'xuan_zhuan_7',
                'actuator_id': 6,
                'body_name': 'rfu',  # 右前腿上body
                'leg_position': 'right_front',
                'joint_type': 'hip',  # 髋关节
                'description': '右前腿髋关节',
                'movement_range': [-1.57, 1.57],  # ±90度
                'default_angle': 0.0,
                'control_gain': 1.0
            },
            'xuan_zhuan_8': {
                'joint_id': 7,
                'joint_name': 'xuan_zhuan_8',
                'actuator_id': 7,
                'body_name': 'rfd',  # 右前腿下body
                'leg_position': 'right_front',
                'joint_type': 'knee',  # 膝关节
                'description': '右前腿膝关节',
                'movement_range': [-1.57, 1.57],  # ±90度
                'default_angle': 0.0,
                'control_gain': 1.0
            }
        }
        
        # 腿部分组
        self.leg_groups = {
            'left_front': ['xuan_zhuan_1', 'xuan_zhuan_2'],
            'left_back': ['xuan_zhuan_3', 'xuan_zhuan_4'],
            'right_back': ['xuan_zhuan_5', 'xuan_zhuan_6'],
            'right_front': ['xuan_zhuan_7', 'xuan_zhuan_8']
        }
        
        # 关节类型分组
        self.joint_types = {
            'hip': ['xuan_zhuan_1', 'xuan_zhuan_3', 'xuan_zhuan_5', 'xuan_zhuan_7'],
            'knee': ['xuan_zhuan_2', 'xuan_zhuan_4', 'xuan_zhuan_6', 'xuan_zhuan_8']
        }
        
        # 关节显示名称（用于UI）
        self.display_names = {
            'xuan_zhuan_1': '左前腿髋关节',
            'xuan_zhuan_2': '左前腿膝关节',
            'xuan_zhuan_3': '左后腿髋关节',
            'xuan_zhuan_4': '左后腿膝关节',
            'xuan_zhuan_5': '右后腿髋关节',
            'xuan_zhuan_6': '右后腿膝关节',
            'xuan_zhuan_7': '右前腿髋关节',
            'xuan_zhuan_8': '右前腿膝关节'
        }
        
        print(f"🔗 关节映射初始化完成：{len(self.joint_definitions)} 个关节")
        self._print_joint_summary()
    
    def _print_joint_summary(self):
        """打印关节汇总信息"""
        print("📋 8自由度关节配置：")
        print("   左前腿：髋关节(xuan_zhuan_1) + 膝关节(xuan_zhuan_2)")
        print("   左后腿：髋关节(xuan_zhuan_3) + 膝关节(xuan_zhuan_4)")
        print("   右后腿：髋关节(xuan_zhuan_5) + 膝关节(xuan_zhuan_6)")
        print("   右前腿：髋关节(xuan_zhuan_7) + 膝关节(xuan_zhuan_8)")
        print(f"   运动范围：±90° (±{np.pi/2:.2f} rad)")
    
    def get_joint_info(self, joint_name: str) -> Optional[Dict]:
        """
        获取关节信息
        
        Args:
            joint_name: 关节名称
            
        Returns:
            Dict: 关节信息字典，不存在则返回None
        """
        return self.joint_definitions.get(joint_name)
    
    def get_joint_by_id(self, joint_id: int) -> Optional[Dict]:
        """
        根据ID获取关节信息
        
        Args:
            joint_id: 关节ID
            
        Returns:
            Dict: 关节信息字典，不存在则返回None
        """
        for joint_info in self.joint_definitions.values():
            if joint_info['joint_id'] == joint_id:
                return joint_info
        return None
    
    def get_actuator_joint(self, actuator_id: int) -> Optional[Dict]:
        """
        根据执行器ID获取关节信息
        
        Args:
            actuator_id: 执行器ID
            
        Returns:
            Dict: 关节信息字典，不存在则返回None
        """
        for joint_info in self.joint_definitions.values():
            if joint_info['actuator_id'] == actuator_id:
                return joint_info
        return None
    
    def get_leg_joints(self, leg_position: str) -> List[str]:
        """
        获取指定腿部的所有关节
        
        Args:
            leg_position: 腿部位置 ('left_front', 'left_back', 'right_back', 'right_front')
            
        Returns:
            List[str]: 关节名称列表
        """
        return self.leg_groups.get(leg_position, [])
    
    def get_joints_by_type(self, joint_type: str) -> List[str]:
        """
        根据类型获取关节
        
        Args:
            joint_type: 关节类型 ('hip', 'knee')
            
        Returns:
            List[str]: 关节名称列表
        """
        return self.joint_types.get(joint_type, [])
    
    def get_display_name(self, joint_name: str) -> str:
        """
        获取关节显示名称
        
        Args:
            joint_name: 关节名称
            
        Returns:
            str: 显示名称
        """
        return self.display_names.get(joint_name, joint_name)
    
    def validate_joint_angle(self, joint_name: str, angle: float) -> Tuple[bool, float]:
        """
        验证并限制关节角度
        
        Args:
            joint_name: 关节名称
            angle: 输入角度（弧度）
            
        Returns:
            Tuple[bool, float]: (是否有效, 限制后的角度)
        """
        joint_info = self.get_joint_info(joint_name)
        if not joint_info:
            return False, angle
        
        min_angle, max_angle = joint_info['movement_range']
        
        # 限制角度范围
        clamped_angle = max(min_angle, min(max_angle, angle))
        
        # 检查是否需要限制
        is_valid = (min_angle <= angle <= max_angle)
        
        return is_valid, clamped_angle
    
    def get_all_joint_names(self) -> List[str]:
        """
        获取所有关节名称
        
        Returns:
            List[str]: 所有关节名称列表
        """
        return list(self.joint_definitions.keys())
    
    def get_ordered_joint_names(self) -> List[str]:
        """
        获取按ID排序的关节名称
        
        Returns:
            List[str]: 排序后的关节名称列表
        """
        return sorted(self.joint_definitions.keys(), 
                     key=lambda name: self.joint_definitions[name]['joint_id'])
    
    def get_default_pose(self) -> Dict[str, float]:
        """
        获取默认姿态（所有关节角度归零）
        
        Returns:
            Dict[str, float]: 关节名称到角度的映射
        """
        return {name: info['default_angle'] for name, info in self.joint_definitions.items()}
    
    def get_symmetric_joint(self, joint_name: str) -> Optional[str]:
        """
        获取对称关节
        
        Args:
            joint_name: 关节名称
            
        Returns:
            Optional[str]: 对称关节名称，不存在则返回None
        """
        joint_info = self.get_joint_info(joint_name)
        if not joint_info:
            return None
        
        leg_position = joint_info['leg_position']
        joint_type = joint_info['joint_type']
        
        # 对称映射
        symmetry_map = {
            'left_front': 'right_front',
            'left_back': 'right_back',
            'right_back': 'left_back',
            'right_front': 'left_front'
        }
        
        symmetric_leg = symmetry_map.get(leg_position)
        if not symmetric_leg:
            return None
        
        # 查找对称关节
        for j_name, j_info in self.joint_definitions.items():
            if (j_info['leg_position'] == symmetric_leg and 
                j_info['joint_type'] == joint_type):
                return j_name
        
        return None
    
    def to_dict(self) -> Dict:
        """
        转换为字典格式
        
        Returns:
            Dict: 完整的映射数据
        """
        return {
            'joint_definitions': self.joint_definitions,
            'leg_groups': self.leg_groups,
            'joint_types': self.joint_types,
            'display_names': self.display_names
        }
    
    def save_to_file(self, filepath: str) -> bool:
        """
        保存映射配置到文件
        
        Args:
            filepath: 文件路径
            
        Returns:
            bool: 保存是否成功
        """
        try:
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(self.to_dict(), f, indent=2, ensure_ascii=False)
            print(f"💾 关节映射配置已保存到: {filepath}")
            return True
        except Exception as e:
            print(f"❌ 保存关节映射配置失败: {e}")
            return False
    
    def print_joint_mapping(self):
        """打印完整的关节映射信息"""
        print("=== ODogExample 8自由度关节映射 ===")
        for joint_name, info in self.joint_definitions.items():
            print(f"{joint_name} ({info['description']}):")
            print(f"  关节ID: {info['joint_id']}")
            print(f"  执行器ID: {info['actuator_id']}")
            print(f"  腿部位置: {info['leg_position']}")
            print(f"  关节类型: {info['joint_type']}")
            print(f"  运动范围: {info['movement_range']} rad")
            print(f"  默认角度: {info['default_angle']} rad")
            print("  " + "-" * 30)
        print("=" * 40)


def create_joint_mapping() -> JointMapping:
    """
    创建关节映射实例
    
    Returns:
        JointMapping: 关节映射实例
    """
    return JointMapping()


if __name__ == "__main__":
    """测试脚本"""
    print("🔗 ODogExample 关节映射测试")
    print("=" * 40)
    
    # 创建关节映射
    mapping = create_joint_mapping()
    
    # 测试基本功能
    print("\n📋 测试关节信息获取:")
    test_joint = 'xuan_zhuan_1'
    info = mapping.get_joint_info(test_joint)
    if info:
        print(f"✅ 成功获取 {test_joint} 信息: {info['description']}")
    
    # 测试角度验证
    print("\n📐 测试角度验证:")
    test_angles = [0.0, 1.0, 2.0, -1.0, -2.0]
    for angle in test_angles:
        is_valid, clamped = mapping.validate_joint_angle(test_joint, angle)
        print(f"  {angle:.2f} rad -> 有效: {is_valid}, 限制后: {clamped:.2f} rad")
    
    # 测试对称关节
    print("\n🔄 测试对称关节:")
    for joint in mapping.get_all_joint_names():
        symmetric = mapping.get_symmetric_joint(joint)
        print(f"  {joint} -> {symmetric}")
    
    # 测试分组
    print("\n📦 测试分组功能:")
    print(f"  髋关节: {mapping.get_joints_by_type('hip')}")
    print(f"  膝关节: {mapping.get_joints_by_type('knee')}")
    print(f"  左前腿: {mapping.get_leg_joints('left_front')}")
    
    # 保存配置
    print("\n💾 保存配置文件:")
    config_path = "joint_mapping_config.json"
    if mapping.save_to_file(config_path):
        print(f"✅ 配置已保存到 {config_path}")
    
    print("\n🎉 关节映射测试完成！")