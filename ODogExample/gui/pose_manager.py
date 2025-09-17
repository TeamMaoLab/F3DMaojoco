"""
ODogExample GUI模块 - 姿态管理器

提供姿态数据的保存、加载、删除等管理功能。
使用JSON文件进行持久化存储。
"""

import json
import os
import sys
from typing import Dict, List, Optional, Any
from datetime import datetime
import threading
from pathlib import Path


class PoseData:
    """姿态数据类"""
    
    def __init__(self, name: str, joint_angles: Dict[str, float], 
                 description: str = "", tags: List[str] = None):
        self.name = name
        self.joint_angles = joint_angles
        self.description = description
        self.tags = tags or []
        self.created_at = datetime.now().isoformat()
        self.updated_at = datetime.now().isoformat()
    
    def to_dict(self) -> Dict[str, Any]:
        """转换为字典格式"""
        return {
            'name': self.name,
            'joint_angles': self.joint_angles,
            'description': self.description,
            'tags': self.tags,
            'created_at': self.created_at,
            'updated_at': self.updated_at
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'PoseData':
        """从字典创建实例"""
        pose = cls(
            name=data['name'],
            joint_angles=data['joint_angles'],
            description=data.get('description', ''),
            tags=data.get('tags', [])
        )
        pose.created_at = data.get('created_at', pose.created_at)
        pose.updated_at = data.get('updated_at', pose.updated_at)
        return pose
    
    def update_timestamp(self):
        """更新时间戳"""
        self.updated_at = datetime.now().isoformat()


class PoseManager:
    """姿态管理器 - 单例模式"""
    
    _instance = None
    _lock = threading.Lock()
    
    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
        return cls._instance
    
    def __init__(self):
        if hasattr(self, '_initialized'):
            return
        
        self._initialized = True
        self._poses: Dict[str, PoseData] = {}
        self._data_file = self._get_data_file_path()
        
        # 确保数据目录存在
        self._ensure_data_dir()
        
        # 加载已有数据
        self._load_poses()
        
        # 如果没有数据，添加默认姿态
        if not self._poses:
            self._add_default_poses()
    
    def _get_data_file_path(self) -> str:
        """获取数据文件路径"""
        # 获取当前文件所在目录
        current_dir = os.path.dirname(os.path.abspath(__file__))
        # 返回上一级目录到ODogExample，然后进入data目录
        data_dir = os.path.join(current_dir, '..', 'data')
        return os.path.join(data_dir, 'poses.json')
    
    def _ensure_data_dir(self):
        """确保数据目录存在"""
        data_dir = os.path.dirname(self._data_file)
        if not os.path.exists(data_dir):
            try:
                os.makedirs(data_dir)
                print(f"📁 创建数据目录: {data_dir}")
            except Exception as e:
                print(f"❌ 创建数据目录失败: {e}")
    
    def _load_poses(self):
        """从文件加载姿态数据"""
        if not os.path.exists(self._data_file):
            print(f"📄 姿态数据文件不存在: {self._data_file}")
            return
        
        try:
            with open(self._data_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            self._poses = {}
            for pose_data in data.get('poses', []):
                pose = PoseData.from_dict(pose_data)
                self._poses[pose.name] = pose
            
            print(f"📂 加载了 {len(self._poses)} 个姿态")
            
        except Exception as e:
            print(f"❌ 加载姿态数据失败: {e}")
            self._poses = {}
    
    def _save_poses(self):
        """保存姿态数据到文件"""
        try:
            data = {
                'version': '1.0',
                'poses': [pose.to_dict() for pose in self._poses.values()],
                'last_updated': datetime.now().isoformat()
            }
            
            with open(self._data_file, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            
            print(f"💾 保存了 {len(self._poses)} 个姿态到 {self._data_file}")
            
        except Exception as e:
            print(f"❌ 保存姿态数据失败: {e}")
    
    def _add_default_poses(self):
        """添加默认姿态"""
        default_poses = [
            {
                'name': '默认姿态',
                'joint_angles': {
                    'xuan_zhuan_1': 0.0,
                    'xuan_zhuan_2': 0.0,
                    'xuan_zhuan_3': 0.0,
                    'xuan_zhuan_4': 0.0,
                    'tui_1': 0.0,
                    'tui_2': 0.0,
                    'tui_3': 0.0,
                    'tui_4': 0.0
                },
                'description': '所有关节归零的默认姿态',
                'tags': ['default', 'home']
            },
            {
                'name': '站立姿态',
                'joint_angles': {
                    'xuan_zhuan_1': 0.0,
                    'xuan_zhuan_2': 0.0,
                    'xuan_zhuan_3': 0.0,
                    'xuan_zhuan_4': 0.0,
                    'tui_1': -0.5,
                    'tui_2': -0.5,
                    'tui_3': -0.5,
                    'tui_4': -0.5
                },
                'description': '四足站立的基础姿态',
                'tags': ['standing', 'basic']
            },
            {
                'name': '趴下姿态',
                'joint_angles': {
                    'xuan_zhuan_1': 0.0,
                    'xuan_zhuan_2': 0.0,
                    'xuan_zhuan_3': 0.0,
                    'xuan_zhuan_4': 0.0,
                    'tui_1': 0.8,
                    'tui_2': 0.8,
                    'tui_3': 0.8,
                    'tui_4': 0.8
                },
                'description': '四足趴下的休息姿态',
                'tags': ['resting', 'lying']
            }
        ]
        
        for pose_data in default_poses:
            pose = PoseData(**pose_data)
            self._poses[pose.name] = pose
        
        self._save_poses()
        print(f"🎯 添加了 {len(default_poses)} 个默认姿态")
    
    def save_pose(self, name: str, joint_angles: Dict[str, float], 
                  description: str = "", tags: List[str] = None) -> bool:
        """
        保存姿态
        
        Args:
            name: 姿态名称
            joint_angles: 关节角度字典
            description: 姿态描述
            tags: 标签列表
            
        Returns:
            bool: 保存是否成功
        """
        try:
            # 验证关节角度数据
            if not joint_angles:
                print("❌ 关节角度数据为空")
                return False
            
            # 如果姿态已存在，更新它
            if name in self._poses:
                self._poses[name].joint_angles = joint_angles
                self._poses[name].description = description
                self._poses[name].tags = tags or []
                self._poses[name].update_timestamp()
                print(f"🔄 更新姿态: {name}")
            else:
                # 创建新姿态
                pose = PoseData(name, joint_angles, description, tags)
                self._poses[name] = pose
                print(f"💾 保存新姿态: {name}")
            
            # 保存到文件
            self._save_poses()
            return True
            
        except Exception as e:
            print(f"❌ 保存姿态失败: {e}")
            return False
    
    def load_pose(self, name: str) -> Optional[Dict[str, float]]:
        """
        加载姿态
        
        Args:
            name: 姿态名称
            
        Returns:
            Dict[str, float]: 关节角度字典，失败返回None
        """
        if name not in self._poses:
            print(f"❌ 姿态不存在: {name}")
            return None
        
        pose = self._poses[name]
        print(f"📁 加载姿态: {name}")
        return pose.joint_angles.copy()
    
    def delete_pose(self, name: str) -> bool:
        """
        删除姿态
        
        Args:
            name: 姿态名称
            
        Returns:
            bool: 删除是否成功
        """
        if name not in self._poses:
            print(f"❌ 姿态不存在: {name}")
            return False
        
        try:
            del self._poses[name]
            self._save_poses()
            print(f"🗑️  删除姿态: {name}")
            return True
            
        except Exception as e:
            print(f"❌ 删除姿态失败: {e}")
            return False
    
    def get_all_poses(self) -> Dict[str, PoseData]:
        """
        获取所有姿态
        
        Returns:
            Dict[str, PoseData]: 所有姿态数据
        """
        return self._poses.copy()
    
    def get_pose_names(self) -> List[str]:
        """
        获取所有姿态名称
        
        Returns:
            List[str]: 姿态名称列表
        """
        return list(self._poses.keys())
    
    def get_pose_info(self, name: str) -> Optional[Dict[str, Any]]:
        """
        获取姿态信息
        
        Args:
            name: 姿态名称
            
        Returns:
            Dict[str, Any]: 姿态信息，失败返回None
        """
        if name not in self._poses:
            return None
        
        pose = self._poses[name]
        return {
            'name': pose.name,
            'description': pose.description,
            'tags': pose.tags,
            'created_at': pose.created_at,
            'updated_at': pose.updated_at,
            'joint_count': len(pose.joint_angles)
        }
    
    def export_poses(self, file_path: str) -> bool:
        """
        导出姿态数据到文件
        
        Args:
            file_path: 导出文件路径
            
        Returns:
            bool: 导出是否成功
        """
        try:
            data = {
                'version': '1.0',
                'poses': [pose.to_dict() for pose in self._poses.values()],
                'exported_at': datetime.now().isoformat()
            }
            
            with open(file_path, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            
            print(f"📤 导出 {len(self._poses)} 个姿态到 {file_path}")
            return True
            
        except Exception as e:
            print(f"❌ 导出姿态失败: {e}")
            return False
    
    def import_poses(self, file_path: str) -> bool:
        """
        从文件导入姿态数据
        
        Args:
            file_path: 导入文件路径
            
        Returns:
            bool: 导入是否成功
        """
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            imported_count = 0
            for pose_data in data.get('poses', []):
                pose = PoseData.from_dict(pose_data)
                # 如果名称冲突，添加后缀
                original_name = pose.name
                counter = 1
                while pose.name in self._poses:
                    pose.name = f"{original_name}_{counter}"
                    counter += 1
                
                self._poses[pose.name] = pose
                imported_count += 1
            
            self._save_poses()
            print(f"📥 导入了 {imported_count} 个姿态")
            return True
            
        except Exception as e:
            print(f"❌ 导入姿态失败: {e}")
            return False
    
    def get_data_file_path(self) -> str:
        """获取数据文件路径"""
        return self._data_file
    
    def reload(self):
        """重新加载数据"""
        self._load_poses()
        if not self._poses:
            self._add_default_poses()


# 全局函数，便于其他模块使用
def get_pose_manager() -> PoseManager:
    """获取姿态管理器实例"""
    return PoseManager()


def save_pose(name: str, joint_angles: Dict[str, float], 
              description: str = "", tags: List[str] = None) -> bool:
    """保存姿态（全局函数）"""
    return get_pose_manager().save_pose(name, joint_angles, description, tags)


def load_pose(name: str) -> Optional[Dict[str, float]]:
    """加载姿态（全局函数）"""
    return get_pose_manager().load_pose(name)


def get_all_poses() -> Dict[str, PoseData]:
    """获取所有姿态（全局函数）"""
    return get_pose_manager().get_all_poses()


if __name__ == "__main__":
    """测试脚本"""
    print("🎯 ODogExample 姿态管理器测试")
    print("=" * 40)
    
    # 获取姿态管理器
    manager = get_pose_manager()
    
    # 测试保存姿态
    test_pose = {
        'xuan_zhuan_1': 0.5,
        'xuan_zhuan_2': -0.3,
        'tui_1': 0.8,
        'tui_2': -0.2
    }
    
    success = manager.save_pose("测试姿态", test_pose, "这是一个测试姿态", ["test"])
    print(f"保存姿态: {'成功' if success else '失败'}")
    
    # 测试获取姿态列表
    poses = manager.get_all_poses()
    print(f"当前姿态数量: {len(poses)}")
    
    # 测试加载姿态
    loaded_pose = manager.load_pose("测试姿态")
    if loaded_pose:
        print(f"加载姿态成功: {len(loaded_pose)} 个关节")
    
    # 测试删除姿态
    success = manager.delete_pose("测试姿态")
    print(f"删除姿态: {'成功' if success else '失败'}")
    
    print("🎉 姿态管理器测试完成！")