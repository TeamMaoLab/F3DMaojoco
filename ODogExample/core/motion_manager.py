"""
ODogExample核心模块 - 动作序列管理器

提供动作序列的保存、加载、删除等管理功能。
使用JSON文件进行持久化存储。
"""

import json
import os
import sys
from typing import Dict, List, Optional, Any
from datetime import datetime
import threading
from pathlib import Path

try:
    from .motion_sequence import MotionSequence, Keyframe, create_test_sequence
except ImportError:
    # 如果相对导入失败，尝试绝对导入
    import sys
    import os
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from motion_sequence import MotionSequence, Keyframe, create_test_sequence


class MotionManager:
    """动作序列管理器 - 单例模式"""
    
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
        self._sequences: Dict[str, MotionSequence] = {}
        self._data_file = self._get_data_file_path()
        
        # 确保数据目录存在
        self._ensure_data_dir()
        
        # 加载已有数据
        self._load_sequences()
        
        # 如果没有数据，添加测试序列
        if not self._sequences:
            self._add_default_sequences()
    
    def _get_data_file_path(self) -> str:
        """获取数据文件路径"""
        # 获取当前文件所在目录
        current_dir = os.path.dirname(os.path.abspath(__file__))
        # 返回上一级目录到ODogExample，然后进入data目录
        data_dir = os.path.join(current_dir, '..', 'data')
        return os.path.join(data_dir, 'motions.json')
    
    def _ensure_data_dir(self):
        """确保数据目录存在"""
        data_dir = os.path.dirname(self._data_file)
        if not os.path.exists(data_dir):
            try:
                os.makedirs(data_dir)
                print(f"📁 创建数据目录: {data_dir}")
            except Exception as e:
                print(f"❌ 创建数据目录失败: {e}")
    
    def _load_sequences(self):
        """从文件加载动作序列数据"""
        if not os.path.exists(self._data_file):
            print(f"📄 动作序列数据文件不存在: {self._data_file}")
            return
        
        try:
            with open(self._data_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            self._sequences = {}
            for seq_data in data.get('sequences', []):
                try:
                    sequence = MotionSequence.from_dict(seq_data)
                    self._sequences[sequence.name] = sequence
                except Exception as e:
                    print(f"❌ 加载动作序列失败: {seq_data.get('name', '未知')}, 错误: {e}")
            
            print(f"📂 加载了 {len(self._sequences)} 个动作序列")
            
        except Exception as e:
            print(f"❌ 加载动作序列数据失败: {e}")
            self._sequences = {}
    
    def _save_sequences(self):
        """保存动作序列数据到文件"""
        try:
            data = {
                'version': '1.0',
                'sequences': [seq.to_dict() for seq in self._sequences.values()],
                'last_updated': datetime.now().isoformat()
            }
            
            with open(self._data_file, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            
            print(f"💾 保存了 {len(self._sequences)} 个动作序列到 {self._data_file}")
            
        except Exception as e:
            print(f"❌ 保存动作序列数据失败: {e}")
    
    def _add_default_sequences(self):
        """添加默认动作序列"""
        default_sequences = [
            {
                'name': '站立-趴下-站立',
                'keyframes': [
                    {
                        'pose_name': '站立姿态',
                        'transition_duration': 0.5,
                        'hold_duration': 1.0,
                        'interpolation_type': 'linear'
                    },
                    {
                        'pose_name': '趴下姿态',
                        'transition_duration': 1.0,
                        'hold_duration': 2.0,
                        'interpolation_type': 'smooth'
                    },
                    {
                        'pose_name': '站立姿态',
                        'transition_duration': 1.0,
                        'hold_duration': 1.0,
                        'interpolation_type': 'smooth'
                    }
                ],
                'loop': False
            },
            {
                'name': '简单打招呼',
                'keyframes': [
                    {
                        'pose_name': '站立姿态',
                        'transition_duration': 0.3,
                        'hold_duration': 0.5,
                        'interpolation_type': 'linear'
                    },
                    {
                        'pose_name': '趴下-抬头',
                        'transition_duration': 0.5,
                        'hold_duration': 1.0,
                        'interpolation_type': 'smooth'
                    },
                    {
                        'pose_name': '站立姿态',
                        'transition_duration': 0.5,
                        'hold_duration': 0.5,
                        'interpolation_type': 'smooth'
                    }
                ],
                'loop': True
            }
        ]
        
        for seq_data in default_sequences:
            try:
                sequence = MotionSequence.from_dict(seq_data)
                self._sequences[sequence.name] = sequence
            except Exception as e:
                print(f"❌ 创建默认动作序列失败: {seq_data['name']}, 错误: {e}")
        
        self._save_sequences()
        print(f"🎯 添加了 {len(default_sequences)} 个默认动作序列")
    
    def create_sequence(self, name: str, keyframes: List[Keyframe] = None, 
                       loop: bool = False) -> bool:
        """
        创建新的动作序列
        
        Args:
            name: 动作序列名称
            keyframes: 关键帧列表
            loop: 是否循环
            
        Returns:
            bool: 创建是否成功
        """
        try:
            if not name:
                print("❌ 动作序列名称不能为空")
                return False
            
            if name in self._sequences:
                print(f"❌ 动作序列已存在: {name}")
                return False
            
            # 验证关键帧数据
            if keyframes is None:
                keyframes = []
            
            for i, keyframe in enumerate(keyframes):
                if not isinstance(keyframe, Keyframe):
                    print(f"❌ 关键帧 {i} 数据无效")
                    return False
            
            # 创建动作序列
            sequence = MotionSequence(
                name=name,
                keyframes=keyframes,
                loop=loop
            )
            
            # 验证序列数据
            errors = sequence.validate()
            if errors:
                print(f"❌ 动作序列数据验证失败: {errors}")
                return False
            
            self._sequences[name] = sequence
            self._save_sequences()
            print(f"✅ 创建动作序列: {name}")
            return True
            
        except Exception as e:
            print(f"❌ 创建动作序列失败: {e}")
            return False
    
    def get_sequence(self, name: str) -> Optional[MotionSequence]:
        """
        获取动作序列
        
        Args:
            name: 动作序列名称
            
        Returns:
            MotionSequence: 动作序列，失败返回None
        """
        if name not in self._sequences:
            print(f"❌ 动作序列不存在: {name}")
            return None
        
        # 返回副本，避免外部修改
        return self._sequences[name].copy()
    
    def update_sequence(self, name: str, sequence: MotionSequence) -> bool:
        """
        更新动作序列
        
        Args:
            name: 原动作序列名称
            sequence: 新的动作序列数据
            
        Returns:
            bool: 更新是否成功
        """
        try:
            if name not in self._sequences:
                print(f"❌ 动作序列不存在: {name}")
                return False
            
            if not isinstance(sequence, MotionSequence):
                print("❌ 动作序列数据无效")
                return False
            
            # 验证序列数据
            errors = sequence.validate()
            if errors:
                print(f"❌ 动作序列数据验证失败: {errors}")
                return False
            
            # 如果名称改变，需要删除旧记录
            if sequence.name != name:
                if sequence.name in self._sequences:
                    print(f"❌ 动作序列名称已存在: {sequence.name}")
                    return False
                del self._sequences[name]
            
            self._sequences[sequence.name] = sequence
            self._save_sequences()
            print(f"✅ 更新动作序列: {sequence.name}")
            return True
            
        except Exception as e:
            print(f"❌ 更新动作序列失败: {e}")
            return False
    
    def delete_sequence(self, name: str) -> bool:
        """
        删除动作序列
        
        Args:
            name: 动作序列名称
            
        Returns:
            bool: 删除是否成功
        """
        if name not in self._sequences:
            print(f"❌ 动作序列不存在: {name}")
            return False
        
        try:
            del self._sequences[name]
            self._save_sequences()
            print(f"🗑️  删除动作序列: {name}")
            return True
            
        except Exception as e:
            print(f"❌ 删除动作序列失败: {e}")
            return False
    
    def get_all_sequences(self) -> Dict[str, MotionSequence]:
        """
        获取所有动作序列
        
        Returns:
            Dict[str, MotionSequence]: 所有动作序列的副本
        """
        # 返回副本，避免外部修改
        return {name: seq.copy() for name, seq in self._sequences.items()}
    
    def get_sequence_names(self) -> List[str]:
        """
        获取所有动作序列名称
        
        Returns:
            List[str]: 动作序列名称列表
        """
        return list(self._sequences.keys())
    
    def get_sequence_info(self, name: str) -> Optional[Dict[str, Any]]:
        """
        获取动作序列信息
        
        Args:
            name: 动作序列名称
            
        Returns:
            Dict[str, Any]: 动作序列信息，失败返回None
        """
        if name not in self._sequences:
            return None
        
        sequence = self._sequences[name]
        return {
            'name': sequence.name,
            'keyframe_count': sequence.keyframe_count,
            'total_duration': sequence.total_duration,
            'loop': sequence.loop,
            'created_at': sequence.created_at,
            'updated_at': sequence.updated_at
        }
    
    def get_all_sequence_info(self) -> List[Dict[str, Any]]:
        """
        获取所有动作序列信息
        
        Returns:
            List[Dict[str, Any]]: 所有动作序列信息
        """
        return [self.get_sequence_info(name) for name in self._sequences.keys()]
    
    def rename_sequence(self, old_name: str, new_name: str) -> bool:
        """
        重命名动作序列
        
        Args:
            old_name: 旧名称
            new_name: 新名称
            
        Returns:
            bool: 重命名是否成功
        """
        if old_name not in self._sequences:
            print(f"❌ 动作序列不存在: {old_name}")
            return False
        
        if new_name in self._sequences:
            print(f"❌ 动作序列名称已存在: {new_name}")
            return False
        
        if not new_name:
            print("❌ 新名称不能为空")
            return False
        
        try:
            sequence = self._sequences[old_name]
            sequence.name = new_name
            sequence.updated_at = datetime.now().isoformat()
            
            del self._sequences[old_name]
            self._sequences[new_name] = sequence
            
            self._save_sequences()
            print(f"📝 重命名动作序列: {old_name} -> {new_name}")
            return True
            
        except Exception as e:
            print(f"❌ 重命名动作序列失败: {e}")
            return False
    
    def duplicate_sequence(self, name: str, new_name: str = None) -> bool:
        """
        复制动作序列
        
        Args:
            name: 原动作序列名称
            new_name: 新名称，如果为None则自动生成
            
        Returns:
            bool: 复制是否成功
        """
        if name not in self._sequences:
            print(f"❌ 动作序列不存在: {name}")
            return False
        
        try:
            original = self._sequences[name]
            
            # 生成新名称
            if new_name is None:
                base_name = f"{original.name}_副本"
                counter = 1
                new_name = base_name
                while new_name in self._sequences:
                    new_name = f"{base_name}_{counter}"
                    counter += 1
            
            # 创建副本
            copy_sequence = original.copy()
            copy_sequence.name = new_name
            copy_sequence.created_at = datetime.now().isoformat()
            copy_sequence.updated_at = datetime.now().isoformat()
            
            self._sequences[new_name] = copy_sequence
            self._save_sequences()
            print(f"📋 复制动作序列: {name} -> {new_name}")
            return True
            
        except Exception as e:
            print(f"❌ 复制动作序列失败: {e}")
            return False
    
    def export_sequences(self, file_path: str, names: List[str] = None) -> bool:
        """
        导出动作序列到文件
        
        Args:
            file_path: 导出文件路径
            names: 要导出的序列名称列表，为None时导出所有
            
        Returns:
            bool: 导出是否成功
        """
        try:
            if names is None:
                sequences_to_export = list(self._sequences.values())
            else:
                sequences_to_export = []
                for name in names:
                    if name in self._sequences:
                        sequences_to_export.append(self._sequences[name])
                    else:
                        print(f"⚠️  动作序列不存在: {name}")
            
            data = {
                'version': '1.0',
                'sequences': [seq.to_dict() for seq in sequences_to_export],
                'exported_at': datetime.now().isoformat(),
                'source_file': self._data_file
            }
            
            with open(file_path, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            
            print(f"📤 导出 {len(sequences_to_export)} 个动作序列到 {file_path}")
            return True
            
        except Exception as e:
            print(f"❌ 导出动作序列失败: {e}")
            return False
    
    def import_sequences(self, file_path: str) -> bool:
        """
        从文件导入动作序列
        
        Args:
            file_path: 导入文件路径
            
        Returns:
            bool: 导入是否成功
        """
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            imported_count = 0
            for seq_data in data.get('sequences', []):
                try:
                    sequence = MotionSequence.from_dict(seq_data)
                    
                    # 如果名称冲突，添加后缀
                    original_name = sequence.name
                    counter = 1
                    while sequence.name in self._sequences:
                        sequence.name = f"{original_name}_{counter}"
                        counter += 1
                    
                    self._sequences[sequence.name] = sequence
                    imported_count += 1
                    print(f"✅ 导入动作序列: {sequence.name}")
                    
                except Exception as e:
                    print(f"❌ 导入动作序列失败: {seq_data.get('name', '未知')}, 错误: {e}")
            
            if imported_count > 0:
                self._save_sequences()
                print(f"📥 成功导入 {imported_count} 个动作序列")
            else:
                print("⚠️  没有成功导入任何动作序列")
            
            return imported_count > 0
            
        except Exception as e:
            print(f"❌ 导入动作序列失败: {e}")
            return False
    
    def get_data_file_path(self) -> str:
        """获取数据文件路径"""
        return self._data_file
    
    def reload(self):
        """重新加载数据"""
        self._load_sequences()
        if not self._sequences:
            self._add_default_sequences()
    
    def clear_all(self) -> bool:
        """清空所有数据（危险操作）"""
        try:
            self._sequences.clear()
            self._save_sequences()
            print("🗑️  已清空所有动作序列")
            return True
        except Exception as e:
            print(f"❌ 清空数据失败: {e}")
            return False
    
    def get_statistics(self) -> Dict[str, Any]:
        """获取统计信息"""
        total_sequences = len(self._sequences)
        total_keyframes = sum(seq.keyframe_count for seq in self._sequences.values())
        total_duration = sum(seq.total_duration for seq in self._sequences.values())
        
        return {
            'total_sequences': total_sequences,
            'total_keyframes': total_keyframes,
            'total_duration': total_duration,
            'average_duration': total_duration / total_sequences if total_sequences > 0 else 0,
            'data_file': self._data_file,
            'file_exists': os.path.exists(self._data_file)
        }


# 全局函数，便于其他模块使用
def get_motion_manager() -> MotionManager:
    """获取动作序列管理器实例"""
    return MotionManager()


def create_sequence(name: str, keyframes: List[Keyframe] = None, loop: bool = False) -> bool:
    """创建动作序列（全局函数）"""
    return get_motion_manager().create_sequence(name, keyframes, loop)


def get_sequence(name: str) -> Optional[MotionSequence]:
    """获取动作序列（全局函数）"""
    return get_motion_manager().get_sequence(name)


def get_all_sequences() -> Dict[str, MotionSequence]:
    """获取所有动作序列（全局函数）"""
    return get_motion_manager().get_all_sequences()


if __name__ == "__main__":
    """测试脚本"""
    print("🎯 ODogExample 动作序列管理器测试")
    print("=" * 50)
    
    # 获取管理器
    manager = get_motion_manager()
    
    # 显示统计信息
    stats = manager.get_statistics()
    print(f"📊 统计信息: {stats}")
    
    # 测试创建动作序列
    test_keyframes = [
        Keyframe("默认姿态", 0.5, 1.0),
        Keyframe("站立姿态", 0.8, 2.0, "smooth")
    ]
    
    success = manager.create_sequence("测试序列", test_keyframes, False)
    print(f"创建序列: {'成功' if success else '失败'}")
    
    # 测试获取序列列表
    sequences = manager.get_all_sequences()
    print(f"当前序列数量: {len(sequences)}")
    
    # 测试获取序列信息
    for name in sequences.keys():
        info = manager.get_sequence_info(name)
        if info:
            print(f"序列信息: {info['name']} - {info['keyframe_count']} 个关键帧, {info['total_duration']:.2f}秒")
    
    # 测试复制序列
    success = manager.duplicate_sequence("测试序列")
    print(f"复制序列: {'成功' if success else '失败'}")
    
    # 测试重命名序列
    success = manager.rename_sequence("测试序列_副本", "重命名测试")
    print(f"重命名序列: {'成功' if success else '失败'}")
    
    # 测试删除序列
    success = manager.delete_sequence("重命名测试")
    print(f"删除序列: {'成功' if success else '失败'}")
    
    # 测试序列化
    loaded_sequence = manager.get_sequence("测试序列")
    if loaded_sequence:
        json_data = loaded_sequence.to_json()
        print(f"序列化成功，长度: {len(json_data)} 字符")
    
    print("🎉 动作序列管理器测试完成！")