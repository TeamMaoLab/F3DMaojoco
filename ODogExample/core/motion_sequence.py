"""
ODogExample核心模块 - 动作序列数据模型

提供动作序列和关键帧的数据结构定义。
"""

import json
from typing import List, Dict, Any, Optional
from datetime import datetime
from dataclasses import dataclass, asdict, field


@dataclass
class Keyframe:
    """关键帧数据类"""
    
    pose_name: str               # 引用的姿态名称
    transition_duration: float  # 过渡时长 (秒)
    hold_duration: float        # 保持时长 (秒)
    interpolation_type: str = "linear"  # 插值类型 (linear/smooth)
    
    def __post_init__(self):
        """数据验证"""
        if not self.pose_name:
            raise ValueError("姿态名称不能为空")
        if self.transition_duration < 0:
            raise ValueError("过渡时长不能为负数")
        if self.hold_duration < 0:
            raise ValueError("保持时长不能为负数")
        if self.interpolation_type not in ["linear", "smooth"]:
            raise ValueError("插值类型必须是 'linear' 或 'smooth'")
    
    @property
    def total_duration(self) -> float:
        """关键帧总时长"""
        return self.transition_duration + self.hold_duration
    
    def to_dict(self) -> Dict[str, Any]:
        """转换为字典格式"""
        return asdict(self)
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'Keyframe':
        """从字典创建实例"""
        return cls(**data)


@dataclass
class MotionSequence:
    """动作序列数据类"""
    
    name: str                   # 动作序列名称
    keyframes: List[Keyframe]   # 关键帧列表
    loop: bool = False          # 是否循环
    created_at: str = field(default_factory=lambda: datetime.now().isoformat())
    updated_at: str = field(default_factory=lambda: datetime.now().isoformat())
    
    def __post_init__(self):
        """数据验证"""
        if not self.name:
            raise ValueError("动作序列名称不能为空")
        
        # 验证关键帧数据
        for i, keyframe in enumerate(self.keyframes):
            if not isinstance(keyframe, Keyframe):
                raise ValueError(f"关键帧 {i} 必须是 Keyframe 实例")
    
    @property
    def total_duration(self) -> float:
        """总时长（计算属性）"""
        if not self.keyframes:
            return 0.0
        
        total = 0.0
        for keyframe in self.keyframes:
            total += keyframe.total_duration
        
        return total
    
    @property
    def keyframe_count(self) -> int:
        """关键帧数量"""
        return len(self.keyframes)
    
    def get_keyframe_timestamps(self) -> List[float]:
        """获取所有关键帧的时间戳"""
        timestamps = []
        current_time = 0.0
        
        for keyframe in self.keyframes:
            timestamps.append(current_time)
            current_time += keyframe.total_duration
        
        return timestamps
    
    def get_keyframe_at_time(self, timestamp: float) -> Optional[int]:
        """获取指定时间对应的关键帧索引"""
        if not self.keyframes:
            return None
        
        current_time = 0.0
        for i, keyframe in enumerate(self.keyframes):
            if current_time <= timestamp < current_time + keyframe.total_duration:
                return i
            current_time += keyframe.total_duration
        
        # 如果超出范围，循环模式返回第一个关键帧
        if self.loop and self.keyframes:
            return 0
        
        return None
    
    def add_keyframe(self, keyframe: Keyframe, position: int = -1) -> bool:
        """添加关键帧"""
        if not isinstance(keyframe, Keyframe):
            return False
        
        if position < 0 or position >= len(self.keyframes):
            self.keyframes.append(keyframe)
        else:
            self.keyframes.insert(position, keyframe)
        
        self.updated_at = datetime.now().isoformat()
        return True
    
    def remove_keyframe(self, index: int) -> bool:
        """删除关键帧"""
        if 0 <= index < len(self.keyframes):
            del self.keyframes[index]
            self.updated_at = datetime.now().isoformat()
            return True
        return False
    
    def update_keyframe(self, index: int, keyframe: Keyframe) -> bool:
        """更新关键帧"""
        if 0 <= index < len(self.keyframes) and isinstance(keyframe, Keyframe):
            self.keyframes[index] = keyframe
            self.updated_at = datetime.now().isoformat()
            return True
        return False
    
    def to_dict(self) -> Dict[str, Any]:
        """转换为字典格式"""
        return {
            'name': self.name,
            'keyframes': [kf.to_dict() for kf in self.keyframes],
            'loop': self.loop,
            'created_at': self.created_at,
            'updated_at': self.updated_at
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'MotionSequence':
        """从字典创建实例"""
        keyframes = [Keyframe.from_dict(kf_data) for kf_data in data.get('keyframes', [])]
        
        return cls(
            name=data['name'],
            keyframes=keyframes,
            loop=data.get('loop', False),
            created_at=data.get('created_at', datetime.now().isoformat()),
            updated_at=data.get('updated_at', datetime.now().isoformat())
        )
    
    def to_json(self) -> str:
        """转换为JSON字符串"""
        return json.dumps(self.to_dict(), indent=2, ensure_ascii=False)
    
    @classmethod
    def from_json(cls, json_str: str) -> 'MotionSequence':
        """从JSON字符串创建实例"""
        data = json.loads(json_str)
        return cls.from_dict(data)
    
    def validate(self) -> List[str]:
        """验证动作序列数据，返回错误列表"""
        errors = []
        
        if not self.name:
            errors.append("动作序列名称不能为空")
        
        if not self.keyframes:
            errors.append("动作序列至少需要一个关键帧")
        
        for i, keyframe in enumerate(self.keyframes):
            try:
                # 验证关键帧数据
                Keyframe(
                    pose_name=keyframe.pose_name,
                    transition_duration=keyframe.transition_duration,
                    hold_duration=keyframe.hold_duration,
                    interpolation_type=keyframe.interpolation_type
                )
            except ValueError as e:
                errors.append(f"关键帧 {i}: {str(e)}")
        
        return errors
    
    def copy(self) -> 'MotionSequence':
        """创建动作序列的副本"""
        return MotionSequence.from_dict(self.to_dict())
    
    def __str__(self) -> str:
        """字符串表示"""
        return f"MotionSequence('{self.name}', {len(self.keyframes)} 个关键帧, 总时长: {self.total_duration:.2f}秒)"
    
    def __repr__(self) -> str:
        """调试字符串表示"""
        return f"MotionSequence(name='{self.name}', keyframes={len(self.keyframes)}, loop={self.loop})"


def create_test_sequence() -> MotionSequence:
    """创建测试用的动作序列"""
    keyframes = [
        Keyframe(
            pose_name="默认姿态",
            transition_duration=0.5,
            hold_duration=1.0,
            interpolation_type="linear"
        ),
        Keyframe(
            pose_name="站立姿态",
            transition_duration=0.8,
            hold_duration=2.0,
            interpolation_type="smooth"
        ),
        Keyframe(
            pose_name="趴下姿态",
            transition_duration=1.0,
            hold_duration=1.5,
            interpolation_type="smooth"
        )
    ]
    
    return MotionSequence(
        name="测试动作序列",
        keyframes=keyframes,
        loop=False
    )


if __name__ == "__main__":
    """测试脚本"""
    print("🎯 ODogExample 动作序列数据模型测试")
    print("=" * 50)
    
    # 创建测试序列
    sequence = create_test_sequence()
    print(f"✅ 创建测试序列: {sequence}")
    
    # 测试时间戳计算
    timestamps = sequence.get_keyframe_timestamps()
    print(f"✅ 关键帧时间戳: {timestamps}")
    
    # 测试序列化
    json_data = sequence.to_json()
    print(f"✅ JSON序列化成功，长度: {len(json_data)} 字符")
    
    # 测试反序列化
    sequence2 = MotionSequence.from_json(json_data)
    print(f"✅ JSON反序列化成功: {sequence2}")
    
    # 测试数据验证
    errors = sequence.validate()
    if errors:
        print(f"❌ 数据验证错误: {errors}")
    else:
        print("✅ 数据验证通过")
    
    # 测试关键帧操作
    new_keyframe = Keyframe("测试姿态", 0.3, 0.7)
    sequence.add_keyframe(new_keyframe)
    print(f"✅ 添加关键帧后: {sequence}")
    
    # 测试时间查询
    target_time = 2.0
    frame_index = sequence.get_keyframe_at_time(target_time)
    if frame_index is not None:
        frame = sequence.keyframes[frame_index]
        print(f"✅ 时间 {target_time}s 对应关键帧 {frame_index}: {frame.pose_name}")
    
    print("🎉 动作序列数据模型测试完成！")