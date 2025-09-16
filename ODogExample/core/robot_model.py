"""
ODogExample核心模块 - 机器人模型封装

提供MuJoCo模型的加载、管理和基础操作功能。
"""

import mujoco
import numpy as np
from typing import Optional, Dict, Any
import os


class RobotModel:
    """MuJoCo机器人模型封装类"""
    
    def __init__(self, model_path: Optional[str] = None):
        """
        初始化机器人模型
        
        Args:
            model_path: MuJoCo模型文件路径，如果为None则后续需要手动加载
        """
        self.model: Optional[mujoco.MjModel] = None
        self.data: Optional[mujoco.MjData] = None
        self.model_path: Optional[str] = None
        
        # 模型信息
        self.joint_names: list = []
        self.joint_ids: Dict[str, int] = {}
        self.nq: int = 0  # 广义坐标数量
        self.nv: int = 0  # 广义速度数量
        self.nu: int = 0  # 控制输入数量
        
        # 统计信息
        self.model_stats: Dict[str, Any] = {}
        
        if model_path:
            self.load_model(model_path)
    
    def load_model(self, model_path: str) -> bool:
        """
        加载MuJoCo模型
        
        Args:
            model_path: 模型文件路径
            
        Returns:
            bool: 加载是否成功
        """
        try:
            if not os.path.exists(model_path):
                print(f"❌ 模型文件不存在: {model_path}")
                return False
            
            print(f"📂 加载模型: {model_path}")
            
            # 加载模型
            self.model = mujoco.MjModel.from_xml_path(model_path)
            self.data = mujoco.MjData(self.model)
            self.model_path = model_path
            
            # 初始化模型状态
            mujoco.mj_forward(self.model, self.data)
            
            # 提取模型信息
            self._extract_model_info()
            self._calculate_model_stats()
            
            print(f"✅ 模型加载成功")
            print(f"   关节数量: {len(self.joint_names)}")
            print(f"   广义坐标: {self.nq}")
            print(f"   广义速度: {self.nv}")
            print(f"   控制输入: {self.nu}")
            
            return True
            
        except Exception as e:
            print(f"❌ 模型加载失败: {e}")
            return False
    
    def _extract_model_info(self):
        """提取模型基本信息"""
        if not self.model:
            return
        
        # 提取关节信息
        self.joint_names = []
        self.joint_ids = {}
        
        for i in range(self.model.njnt):
            joint_name = self.model.joint(i).name
            joint_id = self.model.joint(i).id
            
            self.joint_names.append(joint_name)
            self.joint_ids[joint_name] = joint_id
        
        # 获取模型维度
        self.nq = self.model.nq
        self.nv = self.model.nv
        self.nu = self.model.nu
    
    def _calculate_model_stats(self):
        """计算模型统计信息 - 优先使用手动计算以排除地面"""
        if not self.model or not self.data:
            return
        
        try:
            # 手动计算模型边界，排除地面等大型几何体
            geom_positions = []
            geom_count = 0
            large_geom_count = 0
            
            for i in range(self.model.ngeom):
                geom = self.model.geom(i)
                geom_pos = geom.pos.copy()
                geom_size = geom.size.copy()
                
                geom_count += 1
                
                # 跳过过大的几何体（可能是地面）
                if np.any(geom_size > 2.0):  # 大于2米的几何体可能是地面
                    large_geom_count += 1
                    continue
                
                geom_positions.append(geom_pos)
            
            print(f"📊 几何体统计: 总数={geom_count}, 大型几何体={large_geom_count}, 有效几何体={len(geom_positions)}")
            
            if geom_positions:
                geom_positions = np.array(geom_positions)
                min_pos = np.min(geom_positions, axis=0)
                max_pos = np.max(geom_positions, axis=0)
                center = (min_pos + max_pos) / 2
                extent = np.max(max_pos - min_pos)
                
                print(f"📏 手动计算模型尺寸: {extent:.3f}m")
                print(f"🎯 手动计算模型中心: [{center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f}]")
            else:
                # 如果没有有效几何体，使用body位置
                body_positions = []
                for i in range(1, min(self.model.nbody, 10)):  # 跳过worldbody，限制数量
                    body_pos = self.data.xpos[i].copy()
                    body_positions.append(body_pos)
                
                if body_positions:
                    body_positions = np.array(body_positions)
                    center = np.mean(body_positions, axis=0)
                    # 基于body位置估算范围
                    distances = [np.linalg.norm(pos - center) for pos in body_positions]
                    extent = max(distances) * 2 if distances else 0.1
                    extent = max(extent, 0.1)  # 最小范围
                    
                    print(f"📏 基于Body位置计算尺寸: {extent:.3f}m")
                    print(f"🎯 基于Body位置计算中心: [{center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f}]")
                else:
                    # 最后的默认值
                    center = np.array([0.0, 0.0, 0.05])
                    extent = 0.2
                    print(f"📏 使用默认值: 尺寸={extent:.3f}m, 中心=[{center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f}]")
            
            # 确保有合理的值
            extent = max(extent, 0.05)  # 最小5cm
            extent = min(extent, 1.0)   # 最大1m（适合小型机器人）
            
            self.model_stats = {
                'center': center,
                'extent': extent,
                'num_geoms': self.model.ngeom,
                'num_bodies': self.model.nbody,
                'num_joints': self.model.njnt
            }
            
        except Exception as e:
            print(f"⚠️  计算模型统计信息失败: {e}")
            # 提供安全的默认值 - 适合小型机器人的合理值
            self.model_stats = {
                'center': np.array([0.0, 0.0, 0.05]),
                'extent': 0.2,
                'num_geoms': self.model.ngeom,
                'num_bodies': self.model.nbody,
                'num_joints': self.model.njnt
            }
    
    def get_joint_names(self) -> list:
        """获取所有关节名称"""
        return self.joint_names.copy()
    
    def get_joint_id(self, joint_name: str) -> Optional[int]:
        """根据名称获取关节ID"""
        return self.joint_ids.get(joint_name)
    
    def set_joint_angle(self, joint_name: str, angle: float) -> bool:
        """
        设置关节角度
        
        Args:
            joint_name: 关节名称
            angle: 角度（弧度）
            
        Returns:
            bool: 设置是否成功
        """
        if not self.model or not self.data:
            return False
        
        joint_id = self.get_joint_id(joint_name)
        if joint_id is None:
            print(f"⚠️  关节不存在: {joint_name}")
            return False
        
        try:
            # 获取关节地址
            joint_addr = self.model.jnt_qposadr[joint_id]
            
            # 设置关节角度
            self.data.qpos[joint_addr] = angle
            
            # 前向动力学计算
            mujoco.mj_forward(self.model, self.data)
            
            return True
            
        except Exception as e:
            print(f"❌ 设置关节角度失败: {e}")
            return False
    
    def get_joint_angle(self, joint_name: str) -> Optional[float]:
        """
        获取关节角度
        
        Args:
            joint_name: 关节名称
            
        Returns:
            float: 关节角度（弧度），失败返回None
        """
        if not self.model or not self.data:
            return None
        
        joint_id = self.get_joint_id(joint_name)
        if joint_id is None:
            return None
        
        try:
            joint_addr = self.model.jnt_qposadr[joint_id]
            return self.data.qpos[joint_addr]
            
        except Exception as e:
            print(f"❌ 获取关节角度失败: {e}")
            return None
    
    def get_joint_position(self, joint_name: str) -> Optional[np.ndarray]:
        """
        获取关节在世界坐标系中的位置
        
        Args:
            joint_name: 关节名称
            
        Returns:
            np.ndarray: 关节位置 [x, y, z]，失败返回None
        """
        if not self.model or not self.data:
            return None
        
        joint_id = self.get_joint_id(joint_name)
        if joint_id is None:
            return None
        
        try:
            # 获取关节对应body的ID
            body_id = self.model.jnt_bodyid[joint_id]
            
            # 获取body位置
            body_pos = self.data.xpos[body_id].copy()
            
            return body_pos
            
        except Exception as e:
            print(f"❌ 获取关节位置失败: {e}")
            return None
    
    def step_simulation(self, dt: float = 1/60.0) -> bool:
        """
        执行一步仿真
        
        Args:
            dt: 时间步长
            
        Returns:
            bool: 仿真步骤是否成功
        """
        if not self.model or not self.data:
            return False
        
        try:
            # 设置控制输入（如果需要）
            if self.model.nu > 0:
                # 将当前关节位置作为控制目标（位置控制）
                for i, joint_name in enumerate(self.joint_names):
                    if i < self.model.nu:
                        joint_addr = self.model.jnt_qposadr[self.joint_ids[joint_name]]
                        self.data.ctrl[i] = self.data.qpos[joint_addr]
            
            # 执行仿真步骤
            mujoco.mj_step(self.model, self.data)
            
            return True
            
        except Exception as e:
            print(f"❌ 仿真步骤失败: {e}")
            return False
    
    def reset_model(self):
        """重置模型到初始状态"""
        if not self.model or not self.data:
            return
        
        try:
            # 重置状态
            mujoco.mj_resetData(self.model, self.data)
            
            # 前向动力学计算
            mujoco.mj_forward(self.model, self.data)
            
            print("🔄 模型已重置")
            
        except Exception as e:
            print(f"❌ 重置模型失败: {e}")
    
    def get_model_stats(self) -> Dict[str, Any]:
        """获取模型统计信息"""
        return self.model_stats.copy()
    
    def is_loaded(self) -> bool:
        """检查模型是否已加载"""
        return self.model is not None and self.data is not None
    
    def get_model_info(self) -> Dict[str, Any]:
        """获取模型基本信息"""
        return {
            'model_path': self.model_path,
            'joint_names': self.joint_names.copy(),
            'joint_ids': self.joint_ids.copy(),
            'nq': self.nq,
            'nv': self.nv,
            'nu': self.nu,
            'is_loaded': self.is_loaded()
        }
    
    def __str__(self):
        """字符串表示"""
        if not self.is_loaded():
            return "RobotModel(未加载)"
        
        info = self.get_model_info()
        return f"RobotModel(关节数: {len(info['joint_names'])}, 已加载: {info['is_loaded']})"


def create_test_model():
    """创建测试用的机器人模型实例"""
    # 使用带有执行器的完整模型文件
    model_path = "model-actuator-position.xml"
    
    robot = RobotModel(model_path)
    
    if robot.is_loaded():
        print("🎉 测试模型创建成功！")
        print(f"📋 关节列表: {robot.get_joint_names()}")
        
        # 测试关节控制
        if robot.joint_names:
            test_joint = robot.joint_names[0]
            print(f"🔧 测试关节: {test_joint}")
            
            # 获取初始角度
            initial_angle = robot.get_joint_angle(test_joint)
            print(f"📐 初始角度: {initial_angle:.3f} rad")
            
            # 设置新角度
            test_angle = 0.5
            if robot.set_joint_angle(test_joint, test_angle):
                current_angle = robot.get_joint_angle(test_joint)
                print(f"✅ 设置成功: {current_angle:.3f} rad")
                
                # 获取关节位置
                position = robot.get_joint_position(test_joint)
                if position is not None:
                    print(f"📍 关节位置: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]")
        
        return robot
    else:
        print("❌ 测试模型创建失败")
        return None


if __name__ == "__main__":
    """测试脚本"""
    print("🤖 ODogExample 机器人模型测试")
    print("=" * 40)
    
    # 创建测试模型
    robot = create_test_model()
    
    if robot:
        print("\n📊 模型统计信息:")
        stats = robot.get_model_stats()
        for key, value in stats.items():
            if isinstance(value, np.ndarray):
                print(f"  {key}: [{value[0]:.3f}, {value[1]:.3f}, {value[2]:.3f}]")
            else:
                print(f"  {key}: {value}")