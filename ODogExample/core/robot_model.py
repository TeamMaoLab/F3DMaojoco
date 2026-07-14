"""
ODogExample核心模块 - 机器人模型封装

提供MuJoCo模型的加载、管理和基础操作功能。
"""

import mujoco
import numpy as np
import math
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
        """计算模型统计信息 - 综合使用几何体和body位置"""
        if not self.model or not self.data:
            return
        
        try:
            # 收集几何体和body位置信息
            all_positions = []
            geom_count = 0
            large_geom_count = 0
            
            # 收集有效的几何体信息
            for i in range(self.model.ngeom):
                geom = self.model.geom(i)
                geom_pos = geom.pos.copy()
                geom_size = geom.size.copy()
                geom_count += 1
                
                # 跳过过大的几何体（地面）
                if geom_size is not None and np.any(geom_size > 5.0):
                    large_geom_count += 1
                    continue
                
                # 添加几何体的边界点
                all_positions.extend([
                    geom_pos - geom_size,
                    geom_pos + geom_size
                ])
            
            # 收集body位置信息
            for i in range(1, self.model.nbody):
                body_pos = self.data.xpos[i].copy()
                all_positions.append(body_pos)
            
            print(f"📊 统计信息: 几何体总数={geom_count}, 大型几何体={large_geom_count}, 总采样点={len(all_positions)}")
            
            if all_positions:
                all_positions = np.array(all_positions)
                
                # 计算边界和中心
                min_pos = np.min(all_positions, axis=0)
                max_pos = np.max(all_positions, axis=0)
                center = (min_pos + max_pos) / 2
                extent = np.max(max_pos - min_pos)
                
                # 如果范围太小，使用基于关节位置的估算
                if extent < 0.05:  # 小于5cm
                    # 使用关节位置进行估算
                    joint_positions = []
                    for i in range(self.model.njnt):
                        joint_pos = self.data.xpos[self.model.jnt_bodyid[i]].copy()
                        joint_positions.append(joint_pos)
                    
                    if joint_positions:
                        joint_positions = np.array(joint_positions)
                        joint_center = np.mean(joint_positions, axis=0)
                        joint_distances = [np.linalg.norm(pos - joint_center) for pos in joint_positions]
                        joint_extent = max(joint_distances) * 2.5 if joint_distances else 0.1
                        
                        # 混合使用两种计算结果
                        center = (center + joint_center) / 2
                        extent = max(extent, joint_extent)
                
                print(f"📏 综合计算模型尺寸: {extent:.3f}m")
                print(f"🎯 综合计算模型中心: [{center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f}]")
                print(f"📦 模型边界: [{min_pos[0]:.3f}, {min_pos[1]:.3f}, {min_pos[2]:.3f}] 到 [{max_pos[0]:.3f}, {max_pos[1]:.3f}, {max_pos[2]:.3f}]")
            else:
                # 如果没有数据，使用默认值
                center = np.array([0.0, 0.0, 0.05])
                extent = 0.15
                print(f"📏 使用默认值: 尺寸={extent:.3f}m, 中心=[{center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f}]")
            
            # 确保合理的尺寸范围
            extent = max(extent, 0.08)  # 最小8cm
            extent = min(extent, 0.6)   # 最大60cm
            
            self.model_stats = {
                'center': center,
                'extent': extent,
                'min_pos': min_pos if all_positions else center - extent/2,
                'max_pos': max_pos if all_positions else center + extent/2,
                'num_geoms': self.model.ngeom,
                'num_bodies': self.model.nbody,
                'num_joints': self.model.njnt
            }
            
        except Exception as e:
            print(f"⚠️  计算模型统计信息失败: {e}")
            self.model_stats = {
                'center': np.array([0.0, 0.0, 0.05]),
                'extent': 0.15,
                'min_pos': np.array([-0.075, -0.075, 0.0]),
                'max_pos': np.array([0.075, 0.075, 0.1]),
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

    # TODO(重构): set_joint_angle 与 set_joint_angles 中"查找执行器 id"的循环
    # （遍历 model.actuator(i).trnid[0] == joint_id）重复了 4 次。待抽取为
    # _find_actuator(joint_id) 辅助方法。见 2026-07-14 架构梳理。
    def set_joint_angle(self, joint_name: str, angle: float, smooth: bool = True) -> bool:
        """
        设置关节角度
        
        Args:
            joint_name: 关节名称
            angle: 角度（弧度）
            smooth: 是否使用平滑过渡
            
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
            
            if smooth:
                # 使用执行器控制信号实现平滑过渡
                # 找到对应的执行器ID
                actuator_id = None
                for i in range(self.model.nu):
                    if self.model.actuator(i).trnid[0] == joint_id:  # 执行器控制的关节ID
                        actuator_id = i
                        break
                
                if actuator_id is not None:
                    # 只设置执行器控制信号，让物理引擎平滑过渡到目标角度
                    self.data.ctrl[actuator_id] = angle
                    print(f"🎯 设置目标角度: {joint_name} -> {math.degrees(angle):.1f}° (平滑过渡)")
                else:
                    print(f"⚠️  未找到关节 {joint_name} 对应的执行器")
                    return False
            else:
                # 直接设置关节角度（瞬时切换）
                self.data.qpos[joint_addr] = angle
                
                # 设置执行器控制信号以维持位置
                actuator_id = None
                for i in range(self.model.nu):
                    if self.model.actuator(i).trnid[0] == joint_id:
                        actuator_id = i
                        break
                
                if actuator_id is not None:
                    self.data.ctrl[actuator_id] = angle
                    print(f"⚡ 瞬时设置: {joint_name} -> {math.degrees(angle):.1f}°")
                else:
                    print(f"⚠️  未找到关节 {joint_name} 对应的执行器")
                    return False
            
            # 前向动力学计算
            mujoco.mj_forward(self.model, self.data)
            
            return True
            
        except Exception as e:
            print(f"❌ 设置关节角度失败: {e}")
            return False
    
    def set_joint_angles(self, joint_angles: Dict[str, float], smooth: bool = True) -> bool:
        """
        批量设置关节角度
        
        Args:
            joint_angles: 关节名称到角度的映射
            smooth: 是否使用平滑过渡
            
        Returns:
            bool: 设置是否成功
        """
        if not self.model or not self.data:
            return False
        
        success_count = 0
        total_count = len(joint_angles)
        
        try:
            if smooth:
                # 平滑过渡：只设置执行器控制信号，让物理引擎处理过渡
                for joint_name, target_angle in joint_angles.items():
                    joint_id = self.get_joint_id(joint_name)
                    if joint_id is not None:
                        # 找到对应的执行器ID
                        actuator_id = None
                        for i in range(self.model.nu):
                            if self.model.actuator(i).trnid[0] == joint_id:
                                actuator_id = i
                                break
                        
                        if actuator_id is not None:
                            self.data.ctrl[actuator_id] = target_angle
                            success_count += 1
                        else:
                            print(f"⚠️  未找到关节 {joint_name} 对应的执行器")
                    else:
                        print(f"⚠️  关节不存在: {joint_name}")
                
                if success_count > 0:
                    print(f"🎯 开始平滑过渡: {success_count}/{total_count} 个关节")
                
            else:
                # 瞬时切换：直接设置关节角度
                for joint_name, target_angle in joint_angles.items():
                    joint_id = self.get_joint_id(joint_name)
                    if joint_id is not None:
                        joint_addr = self.model.jnt_qposadr[joint_id]
                        self.data.qpos[joint_addr] = target_angle
                        
                        # 设置执行器控制信号以维持位置
                        actuator_id = None
                        for i in range(self.model.nu):
                            if self.model.actuator(i).trnid[0] == joint_id:
                                actuator_id = i
                                break
                        
                        if actuator_id is not None:
                            self.data.ctrl[actuator_id] = target_angle
                            success_count += 1
                        else:
                            print(f"⚠️  未找到关节 {joint_name} 对应的执行器")
                    else:
                        print(f"⚠️  关节不存在: {joint_name}")
                
                if success_count > 0:
                    print(f"⚡ 瞬时设置完成: {success_count}/{total_count} 个关节")
            
            # 前向动力学计算
            mujoco.mj_forward(self.model, self.data)
            
            return success_count > 0
            
        except Exception as e:
            print(f"❌ 批量设置关节角度失败: {e}")
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
        执行一步仿真 - 优化版本
        
        Args:
            dt: 时间步长
            
        Returns:
            bool: 仿真步骤是否成功
        """
        if not self.model or not self.data:
            return False
        
        try:
            # 重要：持续设置控制输入以维持位置控制
            if self.model.nu > 0:
                # 为每个执行器设置控制信号
                for i in range(self.model.nu):
                    actuator = self.model.actuator(i)
                    # 获取执行器控制的关节ID
                    joint_id = actuator.trnid[0]
                    # 获取关节当前角度作为控制目标
                    joint_addr = self.model.jnt_qposadr[joint_id]
                    current_angle = self.data.qpos[joint_addr]
                    # 设置执行器控制信号
                    self.data.ctrl[i] = current_angle
            
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
    
    def reload_model(self):
        """重新加载模型（完全重新开始模拟）"""
        if not self.model_path:
            print("❌ 没有模型文件路径，无法重新加载")
            return False
        
        try:
            print(f"🔄 重新加载模型: {self.model_path}")
            
            # 重新加载模型
            self.model = mujoco.MjModel.from_xml_path(self.model_path)
            self.data = mujoco.MjData(self.model)
            
            # 重新初始化模型状态
            mujoco.mj_forward(self.model, self.data)
            
            # 重新提取模型信息
            self._extract_model_info()
            self._calculate_model_stats()
            
            print("✅ 模型重新加载成功")
            print(f"   关节数量: {len(self.joint_names)}")
            print(f"   广义坐标: {self.nq}")
            print(f"   广义速度: {self.nv}")
            print(f"   控制输入: {self.nu}")
            
            return True
            
        except Exception as e:
            print(f"❌ 重新加载模型失败: {e}")
            return False
    
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
    import os
    # 获取当前文件的目录，然后找到模型文件
    current_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(current_dir, "..", "model-actuator-position.xml")
    
    # 如果文件不存在，尝试使用相对路径
    if not os.path.exists(model_path):
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