#!/usr/bin/env python3
"""
测试GUI模块拆分后的基本功能
"""

import sys
import os

# 添加项目根目录到路径
sys.path.insert(0, '/Users/maoge/Documents/TeamMaoLab/F3DMaojoco')

def test_imports():
    """测试所有模块是否可以正常导入"""
    print("🧪 测试模块导入...")
    
    try:
        # 测试核心模块导入
        print("  📦 测试核心模块...")
        from core.robot_model import RobotModel, create_test_model
        from core.joint_mapping import JointMapping
        print("  ✅ 核心模块导入成功")
        
        # 测试GUI模块导入
        print("  📦 测试GUI模块...")
        from gui.camera_system import OrbitCamera, InputHandler
        from gui.mujoco_renderer import MuJoCoRenderer
        from gui.joint_controls import JointControlWidget, LegControlGroup
        from gui.global_controls import GlobalControlGroup, CameraControlGroup
        from gui.app_signals import SignalManager
        print("  ✅ GUI基础模块导入成功")
        
        # 测试组件导入
        print("  📦 测试组件模块...")
        from gui.viewer_widget import MuJoCoViewerWidget
        from gui.control_panel import ControlPanel, create_control_panel
        from gui.app_main import MainApplication
        from gui.app_entry import run_application
        print("  ✅ 组件模块导入成功")
        
        print("🎉 所有模块导入测试通过！")
        return True
        
    except Exception as e:
        print(f"❌ 模块导入失败: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_basic_functionality():
    """测试基本功能"""
    print("\n🧪 测试基本功能...")
    
    try:
        # 测试相机系统
        print("  🎥 测试相机系统...")
        from gui.camera_system import OrbitCamera, InputHandler
        camera = OrbitCamera()
        input_handler = InputHandler()
        print("  ✅ 相机系统创建成功")
        
        # 测试信号管理
        print("  🔗 测试信号管理...")
        from gui.app_signals import SignalManager
        signal_manager = SignalManager()
        print("  ✅ 信号管理创建成功")
        
        # 测试关节映射
        print("  🦿 测试关节映射...")
        from core.joint_mapping import JointMapping
        joint_mapping = JointMapping()
        default_pose = joint_mapping.get_default_pose()
        print(f"  ✅ 关节映射成功，默认姿态包含 {len(default_pose)} 个关节")
        
        print("🎉 基本功能测试通过！")
        return True
        
    except Exception as e:
        print(f"❌ 基本功能测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_compatibility():
    """测试向后兼容性"""
    print("\n🧪 测试向后兼容性...")
    
    try:
        # 测试原有的导入方式
        print("  📦 测试原有导入方式...")
        from gui.control_panels import create_control_panel as legacy_create_control_panel
        from gui.main_app import main as legacy_main
        print("  ✅ 原有导入方式工作正常")
        
        print("🎉 向后兼容性测试通过！")
        return True
        
    except Exception as e:
        print(f"❌ 向后兼容性测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """主测试函数"""
    print("=" * 60)
    print("🧪 ODogExample GUI模块拆分测试")
    print("=" * 60)
    
    tests = [
        ("模块导入", test_imports),
        ("基本功能", test_basic_functionality), 
        ("向后兼容性", test_compatibility)
    ]
    
    results = []
    for test_name, test_func in tests:
        print(f"\n🔍 {test_name}测试:")
        result = test_func()
        results.append((test_name, result))
    
    print("\n" + "=" * 60)
    print("📊 测试结果汇总:")
    for test_name, result in results:
        status = "✅ 通过" if result else "❌ 失败"
        print(f"  {test_name}: {status}")
    
    all_passed = all(result for _, result in results)
    if all_passed:
        print("\n🎉 所有测试通过！模块拆分成功！")
        return 0
    else:
        print("\n⚠️  部分测试失败，需要修复问题")
        return 1

if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)