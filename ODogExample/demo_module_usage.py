#!/usr/bin/env python3
"""
演示 ODogExample 的多种运行方式

这个脚本展示了如何以不同的方式启动 ODogExample 应用。
"""

def show_usage():
    print("🎯 ODogExample 多种启动方式演示")
    print("=" * 40)
    print("\n📋 支持的启动方式:")
    print("\n1. 模块化运行 (推荐):")
    print("   python -m ODogExample.gui.app_entry")
    print("   python -m ODogExample.gui.app_main")
    print("\n2. 直接运行:")
    print("   python gui/app_entry.py")
    print("   python gui/app_main.py")
    print("\n3. 编程式启动:")
    print("   from ODogExample import main")
    print("   main()")
    print("\n4. 导入组件:")
    print("   from ODogExample.gui.app_main import MainApplication")
    print("   from ODogExample.core import RobotModel")
    print("   from ODogExample.gui.viewer_widget import MuJoCoViewerWidget")

def test_imports():
    """测试各种导入方式"""
    print("\n🧪 测试导入...")
    
    try:
        # 添加父目录到路径以支持模块化导入
        import sys
        import os
        parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        sys.path.insert(0, parent_dir)
        
        # 测试从包导入
        from ODogExample import main, MainApplication
        from ODogExample.core import RobotModel, create_test_model
        from ODogExample.gui import MuJoCoViewerWidget
        
        print("✅ 包导入测试通过")
        
        # 测试创建机器人模型
        robot = create_test_model()
        if robot and robot.is_loaded():
            print("✅ 机器人模型创建测试通过")
        
        return True
        
    except Exception as e:
        print(f"❌ 导入测试失败: {e}")
        return False

if __name__ == "__main__":
    show_usage()
    
    if test_imports():
        print("\n🎉 所有测试通过！现在可以选择任意一种方式启动应用。")
        print("\n🚀 推荐使用: python -m ODogExample.gui.app_entry")
    else:
        print("\n⚠️  测试失败，请检查环境配置。")