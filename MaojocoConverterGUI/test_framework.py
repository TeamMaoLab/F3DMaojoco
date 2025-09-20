#!/usr/bin/env python3
"""
基础框架测试脚本

验证GUI基础框架的各个组件是否正常工作。
"""

import sys
from pathlib import Path

# 添加项目根目录到路径
project_root = Path(__file__).parent.parent
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

def test_logger():
    """测试日志模块"""
    print("🧪 测试日志模块...")
    try:
        from MaojocoConverterGUI.utils.logger import logger
        logger.info("测试日志信息")
        logger.success("日志模块测试成功")
        return True
    except Exception as e:
        print(f"❌ 日志模块测试失败: {e}")
        return False

def test_gui_imports():
    """测试GUI模块导入"""
    print("🧪 测试GUI模块导入...")
    try:
        # 先创建QApplication实例（避免QWidget警告）
        from PySide6.QtWidgets import QApplication
        import sys
        
        # 检查是否已有QApplication实例
        app = QApplication.instance()
        if app is None:
            app = QApplication(sys.argv)
        
        from MaojocoConverterGUI.gui import MainWindow, Application, create_application
        print("✅ GUI模块导入成功")
        
        # 测试应用程序创建
        gui_app = create_application()
        print("✅ 应用程序创建成功")
        
        # 测试主窗口获取
        window = gui_app.get_main_window()
        print("✅ 主窗口获取成功")
        
        return True
    except Exception as e:
        print(f"❌ GUI模块测试失败: {e}")
        return False

def test_types():
    """测试类型注解"""
    print("🧪 测试类型注解...")
    try:
        from typing import List, Dict, Optional
        from pathlib import Path
        
        # 测试类型注解函数
        def test_function(path: Path, items: List[str]) -> Dict[str, Optional[str]]:
            return {"path": str(path), "items": ", ".join(items)}
        
        result = test_function(Path("/test"), ["a", "b", "c"])
        print(f"✅ 类型注解测试成功: {result}")
        
        return True
    except Exception as e:
        print(f"❌ 类型注解测试失败: {e}")
        return False

def main():
    """主测试函数"""
    print("🚀 开始基础框架测试")
    print("=" * 50)
    
    tests = [
        ("日志模块", test_logger),
        ("GUI模块", test_gui_imports),
        ("类型注解", test_types),
    ]
    
    results = []
    for test_name, test_func in tests:
        print(f"\n📋 {test_name}测试:")
        try:
            result = test_func()
            results.append((test_name, result))
        except Exception as e:
            print(f"❌ {test_name}测试异常: {e}")
            results.append((test_name, False))
    
    print("\n" + "=" * 50)
    print("📊 测试结果汇总:")
    
    passed = 0
    for test_name, result in results:
        status = "✅ 通过" if result else "❌ 失败"
        print(f"  {test_name}: {status}")
        if result:
            passed += 1
    
    print(f"\n🎯 总计: {passed}/{len(tests)} 测试通过")
    
    if passed == len(tests):
        print("🎉 所有测试通过！基础框架搭建成功！")
        return 0
    else:
        print("⚠️  部分测试失败，请检查实现。")
        return 1

if __name__ == "__main__":
    sys.exit(main())