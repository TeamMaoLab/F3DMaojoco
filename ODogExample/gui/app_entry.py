"""
ODogExample GUI模块 - 应用入口

提供应用程序的启动入口和初始化逻辑。
"""

import sys
from PySide6.QtWidgets import QApplication
from .app_main import MainApplication


def setup_application():
    """设置应用程序"""
    print("🚀 启动ODogExample...")
    print("=" * 50)
    
    # 创建应用
    app = QApplication(sys.argv)
    
    # 设置应用信息
    app.setApplicationName("ODogExample")
    app.setApplicationVersion("0.1.0")
    app.setOrganizationName("TeamMaoLab")
    
    return app


def create_main_window():
    """创建主窗口"""
    main_app = MainApplication()
    return main_app


def run_application():
    """运行应用程序"""
    try:
        # 设置应用
        app = setup_application()
        
        # 创建主窗口
        main_app = create_main_window()
        
        # 显示窗口
        main_app.show()
        
        # 打印应用信息
        app_info = main_app.get_application_info()
        print(f"📱 应用信息:")
        print(f"  标题: {app_info['title']}")
        print(f"  尺寸: {app_info['size'][0]}x{app_info['size'][1]}")
        print(f"  查看器: {'已加载' if app_info['viewer_loaded'] else '未加载'}")
        print(f"  控制面板: {'已加载' if app_info['control_panel_loaded'] else '未加载'}")
        print(f"  信号连接: {app_info['signal_connections'].get('total_connections', 0)} 个")
        
        # 运行应用
        return app.exec()
    
    except Exception as e:
        print(f"❌ 应用启动失败: {e}")
        import traceback
        traceback.print_exc()
        return 1


def main():
    """主应用入口"""
    exit_code = run_application()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()