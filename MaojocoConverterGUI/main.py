"""
MaojocoConverter GUI - 主程序入口

基于PySide6和PyVista的图形界面，为MaojocoConverter提供可视化配置和转换流程控制。
"""

import sys
from pathlib import Path

# 添加项目根目录到路径
project_root = Path(__file__).parent.parent
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

# 标准库导入
from typing import Optional

# 第三方库导入
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QDir

# 本地模块导入
from gui import create_application


def main() -> None:
    """主程序入口
    
    创建并运行MaojocoConverter GUI应用程序。
    """
    try:
        # 创建QApplication实例
        app = QApplication(sys.argv)
        
        # 设置应用程序信息
        app.setApplicationName("MaojocoConverter GUI")
        app.setApplicationVersion("1.0.0")
        app.setOrganizationName("TeamMaoLab")
        
        # 创建应用程序
        gui_app = create_application()
        
        # 运行应用程序
        gui_app.run()
        
        # 执行Qt事件循环
        exit_code = app.exec()
        
        sys.exit(exit_code)
        
    except ImportError as e:
        print(f"❌ 导入错误: {e}")
        print("💡 请确保已安装所需依赖:")
        print("   - PySide6>=6.9.2")
        print("   - PyVista>=0.46.3")
        print("   - pyvistaqt>=0.11.3")
        sys.exit(1)
        
    except Exception as e:
        print(f"❌ 程序启动失败: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()