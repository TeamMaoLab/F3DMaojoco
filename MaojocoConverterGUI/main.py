"""
MaojocoConverter GUI - 主程序入口

基于PySide6和PyVista的图形界面，为MaojocoConverter提供可视化配置和转换流程控制。
"""

import sys
import argparse
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
from gui.main_window import MainWindow
from gui.stage_panels import DataLoadingPanel
from utils.logger import logger


def parse_arguments() -> argparse.Namespace:
    """解析命令行参数
    
    Returns:
        argparse.Namespace: 解析后的参数
    """
    parser = argparse.ArgumentParser(
        description="MaojocoConverter GUI - 基于PySide6和PyVista的图形界面",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 基本运行
  %(prog)s
  
  # 快速启动，直接进入数据加载阶段
  %(prog)s --directory /path/to/project
  %(prog)s -d TmpFinger
  
  # 显示帮助
  %(prog)s --help
        """
    )
    
    parser.add_argument(
        "--directory", "-d",
        type=str,
        help="指定项目目录路径，自动跳过初始化阶段"
    )
    
    parser.add_argument(
        "--version", "-v",
        action="version",
        version="%(prog)s 1.0.0"
    )
    
    return parser.parse_args()


def main() -> None:
    """主程序入口
    
    创建并运行MaojocoConverter GUI应用程序。
    """
    try:
        # 解析命令行参数
        args = parse_arguments()
        
        # 创建QApplication实例
        app = QApplication(sys.argv)
        
        # 设置应用程序信息
        app.setApplicationName("MaojocoConverter GUI")
        app.setApplicationVersion("1.0.0")
        app.setOrganizationName("TeamMaoLab")
        
        # 创建应用程序
        gui_app = create_application()
        main_window: MainWindow = gui_app.get_main_window()
        
        # 如果指定了目录，设置快速启动
        if args.directory:
            directory_path = Path(args.directory)
            if directory_path.exists():
                logger.info(f"快速启动模式：使用目录 {directory_path}")
                # 直接切换到数据加载阶段并设置目录
                main_window.stage_manager.switch_to_stage("data_loading")
                data_loading_panel = main_window.stage_manager.stages.get("data_loading")
                if data_loading_panel:
                    data_loading_panel.set_input_directory(directory_path)
                
                # 隐藏3D视图的提示文字
                if main_window.viz_widget:
                    main_window.viz_widget._hide_initial_text()
                    logger.info("已隐藏3D视图提示文字")
            else:
                logger.warning(f"指定目录不存在: {directory_path}，使用正常启动模式")
        
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