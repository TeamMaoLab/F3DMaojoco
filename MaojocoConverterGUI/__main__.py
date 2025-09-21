"""
简化的主程序入口

使用简化的2层架构，直接调用Core服务，去除过度复杂的中间层。
"""

# 标准库
import sys
import argparse
from pathlib import Path

# 添加项目根目录到路径
project_root = Path(__file__).parent
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

# 第三方库导入
from PySide6.QtWidgets import QApplication

# 本地模块导入
from MaojocoConverterGUI.ui.main_window import create_simplified_application
from MaojocoConverterGUI.utils.logger import logger


def parse_arguments() -> argparse.Namespace:
    """解析命令行参数
    
    Returns:
        argparse.Namespace: 解析后的参数
    """
    parser = argparse.ArgumentParser(
        description="MaojocoConverter GUI (简化版) - 基于2层架构的图形界面",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 基本运行
  %(prog)s
  
  # 快速启动，指定项目目录
  %(prog)s --directory /path/to/project
  %(prog)s -d TmpFinger
  
  # 显示帮助
  %(prog)s --help
        """
    )
    
    parser.add_argument(
        "--directory", "-d",
        type=str,
        help="指定项目目录路径，快速启动项目"
    )
    
    parser.add_argument(
        "--version", "-v",
        action="version",
        version="%(prog)s 2.0.0"
    )
    
    return parser.parse_args()


def main() -> None:
    """主程序入口
    
    创建并运行简化版MaojocoConverter GUI应用程序。
    """
    try:
        # 解析命令行参数
        args = parse_arguments()
        
        # 创建QApplication实例
        app = QApplication(sys.argv)
        
        # 设置应用程序信息
        app.setApplicationName("MaojocoConverter GUI (简化版)")
        app.setApplicationVersion("2.0.0")
        app.setOrganizationName("TeamMaoLab")
        
        # 创建简化版应用程序
        gui_app = create_simplified_application()
        main_window = gui_app.get_main_window()
        
        # 如果指定了目录，设置快速启动
        if args.directory:
            directory_path = Path(args.directory)
            if directory_path.exists():
                logger.info(f"快速启动模式：使用目录 {directory_path}")
                
                # 预加载项目数据
                transform_service = main_window.get_transform_service()
                result = transform_service.load_project(directory_path)
                
                if result.success:
                    logger.success(f"项目加载成功: {result.message}")
                    
                    # 直接跳转到模型预览阶段（这会自动处理模型显示）
                    main_window.stage_panels.on_project_loaded(result)
                    
                    logger.info("快速启动设置完成")
                else:
                    logger.error(f"项目加载失败: {result.message}")
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