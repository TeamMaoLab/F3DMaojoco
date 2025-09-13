"""
VistaQuickViewer 主入口点

提供命令行接口和主函数，可以作为模块运行。
"""

import argparse
import logging
import sys
import os
from pathlib import Path

from .viewer import VistaQuickViewer, quick_view


def setup_logging(log_level: str = "INFO"):
    """设置日志系统"""
    logging.basicConfig(
        level=getattr(logging, log_level.upper()),
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    return logging.getLogger(__name__)


def parse_arguments():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(
        description="VistaQuickViewer - F3DMaojoco快速查看器",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例用法:
  python -m VistaQuickViewer output/                    # 查看output目录
  python -m VistaQuickViewer output/ --save             # 保存截图
  python -m VistaQuickViewer output/ --bg black         # 黑色背景
  python -m VistaQuickViewer output/ --show-joints      # 初始显示关节
  python -m VistaQuickViewer output/ --debug            # 调试模式
  
交互控件:
  • Show Joints复选框: 切换关节显示/隐藏
  • 鼠标左键拖拽: 旋转视角
  • 鼠标右键拖拽: 平移视图
  • 鼠标滚轮: 缩放视图
        """
    )
    
    # 位置参数
    parser.add_argument(
        'export_dir',
        type=str,
        nargs='?',  # 可选参数
        default='output',
        help='F3DMaojocoScripts 导出目录路径 (默认: output)'
    )
    
    # 显示选项
    parser.add_argument(
        '--save',
        action='store_true',
        help='保存截图而不显示交互窗口'
    )
    
    parser.add_argument(
        '--output',
        type=str,
        default='scene_screenshot.png',
        help='截图保存路径 (默认: scene_screenshot.png)'
    )
    
    parser.add_argument(
        '--bg',
        type=str,
        default='white',
        help='背景颜色 (默认: white)'
    )
    
    parser.add_argument(
        '--no-show',
        action='store_true',
        help='不显示交互窗口'
    )
    
    # 关节显示选项
    parser.add_argument(
        '--show-joints',
        action='store_true',
        help='初始显示关节（默认隐藏）'
    )
    
    # 调试选项
    parser.add_argument(
        '--debug',
        action='store_true',
        help='启用调试模式'
    )
    
    parser.add_argument(
        '--log-level',
        choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'],
        default='INFO',
        help='日志级别 (默认: INFO)'
    )
    
    return parser.parse_args()


def validate_directory(directory: str) -> bool:
    """验证目录是否为有效的F3DMaojoco导出目录"""
    dir_path = Path(directory)
    
    if not dir_path.exists():
        print(f"错误: 目录不存在: {directory}")
        print("请确保 F3DMaojocoScripts 已经运行并生成了导出数据")
        return False
    
    if not dir_path.is_dir():
        print(f"错误: 路径不是目录: {directory}")
        return False
    
    # 检查是否包含导出数据文件
    json_files = list(dir_path.glob("component_positions.json"))
    if not json_files:
        print(f"错误: 目录中未找到导出数据文件: {directory}")
        print("请确保 F3DMaojocoScripts 已经在该目录中运行并生成了导出数据")
        return False
    
    return True


def main():
    """主函数"""
    # 解析命令行参数
    args = parse_arguments()
    
    # 设置日志级别
    if args.debug:
        args.log_level = 'DEBUG'
    
    # 设置日志系统
    logger = setup_logging(args.log_level)
    
    # 打印欢迎信息
    print("=" * 60)
    print("VistaQuickViewer - F3DMaojoco 快速查看器")
    print("=" * 60)
    
    # 验证目录
    if not validate_directory(args.export_dir):
        print(f"\n使用说明:")
        print(f"1. 确保已运行 F3DMaojocoScripts 并生成了导出数据")
        print(f"2. 导出数据默认位于 'output/' 目录")
        print(f"3. 或者指定正确的导出目录路径")
        print(f"\n示例:")
        print(f"  python -m VistaQuickViewer output/")
        print(f"  python -m VistaQuickViewer /path/to/your/export/")
        sys.exit(1)
    
    logger.info(f"正在查看导出目录: {args.export_dir}")
    
    try:
        if args.save:
            # 保存截图模式
            logger.info("使用保存截图模式")
            print(f"正在保存截图到: {args.output}")
            
            viewer = quick_view(
                args.export_dir, 
                show=False, 
                screenshot_path=args.output,
                show_joints=args.show_joints
            )
            
            if viewer:
                print(f"✓ 截图已保存到: {args.output}")
                
                # 打印场景信息
                summary = viewer.get_scene_summary()
                print(f"\n场景信息:")
                print(f"  - 零部件数量: {summary.get('component_count', 0)}")
                print(f"  - 关节数量: {summary.get('joint_count', 0)}")
                print(f"  - STL文件数量: {summary.get('stl_file_count', 0)}")
                
                print(f"\n完成！")
                sys.exit(0)
            else:
                print(f"✗ 截图保存失败")
                logger.error("截图保存失败")
                sys.exit(1)
        
        else:
            # 创建查看器
            print(f"正在创建 3D 查看器...")
            viewer = VistaQuickViewer(args.export_dir, logger)
            
            # 设置关节显示状态
            viewer.show_joints = args.show_joints
            print(f"关节显示: {'开启' if args.show_joints else '关闭'}")
            
            # 设置背景颜色
            viewer.set_background_color(args.bg)
            print(f"背景颜色: {args.bg}")
            
            # 加载数据
            print(f"正在加载数据...")
            if not viewer.load_data():
                print(f"✗ 数据加载失败")
                logger.error("数据加载失败")
                sys.exit(1)
            
            # 创建场景
            print(f"正在创建 3D 场景...")
            if not viewer.create_scene():
                print(f"✗ 场景创建失败")
                logger.error("场景创建失败")
                sys.exit(1)
            
            # 打印场景信息
            summary = viewer.get_scene_summary()
            print(f"\n场景信息:")
            print(f"  - 零部件数量: {summary.get('component_count', 0)}")
            print(f"  - 关节数量: {summary.get('joint_count', 0)}")
            print(f"  - STL文件数量: {summary.get('stl_file_count', 0)}")
            
            # 显示或保存
            if args.no_show:
                print(f"\n不显示交互窗口（如需显示，请移除 --no-show 参数）")
                print(f"完成！")
                sys.exit(0)
            else:
                print(f"\n正在打开交互式 3D 查看器...")
                print(f"提示：在查看器中可以使用鼠标进行交互")
                print(f"  - 左键拖拽：旋转视角")
                print(f"  - 右键拖拽：平移视图")
                print(f"  - 滚轮：缩放视图")
                print(f"  - Show Joints复选框：切换关节显示/隐藏")
                print(f"\n按 Ctrl+C 退出程序")
                
                viewer.show()
                sys.exit(0)
                
    except KeyboardInterrupt:
        print(f"\n用户中断")
        logger.info("用户中断")
        sys.exit(0)
        
    except Exception as e:
        print(f"\n✗ 程序运行错误: {str(e)}")
        logger.error(f"程序运行错误: {str(e)}")
        import traceback
        logger.error(traceback.format_exc())
        sys.exit(1)


if __name__ == "__main__":
    main()