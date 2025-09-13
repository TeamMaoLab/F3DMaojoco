# -*- coding: utf-8 -*-
"""
F3DMaojocoScripts - Fusion 360 插件主入口

该模块是整个插件的入口点，提供简单的脚本接口来导出数据。
"""

import traceback
import adsk.core
import adsk.fusion
from .inf3d.fusion_export_manager import FusionExportManager
from .common.data_types import MeshQuality
from .inf3d.logger import get_logger, log_performance_start, log_performance_end, initialize_logging

# Initialize the global variables for the Application and UserInterface objects.
app = adsk.core.Application.get()
ui = app.userInterface


def run(_context):
    """运行装配体导出"""
    try:
        # 获取输出目录 - 如果没有，直接退出
        output_dir = _get_output_directory()
        if not output_dir:
            ui.messageBox("导出已取消：未选择输出目录")
            return
        
        # 创建导出管理器
        export_manager = FusionExportManager(mesh_quality=MeshQuality.MEDIUM)
        
        # 创建备份（在日志系统初始化之前）
        from .common.backup_manager import BackupManager
        backup_manager = BackupManager()
        
        # 创建输出目录
        import os
        os.makedirs(output_dir, exist_ok=True)
        
        # 执行备份（不使用日志系统）
        backup_success = backup_manager.create_backup(output_dir)
        
        # 初始化日志系统（备份完成后）
        logger = initialize_logging(output_dir)
        start_time = log_performance_start(logger, "F3DMaojoco导出")
        
        if backup_success:
            logger.info("备份创建成功")
        else:
            logger.warning("备份创建失败，继续执行导出")
        
        logger.info("开始导出装配体")
        
        # 设置日志并执行导出
        export_manager.set_logger(logger)
        export_result = export_manager.export_assembly(output_dir)
        
        # 显示导出摘要
        _show_export_summary(export_manager)
        
        # 使用export_result避免未使用变量警告
        if export_result and export_result.success:
            logger.info(f"导出完成: {len(export_result.data.components)} 个零部件, {len(export_result.data.joints)} 个关节")
        else:
            logger.warning("导出未成功完成")
        
        log_performance_end(logger, "F3DMaojoco导出", start_time)
        logger.info("装配体导出完成")
        ui.messageBox("导出完成！")
        
    except Exception as e:
        # 如果logger可用，记录错误
        try:
            logger.error(f"导出失败: {str(e)}", exc_info=True)
        except:
            # 如果logger不可用，直接显示错误
            pass
        
        error_msg = f"导出失败:\n{str(e)}\n\n{traceback.format_exc()}"
        ui.messageBox(error_msg)
        app.log(f'Failed:\n{traceback.format_exc()}')


def _get_output_directory():
    """获取输出目录"""
    try:
        folder_dialog = ui.createFolderDialog()
        folder_dialog.title = "选择零部件和关节导出目录"
        result = folder_dialog.showDialog()
        
        if result == adsk.core.DialogResults.DialogOK:
            # 在没有logger的情况下，直接返回选择的目录
            # 避免在logger初始化之前调用get_logger()
            return folder_dialog.folder
        
        return None
    
    except Exception as e:
        ui.messageBox(f"获取输出目录失败: {str(e)}")
        return None


def _show_export_summary(export_manager):
    """显示导出摘要"""
    try:
        summary = export_manager.get_export_summary()
        analysis = export_manager.analyze_export_results()
        
        # 获取备份信息
        from .common.backup_manager import BackupManager
        backup_manager = BackupManager()
        backup_info = backup_manager.get_backup_info(summary['output_directory'])
        
        # 记录到日志
        logger = get_logger()
        logger.info(f"导出摘要 - 零部件: {summary['total_components']}, 关节: {summary['total_joints']}")
        
        summary_msg = "导出摘要:\n\n"
        summary_msg += f"📁 输出目录: {summary['output_directory']}\n"
        summary_msg += f"🔧 网格质量: {summary['mesh_quality']}\n"
        summary_msg += f"📦 零部件总数: {summary['total_components']}\n"
        summary_msg += f"🔗 关节总数: {summary['total_joints']}\n"
        summary_msg += f"📄 STL文件数: {summary['stl_export']['exported_files_count']}\n"
        
        # 添加备份信息
        if backup_info.get('exists') and not backup_info.get('error'):
            backup_folders = backup_info.get('backup_folders', [])
            if backup_folders:
                summary_msg += f"💾 备份文件夹: {len(backup_folders)} 个\n"
                summary_msg += f"  • 最新备份: {backup_folders[0]}\n"
        
        summary_msg += "\n"
        
        summary_msg += "📊 详细统计:\n"
        summary_msg += f"  • 有实体的零部件: {analysis['components_with_bodies']}\n"
        summary_msg += f"  • 成功导出STL: {analysis['components_with_stl']}\n"
        summary_msg += f"  • 有子零部件: {analysis['components_with_children']}\n"
        summary_msg += f"  • 活动关节: {analysis['active_joints']}\n\n"
        
        summary_msg += "🔧 关节类型分布:\n"
        for joint_type, count in analysis['joint_types'].items():
            summary_msg += f"  • {joint_type}: {count}\n"
        
        summary_msg += "\n📋 生成文件:\n"
        summary_msg += f"  • component_positions.json (零部件和关节信息)\n"
        summary_msg += f"  • stl_files/ (STL文件目录)\n"
        summary_msg += f"  • export_description.md (导出描述文件)\n"
        summary_msg += f"  • f3d_export.log (执行日志)\n"
        summary_msg += f"  • backup/ (备份目录，带时间戳)\n"
        
        # 添加备份文件夹信息
        if backup_info.get('exists') and not backup_info.get('error'):
            backup_folders = backup_info.get('backup_folders', [])
            if backup_folders:
                summary_msg += f"  • backup_*/ (备份文件夹，包含之前的导出结果)\n"
        
        summary_msg += "\n注意：STL文件使用毫米单位，位置信息使用米单位。"
        summary_msg += "\n💾 备份功能：每次导出前会自动将非备份文件移动到时间戳备份文件夹中。"
        
        ui.palettes.itemById('TextCommands').writeText(summary_msg)
        logger.info("导出摘要已显示给用户")
        
    except Exception as e:
        logger.error(f"显示导出摘要失败: {str(e)}")
        ui.messageBox(f"显示导出摘要失败: {str(e)}")


# 以下是插件模式的入口点（保留用于兼容性）
def stop(_context):
    """插件停止入口（保留用于兼容性）"""
    # 静默忽略context参数，用于插件兼容性
    pass


