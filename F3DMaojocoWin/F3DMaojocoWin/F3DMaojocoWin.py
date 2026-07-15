# -*- coding: utf-8 -*-
"""F3DMaojocoWin - Fusion 360 导出插件（Windows 版）

热重载：run() 开头清除本插件的模块缓存（sys.modules），
这样每次运行都从磁盘重新读 .py，改代码后不用重启 Fusion。
"""
import os
import sys
import traceback

import adsk.core
import adsk.fusion

# 脚本所在目录注入 sys.path
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

app = adsk.core.Application.get()
ui = app.userInterface

# 需要热重载的模块前缀（本插件自己的，不含 adsk/标准库）
_HOT_RELOAD_PREFIXES = ('inf3d', 'common')


def _hot_reload():
    """清除本插件模块的 sys.modules 缓存，强制下次 import 从磁盘重读。

    关键：必须清除 importlib 的内部缓存（_bootstrap_external FileFinder/SourceFileLoader），
    否则 Python 3.14 会用旧的模块规格查找，导致 KeyError。
    清除顺序：先子模块后父包（倒序），避免半残状态。
    """
    import importlib

    # 收集要清除的模块（含父包），倒序排列（子模块先删）
    to_remove = sorted(
        (k for k in sys.modules
         if any(k == p or k.startswith(p + '.') for p in _HOT_RELOAD_PREFIXES)),
        reverse=True
    )
    for key in to_remove:
        # 先 invalidate 模块的 importlib 缓存
        mod = sys.modules.get(key)
        if mod is not None:
            try:
                # 清除该模块的 __loader__ 缓存（如果有的话）
                loader = getattr(mod, '__loader__', None)
                if loader and hasattr(loader, 'invalidate_caches'):
                    loader.invalidate_caches()
            except Exception:
                pass
            # 删 __dict__ 引用，帮助 GC
            try:
                mod_dict = getattr(mod, '__dict__', None)
                if mod_dict:
                    mod_dict.clear()
            except Exception:
                pass
        sys.modules.pop(key, None)

    # 清 importlib 的路径查找缓存
    try:
        importlib.invalidate_caches()
    except Exception:
        pass

    # 清 .pyc 缓存目录
    import shutil
    for sub in ('inf3d', 'common'):
        pyc_dir = os.path.join(_SCRIPT_DIR, sub, '__pycache__')
        if os.path.isdir(pyc_dir):
            shutil.rmtree(pyc_dir, ignore_errors=True)

    return list(reversed(to_remove))  # 返回正序方便看


def run(_context):
    """运行装配体导出"""
    # 热重载：如果 add-in (F3DRemoteControl) 已经清过缓存，这里跳过（避免双重清除导致状态损坏）
    # 检测方法：看 inf3d 是否已不在 sys.modules（被 add-in 清掉了）
    needs_reload = any(('inf3d' in sys.modules.get(k, '')) or k == 'inf3d'
                       for k in sys.modules)
    if needs_reload:
        purged = _hot_reload()
        if purged:
            app.log(f'[热重载] 清除 {len(purged)} 个模块缓存，重新从磁盘加载')

    # import 放在 run 内部（热重载后才能拿到最新代码）
    from inf3d.fusion_export_manager import FusionExportManager
    from common.data_types import MeshQuality
    from inf3d.logger import get_logger, log_performance_start, log_performance_end, initialize_logging

    # logger 在 try 内初始化，异常处理里兜底
    logger = None
    # logger 在 try 内初始化，异常处理里兜底
    logger = None
    try:
        output_dir = _get_output_directory()
        if not output_dir:
            ui.messageBox("导出已取消：未选择输出目录")
            return

        export_manager = FusionExportManager(mesh_quality=MeshQuality.MEDIUM)

        from common.backup_manager import BackupManager
        backup_manager = BackupManager()

        os.makedirs(output_dir, exist_ok=True)
        backup_success = backup_manager.create_backup(output_dir)

        logger = initialize_logging(output_dir)
        start_time = log_performance_start(logger, "F3DMaojoco导出")

        if backup_success:
            logger.info("备份创建成功")
        else:
            logger.warning("备份创建失败，继续执行导出")

        logger.info("开始导出装配体")
        export_manager.set_logger(logger)
        export_result = export_manager.export_assembly(output_dir)

        _show_export_summary(export_manager)

        if export_result and export_result.success:
            logger.info(f"导出完成: {len(export_result.data.components)} 个零部件, "
                        f"{len(export_result.data.joints)} 个关节")
        else:
            logger.warning("导出未成功完成")

        log_performance_end(logger, "F3DMaojoco导出", start_time)
        logger.info("装配体导出完成")
        ui.messageBox("导出完成！")

    except Exception as e:
        try:
            if logger:
                logger.error(f"导出失败: {str(e)}", exc_info=True)
        except Exception:
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
            return folder_dialog.folder
        return None
    except Exception as e:
        ui.messageBox(f"获取输出目录失败: {str(e)}")
        return None


def _show_export_summary(export_manager):
    """显示导出摘要"""
    logger = None
    try:
        summary = export_manager.get_export_summary()
        analysis = export_manager.analyze_export_results()

        from common.backup_manager import BackupManager
        backup_manager = BackupManager()
        backup_info = backup_manager.get_backup_info(summary['output_directory'])

        logger = get_logger()
        logger.info(f"导出摘要 - 零部件: {summary['total_components']}, 关节: {summary['total_joints']}")

        summary_msg = "导出摘要:\n\n"
        summary_msg += f"📁 输出目录: {summary['output_directory']}\n"
        summary_msg += f"🔧 网格质量: {summary['mesh_quality']}\n"
        summary_msg += f"📦 零部件总数: {summary['total_components']}\n"
        summary_msg += f"🔗 关节总数: {summary['total_joints']}\n"
        summary_msg += f"📄 STL文件数: {summary['stl_export']['exported_files_count']}\n"

        if backup_info.get('exists') and not backup_info.get('error'):
            backup_folders = backup_info.get('backup_folders', [])
            if backup_folders:
                summary_msg += f"💾 备份文件夹: {len(backup_folders)} 个\n"
                summary_msg += f"  • 最新备份: {backup_folders[0]}\n"

        summary_msg += "\n📊 详细统计:\n"
        summary_msg += f"  • 有实体的零部件: {analysis['components_with_bodies']}\n"
        summary_msg += f"  • 成功导出STL: {analysis['components_with_stl']}\n"
        summary_msg += f"  • 有子零部件: {analysis['components_with_children']}\n"
        summary_msg += f"  • 活动关节: {analysis['active_joints']}\n\n"

        summary_msg += "🔧 关节类型分布:\n"
        for joint_type, count in analysis['joint_types'].items():
            summary_msg += f"  • {joint_type}: {count}\n"

        summary_msg += "\n📋 生成文件:\n"
        summary_msg += "  • component_positions.json (零部件+关节+BRep曲面信息)\n"
        summary_msg += "  • stl_files/ (STL文件目录)\n"
        summary_msg += "  • export_description.md (导出描述文件)\n"
        summary_msg += "  • f3d_export.log (执行日志)\n"
        summary_msg += "  • backup/ (备份目录，带时间戳)\n"

        if backup_info.get('exists') and not backup_info.get('error'):
            backup_folders = backup_info.get('backup_folders', [])
            if backup_folders:
                summary_msg += "  • backup_*/ (备份文件夹，包含之前的导出结果)\n"

        summary_msg += "\n注意：STL文件使用毫米单位，位置信息使用米单位。"
        summary_msg += "\n💾 备份功能：每次导出前会自动将非备份文件移动到时间戳备份文件夹中。"

        ui.palettes.itemById('TextCommands').writeText(summary_msg)
        logger.info("导出摘要已显示给用户")

    except Exception as e:
        if logger:
            logger.error(f"显示导出摘要失败: {str(e)}")
        ui.messageBox(f"显示导出摘要失败: {str(e)}")


def stop(_context):
    """插件停止入口（保留用于兼容性）"""
    pass
