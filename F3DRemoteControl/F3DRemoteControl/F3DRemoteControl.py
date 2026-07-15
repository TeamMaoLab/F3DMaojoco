# -*- coding: utf-8 -*-
"""F3DRemoteControl - Fusion 360 驻留型 add-in，开本地 HTTP 服务。

外部（WSL 命令行 / 浏览器）发 HTTP 请求就能远程控制 Fusion：
  GET /ping            → 测试连通性
  GET /reload          → 热重载：清除 F3DMaojocoScripts/F3DMaojocoWin 模块缓存
  GET /exec?code=...   → 在 Fusion 进程内执行任意 Python 代码（调试用）
  GET /run_export      → 触发 F3DMaojoco 导出（需先选好输出目录或传 ?dir=路径）

原理：
  - Fusion 的 run() 在主线程，stop() 卸载时清理
  - HTTP 服务跑后台线程（threading），只负责接请求
  - 热重载只清 sys.modules（不碰 Fusion API），线程安全
  - 需要 Fusion API 的操作（导出等）通过命令队列，由主线程定时执行

用法（在 Fusion 的 Scripts and Add-Ins → Add-Ins 选项卡 → 添加本目录）：
  添加后会自动启动，底部显示"F3DRemoteControl 运行中 @ 127.0.0.1:9099"
  外部测试: curl http://127.0.0.1:9099/ping
"""
import os
import sys
import json
import threading
import traceback
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

import adsk.core
import adsk.fusion

app = adsk.core.Application.get()
ui = app.userInterface

# 配置
PORT = 9099
# 要热重载的模块前缀（F3DMaojoco 插件的代码）
HOT_RELOAD_PREFIXES = ('inf3d', 'common')
# F3DMaojoco 脚本目录（热重载时加到 sys.path）
SCRIPT_DIRS = []

_server = None
_server_thread = None
# 主线程待执行的命令队列 [(cmd_type, params), ...]
_cmd_queue = []
_cmd_queue_lock = threading.Lock()


def _find_script_dirs():
    """查找 F3DMaojocoWin / F3DMaojocoScripts 脚本目录。"""
    dirs = []
    # 常见位置：与本 add-in 同级的的项目根目录下
    here = os.path.dirname(os.path.abspath(__file__))
    # 向上找项目根（含 F3DMaojocoWin 或 F3DMaojocoScripts 的目录）
    for parent in [here, os.path.dirname(here), os.path.dirname(os.path.dirname(here))]:
        for sub in ['F3DMaojocoWin/F3DMaojocoWin', 'F3DMaojocoScripts']:
            p = os.path.join(parent, sub)
            if os.path.isdir(p) and p not in dirs:
                dirs.append(p)
    return dirs


def _do_hot_reload():
    """清除 F3DMaojoco 插件模块的 sys.modules 缓存 + __pycache__。

    Python 3.14 健壮清除：倒序删（子模块先于父包），清 __dict__，invalidate importlib 缓存。
    """
    import importlib

    to_remove = sorted(
        (k for k in sys.modules
         if any(k == p or k.startswith(p + '.') for p in HOT_RELOAD_PREFIXES)),
        reverse=True
    )
    for key in to_remove:
        mod = sys.modules.get(key)
        if mod is not None:
            try:
                loader = getattr(mod, '__loader__', None)
                if loader and hasattr(loader, 'invalidate_caches'):
                    loader.invalidate_caches()
            except Exception:
                pass
            try:
                getattr(mod, '__dict__', {}).clear()
            except Exception:
                pass
        sys.modules.pop(key, None)

    try:
        importlib.invalidate_caches()
    except Exception:
        pass

    # 清 __pycache__
    import shutil
    for d in SCRIPT_DIRS:
        for sub in ('inf3d', 'common'):
            pyc_dir = os.path.join(d, sub, '__pycache__')
            if os.path.isdir(pyc_dir):
                shutil.rmtree(pyc_dir, ignore_errors=True)
    # 确保 script dirs 在 sys.path
    for d in SCRIPT_DIRS:
        if d not in sys.path:
            sys.path.insert(0, d)
    return list(reversed(to_remove))


class Handler(BaseHTTPRequestHandler):
    """HTTP 请求处理器。"""

    def log_message(self, format, *args):
        """静默默认日志（避免刷屏）。"""
        pass

    def _json(self, data, code=200):
        body = json.dumps(data, ensure_ascii=False, indent=2).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path
        params = parse_qs(parsed.query)

        try:
            if path == '/ping':
                self._json({'ok': True, 'msg': 'F3DRemoteControl 运行中',
                            'python': sys.version.split()[0],
                            'fusion_doc': app.activeDocument.name if app.activeDocument else '无'})

            elif path == '/reload':
                removed = _do_hot_reload()
                self._json({'ok': True, 'cleared': len(removed), 'modules': removed[:20],
                            'script_dirs': SCRIPT_DIRS})

            elif path == '/exec':
                code = params.get('code', [''])[0]
                if not code:
                    self._json({'ok': False, 'error': '缺少 code 参数'}, 400)
                    return
                # 执行任意代码（调试用，仅 localhost）
                # 用本模块 globals() 作上下文，暴露 __file__/SCRIPT_DIRS/sys 等
                # 约定：代码末尾赋值 _result = ... 取返回值（保持原生类型）
                try:
                    g = globals()
                    g.pop('_result', None)  # 清上次的残留
                    exec(code, g)
                    result = g.get('_result', None)
                    self._json({'ok': True, 'result': result})
                except Exception as e:
                    self._json({'ok': False, 'error': str(e), 'traceback': traceback.format_exc()}, 500)

            elif path == '/export':
                # 参数化导出：?dir=输出目录&quality=15
                self._json(self._handle_export(params))

            elif path == '/model':
                # 查询装配体结构（零件/关节/BRep统计）
                self._json(self._handle_model(params))

            elif path == '/brep_stats':
                # BRep 曲面类型分布（快速看模型复杂度）
                self._json(self._handle_brep_stats(params))

            elif path == '/modules':
                # 列出当前 sys.modules 里我们的模块
                ours = sorted(k for k in sys.modules
                              if any(k == p or k.startswith(p + '.') for p in HOT_RELOAD_PREFIXES))
                self._json({'ok': True, 'count': len(ours), 'modules': ours})

            else:
                self._json({'ok': False, 'error': '未知路径: ' + path,
                            'routes': ['/ping', '/reload', '/exec?code=...', '/export?dir=',
                                       '/model', '/brep_stats', '/modules']}, 404)

        except Exception as e:
            self._json({'ok': False, 'error': str(e), 'traceback': traceback.format_exc()}, 500)

    # ============ 专用端点处理函数 ============

    def _ensure_path(self):
        """确保脚本目录在 sys.path。"""
        for d in SCRIPT_DIRS:
            if d not in sys.path:
                sys.path.insert(0, d)

    def _handle_export(self, params):
        """参数化导出：dir=输出目录 quality=精度(8/11/13/15)。

        quality 枚举：8=Low 11=Normal 13=High 15=VeryHigh（默认）
        """
        try:
            out_dir = params.get('dir', [''])[0]
            if not out_dir:
                return {'ok': False, 'error': '缺少 dir 参数，用法 /export?dir=路径&quality=15'}
            quality = int(params.get('quality', ['15'])[0])

            self._ensure_path()
            # 清缓存确保用最新代码
            _do_hot_reload()

            from inf3d.fusion_export_manager import FusionExportManager
            from common.data_types import MeshQuality
            from inf3d.logger import initialize_logging

            os.makedirs(out_dir, exist_ok=True)
            logger = initialize_logging(out_dir)
            mgr = FusionExportManager(mesh_quality=MeshQuality.MEDIUM)
            mgr.set_logger(logger)
            result = mgr.export_assembly(out_dir)

            # 读 BRep 网格结果（如果有）
            brep_path = os.path.join(out_dir, 'brep_geometry.json')
            brep_info = None
            if os.path.exists(brep_path):
                brep_info = {
                    'size_kb': round(os.path.getsize(brep_path) / 1024, 0),
                    'quality': quality
                }

            return {
                'ok': result.success,
                'output_dir': result.output_directory,
                'stl_count': len(result.stl_files) if result.stl_files else 0,
                'brep_geometry': brep_info,
                'error': result.error_message
            }
        except Exception as e:
            return {'ok': False, 'error': str(e), 'traceback': traceback.format_exc()[-500:]}

    def _handle_model(self, params):
        """查询当前装配体结构：零件列表 + 关节数 + BRep 曲面统计。

        不导出文件，直接读 Fusion API 返回 JSON。
        """
        try:
            design = app.activeProduct
            if not design or design.objectType != 'fusion::Design':
                return {'ok': False, 'error': '无活动 Design'}
            root = design.rootComponent

            parts = []
            joint_count = 0

            def visit(occ, depth=0):
                nonlocal joint_count
                comp = occ.component
                if not comp:
                    return
                if comp.name.startswith('COL_'):
                    return
                # 统计 BRep 曲面
                face_types = {}
                if comp.bRepBodies.count > 0:
                    for bi in range(comp.bRepBodies.count):
                        body = comp.bRepBodies.item(bi)
                        for face in body.faces:
                            geo = face.geometry
                            if geo:
                                t = geo.objectType.split('::')[-1]
                                face_types[t] = face_types.get(t, 0) + 1
                parts.append({
                    'name': comp.name,
                    'occurrence': occ.name,
                    'bodies': comp.bRepBodies.count,
                    'faces': sum(face_types.values()),
                    'surface_types': face_types if face_types else None,
                    'depth': depth,
                })
                # 关节
                joint_count += comp.joints.count
                for child in occ.childOccurrences:
                    visit(child, depth + 1)

            for occ in root.occurrences:
                visit(occ)

            return {
                'ok': True,
                'document': app.activeDocument.name if app.activeDocument else '无',
                'root_component': root.name,
                'parts': parts,
                'part_count': len(parts),
                'joint_count': joint_count,
            }
        except Exception as e:
            return {'ok': False, 'error': str(e), 'traceback': traceback.format_exc()[-500:]}

    def _handle_brep_stats(self, params):
        """BRep 曲面类型分布统计（全装配体）。"""
        try:
            design = app.activeProduct
            if not design:
                return {'ok': False, 'error': '无活动 Design'}
            root = design.rootComponent
            type_stats = {}
            total_faces = 0
            part_count = 0

            def visit(occ):
                nonlocal total_faces, part_count
                comp = occ.component
                if not comp or comp.name.startswith('COL_'):
                    return
                if comp.bRepBodies.count == 0:
                    for c in occ.childOccurrences:
                        visit(c)
                    return
                part_count += 1
                for bi in range(comp.bRepBodies.count):
                    body = comp.bRepBodies.item(bi)
                    for face in body.faces:
                        geo = face.geometry
                        if geo:
                            t = geo.objectType.split('::')[-1]
                            type_stats[t] = type_stats.get(t, 0) + 1
                            total_faces += 1
                for c in occ.childOccurrences:
                    visit(c)

            for occ in root.occurrences:
                visit(occ)

            # 按 数量降序 排
            sorted_types = sorted(type_stats.items(), key=lambda x: -x[1])
            return {
                'ok': True,
                'total_faces': total_faces,
                'part_count': part_count,
                'surface_types': {t: n for t, n in sorted_types},
                'regular_pct': round(
                    sum(n for t, n in type_stats.items()
                        if t in ('Plane', 'Cylinder', 'Cone', 'Sphere', 'Torus'))
                    / max(total_faces, 1) * 100, 1
                ),
            }
        except Exception as e:
            return {'ok': False, 'error': str(e), 'traceback': traceback.format_exc()[-500:]}


def _start_server():
    """启动 HTTP 服务（后台线程）。"""
    global _server, _server_thread
    _server = HTTPServer(('127.0.0.1', PORT), Handler)
    _server_thread = threading.Thread(target=_server.serve_forever, daemon=True)
    _server_thread.start()


def run(context):
    """add-in 启动入口。"""
    global SCRIPT_DIRS
    try:
        SCRIPT_DIRS = _find_script_dirs()
        _start_server()
        msg = (f'F3DRemoteControl 运行中 @ http://127.0.0.1:{PORT}\n'
               f'脚本目录: {SCRIPT_DIRS}\n'
               f'测试: curl http://127.0.0.1:{PORT}/ping')
        ui.palettes.itemById('TextCommands').writeText(msg)
    except Exception as e:
        ui.messageBox('F3DRemoteControl 启动失败:\n{}'.format(traceback.format_exc()))


def stop(context):
    """add-in 卸载入口。"""
    global _server
    if _server:
        _server.shutdown()
        _server = None
