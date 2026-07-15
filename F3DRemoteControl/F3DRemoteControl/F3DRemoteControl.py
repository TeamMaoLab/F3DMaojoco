# -*- coding: utf-8 -*-
"""F3DRemoteControl - Fusion 360 驻留型 add-in，开本地 HTTP 服务（线程安全版）。

架构（解决 Fusion API 线程安全问题）：
  - HTTP 服务跑后台线程，只接请求
  - 需要执行的请求（调 Fusion API 的）放入队列
  - 主线程用 adsk.core.TimerEventHandler 定时轮询队列执行
  - 后台线程用 Event 等待主线程完成后返回结果

这样所有 Fusion API 调用都在主线程，不会崩溃。

API:
  GET /ping            → 连通测试（纯 Python，不走队列）
  GET /reload          → 热重载模块缓存（纯 Python，不走队列）
  GET /exec?code=...   → 执行任意代码（走队列，主线程执行）
  GET /export?dir=...  → 参数化导出（走队列）
  GET /model           → 查询装配体结构（走队列）
  GET /brep_stats      → BRep 曲面统计（走队列）
  GET /modules         → 列出已加载模块（纯 Python，不走队列）
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

PORT = 9099
HOT_RELOAD_PREFIXES = ('inf3d', 'common')
SCRIPT_DIRS = []

_server = None
_server_thread = None

# ============ 主线程任务队列 ============
# 每个 pending: {fn: 可调用, event: threading.Event, result: None/值, error: None/异常}
_pending_tasks = []
_pending_lock = threading.Lock()
# 主线程调度：用 Fusion CustomEvent（notify 在主线程被调用）
_CUSTOM_EVENT_ID = 'F3DRemoteControlMainTask'
_custom_event = None
_custom_handler = None


def _enqueue_main(fn):
    """把一个函数放入主线程队列，阻塞等待结果。返回 (result, error)。

    后台线程调用此函数 → 放队列 → fireCustomEvent 通知主线程 → 主线程 notify 里执行 → 返回。
    """
    task = {'fn': fn, 'event': threading.Event(), 'result': None, 'error': None}
    with _pending_lock:
        _pending_tasks.append(task)
    # 通知主线程有任务
    app.fireCustomEvent(_CUSTOM_EVENT_ID)
    # 阻塞等待主线程执行完
    task['event'].wait(timeout=120)
    if not task['event'].is_set():
        return None, TimeoutError('主线程 120 秒未执行')
    return task['result'], task['error']


def _drain_queue():
    """主线程：执行所有待处理任务。由 CustomEvent.notify 触发。"""
    with _pending_lock:
        tasks = list(_pending_tasks)
        _pending_tasks.clear()
    for task in tasks:
        try:
            task['result'] = task['fn']()
        except Exception as e:
            task['error'] = e
        task['event'].set()


class _MainTaskHandler(adsk.core.CustomEventHandler):
    """CustomEvent 处理器：notify 在主线程被调用。"""

    def notify(self, args):
        try:
            _drain_queue()
        except Exception:
            pass


# ============ 业务逻辑 ============

def _find_script_dirs():
    here = os.path.dirname(os.path.abspath(__file__))
    dirs = []
    for parent in [here, os.path.dirname(here), os.path.dirname(os.path.dirname(here))]:
        for sub in ['F3DMaojocoWin/F3DMaojocoWin', 'F3DMaojocoScripts']:
            p = os.path.join(parent, sub)
            if os.path.isdir(p) and p not in dirs:
                dirs.append(p)
    return dirs


def _do_hot_reload():
    import importlib, shutil
    to_remove = sorted(
        (k for k in sys.modules
         if any(k == p or k.startswith(p + '.') for p in HOT_RELOAD_PREFIXES)),
        reverse=True
    )
    for key in to_remove:
        mod = sys.modules.get(key)
        if mod is not None:
            try:
                getattr(mod, '__dict__', {}).clear()
            except Exception:
                pass
        sys.modules.pop(key, None)
    try:
        importlib.invalidate_caches()
    except Exception:
        pass
    for d in SCRIPT_DIRS:
        for sub in ('inf3d', 'common'):
            pyc_dir = os.path.join(d, sub, '__pycache__')
            if os.path.isdir(pyc_dir):
                shutil.rmtree(pyc_dir, ignore_errors=True)
        if d not in sys.path:
            sys.path.insert(0, d)
    return list(reversed(to_remove))


def _do_exec(code):
    """执行任意代码，返回 _result。在主线程调用。"""
    g = globals()
    g.pop('_result', None)
    exec(code, g)
    return g.get('_result', None)


def _do_export(out_dir, quality):
    """执行导出。在主线程调用。"""
    for d in SCRIPT_DIRS:
        if d not in sys.path:
            sys.path.insert(0, d)
    _do_hot_reload()
    from inf3d.fusion_export_manager import FusionExportManager
    from common.data_types import MeshQuality
    from inf3d.logger import initialize_logging
    os.makedirs(out_dir, exist_ok=True)
    logger = initialize_logging(out_dir)
    mgr = FusionExportManager(mesh_quality=MeshQuality.MEDIUM)
    mgr.set_logger(logger)
    result = mgr.export_assembly(out_dir)
    brep_path = os.path.join(out_dir, 'brep_geometry.json')
    brep_info = None
    if os.path.exists(brep_path):
        brep_info = {'size_kb': round(os.path.getsize(brep_path) / 1024, 0)}
    return {
        'ok': result.success,
        'output_dir': result.output_directory,
        'stl_count': len(result.stl_files) if result.stl_files else 0,
        'brep_geometry': brep_info,
        'error': result.error_message
    }


def _do_model():
    """查询装配体结构。在主线程调用。"""
    design = app.activeProduct
    if not design or design.objectType != 'fusion::Design':
        return {'ok': False, 'error': '无活动 Design'}
    root = design.rootComponent
    parts = []
    joint_count = [0]

    def visit(occ, depth=0):
        comp = occ.component
        if not comp or comp.name.startswith('COL_'):
            return
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
            'name': comp.name, 'occurrence': occ.name,
            'bodies': comp.bRepBodies.count,
            'faces': sum(face_types.values()),
            'surface_types': face_types if face_types else None,
            'depth': depth,
        })
        joint_count[0] += comp.joints.count
        for child in occ.childOccurrences:
            visit(child, depth + 1)

    for occ in root.occurrences:
        visit(occ)
    return {
        'ok': True,
        'document': app.activeDocument.name if app.activeDocument else '无',
        'root': root.name,
        'parts': parts, 'part_count': len(parts),
        'joint_count': joint_count[0],
    }


def _do_brep_stats():
    """BRep 曲面统计。在主线程调用。"""
    design = app.activeProduct
    if not design:
        return {'ok': False, 'error': '无活动 Design'}
    root = design.rootComponent
    type_stats = {}
    total_faces = [0]
    part_count = [0]

    def visit(occ):
        comp = occ.component
        if not comp or comp.name.startswith('COL_'):
            return
        if comp.bRepBodies.count == 0:
            for c in occ.childOccurrences:
                visit(c)
            return
        part_count[0] += 1
        for bi in range(comp.bRepBodies.count):
            body = comp.bRepBodies.item(bi)
            for face in body.faces:
                geo = face.geometry
                if geo:
                    t = geo.objectType.split('::')[-1]
                    type_stats[t] = type_stats.get(t, 0) + 1
                    total_faces[0] += 1
        for c in occ.childOccurrences:
            visit(c)

    for occ in root.occurrences:
        visit(occ)
    tf = total_faces[0]
    sorted_types = sorted(type_stats.items(), key=lambda x: -x[1])
    return {
        'ok': True, 'total_faces': tf, 'part_count': part_count[0],
        'surface_types': {t: n for t, n in sorted_types},
        'regular_pct': round(
            sum(n for t, n in type_stats.items()
                if t in ('Plane', 'Cylinder', 'Cone', 'Sphere', 'Torus'))
            / max(tf, 1) * 100, 1),
    }


# ============ HTTP Handler ============

class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _json(self, data, code=200):
        body = json.dumps(data, ensure_ascii=False, indent=2, default=str).encode('utf-8')
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
            # ---- 纯 Python 端点（不走队列，后台线程直接执行）----
            if path == '/ping':
                self._json({'ok': True, 'msg': 'F3DRemoteControl 运行中',
                            'python': sys.version.split()[0],
                            'thread_safe': True})

            elif path == '/reload':
                removed = _do_hot_reload()
                self._json({'ok': True, 'cleared': len(removed),
                            'modules': removed[:20], 'script_dirs': SCRIPT_DIRS})

            elif path == '/modules':
                ours = sorted(k for k in sys.modules
                              if any(k == p or k.startswith(p + '.') for p in HOT_RELOAD_PREFIXES))
                self._json({'ok': True, 'count': len(ours), 'modules': ours})

            # ---- Fusion API 端点（走队列，主线程执行）----
            elif path == '/exec':
                code = params.get('code', [''])[0]
                if not code:
                    self._json({'ok': False, 'error': '缺少 code 参数'}, 400)
                    return
                result, error = _enqueue_main(lambda: _do_exec(code))
                if error:
                    self._json({'ok': False, 'error': str(error),
                                'traceback': traceback.format_exc()}, 500)
                else:
                    self._json({'ok': True, 'result': result})

            elif path == '/export':
                out_dir = params.get('dir', [''])[0]
                if not out_dir:
                    self._json({'ok': False, 'error': '缺少 dir 参数'}, 400)
                    return
                quality = int(params.get('quality', ['15'])[0])
                result, error = _enqueue_main(lambda: _do_export(out_dir, quality))
                if error:
                    self._json({'ok': False, 'error': str(error),
                                'traceback': traceback.format_exc()}, 500)
                else:
                    self._json(result)

            elif path == '/model':
                result, error = _enqueue_main(_do_model)
                self._json(result if not error else {'ok': False, 'error': str(error)})

            elif path == '/brep_stats':
                result, error = _enqueue_main(_do_brep_stats)
                self._json(result if not error else {'ok': False, 'error': str(error)})

            else:
                self._json({'ok': False, 'error': '未知路径: ' + path,
                            'routes': ['/ping', '/reload', '/exec?code=',
                                       '/export?dir=', '/model', '/brep_stats', '/modules']}, 404)

        except Exception as e:
            self._json({'ok': False, 'error': str(e), 'traceback': traceback.format_exc()}, 500)


# ============ 启动/停止 ============

def _start_server():
    global _server, _server_thread
    _server = HTTPServer(('127.0.0.1', PORT), Handler)
    _server_thread = threading.Thread(target=_server.serve_forever, daemon=True)
    _server_thread.start()


def run(context):
    global SCRIPT_DIRS, _custom_event, _custom_handler
    try:
        SCRIPT_DIRS = _find_script_dirs()
        _start_server()
        # 注册 CustomEvent（主线程调度）
        _custom_event = app.registerCustomEvent(_CUSTOM_EVENT_ID)
        _custom_handler = _MainTaskHandler()
        _custom_event.add(_custom_handler)
        msg = (f'F3DRemoteControl 运行中 @ http://127.0.0.1:{PORT}\n'
               f'线程安全模式（CustomEvent 主线程调度）\n'
               f'脚本目录: {SCRIPT_DIRS}')
        ui.palettes.itemById('TextCommands').writeText(msg)
    except Exception:
        ui.messageBox('F3DRemoteControl 启动失败:\n{}'.format(traceback.format_exc()))


def stop(context):
    global _server, _custom_event, _custom_handler
    if _server:
        _server.shutdown()
        _server = None
    if _custom_event and _custom_handler:
        _custom_event.remove(_custom_handler)
        _custom_handler = None
    if _custom_event:
        app.unregisterCustomEvent(_CUSTOM_EVENT_ID)
        _custom_event = None
