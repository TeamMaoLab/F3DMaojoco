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
    """清除 F3DMaojoco 插件模块的 sys.modules 缓存 + __pycache__。返回清除的模块列表。"""
    removed = []
    for key in list(sys.modules.keys()):
        if any(key == p or key.startswith(p + '.') for p in HOT_RELOAD_PREFIXES):
            del sys.modules[key]
            removed.append(key)
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
    return removed


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
                try:
                    local_ns = {'adsk': adsk, 'app': app, 'ui': ui, 'sys': sys, 'os': os}
                    result = exec(code, {'__name__': '__remote__'}, local_ns)
                    self._json({'ok': True, 'result': str(result)})
                except Exception as e:
                    self._json({'ok': False, 'error': str(e), 'traceback': traceback.format_exc()}, 500)

            elif path == '/modules':
                # 列出当前 sys.modules 里我们的模块
                ours = sorted(k for k in sys.modules
                              if any(k == p or k.startswith(p + '.') for p in HOT_RELOAD_PREFIXES))
                self._json({'ok': True, 'count': len(ours), 'modules': ours})

            else:
                self._json({'ok': False, 'error': '未知路径: ' + path,
                            'routes': ['/ping', '/reload', '/exec?code=...', '/modules']}, 404)

        except Exception as e:
            self._json({'ok': False, 'error': str(e), 'traceback': traceback.format_exc()}, 500)


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
