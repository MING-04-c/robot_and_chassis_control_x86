#!/usr/bin/env python3
"""麒麟本机任务控制页面；同进程运行收图、状态监听与 HTTP 页面。"""
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import secrets
import threading
from uuid import uuid4
import webbrowser

from industrial_pc_protocol import decode, validate_result
from listen_robot import RobotListener
from robot_tcp_common import configuration, parser, request
from send_points import prepare_result

PAGE = Path(__file__).with_name('control_page.html')
ACTIONS = {'start', 'detect', 'go_home', 'stop'}


def make_handler(listener, config, token):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format, *args):
            pass

        def send_json(self, code, value):
            body = json.dumps(value, ensure_ascii=False).encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == '/':
                body = PAGE.read_bytes()
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.send_header('X-Content-Type-Options', 'nosniff')
                self.end_headers()
                self.wfile.write(body)
            elif self.path == '/api/state':
                self.send_json(200, listener.snapshot())
            else:
                self.send_json(404, {'error': '页面不存在'})

        def do_POST(self):
            if self.path not in ('/api/action', '/api/points'):
                return self.send_json(404, {'error': '接口不存在'})
            origin = self.headers.get('Origin')
            expected = 'http://127.0.0.1:%d' % self.server.server_port
            if self.headers.get('X-Control-Token') != token or (origin and origin != expected):
                return self.send_json(403, {'error': '仅允许本机控制页面发起操作'})
            try:
                length = int(self.headers.get('Content-Length', '-1'))
                if not 0 <= length <= 8192:
                    raise ValueError('请求体长度无效')
                body = json.loads(self.rfile.read(length))
                if not isinstance(body, dict):
                    raise ValueError('请求必须为 JSON 对象')
                if self.path == '/api/action':
                    if set(body) != {'action'} or body['action'] not in ACTIONS:
                        raise ValueError('不支持的任务动作')
                    payload = {'schema': 'paint_robot_task_request_v1',
                               'message_id': uuid4().hex, 'action': body['action']}
                else:
                    if set(body) != {'file'} or not isinstance(body['file'], str) or not body['file'].strip():
                        raise ValueError('请输入麒麟本机结果 JSON 路径')
                    file = Path(body['file']).expanduser()
                    if not file.is_file() or file.stat().st_size > 262144:
                        raise ValueError('结果文件不存在或过大')
                    payload = prepare_result(decode(file.read_bytes()), config)
                    kind, points, _ = validate_result(payload)
                    if kind == 'demo' or not points:
                        raise ValueError('只允许发送本批真实检测点，不允许 DEMO 或空点')
                    camera = ((listener.snapshot().get('robot_status') or {}).get('camera_detection') or {})
                    data = camera.get('data') or {}
                    if not camera.get('fresh') or data.get('state') != 'WAIT_RESULT':
                        raise ValueError('机器人当前未处于 WAIT_RESULT，请先完成本轮采图')
                    if payload.get('capture_manifest_sha256') != data.get('capture_manifest_sha256'):
                        raise ValueError('结果文件的照片清单哈希与当前轮不一致')
                response = request(config, payload)
                self.send_json(200, {'message_id': payload['message_id'], 'response': response})
            except (OSError, ValueError, EOFError, KeyError, TypeError) as error:
                self.send_json(400, {'error': str(error)})
    return Handler


def main():
    cli = parser(__doc__)
    cli.add_argument('--port', type=int, default=8765, help='本机页面端口，默认 8765')
    cli.add_argument('--no-browser', action='store_true', help='不自动打开浏览器')
    args = cli.parse_args()
    if not 1 <= args.port <= 65535:
        cli.error('端口必须为 1～65535')
    config = configuration(args)
    listener = RobotListener(config)
    token = secrets.token_urlsafe(32)
    page = PAGE.read_text(encoding='utf-8').replace('__CONTROL_TOKEN__', token)
    # 页面文件保持静态；每次启动生成新令牌，仅送到本机浏览器。
    class PageHandler(make_handler(listener, config, token)):
        def do_GET(self):
            if self.path == '/':
                body = page.encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)
            else:
                super().do_GET()
    server = ThreadingHTTPServer(('127.0.0.1', args.port), PageHandler)
    thread = threading.Thread(target=listener.run, daemon=True)
    thread.start()
    url = 'http://127.0.0.1:%d/' % args.port
    print('麒麟控制页面：'+url, flush=True)
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever(poll_interval=.25)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        listener.stop.set()
        thread.join(timeout=3)


if __name__ == '__main__':
    main()
