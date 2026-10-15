#!/usr/bin/env python3
"""Loopback-only robot console; ROS by default, optional explicit --mock demo."""

import argparse
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
import signal
import sys
from urllib.parse import urlsplit
import uuid

from mtc_webui.mock_backend import ApiError, MockBackend

STATIC = Path(__file__).resolve().parent / "static"
REPO = Path.cwd()  # Installed packages must not write into site-packages.
ASSETS = {"/": ("index.html", "text/html; charset=utf-8"),
          "/app.js": ("app.js", "text/javascript; charset=utf-8"),
          "/styles.css": ("styles.css", "text/css; charset=utf-8"),
          "/favicon.svg": ("favicon.svg", "image/svg+xml")}


class Server(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, backend, tunnel_ports=()):
        self.backend = backend
        self.tunnel_ports = tunnel_ports
        super().__init__(address, Handler)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        # State and task logs are structured and persisted by the backend.
        pass

    def _reply(self, status, value, content_type="application/json; charset=utf-8", download=None):
        data = value if isinstance(value, bytes) else json.dumps(value, ensure_ascii=False, allow_nan=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' data: blob:; connect-src 'self'; object-src 'none'; frame-ancestors 'none'; base-uri 'none'")
        if download:
            self.send_header("Content-Disposition", f'attachment; filename="{download}"')
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _check_origin(self):
        port = self.server.server_port
        allowed = {f"{host}:{p}" for p in (port, *self.server.tunnel_ports) for host in ("127.0.0.1", "localhost")}
        host = self.headers.get("Host", "")
        if host not in allowed:
            raise ApiError(403, "INVALID_HOST", "仅允许本机访问")
        origin = self.headers.get("Origin")
        if origin is not None and origin != f"http://{host}":
            raise ApiError(403, "CROSS_ORIGIN", "拒绝跨站请求")

    def _body(self):
        if self.headers.get("Content-Type", "").split(";")[0] != "application/json":
            raise ApiError(415, "INVALID_CONTENT_TYPE", "需要 application/json")
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0 or length > 8192:
                raise ValueError()
            return json.loads(self.rfile.read(length), parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
        except (ValueError, UnicodeError):
            raise ApiError(400, "INVALID_JSON", "JSON 请求无效或超过 8 KiB") from None

    def _route(self, write=False):
        try:
            self._check_origin()
            path = urlsplit(self.path).path
            backend = self.server.backend
            if not write:
                if path in ASSETS:
                    filename, content_type = ASSETS[path]
                    return self._reply(200, (STATIC / filename).read_bytes(), content_type)
                if path == "/api/snapshot":
                    return self._reply(200, backend.snapshot())
                if path == "/api/export":
                    return self._reply(200, backend.export_session(), download="webui-session.json")
                if path in ("/api/camera/color", "/api/camera/depth") and hasattr(backend, "camera_frame"):
                    return self._reply(200, backend.camera_frame(path.rsplit("/", 1)[1]), "image/jpeg")
                match = re.fullmatch(r"/api/tasks/([a-zA-Z0-9_-]{1,80})", path)
                if match:
                    return self._reply(200, backend.task_record(match[1]), download=f"{match[1]}.json")
            else:
                body = self._body()
                if not isinstance(body, dict):
                    raise ApiError(400, "INVALID_JSON", "请求必须是 JSON 对象")
                if path == "/api/tasks":
                    return self._reply(202, backend.submit(body))
                if path == "/api/stop" and body == {} and hasattr(backend, "request_stop"):
                    return self._reply(202, backend.request_stop())
                match = re.fullmatch(r"/api/tasks/([a-zA-Z0-9_-]{1,80})/cancel", path)
                if match and set(body) == {"goal_id"}:
                    return self._reply(202, backend.cancel(match[1], body["goal_id"]))
                if isinstance(backend, MockBackend) and path == "/api/demo/condition" and set(body) == {"condition"} and isinstance(body["condition"], str):
                    backend.configure(body["condition"])
                    return self._reply(200, {"condition": body["condition"]})
                if isinstance(backend, MockBackend) and path == "/api/demo/confirm-stop" and body == {}:
                    backend.confirm_stop()
                    return self._reply(200, {"mode": "mock", "stop_confirmed": True})
            raise ApiError(404, "NOT_FOUND", "接口不存在")
        except ApiError as error:
            self._reply(error.status, {"error": error.code, "message": error.message})

    def do_GET(self):
        self._route()

    def do_POST(self):
        self._route(write=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8767)
    parser.add_argument("--output-dir", type=Path, help="新目录；拒绝覆盖已有运行")
    parser.add_argument("--cpp-mock", action="store_true", help="连接统一启动的 C++ Mock ROS Action；不接硬件")
    parser.add_argument("--cpp-namespace", default="/mtc_cpp_mock")
    parser.add_argument("--preview-dir", type=Path, help="C++ Mock 页面只读显示已有相机预览文件")
    parser.add_argument("--mock", action="store_true", help="显式启动定时演示器；不连接 ROS")
    parser.add_argument("--config", type=Path, help="ROS 连接 JSON 配置")
    parser.add_argument("--ros-domain-id", type=int, help="实机 domain；也可沿用 ROS_DOMAIN_ID 环境变量")
    parser.add_argument("--readonly", action="store_true", help="只观察，禁用全部写操作")
    parser.add_argument("--tunnel-port", type=int, action="append", default=[], help="额外允许的 SSH 本地转发端口")
    parser.add_argument("--hardware-config", type=Path, help="单次已验证实机轨迹配置")
    parser.add_argument("--auth-stdin", action="store_true", help="仅从 stdin 读取本次内存凭据")
    args = parser.parse_args()
    if args.preview_dir and not args.cpp_mock and not args.hardware_config:
        parser.error("--preview-dir 仅用于 C++ Mock 观察页面")
    if not 1 <= args.port <= 65535 or any(not 1 <= p <= 65535 for p in args.tunnel_port):
        parser.error("端口必须为 1–65535")
    if args.mock and (args.cpp_mock or args.config or args.ros_domain_id is not None or args.readonly):
        parser.error("--mock 不能与 ROS 连接/只读参数混用")
    output_dir = args.output_dir or REPO / "outputs" / f"webui-{datetime.now():%Y%m%d-%H%M%S}-{uuid.uuid4().hex[:6]}"
    if args.hardware_config:
        if args.mock or args.cpp_mock or args.config or args.readonly:
            parser.error("实机轨迹入口不能与其它后端混用")
        from mtc_webui.hardware_backend import HardwareBackend, HardwareTransport
        settings = json.loads(args.hardware_config.read_text())
        credentials = json.loads(sys.stdin.readline()) if args.auth_stdin else {}
        backend = HardwareBackend(settings, output_dir)
        transport = HardwareTransport(backend, credentials)
    elif args.mock:
        backend = MockBackend(output_dir)
    elif args.cpp_mock:
        from mtc_webui.cpp_mock import CppMockBackend, CppMockTransport, cpp_config
        config = cpp_config(args.cpp_namespace, args.ros_domain_id, args.readonly)
        if args.preview_dir:
            from mtc_webui.file_preview import FilePreviewBackend
            backend = FilePreviewBackend(config, output_dir, preview_dir=args.preview_dir)
        else:
            backend = CppMockBackend(config, output_dir)
    else:
        from mtc_webui.config import RosConfig
        from mtc_webui.ros_backend import RosBackend
        from mtc_webui.ros_transport import RosTransport
        try:
            config = RosConfig.load(args.config, args.ros_domain_id, args.readonly)
        except (ValueError, TypeError, OSError) as error:
            parser.error(str(error))
        backend = RosBackend(config, output_dir)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    with Server(("127.0.0.1", args.port), backend, args.tunnel_port) as server:
        if not args.mock:
            (transport if args.hardware_config else CppMockTransport(backend) if args.cpp_mock else RosTransport(backend)).start()
        print(f"{'C++ Mock ROS 操作台' if args.cpp_mock else 'Mock 演练' if args.mock else '实机操作台'}：http://127.0.0.1:{server.server_port}/", flush=True)
        print(f"会话日志：{output_dir}", flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            print("\n操作台已关闭，会话记录已保留。", flush=True)
        finally:
            if hasattr(backend, "close"):
                backend.close()


if __name__ == "__main__":
    main()
