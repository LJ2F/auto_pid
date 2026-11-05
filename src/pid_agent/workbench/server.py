from __future__ import annotations

import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import secrets
from urllib.parse import parse_qs, urlparse

from .schema import schema
from .service import Workbench


def create_server(service: Workbench, host="127.0.0.1", port=8766, allowed_origins=()):
    if host not in ("127.0.0.1", "localhost"):
        raise ValueError("本地桥接服务仅监听回环地址")
    token = secrets.token_urlsafe(32)
    web = Path(__file__).with_name("web")
    permitted = set(allowed_origins)
    for origin in permitted:
        parsed = urlparse(origin)
        if parsed.scheme not in ("http", "https") or not parsed.netloc or parsed.path not in ("", "/"):
            raise ValueError("静态站点来源必须是完整 HTTP(S) origin")

    class Handler(BaseHTTPRequestHandler):
        def allowed(self):
            valid_hosts = {f"127.0.0.1:{self.server.server_port}", f"localhost:{self.server.server_port}"}
            if self.headers.get("Host") not in valid_hosts:
                return False
            origin = self.headers.get("Origin")
            return origin is None or origin in permitted or origin in {"http://"+h for h in valid_hosts}

        def send_content_headers(self, status, content_type, size):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(size))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            origin = self.headers.get("Origin")
            if origin and self.allowed():
                self.send_header("Access-Control-Allow-Origin", origin)
                self.send_header("Vary", "Origin")
            self.end_headers()

        def respond(self, status, payload):
            data = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode()
            self.send_content_headers(status, "application/json; charset=utf-8", len(data))
            self.wfile.write(data)

        def do_OPTIONS(self):
            if not self.allowed():
                self.respond(403, {"error": "来源未授权"})
                return
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", self.headers.get("Origin", ""))
            self.send_header("Access-Control-Allow-Headers", "Content-Type, X-Workbench-Token")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Content-Length", "0")
            self.end_headers()

        def do_GET(self):
            if not self.allowed():
                self.respond(403, {"error": "来源未授权"})
                return
            parsed = urlparse(self.path)
            try:
                if parsed.path == "/api/session":
                    self.respond(200, {"token": token, "schema": schema()})
                elif parsed.path == "/api/state":
                    self.respond(200, service.snapshot(int(parse_qs(parsed.query).get("after", ["0"])[0])))
                elif parsed.path == "/api/ports":
                    self.respond(200, {"ports": service.ports()})
                elif parsed.path == "/api/export":
                    content_type, filename, data = service.export(parse_qs(parsed.query).get("format", ["json"])[0])
                    self.send_response(200)
                    self.send_header("Content-Type", content_type)
                    self.send_header("Content-Disposition", f'attachment; filename="{filename}"')
                    self.send_header("Content-Length", str(len(data)))
                    origin = self.headers.get("Origin")
                    if origin:
                        self.send_header("Access-Control-Allow-Origin", origin)
                        self.send_header("Vary", "Origin")
                    self.end_headers()
                    self.wfile.write(data)
                elif parsed.path in ("/", "/index.html", "/app.js", "/style.css"):
                    name = "index.html" if parsed.path == "/" else parsed.path[1:]
                    data = (web/name).read_bytes()
                    mime = {".html": "text/html", ".js": "text/javascript", ".css": "text/css"}[Path(name).suffix]
                    self.send_content_headers(200, mime+"; charset=utf-8", len(data))
                    self.wfile.write(data)
                else:
                    self.respond(404, {"error": "not found"})
            except (ValueError, KeyError, TypeError) as exc:
                self.respond(400, {"error": str(exc)})
            except (RuntimeError, OSError) as exc:
                self.respond(409, {"error": str(exc)})

        def do_POST(self):
            if not self.allowed() or not secrets.compare_digest(self.headers.get("X-Workbench-Token", ""), token):
                self.respond(403, {"error": "本地会话或来源无效，请重新连接工作台"})
                return
            if self.headers.get_content_type() != "application/json":
                self.respond(415, {"error": "需要 application/json"})
                return
            actions = {"/api/connect": service.connect, "/api/disconnect": service.disconnect,
                       "/api/start": service.start, "/api/stop": service.stop, "/api/apply": service.apply}
            if self.path not in actions:
                self.respond(404, {"error": "not found"})
                return
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 1024*1024:
                    raise ValueError("请求体大小无效")
                def reject_constant(value):
                    raise ValueError(f"不允许 {value}")
                payload = json.loads(self.rfile.read(size), parse_constant=reject_constant)
                if not isinstance(payload, dict):
                    raise ValueError("请求必须是 JSON 对象")
                self.respond(200, actions[self.path](payload))
            except (ValueError, KeyError, TypeError, OverflowError) as exc:
                self.respond(400, {"error": str(exc)})
            except (RuntimeError, OSError, TimeoutError) as exc:
                self.respond(409, {"error": str(exc)})

        def log_message(self, *_):
            pass

    return ThreadingHTTPServer((host, port), Handler)


def serve(host="127.0.0.1", port=8766, results=None, allowed_origins=(), open_browser=False):
    service = Workbench(Path(results) if results else Path.cwd()/"results/workbench")
    server = create_server(service, host, port, allowed_origins)
    print(f"PID 工作台：http://{host}:{server.server_port}（启动不会运行设备）", flush=True)
    if open_browser:
        import webbrowser
        webbrowser.open(f"http://{host}:{server.server_port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        service.close()
        server.server_close()
