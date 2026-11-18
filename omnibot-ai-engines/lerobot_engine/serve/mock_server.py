"""Stdlib HTTP mock of OhhO Serve for CI and latency benchmarks."""

from __future__ import annotations

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import parse_qs, urlparse


class MockServeServer:
    """In-process OhhO Serve stub (mock ACT backend, no GPU)."""

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 0,
        *,
        predict_delay_sec: float = 0.0,
        action_vector: list[float] | None = None,
    ) -> None:
        self.host = host
        self.port = port
        self.predict_delay_sec = predict_delay_sec
        self.action_vector = action_vector or [0.0] * 9
        self.model_loaded = False
        self.model_path = ""
        self._predict_count = 0
        self._latency_ms_total = 0.0
        self._httpd: ThreadingHTTPServer | None = None
        self._thread: threading.Thread | None = None

    @property
    def base_url(self) -> str:
        if not self._httpd:
            raise RuntimeError("mock serve not started")
        host, port = self._httpd.server_address[:2]
        return f"http://{host}:{port}"

    def start(self) -> None:
        server = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, format, *args):  # noqa: A003
                return

            def _json(self, code: int, doc: dict[str, Any]) -> None:
                body = json.dumps(doc).encode("utf-8")
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _text(self, code: int, text: str) -> None:
                body = text.encode("utf-8")
                self.send_response(code)
                self.send_header("Content-Type", "text/plain; version=0.0.4")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self) -> None:  # noqa: N802
                path = urlparse(self.path).path
                if path == "/health":
                    self._json(
                        200,
                        {
                            "status": "ok",
                            "model_loaded": server.model_loaded,
                            "device": "mock",
                        },
                    )
                    return
                if path == "/models":
                    from lerobot_engine.act_vr_sim import models_payload

                    checkpoint = server.model_path or models_payload()["models"][0][
                        "checkpoint"
                    ]
                    self._json(200, models_payload(checkpoint))
                    return
                if path == "/metrics":
                    avg = (
                        server._latency_ms_total / server._predict_count
                        if server._predict_count
                        else 0.0
                    )
                    lines = [
                        "# HELP ohho_serve_predict_requests_total Mock predict count",
                        "# TYPE ohho_serve_predict_requests_total counter",
                        f"ohho_serve_predict_requests_total {server._predict_count}",
                        "# HELP ohho_serve_predict_latency_ms_avg Mock average latency",
                        "# TYPE ohho_serve_predict_latency_ms_avg gauge",
                        f"ohho_serve_predict_latency_ms_avg {avg:.4f}",
                    ]
                    self._text(200, "\n".join(lines) + "\n")
                    return
                self._json(404, {"detail": "not found"})

            def do_POST(self) -> None:  # noqa: N802
                path = urlparse(self.path).path
                length = int(self.headers.get("Content-Length", "0"))
                raw = self.rfile.read(length) if length else b""
                if path == "/load_model":
                    qs = parse_qs(urlparse(self.path).query)
                    model_path = (qs.get("model_path") or [""])[0]
                    if not model_path and raw:
                        try:
                            model_path = json.loads(raw.decode("utf-8")).get(
                                "model_path", ""
                            )
                        except json.JSONDecodeError:
                            model_path = ""
                    if not model_path:
                        self._json(400, {"detail": "model_path required"})
                        return
                    server.model_path = model_path
                    server.model_loaded = True
                    self._json(200, {"status": "loaded", "model": "MockACT"})
                    return
                if path == "/predict":
                    if not server.model_loaded:
                        self._json(503, {"detail": "model not loaded"})
                        return
                    if server.predict_delay_sec > 0:
                        time.sleep(server.predict_delay_sec)
                    t0 = time.perf_counter()
                    try:
                        req = json.loads(raw.decode("utf-8")) if raw else {}
                    except json.JSONDecodeError:
                        req = {}
                    instruction = req.get("instruction", "")
                    latency_ms = (time.perf_counter() - t0) * 1000.0
                    server._predict_count += 1
                    server._latency_ms_total += latency_ms
                    vec = list(server.action_vector)
                    self._json(
                        200,
                        {
                            "action": {
                                "vector": vec,
                                "instruction": instruction,
                                "mock": True,
                            },
                            "latency_ms": latency_ms,
                        },
                    )
                    return
                self._json(404, {"detail": "not found"})

        self._httpd = ThreadingHTTPServer((self.host, self.port), Handler)
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if self._httpd:
            self._httpd.shutdown()
            self._httpd.server_close()
            self._httpd = None
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None


def start_mock_serve(**kwargs: Any) -> MockServeServer:
    srv = MockServeServer(**kwargs)
    srv.start()
    return srv
