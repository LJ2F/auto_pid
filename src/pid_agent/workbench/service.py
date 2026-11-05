from __future__ import annotations

from collections import deque
import copy
import csv
import gzip
import io
import json
from pathlib import Path
import threading
import uuid

from .device import Device, NativeLink, SerialLink, TCPLink
from .runner import Runner
from .schema import Configuration, NUMERIC, KINDS, FORMS, ANTI_WINDUP, schema


class Workbench:
    def __init__(self, results: Path):
        self.results = results
        self.device = None
        self.lock = threading.RLock()
        self.cancel = threading.Event()
        self.worker = None
        self.state = {"status": "idle", "connected": False, "records": [], "frequency_response": [], "report": None}
        self.samples = deque(maxlen=12000)
        self.sequence = 0
        self.job_id = None

    def idle(self):
        if self.worker and self.worker.is_alive():
            raise RuntimeError("任务运行中，请先停止并等待退出")

    def connect(self, payload: dict):
        with self.lock:
            self.idle()
            transport = payload.get("transport")
            if transport == "simulation":
                link = NativeLink(**payload.get("plant", {}))
            elif transport == "serial":
                port, baud = payload["port"], payload.get("baudrate", 115200)
                if not isinstance(port, str) or not port or type(baud) is not int or not 1200 <= baud <= 3000000:
                    raise ValueError("串口或波特率无效")
                link = SerialLink(port, baud)
            elif transport == "tcp":
                host, port = payload["host"], payload["port"]
                if not isinstance(host, str) or not host or type(port) is not int or not 1 <= port <= 65535:
                    raise ValueError("TCP 地址或端口无效")
                link = TCPLink(host, port)
            else:
                raise ValueError("请选择 simulation、serial 或 tcp")
            try:
                device = Device(link)
            except Exception:
                link.close()
                raise
            if self.device:
                try:
                    self.device.close()
                except (OSError, RuntimeError, TimeoutError):
                    pass
            self.device = device
            self.state.update(connected=True, device=device.info, transport=transport)
            return device.info

    def disconnect(self, _=None):
        with self.lock:
            self.idle()
            if self.device:
                try:
                    self.device.close()
                finally:
                    self.device = None
                    self.state.update(connected=False, device=None)
            return {"disconnected": True}

    def start(self, payload: dict):
        with self.lock:
            self.idle()
            if not self.device:
                raise RuntimeError("请先连接设备")
            if not self.device.link.simulated and payload.get("limits_confirmed") is not True:
                raise ValueError("请确认实际设备的行程、输出边界、退出动作和试验初始状态")
            cancel = threading.Event()
            job_id = uuid.uuid4().hex
            runner = Runner(self.device, copy.deepcopy(payload), cancel, self.emit, self.results/job_id)
            self.cancel, self.job_id = cancel, job_id
            self.state.update(status="running", job_id=job_id, records=[], frequency_response=[], report=None,
                              current_trial=None, candidate=None, error=None, designs=[])
            self.samples.clear()

            def work():
                try:
                    report = runner.run()
                    with self.lock:
                        self.state.update(status=report["status"], report=report)
                except Exception as exc:
                    with self.lock:
                        self.state.update(status="failed", error=str(exc))
                finally:
                    with self.lock:
                        self.state["active"] = False

            self.worker = threading.Thread(target=work, name="pid-workbench-trial", daemon=True)
            self.state["active"] = True
            self.worker.start()
            return {"job_id": job_id, "status": "running"}

    def emit(self, kind, data):
        with self.lock:
            if kind == "samples":
                for row in data:
                    self.sequence += 1
                    self.samples.append({"cursor": self.sequence, "trial": self.state["current_trial"]["number"], **row})
            elif kind == "trial_start":
                self.state["current_trial"] = copy.deepcopy(data)
            elif kind == "trial_end":
                self.state["records"].append(copy.deepcopy(data))
            elif kind == "frequency":
                self.state["frequency_response"].append(copy.deepcopy(data))
            elif kind == "candidate":
                self.state["candidate"] = copy.deepcopy(data)
            elif kind == "designs":
                self.state["designs"] = copy.deepcopy(data)

    def stop(self, _=None):
        # Do not hold service lock across a transport request; worker checks cancel
        # between bounded transactions and firmware has its own watchdog.
        with self.lock:
            self.cancel.set()
            self.state["status"] = "stopping" if self.worker and self.worker.is_alive() else self.state["status"]
        return {"stop_requested": True}

    def snapshot(self, after=0):
        with self.lock:
            result = copy.deepcopy(self.state)
            # Full raw data are on disk; polling only transports bounded incremental samples.
            result["samples"] = [dict(row) for row in self.samples if row["cursor"] > after][:2000]
            result["cursor"] = result["samples"][-1]["cursor"] if result["samples"] else after
            result["samples_dropped_from_view"] = bool(self.samples and after and after < self.samples[0]["cursor"]-1)
            return result

    def apply(self, payload: dict):
        with self.lock:
            self.idle()
            if not self.device:
                raise RuntimeError("设备未连接")
            configuration = Configuration.from_dict(payload["configuration"])
            self.device.configure(configuration)
            self.state["device"] = self.device.info
            return {"configuration": configuration.to_dict(), "saved_to_flash": False}

    def export(self, kind: str):
        with self.lock:
            self.idle()
            report = self.state.get("report")
            job_id = self.job_id
        if not report:
            raise ValueError("尚无可导出报告")
        if kind == "json":
            return "application/json", f"pid-{job_id}.json", json.dumps(report, ensure_ascii=False, indent=2).encode()
        if kind == "csv":
            output = io.StringIO(newline="")
            names = ["trial", "purpose", "time", "reference", "measurement", "input", "actuator", "perturbation", "p", "i", "d", "flags"]
            writer = csv.DictWriter(output, fieldnames=names, extrasaction="ignore")
            writer.writeheader()
            for path in sorted((self.results/job_id).glob("trial-*.json.gz")):
                with gzip.open(path, "rt", encoding="utf-8") as f:
                    data = json.load(f)
                for row in data["samples"]:
                    writer.writerow({**row, "trial": data["number"], "purpose": data["purpose"]})
            return "text/csv; charset=utf-8", f"pid-{job_id}.csv", output.getvalue().encode("utf-8-sig")
        if kind == "c":
            if not report.get("validated") or not report.get("best"):
                raise ValueError("只有独立复验通过的参数可导出为推荐 C 配置")
            cfg = Configuration.from_dict(report["best"])
            lines = ['/* Generated configuration. Verify units, sample period and actuator hooks on target. */',
                     '#include "pt_sdk.h"', 'static const pt_config tuned_configuration = {',
                     f'    .loop_count = {len(cfg.loops)}, .outer_divider = {cfg.outer_divider},', '    .loop = {']
            for pid in cfg.loops:
                lines += ['        {', f'            .kind = {KINDS.index(pid.kind)}, .form = {FORMS.index(pid.form)},',
                          f'            .flags = {pid.flags}, .antiwindup = {ANTI_WINDUP.index(pid.antiwindup)},']
                for name in NUMERIC:
                    value = format(getattr(pid, name), ".9g")
                    if "." not in value and "e" not in value:
                        value += ".0"
                    lines.append(f"            .{name} = {value}f,")
                lines += ['        },']
            lines += ['    }', '};', '']
            return "text/plain; charset=utf-8", f"pid-{job_id}.h", "\n".join(lines).encode()
        raise ValueError("支持 json、csv、c 导出")

    @staticmethod
    def ports():
        from serial.tools.list_ports import comports
        return [{"port": p.device, "description": p.description} for p in comports()]

    def close(self):
        self.cancel.set()
        if self.worker:
            self.worker.join(timeout=4)
        if self.device:
            try:
                self.device.close()
            except (OSError, RuntimeError, TimeoutError):
                pass
