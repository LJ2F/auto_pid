from __future__ import annotations

from collections import deque
import ctypes as ct
from dataclasses import dataclass
import math
import os
from pathlib import Path
import secrets
import socket
import struct
import threading
import time

from .protocol import COMMAND, HEADER, Parser, REASONS, SAMPLE, SAMPLE_NAMES, encode
from .schema import Configuration, Plan, finite


class DeviceError(RuntimeError):
    pass


class TrialAborted(DeviceError):
    def __init__(self, reason: str, samples: list[dict]):
        super().__init__(reason)
        self.reason, self.samples = reason, samples


class TCPLink:
    simulated = False

    def __init__(self, host: str, port: int):
        self.socket = socket.create_connection((host, port), timeout=2.0)
        self.socket.settimeout(0.1)
        self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def write(self, data: bytes):
        self.socket.sendall(data)

    def read(self) -> bytes:
        try:
            data = self.socket.recv(4096)
        except socket.timeout:
            return b""
        if not data:
            raise ConnectionError("TCP 连接已关闭")
        return data

    def close(self):
        self.socket.close()


class SerialLink:
    simulated = False

    def __init__(self, port: str, baudrate: int):
        import serial
        self.baudrate = baudrate
        self.serial = serial.Serial(port=port, baudrate=baudrate, timeout=0.05, write_timeout=0.2)

    def write(self, data: bytes):
        if self.serial.write(data) != len(data):
            raise ConnectionError("串口短写")

    def read(self) -> bytes:
        return self.serial.read(min(4096, max(1, self.serial.in_waiting)))

    def close(self):
        self.serial.close()


def native_library():
    path = Path(__file__).with_name("native") / ("pt_sdk.dll" if os.name == "nt" else "libpt_sdk.so")
    if not path.is_file():
        raise DeviceError("缺少同内核仿真库，请先运行 python tools/build_workbench.py")
    lib = ct.CDLL(str(path))
    send_type = ct.CFUNCTYPE(None, ct.c_void_p, ct.POINTER(ct.c_uint8), ct.c_size_t)
    exit_type = ct.CFUNCTYPE(None, ct.c_void_p, ct.c_uint32)
    lib.pt_runtime_size.restype = ct.c_size_t
    lib.pt_runtime_init.argtypes = [ct.c_void_p, ct.c_uint32, send_type, exit_type, ct.c_void_p]
    lib.pt_runtime_receive.argtypes = [ct.c_void_p, ct.c_void_p, ct.c_size_t, ct.c_uint32]
    lib.pt_runtime_tick.argtypes = [ct.c_void_p, ct.c_float, ct.c_float, ct.c_uint32]
    lib.pt_runtime_tick.restype = ct.c_float
    lib.pt_runtime_state.argtypes = [ct.c_void_p]
    lib.pt_runtime_state.restype = ct.c_uint32
    lib.pt_runtime_allow_open_loop.argtypes = [ct.c_void_p, ct.c_int]
    lib.pt_runtime_set_hardware.argtypes = [ct.c_void_p, ct.c_float, ct.c_float, ct.c_float]
    return lib, send_type, exit_type


class NativeLink:
    """Real SDK protocol + controller + state machine, coupled to a simulated plant.

    Time advances on reads, never by wall-clock polling. Explicitly labelled simulation.
    The plant is FOPDT velocity plus integrated position, with deterministic noise.
    """
    simulated = True

    def __init__(self, gain=1.0, tau=0.6, delay=0.03, noise=0.0, dt=0.005, seed=1):
        import numpy as np
        values = [finite(v) for v in (gain, tau, delay, noise, dt)]
        if tau <= 0 or delay < 0 or noise < 0 or not 0.0001 <= dt <= 0.1 or delay > 5:
            raise ValueError("仿真对象参数无效")
        self.lib, send_type, exit_type = native_library()
        self.storage = ct.create_string_buffer(self.lib.pt_runtime_size())
        self.queue: deque[bytes] = deque()
        self.on_send = send_type(lambda _, data, size: self.queue.append(ct.string_at(data, size)))
        self.on_exit = exit_type(self._exit)
        self.lib.pt_runtime_init(self.storage, secrets.randbits(32), self.on_send, self.on_exit, None)
        self.lib.pt_runtime_allow_open_loop(self.storage, 1)
        self.lib.pt_runtime_set_hardware(self.storage, dt, -100.0, 100.0)
        self.gain, self.tau, self.delay, self.noise, self.dt = values
        self.rng = np.random.default_rng(seed)
        self.velocity = self.position = self.output = self.time = 0.0
        self.delay_queue = deque([0.0] * max(1, round(delay/dt)))
        self.closed = False

    def _exit(self, _, reason):
        self.output = 0.0  # simulation-specific exit action, not a universal hardware action

    def write(self, data: bytes):
        if self.closed:
            raise ConnectionError("模拟设备已关闭")
        self.lib.pt_runtime_receive(self.storage, data, len(data), int(self.time*1000) & 0xFFFFFFFF)

    def advance(self, ticks: int = 4):
        for _ in range(ticks):
            measured = self.velocity + (self.rng.normal(0, self.noise) if self.noise else 0.0)
            out = self.lib.pt_runtime_tick(self.storage, measured, self.position, int(self.time*1000) & 0xFFFFFFFF)
            self.output = out if self.lib.pt_runtime_state(self.storage) == 1 else 0.0
            self.delay_queue.append(self.output)
            u = self.delay_queue.popleft()
            self.velocity += (self.gain*u-self.velocity)*(1-math.exp(-self.dt/self.tau))
            self.position += self.velocity*self.dt
            self.time += self.dt

    def read(self) -> bytes:
        if self.queue:
            # Advance after forming the response; results are read on the next request.
            result = self.queue.popleft()
            self.advance()
            return result
        self.advance()
        return b""

    def prepare(self):
        if self.lib.pt_runtime_state(self.storage) == 1:
            raise DeviceError("运行中不能重置仿真对象")
        self.velocity = self.position = self.output = 0.0
        self.delay_queue = deque([0.0]*max(1, round(self.delay/self.dt)))

    def close(self):
        self.closed = True


@dataclass
class Trace:
    samples: list[dict]
    configuration: Configuration
    plan: Plan
    trial: int
    revision: int
    simulated: bool


class Device:
    def __init__(self, link, command=COMMAND):
        self.link, self.command = link, command
        self.session = secrets.randbelow(0xFFFFFFFE)+1
        self.request = self.revision = self.trial = self.boot_id = 0
        self.parser = Parser()
        self.lock = threading.RLock()
        self.configuration = Configuration()
        self.info = {}
        self.status(hello=True)

    def exchange(self, op: int, data=b"", revision=None, trial=None) -> tuple[tuple, bytes]:
        with self.lock:
            self.request += 1
            if self.request >= 0xFFFFFFFF:
                raise DeviceError("请求计数已耗尽，请重新连接")
            packet = encode(HEADER.pack(1, op, 0, self.session, self.request,
                                       self.trial if trial is None else trial,
                                       self.revision if revision is None else revision, self.boot_id)+data, self.command)
            # Identical request bytes on retry; firmware caches the reply, including drained READ data.
            for _ in range(2):
                self.link.write(packet)
                deadline = time.monotonic()+0.35
                while time.monotonic() < deadline:
                    for frame in self.parser.feed(self.link.read()):
                        if frame.command != self.command or len(frame.body) < HEADER.size:
                            continue
                        h = HEADER.unpack_from(frame.body)
                        if h[0] != 1 or h[1] != op|0x80 or (op != 1 and h[3] != self.session) or h[4] != self.request:
                            continue
                        if h[2]:
                            reason = REASONS[h[2]] if h[2] < len(REASONS) else str(h[2])
                            raise DeviceError(f"设备拒绝请求：{reason}")
                        if op == 1:
                            self.session = h[3]
                            self.boot_id = h[7]
                        elif h[7] != self.boot_id:
                            raise DeviceError("设备已重启，请重新连接")
                        self.revision = h[6]
                        return h, frame.body[HEADER.size:]
            raise TimeoutError("设备 ACK 超时；试验由端侧心跳超时终止")

    def status(self, hello=False) -> dict:
        _, data = self.exchange(1 if hello else 2)
        if len(data) != 204:
            raise DeviceError("设备状态长度不匹配")
        boot, state, reason, pending, ticks, opened, dt, lo, hi = struct.unpack(">6I3f", data[:36])
        self.configuration = Configuration.unpack(data[36:])
        self.info = dict(boot_id=boot, state=state, reason=REASONS[reason], pending=pending, ticks=ticks,
                         open_loop_available=bool(opened), dt=dt, output_min=lo, output_max=hi,
                         simulated=self.link.simulated, configuration=self.configuration.to_dict())
        return self.info

    def configure(self, config: Configuration) -> Configuration:
        self.exchange(3, config.pack(), revision=self.revision+1)
        self.status()
        if self.configuration != config:
            raise DeviceError("设备参数回读与提交参数不一致")
        return self.configuration

    def stop(self):
        self.exchange(6)

    def run(self, config: Configuration, plan: Plan, cancel: threading.Event,
            on_samples=lambda rows: None, open_loop_confirmed=False) -> Trace:
        if cancel.is_set():
            raise TrialAborted("cancelled", [])
        if plan.open_loop and not open_loop_confirmed:
            raise ValueError("本次操作尚未显式授权开环辨识")
        if plan.active_loop >= len(config.loops):
            raise ValueError("选择的控制环不存在")
        if plan.open_loop and plan.active_loop:
            outer = config.loops[1]
            if plan.bias-abs(plan.amplitude) < outer.output_min or plan.bias+abs(plan.amplitude) > outer.output_max:
                raise ValueError("外环开环激励必须处于内环目标允许范围内")
        if abs(plan.dt-self.info["dt"]) > self.info["dt"]*0.001:
            raise ValueError("试验控制周期必须与设备实际周期一致")
        if isinstance(self.link, SerialLink) and 75/(plan.dt*plan.decimation) > self.link.baudrate/10*0.75:
            raise ValueError("采样带宽超过串口保守预算，请增大记录采样倍率或提高波特率")
        self.exchange(7)
        if hasattr(self.link, "prepare"):
            self.link.prepare()
        actual = self.configure(config)
        if cancel.is_set():
            raise TrialAborted("cancelled", [])
        self.trial += 1
        rows: list[dict] = []
        deadline = time.monotonic()+plan.duration+max(10.0, plan.duration*0.5)
        try:
            if cancel.is_set():
                raise TrialAborted("cancelled", [])
            self.exchange(4, plan.pack())
            while True:
                if cancel.is_set():
                    self.stop()
                    raise TrialAborted("cancelled", rows)
                h, data = self.exchange(5)
                if h[5] != self.trial or h[6] != self.revision or len(data) < 12:
                    raise DeviceError("试验或配置版本不匹配")
                state, reason, count = struct.unpack(">III", data[:12])
                if len(data) != 12+count*SAMPLE.size:
                    raise DeviceError("采样报文长度不匹配")
                chunk = [dict(zip(SAMPLE_NAMES, SAMPLE.unpack_from(data, 12+i*SAMPLE.size))) for i in range(count)]
                if chunk:
                    if rows and chunk[0]["index"] <= rows[-1]["index"]:
                        raise DeviceError("采样重复或乱序")
                    rows.extend(chunk)
                    on_samples(chunk)
                if state == 3:
                    raise TrialAborted(REASONS[reason], rows)
                if state == 2 and count == 0:
                    break
                if state not in (1, 2):
                    raise DeviceError("设备意外离开试验状态")
                if time.monotonic() > deadline:
                    raise TimeoutError("采集未在截止时间内完成")
                if not self.link.simulated:
                    cancel.wait(min(0.01, plan.timeout_ms/10000))
            return Trace(rows, actual, plan, self.trial, self.revision, self.link.simulated)
        except BaseException:
            try:
                self.stop()
            except (OSError, RuntimeError, TimeoutError):
                pass  # device-side timeout remains active
            raise

    def close(self):
        try:
            self.stop()
        finally:
            self.link.close()
