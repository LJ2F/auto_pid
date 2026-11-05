from __future__ import annotations

from dataclasses import asdict, dataclass, fields, replace
import math
import struct

KINDS = ("P", "PI", "PD", "PID")
FORMS = ("positional", "incremental")
ANTI_WINDUP = ("none", "clamp", "conditional", "backcalc")
FEATURES = {
    "integral_limit": 1, "derivative_on_measurement": 2, "trapezoidal_integral": 4,
    "proportional_on_measurement": 8, "output_filter": 16, "variable_integral": 32,
    "derivative_filter": 64, "blocked_detection": 128, "measurement_feedforward": 512,
}
NUMERIC = (
    "kp", "ki", "kd", "ffj", "ffb", "integral_limit", "deadband", "integral_a",
    "integral_b", "output_tau", "derivative_tau", "output_min", "output_max",
    "aw_gain", "blocked_error", "blocked_time",
)
PARAMETERS = {
    "kp": ("比例增益", "output / error"), "ki": ("积分增益", "output / (error·s)"),
    "kd": ("微分增益", "output·s / error"), "ffj": ("测量变化率补偿 FFJ", "output·s / measurement"),
    "ffb": ("测量值补偿 FFB", "output / measurement"), "integral_limit": ("积分限幅", "output"),
    "deadband": ("控制死区", "error"), "integral_a": ("变速积分过渡宽度", "error"),
    "integral_b": ("全速积分阈值", "error"), "output_tau": ("输出滤波时间常数", "s"),
    "derivative_tau": ("微分滤波时间常数", "s"), "aw_gain": ("反算抗饱和增益", "1/s"),
}


def finite(value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("数值必须是有限数字")
    return float(value)


def f32(value: float) -> float:
    try:
        return struct.unpack(">f", struct.pack(">f", finite(value)))[0]
    except (OverflowError, struct.error) as exc:
        raise ValueError("数值超出 float32 范围") from exc


@dataclass(frozen=True)
class PID:
    kind: str = "PI"
    form: str = "positional"
    flags: int = 1
    antiwindup: str = "conditional"
    kp: float = 0.8
    ki: float = 0.4
    kd: float = 0.0
    ffj: float = 0.0
    ffb: float = 0.0
    integral_limit: float = 5.0
    deadband: float = 0.0
    integral_a: float = 1.0
    integral_b: float = 0.0
    output_tau: float = 0.02
    derivative_tau: float = 0.02
    output_min: float = -5.0
    output_max: float = 5.0
    aw_gain: float = 1.0
    blocked_error: float = 10.0
    blocked_time: float = 1.0

    def __post_init__(self):
        if self.kind not in KINDS or self.form not in FORMS or self.antiwindup not in ANTI_WINDUP:
            raise ValueError("未知 PID 类型、形式或抗饱和方式")
        if type(self.flags) is not int or self.flags < 0 or self.flags & ~767:
            raise ValueError("未知功能开关")
        for name in NUMERIC:
            object.__setattr__(self, name, f32(getattr(self, name)))
        if self.output_min >= self.output_max:
            raise ValueError("输出下界必须小于上界")
        for name in ("integral_limit", "deadband", "integral_b", "output_tau", "derivative_tau", "aw_gain"):
            if getattr(self, name) < 0:
                raise ValueError(f"{name} 不能为负")
        if self.integral_a <= 0 or self.blocked_error <= 0 or self.blocked_time <= 0:
            raise ValueError("积分过渡宽度、堵转误差和时间必须为正")

    def available(self) -> set[str]:
        result = {"kp", "deadband"}
        if "I" in self.kind:
            result.add("ki")
            if self.flags & 1 or self.antiwindup == "clamp":
                result.add("integral_limit")
            if self.flags & 32:
                result.update(("integral_a", "integral_b"))
            if self.antiwindup == "backcalc":
                result.add("aw_gain")
        if "D" in self.kind:
            result.add("kd")
            if self.flags & 64:
                result.add("derivative_tau")
        if self.flags & 16:
            result.add("output_tau")
        if self.flags & 512:
            result.update(("ffj", "ffb"))
        return result

    def pack(self) -> bytes:
        return struct.pack(">4I16f", KINDS.index(self.kind), FORMS.index(self.form), self.flags,
                           ANTI_WINDUP.index(self.antiwindup), *(getattr(self, n) for n in NUMERIC))

    @classmethod
    def unpack(cls, data: bytes) -> PID:
        v = struct.unpack(">4I16f", data)
        return cls(kind=KINDS[v[0]], form=FORMS[v[1]], flags=v[2], antiwindup=ANTI_WINDUP[v[3]],
                   **dict(zip(NUMERIC, v[4:])))


@dataclass(frozen=True)
class Configuration:
    loops: tuple[PID, ...] = (PID(),)
    outer_divider: int = 5

    def __post_init__(self):
        if not 1 <= len(self.loops) <= 2 or not all(isinstance(p, PID) for p in self.loops):
            raise ValueError("支持单环或双环串级")
        if type(self.outer_divider) is not int or not 1 <= self.outer_divider <= 1000:
            raise ValueError("外环周期倍率必须为 1–1000 的整数")

    def pack(self) -> bytes:
        return struct.pack(">II", len(self.loops), self.outer_divider) + self.loops[0].pack() + (
            self.loops[1] if len(self.loops) == 2 else PID()).pack()

    @classmethod
    def unpack(cls, data: bytes) -> Configuration:
        count, divider = struct.unpack(">II", data[:8])
        if count not in (1, 2):
            raise ValueError("设备返回无效环数量")
        return cls(tuple(PID.unpack(data[8 + 80*i:88 + 80*i]) for i in range(count)), divider)

    @classmethod
    def from_dict(cls, data: dict) -> Configuration:
        if set(data) - {"loops", "outer_divider"}:
            raise ValueError("未知配置字段")
        return cls(tuple(PID(**p) for p in data["loops"]), data.get("outer_divider", 5))

    def to_dict(self) -> dict:
        return asdict(self)

    def value(self, key: str) -> float:
        side, name = key.split(".")
        if side not in ("inner", "outer") or name not in NUMERIC:
            raise ValueError(f"未知参数 {key}")
        return getattr(self.loops[("inner", "outer").index(side)], name)

    def updated(self, values: dict[str, float]) -> Configuration:
        loops = list(self.loops)
        for key, value in values.items():
            side, name = key.split(".")
            if side not in ("inner", "outer") or name not in PARAMETERS:
                raise ValueError(f"禁止搜索 {key}")
            idx = ("inner", "outer").index(side)
            loops[idx] = replace(loops[idx], **{name: value})
        return replace(self, loops=tuple(loops))


@dataclass(frozen=True)
class Plan:
    wave: int = 0
    open_loop: int = 0
    active_loop: int = 0
    decimation: int = 2
    frequency_count: int = 1
    timeout_ms: int = 1000
    dt: float = 0.005
    duration: float = 5.0
    settle: float = 0.5
    reference: float = 0.0
    amplitude: float = 1.0
    bias: float = 0.0
    y_min: tuple[float, float] = (-10.0, -10.0)
    y_max: tuple[float, float] = (10.0, 10.0)
    output_min: float = -5.0
    output_max: float = 5.0
    max_slew: float = 0.0
    frequencies: tuple[float, ...] = (1.0,)

    def __post_init__(self):
        for name in ("wave", "open_loop", "active_loop", "decimation", "frequency_count", "timeout_ms"):
            if type(getattr(self, name)) is not int:
                raise ValueError(f"{name} 必须为整数")
        if self.wave not in (0, 1, 2) or self.open_loop not in (0, 1) or self.active_loop not in (0, 1):
            raise ValueError("未知试验模式")
        if not 1 <= self.decimation <= 10000 or not 100 <= self.timeout_ms <= 5000:
            raise ValueError("采样倍率或心跳超时无效")
        if len(self.y_min) != 2 or len(self.y_max) != 2 or not 1 <= len(self.frequencies) <= 8:
            raise ValueError("测量范围或频率数组长度无效")
        for f in fields(self):
            v = getattr(self, f.name)
            for number in v if isinstance(v, (tuple, list)) else (v,):
                finite(number)
        if not 1e-6 <= self.dt <= 1 or not 10*self.dt <= self.duration <= 3600:
            raise ValueError("控制周期或试验时长无效")
        if not 0 <= self.settle < self.duration or self.max_slew < 0 or self.output_min >= self.output_max:
            raise ValueError("稳定时间、变化率或输出范围无效")
        if any(lo >= hi for lo, hi in zip(self.y_min, self.y_max)):
            raise ValueError("测量下界必须小于上界")
        if self.open_loop and not self.active_loop and (self.bias-abs(self.amplitude) < self.output_min or
                               self.bias+abs(self.amplitude) > self.output_max):
            raise ValueError("开环偏置与幅值必须处于输出范围内")
        if self.wave and (self.frequency_count != len(self.frequencies) or
                          (self.wave == 1 and self.frequency_count != 1) or
                          any(f <= 0 or f*self.dt*self.decimation > 0.1 for f in self.frequencies)):
            raise ValueError("辨识频率需至少每周期 10 个记录采样点")

    def pack(self) -> bytes:
        return struct.pack(">6I21f", self.wave, self.open_loop, self.active_loop, self.decimation,
                           self.frequency_count, self.timeout_ms, self.dt, self.duration, self.settle,
                           self.reference, self.amplitude, self.bias, *self.y_min, *self.y_max,
                           self.output_min, self.output_max, self.max_slew,
                           *(tuple(self.frequencies)+(0.0,)*(8-len(self.frequencies))))

    @classmethod
    def from_dict(cls, data: dict) -> Plan:
        data = dict(data)
        for name in ("y_min", "y_max", "frequencies"):
            if name in data:
                data[name] = tuple(data[name])
        return cls(**data)


def schema() -> dict:
    return {"version": 1, "kinds": KINDS, "forms": FORMS, "antiwindup": ANTI_WINDUP,
            "features": FEATURES, "parameters": {k: {"label": v[0], "unit": v[1]} for k, v in PARAMETERS.items()},
            "default_configuration": Configuration().to_dict(), "default_plan": asdict(Plan())}
