from __future__ import annotations

from dataclasses import dataclass, asdict
import math
import numpy as np

from .device import Trace


@dataclass(frozen=True)
class Objective:
    tracking: float = 0.55
    overshoot: float = 0.20
    steady: float = 0.15
    effort: float = 0.05
    saturation: float = 0.05
    max_overshoot: float = 0.5
    max_steady_error: float = 0.1
    max_saturation: float = 0.8
    settling_band: float = 0.02
    settling_dwell: float = 0.3
    initial_tolerance: float = 0.2

    def __post_init__(self):
        if any(not math.isfinite(v) or v < 0 for v in asdict(self).values()):
            raise ValueError("评分设置必须为有限非负数")
        if self.tracking+self.overshoot+self.steady+self.effort+self.saturation <= 0:
            raise ValueError("至少一个性能权重大于零")
        if self.settling_band <= 0 or self.settling_dwell <= 0 or self.max_saturation > 1:
            raise ValueError("调节时间或饱和约束无效")


def arrays(trace: Trace) -> dict[str, np.ndarray]:
    if len(trace.samples) < 20:
        raise ValueError("有效采样点不足 20")
    a = {key: np.asarray([s[key] for s in trace.samples], dtype=float) for key in trace.samples[0]}
    if not all(np.all(np.isfinite(v)) for v in a.values()):
        raise ValueError("采样包含非有限值")
    if np.any(np.diff(a["index"]) != trace.plan.decimation) or np.any(np.diff(a["time"]) <= 0):
        raise ValueError("采样丢失、重复或时间不递增")
    expected = trace.plan.dt*trace.plan.decimation
    if np.max(np.abs(np.diff(a["time"])-expected)) > expected*0.02:
        raise ValueError("采样间隔与试验计划不符")
    if a["time"][0] > expected or trace.plan.duration-a["time"][-1] > 2.1*expected:
        raise ValueError("试验记录不完整")
    return a


def score(trace: Trace, objective: Objective) -> dict:
    p = trace.plan
    if p.open_loop or p.wave:
        raise ValueError("辨识实验不能使用阶跃性能评分")
    a = arrays(trace)
    amplitude = abs(p.amplitude)
    if amplitude < 1e-8:
        raise ValueError("评分阶跃幅值不能为零")
    pre = (a["time"] < p.settle) & (a["time"] >= max(0, p.settle-0.2))
    if pre.sum() < 3:
        raise ValueError("需保留足够的阶跃前基准采样")
    initial_error = abs(float(np.mean(a["measurement"][pre]))-p.reference)/amplitude
    initial_span = float(np.ptp(a["measurement"][pre]))/amplitude
    if initial_error > objective.initial_tolerance or initial_span > objective.initial_tolerance:
        raise ValueError("试验初始状态未稳定到基准，需延长稳定时间或恢复对象")
    active = a["time"] >= p.settle
    if active.sum() < 20:
        raise ValueError("阶跃后有效采样不足 20 点")
    t = a["time"][active]-p.settle
    y = a["measurement"][active]
    e = a["reference"][active]-y
    u = a["actuator"][active]
    horizon = p.duration-p.settle
    integral = np.trapezoid if hasattr(np, "trapezoid") else np.trapz
    itae = float(integral(t*np.abs(e), t)/(amplitude*horizon*horizon))
    iae = float(integral(np.abs(e), t)/(amplitude*horizon))
    overshoot = max(0.0, float(np.max((y-(p.reference+p.amplitude))*np.sign(p.amplitude)))/amplitude)
    tail = t >= 0.8*horizon
    if tail.sum() < 3:
        raise ValueError("稳态窗口有效采样不足 3 点")
    steady = float(np.mean(np.abs(e[tail]))/amplitude)
    noise = float(np.std(y[tail])/amplitude)
    effort = float(np.sum(np.abs(np.diff(u)))/(p.output_max-p.output_min))
    saturated = (a["flags"][active].astype(int)&1) != 0
    sat = float(np.mean(saturated))
    bad = np.flatnonzero(np.abs(e) > objective.settling_band*amplitude)
    start = int(bad[-1]+1) if len(bad) else 0
    settling = None
    if start < len(t) and t[-1]-t[start] >= objective.settling_dwell:
        settling = float(t[start])
    weights = np.array([objective.tracking, objective.overshoot, objective.steady,
                        objective.effort, objective.saturation])
    cost = float(weights @ np.array([itae, overshoot, steady, effort, sat])/weights.sum())
    constraint = max(overshoot-objective.max_overshoot, steady-objective.max_steady_error,
                     sat-objective.max_saturation)
    if not all(math.isfinite(v) for v in (cost, constraint, itae, iae, steady, noise, effort)):
        raise ValueError("评分产生非有限指标")
    return dict(cost=cost, score=100/(1+cost), constraint=constraint, feasible=constraint<=0,
                itae=itae, iae=iae, overshoot=overshoot, steady_error=steady, output_variation=effort,
                saturation=sat, settling_time=settling, steady_noise=noise, samples=len(trace.samples),
                initial_error=initial_error, initial_span=initial_span)


def frequency_response(trace: Trace, frequency: float) -> dict:
    """Joint input/output estimate referenced to known excitation, at one frequency.

    Least squares removes offset/trend and estimates coherent sine components.
    Rejects saturation and insufficient cycles; split-window agreement is a quality
    diagnostic, not a calibrated statistical confidence interval.
    """
    a = arrays(trace)
    if not trace.plan.wave or frequency <= 0:
        raise ValueError("频响估计需要正弦激励")
    start = trace.plan.settle + 2/frequency
    if trace.plan.duration-start < 3/frequency:
        raise ValueError("至少需要过渡后 3 个完整周期，建议 8 个周期")
    keep = a["time"] >= start
    if np.any(a["flags"][keep].astype(int)&1):
        raise ValueError("辨识发生饱和，应降低测试幅度")

    def estimate(mask):
        t = a["time"][mask]
        # Include all injected frequencies to avoid cross-frequency leakage.
        columns = [np.ones(len(t)), t-t.mean()]
        for f in trace.plan.frequencies:
            columns.extend((np.sin(2*np.pi*f*t), np.cos(2*np.pi*f*t)))
        design = np.column_stack(columns)
        values = np.column_stack((a["input"][mask], a["measurement"][mask], a["perturbation"][mask]))
        beta, _, rank, _ = np.linalg.lstsq(design, values, rcond=None)
        if rank != design.shape[1]:
            raise ValueError("激励回归矩阵秩不足")
        j = list(trace.plan.frequencies).index(frequency)
        phasors = beta[2+2*j] + 1j*beta[3+2*j]
        if abs(phasors[0]) < 1e-8 or abs(phasors[2]) < 1e-8:
            raise ValueError("该频率实际输入或激励过小")
        # G=(Y/D)/(U/D). D is exogenous; averaging/replication is still required
        # in strongly noisy closed loops (this is not a noise-free exact model).
        g = (phasors[1]/phasors[2])/(phasors[0]/phasors[2])
        residual = values[:, 1]-design@beta[:, 1]
        snr = abs(phasors[1])/max(float(np.std(residual)), 1e-12)
        return g, snr

    response, snr = estimate(keep)
    mid = (start+trace.plan.duration)/2
    left, _ = estimate(keep & (a["time"] < mid))
    right, _ = estimate(keep & (a["time"] >= mid))
    mismatch = abs(left-right)/max(abs(response), 1e-12)
    if snr < 3 or mismatch > 0.3:
        raise ValueError("频响信噪比或重复窗口一致性不足，请延长试验或调整幅度")
    return dict(frequency=frequency, real=float(response.real), imag=float(response.imag),
                magnitude=float(abs(response)), phase_deg=float(np.angle(response, deg=True)),
                snr=float(snr), window_mismatch=float(mismatch), trial=trace.trial,
                operating_point=float(np.mean(a["measurement"][keep])), simulated=trace.simulated)
