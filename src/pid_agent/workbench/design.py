from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np
from scipy.optimize import least_squares

from .schema import Configuration, PID, finite
from .search import Space


@dataclass(frozen=True)
class PlantModel:
    """Explicit linear model supplied by the user or fitted to accepted FRF data."""
    gain: float = 1.0
    tau: float = 0.6
    delay: float = 0.03
    family: str = "fopdt"
    damping: float = 0.7

    def __post_init__(self):
        for value in (self.gain, self.tau, self.delay, self.damping):
            finite(value)
        if self.family not in ("fopdt", "second_order", "integrator") or self.tau <= 0 or self.delay < 0 or self.damping <= 0:
            raise ValueError("对象模型参数无效")

    def response(self, frequencies):
        s = 2j*np.pi*np.asarray(frequencies)
        if self.family == "fopdt":
            result = self.gain/(1+self.tau*s)
        elif self.family == "integrator":
            result = self.gain/(s*(1+self.tau*s))
        else:
            result = self.gain/(1+2*self.damping*self.tau*s+(self.tau*s)**2)
        return result*np.exp(-self.delay*s)

    def samples(self, frequencies):
        response = self.response(frequencies)
        return [dict(frequency=float(f), real=float(g.real), imag=float(g.imag), source="model")
                for f, g in zip(frequencies, response)]


def pid_response(pid: PID, frequencies, dt: float):
    """Actual SDK linear branches, outside deadband, windup and saturation.

    Returns reference and feedback paths separately, u=Cr*r-Cy*y.
    Rectangular integral uses the CURRENT error (backward Euler).
    """
    q = np.exp(-2j*np.pi*np.asarray(frequencies)*dt)
    derivative = (1-q)/dt
    integral = dt/(1-q)
    if pid.flags&4:
        integral *= (1+q)/2
    d = pid.kd*derivative if "D" in pid.kind else np.zeros_like(q)
    if pid.flags&64:
        d *= dt/(pid.derivative_tau+dt-pid.derivative_tau*q)
    i = pid.ki*integral if "I" in pid.kind else np.zeros_like(q)
    cy = pid.kp+i+d
    cr = (0 if pid.flags&8 else pid.kp)+i+(0 if pid.flags&2 else d)
    if pid.flags&512:
        cy -= pid.ffb+pid.ffj*derivative
    if pid.flags&16:
        filt = dt/(pid.output_tau+dt-pid.output_tau*q)
        cy *= filt
        cr *= filt
    return cr, cy


def frequency_seeds(space: Space, data: list[dict], dt: float, active_loop=0,
                    phase_margin=60.0, seed=1) -> list[dict]:
    """Independent bounded loop-shaping implementation, not MathWorks pidtune.

    Sparse FRF margins are ESTIMATES only. Candidates require time-domain validation.
    No controller switch or unselected numerical parameter is modified.
    """
    if len(data) < 4 or not 20 <= phase_margin <= 85:
        raise ValueError("频域设计需要至少 4 个有效频点，目标裕度为 20–85 度")
    ordered = sorted(data, key=lambda row: row["frequency"])
    frequencies = np.array([finite(row["frequency"]) for row in ordered])
    g = np.array([complex(finite(row["real"]), finite(row["imag"])) for row in ordered])
    if np.any(frequencies <= 0) or len(np.unique(frequencies)) != len(frequencies) or np.any(np.abs(g)<1e-12):
        raise ValueError("频响数据无效")
    period = dt*(space.base.outer_divider if active_loop else 1)
    if frequencies[-1]*period >= 0.45:
        raise ValueError("频响范围超过被调控制环的有效采样频带")
    rng = np.random.default_rng(seed)
    result = []
    x0 = np.clip(space.encode(space.base), 1e-4, 1-1e-4)
    # Design across the measured band; retain headroom above crossover.
    crossovers = np.geomspace(frequencies[0], max(frequencies[0]*1.05, frequencies[-1]/5), 4)
    for fc in crossovers:
        desired = 1/(1+1j*frequencies/fc)
        gc = np.interp(np.log(fc), np.log(frequencies), g.real)+1j*np.interp(np.log(fc), np.log(frequencies), g.imag)

        def residual(x):
            cfg = space.decode(x)
            cr, cy = pid_response(cfg.loops[active_loop], frequencies, period)
            loop = g*cy
            denominator = 1+loop
            if np.any(np.abs(denominator)<1e-8):
                return np.full(2*len(frequencies)+3, 1e5)
            tracking = g*cr/denominator
            _, cc = pid_response(cfg.loops[active_loop], [fc], period)
            lc = gc*cc[0]
            angle_error = np.angle(lc*np.exp(-1j*(-np.pi+np.deg2rad(phase_margin))))
            error = tracking-desired
            sensitivity = max(0.0, float(np.max(1/np.abs(denominator)))-2.0)
            return np.r_[error.real, error.imag, math.log(max(abs(lc), 1e-12)), angle_error, sensitivity*2]

        starts = [x0, rng.uniform(0.05, 0.95, space.dimension)]
        candidates = [least_squares(residual, start, bounds=(0, 1), diff_step=1e-3,
                                    max_nfev=120) for start in starts]
        optimum = min(candidates, key=lambda row: float(row.fun@row.fun))
        cfg = space.decode(optimum.x)
        _, c = pid_response(cfg.loops[active_loop], [fc], period)
        phase = float(np.angle(gc*c[0], deg=True))
        if phase > 0:
            phase -= 360
        result.append(dict(configuration=cfg, target_crossover_hz=float(fc),
                           phase_at_target_deg=180+phase, model_residual=float(np.linalg.norm(optimum.fun)),
                           source="frequency_design", stability_certified=False))
    unique = {}
    for row in sorted(result, key=lambda item: item["model_residual"]):
        unique.setdefault(row["configuration"].pack(), row)
    return list(unique.values())


def simc_seed(space: Space, model: PlantModel, response_time: float, active_loop=0):
    if model.family != "fopdt" or model.gain == 0 or response_time <= 0:
        return None
    pid = space.base.loops[active_loop]
    prefix = ("inner", "outer")[active_loop]
    names = {b.name for b in space.bounds}
    if pid.kind != "PI" or not {prefix+".kp", prefix+".ki"} <= names:
        return None
    kp = model.tau/(model.gain*(response_time+model.delay))
    ti = min(model.tau, 4*(response_time+model.delay))
    cfg = space.base.updated({prefix+".kp": kp, prefix+".ki": kp/ti})
    if not all(b.lower <= cfg.value(b.name) <= b.upper for b in space.bounds):
        return None
    return cfg
