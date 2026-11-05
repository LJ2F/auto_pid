from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np
from scipy.linalg import cho_factor, cho_solve
from scipy.optimize import minimize
from scipy.special import ndtr

from .schema import Configuration, f32, finite


@dataclass(frozen=True)
class Bound:
    name: str
    lower: float
    upper: float
    scale: str = "linear"

    def __post_init__(self):
        lo, hi = f32(self.lower), f32(self.upper)
        if lo >= hi or self.scale not in ("linear", "log") or (self.scale == "log" and lo <= 0):
            raise ValueError(f"{self.name} 搜索边界或尺度无效")
        object.__setattr__(self, "lower", lo)
        object.__setattr__(self, "upper", hi)


class Space:
    def __init__(self, base: Configuration, bounds: list[Bound]):
        if not 1 <= len(bounds) <= 16 or len({b.name for b in bounds}) != len(bounds):
            raise ValueError("需勾选 1–16 个不重复数值参数")
        self.base, self.bounds = base, bounds
        for b in bounds:
            side, name = b.name.split(".")
            if side not in ("inner", "outer"):
                raise ValueError("未知控制环")
            idx = ("inner", "outer").index(side)
            if idx >= len(base.loops) or name not in base.loops[idx].available():
                raise ValueError(f"{b.name} 对当前结构无效或属于保护参数")
            if not b.lower <= base.value(b.name) <= b.upper:
                raise ValueError(f"{b.name} 当前值不在搜索边界内")
        names = {b.name for b in bounds}
        for i, pid in enumerate(base.loops):
            prefix = ("inner", "outer")[i]
            if pid.flags&2 and pid.flags&512 and not pid.flags&64 and {prefix+".kd", prefix+".ffj"} <= names:
                raise ValueError("当前结构中 Kd 与 FFJ 的线性作用冗余，请只勾选其中一个")

    @property
    def dimension(self):
        return len(self.bounds)

    def encode(self, cfg: Configuration) -> np.ndarray:
        result = []
        for b in self.bounds:
            lo, hi, v = b.lower, b.upper, cfg.value(b.name)
            if b.scale == "log":
                lo, hi, v = math.log(lo), math.log(hi), math.log(v)
            result.append((v-lo)/(hi-lo))
        return np.asarray(result)

    def decode(self, x) -> Configuration:
        values = {}
        for b, v in zip(self.bounds, x, strict=True):
            lo, hi = b.lower, b.upper
            if b.scale == "log":
                value = math.exp(math.log(lo)+float(np.clip(v, 0, 1))*(math.log(hi)-math.log(lo)))
            else:
                value = lo+float(np.clip(v, 0, 1))*(hi-lo)
            values[b.name] = min(hi, max(lo, f32(value)))
        return self.base.updated(values)

    def key(self, cfg: Configuration) -> bytes:
        # Includes fixed structure and every parameter, at actual firmware precision.
        return cfg.pack()


class GaussianProcess:
    """Small-data Matérn 5/2 GP with ARD and learned observation noise."""
    def fit(self, x, y):
        self.x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)
        self.offset = float(np.mean(y))
        self.scale = max(float(np.std(y)), 1e-4)
        self.y = (y-self.offset)/self.scale
        d = self.x.shape[1]

        def likelihood(log_parameters):
            length = np.exp(log_parameters[:d])
            noise = np.exp(log_parameters[-1])
            k = self.kernel(self.x, self.x, length) + np.eye(len(x))*(noise+1e-8)
            try:
                factor = cho_factor(k, lower=True)
                alpha = cho_solve(factor, self.y)
                return 0.5*float(self.y@alpha)+float(np.log(np.diag(factor[0])).sum())
            except np.linalg.LinAlgError:
                return 1e20

        result = minimize(likelihood, np.log([0.25]*d+[0.01]), method="L-BFGS-B",
                          bounds=[(math.log(0.03), math.log(2))]*d+[(math.log(1e-5), math.log(0.5))],
                          options={"maxiter": 35})
        self.length = np.exp(result.x[:d])
        self.noise = float(np.exp(result.x[-1]))
        self.factor = cho_factor(self.kernel(self.x, self.x, self.length)+np.eye(len(x))*(self.noise+1e-8), lower=True)
        self.alpha = cho_solve(self.factor, self.y)
        return self

    @staticmethod
    def kernel(a, b, length):
        radius = np.sqrt(np.sum(((a[:, None, :]-b[None, :, :])/length)**2, axis=-1))
        root = math.sqrt(5)*radius
        return (1+root+5*radius*radius/3)*np.exp(-root)

    def predict(self, points, covariance=False):
        points = np.asarray(points)
        k = self.kernel(self.x, points, self.length)
        mean = self.offset+self.scale*(k.T@self.alpha)
        cov = self.kernel(points, points, self.length)-k.T@cho_solve(self.factor, k)
        cov = (cov+cov.T)*0.5*self.scale**2
        if covariance:
            return mean, cov
        return mean, np.maximum(np.diag(cov), 1e-12)


class Search:
    """ask/tell in actual parameter coordinates; aborted and invalid trials are distinct.

    Bayesian mode uses Monte Carlo noisy EI (joint latent samples including the
    incumbent), multiplied by modeled feasibility. Local bounds are a search policy,
    NOT a safety certificate. Population mode implements bounded CMA-ES updates.
    """
    def __init__(self, space: Space, method="bayesian", seed=1):
        if method not in ("bayesian", "pattern", "cmaes"):
            raise ValueError("未知搜索算法")
        self.space, self.method = space, method
        self.rng = np.random.default_rng(seed)
        self.observations = []
        self.rejected = []
        self.radius = 0.2
        self.pattern_index = 0
        self.mean = space.encode(space.base)
        self.cov = np.eye(space.dimension)
        self.population = []
        n = space.dimension
        self.lam = 4+int(3*math.log(n+1))
        mu = self.lam//2
        self.weights = np.log(mu+0.5)-np.log(np.arange(1, mu+1))
        self.weights /= self.weights.sum()
        self.mueff = 1/float(self.weights@self.weights)
        self.cc = (4+self.mueff/n)/(n+4+2*self.mueff/n)
        self.cs = (self.mueff+2)/(n+self.mueff+5)
        self.c1 = 2/((n+1.3)**2+self.mueff)
        self.cmu = min(1-self.c1, 2*(self.mueff-2+1/self.mueff)/((n+2)**2+self.mueff))
        self.damps = 1+2*max(0, math.sqrt((self.mueff-1)/(n+1))-1)+self.cs
        self.chi = math.sqrt(n)*(1-1/(4*n)+1/(21*n*n))
        self.pc, self.ps = np.zeros(n), np.zeros(n)
        self.sigma, self.generation = 0.2, 0
        self.pending = set()
        self.successes = self.failures = 0

    def _population_update(self):
        n = self.space.dimension
        elite = sorted(self.population, key=lambda row: row[0])[:len(self.weights)]
        points = np.array([row[1] for row in elite])
        steps = (points-self.mean)/self.sigma
        step = self.weights@steps
        values, vectors = np.linalg.eigh(self.cov)
        inverse = (vectors/np.sqrt(np.maximum(values, 1e-12)))@vectors.T
        self.ps = (1-self.cs)*self.ps+math.sqrt(self.cs*(2-self.cs)*self.mueff)*(inverse@step)
        self.generation += 1
        hsig = float(np.linalg.norm(self.ps)/math.sqrt(1-(1-self.cs)**(2*self.generation))/self.chi < 1.4+2/(n+1))
        self.pc = (1-self.cc)*self.pc+hsig*math.sqrt(self.cc*(2-self.cc)*self.mueff)*step
        self.cov = ((1-self.c1-self.cmu+self.c1*(1-hsig)*self.cc*(2-self.cc))*self.cov
                    + self.c1*np.outer(self.pc, self.pc)+self.cmu*(steps.T*self.weights)@steps)
        self.cov = (self.cov+self.cov.T)*0.5+np.eye(n)*1e-12
        self.mean = self.weights@points
        self.sigma = float(np.clip(self.sigma*math.exp(self.cs/self.damps*(np.linalg.norm(self.ps)/self.chi-1)), 0.002, 0.5))
        self.population.clear()

    def tell(self, cfg: Configuration, cost=None, constraint=None, status="complete"):
        x = self.space.encode(cfg)
        key = self.space.key(cfg)
        requested = key in self.pending
        self.pending.discard(key)
        old = self.incumbent()
        old_rows = [r for r in self.observations if self.space.key(r[3]) == self.space.key(old)]
        if status == "aborted":
            self.rejected.append(x)
        elif status == "complete":
            cost, constraint = finite(cost), finite(constraint)
            self.observations.append((x, cost, constraint, cfg))
        if requested and self.method == "cmaes" and status in ("complete", "aborted"):
            rank = (constraint > 0, max(0, constraint), cost) if status == "complete" else (True, math.inf, math.inf)
            self.population.append((rank, x))
            if len(self.population) >= self.lam:
                self._population_update()
        if requested and self.method == "bayesian" and status in ("complete", "aborted"):
            improved = status == "complete" and constraint <= 0 and (not old_rows or
                cost < np.mean([r[1] for r in old_rows])-max(1e-5, 0.01*abs(np.mean([r[1] for r in old_rows]))))
            self.successes = self.successes+1 if improved else 0
            self.failures = 0 if improved else self.failures+1
            if self.successes >= 3:
                self.radius = min(0.5, self.radius*1.5)
                self.successes = 0
            if self.failures >= max(4, self.space.dimension):
                self.radius = max(0.01, self.radius*0.5)
                self.failures = 0

    def incumbent(self) -> Configuration:
        if not self.observations:
            return self.space.base
        groups = {}
        for row in self.observations:
            groups.setdefault(self.space.key(row[3]), []).append(row)
        averaged = [(float(np.mean([r[1] for r in rows])), max(r[2] for r in rows), rows[0][3]) for rows in groups.values()]
        return min(averaged, key=lambda r: (r[1] > 0, max(0, r[1]), r[0]))[2]

    def ask(self) -> Configuration:
        d = self.space.dimension
        center = self.space.encode(self.incumbent())
        if self.method == "cmaes":
            if self.generation == 0 and not self.population:
                self.mean = center.copy()
            raw = self.rng.multivariate_normal(self.mean, self.cov*self.sigma**2, size=2048)
            pool = raw[np.all((raw >= 0)&(raw <= 1), axis=1)]
        else:
            pool = self.rng.uniform(np.maximum(0, center-self.radius), np.minimum(1, center+self.radius), (96, d))
        if self.method == "pattern":
            directions = np.vstack((np.eye(d), -np.eye(d)))
            index = self.pattern_index % (2*d)
            if self.pattern_index and index == 0:
                self.radius = max(0.005, self.radius*0.7)
            self.pattern_index += 1
            pool = np.vstack((np.clip(center+directions[index]*self.radius, 0, 1), pool))
        seen = {self.space.key(row[3]) for row in self.observations}
        candidates = []
        for x in pool:
            cfg = self.space.decode(x)
            actual = self.space.encode(cfg)
            if self.space.key(cfg) in seen or any(np.linalg.norm(actual-r)<0.015 for r in self.rejected):
                continue
            candidates.append((cfg, actual))
        if not candidates:
            raise RuntimeError("当前局部范围内没有新的有效候选")
        if self.method != "bayesian" or len(self.observations) < max(4, d+1):
            self.pending.add(self.space.key(candidates[0][0]))
            return candidates[0][0]
        x = np.array([r[0] for r in self.observations])
        objective = GaussianProcess().fit(x, [r[1] for r in self.observations])
        constraint = GaussianProcess().fit(x, [r[2] for r in self.observations])
        points = np.array([r[1] for r in candidates])
        gmean, gvar = constraint.predict(points)
        probability = ndtr(-gmean/np.sqrt(gvar+constraint.noise*constraint.scale**2))
        feasible = np.array([r[2] <= 0 for r in self.observations])
        if not feasible.any():
            selected = candidates[int(np.argmax(probability))][0]
            self.pending.add(self.space.key(selected))
            return selected
        baseline = x[feasible]
        all_points = np.vstack((baseline, points))
        mean, cov = objective.predict(all_points, covariance=True)
        eigenvalues, eigenvectors = np.linalg.eigh(cov)
        draws = mean + self.rng.standard_normal((128, len(mean))) @ (eigenvectors*np.sqrt(np.maximum(eigenvalues, 1e-12))).T
        best = np.min(draws[:, :len(baseline)], axis=1)
        nei = np.maximum(best[:, None]-draws[:, len(baseline):], 0).mean(axis=0)
        utility = nei*probability
        selected = candidates[int(np.argmax(utility))][0]
        self.pending.add(self.space.key(selected))
        return selected
