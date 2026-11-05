from __future__ import annotations

from dataclasses import asdict, replace
import gzip
import json
from pathlib import Path
import threading
import time
import numpy as np

from .analysis import Objective, frequency_response, score
from .design import PlantModel, frequency_seeds, simc_seed
from .device import Device, DeviceError, Trace, TrialAborted
from .schema import Configuration, Plan
from .search import Bound, Search, Space


class BudgetExhausted(RuntimeError):
    pass


class Runner:
    def __init__(self, device: Device, request: dict, cancel: threading.Event, emit, directory: Path):
        self.device, self.request, self.cancel, self.emit, self.directory = device, request, cancel, emit, directory
        self.config = Configuration.from_dict(request["configuration"])
        self.plan = Plan.from_dict(request["plan"])
        self.objective = Objective(**request.get("objective", {}))
        self.action = request.get("action", "experiment")
        self.mode = request.get("start_mode", "existing")
        if self.action not in ("experiment", "identify", "tune") or self.mode not in ("existing", "model", "open_loop", "closed_loop"):
            raise ValueError("未知任务或起步模式")
        self.max_trials = request.get("max_trials", 30)
        self.max_seconds = request.get("max_seconds", 600)
        if type(self.max_trials) is not int or not 1 <= self.max_trials <= 200 or not 1 <= self.max_seconds <= 7200:
            raise ValueError("预算应为 1–200 次试验、1–7200 秒")
        self.confirmed = request.get("open_loop_confirmed") is True
        if (self.mode == "open_loop" or self.plan.open_loop) and not self.confirmed:
            raise ValueError("需要本次显式启用受限开环辨识")
        if self.mode == "open_loop" and request.get("plant_class") not in ("stable", "single_integrator"):
            raise ValueError("开环辨识必须声明对象为稳定或单积分对象，并由设备集成方允许")
        if self.mode == "open_loop" and not device.info["open_loop_available"]:
            raise ValueError("设备未允许开环辨识")
        if self.plan.active_loop >= len(self.config.loops):
            raise ValueError("所选控制环不存在")
        if self.action == "tune" and (self.plan.wave or self.plan.open_loop):
            raise ValueError("调参评分计划必须是闭环阶跃；开环只在辨识阶段启用")
        if abs(self.plan.dt-device.info["dt"]) > device.info["dt"]*0.001:
            raise ValueError("计划周期与设备不一致")
        self.train = [Plan.from_dict(p) for p in request.get("training_plans", [request["plan"]])]
        self.validation = [Plan.from_dict(p) for p in request.get("validation_plans", [request["plan"]])]
        if not 1 <= len(self.train) <= 8 or not 1 <= len(self.validation) <= 8:
            raise ValueError("训练和验证各需要 1–8 个工况")
        if self.action == "tune":
            for p in self.train+self.validation:
                if p.open_loop or p.wave or p.active_loop != self.plan.active_loop or p.amplitude == 0:
                    raise ValueError("训练/验证必须为同一控制环的非零闭环阶跃")
                if p.settle < 3*p.dt*p.decimation or p.duration-p.settle < 25*p.dt*p.decimation:
                    raise ValueError("评分计划需要足够的阶跃前、阶跃后和稳态采样")
        self.space = Space(self.config, [Bound(**b) for b in request.get("parameters", [])]) if self.action == "tune" else None
        if self.space:
            names = [b.name for b in self.space.bounds]
            if self.plan.active_loop == 0 and any(n.startswith("outer.") for n in names):
                raise ValueError("内环试验不会运行外环，不能搜索外环参数")
            prefix = ("inner.", "outer.")[self.plan.active_loop]
            if self.mode != "existing" and any(not n.startswith(prefix) for n in names):
                raise ValueError("频域初值阶段须固定其他环；串级联合微调请选择已有参数模式")
        self.seed = request.get("seed", 1)
        if type(self.seed) is not int or not 0 <= self.seed < 2**32:
            raise ValueError("随机种子应为 uint32")
        self.method = request.get("method", "bayesian")
        if self.method not in ("bayesian", "pattern", "cmaes"):
            raise ValueError("未知搜索器")
        self.count = 0
        self.experiment_seconds = 0.0
        self.started = 0.0
        self.records = []
        self.frequency_data = []
        self.invalid_streak = 0
        self.original = device.configuration

    def check_budget(self, plan: Plan):
        if self.cancel.is_set():
            raise TrialAborted("cancelled", [])
        if self.count >= self.max_trials or self.experiment_seconds+plan.duration > self.max_seconds or time.monotonic()-self.started > self.max_seconds:
            raise BudgetExhausted("已达到总试验次数或时间预算")

    def trial(self, cfg: Configuration, plan: Plan, purpose: str, evaluate=True):
        self.check_budget(plan)
        self.count += 1
        self.experiment_seconds += plan.duration
        record = dict(number=self.count, purpose=purpose, configuration=cfg.to_dict(), plan=asdict(plan),
                      status="running", metrics=None, reason=None, simulated=self.device.link.simulated)
        self.emit("trial_start", record)
        trace = None
        try:
            trace = self.device.run(cfg, plan, self.cancel, lambda rows: self.emit("samples", rows), self.confirmed)
            record["configuration"] = trace.configuration.to_dict()
            record["trial_id"], record["revision"] = trace.trial, trace.revision
            record["metrics"] = score(trace, self.objective) if evaluate else None
            record["status"] = "complete" if not evaluate or record["metrics"]["feasible"] else "infeasible"
            self.invalid_streak = 0
        except TrialAborted as exc:
            record["status"], record["reason"] = "cancelled" if exc.reason == "cancelled" else "aborted", exc.reason
            trace = Trace(exc.samples, cfg, plan, self.device.trial, self.device.revision, self.device.link.simulated)
            if exc.reason == "cancelled":
                raise
            if exc.reason in ("timeout", "overflow", "nonfinite"):
                record["status"] = "invalid"
                raise RuntimeError(f"设备或采样故障：{exc.reason}，本次数据不用于参数搜索") from exc
        except ValueError as exc:
            record["status"], record["reason"] = "invalid", str(exc)
            self.invalid_streak += 1
        except (OSError, DeviceError, TimeoutError) as exc:
            record["status"], record["reason"] = "invalid", str(exc)
            self.invalid_streak += 1
            raise  # infrastructure failures never become artificial large scores
        finally:
            if trace is not None:
                with gzip.open(self.directory/f"trial-{self.count:04d}.json.gz", "wt", encoding="utf-8") as f:
                    json.dump({**record, "samples": trace.samples}, f, ensure_ascii=False, allow_nan=False)
            self.records.append(record)
            self.emit("trial_end", record)
        if self.invalid_streak >= 2:
            raise RuntimeError("连续两次试验数据无效，请检查初始状态、采样或测试计划")
        return record, trace

    def evaluate(self, cfg: Configuration, plans: list[Plan], purpose: str):
        metrics = []
        for plan in plans:
            record, _ = self.trial(cfg, plan, purpose)
            if record["metrics"] is None:
                return None, record["status"]
            metrics.append(record["metrics"])
        costs = [m["cost"] for m in metrics]
        # Equal-condition weighting plus explicit worst-condition term, fixed for the run.
        return dict(cost=float(0.7*np.mean(costs)+0.3*np.max(costs)),
                    constraint=max(m["constraint"] for m in metrics), cases=metrics), "complete"

    def identify(self):
        frequencies = self.request.get("frequencies_hz", [0.2, 0.5, 1.0, 2.0, 4.0])
        if not 4 <= len(frequencies) <= 12 or len(set(frequencies)) != len(frequencies):
            raise ValueError("请选择 4–12 个不重复辨识频率")
        amplitude = self.request.get("identification_amplitude", 0.1)
        if not isinstance(amplitude, (float, int)) or not np.isfinite(amplitude) or amplitude <= 0:
            raise ValueError("辨识幅值必须为正数")
        for frequency in frequencies:
            if not isinstance(frequency, (float, int)) or not np.isfinite(frequency) or frequency <= 0:
                raise ValueError("辨识频率必须为正数")
            plan = replace(self.plan, wave=1, open_loop=int(self.mode == "open_loop"),
                           frequency_count=1, frequencies=(frequency,), amplitude=amplitude,
                           duration=self.plan.settle+8/frequency)
            record, trace = self.trial(self.config, plan, "identification", evaluate=False)
            if record["status"] != "complete" or trace is None:
                raise RuntimeError("辨识试验未完成，不能生成初始参数")
            point = frequency_response(trace, frequency)
            self.frequency_data.append(point)
            self.emit("frequency", point)
        return self.frequency_data

    def run(self) -> dict:
        self.directory.mkdir(parents=True, exist_ok=True)
        self.started = time.monotonic()
        report = dict(version=1, simulated=self.device.link.simulated, request=self.request,
                      validated=False, best=None, records=self.records, frequency_response=self.frequency_data)
        try:
            if self.action == "experiment":
                record, _ = self.trial(self.config, self.plan, "manual", evaluate=not(self.plan.open_loop or self.plan.wave))
                report["status"] = record["status"]
                return report
            if self.action == "identify":
                self.identify()
                report["status"] = "complete"
                return report
            search = Search(self.space, self.method, self.seed)
            reserve = 3*len(self.validation)
            initial_min = 2*len(self.train)
            identification_count = len(self.request.get("frequencies_hz", [0.2, 0.5, 1, 2, 4])) if self.mode in ("open_loop", "closed_loop") else 0
            if self.max_trials < reserve+initial_min+len(self.train)+identification_count:
                raise ValueError("预算不足：需保留基准重复、搜索和 3 次独立复验")
            identification_time = sum(self.plan.settle+8/f for f in self.request.get("frequencies_hz", [0.2, 0.5, 1, 2, 4])) if identification_count else 0
            if identification_time+3*sum(p.duration for p in self.train+self.validation) > self.max_seconds:
                raise ValueError("时间预算不足以完成辨识、基准、搜索和独立复验")
            candidates = []
            baseline_values = []
            if self.mode == "closed_loop":
                for _ in range(2):
                    measured, status = self.evaluate(self.config, self.train, "baseline")
                    if measured is None or measured["constraint"] > 0:
                        raise RuntimeError("闭环辨识前的基准未通过，请先建立可用控制器或调整试验")
                    search.tell(self.config, measured["cost"], measured["constraint"])
                    baseline_values.append(measured["cost"])
            if self.mode in ("open_loop", "closed_loop"):
                self.identify()
            model = None
            if self.mode == "model":
                model = PlantModel(**self.request["model"])
                self.frequency_data.extend(model.samples(self.request.get("frequencies_hz", [0.1, 0.2, 0.5, 1, 2, 4])))
                candidate = simc_seed(self.space, model, max(model.tau, model.delay*3), self.plan.active_loop)
                if candidate:
                    candidates.append(candidate)
            if self.frequency_data:
                designs = frequency_seeds(self.space, self.frequency_data, self.plan.dt, self.plan.active_loop,
                                          self.request.get("phase_margin", 60), self.seed)
                candidates.extend(row["configuration"] for row in designs)
                self.emit("designs", [{**row, "configuration": row["configuration"].to_dict()} for row in designs])
            # Zero-start skips running all-zero PID as a fictitious stabilizing baseline.
            starting = self.config if self.mode in ("existing", "closed_loop") else (candidates.pop(0) if candidates else None)
            if starting is None:
                raise RuntimeError("没有有效初始候选")
            for _ in range(0 if self.mode == "closed_loop" else 2):
                measured, status = self.evaluate(starting, self.train, "baseline")
                if measured is not None:
                    search.tell(starting, measured["cost"], measured["constraint"])
                    baseline_values.append(measured["cost"])
                else:
                    search.tell(starting, status="aborted" if status == "aborted" else "invalid")
                    raise RuntimeError("初始候选未完成有效试验，停止自动搜索")
            report["baseline_cost"] = float(np.mean(baseline_values)) if baseline_values else None
            while self.count+len(self.train)+reserve <= self.max_trials:
                reserved_seconds = 3*sum(p.duration for p in self.validation)
                if self.experiment_seconds+sum(p.duration for p in self.train)+reserved_seconds > self.max_seconds:
                    break
                candidate = candidates.pop(0) if candidates else search.ask()
                measured, status = self.evaluate(candidate, self.train, "search")
                if measured is None:
                    search.tell(candidate, status="aborted" if status == "aborted" else "invalid")
                    continue
                search.tell(candidate, measured["cost"], measured["constraint"])
                self.emit("candidate", dict(configuration=candidate.to_dict(), **measured))
            if not search.observations:
                raise RuntimeError("未获得有效的候选性能数据")
            best = search.incumbent()
            validation = []
            for _ in range(3):
                measured, status = self.evaluate(best, self.validation, "validation")
                if measured is None:
                    report["status"] = "validation_failed"
                    return report
                validation.append(measured)
            report["validation"] = validation
            report["validation_cost_mean"] = float(np.mean([v["cost"] for v in validation]))
            report["validation_cost_std"] = float(np.std([v["cost"] for v in validation], ddof=1))
            report["validated"] = all(v["constraint"] <= 0 for v in validation)
            report["best"] = best.to_dict()
            report["status"] = "validated" if report["validated"] else "validation_failed"
            report["validation_scope"] = "held_out_conditions" if self.train != self.validation else "independent_repetitions"
            report["improvement_comparable"] = self.train == self.validation
            report["observed_improvement"] = (report["baseline_cost"]-report["validation_cost_mean"]) if self.train == self.validation else None
            report["improvement_claim"] = "descriptive_only_not_significance_test"
            return report
        except TrialAborted:
            report["status"] = "cancelled"
            return report
        except BudgetExhausted as exc:
            report["status"], report["error"] = "budget_exhausted", str(exc)
            return report
        except Exception as exc:
            report["status"], report["error"] = "failed", str(exc)
            return report
        finally:
            report["trials_used"] = self.count
            report["planned_experiment_seconds"] = self.experiment_seconds
            report["wall_seconds"] = time.monotonic()-self.started
            try:
                self.device.stop()
                self.device.configure(self.original)
                report["restored_original"] = True
            except (OSError, RuntimeError, TimeoutError, ValueError) as exc:
                report["restored_original"] = False
                report["restore_error"] = str(exc)
            (self.directory/"report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
