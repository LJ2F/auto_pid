# PID Workbench 详细使用说明书

面向使用者、STM32 集成者和后续维护者。本文的物理数值除明确标为“仿真示例”外，都应由实际对象决定；本仓库没有绑定具体 `.ioc`、MCU、引脚或电机驱动。

导航：[README](../README.md) · [函数参考](FUNCTION_REFERENCE.md) · [协议字段](WORKBENCH_PROTOCOL.md) · [算法实现](WORKBENCH.md)

## 1. 系统如何工作

```mermaid
flowchart LR
    UI[浏览器静态网页] <-->|本机 HTTP| Host[Windows 服务 / 数值算法]
    Host <-->|串口或 TCP 字节流| SDK[STM32 SDK]
    SDK -->|控制输出| Drive[现有执行器驱动]
    Drive --> Plant[被控对象]
    Plant --> Sensor[传感器 / 状态估计]
    Sensor -->|测量值| SDK
```

STM32 固定周期执行控制与实验边界检查，电脑负责选择参数、组织试验和处理数据，网页负责编辑与展示。串口不是执行器总线：电脑可以通过 UART 与 STM32 通信，而 STM32 再通过 CAN 控制电机。

`simulation` 将同一 C SDK 接到数学对象；`serial` 和 `tcp` 接到真实固件。只有仿真会在每次试验前重置数学对象，真实设备没有自动回零功能。

## 2. 安装、启动和关闭

### 2.1 安装

1. 从仓库克隆，或下载 ZIP 并解压。
2. 安装 Python 3.10+，确认终端中的 `python --version` 正确。
3. 在含 `README.md` 的工程根目录执行：

```powershell
python -m pip install -r requirements-workbench.txt
```

可选虚拟环境：

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-workbench.txt
```

启动脚本优先使用本目录的 `.venv/Scripts/python.exe`，否则使用 PATH 的 `python`。

### 2.2 启动方式

| 方式 | 操作 |
|---|---|
| 双击 | `open_workbench.cmd` |
| 不打开浏览器 | `powershell -NoProfile -ExecutionPolicy Bypass -File .\start_workbench.ps1 -NoBrowser` |
| 换端口 | 上述命令追加 `-Port 8767`，访问相应端口 |
| 直接启动模块 | 设置 `PYTHONPATH=src` 后执行 `python -m pid_agent.workbench` |

Python 命令行参数：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `--host` | `127.0.0.1` | 仅允许回环地址 `127.0.0.1`/`localhost` |
| `--port` | `8766` | 本地 HTTP 端口 |
| `--results` | 当前工作目录下 `results/workbench` | 结果保存目录 |
| `--allow-origin` | 无额外来源 | 允许另一个静态网站的完整 origin，可重复传入 |
| `--open-browser` | 不启用 | 服务绑定端口后打开默认浏览器 |

直接启动例子：

```powershell
$env:PYTHONPATH = Join-Path $PWD 'src'
python -m pid_agent.workbench --port 8766 --results results/workbench --open-browser
```

网页可独立托管，但应从 HTTP(S) 打开，并明确允许其 origin，例如 `--allow-origin http://localhost:8080`；页面“本地服务”填写 `http://127.0.0.1:8766`。浏览器对访问本地网络可能还有自身限制，首次使用优先选同源托管。单独部署网页不能替代本地桥接进程。

### 2.3 仿真库

Windows x64 仿真使用仓库的 `src/pid_agent/workbench/native/pt_sdk.dll`。缺少库时启动脚本会尝试构建，需要 PATH 有 GCC 或 Clang：

```powershell
python tools/build_workbench.py
```

库与 Python 进程架构应一致。SDK 原代码既用于 STM32，也编译进此主机库，不能拿 ARM 固件二进制代替 DLL。串口/TCP 不通过 DLL 执行实际控制。

### 2.4 关闭与取消

- 停止按钮：发出取消请求，等待通讯事务结束、设备退出、恢复原配置。
- 关闭网页：不取消正在运行的本地任务。
- 服务窗口 `Ctrl+C`：服务请求取消并关闭连接。
- 通信断开/服务异常消失：端侧在后续控制 tick 检查通讯超时并调用退出钩子；任务停止调度、硬件掉电等不由这个软件检查覆盖。

## 3. 首次完整仿真

### 3.1 连接与参数

通信方式选“仿真”，点击连接。设备初始增益可能为零，连接不是对示例参数的自动应用。内置速度对象为：

```text
G(s) = exp(-0.03s) / (0.6s + 1)
基本控制周期约 0.005 s；位置测量由速度积分得到。
```

上式为对应的理想连续模型；实际仿真按固定步长更新，并用离散延迟队列近似延迟。

控制器选 PI、位置式、条件积分，不启用双环。勾选 `kp`、`ki`，均使用 0–5 的线性搜索范围。其余参数/开关先保持载入值。

### 3.2 计划

| 项目 | 仿真示例 |
|---|---:|
| 起步方式 | 从对象模型生成初值 |
| 模型 | 一阶惯性 + 延迟 |
| 模型增益、时间常数、延迟 | 1、0.6 s、0.03 s |
| 算法 | 约束贝叶斯优化 |
| 时长、稳定时间 | 6 s、0.5 s |
| 初始目标、训练阶跃、验证阶跃 | 0、1、0.8 |
| 记录倍率 | 2 |
| 总次数、总时间 | 10、180 s |

点击“开始自动整定”。观察基准、搜索、复验记录及最新试验曲线。结果应明确标为仿真；具体候选值取决于设置、种子和软件环境，不要将例子参数直接用于实际机构。

### 3.3 导出或应用

`validated` 表示三次验证都通过本次设定的性能约束。可以导出 C 头文件，或显式应用到当前连接设备 RAM。结束后默认恢复任务之前的配置，并不会自动保存候选到 Flash。以后切到真实设备时，重新确定单位与边界。

## 4. 页面各项设置

### 4.1 连接与控制器

| 页面项 | 实际作用 |
|---|---|
| 本地服务 | 网页请求的本机服务地址，不是 STM32 的 TCP 地址 |
| 串口/波特率 | 服务打开该系统 COM；USB CDC 也按系统串口处理 |
| TCP 地址/端口 | 电脑主动连接板端/透明传输设备的 TCP 服务 |
| 载入设备参数 | 将最近一次设备确认的配置载入编辑器；按钮本身不额外发起 STATUS |
| 编辑环 | 选择当前编辑内环或外环，不等于选择本次试验的被调环 |
| 双环串级 | 在本次提交的配置中使用两个 PID |
| 外环周期倍率 | 外环周期 = 基本周期 × 倍率；单环时无外环控制作用 |
| 仅应用当前配置 | 校验、写 RAM、回读；不开始试验、不写 Flash |

网页编辑值不会自动应用。一次搜索固定类型、计算形式、功能、边界、评分配置和未选择参数。

### 4.2 PID 数值及单位

令误差 `e=r-y`，输出单位 U，误差单位 E。SDK 增益按秒定义。

| 字段 | 单位/作用 | 何时有作用 |
|---|---|---|
| `kp` | U/E，比例增益 | 所有类型 |
| `ki` | U/(E·s)，积分增益 | PI/PID |
| `kd` | U·s/E，微分增益 | PD/PID |
| `integral_limit` | U，积分贡献绝对上限 | 积分限幅开关或 clamp |
| `deadband` | E，死区内清积分、输出零再限幅 | 大于零时 |
| `integral_a` | E，变速积分衰减区宽度，必须为正 | 变速积分 |
| `integral_b` | E，全速积分误差阈值 | 变速积分 |
| `derivative_tau` | s，微分一阶滤波常数 | 微分滤波开关 |
| `output_tau` | s，输出一阶滤波常数 | 输出滤波开关 |
| `aw_gain` | 1/s，反算修正增益 | backcalc |
| `ffj` | U·s/测量单位，乘测量变化率 | 测量前馈 |
| `ffb` | U/测量单位，乘测量值 | 测量前馈 |
| `output_min/max` | U，PID 输出范围 | 所有类型 |
| `blocked_error` | E，堵转检测误差阈值 | 堵转检测 |
| `blocked_time` | s，大误差且输出靠近限值的累计时间 | 堵转检测 |

比例对测量取 `P=-kp*y`，普通比例为 `kp*(r-y)`；前者相当于设定值比例权重 0。P/PD 没有积分时，比例对测量不会产生普通的目标比例响应。FFJ/FFB 沿用参考代码的测量定义，不是目标速度/加速度前馈。

变速积分仅在误差和当前积分同向时衰减：误差不超过 `integral_b` 时全速，在之后 `integral_a` 范围线性减至零。梯形积分用前后误差均值，第一次调用没有历史时用当前误差。

如果原控制器使用 `I += Ki_old*e`、`D = Kd_old*(e-e_old)`，且其他定义一致，初始换算为 `Ki = Ki_old/dt`、`Kd = Kd_old*dt`。这只是相同公式下的单位换算，仍需核对限幅、滤波和反馈单位。

### 4.3 搜索勾选与范围

- 允许选择 1–16 个对当前结构有效的参数；未选数值保持固定。
- 当前值必须在范围内，float32 量化后下界仍小于上界。
- 对数尺度要求下界严格大于零，不适用于包含零或跨正负的范围。
- 输出边界/堵转阈值不能由算法放宽来提高分数。
- 无滤波的测量微分 Kd 与 FFJ 可能作用冗余；实现拒绝对应的同时搜索组合。
- 内环试验不运行外环，不能搜索外环参数；频域初值模式固定另一环，已有参数模式的外环试验可同时优化两环。

### 4.4 试验设置

| 字段 | 说明 |
|---|---|
| 控制周期 `dt` | 从设备回读，网页只读；改网页不能改变 MCU 调度 |
| `duration` | 包含阶跃前稳定阶段的总时长 |
| `settle` | 此前使用初始目标，之后施加阶跃/正弦 |
| `reference` | 阶跃前目标；阶跃后为 reference + amplitude |
| `amplitude` | 可为负；评分必须非零 |
| 验证幅值 | 网页的一个独立验证阶跃；Python/HTTP 可提供更多工况 |
| `decimation` | 每多少控制 tick 记录一点，不改变 PID 周期 |
| 测量范围 | 检查实际传感器值；双环时调内环也检查外环行程 |
| 实际输出范围 | 实验执行器限幅，同时缩窄内环 PID 有效范围 |
| 最大变化率 | 大于零时超限即终止，不是生成斜坡；起始历史输出为 0 |
| 心跳超时 | 100–5000 ms，依赖设备周期检查 |
| 总试验次数 | 1–200；基准、辨识、搜索、复验都计入 |
| 总时间 | 1–7200 s；同时检查计划实验秒数与墙钟时间 |

评分需要阶跃前至少 3 点、阶跃后至少 20 点、尾部至少 3 点；计划预检查还要求留出足够记录间隔。记录倍率过大可能使窗口采样不足。

## 5. 起步方式、辨识与搜索

### 5.1 四种起步

下面表格描述“开始自动整定”按钮的流程。“运行一次闭环阶跃”仅运行一次闭环计划；“仅频响辨识”只采集频响，不自动搜索或复验，也不自动执行整定流程中的两次基准。仅辨识时选择闭环模式，应由使用者先确认已有可用闭环。

| 模式 | 流程 | 前提 |
|---|---|---|
| 已有参数 | 两次基准→搜索→三次复验 | 初始控制器能完成试验 |
| 模型初值 | 模型频响→可用时 SIMC→频域候选→基准/搜索/复验 | 提供模型及符号方向 |
| 闭环辨识 | 两次基准先通过约束→原闭环逐频注入→搜索→复验 | 已有满足测试约束的控制器 |
| 受限开环 | 绕过被调 PID 注入→频响→闭环初值→搜索→复验 | 本次确认、板端许可、声明 stable/single_integrator |

模型包括一阶惯性延迟、二阶、带一阶惯性的积分对象。代码不会自动证明对象类别。外环开环只旁路外环，保持已经可用的内环运行。

### 5.2 频率与预算

默认 0.2、0.5、1、2、4 Hz，逐频测量。每频点时长 `settle + 8/f`，低频明显增加总时间。要求 4–12 个不重复正频率，单次正弦计划只携带一个频点。

记录每周期至少 10 点，即 `f*dt*decimation <= 0.1`。外环初值设计还按外环实际周期检查频带。去掉前 2 周期并回归去直流/线性趋势；出现饱和、信噪比小于 3、窗口相对差异大于 0.3 时拒绝频响。窗口一致性不是统计置信区间。

### 5.3 三种搜索器

| 算法 | 实现及使用条件 |
|---|---|
| bayesian | Matérn GP、观测噪声、可行概率、Monte Carlo noisy EI、自适应局部范围；少量昂贵实测时先用 |
| pattern | 坐标正负方向、缩小步长、有界随机备用候选；少量参数便于理解 |
| cmaes | 协方差、演化路径、步长适应、可行性优先；预算允许较多候选时使用 |

CMA-ES 每代 `4 + int(3*log(d+1))` 个候选，d 为维度。低预算可能未充分更新种群。算法不保证全局最优，GP 可行概率不是硬件保护。

### 5.4 评分与复验

```text
cost = 加权平均(归一化 ITAE, 超调, 稳态平均绝对误差, 输出总变化, 饱和占比)
score = 100 / (1 + cost)
constraint = max(超调-上限, 稳态误差-上限, 饱和占比-上限)
```

默认权重 0.55、0.20、0.15、0.05、0.05。`constraint <= 0` 才通过，低代价不能抵消违反约束。稳态误差取阶跃后最后 20% 时间的平均绝对误差。

评分先检查完整轨迹、有限数、索引/采样间隔，再检查阶跃前最后 0.2 s 窗口误差与波动。无法恢复工作点时应实现物理恢复流程，不能给无效曲线人工罚分。基础设施错误标 invalid；限位等退出标 aborted；完整但性能不符标 infeasible。

同参数重复观测按平均代价、最坏约束选当前最优，最终三次复验独立采集。通过只表示满足设定条件，不代表统计显著优于基准。

## 6. 真实 STM32 接入

### 6.1 工程信息与源文件

以下项目均需人工确认：MCU/编译器、UART/USB/网络接口、TX/RX 引脚、传感器/执行器单位、控制周期、CAN/PWM 驱动、退出策略、工作点恢复、重启标识来源。本仓库没有 `.ioc` 或固定外设句柄。

仅使用 PID 算法时编译 `pt_pid.c`；使用网页整定则加入 `pt_pid.c`、`pt_protocol.c`、`pt_runtime.c` 和 `pt_sdk.h`，添加 `sdk/include` 路径。按 C99 编译；C++ 工程可用带 `extern "C"` 的头文件。使用工具链数学库，避免让 `isfinite` 失效的 fast-math。

`pt_runtime` 建议静态分配。默认帧 1024 字节、64 点样本，主机 x64 结构体 8640 字节；512 字节帧/16 点样本变体为 3712 字节。ARM RAM/执行时间需实测。宏必须对所有 SDK 编译单元一致；主机最大帧当前为 1024，增大 MCU 帧上限时需同步主机。

### 6.2 板级函数分工

参考 [control_task.c](../sdk/examples/control_task.c)。前五个 `board_*` 只有声明，需要实现：

| 函数 | 要实现的内容 |
|---|---|
| `board_tx_copy(bytes,size)` | 返回前把整帧复制到 TX 队列，由传输任务/DMA 发送 |
| `board_rx_pop(bytes,capacity)` | 从 RX 队列最多取 capacity 字节，返回实际数量，无数据为 0 |
| `board_apply_output(output)` | 将 SDK 输出转换为驱动指令，维持约定单位/方向 |
| `board_leave_test(reason)` | 记录原因并停机/保持/移交，必要时锁存禁止恢复 |
| `board_normal_control()` | 非试验的原控制逻辑，服从退出锁存和使能状态 |
| `tuning_init(...)` | 示例已实现：初始化 SDK，设置周期/硬输出范围 |
| `tuning_control_tick(...)` | 示例已实现：取 RX、执行 SDK、仲裁输出 |

RX/TX 队列、队列溢出状态和传感器读取由应用提供。TX 回调为 void，发送失败不会自动返回 SDK；队列满应由适配层报告并安排退出，不可假定 SDK 已知道失败。

### 6.3 初始化与周期调用

顺序：外设/队列可用→初始化 SDK→设置真实周期/硬输出范围→持续固定周期调度。SDK 默认周期 1 ms、硬输出范围 -1…1，不是实际设备的通用参数。

下面全部 `app_*` 都是需人工实现的板级接口。这是调用示意，缺少外设和函数声明，不能直接当作 HAL 工程编译烧录：

```c
#include "pt_sdk.h"
static pt_runtime tuner;

void app_tuner_init(uint32_t new_boot_id, float period_s,
                    float hardware_min, float hardware_max) {
    pt_runtime_init(&tuner, new_boot_id, app_tx_copy, app_exit_test, NULL);
    pt_runtime_set_hardware(&tuner, period_s, hardware_min, hardware_max);
    /* 仅在该对象允许时：pt_runtime_allow_open_loop(&tuner, 1); */
}

void app_control_tick(uint32_t now_ms) {
    uint8_t rx[128];
    size_t n = app_rx_pop(rx, sizeof(rx));
    if (n) pt_runtime_receive(&tuner, rx, n, now_ms);
    uint32_t before = pt_runtime_state(&tuner);
    float y_inner = app_read_inner();
    float y_outer = app_has_outer() ? app_read_outer() : 0.0f;
    float u = pt_runtime_tick(&tuner, y_inner, y_outer, now_ms);
    if (pt_runtime_state(&tuner) == PT_RUNNING) {
        app_apply_output(u);
    } else if (before != PT_RUNNING && app_normal_control_allowed()) {
        app_normal_control();
    }
}
```

调用约束：

- boot_id 每次重新启动应变化，不能永远填常数；来源由应用计数器/随机源确定。
- now_ms 为当前毫秒时间，不是周期秒数；period_s 才是秒。
- 单环无外环传感器时传有限的 0.0f；当前 tick 对两个输入都做有限性检查。
- receive/tick/abort 归同一个控制上下文，不在 ISR 和任务中并发访问同一对象。
- 空闲时也持续服务接收，否则无法握手/配置。
- tick 退出时可能返回上次输出，只有状态仍 RUNNING 才能写执行器。
- 退出回调动作不能被同一 tick 或下一 tick 的原控制覆盖；禁止两个控制器同时写同一执行器。
- 示例每 tick 只展示一次有界取队列；参考 C 文件最多四批。按实际吞吐分配预算，避免无限处理 RX 占用控制时间。

### 6.4 UART / USB CDC / TCP 与 CubeMX

UART 需人工确认异步模式、8N1、波特率、GPIO 复用、DMA/NVIC 和电平。常规 TTL UART 是 TX→对端 RX、RX→对端 TX、共地；电平/接插件由板卡确定，RS-232 不能直接当 TTL。

接收回调只将字节放入 RX 队列；发送完成回调推进 TX 队列。SDK 不绑定 HAL 回调名。DMA 发送缓冲区必须稳定到发送完成，不能保存 SDK 回调指针后异步直接发。DMA 可访问 RAM、缓存一致性和环形覆盖由适配层处理。

USB CDC 由工程配置设备类/时钟/收发，PC 识别 COM 后按串口连接。TCP 由板端监听、电脑主动连接；SDK 不创建 LwIP 服务、网卡或 socket。透明网关可转发同一协议字节流。

CubeMX 仅配置实际选用接口；引脚、DMA 请求、IRQ 优先级、任务栈和时钟值需人工确认。生成文件调用放 `USER CODE` 区，主要适配放独立用户文件，避免重新生成覆盖。

### 6.5 控制与记录带宽

```text
记录点/秒 = 1 / (dt × decimation)
串口 8N1 字节能力 ≈ baud / 10
主机预检：75 / (dt × decimation) <= (baud / 10) × 0.75
```

仅作带宽例子：dt=0.001、115200 波特、倍率 10，记录 100 Hz，估算 7500 B/s 小于保守预算 8640 B/s。倍率 2 则估算 37500 B/s，超预算。记录倍率不会改变 1 kHz 控制周期。

## 7. 单环、串级与参数迁移

### 7.1 内环

被调环选内环，仅勾选内环数值。双环配置下外环 PID 被旁路，但检查外环行程仍有效。速度测试也可能导致位置持续变化，需给出实际允许行程。

### 7.2 外环

先完成内环并显式应用验证结果，固定内环参数，再切换外环编辑/外环试验，只勾选外环参数。外环输出是内环目标，如位置外环输出速度，不是电流；输出范围使用相应单位。

外环开环保留内环闭合，输入为偏置+激励的内环目标，范围按外环输出范围检查。外环模型应是“已闭合内环到外环测量”的等效模型，不能直接使用电流到速度的原始电机模型。

### 7.3 联合与自写 PID

已有参数模式下外环试验可以同时勾选内外环做时域联合优化。当前没有跨两环一键调度。

导出的头文件包含完整 pt_config。迁移自写 PID 时保持 dt、单位、离散公式、开关和限幅一致；不同定义的 Ki/Kd 不能直接照抄，应先用相同输入轨迹对照实现。

独立 PID 核心用法：

```c
pt_pid_config cfg;
pt_pid controller;
pt_pid_defaults(&cfg);
cfg.kind = PT_PI;
cfg.kp = 1.0f; /* 数学示例，不代表实际电机参数 */
cfg.ki = 2.0f;
cfg.output_min = -5.0f;
cfg.output_max = 5.0f;
if (pt_pid_validate(&cfg)) pt_pid_init(&controller, &cfg, 0.0f);
/* 每个固定周期：float u = pt_pid_step(&controller, target, measured, dt_s); */
```

此实例与实验 pt_runtime.pid[] 独立。要使用导出结果，用导出配置 loop[0]/loop[1] 初始化正常控制实例；仅下载头文件不会自动改变固件。

## 8. 结果、状态和保存

默认目录 `results/workbench/<job-id>/`：

- trial-0001.json.gz 等：实际配置、计划、状态、指标、原始样本。
- report.json：请求、记录、频响、候选、复验、预算、恢复状态。

| 状态 | 含义 |
|---|---|
| complete | 单次完整完成，不等于整个整定已通过复验 |
| infeasible | 有效完整轨迹，性能约束未通过 |
| aborted | 限位/堵转/变化率等退出，没有完整性能评分 |
| invalid | 通信、缓冲、非有限数、初始状态等导致数据无效 |
| cancelled | 用户取消 |
| failed | 无法继续，看 error 和记录原因 |
| budget_exhausted | 次数/时间用尽，未完成最终验证 |
| validated | 三次最终验证都通过设定约束 |
| validation_failed | 最终验证未完整完成或约束未通过 |

JSON 为完整报告；CSV 展开所选字段的原始曲线；C 配置仅验证通过时可导出。完整 sample 的其他字段仍保存在 gzip。`restored_original=true` 表示配置恢复，不表示已回到初始物理位置；失败会提供 restore_error。

## 9. Python 与 HTTP 接口

在根目录先设 PYTHONPATH=src。以下示例完整运行仿真，不启动真实设备：

```python
from dataclasses import asdict
from pathlib import Path
import threading
from pid_agent.workbench.device import Device, NativeLink
from pid_agent.workbench.schema import Configuration, Plan
from pid_agent.workbench.runner import Runner

device = Device(NativeLink())
request = {
    "action": "tune", "start_mode": "model", "method": "bayesian",
    "configuration": Configuration().to_dict(),
    "plan": asdict(Plan(duration=6.0)),
    "model": {"family": "fopdt", "gain": 1.0, "tau": 0.6, "delay": 0.03},
    "parameters": [
        {"name": "inner.kp", "lower": 0, "upper": 5},
        {"name": "inner.ki", "lower": 0, "upper": 5},
    ],
    "max_trials": 10, "max_seconds": 180, "seed": 1,
}
try:
    runner = Runner(device, request, threading.Event(),
                    lambda event, data: None, Path("results/example"))
    report = runner.run()
    print(report["status"], report.get("best"))
finally:
    device.close()
```

本地 HTTP 接口：

| 方法/路径 | 输入 | 作用 |
|---|---|---|
| GET /api/session | 无 | token 与 schema |
| GET /api/ports | 无 | 系统串口列表 |
| POST /api/connect | transport；serial 加 port/baudrate，tcp 加 host/port | 连接与设备信息 |
| GET /api/state?after=游标 | 可选游标 | 状态/记录/报告，最多 2000 个增量显示点 |
| POST /api/apply | configuration | 只应用配置 |
| POST /api/start | 上述 Runner 请求结构 | 开后台任务，返回 job_id |
| POST /api/stop | 空 JSON 对象 | 请求取消 |
| POST /api/disconnect | 空 JSON 对象 | 非运行时断开 |
| GET /api/export?format=json/csv/c | 选择格式 | 下载结果，C 需已验证 |

POST 使用 application/json，添加 session 返回的 X-Workbench-Token。服务限制 Host/Origin，拒绝 NaN/Infinity，体积最多 1 MiB。

真实设备 start 还要求 limits_confirmed=true；开环还需 open_loop_confirmed=true、对象类别与板端许可。training_plans/validation_plans 各支持 1–8 个。直接调用 Runner/Device 不经过 HTTP 的实际设备确认字段，调用者需自己核对物理条件。

## 10. 常见问题

| 现象 | 排查 |
|---|---|
| 双击立即退出 | 在 PowerShell 运行查看错误，确认 Python/依赖环境 |
| DLL 无法加载 | 核对 Python 与库位数，使用当前架构编译器重建 |
| 8766 占用 | 换 -Port，并更新浏览器地址 |
| COM 为空/打不开 | 驱动/线缆/占用/正确设备；关闭其他串口程序 |
| 插 USB 但不握手 | 固件需适配 SDK 应用消息，仅支持外帧或日志还不够 |
| ACK 超时 | 波特率、TX/RX/地、队列、任务调度、通道混入其他字节 |
| bad_request | 计划/配置长度、输出范围、周期、频率等不符合端侧要求 |
| revision/session/sequence | 重启或计数不一致，停止旧任务并重新连接，检查 boot_id |
| open_denied | 板端尚未许可；网页确认不代替固件许可 |
| limit/blocked/slew | 检查实际越界、单位/方向；slew 超限直接终止 |
| overflow | 带宽、记录倍率、主机轮询或队列不足 |
| timeout | 端侧没收到有效联系；还要保证固件按时运行 |
| nonfinite | 传感器/计算非法值，包括单环误传 NaN 外环输入 |
| 初始状态不稳定 | 对象没回到 reference；实现恢复/保持或增加 settle |
| 频响质量不足 | 幅值、噪声、饱和、频带和持续时间不合适 |
| 预算不足 | 低频辨识耗时大，还需基准、搜索和三次验证预算 |
| 复验通过但参数未变 | 默认恢复旧配置，需要显式应用或在固件初始化 |
| 关闭网页仍运行 | 本地服务独立；重新打开查看或发停止请求 |

## 11. 已知边界

主机验证环境为 Windows x64、Python 3.14、GCC、Edge；项目声明 Python 3.10+，其他具体组合需复验。串口测试为模拟流、TCP 为本机回环，真实外设/板卡/最坏执行时间尚未验收。

需人工确认项：MCU/IDE、引脚/电平、时钟、DMA/NVIC、RTOS 栈/优先级、真实周期、增益符号/单位、输出范围、行程、退出动作、工作点恢复。缺少这些信息时，不把示例常数或某块板的 HAL 配置写成通用事实。

## 12. 开发者复验命令

Python 集成测试：

```powershell
$env:PYTHONPATH = Join-Path $PWD 'src'
python -m unittest discover -s tests -p test_workbench.py -v
```

安装了 GCC 时，在根目录运行 C 核心测试：

```powershell
New-Item -ItemType Directory -Force .test-tmp | Out-Null
gcc -std=c99 -O2 -Wall -Wextra -Werror -pedantic -Isdk/include tests/workbench_core_test.c sdk/src/pt_pid.c sdk/src/pt_protocol.c sdk/src/pt_runtime.c -lm -o .test-tmp/workbench_core_test.exe
.\.test-tmp\workbench_core_test.exe
```

也可以通过 `cmake -S sdk -B build/sdk`、`cmake --build build/sdk` 构建静态 SDK；具体生成器/目标架构由本机工具链决定，这不等于已经构建 STM32 固件。测试目录与结果默认由 .gitignore 排除。
