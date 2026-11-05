# PID Workbench — STM32 自动整定工作台

PID Workbench 由 **STM32 端纯 C SDK、Windows 本地服务、静态网页**组成。使用者固定控制器结构和功能开关，选择需要优化的数值参数；电脑通过串口或 TCP 发起试验，根据实际曲线打分、搜索，再独立复验。数值整定不依赖大模型或 MATLAB。

> 当前已验证主机仿真、协议和浏览器完整流程；尚未针对具体 STM32 板卡完成烧录与真机验收。真实接入需要实现传感器、执行器、通信队列和退出动作。仓库不是可以直接烧进任意 STM32 的完整固件。

## 文档入口

| 想做什么 | 阅读位置 |
|---|---|
| 安装、仿真、串口/TCP、自动整定、导出 | [详细使用说明书](docs/USER_MANUAL.md) |
| 将 SDK 接入 STM32、理解板级函数与控制权 | [说明书：真实 STM32 接入](docs/USER_MANUAL.md#6-真实-stm32-接入) |
| 查询 C/Python/网页中每个函数的作用与调用约束 | [函数参考手册](docs/FUNCTION_REFERENCE.md) |
| 理解辨识、评分、初值和三种搜索器 | [算法与工程说明](docs/WORKBENCH.md) |
| 自己实现主机或兼容设备端 | [通信协议](docs/WORKBENCH_PROTOCOL.md) |
| 查看最小板级适配代码 | [control_task.c](sdk/examples/control_task.c) |

## 1. 支持的功能

- P、PI、PD、PID，位置式/增量式，单环/双环串级。
- 矩形/梯形积分、比例对测量、微分对测量、变速积分、积分限幅、微分/输出滤波、堵转检测、测量前馈 FFJ/FFB。
- 无抗饱和、积分限幅、条件积分、反算抗饱和。
- 串口/USB CDC、电脑作为 TCP 客户端主动连接；内置使用同一 C SDK 的仿真设备。
- 优化已有参数、模型生成初值、闭环频域辨识、显式启用的受限开环辨识。
- 约束贝叶斯优化、局部模式搜索、CMA-ES；线性/对数搜索范围。
- 实时曲线、逐次指标、三次独立复验、JSON/CSV/C 配置导出。

串级按“先内环，再外环”分阶段操作；已有参数模式支持内外环联合微调，当前没有跨两环一键自动调度。网页与代码都能选择功能，但算法只调整本次勾选的数值，不自动切换 PID 类型或功能开关。

## 2. 安装与启动

正式桌面支持 Windows。需要 Python 3.10+，从 GitHub 克隆或下载 ZIP 后解压，在工程根目录打开 PowerShell：

```powershell
python -m pip install -r requirements-workbench.txt
.\open_workbench.cmd
```

浏览器打开 <http://127.0.0.1:8766>。服务窗口保持开启，`Ctrl+C` 关闭服务。**启动服务、打开网页、连接设备，都不会自动开始试验。**

不自动打开浏览器或更换端口：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\start_workbench.ps1 -NoBrowser -Port 8767
```

直接用 Python 启动：

```powershell
$env:PYTHONPATH = Join-Path $PWD 'src'
python -m pid_agent.workbench --port 8766 --open-browser
```

仓库包含 Windows x64 的 `pt_sdk.dll`，用于仿真。缺少它时，启动脚本会尝试构建；重新构建需要 PATH 中有 GCC 或 Clang：

```powershell
python tools/build_workbench.py
```

## 3. 首次完整仿真

1. 网页通信方式选 **仿真 · 同一 C 控制核心**，点击连接。
2. 控制器选 **PI、位置式、条件积分**，先不选双环串级。
3. 勾选 `kp`、`ki`；两者下界均为 `0`、上界均为 `5`，尺度选线性。当前值可以为零。
4. 起步方式选 **从对象模型生成初值**，算法选约束贝叶斯优化。
5. 使用下面的仿真设置，点击 **开始自动整定**。

| 设置 | 仿真示例值 |
|---|---:|
| 每次时长 / 阶跃前稳定时间 | 6 s / 0.5 s |
| 初始目标 / 训练阶跃幅值 | 0 / 1 |
| 独立验证幅值 | 0.8 |
| 记录采样倍率 | 2 |
| 总次数 / 总时间预算 | 10 / 180 s |
| 对象模型 | 一阶惯性 + 延迟 |
| 对象增益 / 时间常数 / 延迟 | 1 / 0.6 s / 0.03 s |

这些值对应内置仿真对象，不是实际电机的默认整定参数。仿真时间会加速推进，完成所需墙钟时间通常短于计划试验秒数。

界面会记录基准、搜索、独立复验三个阶段；上图显示目标和测量，下图显示实际输出。`cost` 越小越好，显示分数越大越好。复验通过后可以导出报告、CSV 和 C 配置。

**结束后默认恢复任务开始前设备上的配置。** 如果需要使用候选结果，显式点击“应用已验证参数”；这只更新 RAM，不会写 Flash。关闭网页后任务仍由本地服务管理；要停止试验，点击停止并等待退出完成。

## 4. 真实 STM32 怎样使用

电脑端仍使用同一网页，STM32 工程需要加入：

```text
sdk/include/pt_sdk.h
sdk/src/pt_pid.c
sdk/src/pt_protocol.c
sdk/src/pt_runtime.c
```

参考 [适配示例](sdk/examples/control_task.c)，在设备固定周期任务中接收排队的字节、读取传感器、调用 SDK、写执行器。中断/网络回调仅收发入队。发送回调须复制报文后再使用 DMA；退出回调决定停机、保持或切回原控制。原 PID 与调试 PID 不能同时向同一执行器写输出。

| 必须确定 | 例子及含义 |
|---|---|
| 测量值与单位 | 速度、位置、角度或电流；需与目标单位一致 |
| 控制输出与单位 | 电流指令、转矩或 PWM；范围由驱动实际能力确定 |
| 真实控制周期 | 由控制任务/定时器决定，不是网页刷新周期 |
| 通信接口 | UART/USB CDC，或板端 TCP 服务 |
| 退出与初始状态 | 实验结束怎么处理执行器，每次实验如何恢复工作点 |

完成固件适配、编译、烧录后，网页选串口和波特率，或设备 TCP 地址/端口；连接并核对设备回读，再填写实际范围和试验设置。先确认小幅试验的方向、曲线和退出动作，再开启自动整定。逐步接入示例、单位换算、带宽与错误排查见 [详细使用说明书](docs/USER_MANUAL.md)。

## 5. 项目结构

```text
sdk/                         纯 C99 PID、帧编解码、实验状态机、板级示例
src/pid_agent/workbench/     Python 配置、设备、辨识、评分、搜索与本地服务
src/pid_agent/workbench/web/ 静态网页，无前端构建依赖
docs/                       使用说明书、函数手册、协议和算法说明
tests/                      Python 集成测试、C 控制核心测试
tools/build_workbench.py    将同一 SDK 编译为主机仿真库
open_workbench.cmd          Windows 双击入口
start_workbench.ps1         启动参数与环境检查
```

## 6. 测试与能力边界

```powershell
$env:PYTHONPATH = Join-Path $PWD 'src'
python -m unittest discover -s tests -p test_workbench.py -v
```

32 项 Python 测试已通过，覆盖协议、故障处理、搜索器、辨识起步和完整 HTTP 整定链路；C 核心及浏览器操作也已在 Windows 主机验证。实际 STM32 的编译链接、DMA、实时周期和物理退出动作仍需具体工程验收。模型、稀疏频响和三次复验均不等于对任意未知对象的稳定性证明，也不保证全局最优参数。
