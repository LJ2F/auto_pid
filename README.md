# PID Workbench

通用 STM32 PID SDK + Windows 本地服务 + 静态调参网页，无需大模型或 MATLAB。

- P / PI / PD / PID，位置式、增量式与双环串级。
- 兼容参考 transfer_protocol 帧格式的固定缓冲编解码器。
- 串口 / USB CDC、电脑主动连接 TCP。
- 模型初值、受限开环/闭环辨识；贝叶斯优化、模式搜索、CMA-ES。
- 只搜索勾选的数值参数；评分、独立复验和 JSON / CSV / C 配置导出。

## 启动

需要 Python 3.10+，在工程目录执行：

```powershell
python -m pip install -r requirements-workbench.txt
.\open_workbench.cmd
```

浏览器访问 `http://127.0.0.1:8766`。启动不会运行设备。首次体验选“仿真”，PI，搜索 Kp/Ki 范围 0–5，起步选“从对象模型生成初值”，模型保留默认值。

包含 Windows x64 仿真 DLL；重新构建需 GCC/Clang：`python tools/build_workbench.py`。

## 使用与接板

- [工作台操作与算法](docs/WORKBENCH.md)
- [STM32 SDK 接入](sdk/README.md)
- [通信协议](docs/WORKBENCH_PROTOCOL.md)

实际设备需实现传感器、执行器、通信队列和退出动作。当前验证覆盖主机仿真；未进行 STM32 真机验收。串级按内环→外环分阶段整定，尚无跨环一键调度。试验结束恢复原配置，参数不自动写 Flash。

## 测试

```powershell
$env:PYTHONPATH='src'
python -m unittest discover -s tests -p test_workbench.py -v
```

纯 C 核心测试：`tests/workbench_core_test.c`。
