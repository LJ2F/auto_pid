# PID Workbench 函数参考手册

本手册按当前源码说明项目自有函数，覆盖 C 公开接口/内部辅助、板级钩子、Python、网页和构建入口；测试函数在末尾单独索引。自动生成的 dataclass 方法、标准库和 HAL 函数不当作本项目手写函数重复说明。

使用流程见 [详细说明书](USER_MANUAL.md)，消息字段见 [协议](WORKBENCH_PROTOCOL.md)。标“内部”的函数由所属模块调用，不能绕过外层校验直接作为公共协议使用。

## 1. 调用关系和类型

```text
网页 request → HTTP Workbench.start → Runner.run
  → 初值/辨识 → Search.ask → Runner.evaluate → Runner.trial
  → Device.run → exchange → SerialLink/TCPLink → STM32
  → receive → dispatch → 配置/启动/取样/停止
  → 周期 tick → pt_pid_step → 实际输出与样本
  → score → Search.tell → 最终三次验证 → 恢复原配置
```

| C 类型 | 内容和生命周期 |
|---|---|
| `pt_pid_config` | 单环类型、形式、flags、抗饱和和 16 个数值；更新前校验 |
| `pt_pid` | 配置副本、积分/微分/历史输出、fault 与 P/I/D；每个独立控制器一个实例 |
| `pt_config` | 一或两环，内环在 loop[0]，外环在 loop[1]，含 outer_divider |
| `pt_plan` | 波形/模式/周期/时长/目标/激励/限值/记录倍率 |
| `pt_sample` | 60 字节线上记录：索引、13 个浮点和 flags |
| `pt_parser` | 流式接收缓冲与最后字节时间 |
| `pt_runtime` | 整个实验上下文、两环 PID、队列、协议缓存、状态与回调；建议静态存储 |

所有 C 指针参数均要求指向合法、可访问、足够长度的对象，函数没有统一的空指针保护。`pt_runtime` 相关调用归同一任务上下文，ISR 仅排队。`user` 是应用自定义上下文指针，SDK 只透传给回调。

## 2. C 公开接口：PID 核心

声明见 [pt_sdk.h](../sdk/include/pt_sdk.h)，实现在 [pt_pid.c](../sdk/src/pt_pid.c)。这些函数不直接读传感器、发 UART、写执行器，也不依赖 HAL。

### `pt_pid_defaults(pt_pid_config *config) → void`

填充完整初始配置；覆盖传入对象原内容。默认为 PI、位置式、条件积分、增益零、输出 -1…1，flags 为 0。积分上限 10、滤波常数 0.01 s、变速积分宽度 1、反算增益 1、堵转误差和时间 1。调用方为应用初始化或 runtime 初始化；之后按设备改增益、范围、功能，不能将这些默认值视为电机参数。

### `pt_pid_validate(const pt_pid_config *config) → int`

检查枚举/flags、16 个数值有限性、输出上下界、非负限幅/死区/滤波/反算参数，以及正积分宽度/堵转参数。有效返回非零，无效返回 0；不修改配置。允许正负增益，负号由对象方向决定。验证不证明闭环稳定；未启用功能对应的字段也须满足基本数值规则。

### `pt_pid_init(pt_pid *pid, const pt_pid_config *config, float bias) → void`

清空历史状态，复制 config，将 bias 存为固定偏置，并初始化 output/accumulator。`pid` 为待初始化实例，`config` 为配置，bias 单位与输出相同。此函数不调用 validate，也不保证 bias 在限幅内；调用方先校验。原型 bias 与实验计划的开环 bias 不是同一机制，runtime 启动 PID 时传 0。重新调用会清除积分、fault 和历史值，不适合每个控制 tick 调用。

### `pt_pid_step(pt_pid *pid, float reference, float measurement, float dt) → float`

完成一次控制计算并更新实例：误差→P/I/D/FF→抗饱和→输出滤波→限幅→死区→堵转累计→保存历史。reference/measurement 同单位，dt 为正的秒数，返回 PID 输出而非已经发送到电机的结果。

输入、配置或中间计算出现非法值时置 `fault=PT_NONFINITE` 并返回之前输出；堵转达到阈值置 PT_BLOCKED。fault 可在返回后非零，应用应检查；它不会自动停机或自动清除。runtime 负责看 fault 并调用退出回调。第一次微分为零；位置/增量两种形式共享显式积分抗饱和策略。当前实现验证配置并计算所有分支，实际周期耗时需目标测量。

## 3. C 公开接口：帧编解码

实现在 [pt_protocol.c](../sdk/src/pt_protocol.c)。

| 原型 | 参数、返回值与用途 |
|---|---|
| `uint16_t pt_crc16(const uint8_t *data, size_t size)` | 对连续 size 字节计算 CRC16，初值 FFFF/多项式 A001；返回 CRC 数值，不加头尾，不跳过数据。输入 "123456789" 得 4B37。 |
| `uint16_t pt_frame_crc(const uint8_t *frame, size_t size)` | 输入完整帧及总长，遍历时跳过偏移 6/7，包含尾部，返回校验值；不会校验帧形状。由编码器与接收解析调用。 |
| `size_t pt_encode(uint16_t command, const uint8_t *body, size_t size, uint8_t *out, size_t capacity)` | command 是外帧命令，size 是 body 长度，out 是输出缓冲，capacity 为容量；写 AA55/长度/命令/CRC/body/55AA，成功返回总长=size+10；长度超过 65525、容量不足或非零 body 长度却空指针时返回 0。允许 body=out+8 的内部场景；其他任意重叠布局不应依赖。 |

CRC 写到线上时采用大端；普通 CRC 值的低/高字节顺序不要按其他 Modbus 协议惯例直接发送。编码函数本身按 capacity 检查，不强制默认 1024 上限；应用必须与对端帧能力一致。

## 4. C 公开接口：实验运行时

实现在 [pt_runtime.c](../sdk/src/pt_runtime.c)。只有它管理实验状态与协议，不能仅调用 PID 核心就期待网页可连接。

### `pt_runtime_init(pt_runtime *rt, uint32_t boot_id, pt_send_fn send, pt_exit_fn exit, void *user) → void`

清空 rt、设置本次启动标识、回调/上下文、默认单环和两个合法占位 PID，默认命令 PT_COMMAND、周期 0.001 s、硬输出 -1…1。开环默认不许可。boot_id 每次重新启动应变化。send 用于复制回复，exit 用于设备退出；exit 缺失时 START 会被拒绝。仅启动时调用，不在运行中重置对象。

### `pt_runtime_set_hardware(pt_runtime *rt, float period, float output_min, float output_max) → void`

在非 RUNNING 且参数有限、period>0、min<max 时设置基础周期与硬输出范围；不满足时静默不修改，没有成功布尔返回。它只描述已有硬件周期，不配置定时器，不自动重写当前 PID 范围。主机配置必须落在硬范围内。

### `pt_runtime_allow_open_loop(pt_runtime *rt, int allowed) → void`

非运行时将开环许可设为 allowed!=0；RUNNING 时忽略。由板级集成代码调用，网页不能凭确认字段绕过这个限制。它只改变能力，不发起实验。

### `pt_runtime_receive(pt_runtime *rt, const uint8_t *data, size_t size, uint32_t now_ms) → void`

向流式解析器提交 size 个新字节，now_ms 为当前毫秒时间。支持分片/粘包，错误头/长度/CRC/尾逐字节重新找帧，旧半帧超过 250 ms 后丢弃；命令匹配时调用内部 dispatch。完整有效命令可配置、启动、停止和调用发送回调，所以在拥有控制权的任务中处理，不在 ISR 与 tick 并发执行。传入无关命令帧不会变成整定指令。

### `pt_runtime_tick(pt_runtime *rt, float inner_measurement, float outer_measurement, uint32_t now_ms) → float`

按硬件真实固定周期调用一次，生成激励、运行相应 PID/串级、检查通讯超时/测量范围/输出/变化率/堵转/非有限数/记录队列，按 decimation 写样本。传感器值应采用物理量约定，now_ms 是单调毫秒计时。

非运行时直接返回上次 actuator；运行过程中退出也可能返回历史输出。**返回后 state==PT_RUNNING 才能把值写执行器。** 双环时内环试验仍检查外环范围；单环时也检查第二输入有限性，无外环传感器传 0。实验时间由 tick×dt 推算，不测量实际调度抖动。正常 duration 到达时走 abort(PT_OK)。

### `pt_runtime_abort(pt_runtime *rt, uint32_t reason) → void`

仅 RUNNING 时生效：存 reason，PT_OK 变 COMPLETE，其他原因变 ABORTED，再同步调用 exit(user,reason)。调用方可以是 tick、STOP 分派或板级故障协调代码。非运行时无动作；不自动清空未读样本、不自动恢复物理工作点、不自动写输出零。中断检测到故障时，应按应用并发契约将事件交给拥有 rt 的任务处理。

### `pt_runtime_state(const pt_runtime *rt) → uint32_t`

返回当前状态：PT_IDLE=0、PT_RUNNING=1、PT_COMPLETE=2、PT_ABORTED=3，无副作用。只表示该 rt 的软件状态，不代表电机驱动已接受指令。

### `pt_runtime_size(void) → size_t`

返回本编译配置的 sizeof(pt_runtime)。供主机 ctypes 分配不透明实例，或目标工程检查存储预算；不分配内存。ABI、指针宽度和缓冲宏会影响结果。

### 回调类型

| 回调 | 输入、调用点、要求 |
|---|---|
| `pt_send_fn(void *user, const uint8_t *data, size_t size)` | 回复已编码的完整帧。返回前复制到应用自己的稳定队列；不得保留临时指针供异步 DMA 使用；无返回值，发送错误需应用处理。 |
| `pt_exit_fn(void *user, uint32_t reason)` | 正常完成/停止/故障退出时调用，具体停机/保持/控制权移交由应用决定；不能由随后原控制逻辑覆盖动作。 |

## 5. C 内部辅助函数与板级示例

这些 static 函数不通过头文件公开：

| 文件/函数 | 输入→输出、作用 |
|---|---|
| pt_pid.c：`limit(x,lo,hi)` | float→float，将 x 截断到区间，供限幅/积分衰减使用 |
| pt_protocol.c：`update(crc,byte)` | 当前 CRC、一个字节→新的 uint16 CRC，执行 8 次位更新 |
| pt_runtime.c：`get32(p)` | 4 字节地址→大端 uint32；调用者保证长度 |
| pt_runtime.c：`put32(p,v)` | 写大端 uint32 到 4 字节地址，无返回值 |
| pt_runtime.c：`getf(p)` | 按大端取 32 位并 memcpy 为 float，不做数值范围验证 |
| pt_runtime.c：`putf(p,f)` | float 位模式转大端 4 字节，不采用本机结构体打包 |
| pt_runtime.c：`clamp(v,lo,hi)` | float→float，实验/外环目标限幅 |
| pt_runtime.c：`config_read(c,b)` | 168 字节负载→两环配置，随后由 dispatch 校验 |
| pt_runtime.c：`config_write(c,b)` | 配置→168 字节状态负载，供 HELLO/STATUS |
| pt_runtime.c：`plan_read(p,b)` | 108 字节负载→计划，随后 plan_valid 校验 |
| pt_runtime.c：`plan_valid(rt,p)` | 计划及设备配置→int，检查模式、周期、范围、频带、开环激励等，不能替代物理检查 |
| pt_runtime.c：`sample_write(s,b)` | 一个样本→60 字节大端记录，供 READ |
| pt_runtime.c：`dispatch(rt,frame,size,now)` | 完整匹配帧→命令执行及回调发送；检查会话/boot/请求/配置版本，缓存最后请求响应。READ 会消耗样本，重复相同请求回放缓存。 |

示例位于 [control_task.c](../sdk/examples/control_task.c)：

| 函数 | 作用、参数和返回值 |
|---|---|
| `send_copy(user,bytes,size)` | static 回调，无返回；user 当前未用，转发 board_tx_copy |
| `leave_test(user,reason)` | static 回调，无返回；user 当前未用，转发 board_leave_test |
| `tuning_init(unique_boot_id,actual_period,output_min,output_max)` | void，初始化静态 tuner 并设置周期/硬限制；不自动开启开环 |
| `tuning_control_tick(inner_measurement,outer_measurement,now_ms)` | void，每 tick 最多取四批 128 字节 RX，再 tick 并依据状态写输出/调用原控制；退出当 tick 不覆盖回调动作 |
| `board_tx_copy(bytes,size)` | extern 未实现；返回前复制 TX 数据，无返回值 |
| `board_rx_pop(bytes,capacity)` | extern 未实现；最多取 capacity 字节，返回 size_t 实际数量 |
| `board_apply_output(output)` | extern 未实现；实际驱动接口，无返回值，输出单位由应用定义 |
| `board_leave_test(reason)` | extern 未实现；设备退出/锁存策略，无返回值 |
| `board_normal_control()` | extern 未实现；正常控制，无返回值，必须服从退出锁存 |

## 6. Python 函数逐项参考

函数名含类名；缩进中的内部函数用完整作用域标识。各条附源码行链接。除非明确返回值，构造器/动作方法返回 None；验证失败一般 ValueError，通信失败为 DeviceError/OSError/TimeoutError，取消/端侧退出用 TrialAborted。dataclass 构造器由 Python 生成，不另编写虚构函数体。

### schema.py

文件：[源码](../src/pid_agent/workbench/schema.py)。

#### `finite`

```python
finite(value: object) -> float
```

输入任意对象，返回有限 float；拒绝 bool、非 int/float 和 NaN/Infinity，失败抛 ValueError。作为配置/搜索共同入口，不做 float32 量化。

[源码位置](../src/pid_agent/workbench/schema.py#L30)

#### `f32`

```python
f32(value: float) -> float
```

输入有限数字，按 IEEE-754 单精度打包再读回，返回设备实际精度的 float；超出范围转成 ValueError，供所有 PID 数值和搜索边界使用。

[源码位置](../src/pid_agent/workbench/schema.py#L36)

#### `PID.__post_init__`

```python
PID.__post_init__(self)
```

dataclass 构造后检查 kind/form/antiwindup/flags，将 16 个数值逐个量化并检查范围。修改冻结对象仅限这里的 object.__setattr__；失败 ValueError，返回 None。

[源码位置](../src/pid_agent/workbench/schema.py#L66)

#### `PID.available`

```python
PID.available(self) -> set[str]
```

无额外参数，返回当前类型/开关下可调字段 set。只包含数值候选，不包含输出边界和堵转保护；供 Space 和网页能力对应校验。

[源码位置](../src/pid_agent/workbench/schema.py#L81)

#### `PID.pack`

```python
PID.pack(self) -> bytes
```

返回单环 80 字节的大端 bytes（4 个 u32、16 个 f32），不含外帧或应用头；实例创建时已校验。

[源码位置](../src/pid_agent/workbench/schema.py#L101)

#### `PID.unpack`

```python
PID.unpack(cls, data: bytes) -> PID
```

输入恰好 80 字节，解码枚举和数值，返回新 PID 并执行构造校验；结构长度错误可抛 struct.error，未知枚举可能 IndexError，不应传任意原始串口块。

[源码位置](../src/pid_agent/workbench/schema.py#L106)

#### `Configuration.__post_init__`

```python
Configuration.__post_init__(self)
```

检查 loops 只含 1–2 个 PID，outer_divider 为 1–1000 的整数；失败 ValueError，返回 None。配置对象本身不可变。

[源码位置](../src/pid_agent/workbench/schema.py#L117)

#### `Configuration.pack`

```python
Configuration.pack(self) -> bytes
```

返回 168 字节完整配置；单环也追加合法占位外环 PID，保证线上格式固定。

[源码位置](../src/pid_agent/workbench/schema.py#L123)

#### `Configuration.unpack`

```python
Configuration.unpack(cls, data: bytes) -> Configuration
```

输入配置字节，读取环数/倍率及每环，返回 Configuration；非法环数 ValueError，不负责接收流找帧。

[源码位置](../src/pid_agent/workbench/schema.py#L128)

#### `Configuration.from_dict`

```python
Configuration.from_dict(cls, data: dict) -> Configuration
```

输入含 loops 和可选 outer_divider 的 dict，逐环构造 PID；拒绝未知根字段，返回配置对象。缺失/错误类型可能抛 KeyError/TypeError。

[源码位置](../src/pid_agent/workbench/schema.py#L135)

#### `Configuration.to_dict`

```python
Configuration.to_dict(self) -> dict
```

返回适合 JSON 的递归 dataclass 数据；不发设备消息，不保存文件。

[源码位置](../src/pid_agent/workbench/schema.py#L140)

#### `Configuration.value`

```python
Configuration.value(self, key: str) -> float
```

输入 inner.kp/outer.ki 等参数键，返回对应环数值；检查环名和数值字段，环不存在时不能直接索引。用于搜索编码和边界检查。

[源码位置](../src/pid_agent/workbench/schema.py#L143)

#### `Configuration.updated`

```python
Configuration.updated(self, values: dict[str, float]) -> Configuration
```

输入数值键→新值的 dict，返回替换所选字段后的新配置，原对象不变。只接受 PARAMETERS 中的参数名并触发 PID 重校验，用于保持固定结构/保护参数。

[源码位置](../src/pid_agent/workbench/schema.py#L149)

#### `Plan.__post_init__`

```python
Plan.__post_init__(self)
```

检查模式整数、倍率、超时、有限数、时长、测量/输出范围、开环幅值及每周期至少 10 个记录点的频率条件。失败 ValueError，不主动读取设备；还需 Device/端侧检查实际周期和能力。

[源码位置](../src/pid_agent/workbench/schema.py#L181)

#### `Plan.pack`

```python
Plan.pack(self) -> bytes
```

返回 108 字节计划（6 个 u32、21 个 f32），频率不足 8 项补零；y 范围在线上顺序是 min0/min1/max0/max1。

[源码位置](../src/pid_agent/workbench/schema.py#L209)

#### `Plan.from_dict`

```python
Plan.from_dict(cls, data: dict) -> Plan
```

复制 dict 并将 y_min/y_max/frequencies 转 tuple，返回经过校验的 Plan；不修改调用方原字典。

[源码位置](../src/pid_agent/workbench/schema.py#L217)

#### `schema`

```python
schema() -> dict
```

无参数，返回版本、枚举、功能位、字段标签/单位、默认配置和计划的 dict；供 HTTP session 和网页初始渲染。

[源码位置](../src/pid_agent/workbench/schema.py#L225)

### protocol.py

文件：[源码](../src/pid_agent/workbench/protocol.py)。

#### `crc16`

```python
crc16(data: bytes) -> int
```

输入连续 bytes，返回 FFFF/A001 的 CRC16 整数；不自动跳过字段。

[源码位置](../src/pid_agent/workbench/protocol.py#L17)

#### `encode`

```python
encode(body: bytes, command: int=COMMAND) -> bytes
```

输入应用负载 body、可选外帧 command，返回完整兼容帧 bytes。默认最大 1024 字节，过大/命令越界 ValueError；CRC 覆盖头/体/尾并跳过自身。

[源码位置](../src/pid_agent/workbench/protocol.py#L26)

#### `Parser.__init__`

```python
Parser.__init__(self)
```

初始化 bytearray 接收缓冲、上次接收时间、错误计数，无硬件访问。

[源码位置](../src/pid_agent/workbench/protocol.py#L41)

#### `Parser.feed`

```python
Parser.feed(self, data: bytes, now: float | None=None) -> list[Frame]
```

输入新字节和可选单调秒数 now，返回本次解析出的 Frame 列表。逐字节处理，半帧超 0.25 s 清空，错误头/长度/CRC/尾重新同步；坏帧不会变成有效消息，errors 记录部分错误。

[源码位置](../src/pid_agent/workbench/protocol.py#L46)

### device.py

文件：[源码](../src/pid_agent/workbench/device.py)。

#### `TrialAborted.__init__`

```python
TrialAborted.__init__(self, reason: str, samples: list[dict])
```

输入原因字符串和已采集样本，创建包含 reason/samples 的 DeviceError 子类；返回 None。用于保留异常前曲线，不等于完整轨迹。

[源码位置](../src/pid_agent/workbench/device.py#L24)

#### `TCPLink.__init__`

```python
TCPLink.__init__(self, host: str, port: int)
```

输入设备 host/port，建立客户端 socket，连接超时 2 s、读取超时 0.1 s，启用 TCP_NODELAY；失败抛 socket/OSError。simulated=False。

[源码位置](../src/pid_agent/workbench/device.py#L32)

#### `TCPLink.write`

```python
TCPLink.write(self, data: bytes)
```

输入 bytes，sendall 发完整字节流，返回 None；失败抛网络异常，不自行编码协议。

[源码位置](../src/pid_agent/workbench/device.py#L37)

#### `TCPLink.read`

```python
TCPLink.read(self) -> bytes
```

无额外参数，最多读 4096 字节；超时返回空 bytes，EOF 抛 ConnectionError，避免把断线当空闲。

[源码位置](../src/pid_agent/workbench/device.py#L40)

#### `TCPLink.close`

```python
TCPLink.close(self)
```

关闭 socket，返回 None；由 Device.close/服务断开调用。

[源码位置](../src/pid_agent/workbench/device.py#L49)

#### `SerialLink.__init__`

```python
SerialLink.__init__(self, port: str, baudrate: int)
```

输入系统串口名和波特率，用 pyserial 打开，读取超时 0.05 s、写超时 0.2 s；返回 None，串口错误向上抛。波特率保留供带宽预检。

[源码位置](../src/pid_agent/workbench/device.py#L56)

#### `SerialLink.write`

```python
SerialLink.write(self, data: bytes)
```

输入 bytes，调用串口写并验证返回长度，短写抛 ConnectionError；返回 None。

[源码位置](../src/pid_agent/workbench/device.py#L61)

#### `SerialLink.read`

```python
SerialLink.read(self) -> bytes
```

读取当前可用数量，至少尝试 1 字节、最多 4096，返回 bytes；受串口读超时约束，不保证一次就是一帧。

[源码位置](../src/pid_agent/workbench/device.py#L65)

#### `SerialLink.close`

```python
SerialLink.close(self)
```

关闭 pyserial 设备，返回 None。

[源码位置](../src/pid_agent/workbench/device.py#L68)

#### `native_library`

```python
native_library()
```

无参数，加载 native/pt_sdk.dll 或 libpt_sdk.so，设置 ctypes 签名，返回 (lib,send_callback_type,exit_callback_type)。缺失库抛 DeviceError，架构/加载错误可能 OSError；只供主机仿真。

[源码位置](../src/pid_agent/workbench/device.py#L72)

#### `NativeLink.__init__`

```python
NativeLink.__init__(self, gain=1.0, tau=0.6, delay=0.03, noise=0.0, dt=0.005, seed=1)
```

输入 gain/tau/delay/noise/dt/seed，校验并分配真实 C SDK 上下文，绑定回调，建立速度惯性与积分位置数学对象，返回 None。许可开环、硬范围 ±100 仅属于仿真；不代表真实板默认。

[源码位置](../src/pid_agent/workbench/device.py#L99)

#### `NativeLink._exit`

```python
NativeLink._exit(self, _, reason)
```

C 回调输入上下文和原因，将模拟输出设零，返回 None；当前不使用上下文/原因区别，不能把此动作照搬任意物理机构。

[源码位置](../src/pid_agent/workbench/device.py#L118)

#### `NativeLink.write`

```python
NativeLink.write(self, data: bytes)
```

输入协议 bytes，在当前模拟毫秒时间调用 C receive；关闭后抛 ConnectionError，返回 None。

[源码位置](../src/pid_agent/workbench/device.py#L121)

#### `NativeLink.advance`

```python
NativeLink.advance(self, ticks: int=4)
```

输入要推进的 tick 数，默认 4；调用 C tick、按运行状态取输出、更新延迟队列/速度/位置/虚拟时间，可叠加测量噪声。返回 None，不按真实墙钟等待。

[源码位置](../src/pid_agent/workbench/device.py#L126)

#### `NativeLink.read`

```python
NativeLink.read(self) -> bytes
```

返回一个待回复帧并推进 4 个模拟 tick；无回复也推进时间并返回空 bytes。对应 Device 的读取循环，不启动额外仿真线程。

[源码位置](../src/pid_agent/workbench/device.py#L137)

#### `NativeLink.prepare`

```python
NativeLink.prepare(self)
```

只在非 RUNNING 时将对象速度/位置/输出及延迟历史重置；运行中 DeviceError。不重置整个协议上下文，不提供真实对象回零。

[源码位置](../src/pid_agent/workbench/device.py#L146)

#### `NativeLink.close`

```python
NativeLink.close(self)
```

设置关闭标志，返回 None；Device.close 会先请求 STOP。

[源码位置](../src/pid_agent/workbench/device.py#L152)

#### `Device.__init__`

```python
Device.__init__(self, link, command=COMMAND)
```

输入实现 write/read/close 的 link 及命令号，初始化会话/请求/版本/试验、解析器和 RLock，立即 HELLO 读取实际配置。成功返回新设备，不能把构造当纯本地赋值。

[源码位置](../src/pid_agent/workbench/device.py#L167)

#### `Device.exchange`

```python
Device.exchange(self, op: int, data=b'', revision=None, trial=None) -> tuple[tuple, bytes]
```

输入 opcode、可选负载及 revision/trial，编码应用头，发送并等待匹配响应，返回 (header_tuple,payload)。请求 ID 递增，最多两次相同字节发送，每次等待约 0.35 s；校验命令/请求/会话/boot。设备拒绝抛 DeviceError，未收到 ACK 抛 TimeoutError；延迟受底层读写超时影响。

[源码位置](../src/pid_agent/workbench/device.py#L177)

#### `Device.status`

```python
Device.status(self, hello=False) -> dict
```

hello=True 时 HELLO 并采用设备分配会话，否则 STATUS；解码 204 字节状态与配置，更新 self.info/configuration，返回 info dict。错误长度 DeviceError。

[源码位置](../src/pid_agent/workbench/device.py#L208)

#### `Device.configure`

```python
Device.configure(self, config: Configuration) -> Configuration
```

输入合法 Configuration，以 revision+1 提交，再 STATUS 回读比对，返回实际配置；不一致 DeviceError。只在设备允许的非运行状态使用，不写 Flash。

[源码位置](../src/pid_agent/workbench/device.py#L219)

#### `Device.stop`

```python
Device.stop(self)
```

发 PT_STOP，返回 None；设备运行时退出，非运行时不会制造额外实验。通信错误仍可能向上抛。

[源码位置](../src/pid_agent/workbench/device.py#L226)

#### `Device.run`

```python
Device.run(self, config: Configuration, plan: Plan, cancel: threading.Event, on_samples=lambda rows: None, open_loop_confirmed=False) -> Trace
```

输入 cfg/plan/cancel/on_samples/开环确认，先检取消/周期/能力/串口带宽，CLEAR→仿真重置(如有)→配置回读→START→批量 READ。完整时返回 Trace，分块通过回调传出。取消/端侧退出抛 TrialAborted 并带已有样本；异常尝试 STOP；自身不恢复任务前配置，恢复由 Runner 做。

[源码位置](../src/pid_agent/workbench/device.py#L229)

#### `Device.close`

```python
Device.close(self)
```

先尝试 STOP，finally 关闭 link，返回 None；停止失败也关闭传输，端侧超时策略仍由固件实现。

[源码位置](../src/pid_agent/workbench/device.py#L292)

### analysis.py

文件：[源码](../src/pid_agent/workbench/analysis.py)。

#### `Objective.__post_init__`

```python
Objective.__post_init__(self)
```

检查性能权重/约束/窗口均为有限非负，至少一项权重大于零，调节带/停留时间为正，饱和上限不大于 1；无业务返回，非法 ValueError。

[源码位置](../src/pid_agent/workbench/analysis.py#L24)

#### `arrays`

```python
arrays(trace: Trace) -> dict[str, np.ndarray]
```

输入 Trace，返回按样本字段组织的 numpy 数组 dict；要求至少 20 点、有限数、索引间距等于倍率、时间递增且间隔匹配、起止完整。非法 ValueError，不插值补点。

[源码位置](../src/pid_agent/workbench/analysis.py#L33)

#### `score`

```python
score(trace: Trace, objective: Objective) -> dict
```

输入完整闭环阶跃 Trace 与 Objective，返回 cost/score/constraint/feasible、ITAE/IAE/超调/稳态/变化/饱和/调节时间等 dict。拒绝正弦、开环、零阶跃、初始状态不稳、采样不足、非有限指标；不调用设备，不给坏数据编造罚分。

[源码位置](../src/pid_agent/workbench/analysis.py#L49)

#### `frequency_response`

```python
frequency_response(trace: Trace, frequency: float) -> dict
```

输入正弦 Trace 和其中一个 frequency，返回复频响的实/虚部、幅值、相位、SNR、窗口差异、工况与来源。调用内部 estimate 计算全窗和两个分窗，拒绝饱和/不足周期/低质量；不是稳定性证明或统计置信区间。

[源码位置](../src/pid_agent/workbench/analysis.py#L102)

#### `frequency_response.estimate`

```python
frequency_response.estimate(mask)
```

内部输入采样 mask，构造直流/趋势/已知正余弦矩阵，最小二乘求 input/measurement/perturbation 相量，返回 (复数 G,SNR)。激励/输入过小、秩不足 ValueError；仅由 frequency_response 使用。

[源码位置](../src/pid_agent/workbench/analysis.py#L119)

### design.py

文件：[源码](../src/pid_agent/workbench/design.py)。

#### `PlantModel.__post_init__`

```python
PlantModel.__post_init__(self)
```

检查 gain/tau/delay/damping 有限、模型族合法、tau/damping 正、delay 非负；无业务返回，非法 ValueError。仅检查参数，不证明给定模型对应真实对象。

[源码位置](../src/pid_agent/workbench/design.py#L21)

#### `PlantModel.response`

```python
PlantModel.response(self, frequencies)
```

输入 Hz 频率数组，返回模型复频响 numpy 数组。支持一阶延迟、二阶延迟、积分惯性延迟；积分模型不应传零频率。

[源码位置](../src/pid_agent/workbench/design.py#L27)

#### `PlantModel.samples`

```python
PlantModel.samples(self, frequencies)
```

输入频率序列，调用 response，返回 frequency/real/imag/source=model 字典列表，供频域设计。

[源码位置](../src/pid_agent/workbench/design.py#L37)

#### `pid_response`

```python
pid_response(pid: PID, frequencies, dt: float)
```

输入 PID、Hz 频率和该控制环周期，返回 (Cr,Cy) 复数组，对应 u=Cr*r-Cy*y。包含离散积分/微分、测量比例/微分、滤波、测量前馈；不模拟饱和/死区/非线性抗饱和。

[源码位置](../src/pid_agent/workbench/design.py#L43)

#### `frequency_seeds`

```python
frequency_seeds(space: Space, data: list[dict], dt: float, active_loop=0, phase_margin=60.0, seed=1) -> list[dict]
```

输入 Space、至少 4 个有效频点、基本 dt、被调环、目标裕度/种子，返回候选设计 dict 列表。检查频带，多个目标交越点做有界 least_squares，保留所选数值并按残差去重。裕度估计不等于稳定性认证；错误输入 ValueError。

[源码位置](../src/pid_agent/workbench/design.py#L69)

#### `frequency_seeds.residual`

```python
frequency_seeds.residual(x)
```

内部输入归一化候选 x，Space.decode 后计算跟踪频响误差、交越增益/相位误差、灵敏度惩罚，返回最小二乘残差向量；接近奇异分母时返回大残差，不发真实试验。

[源码位置](../src/pid_agent/workbench/design.py#L95)

#### `simc_seed`

```python
simc_seed(space: Space, model: PlantModel, response_time: float, active_loop=0)
```

输入 Space、PlantModel、目标响应时间和被调环；仅 FOPDT、非零增益、PI 且 kp/ki 都被选时生成 SIMC 配置。范围不符或不适用返回 None，否则返回新 Configuration，不修改固定参数。

[源码位置](../src/pid_agent/workbench/design.py#L128)

### search.py

文件：[源码](../src/pid_agent/workbench/search.py)。

#### `Bound.__post_init__`

```python
Bound.__post_init__(self)
```

将上下界量化 float32，检查下界小于上界，尺度 linear/log 合法，对数下界正；非法 ValueError，返回 None。

[源码位置](../src/pid_agent/workbench/search.py#L20)

#### `Space.__init__`

```python
Space.__init__(self, base: Configuration, bounds: list[Bound])
```

输入基准配置和 Bound 列表，检查 1–16 个不重复维度、有效字段、当前值在范围内、冗余 Kd/FFJ 组合；返回 None。固定结构和未选参数保存在 base。

[源码位置](../src/pid_agent/workbench/search.py#L29)

#### `Space.dimension`

```python
Space.dimension(self)
```

只读属性，返回所选参数数量 int，不是可调用的重新配置接口。

[源码位置](../src/pid_agent/workbench/search.py#L49)

#### `Space.encode`

```python
Space.encode(self, cfg: Configuration) -> np.ndarray
```

输入 Configuration，取选定数值按线性/对数边界映射为归一化 numpy 向量；要求配置和范围一致。

[源码位置](../src/pid_agent/workbench/search.py#L52)

#### `Space.decode`

```python
Space.decode(self, x) -> Configuration
```

输入每维归一化 x，限制到 [0,1]，反变换并量化实际 float32，返回只改所选数值的新 Configuration；不修改 base，不自动选择功能。

[源码位置](../src/pid_agent/workbench/search.py#L61)

#### `Space.key`

```python
Space.key(self, cfg: Configuration) -> bytes
```

输入配置，返回完整 pack bytes 作为去重键；包含结构/固定值，精度等于固件值，不是浮点字符串比较。

[源码位置](../src/pid_agent/workbench/search.py#L72)

#### `GaussianProcess.fit`

```python
GaussianProcess.fit(self, x, y)
```

输入二维 x 和一维 y，用 Matérn5/2 ARD 核及观测噪声做小样本拟合；归一化 y、L-BFGS-B 拟合超参、Cholesky 存储，返回 self。数值分解失败可抛异常，无设备调用。

[源码位置](../src/pid_agent/workbench/search.py#L79)

#### `GaussianProcess.fit.likelihood`

```python
GaussianProcess.fit.likelihood(log_parameters)
```

内部输入超参数对数向量，返回目标似然标量；分解失败返回 1e20，供 minimize 调整长度尺度/噪声。

[源码位置](../src/pid_agent/workbench/search.py#L87)

#### `GaussianProcess.kernel`

```python
GaussianProcess.kernel(a, b, length)
```

输入两组归一化点及每维 length，返回核矩阵；staticmethod，无实例状态修改。

[源码位置](../src/pid_agent/workbench/search.py#L108)

#### `GaussianProcess.predict`

```python
GaussianProcess.predict(self, points, covariance=False)
```

输入候选 points 和 covariance 开关，返回 (均值,方差向量) 或 (均值,完整协方差矩阵)。须先 fit；对应潜在函数不确定性，不是控制稳定保证。

[源码位置](../src/pid_agent/workbench/search.py#L113)

#### `Search.__init__`

```python
Search.__init__(self, space: Space, method='bayesian', seed=1)
```

输入 Space、method=bayesian/pattern/cmaes、随机种子，建立观测/拒绝列表、局部范围、GP/CMA 所需状态；未知方法 ValueError，不自动执行实验。

[源码位置](../src/pid_agent/workbench/search.py#L131)

#### `Search._population_update`

```python
Search._population_update(self)
```

内部使用已完成的一代 CMA 观测，按可行性排序，更新均值、演化路径、协方差与 sigma 并清空种群；返回 None，只由 tell 达到代大小时调用。

[源码位置](../src/pid_agent/workbench/search.py#L160)

#### `Search.tell`

```python
Search.tell(self, cfg: Configuration, cost=None, constraint=None, status='complete')
```

输入实际配置、完整 cost/constraint 或失败 status。complete 记录有限观测；aborted 进入局部拒绝列表；invalid 不送性能/约束模型。ask 生成的候选额外更新 CMA 或贝叶斯范围；调用方不能把通信故障误标 aborted。

[源码位置](../src/pid_agent/workbench/search.py#L179)

#### `Search.incumbent`

```python
Search.incumbent(self) -> Configuration
```

无额外输入，返回当前推荐配置；同参数按平均代价/最坏约束合并，可行优先，再看代价；没有观测返回 base。这是训练期候选，未代表最终复验通过。

[源码位置](../src/pid_agent/workbench/search.py#L208)

#### `Search.ask`

```python
Search.ask(self) -> Configuration
```

返回下一 Configuration；在有界空间生成候选，排除已测试和接近拒绝点的配置，按方法选取并登记 pending。贝叶斯数据足够后拟合两 GP，使用噪声 EI×可行概率；没有新候选 RuntimeError。不调用设备。

[源码位置](../src/pid_agent/workbench/search.py#L217)

### runner.py

文件：[源码](../src/pid_agent/workbench/runner.py)。

#### `Runner.__init__`

```python
Runner.__init__(self, device: Device, request: dict, cancel: threading.Event, emit, directory: Path)
```

输入 Device、请求 dict、取消 Event、事件回调 emit、结果目录，解析并冻结结构/计划/评分/搜索/预算，校验开环/活跃环/选参，记下设备原配置。无实际 START，不自动创建试验目录。

[源码位置](../src/pid_agent/workbench/runner.py#L23)

#### `Runner.check_budget`

```python
Runner.check_budget(self, plan: Plan)
```

输入下一 Plan，检查取消、次数、累计计划时长和墙钟；取消抛 TrialAborted(cancelled)，超限 BudgetExhausted，成功返回 None。依赖 run 初始化 started。

[源码位置](../src/pid_agent/workbench/runner.py#L81)

#### `Runner.trial`

```python
Runner.trial(self, cfg: Configuration, plan: Plan, purpose: str, evaluate=True)
```

输入配置/计划/purpose、是否评分，执行一次设备实验，记录状态并发 start/samples/end 事件，finally 保存可用轨迹和记录；正常返回 (record,trace)。基础设施异常终止、不伪造分数；连续无效数据限制后续运行。

[源码位置](../src/pid_agent/workbench/runner.py#L87)

#### `Runner.evaluate`

```python
Runner.evaluate(self, cfg: Configuration, plans: list[Plan], purpose: str)
```

输入配置、多个工况计划和用途，逐工况 trial，完整时返回 (聚合指标,complete)，cost=0.7均值+0.3最坏值、constraint 取最大；无指标时返回 (None,失败状态)。不把不同工况原始采样拼成一条曲线。

[源码位置](../src/pid_agent/workbench/runner.py#L127)

#### `Runner.identify`

```python
Runner.identify(self)
```

按请求的 4–12 频率生成逐频正弦计划，每点 settle+8/f，调用 trial(不做阶跃评分) 和 frequency_response；累积并返回 frequency_data，发 frequency 事件。中途质量/完成检查失败即异常，不凭坏点生成初值。

[源码位置](../src/pid_agent/workbench/runner.py#L139)

#### `Runner.run`

```python
Runner.run(self) -> dict
```

创建目录并组织 experiment/identify/tune，按起步模式产生初值/辨识、两次基准、预算内 ask/evaluate/tell、三次最终验证。返回含 status/records/best/验证/来源的报告；finally 尝试 STOP+恢复原配置并写 report.json。报告 validated 与 restored_original 是两个不同维度。

[源码位置](../src/pid_agent/workbench/runner.py#L160)

### service.py

文件：[源码](../src/pid_agent/workbench/service.py)。

#### `Workbench.__init__`

```python
Workbench.__init__(self, results: Path)
```

输入结果目录 Path，创建 RLock、取消 Event、状态 dict 和 12000 点显示 deque；不连接硬件，不启动后台任务。

[源码位置](../src/pid_agent/workbench/service.py#L19)

#### `Workbench.idle`

```python
Workbench.idle(self)
```

无额外输入，若当前 worker 活着则 RuntimeError，否则返回 None；用于串行化连接/应用/导出等动作。

[源码位置](../src/pid_agent/workbench/service.py#L30)

#### `Workbench.connect`

```python
Workbench.connect(self, payload: dict)
```

输入 transport 配置，在 idle 下创建 NativeLink/SerialLink/TCPLink 和 Device 完成握手，新连接成功后关闭旧连接，更新状态并返回 info。失败清理新 link；不发 START。

[源码位置](../src/pid_agent/workbench/service.py#L34)

#### `Workbench.disconnect`

```python
Workbench.disconnect(self, _=None)
```

占位参数当前未用，要求 idle，关闭设备并清状态，返回 disconnected=true；运行期间应先 stop 并等待退出。

[源码位置](../src/pid_agent/workbench/service.py#L66)

#### `Workbench.start`

```python
Workbench.start(self, payload: dict)
```

输入任务请求，idle/已连接检查，真实设备要求 limits_confirmed=True，构造 Runner 并创建取消 Event/job_id/线程，返回 job_id/status。实际工作由 work 执行；构造校验错误不会启动线程。

[源码位置](../src/pid_agent/workbench/service.py#L77)

#### `Workbench.start.work`

```python
Workbench.start.work()
```

内部线程函数，执行 runner.run，将最终报告和状态写回；捕获未处理异常存 failed/error，finally 清 active。无业务返回，线程锁只保护服务状态不包住所有通信。

[源码位置](../src/pid_agent/workbench/service.py#L92)

#### `Workbench.emit`

```python
Workbench.emit(self, kind, data)
```

输入事件 kind/data，将样本加全局 cursor/trial 后存显示 deque，或更新当前试验/记录/频响/候选/设计；在锁内执行，返回 None，不重新评分。

[源码位置](../src/pid_agent/workbench/service.py#L109)

#### `Workbench.stop`

```python
Workbench.stop(self, _=None)
```

占位输入未用，在同一锁内 set 当前取消 Event，运行时标 stopping，返回 stop_requested=true。不是等待停止完成的阻塞接口，随后看 snapshot.active。

[源码位置](../src/pid_agent/workbench/service.py#L126)

#### `Workbench.snapshot`

```python
Workbench.snapshot(self, after=0)
```

输入显示游标 after，返回状态深复制和最多 2000 条后续显示样本、新游标、显示丢点提示。原始数据在磁盘，显示缓存丢点不自动使原始评分丢点。

[源码位置](../src/pid_agent/workbench/service.py#L134)

#### `Workbench.apply`

```python
Workbench.apply(self, payload: dict)
```

输入 configuration dict，idle/已连接后构造配置并由 Device.configure 回读，更新状态；返回配置和 saved_to_flash=false，不启动实验。

[源码位置](../src/pid_agent/workbench/service.py#L143)

#### `Workbench.export`

```python
Workbench.export(self, kind: str)
```

输入 json/csv/c，要求 idle 和已有报告，返回 (MIME,下载文件名,bytes)。JSON 为报告，CSV 读取 gzip 逐次曲线，C 仅对 validated/best 生成完整头文件；不支持类型或无验证 ValueError。

[源码位置](../src/pid_agent/workbench/service.py#L153)

#### `Workbench.ports`

```python
Workbench.ports()
```

staticmethod，无参数，返回系统串口 port/description 列表，不会打开这些串口。

[源码位置](../src/pid_agent/workbench/service.py#L194)

#### `Workbench.close`

```python
Workbench.close(self)
```

set 取消，最多等待 worker 4 s，再尝试关闭设备；返回 None，部分关闭错误被处理。正常服务退出调用，不代表强杀进程时能执行。

[源码位置](../src/pid_agent/workbench/service.py#L198)

### server.py

文件：[源码](../src/pid_agent/workbench/server.py)。

#### `create_server`

```python
create_server(service: Workbench, host='127.0.0.1', port=8766, allowed_origins=())
```

输入 Workbench、回环 host/port、额外允许 origin，返回绑定好的 ThreadingHTTPServer；生成本进程 token 并构造 Handler。不会自己 serve_forever；端口占用 OSError，非法 host/origin ValueError。

[源码位置](../src/pid_agent/workbench/server.py#L13)

#### `create_server.Handler.allowed`

```python
create_server.Handler.allowed(self)
```

内部检查 Host 是否为当前回环端口，Origin 是否缺省/同源/明确允许，返回 bool；不查硬件。

[源码位置](../src/pid_agent/workbench/server.py#L25)

#### `create_server.Handler.send_content_headers`

```python
create_server.Handler.send_content_headers(self, status, content_type, size)
```

内部输入 HTTP 状态、内容类型、字节长度，写无缓存/nosniff 和适用 CORS 头并结束响应头，返回 None。

[源码位置](../src/pid_agent/workbench/server.py#L32)

#### `create_server.Handler.respond`

```python
create_server.Handler.respond(self, status, payload)
```

内部输入状态码和 JSON 数据，序列化 UTF-8、禁止 NaN，调用发送头并写响应体，返回 None。

[源码位置](../src/pid_agent/workbench/server.py#L44)

#### `create_server.Handler.do_OPTIONS`

```python
create_server.Handler.do_OPTIONS(self)
```

内部处理浏览器预检，允许时返回 204 及方法/头部许可，未允许 403；无 Python 业务返回。

[源码位置](../src/pid_agent/workbench/server.py#L49)

#### `create_server.Handler.do_GET`

```python
create_server.Handler.do_GET(self)
```

内部分派 session/state/ports/export 和精确允许的静态文件，错误转成 400/409 或 404；不提供任意路径文件浏览。

[源码位置](../src/pid_agent/workbench/server.py#L60)

#### `create_server.Handler.do_POST`

```python
create_server.Handler.do_POST(self)
```

内部校验 Host/Origin/token、JSON 类型与 1 MiB 上限，再调用 connect/disconnect/start/stop/apply；输入错误 400、会话错误 403、类型错误 415、状态错误 409。

[源码位置](../src/pid_agent/workbench/server.py#L97)

#### `create_server.Handler.do_POST.reject_constant`

```python
create_server.Handler.do_POST.reject_constant(value)
```

内部 json.loads 回调，输入 NaN/Infinity 名称就抛 ValueError，使非标准数值无法进入服务请求。

[源码位置](../src/pid_agent/workbench/server.py#L113)

#### `create_server.Handler.log_message`

```python
create_server.Handler.log_message(self, *_)
```

覆写默认访问日志输出，当前实现 pass，返回 None；不是独立日志系统。

[源码位置](../src/pid_agent/workbench/server.py#L124)

#### `serve`

```python
serve(host='127.0.0.1', port=8766, results=None, allowed_origins=(), open_browser=False)
```

输入服务地址、结果目录、允许来源、是否开浏览器，构造 Workbench/HTTP 服务并阻塞 serve_forever；Ctrl+C/finally 关闭服务/设备。模块入口调用，正常退出返回 None。

[源码位置](../src/pid_agent/workbench/server.py#L130)


## 7. 网页 JavaScript 函数

文件：[app.js](../src/pid_agent/workbench/web/app.js)。这些函数通过 DOM、全局编辑配置与 HTTP 协同；不直接操作串口或执行 PID。异步 api/session/poll/ports 返回 Promise，错误由 safe 或轮询捕获显示。

| 函数 | 参数→返回；作用 |
|---|---|
| `$(id)` | DOM id→元素，简化查找 |
| `clone(value)` | JSON 可序列化值→深复制值，不用于函数/循环引用 |
| `number(id)` | 数值输入 id→有限 number，空或非法时报错 |
| `fieldNumber(element)` | input 元素→有限 number，用于表格数值/边界 |
| `fmt(n,digits=4)` | 数字→有效位格式字符串；空值显示横线 |
| `message(text,error=false)` | 更新消息栏文本、隐藏状态与错误颜色，返回 undefined |
| `safe(action)` | 函数→异步包装函数，捕获异常并显示 message；用于按钮回调 |
| `base()` | 无参数→清理尾斜杠的本地服务地址 |
| `api(path,body)` | 路径/可选 JSON→Promise 数据；有 body 时 POST 并携带 token，HTTP 非成功抛错 |
| `session()` | 无参数→Promise；读取 token/schema，首次初始化编辑器，不连接设备 |
| `activeParameters(pid)` | PID 对象→Set，有效可调字段；配合后端 PID.available |
| `input(value,type='number')` | 初值与类型→新 input 元素；数值默认任意步长 |
| `cell(row,value)` | tr/文本或节点→新 td，并加入行；文本使用 textContent |
| `capture()` | 将当前环 DOM 写回编辑配置和 bounds；不是设备写入 |
| `renderController()` | 按当前编辑环重建类型、开关、参数表与保护字段 |
| `getConfiguration()` | 捕获编辑→只包含勾选环数量的配置副本 |
| `loadConfiguration(cfg)` | 配置→重设编辑器/环选择/搜索边界，再渲染 |
| `cascadeChanged()` | 按双环复选框启用/关闭外环编辑与试验选择 |
| `plan()` | 读取试验表单→计划对象，基础字段来自 schema 默认值 |
| `request(action)` | experiment/identify/tune→完整请求：结构、数值勾选、计划、验证、模型、目标和确认字段 |
| `updateControls(state)` | 根据已连接/active/report 切换按钮、设备周期、确认项与状态徽标 |
| `renderResults(state)` | 更新最近评分、历史表、频响与最终报告；不重新计算原始性能分数 |
| `draw()` | 依据当前 trace 和屏幕像素比例绘制两幅曲线，空记录显示提示 |
| `draw.plot(keys,colors,top,height)` | 内部：指定字段/颜色/区域→画刻度和折线；为显示做抽点，不改变磁盘数据 |
| `draw.plot.x(t)` | 内部箭头函数：时间→横坐标 |
| `draw.plot.y(v)` | 内部箭头函数：数值→纵坐标 |
| `poll()` | 每 500 ms 轮询增量状态；防止自身并发，更新游标/曲线，保留最多 12000 显示点 |
| `ports()` | 读取 /api/ports 并填充串口选择器 |
| `modeChanged()` | 根据起步模式显示模型设置/开环确认区 |

### 匿名事件回调

| 触发项 | 实际调用 |
|---|---|
| 初始页面加载 | session→poll→设置轮询，初始化评分权重；不发 START |
| transport 改变 | 显示串口或 TCP 输入 |
| connect | session→/api/connect→loadConfiguration→poll |
| disconnect | /api/disconnect→poll |
| load-device | 读取 lastState.device.configuration 到编辑器，不额外查询设备 |
| apply | getConfiguration→/api/apply→poll |
| run-tuning / run-experiment / run-identification | request 对应 action→/api/start→清显示缓存→poll |
| stop | /api/stop→poll，等待后台实际退出 |
| apply-best | /api/apply 使用 report.best，随后载入编辑器 |
| 三个 export | 获取导出响应→Blob 临时 URL→触发下载→释放 URL |
| edit-loop / cascade / kind / form / antiwindup / feature | capture 后更新编辑环/能力并重渲染，不自动写设备 |
| resize / nav 点击 | 重绘曲线/更新导航样式 |

其他匿名 map/filter/reduce/sort 回调用于构建界面/格式化数据；其外部可见行为由上述命名函数说明。

## 8. 构建与启动入口

| 文件/函数 | 作用、参数、返回与约束 |
|---|---|
| `tools/build_workbench.py::build()` | 无参数→Path；查找 GCC/Clang，将三个 SDK 源文件 C99 严格编译成 native/pt_sdk.dll 或 libpt_sdk.so，覆盖对应输出。相对路径避免 MinGW 的 Unicode 绝对路径问题；失败抛异常，不烧录硬件。 |
| `workbench/__main__.py` | 模块入口无自定义函数；argparse 读取 host/port/results/allow-origin/open-browser，再调用 serve。 |
| `start_workbench.ps1` | 参数 Port=8766、NoBrowser；进入工程根，设置 Python 环境，检查依赖/仿真库，调用模块；缺库时尝试 build。 |
| `open_workbench.cmd` | 进入脚本目录，调用 PowerShell 启动器并转交参数；异常时暂停便于查看。 |

## 9. 测试函数索引

测试代码属于验证工具，不是对外控制接口；unittest 自动调用 setUp→test_*→tearDown。测试断言异常或不符合预期时该用例失败，没有业务返回值。32 个 test 方法的作用见下表，内部模拟传输仅连接内置 C 库/回环，不能拿测试桩作为实际 UART 实现。

| 函数 | 作用 |
|---|---|
| [`test_directory`](../tests/test_workbench.py#L28) | 创建并保留工作区唯一测试目录，contextmanager 提供路径，不写系统临时目录 |
| [`request`](../tests/test_workbench.py#L36) | 构造完整仿真整定请求，参数 changes 覆盖可选项，返回 dict |
| [`ProtocolTests.test_crc_vector_and_c_encoder`](../tests/test_workbench.py#L43) | 已知 CRC 和多尺寸 C/Python 编码一致 |
| [`ProtocolTests.test_fragment_corruption_resynchronization`](../tests/test_workbench.py#L54) | 碎片、坏帧、错误长度后的重新同步 |
| [`ProtocolTests.test_partial_expiry`](../tests/test_workbench.py#L62) | 过期半帧被清除并能接收新帧 |
| [`ProtocolTests.test_replayed_configuration_ack_is_idempotent`](../tests/test_workbench.py#L66) | 相同配置请求重放不重复更新版本 |
| [`ProtocolTests.test_stale_session_and_boot_rejected`](../tests/test_workbench.py#L73) | 旧 session 与错误 boot 被拒绝 |
| [`DeviceTests.setUp`](../tests/test_workbench.py#L83) | 每次设备测试前创建 Device(NativeLink()) |
| [`DeviceTests.tearDown`](../tests/test_workbench.py#L84) | 每次设备测试后关闭设备 |
| [`DeviceTests.test_step_and_scoring`](../tests/test_workbench.py#L86) | 同 C 核心完整阶跃与有限评分 |
| [`DeviceTests.test_cancel_before_start_sends_no_commands`](../tests/test_workbench.py#L93) | 开始前已取消时不发设备命令 |
| [`DeviceTests.test_open_requires_confirmation`](../tests/test_workbench.py#L98) | 未确认的开环请求被拒绝 |
| [`DeviceTests.test_physical_limit_abort`](../tests/test_workbench.py#L101) | 内环测量越限明确退出 |
| [`DeviceTests.test_outer_range_checked_during_inner_test`](../tests/test_workbench.py#L106) | 内环试验也检查双环外环范围 |
| [`DeviceTests.test_watchdog_and_overflow`](../tests/test_workbench.py#L112) | 端侧通讯超时与样本缓冲溢出 |
| [`DeviceTests.test_short_scoring_window_rejected`](../tests/test_workbench.py#L122) | 阶跃后窗口不足被拒绝 |
| [`DeviceTests.test_missing_samples_rejected`](../tests/test_workbench.py#L126) | 缺点轨迹不能评分 |
| [`DeviceTests.test_negative_step`](../tests/test_workbench.py#L130) | 负阶跃得到正确方向的有限指标 |
| [`DeviceTests.test_open_frequency_response_matches_plant`](../tests/test_workbench.py#L134) | 开环估计频响与已知对象误差受限 |
| [`DeviceTests.test_outer_open_keeps_inner_closed`](../tests/test_workbench.py#L142) | 旁路外环时内环仍实际控制 |
| [`SearchTests.setUp`](../tests/test_workbench.py#L152) | 创建默认 PI 的 kp/ki 搜索空间 |
| [`SearchTests.test_lock_and_float32`](../tests/test_workbench.py#L153) | 未选字段不变、量化后打包回读一致 |
| [`SearchTests.test_inactive_or_protection_parameter_rejected`](../tests/test_workbench.py#L159) | 无效或保护字段不能当搜索维度 |
| [`SearchTests.test_duplicate_effect_rejected`](../tests/test_workbench.py#L163) | 冗余 Kd/FFJ 同选被拒绝 |
| [`SearchTests.test_noisy_gp_covariance`](../tests/test_workbench.py#L167) | 含重复噪声观测时 GP 有限且协方差合理 |
| [`SearchTests.test_all_searchers_finite_bounded_and_improve`](../tests/test_workbench.py#L172) | 三个搜索器在有界数学目标下改进，并更新 CMA 种群 |
| [`SearchTests.test_all_searchers_finite_bounded_and_improve.cost`](../tests/test_workbench.py#L176) | 内部：归一化候选到目标点的平方距离，返回数学测试代价 |
| [`SearchTests.test_invalid_does_not_enter_model`](../tests/test_workbench.py#L184) | invalid 不进入观测或参数拒绝区 |
| [`SearchTests.test_repeated_incumbent_uses_mean`](../tests/test_workbench.py#L188) | 重复观测按均值选择候选 |
| [`RunnerTests.test_open_and_closed_identification_to_validation`](../tests/test_workbench.py#L195) | 两条辨识起步路线完整完成频响与复验 |
| [`RunnerTests.test_model_tuning_validates_and_restores`](../tests/test_workbench.py#L208) | 模型初值整定完成三次验证并恢复原配置 |
| [`RunnerTests.test_outer_parameters_in_inner_trial_rejected`](../tests/test_workbench.py#L217) | 内环试验拒绝外环搜索字段 |
| [`RunnerTests.test_cancelled_job_never_starts_trial`](../tests/test_workbench.py#L222) | 已取消任务不用预算启动实验 |
| [`RunnerTests.test_infrastructure_abort_not_parameter_failure`](../tests/test_workbench.py#L228) | 溢出标 invalid/停止，不归咎于参数性能 |
| [`TransportAndHTTPTests.test_tcp_client_handshake_and_configuration`](../tests/test_workbench.py#L237) | 真实 socket 回环分片下的握手和配置 |
| [`TransportAndHTTPTests.test_tcp_client_handshake_and_configuration.peer`](../tests/test_workbench.py#L240) | 内部 TCP 服务端线程：接收真实 socket 字节交给 NativeLink，再按 7 字节分片回复 |
| [`TransportAndHTTPTests.test_serial_transport_uses_sdk_packets`](../tests/test_workbench.py#L256) | 模拟串口流使用 SDK 真实消息 |
| [`TransportAndHTTPTests.test_serial_transport_uses_sdk_packets.Port.__init__`](../tests/test_workbench.py#L258) | 模拟串口初始化：创建 NativeLink 和空接收缓存 |
| [`TransportAndHTTPTests.test_serial_transport_uses_sdk_packets.Port.in_waiting`](../tests/test_workbench.py#L260) | 模拟串口属性：把 C 回复入缓存并返回可读字节数 |
| [`TransportAndHTTPTests.test_serial_transport_uses_sdk_packets.Port.write`](../tests/test_workbench.py#L263) | 模拟串口：协议交给 C NativeLink，返回写入长度 |
| [`TransportAndHTTPTests.test_serial_transport_uses_sdk_packets.Port.read`](../tests/test_workbench.py#L264) | 模拟串口：取出最多 n 字节并更新剩余缓存 |
| [`TransportAndHTTPTests.test_serial_transport_uses_sdk_packets.Port.close`](../tests/test_workbench.py#L265) | 模拟串口：关闭 NativeLink |
| [`TransportAndHTTPTests.test_serial_bandwidth_rejected_before_start`](../tests/test_workbench.py#L269) | 串口带宽不足时开始前拒绝 |
| [`TransportAndHTTPTests.test_serial_bandwidth_rejected_before_start.Port.__init__`](../tests/test_workbench.py#L271) | 带宽测试的最小串口占位方法，当前 pass，无实际打开/关闭 |
| [`TransportAndHTTPTests.test_serial_bandwidth_rejected_before_start.Port.close`](../tests/test_workbench.py#L272) | 带宽测试的最小串口占位方法，当前 pass，无实际打开/关闭 |
| [`TransportAndHTTPTests.test_http_session_auth_and_workflow`](../tests/test_workbench.py#L279) | HTTP 会话/来源检查、完整任务及三个导出 |
| [`TransportAndHTTPTests.test_http_session_auth_and_workflow.call`](../tests/test_workbench.py#L284) | 内部 HTTP 辅助：发送请求并读取响应，返回状态码和 body bytes |

C 测试 [workbench_core_test.c](../tests/workbench_core_test.c)：`near(a,b)` 断言两值误差小于 0.0001；`main()` 执行 PID/CRC 性质测试，成功打印运行时大小并返回 0，断言失败终止。

## 10. 查阅与修改约束

只需使用产品时读 README/使用说明书；移植 MCU 先看 C 公开接口和板级函数；更换优化器时遵守 Space 的参数锁定与 Search ask/tell；修改协议时同步 C/Python 序列化并复验。不要用“能计算出分数”代替确认轨迹完整，不要由搜索器更改保护范围。

文档按当前函数定义核对。Python 的每个手写 def、C 公开/内部函数及网页命名函数均有条目；嵌套函数按所属作用域说明，生成代码和第三方库内部不展开。
