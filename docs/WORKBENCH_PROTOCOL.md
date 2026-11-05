# PID 工作台应用协议 v1

所有多字节字段为大端。浮点数 IEEE-754 binary32，不能直接发送 C 结构体内存。

## 兼容外帧

| 偏移 | 长度 | 字段 |
|---|---:|---|
| 0 | 2 | `AA 55` |
| 2 | 2 | 总帧长，包括头尾与校验 |
| 4 | 2 | 应用命令，默认 `7D 50` |
| 6 | 2 | CRC16 大端 |
| 8 | N | 应用负载 |
| 8+N | 2 | `55 AA` |

CRC 初值 `FFFF`、反射多项式 `A001`，覆盖整帧但跳过偏移 6/7，包含尾部。`123456789` 的普通 CRC16 为 `4B37`。默认总长最多 1024。坏 CRC/长度逐字节重新找帧，残缺帧超过 250 ms 清除。串口/TCP 可分片粘包。

## 应用头（24 字节）

`version:u8, opcode:u8, status:u16, session:u32, request:u32, trial:u32, revision:u32, boot_id:u32`。

版本 1。请求 status=0，响应 opcode 为请求 opcode OR 80h。HELLO 使用主机非零暂定 session，设备分配本次启动内单调递增的实际 session，主机采用响应值；此后必须匹配 session 与 boot_id。设备重启必须更换 boot_id。

请求 ID 单调递增；最后一个请求完全相同的字节重试，返回缓存响应，包括已读取的样本，不重复执行。相同 ID 不同内容拒绝；更旧 ID 拒绝。HELLO 重试仅在缓存仍是同一次 HELLO 时返回原响应；旧 HELLO 不重新激活旧 session。计数不允许回绕复用。

| opcode | 请求体 | 成功响应体 |
|---|---|---|
| 1 HELLO | 无 | 状态 + 配置，204 字节 |
| 2 STATUS | 无 | 同上 |
| 3 CONFIGURE | 168 字节配置；revision=当前+1 | 无 |
| 4 START | 108 字节计划；非零 trial、当前 revision | 无 |
| 5 READ | 无 | state/reason/count 各 u32 + count 个样本 |
| 6 STOP | 无 | 无；运行中取消并调用退出钩子 |
| 7 CLEAR | 无 | 无；仅非运行时清空样本并回 IDLE |

状态体前 36 字节为 `boot_id,state,reason,pending,ticks,open_loop_available` 六个 u32 与 `dt,hard_output_min,hard_output_max` 三个 f32，后接配置。状态 IDLE=0、RUNNING=1、COMPLETE=2、ABORTED=3。COMPLETE 后仍要 READ 到 count=0，以取完最后样本。

状态原因顺序：0 ok，1 bad_request，2 busy，3 session，4 revision，5 limit，6 timeout，7 cancelled，8 nonfinite，9 overflow，10 blocked，11 open_denied，12 sequence，13 slew。

## 配置（168 字节）

`loop_count:u32, outer_divider:u32, inner:80字节, outer:80字节`。单环也传输合法的占位外环配置。

每环：`kind,form,flags,antiwindup` 四个 u32，随后 16 个 f32：

```text
kp, ki, kd, ffj, ffb, integral_limit, deadband, integral_a,
integral_b, output_tau, derivative_tau, output_min, output_max,
aw_gain, blocked_error, blocked_time
```

kind P/PI/PD/PID = 0/1/2/3；form 位置/增量 = 0/1；antiwindup 无/限幅/条件积分/反算 = 0/1/2/3。

flags：1 积分限幅，2 微分对测量，4 梯形积分，8 比例对测量，16 输出滤波，32 变速积分，64 微分滤波，128 堵转，512 测量前馈；256 未定义。

## 计划（108 字节）

六个 u32：`wave,open_loop,active_loop,decimation,frequency_count,timeout_ms`。

21 个 f32：`dt,duration,settle,reference,amplitude,bias,y_min0,y_min1,y_max0,y_max1,output_min,output_max,max_slew,frequencies[8]`。

wave 0阶跃/1单频正弦/2等幅多正弦；频率单位 Hz，多频总幅值按频率数量均分。active_loop 0内环/1外环。闭环阶跃改目标；闭环正弦向执行器输入（内环）或内环目标（外环）叠加激励。开环偏置加激励直接作用被辨识对象输入。duration 包括 settle，dt 必须等于板上配置周期。

## 样本（60 字节）

`index:u32` + 下列 13 个 f32 + `flags:u32`：

```text
time, reference, measurement, input, actuator, perturbation,
inner_reference, inner_measurement, outer_measurement,
controller_output, p, i, d
```

`input` 是所选对象实际输入（内环为实际执行器输出，外环为内环目标）；`actuator` 始终是物理执行器输出。`controller_output` 是激励叠加前控制器输出/开环偏置。flags bit0 表示限幅；P/I/D 属于被调环。index 每控制 tick 增加，记录间距等于 decimation。time 为设备控制 tick 乘 dt，不是独立高分辨率执行时间测量。
