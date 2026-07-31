# 摆杆角度标定 —— 上位机 ↔ STM32 串口协议 v1.1

> 本文档是上位机与嵌入式端的**唯一协议依据**。上位机实现见同目录 `protocol.py` /
> `links.py`；嵌入式端按本文档实现 `PCLink`（标定串口）模块。协议冻结后如要改动，
> 先改本文档并同步两端。

## 1. 物理层

- 通信方式：USB-TTL 转串口模块（CH340/CP2102 等）连接 PC 与 STM32，**3.3V 电平、共地**。
- STM32 侧：**USART1**，`PB6 = STM32 TX`，`PB7 = STM32 RX`（balance 分支已配置，
  115200 8N1，见 `CM7/Src/main.c` 的 `MX_USART1_UART_Init` 与 `stm32h7xx_hal_msp.c`）。
  若实际焊接改用其它串口，只需同步修改 `HOST_DEBUG_UART_INSTANCE` 与 MSP 配置，
  协议与引脚无关。
- **不得与以下外设冲突**（balance 分支已占用）：
  - USART2（TJC 屏，PD5/PD6，9600 baud）；
  - UART4（K230，PB8/PB9，115200 baud）；
  - **I2C2（LSM6DSV16X IMU，PB10/PB11）—— 所以 USART3 的 PB10/PB11 不可用**；
  - I2C4（OLED + 八路红外，PF14/PF15）；
  - TIM2（电机 PWM，PA0/PA2）、TIM4 CH4（舵机 PWM，PD15）、TIM1/TIM8（编码器）。
- 参数：`115200 8N1`，无流控。
- 帧格式：文本行，ASCII，**行尾 `\r\n`（兼容单独 `\n`）**，单行 ≤ 80 字节。
- 交互模型：上位机发一条命令，设备**同步应答一条**；上位机默认超时 1.5 s。
  设备主动推送的实时行（STREAM）以 `S ` 开头，上位机与应答行分开处理。
- 无校验和：标定链路非安全关键，USB-TTL 误码率可接受；格式非法的行直接丢弃
  （设备回 `ERR parse`，上位机对无法解析的应答报错）。

## 2. 符号约定（全工程统一，含 K230、STM32、上位机）

- 水管俯仰角 `pitch > 0`：**右端升高**，单位度。
- 舵机 PWM：实测安全区间为 `500～1600 us`，`1050 us` 对应水管水平位置。
- `ts_ms`：设备上电后的毫秒计数（`HAL_GetTick()` 风格）。
- 标定模式中设备必须**保持小车电机停止**；标定串口收到任何命令都不应触发行驶。
- **标定模式开关**：设备上电默认关闭标定模式。上位机连接后先发 `CAL 1` 进入
  （设备会先停车），否则除 `PING`/`CAL` 外的命令全部返回 `ERR disabled`。

## 3. 命令表

### 3.0 CAL（标定模式开关）

请求：`CAL <1|0>\r\n`　应答：`OK\r\n` 或 `ERR busy\r\n`

示例：`CAL 1` → `OK`；`CAL 0` → `OK`

说明：`CAL 1` 进入标定模式并**先执行停车**（避免与循迹/平衡任务冲突），
`CAL 0` 退出。该命令在任何状态下都可执行，是唯一不受 `ERR disabled` 限制的
功能命令。固件编译开关：`APP_USE_HOST_CALIBRATION`（app_config.h），
上电默认状态由 `APP_HOST_CALIBRATION_DEFAULT` 决定（默认 0）。

### 3.1 PING

请求：`PING\r\n`　应答：`PONG <version>\r\n`

示例：`PING` → `PONG pclink-1.1`

用途：连接探测、固件版本确认。上位机连接后先发 3 次 PING 确认链路。

### 3.2 SERVO

请求：`SERVO <pwm_us>\r\n`　应答：`OK\r\n` 或 `ERR range\r\n`

示例：`SERVO 1050` → `OK`

说明：直接设置舵机PWM目标值。越界（<500或>1600）返回`ERR range`且
**不改变当前值**。固件调用`ServoDriver_SetPulseUs`直接输出，并在驱动层再次执行
500～1600 us安全限幅。标定场景允许直接跳变，无需斜坡。

### 3.3 PWM

请求：`PWM\r\n`　应答：`PWM <pwm_us>\r\n`

示例：`PWM` → `PWM 1050`

说明：查询当前舵机 PWM 目标值（最后一次成功 SERVO 的值）。

### 3.4 IMU

请求：`IMU\r\n`　应答：`IMU <pitch_deg> <rate_deg_s> <ts_ms>\r\n`

示例：`IMU` → `IMU 2.3140 0.512 123456`

说明：读取一次水管俯仰角和俯仰角速度。**标定模式下返回的是"原始加速度俯仰角"**
（`atan2(accel_y, accel_z)`），不依赖固件九点标定；
角速度来自陀螺仪 X 轴原始值。浮点打印建议 `%.4f`（角度）、`%.3f`（角速度）。

### 3.5 SAMPLE（标定采样的核心命令）

请求：`SAMPLE <count> <dt_ms>\r\n`
应答：`SAMPLE_OK <mean_deg> <std_deg> <count> <ts_ms>\r\n`

示例：`SAMPLE 100 5` → `SAMPLE_OK 2.3120 0.0180 100 123999`

说明：设备连续采集 `count` 个俯仰角样本（原始加速度俯仰角），
间隔 `dt_ms`，返回均值、标准差和完成时刻。
约束：`1 <= count <= 200`，`10 <= dt_ms <= 100`，越界返回 `ERR range`。
**注意**：LSM6DSV16X 当前配置为 60 Hz 输出（约 16.7 ms 一个新样本），因此
`dt_ms` 建议取 20（约 50 Hz 有效采样）；小于 16 时设备会重复读到陈旧样本，
标准差失真。**SAMPLE 阻塞主循环期间 IMU 轮询不会运行，固件在采样循环内
直接轮询 IMU 并现场计算俯仰角**，保证每个样本都是新读数。标定中推荐
`SAMPLE 100 20`（2 秒一采）；`ts_ms` 为最后一次采样完成时刻。

### 3.6 STREAM（实时显示）

请求：`STREAM <1|0>\r\n`　应答：`OK\r\n`

说明：置 1 后设备以 **10 Hz（100 ms 间隔）** 主动推送：

```
S <ts_ms> <pitch_deg> <rate_deg_s>\r\n
```

示例：`S 123500 2.3100 0.500`

设备持续推送直到收到 `STREAM 0`。上位机进行标定/验证前应发 `STREAM 0`；
即使忘记关闭，上位机也会把 `S ` 行与应答行分开，不影响 SAMPLE/SERVO 时序。

### 3.7 VIN（可选）

请求：`VIN\r\n`　应答：`VIN <mv>\r\n` 或 `ERR unsupported\r\n`

说明：读取电池电压，用于记录不同电量下的标定条件。当前固件未接 ADC 采样，
固定返回 `ERR unsupported`。

## 4. 错误码

| 响应 | 含义 |
|---|---|
| `ERR parse` | 请求行格式/参数无法解析 |
| `ERR range` | 参数越界（PWM、SAMPLE 参数等） |
| `ERR unknown` | 未知命令 |
| `ERR busy` | 上一条命令未完成（实现可选，不做也没关系） |
| `ERR unsupported` | 命令本身合法但该固件未实现（如 VIN） |
| `ERR disabled` | 标定模式未开启（需先发 `CAL 1`） |

## 5. 嵌入式端（STM32）适配要点

实现清单（balance 分支已完成，供核对）：

1. `Bsp/Src/host_link_uart.c`：USART1 单字节接收中断 + 256B 环形缓冲 + 阻塞发送；
2. `stm32h7xx_it.c` 已有 `USART1_IRQHandler`，`tjc_uart.c` 的 HAL 回调已加入
   `HostLinkUart_HandleRxComplete/HandleError` 分发；
3. `Protocol/Src/host_calibration_protocol.c`：行状态机 + 命令分发
   （PING/CAL/SERVO/PWM/IMU/SAMPLE/STREAM/VIN），浮点输出用整数定点格式化，
   不依赖 printf 的 %f 支持；
4. `SAMPLE` 阻塞循环 `count` 次读 `App_GetPipePitchDeg()`，`HAL_Delay(dt_ms)` 间隔；
5. `STREAM` 在 `HostCalibrationProtocol_Task` 中按 100 ms 节拍推送 `S ` 行；
6. `SERVO`调用`ServoDriver_SetPulseUs`直接输出，协议层和驱动层均限制为500～1600 us；
7. 标定模式下水管俯仰角来自
   `App_GetPipeRawAccelPitchDeg()/App_GetPipeRawPitchRateDps()`：
   原始加速度俯仰角（`atan2(accel_y, accel_z)`，LSM6DSV16X / I2C2 / 60Hz）
   不受九点标定门控；零位由用户按"球平衡"确定，协议不做置零；
8. 模式互斥：`App_SetHostCalibrationEnabled(1)` 先执行 `APP_MODE_STOP` 停车，
   未开启时命令返回 `ERR disabled`。

## 6. 上位机联调检查清单

1. `PING`三连成功并返回`pclink-1.1`；
2. `SERVO 500 / 1050 / 1600`依次执行，1050 us时水管水平；
3. PWM从1050增加到1600时水管右端升高、原始IMU角度减小；
4. `IMU`连续读值稳定，静止时标准差小；
5. `SAMPLE 100 20`与连续`IMU`读值的均值一致；
6. `STREAM 1`后10 Hz稳定推送，10秒内丢行率<1%；
7. 越界命令（如`SERVO 1601`）返回`ERR range`且不改变舵机位置。
