# STM32Car 固件

本目录是 STM32H745ZI CM7-only 智能车底盘固件，使用 IAR EWARM 8.3 和 STM32H7 HAL。

## 当前功能

- TJC 触摸屏通过 USART2 通信：TX 为 PD5，RX 为 PD6，115200 8N1。
- TJC 停止命令 `5A A5 00 FF` 进入停车模式并禁用钢球闭环。
- TJC 基础巡线命令 `5A A5 01 FE` 进入八路 I²C 红外巡线模式。
- TJC 钢球平衡启用命令 `5A A5 02 FD`，禁用命令 `5A A5 03 FC`。
- TJC 水管IMU单点水平标定命令`5A A5 04 FB`：固定1050 μs后采集水平零偏和gyro X偏置。
- TJC 第三题开环水管角序列命令 `5A A5 05 FA`：水管按 负/正/负 角幅值各持续一段可调时间后回水平，K230 不参与运行期控制；角度/时间通过 USART1 `SHUTTLE` 子命令设置。
- TJC 第四题 AB 命令 `5A A5 06 F9` 和第五题 ABCD 命令 `5A A5 07 F8`，会先等待钢球中心稳定，再启动混合前馈和平滑底盘任务。
- TJC 查询命令 `5A A5 0E F1` 返回当前底盘工作模式。
- K230 UART4：STM32 TX 为 PB9，RX 为 PB8，115200 8N1，接收钢球位置 ASCII 消息。
- 八路红外模块通过 I2C4 读取：SCL 为 PF14，SDA 为 PF15，设备地址为 `0x12`。
- 基础红外巡线：八路物理位置误差经过低通、PD、限幅和斜率限制后生成左右轮速度目标。
- 平滑行驶：巡线启动时通过加速度和jerk双重限制将基础速度从当前车速平滑提升到巡航速度；第四至六项可通过`App_StartSmoothLineRun()`按150 cm或614.2 cm里程启动不停车的通过模式。
- 底盘加速度估计：左右编码器平均速度先低通再差分，并对加速度再次低通和限幅，降低量化噪声对水管IMU姿态融合的干扰。
- 双轮速度闭环：20 ms 编码器采样，左右轮独立 PID，包含积分限幅、启动补偿和编码器反馈故障检测。
- 钢球平衡：K230位置外环生成约−5.55°～+5.55°目标水管角；目标点采用±4 mm位置死区，死区内清除P/I输出并保持水管水平；距离目标不超过10 mm时禁用并清除积分，仅在10～20 mm范围内启用默认`Ki=0.008 deg/(mm·s)`的位置积分并限制为0.8°，超过20 mm或越过目标点时自动清零；IMU姿态内环使用默认`Kp=0.60`、`Kd=0.050 s`修正线性PWM前馈误差。
- 水管姿态：IMU固定在水管远端，+Y轴沿水管指向钢球、+Z轴向上；PWM增大时原始IMU角减小，固件固定使用−1符号归一化为正水管角。
- 自动标定：停车后把舵机缓速移到1050 μs水平点，静止采集IMU水平零偏和gyro X偏置；每次上电必须成功标定后才能启用平衡。
- 舵机PWM：TIM4_CH4 / PD15，50 Hz；实测安全范围500～1600 μs，1050 μs为水平，线性关系为`φ=0.01009×(PWM−1050)`。
- 安全停车：停止模式、持续丢线或编码器反馈故障时清空速度控制器并停止电机。
- LSM6DSV16X：通过 I2C2（PB10/PB11）接入，自动探测 `0x6A`/`0x6B` 并读取三轴陀螺仪和三轴加速度计。
- USART1上位机链路：PB6/PB7，115200 8N1，通过USB-TTL连接PC。
  `Tools/pipe_calibration/calib_gui.py`提供基础区、PID调参、标定区和验证区；PID调参页可在线
  读写位置外环Kp/Ki/Kd与姿态内环Kp/Kd，以20 Hz同步显示钢球位置、速度、两级控制项、
  水管姿态和PWM，并保存CSV及JSON分析摘要。舵机标定需`CAL 1`并停车，PID闭环调参需
  `CAL 0`且车辆保持停车；协议见`Tools/pipe_calibration/PROTOCOL.md`。

K230 红线协议和视觉巡线控制已移除，底盘巡线输入为八路 I²C 红外传感器；K230 UART4 当前用于接收 `B,<valid>,<position_d10_mm>,<quality>,<sequence>` 钢球位置消息。

## 硬件映射

| 功能 | 外设与引脚 |
|---|---|
| TJC UART TX | USART2_TX / PD5 |
| TJC UART RX | USART2_RX / PD6 |
| K230 UART TX | UART4_TX / PB9 |
| K230 UART RX | UART4_RX / PB8 |
| 上位机标定 UART TX | USART1_TX / PB6 |
| 上位机标定 UART RX | USART1_RX / PB7 |
| 右电机 PWM | TIM2_CH1 / PA0 |
| 右电机方向 | PA1 |
| 左电机 PWM | TIM2_CH3 / PA2 |
| 左电机方向 | PA3 |
| 右编码器 | TIM1_CH1 / PA8、TIM1_CH2 / PE11 |
| 左编码器 | TIM8_CH1 / PC6、TIM8_CH2 / PC7 |
| 红外 I2C4 SCL | PF14 |
| 红外 I2C4 SDA | PF15 |
| 位置舵机 PWM | TIM4_CH4 / PD15 / 50 Hz，安全范围500～1600 μs，水平1050 μs |
| LSM6DSV16X SCL | I2C2_SCL / PB10 |
| LSM6DSV16X SDA | I2C2_SDA / PB11 |

八路红外模块使用 5V 供电、与 STM32 共地；实测 SDA、SCL 空闲电压均为 3.3V。模块 7 位 I²C 地址为 `0x12`，状态寄存器为 `0x30`，黑线对应位为 0；实测取反后的位序仍与车辆左右相反，驱动会再次反转八位，统一为 bit0 在车辆左侧、bit7 在车辆右侧。

OLED 与八路红外模块共用 I2C4，OLED 地址为 `0x3C`。模块上电后应等待至少 20 秒并按实际安装高度完成黑白校准，固件不强制等待。

LSM6DSV16X 独占 I2C2，固件会依次探测 7 位地址 `0x6A` 和 `0x6B`，期望 `WHO_AM_I` 为 `0x70`。加速度计和陀螺仪均配置为120 Hz，主循环每2 ms检查数据就绪标志。水管姿态使用 X 轴陀螺仪积分和 Y/Z 加速度重力角互补融合，姿态内环直接使用校零后的 X 轴角速度产生`−Kd×角速度`阻尼；底盘加减速时使用编码器估计加速度降低 Accel 融合权重。单个水管 IMU 仍无法完全区分持续车体加速度与重力倾角。

电机 PWM 约为 6.4 kHz。最大 PID 输出限制为 50%，当前基础巡线速度由 `APP_BASE_SPEED_CM_S` 配置。电机驱动仍使用参考工程要求的特殊 PWM/DIR 电气组合，不能擅自改成普通方向脚控制。

## 第四至六项平滑行驶接口

普通 `0x01` 巡线仍保留第二项返程 A 点停车逻辑，但基础速度不再从零直接阶跃到40 cm/s，而是经过最大加速度和最大jerk限制。第四至六项应在钢球中心闭环已经稳定后调用：

```c
App_StartSmoothAbRun();  /* 第四项：A到B通过150 cm。 */
App_StartSmoothLapRun(); /* 第五、六项：整圈通过614.2 cm。 */
```

需要其他试验里程时，也可以调用`App_StartSmoothLineRun(completion_distance_cm)`。

该接口只锁存名义通过里程并保持巡线继续运行，不会在达到里程时急停。`g_app_motion_distance_reached`用于观察已经达到名义里程；现场仍应结合赛道标志确认实际通过位置。当前起步参数为最大加速度25 cm/s²、最大减速度30 cm/s²、最大jerk 100 cm/s³，这些只是保守起始值，尚未完成实车钢球误差验证。

## AB / ABCD 中心稳定模式

串口屏 `page0` 保留两个独立按钮：

- `AB`：发送 `5A A5 06 F9`，目标速度 35 cm/s，最大起步加速度 10 cm/s²；编码器累计达到 150 cm 时冻结计时，再以 8 cm/s² 的上限柔和停车。
- `ABCD`：发送 `5A A5 07 F8`，目标速度 27.5 cm/s，保持已实测稳定的最大起步加速度 10 cm/s²、停车减速度 8 cm/s²和 jerk 30 cm/s³；启动后先确认已离开 A 点横线，累计达到 400 cm 后才允许连续两帧以至少 4 路黑线确认返程通过 A 点。

两个模式都会自动启用钢球中心控制，但不会在视觉刚有效时立即起步。准备阶段先使用完整位置控制将钢球带回中心；固件要求位置误差绝对值不超过 5 mm、球速绝对值不超过 15 mm/s，且视觉和水管姿态持续有效 300 ms 后才切换到混合前馈、启动底盘与成绩计时。准备最长等待 10 s；第四题运行限时为 8 s，第五题为 30 s，超时后冻结计时并柔和停车。

起步、巡航和柔和停车全程使用同一套混合补偿：底盘速度规划器的命令加速度提供提前量，水管 IMU 的低频惯性扰动修正理论前馈，钢球位置与速度反馈只处理剩余偏差。起步和停车阶段的位置死区保持 ±2 mm；命令加速度绝对值不超过 1 cm/s² 的匀速段扩大为 ±5 mm。普通匀速控制不使用低帧率噪声较大的速度D项；但实际误差或按当前速度外推的预测误差达到 ±7 mm 时，控制器会提前锁存强回收，启用增强D制动、提高总反馈角限制并保证至少 0.75° 的位置回拉。只有实际位置回到 ±5 mm 且球速降至 10 mm/s 以内才退出，避免等到题目 ±10 mm 边界才动作而继续惯性外冲。该改动不影响停车/第三题控制，也不修改前馈系数、PWM 限幅或电机极性。

K230 实车只有约 5～6 FPS 时，AB/ABCD 行驶中相邻视觉帧可能包含明显真实位移。控制器因此按模式使用两套跳变门槛：停车/静态平衡仍以 15 mm、400 ms 严格抑制误检；底盘进入运输模式后放宽为 30 mm，并在持续偏离 150 ms 后重采样。该改动只恢复动态任务对真实球位移的响应，不改变第三题/停车调参使用的严格保护，也不把静态控制器的回程制动或破静摩擦参数带入运输控制律。

钢球中心误差达到或超过 10 mm 时，STM32 会在串口屏中央绘制四层红圈；误差回到 8 mm 内并连续保持三帧后清除。连续两帧达到 10 mm 才锁存“完成但超差”，单帧视觉毛刺不会改变最终结果。任务会记录最大绝对误差、RMS、有效样本数、超差事件数、超差累计时长、通过距离和毫秒级运行时间；串口屏最终以 0.1 s 显示“通过”“完成但超差”或“超时”及最大误差。

动态任务运行和柔停期间只接受全局 STOP、状态查询，以及按安全急停处理的“禁用平衡”命令；其他模式命令会被拒绝并显示“动态任务运行中”。成绩冻结前，红外、IMU、映射、视觉或电机故障超过各自已有门槛后立即停车并显示具体原因。通过 B/返程 A 或计时超时后，成绩和故障结论不再被柔停阶段的视觉、姿态状态改写；红外和电机硬故障仍会直接安全停车。AB 柔停期间钢球中心闭环继续保持，直到车辆停稳或收到全局 STOP。

也可以直接从固件调用：

```c
App_StartAbRun();
App_StartAbcdRun();
```

主要调试变量包括 `g_app_dynamic_run_state`、`g_app_dynamic_run_result`、`g_app_dynamic_elapsed_ms`、`g_app_dynamic_distance_cm`、`g_app_dynamic_max_abs_error_mm`、`g_app_dynamic_rms_error_mm`、`g_app_dynamic_overrange_duration_ms`、`g_app_ball_limit_warning` 和 `g_app_ball_limit_warning_count`。

## 水管角与PWM线性映射

当前水管已经调平并按刚性结构处理。上位机实测得到：

```text
φ(°) = 0.01009 × (PWM − 1050)
PWM(us) = 1050 + φ / 0.01009
```

正式控制直接使用该线性关系：位置外环和IMU姿态内环输出最终相对水管角，BSP再换算为PWM并执行500～1600 μs双重限幅。对应范围约为−5.5495°～+5.5495°。

旧的闭式连杆模型和九点曲线文件仍保留作为历史资料及离线对比，不再参与运行时控制。USART1上位机标定模式可直接发送500～1600 μs并读取原始IMU，用于复核机械方向、回差和线性误差。

## TJC 协议

触摸屏发送 4 字节命令帧：

```text
5A A5 <command> <checksum>
```

校验字节为四字节帧前三个字节的异或值：

```text
checksum = 0x5A ^ 0xA5 ^ command
```

支持的命令如下：

| 命令 | 帧 | 含义 |
|---|---|---|
| `0x00` | `5A A5 00 FF` | 全局停止并禁用钢球平衡 |
| `0x01` | `5A A5 01 FE` | 基础八路红外巡线 |
| `0x02` | `5A A5 02 FD` | 启用中心钢球位置闭环 |
| `0x03` | `5A A5 03 FC` | 禁用钢球平衡并回中位；标定中收到时立即中断 |
| `0x04` | `5A A5 04 FB` | 停车模式下回到1050 μs并执行IMU单点水平标定 |
| `0x05` | `5A A5 05 FA` | 钢球放回中心（最新K230单帧±5 mm内）后启动第三题开环角序列，不自动启用平衡 |
| `0x06` | `5A A5 06 F9` | AB：150 cm 中心稳定通过，过 B 后柔和停车 |
| `0x07` | `5A A5 07 F8` | ABCD：27.5 cm/s 平滑整圈，过 A 后柔和停车 |
| `0x0E` | `5A A5 0E F1` | 查询当前底盘模式 |

固件通过 USART2 返回 5 字节结果帧：

```text
A5 <result> <current_mode> <request> <xor>
```

`0x04` 被接受后先返回上述结果帧，后续标定进度通过淘晶驰指令异步更新全局文本控件 `page0.t0`：

```text
page0.t0.txt="<UTF-8 中文状态>" FF FF FF
```

标定状态包括准备、回到1050 μs、水平静置采样、成功和失败原因。固件平均约3秒IMU样本得到水平加速度角与gyro X偏置，并固定使用`imu_pitch_sign=-1`；成功后`scale=1.00`、`offset=0.00`仅作为兼容状态显示。每次上电必须先成功执行`0x04`，才允许在停车模式下发送`0x02`；`0x02`会先复核约150 ms的水平和静止状态，再进入平衡。

推荐比赛启动顺序：

```text
放置钢球并人工确认位于 O 点
    → 0x05 启动第三题（钢球不在中心会被拒绝并在串口屏提示）

第三题为纯开环水管角序列：按“负/正/负”角幅值各持续一段可调时间后回水平，
期间完全忽略 K230 与 IMU，目标角经限速斜坡平滑过渡到舵机，无需 0x04 标定。
角度幅值 0～5.5°、单段时间 20～5000 ms、三段总和 ≤5000 ms，仅 RAM 保存，
复位恢复默认 5.0/5.0/5.0°、300/600/300 ms。参数可用 USART1 `SHUTTLE` 子命令
读取/写入/恢复默认/启动/停止，见下文。
```

## USART1上位机PID调参

运行：

```bash
python Tools/pipe_calibration/calib_gui.py
```

使用顺序：

1. 完成TJC `0x04`水管水平标定；
2. 连接USART1并确认上位机顶部“标定模式(CAL)”未勾选；
3. 进入“PID调参”页读取当前参数；
4. 开启20 Hz遥测，必要时设置目标位置并启用平衡；
5. 每次只调整一组参数，点击“开始录制”，至少记录10～15秒；
6. “停止并保存CSV”会在`Tools/pipe_calibration/data/`生成同名CSV和JSON摘要；
7. 后续调参分析应同时保留CSV和JSON，以便对比误差RMS、越零次数、水管跟随误差、
   角速度、P/I/D控制项和PWM范围。

上位机写入的PID只保存在RAM，复位后恢复固件默认值。PID写入、目标设置和闭环启停只允许
车辆停车且`CAL 0`时执行；动态任务运行期间会返回`ERR busy`，不会在线改变比赛控制参数。

### 第三题 SHUTTLE 子命令

第三题参数与启停通过 USART1 文本命令 `SHUTTLE <子命令>` 管理，要求 `CAL 0`（标定模式关闭），
否则返回 `ERR busy`。角度为幅值，执行符号固定为 负/正/负；时间单位毫秒。
第三题角度输出为纯开环，**不需要 0x04 水管标定，也不依赖 IMU**；`SHUTTLE START`
仅要求车辆停车、CAL 0、钢球最新单帧位于中心 ±5 mm。

| 命令 | 含义 |
|---|---|
| `SHUTTLE GET` | 读取当前参数，回复 `SHUTTLE <a1> <ms1> <a2> <ms2> <a3> <ms3>` |
| `SHUTTLE SET <a1> <ms1> <a2> <ms2> <a3> <ms3>` | 写入三段角度幅值(°)与时间(ms)，校验后返回 `OK`，超范围返回 `ERR range` |
| `SHUTTLE DEFAULT` | 恢复默认参数 5.0/5.0/5.0°、300/600/300 ms |
| `SHUTTLE START` | 钢球放回中心后启动开环序列；拒绝时返回 `ERR state` 并在串口屏提示 |
| `SHUTTLE STOP` | 中止序列并回水平保持（不启动钢球位置平衡） |

`Tools/pipe_calibration/calib_gui.py` 的“第三题”页签可填写三段角度与时间、实时显示总时长，
点击“启动第三题”会先自动 `SET` 再 `START`。

## IAR 构建

打开：

```text
EWARM/STM32Car.eww
```

命令行全量构建：

```powershell
& "D:\Program Files (x86)\IAR Systems\Embedded Workbench 8.3\common\bin\IarBuild.exe" `
  "D:\Develop\2026_STM32Car\STM32Car\EWARM\STM32Car.ewp" `
  -build STM32H745I_Nucleo_CM7
```

输出文件名为 `STM32Car_CM7.out` 和 `STM32Car_CM7.hex`。

## 主机测试

```bash
gcc -std=c11 -Wall -Wextra -Werror \
  -IApp/Inc -IBsp/Inc -IControl/Inc -IProtocol/Inc \
  tests/test_basic_control.c \
  Control/Src/infrared_line_control.c \
  Control/Src/wheel_speed_pid.c \
  Control/Src/lap_stop_detector.c \
  Control/Src/linkage_kinematics.c \
  Control/Src/pipe_attitude_control.c \
  Control/Src/ball_balance_control.c \
  Control/Src/ball_shuttle_control.c \
  Protocol/Src/ball_position_protocol.c \
  -lm -o tests/test_basic_control.exe
./tests/test_basic_control.exe
```

平滑行驶和底盘加速度估计使用独立主机测试：

```bash
gcc -std=c11 -Wall -Wextra -Werror \
  -IApp/Inc -IControl/Inc \
  tests/test_smooth_chassis_control.c \
  Control/Src/chassis_motion_profile.c \
  Control/Src/chassis_accel_estimator.c \
  Control/Src/infrared_line_control.c \
  -o tests/test_smooth_chassis_control.exe
./tests/test_smooth_chassis_control.exe
```

钢球平衡诊断日志使用独立主机测试（默认固件关闭日志，测试时显式开启）：

```bash
gcc -std=c11 -Wall -Wextra -Werror \
  -DAPP_ENABLE_BALANCE_LOG=1 \
  -IApp/Inc -IControl/Inc \
  tests/test_balance_debug_log.c \
  App/Src/balance_debug_log.c \
  -o tests/test_balance_debug_log.exe
./tests/test_balance_debug_log.exe
```

IAR 内存采集和 CSV 转换步骤见 [钢球平衡诊断日志](Tools/BalanceDebugLog/README.md)。

第四、五题准备、计时、超差统计、通过、超时和故障状态机使用独立主机测试：

```bash
gcc -std=c11 -Wall -Wextra -Werror \
  -IApp/Inc \
  tests/test_dynamic_run_task.c \
  App/Src/dynamic_run_task.c \
  -lm -o tests/test_dynamic_run_task.exe
./tests/test_dynamic_run_task.exe
```

## 上车前必须验证

1. 烧录前清空 CM4 所在的 Flash Bank 2，或在 Option Bytes 中关闭 CM4 启动，避免旧 CM4 固件继续占用外设。
2. 模块上电后等待至少 20 秒，并按实际巡线高度完成黑白校准。
3. 确认红外模块 SDA、SCL 空闲电压为 3.3V，且与 STM32 共地。
4. 不启动电机，确认 I²C 地址 `0x12` 应答，逐个压住物理左到右八个探头，观察 `g_app_line_sensor_mask` 依次为 `0x01`～`0x80`。
5. 确认 OLED 地址 `0x3C` 与红外模块同时工作。
6. 架空测试左右电机正反转和停止电平组合。
7. 确认车辆向前时 TIM1/TIM8 编码器速度均为正。
8. 实测轮子一圈的编码器计数，校准当前 counts/rev 参数。
9. 架空低速测试，确认左侧探头压线时左轮目标低于右轮目标，右侧探头压线时方向相反。
10. 连续断开 I²C 通信，确认第 3 次失败后传感器故障停车；恢复连接后能够重新读取。
11. 确认八路持续丢线后电机可靠停止。
12. 第二题和第五题实车验证时，确认车辆先连续驶离起点横线，且累计里程达到约 400 cm 前再次出现 4～5 路黑线不会被判为返程 A 点。
13. 返回 A 点连续 2 个有效采样均检测到至少 4 路黑线后，确认完成事件只触发一次；第二题立即停车，第五题先冻结成绩再按速度规划柔和停车。
14. 记录 `MotorDriver_Stop()` 实际造成的停车过冲；安全故障使用立即停车，第四、五题正常通过和计时超时使用受限减速度停车。
15. 首次适配时移除钢球并确认无机械阻挡，使用USART1上位机依次发送500、1050、1600 μs；确认1050 μs水管水平，两个端点均无碰撞。
16. 确认IMU刚性固定在水管远端并随管段同步转动；保持+Y朝钢球、+Z向上，SDA/SCL空闲电压为3.3 V。
17. 连接IMU后确认`g_imu_ready=1`、`g_imu_who_am_i=0x70`，且`g_imu_i2c_address`为`0x6A`或`0x6B`；静置观察`g_imu_read_count`，确认读取速率接近120次/秒且没有持续增加I²C错误。
18. 移除钢球，在停车模式发送`5A A5 04 FB`；确认舵机缓速回到1050 μs，静置并采样约3秒后显示标定成功；任何时候发送0x00/0x03都应中断标定并回水平。
19. 标定成功后确认`g_app_pipe_calibration_valid=1`、`g_app_pipe_imu_pitch_sign=-1`、`g_app_pipe_accel_zero_deg`接近当前水平原始IMU角；控制范围应为约−5.5495°～+5.5495°。
20. 标定失败、复位或尚未执行0x04时发送0x02，确认STM32拒绝启用钢球平衡。
21. 放置钢球后发送0x02，确认水管在约150 ms水平复核期间保持1050 μs，随后显示“平衡已启用”。
22. 空载手动抬高水管右端，确认 `g_app_pipe_pitch_deg` 为正，`g_app_pipe_pitch_rate_dps` 动态方向正确。
23. 按 balance 分支的实机曲柄摇杆方向验证，K230 正位置应产生正目标水管角，负位置应产生负目标水管角；必须分别确认两侧实际运动都朝目标点，方向错误时禁止继续运行。
24. 确认目标和最终执行器相对水管角均限制在约−5.5495°～+5.5495°，实际PWM始终位于500～1600 μs；实测俯仰绝对值超过7°时应暂时回中。
25. 断开IMU或制造I2C超时，确认舵机回到1050 μs；恢复后连续稳定约150 ms才自动恢复闭环。
26. 遮挡 K230 或发送低质量帧，确认位置外环回到 0°目标，IMU 内环继续主动保持水管水平。
27. 人工确认小球位于 O 点后发送 `5A A5 05 FA`（或 USART1 `SHUTTLE START`）；最新 K230 单帧位置在 0±5 mm 内才启动，否则拒绝并在串口屏提示放回中心。启动后确认水管按“负/正/负”角幅值各持续可调时间、经限速斜坡平滑过渡后回水平，第三段结束报告成功并水平保持；期间 K230 与 IMU 完全忽略，无需 0x04 标定。用 USART1 `SHUTTLE SET` 修改角度/时间，`SHUTTLE STOP` 中止并回水平。
28. 确认 K230 钢球 UART 使用 20 ms 相位节拍，测量实际消息频率、抖动、延迟和质量分布，OSD 仍保持 25 Hz；再依次整定位置外环Kp/Kd、目标误差10～20 mm范围内的Ki和姿态内环Kp/Kd，观察`g_app_ball_integral_active`、`g_app_ball_i_term_deg`及`g_app_ball_pipe_d_term_deg`。第三题已是开环角序列，无端点积分，按需用 `SHUTTLE` 子命令调整三段角度幅值与持续时间。
29. 架空或低附着条件下观察`g_app_motion_command_speed_cm_s`和`g_app_motion_command_accel_cm_s2`，确认启动目标速度连续上升且没有直接跳到40 cm/s。
30. 在直线低速实测中同时记录`g_app_chassis_accel_raw_cm_s2`和`g_app_chassis_accel_cm_s2`，确认滤波值没有编码器单周期尖峰，并重新整定加速度和jerk参数。
31. 第四、五题按键启动后，确认串口屏先显示等待钢球稳定；位置误差 ≤5 mm、球速绝对值 ≤15 mm/s 且连续满足 300 ms 后才开始行驶和计时，单次不稳定样本或视觉间隔超过 200 ms 会重新累计稳定时间。
32. 第四题确认编码器累计 150 cm 时冻结用时并柔和停车；第五题确认驶离起点、累计达到 400 cm 且返程 A 点连续两帧至少 4 路黑线时冻结用时并柔和停车。第四题超过 8 s、第五题超过 30 s 时应显示超时并柔和停车。
33. 在低速直线中人为制造两帧连续超过 10 mm 的钢球误差，确认红圈、超差锁存和最大误差统计生效；单帧毛刺不得把最终结果改为超差。分别制造视觉、IMU、红外和电机故障，确认立即停车并显示具体原因。
34. 270°舵机、单连杆机构、混合前馈、平滑底盘参数和钢球实物稳定性仍需重新验证；编译通过不代表 PWM 标定、机械方向或题目 ±1 cm 指标已经在硬件上成立。
