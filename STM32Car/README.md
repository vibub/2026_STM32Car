# STM32Car 固件

本目录是 STM32H745ZI CM7-only 智能车底盘固件，使用 IAR EWARM 8.3 和 STM32H7 HAL。

## 当前功能

- TJC 触摸屏通过 USART2 通信：TX 为 PD5，RX 为 PD6，115200 8N1。
- TJC 停止命令 `5A A5 00 FF` 进入停车模式并禁用钢球闭环。
- TJC 基础巡线命令 `5A A5 01 FE` 进入八路 I²C 红外巡线模式。
- TJC 钢球平衡启用命令 `5A A5 02 FD`，禁用命令 `5A A5 03 FC`。
- TJC 水管 IMU 和连杆模型自动三点标定命令 `5A A5 04 FB`。
- TJC 查询命令 `5A A5 0E F1` 返回当前底盘工作模式。
- K230 UART4：STM32 TX 为 PB9，RX 为 PB8，115200 8N1，接收钢球位置 ASCII 消息。
- 八路红外模块通过 I2C4 读取：SCL 为 PF14，SDA 为 PF15，设备地址为 `0x12`。
- 基础红外巡线：八路物理位置误差经过低通、PD、限幅和斜率限制后生成左右轮速度目标。
- 双轮速度闭环：20 ms 编码器采样，左右轮独立 PID，包含积分限幅、启动补偿和编码器反馈故障检测。
- 钢球平衡：K230 钢球位置外环生成名义 ±3°目标水管俯仰角，再根据柔性水管标定结果自动裁剪到连杆模型实际可实现范围；IMU 姿态前馈＋PD 内环经完整非线性连杆模型生成受限舵机命令。
- 水管姿态：IMU 刚性固定在水管远离原转轴安装点的另一端，+Y 轴沿水管指向钢球、+Z 轴向上；使用 gyro X 和 accel Y/Z 估计该端实际俯仰，并由三点标定自动处理相对原安装方向的符号反转。
- 自动标定：停车且移除钢球后按模型完整行程执行 −6°→0°→+6°→0°流程，计算 IMU 零偏、方向、模型比例 `scale`、零偏 `offset` 和回中误差。
- 舵机 PWM：TIM4_CH4 / PD15，50 Hz；500～2500 μs 对应 0°～270°，实机反向安装下名义水平命令为 126.206°，连杆装配后始终限制在 100°～170°。
- 安全停车：停止模式、持续丢线或编码器反馈故障时清空速度控制器并停止电机。
- LSM6DSV16X：通过 I2C2（PB10/PB11）接入，自动探测 `0x6A`/`0x6B` 并读取三轴陀螺仪和三轴加速度计。

K230 红线协议和视觉巡线控制已移除，底盘巡线输入为八路 I²C 红外传感器；K230 UART4 当前用于接收 `B,<valid>,<position_d10_mm>,<quality>,<sequence>` 钢球位置消息。

## 硬件映射

| 功能 | 外设与引脚 |
|---|---|
| TJC UART TX | USART2_TX / PD5 |
| TJC UART RX | USART2_RX / PD6 |
| K230 UART TX | UART4_TX / PB9 |
| K230 UART RX | UART4_RX / PB8 |
| 右电机 PWM | TIM2_CH1 / PA0 |
| 右电机方向 | PA1 |
| 左电机 PWM | TIM2_CH3 / PA2 |
| 左电机方向 | PA3 |
| 右编码器 | TIM1_CH1 / PA8、TIM1_CH2 / PE11 |
| 左编码器 | TIM8_CH1 / PC6、TIM8_CH2 / PC7 |
| 红外 I2C4 SCL | PF14 |
| 红外 I2C4 SDA | PF15 |
| 270°位置舵机 PWM | TIM4_CH4 / PD15 / 50 Hz，500～2500 μs |
| LSM6DSV16X SCL | I2C2_SCL / PB10 |
| LSM6DSV16X SDA | I2C2_SDA / PB11 |

八路红外模块使用 5V 供电、与 STM32 共地；实测 SDA、SCL 空闲电压均为 3.3V。模块 7 位 I²C 地址为 `0x12`，状态寄存器为 `0x30`，黑线对应位为 0；实测取反后的位序仍与车辆左右相反，驱动会再次反转八位，统一为 bit0 在车辆左侧、bit7 在车辆右侧。

OLED 与八路红外模块共用 I2C4，OLED 地址为 `0x3C`。模块上电后应等待至少 20 秒并按实际安装高度完成黑白校准，固件不强制等待。

LSM6DSV16X 独占 I2C2，固件会依次探测 7 位地址 `0x6A` 和 `0x6B`，期望 `WHO_AM_I` 为 `0x70`。水管姿态使用 X 轴陀螺仪积分和 Y/Z 加速度重力角互补融合；底盘加减速时使用编码器估计加速度降低 Accel 融合权重。单个水管 IMU 仍无法完全区分持续车体加速度与重力倾角。

电机 PWM 约为 6.4 kHz。最大 PID 输出限制为 50%，当前基础巡线速度由 `APP_BASE_SPEED_CM_S` 配置。电机驱动仍使用参考工程要求的特殊 PWM/DIR 电气组合，不能擅自改成普通方向脚控制。

## 连杆模型

水管俯仰与 270°位置舵机内部角之间使用完整闭式运动学，而不是固定线性比例。名义尺寸、正反解公式、可达条件、标定表和装配步骤见 [连杆数学模型](Docs/linkage/linkage-math-model.md)。

固件使用模型加号装配分支。实机验证发现舵机采用 `s=-1`反向安装：水管右端抬高定义为正俯仰，此时舵机内部角增大。模型校正采用：

```text
实际水管角 = scale × 模型输入角 + offset
模型输入角 = (目标实际角 - offset) / scale
可实现下限 = max(-3°, scale × -6° + offset)
可实现上限 = min(+3°, scale × +6° + offset)
```

柔性水管允许的 `scale` 范围为 0.35～1.25。钢球外环、姿态内环和模型输入都会使用上述动态上下限，避免低比例标定时把目标放大到模型 ±6°之外。以 `scale=0.42`、`offset=+0.38°` 为例，可实现范围约为 −2.14°～+2.90°。

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
| `0x02` | `5A A5 02 FD` | 启用中心钢球平衡 |
| `0x03` | `5A A5 03 FC` | 禁用钢球平衡并回中位；标定中收到时立即中断 |
| `0x04` | `5A A5 04 FB` | 停车模式下启动水管 IMU 和连杆模型自动三点标定 |
| `0x0E` | `5A A5 0E F1` | 查询当前底盘模式 |

固件通过 USART2 返回 5 字节结果帧：

```text
A5 <result> <current_mode> <request> <xor>
```

`0x04` 被接受后先返回上述结果帧，后续标定进度通过淘晶驰指令异步更新全局文本控件 `page2.t0`：

```text
page2.t0.txt="<UTF-8 中文状态>" FF FF FF
```

标定状态包括准备、水平零偏采样、模型 −6°采样、0°复核、+6°采样、回中检查、成功和失败原因。舵机以 30°/s 移动，到达每个模型命令并静止后直接平均加速度重力角，避免互补滤波滞后低估标定行程；成功时显示模型校正比例 `S` 和零偏 `O`，行程验收失败时显示 `S`、`O` 和回中误差 `R`。每次上电必须先成功执行 `0x04`，才允许在停车模式下发送 `0x02`；`0x02` 会先复核约 500 ms 的水平和静止状态，再进入平衡。

推荐比赛启动顺序：

```text
移除钢球并确认摆杆无阻挡
    → 0x04 等待标定成功
    → 放置钢球
    → 0x02 等待平衡已启用
    → 0x01 开始巡线
```

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
  Protocol/Src/ball_position_protocol.c \
  -lm -o tests/test_basic_control.exe
./tests/test_basic_control.exe
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
12. 第二题实车验证时，确认累计里程达到约 400 cm 前全黑不会停车。
13. 返回 A 点连续 2 个有效采样均检测到至少 5 路黑线后，确认电机输出立即清零并冻结 OLED 计时。
14. 记录 `MotorDriver_Stop()` 实际造成的停车过冲；本功能当前未加入主动制动和停车距离补偿。
15. 移除钢球并确认连杆无干涉后上电，确认舵机保持实机反向安装下的名义水平命令约 126.206°且不会自动扫动。
16. 确认 IMU 刚性固定在水管另一端并随钢球所在管段同步转动，不能装在会与管端产生弯曲角差的转轴附近；保持 +Y 朝钢球、+Z 朝上，SDA/SCL 空闲电压为 3.3 V。
17. 连接 IMU 后确认 `g_imu_ready=1`、`g_imu_who_am_i=0x70`，且 `g_imu_i2c_address` 为 `0x6A` 或 `0x6B`。
18. 移除钢球并确认摆杆无阻挡，在停车模式发送 `5A A5 04 FB`；观察舵机按模型完整目标 −6°→0°→+6°→0°缓速运行，其中 −6°应使水管右端下沉、+6°应使右端抬高；对应舵机角约为 102.083°和167.917°，任何时候发送 0x00/0x03 都应中断采样并缓速回水平。
19. 在 USART HMI 工程中把 `page2.t0` 的 `vscope` 设为“全局”、`txt_maxl` 设为至少 64，并保持工程为 UTF-8 编码；重新编译下载屏幕工程后，确认标定进度能够显示。成功后 `g_app_pipe_calibration_valid=1`，`g_app_pipe_model_scale` 在 0.35～1.25、`g_app_pipe_model_offset_deg` 绝对值不超过 0.75°，方向和四个标定角合理；通过 `g_app_ball_minimum_pipe_pitch_deg` 和 `g_app_ball_maximum_pipe_pitch_deg` 检查实际控制范围。
20. 标定失败、复位或尚未执行 0x04 时发送 0x02，确认 STM32 拒绝启用钢球平衡。
21. 放置钢球后发送 0x02，确认水管在约 500 ms 水平复核期间保持中位，随后显示“平衡已启用”。
22. 空载手动抬高水管右端，确认 `g_app_pipe_pitch_deg` 为正，`g_app_pipe_pitch_rate_dps` 动态方向正确。
23. 手动注入正负钢球位置：正位置应产生正目标水管角，使右端抬高并推动钢球向左回中心；方向错误时禁止装球运行。
24. 确认名义目标水管角不超过 ±3°，并进一步限制在标定后可实现范围内；连杆模型输入保持在 ±6°、舵机内部命令保持在 100°～170°，实测俯仰超过 ±6°时应暂时回中。
25. 断开 IMU 或制造 I2C 超时，确认舵机回中；恢复后连续稳定约 500 ms 才自动恢复闭环。
26. 遮挡 K230 或发送低质量帧，确认位置外环回到 0°目标，IMU 内环继续主动保持水管水平。
27. 测量 K230 钢球消息频率、延迟和质量分布，再依次整定位置外环 Kp/Kd 和姿态内环 Kp/Kd，最后才考虑 Ki。
28. 270°舵机、单连杆机构和钢球实物稳定性仍需重新验证；编译通过不代表 PWM 标定、机械方向或题目 ±1 cm 指标已经在硬件上成立。
