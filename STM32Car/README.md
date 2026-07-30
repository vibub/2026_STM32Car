# STM32Car 固件

本目录是 STM32H745ZI CM7-only 智能车底盘固件，使用 IAR EWARM 8.3 和 STM32H7 HAL。

## 当前功能

- TJC 触摸屏通过 USART2 通信：TX 为 PD5，RX 为 PD6，115200 8N1。
- TJC 停止命令 `5A A5 00 FF` 进入停车模式并禁用钢球闭环。
- TJC 基础巡线命令 `5A A5 01 FE` 进入八路 I²C 红外巡线模式。
- TJC 钢球平衡启用命令 `5A A5 02 FD`，禁用命令 `5A A5 03 FC`。
- TJC 查询命令 `5A A5 0E F1` 返回当前底盘工作模式。
- K230 UART4：STM32 TX 为 PB9，RX 为 PB8，115200 8N1，接收钢球位置 ASCII 消息。
- 八路红外模块通过 I2C4 读取：SCL 为 PF14，SDA 为 PF15，设备地址为 `0x12`。
- 基础红外巡线：八路物理位置误差经过低通、PD、限幅和斜率限制后生成左右轮速度目标。
- 双轮速度闭环：20 ms 编码器采样，左右轮独立 PID，包含积分限幅、启动补偿和编码器反馈故障检测。
- 钢球平衡：解析 K230 位置、质量和序号，使用滤波 PD/PID 生成受限 SG90 舵机角度命令。
- 舵机 PWM：TIM4_CH4 / PD15，50 Hz；上电保持 90°中位且闭环默认禁用。
- 安全停车：停止模式、持续丢线或编码器反馈故障时清空速度控制器并停止电机。
- LSM6DSV16X：已预留平台接口，等待 STM32 侧 I2C/SPI 接线确认后接入。

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
| SG90 舵机 PWM | TIM4_CH4 / PD15 / 50 Hz |

八路红外模块使用 5V 供电、与 STM32 共地；实测 SDA、SCL 空闲电压均为 3.3V。模块 7 位 I²C 地址为 `0x12`，状态寄存器为 `0x30`，黑线对应位为 0；实测取反后的位序仍与车辆左右相反，驱动会再次反转八位，统一为 bit0 在车辆左侧、bit7 在车辆右侧。

OLED 与八路红外模块共用 I2C4，OLED 地址为 `0x3C`。模块上电后应等待至少 20 秒并按实际安装高度完成黑白校准，固件不强制等待。

电机 PWM 约为 6.4 kHz。最大 PID 输出限制为 50%，当前基础巡线速度由 `APP_BASE_SPEED_CM_S` 配置。电机驱动仍使用参考工程要求的特殊 PWM/DIR 电气组合，不能擅自改成普通方向脚控制。

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
| `0x03` | `5A A5 03 FC` | 禁用钢球平衡并回中位 |
| `0x0E` | `5A A5 0E F1` | 查询当前底盘模式 |

固件通过 USART2 返回 5 字节结果帧：

```text
A5 <result> <current_mode> <request> <xor>
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
  Control/Src/ball_balance_control.c \
  Protocol/Src/ball_position_protocol.c \
  -o tests/test_basic_control.exe
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
15. 不连接齿轮和钢球时上电，确认舵机保持 90°中位且不会自动扫动。
16. 空载确认舵机闭环命令始终限制在 85°～95°，并校准实际水平中位和机械安全范围。
17. 手动注入正负钢球位置，确认舵机控制方向为负反馈；方向错误时禁止装球运行。
18. 测量 K230 钢球消息频率、延迟和质量分布，再整定位置滤波、Kp 和 Kd。
19. 确认停止或视觉数据超时后舵机平滑回到水平中位。
20. SG90、齿轮和钢球实物稳定性未经过验证，编译通过不代表达到题目 ±1 cm 指标。
21. 后续为 IMU 选择 I2C/SPI 引脚时避开当前 I2C4、TJC USART2 的 PD5/PD6 和 K230 UART4 的 PB8/PB9。
