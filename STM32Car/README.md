# STM32Car 固件

本目录是 STM32H745ZI CM7-only 智能车底盘固件，使用 IAR EWARM 8.3 和 STM32H7 HAL。

## 当前功能

- TJC 触摸屏通过 USART2 通信：TX 为 PD5，RX 为 PD6，115200 8N1。
- TJC 停止命令 `5A A5 00 FF` 进入停车模式。
- TJC 巡线命令 `5A A5 0D F2` 进入四路红外巡线模式。
- TJC 查询命令 `5A A5 0E F1` 返回当前工作模式。
- K230 UART4 通讯底层继续保留：STM32 TX 为 PB9，RX 为 PB8，115200 8N1。
- 四路红外输入：YH-4LXJ V1.1 的 X1～X4 暂接 PD8～PD11，按数字电平读取。
- 基础红外巡线：四路加权误差经过低通、PD、限幅和斜率限制后生成左右轮速度目标。
- 双轮速度闭环：20 ms 编码器采样，左右轮独立 PID，包含积分限幅、启动补偿和编码器反馈故障检测。
- 安全停车：停止模式、持续丢线或编码器反馈故障时清空速度控制器并停止电机。
- LSM6DSV16X：已预留平台接口，等待 STM32 侧 I2C/SPI 接线确认后接入。

K230 UART4 的字节接收通讯仍在正式工程中保留，但 K230 红线协议和视觉巡线控制已移除；当前巡线输入为四路红外传感器。

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
| 红外 X1 | PD8 / 核心板 JP2-35 |
| 红外 X2 | PD9 / 核心板 JP2-33 |
| 红外 X3 | PD10 / 核心板 JP2-31 |
| 红外 X4 | PD11 / 核心板 JP2-29 |

红外模块建议使用 3.3V 供电并与 STM32 共地。初始软件按黑线有效高电平处理；实际接线后应先观察 `g_app_line_sensor_mask`，必要时修改 `INFRARED_LINE_ACTIVE_HIGH`。

如果 X1～X4 的实际安装方向与软件约定相反，修改 `INFRARED_LINE_REVERSE_ORDER`。

电机 PWM 约为 6.4 kHz。首版最大 PID 输出限制为 50%，基础巡线速度为 15 cm/s。电机驱动仍使用参考工程要求的特殊 PWM/DIR 电气组合，不能擅自改成普通方向脚控制。

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
| `0x00` | `5A A5 00 FF` | 停止 |
| `0x0D` | `5A A5 0D F2` | 四路红外巡线 |
| `0x0E` | `5A A5 0E F1` | 查询当前模式 |

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
  -o tests/test_basic_control.exe
./tests/test_basic_control.exe
```

## 上车前必须验证

1. 烧录前清空 CM4 所在的 Flash Bank 2，或在 Option Bytes 中关闭 CM4 启动，避免旧 CM4 固件继续占用外设。
2. 红外模块使用 3.3V 供电，确认输出高电平不会超过 STM32 的 3.3V 输入范围。
3. 不启动电机，逐个遮挡 X1～X4，确认 `g_app_line_sensor_mask` 的位序和黑/白有效电平。
4. 必要时修改 `INFRARED_LINE_ACTIVE_HIGH` 或 `INFRARED_LINE_REVERSE_ORDER`。
5. 架空测试左右电机正反转和停止电平组合。
6. 确认车辆向前时 TIM1/TIM8 编码器速度均为正。
7. 实测轮子一圈的编码器计数，校准当前 counts/rev 参数。
8. 架空低速测试，确认左侧传感器压线时右轮目标大于左轮目标，右侧压线时方向相反。
9. 断开或抬起红外模块，确认持续丢线后电机可靠停止。
10. 后续为 IMU 选择 I2C 引脚时避开 TJC USART2 的 PD5/PD6 和 K230 UART4 的 PB8/PB9。
