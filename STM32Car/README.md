# STM32Car 基础巡线固件

本目录是从 MSPM0 巡线程序移植出的 STM32H745ZI CM7-only 固件，使用 IAR EWARM 8.3 和 STM32H7 HAL。

## 当前功能

- 四路红外输入：YH-4LXJ V1.1 的 X1～X4 暂接 PD8～PD11，按数字电平读取。
- 基础红外巡线：四路加权误差经过低通、PD、限幅和斜率限制后生成左右轮速度目标。
- 双轮速度闭环：20 ms 编码器采样，左右轮独立 PID，包含积分限幅、启动补偿和编码器反馈故障检测。
- 安全停车：持续丢线或编码器反馈故障时清空速度控制器并停止电机。
- K230 红线协议和 UART4 底层文件继续保留，但当前 App 不以 K230 帧作为巡线输入。
- LSM6DSV16X：已预留平台接口，等待 STM32 侧 I2C/SPI 接线确认后接入。

未迁移蓝牙、CCD、TJC、云台、送药、路口、路线规划和模式状态机。

## 硬件映射

| 功能 | 外设与引脚 |
|---|---|
| 右电机 PWM | TIM2_CH1 / PA0 |
| 右电机方向 | PA1 |
| 左电机 PWM | TIM2_CH3 / PA2 |
| 左电机方向 | PA3 |
| 右编码器 | TIM1_CH1 PA8、TIM1_CH2 PE11 |
| 左编码器 | TIM8_CH1 PC6、TIM8_CH2 PC7 |
| K230 UART | UART4 TX PB9、RX PB8 |
| 红外 X1～X4 | PD8、PD9、PD10、PD11（核心板 JP2-35、JP2-33、JP2-31、JP2-29） |

红外模块建议使用 3.3V 供电并与 STM32 共地。初始软件按黑线有效高电平处理；实际接线后应先观察 `g_app_line_sensor_mask`，必要时修改 `INFRARED_LINE_ACTIVE_HIGH`。

电机 PWM 约为 6.4 kHz。首版最大 PID 输出限制为 50%，基础巡线速度为 15 cm/s。

## IAR 构建

打开：

```text
EWARM/STM32Car.eww
```

命令行构建：

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
  Protocol/Src/k230_line_protocol.c \
  Control/Src/line_follow_control.c \
  Control/Src/infrared_line_control.c \
  Control/Src/wheel_speed_pid.c \
  -o tests/test_basic_control.exe
./tests/test_basic_control.exe
```

## 上车前必须验证

1. 烧录前清空 CM4 所在的 Flash Bank 2，或在 Option Bytes 中关闭 CM4 启动，避免旧 CM4 固件继续占用外设。
2. 架空测试左右电机正反转和停止电平组合。
3. 确认车辆向前时 TIM1/TIM8 编码器速度均为正。
4. 实测轮子一圈的编码器计数，校准当前 56000 counts/rev 参数。
5. 不启动电机，逐个遮挡 X1～X4，确认 `g_app_line_sensor_mask` 的位序和黑/白有效电平。
6. 架空低速测试，确认左侧传感器压线时右轮目标大于左轮目标，右侧压线时方向相反。
7. 断开或抬起红外模块，确认持续丢线后电机可靠停止。
8. 低速落地测试前确认模块输出高电平不超过 3.3V。
9. 后续为 IMU 选择 I2C 引脚时避开已被 UART4 占用的 PB8/PB9。
