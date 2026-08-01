# ABCD 实车遥测

STM32 与 K230 使用现有 UART 全双工连接：

- K230 IO3 TX -> STM32 PB8 RX（原钢球位置通道）；
- STM32 PB9 TX -> K230 IO4 RX（新增 ABCD 遥测通道）；
- 两板共地，115200 8N1。

ABCD 底盘开始运动后，STM32 每 50 ms 发送一帧 44 字节二进制遥测。K230
运行中只把解析结果放在内存中，不写 SD 卡。返程通过 A 点后，STM32 发送结束帧，
K230 才把本圈数据写入：

```text
/sdcard/logs/abcd_0001.csv
/sdcard/logs/abcd_0002.csv
```

CSV 包含轮速、横摆角速度、IMU 横向加速度、`v*gyro_z` 向心加速度、融合结果、
钢球误差和速度、水管角、管壁等效加速度/补偿角及累计里程。`flags` 位定义：

| 位 | 含义 |
|---:|---|
| 0 | 横向估计有效 |
| 1 | 正在转弯 |
| 2 | `accel_x` 静态零偏已完成 |
| 3 | IMU 横向符号已由 `v*gyro_z` 自动确认 |
| 4 | 钢球跟踪有效 |
| 5 | 本圈曾超过允许误差 |
| 6 | 姿态滤波当前信任加速度计 |
| 7 | 管壁补偿通道已进入弯道状态 |

首圈无补偿基线在 B-C 半圆内由 0 漂到约 -50 mm；当前首轮补偿系数为：

```c
#define APP_ABCD_WALL_COUPLING_GAIN 0.020f
```

根据后续两段弯道的 `ball_error_mm`、`lateral_fused_cm_s2` 和
`transport_feedback` 方向继续修改该有符号系数。
取出 SD 卡后可用仓库内脚本给出首个拟合值：

```text
python analyze_abcd_log.py abcd_0001.csv
```

通过 A 点后计分结束，K230 写卡，钢球控制退出，底盘继续按原减速度在 A 前方柔停。
