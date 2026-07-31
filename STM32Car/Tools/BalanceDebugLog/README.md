# 钢球平衡诊断日志

该工具用于记录同一时刻的小球位置与速度、位置外环各控制项、水管IMU姿态、姿态内环、执行器相对水管角和实际PWM脉宽，定位“水管倾斜角度不够”究竟来自外环目标、姿态跟随、线性PWM映射还是安全限幅。

## 固件配置

[app_config.h](../../App/Inc/app_config.h) 中：

```c
#define APP_ENABLE_BALANCE_LOG 1U
```

启用后在 AXI SRAM 中分配约 98 KiB 日志缓冲区，按照 20 ms 周期保存 500 点，即10秒。比赛固件确认不再需要采集时可改为 `0U` 释放 RAM。

IAR 可观察以下全局变量：

| 变量 | 含义 |
|---|---|
| `g_balance_debug_log_command` | 写 `1` 清空计数并开始，写 `2` 提前停止；固件处理后自动清零 |
| `g_balance_debug_log.count` | 已保存样本数，最大500 |
| `g_balance_debug_log.state` | `0`空闲、`1`采集中、`2`已采满、`3`手动停止 |
| `g_balance_debug_log.buffer_size` | 从结构体首地址需要导出的总字节数，当前为98032 |

日志只写 RAM，不通过 TJC 或 K230 串口发送，不会阻塞控制循环。视觉或 IMU 失效后仍继续记录到10秒结束，CSV 中的状态列会标出失效时刻。

## 首轮测试步骤

先执行0x04单点水平标定，确认1050 us时水管水平且500～1600 us实测范围无碰撞。底盘保持停车，左右各采一次：

### 从 +50 mm 释放

1. 发送 `0x03` 禁用平衡，等待水管回到水平。
2. 把小球放在约 `+50 mm` 标记处并暂时扶住。
3. 在 IAR Watch 中把 `g_balance_debug_log_command` 改为 `1`。
4. 继续运行程序，确认 `g_balance_debug_log.state == 1`。
5. 发送 `0x02` 启用中心平衡；屏幕显示平衡已启用后立即松开小球。
6. 等待 `g_balance_debug_log.state == 2`、`count == 500`。
7. 暂停程序并导出为 `plus50.bin`。

### 从 -50 mm 释放

重新发送 `0x03`，等待水管回平后按相同步骤从 `-50 mm` 释放，导出为 `minus50.bin`。开始第二轮采集前必须先保存第一轮文件，因为写 `1` 会从下标0重新覆盖。

如果小球有脱轨风险，可以在 IAR Watch 中把 `g_balance_debug_log_command` 写为 `2` 提前冻结，然后立即人工接球；安全优先于收满500点。

## IAR 内存导出

日志采满或手动停止后：

1. 暂停 CPU。
2. 在 Memory 窗口定位到 `&g_balance_debug_log`。
3. 从该地址开始保存 `g_balance_debug_log.buffer_size` 字节。
4. 当前格式总长度是 `98032` 字节。
5. 可使用 Raw Binary、Intel HEX 或 Motorola S-record 格式。

必须从结构体头部开始导出，不能只导出 `samples` 数组，否则转换脚本无法检查版本、有效点数和样本大小。

## 转换为 CSV

在 `STM32Car` 目录执行：

```bash
python Tools/BalanceDebugLog/export_balance_log.py plus50.bin -o plus50.csv
python Tools/BalanceDebugLog/export_balance_log.py minus50.bin -o minus50.csv
```

把两个 CSV 一起反馈即可。不要只截图曲线，因为完整数据需要检查：

- 原始、滤波和预测位置是否同向；
- 速度估计是否过小或延迟；
- `P/I/D/brake` 哪一项限制了外环角度；
- `desired_pipe_pitch_deg` 是否已经碰到上下限；
- `command_pipe_pitch_deg` 是否受到变化率限制；
- IMU 实测角是否能跟上命令角；
- 姿态内环 `P/D` 输出是否足够；
- 执行器相对水管角和实际PWM是否到达预期；
- K230 丢帧、低质量帧或 IMU 不可信是否与异常同时发生。

CSV 中每一行都是同一20 ms时刻的完整快照。`measurement_accepted` 只在控制器接受新 K230 数据时为1；由于 K230 约40 ms上报一次，相邻两行出现相同 `ball_sequence` 是正常现象。
