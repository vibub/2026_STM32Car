# 50 点水管标定 CSV 导出

固件仍使用原来的触屏“标定”入口和状态机，只是把原来 9 个目标点在相同范围内细分成 50 个目标点，并精确保留舵机 122 度的水平点。每个点完成稳定和 IMU 采样后，调试变量中会同时保留：

- `g_app_pipe_mapping_pitch_deg[50]`：实测 IMU 俯仰角，`float32`，共 200 字节。
- `g_app_pipe_mapping_pwm_us[50]`：该点舵机 PWM 脉宽，`uint16_t`，共 100 字节。
- `g_app_pipe_mapping_servo_angle_deg[50]`：对应舵机内部角，便于现场检查。

## 使用步骤

1. 烧录固件并点击原来的“标定”，等待 `g_app_pipe_calibration_state` 进入成功状态。50 点完整采集预计需要约 1.5～2 分钟。
2. 在 IAR 调试器中暂停程序，分别把下面两段连续内存按 **motorola** 格式保存：
   - 从 `g_app_pipe_mapping_pwm_us` 地址开始保存 100 字节为 `pwm.srec`。
   - 从 `g_app_pipe_mapping_pitch_deg` 地址开始保存 200 字节为 `imu.srec`。
3. 在 `STM32Car` 目录执行：

```powershell
python Tools\PipeCalibrationCsv\export_calibration.py pwm.srec imu.srec `
  --output pipe_calibration_50.csv
```

生成的 CSV 列为：

```text
index,pwm_us,duty_percent,servo_angle_deg,imu_pitch_deg
```

脚本会自动识别 IAR 的 Motorola S-record、Intel HEX 和 Raw Binary，再按 STM32H7 的小端格式读取数据，并检查文件大小、PWM 范围以及 IMU 数值是否合法。CSV 使用 UTF-8 BOM，Excel 可直接打开。
