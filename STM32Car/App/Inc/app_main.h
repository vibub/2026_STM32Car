#ifndef APP_MAIN_H
#define APP_MAIN_H

#include <stdint.h>

extern volatile uint8_t g_app_init_step;

#define APP_MODE_STOP       0x00U /* 停车模式编号。 */
#define APP_MODE_BASIC_LINE 0x01U /* 基础八路红外巡线模式编号。 */
#define APP_MODE_AB         0x06U /* 第四项：A 到 B 中心稳定通过。 */
#define APP_MODE_ABCD       0x07U /* 第五项：ABCD 整圈中心稳定通过。 */

void App_Init(void);
void App_Loop(void);
uint8_t App_RequestMode(uint8_t mode);
uint8_t App_GetCurrentMode(void);
uint8_t App_StartSmoothLineRun(float completion_distance_cm);
uint8_t App_StartSmoothAbRun(void);
uint8_t App_StartSmoothLapRun(void);
uint8_t App_StartAbRun(void);
uint8_t App_StartAbcdRun(void);
uint8_t App_SetBallBalanceEnabled(uint8_t enabled);
uint8_t App_GetBallBalanceEnabled(void);
uint8_t App_StartBallShuttle(void);
uint8_t App_StartPipeCalibration(void);
uint8_t App_GetPipeCalibrationActive(void);
uint8_t App_GetPipeCalibrationValid(void);
void App_SetBallBalanceTargetMm(float target_mm);

#endif /* APP_MAIN_H */
