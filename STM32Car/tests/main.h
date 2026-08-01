#ifndef SERVO_DRIVER_TEST_MAIN_H
#define SERVO_DRIVER_TEST_MAIN_H

#include <stdint.h>

typedef struct {
    uint32_t compare;
} TIM_HandleTypeDef;

typedef enum {
    HAL_OK = 0
} HAL_StatusTypeDef;

#define SERVO_PWM_CHANNEL 4U /* 主机测试使用的虚拟PWM通道编号。 */

extern TIM_HandleTypeDef htim4;

static inline HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer,
                                                   uint32_t channel)
{
    (void) timer;
    (void) channel;
    return HAL_OK;
}

#define __HAL_TIM_SET_COMPARE(timer, channel, pulse) \
    ((void) ((timer)->compare = (pulse)), (void) (channel))

void Error_Handler(void);

#endif /* SERVO_DRIVER_TEST_MAIN_H */
