#include "encoder_driver.h"
#include "main.h"

#define ENCODER_WHEEL_DIAMETER_CM 6.5f
#define ENCODER_COUNTS_PER_REV    56000.0f
#define ENCODER_SAMPLE_SECONDS    0.020f
#define ENCODER_LEFT_SIGN         (-1.0f)
#define ENCODER_RIGHT_SIGN        (-1.0f)
#define ENCODER_COUNT_TO_CM \
    ((3.14159265f * ENCODER_WHEEL_DIAMETER_CM) / ENCODER_COUNTS_PER_REV)

void EncoderDriver_Init(void)
{
    __HAL_TIM_SET_COUNTER(&htim1, 0U);
    __HAL_TIM_SET_COUNTER(&htim8, 0U);
    if (HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL) != HAL_OK) {
        Error_Handler();
    }
}

void EncoderDriver_Sample(EncoderSample *sample)
{
    int16_t left_raw;
    int16_t right_raw;

    if (sample == 0) {
        return;
    }

    /* 20 ms 内计数远小于 int16_t 范围，强制转换可自然处理 16 位回绕。 */
    left_raw = (int16_t) __HAL_TIM_GET_COUNTER(&htim8);
    right_raw = (int16_t) __HAL_TIM_GET_COUNTER(&htim1);
    __HAL_TIM_SET_COUNTER(&htim8, 0U);
    __HAL_TIM_SET_COUNTER(&htim1, 0U);

    sample->left_counts = (int16_t) ((float) left_raw * ENCODER_LEFT_SIGN);
    sample->right_counts = (int16_t) ((float) right_raw * ENCODER_RIGHT_SIGN);
    sample->left_cm_s = ((float) sample->left_counts * ENCODER_COUNT_TO_CM) /
                        ENCODER_SAMPLE_SECONDS;
    sample->right_cm_s = ((float) sample->right_counts * ENCODER_COUNT_TO_CM) /
                         ENCODER_SAMPLE_SECONDS;
}
