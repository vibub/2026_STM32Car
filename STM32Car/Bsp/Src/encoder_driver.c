#include "encoder_driver.h"
#include "main.h"

#define ENCODER_WHEEL_DIAMETER_CM 6.5f     /* 驱动轮有效滚动直径，单位为厘米。 */
#define ENCODER_COUNTS_PER_REV    56000.0f /* 车轮旋转一圈对应的编码器计数。 */
#define ENCODER_SAMPLE_SECONDS    0.020f   /* 编码器速度采样周期，单位为秒。 */
#define ENCODER_LEFT_SIGN         (-1.0f)  /* 左编码器计数方向修正系数。 */
#define ENCODER_RIGHT_SIGN        (-1.0f)  /* 右编码器计数方向修正系数。 */
#define ENCODER_COUNT_TO_CM \
    ((3.14159265f * ENCODER_WHEEL_DIAMETER_CM) / ENCODER_COUNTS_PER_REV) /* 单个编码器计数对应的轮距，单位为厘米。 */

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
