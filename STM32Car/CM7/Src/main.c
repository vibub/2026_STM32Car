#include "main.h"
#include "app_main.h"
#include "motor_driver.h"

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c4;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart4;

const char * volatile g_error_file;
volatile uint32_t g_error_line;
volatile uint8_t g_boot_step;

static void MPU_Config(void);
static void CPU_CACHE_Enable(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_I2C4_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART4_Init(void);

int main(void)
{
    g_boot_step = 1U;
    MPU_Config();
    CPU_CACHE_Enable();
    HAL_Init();
    g_boot_step = 2U;
    SystemClock_Config();
    g_boot_step = 3U;

    MX_GPIO_Init();
    g_boot_step = 4U;
    MX_TIM1_Init();
    g_boot_step = 5U;
    MX_TIM2_Init();
    g_boot_step = 6U;
    MX_TIM4_Init();
    g_boot_step = 7U;
    MX_TIM8_Init();
    g_boot_step = 8U;
    MX_I2C4_Init();
    g_boot_step = 9U;
    MX_I2C2_Init();
    g_boot_step = 10U;
    MX_USART2_UART_Init();
    g_boot_step = 11U;
    MX_UART4_Init();
    g_boot_step = 12U;
    App_Init();
    g_boot_step = 13U;

    while (1) {
        g_boot_step = 14U;
        App_Loop();
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    if (HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY) != HAL_OK) {
        Error_Handler();
    }
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    while (__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY) == 0U) {
    }

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.CSIState = RCC_CSI_OFF;
    osc.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        Error_Handler();
    }

    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                    RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_PCLK1 |
                    RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    clk.SYSCLKDivider = RCC_SYSCLK_DIV1;
    clk.AHBCLKDivider = RCC_HCLK_DIV1;
    clk.APB3CLKDivider = RCC_APB3_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    clk.APB4CLKDivider = RCC_APB4_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    /*
     * Keep the motor bridge inputs low from the first GPIO setup.  TIM2 later
     * changes PA0/PA2 to alternate function, but its PWM channels are not
     * started until MotorDriver_Set() receives a non-zero command.
     */
    HAL_GPIO_WritePin(GPIOA, MOTOR_RIGHT_DIR_PIN | MOTOR_LEFT_DIR_PIN |
                      MOTOR_RIGHT_PWM_PIN | MOTOR_LEFT_PWM_PIN,
                      GPIO_PIN_RESET);
    gpio.Pin = MOTOR_RIGHT_DIR_PIN | MOTOR_LEFT_DIR_PIN |
               MOTOR_RIGHT_PWM_PIN | MOTOR_LEFT_PWM_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);

}

static void MX_I2C4_Init(void)
{
    RCC_PeriphCLKInitTypeDef clock = {0};

    clock.PeriphClockSelection = RCC_PERIPHCLK_I2C4;
    clock.I2c4ClockSelection = RCC_I2C4CLKSOURCE_HSI;
    if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK) {
        Error_Handler();
    }

    hi2c4.Instance = OLED_I2C_INSTANCE;
    hi2c4.Init.Timing = 0x10D19CE4U;
    hi2c4.Init.OwnAddress1 = 0U;
    hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c4.Init.OwnAddress2 = 0U;
    hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c4) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_I2C2_Init(void)
{
    RCC_PeriphCLKInitTypeDef clock = {0};

    clock.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
    clock.I2c123ClockSelection = RCC_I2C2CLKSOURCE_HSI;
    if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK) {
        Error_Handler();
    }

    hi2c2.Instance = IMU_I2C_INSTANCE;
    hi2c2.Init.Timing = 0x10D19CE4U;
    hi2c2.Init.OwnAddress1 = 0U;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0U;
    hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_TIM1_Init(void)
{
    TIM_Encoder_InitTypeDef encoder = {0};
    TIM_MasterConfigTypeDef master = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0U;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 0xFFFFU;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0U;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    encoder.EncoderMode = TIM_ENCODERMODE_TI12;
    encoder.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    encoder.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    encoder.IC1Prescaler = TIM_ICPSC_DIV1;
    encoder.IC2Prescaler = TIM_ICPSC_DIV1;
    encoder.IC1Polarity = TIM_ICPOLARITY_RISING;
    encoder.IC2Polarity = TIM_ICPOLARITY_RISING;
    encoder.IC1Filter = 0x0FU;
    encoder.IC2Filter = 0x0FU;
    if (HAL_TIM_Encoder_Init(&htim1, &encoder) != HAL_OK) {
        Error_Handler();
    }

    master.MasterOutputTrigger = TIM_TRGO_RESET;
    master.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &master) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_TIM2_Init(void)
{
    TIM_OC_InitTypeDef channel = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0U;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 9999U;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }

    channel.OCMode = TIM_OCMODE_PWM1;
    channel.Pulse = 0U;
    channel.OCPolarity = TIM_OCPOLARITY_HIGH;
    channel.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &channel, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &channel, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_TIM4_Init(void)
{
    TIM_OC_InitTypeDef channel = {0};
    TIM_MasterConfigTypeDef master = {0};

    /* APB1 定时器时钟为 64 MHz，分频后以 1 us 为一个计数。 */
    htim4.Instance = SERVO_PWM_INSTANCE;
    htim4.Init.Prescaler = 63U;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 19999U;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
        Error_Handler();
    }

    master.MasterOutputTrigger = TIM_TRGO_RESET;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &master) != HAL_OK) {
        Error_Handler();
    }

    channel.OCMode = TIM_OCMODE_PWM1;
    channel.Pulse = 0U;
    channel.OCPolarity = TIM_OCPOLARITY_HIGH;
    channel.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &channel,
                                  SERVO_PWM_CHANNEL) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_TIM8_Init(void)
{
    TIM_Encoder_InitTypeDef encoder = {0};
    TIM_MasterConfigTypeDef master = {0};

    htim8.Instance = TIM8;
    htim8.Init.Prescaler = 0U;
    htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim8.Init.Period = 0xFFFFU;
    htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.RepetitionCounter = 0U;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    encoder.EncoderMode = TIM_ENCODERMODE_TI12;
    encoder.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    encoder.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    encoder.IC1Prescaler = TIM_ICPSC_DIV1;
    encoder.IC2Prescaler = TIM_ICPSC_DIV1;
    encoder.IC1Polarity = TIM_ICPOLARITY_RISING;
    encoder.IC2Polarity = TIM_ICPOLARITY_RISING;
    encoder.IC1Filter = 0x0FU;
    encoder.IC2Filter = 0x0FU;
    if (HAL_TIM_Encoder_Init(&htim8, &encoder) != HAL_OK) {
        Error_Handler();
    }

    master.MasterOutputTrigger = TIM_TRGO_RESET;
    master.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &master) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_USART2_UART_Init(void)
{
    RCC_PeriphCLKInitTypeDef clock = {0};

    clock.PeriphClockSelection = RCC_PERIPHCLK_USART2;
    clock.Usart234578ClockSelection = RCC_USART2CLKSOURCE_HSI;
    if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK) {
        Error_Handler();
    }

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 9600U;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_UART4_Init(void)
{
    RCC_PeriphCLKInitTypeDef clock = {0};

    clock.PeriphClockSelection = RCC_PERIPHCLK_UART4;
    clock.Usart234578ClockSelection = RCC_UART4CLKSOURCE_HSI;
    if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK) {
        Error_Handler();
    }

    huart4.Instance = UART4;
    huart4.Init.BaudRate = 115200U;
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.StopBits = UART_STOPBITS_1;
    huart4.Init.Parity = UART_PARITY_NONE;
    huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart4) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK) {
        Error_Handler();
    }
}

static void CPU_CACHE_Enable(void)
{
    SCB_EnableICache();
    SCB_EnableDCache();
}

static void MPU_Config(void)
{
    MPU_Region_InitTypeDef mpu = {0};

    HAL_MPU_Disable();
    mpu.Enable = MPU_REGION_ENABLE;
    mpu.BaseAddress = 0x00U;
    mpu.Size = MPU_REGION_SIZE_4GB;
    mpu.AccessPermission = MPU_REGION_NO_ACCESS;
    mpu.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    mpu.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    mpu.IsShareable = MPU_ACCESS_SHAREABLE;
    mpu.Number = MPU_REGION_NUMBER0;
    mpu.TypeExtField = MPU_TEX_LEVEL0;
    mpu.SubRegionDisable = 0x87U;
    mpu.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    HAL_MPU_ConfigRegion(&mpu);
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void Error_HandlerAt(const char *file, uint32_t line)
{
    g_error_file = file;
    g_error_line = line;
    if (htim2.Instance == TIM2) {
        MotorDriver_Stop();
    }
    __disable_irq();
    while (1) {
    }
}
