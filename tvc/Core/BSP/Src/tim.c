#include "tim.h"
#include "main.h"
#include <stdint.h>
#include "uart.h"

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;
uint32_t g_elapsed_time = 0;

volatile uint8_t  g_loop_flag = 0;
volatile uint32_t g_overrun_count = 0;

void TIM2_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = (TIM2_CLK_HZ / TIM2_TICK_HZ) - 1;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = (TIM2_TICK_HZ / SERVO_FRAME_HZ) - 1;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    /* Start at a valid pulse rather than 0us. NOTE: this is a generic 1500us,
       NOT either axis's calibrated trim - the two axes trim to different
       widths, so the gimbal sits off-neutral from power-on until the first
       PID_Control_Loop tick writes the real values. If that startup jump
       loads the gimbal against a stop, set each channel's Pulse to its own
       SERVO_*_TRIM instead. */
    sConfigOC.Pulse = SERVO_CENTER_US;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_TIM_MspPostInit(&htim2);
}

void TIM5_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim5.Instance = TIM5;
    htim5.Init.Prescaler = (TIM5_CLK_HZ / TIM5_TICK_HZ) - 1;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = (TIM5_TICK_HZ / CONTROL_RATE_HZ) - 1;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

void TIM_Init(void)
{
    TIM2_Init();
    TIM5_Init();
}

void TIM_Start(void)
{
    HAL_TIM_PWM_Start(&htim2, SERVO_YAW_CH);
    HAL_TIM_PWM_Start(&htim2, SERVO_PITCH_CH);

    HAL_TIM_Base_Start_IT(&htim5);
}

/* Sets a servo pulse width in microseconds. Clamps to the absolute servo
   limits so a runaway control output can't drive the horn past its travel. */
void TIM_Set_Servo_Us(uint32_t channel, uint32_t pulse_us)
{
    if (pulse_us < SERVO_MIN_US)
        pulse_us = SERVO_MIN_US;
    else if (pulse_us > SERVO_MAX_US)
        pulse_us = SERVO_MAX_US;

    __HAL_TIM_SET_COMPARE(&htim2, channel, pulse_us);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM5) {
        /* flag still set means the main loop never finished the last iteration */
        if (g_loop_flag)
            g_overrun_count++;

        g_loop_flag = 1;
    }
}