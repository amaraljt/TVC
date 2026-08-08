#ifndef TIM_H
#define TIM_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define TIM5_CLK_HZ       16000000UL   /* APB1 timer clock */
#define TIM5_TICK_HZ      1000000UL    /* 1 tick = 1us, so CNT reads directly in us */
#define CONTROL_RATE_HZ   400UL        /* matches the IMU's 416Hz ODR */
#define CONTROL_DT        (1.0f / (float)CONTROL_RATE_HZ)

/* Read the barometer every Nth control tick (400Hz / 16 = 25Hz, and the BMP280
   only produces a new sample every ~43ms at osrs_p=x16 anyway) */
#define BARO_DIVIDER      16

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;

extern volatile uint8_t  g_loop_flag;
extern volatile uint32_t g_overrun_count;

void TIM2_Init(void);
void TIM5_Init(void);
void TIM_Init(void);
void TIM_Start(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif /* TIM_H */
