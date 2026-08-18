#include "main.h"
#include "clock.h"
#include "gpio.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "spi.h"
#include "uart.h"
#include "ism300dlc.h"
#include "bmp280.h"
#include "control.h"
#include "w25qxx.h"
#include <string.h>

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  GPIO_Init();
  DMA_Clock_Init();
  SPI_Init();
  UART_Init();
  TIM_Init();

  IMU_Init();
  BMP_Init();

  Flash_Selftest();

  TIM_Start();

  uint32_t tick = 0;

  while (1)
  {
    if (!g_loop_flag)
      continue;

    g_loop_flag = 0;
    tick++;

    IMU_Get_Gyro_Out();
    IMU_Get_Accel_Out();
    IMU_Mahony_Filter();

    PID_Control_Loop();

    if ((tick % CONTROL_RATE_HZ) == 0)
      PID_Print();
  }
}

/* TODO */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
