#include "main.h"
#include "clock.h"
#include "gpio.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "spi.h"
#include "uart.h"
#include "ism300dlc.h"
#include "bmp280.h"

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  GPIO_Init();
  SPI_Init();
  UART_Init();
  TIM_Init();

  //IMU_Init();
  BMP_Init();

  while (1)
  {
    //IMU_Get_Gyro_Out();
    //IMU_Get_Accel_Out();
    //IMU_Print();

    BMP_Get_Baro_Out();
    BMP_Print();
    HAL_Delay(500);
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
