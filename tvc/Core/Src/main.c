#include "main.h"
#include "clock.h"
#include "gpio.h"
#include "tim.h"
#include "spi.h"
#include "uart.h"

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  GPIO_Init();
  SPI_Init();
  UART_Init();
  TIM_Init();

  while (1)
  {
    // PID Loop
    
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
