#ifndef UART_H
#define UART_H

#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart2;

void UART_Init(void);
void UART_Print(const char *fmt, ...);

#endif /* UART_H */
