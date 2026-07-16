#ifndef SPI_H
#define SPI_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

extern SPI_HandleTypeDef hspi1;

/* GPIO Control */
void SPI_CS_Low(GPIO_TypeDef *port, uint16_t pin);
void SPI_CS_High(GPIO_TypeDef *port, uint16_t pin);

/* SPI Transfer */
uint8_t SPI_Send(uint8_t reg, uint8_t data, uint8_t read);
void SPI_BurstRead(GPIO_TypeDef *port, uint16_t pin, uint8_t reg, uint8_t *buf, uint8_t len);

void SPI_Init(void);

#endif /* SPI_H */
