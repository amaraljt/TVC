#ifndef LCD_DRIVER_H
#define LCD_DRIVER_H

#include "stm32f446xx.h"
#include "main.h"

void lcd_send_cmd(char cmd);
void lcd_send_data(char data);
void lcd_init(I2C_HandleTypeDef *hi2c1);
void lcd_send_string(char *str);
void lcd_put_cur(uint8_t row, uint8_t col);
void lcd_clear(void);

#endif /* LCD_DRIVER_H */