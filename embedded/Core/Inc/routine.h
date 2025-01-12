#ifndef ROUTINE_H
#define ROUTINE_H

#include "stm32f4xx_hal.h"

void setOutputUart(UART_HandleTypeDef *huart);
void uart_printf(const char* format, ...);
void I2C_Scan(I2C_HandleTypeDef* hi2c);

#endif // ROUTINE_H