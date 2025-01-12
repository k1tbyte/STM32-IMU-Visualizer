#include "routine.h"
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef *outputUart = NULL;

void setOutputUart(UART_HandleTypeDef *huart) {
    outputUart = huart;
}

void uart_printf(const char* format, ...) {
    va_list args;
    va_start(args, format);

    int bufferSize = vsnprintf(NULL, 0, format, args) + 3;
    va_end(args);

    char* buffer = (char*)malloc(bufferSize);
    if (buffer == NULL) {
        return; 
    }

    va_start(args, format);
    vsnprintf(buffer, bufferSize - 2, format, args); 

    buffer[bufferSize - 3] = '\r';
    buffer[bufferSize - 2] = '\n';
    buffer[bufferSize - 1] = '\0';

    HAL_UART_Transmit(outputUart, (uint8_t*)buffer, bufferSize - 1, 1000); 

    free(buffer);
}

void I2C_Scan(I2C_HandleTypeDef* hi2c)
{
  HAL_StatusTypeDef res;
  for (uint16_t i = 0; i < 128; i++)
  {
    res = HAL_I2C_IsDeviceReady(hi2c, i << 1, 1, HAL_MAX_DELAY);
    if (res == HAL_OK)
    {
      uart_printf("Found device at address: 0x%02X", i);
    }
  }
}
