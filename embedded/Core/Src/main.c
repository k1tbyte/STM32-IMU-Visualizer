/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <semphr.h>
#include "bmi160.h"
#include "routine.h"
#include "lcd1602_i2c_lib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*! bmi160 shuttle id */
#define BMI160_SHUTTLE_ID     0x38

/*! bmi160 Device address */
#define BMI160_ADDR       0x69
#define LCD_ADDR (0x27 << 1)

#define ACCEL_CONVERSION 0.000598  // ±2g (м/с²/LSB)
#define GYRO_CONVERSION  0.00381   // ±125°/с (°/с/LSB)

// SDA - PB7  (LEFT)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 2056,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for buttonTask */
osThreadId_t buttonTaskHandle;
const osThreadAttr_t buttonTask_attributes = {
  .name = "buttonTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */


I2C_LCD_HandleTypeDef lcd1;
struct bmi160_dev sensor;
SemaphoreHandle_t sensorSemaphore;
StaticSemaphore_t displayStateMutexBuffer;
SemaphoreHandle_t displayStateMutex;
static EDisplayState displayState = All;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
void StartDefaultTask(void *argument);
void StartButtonTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int8_t i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    return HAL_I2C_Mem_Write(&hi2c2, dev_addr << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY) == HAL_OK ? 0 : -1;
}


int8_t i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    return HAL_I2C_Mem_Read(&hi2c2, dev_addr << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY) == HAL_OK ? 0 : -1;
}


void displayAcceleration(float accelX, float accelY, float accelZ) {
    char line[16];
    float totalAcceleration = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

    // min max вираховуємо
    float maxAccel = fmax(fmax(fabs(accelX), fabs(accelY)), fabs(accelZ));
    float minAccel = fmin(fmin(fabs(accelX), fabs(accelY)), fabs(accelZ));

    const char maxAxis = maxAccel == fabs(accelX) ? 'X' : maxAccel == fabs(accelY) ? 'Y' : 'Z';
    const char minAxis =  minAccel == fabs(accelX) ? 'X' : minAccel == fabs(accelY) ? 'Y' : 'Z';

    lcd_clear(&lcd1);
    lcd_gotoxy(&lcd1, 0, 0);
    switch(displayState) {
      case Min:
          snprintf(line, sizeof(line), "> Min. Axis: %c", minAxis);
          lcd_puts(&lcd1, line);
          lcd_gotoxy(&lcd1, 0, 1);
          snprintf(line, sizeof(line), "Value: %.4f", minAccel);
          lcd_puts(&lcd1, line);
        break;
  
      case Max:
          snprintf(line, sizeof(line), "> Max. Axis: %c", maxAxis);
          lcd_puts(&lcd1, line);
          lcd_gotoxy(&lcd1, 0, 1);
          snprintf(line, sizeof(line), "Value: %.4f", maxAccel);
          lcd_puts(&lcd1, line);
        break;
      case All:
        lcd_puts(&lcd1, " Min  Max  Total");
        lcd_gotoxy(&lcd1, 0, 1);
        snprintf(line, sizeof(line), "%c%.1f %c%.1f %.1f",minAxis, minAccel, maxAxis, maxAccel, totalAcceleration);
        lcd_puts(&lcd1, line);
        break;
      case Total:
          lcd_puts(&lcd1, "> Total accel.");
          lcd_gotoxy(&lcd1, 0, 1);
          snprintf(line, sizeof(line), "Value: %.4f", totalAcceleration);
          lcd_puts(&lcd1, line);
        break;
    }


}

void process_sensor_data(struct bmi160_sensor_data accel, struct bmi160_sensor_data gyro) {
    float accelX = accel.x * ACCEL_CONVERSION;  // м/с²
    float accelY = accel.y * ACCEL_CONVERSION;  // м/с²
    float accelZ = accel.z * ACCEL_CONVERSION;  // м/с²

  /*  float gyroX = gyro.x * GYRO_CONVERSION;  // °/с
    float gyroY = gyro.y * GYRO_CONVERSION;  // °/с
    float gyroZ = gyro.z * GYRO_CONVERSION;  // °/с */

    // print to lcd
    // 1. Maximum acceleration (and axis);
    // 2. Minimum acceleration (and axis);
    // 3. Total acceleration vector
    displayAcceleration(accelX, accelY, accelZ);

    uart_printf("{ \"Ax\": %d, \"Ay\":%d, \"Az\":%d, \"Gx\":%d, \"Gy\":%d, \"Gz\":%d }", 
                    accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z);
}

void BMI160_Init(void)
{
  int8_t rslt;

  sensor.write = i2c_write;
  sensor.read = i2c_read;
  sensor.delay_ms = HAL_Delay;

  /* set correct i2c address */
  sensor.id = BMI160_ADDR;
  sensor.intf = BMI160_I2C_INTF;

  rslt = bmi160_init(&sensor);

  if (bmi160_init(&sensor) != BMI160_OK)
  {
    uart_printf("BMI160 initialization failed");
    exit(rslt);
  }

  sensor.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
  sensor.accel_cfg.range = BMI160_ACCEL_RANGE_2G; // ±2g
  sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

  /* Select the power mode of accelerometer sensor */
  sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

  /* Select the Output data rate, range of Gyroscope sensor */
  sensor.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
  sensor.gyro_cfg.range = BMI160_GYRO_RANGE_125_DPS; // ±125°/с
  sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

  /* Select the power mode of Gyroscope sensor */
  sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

  rslt = bmi160_set_sens_conf(&sensor);
  if (rslt != BMI160_OK)
  {
    uart_printf("Error setting sensor configuration: %d", rslt);
    exit(rslt);
  }
}

void LCD1602_Init(void)
{
  lcd1.hi2c = &hi2c2;     // hi2c2 is your I2C handler
  lcd1.address = LCD_ADDR;    // I2C address for the first LCD
  lcd_init(&lcd1);        // Initialize the first LCD
  lcd_clear(&lcd1);
}

void initResources() {    
    BMI160_Init();
    LCD1602_Init();
    sensorSemaphore = xSemaphoreCreateMutex();
    displayStateMutex = xSemaphoreCreateMutexStatic(&displayStateMutexBuffer);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  setOutputUart(&huart2);
  I2C_Scan(&hi2c2);
  initResources();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of buttonTask */
  buttonTaskHandle = osThreadNew(StartButtonTask, NULL, &buttonTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : USER_BTN_Pin */
  GPIO_InitStruct.Pin = USER_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BTN_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static int readBMI160(struct bmi160_sensor_data* accel, struct bmi160_sensor_data* gyro) {
    int8_t result = -1;
    if(xSemaphoreTake(sensorSemaphore, 0) == pdTRUE) {
        result = bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), accel, gyro, &sensor);
        xSemaphoreGive(sensorSemaphore);
    }
    return result;
}

static void updateDisplayState() {
  if(xSemaphoreTake(displayStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    if(++displayState > Total) {
        displayState = All;
    }
    xSemaphoreGive(displayStateMutex);
}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  struct bmi160_sensor_data accel;
  struct bmi160_sensor_data gyro;
  /* Infinite loop */
  for(;;)
  {
    if (readBMI160(&accel, &gyro) == 0) {
        process_sensor_data(accel, gyro);
    } else {
        uart_printf("Error reading accelerometer data");
    }
    
    osDelay(30);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartButtonTask */
/**
* @brief Function implementing the buttonTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartButtonTask */
void StartButtonTask(void *argument)
{
  /* USER CODE BEGIN StartButtonTask */
  GPIO_PinState lastState = -1;
  /* Infinite loop */
  for(;;)
  {
    GPIO_PinState currentState = HAL_GPIO_ReadPin(USER_BTN_GPIO_Port, USER_BTN_Pin);
    if(currentState == GPIO_PIN_SET && lastState == GPIO_PIN_RESET) {
      updateDisplayState();
      osDelay(50);
    }
    
    lastState = currentState;
    osDelay(10);
  }
  /* USER CODE END StartButtonTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
