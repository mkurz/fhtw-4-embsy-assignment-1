/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* Definitions for mallocTask1 */
osThreadId_t mallocTask1Handle;
const osThreadAttr_t mallocTask1_attributes = {
  .name = "mallocTask1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for mallocTask2 */
osThreadId_t mallocTask2Handle;
const osThreadAttr_t mallocTask2_attributes = {
  .name = "mallocTask2",
  .stack_size = 132 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for mallocTask3 */
osThreadId_t mallocTask3Handle;
const osThreadAttr_t mallocTask3_attributes = {
  .name = "mallocTask3",
  .stack_size = 136 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for watchMemTask */
osThreadId_t watchMemTaskHandle;
const osThreadAttr_t watchMemTask_attributes = {
  .name = "watchMemTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void task1_allocate_memory(void *argument);
void task2_allocate_memory(void *argument);
void task3_allocate_memory(void *argument);
void watch_allocated_memory(void *argument);

/* USER CODE BEGIN PFP */
void recursive_function(int);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Print given character on UART 2. Translate '\n' to "\r\n" on the fly. */
int __io_putchar(int ch)
{
  int ret;
  while ((ret=HAL_UART_GetState(&huart2)) != HAL_UART_STATE_READY)
    ;

  if (ch == '\n')
  {
    static uint8_t buf[2] = { '\r', '\n' };
    HAL_UART_Transmit_IT(&huart2, buf, sizeof(buf));
  }
  else
  {
    static char buf;
    buf = ch;
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)&buf, 1);
  }
  return ch;
}

int _write(int file, char *ptr, int len)
{
  for (int DataIdx = 0; DataIdx < len; DataIdx++)
  {
    __io_putchar(*ptr++);
  }
  return len;
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
  /* USER CODE BEGIN 2 */

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
  /* creation of mallocTask1 */
  mallocTask1Handle = osThreadNew(task1_allocate_memory, NULL, &mallocTask1_attributes);

  /* creation of mallocTask2 */
  mallocTask2Handle = osThreadNew(task2_allocate_memory, NULL, &mallocTask2_attributes);

  /* creation of mallocTask3 */
  mallocTask3Handle = osThreadNew(task3_allocate_memory, NULL, &mallocTask3_attributes);

  /* creation of watchMemTask */
  watchMemTaskHandle = osThreadNew(watch_allocated_memory, NULL, &watchMemTask_attributes);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
  * @brief  Recursive function which just calls itself to allocate stack size.
  * @param  count: Used to count how often the function was called. Recursion stops when value is 20.
  * @retval None
  */
void recursive_function(int count) {
  osDelay(900);
  if (count < 20) {
    recursive_function(++count);
  }
  // Stop after calling the method 20 times recursive to not overflow and kill the MCU
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_task1_allocate_memory */
/**
  * @brief  Function implementing the mallocTask1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_task1_allocate_memory */
void task1_allocate_memory(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    recursive_function(0);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_task2_allocate_memory */
/**
* @brief Function implementing the mallocTask2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task2_allocate_memory */
void task2_allocate_memory(void *argument)
{
  /* USER CODE BEGIN task2_allocate_memory */
  /* Infinite loop */
  for(;;)
  {
    recursive_function(0);
  }
  /* USER CODE END task2_allocate_memory */
}

/* USER CODE BEGIN Header_task3_allocate_memory */
/**
* @brief Function implementing the mallocTask3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task3_allocate_memory */
void task3_allocate_memory(void *argument)
{
  /* USER CODE BEGIN task3_allocate_memory */
  /* Infinite loop */
  for(;;)
  {
    recursive_function(0);
  }
  /* USER CODE END task3_allocate_memory */
}

/* USER CODE BEGIN Header_watch_allocated_memory */
/**
* @brief Function implementing the watchMemTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_watch_allocated_memory */
void watch_allocated_memory(void *argument)
{
  /* USER CODE BEGIN watch_allocated_memory */
  osThreadAttr_t thread_attrs[] = { mallocTask1_attributes, mallocTask2_attributes, mallocTask3_attributes };
  osThreadId_t thread_ids[] = { mallocTask1Handle, mallocTask2Handle, mallocTask3Handle };
  size_t total_threads = sizeof(thread_attrs) / sizeof(thread_attrs[0]);

  /* Infinite loop */
  for(;;)
  {
    printf("\n"); // Just empty line for better readability
    for(int i = 0; i < total_threads; i++)
    {
      printf("Remaining stack size for %s: %lu bytes of total %lu bytes\n",
        thread_attrs[i].name, osThreadGetStackSpace(thread_ids[i]), thread_attrs[i].stack_size);
    }
    osDelay(1000);
  }
  /* USER CODE END watch_allocated_memory */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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

