///* USER CODE BEGIN Header */
///**
//  ******************************************************************************
//  * @file           : main.c
//  * @brief          : Main program body
//  ******************************************************************************
//  * @attention
//  *
//  * Copyright (c) 2024 STMicroelectronics.
//  * All rights reserved.
//  *
//  * This software is licensed under terms that can be found in the LICENSE file
//  * in the root directory of this software component.
//  * If no LICENSE file comes with this software, it is provided AS-IS.
//  *
//  ******************************************************************************
//  */
///* USER CODE END Header */
///* Includes ------------------------------------------------------------------*/
//#include "main.h"
//#include "cmsis_os.h"
//
///* Private includes ----------------------------------------------------------*/
///* USER CODE BEGIN Includes */
//#include "../layer_0/hal.h"
///* USER CODE END Includes */
//
///* Private typedef -----------------------------------------------------------*/
///* USER CODE BEGIN PTD */
//
///* USER CODE END PTD */
//
///* Private define ------------------------------------------------------------*/
///* USER CODE BEGIN PD */
///* USER CODE END PD */
//
///* Private macro -------------------------------------------------------------*/
///* USER CODE BEGIN PM */
//
///* USER CODE END PM */
//
///* Private variables ---------------------------------------------------------*/
//// RTC_HandleTypeDef hrtc;
//
////SPI_HandleTypeDef hspi1;
////SPI_HandleTypeDef hspi2;
////SPI_HandleTypeDef hspi3;
//
////TIM_HandleTypeDef htim2;
//
////UART_HandleTypeDef huart2;
//
///* Definitions for client_task */
//osThreadId_t client_task_handle;
//const osThreadAttr_t client_task_attributes = {
//  .name = "client_task",
//  .stack_size = 512 * 4,
//  .priority = (osPriority_t) osPriorityNormal,
//};
///* Definitions for spi_task */
//osThreadId_t spi_task_handle;
//const osThreadAttr_t spi_task_attributes = {
//  .name = "spi_task",
//  .stack_size = 512 * 4,
//  .priority = (osPriority_t) osPriorityNormal,
//};
///* Definitions for heartbeat_task */
//osThreadId_t heartbeat_task_handle;
//const osThreadAttr_t heartbeat_task_attributes = {
//  .name = "heartbeat_task",
//  .stack_size = 128 * 4,
//  .priority = (osPriority_t) osPriorityNormal,
//};
///* Definitions for comms_handler_tick */
//osTimerId_t comms_handler_tick_handle;
//const osTimerAttr_t comms_handler_tick_attributes = {
//  .name = "comms_handler_tick"
//};
///* Definitions for spi_tx_data_buffer_mutex */
//osMutexId_t spi_tx_data_buffer_mutexHandle;
//const osMutexAttr_t spi_tx_data_buffer_mutex_attributes = {
//  .name = "spi_tx_data_buffer_mutex"
//};
///* Definitions for spi_rx_data_buffer_mutex */
//osMutexId_t spi_rx_data_buffer_mutexHandle;
//const osMutexAttr_t spi_rx_data_buffer_mutex_attributes = {
//  .name = "spi_rx_data_buffer_mutex"
//};
///* Definitions for i2c_tx_data_buffer_mutex */
//osMutexId_t i2c_tx_data_buffer_mutexHandle;
//const osMutexAttr_t i2c_tx_data_buffer_mutex_attributes = {
//  .name = "i2c_tx_data_buffer_mutex"
//};
///* Definitions for i2c_rx_data_buffer_mutex */
//osMutexId_t i2c_rx_data_buffer_mutexHandle;
//const osMutexAttr_t i2c_rx_data_buffer_mutex_attributes = {
//  .name = "i2c_rx_data_buffer_mutex"
//};
///* USER CODE BEGIN PV */
//
///* USER CODE END PV */
//
///* Private function prototypes -----------------------------------------------*/
//
//
//
//void start_client_task(void *argument);
//void start_spi_task(void *argument);
//void start_heartbeat_task(void *argument);
//void comms_handler_tick_callback(void *argument);
//
///* USER CODE BEGIN PFP */
//
///* USER CODE END PFP */
//
///* Private user code ---------------------------------------------------------*/
///* USER CODE BEGIN 0 */
//
///* USER CODE END 0 */
//
///**
//  * @brief  The application entry point.
//  * @retval int
//  */
//int main(void)
//{
//  /* USER CODE BEGIN 1 */
//
//  /* USER CODE END 1 */
//
//  /* MCU Configuration--------------------------------------------------------*/
//
//  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  HAL_Init();
//
//  /* USER CODE BEGIN Init */
//
//  /* USER CODE END Init */
//
//  /* Configure the system clock */
//  SystemClock_Config();
//
//  /* USER CODE BEGIN SysInit */
//
//  /* USER CODE END SysInit */
//
//  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_USART2_UART_Init();
//  MX_TIM2_Init();
//  MX_RTC_Init();
//  MX_SPI3_Init();
//  /* USER CODE BEGIN 2 */
//
//  /* USER CODE END 2 */
//
//  /* Init scheduler */
//  osKernelInitialize();
//  /* Create the mutex(es) */
//  /* creation of spi_tx_data_buffer_mutex */
//  spi_tx_data_buffer_mutexHandle = osMutexNew(&spi_tx_data_buffer_mutex_attributes);
//
//  /* creation of spi_rx_data_buffer_mutex */
//  spi_rx_data_buffer_mutexHandle = osMutexNew(&spi_rx_data_buffer_mutex_attributes);
//
//  /* creation of i2c_tx_data_buffer_mutex */
//  i2c_tx_data_buffer_mutexHandle = osMutexNew(&i2c_tx_data_buffer_mutex_attributes);
//
//  /* creation of i2c_rx_data_buffer_mutex */
//  i2c_rx_data_buffer_mutexHandle = osMutexNew(&i2c_rx_data_buffer_mutex_attributes);
//
//  /* USER CODE BEGIN RTOS_MUTEX */
//  /* add mutexes, ... */
//  /* USER CODE END RTOS_MUTEX */
//
//  /* USER CODE BEGIN RTOS_SEMAPHORES */
//  /* add semaphores, ... */
//  /* USER CODE END RTOS_SEMAPHORES */
//
//  /* Create the timer(s) */
//  /* creation of comms_handler_tick */
//  comms_handler_tick_handle = osTimerNew(comms_handler_tick_callback, osTimerPeriodic, NULL, &comms_handler_tick_attributes);
//
//  /* USER CODE BEGIN RTOS_TIMERS */
//  /* start timers, add new ones, ... */
//  /* USER CODE END RTOS_TIMERS */
//
//  /* USER CODE BEGIN RTOS_QUEUES */
//  /* add queues, ... */
//  /* USER CODE END RTOS_QUEUES */
//
//  /* Create the thread(s) */
//  /* creation of client_task */
//  client_task_handle = osThreadNew(start_client_task, NULL, &client_task_attributes);
//
//  /* creation of spi_task */
//  spi_task_handle = osThreadNew(start_spi_task, NULL, &spi_task_attributes);
//
//  /* creation of heartbeat_task */
//  heartbeat_task_handle = osThreadNew(start_heartbeat_task, NULL, &heartbeat_task_attributes);
//
//  /* USER CODE BEGIN RTOS_THREADS */
//  /* add threads, ... */
//  /* USER CODE END RTOS_THREADS */
//
//  /* USER CODE BEGIN RTOS_EVENTS */
//  /* add events, ... */
//  /* USER CODE END RTOS_EVENTS */
//
//  /* Start scheduler */
//  osKernelStart();
//
//  /* We should never get here as control is now taken by the scheduler */
//  /* Infinite loop */
//  /* USER CODE BEGIN WHILE */
//  while (1)
//  {
//    /* USER CODE END WHILE */
//
//    /* USER CODE BEGIN 3 */
//  }
//  /* USER CODE END 3 */
//}
//
//
//
///* USER CODE BEGIN 4 */
//
///* USER CODE END 4 */
//
///* USER CODE BEGIN Header_start_client_task */
///**
//  * @brief  Function implementing the client_task thread.
//  * @param  argument: Not used
//  * @retval None
//  */
///* USER CODE END Header_start_client_task */
//void start_client_task(void *argument)
//{
//  /* USER CODE BEGIN 5 */
//  /* Infinite loop */
//  for(;;)
//  {
//    osDelay(1);
//  }
//  /* USER CODE END 5 */
//}
//
///* USER CODE BEGIN Header_start_spi_task */
///**
//* @brief Function implementing the spi_task thread.
//* @param argument: Not used
//* @retval None
//*/
///* USER CODE END Header_start_spi_task */
//void start_spi_task(void *argument)
//{
//  /* USER CODE BEGIN start_spi_task */
//  /* Infinite loop */
//  for(;;)
//  {
//    osDelay(1);
//  }
//  /* USER CODE END start_spi_task */
//}
//
///* USER CODE BEGIN Header_start_heartbeat_task */
///**
//* @brief Function implementing the heartbeat_task thread.
//* @param argument: Not used
//* @retval None
//*/
///* USER CODE END Header_start_heartbeat_task */
//void start_heartbeat_task(void *argument)
//{
//  /* USER CODE BEGIN start_heartbeat_task */
//  /* Infinite loop */
//  for(;;)
//  {
//    osDelay(1);
//  }
//  /* USER CODE END start_heartbeat_task */
//}
//
///* comms_handler_tick_callback function */
//void comms_handler_tick_callback(void *argument)
//{
//  /* USER CODE BEGIN comms_handler_tick_callback */
//
//  /* USER CODE END comms_handler_tick_callback */
//}
//
///**
//  * @brief  Period elapsed callback in non blocking mode
//  * @note   This function is called  when TIM4 interrupt took place, inside
//  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
//  * a global variable "uwTick" used as application time base.
//  * @param  htim : TIM handle
//  * @retval None
//  */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//  /* USER CODE BEGIN Callback 0 */
//
//  /* USER CODE END Callback 0 */
//  if (htim->Instance == TIM4) {
//    HAL_IncTick();
//  }
//  /* USER CODE BEGIN Callback 1 */
//
//  /* USER CODE END Callback 1 */
//}
//
///**
//  * @brief  This function is executed in case of error occurrence.
//  * @retval None
//  */
//void Error_Handler(void)
//{
//  /* USER CODE BEGIN Error_Handler_Debug */
//  /* User can add his own implementation to report the HAL error return state */
//  __disable_irq();
//  while (1)
//  {
//  }
//  /* USER CODE END Error_Handler_Debug */
//}
//
//#ifdef  USE_FULL_ASSERT
///**
//  * @brief  Reports the name of the source file and the source line number
//  *         where the assert_param error has occurred.
//  * @param  file: pointer to the source file name
//  * @param  line: assert_param error line source number
//  * @retval None
//  */
//void assert_failed(uint8_t *file, uint32_t line)
//{
//  /* USER CODE BEGIN 6 */
//  /* User can add his own implementation to report the file name and line number,
//     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//  /* USER CODE END 6 */
//}
//#endif /* USE_FULL_ASSERT */
