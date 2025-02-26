/***********************************************************************************************************************
 * stm32_cpp_spi_lib
 * main.cpp
 *
 * wilson
 * 11/28/24
 * 1:18 AM
 *
 * Description:
 *
 **********************************************************************************************************************/

/* c/c++ includes */
#include <memory>
#include <cstring>
#include <cstdio>
/* stm32 includes */
#include "stm32f4xx_it.h"
/* third-party includes */
#include "cmsis_os.h"
/* layer_0 includes */
#include "../layer_0/hal.h"
#include "../layer_0/hal_spi.h"
#include "../layer_0/rtosal.h"
#include "../Inc/device.h"
#include "../Inc/serial_monitor.h"
/* main header */
#include "main.h"


static constexpr uint8_t SYSTEM_RUN = 1U;




UART_HandleTypeDef huart2;


serial_monitor debug_serial_monitor;

osThreadId_t client_task_handle;
osThreadId_t spi_task_handle;
osThreadId_t heartbeat_task_handle;
osTimerId_t comms_handler_tick_handle;

const osThreadAttr_t client_task_attributes     = { .name = "client_task",      .stack_size = 512 * 4, .priority = (osPriority_t) osPriorityNormal, };
const osThreadAttr_t spi_task_attributes        = { .name = "spi_task",         .stack_size = 512 * 4, .priority = (osPriority_t) osPriorityNormal, };
const osThreadAttr_t heartbeat_task_attributes  = { .name = "heartbeat_task",   .stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };



const osTimerAttr_t comms_handler_tick_attributes = { .name = "comms_handler_tick" };

SPI_HandleTypeDef hspi2;
void MX_SPI2_Init();

[[noreturn]] void start_client_task(void *argument);
[[noreturn]] void start_spi_task(void *argument);
[[noreturn]] void start_heartbeat_task(void *argument);

void comms_handler_tick_callback(void *argument);

int main()
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_TIM2_Init();
    MX_RTC_Init();
    MX_SPI2_Init();
    HAL_TIM_Base_Start(get_timer_2_handle());

    osKernelInitialize();

    comms_handler_tick_handle = osTimerNew(comms_handler_tick_callback, osTimerPeriodic, nullptr,
                                           &comms_handler_tick_attributes);
    client_task_handle = osThreadNew(start_client_task, nullptr, &client_task_attributes);
    spi_task_handle = osThreadNew(start_spi_task, nullptr, &spi_task_attributes);
    heartbeat_task_handle = osThreadNew(start_heartbeat_task, nullptr, &heartbeat_task_attributes);

    rtosal::initialize();

    osKernelStart();

    while (1)
    {

    }
}

[[noreturn]] void start_client_task(void *argument)
{
    osEventFlagsId_t initialization_event_flags_handle = get_initialization_event_flags_handle();
    rtosal::event_flag_wait(initialization_event_flags_handle, READY_FOR_USER_INIT_FLAG, rtosal::OS_FLAGS_ANY, rtosal::OS_WAIT_FOREVER);

    while (SYSTEM_RUN)
    {

    }
}

[[noreturn]] void start_spi_task(void *argument)
{
    osEventFlagsId_t initialization_event_flags_handle = get_initialization_event_flags_handle();

    static uint32_t spi_task_count = 0U;


    uint8_t tx_bytes[8] = { 0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0C, 0x0E };
    uint8_t rx_bytes[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

    rtosal::event_flag_set(initialization_event_flags_handle, READY_FOR_USER_INIT_FLAG);

    hal::gpio_write_pin(PORT_B, GPIO_PIN_14, 1U);

    while (SYSTEM_RUN)
    {
        if (get_timer_count(get_timer_2_handle()) - spi_task_count > 300000)
        {
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            hal::gpio_write_pin(PORT_B, GPIO_PIN_14, 0U);
            HAL_SPI_Transmit(&hspi2, tx_bytes, 8, 100U);
            hal::gpio_write_pin(PORT_B, GPIO_PIN_14, 1U);
            spi_task_count = get_timer_count(get_timer_2_handle());
        }

//        if (hal::gpio_read_pin(PORT_B, GPIO_PIN_14) == 0U)
//        {
//            HAL_SPI_Receive(&hspi2, rx_bytes, 8, 100U);
//        }


    }
}


[[noreturn]] void start_heartbeat_task(void *argument)
{
    static uint32_t count = 0U;

    while (SYSTEM_RUN)
    {
//        rtosal::thread_yield();
    }
}

void comms_handler_tick_callback(void *argument)
{

}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

void SPI2_IRQHandler()
{
    HAL_SPI_IRQHandler(&hspi2);
}

void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}

void MX_SPI2_Init()
{
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 7;
    if (HAL_SPI_Init(&hspi2) != HAL_OK)
    {
        Error_Handler();
    }

}
void callback_spi_peripheral_tx_rx_complete(SPI_HandleTypeDef *hspi)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
}
void callback_spi_controller_error(SPI_HandleTypeDef *hspi)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
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



void MX_USART2_UART_Init()
{
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
        error_handler();
    }
}
