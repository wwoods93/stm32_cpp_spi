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

/* stm32 includes */
#include "stm32f4xx_it.h"
/* third-party includes */

/* layer_0_hal includes */
#include "../layer_0/hal.h"
#include "../layer_0/hal_spi.h"
#include "../layer_0/hal_callback.h"
#include "../layer_0/rtosal.h"
/* layer_1_rtosal includes */

/* layer_2_device includes */

/* layer_3_control includes */

/* layer_4_sys_op includes */

/* layer_n_meta_structure includes */

/* main header */

#include "main.h"
#include "cmsis_os.h"

osThreadId_t client_taskHandle;
osThreadId_t spi_taskHandle;
osThreadId_t heartbeat_taskHandle;
const osThreadAttr_t client_task_attributes     = { .name = "client_task",      .stack_size = 512 * 4, .priority = (osPriority_t) osPriorityNormal, };
const osThreadAttr_t spi_task_attributes        = { .name = "spi_task",         .stack_size = 512 * 4, .priority = (osPriority_t) osPriorityNormal, };
const osThreadAttr_t heartbeat_task_attributes  = { .name = "heartbeat_task",   .stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };


osTimerId_t comms_handler_tickHandle;
const osTimerAttr_t comms_handler_tick_attributes = { .name = "comms_handler_tick" };

SPI_HandleTypeDef hspi2;
void MX_SPI2_Init();
void callback_spi_peripheral_tx_rx_complete(SPI_HandleTypeDef *hspi);
void callback_spi_controller_error(SPI_HandleTypeDef *hspi);

void start_client_task(void *argument);
void start_spi_task(void *argument);
void start_heartbeat_task(void *argument);
void comms_handler_tick_callback(void *argument);

int main()
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_TIM2_Init();
    MX_RTC_Init();

    MX_SPI3_Init();
    MX_SPI2_Init();
    HAL_TIM_Base_Start(get_timer_2_handle());

    osKernelInitialize();

    comms_handler_tickHandle = osTimerNew(comms_handler_tick_callback, osTimerPeriodic, nullptr, &comms_handler_tick_attributes);


    client_taskHandle = osThreadNew(start_client_task, nullptr, &client_task_attributes);
    spi_taskHandle = osThreadNew(start_spi_task, nullptr, &spi_task_attributes);
    heartbeat_taskHandle = osThreadNew(start_heartbeat_task, nullptr, &heartbeat_task_attributes);

    rtosal::initialize();
    osKernelStart();

    while (1)
    {

    }
}

void start_client_task(void *argument)
{
    static uint32_t client_task_count = 0U;
    common_packet_t request_packet;
    common_packet_t result_packet;
    uint8_t complete_tx[8] = { 0x01, 0x03, 0x05, 0x06, 0x08, 0x0A, 0x0B, 0x0F};
    uint8_t bytes_per_tx[8] = { 8, 0, 0, 0, 0, 0, 0, 0 };
    rtosal::message_queue_handle_t tx_queue_handle;
    rtosal::message_queue_handle_t rx_queue_handle;
    tx_queue_handle = get_spi_2_client_tx_queue_handle();
    rx_queue_handle = get_spi_2_client_rx_queue_handle();

    rtosal::build_common_packet(request_packet, 0U, complete_tx, bytes_per_tx);
    for(;;)
    {
        if (client_task_count > 50U)
        {
            if (rtosal::message_queue_send(tx_queue_handle, &request_packet, 0U) == rtosal::OS_OK)
            {
                client_task_count = 10U;
            }
            client_task_count = 0U;
        }

        if (rtosal::message_queue_receive( rx_queue_handle, &result_packet, 0U) == rtosal::OS_OK)
        {

        }
        ++client_task_count;
        rtosal::thread_yield();
    }
}

void start_spi_task(void *argument)
{
    spi::module_t spi_2_handle;
    int16_t rtd_0_channel_id = 0U;
    static uint32_t spi_task_count = 0U;

    rtosal::message_queue_handle_t tx_queue_handle;
    rtosal::message_queue_handle_t rx_queue_handle;
    tx_queue_handle = get_spi_2_client_tx_queue_handle();
    rx_queue_handle = get_spi_2_client_rx_queue_handle();

    hal::spi_2.initialize(&spi_2_handle, SPI_2_ID, get_timer_2_handle());
    hal::spi_2.create_channel(rtd_0_channel_id, PORT_B, GPIO_PIN_14, tx_queue_handle, rx_queue_handle);
    for(;;)
    {
        hal::spi_2.receive_inter_task_transaction_requests();
        hal::spi_2.process_send_buffer();
        hal::spi_2.process_return_buffers();
    }
}


void start_heartbeat_task(void *argument)
{
    static uint32_t count = 0U;

    for(;;)
    {
        if (count > 200000U)
        {
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            count = 0U;
        }
        ++count;
        rtosal::thread_yield();
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
    /* USER CODE END Error_Handler_Debug */
}

void SPI2_IRQHandler()
{
    spi_irq_handler(hal::get_spi_2_object());
//    HAL_SPI_IRQHandler(&hspi2);
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
