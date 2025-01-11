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
/* stm32 includes */
#include "stm32f4xx_it.h"
/* third-party includes */

/* layer_0_hal includes */
#include "../layer_0/hal.h"
#include "../layer_0/hal_spi.h"
#include "../layer_0/hal_callback.h"
#include "../layer_0/rtosal.h"
/* layer_1_rtosal includes */
#include "../layer_0/rtosal_globals.h"
/* layer_2_device includes */

/* layer_3_control includes */

/* layer_4_sys_op includes */

/* layer_n_meta_structure includes */

/* main header */

#include "main.h"
#include "cmsis_os.h"

static constexpr uint8_t SYSTEM_RUN = 1U;

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

    HAL_TIM_Base_Start(get_timer_2_handle());

    osKernelInitialize();

    comms_handler_tick_handle = osTimerNew(comms_handler_tick_callback, osTimerPeriodic, nullptr, &comms_handler_tick_attributes);
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

    static uint32_t client_task_count = 0U;
    uint8_t tx_bytes[8] = { 0x01, 0x03, 0x05, 0x06, 0x08, 0x0A, 0x0B, 0x0F};
    uint8_t rx_bytes[8] = { 1, 0, 1, 0, 1, 0, 1, 0 };
    uint8_t bytes_per_tx[8] = { 8, 0, 0, 0, 0, 0, 0, 0 };

    while (SYSTEM_RUN)
    {

        hal::spi_2.send_async(tx_bytes, bytes_per_tx, 0U);
        hal::spi_2.receive_async(rx_bytes, 0U);

//        hal::spi_2.send_async(tx_bytes, bytes_per_tx, 0);
//
//        rx_bytes[0] = 1;
//        hal::spi_2.receive_async_remote(rx_bytes, 0);
//        if (rx_bytes[0] == 0)
//        {
//            rx_bytes[0] = 5;
//            ++client_task_count;
//        }
        rtosal::thread_yield();
    }
}

[[noreturn]] void start_spi_task(void *argument)
{
    osEventFlagsId_t initialization_event_flags_handle = get_initialization_event_flags_handle();
    rtosal::message_queue_handle_t tx_queue_handle = get_spi_2_client_tx_queue_handle();
    rtosal::message_queue_handle_t rx_queue_handle = get_spi_2_client_rx_queue_handle();

    uint8_t tx_bytes[8] = { 0x01, 0x03, 0x05, 0x06, 0x08, 0x0A, 0x0B, 0x0F};
    uint8_t rx_bytes[8] = { 1, 0, 1, 0, 1, 0, 1, 0 };
    uint8_t bytes_per_tx[8] = { 8, 0, 0, 0, 0, 0, 0, 0 };
    uint8_t tx_byte = 0x1AU;
    uint8_t rx_byte = 0x00U;
    int16_t channel_0_id = 0U;
    static uint32_t spi_task_count = 0U;
    spi::packet_t packet;

    spi::module_t spi_2_handle;
    hal::spi_2.initialize(&spi_2_handle, SPI_2_ID, get_timer_2_handle());
    hal::spi_2.create_channel(channel_0_id, PORT_B, GPIO_PIN_14, 0U, tx_queue_handle, rx_queue_handle);
    rtosal::event_flag_set(initialization_event_flags_handle, READY_FOR_USER_INIT_FLAG);

    while (SYSTEM_RUN)
    {
        if (get_timer_count(get_timer_2_handle()) - spi_task_count > 1000000)
        {
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            spi_task_count = get_timer_count(get_timer_2_handle());
        }
//        hal::spi_2.send(tx_bytes, bytes_per_tx, 0);
//        hal::spi_2.send_receive_byte(tx_byte, rx_byte, 0U);
//        hal::spi_2.send_receive(tx_bytes, rx_bytes, bytes_per_tx, 0U);

//        hal::spi_2.send_async(tx_bytes, bytes_per_tx, 0U);
//        hal::spi_2.receive_async(rx_bytes, 0U);
        hal::spi_2.process_async(packet);
    }
}


[[noreturn]] void start_heartbeat_task(void *argument)
{
    static uint32_t count = 0U;

    while (SYSTEM_RUN)
    {
//        if (count > 200000U)
//        {
//            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//            count = 0U;
//        }
//        ++count;
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
}

void SPI2_IRQHandler()
{
    spi_irq_handler(hal::get_spi_2_object());
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
