/***********************************************************************************************************************
 * Main_Controller
 * peripheral_initialization.h
 *
 * wilson
 * 6/30/22
 * 7:27 PM
 *
 * Description:
 *
 **********************************************************************************************************************/


#ifndef MAIN_CONTROLLER_HAL_H
#define MAIN_CONTROLLER_HAL_H

/* c/c++ includes */

/* stm32 includes */
#include "stm32f4xx_hal.h"
//#include "stm32f4xx_hal_uart.h"
/* third-party includes */

/* layer_0 includes */
#include "hal_spi.h"
/* layer_1_rtosal includes */

/* layer_1 includes */

/* layer_3_control includes */

/* layer_4_sys_op includes */

/* layer_n_meta_structure includes */


#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define SPI1_CS2_Pin GPIO_PIN_4
#define SPI1_CS2_GPIO_Port GPIOC
#define SPI2_CS1_Pin GPIO_PIN_14
#define SPI2_CS1_GPIO_Port GPIOB
#define SPI2_CS2_Pin GPIO_PIN_15
#define SPI2_CS2_GPIO_Port GPIOB
#define SPI3_CS1_Pin GPIO_PIN_8
#define SPI3_CS1_GPIO_Port GPIOC
#define SPI1_CS1_Pin GPIO_PIN_10
#define SPI1_CS1_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA

namespace hal
{
    extern spi spi_2;

    spi* get_spi_2_object();
    void i2c_build_packet_array_from_converted_bytes(uint8_t* arg_i2c_packet_array, uint8_t arg_global_id, const uint8_t* arg_converted_bytes);

    void timer_2_initialize();
}

void SystemClock_Config();
void MX_GPIO_Init();
void MX_USART2_UART_Init();
void MX_TIM2_Init();
void MX_RTC_Init();
void MX_SPI1_Init();
void MX_SPI3_Init();

void error_handler();
//void Error_Handler();

SPI_HandleTypeDef* get_spi_1_handle();
RTC_HandleTypeDef* get_rtc_handle();
TIM_HandleTypeDef* get_timer_2_handle();
uint32_t get_timer_2_count();
uint32_t get_timer_count(hal::timer_handle_t* arg_timer_handle);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

//UART_HandleTypeDef* get_usart_2_handle();
//void initialize_peripherals();

//void MX_USART2_UART_Init();
//
//void MX_RTC_Init();
//
//
//void SystemClock_Config();

#endif //MAIN_CONTROLLER_HAL_H
