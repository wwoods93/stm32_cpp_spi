/***********************************************************************************************************************
 * Main_Controller
 * device_serial_monitor.cpp
 *
 * wilson
 * 10/8/24
 * 12:02 AM
 *
 * Description:
 *
 **********************************************************************************************************************/

/* c/c++ includes */
#include <cstdint>
#include <memory>
#include <cstring>
#include <cstdio>
/* stm32 includes */

/* third-party includes */
#include "../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.h"
/* layer_0 includes */
#include "../layer_0/hal_wrapper.h"
/* layer_1_rtosal includes */
#include "../layer_0/rtosal.h"
/* layer_1 includes */

/* layer_3_control includes */

/* layer_4_sys_op includes */

/* layer_n_meta_structure includes */

/* device_serial_monitor header */
#include "serial_monitor.h"


uint32_t serial_monitor::initialize(UART_HandleTypeDef* arg_usart_module)
{
    usart_module = arg_usart_module;
    return 0;
}

uint32_t serial_monitor::print( uint8_t* arg_output_string)
{
//    packet_t packet;
//    hal::rtc_get_time_stamp(time_stamp);
//    memset(&packet, '\0', sizeof(packet_t));
//    sprintf(packet.message, "%s->%s:: %s", time_stamp, name, arg_output_string);


HAL_UART_Transmit_IT(usart_module, (uint8_t *) arg_output_string,sizeof(arg_output_string));





    HAL_UART_Transmit_IT(usart_module, (uint8_t *) newline,2);
    return 0;
}
