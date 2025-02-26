/***********************************************************************************************************************
 * Main_Controller
 * device_serial_monitor.h
 *
 * wilson
 * 10/8/24
 * 12:02 AM
 *
 * Description:
 *
 **********************************************************************************************************************/

#ifndef MAIN_CONTROLLER_SERIAL_MONITOR_H
#define MAIN_CONTROLLER_SERIAL_MONITOR_H

/* c/c++ includes */
#include <cstdint>
/* stm32 includes */
#include "stm32f4xx.h"
/* third-party includes */
#include "cmsis_os2.h"
/* layer_0 includes */

/* layer_1_rtosal includes */
#include "../layer_0/rtosal.h"
/* layer_1 includes */

/* layer_3_control includes */

/* layer_4_sys_op includes */

/* layer_n_meta_structure includes */




class serial_monitor
{
    public:


        static constexpr uint8_t USE_SERIAL_MONITOR = 1U;
        static constexpr uint8_t NAME_LENGTH_MAX = 15U;
        static constexpr uint8_t MESSAGE_LENGTH_MAX = 65U;

        typedef struct
        {
            char message[MESSAGE_LENGTH_MAX];
        } packet_t;

        uint32_t initialize(UART_HandleTypeDef* arg_usart_module);
        uint32_t print(uint8_t* arg_output_string);



    private:
        char name[NAME_LENGTH_MAX] = "serial_monitor";
        char message[MESSAGE_LENGTH_MAX];
        uint8_t newline[2] = { '\n', '\0' };
        char hex[5] = "";
        char time_stamp[9] = "";
        UART_HandleTypeDef* usart_module;
};


#endif //MAIN_CONTROLLER_SERIAL_MONITOR_H
