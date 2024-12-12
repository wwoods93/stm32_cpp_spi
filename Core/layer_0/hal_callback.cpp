/***********************************************************************************************************************
 * Main_Controller
 * hal_callbacks.cpp
 *
 * wilson
 * 11/4/22
 * 12:38 AM
 *
 * Description:
 *
 **********************************************************************************************************************/

/* c/c++ includes */

/* stm32 includes */
#include "stm32f4xx_hal.h"
/* third-party includes */

/* layer_0 includes */
#include "hal_general.h"
/* layer_1_rtosal includes */

/* layer_1 includes */

/* layer_3_control includes */

/* layer_4_sys_op includes */

/* layer_n_meta_structure includes */

/* hal_callbacks header */
#include "hal_callback.h"


void hal_callback_uart_tx_complete(UART_HandleTypeDef *huart)
{
//    memset(&user_data, '\0' , strlen(user_data)); //empty the transmission data buffer
}

void hal_callback_uart_rx_complete_callback(UART_HandleTypeDef *huart)
{
//    if(recvd_data == '\r')
//    {
//        data_buffer[cnt++]='\r';
//        HAL_UART_Transmit(huart, send_data, cnt,HAL_MAX_DELAY);
//        memset(data_buffer, 0, cnt);
//    }
//    else
//    {
//        data_buffer[cnt++] = recvd_data;
//    }
//    HAL_UART_Receive_IT(get_usart_2_handle(), &recvd_data,1);

}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        HAL_IncTick();
    }
}
