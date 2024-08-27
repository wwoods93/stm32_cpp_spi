/***********************************************************************************************************************
 * Main_Controller
 * hal_callbacks.h
 *
 * wilson
 * 11/4/22
 * 12:38 AM
 *
 * Description:
 *
 **********************************************************************************************************************/

#ifndef MAIN_CONTROLLER_HAL_CALLBACK_H
#define MAIN_CONTROLLER_HAL_CALLBACK_H

/* c/c++ includes */

/* stm32 includes */

/* third-party includes */

/* hal includes */
#include "hal_spi.h"
/* driver includes */

/* rtos abstraction includes */

/* sys op includes */

/* meta structure includes */


void hal_callback_spi_rx_tx_complete(spi::module_t *arg_module);
void hal_callback_spi_error(spi::module_t *arg_module);

#endif //MAIN_CONTROLLER_HAL_CALLBACK_H
