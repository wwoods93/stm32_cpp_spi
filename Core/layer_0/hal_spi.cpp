/***********************************************************************************************************************
 * hal_spi.cpp
 *
 * wilson
 * 10/16/22
 * 9:41 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

/* c/c++ includes */
#include <cstdlib>
#include <cstring>
#include <memory>
/* stm32 includes */

/* third-party includes */

/* layer_0 includes */
#include "hal.h"
#include "hal_wrapper.h"
//#include "hal_callback.h"
#include "hal_spi_definitions.h"
/* layer_1_rtosal includes */
#include "rtosal.h"
#include "rtosal_wrapper.h"
/* layer_1 includes */

/* layer_3_control includes */

/* layer_4_sys_op includes */

/* layer_n_meta_structure includes */

/* hal_spi header */
#include "hal_spi.h"

spi::procedure_status_t spi::initialize(module_t* arg_module, uint8_t arg_instance_id, hal::timer_handle_t* arg_timeout_timer_handle)
{
    procedure_status_t status = PROCEDURE_STATUS_OK;

    module = arg_module;
    if (module == nullptr)
    {
        set_config_status_bit(CONFIG_STATUS_BIT_MODULE);
        return PROCEDURE_STATUS_ERROR;
    }
    module->status = MODULE_STATUS_RESET;

    switch (arg_instance_id)
    {
        case SPI_1_ID:
        {
            module->register_map = SPI_1;
            module->config.baud_rate_prescaler = SPI_CONFIG_BAUD_RATE_PRESCALER_4;
            break;
        }
        case SPI_2_ID:
        {
            module->register_map = SPI_2;
            module->config.baud_rate_prescaler = SPI_CONFIG_BAUD_RATE_PRESCALER_32;

            break;
        }
        case SPI_3_ID:
        {
            module->register_map = SPI_3;
            break;
        }
        case SPI_4_ID:
        {
            module->register_map = SPI_4;
            break;
        }
        default:
        {
            set_error_bit(SPI_ERROR_CONFIG_INVALID);
            break;
        }
    }

    timeout_timer_handle = arg_timeout_timer_handle;

    module->config.clock_phase = SPI_CONFIG_CLOCK_PHASE_TRAILING_EDGE;
    module->config.clock_polarity = SPI_CONFIG_CLOCK_POLARITY_LOW;
    module->config.mode = SPI_CONFIG_MODE_CONTROLLER;
    module->config.data_size = SPI_CONFIG_DATA_SIZE_8_BIT;
    module->config.first_bit_setting = SPI_CONFIG_DATA_MSB_FIRST;
    module->config.chip_select_setting = SPI_CONFIG_CHIP_SELECT_SOFTWARE;
    module->config.direction = SPI_CONFIG_DIRECTION_2_LINE;
    module->config.ti_mode = SPI_CONFIG_TI_MODE_DISABLE;
    module->config.crc_calculation = SPI_CONFIG_CRC_CALCULATION_DISABLE;
    module->config.crc_polynomial = 7U;
    module->rx_data_ready_flag = 0U;

    if (module->register_map != SPI_1 && module->register_map != SPI_2 && module->register_map != SPI_3 && module->register_map != SPI_4)
    {
        set_config_status_bit(CONFIG_STATUS_BIT_INSTANCE);
        status = PROCEDURE_STATUS_ERROR;
    }
    if (module->config.clock_phase != SPI_CONFIG_CLOCK_PHASE_LEADING_EDGE && module->config.clock_phase != SPI_CONFIG_CLOCK_PHASE_TRAILING_EDGE)
    {
        set_config_status_bit(CONFIG_STATUS_BIT_CLOCK_PHASE);
        status = PROCEDURE_STATUS_ERROR;
    }
    if (module->config.clock_polarity != SPI_CONFIG_CLOCK_POLARITY_LOW && module->config.clock_polarity != SPI_CONFIG_CLOCK_POLARITY_HIGH)
    {
        set_config_status_bit(CONFIG_STATUS_BIT_CLOCK_POLARITY);
        status = PROCEDURE_STATUS_ERROR;
    }
    if (module->config.mode != SPI_CONFIG_MODE_CONTROLLER && module->config.mode != SPI_CONFIG_MODE_PERIPHERAL)
    {
        set_config_status_bit(CONFIG_STATUS_BIT_MODE);
        status = PROCEDURE_STATUS_ERROR;
    }
    if (module->config.baud_rate_prescaler    != SPI_CONFIG_BAUD_RATE_PRESCALER_2   && module->config.baud_rate_prescaler != SPI_CONFIG_BAUD_RATE_PRESCALER_4
        && module->config.baud_rate_prescaler != SPI_CONFIG_BAUD_RATE_PRESCALER_8   && module->config.baud_rate_prescaler != SPI_CONFIG_BAUD_RATE_PRESCALER_16
        && module->config.baud_rate_prescaler != SPI_CONFIG_BAUD_RATE_PRESCALER_32  && module->config.baud_rate_prescaler != SPI_CONFIG_BAUD_RATE_PRESCALER_64
        && module->config.baud_rate_prescaler != SPI_CONFIG_BAUD_RATE_PRESCALER_128 && module->config.baud_rate_prescaler != SPI_CONFIG_BAUD_RATE_PRESCALER_256)
    {
        set_config_status_bit(CONFIG_STATUS_BIT_BAUD_RATE);
        status = PROCEDURE_STATUS_ERROR;
    }
    if (module->config.first_bit_setting != SPI_CONFIG_DATA_MSB_FIRST && module->config.first_bit_setting != SPI_CONFIG_DATA_LSB_FIRST)
    {
        set_config_status_bit(CONFIG_STATUS_BIT_FIRST_BIT_SETTING);
        status = PROCEDURE_STATUS_ERROR;
    }
    if (module->config.chip_select_setting != SPI_CONFIG_CHIP_SELECT_SOFTWARE
        && module->config.chip_select_setting != SPI_CONFIG_CHIP_SELECT_HARDWARE_INPUT
        && module->config.chip_select_setting != SPI_CONFIG_CHIP_SELECT_HARDWARE_OUTPUT)
    {
        set_config_status_bit(CONFIG_STATUS_BIT_CHIP_SELECT);
        status = PROCEDURE_STATUS_ERROR;
    }
    if (module->config.direction != SPI_CONFIG_DIRECTION_2_LINE
        && module->config.direction != SPI_CONFIG_DIRECTION_2_LINE_RX_ONLY
        && module->config.direction != SPI_CONFIG_DIRECTION_1_LINE)
    {
        set_config_status_bit(CONFIG_STATUS_BIT_DIRECTION);
        status = PROCEDURE_STATUS_ERROR;
    }
    if (module->config.ti_mode != SPI_CONFIG_TI_MODE_DISABLE)
    {
        set_config_status_bit(CONFIG_STATUS_BIT_FRAME_FORMAT);
        status = PROCEDURE_STATUS_ERROR;
    }
    if (module->config.crc_calculation != SPI_CONFIG_CRC_CALCULATION_DISABLE)
    {
        set_config_status_bit(CONFIG_STATUS_BIT_CRC);
        status = PROCEDURE_STATUS_ERROR;
    }
    if (module->config.data_size != SPI_CONFIG_DATA_SIZE_8_BIT && module->config.data_size != SPI_CONFIG_DATA_SIZE_16_BIT)
    {
        set_config_status_bit(CONFIG_STATUS_BIT_DATA_SIZE);
        status = PROCEDURE_STATUS_ERROR;
    }

    if (status == PROCEDURE_STATUS_ERROR)
    {
        return status;
    }

    switch (arg_instance_id)
    {
        case SPI_1_ID:
        {
            hal::spi_1_msp_initialize();
            break;
        }
        case SPI_2_ID:
        {
            hal::spi_2_msp_initialize();
            break;
        }
        default:
        {
            set_config_status_bit(CONFIG_STATUS_BIT_MSP_INIT);
            status = PROCEDURE_STATUS_ERROR;
            break;
        }
    }

    module->status = MODULE_STATUS_BUSY;
    disable_module();

    set_register_bit(CONTROL_REG_1_ID, (
        (module->config.clock_phase & SPI_CR1_BIT_CLOCK_PHASE) |
        (module->config.clock_polarity & SPI_CR1_BIT_CLOCK_POLARITY) |
        (module->config.mode & (SPI_CR1_BIT_CONTROLLER_MODE | SPI_CR1_BIT_INTERNAL_CHIP_SELECT)) |
        (module->config.baud_rate_prescaler & SPI_CR1_BIT_BAUD_RATE) |
        (module->config.first_bit_setting & SPI_CR1_BIT_LSB_FIRST) |
        (module->config.chip_select_setting & SPI_CR1_BIT_SOFTWARE_CHIP_SELECT) |
        (module->config.data_size & SPI_CR1_BIT_DATA_FRAME_FORMAT) |
        (module->config.crc_calculation & SPI_CR1_BIT_CRC_ENABLE) |
        (module->config.direction & (SPI_CR1_BIT_RECEIVE_ONLY | SPI_CR1_BIT_BIDIRECTIONAL_MODE))));

    set_register_bit(CONTROL_REG_2_ID,
                     (((module->config.chip_select_setting >> 16U) & SPI_CR2_BIT_CHIP_SELECT_OUTPUT_ENABLE) |
                      (module->config.ti_mode & SPI_CR2_BIT_FRAME_FORMAT)));

    module->register_map->I2S_CONFIG_REG &= ~(0x01UL << 11U);



    switch (module->config.direction)
    {
        case SPI_CONFIG_DIRECTION_2_LINE:
        {
            if (module->config.data_size == SPI_CONFIG_DATA_SIZE_8_BIT)
            {
                tx_isr_id = ISR_TX_8_BIT_2_LINE;
                rx_isr_id = ISR_RX_8_BIT_2_LINE;

            }
            else if (module->config.data_size == SPI_CONFIG_DATA_SIZE_16_BIT)
            {
                tx_isr_id = ISR_TX_16_BIT_2_LINE;
                rx_isr_id = ISR_RX_16_BIT_2_LINE;
            }
            break;
        }
        case SPI_CONFIG_DIRECTION_2_LINE_RX_ONLY:
        {
            if (module->config.data_size == SPI_CONFIG_DATA_SIZE_8_BIT)
            {
                rx_isr_id = ISR_RX_8_BIT_2_LINE;

            }
            else if (module->config.data_size == SPI_CONFIG_DATA_SIZE_16_BIT)
            {
                rx_isr_id = ISR_RX_16_BIT_2_LINE;
            }
            break;
        }
        case SPI_CONFIG_DIRECTION_1_LINE:
        {
            if (module->config.data_size == SPI_CONFIG_DATA_SIZE_8_BIT)
            {
                tx_isr_id = ISR_TX_8_BIT_2_LINE;

            }
            else if (module->config.data_size == SPI_CONFIG_DATA_SIZE_16_BIT)
            {
                tx_isr_id = ISR_TX_16_BIT_2_LINE;
            }
            break;
        }
        default:
        {
            set_config_status_bit(CONFIG_STATUS_BIT_ISR_INIT);
            status = PROCEDURE_STATUS_ERROR;
            break;
        }
    }

    std::shared_ptr<uint8_t[]> rx_pointer_tmp(new uint8_t[TX_SIZE_MAX]);
    rx_pointer = rx_pointer_tmp.get();

    if (rx_pointer == nullptr)
    {
        set_config_status_bit(CONFIG_STATUS_BIT_RX_PTR_INIT);
        status = PROCEDURE_STATUS_ERROR;
    }

    if (timeout_timer_handle == nullptr)
    {
        set_config_status_bit(CONFIG_STATUS_BIT_TIMEOUT_TIMER);
        status = PROCEDURE_STATUS_ERROR;
    }

    if (status != PROCEDURE_STATUS_OK)
    {
        return status;
    }

    module->error_bit_field   = SPI_ERROR_NONE;
    module->status        = MODULE_STATUS_READY;
    return PROCEDURE_STATUS_OK;
}

spi::procedure_status_t spi::create_channel(int16_t& arg_channel_id, hal::gpio_t* arg_chip_select_port, uint16_t arg_chip_select_pin, uint8_t arg_is_inter_task, rtosal::message_queue_handle_t arg_tx_message_queue, rtosal::message_queue_handle_t arg_rx_message_queue)
{
    procedure_status_t status = PROCEDURE_STATUS_OK;

    arg_channel_id = ID_INVALID;

    int16_t new_channel_id = assign_next_available_channel_id();
    channel_array[new_channel_id] = 1U;

    channel_t new_channel;
    memset(&new_channel, '\0', sizeof(channel_t));

    new_channel.channel_id = new_channel_id;
    new_channel.chip_select.port = arg_chip_select_port;
    new_channel.chip_select.pin  = arg_chip_select_pin;
    new_channel.is_inter_task    = arg_is_inter_task;
    new_channel.tx_message_queue = arg_tx_message_queue;
    new_channel.rx_message_queue = arg_rx_message_queue;

    if (new_channel.channel_id == ID_INVALID)
    {
        set_error_bit(SPI_ERROR_CHANNEL_ID | SPI_ERROR_CREATE_CHANNEL);
        status = PROCEDURE_STATUS_ERROR;
    }

    if ((new_channel.chip_select.port != PORT_A && new_channel.chip_select.port != PORT_B
    && new_channel.chip_select.port != PORT_C && new_channel.chip_select.port != PORT_D
    && new_channel.chip_select.port != PORT_E && new_channel.chip_select.port != PORT_F
    && new_channel.chip_select.port != PORT_G && new_channel.chip_select.port != PORT_H)
    || (new_channel.chip_select.pin & PIN_ALL) == 0U)
    {
        set_error_bit(SPI_ERROR_CHIP_SELECT | SPI_ERROR_CREATE_CHANNEL);
        status = PROCEDURE_STATUS_ERROR;
    }

    if (new_channel.is_inter_task)
    {
        if (new_channel.tx_message_queue == nullptr)
        {
            set_error_bit(SPI_ERROR_INTER_TASK_QUEUE_FROM_CLIENT | SPI_ERROR_CREATE_CHANNEL);
            status = PROCEDURE_STATUS_ERROR;
        }

        if (new_channel.rx_message_queue == nullptr)
        {
            set_error_bit(SPI_ERROR_INTER_TASK_QUEUE_TO_CLIENT | SPI_ERROR_CREATE_CHANNEL);
            status = PROCEDURE_STATUS_ERROR;
        }
    }

    if (status != PROCEDURE_STATUS_OK)
    {
        return status;
    }

    switch (new_channel_id)
    {
        case CHANNEL_0:
        {
            memset(&channel_list.channel_0, '\0', sizeof(channel_t));
            memcpy(&channel_list.channel_0, &new_channel, sizeof(channel_t));
            break;
        }
        case CHANNEL_1:
        {
            memset(&channel_list.channel_1, '\0', sizeof(channel_t));
            memcpy(&channel_list.channel_1, &new_channel, sizeof(channel_t));
            break;
        }
        case CHANNEL_2:
        {
            memset(&channel_list.channel_2, '\0', sizeof(channel_t));
            memcpy(&channel_list.channel_2, &new_channel, sizeof(channel_t));
            break;
        }
        case CHANNEL_3:
        {
            memset(&channel_list.channel_3, '\0', sizeof(channel_t));
            memcpy(&channel_list.channel_3, &new_channel, sizeof(channel_t));
            break;
        }
        case CHANNEL_4:
        {
            memset(&channel_list.channel_4, '\0', sizeof(channel_t));
            memcpy(&channel_list.channel_4, &new_channel, sizeof(channel_t));
            break;
        }
        case CHANNEL_5:
        {
            memset(&channel_list.channel_5, '\0', sizeof(channel_t));
            memcpy(&channel_list.channel_5, &new_channel, sizeof(channel_t));
            break;
        }
        case CHANNEL_6:
        {
            memset(&channel_list.channel_6, '\0', sizeof(channel_t));
            memcpy(&channel_list.channel_6, &new_channel, sizeof(channel_t));
            break;
        }
        case CHANNEL_7:
        {
            memset(&channel_list.channel_7, '\0', sizeof(channel_t));
            memcpy(&channel_list.channel_7, &new_channel, sizeof(channel_t));
            break;
        }
        default:
        {
            set_error_bit(SPI_ERROR_CREATE_CHANNEL);
            status = PROCEDURE_STATUS_ERROR;
            return status;
            break;
        }
    }
    hal::gpio_write_pin(new_channel.chip_select.port, new_channel.chip_select.pin, CHIP_SELECT_RESET);

    arg_channel_id = new_channel_id;

    return PROCEDURE_STATUS_OK;
}

int16_t spi::send(uint8_t* arg_tx_bytes, uint8_t* arg_bytes_per_tx, int16_t arg_channel_id)
{
    if (channel_array[arg_channel_id] == 1U)
    {

        channel_t channel;
        get_channel_by_channel_id(channel, arg_channel_id);

        packet_t packet;
        memset(&packet, '\0', sizeof(packet_t));
        packet.packet_id = next_available_packet_id++;
        packet.channel_id = arg_channel_id;
        packet.chip_select.port = channel.chip_select.port;
        packet.chip_select.pin = channel.chip_select.pin;

        memcpy(&packet.bytes_per_transaction, arg_bytes_per_tx, sizeof(packet.bytes_per_transaction));
        memcpy(&packet.tx_bytes, arg_tx_bytes, sizeof(packet.tx_bytes));
        send_buffer.push(packet);
        return packet.channel_id;
    }
    return ID_INVALID;
}

spi::procedure_status_t spi::receive(uint8_t* arg_rx_bytes, int16_t arg_channel_id)
{
    procedure_status_t status = PROCEDURE_STATUS_OK;
    packet_t packet;
    switch(arg_channel_id)
    {
        case CHANNEL_0:
        {
            memcpy(&packet, &channel_0_rx_packet, sizeof(packet_t));
            break;
        }
        case CHANNEL_1:
        {
            memcpy(&packet, &channel_1_rx_packet, sizeof(packet_t));
            break;
        }
        case CHANNEL_2:
        {
            memcpy(&packet, &channel_2_rx_packet, sizeof(packet_t));
            break;
        }
        case CHANNEL_3:
        {
            memcpy(&packet, &channel_3_rx_packet, sizeof(packet_t));
            break;
        }
        case CHANNEL_4:
        {
            memcpy(&packet, &channel_4_rx_packet, sizeof(packet_t));
            break;
        }
        case CHANNEL_5:
        {
            memcpy(&packet, &channel_5_rx_packet, sizeof(packet_t));
            break;
        }
        case CHANNEL_6:
        {
            memcpy(&packet, &channel_6_rx_packet, sizeof(packet_t));
            break;
        }
        case CHANNEL_7:
        {
            memcpy(&packet, &channel_7_rx_packet, sizeof(packet_t));
            break;
        }
        default:
        {
            status = PROCEDURE_STATUS_ERROR;
            break;
        }
    }
    memset(&arg_rx_bytes, '\0', sizeof(arg_rx_bytes));
    memcpy(&arg_rx_bytes, &packet.rx_bytes, sizeof(arg_rx_bytes));

    return status;
}

int16_t spi::send_inter_task(uint8_t* arg_tx_bytes, uint8_t* arg_bytes_per_tx, int16_t arg_channel_id)
{
    if (channel_array[arg_channel_id] == 1U)
    {
        packet_t packet;
        channel_t channel;
        memset(&packet, '\0', sizeof(packet_t));
        packet.packet_id = next_available_packet_id++;
        packet.channel_id = arg_channel_id;
        memcpy(&packet.bytes_per_transaction, arg_bytes_per_tx, sizeof(packet.bytes_per_transaction));
        memcpy(&packet.tx_bytes, arg_tx_bytes, sizeof(packet.tx_bytes));

        get_channel_by_channel_id(channel, arg_channel_id);
        if (rtosal::message_queue_send(channel.tx_message_queue, &packet, 0U) == rtosal::OS_OK)
        {
            return packet.packet_id;
        }
    }
    return ID_INVALID;
}

spi::procedure_status_t spi::receive_inter_task(uint8_t* arg_rx_bytes, int16_t arg_channel_id)
{
    procedure_status_t status = PROCEDURE_STATUS_TIMEOUT;
    if (channel_array[arg_channel_id] == 1U)
    {
        channel_t channel;

        get_channel_by_channel_id(channel, arg_channel_id);

        packet_t packet;

        if (rtosal::message_queue_receive( channel.rx_message_queue, &packet, 0U) == rtosal::OS_OK)
        {
            memset(arg_rx_bytes, '\0', TX_SIZE_MAX);
            memcpy(arg_rx_bytes, &packet.rx_bytes, TX_SIZE_MAX);
            status = PROCEDURE_STATUS_OK;
        }
    }

    return status;
}

spi::procedure_status_t spi::receive_inter_task_transaction_requests()
{
    procedure_status_t status = PROCEDURE_STATUS_OK;
    channel_t channel;
    packet_t packet;
    common_packet_t common_packet;
    for (uint8_t index = 0U; index < SPI_CHANNELS_MAX; ++index)
    {
        if (channel_array[index] == 1U)
        {
            get_channel_by_channel_id(channel, index);
            if (channel.is_inter_task)
            {
                if (rtosal::message_queue_receive(channel.tx_message_queue, &common_packet, 0U) == rtosal::OS_OK)
                {
                    memset(&packet, '\0', sizeof(packet_t));
                    memcpy(&packet.tx_bytes, common_packet.bytes, sizeof(packet.tx_bytes));
                    memcpy(&packet.bytes_per_transaction, common_packet.bytes_per_transaction, sizeof(common_packet.bytes_per_transaction));
                    packet.channel_id = common_packet.channel_id;
                    packet.packet_id = ++next_available_packet_id;
                    packet.chip_select.port = channel.chip_select.port;
                    packet.chip_select.pin = channel.chip_select.pin;
                    send_buffer.push(packet);
                }
                else
                {
                    set_error_bit(SPI_ERROR_INTER_TASK_QUEUE_FROM_CLIENT);
                    status = PROCEDURE_STATUS_ERROR;
                }
            }
        }
    }
    return status;
}

spi::procedure_status_t spi::process_send_buffer()
{
    procedure_status_t status = PROCEDURE_STATUS_OK;

    static uint8_t current_transaction = 0U;
    static uint8_t bus_ready = 0U;

    if (!send_buffer.empty())
    {
        if (process_send_buffer_state == SEND_STATE_BEGIN)
        {
            memset(&active_packet, '\0', sizeof(packet_t));
            memcpy(&active_packet, &send_buffer.front(), sizeof(packet_t));

            module->chip_select.port = active_packet.chip_select.port;
            module->chip_select.pin = active_packet.chip_select.pin;
            memset(&active_packet.rx_bytes, '\0', sizeof(active_packet.rx_bytes));

            packet_index = 0U;
            transaction_byte_count = 0U;
            ++packets_requested_count;
            current_transaction = 0U;
            bus_ready = 1U;
            process_send_buffer_state = SEND_STATE_IN_PROGRESS;
        }

        if (process_send_buffer_state == SEND_STATE_IN_PROGRESS)
        {
            while (current_transaction < TX_SIZE_MAX)
            {
                transaction_byte_count = active_packet.bytes_per_transaction[current_transaction];
                if (transaction_byte_count != 0U && bus_ready)
                {
                    process_send_buffer_timeout_start = get_timer_count(timeout_timer_handle);
                    spi_transmit_receive_interrupt(&active_packet.tx_bytes[packet_index], rx_pointer, transaction_byte_count);
                    bus_ready = 0U;
                }

                if (get_timer_count(timeout_timer_handle) - process_send_buffer_timeout_start > PROCESS_SEND_BUFFER_TIMEOUT)
                {
                    set_error_bit(SPI_ERROR_TRANSACTION_TIMEOUT);
                    status = PROCEDURE_STATUS_TIMEOUT;
                    break;
                }

                if (module->rx_data_ready_flag)
                {
                    bus_ready = 1U;
                    if (++current_transaction >= TX_SIZE_MAX)
                    {
                        process_send_buffer_state = SEND_STATE_COMPLETE;
                    }
                }
                else
                {
                    break;
                }

            }
        }

        if (process_send_buffer_state == SEND_STATE_COMPLETE)
        {
            ++packets_received_count;
            send_buffer.pop();
            push_active_packet_to_return_buffer();
            memset(&active_packet, '\0', sizeof(packet_t));
            active_packet.channel_id = ID_INVALID;
            process_send_buffer_state = SEND_STATE_BEGIN;
        }
    }

    return status;
}

spi::procedure_status_t spi::process_return_buffers(spi::packet_t& arg_packet)
{
    procedure_status_t status = PROCEDURE_STATUS_ERROR;
    uint8_t buffer_accessed = 0U;
    channel_t channel;
    packet_t packet;


    memset(&packet, '\0', sizeof(packet_t));
    memset(&arg_packet, '\0', sizeof(packet_t));
    arg_packet.packet_id = ID_INVALID;
    arg_packet.channel_id = ID_INVALID;

    for (int16_t index = 0U; index < next_available_channel_id; ++index)
    {
        if (channel_array[index] == 1U)
        {
            switch (index)
            {
                case CHANNEL_0:
                {
                    if (!return_buffer_0.empty())
                    {
                        memcpy(&packet, &return_buffer_0.front(), sizeof(packet_t));
                        return_buffer_0.pop();
                        buffer_accessed = 1U;
                    }

                    break;
                }
                case CHANNEL_1:
                {
                    if (!return_buffer_1.empty())
                    {
                        memcpy(&packet, &return_buffer_1.front(), sizeof(packet_t));
                        return_buffer_1.pop();
                        buffer_accessed = 1U;
                    }

                    break;
                }
                case CHANNEL_2:
                {
                    if (!return_buffer_2.empty())
                    {
                        memcpy(&packet, &return_buffer_2.front(), sizeof(packet_t));
                        return_buffer_2.pop();
                        buffer_accessed = 1U;
                    }

                    break;
                }
                case CHANNEL_3:
                {
                    if (!return_buffer_3.empty())
                    {
                        memcpy(&packet, &return_buffer_3.front(), sizeof(packet_t));
                        return_buffer_3.pop();
                        buffer_accessed = 1U;
                    }

                    break;
                }
                case CHANNEL_4:
                {
                    if (!return_buffer_4.empty())
                    {
                        memcpy(&packet, &return_buffer_4.front(), sizeof(packet_t));
                        return_buffer_4.pop();
                        buffer_accessed = 1U;
                    }

                    break;
                }
                case CHANNEL_5:
                {
                    if (!return_buffer_5.empty())
                    {
                        memcpy(&packet, &return_buffer_5.front(), sizeof(packet_t));
                        return_buffer_5.pop();
                        buffer_accessed = 1U;
                    }

                    break;
                }
                case CHANNEL_6:
                {
                    if (!return_buffer_6.empty())
                    {
                        memcpy(&packet, &return_buffer_6.front(), sizeof(packet_t));
                        return_buffer_6.pop();
                        buffer_accessed = 1U;
                    }

                    break;
                }
                case CHANNEL_7:
                {
                    if (!return_buffer_7.empty())
                    {
                        memcpy(&packet, &return_buffer_7.front(), sizeof(packet_t));
                        return_buffer_7.pop();
                        buffer_accessed = 1U;
                    }

                    break;
                }
                default:
                {
                    set_error_bit(SPI_ERROR_RETURN_BUFFER);
                    status = PROCEDURE_STATUS_ERROR;
                    break;
                }
            }

            if (buffer_accessed)
            {
                get_channel_by_channel_id(channel, index);

                if (channel.is_inter_task)
                {
                    status = send_inter_task_transaction_result(channel.rx_message_queue, packet);
                }
                else
                {
                    post_channel_rx_result(packet, channel.channel_id);
                }
            }
        }
    }

    return status;
}

uint32_t spi::get_packets_requested_count() const
{
    return packets_requested_count;
}

uint32_t spi::get_packets_received_count() const
{
    return packets_received_count;
}

void tx_isr(spi arg_object, struct spi::module_struct *arg_module)
{
    switch (arg_object.tx_isr_id)
    {
        case spi::ISR_TX_8_BIT_2_LINE:
        {
            *(volatile uint8_t *)(&arg_module->register_map->DATA_REG) = (*arg_module->tx_buffer_ptr);
            arg_module->tx_buffer_ptr++;
            arg_module->tx_transfer_counter--;
            if (arg_module->tx_transfer_counter == 0U)
            {
                arg_object.disable_interrupts(SPI_CR2_BIT_TX_BUFFER_EMPTY_INTERRUPT_ENABLE);
                if (arg_module->rx_transfer_counter == 0U)
                {
                    arg_object.close_isr(spi::TX_RX);
                }
            }
            break;
        }
        case spi::ISR_TX_16_BIT_2_LINE:
        {
            arg_module->register_map->DATA_REG = *((uint16_t *)arg_module->tx_buffer_ptr);
            arg_module->tx_buffer_ptr += sizeof(uint16_t);
            arg_module->tx_transfer_counter--;

            if (arg_module->tx_transfer_counter == 0U)
            {
                arg_object.disable_interrupts(SPI_CR2_BIT_TX_BUFFER_EMPTY_INTERRUPT_ENABLE);
                if (arg_module->rx_transfer_counter == 0U)
                {
                    arg_object.close_isr(spi::TX_RX);
                }
            }
            break;
        }
        default:
        {
            break;
        }
    }
}

void rx_isr(spi arg_object, struct spi::module_struct *arg_module)
{
    switch (arg_object.rx_isr_id)
    {
        case spi::ISR_RX_8_BIT_2_LINE:
        {
            *(arg_module->rx_buffer_ptr) = *((volatile uint8_t *)&arg_module->register_map->DATA_REG);     // receive data in 8-bit mode
            arg_module->rx_buffer_ptr++;
            arg_module->rx_transfer_counter--;

            if (arg_module->rx_transfer_counter == 0U)
            {
                arg_object.disable_interrupts(SPI_CR2_BIT_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_CR2_BIT_ERROR_INTERRUPT_ENABLE);
                if (arg_module->tx_transfer_counter == 0U)
                {
                    arg_object.close_isr(spi::TX_RX);
                }
            }
            break;
        }
        case spi::ISR_RX_16_BIT_2_LINE:
        {
            *((uint16_t *)arg_module->rx_buffer_ptr) = (uint16_t)(arg_module->register_map->DATA_REG);
            arg_module->rx_buffer_ptr += sizeof(uint16_t);
            arg_module->rx_transfer_counter--;

            if (arg_module->rx_transfer_counter == 0U)
            {
                arg_object.disable_interrupts(SPI_CR2_BIT_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE);
                if (arg_module->tx_transfer_counter == 0U)
                {
                    arg_object.close_isr(spi::TX_RX);
                }
            }
            break;
        }
        default:
        {
            break;
        }
    }
}

void spi_irq_handler(spi* arg_object)
{
    if (arg_object->check_interrupt_source(SPI_CR2_BIT_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE) == BIT_SET)
    {
        if (arg_object->get_status_register_bit(SPI_SR_BIT_RX_BUFFER_NOT_EMPTY) == BIT_SET)
        {
            if (arg_object->get_status_register_bit(SPI_SR_BIT_OVERRUN) != BIT_SET)
            {
                rx_isr(*arg_object, arg_object->module);
                return;
            }
        }
    }

    if (arg_object->check_interrupt_source(SPI_CR2_BIT_TX_BUFFER_EMPTY_INTERRUPT_ENABLE) == BIT_SET)
    {
        if (arg_object->get_status_register_bit(SPI_SR_BIT_TX_BUFFER_EMPTY) == BIT_SET)
        {
            tx_isr(*arg_object, arg_object->module);
            return;
        }
    }

    if (arg_object->check_interrupt_source(SPI_CR2_BIT_ERROR_INTERRUPT_ENABLE) == BIT_SET)
    {
        if (arg_object->get_status_register_bit(SPI_SR_BIT_OVERRUN) == BIT_SET)
        {
            if (arg_object->module->status != spi::MODULE_STATUS_BUSY_TX)
            {
                arg_object->set_error_bit(spi::SPI_ERROR_OVERRUN);
            }
            arg_object->clear_overrun_flag();
            arg_object->disable_interrupts(SPI_CR2_BIT_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_CR2_BIT_TX_BUFFER_EMPTY_INTERRUPT_ENABLE | SPI_CR2_BIT_ERROR_INTERRUPT_ENABLE);
        }

        if (arg_object->get_status_register_bit(SPI_SR_BIT_MODE_FAULT) == BIT_SET)
        {
            arg_object->set_error_bit(spi::SPI_ERROR_MODE_FAULT);
            arg_object->clear_mode_fault_flag();
            arg_object->disable_interrupts(SPI_CR2_BIT_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_CR2_BIT_TX_BUFFER_EMPTY_INTERRUPT_ENABLE | SPI_CR2_BIT_ERROR_INTERRUPT_ENABLE);
        }
        arg_object->handle_transaction_error();
        arg_object->module->status = spi::MODULE_STATUS_READY;

        return;
    }
}

spi::procedure_status_t spi::spi_transmit_receive_interrupt(uint8_t *arg_tx_data_ptr, uint8_t *arg_rx_data_ptr, uint16_t arg_packet_size)
{
    uint8_t spi_procedure_error = SPI_PROCEDURE_ERROR_NONE;
    channel_t channel_tmp;

    if ((arg_tx_data_ptr == nullptr) || (arg_rx_data_ptr == nullptr) || (arg_packet_size == 0U))
    {
        spi_procedure_error = SPI_PROCEDURE_STATE_DATA_ERROR;
    }

    if (verify_communication_direction(SPI_CONFIG_DIRECTION_2_LINE) == PROCEDURE_STATUS_OK)
    {
        if (lock_module() == PROCEDURE_STATUS_OK)
        {
            if ((module->status != MODULE_STATUS_READY) && (module->config.mode != SPI_CONFIG_MODE_CONTROLLER || module->config.direction != SPI_CONFIG_DIRECTION_2_LINE || module->status != MODULE_STATUS_BUSY_RX))
            {
                spi_procedure_error = SPI_PROCEDURE_STATE_BUS_ERROR;
            }
            else if (module->status != MODULE_STATUS_BUSY_RX)
            {
                module->status = MODULE_STATUS_BUSY_TX_RX;
            }

            module->error_bit_field = SPI_ERROR_NONE;
            module->tx_buffer_ptr = (uint8_t *)arg_tx_data_ptr;
            module->rx_buffer_ptr = (uint8_t *)arg_rx_data_ptr;
            module->tx_transfer_counter = arg_packet_size;
            module->rx_transfer_counter = arg_packet_size;

            module->chip_select_port = active_packet.chip_select.port;
            module->chip_select_pin = active_packet.chip_select.pin;

            hal::gpio_write_pin(active_packet.chip_select.port, active_packet.chip_select.pin, (GPIO_PinState) CHIP_SELECT_SET);

            enable_interrupts(SPI_CR2_BIT_TX_BUFFER_EMPTY_INTERRUPT_ENABLE | SPI_CR2_BIT_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_CR2_BIT_ERROR_INTERRUPT_ENABLE);

            if ((module->register_map->CONTROL_REG_1 & SPI_CR1_BIT_SPI_ENABLE) != SPI_CR1_BIT_SPI_ENABLE)
            {
                enable_module();
            }

            unlock_module();
        }
        else
        {
            spi_procedure_error = SPI_PROCEDURE_STATE_BUS_ERROR;
        }
    }
    else
    {
        spi_procedure_error = SPI_PROCEDURE_STATE_DATA_ERROR;
    }

    if (spi_procedure_error == SPI_PROCEDURE_STATE_BUS_ERROR)  { return PROCEDURE_STATUS_BUSY;  }
    if (spi_procedure_error == SPI_PROCEDURE_STATE_DATA_ERROR) { return PROCEDURE_STATUS_ERROR; }

    return PROCEDURE_STATUS_OK;
}

spi::procedure_status_t spi::send_inter_task_transaction_result(rtosal::message_queue_handle_t arg_message_queue_id, packet_t& arg_packet)
{
    procedure_status_t status = PROCEDURE_STATUS_OK;

    if (rtosal::message_queue_send(arg_message_queue_id, &arg_packet, 0U) != rtosal::OS_OK)
    {
        set_error_bit(SPI_ERROR_INTER_TASK_QUEUE_TO_CLIENT);
        status = PROCEDURE_STATUS_ERROR;
    }

    return status;
}

void spi::close_isr(transaction_t arg_transaction_type)
{
    uint32_t active_interrupts = 0;

    switch(arg_transaction_type)
    {
        case TX_RX:
        {
            active_interrupts = SPI_CR2_BIT_ERROR_INTERRUPT_ENABLE;
            flag_timeout(SPI_SR_BIT_TX_BUFFER_EMPTY, BIT_SET);
            break;
        }
        case TX_ONLY:
        {
            active_interrupts = SPI_CR2_BIT_TX_BUFFER_EMPTY_INTERRUPT_ENABLE | SPI_CR2_BIT_ERROR_INTERRUPT_ENABLE;
            flag_timeout(SPI_SR_BIT_TX_BUFFER_EMPTY, BIT_SET);
            break;
        }
        case RX_ONLY:
        {
            active_interrupts = SPI_CR2_BIT_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_CR2_BIT_ERROR_INTERRUPT_ENABLE;
            break;

        }
        default:
        {
            break;
        }
    }

    disable_interrupts(active_interrupts);

    if (wait_for_pending_flags_and_end_transaction(arg_transaction_type) != PROCEDURE_STATUS_OK)
    {
        set_error_bit(SPI_ERROR_WAITING_FOR_FLAG);
    }

    if (module->config.direction == SPI_CONFIG_DIRECTION_2_LINE)
    {
        clear_overrun_flag();
    }

    if (module->error_bit_field != SPI_ERROR_NONE)
    {
        module->status = MODULE_STATUS_READY;
        handle_transaction_error();
    }
    else
    {
        if (arg_transaction_type == TX_RX)
        {
            handle_tx_rx_success();
        }
        else if (arg_transaction_type == TX_ONLY)
        {
            handle_tx_success();
        }
        else if (arg_transaction_type == RX_ONLY || (arg_transaction_type == TX_RX && module->status == MODULE_STATUS_BUSY_RX))
        {
            handle_rx_success();
        }

        module->status = MODULE_STATUS_READY;
    }
}

spi::procedure_status_t spi::verify_communication_direction(uint32_t arg_intended_direction) const
{
    procedure_status_t status = PROCEDURE_STATUS_OK;

    if (module->config.direction != arg_intended_direction)
    {
        status = PROCEDURE_STATUS_ERROR;
    }

    return status;
}

int16_t spi::assign_next_available_channel_id()
{
    int16_t channel_id = ID_INVALID;

    if (next_available_channel_id <= SPI_CHANNELS_MAX)
    {
        channel_id = next_available_channel_id;
        ++next_available_channel_id;
    }

    return channel_id;
}

void spi::get_channel_by_channel_id(channel_t& arg_channel, int16_t arg_channel_id)
{
    memset(&arg_channel, '\0', sizeof(channel_t));

    switch (arg_channel_id)
    {
        case CHANNEL_0:
        {
            memcpy(&arg_channel, &channel_list.channel_0, sizeof(channel_t));
            break;
        }
        case CHANNEL_1:
        {
            memcpy(&arg_channel, &channel_list.channel_1, sizeof(channel_t));
            break;
        }
        case CHANNEL_2:
        {
            memcpy(&arg_channel, &channel_list.channel_2, sizeof(channel_t));
            break;
        }
        case CHANNEL_3:
        {
            memcpy(&arg_channel, &channel_list.channel_3, sizeof(channel_t));
            break;
        }
        case CHANNEL_4:
        {
            memcpy(&arg_channel, &channel_list.channel_4, sizeof(channel_t));
            break;
        }
        case CHANNEL_5:
        {
            memcpy(&arg_channel, &channel_list.channel_5, sizeof(channel_t));
            break;
        }
        case CHANNEL_6:
        {
            memcpy(&arg_channel, &channel_list.channel_6, sizeof(channel_t));
            break;
        }
        case CHANNEL_7:
        {
            memcpy(&arg_channel, &channel_list.channel_7, sizeof(channel_t));
            break;
        }
        default:
        {
            break;
        }
    }
}

void spi::push_active_packet_to_return_buffer()
{
    switch(active_packet.channel_id)
    {
        case CHANNEL_0:
        {
            return_buffer_0.push(active_packet);
            break;
        }
        case CHANNEL_1:
        {
            return_buffer_1.push(active_packet);
            break;
        }
        case CHANNEL_2:
        {
            return_buffer_2.push(active_packet);
            break;
        }
        case CHANNEL_3:
        {
            return_buffer_3.push(active_packet);
            break;
        }
        case CHANNEL_4:
        {
            return_buffer_4.push(active_packet);
            break;
        }
        case CHANNEL_5:
        {
            return_buffer_5.push(active_packet);
            break;
        }
        case CHANNEL_6:
        {
            return_buffer_6.push(active_packet);
            break;
        }
        case CHANNEL_7:
        {
            return_buffer_7.push(active_packet);
            break;
        }
        default:
        {
            break;
        }
    }
}

spi::procedure_status_t spi::post_channel_rx_result(spi::packet_t arg_packet, int16_t arg_channel_id)
{
    switch(arg_channel_id)
    {
        case CHANNEL_0:
        {
            memset(&channel_0_rx_packet, '\0', sizeof(packet_t));
            memcpy(&channel_0_rx_packet, &arg_packet, sizeof(packet_t));
            break;
        }
        case CHANNEL_1:
        {
            memset(&channel_1_rx_packet, '\0', sizeof(packet_t));
            memcpy(&channel_1_rx_packet, &arg_packet, sizeof(packet_t));
            break;
        }
        case CHANNEL_2:
        {
            memset(&channel_2_rx_packet, '\0', sizeof(packet_t));
            memcpy(&channel_3_rx_packet, &arg_packet, sizeof(packet_t));
            break;
        }
        case CHANNEL_3:
        {
            memset(&channel_3_rx_packet, '\0', sizeof(packet_t));
            memcpy(&channel_3_rx_packet, &arg_packet, sizeof(packet_t));
            break;
        }
        case CHANNEL_4:
        {
            memset(&channel_4_rx_packet, '\0', sizeof(packet_t));
            memcpy(&channel_4_rx_packet, &arg_packet, sizeof(packet_t));
            break;
        }
        case CHANNEL_5:
        {
            memset(&channel_5_rx_packet, '\0', sizeof(packet_t));
            memcpy(&channel_5_rx_packet, &arg_packet, sizeof(packet_t));
            break;
        }
        case CHANNEL_6:
        {
            memset(&channel_6_rx_packet, '\0', sizeof(packet_t));
            memcpy(&channel_6_rx_packet, &arg_packet, sizeof(packet_t));
            break;
        }
        case CHANNEL_7:
        {
            memset(&channel_7_rx_packet, '\0', sizeof(packet_t));
            memcpy(&channel_7_rx_packet, &arg_packet, sizeof(packet_t));
            break;
        }
        default:
        {
            break;
        }
    }
    return PROCEDURE_STATUS_OK;
}

spi::procedure_status_t spi::wait_for_pending_flags_and_end_transaction(transaction_t arg_transaction_type)
{
    procedure_status_t status = PROCEDURE_STATUS_OK;
    uint32_t pending_flag = 0U;

    if ((module->config.mode == SPI_CONFIG_MODE_CONTROLLER) && ((module->config.direction == SPI_CONFIG_DIRECTION_1_LINE)
        || (module->config.direction == SPI_CONFIG_DIRECTION_2_LINE_RX_ONLY)))
    {
        disable_module();
    }

    if (module->config.mode == SPI_CONFIG_MODE_CONTROLLER)
    {
        if (arg_transaction_type == TX_RX || module->config.direction != SPI_CONFIG_DIRECTION_2_LINE_RX_ONLY)
        {
            pending_flag = SPI_SR_BIT_RESOURCE_BUSY;
        }
        else
        {
            pending_flag = SPI_SR_BIT_RX_BUFFER_NOT_EMPTY;
        }
    }
    else
    {
        pending_flag = SPI_SR_BIT_RX_BUFFER_NOT_EMPTY;
    }

    status = flag_timeout(pending_flag, BIT_CLEAR);

    return status;
}

spi::procedure_status_t spi::flag_timeout(uint32_t arg_status_reg_bit, bit_status_t arg_bit_status) const
{
    uint32_t start_time = get_timer_count(timeout_timer_handle);
    uint16_t fallback_countdown = FALLBACK_COUNTDOWN;

    while (get_status_register_bit(arg_status_reg_bit) != arg_bit_status)
    {
        if (get_timer_count(timeout_timer_handle) - start_time >= FLAG_TIMEOUT || fallback_countdown == 0)
        {
            disable_interrupts(SPI_CR2_BIT_TX_BUFFER_EMPTY_INTERRUPT_ENABLE | SPI_CR2_BIT_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_CR2_BIT_ERROR_INTERRUPT_ENABLE);

            if ((module->config.mode == SPI_CONFIG_MODE_CONTROLLER) && ((module->config.direction == SPI_CONFIG_DIRECTION_1_LINE) || (module->config.direction == SPI_CONFIG_DIRECTION_2_LINE_RX_ONLY)))
            {
                disable_module();
            }
            set_error_bit(SPI_ERROR_WAITING_FOR_FLAG);
            module->status = MODULE_STATUS_READY;
            return PROCEDURE_STATUS_TIMEOUT;
        }
        --fallback_countdown;
    }
    return PROCEDURE_STATUS_OK;
}
