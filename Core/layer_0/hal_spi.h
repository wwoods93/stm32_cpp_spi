/***********************************************************************************************************************
 * hal_spi.h
 *
 * wilson
 * 10/16/22
 * 9:41 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

#ifndef MAIN_CONTROLLER_HAL_SPI_H
#define MAIN_CONTROLLER_HAL_SPI_H

/* c/c++ includes */
#include <vector>
#include <queue>
#include <memory>
/* stm32 includes */

/* third-party includes */

/* layer_0 includes */
#include "hal_wrapper.h"
#include "hal_spi_definitions.h"
/* layer_1_rtosal includes */
#include "rtosal.h"
#include "rtosal_wrapper.h"
/* layer_1 includes */

/* layer_3_control includes */

/* layer_4_sys_op includes */

/* layer_n_meta_structure includes */


class spi
{
    public:

        static constexpr uint8_t    TX_SIZE_MAX                     = 8U;

        static constexpr int8_t     ISR_TX_8_BIT_2_LINE             = 0U;
        static constexpr int8_t     ISR_RX_8_BIT_2_LINE             = 1U;
        static constexpr int8_t     ISR_TX_16_BIT_2_LINE            = 2U;
        static constexpr int8_t     ISR_RX_16_BIT_2_LINE            = 3U;

        static constexpr uint8_t    TRANSACTION_TYPE_NONE           = 0U;
        static constexpr uint8_t    TRANSACTION_TYPE_SYNC           = 1U;
        static constexpr uint8_t    TRANSACTION_TYPE_ASYNC          = 2U;

        static constexpr uint8_t    SEND_STATE_BEGIN                = 0U;
        static constexpr uint8_t    SEND_STATE_IN_PROGRESS          = 1U;
        static constexpr uint8_t    SEND_STATE_COMPLETE             = 2U;

        static constexpr uint32_t   FLAG_TIMEOUT                    = 50U;
        static constexpr uint32_t   TRANSACTION_TIMEOUT             = 100U;
        static constexpr uint32_t   SEND_TIMEOUT                    = 1000U;
        static constexpr uint32_t   PROCESS_SEND_BUFFER_TIMEOUT     = 5000U;
        static constexpr uint16_t   FALLBACK_COUNTDOWN              = 1000U;
        static constexpr uint8_t    SPI_PROCEDURE_ERROR_NONE        = 0U;
        static constexpr uint8_t    SPI_PROCEDURE_STATE_BUS_ERROR   = 1U;
        static constexpr uint8_t    SPI_PROCEDURE_STATE_DATA_ERROR  = 2U;

        static constexpr uint8_t    CHANNEL_0                       = 0x00U;
        static constexpr uint8_t    CHANNEL_1                       = 0x01U;
        static constexpr uint8_t    CHANNEL_2                       = 0x02U;
        static constexpr uint8_t    CHANNEL_3                       = 0x03U;
        static constexpr uint8_t    CHANNEL_4                       = 0x04U;
        static constexpr uint8_t    CHANNEL_5                       = 0x05U;
        static constexpr uint8_t    CHANNEL_6                       = 0x06U;
        static constexpr uint8_t    CHANNEL_7                       = 0x07U;

        static constexpr uint32_t  SPI_ERROR_NONE                            = (0x00000000U);
        static constexpr uint32_t  SPI_ERROR_MODE_FAULT                      = (0x00000001U);
        static constexpr uint32_t  SPI_ERROR_CRC                             = (0x00000002U);
        static constexpr uint32_t  SPI_ERROR_OVERRUN                         = (0x00000004U);
        static constexpr uint32_t  SPI_ERROR_TI_MODE_FRAME_FORMAT            = (0x00000008U);
        static constexpr uint32_t  SPI_ERROR_DMA_TRANSFER                    = (0x00000010U);
        static constexpr uint32_t  SPI_ERROR_WAITING_FOR_FLAG                = (0x00000020U);
        static constexpr uint32_t  SPI_ERROR_DURING_ABORT                    = (0x00000040U);
        static constexpr uint32_t  SPI_ERROR_CONFIG_INVALID                  = (0x00000080U);
        static constexpr uint32_t  SPI_ERROR_CHANNEL_ID                      = (0x00000100U);
        static constexpr uint32_t  SPI_ERROR_CHIP_SELECT                     = (0x00000200U);
        static constexpr uint32_t  SPI_ERROR_INTER_TASK_QUEUE_TO_CLIENT      = (0x00000400U);
        static constexpr uint32_t  SPI_ERROR_INTER_TASK_QUEUE_FROM_CLIENT    = (0x00000800U);
        static constexpr uint32_t  SPI_ERROR_TRANSACTION_TIMEOUT             = (0x00001000U);
        static constexpr uint32_t  SPI_ERROR_SEND_BUFFER                     = (0x00002000U);
        static constexpr uint32_t  SPI_ERROR_RETURN_BUFFER                   = (0x00004000U);
        static constexpr uint32_t  SPI_ERROR_PAYLOAD                         = (0x00008000U);
        static constexpr uint32_t  SPI_ERROR_BUS_NOT_AVAILABLE               = (0x00010000U);
        static constexpr uint32_t  SPI_ERROR_CREATE_CHANNEL                  = (0x00020000U);

        static constexpr uint32_t  CONFIG_STATUS_BIT_OK                      = (uint32_t)(0x00000000U);
        static constexpr uint32_t  CONFIG_STATUS_BIT_INSTANCE                = (0x00000001U);
        static constexpr uint32_t  CONFIG_STATUS_BIT_MODULE                  = (0x00000002U);
        static constexpr uint32_t  CONFIG_STATUS_BIT_CLOCK_PHASE             = (0x00000004U);
        static constexpr uint32_t  CONFIG_STATUS_BIT_CLOCK_POLARITY          = (0x00000008U);
        static constexpr uint32_t  CONFIG_STATUS_BIT_MODE                    = (0x00000010U);
        static constexpr uint32_t  CONFIG_STATUS_BIT_BAUD_RATE               = (0x00000020U);
        static constexpr uint32_t  CONFIG_STATUS_BIT_FIRST_BIT_SETTING       = (0x00000040U);
        static constexpr uint32_t  CONFIG_STATUS_BIT_CHIP_SELECT             = (0x00000080U);
        static constexpr uint32_t  CONFIG_STATUS_BIT_DIRECTION               = (0x00000100U);
        static constexpr uint32_t  CONFIG_STATUS_BIT_FRAME_FORMAT            = (0x00000200U);
        static constexpr uint32_t  CONFIG_STATUS_BIT_CRC                     = (0x00000400U);
        static constexpr uint32_t  CONFIG_STATUS_BIT_TIMEOUT_TIMER           = (0x00000800U);
        static constexpr uint32_t  CONFIG_STATUS_BIT_DATA_SIZE               = (0x00001000U);
        static constexpr uint32_t  CONFIG_STATUS_BIT_MSP_INIT                = (0x00002000U);
        static constexpr uint32_t  CONFIG_STATUS_BIT_ISR_INIT                = (0x00004000U);
        static constexpr uint32_t  CONFIG_STATUS_BIT_RX_PTR_INIT             = (0x00008000U);


        typedef enum
        {
            MODULE_UNLOCKED                     = 0x00U,
            MODULE_LOCKED                       = 0x01U
        } lock_t;

        typedef enum
        {
            TX_RX                               = 0x00,
            TX_ONLY                             = 0x01,
            RX_ONLY                             = 0x02,
        }transaction_t;

        typedef enum
        {
            DATA_REG_ID                         = 0x00U,
            STATUS_REG_ID                       = 0x01U,
            CONTROL_REG_1_ID                    = 0x02U,
            CONTROL_REG_2_ID                    = 0x03U,
        } register_id_t;

        typedef enum
        {
            PROCEDURE_STATUS_OK                 = 0x00U,
            PROCEDURE_STATUS_ERROR              = 0x01U,
            PROCEDURE_STATUS_BUSY               = 0x02U,
            PROCEDURE_STATUS_TIMEOUT            = 0x03U
        } procedure_status_t;

        typedef enum
        {
            MODULE_STATUS_RESET                 = 0x00U,
            MODULE_STATUS_READY                 = 0x01U,
            MODULE_STATUS_BUSY                  = 0x02U,
            MODULE_STATUS_BUSY_TX               = 0x03U,
            MODULE_STATUS_BUSY_RX               = 0x04U,
            MODULE_STATUS_BUSY_TX_RX            = 0x05U,
            MODULE_STATUS_ERROR                 = 0x06U,
            MODULE_STATUS_ABORT                 = 0x07U
        } module_status_t;

        typedef struct
        {
            hal::gpio_t*    port;
            uint16_t        pin;
        } chip_select_t;

        typedef struct
        {
            int16_t         channel_id;
            int16_t         packet_id;
            uint8_t         bytes_per_transaction[TX_SIZE_MAX];
            uint8_t         tx_bytes[TX_SIZE_MAX];
            uint8_t         rx_bytes[TX_SIZE_MAX];
            chip_select_t   chip_select;
        } packet_t;

        typedef struct
        {
            int16_t                         channel_id;
            chip_select_t                   chip_select;
            uint8_t                         is_inter_task;
            rtosal::message_queue_handle_t  tx_message_queue;
            rtosal::message_queue_handle_t  rx_message_queue;

        } channel_t;

        typedef struct
        {
            uint32_t mode;
            uint32_t direction;
            uint32_t data_size;
            uint32_t clock_polarity;
            uint32_t clock_phase;
            uint32_t chip_select_setting;
            uint32_t baud_rate_prescaler;
            uint32_t first_bit_setting;
            uint32_t ti_mode;
            uint32_t crc_calculation;
            uint32_t crc_polynomial;
        } config_t;

        typedef struct module_struct
        {
            hal_spi_t                   *register_map;
            config_t                    config;
            volatile module_status_t    status;
            volatile uint32_t           error_bit_field;
            volatile uint32_t           config_status_bit_field;
            chip_select_t               chip_select;
            uint8_t                     *tx_buffer_ptr;
            uint8_t                     *rx_buffer_ptr;
            volatile uint16_t           tx_transfer_counter;
            volatile uint16_t           rx_transfer_counter;
            uint8_t                     rx_data_ready_flag;
            lock_t                      lock;
            hal::gpio_t*                chip_select_port;
            uint16_t                    chip_select_pin;
        } module_t;

        module_t*                   module;
        packet_t                    active_packet;
        int16_t                     next_available_packet_id = ID_INVALID;
        uint32_t                    packets_requested_count = 0U;
        uint32_t                    packets_received_count = 0U;
        uint8_t                     process_send_buffer_state = SEND_STATE_BEGIN;
        uint8_t                     packet_index = 0U;
        uint8_t                     transaction_byte_count = 0U;
        uint8_t                     transaction_index = 0U;
        int8_t                      tx_isr_id = ID_INVALID;
        int8_t                      rx_isr_id = ID_INVALID;
        uint8_t*                    rx_pointer;
        hal::timer_handle_t*        timeout_timer_handle;
        uint32_t                    send_timeout_start;
        uint32_t                    process_send_buffer_timeout_start;
        int16_t                     next_available_channel_id = 0U;
        uint8_t                     channel_array[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
        uint8_t                     active_transaction_type = TRANSACTION_TYPE_NONE;

        std::queue<packet_t>        send_buffer;
        std::queue<packet_t>        pending_buffer;
        std::queue<packet_t>        return_buffer_0;
        std::queue<packet_t>        return_buffer_1;
        std::queue<packet_t>        return_buffer_2;
        std::queue<packet_t>        return_buffer_3;
        std::queue<packet_t>        return_buffer_4;
        std::queue<packet_t>        return_buffer_5;
        std::queue<packet_t>        return_buffer_6;
        std::queue<packet_t>        return_buffer_7;

        packet_t channel_0_rx_packet;
        packet_t channel_1_rx_packet;
        packet_t channel_2_rx_packet;
        packet_t channel_3_rx_packet;
        packet_t channel_4_rx_packet;
        packet_t channel_5_rx_packet;
        packet_t channel_6_rx_packet;
        packet_t channel_7_rx_packet;

        struct
        {
            channel_t channel_0;
            channel_t channel_1;
            channel_t channel_2;
            channel_t channel_3;
            channel_t channel_4;
            channel_t channel_5;
            channel_t channel_6;
            channel_t channel_7;
        } channel_list;









        procedure_status_t initialize(module_t* arg_module, uint8_t arg_instance_t, TIM_HandleTypeDef* arg_timeout_timer_handle);
        procedure_status_t create_channel(int16_t& arg_channel_id, hal::gpio_t* arg_chip_select_port, uint16_t arg_chip_select_pin, uint8_t arg_is_inter_task, rtosal::message_queue_handle_t arg_tx_message_queue, rtosal::message_queue_handle_t arg_rx_message_queue);
        procedure_status_t send_receive(uint8_t* arg_tx_bytes, uint8_t* arg_rx_bytes, uint8_t* arg_bytes_per_tx, int16_t arg_channel_id);
        procedure_status_t send_receive_byte(uint8_t& arg_tx_byte, uint8_t& arg_rx_byte, int16_t arg_channel_id);
        procedure_status_t receive(uint8_t* arg_rx_bytes, int16_t arg_channel_id);
        int16_t send_async(uint8_t* arg_tx_bytes, uint8_t* arg_bytes_per_tx, int16_t arg_channel_id);
        procedure_status_t receive_async(uint8_t* arg_rx_bytes, int16_t arg_channel_id);
        procedure_status_t receive_inter_task_transaction_requests();
        procedure_status_t process_send_buffer();
        procedure_status_t process_return_buffers(packet_t& arg_packet);
        [[nodiscard]] uint32_t get_packets_requested_count() const;
        [[nodiscard]] uint32_t get_packets_received_count() const;
        friend void tx_isr(spi arg_object, struct spi::module_struct *arg_module);
        friend void rx_isr(spi arg_object, struct spi::module_struct *arg_module);
        friend void spi_irq_handler(spi* arg_object);

    private:

        procedure_status_t spi_transmit_receive_interrupt(uint8_t *arg_tx_data_ptr, uint8_t *arg_rx_data_ptr, uint16_t arg_packet_size);
        procedure_status_t send_inter_task_transaction_result(rtosal::message_queue_handle_t arg_message_queue_id, packet_t& arg_packet);
        void close_isr(transaction_t arg_transaction_type);
        [[nodiscard]] procedure_status_t verify_communication_direction(uint32_t arg_intended_direction) const;
        int16_t assign_next_available_channel_id();
        void get_channel_by_channel_id(channel_t& arg_channel, int16_t arg_channel_id);
        void push_active_packet_to_return_buffer();
        procedure_status_t post_channel_rx_result(packet_t arg_packet, int16_t arg_channel_id);
        procedure_status_t wait_for_pending_flags_and_end_transaction(transaction_t arg_transaction_type);
        procedure_status_t flag_timeout(uint32_t arg_status_reg_bit, bit_status_t arg_bit_status) const;

        void enable_module() const;
        void disable_module() const;
        [[nodiscard]] procedure_status_t lock_module() const;
        procedure_status_t unlock_module() const;
        void enable_interrupts(uint32_t arg_interrupts) const;
        void disable_interrupts(uint32_t arg_interrupts) const;
        [[nodiscard]] bit_status_t check_interrupt_source(uint32_t arg_interrupt) const;
        void handle_tx_rx_success();
        void handle_tx_success();
        void handle_rx_success();
        void handle_transaction_error() const;
        void set_register_bit(register_id_t arg_register, uint32_t arg_bit) const;
        void clear_register_bit(register_id_t arg_register, uint32_t arg_bit) const;
        [[nodiscard]] bit_status_t get_status_register_bit(uint32_t arg_bit) const;
        void set_error_bit(uint32_t arg_bit) const;
        void set_config_status_bit(uint32_t arg_bit) const;
        void clear_config_status_bit_field();
        void clear_mode_fault_flag() const;
        void clear_overrun_flag() const;
};

inline void spi::enable_module() const
{
    module->register_map->CONTROL_REG_1 |= SPI_CR1_BIT_SPI_ENABLE;
}

inline void spi::disable_module() const
{
    clear_register_bit(CONTROL_REG_1_ID, SPI_CR1_BIT_SPI_ENABLE);
}

inline spi::procedure_status_t spi::lock_module() const
{
    if (module->lock == MODULE_LOCKED) { return PROCEDURE_STATUS_BUSY; }
    module->lock = MODULE_LOCKED;
    return PROCEDURE_STATUS_OK;
}

inline spi::procedure_status_t spi::unlock_module() const
{
    module->lock = MODULE_UNLOCKED;
    return PROCEDURE_STATUS_OK;
}

inline void spi::enable_interrupts(uint32_t arg_interrupts) const
{
    module->register_map->CONTROL_REG_2 |= arg_interrupts;
}

inline void spi::disable_interrupts(uint32_t arg_interrupts) const
{
    module->register_map->CONTROL_REG_2 &= (~arg_interrupts);
}

inline bit_status_t spi::check_interrupt_source(uint32_t arg_interrupt) const
{
    bit_status_t bit_status = BIT_CLEAR;
    if ((module->register_map->CONTROL_REG_2 & arg_interrupt) == arg_interrupt)
    {
        bit_status = BIT_SET;
    }

    return bit_status;
}

inline void spi::handle_tx_rx_success()
{
    for (transaction_index = 0U; transaction_index <  transaction_byte_count; ++transaction_index)
    {
        active_packet.rx_bytes[packet_index++] = rx_pointer[transaction_index];
    }

    hal::gpio_write_pin(module->chip_select_port, module->chip_select_pin, GPIO_PIN_SET);
    module->rx_data_ready_flag = 1U;
}

inline void spi::handle_tx_success()
{
    hal::gpio_write_pin(module->chip_select_port, module->chip_select_pin, GPIO_PIN_SET);
    module->rx_data_ready_flag = 1U;
}

inline void spi::handle_rx_success()
{
    hal::gpio_write_pin(module->chip_select_port, module->chip_select_pin, GPIO_PIN_SET);
    module->rx_data_ready_flag = 1U;
}

inline void spi::handle_transaction_error() const
{
    hal::gpio_write_pin(module->chip_select_port, module->chip_select_pin, GPIO_PIN_SET);
    module->rx_data_ready_flag = 1U;
}

inline void spi::set_register_bit(register_id_t arg_register, uint32_t arg_bit) const
{
    switch(arg_register)
    {
        case CONTROL_REG_1_ID:
        {
            module->register_map->CONTROL_REG_1 |= arg_bit;
            break;
        }
        case CONTROL_REG_2_ID:
        {
            module->register_map->CONTROL_REG_2 |= arg_bit;
            break;
        }
        default:
        {
            break;
        }
    }
}

inline void spi::clear_register_bit(register_id_t arg_register, uint32_t arg_bit) const
{
    switch(arg_register)
    {
        case STATUS_REG_ID:
        {
            module->register_map->STATUS_REG &= (~arg_bit);
            break;
        }
        case CONTROL_REG_1_ID:
        {
            module->register_map->CONTROL_REG_1 &= (~arg_bit);
            break;
        }
        case CONTROL_REG_2_ID:
        {
            module->register_map->CONTROL_REG_2 &= (~arg_bit);
            break;
        }
        default:
        {
            break;
        }
    }
}

inline bit_status_t spi::get_status_register_bit(uint32_t arg_bit) const
{
    bit_status_t bit_status = BIT_CLEAR;
    if ((module->register_map->STATUS_REG & arg_bit & SPI_SR_BITS_MASK) == (arg_bit & SPI_SR_BITS_MASK))
    {
        bit_status = BIT_SET;
    }
    return bit_status;
}

inline void spi::set_error_bit(uint32_t arg_bit) const
{
    module->error_bit_field |= arg_bit;
}

inline void spi::set_config_status_bit(uint32_t arg_bit) const
{
    module->config_status_bit_field |= arg_bit;
}

inline void spi::clear_config_status_bit_field()
{
    module->config_status_bit_field = CONFIG_STATUS_BIT_OK;
}

inline void spi::clear_mode_fault_flag() const
{
    uint32_t register_contents = module->register_map->STATUS_REG;
    UNUSED_CAST_VOID(register_contents);
    clear_register_bit(CONTROL_REG_1_ID, SPI_CR1_BIT_SPI_ENABLE);
}

inline void spi::clear_overrun_flag() const
{
    uint32_t data_reg_contents = module->register_map->DATA_REG;
    uint32_t status_reg_contents = module->register_map->STATUS_REG;
    UNUSED_CAST_VOID(data_reg_contents);
    UNUSED_CAST_VOID(status_reg_contents);
}

#endif //MAIN_CONTROLLER_HAL_SPI_H
