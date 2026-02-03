#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"
extern "C" {
#include "wake_protocol.h"
}


constexpr uint16_t UART_WAIT_TIME = 100;
typedef int16_t len_t;

typedef enum {
    ESP32WAKE_NOT_START_BYTE = -3,
    ESP32WAKE_CRC_ERROR = -2,
    ESP32WAKE_READ_FAILED = -1,
    ESP32WAKE_OK,
} esp32wake_result_t;


class ESP32Wake {
    public:
        /**
         * @brief Default constructor 
         * 
         * @param uart_prt Hardware UART port number
         */
        ESP32Wake(uart_port_t uart_prt=UART_NUM_0);

        /**
         * @brief Set UART interface number
         * @param uart_prt  UART port number
         * @retval          None
         */
        void set_uart_num(uart_port_t uart_prt);

        /**
         * @brief Starts the serial connection with fixed baudrate
         * 
         * Method can be used without arguments with standart UART0 pins
         * 
         * @param tx_pin GPIO pin number for data transmitting
         * @param rx_pin GPIO pin number for data receiving
         * 
         */
        void begin(gpio_num_t tx_pin=GPIO_NUM_1, gpio_num_t rx_pin=GPIO_NUM_3);

        /**
         * @brief Send Wake package through Serial (UART)
         * 
         * There are checking of the address, command, and data_buf_len values.
         * If there is an error with input, corresponding error will be returned (only one)
         * Method forms the array of the bytes and sends it via UART
         * 
         * @param p_pckg Address of the wake_package_info_t struct
         * 
         * @return Status, defined in wake_base_protocol.h
         */
        wake_status_t send_package(wake_package_info_t *p_pckg);

        /**
         * @brief Get the input package.
         * 
         * Method checks the input UART buffer and fills the package structure
         * with help p_pckg pointer
         * @param p_pckg Pointer to the package structure (out)
         * @return Received bytes count, or -1 in error case, -2 if crc check is failed
         */
         [[deprecated("May lose package")]]
        len_t catch_input_package(wake_package_info_t *p_pckg);

        /**
         * @brief Parse the input byte.
         * 
         * Method checks the input byte and fills the package structure
         * with help p_pckg pointer
         * @param p_pckg[out]           Pointer to the package structure (out)
         * @param byte                  Byte to process
         * @param current_status[out]    Status to check current packet processing state
         * @return Processing status: true - packet is ready, false - packet in process or there is error
         */
        bool parse_byte(wake_package_info_t *p_pckg, uint8_t &byte, esp32wake_result_t &current_status);

        /**
         * @brief Setting the flag for activating or disabling address field ignoring
         * @param flag true means ignoring address, false means not ignoring address
         */
        void set_ignore_address_flag(bool flag);

        /**
         * @brief Getting the address ignoring flag value
         */
        bool get_ignore_address_flag(void);

        /// @brief Getting uart
        uart_port_t get_uart_port(void) const;

    protected:
        uart_port_t uart;
        bool ignore_address_flg{false};
        bool start_byte_is_received{false};
        len_t bytes_count{0};
        volatile len_t max_bytes_count; 
        volatile bool unst_flag{false};
};
