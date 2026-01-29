#include <string.h>
#include "esp32wake.hpp"
#include "wake_protocol.h"


ESP32Wake::ESP32Wake(uart_port_t uart_prt) : uart{uart_prt} {}

void ESP32Wake::set_uart_num(uart_port_t uart_prt) {
    if (uart_prt < UART_NUM_MAX) {
        uart = uart_prt;
    }
}

void ESP32Wake::begin(gpio_num_t tx_pin, gpio_num_t rx_pin) {
    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 100,
        .source_clk = UART_SCLK_DEFAULT,
    };
    const int uart_buffer_size = WAKE_MAX_PACKAGE_LEN;

    ESP_ERROR_CHECK(uart_driver_install(uart, uart_buffer_size, \
                                        uart_buffer_size, 20, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(uart, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

wake_status_t ESP32Wake::send_package(wake_package_info_t *p_pckg) {
    static uint8_t buffer[WAKE_MAX_PACKAGE_LEN] = {};
    
    uint16_t total;
    wake_status_t ret;

    p_pckg->crc = wake_calculate_package_crc(p_pckg, ignore_address_flg);
    ret = wake_package_to_bytes(p_pckg, ignore_address_flg, buffer, &total);
    if (ret != WAKE_OK) {
        memset(buffer, 0, total < WAKE_MAX_PACKAGE_LEN ? total : WAKE_MAX_PACKAGE_LEN);
        return ret;
    }
    uart_write_bytes(uart, buffer, total);

    memset(buffer, 0, total < WAKE_MAX_PACKAGE_LEN ? total : WAKE_MAX_PACKAGE_LEN);
    return WAKE_OK;
}

len_t ESP32Wake::catch_input_package(wake_package_info_t *p_pckg) {
    uint8_t b;
    len_t bytes_count{0};
    volatile len_t max_bytes_count; 
    volatile bool unst_flag{false};

    uart_read_bytes(uart, &p_pckg->start_byte, 1, pdMS_TO_TICKS(1));
    if (p_pckg->start_byte != FEND) return static_cast<len_t>(ESP32WAKE_NOT_START_BYTE);

    max_bytes_count = (ignore_address_flg) ? 259 : 260;
    ++bytes_count;
    while (bytes_count < max_bytes_count) {
        if (uart_read_bytes(uart, &b, 1, pdMS_TO_TICKS(1)) > 0) {
            unst_flag = wake_unstuffing(&b, unst_flag);
            if (!unst_flag) {
                if (ignore_address_flg) {
                    if (bytes_count == 1) {
                        p_pckg->cmd = b;
                    } else if (bytes_count == 2) {
                        p_pckg->n = b;
                        max_bytes_count = 4 + b;                        
                    } else if (bytes_count < (max_bytes_count - 1)) {
                        p_pckg->data[bytes_count - 3] = b;
                    } else {
                        p_pckg->crc = b;
                    }
                } else {
                    if (bytes_count == 1) {
                        p_pckg->addr = b & 0x7F;
                    } else if (bytes_count == 2) {
                        p_pckg->cmd = b;
                    } else if (bytes_count == 3) {
                        p_pckg->n = b;
                        max_bytes_count = 5 + b;
                    } else if (bytes_count < (max_bytes_count - 1)) {
                        p_pckg->data[bytes_count - 4] = b;
                    } else {
                        p_pckg->crc = b;
                    }
                }
                bytes_count++;
            }
        } else {
            return ESP32WAKE_READ_FAILED;
        }
    }

    if (!wake_check_crc(p_pckg, ignore_address_flg)) return static_cast<len_t>(ESP32WAKE_CRC_ERROR);
    if (ignore_address_flg) p_pckg->addr = 0;
    return bytes_count;
}

bool ESP32Wake::parse_byte(wake_package_info_t *p_pckg, uint8_t &byte, esp32wake_result_t &current_status) {
    if (!start_byte_is_received && (byte == FEND)) {
        start_byte_is_received = true;
        bytes_count = 1;
        unst_flag = false;
        max_bytes_count = (ignore_address_flg) ? 259 : 260;
        unst_flag = false;
        // because only start byte is processed, exit to check next packet bytes in future
        current_status = ESP32WAKE_OK;
        return false;
    } else if (!start_byte_is_received && (byte != FEND)) {
        current_status = ESP32WAKE_NOT_START_BYTE;
        return false;
    }
    // here start byte already received, process next bytes
    unst_flag = wake_unstuffing(&byte, unst_flag);
    if (!unst_flag) {
        if (ignore_address_flg) {
            if (bytes_count == 1) {
                p_pckg->cmd = byte;
            } else if (bytes_count == 2) {
                p_pckg->n = byte;
                max_bytes_count = 4 + byte;               
            } else if (bytes_count < (max_bytes_count - 1)) {
                p_pckg->data[bytes_count - 3] = byte;
            } else {
                p_pckg->crc = byte;
            }
        } else {
            if (bytes_count == 1) {
                p_pckg->addr = byte & 0x7F;
            } else if (bytes_count == 2) {
                p_pckg->cmd = byte;
            } else if (bytes_count == 3) {
                p_pckg->n = byte;
                max_bytes_count = 5 + byte;
            } else if (bytes_count < (max_bytes_count - 1)) {
                p_pckg->data[bytes_count - 4] = byte;
            } else {
                p_pckg->crc = byte;
            }
        }
        bytes_count++;
    }
    if (bytes_count >= max_bytes_count) {
        // packet processing is finished
        start_byte_is_received = false;
        if (!wake_check_crc(p_pckg, ignore_address_flg)) {
            // there is data corruption, because CRC8 check is failed
            current_status = ESP32WAKE_CRC_ERROR;
            return false;
        }
        if (ignore_address_flg) p_pckg->addr = 0;
        // packet is ready
        current_status = ESP32WAKE_OK;
        return true;
    }
    // packet isn't ready
    return false;
}

void ESP32Wake::set_ignore_address_flag(bool flag) {ignore_address_flg = flag;}

bool ESP32Wake::get_ignore_address_flag(void) {return ignore_address_flg;}
