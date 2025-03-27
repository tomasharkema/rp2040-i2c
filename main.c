/**
 * Copyright (c) 2022 Nicolai Electronics
 *
 * SPDX-License-Identifier: MIT
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bsp/board.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/structs/watchdog.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"
#include "kernel_i2c_flags.h"
#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include "tusb.h"
#include "usb_descriptors.h"

#define I2C_INST i2c0
#define I2C_SDA  4
#define I2C_SCL  5

#define A0PIN 0

// al3320a
#define I2C_DEVICE_ADDRESS 0x1c

#define UART_ID   uart0
#define BAUD_RATE 115200

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define UART_TX_PIN 0
#define UART_RX_PIN 1

uint8_t debug_buffer[256] = {0};

void debug_print(const char* buffer) {
#ifdef DEBUG
    uart_puts(UART_ID, buffer);
#endif
}

void debug_println(const char* buffer) {
    debug_print(buffer);
    debug_print("\r\n");
}

int main(void) {
    board_init();
    tusb_init();

#ifdef DEBUG
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_TX_PIN));
    gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_RX_PIN));
#endif

    debug_println("\033[2J");

    gpio_init(I2C_SDA);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);

    gpio_init(I2C_SCL);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SCL);

    i2c_init(I2C_INST, 100000);

    adc_init();

    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(26);

    // Select ADC input 0 (GPIO26)
    adc_select_input(A0PIN);

    while (1) {
        tud_task();
    }

    return 0;
}

// Invoked when device is mounted
void tud_mount_cb(void) {}

// Invoked when device is unmounted
void tud_umount_cb(void) {}

// Invoked when usb bus is suspended
void tud_suspend_cb(bool remote_wakeup_en) {}

// Invoked when usb bus is resumed
void tud_resume_cb(void) {}

/* commands via USB, must match command ids in the firmware */
#define CMD_ECHO         0
#define CMD_GET_FUNC     1
#define CMD_SET_DELAY    2
#define CMD_GET_STATUS   3
#define CMD_I2C_IO       4
#define CMD_I2C_IO_BEGIN (1 << 0)
#define CMD_I2C_IO_END   (1 << 1)

const unsigned long i2c_func = I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;

#define STATUS_IDLE        0
#define STATUS_ADDRESS_ACK 1
#define STATUS_ADDRESS_NAK 2

static uint8_t i2c_state = STATUS_IDLE;

uint8_t i2c_data[1024] = {0};
uint8_t adc_data[1024] = {0};

uint8_t reply_buf[64] = {0};

bool handle_i2c(uint8_t rhport, uint8_t stage, tusb_control_request_t const* request) {
    if (stage != CONTROL_STAGE_SETUP && stage != CONTROL_STAGE_DATA) return true;
    bool nostop = !(request->bRequest & CMD_I2C_IO_END);

    debug_println("\n\n\n===>");

    sprintf(debug_buffer, "%s i2c %s at 0x%02x, value = 0x%02x, len = %d, nostop = %d", (stage != CONTROL_STAGE_SETUP) ? "[D]" : "[S]",
            (request->wValue & I2C_M_RD) ? "rd" : "wr", request->wIndex, request->wValue, request->wLength, nostop);
    debug_println(debug_buffer);

    if (request->wLength > sizeof(i2c_data)) {
        return false;  // Prevent buffer overflow in case host sends us an impossible request
    }

    sprintf(debug_buffer, "stage 0x%02x, bRequest = 0x%02x, wValue = 0x%02x, wIndex = 0x%02x, wLength = 0x%02x", stage, request->bRequest, request->wValue,
            request->wIndex, request->wLength);
    debug_println(debug_buffer);

    sprintf(debug_buffer, "i2c_data %x %x %x %x", i2c_data[0], i2c_data[1], i2c_data[2], i2c_data[3]);
    debug_println(debug_buffer);

    sprintf(debug_buffer, "adc_data %x %x %x %x", adc_data[0], adc_data[1], adc_data[2], adc_data[3]);
    debug_println(debug_buffer);

    sprintf(debug_buffer, "adc_data %x %x %x %x", reply_buf[0], reply_buf[1], reply_buf[2], reply_buf[3]);
    debug_println(debug_buffer);

    if (stage == CONTROL_STAGE_SETUP) {  // Before transfering data

        if (request->wValue & I2C_M_RD) {
            if (request->wIndex == I2C_DEVICE_ADDRESS) {
                adc_data[0x07] = 0x04;

                adc_select_input(0);

                // const float conversion_factor = 3.3f / (1 << 12);

                uint16_t value = adc_read();  // * conversion_factor;

                uint8_t high = *((uint8_t*) &(value) + 1);  // high byte (0x12)
                uint8_t low  = *((uint8_t*) &(value) + 0);  // low byte  (0x34)

                adc_data[0x22] = low;
                adc_data[0x23] = high;

                sprintf(debug_buffer, "READ ADC 0x%04x high 0x%02x low 0x%02x", value, high, low);
                debug_println(debug_buffer);

                // adc_select_input(1);
                // adc_data[0] = adc_read();

                // adc_select_input(2);
                // adc_data[0] = adc_read();

                // adc_select_input(3);
                // adc_data[0] = adc_read();

                i2c_state = STATUS_ADDRESS_ACK;

            } else {
                sprintf(debug_buffer, "READ I2C %i", reply_buf[0]);
                debug_println(debug_buffer);

                // Reading from I2C device
                int res = i2c_read_blocking(I2C_INST, request->wIndex, (void*) reply_buf, request->wLength, nostop);
                if (res == PICO_ERROR_GENERIC) {
                    i2c_state = STATUS_ADDRESS_NAK;
                } else {
                    i2c_state = STATUS_ADDRESS_ACK;
                }
            }

        } else if (request->wLength == 0) {  // Writing with length of 0, this is used for bus scanning, do dummy read

            sprintf(debug_buffer, "SCAN INDEX = 0x%02x", request->wIndex);
            debug_println(debug_buffer);

            if (request->wIndex == I2C_DEVICE_ADDRESS) {
                i2c_state = STATUS_ADDRESS_ACK;
            } else {
                uint8_t dummy = 0x00;
                int     res   = i2c_read_blocking(I2C_INST, request->wIndex, (void*) &dummy, 1, nostop);
                if (res == PICO_ERROR_GENERIC) {
                    debug_println("NAK");
                    i2c_state = STATUS_ADDRESS_NAK;
                } else {
                    debug_println("ACK");
                    i2c_state = STATUS_ADDRESS_ACK;
                }
            }
        }

        if (request->wIndex == I2C_DEVICE_ADDRESS) {
            int startAdress = reply_buf[0];

            for (uint8_t i = 0; i < request->wLength; i++) {
                reply_buf[i] = adc_data[startAdress + i];
            }

            tud_control_xfer(rhport, request, (void*) reply_buf, request->wLength);

        } else {
            tud_control_xfer(rhport, request, (void*) reply_buf, request->wLength);
        }
    }

    if (stage == CONTROL_STAGE_DATA) {        // After transfering data
        if (!(request->wValue & I2C_M_RD)) {  // I2C write operation

            if (request->wIndex == I2C_DEVICE_ADDRESS) {
                // adc_data[request->wIndex] = 0; // TODO: fix!
                i2c_state = STATUS_ADDRESS_ACK;

            } else {
                int res = i2c_write_blocking(I2C_INST, request->wIndex, i2c_data, request->wLength, nostop);
                if (res == PICO_ERROR_GENERIC) {
                    i2c_state = STATUS_ADDRESS_NAK;
                } else {
                    i2c_state = STATUS_ADDRESS_ACK;
                }
            }
        }
    }

    return true;
}

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const* request) {
    if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR) {
        switch (request->bRequest) {
            case CMD_ECHO:
                if (stage != CONTROL_STAGE_SETUP) return true;
                return tud_control_xfer(rhport, request, (void*) &request->wValue, sizeof(request->wValue));
            case CMD_GET_FUNC:
                if (stage != CONTROL_STAGE_SETUP) return true;
                return tud_control_xfer(rhport, request, (void*) &i2c_func, sizeof(i2c_func));
                break;
            case CMD_SET_DELAY:
                if (stage != CONTROL_STAGE_SETUP) return true;
                if (request->wValue == 0) {
                    i2c_set_baudrate(I2C_INST, 100000);  // Use default: 100kHz
                } else {
                    int baudrate = 1000000 / request->wValue;
                    if (baudrate > 400000) baudrate = 400000;  // Limit to 400kHz
                    i2c_set_baudrate(I2C_INST, baudrate);
                }
                return tud_control_status(rhport, request);
            case CMD_GET_STATUS:
                if (stage != CONTROL_STAGE_SETUP) return true;
                debug_println("GET STATUS!");
                return tud_control_xfer(rhport, request, (void*) &i2c_state, sizeof(i2c_state));
            case CMD_I2C_IO:
            case CMD_I2C_IO + CMD_I2C_IO_BEGIN:
            case CMD_I2C_IO + CMD_I2C_IO_END:
            case CMD_I2C_IO + CMD_I2C_IO_BEGIN + CMD_I2C_IO_END:
                return handle_i2c(rhport, stage, request);

            default:
                if (stage != CONTROL_STAGE_SETUP) return true;
                break;
        }
    } else {
        if (stage != CONTROL_STAGE_SETUP) return true;
    }

    return false;  // stall unknown request
}

bool tud_vendor_control_complete_cb(uint8_t rhport, tusb_control_request_t const* request) {
    (void) rhport;
    (void) request;
    return true;
}
