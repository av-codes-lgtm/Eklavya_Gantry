#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <stdio.h>
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

#define STEP_PIN GPIO_NUM_18
#define DIR_PIN  GPIO_NUM_19



#define STEP_HIGH_TIME_uS 5
#define STEP_TOTAL_TIME_uS 30

#define UART_PORT UART_NUM_2
#define TMC_TX 17
#define TMC_RX 16
#define REG_IHOLD_IRUN   0x10
#define REG_CHOPCONF     0x6C
#define BUF_SIZE 128


static const char *TAG = "TMC2208";
static uart_port_t g_uart_num;

// --- recommended uart init ---
void tmc_uart_init(uart_port_t uart_num, int tx_pin, int rx_pin) {
    g_uart_num = uart_num;
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    // 1) param config
    uart_param_config(uart_num, &uart_config);

    // 2) set pins (for half-duplex, tx_pin and rx_pin can be same or wired together)
    uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // 3) install driver: RX buffer (TX buffer not required)
    // Make RX buffer reasonably large; 256-1024 is typical for command/response
    uart_driver_install(uart_num, BUF_SIZE * 4, 0, 0, NULL, 0);

    // 4) enable RS485 half-duplex mode for single-wire TMC
    uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX);
}

void setup_stepper_pins() {
    // Configure stepper driver output pins
    gpio_config_t io_conf_out = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << STEP_PIN) | (1ULL << DIR_PIN),  
                    
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_conf_out);


}

uint8_t tmc_crc(uint8_t *data, int length) {
    uint8_t crc = 0;
    for (int i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    return crc;
}

void tmc_write_reg(uint8_t reg, uint32_t value) {
    uint8_t packet[8];
    packet[0] = 0x05;
    packet[1] = 0x00;
    packet[2] = reg | 0x80; 
    packet[3] = (value >> 24) & 0xFF;
    packet[4] = (value >> 16) & 0xFF;
    packet[5] = (value >> 8) & 0xFF;
    packet[6] = value & 0xFF;
    packet[7] = tmc_crc(packet, 7);

    uart_write_bytes(g_uart_num, (const char*)packet, 8);
}

uint32_t tmc_read_reg(uint8_t reg) {
    uint8_t packet[4];
    packet[0] = 0x05;
    packet[1] = 0x00;
    packet[2] = reg & 0x7F;
    packet[3] = tmc_crc(packet, 3);

    uart_write_bytes(g_uart_num, (const char*)packet, 4);

    uint8_t resp[8];
    int len = uart_read_bytes(g_uart_num, resp, 8, 100 / portTICK_PERIOD_MS);
    if (len == 8 && resp[7] == tmc_crc(resp, 7)) {
        return (resp[3] << 24) | (resp[4] << 16) | (resp[5] << 8) | resp[6];
    }
    //ESP_LOGW(TAG, "Bad read response");
    return 0;
}

// Set motor to 256 microstepping
void tmc_set_microstepping() {
    uint32_t chopconf = tmc_read_reg(REG_CHOPCONF);

    // Clear MRES bits [24..27]
    chopconf &= ~(0xF << 24);
    // Set to 256 microsteps
    chopconf |= (0x0 << 24);

    tmc_write_reg(REG_CHOPCONF, chopconf);
}

// --- fix current packing: iholddelay is 4 bits ---
void tmc_set_current(uint8_t ihold, uint8_t irun, uint8_t iholddelay) {
    uint32_t val =
        ((uint32_t)(ihold      & 0x1F)      ) |
        ((uint32_t)(irun       & 0x1F) <<  8) |
        ((uint32_t)(iholddelay & 0x0F) << 16);  // 0x0F here
    tmc_write_reg(REG_IHOLD_IRUN, val);
}

void step(int no_steps) {
    int accel_steps = 200*256;
    int decel_steps = accel_steps;
    if (no_steps<accel_steps){
        accel_steps=no_steps/2;
        decel_steps=accel_steps;
    }
    int const_steps = no_steps - accel_steps - decel_steps;
    if (const_steps<0){
        const_steps=0;
    }
    // Define min and max total step time (in microseconds)
    int min_step_time = 100;  // Fastest (small delay between steps)
    int max_step_time = 150; // Slowest (long delay between steps)

    // Acceleration Phase
    for (int i = 0; i < accel_steps; i++) {
        int step_time = max_step_time - ((max_step_time - min_step_time) * i / accel_steps);
        gpio_set_level(STEP_PIN, 1);
        esp_rom_delay_us(STEP_HIGH_TIME_uS);
        gpio_set_level(STEP_PIN, 0);
        esp_rom_delay_us(step_time - STEP_HIGH_TIME_uS);
    }

    // Constant Speed Phase
    for (int i = 0; i < const_steps; i++) {
        gpio_set_level(STEP_PIN, 1);
        esp_rom_delay_us(STEP_HIGH_TIME_uS);
        gpio_set_level(STEP_PIN, 0);
        esp_rom_delay_us(min_step_time - STEP_HIGH_TIME_uS);
    }

    // Deceleration Phase
    for (int i = 0; i < decel_steps; i++) {
        int step_time = min_step_time + ((max_step_time - min_step_time) * i / decel_steps);
        gpio_set_level(STEP_PIN, 1);
        esp_rom_delay_us(STEP_HIGH_TIME_uS);
        gpio_set_level(STEP_PIN, 0);
        esp_rom_delay_us(step_time - STEP_HIGH_TIME_uS);
    }
}
void app_main() {
    setup_stepper_pins();
    tmc_uart_init(UART_PORT, TMC_TX, TMC_RX);
    tmc_set_microstepping(); 
    tmc_set_current(8, 31, 6);
    // home();
    vTaskDelay(pdMS_TO_TICKS(1000));

    while (1) {
        gpio_set_level(DIR_PIN, 0); // Set direction (1 or 0)
        vTaskDelay(pdMS_TO_TICKS(10)); // or use ets_delay_us(10);
        step(51200*2);
        gpio_set_level(DIR_PIN, 1); // Set direction (1 or 0)
        vTaskDelay(pdMS_TO_TICKS(10)); // or use ets_delay_us(10);
        step(51200*2);
    }
}