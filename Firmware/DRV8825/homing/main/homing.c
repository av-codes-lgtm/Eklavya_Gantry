#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <stdio.h>
#include "esp_rom_sys.h"

#define STEP_PIN GPIO_NUM_12
#define DIR_PIN  GPIO_NUM_14

#define M0_PIN  GPIO_NUM_19
#define M1_PIN  GPIO_NUM_18
#define M2_PIN  GPIO_NUM_5

#define LIM_SWITCH_PIN  GPIO_NUM_15

#define STEP_HIGH_TIME_uS 5
#define STEP_TOTAL_TIME_uS 30

void setup_stepper_pins() {
    // Configure stepper driver output pins
    gpio_config_t io_conf_out = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << STEP_PIN) | (1ULL << DIR_PIN) | 
                        (1ULL << M0_PIN) | (1ULL << M1_PIN) | (1ULL << M2_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_conf_out);

    // Configure limit switch input pin with pull-up
    gpio_config_t io_conf_in = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << LIM_SWITCH_PIN),
        .pull_down_en = 0,
        .pull_up_en = 1 // Use pull-up if switch connects to GND when pressed
    };
    gpio_config(&io_conf_in);
}

void single_step() {
    gpio_set_level(STEP_PIN, 1);
    esp_rom_delay_us(STEP_HIGH_TIME_uS);
    gpio_set_level(STEP_PIN, 0);
    esp_rom_delay_us(100 - STEP_HIGH_TIME_uS);
}

void app_main() {
    setup_stepper_pins();
    
    gpio_set_level(M0_PIN, 1);
    gpio_set_level(M1_PIN, 0);
    gpio_set_level(M2_PIN, 1);

    while (1) {
        gpio_set_level(DIR_PIN, 0); // Adjust depending on your wiring

        int level1 = gpio_get_level(LIM_SWITCH_PIN);
        while (level1 == 1) {
            single_step();
            level1=gpio_get_level(LIM_SWITCH_PIN);
        }
    }
}
