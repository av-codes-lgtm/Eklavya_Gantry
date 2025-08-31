#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <stdio.h>

#define STEP_PIN GPIO_NUM_22
#define DIR_PIN  GPIO_NUM_23

#define STEP_HIGH_TIME_MS 1.9
#define STEP_TOTAL_TIME_MS 25

void setup_stepper_pins() {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << STEP_PIN) | (1ULL << DIR_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_conf);

    gpio_set_level(DIR_PIN, 1); // Set direction (1 or 0)
    vTaskDelay(pdMS_TO_TICKS(1)); // or use ets_delay_us(10);
}

void app_main() {

    setup_stepper_pins();

    while (1) {
        gpio_set_level(STEP_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(STEP_HIGH_TIME_MS)); // Minimum high time (1.9 ms required)
        gpio_set_level(STEP_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(STEP_TOTAL_TIME_MS-STEP_HIGH_TIME_MS)); // Remaining delay to make total 300 ms per step
    }
}
