#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <stdio.h>
#include "esp_rom_sys.h"

#define X_STEP_PIN GPIO_NUM_12
#define X_DIR_PIN  GPIO_NUM_14
#define Y_STEP_PIN GPIO_NUM_23
#define Y_DIR_PIN  GPIO_NUM_22

#define M0_PIN  GPIO_NUM_19
#define M1_PIN  GPIO_NUM_18
#define M2_PIN  GPIO_NUM_5

#define X_LIM_SWITCH_PIN  GPIO_NUM_15
#define Y_LIM_SWITCH_PIN  GPIO_NUM_13

#define STEP_HIGH_TIME_uS 5
#define STEP_TOTAL_TIME_uS 30

void setup_stepper_pins() {
    gpio_config_t io_conf_out = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << X_STEP_PIN) | (1ULL << X_DIR_PIN) |
                        (1ULL << Y_STEP_PIN) | (1ULL << Y_DIR_PIN) |
                        (1ULL << M0_PIN) | (1ULL << M1_PIN) | (1ULL << M2_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_conf_out);

    gpio_config_t io_conf_in = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << X_LIM_SWITCH_PIN) | (1ULL << Y_LIM_SWITCH_PIN),
        .pull_down_en = 0,
        .pull_up_en = 1
    };
    gpio_config(&io_conf_in);
}

void single_step(gpio_num_t STEP_PIN) {
    gpio_set_level(STEP_PIN, 1);
    esp_rom_delay_us(STEP_HIGH_TIME_uS);
    gpio_set_level(STEP_PIN, 0);
    esp_rom_delay_us(STEP_TOTAL_TIME_uS - STEP_HIGH_TIME_uS);
}

void home_x_task(void *pvParameters) {
    gpio_set_level(X_DIR_PIN, 0); // Move towards switch
    while (gpio_get_level(X_LIM_SWITCH_PIN) == 1) {
        single_step(X_STEP_PIN);
        vTaskDelay(pdMS_TO_TICKS(1)); // allow FreeRTOS to switch tasks
    }
    vTaskDelete(NULL);
}

void home_y_task(void *pvParameters) {
    gpio_set_level(Y_DIR_PIN, 0);
    while (gpio_get_level(Y_LIM_SWITCH_PIN) == 1) {
        single_step(Y_STEP_PIN);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    vTaskDelete(NULL);
}

void total_homing(){
    // Create tasks for X and Y homing to run in parallel
    xTaskCreate(home_x_task, "Home X Axis", 2048, NULL, 1, NULL);
    xTaskCreate(home_y_task, "Home Y Axis", 2048, NULL, 1, NULL);
}

void app_main() {
    setup_stepper_pins();

    gpio_set_level(M0_PIN, 1);
    gpio_set_level(M1_PIN, 0);
    gpio_set_level(M2_PIN, 1);

    total_homing();        
}
