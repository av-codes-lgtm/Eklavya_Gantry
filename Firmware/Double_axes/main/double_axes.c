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

#define LIM_SWITCH_PIN  GPIO_NUM_15

#define STEP_HIGH_TIME_uS 5
#define STEP_TOTAL_TIME_uS 30

void setup_stepper_pins() {
    // Configure stepper driver output pins
    gpio_config_t io_conf_out = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << X_STEP_PIN) | (1ULL << X_DIR_PIN) | (1ULL << Y_STEP_PIN) | (1ULL << Y_DIR_PIN) | 
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
        .pull_down_en = 1,
        .pull_up_en = 0 // Use pull-up if switch connects to GND when pressed
    };
    gpio_config(&io_conf_in);
}

void step(gpio_num_t STEP_PIN, int no_steps) {
    int accel_steps = 6400;
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
    int min_step_time = 30;  // Fastest (small delay between steps)
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

void single_step(gpio_num_t STEP_PIN) {
    gpio_set_level(STEP_PIN, 1);
    esp_rom_delay_us(STEP_HIGH_TIME_uS);
    gpio_set_level(STEP_PIN, 0);
    esp_rom_delay_us(100 - STEP_HIGH_TIME_uS);
}

void home(gpio_num_t DIR_PIN, gpio_num_t STEP_PIN) {
    gpio_set_level(DIR_PIN, 1); 
    int level1 = gpio_get_level(LIM_SWITCH_PIN);
    while (level1 == 1) {
        single_step(STEP_PIN);
        level1=gpio_get_level(LIM_SWITCH_PIN);
        if (level1==0){
            break;
        }
    }
    gpio_set_level(DIR_PIN, 0);
    step(STEP_PIN ,250);
}
void app_main() {
    setup_stepper_pins();
    
    gpio_set_level(M0_PIN, 1);
    gpio_set_level(M1_PIN, 0);
    gpio_set_level(M2_PIN, 1); // 1/32 microstepping.

    vTaskDelay(pdMS_TO_TICKS(1000));

    while (1) {
        gpio_set_level(X_DIR_PIN, 0); // Set direction (1 or 0)
        vTaskDelay(pdMS_TO_TICKS(10)); // or use ets_delay_us(10);
        step(X_STEP_PIN, 12800*5/2);
        gpio_set_level(X_DIR_PIN, 1); // Set direction (1 or 0)
        vTaskDelay(pdMS_TO_TICKS(10)); // or use ets_delay_us(10);
        step(X_STEP_PIN, 12800*5/2);

        gpio_set_level(Y_DIR_PIN, 0); // Set direction (1 or 0)
        vTaskDelay(pdMS_TO_TICKS(10)); // or use ets_delay_us(10);
        step(Y_STEP_PIN, 12800*5/2);
        gpio_set_level(Y_DIR_PIN, 1); // Set direction (1 or 0)
        vTaskDelay(pdMS_TO_TICKS(10)); // or use ets_delay_us(10);
        step(Y_STEP_PIN, 12800*5/2);
    }
}
