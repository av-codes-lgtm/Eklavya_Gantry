#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define EnablePIN GPIO_NUM_4
#define PIN1 GPIO_NUM_16
#define PIN2 GPIO_NUM_17

void app_main(void)
{
    // Configure GPIOs as outputs
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << EnablePIN) | (1ULL << PIN1) | (1ULL << PIN2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(EnablePIN, 1);
    while (1) {
        // ===== Level 1: All HIGH =====
        printf("Level 1: forward\n");    
        gpio_set_level(PIN1, 1);
        gpio_set_level(PIN2, 0);
        vTaskDelay(pdMS_TO_TICKS(2000)); // 5 seconds

        // ===== Level 2: GPIO_19 LOW =====
        printf("Level 2: backward\n");
        gpio_set_level(PIN1, 0);
        gpio_set_level(PIN2, 1);
        vTaskDelay(pdMS_TO_TICKS(2000)); // 5 seconds
    }
}