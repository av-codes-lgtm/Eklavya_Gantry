#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <stdio.h>
#include "esp_rom_sys.h"


#define LIM_SWITCH_PIN  GPIO_NUM_15

void setup_stepper_pins() {

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

void app_main() {
    setup_stepper_pins();

    while (1) {
        int level1 = gpio_get_level(LIM_SWITCH_PIN);
        while (level1 == 1) {
            level1 = gpio_get_level(LIM_SWITCH_PIN);
            printf("1\n");                       // Print with newline so it's flushed better
            vTaskDelay(pdMS_TO_TICKS(50));        // Let FreeRTOS run & feed watchdog
        }
        printf("0\n");
        vTaskDelay(pdMS_TO_TICKS(50));            // Delay here too
    }
}

// void app_main() {
//     setup_stepper_pins();

//     while (1) {
//         int level1 = gpio_get_level(LIM_SWITCH_PIN);
//         while (level1 == 1) {
//             level1=gpio_get_level(LIM_SWITCH_PIN);
//             printf("1");
//         }
//         printf("0");
//     }
// }
