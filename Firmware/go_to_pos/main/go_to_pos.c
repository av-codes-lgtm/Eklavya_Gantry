#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include <stdio.h>
#include "esp_rom_sys.h"

#define STEP_PIN GPIO_NUM_12
#define DIR_PIN  GPIO_NUM_14

#define M0_PIN  GPIO_NUM_19
#define M1_PIN  GPIO_NUM_18
#define M2_PIN  GPIO_NUM_5
#define PulleyRadius 6.36

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

    const uart_port_t uart_num = UART_NUM_0;

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(uart_num, &uart_config);
    uart_driver_install(uart_num, 1024, 0, 0, NULL, 0);

}

void step(int no_steps) {
    int accel_steps = 6400;
    int decel_steps = accel_steps;
    if (no_steps<accel_steps){
        accel_steps=no_steps/2;
        decel_steps=accel_steps;
    }
    int const_steps = no_steps - accel_steps - decel_steps;
    if (const_steps<0){
        const_steps=0;
        accel_steps=no_steps/2;
        decel_steps=accel_steps;
    }
    // Define min and max total step time (in microseconds)
    int min_step_time = 30;  // Fastest (small delay between steps)
    int max_step_time = 150; // Slowest (long delay between steps)

    int total_steps=0;

    // Acceleration Phase
    for (int i = 0; i < accel_steps; i++) {
        int step_time = max_step_time - ((max_step_time - min_step_time) * i / accel_steps);
        gpio_set_level(STEP_PIN, 1);
        esp_rom_delay_us(STEP_HIGH_TIME_uS);
        gpio_set_level(STEP_PIN, 0);
        esp_rom_delay_us(step_time - STEP_HIGH_TIME_uS);
        total_steps++;
    }

    // Constant Speed Phase
    for (int i = 0; i < const_steps; i++) {
        gpio_set_level(STEP_PIN, 1);
        esp_rom_delay_us(STEP_HIGH_TIME_uS);
        gpio_set_level(STEP_PIN, 0);
        esp_rom_delay_us(min_step_time - STEP_HIGH_TIME_uS);
        total_steps++;
    }

    // Deceleration Phase
    for (int i = 0; i < decel_steps; i++) {
        int step_time = min_step_time + ((max_step_time - min_step_time) * i / decel_steps);
        gpio_set_level(STEP_PIN, 1);
        esp_rom_delay_us(STEP_HIGH_TIME_uS);
        gpio_set_level(STEP_PIN, 0);
        esp_rom_delay_us(step_time - STEP_HIGH_TIME_uS);
        total_steps++;
    }

    printf("Total steps taken = %d\n" ,total_steps);
}

void single_step() {
    gpio_set_level(STEP_PIN, 1);
    esp_rom_delay_us(STEP_HIGH_TIME_uS);
    gpio_set_level(STEP_PIN, 0);
    esp_rom_delay_us(100 - STEP_HIGH_TIME_uS);
}

void home() {
    gpio_set_level(DIR_PIN, 0); // Move towards switch
    while (gpio_get_level(LIM_SWITCH_PIN) == 1) {
        single_step(STEP_PIN);
        //vTaskDelay(pdMS_TO_TICKS(1)); // allow FreeRTOS to switch tasks
    }
    //vTaskDelete(NULL);
}

int uart_read_int() {
    char input[32] = {0};
    int idx = 0;
    uint8_t ch;

    while (idx < sizeof(input) - 1) {
        int len = uart_read_bytes(UART_NUM_0, &ch, 1, pdMS_TO_TICKS(10000));
        if (len > 0) {
            if (ch == '\n' || ch == '\r') {
                break;
            }
            input[idx++] = ch;
            uart_write_bytes(UART_NUM_0, (const char *)&ch, 1);  // Echo back
        }
    }
    input[idx] = '\0';
    return atoi(input);
}


void app_main() {
    setup_stepper_pins();
    
    gpio_set_level(M0_PIN, 1);
    gpio_set_level(M1_PIN, 0);
    gpio_set_level(M2_PIN, 1); // 1/32 microstepping.

    home();
    int current_position=0;
    vTaskDelay(pdMS_TO_TICKS(1000));

    while (1) {
        int final_position;
        printf("\nEnter position in mm: ");
        final_position = uart_read_int();
        printf("\nReceived: %d ", final_position);
        if (final_position>540){
            printf("\neeee bhaii itna nhi jaeega mai.");
        } else {
            if (current_position<final_position){
                int n=(final_position-current_position)*3200/(3.141592653589*PulleyRadius);
                gpio_set_level(DIR_PIN, 1);
                step(n);
            } else {
                int n=(current_position-final_position)*3200/(3.141592653589*PulleyRadius);
                gpio_set_level(DIR_PIN, 0); 
                step(n);
            }

            current_position=final_position;
        }
    }
}