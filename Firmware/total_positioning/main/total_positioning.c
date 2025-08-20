#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_rom_sys.h"
#include <stdio.h>
#include <stdlib.h>

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

#define PulleyRadius 6.36

int current_x_position = 0;
int current_y_position = 0;

int final_x_position = 0;
int final_y_position = 0;

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

void setup_uart() {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void step(gpio_num_t STEP_PIN,int no_steps) {
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
    int min_step_time = 30;
    int max_step_time = 150;

    int total_steps=0;

    for (int i = 0; i < accel_steps; i++) {
        int step_time = max_step_time - ((max_step_time - min_step_time) * i / accel_steps);
        gpio_set_level(STEP_PIN, 1);
        esp_rom_delay_us(STEP_HIGH_TIME_uS);
        gpio_set_level(STEP_PIN, 0);
        esp_rom_delay_us(step_time - STEP_HIGH_TIME_uS);
        total_steps++;
    }

    for (int i = 0; i < const_steps; i++) {
        gpio_set_level(STEP_PIN, 1);
        esp_rom_delay_us(STEP_HIGH_TIME_uS);
        gpio_set_level(STEP_PIN, 0);
        esp_rom_delay_us(min_step_time - STEP_HIGH_TIME_uS);
        total_steps++;
    }

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

void single_step(gpio_num_t STEP_PIN) {
    gpio_set_level(STEP_PIN, 1);
    esp_rom_delay_us(STEP_HIGH_TIME_uS);
    gpio_set_level(STEP_PIN, 0);
    esp_rom_delay_us(STEP_TOTAL_TIME_uS - STEP_HIGH_TIME_uS);
}

void home_x_task(void *pvParameters) {
    gpio_set_level(X_DIR_PIN, 0);
    while (gpio_get_level(X_LIM_SWITCH_PIN) == 1) {
        single_step(X_STEP_PIN);
        vTaskDelay(pdMS_TO_TICKS(1));
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
    xTaskCreate(home_x_task, "Home X Axis", 2048, NULL, 1, NULL);
    xTaskCreate(home_y_task, "Home Y Axis", 2048, NULL, 1, NULL);
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
            uart_write_bytes(UART_NUM_0, (const char *)&ch, 1);
        }
    }
    input[idx] = '\0';
    return atoi(input);
}

void go_to_x_pos(void *pvParameters){
    if (current_x_position<final_x_position){
        int n=(final_x_position-current_x_position)*3200/(3.141592653589*PulleyRadius);
        gpio_set_level(X_DIR_PIN, 1);
        step(X_STEP_PIN,n);
    } else {
        int n=(current_x_position-final_x_position)*3200/(3.141592653589*PulleyRadius);
        gpio_set_level(X_DIR_PIN, 0); 
        step(X_STEP_PIN,n);
    }
    current_x_position=final_x_position;
    vTaskDelete(NULL);
}

void go_to_y_pos(void *pvParameters){
    if (current_y_position<final_y_position){
        int n=(final_y_position-current_y_position)*3200/(3.141592653589*PulleyRadius);
        gpio_set_level(Y_DIR_PIN, 1);
        step(Y_STEP_PIN,n);
    } else {
        int n=(current_y_position-final_y_position)*3200/(3.141592653589*PulleyRadius);
        gpio_set_level(Y_DIR_PIN, 0); 
        step(Y_STEP_PIN,n);
    }
    current_y_position=final_y_position;
    vTaskDelete(NULL);
}

void total_positioning(){
    xTaskCreate(go_to_x_pos, "Position X Axis", 4096, NULL, 1, NULL);
    xTaskCreate(go_to_y_pos, "Position Y Axis", 4096, NULL, 1, NULL);
}

void app_main() {
    setup_stepper_pins();
    setup_uart();

    gpio_set_level(M0_PIN, 1);
    gpio_set_level(M1_PIN, 0);
    gpio_set_level(M2_PIN, 1);

    total_homing();
    current_x_position=0;
    current_y_position=0;
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    while (1) {
        printf("\nEnter x position in mm: ");
        final_x_position = uart_read_int();
        printf("\nReceived: %d ", final_x_position);

        printf("\nEnter y position in mm: ");
        final_y_position = uart_read_int();
        printf("\nReceived: %d ", final_y_position);

        if (final_x_position>540 || final_y_position>400){
            printf("\nError: Out of bounds.");
        } else {
            total_positioning();
        }
    }
}
