#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_rom_sys.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define X_STEP_PIN GPIO_NUM_12
#define X_DIR_PIN  GPIO_NUM_14
#define Y_STEP_PIN GPIO_NUM_23
#define Y_DIR_PIN  GPIO_NUM_22
#define Z_STEP_PIN GPIO_NUM_26
#define Z_DIR_PIN  GPIO_NUM_25

#define M0_PIN  GPIO_NUM_19
#define M1_PIN  GPIO_NUM_18
#define M2_PIN  GPIO_NUM_5

#define EnablePIN GPIO_NUM_4
#define PIN1 GPIO_NUM_16
#define PIN2 GPIO_NUM_17

#define X_LIM_SWITCH_PIN  GPIO_NUM_15
#define Y_LIM_SWITCH_PIN  GPIO_NUM_13
// #define Z_LIM_SWITCH_PIN  GPIO_NUM_
#define LIM_ACTIVE_LEVEL  0

#define STEP_PULSE_HIGH_US 5
#define MAX_STEP_INTERVAL_US 150
#define MIN_STEP_INTERVAL_US 30

#define STEP_HIGH_TIME_uS 5
#define STEP_TOTAL_TIME_uS 30

#define PULLEY_RADIUS_MM 6.36
#define STEPS_PER_REV 3200
#define PI 6.28318530717958647692/2
#define STEPS_PER_MM ( (double)STEPS_PER_REV / (PI * (double)PULLEY_RADIUS_MM) )

#define X_MAX_MM 540
#define Y_MAX_MM 390

static volatile int current_x_mm = 0;
static volatile int current_y_mm = 0;

static inline void step_pulse(gpio_num_t step_pin) {
    gpio_set_level(step_pin, 1);
    esp_rom_delay_us(STEP_PULSE_HIGH_US);
    gpio_set_level(step_pin, 0);
}

static void setup_stepper_pins(void) {
    gpio_config_t io_conf_out = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask =
            (1ULL << X_STEP_PIN) | (1ULL << X_DIR_PIN) |
            (1ULL << Y_STEP_PIN) | (1ULL << Y_DIR_PIN) |
            (1ULL << Z_STEP_PIN) | (1ULL << Z_DIR_PIN) |
            (1ULL << EnablePIN) | (1ULL << PIN1) | (1ULL << PIN2) |
            (1ULL << M0_PIN) | (1ULL << M1_PIN) | (1ULL << M2_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_conf_out);

    gpio_config_t io_conf_in = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << X_LIM_SWITCH_PIN) | (1ULL << Y_LIM_SWITCH_PIN) ,
        .pull_down_en = 0,
        .pull_up_en = 1
    };
    gpio_config(&io_conf_in);
}

static void setup_uart(void) {
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

static int uart_read_int_blocking(void) {
    char buf[32] = {0};
    int idx = 0;
    uint8_t ch;
    while (idx < (int)sizeof(buf) - 1) {
        int len = uart_read_bytes(UART_NUM_0, &ch, 1, portMAX_DELAY);
        if (len > 0) {
            if (ch == '\n' || ch == '\r') break;
            buf[idx++] = (char)ch;
            uart_write_bytes(UART_NUM_0, (const char *)&ch, 1);
        }
    }
    buf[idx] = '\0';
    return atoi(buf);
}

static inline int mm_to_steps(double mm) {
    double s = mm * STEPS_PER_MM;
    return (int)llround(s);
}

static inline int read_limit(gpio_num_t pin) {
    return gpio_get_level(pin);
}

static void home_axis(gpio_num_t step_pin, gpio_num_t dir_pin, gpio_num_t lim_pin, int *current_mm) {
    gpio_set_level(dir_pin, 0);
    int guard_steps = mm_to_steps( (double)(X_MAX_MM + Y_MAX_MM) * 2.0 );
    while (read_limit(lim_pin) != LIM_ACTIVE_LEVEL && guard_steps-- > 0) {
        step_pulse(step_pin);
        esp_rom_delay_us(MAX_STEP_INTERVAL_US);
    }
    vTaskDelay(pdMS_TO_TICKS(50));
    if (read_limit(lim_pin) != LIM_ACTIVE_LEVEL) {
        printf("\n[WARN] Homing failed on pin %d (limit not triggered)\n", (int)lim_pin);
    }
    *current_mm = 0;
}

static void home_all(void) {
    home_axis(X_STEP_PIN, X_DIR_PIN, X_LIM_SWITCH_PIN, (int*)&current_x_mm);
    home_axis(Y_STEP_PIN, Y_DIR_PIN, Y_LIM_SWITCH_PIN, (int*)&current_y_mm);
    // home_axis(Z_STEP_PIN, Z_DIR_PIN, Z_LIM_SWITCH_PIN, (int*)&current_x_mm);
}

static void line_move_mm_blocking(int target_x_mm, int target_y_mm) {
    if (target_x_mm < 0) target_x_mm = 0;
    if (target_y_mm < 0) target_y_mm = 0;
    if (target_x_mm > X_MAX_MM) target_x_mm = X_MAX_MM;
    if (target_y_mm > Y_MAX_MM) target_y_mm = Y_MAX_MM;

    int dx_mm = target_x_mm - current_x_mm;
    int dy_mm = target_y_mm - current_y_mm;

    int sx = (dx_mm >= 0) ? 1 : -1;
    int sy = (dy_mm >= 0) ? 1 : -1;

    int dx_steps = abs(mm_to_steps((double)dx_mm));
    int dy_steps = abs(mm_to_steps((double)dy_mm));

    if (dx_steps == 0 && dy_steps == 0) return;

    gpio_set_level(X_DIR_PIN, (sx > 0) ? 1 : 0);
    gpio_set_level(Y_DIR_PIN, (sy > 0) ? 1 : 0);

    int primary = (dx_steps >= dy_steps) ? dx_steps : dy_steps;
    int accel_steps = (int)llround(0.2 * (double)primary);
    if (accel_steps < 1) accel_steps = (primary >= 3) ? 1 : 0;
    int decel_steps = accel_steps;
    int const_steps = primary - accel_steps - decel_steps;
    if (const_steps < 0) { const_steps = 0; accel_steps = primary / 2; decel_steps = primary - accel_steps; }

    int err = (dx_steps >= dy_steps) ? (dx_steps / 2) : (dy_steps / 2);
    int x_cnt = 0, y_cnt = 0;

    for (int i = 0; i < primary; i++) {
        if (dx_steps >= dy_steps) {
            err -= dy_steps;
            if (err < 0) { err += dx_steps; step_pulse(Y_STEP_PIN); y_cnt++; }
            step_pulse(X_STEP_PIN); x_cnt++;
        } else {
            err -= dx_steps;
            if (err < 0) { err += dy_steps; step_pulse(X_STEP_PIN); x_cnt++; }
            step_pulse(Y_STEP_PIN); y_cnt++;
        }

        if ((read_limit(X_LIM_SWITCH_PIN) == LIM_ACTIVE_LEVEL && sx < 0) ||
            (read_limit(Y_LIM_SWITCH_PIN) == LIM_ACTIVE_LEVEL && sy < 0)) {
            printf("\n[HALT] Hit limit during move.\n");
            break;
        }

        int interval_us = MAX_STEP_INTERVAL_US;
        if (i < accel_steps && accel_steps > 0) {
            interval_us = MAX_STEP_INTERVAL_US - (int)((double)(MAX_STEP_INTERVAL_US - MIN_STEP_INTERVAL_US) * (double)i / (double)accel_steps);
        } else if (i >= accel_steps + const_steps && decel_steps > 0) {
            int di = i - (accel_steps + const_steps);
            interval_us = MIN_STEP_INTERVAL_US + (int)((double)(MAX_STEP_INTERVAL_US - MIN_STEP_INTERVAL_US) * (double)di / (double)decel_steps);
        } else {
            interval_us = MIN_STEP_INTERVAL_US;
        }
        if (interval_us < STEP_PULSE_HIGH_US + 1) interval_us = STEP_PULSE_HIGH_US + 1;
        esp_rom_delay_us(interval_us - STEP_PULSE_HIGH_US);
    }

    current_x_mm += sx * (int)llround((double)x_cnt / STEPS_PER_MM);
    current_y_mm += sy * (int)llround((double)y_cnt / STEPS_PER_MM);
    if (current_x_mm < 0) current_x_mm = 0;
    if (current_y_mm < 0) current_y_mm = 0;
    if (current_x_mm > X_MAX_MM) current_x_mm = X_MAX_MM;
    if (current_y_mm > Y_MAX_MM) current_y_mm = Y_MAX_MM;
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
    }
    // Define min and max total step time (in microseconds)
    int min_step_time = 30;  // Fastest (small delay between steps)
    int max_step_time = 150; // Slowest (long delay between steps)

    // Acceleration Phase
    for (int i = 0; i < accel_steps; i++) {
        int step_time = max_step_time - ((max_step_time - min_step_time) * i / accel_steps);
        gpio_set_level(Z_STEP_PIN, 1);
        esp_rom_delay_us(STEP_HIGH_TIME_uS);
        gpio_set_level(Z_STEP_PIN, 0);
        esp_rom_delay_us(step_time - STEP_HIGH_TIME_uS);
    }

    // Constant Speed Phase
    for (int i = 0; i < const_steps; i++) {
        gpio_set_level(Z_STEP_PIN, 1);
        esp_rom_delay_us(STEP_HIGH_TIME_uS);
        gpio_set_level(Z_STEP_PIN, 0);
        esp_rom_delay_us(min_step_time - STEP_HIGH_TIME_uS);
    }

    // Deceleration Phase
    for (int i = 0; i < decel_steps; i++) {
        int step_time = min_step_time + ((max_step_time - min_step_time) * i / decel_steps);
        gpio_set_level(Z_STEP_PIN, 1);
        esp_rom_delay_us(STEP_HIGH_TIME_uS);
        gpio_set_level(Z_STEP_PIN, 0);
        esp_rom_delay_us(step_time - STEP_HIGH_TIME_uS);
    }
}

void app_main(void) {
    setup_stepper_pins();
    setup_uart();

    gpio_set_level(M0_PIN, 1);
    gpio_set_level(M1_PIN, 0);
    gpio_set_level(M2_PIN, 1);

    printf("\nHoming...\n");
    home_all();
    vTaskDelay(pdMS_TO_TICKS(200));

    while (1) {
        printf("\nEnter X (mm, 0-%d): ", X_MAX_MM);
        int tx = uart_read_int_blocking();
        printf("\nEnter Y (mm, 0-%d): ", Y_MAX_MM);
        int ty = uart_read_int_blocking();
        printf("\nEnter Z action (0 = Screw, 1 = Unscrew): ");
        int tz = uart_read_int_blocking();

        if (tz != 0 && tz != 1) {
            printf("\nInvalid Z action! Please enter 0 or 1.\n");
            // you can choose to re-prompt here if needed
        }

        if (tx < 0 || ty < 0 || tx > X_MAX_MM || ty > Y_MAX_MM) {
            printf("\nError: target out of bounds\n");
            continue;
        }

        line_move_mm_blocking(tx, ty);
        if (tz == 0){
            gpio_set_level(Z_DIR_PIN, 0); // Set direction (1 or 0)
            vTaskDelay(pdMS_TO_TICKS(20)); // or use ets_delay_us(10);
            step(6400*4);
        }
        if (tz == 1){
            gpio_set_level(Z_DIR_PIN, 1); // Set direction (1 or 0)
            vTaskDelay(pdMS_TO_TICKS(20)); // or use ets_delay_us(10);
            step(6400*4);
        }
        // gpio_set_level(EnablePIN, 1);
        // if (tz == 0) {
        //     // ===== Screw (Forward) =====
        //     gpio_set_level(Z_DIR_PIN, 1);
        //         for (int i = 0; i < 6400; i++) {
        //         step_pulse(Z_STEP_PIN);
        //     }

        //     printf("Screw: forward\n");    
        //     gpio_set_level(PIN1, 1);
        //     gpio_set_level(PIN2, 0);
        //     vTaskDelay(pdMS_TO_TICKS(2000)); // run for 2 sec
        // } else {
        //     // ===== Unscrew (Backward) =====
        //     gpio_set_level(Z_DIR_PIN, 0);
        //         for (int i = 0; i < 6400; i++) {
        //         step_pulse(Z_STEP_PIN);
        //     }
        //     printf("Unscrew: backward\n");
        //     gpio_set_level(PIN1, 0);
        //     gpio_set_level(PIN2, 1);
        //     vTaskDelay(pdMS_TO_TICKS(2000)); // run for 2 sec
        // }


        printf("\nReached (%d, %d)\n", current_x_mm, current_y_mm);
    }
}
