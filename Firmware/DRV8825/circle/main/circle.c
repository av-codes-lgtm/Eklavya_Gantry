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

#define M0_PIN  GPIO_NUM_19
#define M1_PIN  GPIO_NUM_18
#define M2_PIN  GPIO_NUM_5

#define X_LIM_SWITCH_PIN  GPIO_NUM_15
#define Y_LIM_SWITCH_PIN  GPIO_NUM_13
#define LIM_ACTIVE_LEVEL  0

#define STEP_PULSE_HIGH_US 5
#define MAX_STEP_INTERVAL_US 150
#define MIN_STEP_INTERVAL_US 30

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
}

static void line_move_mm_constant(int target_x_mm, int target_y_mm) {
    // Clamp target within limits
    if (target_x_mm < 0) target_x_mm = 0;
    if (target_y_mm < 0) target_y_mm = 0;
    if (target_x_mm > X_MAX_MM) target_x_mm = X_MAX_MM;
    if (target_y_mm > Y_MAX_MM) target_y_mm = Y_MAX_MM;

    // Calculate deltas
    int dx_mm = target_x_mm - current_x_mm;
    int dy_mm = target_y_mm - current_y_mm;

    int sx = (dx_mm >= 0) ? 1 : -1;
    int sy = (dy_mm >= 0) ? 1 : -1;

    int dx_steps = abs(mm_to_steps((double)dx_mm));
    int dy_steps = abs(mm_to_steps((double)dy_mm));

    if (dx_steps == 0 && dy_steps == 0) return;

    // Set direction pins
    gpio_set_level(X_DIR_PIN, (sx > 0) ? 1 : 0);
    gpio_set_level(Y_DIR_PIN, (sy > 0) ? 1 : 0);

    // Bresenham setup
    int primary = (dx_steps >= dy_steps) ? dx_steps : dy_steps;
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

        // Limit switch stop
        if ((read_limit(X_LIM_SWITCH_PIN) == LIM_ACTIVE_LEVEL && sx < 0) ||
            (read_limit(Y_LIM_SWITCH_PIN) == LIM_ACTIVE_LEVEL && sy < 0)) {
            printf("\n[HALT] Hit limit during move.\n");
            break;
        }

        // Constant speed delay
        esp_rom_delay_us(350 - STEP_PULSE_HIGH_US);
    }

    // Update current position
    current_x_mm += sx * (int)llround((double)x_cnt / STEPS_PER_MM);
    current_y_mm += sy * (int)llround((double)y_cnt / STEPS_PER_MM);

    if (current_x_mm < 0) current_x_mm = 0;
    if (current_y_mm < 0) current_y_mm = 0;
    if (current_x_mm > X_MAX_MM) current_x_mm = X_MAX_MM;
    if (current_y_mm > Y_MAX_MM) current_y_mm = Y_MAX_MM;
}

static void circle_move_mm_blocking(int cx, int cy, int radius, int segments) {
    double angle_step = 2.0 * M_PI / (double)segments;
    double angle = 0.0;

    int start_x = cx + (int)llround((double)radius);
    int start_y = cy;

    // move to starting point
    line_move_mm_constant(start_x, start_y);

    for (int i = 1; i <= segments; i++) {
        angle = i * angle_step;
        int x = cx + (int)llround((double)radius * cos(angle));
        int y = cy + (int)llround((double)radius * sin(angle));
        line_move_mm_constant(x, y);
    }
}

// static void circle_move_mm_blocking(int cx, int cy, int radius, int segments) {
//     double angle_step = 2.0 * M_PI / (double)segments;

//     // start point
//     double x_prev = cx + radius;
//     double y_prev = cy;

//     // loop through arc segments
//     for (int i = 1; i <= segments; i++) {
//         double angle = i * angle_step;
//         double x_next = cx + radius * cos(angle);
//         double y_next = cy + radius * sin(angle);

//         // convert mm to steps
//         int dx = (int)llround((x_next - x_prev) * STEPS_PER_MM);
//         int dy = (int)llround((y_next - y_prev) * STEPS_PER_MM);

//         int steps = (int)llround(sqrt((double)(dx * dx + dy * dy)));
//         if (steps <= 0) continue;

//         // increment per step
//         double x_inc = (double)dx / (double)steps;
//         double y_inc = (double)dy / (double)steps;

//         double x = x_prev * STEPS_PER_MM;
//         double y = y_prev * STEPS_PER_MM;

//         for (int s = 0; s < steps; s++) {
//             x += x_inc;
//             y += y_inc;

//             // issue step pulses
//             if ((int)llround(x) % 1 == 0) {
//                 gpio_set_level(X_STEP_PIN, 1);
//                 esp_rom_delay_us(STEP_PULSE_HIGH_US);
//                 gpio_set_level(X_STEP_PIN, 0);
//             }
//             if ((int)llround(y) % 1 == 0) {
//                 gpio_set_level(X_STEP_PIN, 1);
//                 esp_rom_delay_us(STEP_PULSE_HIGH_US);
//                 gpio_set_level(X_STEP_PIN, 0);
//             }

//             // constant interval between steps
//             esp_rom_delay_us(MIN_STEP_INTERVAL_US - STEP_PULSE_HIGH_US);
//         }

//         x_prev = x_next;
//         y_prev = y_next;
//     }
// }


void app_main(void) {
    setup_stepper_pins();
    // setup_uart();

    gpio_set_level(M0_PIN, 1);
    gpio_set_level(M1_PIN, 0);
    gpio_set_level(M2_PIN, 1);

    printf("\nHoming...\n");
    home_all();
    vTaskDelay(pdMS_TO_TICKS(200));
    
    circle_move_mm_blocking(250, 200, 75, 3600);

}
