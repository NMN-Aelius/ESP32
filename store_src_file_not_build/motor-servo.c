/*
* SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
*
* SPDX-License-Identifier: Apache-2.0
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"

static const char *TAG = "example";

// Please consult the datasheet of your servo before changing the following parameters
#define SERVO_MIN_PULSEWIDTH_US 0     // Minimum pulse width in microsecond = 0.5ms
#define SERVO_MAX_PULSEWIDTH_US 2000    // Maximum pulse width in microsecond = 2.5ms
#define SERVO_MIN_DEGREE        0     // Minimum angle
#define SERVO_MAX_DEGREE        180      // Maximum angle

#define SERVO_PULSE_GPIO             19        // GPIO connects to the PWM signal line
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

static inline uint32_t example_angle_to_compare(int angle)
{
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US)\
            / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;

    /* 
    Imaging the Oxy dimention with
    - Ox: is the Delta change of Angle
    - Oy: is the Phi change of Pulse width
    Linear interpolation (lerp), We can compute the amount of change Angle (degree)
    with respectively PulseWidth in PWM
    */
}

void app_main(void)
{
    /* Timer này được cấu hình: 0.02s (hoàn thành 1 chu kì timer đếm lên rồi về 0)
    Mục đích của timer này để tạo tính hiệu PWM
    Độ rộng xung PWM phụ thuộc vào config:
    - 1 chu kì xung PWM: 0.02s
    - tần số là 50Hz
    */
    ESP_LOGI(TAG, "Create timer and operator");
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ, //1.000.000hz => 1us
        .period_ticks = SERVO_TIMEBASE_PERIOD, // số lượng tick cho mỗi chu kì PWM 20.000 
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    /* Tạo operator mới để đảm nhận control (servo) */
    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator must be in the same group to the timer
        /* Cần phải nằm chung với group cấu hình timer */
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    /* Kết nối chúng lại với nhau */
    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    ESP_LOGI(TAG, "Create comparator and generator from the operator");
    mcpwm_cmpr_handle_t comparator = NULL;
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true, //update lại giá trị duty cycles
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    /* Sau khi config tất cả các thứ liên quan về TIMER 
    Phase tiếp theo là set cho một pin cụ thể */
    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = SERVO_PULSE_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));
    // 1 thông số chung về khối operator - 2 thông số còn lại thuộc oop, trong đó gồm config và handle ptr

    // set the initial compare value, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0)));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
                                                            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
                                                            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    int angle = 0;
    int step = 1;
    while (1) {
        ESP_LOGI(TAG, "Angle of rotation: %d", angle);
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(angle)));
        //Add delay, since it takes time for servo to rotate, usually 200ms/60degree rotation under 5V power supply
        vTaskDelay(pdMS_TO_TICKS(500));
        // if ((angle + step) > 60 || (angle + step) < -60) {
        //     step *= -1; //đảo chiều
        // }
        if ((angle + step) > 45 ) {
            step = 0; //stop
            mcpwm_timer_start_stop(timer, MCPWM_TIMER_STOP_EMPTY);
        }
        angle += step;
    }
}