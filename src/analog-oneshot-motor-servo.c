/*
* SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
*
* SPDX-License-Identifier: Apache-2.0
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/mcpwm_prelude.h"

#include "soc/soc_caps.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "demo-adc-motor";

/* MOTOR STEP DEFINE VALUE */
#define MIN_PULSEWIDTH_US 500   // Minimum pulse width in microsecond = 0.5ms
#define MAX_PULSEWIDTH_US 2120  // Maximum pulse width in microsecond = 2.5ms
//datasheet of servo
#define MIN_ANGLE_DEGREE  0     // Minimum angle
#define MAX_ANGLE_DEGREE  180   // Maximum angle

#define MOTOR_GPIO_19             19
#define MOTOR_GPIO_18             18
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        3000    // 20000 ticks, 20ms

/* ADC DEFINE VALUE */
#define ADC1_PIN34 ADC_CHANNEL_6 //PIN 34 DEVKIT-DOIT
#define ADC1_PIN35 ADC_CHANNEL_7 //PIN 35 DEVKIT-DOIT

// static int adc_raw[1][10]; // 1 is x in ADCx, 10 is number of channel can have in ADC
// static int voltage[1][10]; // respectively with adc_raw.

/* FUNCTION MOTOR SERVO */
static inline uint32_t scale_angle_to_ticks(int angle)
{
    return ((angle-MIN_ANGLE_DEGREE) * (MAX_PULSEWIDTH_US-MIN_PULSEWIDTH_US) / (MAX_ANGLE_DEGREE-MIN_ANGLE_DEGREE)) + MIN_PULSEWIDTH_US;
}
/* ================ END FUNCTION ================*/

void app_main(void)
{
    /*=================MOTOR CONTROL=================*/
    mcpwm_timer_handle_t timer_handle = NULL;
    mcpwm_timer_config_t timer_cfg = {
        .group_id = 0, //unit 0 pwm có hai unit
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        
        .resolution_hz = 1000000,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
    };
    mcpwm_new_timer(&timer_cfg, &timer_handle);

    mcpwm_oper_handle_t oper_handle =  NULL;
    mcpwm_operator_config_t oper_config = {
        .group_id = 0,
        // .flags no flag is need
    };
    mcpwm_new_operator(&oper_config, &oper_handle);
    mcpwm_operator_connect_timer(oper_handle, timer_handle);

    mcpwm_cmpr_handle_t cmp_handle =NULL;
    mcpwm_comparator_config_t compare_cfg = {
        .flags.update_cmp_on_tez = 1,
        //only take value 1 or 0, because it is a bit (when timer set to 0 it will be trigger)
    };
    mcpwm_new_comparator(oper_handle, &compare_cfg, &cmp_handle);

    mcpwm_gen_handle_t gen_handle = NULL;
    mcpwm_generator_config_t gene_config = {
        .gen_gpio_num = MOTOR_GPIO_18,
    };
    mcpwm_new_generator(oper_handle, &gene_config, &gen_handle);

    //Set init value for the first time of servo motor
    mcpwm_comparator_set_compare_value(cmp_handle, scale_angle_to_ticks(0));

    //Generator là bộ tạo xung pwm, dựa trên event được gửi từ comparator nó sẽ có action tương ứng
    //Sau đây là việc cấu hình các actions này

    ESP_LOGI(TAG, "Set actions for generator to deal with pwm pulse!!");
    mcpwm_generator_set_action_on_timer_event(gen_handle,\
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
        /* Khi timer đang đếm lên, và đạt giá trị bằng 0 (TEZ), thì generator sẽ đưa chân GPIO đầu ra lên mức cao (HIGH). */

    mcpwm_generator_set_action_on_compare_event(gen_handle,\
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmp_handle , MCPWM_GEN_ACTION_LOW));
        /* Lấy event từ cmp_handle block to get action LOW when COMPARE condition is met. (COUNT UP mới có action này) */

    ESP_LOGI(TAG, "Timer go go!!");
    mcpwm_timer_enable(timer_handle);
    mcpwm_timer_start_stop(timer_handle, MCPWM_TIMER_START_NO_STOP);

    /*=================ADC CONTROL=================*/

    int angle = 0;
    int step = 1;
    bool flag = 0;

    while (1) {
        
        ESP_LOGI(TAG, "Angle of rotation: %d", angle);
        int check = scale_angle_to_ticks(angle);
        ESP_LOGI(TAG, "TICKs %d", check);
        mcpwm_comparator_set_compare_value(cmp_handle, scale_angle_to_ticks(angle));

        if(flag)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
            ESP_LOGI(TAG, "Timer go go!!");
            mcpwm_timer_start_stop(timer_handle, MCPWM_TIMER_START_NO_STOP);
            flag = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(500)); //take time for motor rotation.
        if ((angle + step) > 180 ) {
            // step = 0; //stop
            angle = 0;
            mcpwm_timer_start_stop(timer_handle, MCPWM_TIMER_STOP_EMPTY);
            flag = 1;
        }
        angle += step;
    }
}