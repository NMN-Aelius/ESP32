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

/* ================ START DEFINE ================*/
//MOTOR Parameter
#define MIN_PULSEWIDTH_US 500   // Minimum pulse width in microsecond = 0.5ms
#define MAX_PULSEWIDTH_US 2120  // Maximum pulse width in microsecond = 2.5ms

#define MIN_ANGLE_DEGREE    0   // Minimum angle
#define MIN_RAW_ADC_DEGREE  0   // Minimum rawADC
#define MAX_ANGLE_DEGREE    180 // Maximum angle
#define MAX_RAW_ADC_DEGREE  4095// Maximum rawADC

#define MOTOR_GPIO_18             18
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        3000    // 20000 ticks, 20ms

//ADC Parameter
#define ADC1_PIN34 ADC_CHANNEL_6 //PIN 34 DEVKIT-DOIT
#define ADC1_PIN35 ADC_CHANNEL_7 //PIN 35 DEVKIT-DOIT
#define ATTEN_MODE ADC_ATTEN_DB_12 // Max range
#define MAX_RAW_VALUE 4095

//PID Parameter
#define P_PROPORTIAL 70
#define P_INTERGAL 3
#define P_DERIVATIVE 2

#define SAMPLE_TIME 0.001 //1ms

//Variable
static int sg_raw_adc_value = 0;
static int sg_raw_adc_prev = 0;
// static int sg_raw_conv = 0; //=> for input is angle user input

//PID parameter
int inte = 0;
int deri = 0;

int err_prev = 0;

/* ================ START FUNCTION ================*/
/* FUNCTION PID CONTROL */
void pid_tune_raw_value(int *val_cur, int *val_prev, int *_err_prev)
{
    int err = *val_cur - *val_prev;
    *val_prev = *val_cur;

    inte += err * SAMPLE_TIME;
    deri = (err - *_err_prev)/SAMPLE_TIME;

    *_err_prev = err;
    *val_cur += P_PROPORTIAL* err + P_INTERGAL* inte + P_DERIVATIVE * deri;
}

/* FUNCTION MOTOR SERVO */
static inline uint32_t scale_angle_to_ticks(int angle)
{
    return ((angle-MIN_ANGLE_DEGREE) * (MAX_PULSEWIDTH_US-MIN_PULSEWIDTH_US) / (MAX_ANGLE_DEGREE-MIN_ANGLE_DEGREE)) + MIN_PULSEWIDTH_US;
}
static inline uint32_t scale_rawADC_to_ticks(int rawADC)
{
    return ((rawADC-MIN_RAW_ADC_DEGREE) * (MAX_PULSEWIDTH_US-MIN_PULSEWIDTH_US) / (MAX_RAW_ADC_DEGREE-MIN_RAW_ADC_DEGREE)) + MIN_PULSEWIDTH_US;
}
/* FUNCTION ADC */
static bool init_calib_oneshot_adc(adc_oneshot_chan_cfg_t *i_config, adc_unit_t i_unit, adc_channel_t i_channel, adc_cali_handle_t *rtn_handle)
{
    ESP_LOGI(TAG, "ESP jump to calib function");
    adc_cali_handle_t handle = NULL;
    esp_err_t ret_status = ESP_FAIL;
    int check = 0;

    if(!check)
    {
        adc_cali_line_fitting_config_t config = {
            .unit_id = i_unit,
            .atten = i_config->atten,
            .bitwidth = i_config->bitwidth
        };
        ret_status = adc_cali_create_scheme_line_fitting(&config, &handle);

        if(ret_status == ESP_OK) check = 0;
        else check = -1;
    }

    *rtn_handle = handle;
    return check;
}
static bool deinit_calib_oneshot_adc(adc_cali_handle_t init_handle)
{
    if(adc_cali_delete_scheme_line_fitting(init_handle) != ESP_OK) return -1;
    return 0;
}

//PID FUNCTION

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
    };
    mcpwm_new_operator(&oper_config, &oper_handle);
    mcpwm_operator_connect_timer(oper_handle, timer_handle);

    mcpwm_cmpr_handle_t cmp_handle =NULL;
    mcpwm_comparator_config_t compare_cfg = {
        .flags.update_cmp_on_tez = 1
    };
    mcpwm_new_comparator(oper_handle, &compare_cfg, &cmp_handle);

    mcpwm_gen_handle_t gen_handle = NULL;
    mcpwm_generator_config_t gene_config = {
        .gen_gpio_num = MOTOR_GPIO_18,
    };
    mcpwm_new_generator(oper_handle, &gene_config, &gen_handle);

    //Set init value for the first time of servo motor
    mcpwm_comparator_set_compare_value(cmp_handle, scale_angle_to_ticks(0));

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

    adc_oneshot_unit_handle_t adc_handle_1 = NULL;
    adc_oneshot_unit_init_cfg_t init_adc_1 = {
        .unit_id = ADC_UNIT_1
    };

    adc_oneshot_chan_cfg_t channel_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT, //12bits
        .atten = ATTEN_MODE
    };

    adc_oneshot_new_unit(&init_adc_1, &adc_handle_1);
    // Set pin using channel config
    adc_oneshot_config_channel(adc_handle_1, ADC_CHANNEL_6, &channel_cfg); //pin 34

    adc_cali_handle_t handle_channel_6;
    bool channel_6 = init_calib_oneshot_adc(&channel_cfg, init_adc_1.unit_id, ADC_CHANNEL_6, &handle_channel_6);

    while (1)
    {

        if(!channel_6)
        {
            adc_oneshot_read(adc_handle_1, ADC_CHANNEL_6, &sg_raw_adc_value);
            ESP_LOGI(TAG, "Raw of channel Channel 6: %d", sg_raw_adc_value);
        }

        pid_tune_raw_value(&sg_raw_adc_value, &sg_raw_adc_prev, &err_prev);
        ESP_LOGI(TAG, "Raw value is: %d", sg_raw_adc_value);
        mcpwm_comparator_set_compare_value(cmp_handle,  scale_rawADC_to_ticks(sg_raw_adc_value));

/***
        angle = (sg_raw_adc_value * MAX_ANGLE_DEGREE) / MAX_RAW_VALUE;

        ESP_LOGI(TAG, "Angle of rotation: %d", angle);
        int check = scale_angle_to_ticks(angle);
        ESP_LOGI(TAG, "TICKs %d", check);
        mcpwm_comparator_set_compare_value(cmp_handle, scale_angle_to_ticks(angle));
***/
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    deinit_calib_oneshot_adc(handle_channel_6);
}