/*
* SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
*
* SPDX-License-Identifier: Apache-2.0
*/

#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"

#define EXAMPLE_ADC_UNIT                    ADC_UNIT_1 // using adc1:0 , adc2: 1
#define _EXAMPLE_ADC_UNIT_STR(unit)         #unit
#define EXAMPLE_ADC_UNIT_STR(unit)          _EXAMPLE_ADC_UNIT_STR(unit)
#define EXAMPLE_ADC_CONV_MODE               ADC_CONV_SINGLE_UNIT_1 //can use single or both convert 2 adc1 and adc2
#define EXAMPLE_ADC_ATTEN                   ADC_ATTEN_DB_0
#define EXAMPLE_ADC_BIT_WIDTH               SOC_ADC_DIGI_MAX_BITWIDTH

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define EXAMPLE_ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define EXAMPLE_ADC_GET_CHANNEL(p_data)     ((p_data)->type1.channel)
#define EXAMPLE_ADC_GET_DATA(p_data)        ((p_data)->type1.data)
#else
#define EXAMPLE_ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define EXAMPLE_ADC_GET_CHANNEL(p_data)     ((p_data)->type2.channel)
#define EXAMPLE_ADC_GET_DATA(p_data)        ((p_data)->type2.data)
#endif

#define EXAMPLE_READ_LEN                    256

#if CONFIG_IDF_TARGET_ESP32
static adc_channel_t channel[1] = {ADC_CHANNEL_4};
#else
static adc_channel_t channel[2] = {ADC_CHANNEL_2, ADC_CHANNEL_3};
#endif

static TaskHandle_t s_task_handle;
static const char *TAG = "EXAMPLE";


/* 
    Interrupt, IRAM_ATTR-instruction RAM function use for interrupt service routines
    IRAM_ATTR is a macro used in ESP32 and ESP8266 development (with the ESP-IDF or Arduino core)
    to place functions into IRAM (Instruction RAM), specifically to ensure they are stored in internal RAM 
    instead of flash memory. This is crucial for functions used in interrupt service routines (ISRs).
*/
static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    //Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    return (mustYield == pdTRUE);
}

/*init before use*/
static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = EXAMPLE_READ_LEN, //256
    };
/*
    conv_frame_size = sample_size * num_samples_per_read(bytes)

    Ví dụ:
        Bạn đọc 2 kênh ADC
        Mỗi mẫu là 4 byte (gồm dữ liệu + channel info)
        Muốn đọc 128 mẫu mỗi lần:

    sample_size = 4;
    num_samples_per_read = 128;
    conv_frame_size = 4 * 128 = 512;

*/
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 20 * 1000,//20000 Hz => 50us/cycle
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1, // co cau truc ro rang. de xu lu hon nhung bi lost 4 bits moi data (no use)
    };

/*
    ADC2 xung đột với WiFi: Khi WiFi bật, ADC2 không hoạt động được → nên tránh dùng BOTH mode nếu bạn cần WiFi.
    Single mode khac voi Both mode:
    - chi chuyen doi cac channel trong 1 adc
    - toc do sample cho cac medium rate
*/

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
/*
    pattern la cau hinh can lay mot mau, mieu ta trong lan chuyen doi ADC do
    kenh nao duoc su dung?, do phan gia la bao nhieu bit?, dien ap su dung trong lan chuyen doi la bao nhieu?
*/

    for (int i = 0; i < channel_num; i++) {
        adc_pattern[i].atten = ADC_ATTEN_DB_0; ///0
        adc_pattern[i].channel = channel[i] & 0x7; // can nhac su dung 0xF thay vi 0x7 khi su dung cac channel 0,8,9
        adc_pattern[i].unit = ADC_UNIT_1;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%"PRIx8, i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%"PRIx8, i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%"PRIx8, i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}

//using
void app_main(void)
{
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[EXAMPLE_READ_LEN] = {0}; // EXAMPLE_READ_LEN =256
    memset(result, 0xcc, EXAMPLE_READ_LEN);

    s_task_handle = xTaskGetCurrentTaskHandle();

    adc_continuous_handle_t handle = NULL;
    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle);

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
/*
 *user-data là một dữ liệu người dùng tùy chỉnh được truyền vào hàm khi bạn đăng ký các callback events.
 Cụ thể, user_data là một con trỏ, có thể trỏ đến bất kỳ đối tượng hoặc cấu trúc dữ liệu nào mà bạn muốn
 truyền qua callback để các hàm callback có thể truy cập.

 Current NULL is set as user-data
*/

    ESP_ERROR_CHECK(adc_continuous_start(handle));

    while (1) {

        /**
         * This is to show you the way to use the ADC continuous mode driver event callback.
         * This `ulTaskNotifyTake` will block when the data processing in the task is fast.
         * However in this example, the data processing (print) is slow, so you barely block here.
         *
         * Without using this event callback (to notify this task), you can still just call
         * `adc_continuous_read()` here in a loop, with/without a certain block timeout.
         */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); //optional use or not

        char unit[] = EXAMPLE_ADC_UNIT_STR(EXAMPLE_ADC_UNIT);

        while (1) {
            ret = adc_continuous_read(handle, result, EXAMPLE_READ_LEN, &ret_num, 0);
            if (ret == ESP_OK) {
                ESP_LOGI("TASK", "ret is %x, ret_num is %"PRIu32" bytes", ret, ret_num);
                for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) 
/*
                moi phan tu chiem 2 bytes (16bits) de doc 12 bit resolution, so do tang 2 sau moi lan loop
*/
                {
                    adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
                    uint32_t chan_num = EXAMPLE_ADC_GET_CHANNEL(p);
                    uint32_t data = EXAMPLE_ADC_GET_DATA(p);
                    /* Check the channel number validation, the data is invalid if the channel num exceed the maximum channel */
                    if (chan_num < SOC_ADC_CHANNEL_NUM(EXAMPLE_ADC_UNIT))
                    //channel number la sp luong channel cua adc su dung
                    {
                        ESP_LOGI(TAG, "Unit: %s, Channel: %"PRIu32", Value: %"PRIx32, unit, chan_num, data);
                    } 
                    else
                    {
                        ESP_LOGW(TAG, "Invalid data [%s_%"PRIu32"_%"PRIx32"]", unit, chan_num, data);
                    }
                }
                /**
                 * Because printing is slow, so every time you call `ulTaskNotifyTake`, it will immediately return.
                 * To avoid a task watchdog timeout, add a delay here. When you replace the way you process the data,
                 * usually you don't need this delay (as this task will block for a while).
                 */
                vTaskDelay(1);
            } else if (ret == ESP_ERR_TIMEOUT) {
                //We try to read `EXAMPLE_READ_LEN` until API returns timeout, which means there's no available data
                break;
            }
        }
    }

    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}