/*
* SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
*
* SPDX-License-Identifier: Unlicense OR CC0-1.0
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "motor-step-encoder.h"

///////////////////////////////Change the following configurations according to your board//////////////////////////////
#define STEP_MOTOR_GPIO_EN       0
#define STEP_MOTOR_GPIO_DIR      2
#define STEP_MOTOR_GPIO_STEP     4
#define STEP_MOTOR_ENABLE_LEVEL  0 // DRV8825 is enabled on low level
#define STEP_MOTOR_SPIN_DIR_CLOCKWISE 0
#define STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE !STEP_MOTOR_SPIN_DIR_CLOCKWISE

#define STEP_MOTOR_RESOLUTION_HZ 1000000 // 1MHz resolution

static const char *TAG = "example";

void app_main(void)
{
    //khởi tạo gpio
    ESP_LOGI(TAG, "Initialize EN + DIR GPIO");
    gpio_config_t en_dir_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = 1ULL << STEP_MOTOR_GPIO_DIR | 1ULL << STEP_MOTOR_GPIO_EN,
    };
    ESP_ERROR_CHECK(gpio_config(&en_dir_gpio_config));

    /*
        This is remote control function, được dùng truyền data in esp từ remote
        có thể chứa các info là cycles, state, byte/bits.
        Mục đích là để notify dữ liệu sẽ được truyền đã hoàn thành
    */
    ESP_LOGI(TAG, "Create RMT TX channel"); //rmx -> remote
    rmt_channel_handle_t motor_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        /*
            Select clock source (depend on CPU clk src)
            APB_CLOCK = CPU_SRC/devider
        */
        .gpio_num = STEP_MOTOR_GPIO_STEP,
        .mem_block_symbols = 64,
        .resolution_hz = STEP_MOTOR_RESOLUTION_HZ,
        .trans_queue_depth = 10, 
        /* set the number of transactions that can be pending in the background */
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &motor_chan));
    /*  Tạo một process channel để control motor_chan   */

    ESP_LOGI(TAG, "Set spin direction");
    gpio_set_level(STEP_MOTOR_GPIO_DIR, STEP_MOTOR_SPIN_DIR_CLOCKWISE);
    ESP_LOGI(TAG, "Enable step motor");
    gpio_set_level(STEP_MOTOR_GPIO_EN, STEP_MOTOR_ENABLE_LEVEL);

    /* Thời gian cài đặt hệ thống được tính theo công thức
        Hz = (end_hz - start_hz)/samples

        Sẽ gồm hai quá trình là acceleration và deceleration
    */
    ESP_LOGI(TAG, "Create motor encoders");
    //quá trình tăng tốc
    stepper_motor_curve_encoder_config_t accel_encoder_config = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
        .sample_points = 500,
        .start_freq_hz = 500,
        .end_freq_hz = 1500,
    };
    rmt_encoder_handle_t accel_motor_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&accel_encoder_config, &accel_motor_encoder));

    //quá trình giảm tốc
    stepper_motor_curve_encoder_config_t decel_encoder_config = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
        .sample_points = 500,
        .start_freq_hz = 1500,
        .end_freq_hz = 500,
    };
    rmt_encoder_handle_t decel_motor_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&decel_encoder_config, &decel_motor_encoder));

    //hàm này sẽ thực hiện điều khiển tức thì, không curve smooth như hàm trên
    stepper_motor_uniform_encoder_config_t uniform_encoder_config = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
    };  
    rmt_encoder_handle_t uniform_motor_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_stepper_motor_uniform_encoder(&uniform_encoder_config, &uniform_motor_encoder));

    //information
    ESP_LOGI(TAG, "Enable RMT channel");
    ESP_ERROR_CHECK(rmt_enable(motor_chan));

    ESP_LOGI(TAG, "Spin motor for 6000 steps: 500 accel + 5000 uniform + 500 decel");
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };
    const static uint32_t accel_samples = 500;
    const static uint32_t uniform_speed_hz = 1500;
    const static uint32_t decel_samples = 500;

    while (1) {
        // acceleration phase
        tx_config.loop_count = 0;
        ESP_ERROR_CHECK(rmt_transmit(motor_chan, accel_motor_encoder, &accel_samples, sizeof(accel_samples), &tx_config));

        // uniform phase
        tx_config.loop_count = 5000;
        ESP_ERROR_CHECK(rmt_transmit(motor_chan, uniform_motor_encoder, &uniform_speed_hz, sizeof(uniform_speed_hz), &tx_config));

        // deceleration phase
        tx_config.loop_count = 0;
        ESP_ERROR_CHECK(rmt_transmit(motor_chan, decel_motor_encoder, &decel_samples, sizeof(decel_samples), &tx_config));

        // wait all transactions finished
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, -1));

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}