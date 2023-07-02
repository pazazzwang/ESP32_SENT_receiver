/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/rmt_rx.h"
#include <string.h>
#include "esp_task_wdt.h"


#define EXAMPLE_IR_RESOLUTION_HZ     40*1000000 // 1MHz resolution, 1 tick = 1us

#define EXAMPLE_IR_RX_GPIO_NUM_CHANNEL_1       12
#define EXAMPLE_IR_RX_GPIO_NUM_CHANNEL_2       14
#define EXAMPLE_IR_RX_GPIO_NUM_CHANNEL_3       27
#define EXAMPLE_IR_RX_GPIO_NUM_CHANNEL_4       26


static const char *TAG = "example";

/**
 * @brief Divide with 4 dump 5 keep
 */

static uint16_t divide_4d_5k(uint16_t devided, uint16_t divider){
    int result = devided / divider;
    if (2 * (devided % divider) >= divider) {
        result += 1;
    }
    return result;
}

// 创建一个输出缓存
static char SENT_signals_buffer[100];

/**
 * @brief Decode RMT symbols and buffer into buffer
 */
static void buffer_SENT_frame(uint16_t channel_num, rmt_symbol_word_t *rmt_SENT_symbols, size_t symbol_num)
{   
    static uint16_t tick = 119;

    // static uint16_t duration_P0 = 0;
    static uint16_t duration_P1 = 0;
    static uint16_t duration_P2 = 0;
    static uint16_t duration_P3 = 0;
    // static uint16_t duration_P4 = 0;
    // static uint16_t duration_P5 = 0;

    // static uint16_t VALUE_P0 = 0;
    static uint16_t VALUE_P1 = 0;
    static uint16_t VALUE_P2 = 0;
    static uint16_t VALUE_P3 = 0;
    // static uint16_t VALUE_P4 = 0;
    // static uint16_t VALUE_P5 = 0;



    
    // duration_P0 = rmt_SENT_symbols[0].duration0 + rmt_SENT_symbols[0].duration1;
    duration_P1 = rmt_SENT_symbols[1].duration0 + rmt_SENT_symbols[1].duration1;
    duration_P2 = rmt_SENT_symbols[2].duration0 + rmt_SENT_symbols[2].duration1;
    duration_P3 = rmt_SENT_symbols[3].duration0 + rmt_SENT_symbols[3].duration1;
    // duration_P4 = rmt_SENT_symbols[4].duration0 + rmt_SENT_symbols[4].duration1;
    // duration_P5 = rmt_SENT_symbols[5].duration0 + rmt_SENT_symbols[5].duration1;
        
    // VALUE_P0 = divide_4d_5k(duration_P0, tick) - 12;
    VALUE_P1 = divide_4d_5k(duration_P1, tick) - 12;
    VALUE_P2 = divide_4d_5k(duration_P2, tick) - 12;
    VALUE_P3 = divide_4d_5k(duration_P3, tick) - 12;
    // VALUE_P4 = divide_4d_5k(duration_P4, tick) - 12;
    // VALUE_P5 = divide_4d_5k(duration_P5, tick) - 12;

    char temp[50];
    // sprintf(temp, "%X%X%X%X%X%X%X ", channel_num, VALUE_P0, VALUE_P1, VALUE_P2, VALUE_P3, VALUE_P4, VALUE_P5);
    sprintf(temp, "%X%X%X%X ", channel_num, VALUE_P1, VALUE_P2, VALUE_P3);
    // sprintf(temp, "%X%X ", VALUE_P4, VALUE_P5);
    strcat(SENT_signals_buffer, temp);
}

/**
 * @brief Decode RMT symbols into SENT scan code and print the result
 */
static void example_parse_SENT_frame(uint16_t channel_num, rmt_symbol_word_t *rmt_SENT_symbols, size_t symbol_num)
{   
    static uint16_t tick = 119;

    static uint16_t duration_P0 = 0;
    static uint16_t duration_P1 = 0;
    static uint16_t duration_P2 = 0;
    static uint16_t duration_P3 = 0;
    static uint16_t duration_P4 = 0;
    static uint16_t duration_P5 = 0;

    static uint16_t VALUE_P0 = 0;
    static uint16_t VALUE_P1 = 0;
    static uint16_t VALUE_P2 = 0;
    static uint16_t VALUE_P3 = 0;
    static uint16_t VALUE_P4 = 0;
    static uint16_t VALUE_P5 = 0;

    // printf("SENT frame start---\r\n");
    // printf("%d",symbol_num);

    if(symbol_num == 9){
        duration_P0 = rmt_SENT_symbols[0].duration0 + rmt_SENT_symbols[0].duration1;
        duration_P1 = rmt_SENT_symbols[1].duration0 + rmt_SENT_symbols[1].duration1;
        duration_P2 = rmt_SENT_symbols[2].duration0 + rmt_SENT_symbols[2].duration1;
        duration_P3 = rmt_SENT_symbols[3].duration0 + rmt_SENT_symbols[3].duration1;
        duration_P4 = rmt_SENT_symbols[4].duration0 + rmt_SENT_symbols[4].duration1;
        duration_P5 = rmt_SENT_symbols[5].duration0 + rmt_SENT_symbols[5].duration1;
        
        VALUE_P0 = divide_4d_5k(duration_P0, tick) - 12;
        VALUE_P1 = divide_4d_5k(duration_P1, tick) - 12;
        VALUE_P2 = divide_4d_5k(duration_P2, tick) - 12;
        VALUE_P3 = divide_4d_5k(duration_P3, tick) - 12;
        VALUE_P4 = divide_4d_5k(duration_P4, tick) - 12;
        VALUE_P5 = divide_4d_5k(duration_P5, tick) - 12;



        // printf("%X%X%X%X%X%X ", VALUE_P0, VALUE_P1, VALUE_P2, VALUE_P3,  VALUE_P4, VALUE_P5);
        printf("%X%X%X%X%X%X%X ", channel_num, VALUE_P0, VALUE_P1, VALUE_P2, VALUE_P3,  VALUE_P4, VALUE_P5);
        // printf("%X%X%X%X ", VALUE_P0, VALUE_P1, VALUE_P2, VALUE_P3);
        // printf("%X%X%X%X ", channel_num, VALUE_P1, VALUE_P2, VALUE_P3);
        // printf("%X ", channel_num);
        // printf("%X%X ", VALUE_P4, VALUE_P5);
    // }
    
    // for (size_t i = 0; i < symbol_num; i++) {
        // printf("{%d:%d},{%d:%d}\r\n", rmt_SENT_symbols[i].level0, rmt_SENT_symbols[i].duration0,
        //        rmt_SENT_symbols[i].level1, rmt_SENT_symbols[i].duration1);
    }
    // printf("---SENT frame end: \n");
    
}

static bool example_rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    // send the received RMT symbols to the parser task
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

void app_main(void)
{
    // 关掉看门狗
    esp_task_wdt_deinit();

    ESP_LOGI(TAG, "create RMT RX channel");
    rmt_rx_channel_config_t rx_channel_1_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = EXAMPLE_IR_RESOLUTION_HZ,
        .mem_block_symbols = 64, // amount of RMT symbols that the channel can store at a time
        .gpio_num = EXAMPLE_IR_RX_GPIO_NUM_CHANNEL_1,
        .flags.with_dma = false,
    };
    rmt_rx_channel_config_t rx_channel_2_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = EXAMPLE_IR_RESOLUTION_HZ,
        .mem_block_symbols = 64, // amount of RMT symbols that the channel can store at a time
        .gpio_num = EXAMPLE_IR_RX_GPIO_NUM_CHANNEL_2,
        .flags.with_dma = false,
    };
        rmt_rx_channel_config_t rx_channel_3_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = EXAMPLE_IR_RESOLUTION_HZ,
        .mem_block_symbols = 64, // amount of RMT symbols that the channel can store at a time
        .gpio_num = EXAMPLE_IR_RX_GPIO_NUM_CHANNEL_3,
        .flags.with_dma = false,
    };
        rmt_rx_channel_config_t rx_channel_4_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = EXAMPLE_IR_RESOLUTION_HZ,
        .mem_block_symbols = 64, // amount of RMT symbols that the channel can store at a time
        .gpio_num = EXAMPLE_IR_RX_GPIO_NUM_CHANNEL_4,
        .flags.with_dma = false,
    };

    rmt_channel_handle_t rx_channel_1 = NULL;
    rmt_channel_handle_t rx_channel_2 = NULL;
    rmt_channel_handle_t rx_channel_3 = NULL;
    rmt_channel_handle_t rx_channel_4 = NULL;

    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_channel_1_cfg, &rx_channel_1));
    // printf("channel_1 created\n");
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_channel_2_cfg, &rx_channel_2));
    // printf("channel_2 created\n");
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_channel_3_cfg, &rx_channel_3));
    // printf("channel_3 created\n");
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_channel_4_cfg, &rx_channel_4));
    // printf("channel_4 created\n");

    ESP_LOGI(TAG, "register RX done callback");
    QueueHandle_t receive_queue_channel_1 = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    assert(receive_queue_channel_1);
    QueueHandle_t receive_queue_channel_2 = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    assert(receive_queue_channel_2);
    QueueHandle_t receive_queue_channel_3 = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    assert(receive_queue_channel_2);
    QueueHandle_t receive_queue_channel_4 = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    assert(receive_queue_channel_2);

    rmt_rx_event_callbacks_t cbs_channel_1 = {
        .on_recv_done = example_rmt_rx_done_callback,
    };
    rmt_rx_event_callbacks_t cbs_channel_2 = {
        .on_recv_done = example_rmt_rx_done_callback,
    };
    rmt_rx_event_callbacks_t cbs_channel_3 = {
        .on_recv_done = example_rmt_rx_done_callback,
    };
    rmt_rx_event_callbacks_t cbs_channel_4 = {
        .on_recv_done = example_rmt_rx_done_callback,
    };

    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel_1, &cbs_channel_1, receive_queue_channel_1));
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel_2, &cbs_channel_2, receive_queue_channel_2));
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel_3, &cbs_channel_3, receive_queue_channel_3));
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel_4, &cbs_channel_4, receive_queue_channel_4));

    // the following timing requirement is based on SENT protocol
    rmt_receive_config_t receive_config = {
        .signal_range_min_ns = 12*1000,     // the shortest duration for SENT signal is 36us, 30us < 36us, valid signal won't be treated as noise
        .signal_range_max_ns = 83*1000, // the longest duration for SENT signal is 81us, 100us > 81us, the receive won't stop early
    };




    ESP_LOGI(TAG, "enable RMT RX channels");
    ESP_ERROR_CHECK(rmt_enable(rx_channel_1));
    ESP_ERROR_CHECK(rmt_enable(rx_channel_2));
    ESP_ERROR_CHECK(rmt_enable(rx_channel_3));
    ESP_ERROR_CHECK(rmt_enable(rx_channel_4));

    // save the received RMT symbols
    rmt_symbol_word_t raw_symbols_channel_1[64]; // 64 symbols should be sufficient for a standard SENT frame
    rmt_symbol_word_t raw_symbols_channel_2[64]; // 64 symbols should be sufficient for a standard SENT frame
    rmt_symbol_word_t raw_symbols_channel_3[64]; // 64 symbols should be sufficient for a standard SENT frame
    rmt_symbol_word_t raw_symbols_channel_4[64]; // 64 symbols should be sufficient for a standard SENT frame
    rmt_rx_done_event_data_t rx_data_channel_1;
    rmt_rx_done_event_data_t rx_data_channel_2;
    rmt_rx_done_event_data_t rx_data_channel_3;
    rmt_rx_done_event_data_t rx_data_channel_4;
    // ready to receive
    ESP_ERROR_CHECK(rmt_receive(rx_channel_1, raw_symbols_channel_1, sizeof(raw_symbols_channel_1), &receive_config));
    // ESP_ERROR_CHECK(rmt_receive(rx_channel_2, raw_symbols_channel_2, sizeof(raw_symbols_channel_2), &receive_config));
    // ESP_ERROR_CHECK(rmt_receive(rx_channel_3, raw_symbols_channel_3, sizeof(raw_symbols_channel_3), &receive_config));
    // ESP_ERROR_CHECK(rmt_receive(rx_channel_4, raw_symbols_channel_4, sizeof(raw_symbols_channel_4), &receive_config));

    while (1)
    {
        /* code */

        while (1) {
            // channel 1 start receive
            // rmt_receive(rx_channel_1, raw_symbols_channel_1, sizeof(raw_symbols_channel_1), &receive_config);
            if (xQueueReceive(receive_queue_channel_1, &rx_data_channel_1, pdMS_TO_TICKS(0)) == pdPASS) {
                
                if(rx_data_channel_1.num_symbols == 9){
                    rmt_receive(rx_channel_2, raw_symbols_channel_2, sizeof(raw_symbols_channel_2), &receive_config);
                    buffer_SENT_frame(1, rx_data_channel_1.received_symbols, rx_data_channel_1.num_symbols);
                    break;
                }
                else{
                    rmt_receive(rx_channel_1, raw_symbols_channel_1, sizeof(raw_symbols_channel_1), &receive_config);
                }
            } 
        }

        while (1) {
            // channel 2 start receive
            // rmt_receive(rx_channel_2, raw_symbols_channel_2, sizeof(raw_symbols_channel_2), &receive_config);
            if (xQueueReceive(receive_queue_channel_2, &rx_data_channel_2, pdMS_TO_TICKS(0)) == pdPASS) {
                
                if(rx_data_channel_2.num_symbols == 9){
                    // rmt_receive(rx_channel_3, raw_symbols_channel_3, sizeof(raw_symbols_channel_3), &receive_config);
                    buffer_SENT_frame(2, rx_data_channel_2.received_symbols, rx_data_channel_2.num_symbols);
                    break;
                } 
                else{
                    rmt_receive(rx_channel_2, raw_symbols_channel_2, sizeof(raw_symbols_channel_2), &receive_config);
                }
            }
        }

        // while (1) {
        //     // channel 3 start receive
        //     // rmt_receive(rx_channel_3, raw_symbols_channel_3, sizeof(raw_symbols_channel_3), &receive_config);
        //     if (xQueueReceive(receive_queue_channel_3, &rx_data_channel_3, pdMS_TO_TICKS(0)) == pdPASS) {
        //         if(rx_data_channel_3.num_symbols == 9){
        //             rmt_receive(rx_channel_4, raw_symbols_channel_4, sizeof(raw_symbols_channel_4), &receive_config);
        //             buffer_SENT_frame(3, rx_data_channel_3.received_symbols, rx_data_channel_3.num_symbols);
        //             break;
        //         } 
        //     }
        //     else{
        //         rmt_receive(rx_channel_3, raw_symbols_channel_3, sizeof(raw_symbols_channel_3), &receive_config);
        //     }
        // }

        // while (1) {
        //     // channel 4 start receive
        //     // rmt_receive(rx_channel_4, raw_symbols_channel_4, sizeof(raw_symbols_channel_4), &receive_config);
        //     if (xQueueReceive(receive_queue_channel_4, &rx_data_channel_4, pdMS_TO_TICKS(0)) == pdPASS) {
        //         if(rx_data_channel_4.num_symbols == 9){
        //             //channel 1 receive start after print
        //             buffer_SENT_frame(4, rx_data_channel_4.received_symbols, rx_data_channel_4.num_symbols);
        //             break;
        //         } 
        //     }
        //     else{
        //         rmt_receive(rx_channel_4, raw_symbols_channel_4, sizeof(raw_symbols_channel_4), &receive_config);
        //     }
        // }


        printf("%s", SENT_signals_buffer);
        memset(SENT_signals_buffer, 0, sizeof(SENT_signals_buffer));
        rmt_receive(rx_channel_1, raw_symbols_channel_1, sizeof(raw_symbols_channel_1), &receive_config);
    }
}