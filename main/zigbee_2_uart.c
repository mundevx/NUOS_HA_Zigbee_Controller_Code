#include "app_hardware_driver.h"

#if(defined(USE_IR_UART_WS4_HW) || defined(USE_C3_ADAPTER_UART_HW))

    #include <string.h>
    #include <stdio.h>
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "esp_system.h"
    #include "esp_log.h"
    #include "esp_zigbee_core.h"
    #include "app_zigbee_clusters.h"

    #include "parson.h"
    #include "driver/uart.h"
    #include "light_driver.h"
    #include "app_zigbee_misc.h"
    #include "freertos/semphr.h"

#define UART_HEARTBEAT_FXN             250
#define UART_HEARTBEAT_ACK_FXN         251
#define UART_HEARTBEAT_FAULT_FXN       252
#define UART_HEARTBEAT_PERIOD_OK_MS    60000
#define UART_HEARTBEAT_PERIOD_FAULT_MS 2000
#define UART_HEARTBEAT_TIMEOUT_MS      75000


    #define UART_PORT_NUM                                           UART_NUM_0

    #define UART_TX_PIN                                             TOUCH_5_PIN
    #define UART_RX_PIN                                             TOUCH_6_PIN

    #define UART_BAUD_RATE                                          115200
    #define UART_BUF_SIZE                                           2048
    #define RX_ACC_BUF_SIZE                                         4096

    static bool toggle_status_led_long_press = false;
    static SemaphoreHandle_t uart_tx_mutex = NULL;
    volatile TickType_t c6_last_uart_heartbeat = 0;
    volatile bool c6_uart_link_fault = true;
    extern void parse_json(const char *json_string);
    extern char * parse_query(const char *query);

    void json_parse_data_light(const char* data) {
        //#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI)
            #ifdef USE_C3_ADAPTER_UART_HW
                wifi_webserver_active_counts = 0;
                parse_json(data);
            #endif
        //#endif
    }

    static void esp_zb_callback2(uint8_t param) {
        printf("param:%d\n", param);
    }

    bool is_valid_json(const uint8_t *data, size_t len)
    {
        if (data == NULL || len == 0) {
            return false;
        }

        // Ensure null-terminated string
        char *json_str = (char *)malloc(len + 1);
        if (!json_str) return false;

        memcpy(json_str, data, len);
        json_str[len] = '\0';

        // Parse JSON
        cJSON *root = cJSON_Parse(json_str);

        free(json_str);

        if (root == NULL) {
            return false;  // Invalid JSON
        }

        cJSON_Delete(root);
        return true;  // Valid JSON
    }

    static void uart_send_line_locked(const char *line)
    {
        if (!line) return;
        if (uart_tx_mutex) xSemaphoreTake(uart_tx_mutex, portMAX_DELAY);
        int ret = printf("%s\n", line);
        if (uart_tx_mutex) xSemaphoreGive(uart_tx_mutex);
        if (ret < 0) {
            c6_uart_link_fault = true;
            nuos_set_uart_link_fault(true);
        }
    }

    static void uart_heartbeat_task(void *arg)
    {
        (void)arg;
        uint32_t seq = 0;
        c6_last_uart_heartbeat = xTaskGetTickCount();
        while (1) {
            char hb[128];
            snprintf(hb, sizeof(hb),
                     "{\"fxn\":%d,\"type\":\"uart_heartbeat\",\"side\":\"C6\",\"seq\":%lu}",
                     UART_HEARTBEAT_FXN, (unsigned long)++seq);
            uart_send_line_locked(hb);
            TickType_t now = xTaskGetTickCount();
            if ((now - c6_last_uart_heartbeat) > pdMS_TO_TICKS(UART_HEARTBEAT_TIMEOUT_MS)) {
                if (!c6_uart_link_fault) ESP_LOGE("C6_UART", "UART link timeout: no heartbeat from C3");
                c6_uart_link_fault = true;
                nuos_set_uart_link_fault(true);
            }
            /*
             * Normal operation: heartbeat once every 60 seconds.
             * UART fault: poll rapidly every 2 seconds until communication recovers.
             */
            uint32_t heartbeat_period_ms = c6_uart_link_fault
                                          ? UART_HEARTBEAT_PERIOD_FAULT_MS
                                          : UART_HEARTBEAT_PERIOD_OK_MS;
            vTaskDelay(pdMS_TO_TICKS(heartbeat_period_ms));
        }
    }

    void uart_receiver_task(void *arg)
    {
        (void)arg;
        uint8_t data[UART_BUF_SIZE];
        static char rx_buffer[RX_ACC_BUF_SIZE];
        static int rx_index = 0;
        while (1) {
            int len = uart_read_bytes(UART_PORT_NUM, data, sizeof(data), pdMS_TO_TICKS(20));
            if (len > 0) {
                for (int i = 0; i < len; i++) {
                    char c = (char)data[i];
                    if (rx_index < RX_ACC_BUF_SIZE - 1) rx_buffer[rx_index++] = c;
                    else { ESP_LOGW("C6_UART", "RX frame overflow; dropping partial frame"); rx_index = 0; }
                    if (c == '\n') {
                        rx_buffer[rx_index] = '\0';
                        if (is_valid_json((uint8_t*)rx_buffer, rx_index)) json_parse_data_light(rx_buffer);
                        else ESP_LOGW("C6_UART", "Invalid UART JSON frame");
                        rx_index = 0;
                    }
                }
            }
        }
    }

    void uart_init()
    {
        const uart_config_t uart_config = {
            .baud_rate = UART_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_XTAL,
        };
        uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0);
        uart_param_config(UART_PORT_NUM, &uart_config);
        /* Keep existing UART0 pin/console routing unchanged. */
        uart_tx_mutex = xSemaphoreCreateMutex();
        xTaskCreate(uart_receiver_task, "uart_receiver_task", 8192, NULL, TASK_PRIORITY_UART_RECEIVER, NULL);
        //xTaskCreate(uart_heartbeat_task, "uart_heartbeat_task", 3072, NULL, TASK_PRIORITY_UART_RECEIVER - 1, NULL);
        ESP_LOGI("C6_UART", "UART initialized with heartbeat monitor");
    }

    void send_serial(const char* data){ uart_send_line_locked(data); }
    
#endif
