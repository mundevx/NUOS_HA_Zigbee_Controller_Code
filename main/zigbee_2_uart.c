


#include "app_hardware_driver.h"

#ifdef USE_IR_UART_WS4_HW

    #include <string.h>
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "esp_system.h"
    #include "esp_log.h"
    #include "cJSON.h"
    #include "esp_zigbee_core.h"
    #include "driver/uart.h"
    #include "app_zigbee_clusters.h"

    #define UART_PORT_NUM                                           UART_NUM_1
    #define UART_TX_PIN                                             14
    #define UART_RX_PIN                                             13
    #define UART_BAUD_RATE                                          115200
    #define UART_BUF_SIZE                                           1024

    static void esp_zb_callback2(uint8_t param) {
        printf("param:%d\n", param);
    }
    void uart_receiver_task(void *arg)
    {
        uint8_t data[UART_BUF_SIZE];
        bool scene_set_flag = false;

        while (1) {
            int len = uart_read_bytes(UART_PORT_NUM, data, sizeof(data), 100 / portTICK_PERIOD_MS);
            if (len > 0 && len < UART_BUF_SIZE) {
                data[len] = '\0'; // Null-terminate safely
                printf("As string: %s\n", (char*)data);
                cJSON *parsed = cJSON_Parse((char*)data);

                    // Extract "power"
                cJSON *powerItem = cJSON_GetObjectItem(parsed, "power");
                if(powerItem != NULL){
                    if (cJSON_IsNumber(powerItem)) {
                        printf("Power = %d\n", powerItem->valueint);
                        device_info[0].device_state = powerItem->valueint>0?true:false;
                    }
                }
                // Extract "temp"
                cJSON *tempItem = cJSON_GetObjectItem(parsed, "temp");
                if(tempItem != NULL){
                    if (cJSON_IsNumber(tempItem)) {
                        printf("Temp = %d\n", tempItem->valueint);
                        device_info[0].ac_temperature = tempItem->valueint;

                        for(int i=0; i<17; i++){
                            if(ac_temp_values[i] == device_info[0].ac_temperature){
                                device_info[0].device_level = i;
                                break;
                            }
                        }
                    }
                }
                // cJSON *longItem = cJSON_GetObjectItem(parsed, "press");
                // if (cJSON_IsString(longItem)) {
                    cJSON *timeItem = cJSON_GetObjectItem(parsed, "time");
                    if(timeItem != NULL){
                        if (cJSON_IsNumber(timeItem)) {
                            int time = timeItem->valueint;
                            if(time >= 20){
                                setNVSCommissioningFlag(true);
                                setNVSWebServerEnableFlag(false);
                                setNVSPanicAttack(0);
                                // esp_zb_scheduler_alarm_cancel(esp_zb_callback2, 0);
                                // esp_zb_bdb_reset_via_local_action(); 
                                // printf("esp_zb_bdb_reset_via_local_action\n");
                                esp_zb_factory_reset();
                            }
                        }
                    }
                // Cleanup
                cJSON_Delete(parsed); 

                nuos_set_zigbee_attribute(0);
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }


    void uart_init()
    {
        const uart_config_t uart_config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_XTAL,
        };
        uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0);
        uart_param_config(UART_PORT_NUM, &uart_config);
        uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

        xTaskCreate(uart_receiver_task, "uart_receiver_task", 4096, NULL, 21, NULL);
    }

    void send_serial(const char* data){
        uart_write_bytes(UART_PORT_NUM, data, strlen(data));
    }
    
#endif
