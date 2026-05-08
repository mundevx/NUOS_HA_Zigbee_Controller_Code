#include "app_hardware_driver.h"

#if(defined(USE_IR_UART_WS4_HW) || defined(USE_C3_ADAPTER_UART_HW))

    #include <string.h>
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "esp_system.h"
    #include "esp_log.h"
    #include "esp_zigbee_core.h"
    #include "app_zigbee_clusters.h"

    #include "parson.h"
    #include "driver/uart.h"
    #include "light_driver.h"


    #define UART_PORT_NUM                                           UART_NUM_0
    #ifdef USE_C3_ADAPTER_UART_HW
    #define UART_TX_PIN                                             TOUCH_5_PIN
    #define UART_RX_PIN                                             TOUCH_6_PIN
    #else
    #define UART_TX_PIN                                             TOUCH_5_PIN
    #define UART_RX_PIN                                             TOUCH_6_PIN
    #endif
    #define UART_BAUD_RATE                                          115200
    #define UART_BUF_SIZE                                           2048

    static bool toggle_status_led_long_press = false;
    extern void parse_json(const char *json_string);

    void json_parse_data_light(const char* data) {
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
            JSON_Value *parsed = json_parse_string(data);
            JSON_Object *root_object = json_value_get_object(parsed);
            if (root_object != NULL) {
                // Extract "power"
                JSON_Value *powerItem = json_object_get_value(root_object, "power");
                if (powerItem != NULL && json_value_get_type(powerItem) == JSONNumber) {
                    int power = (int)json_value_get_number(powerItem);
                    printf("Power = %d\n", power);
                    device_info[0].device_state = power > 0 ? true : false;
                }

                // Extract "temp"
                JSON_Value *tempItem = json_object_get_value(root_object, "temp");
                if (tempItem != NULL && json_value_get_type(tempItem) == JSONNumber) {
                    int temp = (int)json_value_get_number(tempItem);
                    printf("Temp = %d\n", temp);
                    device_info[0].ac_temperature = temp;

                    for (int i = 0; i < 17; i++) {
                        if (ac_temp_values[i] == device_info[0].ac_temperature) {
                            device_info[0].device_level = i;
                            break;
                        }
                    }
                }


                #ifdef USE_FAN_SPEED
                    JSON_Value *fanItem = json_object_get_value(root_object, "fan");
                    if (fanItem != NULL && json_value_get_type(fanItem) == JSONNumber) {
                        int fspeed = (int)json_value_get_number(fanItem);
                        printf("fspeed = %d\n", fspeed);
                        //device_info[0].fan_speed = fspeed;
                        // for (int i = 0; i < 17; i++) {
                        //     if (ac_temp_values[i] == device_info[0].fan_speed) {
                        //         device_info[0].device_level = i;
                        //         break;
                        //     }
                        // }
                    }
                #endif
                nuos_set_zigbee_attribute(0);
                nuos_store_data_to_nvs(0);

                // Extract "time"
                JSON_Value *timeItem = json_object_get_value(root_object, "time");
                if (timeItem != NULL && json_value_get_type(timeItem) == JSONNumber) {
                    int time = (int)json_value_get_number(timeItem);
                    if (time >= 20) {
                        #ifdef USE_RGB_LED
                            light_driver_set_power(false);
                        #endif 
                        setNVSCommissioningFlag(true);
                        setNVSWebServerEnableFlag(false);
                        setNVSPanicAttack(0);
                        esp_zb_factory_reset();
                    }
                    #ifdef USE_RGB_LED
                    else{
                        if (time <= 10) {
                            light_driver_set_color_RGB(0, 0xff, 0x00);  //green
                            light_driver_set_power(true);
                            vTaskDelay(pdMS_TO_TICKS(400)); 
                            light_driver_set_power(false);
                                            
                        }else{
                            light_driver_set_color_RGB(0, 0x00, 0xff);  //blue
                            light_driver_set_power(true);
                            vTaskDelay(pdMS_TO_TICKS(400)); 
                            light_driver_set_power(false);
                        }
                    }
                    #endif 
                } 
                // Cleanup
                json_value_free(parsed); 
            }                          
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER)
            JSON_Value *parsed = json_parse_string(data);
            JSON_Object *root_object = json_value_get_object(parsed); 
            if (root_object != NULL) {       
                // Extract "power"
                JSON_Value *powerItem = json_object_get_value(root_object, "power");
                if (powerItem != NULL && json_value_get_type(powerItem) == JSONNumber) {
                    int power = (int)json_value_get_number(powerItem);
                    printf("Power = %d\n", power);
                    device_info[0].device_state = power > 0 ? true : false;
                    device_info[1].device_state = device_info[0].device_state;
                    if(is_my_device_commissionned){ 
                        nuos_set_thermostat_attribute(1);
                    }
                    
                }

                // Extract "temp"
                JSON_Value *tempItem = json_object_get_value(root_object, "temp");
                if (tempItem != NULL && json_value_get_type(tempItem) == JSONNumber) {
                    int temp = (int)json_value_get_number(tempItem);
                    printf("Temp = %d\n", temp);
                    device_info[0].ac_temperature = temp;
                    device_info[1].ac_temperature = temp;
                    for (int i = 0; i < 17; i++) {
                        if (ac_temp_values[i] == device_info[0].ac_temperature) {
                            device_info[0].device_level = i;
                            device_info[1].device_level = i;
                            break;
                        }
                    }
                    if(is_my_device_commissionned){ 
                        nuos_set_thermostat_temp_attribute(0);
                    }
                }

                #ifdef USE_FAN_SPEED
                    JSON_Value *fanItem = json_object_get_value(root_object, "fan");
                    if (fanItem != NULL && json_value_get_type(fanItem) == JSONNumber) {
                        int fspeed = (int)json_value_get_number(fanItem);
                        printf("fspeed = %d\n", fspeed);
                        //device_info[0].fan_speed = fspeed;
                        // for (int i = 0; i < 17; i++) {
                        //     if (ac_temp_values[i] == device_info[0].fan_speed) {
                        //         device_info[0].device_level = i;
                        //         break;
                        //     }
                        // }
                    }
                #endif            
                nuos_store_data_to_nvs(0);
                nuos_store_data_to_nvs(1);

                // Extract "time"
                JSON_Value *timeItem = json_object_get_value(root_object, "time");
                if (timeItem != NULL && json_value_get_type(timeItem) == JSONNumber) {
                    int time = (int)json_value_get_number(timeItem);
                    if (time >= 20) {
                        #ifdef USE_RGB_LED
                            light_driver_set_power(false);
                        #endif 
                        setNVSCommissioningFlag(true);
                        setNVSWebServerEnableFlag(false);
                        setNVSPanicAttack(0);
                        esp_zb_factory_reset();
                    }
                    #ifdef USE_RGB_LED
                    else{
                        if (time <= 10) {
                            light_driver_set_color_RGB(0, 0xff, 0x00);  //green
                            light_driver_set_power(true);
                            vTaskDelay(pdMS_TO_TICKS(400)); 
                            light_driver_set_power(false);
                                            
                        }else{
                            light_driver_set_color_RGB(0, 0x00, 0xff);  //blue
                            light_driver_set_power(true);
                            vTaskDelay(pdMS_TO_TICKS(400)); 
                            light_driver_set_power(false);
                        }
                    }
                    #endif 
                } 
                // Cleanup
                json_value_free(parsed); 
            }          
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI) 
            #ifdef USE_C3_ADAPTER_UART_HW
                parse_json(data);
            #endif    
        #endif
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
    // void uart_receiver_task(void *arg)
    // {
    //     uint8_t data[UART_BUF_SIZE];
    //     bool scene_set_flag = false;

    //     while (1) {
    //         int len = uart_read_bytes(UART_PORT_NUM, data, sizeof(data), 10 / portTICK_PERIOD_MS);
    //         if (len > 0 && len < UART_BUF_SIZE) {
    //             data[len] = '\0'; // Null-terminate safely
    //             const char* m_data = (const char*)data;
    //             printf("As string: %s len:%d\n", m_data, len);
    //             if(m_data != NULL){
    //                 json_parse_data_light(m_data);
    //             }
    //         }
    //         //vTaskDelay(10 / portTICK_PERIOD_MS);
    //     }
    // }

    #define RX_ACC_BUF_SIZE 4096

    void uart_receiver_task(void *arg)
    {
        uint8_t data[UART_BUF_SIZE];
        static char rx_buffer[RX_ACC_BUF_SIZE];
        static int rx_index = 0;

        while (1) {
            int len = uart_read_bytes(UART_PORT_NUM, data, sizeof(data), 10 / portTICK_PERIOD_MS);
            
            if (len > 0) {
                printf("Length:%d\n", len); 
                for (int i = 0; i < len; i++) {
                    char c = data[i];
                    data[len] = '\0'; // Null-terminate safely
                    //const char* m_data = (const char*)data;
                    printf("%c", c);
                    // Prevent overflow
                    if (rx_index < RX_ACC_BUF_SIZE - 1) {
                        rx_buffer[rx_index++] = c;
                    }

                    // Detect end of JSON (newline)
                    if (c == '\n') {
                        rx_buffer[rx_index] = '\0';

                        printf("Full JSON: %s\n", rx_buffer);

                        if (is_valid_json((uint8_t*)rx_buffer, rx_index)) {
                            json_parse_data_light(rx_buffer);
                        } else {
                            printf("Invalid JSON received\n");
                        }

                        // Reset buffer
                        rx_index = 0;
                    }
                }
            }
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

        //uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

        xTaskCreate(uart_receiver_task, "uart_receiver_task", 4096, NULL, 21, NULL);
        printf("========>UART initialized\n");
    }

    void send_serial(const char* data){
        printf("%s\n", data);
        //uart_write_bytes(UART_PORT_NUM, data, strlen(data));
        //uart_write_bytes(UART_PORT_NUM, "\n", 1);
    }
    
#endif
