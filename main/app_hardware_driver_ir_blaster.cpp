#include "app_hardware_driver.h"
#include "app_zigbee_clusters.h"
#include "app_zigbee_misc.h"
#include "app_nvs_store_info.h"
#ifdef USE_IR_UART_WS4_HW
#include "zigbee_2_uart.h"
#endif
#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)

    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "freertos/portmacro.h"
    #include "driver/gpio.h"
    #include "app_zigbee_misc.h"
    #ifndef USE_IR_UART_WS4_HW   
        #include <Arduino.h>
        #include <IRremoteESP8266.h>
        #include <IRac.h>
        #include <IRutils.h>   

        const uint16_t kIrLed                       = (uint16_t)gpio_ir_led_pins[0];

        IRac                                        ac(kIrLed);   
    #else
        #include <stdio.h>
        #include <stdlib.h>
        #include "cJSON.h"
    #endif


    static const char *TAG                      = "ESP_ZB_IR_BLASTER";

    bool  init_ok_ir                            = false;
    volatile bool send_ac_flag                  = false;
    TaskHandle_t ir_task_handle                 = NULL;

    //Other functions
    #ifndef USE_IR_UART_WS4_HW   
    extern "C" stdAc::opmode_t convert_zb_mode_to_esp_ac(uint8_t mode){
        switch(mode){
            case ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF:
            return stdAc::opmode_t::kOff;
            case ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_AUTO:
            return stdAc::opmode_t::kAuto;
            case ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_COOL:
            return stdAc::opmode_t::kCool;
            case ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_HEAT:
            return stdAc::opmode_t::kHeat;
            case ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_FAN_ONLY:
            return stdAc::opmode_t::kFan;
            case ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_DRY:
            return stdAc::opmode_t::kDry;                                                            
        }
        return stdAc::opmode_t::kOff;
    }

    extern "C" stdAc::fanspeed_t convert_zb_fan_speed_to_esp_ac(uint8_t mode){
        switch(mode){
            case ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_OFF:
            return stdAc::fanspeed_t::kMin;
            case ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_LOW:
            return stdAc::fanspeed_t::kLow;
            case ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_MEDIUM:
            return stdAc::fanspeed_t::kMedium;
            case ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_HIGH:
            return stdAc::fanspeed_t::kHigh;
            case ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_ON:
            return stdAc::fanspeed_t::kMax;
            case ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_AUTO:
            return stdAc::fanspeed_t::kAuto;                                                            
        }
        return stdAc::fanspeed_t::kMin;
    }

    extern "C" decode_type_t get_decode_type(int8_t decode_type){
        if (decode_type >= -1 && decode_type <= 126) {
            return static_cast<decode_type_t>(decode_type);
        }
        return UNKNOWN; // Return UNKNOWN for any invalid input
    }

     
    static void ir_tx_task(void* args){
        while(1){
            vTaskDelay(pdMS_TO_TICKS(10));
            if(init_ok_ir){
                if(send_ac_flag){
                    send_ac_flag = false;
                    esp_zb_lock_acquire(portMAX_DELAY);
                    ac.sendAc();
                    esp_zb_lock_release();
                }
            }
        }
    }
    #endif
    extern "C" void nuos_init_ir_ac(uint8_t index) {
        // Set up what we want to send.
        init_ok_ir = true;
        #ifndef USE_IR_UART_WS4_HW
        ac.next.protocol = get_decode_type(device_info[0].ac_decode_type);  // Set a protocol to use. //
        ac.next.model = 1;  // Some A/Cs have different models. Try just the first.
        ac.next.mode = stdAc::opmode_t::kCool;  // Run in cool mode initially.
        ac.next.celsius = true;  // Use Celsius for temp units. False = Fahrenheit
        ac.next.degrees = device_info[0].ac_temperature;  // 25 degrees.
        ac.next.fanspeed = stdAc::fanspeed_t::kMedium;  // Start the fan at low.
        ac.next.swingv = stdAc::swingv_t::kOff;  // Don't swing the fan up or down.
        ac.next.swingh = stdAc::swingh_t::kOff;  // Don't swing the fan left or right.
        ac.next.light = false;  // Turn off any LED/Lights/Display that we can.
        ac.next.beep = false;  // Turn off any beep from the A/C if we can.
        ac.next.econo = false;  // Turn off any economy modes if we can.
        ac.next.filter = false;  // Turn off any Ion/Mold/Health filters if we can.
        ac.next.turbo = false;  // Don't use any turbo/powerful/etc modes.
        ac.next.quiet = false;  // Don't use any quiet/silent/etc modes.
        ac.next.sleep = -1;  // Don't set any sleep time or modes.
        ac.next.clean = false;  // Turn off any Cleaning options if we can.
        ac.next.clock = -1;  // Don't set any current time if we can avoid it.
        ac.next.power = device_info[0].device_state;  // Initially start with the unit off.        
        xTaskCreate(ir_tx_task, "ir_tx", 8192, NULL, 12, &ir_task_handle);
        #endif
    }

    void send_ac(bool state){
        // 1. Create JSON
        #ifdef USE_IR_UART_WS4_HW
            cJSON *root = cJSON_CreateObject();   // Create object
            cJSON_AddNumberToObject(root, "power", state);  // Add key: power=1
            cJSON_AddNumberToObject(root, "temp", device_info[0].ac_temperature);  // Add key: temp=37
            // Convert to JSON string
            char *json_str = cJSON_PrintUnformatted(root);
            printf("Generated JSON: %s\n", json_str);
            send_serial(json_str);
            // Free the original JSON object
            cJSON_Delete(root); 
            free(json_str); 
        #else
            send_ac_flag = true;
        #endif
    }

    extern "C" void nuos_zb_init_hardware(){
        gpio_reset_pin(gpio_ir_led_pins[0]);
        /* Set the GPIO as a push/pull output */
        gpio_set_direction(gpio_ir_led_pins[0], GPIO_MODE_OUTPUT);
        gpio_set_drive_capability(gpio_ir_led_pins[0], GPIO_DRIVE_CAP_3);    
    }

    static bool state = false;
    void nuos_zb_set_hardware_led_for_zb_commissioning(uint8_t is_toggle){
        #ifdef USE_RGB_LED
            nuos_toggle_rgb_led();
        #else        
            if(is_toggle>0) {
                state = !state;
                for(int index=0; index<TOTAL_LEDS; index++){
                    gpio_set_level(gpio_ir_led_pins[index], state);
                }
            }else{
                for(int i=0; i<TOTAL_ENDPOINTS; i++){
                    nuos_zb_set_hardware(i, false);
                }
            }
        #endif    
    }

    extern "C" void set_decode_type(int decode_type){
        device_info[0].ac_decode_type = decode_type;
        #ifndef USE_IR_UART_WS4_HW  
        ac.next.protocol = (decode_type_t)decode_type;
        #endif
        send_ac(device_info[0].device_state);
        nuos_store_data_to_nvs(0);
    }

    extern "C" void nuos_set_hardware_fan_ctrl(uint8_t index){
        #ifndef USE_IR_UART_WS4_HW  
        ac.next.fanspeed = convert_zb_fan_speed_to_esp_ac(device_info[0].fan_speed);
        #endif
        send_ac(device_info[0].device_state);
        nuos_store_data_to_nvs(0);
    }

    extern "C" void nuos_zb_set_hardware(uint8_t index, uint8_t is_toggle){
        if(is_toggle>0) {
            device_info[0].device_state = !device_info[0].device_state;
        }
        // printf("device_state:%d\n", device_info[0].device_state);
        device_info[0].ac_mode = device_info[0].device_state ? ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_COOL : ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF;
        printf("ac_mode:%d\n", device_info[0].ac_mode);
        #ifndef USE_IR_UART_WS4_HW  
            ac.next.protocol = get_decode_type(device_info[0].ac_decode_type);
            ac.next.mode = stdAc::opmode_t::kCool;
            ac.next.fanspeed = stdAc::fanspeed_t::kMedium;
            ac.next.power = device_info[0].device_state;
            ac.next.degrees = device_info[0].ac_temperature;
        #endif
        send_ac(device_info[0].device_state);
        nuos_store_data_to_nvs(0);
    }

    extern "C" void nuos_try_ac(int decode_type, bool state) {
        #ifndef USE_IR_UART_WS4_HW 
        if(!state){
            ac.next.power = false;
        }else{
            ac.next.power = true;
        }
        #endif
        send_ac(state);
    }

    extern "C" void nuos_set_state_touch_leds(bool state) {

    }

    void nuos_toggle_leds(uint8_t index){
        state = !state;
        #ifndef USE_IR_UART_WS4_HW 
        ac.next.power = state;
        #endif
        send_ac(state);   
    }

#endif