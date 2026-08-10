
#include "app_hardware_driver.h"
#include "app_zigbee_clusters.h"
#include "app_nuos_timer.h"
#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI)

    #include "esp_log.h"
    #include "esp_err.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include <esp_check.h>
    #include <esp_log.h>
    #include "driver/ledc.h"
    #include "DaliCommands.h"
    #include "DALI.h"
    #include "zigbee_2_uart.h" 
    #include "esp_wifi.h"  // For esp_wifi_stop() and esp_wifi_start()
    #include "esp_wifi_station.h"
    // instantiate global object (adjust constructor args as needed)
    // DALI                                                       dali(gpio_load_pins[1], gpio_load_pins[0]);
    DaliCommands                                               dali(gpio_load_pins[1], gpio_load_pins[0]);
    //static DALI                                                 daliCore(gpio_load_pins[1], gpio_load_pins[0]);
    bool is_init_done                                          = false;
    static bool state                                          = false;
    #define IS_USE_DALI_HARDWARE

    extern volatile uint16_t total_press_in_secs;
    extern "C" void nuos_set_state_touch_leds(bool state);
    extern "C" bool nuos_check_state_touch_leds();
    // extern "C" void process_dali_receive_tasks(uint8_t index, bool _state_, uint8_t _level_);
    int start_dali_addressing(uint8_t startAddresses, uint8_t numAddresses);

    #define LEDC_TIMER              		                    LEDC_TIMER_0
    #define LEDC_MODE               		                    LEDC_LOW_SPEED_MODE

    #define LEDC_DUTY_RES          		 	                    LEDC_TIMER_8_BIT // Set duty resolution to 13 bits
    #define LEDC_DUTY               		                    100      // Set duty to 50%. ((2 ** 8) - 1) * 50% = 4095
    #define LEDC_MAX_DUTY           		                    254     //8191
    //#define LEDC_MIN_DUTY           		                    (LEDC_MAX_DUTY*10)/100     //10%

    // #define LEDC_FREQUENCY          	                        1000    // Frequency in Hertz. Set frequency at 1 KHz


    #define LEDC_FREQUENCY                                      (1000) // Frequency in Hz (5 kHz)
    #define LEDC_FADE_TIME                                      (500) // Fade time in milliseconds (1 second)
    #ifdef DALI_DIRECT_ADDRESSING 
    ledc_channel_t pwm_channels[TOTAL_ENDPOINTS]                = { LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    #else
    ledc_channel_t pwm_channels[TOTAL_ENDPOINTS]                = { LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3};

    #endif

    // Add at top with other defines
    #define DALI_TASK_PRIORITY                                  6           // Higher than webserver tasks
    #define DALI_ADDRESSING_TIMEOUT_MS                          30000       // 30 second timeout
    static TaskHandle_t dali_comm_task_handle                   = NULL;
    bool _toggle_                                               = false;
    static uint8_t group_addr                                   = 0xff;

    static QueueHandle_t rxFrameQueue                           = nullptr;  // Queue for received frames (each is uint32_t)
    static bool send_command_flag                               = false;
    static bool _state_                                         = false;
    static uint8_t button_pressed_index                         = 0xff;

    uint8_t map_1_255_to_100_255(uint8_t in)
    {
        return (uint8_t)((in * dali_range_size) / 254 + dali_min_off_offset);
    }

    void set_fade_time_all_devices(uint8_t time, uint8_t rate){
        for(uint8_t index=0; index<TOTAL_ENDPOINTS; index++){
            for(int i=0; i<scene_group_switch_info.total_ids[index]; i++){
                //printf("Set fade time %d rate %d for %d\n", time, rate, scene_group_switch_info.device_ids[index][i]);
                dali.set_fade_time(scene_group_switch_info.device_ids[index][i], time);
                vTaskDelay(10 / portTICK_PERIOD_MS);
                dali.set_fade_rate(scene_group_switch_info.device_ids[index][i], rate);
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
        }
    }

    void nuos_set_dali_fade_time(uint8_t time){
        for(uint8_t index=0; index<TOTAL_ENDPOINTS; index++){
            for(int i=0; i<scene_group_switch_info.total_ids[index]; i++){
                //printf("Set fade time %d for %d\n", time, scene_group_switch_info.device_ids[index][i]);
                dali.set_fade_time(scene_group_switch_info.device_ids[index][i], time);
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
        }
    }
    
    void nuos_set_dali_fade_rate(uint8_t rate){
        for(uint8_t index=0; index<TOTAL_ENDPOINTS; index++){
            for(int i=0; i<scene_group_switch_info.total_ids[index]; i++){
                //printf("Set fade rate %d for %d\n", rate, scene_group_switch_info.device_ids[index][i]);
                dali.set_fade_rate(scene_group_switch_info.device_ids[index][i], rate);
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
        }
    }


    extern "C" void nuos_zb_init_hardware(){
        // Prepare and then apply the LEDC PWM timer configuration
        // Prepare and then apply the LEDC PWM timer configuration
        ledc_timer_config_t ledc_timer = {
            .speed_mode       = LEDC_MODE,
            .duty_resolution  = LEDC_DUTY_RES,
            .timer_num        = LEDC_TIMER,
            .freq_hz          = LEDC_FREQUENCY,
            .clk_cfg          = LEDC_AUTO_CLK
        };
        ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));       
        for(int index=0; index<TOTAL_ENDPOINTS; index++){
            // Prepare and then apply the LEDC PWM channel configuration
            if(index < TOTAL_LEDS){
                ledc_channel_config_t ledc_channel = {
                    .gpio_num       = gpio_touch_led_pins[index],
                    .speed_mode     = LEDC_MODE,
                    .channel        = pwm_channels[index],
                    .intr_type      = LEDC_INTR_DISABLE,
                    .timer_sel      = LEDC_TIMER,
                    .duty           = 0, // Set duty to 0%
                    .hpoint         = 0
                };
                ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel)); 
            }
        }      
        dali.begin(&isr_service_installed); 
        
        is_init_done = true; 
    }



    extern "C" void dali_query_send(uint8_t id, uint8_t command){
        //printf("===Sending DALi Query...\n");
        dali.query(id, command); // Send your query command
    }
    struct DaliMessage {
        uint8_t data[3];
        size_t len;
    };

    TaskHandle_t xQueryingTaskHandle = NULL;
    volatile uint8_t queryResponseValue = 0;

static void receiveDaliFrame(void *arg) {
    DaliMessage msg;
    // Assuming 'dali' instance is accessible here (either global or passed via arg)
    while (1) {
        if (rxFrameQueue != nullptr) {
            if (xQueueReceive(rxFrameQueue, &msg, portMAX_DELAY) == pdTRUE) {
                
                // Check if a query task is currently waiting for this data
                if (xQueryingTaskHandle != NULL) {
                    // Store the incoming data byte directly into the response buffer
                    queryResponseValue = msg.data[0];
                    
                    // Immediately unblock the waiting query task
                    xTaskNotifyGive(xQueryingTaskHandle);
                } else {
                    // Handle asynchronous DALI events/traffic here if needed
                    //printf("Asynchronous DALI data received: 0x%02X\n", msg.data[0]);
                }
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}
//    static void receiveDaliFrame(void *arg) {
//         DaliMessage msg;
//         while (1) {
//             if (rxFrameQueue != nullptr) {
//                 if(xQueueReceive(rxFrameQueue, &msg, portMAX_DELAY)== pdTRUE) {
//                     printf("===Received DALi Frame: 0x%02X 0x%02X\n", msg.data[0], msg.data[1]);  
//                     //interpret_frame(msg.data[0], msg.data[1]);
//                 }
//             }else{
//                 vTaskDelay(10 / portTICK_PERIOD_MS);
//             }
//         }
//     }
    void init_dali_hw() {
        
        //if(wifi_webserver_active_flag > 0){
            #ifdef IS_USE_DALI_HARDWARE
                rxFrameQueue = xQueueCreate(10, sizeof(DaliMessage));
                if (rxFrameQueue == nullptr) {  
                }   
                    
                dali.begin_rx(&isr_service_installed, rxFrameQueue);  
            #endif

            xTaskCreate(receiveDaliFrame, "dali_task_2", TASK_STACK_SIZE_DALI_RX_FRAME, NULL, TASK_PRIORITY_DALI_RX_FRAME, NULL); 

        //}              
        is_init_done = true;  
    }


    // uint8_t nuos_dali_switch_type() {
    //     if(scene_group_switch_info.control_type == 0) {  //individual control
    //         return 1; 
    //     }else if(scene_group_switch_info.control_type == 1) { //group control
    //         return 2; 
    //     }else if(scene_group_switch_info.control_type == 2) { //scene control
    //         return 3; 
    //     }else if(scene_group_switch_info.control_type == 3) { //broadcast control
    //         return 4; 
    //     }
    //     return 0;
    // }     
    
    uint8_t nuos_dali_switch_scene_type() {
        // if(scene_group_switch_info.scn_ctrl_type == 0) {  //broadcast control
        //     return 1; 
        // }else if(scene_group_switch_info.scn_ctrl_type == 1) { //group control
        //     return 2; 
        // }else if(scene_group_switch_info.scn_ctrl_type == 2) { //individual control
        //     return 3; 
        // }
        return 1;
    }  

    static int get_delay(int total_ids, uint8_t index) {
        int delay_time = 1;
        if(total_ids <=4) delay_time = 10;
        else if(total_ids >4 && total_ids <=8) delay_time = 5;
        else if(total_ids >8 && total_ids <=10) delay_time = 2;
        else delay_time = 1;
        return delay_time;
    }
    static void esp_dali_off_task(void* args) {
        uint8_t index = *(uint8_t*)args;
        for(int i=0; i<scene_group_switch_info.total_ids[index]; i++){
            dali.turn_off(scene_group_switch_info.device_ids[index][i]);
            //printf("%d off\n", scene_group_switch_info.device_ids[index][i]);
            vTaskDelay(get_delay(scene_group_switch_info.total_ids[index], index) / portTICK_PERIOD_MS);
        }
        vTaskDelete(NULL); // Delete the task after executing
    }

    static void esp_dali_on_task(void* args) {
        uint8_t index = *(uint8_t*)args;
        for(int i=0; i<scene_group_switch_info.total_ids[index]; i++){
            //printf("%d on at %d\n", scene_group_switch_info.device_ids[index][i], device_info[index].device_level);
            dali.set_dim_value(scene_group_switch_info.device_ids[index][i],
            map_1_255_to_100_255(device_info[index].device_level));
            vTaskDelay(get_delay(scene_group_switch_info.total_ids[index], index) / portTICK_PERIOD_MS);    
        }
        vTaskDelete(NULL); // Delete the task after executing
    }

    static void esp_dali_set_scene_task(void* args) {
        uint8_t index = *(uint8_t*)args;
        for(int i=0; i<scene_group_switch_info.total_ids[index]; i++){
            dali.go_to_scene(scene_group_switch_info.device_ids[index][i], scene_group_switch_info.scene_ids[index]);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }  
        vTaskDelete(NULL); // Delete the task after executing
    }


    extern "C" void process_dali_tasks(uint8_t index, uint8_t is_toggle, uint8_t is_scene){
        send_command_flag = true;
        button_pressed_index = index;
        
        _state_ = (bool)device_info[index].device_state; 
        if(scene_group_switch_info.control_type == 0) { //group control
            if(is_toggle>0) device_info[index].device_state = !device_info[index].device_state;
            if(!device_info[index].device_state) {
                // printf("DALI Group OFF :%d , btn_id:%d\n", index, scene_group_switch_info.scene_ids[index]);
                #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
                    ledc_set_duty(LEDC_MODE, pwm_channels[index], 0);            
                    ledc_update_duty(LEDC_MODE, pwm_channels[index]);
                #else
                    gpio_set_level(gpio_touch_led_pins[index], 0);
                #endif
                dali.set_group_off(scene_group_switch_info.scene_ids[index]);
            } else {
                // printf("DALI Group ON :%d , btn_id:%d\n", index, scene_group_switch_info.scene_ids[index]);
                #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
                    ledc_set_duty(LEDC_MODE, pwm_channels[index], device_info[index].device_level);            
                    ledc_update_duty(LEDC_MODE, pwm_channels[index]);
                #else
                    gpio_set_level(gpio_touch_led_pins[index], 1);
                #endif                    
                nuos_dali_set_group_brightness(scene_group_switch_info.scene_ids[index], index, device_info[index].device_level);  
            }                      
        }else if(scene_group_switch_info.control_type == 1) { //scene control
            scene_group_switch_info.selected_id = index;
            for(int i=0; i<TOTAL_ENDPOINTS; i++) {
                if(i == index) {
                    #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
                        ledc_set_duty(LEDC_MODE, pwm_channels[i], MAX_DIM_LEVEL_VALUE);        
                        ledc_update_duty(LEDC_MODE, pwm_channels[i]);
                    #else
                        gpio_set_level(gpio_touch_led_pins[index], 1);
                    #endif

                    //printf("Recall DALI Scene (Broadcast) :%d\n", scene_group_switch_info.scene_ids[index]);
                    dali.go_to_scene(0xff, scene_group_switch_info.scene_ids[index]);  
                                       
                } else {
                    #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
                        ledc_set_duty(LEDC_MODE, pwm_channels[i], 0);            
                        ledc_update_duty(LEDC_MODE, pwm_channels[i]);
                    #else
                        gpio_set_level(gpio_touch_led_pins[index], 0);
                    #endif
                }
            }   
            nuos_store_dali_scene_switch_data_to_nvs(&scene_group_switch_info);
        } 
        //start_my_task();
        nuos_store_data_to_nvs(index);   
    }

    void set_all_leds_to_original_state(){
        if(is_init_done){
            for(int i=0; i<TOTAL_LEDS; i++){
                #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
                    if(scene_group_switch_info.control_type == 0) {  //individual control
                        if(device_info[i].device_state) ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], device_info[i].device_level));
                        else ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], 0));
                        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[i]));
                    } else {
                        if(device_info[i].device_state) ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], MAX_DIM_LEVEL_VALUE));
                        else ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], 0));
                        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[i]));
                    }  
                #else
                    if(scene_group_switch_info.control_type == 0 || scene_group_switch_info.control_type == 1) { 
                        if(device_info[i].device_state) gpio_set_level(gpio_touch_led_pins[i], 1);
                        else gpio_set_level(gpio_touch_led_pins[i], 0);
                    }
                #endif
            }
        }
    }

    void nuos_zb_set_hardware(uint8_t index, uint8_t is_toggle) {
        if(is_init_done){  
            call_common_check_auto_off();
            esp_stop_timer();
            process_dali_tasks(index, is_toggle, false);  
            esp_start_timer(); 
        }
    }

    void nuos_on_off_led(uint8_t index, uint8_t _state) {
        if(index < TOTAL_LEDS){
            #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
                if(!_state) ledc_set_duty(LEDC_MODE, pwm_channels[index], 0);
                else ledc_set_duty(LEDC_MODE, pwm_channels[index], MAX_DIM_LEVEL_VALUE);            
                ledc_update_duty(LEDC_MODE, pwm_channels[index]);
            #else
                gpio_set_level(gpio_touch_led_pins[index], _state);
            #endif
        }
    }

    void nuos_toggle_leds(uint8_t index){
        state = !state;       
        nuos_on_off_led(index, state);
    }
    void nuos_zb_set_hardware_led_for_zb_commissioning(uint8_t is_toggle) {
        // #ifdef USE_RGB_LED
        //     nuos_toggle_rgb_led();
        // #else
        //     if(TOTAL_LEDS >= TOTAL_LEDS_SHOW_ON_COMMISSIONING) {
        //         if(is_toggle>0) {
        //             state = !state;
        //             for(int index=TOTAL_LEDS-TOTAL_LEDS_SHOW_ON_COMMISSIONING; index<TOTAL_LEDS; index++) {
        //                 gpio_set_level(gpio_touch_led_pins[index], state);
        //             }
        //         }else{
        //             // for(int i=0; i<TOTAL_ENDPOINTS; i++) {
        //             //     nuos_zb_set_hardware(i, false);
        //             // }
        //         }
        //     }
        // #endif
    }


    uint8_t nuos_get_button_press_index(uint32_t pin) {
        for(int index=0; index<TOTAL_BUTTONS; index++){
            if(pin == gpio_touch_btn_pins[index]){
                return index;
            }
        }
        return 0;
    }

    #ifdef LONG_PRESS_BRIGHTNESS_ENABLE

    void nuos_init_hardware_dimming_up_down(uint32_t pin) {
        uint8_t index = nuos_get_button_press_index(pin);
        if(device_info[index].device_state){


            if(!change_cw_ww_color_flag) {
                if(device_info[index].device_level <= MIN_DIM_LEVEL_VALUE+50){
                    device_info[index].dim_up = 1;
                }else if(device_info[index].device_level >= MAX_DIM_LEVEL_VALUE-20){
                    device_info[index].dim_up = 0;
                }
            }else{
                if(device_info[index].device_val <= MIN_CCT_VALUE+400){
                    device_info[index].dim_up = 1;
                }else if(device_info[index].device_val >= MAX_CCT_VALUE-400){
                    device_info[index].dim_up = 0;
                }                
            }


        }
    }

    bool nuos_set_hardware_brightness(uint32_t pin) {
        uint8_t index = nuos_get_button_press_index(pin);
        if(global_switch_state == SWITCH_PRESS_DETECTED){ 

            call_common_check_auto_off();

            if(!device_info[index].device_state){
                device_info[index].device_state = true;
                device_info[index].device_level = MIN_DIM_LEVEL_VALUE;
                device_info[index].dim_up = 1;
            }
            if(device_info[index].device_state){

                uint8_t last_dim =  device_info[index].device_level;
                if(device_info[index].dim_up == 1){
                    if(device_info[index].device_level + DIMMING_STEPS <= (MAX_DIM_LEVEL_VALUE)){
                        device_info[index].device_level += DIMMING_STEPS;
                    } else {
                        device_info[index].device_level = MAX_DIM_LEVEL_VALUE;
                    }
                }else{
                    if(device_info[index].device_level - DIMMING_STEPS >= MIN_DIM_LEVEL_VALUE){
                        device_info[index].device_level -= DIMMING_STEPS;  
                    }else {
                        device_info[index].device_level = MIN_DIM_LEVEL_VALUE;
                    } 
                }
                //printf("brightness: %d\n", device_info[index].device_level);
                ledc_set_duty(LEDC_MODE, pwm_channels[index], device_info[index].device_level);            
                ledc_update_duty(LEDC_MODE, pwm_channels[index]);
                #ifdef DALI_DIRECT_ADDRESSING 
                    nuos_set_level_attribute(index);
                #endif
                
                process_dali_tasks(index, false, false);  
            }
        }
        return false;
    }
    #endif

    extern "C" void nuos_dali_add_light_to_group(uint8_t addr, uint8_t group_id) {
        // printf("Adding addr:%d to group:%d\n", addr, group_id);
        dali.add_to_group(addr, group_id);   
    }

    extern "C" void nuos_dali_remove_light_from_group(uint8_t addr, uint8_t group_id) {
        dali.remove_from_group(addr, group_id); 
    }
  
    extern "C" void nuos_dali_set_state(uint8_t dali_id, uint8_t state) { 
        if(state == 0) dali.turn_off(dali_id);
        else dali.set_dim_value(dali_id, MAX_DIM_LEVEL_VALUE); 
    } 
    extern "C" void nuos_dali_set_brightness(uint8_t dali_id, uint8_t level) { 
        dali.set_dim_value(dali_id,  map_1_255_to_100_255(level));
    }   
    
    extern "C" void nuos_dali_set_cct_color(uint8_t did, uint16_t value) {
        dali.set_color_temperature(did, value);
    }
    extern "C" void nuos_dali_set_rgb_color(uint8_t did, uint8_t r, uint8_t g, uint8_t b, bool mode_change_flag) {
        dali.set_color_rgb(did, r, g, b, 0xff, mode_change_flag);
    }
    extern "C" void nuos_dali_broadcast_state(uint8_t toggle_state) { 
        if(!toggle_state) dali.send_broadcast(0b00000000);
        else dali.set_broadcast_level(MAX_DIM_LEVEL_VALUE); 
    } 

    extern "C" void nuos_dali_broadcast_level(uint8_t level) {     
        dali.set_broadcast_level(map_1_255_to_100_255(level)); 
    } 

    extern "C" void nuos_dali_toggle_group(uint8_t group_id, uint8_t index, bool toggle_state, uint8_t brightness) {  
        if(!toggle_state) dali.set_group_off(group_id);
        else dali.set_group_on(group_id);
    } 

    extern "C" void nuos_dali_set_state_group(uint8_t group_id, bool _state) { 
        if(!_state) dali.set_group_off(group_id);
        else dali.set_group_level_normal(group_id, 254);//dali.set_group_on(group_id);
    }

    extern "C" void nuos_dali_add_device_to_scene(uint8_t device_id, uint8_t scene_id, uint8_t scene_level, uint16_t cct_temp) {
        if(scene_level == 0){
            dali.set_level_scene(device_id, scene_id, 0);
        }else{
            dali.set_color_scene(device_id, scene_id, scene_level, cct_temp);
        }
    }

    extern "C" void nuos_dali_rgb_add_device_to_scene(uint8_t device_id, uint8_t scene_id, uint8_t scene_level, uint8_t r, uint8_t g, uint8_t b) {
        if(scene_level == 0){
            dali.set_level_scene(device_id, scene_id, 0);
        }else{ 
            dali.set_rgb_scene(device_id, scene_id, scene_level, r, g, b);
        }
    }

    extern "C" void nuos_dali_remove_device_from_scene(uint8_t device_id, uint8_t scene_id) {
        dali.remove_from_scene(device_id, scene_id);
    }

    extern "C" void nuos_dali_add_device_state_to_scene(uint8_t device_id, uint8_t scene_id) {
        dali.add_to_scene(device_id, scene_id);
    }

    
    extern "C" void nuos_dali_set_power_on_level(uint8_t dali_id, uint8_t level) {
        //printf("Setting Power On Level of dali_id:%d to %d\n", dali_id, level);
        dali.set_power_on_level(dali_id, level);
    }

    extern "C" void nuos_dali_set_fade_time_fade_rate(uint8_t dali_id, uint8_t fade_time, uint8_t fade_rate) {
        vTaskDelay(pdMS_TO_TICKS(20));
        dali.set_fade_time(dali_id, fade_time);
        //printf("Setting Fade Time:%d fade Rate:%d of dali_id:%d\n", fade_time, fade_rate, dali_id);
        vTaskDelay(pdMS_TO_TICKS(20));
        dali.set_fade_rate(dali_id, fade_rate);
    }
   extern "C" int get_all_dali_addresses(uint8_t *foundAddresses) {
         // DALI allows up to 64 short addresses (0-63)
        uint8_t maxAddresses = 64;
        //printf("Discovering assigned DALI addresses:\n");
        // Scan for assigned short addresses
        int numFound = dali.scanAssignedShortAddresses(foundAddresses, maxAddresses);
        //printf("Found %d assigned DALI addresses:\n", numFound);
        for (int i = 0; i < numFound; ++i) {
            //printf("  Short address: %d\n", foundAddresses[i]>>1);
        }    
        return numFound;
    }

    extern "C" void nuos_dali_set_group_color_temperature(uint8_t group_id, uint8_t index, uint16_t value) {
       
       //xTaskCreate(dali_set_cct, "dali_set_cct", 4096, &index, 25, NULL); 
       dali.set_group_color_cct(group_id, value);
       //nuos_store_data_to_nvs(index);
      // #endif
    } 

    extern "C" void nuos_dali_set_group_rgb_temperature(uint8_t group_id, uint8_t r, uint8_t g, uint8_t b) { 
        dali.set_group_color_rgb(group_id, r, g, b, 255, false);
    } 
    

    extern "C" void nuos_dali_set_group_brightness(uint8_t group_id, uint8_t index, uint8_t value) {
        
        dali.set_group_level(group_id, map_1_255_to_100_255(value));
        
    }

    extern "C" void nuos_dali_add_group_to_scene(uint8_t group_id, uint8_t scene_id, uint8_t scene_level, uint16_t cct_temp) {
        
        // dali.add_to_scene(group_id | (1<<7), scene_id);
        dali.set_color_scene(group_id | (1<<7), scene_id, scene_level, cct_temp);
        
    }
   
    extern "C" void nuos_dali_remove_group_from_scene(uint8_t group_id, uint8_t scene_id) {
        
        dali.remove_from_scene(group_id | (1<<7), scene_id);
        
    } 



    static void esp_dali_commissioning_led_blink_task(void *pvParameters){
        // printf("On Task esp_commissioning_detect_task\n");
        bool _toggle = false;
        while(1){
            vTaskDelay(pdMS_TO_TICKS(500));
            for(int i=0; i<TOTAL_LEDS; i++){
                //gpio_set_level(gpio_touch_led_pins[i], _toggle);
                if(!_toggle){
                    ledc_set_duty(LEDC_MODE, pwm_channels[i], 0);            
                    ledc_update_duty(LEDC_MODE, pwm_channels[i]);
                }else{
                    ledc_set_duty(LEDC_MODE, pwm_channels[i], 0xff);            
                    ledc_update_duty(LEDC_MODE, pwm_channels[i]);
                }
            }
            _toggle = !_toggle;
        }
        vTaskDelete(NULL);
    }



    DaliNodeList read_dali_addressed_nodes() {
        DaliNodeList nodeList;
        // Initialize
        nodeList.total = 0;
        memset(nodeList.addresses, 0xFF, sizeof(nodeList.addresses));

        // Read existing drivers
        nodeList.total = dali.readExistingDrivers(nodeList.addresses, 64);

        return nodeList;
}
uint16_t daliQueryGear(uint8_t addr, uint8_t query_cmd){
    // 1. Save the current task handle so the RX task knows who to notify
    xQueryingTaskHandle = xTaskGetCurrentTaskHandle();
    queryResponseValue = 0; // Reset response buffer
    dali.queryGear(addr, query_cmd);    
// 3. Wait for the RX task to notify us. Timeout after 50ms (DALI typical max response window)
    uint32_t notificationValue = 0;
    BaseType_t status = xTaskNotifyWait(
        0x00,               // Do not clear bits on entry
        ULONG_MAX,          // Clear all bits on exit
        &notificationValue, // Stores notification value (optional)
        pdMS_TO_TICKS(100)   // 50ms timeout window (much better than 1000ms!)
    );

    // 4. Clean up the handle reference
    xQueryingTaskHandle = NULL;

    // 5. Evaluate if we successfully received data or timed out
    if (status == pdTRUE) {
        // Return the clean 0-255 byte cast into the wider signed integer
        return (int32_t)queryResponseValue; 
    } else {
        return -1; // Return -1 strictly for "Hardware Timeout / No Device Present"
    }
}

int32_t daliQueryPowerOnLevel(const uint8_t addr){

    xQueryingTaskHandle = xTaskGetCurrentTaskHandle();
    queryResponseValue = 0; // Reset response buffer
    dali.queryPowerOnLevel(addr);    
    // 3. Wait for the RX task to notify us. Timeout after 50ms (DALI typical max response window)
    uint32_t notificationValue = 0;
    BaseType_t status = xTaskNotifyWait(
        0x00,               // Do not clear bits on entry
        ULONG_MAX,          // Clear all bits on exit
        &notificationValue, // Stores notification value (optional)
        pdMS_TO_TICKS(1000)   // 50ms timeout window (much better than 1000ms!)
    );

    // 4. Clean up the handle reference
    xQueryingTaskHandle = NULL;

    // 5. Evaluate if we successfully received data or timed out
    if (status == pdTRUE) {
        // Return the clean 0-255 byte cast into the wider signed integer
        return (int32_t)queryResponseValue; 
    } else {
        return -1; // Return -1 strictly for "Hardware Timeout / No Device Present"
    }
}

int32_t daliQueryFadeTimeFadeRate(uint8_t addr){

    xQueryingTaskHandle = xTaskGetCurrentTaskHandle();
    queryResponseValue = 0; // Reset response buffer
    dali.queryFadeTimeFadeRate(addr);    
// 3. Wait for the RX task to notify us. Timeout after 50ms (DALI typical max response window)
    uint32_t notificationValue = 0;
    BaseType_t status = xTaskNotifyWait(
        0x00,               // Do not clear bits on entry
        ULONG_MAX,          // Clear all bits on exit
        &notificationValue, // Stores notification value (optional)
        pdMS_TO_TICKS(100)   // 100ms timeout window (much better than 1000ms!)
    );

    // 4. Clean up the handle reference
    xQueryingTaskHandle = NULL;

    // 5. Evaluate if we successfully received data or timed out
    if (status == pdTRUE) {
        // Return the clean 0-255 byte cast into the wider signed integer
        return (int32_t)queryResponseValue; 
    } else {
        return -1; // Return -1 strictly for "Hardware Timeout / No Device Present"
    }    
}

int32_t daliQueryDeviceType(uint8_t addr){

    xQueryingTaskHandle = xTaskGetCurrentTaskHandle();
    queryResponseValue = 0; // Reset response buffer
    dali.queryDeviceType(addr);    
// 3. Wait for the RX task to notify us. Timeout after 50ms (DALI typical max response window)
    uint32_t notificationValue = 0;
    BaseType_t status = xTaskNotifyWait(
        0x00,               // Do not clear bits on entry
        ULONG_MAX,          // Clear all bits on exit
        &notificationValue, // Stores notification value (optional)
        pdMS_TO_TICKS(100)   // 100ms timeout window (much better than 1000ms!)
    );

    // 4. Clean up the handle reference
    xQueryingTaskHandle = NULL;

    // 5. Evaluate if we successfully received data or timed out
    if (status == pdTRUE) {
        // Return the clean 0-255 byte cast into the wider signed integer
        return (int32_t)queryResponseValue; 
    } else {
        return -1; // Return -1 strictly for "Hardware Timeout / No Device Present"
    }    
}


int32_t daliQueryNextDeviceType(uint8_t addr){

    xQueryingTaskHandle = xTaskGetCurrentTaskHandle();
    queryResponseValue = 0; // Reset response buffer
    dali.queryNextDeviceType(addr);    
// 3. Wait for the RX task to notify us. Timeout after 50ms (DALI typical max response window)
    uint32_t notificationValue = 0;
    BaseType_t status = xTaskNotifyWait(
        0x00,               // Do not clear bits on entry
        ULONG_MAX,          // Clear all bits on exit
        &notificationValue, // Stores notification value (optional)
        pdMS_TO_TICKS(100)   // 100ms timeout window (much better than 1000ms!)
    );

    // 4. Clean up the handle reference
    xQueryingTaskHandle = NULL;

    // 5. Evaluate if we successfully received data or timed out
    if (status == pdTRUE) {
        // Return the clean 0-255 byte cast into the wider signed integer
        return (int32_t)queryResponseValue; 
    } else {
        return -1; // Return -1 strictly for "Hardware Timeout / No Device Present"
    }    
}

int32_t daliQueryGearFeatures(uint8_t addr){

    xQueryingTaskHandle = xTaskGetCurrentTaskHandle();
    queryResponseValue = 0; // Reset response buffer
    dali.queryGearFeatures(addr);    
// 3. Wait for the RX task to notify us. Timeout after 50ms (DALI typical max response window)
    uint32_t notificationValue = 0;
    BaseType_t status = xTaskNotifyWait(
        0x00,               // Do not clear bits on entry
        ULONG_MAX,          // Clear all bits on exit
        &notificationValue, // Stores notification value (optional)
        pdMS_TO_TICKS(100)   // 100ms timeout window (much better than 1000ms!)
    );

    // 4. Clean up the handle reference
    xQueryingTaskHandle = NULL;

    // 5. Evaluate if we successfully received data or timed out
    if (status == pdTRUE) {
        // Return the clean 0-255 byte cast into the wider signed integer
        return (int32_t)queryResponseValue; 
    } else {
        return -1; // Return -1 strictly for "Hardware Timeout / No Device Present"
    }    
}

int32_t daliQueryDeviceInGroupA(uint8_t addr){

    xQueryingTaskHandle = xTaskGetCurrentTaskHandle();
    queryResponseValue = 0; // Reset response buffer
    dali.queryDeviceInGroupA(addr);    
// 3. Wait for the RX task to notify us. Timeout after 50ms (DALI typical max response window)
    uint32_t notificationValue = 0;
    BaseType_t status = xTaskNotifyWait(
        0x00,               // Do not clear bits on entry
        ULONG_MAX,          // Clear all bits on exit
        &notificationValue, // Stores notification value (optional)
        pdMS_TO_TICKS(100)   // 100ms timeout window (much better than 1000ms!)
    );

    // 4. Clean up the handle reference
    xQueryingTaskHandle = NULL;

    // 5. Evaluate if we successfully received data or timed out
    if (status == pdTRUE) {
        // Return the clean 0-255 byte cast into the wider signed integer
        return (int32_t)queryResponseValue; 
    } else {
        return -1; // Return -1 strictly for "Hardware Timeout / No Device Present"
    }    
}

int32_t daliQueryDeviceInGroupB(uint8_t addr){

    xQueryingTaskHandle = xTaskGetCurrentTaskHandle();
    queryResponseValue = 0; // Reset response buffer
    dali.queryDeviceInGroupB(addr);    
// 3. Wait for the RX task to notify us. Timeout after 50ms (DALI typical max response window)
    uint32_t notificationValue = 0;
    BaseType_t status = xTaskNotifyWait(
        0x00,               // Do not clear bits on entry
        ULONG_MAX,          // Clear all bits on exit
        &notificationValue, // Stores notification value (optional)
        pdMS_TO_TICKS(100)   // 100ms timeout window (much better than 1000ms!)
    );

    // 4. Clean up the handle reference
    xQueryingTaskHandle = NULL;

    // 5. Evaluate if we successfully received data or timed out
    if (status == pdTRUE) {
        // Return the clean 0-255 byte cast into the wider signed integer
        return (int32_t)queryResponseValue; 
    } else {
        return -1; // Return -1 strictly for "Hardware Timeout / No Device Present"
    }    
}
    int total_nodes_assigned = 0;
    static void esp_dali_init_node_task(void *pvParameters) {
        uint16_t addr = *(uint16_t*)pvParameters;
        
        uint8_t numAddresses = (uint8_t)(addr & 0xff);
        uint8_t startAddresses = (uint8_t)((addr >> 8) & 0xff);
        if(numAddresses > 63) return;
        if(startAddresses > 63) return;

        ESP_LOGI("DALI", "=== Starting DALI addressing from %d to %d of total=%d ===", startAddresses, numAddresses, numAddresses-startAddresses);
        
        // // Pause WiFi to reduce interference
        #ifdef USE_WIFI_WEBSERVER
        #ifndef USE_C3_ADAPTER_UART_HW
        esp_wifi_stop();
        printf("WiFi stopped for DALI commissioning\n");
        vTaskDelay(pdMS_TO_TICKS(100));  // Let WiFi fully stop
        #endif
        #endif
        switch_driver_gpios_intr_enabled(false); 
        //printf("Starting DALI addressing...\n");
        // Perform DALI addressing with increased priority
        //vTaskPrioritySet(NULL, DALI_TASK_PRIORITY);
        uint8_t total_addr = (numAddresses-startAddresses)+1;
        printf("Initialize %d DALI nodes...\n", total_addr);

      //  dali.commissionNewNodes();

        // uint8_t addresses[64];

        // int total = dali.readExistingDrivers(addresses, 64);

        // for (int i = 0; i < total; i++)
        // {
        //     printf("Address[%d] = %d\n", i, addresses[i]);
            
        //    //dali.resetDriver(addresses[i]);

        //   // dali.clearShortAddress(addresses[i]);
        // }
        total_nodes_assigned = dali.commissionNewNodes();   
        // int totalfoundnodes = dali.initNodes(&global_dali_id[startAddresses], total_addr);

        // printf("Found nodes: %d\n", totalfoundnodes);
        // for (int i = 0; i < totalfoundnodes; i++) {
        //     printf("Node number %d, node address %d\n", i, global_dali_id[startAddresses+i]);
        // }
        
        // Signal completion
        if (recordsSemaphore != NULL) {
            xSemaphoreGive(recordsSemaphore);
        }
        switch_driver_gpios_intr_enabled(true); 
        ESP_LOGI("DALI", "=== DALI addressing complete ===");
        #ifndef USE_C3_ADAPTER_UART_HW
        wifi_webserver_active_flag = true;           
        setNVSCommissioningFlag(0);
        setNVSWebServerEnableFlag(wifi_webserver_active_flag);                    
        esp_restart();	
        #endif        
        vTaskDelete(NULL);
    }

    extern "C" int start_dali_addressing(uint8_t startAddresses, uint8_t numAddresses) {  
        total_nodes_assigned = 0;          
        recordsSemaphore = xSemaphoreCreateBinary();
        if (recordsSemaphore == NULL) {
            // Handle semaphore creation failure
            //printf("Failed to create semaphore!\n");
           return 0;
        }    
        uint16_t  addr = (numAddresses & 0xff) | ((startAddresses & 0xff) << 8);
        xTaskCreate(esp_dali_init_node_task, "dali_task", 8192, &addr, 11, NULL);
        start_dali_led_commissioning_task_flag = true;

        if (recordsSemaphore != NULL) {
            // Wait for the semaphore to be given by thaddre records task
            xSemaphoreTake(recordsSemaphore, portMAX_DELAY);
        }

        start_dali_led_commissioning_task_flag = false;
        // Restart WiFi
        #ifdef USE_WIFI_WEBSERVER
        #ifndef USE_C3_ADAPTER_UART_HW
        vTaskDelay(pdMS_TO_TICKS(200));
        wifi_restart();
        vTaskDelay(pdMS_TO_TICKS(500));  // Allow WiFi to stabilize
        #endif
        #endif
        
        for(int i=0; i<TOTAL_ENDPOINTS; i++){
            if(scene_group_switch_info.control_type == 1) { //scene control
                if(scene_group_switch_info.selected_id == i){
                    nuos_zb_set_hardware(i, false);
                    break;
                }
            }else{
                nuos_zb_set_hardware(i, false);
            }
        }
        return total_nodes_assigned;
    }  //end extern "C" void start_dali_addressing(uint8_t startAddresses, uint8_t numAddresses)

    extern "C" void nuos_dali_set_color_temperate(uint8_t index) {

    } 

    void start_dali_led_blink_task() {
        if(start_dali_led_commissioning_task_flag){
            for(int i=0; i<TOTAL_LEDS; i++){
                if(!_toggle_){
                    ledc_set_duty(LEDC_MODE, pwm_channels[i], 0);            
                    ledc_update_duty(LEDC_MODE, pwm_channels[i]);
                }else{
                    ledc_set_duty(LEDC_MODE, pwm_channels[i], 0xff);            
                    ledc_update_duty(LEDC_MODE, pwm_channels[i]);
                }
            }
            _toggle_ = !_toggle_;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    extern "C" void nuos_set_state_touch_leds(bool state) {
        if(is_init_done){
            for(int i=0; i<TOTAL_LEDS; i++){
                if(state){
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], MAX_DIM_LEVEL_VALUE));
                }else{
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], 0));
                }
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[i]));
            }
        }
    }

    extern "C" void nuos_set_state_touch_leds_to_original() {
        if(is_init_done){
            for(int i=0; i<TOTAL_LEDS; i++){
                if(device_info[i].device_state){
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], device_info[i].device_level));
                }else{
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], 0));
                }
    
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[i]));
            }
        }
    }

    extern "C" bool nuos_check_state_touch_leds() {
        bool getting_on_state = false;
        if(touchLedsOffAfter1MinuteEnable) {
            for(int i=0; i<TOTAL_LEDS; i++) {
                if(device_info[i].device_state) {
                    getting_on_state = true;
                }
            }
        }
        return getting_on_state;
    }   
#endif





