
#include "app_hardware_driver.h"
#include "app_zigbee_clusters.h"

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

    void start_dali_addressing(uint8_t startAddresses, uint8_t numAddresses);

    #define LEDC_TIMER              		                    LEDC_TIMER_0
    #define LEDC_MODE               		                    LEDC_LOW_SPEED_MODE

    #define LEDC_DUTY_RES          		 	                    LEDC_TIMER_8_BIT // Set duty resolution to 13 bits
    #define LEDC_DUTY               		                    100      // Set duty to 50%. ((2 ** 8) - 1) * 50% = 4095
    #define LEDC_MAX_DUTY           		                    254     //8191
    //#define LEDC_MIN_DUTY           		                    (LEDC_MAX_DUTY*10)/100     //10%

    // #define LEDC_FREQUENCY          	                        1000    // Frequency in Hertz. Set frequency at 1 KHz


    #define LEDC_FREQUENCY                                      (1000) // Frequency in Hz (5 kHz)
    #define LEDC_FADE_TIME                                      (500) // Fade time in milliseconds (1 second)

    ledc_channel_t pwm_channels[TOTAL_ENDPOINTS]                = { LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3};


    // Add at top with other defines
    #define DALI_TASK_PRIORITY                                  6  // Higher than webserver tasks
    #define DALI_ADDRESSING_TIMEOUT_MS                          30000  // 30 second timeout
    static TaskHandle_t dali_comm_task_handle                   = NULL;
    bool _toggle_                                               = false;
    static uint8_t group_addr                                   = 0xff;


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
            if(index < 4){
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
        
        #ifdef IS_USE_DALI_HARDWARE
            dali.begin();
        #endif
        is_init_done = true; 
    }
    
    void init_dali_hw() {
        #ifdef IS_USE_DALI_HARDWARE
            dali.begin();
            vTaskDelay(100/ portTICK_PERIOD_MS);
        #endif
        is_init_done = true;  
    }


    uint8_t nuos_dali_switch_type() {
        if(scene_group_switch_info.control_type == 0) {  //individual control
            return 1; 
        }else if(scene_group_switch_info.control_type == 1) { //group control
            return 2; 
        }else if(scene_group_switch_info.control_type == 2) { //scene control
            return 3; 
        }else if(scene_group_switch_info.control_type == 3) { //broadcast control
            return 4; 
        }
        return 0;
    }     
    
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

    static void esp_dali_off_task(void* args) {
        uint8_t index = *(uint8_t*)args;
        for(int i=0; i<scene_group_switch_info.total_ids[index]; i++){
            dali.turn_off(scene_group_switch_info.device_ids[index][i]);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        vTaskDelete(NULL); // Delete the task after executing
    }

    static void esp_dali_on_task(void* args) {
        uint8_t index = *(uint8_t*)args;
        for(int i=0; i<scene_group_switch_info.total_ids[index]; i++){
            
            dali.set_dim_value(scene_group_switch_info.device_ids[index][i], device_info[index].device_level);
            vTaskDelay(10 / portTICK_PERIOD_MS);
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

    extern "C" void process_dali_tasks(uint8_t index, uint8_t is_toggle){
        //pause_my_task();
        if(scene_group_switch_info.control_type == 0) {  //individual control
            if(is_toggle>0) device_info[index].device_state = !device_info[index].device_state;
            if(!device_info[index].device_state) {
                printf("DALI ID:%d OFF\n", scene_group_switch_info.scene_ids[index]);
                #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
                    ledc_set_duty(LEDC_MODE, pwm_channels[index], 0);            
                    ledc_update_duty(LEDC_MODE, pwm_channels[index]);
                #else
                    gpio_set_level(gpio_touch_led_pins[index], 0);
                #endif
                // for(int i=0; i<scene_group_switch_info.total_ids[index]; i++){
                //     dali.turn_off(scene_group_switch_info.device_ids[index][i]);
                //     vTaskDelay(5 / portTICK_PERIOD_MS);
                // }
                xTaskCreate(esp_dali_off_task, "dali_off_task", 4096, &index, TASK_PRIORITY_RGB, NULL);
            } else {
                printf("DALI ID:%d ON\n", scene_group_switch_info.scene_ids[index]);
                #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
                    ledc_set_duty(LEDC_MODE, pwm_channels[index], device_info[index].device_level);            
                    ledc_update_duty(LEDC_MODE, pwm_channels[index]);
                #else
                    gpio_set_level(gpio_touch_led_pins[index], 1);
                #endif
                //dali.set_dim_value(scene_group_switch_info.scene_ids[index], device_info[index].device_level);
                // for(int i=0; i<scene_group_switch_info.total_ids[index]; i++){
                //     dali.set_dim_value(scene_group_switch_info.device_ids[index][i], MAX_DIM_LEVEL_VALUE);
                //     vTaskDelay(5/ portTICK_PERIOD_MS);
                // }
                xTaskCreate(esp_dali_on_task, "dali_on_task", 4096, &index, TASK_PRIORITY_RGB, NULL);
            }                    
        }else if(scene_group_switch_info.control_type == 1) { //group control
            if(is_toggle>0) device_info[index].device_state = !device_info[index].device_state;
            if(!device_info[index].device_state) {
                printf("DALI Group OFF :%d , btn_id:%d\n", index, scene_group_switch_info.scene_ids[index]);
                #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
                    ledc_set_duty(LEDC_MODE, pwm_channels[index], 0);            
                    ledc_update_duty(LEDC_MODE, pwm_channels[index]);
                #else
                    gpio_set_level(gpio_touch_led_pins[index], 0);
                #endif
                dali.set_group_off(scene_group_switch_info.scene_ids[index]);
            } else {
                printf("DALI Group ON :%d , btn_id:%d\n", index, scene_group_switch_info.scene_ids[index]);
                #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
                    ledc_set_duty(LEDC_MODE, pwm_channels[index], device_info[index].device_level);            
                    ledc_update_duty(LEDC_MODE, pwm_channels[index]);
                #else
                    gpio_set_level(gpio_touch_led_pins[index], 1);
                #endif                    
                dali.set_group_level(scene_group_switch_info.scene_ids[index], device_info[index].device_level);
                if(change_cw_ww_color_flag) {
                    dali.set_group_color_cct(scene_group_switch_info.scene_ids[index], device_info[index].device_val);   
                }   
            }                      
        }else if(scene_group_switch_info.control_type == 2) { //scene control
            scene_group_switch_info.selected_id = index;
            for(int i=0; i<TOTAL_ENDPOINTS; i++) {
                if(i == index) {
                    #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
                        ledc_set_duty(LEDC_MODE, pwm_channels[i], MAX_DIM_LEVEL_VALUE);        
                        ledc_update_duty(LEDC_MODE, pwm_channels[i]);
                    #else
                        gpio_set_level(gpio_touch_led_pins[index], 1);
                    #endif

                    // if(scene_group_switch_info.scn_ctrl_type == 0) {  //broadcast control
                        printf("Recall DALI Scene (Broadcast) :%d\n", scene_group_switch_info.scene_ids[index]);
                        dali.go_to_scene(0xff, scene_group_switch_info.scene_ids[index]);  
                    // }else if(scene_group_switch_info.scn_ctrl_type == 1) { //group control
                    //     printf("Recall DALI Scene (Group:%d) :%d\n", scene_group_switch_info.device_ids[index][0], scene_group_switch_info.scene_ids[index]);
                    //     //uint8_t group_addr = dali.get_group_addr(scene_group_switch_info.group_id); 
                    //     dali.go_to_group_scene(scene_group_switch_info.device_ids[index][0], scene_group_switch_info.scene_ids[index]);
                    //     //dali.go_to_scene(0xff, scene_group_switch_info.scene_ids[index]);
                    // }else if(scene_group_switch_info.scn_ctrl_type == 2) { //individual control    
                    //     xTaskCreate(esp_dali_set_scene_task, "dali_scene_task", 4096, &index, TASK_PRIORITY_RGB, NULL);  
                    // }                    
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

    void nuos_zb_set_hardware(uint8_t index, uint8_t is_toggle) {
        if(is_init_done){  
            if(timer3_running_flag){
                process_dali_tasks(index, is_toggle);            
            }else{
                if(nuos_check_state_touch_leds()) {
                    #ifndef USE_INDIVIDUAL_DALI_ADDRESSING
                    for(int i=0; i<TOTAL_LEDS; i++){
                        if(device_info[i].device_state) {
                            #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
                                ledc_set_duty(LEDC_MODE, pwm_channels[i], MAX_DIM_LEVEL_VALUE);            
                                ledc_update_duty(LEDC_MODE, pwm_channels[i]);
                            #else
                                gpio_set_level(gpio_touch_led_pins[index], 1);
                            #endif
                        }else{
                            #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
                                ledc_set_duty(LEDC_MODE, pwm_channels[i], 0);            
                                ledc_update_duty(LEDC_MODE, pwm_channels[i]);
                            #else
                                gpio_set_level(gpio_touch_led_pins[index], 0);
                            #endif
                        }
                    }
                    #endif
                }else{
                    process_dali_tasks(index, is_toggle);           
                }
            } 
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
                if(device_info[index].device_level <= MIN_DIM_LEVEL_VALUE){
                    device_info[index].dim_up = 1;
                }else if(device_info[index].device_level >= MAX_DIM_LEVEL_VALUE){
                    device_info[index].dim_up = 0;
                }
            }else{
                if(device_info[index].device_val <= MIN_CCT_VALUE){
                    device_info[index].dim_up = 1;
                }else if(device_info[index].device_val >= MAX_CCT_VALUE){
                    device_info[index].dim_up = 0;
                }                
            }


        }
    }

    bool nuos_set_hardware_brightness(uint32_t pin) {
        uint8_t index = nuos_get_button_press_index(pin);
        if(global_switch_state == SWITCH_PRESS_DETECTED){ 
            if(scene_group_switch_info.control_type != 1) {  //group ctrl
                change_cw_ww_color_flag = false;
            }
            if(!device_info[index].device_state){
                device_info[index].device_state = true;
                if(!change_cw_ww_color_flag) {
                    device_info[index].device_level = MIN_DIM_LEVEL_VALUE;
                }else{
                    device_info[index].device_val = MIN_CCT_VALUE;
                }
                device_info[index].dim_up = 1;
            }
            if(device_info[index].device_state){

                if(!change_cw_ww_color_flag) {
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
                    ledc_set_duty(LEDC_MODE, pwm_channels[index], device_info[index].device_level);            
                    ledc_update_duty(LEDC_MODE, pwm_channels[index]);
                }else{
                    if(device_info[index].dim_up == 1){
                        if(device_info[index].device_val + COLOR_CHANGE_STEPS <= (MAX_CCT_VALUE)){
                            device_info[index].device_val += COLOR_CHANGE_STEPS;
                        } else {
                            device_info[index].device_val = MAX_CCT_VALUE;
                        }
                    }else{
                        if(device_info[index].device_val - COLOR_CHANGE_STEPS >= MIN_CCT_VALUE){
                            device_info[index].device_val -= COLOR_CHANGE_STEPS;  
                        }else {
                            device_info[index].device_val = MIN_CCT_VALUE;
                        } 
                    }
                    printf("color_temp: %d\n", device_info[index].device_val);                  
                }
                
                process_dali_tasks(index, false);  

            }
        }
        return false;
    }
    #endif

    extern "C" void nuos_dali_add_light_to_group(uint8_t addr, uint8_t group_id) {
        //#ifdef IS_USE_DALI_HARDWARE
        printf("Adding addr:%d to group:%d\n", addr, group_id);
        dali.add_to_group(addr, group_id); 
        //#endif
    }

    extern "C" void nuos_dali_remove_light_from_group(uint8_t addr, uint8_t group_id) {
        //#ifdef IS_USE_DALI_HARDWARE
        dali.remove_from_group(addr, group_id); 
        //#endif
    }
  
    extern "C" void nuos_dali_set_state(uint8_t dali_id, uint8_t state) { 
        //#ifdef IS_USE_DALI_HARDWARE
        if(state == 0) dali.turn_off(dali_id);
        else dali.set_dim_value(dali_id, MAX_DIM_LEVEL_VALUE);
        //#endif
    } 
    extern "C" void nuos_dali_set_brightness(uint8_t dali_id, uint8_t level) { 
        dali.set_dim_value(dali_id, level);
    }   
    
    extern "C" void nuos_dali_set_cct_color(uint8_t did, uint16_t value) {
        dali.set_color_temperature(did, value);
    }

    extern "C" void nuos_dali_broadcast_state(uint8_t toggle_state) { 
        //#ifdef IS_USE_DALI_HARDWARE
        if(!toggle_state) dali.send_broadcast(0b00000000); //dali.OFF_C
        else dali.set_broadcast_level(MAX_DIM_LEVEL_VALUE);
        //#endif
    } 

    extern "C" void nuos_dali_broadcast_level(uint8_t level) { 
        //#ifdef IS_USE_DALI_HARDWARE
        dali.set_broadcast_level(level);
        //#endif
    } 

    extern "C" void nuos_dali_toggle_group(uint8_t group_id, uint8_t index, bool toggle_state, uint8_t brightness) { 
        //#ifdef IS_USE_DALI_HARDWARE
        if(!toggle_state) dali.set_group_off(group_id);
        else dali.set_group_on(group_id);
        //#endif
    } 
    extern "C" void nuos_dali_set_state_group(uint8_t group_id, bool _state) { 
        //#ifdef IS_USE_DALI_HARDWARE
        if(!_state) dali.set_group_off(group_id);
        else dali.set_group_on(group_id);
        //#endif
    }


    extern "C" void nuos_dali_add_device_to_scene(uint8_t device_id, uint8_t scene_id, uint8_t scene_level, uint16_t cct_temp) {
        dali.set_color_scene(device_id, scene_id, scene_level, cct_temp);
    }
    extern "C" void nuos_dali_remove_device_from_scene(uint8_t device_id, uint8_t scene_id) {
        dali.remove_from_scene(device_id, scene_id);
    }

    extern "C" void nuos_dali_add_device_state_to_scene(uint8_t device_id, uint8_t scene_id) {
        dali.add_to_scene(device_id, scene_id);
    }
   extern "C" int get_all_dali_addresses(uint8_t *foundAddresses) {
         // DALI allows up to 64 short addresses (0-63)
        uint8_t maxAddresses = 64;
        printf("Discovering assigned DALI addresses:\n");
        // Scan for assigned short addresses
        int numFound = dali.scanAssignedShortAddresses(foundAddresses, maxAddresses);
        printf("Found %d assigned DALI addresses:\n", numFound);
        for (int i = 0; i < numFound; ++i) {
            printf("  Short address: %d\n", foundAddresses[i]>>1);
        }    
        return numFound;
    }

    extern "C" void nuos_dali_set_group_color_temperature(uint8_t group_id, uint8_t index, uint16_t value) {
       //#ifdef IS_USE_DALI_HARDWARE
       //xTaskCreate(dali_set_cct, "dali_set_cct", 4096, &index, 25, NULL); 
       dali.set_group_color_cct(group_id, value);
       //nuos_store_data_to_nvs(index);
      // #endif
    } 

    extern "C" void nuos_dali_set_group_brightness(uint8_t group_id, uint8_t index, uint8_t value) {
        //#ifdef IS_USE_DALI_HARDWARE
        dali.set_group_level(group_id, value);
        //#endif
    }

    extern "C" void nuos_dali_add_group_to_scene(uint8_t group_id, uint8_t scene_id, uint8_t scene_level, uint16_t cct_temp) {
        //#ifdef IS_USE_DALI_HARDWARE
        // dali.add_to_scene(group_id | (1<<7), scene_id);
        dali.set_color_scene(group_id | (1<<7), scene_id, scene_level, cct_temp);
        //#endif
    }
   
    extern "C" void nuos_dali_remove_group_from_scene(uint8_t group_id, uint8_t scene_id) {
        //#ifdef IS_USE_DALI_HARDWARE
        dali.remove_from_scene(group_id | (1<<7), scene_id);
        //#endif
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

    static void esp_dali_init_node_task(void *pvParameters) {
        uint16_t addr = *(uint16_t*)pvParameters;
        
        uint8_t numAddresses = (uint8_t)(addr & 0xff);
        uint8_t startAddresses = (uint8_t)((addr >> 8) & 0xff);
        if(numAddresses > 63) return;
        if(startAddresses > 63) return;

        ESP_LOGI("DALI", "=== Starting DALI addressing from %d to %d of total=%d ===", startAddresses, numAddresses, numAddresses-startAddresses);
        
        // // Pause WiFi to reduce interference
        #ifdef USE_WIFI_WEBSERVER
        esp_wifi_stop();
        vTaskDelay(pdMS_TO_TICKS(100));  // Let WiFi fully stop
        #endif
        switch_driver_gpios_intr_enabled(false); 
        // Perform DALI addressing with increased priority
        vTaskPrioritySet(NULL, DALI_TASK_PRIORITY);
        uint8_t total_addr = (numAddresses-startAddresses)+1;
        printf("Initialize %d DALI nodes...\n", total_addr);
        int totalfoundnodes = dali.initNodes(&global_dali_id[startAddresses], total_addr);

        printf("Found nodes: %d\n", totalfoundnodes);
        for (int i = 0; i < totalfoundnodes; i++) {
            printf("Node number %d, node address %d\n", i, global_dali_id[startAddresses+i]);
        }
        
        // Signal completion
        if (recordsSemaphore != NULL) {
            xSemaphoreGive(recordsSemaphore);
        }
        switch_driver_gpios_intr_enabled(true); 
        ESP_LOGI("DALI", "=== DALI addressing complete ===");
        
        vTaskDelete(NULL);
    }

    extern "C" void start_dali_addressing(uint8_t startAddresses, uint8_t numAddresses) {            
        recordsSemaphore = xSemaphoreCreateBinary();
        if (recordsSemaphore == NULL) {
            // Handle semaphore creation failure
            printf("Failed to create semaphore!\n");
            return;
        }    
        uint16_t  addr = (numAddresses & 0xff) | ((startAddresses & 0xff) << 8);
        xTaskCreate(esp_dali_init_node_task, "dali_task", 8192, &addr, 8, NULL);
        start_dali_led_commissioning_task_flag = true;
        //xTaskCreate(esp_dali_commissioning_led_blink_task, "dali_comm_task", 2048, NULL, 16, &dali_comm_task_handle);
        if (recordsSemaphore != NULL) {
            // Wait for the semaphore to be given by thaddre records task
            xSemaphoreTake(recordsSemaphore, portMAX_DELAY);
        }

        start_dali_led_commissioning_task_flag = false;
        // Restart WiFi
        #ifdef USE_WIFI_WEBSERVER
        vTaskDelay(pdMS_TO_TICKS(200));
        wifi_restart();
        vTaskDelay(pdMS_TO_TICKS(500));  // Allow WiFi to stabilize
        #endif
        
        for(int i=0; i<TOTAL_ENDPOINTS; i++){
            if(scene_group_switch_info.control_type == 2) { //scene control
                if(scene_group_switch_info.selected_id == i){
                    nuos_zb_set_hardware(i, false);
                    break;
                }
            }else{
                nuos_zb_set_hardware(i, false);
            }
        }
    }  //end extern "C" void start_dali_addressing(uint8_t startAddresses, uint8_t numAddresses)

    extern "C" void nuos_dali_set_color_temperate(uint8_t index) {

    } 

    
    void start_dali_led_blink_task(){
    
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
    }

    extern "C" void nuos_set_state_touch_leds(bool state) {
        if(is_init_done){
            for(int i=0; i<TOTAL_LEDS; i++){
                // if(state){
                //     ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], MAX_DIM_LEVEL_VALUE));
                // }else{
                //     ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], 0));
                // }
                // ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[i]));
            }
        }
    }

    extern "C" void nuos_set_state_touch_leds_to_original() {
        if(is_init_done){
        for(int i=0; i<TOTAL_LEDS; i++){
            // if(device_info[i].device_state){
            //     ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], device_info[i].device_level));
            // }else{
            //     ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], 0));
            // }
  
            // ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[i]));
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





