
#include "app_hardware_driver.h"
#include "app_zigbee_clusters.h"

#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI)

    #include "esp_log.h"
    #include "esp_err.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include <esp_check.h>
    #include <esp_log.h>
    #include "driver/ledc.h"
    #include "DaliCommands.h"


    DaliCommands 										        dali(gpio_load_pins[1], gpio_load_pins[0]);

    #define LEDC_TIMER              		                    LEDC_TIMER_0
    #define LEDC_MODE               		                    LEDC_LOW_SPEED_MODE

    #define LEDC_DUTY_RES          		 	                    LEDC_TIMER_8_BIT // Set duty resolution to 13 bits
    #define LEDC_DUTY               		                    100      // Set duty to 50%. ((2 ** 8) - 1) * 50% = 4095
    #define LEDC_MAX_DUTY           		                    254     //8191
    #define LEDC_FREQUENCY          	                        1000    // Frequency in Hertz. Set frequency at 1 KHz


    

    #if (TOTAL_ENDPOINTS == 4)
        ledc_channel_t pwm_channels[TOTAL_ENDPOINTS]    = { LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3};  
    #elif (TOTAL_ENDPOINTS == 3)
        ledc_channel_t pwm_channels[TOTAL_ENDPOINTS]    = { LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2};
    #elif (TOTAL_ENDPOINTS == 2)
        ledc_channel_t pwm_channels[TOTAL_ENDPOINTS]    = { LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    #elif (TOTAL_ENDPOINTS == 1)
        ledc_channel_t pwm_channels[TOTAL_ENDPOINTS]    = { LEDC_CHANNEL_0};
    #endif
    bool is_init_done = false;
    #define IS_USE_DALI_HARDWARE

    extern "C" void nuos_set_state_touch_leds(bool state);
    extern "C" bool nuos_check_state_touch_leds();

    static bool state = false;
    uint8_t level = 0;

    extern "C" void nuos_zb_init_hardware(){
        for(int index=1; index<TOTAL_LEDS; index++){
            gpio_reset_pin(gpio_touch_led_pins[index]);
            // /* Set the GPIO as a push/pull output */
            gpio_set_direction(gpio_touch_led_pins[index], GPIO_MODE_OUTPUT);
            gpio_set_level(gpio_touch_led_pins[index], 0);
        }                  
        // Prepare and then apply the LEDC PWM timer configuration
        ledc_timer_config_t ledc_timer = {
            .speed_mode       = LEDC_MODE,
            .duty_resolution  = LEDC_DUTY_RES,
            .timer_num        = LEDC_TIMER,
            .freq_hz          = LEDC_FREQUENCY,
            .clk_cfg          = LEDC_AUTO_CLK
        };
        ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));       
        for(int index=0; index<TOTAL_LEDS; index++){
            // Prepare and then apply the LEDC PWM channel configuration
            ledc_channel_config_t ledc_channel = {
                .gpio_num       = gpio_touch_led_pins[index],
                .speed_mode     = LEDC_MODE,
                .channel        = pwm_channels[index],
                .intr_type      = LEDC_INTR_DISABLE,
                .timer_sel      = LEDC_TIMER,
                .duty           = device_info[index].device_level, // Set duty to 0%
                .hpoint         = 0
            };
            ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel)); 
        }
        is_init_done = true;     
    }

    extern "C" void init_dali_hw(){
        #ifdef IS_USE_DALI_HARDWARE
            dali.begin();
            vTaskDelay(100/ portTICK_PERIOD_MS);
        #endif
        for(int i=0; i<TOTAL_ENDPOINTS; i++){
            printf("----------VALUE:%d", device_info[i].device_val);
            #ifdef IS_USE_DALI_HARDWARE
                //nuos_dali_set_group_color_temperature(global_group_id[i], i, device_info[i].device_val);
                vTaskDelay(10/ portTICK_PERIOD_MS);
                dali.set_group_fade_rate(global_group_id[i], 12);
                vTaskDelay(10/ portTICK_PERIOD_MS);
                dali.set_group_fade_time(global_group_id[i], 1);  
                vTaskDelay(10/ portTICK_PERIOD_MS);
            #endif
            //nuos_zb_set_hardware(i, 0); 
        }    
        //start_dali_addressing(3);
    }


    void set_harware(uint8_t index, uint8_t is_toggle){
        if(is_toggle>0) device_info[index].device_state = !device_info[index].device_state;
        if(is_init_done){
            if(!device_info[index].device_state){
                //ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], 0));
                dali.set_group_off(global_group_id[index]);
            }else{
                // if(device_info[index].device_level >= MIN_DIM_LEVEL_VALUE){
                //     ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], device_info[index].device_level));
                // }else{
                //     ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], MIN_DIM_LEVEL_VALUE));                   
                // }
                nuos_dali_set_group_color_temperature(dali_nvs_stt[index].group_id, index, device_info[index].device_val);
                vTaskDelay(5/ portTICK_PERIOD_MS);
                nuos_dali_set_group_brightness(dali_nvs_stt[index].group_id, index, device_info[index].device_level);                
            }
           // ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[index]));
        }
        // #ifdef USE_RGB_LED
        //     nuos_rgb_trigger_blink();
        // #endif             
    }

    void set_leds(int i, bool state){
        if(is_init_done){
            if(i == 0){
                if(gpio_touch_led_pins[0] == gpio_load_pins[0]){
                    if(state) ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[0], device_info[i].device_level));
                    else ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[0], 0));
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[0]));
                }else{
                    if(state) ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], device_info[i].device_level));
                    else ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], 0));
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[i]));
                }
            }else{
                if(state) ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], device_info[i].device_level));
                else ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], 0));
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[i]));
            } 
        }
    }
    void nuos_zb_set_hardware(uint8_t index, uint8_t is_toggle){
        //set touch led pins
        if(timer3_running_flag){
            //toggle pins on button press
            set_harware(index, is_toggle);
        }else{
            if(nuos_check_state_touch_leds()){
                for(int i=0; i<TOTAL_LEDS; i++){
                    set_leds(i, device_info[i].device_state);                       
                }
            }else{
                //toggle pins on button press
                set_harware(index, is_toggle);               
            }
        }
        nuos_store_data_to_nvs(index);
    }

    uint8_t nuos_get_button_press_index(uint32_t pin){
        for(int index=0; index<TOTAL_BUTTONS; index++){
            if(pin == gpio_touch_btn_pins[index]){
                return index;
            }
        }
        return 0;
    }

    static unsigned char last_level = 0;
    void nuos_init_hardware_dimming_up_down(uint32_t pin){
        uint8_t index = nuos_get_button_press_index(pin);
        if(device_info[index].device_state){
            if(device_info[index].device_level <= MIN_DIM_LEVEL_VALUE){
                device_info[index].dim_up = 1;
            }else if(device_info[index].device_level >= MAX_DIM_LEVEL_VALUE){
                device_info[index].dim_up = 0;
            }
        }
    }


    bool nuos_set_hardware_brightness(uint32_t pin){
        uint8_t index = nuos_get_button_press_index(pin);
        if(global_switch_state == SWITCH_PRESS_DETECTED){ 
            if(device_info[index].device_state){
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
                if(is_init_done){
                //ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], device_info[index].device_level));
                printf("device_level:%d\n", device_info[index].device_level);
                }
                nuos_dali_set_group_brightness(dali_nvs_stt[index].group_id, index, device_info[index].device_level);
            }
            if(level != device_info[index].device_level){
                level = device_info[index].device_level;
                if(is_init_done){
                //ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[index]));
                }
                nuos_store_data_to_nvs(index);            
            }
        }
        // if(wifi_webserver_active_flag == 0){
        //     nuos_set_zigbee_attribute(index);
        // }        
        return false;

    }



    void nuos_on_off_led(uint8_t index, uint8_t _level){
        if(is_init_done){
            if(index == 0){
                if(gpio_touch_led_pins[0] == gpio_load_pins[0]){
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[0], _level));
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[0]));
                }else{
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], _level));
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[index]));
                }
            }else{
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], _level));
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[index]));
            }
        }
    }

    extern "C" void nuos_toggle_leds(uint8_t index){
        state = !state;
        uint8_t level = 0;
        if(state) level = LEDC_MAX_DUTY;        
        nuos_on_off_led(index, level);
    }

    
    extern "C" void nuos_zb_set_hardware_led_for_zb_commissioning(uint8_t is_toggle){
        uint8_t level = 0;
		if(TOTAL_LEDS >= TOTAL_LEDS_SHOW_ON_COMMISSIONING){
            if(is_toggle>0) {
                state = !state; 
                if(state) level = LEDC_MAX_DUTY; 
                else level = 0;
            }
			for(int index=TOTAL_LEDS-TOTAL_LEDS_SHOW_ON_COMMISSIONING; index<TOTAL_LEDS; index++){
                if(is_toggle==0) {
                    state = device_info[index].device_state; 
                    level = device_info[index].device_level;
                    // if(device_info[index].device_level > LEDC_MAX_DUTY) level = LEDC_MAX_DUTY;
                }
                
                if(is_init_done){
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], level));
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[index]));
                }

			}
		}
    }  



    extern "C" void nuos_dali_add_light_to_group(uint8_t addr, uint8_t group_id){
        //#ifdef IS_USE_DALI_HARDWARE
        dali.add_to_group(addr, group_id); 
        //#endif
    }

    extern "C" void nuos_dali_remove_light_from_group(uint8_t addr, uint8_t group_id){
        //#ifdef IS_USE_DALI_HARDWARE
        dali.remove_from_group(addr, group_id); 
        //#endif
    }
  
    extern "C" void nuos_dali_toggle_group(uint8_t group_id, uint8_t index, bool toggle_state, uint8_t brightness){
        //#ifdef IS_USE_DALI_HARDWARE
        if(!toggle_state) dali.set_group_off(group_id);
        else { 
            //dali.set_group_on(group_id); 
            nuos_dali_set_group_brightness(dali_nvs_stt[index].group_id, index, brightness);
        } 
        //#endif
    } 
    
    void dali_set_cct(void* args) {
        uint8_t index = *((uint8_t *)args);
        //printf("index:%d", index);
        dali.set_group_color_cct(global_group_id[index], device_info[index].device_val);
        vTaskDelete(NULL); // Delete the task after executing
    }

    extern "C" void nuos_dali_set_group_color_temperature(uint8_t group_id, uint8_t index, uint16_t value){
       //#ifdef IS_USE_DALI_HARDWARE
       //xTaskCreate(dali_set_cct, "dali_set_cct", 4096, &index, 25, NULL); 
       dali.set_group_color_cct(global_group_id[index], value);
       nuos_store_data_to_nvs(index);
      // #endif
    } 

    extern "C" void nuos_dali_set_group_brightness(uint8_t group_id, uint8_t index, uint8_t value){
        //#ifdef IS_USE_DALI_HARDWARE
        dali.set_group_level(group_id, value);
        //#endif
    } 

    uint16_t cct_values[5] = {MIN_CCT_VALUE, MIN_CCT_VALUE_1, MIN_CCT_VALUE_2, MIN_CCT_VALUE_3, MAX_CCT_VALUE};
    uint8_t cct_counts = 0;
    extern "C" void nuos_dali_set_color_temperate(uint8_t index){
        device_info[index].device_val = cct_values[cct_counts];
        //#ifdef IS_USE_DALI_HARDWARE
        dali.set_group_color_cct(dali_nvs_stt[index].group_id, cct_values[cct_counts]);
        //#endif
        if(cct_counts++ >= 4){
            cct_counts = 0;
        }
        nuos_store_data_to_nvs(index);
        // if(wifi_webserver_active_flag == 0){
        //     nuos_set_zigbee_attribute(index);
        // }
    }

    //User can expand or lower this array according to number of Lights !!
    const uint8_t addresses[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};    
    extern "C" void start_dali_addressing(int numAddresses)
    {                                                                                                                                                                                          
        printf("Initialize DALI nodes...");
        int total_found_nodes = dali.initNodes(addresses, numAddresses);
        printf("Found nodes:%d\n", total_found_nodes);
        for (int i = 0; i < total_found_nodes; ++i) {
            printf("Node number: %d, node address: %d\n", i, addresses[i]);
        }
    }// end void start_dali_addressing(int numAddresses)

    void nuos_set_state_touch_leds(bool state){
        for(int i=0; i<TOTAL_LEDS; i++){
             set_leds(i, state);
        }
    }

    extern "C" void nuos_set_state_touch_leds_to_original(){
        for(int i=0; i<TOTAL_LEDS; i++){
            if(device_info[i].device_state){
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], device_info[i].device_level));
            }else{
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], 0));
            }
            
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[i]));
        }
    } 

    extern "C" bool nuos_check_state_touch_leds(){
        bool getting_on_state = false;
        if(touchLedsOffAfter1MinuteEnable){
            for(int i=0; i<TOTAL_LEDS; i++){
                if(device_info[i].device_state){
                    getting_on_state = true;
                }
            }
        }
        return getting_on_state;
    }      
#endif




