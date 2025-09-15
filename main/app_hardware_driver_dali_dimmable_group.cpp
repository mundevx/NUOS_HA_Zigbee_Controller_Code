
#include "app_hardware_driver.h"
#include "app_zigbee_clusters.h"

#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI)

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
    #define LEDC_MAX_DUTY           		                    MAX_DIM_LEVEL_VALUE     //8191
    #define LEDC_FREQUENCY          	                        5000    // Frequency in Hertz. Set frequency at 5 KHz


    #define DIMMING_STEPS                                       20   //(0- 254)

    int LEDC_TIMER_T[4]                                            = { LEDC_TIMER_0, LEDC_TIMER_0, LEDC_TIMER_0, LEDC_TIMER_0 }; 
    #if (TOTAL_LEDS == 4)
        ledc_channel_t pwm_channels[TOTAL_BUTTONS]                     = { LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3};  
    #elif (TOTAL_LEDS == 3)
        ledc_channel_t pwm_channels[TOTAL_BUTTONS]                     = { LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2};
    #elif (TOTAL_LEDS == 2)
        ledc_channel_t pwm_channels[TOTAL_BUTTONS]                     = { LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    #elif (TOTAL_LEDS == 1)
        ledc_channel_t pwm_channels[TOTAL_BUTTONS]                     = { LEDC_CHANNEL_0};
    #endif
    bool is_init_done = false;
    #define IS_USE_DALI_HARDWARE

    extern "C" void nuos_set_state_touch_leds(bool state);
    extern "C" bool nuos_check_state_touch_leds();

    extern "C" void nuos_zb_init_hardware(){
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
                .duty           = 0, // Set duty to 0%
                .hpoint         = 0
            };
            ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
        }
        
        #ifdef IS_USE_DALI_HARDWARE
            dali.begin();
        #endif
       
    }

    void init_dali_hw() {
        #ifdef IS_USE_DALI_HARDWARE
            dali.begin();
            vTaskDelay(100/ portTICK_PERIOD_MS);
        #endif
        is_init_done = true; 
        for(int i=0; i<TOTAL_BUTTONS; i++){
            printf("----------VALUE:%d", device_info[i].device_val);
            // #ifdef IS_USE_DALI_HARDWARE
            //     //nuos_dali_set_group_color_temperature(global_group_id[i], i, device_info[i].device_val);
            //     vTaskDelay(10/ portTICK_PERIOD_MS);
            //     dali.set_group_fade_rate(global_group_id[i], 12);
            //     vTaskDelay(10/ portTICK_PERIOD_MS);
            //     dali.set_group_fade_time(global_group_id[i], 1);  
            //     vTaskDelay(10/ portTICK_PERIOD_MS);
            // #endif
        }        
        // start_dali_addressing(3);   
        // uint8_t foundAddresses[64]; // DALI allows up to 64 short addresses (0-63)
        // uint8_t maxAddresses = 64;
        // printf("Discovering assigned DALI addresses:\n");
        // // Scan for assigned short addresses
        // int numFound = dali.scanAssignedShortAddresses(foundAddresses, maxAddresses);
        // printf("Found %d assigned DALI addresses:\n", numFound);
        // for (int i = 0; i < numFound; ++i) {
        //     printf("  Short address: %d\n", foundAddresses[i]>>1);
        // }
    }

  void nuos_zb_set_hardware(uint8_t index, uint8_t is_toggle){
        
    if(is_init_done){
        if(timer3_running_flag){
            //toggle pins on button press
            if(is_toggle>0) device_info[index].device_state = !device_info[index].device_state;
            //set led pins
            if(!device_info[index].device_state){
                
                #ifndef USE_INDIVIDUAL_DALI_ADDRESSING
                    printf("DALI Group OFF :%d\n", index);
                    dali.set_group_off(global_group_id[index]);
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], 0));
                    if(device_info[index].dim_up == 0) device_info[index].dim_up = 1;
                    else device_info[index].dim_up = 0;
                #else
                    printf("DALi ID=%d OFF\n", global_dali_id[index]);
                    dali.turn_off(global_dali_id[index]);
                #endif
            }else{
                #ifndef USE_INDIVIDUAL_DALI_ADDRESSING
                printf("DALi Group=%d ON at Level=%d\n", dali_nvs_stt[index].group_id, device_info[index].device_level);
                nuos_dali_set_group_brightness(dali_nvs_stt[index].group_id, index, device_info[index].device_level);
                //dali.set_group_on(dali_nvs_stt[index].group_id);
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], device_info[index].device_level));
                #else
                printf("DALi ID=%d ON at Level=%d\n", global_dali_id[index], device_info[index].device_level);
                dali.set_dim_value(global_dali_id[index], device_info[index].device_level);
                #endif
            }
            #ifndef USE_INDIVIDUAL_DALI_ADDRESSING
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[index]));
            #endif
        }else{
            if(nuos_check_state_touch_leds()){
                #ifndef USE_INDIVIDUAL_DALI_ADDRESSING
                for(int i=0; i<TOTAL_LEDS; i++){
                    if(device_info[i].device_state){
                        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], device_info[i].device_level));
                    }else{
                        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], 0));   
                    }
                    
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[i]));
                }
                #endif
            }else{
                //toggle pins on button press
                if(is_toggle>0) device_info[index].device_state = !device_info[index].device_state;
                //set led pins
                if(!device_info[index].device_state){
                    
                    #ifndef USE_INDIVIDUAL_DALI_ADDRESSING
                    printf("DALI Group OFF :%d\n", index);
                    dali.set_group_off(global_group_id[index]);
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], 0));
                    if(device_info[index].dim_up == 0) device_info[index].dim_up = 1;
                    else device_info[index].dim_up = 0;
                    #else
                    printf("DALi ID=%d OFF\n", global_dali_id[index]);
                    dali.turn_off(global_dali_id[index]);
                    #endif
                }else{
                    #ifndef USE_INDIVIDUAL_DALI_ADDRESSING
                    printf("DALi Group=%d ON at Level=%d\n", dali_nvs_stt[index].group_id, device_info[index].device_level);
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], device_info[index].device_level));
                    nuos_dali_set_group_brightness(dali_nvs_stt[index].group_id, index, device_info[index].device_level);
                    //dali.set_group_on(dali_nvs_stt[index].group_id);
                    //nuos_dali_set_group_color_temperature(dali_nvs_stt[index].group_id, index, device_info[index].device_val);                    
                    #else
                    printf("DALi ID=%d ON at Level=%d\n", global_dali_id[index], device_info[index].device_level);
                    dali.set_dim_value(global_dali_id[index], device_info[index].device_level);
                    #endif 
                }
                #ifndef USE_INDIVIDUAL_DALI_ADDRESSING
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[index]));    
                #endif           
            }
        }
        nuos_store_data_to_nvs(index);
    }
 }


    static bool state = false;
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
                    if(level > LEDC_MAX_DUTY) level = LEDC_MAX_DUTY;
                }
                #ifndef USE_INDIVIDUAL_DALI_ADDRESSING
                if(is_init_done){
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], level));
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[index]));
                }
                #endif

			}
		}
    }  


    uint8_t nuos_get_button_press_index(uint32_t pin){
        for(int index=0; index<TOTAL_BUTTONS; index++){
            if(pin == gpio_touch_btn_pins[index]){
                return index;
            }
        }
        return 0;
    }

    extern "C" void nuos_init_hardware_dimming_up_down(uint32_t pin){
        uint8_t index = nuos_get_button_press_index(pin);
        #ifndef USE_INDIVIDUAL_DALI_ADDRESSING
        if(device_info[index].device_state){
            if(device_info[index].dim_up == 0) device_info[index].dim_up = 1;
            else device_info[index].dim_up = 0;
        }
        #endif
    }

    
    extern "C" bool nuos_set_hardware_brightness(uint32_t pin){
        uint8_t index = nuos_get_button_press_index(pin);
        #ifndef USE_INDIVIDUAL_DALI_ADDRESSING
        if(device_info[index].device_state){
            if(device_info[index].dim_up == 1){
                if(device_info[index].device_level <= 255 - DIMMING_STEPS){
                    device_info[index].device_level += DIMMING_STEPS;
                } 
            }else{
                if(device_info[index].device_level >=DIMMING_STEPS) device_info[index].device_level -= DIMMING_STEPS;
            }
            if(device_info[index].device_level < MIN_DIM_LEVEL_VALUE) device_info[index].device_level = MIN_DIM_LEVEL_VALUE;
            if(is_init_done){
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], device_info[index].device_level));
                
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[index]));
            }
            //#ifdef IS_USE_DALI_HARDWARE
            nuos_dali_set_group_brightness(dali_nvs_stt[index].group_id, index, device_info[index].device_level);
            //#endif
        }
        nuos_store_data_to_nvs(index);
        #endif
        // if(wifi_webserver_active_flag == 0){
        //     nuos_set_zigbee_attribute(index);
        // }
        return false;
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
        else dali.set_group_on(group_id);
        //#endif
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

    //User can expand or lower this array according to number of Lights !!
    const uint8_t addresses[] = {11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30};    
    extern "C" void start_dali_addressing(int numAddresses)
    {                                                                                                                                                                                          
        printf("Initialize DALI nodes...");
        int total_found_nodes = dali.initNodes(addresses, numAddresses);
        printf("Found nodes:%d\n", total_found_nodes);
        for (int i = 0; i < total_found_nodes; ++i) {
            printf("Node number: %d, node address: %d\n", i, addresses[i]);
        }
    }

    extern "C" void nuos_dali_set_color_temperate(uint8_t index){
        
    }  

    extern "C" void nuos_set_state_touch_leds(bool state){
        if(is_init_done){
            for(int i=0; i<TOTAL_LEDS; i++){
                if(state){
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], 255));
                }else{
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], 0));
                }
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[i]));
            }
        }
    }

    extern "C" void nuos_set_state_touch_leds_to_original(){
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





