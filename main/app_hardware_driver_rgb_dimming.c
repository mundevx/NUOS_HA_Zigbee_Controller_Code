
 #include "app_hardware_driver.h"
 #include "app_zigbee_clusters.h"
#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4T_COLOR_DIMMABLE_LIGHT)
    #include "esp_log.h"
    #include "esp_err.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include <esp_check.h>
    #include <esp_log.h>
    #include "app_dali_commands.h"
    #include "esp_zigbee_core.h"
    #include "light_driver.h"
    #include "driver/ledc.h"

    extern  uint8_t dali_found_nodes_array[TOTAL_ENDPOINTS];
    extern	uint16_t dali_found_nodes_length;

	static const char *TAGX = "ESP_DALI";

	#define DALI_RX      							GPIO_NUM_2//12
	#define DALI_TX      							GPIO_NUM_3//13

    #define  RGB_DALI_ADDRESS                       1

    #define NO_CHANGE                               255
    #define MAX_VALUE                               20
    #define MIN_VALUE                               0
    //inc or dec speed
    #define DIMMING_STEPS                           20

    void check_all_rgb_off();




    #define LEDC_TIMER              		    LEDC_TIMER_0
    #define LEDC_MODE               		    LEDC_LOW_SPEED_MODE

    #define LEDC_DUTY_RES          		 	    LEDC_TIMER_8_BIT // Set duty resolution to 13 bits
    #define LEDC_DUTY               		    10      // Set duty to 50%. ((2 ** 8) - 1) * 50% = 4095
    #define LEDC_MAX_DUTY           		    254     //8191
    #define LEDC_FREQUENCY          	        1000    // Frequency in Hertz. Set frequency at 1 KHz

    uint8_t pwm_channels[TOTAL_ENDPOINTS]       = { LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3};


    void nuos_zb_init_hardware(){
        // Prepare and then apply the LEDC PWM timer configuration
        ledc_timer_config_t ledc_timer = {
            .speed_mode       = LEDC_MODE,
            .timer_num        = LEDC_TIMER,
            .duty_resolution  = LEDC_DUTY_RES,
            .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 100Hz
            .clk_cfg          = LEDC_AUTO_CLK
        };
        ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

        for(int index=0; index<TOTAL_ENDPOINTS; index++){
            // Prepare and then apply the LEDC PWM channel configuration
            ledc_channel_config_t ledc_channel = {
                .speed_mode     = LEDC_MODE,
                .channel        = pwm_channels[index],
                .timer_sel      = LEDC_TIMER,
                .intr_type      = LEDC_INTR_DISABLE,
                .gpio_num       = gpio_touch_led_pins[index],
                .duty           = device_info[index].device_level, // Set duty to 0%
                .hpoint         = 0
            };
            ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
        }
    }

    void init_dali_hw(){
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
            nuos_zb_set_hardware(i, 0); 
        }    
        is_init_done = true;    
        //start_dali_addressing(3);
    }

    void check_all_rgb_off(){
        if(device_info[0].device_val == MIN_VALUE && device_info[1].device_val == MIN_VALUE && device_info[2].device_val == MIN_VALUE){
            device_info[0].device_val = MIN_VALUE;
            device_info[1].device_val = MIN_VALUE;
            device_info[2].device_val = MIN_VALUE;
            device_info[3].device_val = MIN_VALUE;
            //dali_set_value(RGB_DALI_ADDRESS, 0);
            dali_set_color_rgb(RGB_DALI_ADDRESS, device_info[0].device_val, device_info[1].device_val, device_info[2].device_val, device_info[3].device_val);  
        }
    }
    void nuos_zb_set_hardware(uint8_t index, uint8_t is_toggle){
        //toggle pins on button press
        if(is_toggle>0) device_info[index].device_state = !device_info[index].device_state;
        //set touch led pins
        dali_set_value(RGB_DALI_ADDRESS, device_info[index].device_level);
        gpio_set_level(gpio_touch_led_pins[index], device_info[index].device_state);
        switch(index){
            case 0:
                if(!device_info[index].device_state){  //off
                    //check_all_rgb_off();
                    dali_set_color_rgb(RGB_DALI_ADDRESS, MIN_VALUE, device_info[1].device_val, NO_CHANGE, NO_CHANGE);
                }else{
                    dali_set_color_rgb(RGB_DALI_ADDRESS, device_info[0].device_val, NO_CHANGE, NO_CHANGE, NO_CHANGE);
                }       
            break;

            case 1:
                if(!device_info[index].device_state){
                    //check_all_rgb_off();
                    dali_set_color_rgb(RGB_DALI_ADDRESS, device_info[0].device_val, MIN_VALUE, NO_CHANGE, NO_CHANGE); 
                }else{
                    dali_set_color_rgb(RGB_DALI_ADDRESS, NO_CHANGE, device_info[1].device_val, NO_CHANGE, NO_CHANGE);
                }
            break;

            case 2:
                if(!device_info[index].device_state){
                   // check_all_rgb_off();
                    dali_set_color_rgb(RGB_DALI_ADDRESS, NO_CHANGE, NO_CHANGE, MIN_VALUE, NO_CHANGE);
                }else{
                    dali_set_color_rgb(RGB_DALI_ADDRESS, NO_CHANGE, NO_CHANGE, device_info[2].device_val, NO_CHANGE);
                }
            break;

            case 3:
                if(!device_info[index].device_state){
                   // check_all_rgb_off();
                    dali_set_color_rgb(RGB_DALI_ADDRESS, NO_CHANGE, NO_CHANGE, NO_CHANGE, MIN_VALUE); 
                }else{
                    dali_set_color_rgb(RGB_DALI_ADDRESS, NO_CHANGE, NO_CHANGE, NO_CHANGE, device_info[3].device_val);
                }
            break;
            default: break;
        }


        if(!device_info[index].device_state){
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], 0));
        }else{
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], device_info[index].device_level));
        }
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[index]));
        nuos_store_data_to_nvs(index);
    }


    void nuos_init_hardware_dimming_up_down(uint32_t pin){
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4T_COLOR_DIMMABLE_LIGHT)
            uint8_t index = nuos_get_button_press_index(pin);
            if(device_info[index].dim_up == 0) device_info[index].dim_up = 1;
            else device_info[index].dim_up = 0;
        #endif
    }


    bool nuos_set_hardware_brightness(uint32_t pin){
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4T_COLOR_DIMMABLE_LIGHT)
            uint8_t index = nuos_get_button_press_index(pin);
            if(device_info[index].device_state){
                if(device_info[index].dim_up == 1){
                    if(device_info[index].device_level <= 255 - DIMMING_STEPS){
                        device_info[index].device_level += DIMMING_STEPS;
                    } 
                }else{  
                    if(device_info[index].device_level >=DIMMING_STEPS) device_info[index].device_level -= DIMMING_STEPS;
                }
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], device_info[index].device_level));
            }
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[index]));
            nuos_store_data_to_nvs(index);

            nuos_set_zigbee_attribute(index);
            return false;
        #else
            return true;    
        #endif
    }

    void nuos_set_level_to_rgb(uint8_t index){
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4T_COLOR_DIMMABLE_LIGHT)
            switch(index){
                case 0:
                    device_info[0].device_val = (uint8_t)device_info[index].device_level;
                    device_info[1].device_val = NO_CHANGE;
                    device_info[2].device_val = NO_CHANGE;
                    device_info[3].device_val = NO_CHANGE;
                break;
                case 1:
                    device_info[0].device_val = NO_CHANGE;
                    device_info[1].device_val = (uint8_t)device_info[index].device_level;
                    device_info[2].device_val = NO_CHANGE;
                    device_info[3].device_val = NO_CHANGE;
                break;
                case 2:
                    device_info[0].device_val = NO_CHANGE;
                    device_info[1].device_val = NO_CHANGE;
                    device_info[2].device_val = (uint8_t)device_info[index].device_level;
                    device_info[3].device_val = NO_CHANGE;
                break;
                case 3:
                    device_info[0].device_val = NO_CHANGE;
                    device_info[1].device_val = NO_CHANGE;
                    device_info[2].device_val = NO_CHANGE;
                    device_info[3].device_val = (uint8_t)device_info[index].device_level;
                break;  
                default: break;                                                                      
            }
        #endif 
    }

    void nuos_set_state_touch_leds(bool state){

    }
#endif

    void nuos_zb_convert_xy_to_rgb(uint8_t index, float red_f, float blue_f, float green_f){

        //float ratio = (float)device_info[index].device_level / 255;
        switch(index){
            case 0:
                device_info[0].device_val = (uint8_t)(red_f * (float)255);
                device_info[1].device_val = 255;
                device_info[2].device_val = 255;
                device_info[3].device_val = 255;
            break;
            case 1:
                device_info[0].device_val = 255;
                device_info[1].device_val = (uint8_t)(blue_f * (float)255);
                device_info[2].device_val = 255;
                device_info[3].device_val = 255;
            break;
            case 2:
                device_info[0].device_val = 255;
                device_info[1].device_val = 255;
                device_info[2].device_val = (uint8_t)(green_f * (float)255);
                device_info[3].device_val = 255;
            break;
            default: break;
            // case 3:
            //     device_info[0].device_val = 255;
            //     device_info[1].device_val = 255;
            //     device_info[2].device_val = 255;
            //     device_info[3].device_val = (uint8_t)(white_f * (float)255);
            // break;                                                                        
        }
    }



