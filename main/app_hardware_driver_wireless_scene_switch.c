
#include "app_hardware_driver.h"
#include "esp_timer.h"
#include "app_zigbee_group_commands.h"

#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_GROUP_SWITCH)
    #include "driver/ledc.h"
    static bool set_color_temp_flag                                     = false;
    #define MAX_CCT_SCENES_VALUES                                       6
    uint16_t cct_values[MAX_CCT_SCENES_VALUES]                          = {0, MIN_CCT_VALUE, MIN_CCT_VALUE_1, MIN_CCT_VALUE_2, MIN_CCT_VALUE_3, MAX_CCT_VALUE};
    uint16_t cct_1000_values[MAX_CCT_SCENES_VALUES]                     = {0, 200, 400, 600, 750, 1000};
    #define DIMMING_STEPS                                              10
    static uint8_t level_backup[4]                                      = {0, 0, 0, 0};
    static uint16_t color_backup[4]                                     = {0, 0, 0, 0};
    #define CCT_CHANGE_STEPS                                            100

    bool change_color_temp_mode_level_or_color = false;
    uint32_t gpio_total_init_pins[TOTAL_LEDS]                   = {0, 0, 0, 0};
    extern unsigned int map_cct1(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);

    #define LEDC_TIMER              		                    LEDC_TIMER_0
    #define LEDC_MODE               		                    LEDC_LOW_SPEED_MODE

    #define LEDC_DUTY_RES          		 	                    LEDC_TIMER_8_BIT // Set duty resolution to 13 bits
    #define LEDC_DUTY               		                    100         // Set duty to 50%. ((2 ** 8) - 1) * 50% = 4095
    #define LEDC_MAX_DUTY           		                    254         // 8191
    #define LEDC_FREQUENCY          	                        1000        // Frequency in Hertz. Set frequency at 1 KHz
    #define LEDC_FADE_TIME                                      50        // Fade time in milliseconds (1 second)


    uint8_t pwm_channels[TOTAL_LEDS]                            = { LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    bool is_init_done                                           = false;
    

    
    void init_hw(){
        for(int index=0; index<TOTAL_ENDPOINTS; index++){
            device_info[index].device_level = 255;
            device_info[index].device_val = 2000;
            device_info[index].device_state = true;
       
            for(int i=0; i<6; i++){
                cct_1000_values[i] = map_cct1(cct_values[i], MIN_CCT_VALUE, MAX_CCT_VALUE, 0, 1000);
            }
        }
        
    }

    void nuos_zb_init_hardware(){

        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_GROUP_SWITCH)
        init_hw();       
        // Prepare and then apply the LEDC PWM timer configuration
        ledc_timer_config_t ledc_timer = {
            .speed_mode       = LEDC_MODE,
            .timer_num        = LEDC_TIMER,
            .duty_resolution  = LEDC_DUTY_RES,
            .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 100Hz
            .clk_cfg          = LEDC_AUTO_CLK
        };
        ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

        for(int index=0; index<TOTAL_LEDS; index++){
            // Prepare and then apply the LEDC PWM channel configuration
            ledc_channel_config_t ledc_channel = {
                .speed_mode     = LEDC_MODE,
                .channel        = pwm_channels[index],
                .timer_sel      = LEDC_TIMER_0,
                .intr_type      = LEDC_INTR_DISABLE,
                .gpio_num       = gpio_touch_led_pins[index],
                .duty           = 0, // Set duty to 0%
                .hpoint         = 0
            };
            ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
        }
        
        #ifdef BEHAVE_AS_DEVICE_NOT_REMOTE
        for(int j=0; j<TOTAL_LOADS; j++){
            gpio_reset_pin(gpio_load_pins[j]);
            /* Set the GPIO as a push/pull output */
            gpio_set_direction(gpio_load_pins[j], GPIO_MODE_OUTPUT);
            gpio_set_level(gpio_load_pins[j], 0);
        }  
        #endif
        #else
            for(int index=0; index<TOTAL_ENDPOINTS; index++){
                gpio_reset_pin(gpio_touch_led_pins[index]);
                /* Set the GPIO as a push/pull output */
                gpio_set_direction(gpio_touch_led_pins[index], GPIO_MODE_OUTPUT);
                gpio_set_level(gpio_touch_led_pins[index], 0);

                gpio_reset_pin(gpio_load_pins[index]);
                /* Set the GPIO as a push/pull output */
                gpio_set_direction(gpio_load_pins[index], GPIO_MODE_OUTPUT);
                gpio_set_level(gpio_load_pins[index], 0);
            }

        #endif
        is_init_done = true;
        // printf("is_init_done:%d\n", is_init_done);                   
    }

    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_GROUP_SWITCH)
    void nuos_set_led_level(uint8_t index, uint8_t _level) {
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], _level));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[index]));
    }

    void nuos_init_hardware_dimming_up_down(uint32_t pin) {
        uint8_t index = nuos_get_button_press_index(pin);
        if(index == 3){
            if(!change_color_temp_mode_level_or_color){
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
        }else{
            if(device_info[index].device_level <= MIN_DIM_LEVEL_VALUE){
                device_info[index].dim_up = 1;
            }else if(device_info[index].device_level >= MAX_DIM_LEVEL_VALUE){
                device_info[index].dim_up = 0;
            }           
        }
    }
    
    bool inc_dec_level_value(uint8_t index){
        gpio_set_level(gpio_touch_led_pins[2], 1);
        gpio_set_level(gpio_touch_led_pins[1], 1);
        if(device_info[index].device_level < MIN_DIM_LEVEL_VALUE){
            device_info[index].device_level = MIN_DIM_LEVEL_VALUE;
        }
        if(device_info[index].dim_up == 1){
            if(device_info[index].device_level + DIMMING_STEPS <= (MAX_DIM_LEVEL_VALUE)){
                device_info[index].device_level += DIMMING_STEPS;
            } else {
                device_info[index].device_level = MAX_DIM_LEVEL_VALUE;
                gpio_set_level(gpio_touch_led_pins[2], 0);
                gpio_set_level(gpio_touch_led_pins[1], 1);
            }
        }else{
            if(device_info[index].device_level - DIMMING_STEPS >= MIN_DIM_LEVEL_VALUE){
                device_info[index].device_level -= DIMMING_STEPS;  
            }else {
                device_info[index].device_level = MIN_DIM_LEVEL_VALUE;
                gpio_set_level(gpio_touch_led_pins[2], 1);
                gpio_set_level(gpio_touch_led_pins[1], 0);
            } 
        }
        return false;
    }
    bool inc_dec_cct_value(uint8_t index){
        if(device_info[index].dim_up == 1){
            if(device_info[index].device_val + CCT_CHANGE_STEPS <= MAX_CCT_VALUE){
                device_info[index].device_val += CCT_CHANGE_STEPS;
            } else {
                device_info[index].device_val = MAX_CCT_VALUE;
            }
        }else{
            if(device_info[index].device_val - CCT_CHANGE_STEPS >= MIN_CCT_VALUE){
                device_info[index].device_val -= CCT_CHANGE_STEPS;  
            }else {
                device_info[index].device_val = MIN_CCT_VALUE;
            } 
        }
        if(color_backup[index] != device_info[index].device_val){
            color_backup[index] = device_info[index].device_val;
            return true;
        }   
        return false;              
    }

    void set_color_temp_leds(uint8_t index){
        if(device_info[0].device_state){
            nuos_set_led_level(3, MAX_DIM_LEVEL_VALUE); 
            nuos_set_led_level(1, MAX_DIM_LEVEL_VALUE); 
            if(set_color_temp_flag){
                if(index == 1) {
                    printf("fan_speed1:%d\n", device_info[0].fan_speed);
                    if(device_info[0].fan_speed < MAX_CCT_SCENES_VALUES-2) device_info[0].fan_speed++;
                    else {
                        device_info[0].fan_speed = MAX_CCT_SCENES_VALUES-1;
                        nuos_set_led_level(3, MAX_DIM_LEVEL_VALUE); 
                        nuos_set_led_level(1, 0); 
                    } 
                    printf("fan_speed2:%d\n", device_info[0].fan_speed);
                    device_info[0].device_val = cct_values[device_info[0].fan_speed];
                    printf("cct_temp:%d\n", device_info[0].device_val);
                }else if(index == 3) {
                    if(device_info[0].fan_speed > 2) device_info[0].fan_speed--;
                    else {
                        device_info[0].fan_speed = 1;
                        nuos_set_led_level(3, 0); 
                        nuos_set_led_level(1, MAX_DIM_LEVEL_VALUE);
                    }
                    device_info[0].device_val = cct_values[device_info[0].fan_speed];
                }
            }else{
                if (index == 1) {
                    // Increase brightness
                    if (device_info[0].device_level < MAX_DIM_LEVEL_VALUE - DIMMING_STEPS) {
                        device_info[0].device_level += DIMMING_STEPS;
                    } else {
                        device_info[0].device_level = MAX_DIM_LEVEL_VALUE;
                    }                    
                } else if (index == 3) {
                    // Decrease brightness
                    if (device_info[0].device_level > MIN_DIM_LEVEL_VALUE + DIMMING_STEPS) {
                        device_info[0].device_level -= DIMMING_STEPS;
                    } else {
                        device_info[0].device_level = MIN_DIM_LEVEL_VALUE;
                    }
                }

                // Set LED levels based on the current dim level
                if (device_info[0].device_level >= MAX_DIM_LEVEL_VALUE) {
                    nuos_set_led_level(3, MAX_DIM_LEVEL_VALUE);  // ON
                    nuos_set_led_level(1, 0);                    // OFF
                } else if (device_info[0].device_level <= MIN_DIM_LEVEL_VALUE) {
                    nuos_set_led_level(3, 0);                    // OFF
                    nuos_set_led_level(1, MAX_DIM_LEVEL_VALUE);  // ON
                } else {
                    nuos_set_led_level(3, MAX_DIM_LEVEL_VALUE);  // ON
                    nuos_set_led_level(1, MAX_DIM_LEVEL_VALUE);  // ON
                }
            }
            nuos_store_data_to_nvs(0);
        } else{
            nuos_set_led_level(3, 0); 
            nuos_set_led_level(1, 0);
        }  
    }

    uint8_t set_color_temp_only_leds(uint8_t index){
        if(device_info[0].device_state){
            if(set_color_temp_flag){
                if(device_info[0].fan_speed > 1 && device_info[0].fan_speed < MAX_CCT_SCENES_VALUES-1){
                    nuos_set_led_level(3, MAX_DIM_LEVEL_VALUE); 
                    nuos_set_led_level(1, MAX_DIM_LEVEL_VALUE); 
                    return 2;
                }else{
                    if(device_info[0].fan_speed >= MAX_CCT_SCENES_VALUES-1) {
                        nuos_set_led_level(3, MAX_DIM_LEVEL_VALUE); 
                        nuos_set_led_level(1, 0); 
                        return 3;                    
                    } else {
                        nuos_set_led_level(3, 0); 
                        nuos_set_led_level(1, MAX_DIM_LEVEL_VALUE);
                        return 1;                     
                    }
                }
            }else{
                if(device_info[0].device_level > MIN_DIM_LEVEL_VALUE && device_info[0].device_level < MAX_DIM_LEVEL_VALUE){
                    nuos_set_led_level(3, MAX_DIM_LEVEL_VALUE); 
                    nuos_set_led_level(1, MAX_DIM_LEVEL_VALUE); 
                    return 2;
                }else{
                    if(device_info[0].device_level >= MAX_DIM_LEVEL_VALUE) {
                        nuos_set_led_level(3, MAX_DIM_LEVEL_VALUE); 
                        nuos_set_led_level(1, 0); 
                        return 3;                    
                    } else {
                        nuos_set_led_level(3, 0); 
                        nuos_set_led_level(1, MAX_DIM_LEVEL_VALUE);
                        return 1;                     
                    }
                }                
            }
        }  else{
            nuos_set_led_level(3, 0); 
            nuos_set_led_level(1, 0);
            return 0; 
        }  
        return 0;          
    }
    #endif
    void nuos_zb_set_hardware(uint8_t index, uint8_t is_toggle) {

        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_SCENE_SWITCH)
        for(int i=0; i<TOTAL_ENDPOINTS; i++){
            if(i == index) {
                device_info[i].device_state = true;
                gpio_set_level(gpio_touch_led_pins[i], device_info[i].device_state);
            }else{
                device_info[i].device_state = false;
                gpio_set_level(gpio_touch_led_pins[i], device_info[i].device_state);
            }
            nuos_store_data_to_nvs(i);
        }
        #else
            if(index == 0){
                if(is_toggle>0) device_info[index].device_state = !device_info[index].device_state;
                nuos_zb_group_send_light_onoff_groupcast(zb_scene_info[index].group_id, device_info[index].device_state);
                if(!device_info[index].device_state){
                    nuos_set_led_level(0, 0); 
                    nuos_set_led_level(1, 0);
                    nuos_set_led_level(2, 0);
                    nuos_set_led_level(3, 0);
                }else{
                    nuos_set_led_level(0, device_info[index].device_level); 
                    if(set_color_temp_flag){
                        nuos_set_led_level(2, MAX_DIM_LEVEL_VALUE);
                    }else{
                        nuos_set_led_level(2, 0);
                    }
                    set_color_temp_only_leds(0);             
                    
                } 
            }else if(index == 2){
                // Do Nothing
                printf("Nothing to do!!\n");
                set_color_temp_flag = !set_color_temp_flag;
                if(set_color_temp_flag){
                    nuos_set_led_level(2, MAX_DIM_LEVEL_VALUE);
                }else{
                    nuos_set_led_level(2, 0);
                }
                set_color_temp_only_leds(0);
            }else{
                if(device_info[0].device_state){
                    set_color_temp_leds(index);
                    if(set_color_temp_flag){
                        printf("COLOR TEMP: %d \n", device_info[0].device_val); 
                        zigbee_cluster_color_temperature_update(zb_scene_info[0].group_id, device_info[0].device_val);
                    }else{
                        printf("LEVEL: %d \n", device_info[0].device_level); 
                        nuos_set_led_level(0, device_info[0].device_level);
                        nuos_zb_group_send_light_level_groupcast(zb_scene_info[0].group_id, device_info[0].device_level); 
                    }
                }
            } 
            nuos_store_data_to_nvs(0);             

        #endif
        //0x0 0x0 0x0 0x1 0x0 0x0 0x0 0x5 0x0 0x0 0x0 0x5 0x0 0x0 0x0 0x0 0x0
    }

    static bool state = false;
    void nuos_zb_set_hardware_led_for_zb_commissioning(uint8_t is_toggle){
        if(TOTAL_LEDS > TOTAL_LEDS_SHOW_ON_COMMISSIONING){
            if(is_toggle>0) {
                state = !state;
                for(int index=TOTAL_LEDS-TOTAL_LEDS_SHOW_ON_COMMISSIONING; index<TOTAL_LEDS; index++){
                    gpio_set_level(gpio_touch_led_pins[index], state);
                }
            }else{
                for(int i=0; i<TOTAL_LEDS; i++){
                    device_info[i].device_state = 0; 
                    nuos_zb_set_hardware(i, false);
                }
            }
		}
    }
    void nuos_zb_set_scene_switch_click(uint32_t io_num, uint8_t state){
        int index = nuos_get_button_press_index(io_num);
        //set touch led pins
        #ifdef NUOS_TYPE_LED_INDICATION
            for(int i=0; i<TOTAL_ENDPOINTS; i++){
                gpio_set_level(gpio_touch_led_pins[i], 0); 
                if(state>0){
                    if(i == e_index) {
                        gpio_set_level(gpio_touch_led_pins[i], 1);  
                    }
                }           
            }
        #else 
            nuos_zb_set_hardware(index, false) ; 
        #endif 

    }


    void nuos_on_off_led(uint8_t index, uint8_t _state){
        gpio_set_level(gpio_touch_led_pins[index], _state);  
    }

    void nuos_toggle_leds(uint8_t index){
        state = !state;        
        nuos_on_off_led(index, state);
    }

    void nuos_set_state_touch_leds(bool state){
        //no need to do this work in case of wireless scene switch
    }  
    
    bool nuos_set_hardware_brightness(uint32_t pin){
        uint8_t index = nuos_get_button_press_index(pin);
        if(global_switch_state == SWITCH_PRESS_DETECTED){ 
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
                nuos_zb_group_send_light_level_groupcast(zb_scene_info[index].group_id, device_info[index].device_level);           
            }
        }
        return false;
    }    
 #endif