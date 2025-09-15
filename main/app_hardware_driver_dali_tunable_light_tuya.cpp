
#include "app_hardware_driver.h"
#include "app_zigbee_clusters.h"

#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)

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
    #define LEDC_MAX_DUTY           		                    254     // 8191
    #define LEDC_FREQUENCY          	                        1000    // Frequency in Hertz. Set frequency at 1 KHz


    #if (TOTAL_ENDPOINTS == 4)
        ledc_channel_t pwm_channels[TOTAL_ENDPOINTS]    = { LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3};  
    #elif (TOTAL_ENDPOINTS == 3)
        ledc_channel_t pwm_channels[TOTAL_ENDPOINTS]    = { LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2};
    #elif (TOTAL_ENDPOINTS == 2)
        ledc_channel_t pwm_channels[TOTAL_ENDPOINTS]    = { LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    #elif (TOTAL_ENDPOINTS == 1)
        ledc_channel_t pwm_channels[TOTAL_ENDPOINTS]    = { LEDC_CHANNEL_0};
    #else
        ledc_channel_t pwm_channels[1]    = { LEDC_CHANNEL_0};
    #endif
    bool is_init_done = false;
    // #define IS_USE_DALI_HARDWARE

    extern "C" void nuos_set_state_touch_leds(bool state);
    extern "C" bool nuos_check_state_touch_leds();
    // extern "C" void start_color_temp_timer();
    // extern "C" void stop_color_temp_timer();
    
    static bool state = false;
    uint8_t level = 0;

    #define MAX_CCT_SCENES_VALUES    6

    uint16_t cct_values[MAX_CCT_SCENES_VALUES] = {0, MIN_CCT_VALUE, MIN_CCT_VALUE_1, MIN_CCT_VALUE_2, MIN_CCT_VALUE_3, MAX_CCT_VALUE};

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
        is_init_done = true;
    }


    static void dali_addr_task(void *pvParameters){
        start_dali_addressing(3);
        vTaskDelete(NULL);
    }

    static uint8_t last_dali_level = 0; 
    static uint16_t last_color_temp = 0;

    // DALI limits
#define DALI_MIN MIN_DIM_LEVEL_VALUE
#define DALI_MAX MAX_DIM_LEVEL_VALUE
#define CALIBRATION_GAMMA 0.5  // Tune as needed

extern "C" uint8_t calibrate_dali_brightness(uint8_t input_value) {
    if (input_value <= 0) return DALI_MIN; 
    if (input_value >= 254) return DALI_MAX;

    // Normalize input to 0..1
    double normalized = input_value / 254.0;

    // Apply gamma correction
    double calibrated = pow(normalized, CALIBRATION_GAMMA);

    // Scale to DALI range
    uint8_t dali_value = (uint8_t)(DALI_MIN + calibrated * (DALI_MAX - DALI_MIN) + 0.5);

    return dali_value;
}

    extern "C" void set_state(uint8_t index){
        esp_zb_lock_acquire(portMAX_DELAY);
        if(!device_info[index].device_state) {
            
            #if(COMMUNICATION_MODE == COMM_MODE_ADDR_CTRL)
            dali.turn_off(global_dali_id[0]); 
            #elif(COMMUNICATION_MODE == COMM_MODE_GROUP_CTRL)
            dali.set_group_off(global_group_id[0]); 
            #elif(COMMUNICATION_MODE == COMM_MODE_BROADCAST)
            dali.send_broadcast(dali.OFF_C); 
            #endif  
        } else {
            //dali.send_broadcast(dali.ON_C);
            #if(COMMUNICATION_MODE == COMM_MODE_ADDR_CTRL)
            // dali.turn_on_to_last_level(global_dali_id[0]);
            // vTaskDelay(5 / portTICK_PERIOD_MS);
            dali.set_dim_value(global_dali_id[0], calibrate_dali_brightness(device_info[index].device_level));
            #elif(COMMUNICATION_MODE == COMM_MODE_GROUP_CTRL)
            dali.set_group_level(global_group_id[0], calibrate_dali_brightness(device_info[index].device_level));
            vTaskDelay(5 / portTICK_PERIOD_MS);
            dali.set_group_level(global_group_id[0], calibrate_dali_brightness(device_info[index].device_level));
            #elif(COMMUNICATION_MODE == COMM_MODE_BROADCAST)
            uint8_t cal_val = calibrate_dali_brightness(device_info[index].device_level);
            dali.set_broadcast_level(cal_val);  
            vTaskDelay(5 / portTICK_PERIOD_MS);
            cal_val = calibrate_dali_brightness(device_info[index].device_level);
            dali.set_broadcast_level(cal_val);  

            // dali.set_broadcast_level(calibrate_dali_brightness(device_info[index].device_level));
            // vTaskDelay(5 / portTICK_PERIOD_MS);
            // dali.set_broadcast_level(calibrate_dali_brightness(device_info[index].device_level));
            #endif   
        }
        esp_zb_lock_release();
        nuos_set_state_attribute(0);
        //
    }
    extern "C" void set_level(uint8_t index){
            esp_zb_lock_acquire(portMAX_DELAY);
            #if(COMMUNICATION_MODE == COMM_MODE_ADDR_CTRL)
            // dali.turn_on_to_last_level(global_dali_id[0]);
            // vTaskDelay(5 / portTICK_PERIOD_MS);
            dali.set_dim_value(global_dali_id[0], device_info[index].device_level);
            #elif(COMMUNICATION_MODE == COMM_MODE_GROUP_CTRL)
            dali.set_group_level(global_group_id[0], device_info[index].device_level);
            #elif(COMMUNICATION_MODE == COMM_MODE_BROADCAST)
            //esp_zb_lock_acquire(portMAX_DELAY);
            uint8_t cal_val = calibrate_dali_brightness(device_info[index].device_level);
            printf("cal_val:%d\n", cal_val);
            dali.set_broadcast_level(cal_val);        
            //esp_zb_lock_release(); 
            #endif 
            esp_zb_lock_release();
            nuos_set_color_temp_level_attribute(0); 

    }
    extern "C" void set_color_temp(){
            #if(COMMUNICATION_MODE == COMM_MODE_ADDR_CTRL)
            dali.set_color_temperature(global_dali_id[0], device_info[0].device_val); 
            #elif(COMMUNICATION_MODE == COMM_MODE_GROUP_CTRL)
            dali.set_color_temperature(global_group_id[0], device_info[0].device_val);
            #elif(COMMUNICATION_MODE == COMM_MODE_BROADCAST)
            //dali.set_broadcast_color_rgb(0, 0, 0, device_info[0].device_level);
            dali.set_color_temperature(dali.BROADCAST_C, device_info[0].device_val); 
            #endif 

            nuos_set_color_temperature_attribute(0);     
    }
    extern "C" void set_color_temp_args(uint16_t color){
            #if(COMMUNICATION_MODE == COMM_MODE_ADDR_CTRL)
            dali.set_color_temperature(global_dali_id[0], device_info[0].device_val); 
            #elif(COMMUNICATION_MODE == COMM_MODE_GROUP_CTRL)
            dali.set_color_temperature(global_group_id[0], device_info[0].device_val);
            #elif(COMMUNICATION_MODE == COMM_MODE_BROADCAST)
            dali.set_color_temperature(dali.BROADCAST_C, color); 
            #endif 

            nuos_set_color_temperature_attribute(0);     
    }
    static void update_duty_cycle_task(void *pvParameters){
        uint8_t index = *(uint8_t*)pvParameters;
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[index]));
        vTaskDelete(NULL);
    } 


    void set_dali_fading(){

        dali.set_broadcast_fade_rate(12);
        vTaskDelay(20/ portTICK_PERIOD_MS); 
        dali.set_broadcast_fade_time(2);
        vTaskDelay(20 / portTICK_PERIOD_MS);              
        //dali.set_ext_fade_time(dali.BROADCAST_C, 1);
        //vTaskDelay(20 / portTICK_PERIOD_MS);
  
    }

    extern "C" void init_dali_hw(){
        dali.begin();
        // taskYIELD(); 
        // set_dali_fading(); 
        // taskYIELD();      
    }

    void set_color_temp_only_leds_2(){
        #ifdef USE_CCT_TIME_SYNC
        if(device_info[0].device_state){
        #else
        if(device_info[0].device_state && device_info[0].color_or_fan_state){
        #endif
        
            if(device_info[0].device_level >= MIN_DIM_LEVEL_VALUE){
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[0], device_info[0].device_level));
            }else{
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[0], MIN_DIM_LEVEL_VALUE));                   
            }
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[0]));
            #ifdef USE_CCT_TIME_SYNC
            #else
            gpio_set_level(gpio_touch_led_pins[1], 1);
            #endif
            if(device_info[0].fan_speed > 1 && device_info[0].fan_speed < MAX_CCT_SCENES_VALUES-1){
                gpio_set_level(gpio_touch_led_pins[3], 1);
                gpio_set_level(gpio_touch_led_pins[2], 1);
            }else{
                if(device_info[0].fan_speed >= MAX_CCT_SCENES_VALUES-1) {
                    gpio_set_level(gpio_touch_led_pins[3], 1);
                    gpio_set_level(gpio_touch_led_pins[2], 0);
                } else {
                    gpio_set_level(gpio_touch_led_pins[2], 1);
                    gpio_set_level(gpio_touch_led_pins[3], 0);
                }
            }
        }  else{
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[0], 0));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[0])); 
            #ifdef USE_CCT_TIME_SYNC
            #else             
            gpio_set_level(gpio_touch_led_pins[1], 0);
            #endif
            gpio_set_level(gpio_touch_led_pins[3], 0);
            gpio_set_level(gpio_touch_led_pins[2], 0);
        }            

    }

    void set_color_temp_leds(uint8_t index){
        #ifdef USE_CCT_TIME_SYNC
        if(device_info[0].device_state){
        #else
        if(device_info[0].device_state && device_info[0].color_or_fan_state){
        #endif
            #ifdef USE_CCT_TIME_SYNC
            #else
            gpio_set_level(gpio_touch_led_pins[1], 1);
            #endif
            gpio_set_level(gpio_touch_led_pins[3], 1);
            gpio_set_level(gpio_touch_led_pins[2], 1);
            if(index == 2) {
                if(device_info[0].fan_speed < MAX_CCT_SCENES_VALUES-2) device_info[0].fan_speed++;
                else {
                    device_info[0].fan_speed = MAX_CCT_SCENES_VALUES-1;
                    gpio_set_level(gpio_touch_led_pins[3], 1);
                    gpio_set_level(gpio_touch_led_pins[2], 0);
                } 
                device_info[0].device_val = cct_values[device_info[0].fan_speed];
            }else if(index == 3) {
                if(device_info[0].fan_speed > 2) device_info[0].fan_speed--;
                else {
                    device_info[0].fan_speed = 1;
                    gpio_set_level(gpio_touch_led_pins[2], 1);
                    gpio_set_level(gpio_touch_led_pins[3], 0);
                }
                device_info[0].device_val = cct_values[device_info[0].fan_speed];
            }
            nuos_store_data_to_nvs(0);
        } else{
            #ifdef USE_CCT_TIME_SYNC
            #else
            gpio_set_level(gpio_touch_led_pins[1], 0);
            #endif
            gpio_set_level(gpio_touch_led_pins[3], 0);
            gpio_set_level(gpio_touch_led_pins[2], 0);
        }
        
    }

    void set_hardware(uint8_t index, uint8_t is_toggle){
        if(index == 0){
            if(is_toggle>0) device_info[0].device_state = !device_info[0].device_state;
            device_info[0].color_or_fan_state = false;
            gpio_set_level(gpio_touch_led_pins[1], device_info[0].color_or_fan_state);
            if(!device_info[0].device_state){
                printf("Turn OFF\n");
                #ifdef USE_CCT_TIME_SYNC
                gpio_set_level(gpio_touch_led_pins[1], 0);
                #else
                device_info[0].color_or_fan_state = false;
                gpio_set_level(gpio_touch_led_pins[1], device_info[0].color_or_fan_state);
                #endif
                //xTaskCreate(group_off_task, "off_task", 2048, NULL, 14, NULL);
                // esp_zb_lock_acquire(portMAX_DELAY);
                // dali.send_broadcast(dali.OFF_C);
                // esp_zb_lock_release();
            }else{
                printf("Turn ON\n");

                //xTaskCreate(update_duty_cycle_task, "update_duty_cycle_task", 2048, &index, 28, NULL); 
                
                #ifdef USE_CCT_TIME_SYNC
                gpio_set_level(gpio_touch_led_pins[1], device_info[0].color_or_fan_state);
                #else
                device_info[0].color_or_fan_state = true;
                gpio_set_level(gpio_touch_led_pins[1], device_info[0].color_or_fan_state);
                #endif

            } 
            set_color_temp_only_leds_2();
            // #ifdef USE_RGB_LED
            //     nuos_rgb_trigger_blink();
            // #endif    
        }else if(index == 1){
            #ifdef USE_CCT_TIME_SYNC
            if(device_info[0].device_state){
                device_info[0].color_or_fan_state = !device_info[0].color_or_fan_state;
                // if(!device_info[0].color_or_fan_state) { set_cct_time_sync_request_flag = false;}
                // else { set_cct_time_sync_request_flag = true;}
                gpio_set_level(gpio_touch_led_pins[1], device_info[0].color_or_fan_state);
            }
            #endif
        } else {
            #ifdef USE_CCT_TIME_SYNC
                set_color_temp_leds(index);
            #else
                if(device_info[0].color_or_fan_state)
                    set_color_temp_leds(index);
            #endif    
        }
        // #ifdef USE_CCT_TIME_SYNC
        // gpio_set_level(gpio_touch_led_pins[1], device_info[0].color_or_fan_state);
        // #endif
        nuos_store_data_to_nvs(0);
    }

    void set_leds(int i, bool state){
        if(i == 0){
            if(state) ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], device_info[i].device_level));
            else ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], 0));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[i]));
            //xTaskCreate(update_duty_cycle_task, "update_duty_cycle_task", 2048, &i, 26+i, NULL); 
        }else{
            if(i == 1){
                if(state) gpio_set_level(gpio_touch_led_pins[i], device_info[i].device_state);
                else gpio_set_level(gpio_touch_led_pins[i], 0);
            }else{
                gpio_set_level(gpio_touch_led_pins[i], device_info[i].device_state);
            }
        } 
    }

    void nuos_zb_set_hardware(uint8_t index, uint8_t is_toggle){
        //Set Touch LED pins
        if(timer3_running_flag){
            //toggle pins on button press
            set_hardware(index, is_toggle);
        }else{
            if(nuos_check_state_touch_leds()){
                for(int i=0; i<TOTAL_LEDS; i++){
                    if(i == 0){
                        set_leds(i, device_info[i].device_state); 
                        #ifdef USE_CCT_TIME_SYNC

                        #else
                        device_info[0].color_or_fan_state = device_info[0].device_state;
                        #endif
                        gpio_set_level(gpio_touch_led_pins[1], device_info[0].color_or_fan_state);
                        set_color_temp_only_leds_2();
                    }                       
                }
            }else{
                //toggle pins on button press
                set_hardware(index, is_toggle);               
            }
        }

        nuos_store_data_to_nvs(0);
    }

    uint8_t nuos_get_button_press_index(uint32_t pin){
        for(int index=0; index<TOTAL_BUTTONS; index++){
            if(pin == gpio_touch_btn_pins[index]){
                return index;
            }
        }
        return 0;
    }

    void nuos_init_hardware_dimming_up_down(uint32_t pin){
        uint8_t index = nuos_get_button_press_index(pin);
        if(index == 0){
            if(device_info[index].device_state){
                if(device_info[index].device_level <= MIN_DIM_LEVEL_VALUE){
                    device_info[index].dim_up = 1;
                }else if(device_info[index].device_level >= MAX_DIM_LEVEL_VALUE){
                    device_info[index].dim_up = 0;
                }
            }
        }else if(index == 2 || index == 3){  
            if(device_info[0].device_state){
                if(device_info[0].device_val <= MIN_CCT_VALUE){
                    device_info[0].level_up = 1;
                }else if(device_info[0].device_val >= MAX_CCT_VALUE){
                    device_info[0].level_up = 0;
                }
            }      
        }else{
             
                
        }
       
    }

    static uint8_t level_backup = 0;
    static uint16_t color_backup = 0;

    int find_closest_index(uint16_t value) {
        for (int i = 1; i < MAX_CCT_SCENES_VALUES; i++) {
            if (value < cct_values[i]) {
                return i - 1; // Return previous index as the closest lower bound
            }
        }
        return MAX_CCT_SCENES_VALUES - 1; // If greater than all, return last index
    }

    void convert_colors_to_index(bool is_long_press){
        //printf("Set_Colors\n");
        if(device_info[0].level_up){
            if(device_info[0].device_val + COLOR_STEPS <= (MAX_CCT_VALUE)){
                device_info[0].fan_speed = find_closest_index(device_info[0].device_val);
                if(is_long_press) device_info[0].device_val += COLOR_STEPS;
            } else {
                device_info[0].device_val = MAX_CCT_VALUE;
                device_info[0].fan_speed = MAX_CCT_SCENES_VALUES-1;               
            }
            //printf("device_info[0].fan_speed:%d\n", device_info[0].fan_speed);
            set_color_temp_only_leds_2();
        }else{
            if(device_info[0].device_val - COLOR_STEPS >= MIN_CCT_VALUE){
                device_info[0].fan_speed = find_closest_index(device_info[0].device_val); 
                if(is_long_press) device_info[0].device_val -= COLOR_STEPS;                        
            }else {
                device_info[0].device_val = MIN_CCT_VALUE;
                device_info[0].fan_speed = 1;
            }
            //printf("device_info[1].fan_speed:%d\n", device_info[0].fan_speed);
            set_color_temp_only_leds_2();        
        }
        
    }

    static uint8_t counts_level = 0;
    static uint8_t counts_color = 0;

    bool nuos_set_hardware_brightness(uint32_t pin){
        uint8_t index = nuos_get_button_press_index(pin);
        
        if(global_switch_state == SWITCH_PRESS_DETECTED){ 
            if(!device_info[0].device_state){
                device_info[0].device_state = true;
                device_info[0].device_level = MIN_DIM_LEVEL_VALUE;
                device_info[0].dim_up = 1;
                nuos_set_state_attribute(0);
                device_info[0].level_up = true;
                #ifdef USE_CCT_TIME_SYNC

                #else
                device_info[0].color_or_fan_state = true;
                #endif
                convert_colors_to_index(false);
            }        
            if(index == 0){
                if(device_info[0].device_state){
                    uint8_t last_dim =  device_info[0].device_level;
                    if(device_info[0].dim_up == 1){
                        if(device_info[0].device_level + DIMMING_STEPS <= (MAX_DIM_LEVEL_VALUE)){
                            device_info[0].device_level += DIMMING_STEPS;
                        } else {
                            device_info[0].device_level = MAX_DIM_LEVEL_VALUE;
                        }
                    }else{
                        if(device_info[0].device_level - DIMMING_STEPS >= MIN_DIM_LEVEL_VALUE){
                            device_info[0].device_level -= DIMMING_STEPS;  
                        }else {
                            device_info[0].device_level = MIN_DIM_LEVEL_VALUE;
                        } 
                    }

                    #define LEDC_FADE_NO_WAIT   0

                    if(level_backup != device_info[0].device_level){
                         level_backup = device_info[0].device_level;

                        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[0], device_info[0].device_level));                
                        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[0]));

                        #if(COMMUNICATION_MODE == COMM_MODE_ADDR_CTRL)
                        dali.set_dim_value(global_dali_id[0], device_info[index].device_level);
                        #elif(COMMUNICATION_MODE == COMM_MODE_GROUP_CTRL)
                        dali.set_group_level(global_group_id[0], device_info[index].device_level);
                        #elif(COMMUNICATION_MODE == COMM_MODE_BROADCAST)
                        //esp_zb_lock_acquire(portMAX_DELAY);
                        uint8_t cal_val = calibrate_dali_brightness(device_info[index].device_level);
                        dali.set_broadcast_level(cal_val);        
                        //esp_zb_lock_release(); 
                        #endif

                        if(counts_level++ >= 10){
                            counts_level = 0;
                            nuos_set_color_temp_level_attribute(0);
                        }  
                        nuos_store_data_to_nvs(0);
                    }                      
                }
            }else if(index == 2 || index == 3){
                   convert_colors_to_index(is_long_press_brightness);
                    if(color_backup != device_info[0].device_val){
                        color_backup = device_info[0].device_val;
                        #if(COMMUNICATION_MODE == COMM_MODE_ADDR_CTRL)
                        dali.set_color_temperature(global_dali_id[0], device_info[0].device_val); 
                        #elif(COMMUNICATION_MODE == COMM_MODE_GROUP_CTRL)
                        dali.set_color_temperature(global_group_id[0], device_info[0].device_val);
                        #elif(COMMUNICATION_MODE == COMM_MODE_BROADCAST)
                        //dali.set_broadcast_color_rgb(0, 0, 0, device_info[0].device_level);
                        dali.set_color_temperature(dali.BROADCAST_C, device_info[0].device_val); 
                        #endif 
                        if(counts_color++ >= 10){
                            counts_color = 0;
                            nuos_set_color_temperature_attribute(0);
                        }                                        
                    }  
                    nuos_store_data_to_nvs(0);                      
            }else{
                convert_colors_to_index(is_long_press_brightness);
                #if(COMMUNICATION_MODE == COMM_MODE_ADDR_CTRL)
                dali.set_color_temperature(global_dali_id[0], device_info[0].device_val); 
                #elif(COMMUNICATION_MODE == COMM_MODE_GROUP_CTRL)
                dali.set_color_temperature(global_group_id[0], device_info[0].device_val);
                #elif(COMMUNICATION_MODE == COMM_MODE_BROADCAST)
                //dali.set_broadcast_color_rgb(0, 0, 0, device_info[0].device_level);
                dali.set_color_temperature(dali.BROADCAST_C, device_info[0].device_val); 
                #endif 
                if(counts_color++ >= 10){
                    counts_color = 0;
                    nuos_set_color_temperature_attribute(0);
                }                  
                nuos_store_data_to_nvs(0);                     
            }
        }else{
            printf("switch state not detected!!");
        }
        return false;
    }

    extern "C" bool nuos_set_hardware_brightness_2(uint8_t index){
        global_switch_state = SWITCH_PRESS_DETECTED;
        return nuos_set_hardware_brightness(gpio_touch_btn_pins[index]);        
    }
    
    void nuos_on_off_led(uint8_t index, uint8_t _level){
        if(is_init_done){
            if(index == 0){
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], _level));
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[index]));
                //xTaskCreate(update_duty_cycle_task, "update_duty_cycle_task", 2048, &index, 28, NULL); 
            }else{
                gpio_set_level(gpio_touch_led_pins[index], device_info[index].device_state);
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
                }
                
                if(is_init_done){
                    if(index == 0){
                        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], level));
                        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[index]));
                        //xTaskCreate(update_duty_cycle_task, "update_duty_cycle_task", 2048, &index, 28, NULL); 
                    }else{
                        gpio_set_level(gpio_touch_led_pins[index], state);
                    }

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
        if(!toggle_state) dali.set_group_off(group_id);
        else { 
            dali.set_group_on(group_id); 
        } 
    } 

    extern "C" void nuos_dali_set_group_color_temperature(uint8_t group_id, uint8_t index, uint16_t value){
        if(last_color_temp != device_info[0].device_val){
            last_color_temp = device_info[0].device_val;
            dali.set_broadcast_color_cct(value);
        }
    } 

    extern "C" void nuos_dali_set_group_brightness(uint8_t group_id, uint8_t index, uint8_t value){
        //#ifdef IS_USE_DALI_HARDWARE
        //dali.set_group_level(group_id, value);
        dali.set_broadcast_level(value);
        //#endif
    } 

   
    uint8_t cct_counts = 0;
    extern "C" void nuos_dali_set_color_temperate(uint8_t index){
        device_info[index].device_val = cct_values[cct_counts];
        //#ifdef IS_USE_DALI_HARDWARE
        //dali.set_group_color_cct(dali_nvs_stt[index].group_id, cct_values[cct_counts]); //

        dali.set_color_temperature(dali.BROADCAST_C, cct_values[cct_counts]);  
        //#endif
        nuos_store_data_to_nvs(0);
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
            if(i==0){
                if(device_info[i].device_state){
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], device_info[i].device_level));
                }else{
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], 0));
                }
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[i]));
                //xTaskCreate(update_duty_cycle_task, "update_duty_cycle_task", 2048, &i, i+29, NULL); 
            }else{
                set_leds(i, device_info[i].device_state);
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




