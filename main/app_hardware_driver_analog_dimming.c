
 #include "app_hardware_driver.h"
 #include "app_zigbee_clusters.h"
#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4T_ANALOG_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_ANALOG_DIMMABLE_LIGHT)
    #include "esp_log.h"
    #include "esp_err.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"

    #include <esp_check.h>
    #include <esp_log.h>
    #include "driver/ledc.h"


    #define USE_NVS_STORE

    // ledc_mode_t LEDC_MODE_T[4] = {LEDC_LOW_SPEED_MODE, LEDC_LOW_SPEED_MODE, LEDC_SPEED_MODE_MAX, LEDC_SPEED_MODE_MAX};
    

    #define LEDC_TIMER              		                    LEDC_TIMER_0
    #define LEDC_MODE               		                    LEDC_LOW_SPEED_MODE

    #define LEDC_DUTY_RES          		 	                    LEDC_TIMER_8_BIT // Set duty resolution to 13 bits
    #define LEDC_DUTY               		                    100      // Set duty to 50%. ((2 ** 8) - 1) * 50% = 4095
    #define LEDC_MAX_DUTY           		                    254     //8191
    //#define LEDC_MIN_DUTY           		                    (LEDC_MAX_DUTY*10)/100     //10%

    // #define LEDC_FREQUENCY          	                        1000    // Frequency in Hertz. Set frequency at 1 KHz

    #define LEDC_FREQUENCY                                      (1000) // Frequency in Hz (5 kHz)
    #define LEDC_FADE_TIME                                      (500) // Fade time in milliseconds (1 second)

    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4T_ANALOG_DIMMABLE_LIGHT)
        uint8_t pwm_channels[TOTAL_ENDPOINTS]                   = { LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
        #define LED_INDEX_START                  0
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_ANALOG_DIMMABLE_LIGHT)
        #define LED_INDEX_START                  2 
        uint8_t pwm_channels[LED_INDEX_START+TOTAL_ENDPOINTS]    = { LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3 };
        int LEDC_TIMER_T[LED_INDEX_START+TOTAL_ENDPOINTS]        = { LEDC_TIMER_0, LEDC_TIMER_0, LEDC_TIMER_0, LEDC_TIMER_0 };    
    #endif


    #if (CHIP_INFO == USE_ESP32C6)
        #define START_LED_INDEX                                         1
    #elif(CHIP_INFO == USE_ESP32C6_MINI1)      
        #define START_LED_INDEX                                         1                      
    #elif(CHIP_INFO == USE_ESP32H2)    
        #define START_LED_INDEX                                         1
    #elif(CHIP_INFO == USE_ESP32H2_MINI1) 
        #define START_LED_INDEX                                         0   
    #elif(CHIP_INFO == USE_ESP32H2_MINI1_V2) 
        #define START_LED_INDEX                                         0                             
    #elif(CHIP_INFO == USE_ESP32C6_MINI1_V2) 
        #define START_LED_INDEX                                         0
    #elif(CHIP_INFO == USE_ESP32H2_MINI1_V3) 
        #define START_LED_INDEX                                         0                             
    #elif(CHIP_INFO == USE_ESP32C6_MINI1_V3) 
        #define START_LED_INDEX                                         0                                                                 
    #endif

    bool nuos_check_state_touch_leds();

    bool is_init_done                                               = false;
    static bool state                                               = false;
    uint8_t level = 0;
    gpio_num_t gpio_total_init_pins[TOTAL_LOADS+TOTAL_LEDS]         = {0,0,0,0};
    static uint8_t level_backup[4]                                  = {0, 0, 0, 0};

    #ifdef USE_FADING
    void init_fading(){
        // Install the fade function
        ledc_fade_func_install(0);
    } 
    #endif 
    void nuos_zb_init_hardware(){
        if(!is_init_done){
            for(int i=0; i<TOTAL_LOADS; i++){
                gpio_total_init_pins[i] = gpio_load_pins[i];
                // gpio_set_drive_capability(gpio_load_pins[i], GPIO_DRIVE_CAP_3);
            }
            for(int j=0; j<TOTAL_LEDS; j++){
                gpio_total_init_pins[LED_INDEX_START+j] = gpio_touch_led_pins[j];
                // gpio_set_drive_capability(gpio_touch_led_pins[j], GPIO_DRIVE_CAP_3);
            } 

            // Prepare and then apply the LEDC PWM timer configuration
            ledc_timer_config_t ledc_timer = {
                .speed_mode       = LEDC_MODE,
                .timer_num        = LEDC_TIMER,
                .duty_resolution  = LEDC_DUTY_RES,
                .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 100Hz
                .clk_cfg          = LEDC_AUTO_CLK
            };
            ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

            // Prepare and then apply the LEDC PWM channel configuration
            for(int index=0; index<TOTAL_LOADS+TOTAL_LEDS; index++){
                ledc_channel_config_t ledc_channel = {
                    .speed_mode     = LEDC_MODE,
                    .channel        = pwm_channels[index],
                    .timer_sel      = LEDC_TIMER_T[index],
                    .intr_type      = LEDC_INTR_DISABLE,
                    .gpio_num       = gpio_total_init_pins[index],
                    .duty           = level, // Set duty to 0%
                    .hpoint         = 0
                };
                ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
            } 
        
            #ifdef USE_FADING
                init_fading();   
            #endif        
            is_init_done = true;    
        }         
    }


    void nuos_on_off_led(uint8_t index, uint8_t _state){
        if(is_init_done){
            if(index < TOTAL_LEDS){
                if(_state)  {
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[LED_INDEX_START+index], LEDC_MAX_DUTY));
                }else{
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[LED_INDEX_START+index], 0));
                }            
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[LED_INDEX_START+index]));
            }
        }
    }

    void nuos_toggle_leds(uint8_t index){
        state = !state;
        if(state) level = LEDC_MAX_DUTY; 
        else level = 0;         
        nuos_on_off_led(index, state);
    }

    void set_level_value(uint8_t _level){
        level = _level;
    }

    void nuos_zb_set_hardware_led_for_zb_commissioning(uint8_t is_toggle){
        #ifdef USE_RGB_LED
            nuos_toggle_rgb_led();
        #else
            // uint8_t _level = 0;
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
                        nuos_on_off_led(index, state);
                    }
                }
            }
        #endif
    }

    void set_harware(uint8_t index, uint8_t is_toggle){
        if(is_toggle>0) device_info[index].device_state = !device_info[index].device_state;
        if(is_init_done){
            #ifdef USE_FADING
                //xTaskCreate(nuos_set_hw_task, "nuos_set_hw_task", 4096, &index, 28, NULL); 
                if(is_init_done){
                    if(!device_info[index].device_state) {
                        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[LED_INDEX_START+index], level));
                        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[LED_INDEX_START+index]));  
                        ledc_set_fade_time_and_start(LEDC_MODE, pwm_channels[index], 0, LEDC_FADE_TIME, LEDC_FADE_NO_WAIT);   
                    }else{
                        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[LED_INDEX_START+index], device_info[index].device_level));
                        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[LED_INDEX_START+index]));  
                        ledc_set_fade_time_and_start(LEDC_MODE, pwm_channels[index], device_info[index].device_level, LEDC_FADE_TIME, LEDC_FADE_NO_WAIT);   
                    }               
                }
            #else
                if(!device_info[index].device_state){
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], 0));
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[LED_INDEX_START+index], 0));
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[index]));
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[LED_INDEX_START+index]));
                }else{
                    if(device_info[index].device_level >= MIN_DIM_LEVEL_VALUE){
                        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], device_info[index].device_level));
                        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[LED_INDEX_START+index], device_info[index].device_level));
                        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[index]));
                        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[LED_INDEX_START+index]));
                    }else{
                        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], MIN_DIM_LEVEL_VALUE));
                        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[LED_INDEX_START+index], MIN_DIM_LEVEL_VALUE));
                        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[index]));
                        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[LED_INDEX_START+index]));                   
                    }
                }
            #endif  
        }
        #ifdef USE_RGB_LED
            nuos_rgb_trigger_blink();
        #endif               
    }

    void set_leds(int i, bool state){
        if(is_init_done){
            if(i == 0){
                if(gpio_touch_led_pins[0] == gpio_load_pins[0]){
                    if(state) ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[0], device_info[i].device_level));
                    else ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[0], 0));
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[0]));
                }else{
                    if(state) ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[LED_INDEX_START+i], device_info[i].device_level));
                    else ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[LED_INDEX_START+i], 0));
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[LED_INDEX_START+i]));
                }
            }else{
                if(state) ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[LED_INDEX_START+i], device_info[i].device_level));
                else ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[LED_INDEX_START+i], 0));
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[LED_INDEX_START+i]));
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

        #ifdef USE_NVS_STORE          
        nuos_store_data_to_nvs(index);
        #endif
    }

    void nuos_set_state_touch_leds(bool state){
        for(int i=0; i<TOTAL_LEDS; i++){
             set_leds(i, state);
        }
    }

    bool nuos_check_state_touch_leds(){
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

    void nuos_init_hardware_dimming_up_down(uint32_t pin){
        uint8_t index = nuos_get_button_press_index(pin);
        if(device_info[index].device_state){
            if(device_info[index].device_level <= MIN_DIM_LEVEL_VALUE+20){
                device_info[index].dim_up = 1;
            }else if(device_info[index].device_level >= MAX_DIM_LEVEL_VALUE-20){
                device_info[index].dim_up = 0;
            }
        }
    }

    void set_load(uint8_t index, uint8_t value){
        if(is_init_done){
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[LED_INDEX_START+index], value));           
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], value));
            
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[LED_INDEX_START+index]));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[index]));
        }
        #ifdef USE_NVS_STORE 
        nuos_store_data_to_nvs(index);
        #endif        
    }

    bool nuos_set_hardware_brightness(uint32_t pin){
        uint8_t index = nuos_get_button_press_index(pin);
        //if(global_switch_state == SWITCH_PRESS_DETECTED){ 
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
                set_load(index, device_info[index].device_level);            
            }
        //}
        return false;
    }
#endif

