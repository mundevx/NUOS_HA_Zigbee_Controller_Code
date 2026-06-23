
#include "app_hardware_driver.h"
#include "app_zigbee_clusters.h"

#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
    #include <stdint.h>
    #include "esp_log.h"
    #include "esp_err.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include <esp_check.h>
    #include <esp_log.h>
    #include "driver/ledc.h"
    #include "DaliCommands.h"
    #include "esp_wifi.h"  // For esp_wifi_stop() and esp_wifi_start()
    #include "esp_wifi_station.h"

    
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
        #ifndef USE_TWO_SWITCH_MODE
        ledc_channel_t pwm_channels[TOTAL_ENDPOINTS]    = { LEDC_CHANNEL_0};
        #else
        ledc_channel_t pwm_channels[TOTAL_LEDS]    = { LEDC_CHANNEL_0, LEDC_CHANNEL_1};
        #endif
    #else
        ledc_channel_t pwm_channels[1]    = { LEDC_CHANNEL_0};
    #endif
    bool is_init_done                       = false;
    bool _toggle_                           = false;
    // #define IS_USE_DALI_HARDWARE

    extern "C" void nuos_set_state_touch_leds(bool state);
    extern "C" bool nuos_check_state_touch_leds();
    extern "C" void start_color_temp_timer();
    extern "C" void stop_color_temp_timer();
    extern "C" void nuos_dali_set_group_brightness(uint8_t group_id, uint8_t index, uint8_t value); 
    static bool state = false;
    uint8_t level = 0;

    #define MAX_CCT_SCENES_VALUES    6

    uint16_t cct_values[MAX_CCT_SCENES_VALUES] = {MIN_CCT_VALUE, MIN_CCT_VALUE_1, MIN_CCT_VALUE_2, MIN_CCT_VALUE_3, MIN_CCT_VALUE_4, MAX_CCT_VALUE};

    #define MAX_DIMMING_VALUES      15

    uint16_t dim_values[MAX_DIMMING_VALUES] = {0, MIN_DIM_LEVEL_VALUE, 20, 40, 60, 80, 100, 120, 140, 160, 180, 200, 220, 240, MAX_DIM_LEVEL_VALUE};

    static uint8_t last_dali_level = 0; 
    static uint16_t last_color_temp = 0;

        // DALI limits
    #define DALI_MIN MIN_DIM_LEVEL_VALUE
    #define DALI_MAX MAX_DIM_LEVEL_VALUE
    #define CALIBRATION_GAMMA 0.5  // Tune as needed
    static uint8_t level_backup = 0;
    static uint16_t color_backup = 0;
    static uint8_t counts_level = 0;
    static uint8_t counts_color = 0;
    static QueueHandle_t rxFrameQueue = nullptr;           // Queue for received frames (each is uint32_t)

    extern "C" void nuos_zb_init_hardware(){
        #ifndef USE_TWO_SWITCH_MODE
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
        #else
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
        #endif
        is_init_done = true;
    }


    #define CCT_MIN_K   2000
    #define CCT_MAX_K   6500

    #define CCT_MIN_LVL 1
    #define CCT_MAX_LVL 255

    uint16_t map_level_to_cct_k(uint8_t level)
    {
        /* Clamp input */
        if (level <= CCT_MIN_LVL)
            return CCT_MIN_K;

        if (level >= CCT_MAX_LVL)
            return CCT_MAX_K;

        /* Linear map */
        uint32_t num = (uint32_t)(level - CCT_MIN_LVL) * (CCT_MAX_K - CCT_MIN_K);

        uint32_t den = (CCT_MAX_LVL - CCT_MIN_LVL);

        return (uint16_t)(CCT_MIN_K + (num / den));
    }

    uint8_t map_cct_k_to_level(uint16_t kelvin)
    {
        /* Clamp input */
        if (kelvin <= CCT_MIN_K)
            return CCT_MIN_LVL;

        if (kelvin >= CCT_MAX_K)
            return CCT_MAX_LVL;

        /* Linear map */
        uint32_t num = (uint32_t)(kelvin - CCT_MIN_K) * (CCT_MAX_LVL - CCT_MIN_LVL);

        uint32_t den = (CCT_MAX_K - CCT_MIN_K);

        return (uint8_t)(CCT_MIN_LVL + (num / den));
    }


    uint8_t map_1_255_to_100_255(uint8_t in)
    {
        return (uint8_t)((in * dali_range_size) / 254 + dali_min_off_offset);
    }

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
        
        if(index == 0){
            //esp_zb_lock_acquire(portMAX_DELAY);
            if(!device_info[index].device_state) {
                if(scene_group_switch_info.control_type != 0) { 

                    #if(COMMUNICATION_MODE == COMM_MODE_ADDR_CTRL)
                    dali.turn_off(global_dali_id[0]); 
                    #elif(COMMUNICATION_MODE == COMM_MODE_GROUP_CTRL)
                    dali.set_group_off(global_group_id[0]); 
                    #elif(COMMUNICATION_MODE == COMM_MODE_BROADCAST)
                    dali.send_broadcast(dali.OFF_C); 
                    #endif
                   
                }else{
                    printf("gid0:%d\n", scene_group_switch_info.group_id[index]);
                    dali.set_group_off(scene_group_switch_info.group_id[index]);
                }
            
            } else {
                if(scene_group_switch_info.control_type != 0) { 
                    #if(COMMUNICATION_MODE == COMM_MODE_ADDR_CTRL)
                    // dali.turn_on_to_last_level(global_dali_id[0]);
                    // vTaskDelay(5 / portTICK_PERIOD_MS);
                    dali.set_dim_value(global_dali_id[0], calibrate_dali_brightness(device_info[index].device_level));
                    #elif(COMMUNICATION_MODE == COMM_MODE_GROUP_CTRL)
                    dali.set_group_level(global_group_id[0], calibrate_dali_brightness(device_info[index].device_level));
                    vTaskDelay(5 / portTICK_PERIOD_MS);
                    dali.set_group_level(global_group_id[0], calibrate_dali_brightness(device_info[index].device_level));
                    #elif(COMMUNICATION_MODE == COMM_MODE_BROADCAST)  
                    dali.set_broadcast_level(map_1_255_to_100_255(device_info[index].device_level));
                    vTaskDelay(5 / portTICK_PERIOD_MS);
                    dali.set_broadcast_level(map_1_255_to_100_255(device_info[index].device_level)); 
                    #endif 
                }else{
                    printf("gid1:%d\n", scene_group_switch_info.group_id[index]);
                    nuos_dali_set_group_brightness(scene_group_switch_info.group_id[index], index, device_info[index].device_level);
                }  
            }
        }
        
        nuos_set_state_attribute(0);
    }
    extern "C" void set_dali_level(uint8_t index){
        if(scene_group_switch_info.control_type != 0) { 
            #if(COMMUNICATION_MODE == COMM_MODE_ADDR_CTRL)
            dali.set_dim_value(global_dali_id[0], device_info[index].device_level);
            #elif(COMMUNICATION_MODE == COMM_MODE_GROUP_CTRL)
            dali.set_group_level(global_group_id[0], device_info[index].device_level);
            #elif(COMMUNICATION_MODE == COMM_MODE_BROADCAST)  
            dali.set_broadcast_level(map_1_255_to_100_255(device_info[index].device_level));
            #endif 
        }else{  
            if(device_info[index].device_state){
                nuos_dali_set_group_brightness(scene_group_switch_info.group_id[index], index, device_info[index].device_level);
            }else{
                nuos_dali_set_group_brightness(scene_group_switch_info.group_id[index], index, 0);
            }
        }
        nuos_set_color_temp_level_attribute(0); 
    }

    extern "C" void set_dali_color_temp(uint8_t index, bool status){
        if(scene_group_switch_info.control_type != 0) { 
            dali.set_color_temperature(dali.BROADCAST_C, device_info[index].device_val); 
        }else{
            dali.set_group_color_cct(scene_group_switch_info.group_id[index], device_info[index].device_val);  
        }   
    }


    static void update_duty_cycle_task(void *pvParameters){
        uint8_t index = *(uint8_t*)pvParameters;
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[index]));
        vTaskDelete(NULL);
    } 

    void set_color_temp_only_leds_2() {

        #ifndef USE_TWO_SWITCH_MODE
        #ifdef USE_CCT_TIME_SYNC
            if(device_info[0].device_state){
        #else
            if(device_info[0].device_state){
                if(device_info[0].color_or_fan_state){
        #endif
                gpio_set_level(gpio_touch_led_pins[1], 1);
               
                #ifndef USE_TWO_SWITCH_MODE
                #ifdef USE_CCT_TIME_SYNC
                #else

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
                #endif
            } else {
                #ifndef USE_TWO_SWITCH_MODE
                #ifdef USE_CCT_TIME_SYNC
                #else
                gpio_set_level(gpio_touch_led_pins[1], 0);
                #endif
                if(device_info[0].ac_temperature > 1 && device_info[0].ac_temperature < MAX_DIMMING_VALUES-1){
                    gpio_set_level(gpio_touch_led_pins[3], 1);
                    gpio_set_level(gpio_touch_led_pins[2], 1);
                }else{
                    if(device_info[0].ac_temperature >= MAX_DIMMING_VALUES-1) {
                        gpio_set_level(gpio_touch_led_pins[3], 1);
                        gpio_set_level(gpio_touch_led_pins[2], 0);
                    } else {
                        gpio_set_level(gpio_touch_led_pins[2], 1);
                        gpio_set_level(gpio_touch_led_pins[3], 0);
                    }
                }
                #endif                
                                
            }   
        }else{
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[0], 0));
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[0]));
                #ifndef USE_TWO_SWITCH_MODE
                #ifdef USE_CCT_TIME_SYNC
                #else             
                gpio_set_level(gpio_touch_led_pins[1], 0);
                #endif
                gpio_set_level(gpio_touch_led_pins[3], 0);
                gpio_set_level(gpio_touch_led_pins[2], 0);
                #else
                    if(device_info[0].device_val >= MIN_CCT_VALUE){
                        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[1], map_cct_k_to_level(device_info[0].device_val)));
                    }else{
                        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[1], map_cct_k_to_level(MAX_CCT_VALUE)));                   
                    }
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[1]));  
                #endif     
        }
        #endif         

    }

    void set_color_temp_leds(uint8_t index){
        #ifndef USE_TWO_SWITCH_MODE
        #ifdef USE_CCT_TIME_SYNC
        if(device_info[0].device_state){
        #else
        if(device_info[0].device_state){
            if(device_info[0].color_or_fan_state){
        #endif
                #ifdef USE_CCT_TIME_SYNC
                #else
                gpio_set_level(gpio_touch_led_pins[1], 1);
                #endif

                if(index == 2) {
                    if(device_info[0].fan_speed < MAX_CCT_SCENES_VALUES-2){
                        device_info[0].fan_speed++;
                        gpio_set_level(gpio_touch_led_pins[3], 1);
                        gpio_set_level(gpio_touch_led_pins[2], 1);
                    } else {
                        device_info[0].fan_speed = MAX_CCT_SCENES_VALUES-1;
                        gpio_set_level(gpio_touch_led_pins[3], 1);
                        gpio_set_level(gpio_touch_led_pins[2], 0);
                    } 
                    device_info[0].device_val = cct_values[device_info[0].fan_speed];
                }else if(index == 3) {
                    if(device_info[0].fan_speed > 2){
                        device_info[0].fan_speed--;
                        gpio_set_level(gpio_touch_led_pins[3], 1);
                        gpio_set_level(gpio_touch_led_pins[2], 1);
                    } 
                    else {
                        device_info[0].fan_speed = 1;
                        
                        gpio_set_level(gpio_touch_led_pins[3], 0);
                        gpio_set_level(gpio_touch_led_pins[2], 1);
                    }
                    device_info[0].device_val = cct_values[device_info[0].fan_speed];
                }else{
                    if(device_info[0].fan_speed == 0){
                        gpio_set_level(gpio_touch_led_pins[3], 0);
                        gpio_set_level(gpio_touch_led_pins[2], 1);
                    }else if(device_info[0].fan_speed == MAX_CCT_SCENES_VALUES-1){
                        gpio_set_level(gpio_touch_led_pins[3], 1);
                        gpio_set_level(gpio_touch_led_pins[2], 0);
                    }else{
                        gpio_set_level(gpio_touch_led_pins[3], 1);
                        gpio_set_level(gpio_touch_led_pins[2], 1);
                    }
                }
                nuos_store_data_to_nvs(0);
            }
        } else{
            #ifdef USE_CCT_TIME_SYNC
            #else
            gpio_set_level(gpio_touch_led_pins[1], 0);
            #endif
            gpio_set_level(gpio_touch_led_pins[3], 0);
            gpio_set_level(gpio_touch_led_pins[2], 0);
        }
        #endif
    }

    void set_dimming_control_leds(uint8_t index){
        #ifndef USE_TWO_SWITCH_MODE
        #ifdef USE_CCT_TIME_SYNC
        if(device_info[0].device_state){
        #else
        if(device_info[0].device_state){
            if(!device_info[0].color_or_fan_state){
        #endif
                #ifdef USE_CCT_TIME_SYNC
                #else
                gpio_set_level(gpio_touch_led_pins[1], 0);
                #endif
                // printf("ok done index:%d\n", index);
                if(index == 2) {
                    if(device_info[0].ac_temperature < MAX_DIMMING_VALUES-2){
                        gpio_set_level(gpio_touch_led_pins[3], 1);
                        gpio_set_level(gpio_touch_led_pins[2], 1);
                        device_info[0].ac_temperature++;
                    } else {
                        device_info[0].ac_temperature = MAX_DIMMING_VALUES-1;
                        gpio_set_level(gpio_touch_led_pins[3], 1);
                        gpio_set_level(gpio_touch_led_pins[2], 0);
                    } 
                    device_info[0].device_level = dim_values[device_info[0].ac_temperature];
                    printf("Increment level:%d\n", device_info[0].device_level);
                    ledc_set_duty(LEDC_MODE, pwm_channels[0], device_info[0].device_level);            
                    ledc_update_duty(LEDC_MODE, pwm_channels[0]);
                }else if(index == 3) {
                    if(device_info[0].ac_temperature > 2){
                        gpio_set_level(gpio_touch_led_pins[3], 1);
                        gpio_set_level(gpio_touch_led_pins[2], 1);
                        device_info[0].ac_temperature--;
                    } else {
                        device_info[0].ac_temperature = 1;
                        gpio_set_level(gpio_touch_led_pins[3], 0);
                        gpio_set_level(gpio_touch_led_pins[2], 1);
                    }
                    device_info[0].device_level = dim_values[device_info[0].ac_temperature];
                    printf("Decrement level:%d\n", device_info[0].device_level);
                    ledc_set_duty(LEDC_MODE, pwm_channels[0], device_info[0].device_level);            
                    ledc_update_duty(LEDC_MODE, pwm_channels[0]); 

                }else{
                    printf("ac_temperature:%d\n", device_info[0].ac_temperature);
                    if(device_info[0].ac_temperature == 1){
                        gpio_set_level(gpio_touch_led_pins[3], 0);
                        gpio_set_level(gpio_touch_led_pins[2], 1);
                    }else if(device_info[0].ac_temperature == MAX_DIMMING_VALUES-1){
                        gpio_set_level(gpio_touch_led_pins[3], 1);
                        gpio_set_level(gpio_touch_led_pins[2], 0);
                    }else{
                        gpio_set_level(gpio_touch_led_pins[3], 1);
                        gpio_set_level(gpio_touch_led_pins[2], 1);
                    }
                    printf("No Change level:%d\n", device_info[0].device_level);
                    ledc_set_duty(LEDC_MODE, pwm_channels[0], device_info[0].device_level);            
                    ledc_update_duty(LEDC_MODE, pwm_channels[0]);
                }
                nuos_store_data_to_nvs(0);
            }
        } else{
            #ifdef USE_CCT_TIME_SYNC
            #else
            gpio_set_level(gpio_touch_led_pins[1], 0);
            #endif
            gpio_set_level(gpio_touch_led_pins[3], 0);
            gpio_set_level(gpio_touch_led_pins[2], 0);
        }
        #endif
    }



    void set_hardware_for_dali_rx(uint8_t index, bool device_state, uint8_t device_level){
        if(index == 0){
            if(!device_state) {
                printf("DALI ID:%d OFF\n", scene_group_switch_info.group_id[index]);
                #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
                    #ifndef USE_TWO_SWITCH_MODE
                    ledc_set_duty(LEDC_MODE, pwm_channels[index], 0);            
                    ledc_update_duty(LEDC_MODE, pwm_channels[index]);
                    #else
                    ledc_set_duty(LEDC_MODE, pwm_channels[0], 0);            
                    ledc_update_duty(LEDC_MODE, pwm_channels[0]);

                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[1], 0));
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[1]));       
                    #endif             
                #else
                    gpio_set_level(gpio_touch_led_pins[index], 0);
                #endif
                #ifdef USE_CCT_TIME_SYNC
                if(device_info[0].color_or_fan_state){
                    device_info[0].color_or_fan_state = false;
                    gpio_set_level(gpio_touch_led_pins[1], 0);
                    stop_color_temp_timer();
                }    
                #endif    
                nuso_set_state_attribute_on_dali_rx(index, device_state);        
            } else {
                printf("DALI ID:%d ON\n", scene_group_switch_info.group_id[index]);
                #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
                    #ifndef USE_TWO_SWITCH_MODE
                    ledc_set_duty(LEDC_MODE, pwm_channels[index], device_level);            
                    ledc_update_duty(LEDC_MODE, pwm_channels[index]);
                    #else
                    ledc_set_duty(LEDC_MODE, pwm_channels[0], device_info[0].device_level);            
                    ledc_update_duty(LEDC_MODE, pwm_channels[0]);                    
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[1], map_cct_k_to_level(device_info[0].device_val)));
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[1]));
                    #endif
                #else
                    gpio_set_level(gpio_touch_led_pins[index], 1);
                #endif
                nuos_set_color_temp_level_attribute_on_dali_rx(index, device_level);
            }
            #ifndef USE_TWO_SWITCH_MODE
            set_color_temp_only_leds_2();
            #endif
            nuos_store_data_to_nvs(0);
        }
        #ifndef USE_TWO_SWITCH_MODE
        else if(index == 1){
            if(device_info[0].device_state){
                device_info[0].color_or_fan_state = !device_info[0].color_or_fan_state;
                gpio_set_level(gpio_touch_led_pins[1], device_info[0].color_or_fan_state);

                #ifdef USE_CCT_TIME_SYNC
                if(device_info[0].color_or_fan_state){
                    start_color_temp_timer();
                }else{
                    stop_color_temp_timer();
                }
                #else
                if(device_info[0].color_or_fan_state){
                    printf("Enabled Color Control!!\n");
                    set_color_temp_leds(index);
                }else{
                    printf("Enabled Dimming Control!!\n");
                    set_dimming_control_leds(index);
                }
                #endif
            }else{

            }
           

        } else {
            #ifdef USE_CCT_TIME_SYNC
                set_color_temp_leds(index);
                if(device_info[0].color_or_fan_state){
                    device_info[0].color_or_fan_state = false;
                    gpio_set_level(gpio_touch_led_pins[1], 0);
                    stop_color_temp_timer();
                }                  
            #else
                if(device_info[0].color_or_fan_state){
                    set_color_temp_leds(index);
                }else{
                    set_dimming_control_leds(index);
                }
                    
            #endif    
        }
        #endif
    }

    void process_dali_task(uint8_t index, uint8_t is_toggle){
        if(index == 0){
            if(is_toggle>0) device_info[index].device_state = !device_info[index].device_state;
            if(!device_info[index].device_state) {
                printf("DALI ID:%d OFF\n", scene_group_switch_info.group_id[index]);
                #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
                    #ifndef USE_TWO_SWITCH_MODE
                    ledc_set_duty(LEDC_MODE, pwm_channels[index], 0);            
                    ledc_update_duty(LEDC_MODE, pwm_channels[index]);
                    #else
                    ledc_set_duty(LEDC_MODE, pwm_channels[0], 0);            
                    ledc_update_duty(LEDC_MODE, pwm_channels[0]);

                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[1], 0));
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[1]));       
                    #endif             
                #else
                    gpio_set_level(gpio_touch_led_pins[index], 0);
                #endif
                #ifdef USE_CCT_TIME_SYNC
                if(device_info[0].color_or_fan_state){
                    device_info[0].color_or_fan_state = false;
                    gpio_set_level(gpio_touch_led_pins[1], 0);
                    stop_color_temp_timer();
                }    
                #endif  
                // dali.set_power_on_level(0x0A, 0x0);          
                //xTaskCreate(esp_dali_off_task, "dali_off_task", 4096, &index, TASK_PRIORITY_RGB, NULL);
            } else {
                // printf("DALI ID:%d ON\n", scene_group_switch_info.scene_ids[index]);
                #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
                    #ifndef USE_TWO_SWITCH_MODE
                    ledc_set_duty(LEDC_MODE, pwm_channels[index], device_info[index].device_level);            
                    ledc_update_duty(LEDC_MODE, pwm_channels[index]);
                    #else
                    ledc_set_duty(LEDC_MODE, pwm_channels[0], device_info[0].device_level);            
                    ledc_update_duty(LEDC_MODE, pwm_channels[0]);                    
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[1], map_cct_k_to_level(device_info[0].device_val)));
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[1]));
                    #endif
                #else
                    gpio_set_level(gpio_touch_led_pins[index], 1);
                #endif
            }
            #ifndef USE_TWO_SWITCH_MODE
            set_color_temp_only_leds_2();
            #endif
            nuos_store_data_to_nvs(0);
        }
        #ifndef USE_TWO_SWITCH_MODE
        else if(index == 1){
            if(device_info[0].device_state){
                device_info[0].color_or_fan_state = !device_info[0].color_or_fan_state;
                gpio_set_level(gpio_touch_led_pins[1], device_info[0].color_or_fan_state);

                #ifdef USE_CCT_TIME_SYNC
                if(device_info[0].color_or_fan_state){
                    start_color_temp_timer();
                }else{
                    stop_color_temp_timer();
                }
                #else
                if(device_info[0].color_or_fan_state){
                    printf("Enabled Color Control!!\n");
                    set_color_temp_leds(index);
                }else{
                    printf("Enabled Dimming Control!!\n");
                    set_dimming_control_leds(index);
                }
                #endif
            }else{

            }
           

        } else {
            #ifdef USE_CCT_TIME_SYNC
                set_color_temp_leds(index);
                if(device_info[0].color_or_fan_state){
                    device_info[0].color_or_fan_state = false;
                    gpio_set_level(gpio_touch_led_pins[1], 0);
                    stop_color_temp_timer();
                }                  
            #else
                if(device_info[0].color_or_fan_state){
                    set_color_temp_leds(index);
                }else{
                    set_dimming_control_leds(index);
                }
                    
            #endif    
        }
        #endif
    }

    void set_leds(int i, bool _state_){
        #ifndef USE_TWO_SWITCH_MODE
        if(i == 0){
            if(_state_) ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], device_info[i].device_level));
            else ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], 0));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[i]));
            //xTaskCreate(update_duty_cycle_task, "update_duty_cycle_task", 2048, &i, 26+i, NULL); 


        }else{
            if(i == 1){
                if(_state_) gpio_set_level(gpio_touch_led_pins[i], device_info[i].device_state);
                else gpio_set_level(gpio_touch_led_pins[i], 0);
            }else{
                gpio_set_level(gpio_touch_led_pins[i], device_info[i].device_state);
            }
        } 
        #else
            if(i == 0){
                if(_state_) ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], device_info[i].device_level));
                else ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], 0));
            }else{
                if(_state_) ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], map_cct_k_to_level(device_info[0].device_val)));
                else ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], 0));
            }
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[i]));
        #endif
    }

    void nuos_zb_set_hardware(uint8_t index, uint8_t is_toggle){
        //Set Touch LED pins
        call_common_check_auto_off();
        process_dali_task(index, is_toggle);
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
    extern "C" void dali_query_send(uint8_t id, uint8_t command){
        printf("===Sending DALi Query...\n");
        dali.query(id, command); // Send your query command
    }

    extern "C" void dali_disable_query_mode(){
        printf("===Disable Query Mode...\n");
        dali.disable_query_mode(); // Send your query command
    }

    void nuos_init_hardware_dimming_up_down(uint32_t pin){
        uint8_t index = nuos_get_button_press_index(pin);
        if(index == 0){
            if(device_info[index].device_state){
                if(device_info[index].device_level <= MIN_DIM_LEVEL_VALUE+20){
                    device_info[index].dim_up = 1;
                }else if(device_info[index].device_level >= MAX_DIM_LEVEL_VALUE-20){
                    device_info[index].dim_up = 0;
                }
            }
        }
        #ifdef USE_TWO_SWITCH_MODE
            else if(index == 1){  
                if(device_info[0].device_state){
                    if(device_info[0].device_val <= MIN_CCT_VALUE+800){
                        device_info[0].level_up = 1;
                    }else if(device_info[0].device_val >= MAX_CCT_VALUE-800){
                        device_info[0].level_up = 0;
                    }
                }      
            }        
        #else
        else if(index == 2 || index == 3){ 
            if(device_info[0].color_or_fan_state){ 
                if(device_info[0].device_state){
                    if(device_info[0].device_val <= MIN_CCT_VALUE){
                        device_info[0].level_up = 1;
                    }else if(device_info[0].device_val >= MAX_CCT_VALUE){
                        device_info[0].level_up = 0;
                    }
                }   
            }else{
                if(device_info[index].device_state){
                    if(device_info[index].device_level <= MIN_DIM_LEVEL_VALUE+20){
                        device_info[index].dim_up = 1;
                    }else if(device_info[index].device_level >= MAX_DIM_LEVEL_VALUE-20){
                        device_info[index].dim_up = 0;
                    }
                }                
            }   
        }
        #endif
    }

    int find_closest_index(uint16_t value) {
        for (int i = 0; i < MAX_CCT_SCENES_VALUES-1; i++) {
            
            if (value <= cct_values[i]) {
                printf("closest index 0:%d\n", i);
                return i; // Return previous index as the closest lower bound
            }
        }
        printf("closest index 3:%d\n", MAX_CCT_SCENES_VALUES-1);
        return MAX_CCT_SCENES_VALUES - 1; // If greater than all, return last index
    }

    int find_closest_index_2(uint16_t value) {
        for (int i = 1; i < MAX_DIMMING_VALUES; i++) {
            if (value < dim_values[i]) {
                printf("closest index:%d\n", i-1);
                return i - 1; // Return previous index as the closest lower bound
            }
        }
        printf("closest index 2:%d\n", MAX_DIMMING_VALUES-1);
        return MAX_DIMMING_VALUES - 1; // If greater than all, return last index
    }

    void convert_colors_to_index(bool is_long_press){
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

   void convert_dimming_to_index(bool is_long_press){
        if(device_info[0].dim_up){
            if(device_info[0].device_level + DIMMING_STEPS <= (MAX_DIM_LEVEL_VALUE)){
                device_info[0].ac_temperature = find_closest_index_2(device_info[0].device_level);
                if(is_long_press) device_info[0].device_level += DIMMING_STEPS;
            } else {
                device_info[0].device_level = MAX_DIM_LEVEL_VALUE;
                device_info[0].ac_temperature = MAX_DIMMING_VALUES-1;               
            }
            set_color_temp_only_leds_2();
        }else{
            if(device_info[0].device_level - DIMMING_STEPS >= MIN_DIM_LEVEL_VALUE){
                device_info[0].ac_temperature = find_closest_index_2(device_info[0].device_level); 
                if(is_long_press) device_info[0].device_level -= DIMMING_STEPS;                        
            }else {
                device_info[0].device_level = MIN_DIM_LEVEL_VALUE;
                device_info[0].ac_temperature = 1;
            }
            set_color_temp_only_leds_2();        
        }
        
    }

    bool nuos_set_hardware_brightness(uint32_t pin){
        uint8_t index = nuos_get_button_press_index(pin);
        
        call_common_check_auto_off();
        // if(global_switch_state == SWITCH_PRESS_DETECTED){ 
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
                #ifndef USE_TWO_SWITCH_MODE
                convert_colors_to_index(false);
                #else
                //device_info[0].device_val = MIN_CCT_VALUE;
                #endif
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
                    
                    printf("LEVEL:%d\n", device_info[0].device_level);
                    #define LEDC_FADE_NO_WAIT   0

                    if(level_backup != device_info[0].device_level){
                         level_backup = device_info[0].device_level;

                        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[0], device_info[0].device_level));                
                        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[0]));

                        if(scene_group_switch_info.control_type == 0) { 
                            nuos_dali_normal_set_group_brightness(scene_group_switch_info.group_id[index], index, device_info[index].device_level);
                        }else{
                            #if(COMMUNICATION_MODE == COMM_MODE_ADDR_CTRL)
                            dali.set_dim_value(global_dali_id[0], device_info[index].device_level);
                            #elif(COMMUNICATION_MODE == COMM_MODE_GROUP_CTRL)
                            dali.set_group_level(global_group_id[0], device_info[index].device_level);
                            #elif(COMMUNICATION_MODE == COMM_MODE_BROADCAST)
                            dali.set_broadcast_level(map_1_255_to_100_255(device_info[index].device_level));
                            #endif
                        }
                        convert_dimming_to_index(is_long_press_brightness);
                        if(counts_level++ >= 10){
                            counts_level = 0;
                            nuos_set_color_temp_level_attribute(0);
                        }  
                        nuos_store_data_to_nvs(0);
                    }                      
                }
            }
            #ifndef USE_TWO_SWITCH_MODE
            else if(index == 2 || index == 3){
                if(device_info[0].color_or_fan_state){ 
                    convert_colors_to_index(is_long_press_brightness);
                    if(color_backup != device_info[0].device_val){
                        color_backup = device_info[0].device_val;

                        if(scene_group_switch_info.control_type == 0) { 
                            dali.set_group_color_cct(scene_group_switch_info.group_id[0], device_info[0].device_val);
                        }else{
                            #if(COMMUNICATION_MODE == COMM_MODE_ADDR_CTRL)
                            dali.set_color_temperature(global_dali_id[0], device_info[0].device_val); 
                            #elif(COMMUNICATION_MODE == COMM_MODE_GROUP_CTRL)
                            dali.set_group_color_cct(global_group_id[0], device_info[0].device_val);
                            #elif(COMMUNICATION_MODE == COMM_MODE_BROADCAST)
                            //dali.set_broadcast_color_rgb(0, 0, 0, device_info[0].device_level);
                            dali.set_color_temperature(dali.BROADCAST_C, device_info[0].device_val); 
                            #endif 
                        }
                        if(counts_color++ >= 10){
                            counts_color = 0;
                            nuos_set_color_temperature_attribute(0);
                        }                                        
                    }  
                }
                nuos_store_data_to_nvs(0);                      
            }else{
                         
            }
            #else
                else if(index == 1){
                    //printf("index_1:%d\n", index);                  
                    if(device_info[0].device_state){
      
                        if(device_info[0].level_up == 1){
                            if(device_info[0].device_val + COLOR_STEPS <= (MAX_CCT_VALUE)){
                                device_info[0].device_val += COLOR_STEPS;
                            } else {
                                device_info[0].device_val = MAX_CCT_VALUE;
                            }
                        }else{
                            if(device_info[0].device_val - COLOR_STEPS >= MIN_CCT_VALUE){
                                device_info[0].device_val -= COLOR_STEPS;  
                            }else {
                                device_info[0].device_val = MIN_CCT_VALUE;
                            } 
                        }

                        if(color_backup != device_info[0].device_val){
                            color_backup = device_info[0].device_val;
                            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[0], device_info[0].device_level));                
                            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[0]));

                            if(device_info[0].device_val >= MIN_CCT_VALUE){
                                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[1], map_cct_k_to_level(device_info[0].device_val)));
                            }else{
                                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[1], map_cct_k_to_level(MAX_CCT_VALUE)));                   
                            }
                            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[1]));
                            if(scene_group_switch_info.control_type == 0) { 
                                // for(int i=0; i<scene_group_switch_info.total_ids[0]; i++){
                                //     dali.set_color_temperature(scene_group_switch_info.device_ids[0][i], device_info[0].device_val);
                                // }
                                dali.set_group_color_cct(scene_group_switch_info.scene_ids[0], device_info[0].device_val);
                            }else{
                                #if(COMMUNICATION_MODE == COMM_MODE_ADDR_CTRL)
                                dali.set_color_temperature(global_dali_id[0], device_info[0].device_val); 
                                #elif(COMMUNICATION_MODE == COMM_MODE_GROUP_CTRL)
                                dali.set_group_color_cct(global_group_id[0], device_info[0].device_val);
                                #elif(COMMUNICATION_MODE == COMM_MODE_BROADCAST)
                                dali.set_color_temperature(dali.BROADCAST_C, device_info[0].device_val); 
                                #endif 
                            }
                            if(counts_color++ >= 10){
                                counts_color = 0;
                                nuos_set_color_temperature_attribute(0);
                            }                                        
                        } 
                    } 
                    nuos_store_data_to_nvs(0);                      
            }
            #endif

        // }else{
        //     printf("switch state not detected!!");
        // }
        return false;
    }

    extern "C" bool nuos_set_hardware_brightness_2(uint8_t index){
        //global_switch_state = SWITCH_PRESS_DETECTED;
        call_common_check_auto_off();
        if(index == 0){
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], device_info[index].device_level));                
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[index]));

            if(!device_info[0].color_or_fan_state){
                gpio_set_level(gpio_touch_led_pins[1], 0);
                //find_closest_index_2(device_info[0].device_level);
                device_info[0].ac_temperature = find_closest_index_2(device_info[0].device_level);
                set_dimming_control_leds(0);
            }else{
                gpio_set_level(gpio_touch_led_pins[1], 1);
                device_info[0].fan_speed = find_closest_index(device_info[0].device_val);
                set_color_temp_leds(0);
            }
        }else{
            if(device_info[0].color_or_fan_state){
                gpio_set_level(gpio_touch_led_pins[1], 1);
                device_info[0].fan_speed = find_closest_index(device_info[0].device_val);
                set_color_temp_leds(0);
            }else{
                gpio_set_level(gpio_touch_led_pins[1], 0);
                device_info[0].ac_temperature = find_closest_index_2(device_info[0].device_level);
                set_dimming_control_leds(0);
            }
        }
        #if 0
        else if(index == 1){
            convert_colors_to_index(is_long_press_brightness);
        }
        #endif
        #ifdef USE_CCT_TIME_SYNC
        if(device_info[0].color_or_fan_state){
            device_info[0].color_or_fan_state = false;
            gpio_set_level(gpio_touch_led_pins[1], 0);
            stop_color_temp_timer();
        }         
        #endif
        #ifdef USE_TWO_SWITCH_MODE
        if(index == 0){
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], device_info[index].device_level));                
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[index]));
        }else if(index == 1){
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], map_cct_k_to_level(device_info[0].device_val)));              
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[index]));
        }
        #endif
        return (gpio_touch_btn_pins[index]);        
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


    void set_fade_time_all_devices(uint8_t time, uint8_t rate){
        for(uint8_t index=0; index<TOTAL_ENDPOINTS; index++){
            for(int i=0; i<scene_group_switch_info.total_ids[index]; i++){
                printf("Set fade time %d rate %d for %d\n", time, rate, scene_group_switch_info.device_ids[index][i]);
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
                printf("Set fade time %d for %d\n", time, scene_group_switch_info.device_ids[index][i]);
                dali.set_fade_time(scene_group_switch_info.device_ids[index][i], time);
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
        }
    }
    
    void nuos_set_dali_fade_rate(uint8_t rate){
        for(uint8_t index=0; index<TOTAL_ENDPOINTS; index++){
            for(int i=0; i<scene_group_switch_info.total_ids[index]; i++){
                printf("Set fade rate %d for %d\n", rate, scene_group_switch_info.device_ids[index][i]);
                dali.set_fade_rate(scene_group_switch_info.device_ids[index][i], rate);
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
        }
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
            dali.set_group_color_cct(group_id, value);
            
        }
    } 
    extern "C" void nuos_dali_set_broadcast_color_temperature(uint8_t index, uint16_t value){
        if(last_color_temp != device_info[0].device_val){
            last_color_temp = device_info[0].device_val;
            dali.set_broadcast_color_cct(value);
        }
    } 
    extern "C" void nuos_dali_set_group_brightness(uint8_t group_id, uint8_t index, uint8_t value){
        dali.set_group_level(group_id, map_1_255_to_100_255(value));
    } 
    extern "C" void nuos_dali_normal_set_group_brightness(uint8_t group_id, uint8_t index, uint8_t value) {
        uint8_t val = map_1_255_to_100_255(value);
        // printf("Brightness set to %d\n", val);
        dali.set_group_level_normal(group_id, val);
    }
        
    void start_dali_led_blink_task(){
    
        if(start_dali_led_commissioning_task_flag){
            for(int i=0; i<TOTAL_LEDS; i++){
                if(i == 0){
                    if(!_toggle_){
                        ledc_set_duty(LEDC_MODE, pwm_channels[i], 0);            
                        ledc_update_duty(LEDC_MODE, pwm_channels[i]);
                    }else{
                        ledc_set_duty(LEDC_MODE, pwm_channels[i], 0xff);            
                        ledc_update_duty(LEDC_MODE, pwm_channels[i]);
                    }
                }else{
                    gpio_set_level(gpio_touch_led_pins[i], _toggle_);
                }
            }
            _toggle_ = !_toggle_;
        }
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

    uint8_t cct_counts = 0;
    extern "C" void nuos_dali_set_color_temperate(uint8_t index){
        device_info[index].device_val = cct_values[cct_counts];
        dali.set_color_temperature(dali.BROADCAST_C, cct_values[cct_counts]);  
        //#endif
        nuos_store_data_to_nvs(0);
    }


    void nuos_set_state_touch_leds(bool state){
        for(int i=0; i<TOTAL_LEDS; i++){
            set_leds(i, state);
        }
    }

    void set_all_leds_to_original_state(){
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
    
    


    // Add at top with other defines
    #define DALI_TASK_PRIORITY 6  // Higher than webserver tasks
    #define DALI_ADDRESSING_TIMEOUT_MS 30000  // 30 second timeout
    static TaskHandle_t dali_comm_task_handle = NULL;

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
        
        wifi_webserver_active_flag = 0;
        esp_stop_timer();
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
        wifi_webserver_active_flag = true;  
                            
        setNVSCommissioningFlag(0);
        setNVSWebServerEnableFlag(wifi_webserver_active_flag);                    
        esp_restart();	
        vTaskDelete(NULL);
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
    extern "C" void start_dali_addressing(uint8_t startAddresses, uint8_t numAddresses) {            
        recordsSemaphore = xSemaphoreCreateBinary();
        if (recordsSemaphore == NULL) {
            // Handle semaphore creation failure
            printf("Failed to create semaphore!\n");
            return;
        }    
        uint16_t  addr = (numAddresses & 0xff) | ((startAddresses & 0xff) << 8);
        xTaskCreate(esp_dali_init_node_task, "dali_task", 8192, &addr, TASK_PRIORITY_DALI_TASK, NULL);
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
            nuos_zb_set_hardware(i, false);
        }
    } 
    
    struct DaliMessage {
        uint8_t data[3];
        size_t len;
    };
    typedef enum
    {
        CMD_UNKNOWN,
        CMD_SET_DTR0,
        CMD_SET_DTR1,
        CMD_ENABLE_DT8,
        CMD_SET_COLOR_TEMP,
        CMD_COLOR_ACTIVATE,
        CMD_STORE_SCENE,
        CMD_RECALL_SCENE_BROADCAST
    } dali_cmd_t;





    void interpret_frame(uint8_t b1, uint8_t b2)
    {

        static uint8_t last_dtr0 = 0;
        static uint8_t last_dtr1 = 0;
        static uint8_t last_color_dtr0 = 0;
        static uint8_t last_color_dtr1 = 0;
        static uint8_t last_color_dtr2 = 0;
        static uint8_t enabled_device_type = 0xFF;   // 0x08 = DT8
        static uint8_t last_color_mode = 0;          // 0 = Tc/CCT, 1 = RGB

        uint8_t addr = 0xFF;
        uint8_t group = 0xFF;
        uint8_t scene = 0;
        bool is_broadcast = false;
        bool is_group = false;
        bool is_short = false;
        bool is_command = false;

        is_command = (b1 & 0x01) ? true : false;

        if (b1 == 0xFE || b1 == 0xFF) {
            is_broadcast = true;
        } else if (b1 & 0x80) {
            is_group = true;
            group = (b1 >> 1) & 0x0F;
        } else {
            is_short = true;
            addr = (b1 >> 1) & 0x3F;
        }
        // ---------------------------
        // SET_DTR0
        // ---------------------------
        if(b1 == 0xC3){
            last_color_dtr0 = last_dtr0;
            last_color_dtr1 = b2;
            //printf("SET_DTR1 = %d\n", ((last_color_dtr1<<8 ) | last_color_dtr0));
            return;
        }

        // -------------------------------------------------
        // Enable device type
        // -------------------------------------------------
        if (b1 == 0xC1) {
            if (b2 == 0x08) {
                enabled_device_type = 0x08;
            } else {
                enabled_device_type = 0xFF;
            }
            return;
        }
        // -------------------------------------------------
        // DT8 command handling
        // -------------------------------------------------
        if (enabled_device_type == 0x08) {
            if (b2 == 0xE7) {
                if (is_short) {
                    // DT8 command for short address
                } else if (is_group) {
                    // DT8 command for group address
                } else if (is_broadcast) {
                    // DT8 command for broadcast
                }
                return;
            }
        }        
        // ---------------------------
        // SET COLOR TEMPERATURE (DT8)
        // ---------------------------
        if(b2 == 0xE7){
            addr = (b1 >> 1) & 0x3F;
            //printf("SET COLOR TEMP → Device %d\n", addr);
            return;
        }
        // ---------------------------
        // COLOR ACTIVATE
        // ---------------------------
        if(b1 == 0xA3){
            last_dtr0 = b2;
            //printf("SET_DTR0 = %d\n", last_dtr0);
            return;
        }
        // ---------------------------
        // STORE SCENE
        // ---------------------------
        if(b2 >= 0x40 && b2 <= 0x4F){
            addr = (b1 >> 1) & 0x3F;
            scene = b2 - 0x40;

            // printf("STORE SCENE %d → Device %d  Level=%d Color=%d\n",
            //     scene, addr, last_dtr0, last_color_dtr0);

            for(int i=0; i<scene_group_switch_info.total_ids[0]; i++){
                if(addr == scene_group_switch_info.device_ids[0][i]){
                    scene_group_switch_info.device_scene[scene][i] = scene;  
                    scene_group_switch_info.device_level[scene][i] = last_dtr0;

                    uint16_t color_temp_mirek = ((last_color_dtr1 << 8) & 0xff00) | last_color_dtr0;
                    uint16_t kelvin_cct = 1000000 / color_temp_mirek;
                    //2000 to 6500
                    scene_group_switch_info.device_color[scene][i] = kelvin_cct;  
                    if(last_dtr0 == 0) scene_group_switch_info.device_state[scene][i]  = false;
                    else scene_group_switch_info.device_state[scene][i]  = true;
                    //printf("============DATA SAVED SUCCESSFULLY============\n");
                    switch_driver_gpios_intr_enabled(false);
                    dali.dali_rx_intr_enabled(false);
                    nuos_store_dali_scene_switch_data_to_nvs(&scene_group_switch_info);
                    dali.dali_rx_intr_enabled(true);
                    switch_driver_gpios_intr_enabled(true);
                    break;
                }
            }                
            return;
        }

        // ---------------------------
        // BROADCAST SCENE RECALL
        // ---------------------------
        if(b1 == 0xFF && b2 >= 0x10 && b2 <= 0x1F){
            scene = b2 - 0x10;
            //printf("BROADCAST RECALL SCENE %d\n", scene);
            // scene_group_switch_info.scene_ids[0] = scene;
            bool all_off = true;
            uint8_t max_level = 0;
            uint16_t max_cct = 2000;

            //printf("records:%d\n", scene_group_switch_info.total_ids[0]);
            if(scene_group_switch_info.total_ids[0] > 0){
                for(int j=0; j<scene_group_switch_info.total_ids[0]; j++){
                    //printf("Device %d found in Records!!\n", scene_group_switch_info.device_ids[0][j]);
                    if(scene_group_switch_info.device_scene[scene][j] == scene){
                        //printf("Scene %d found in Records!!\n", scene);
                        if(scene_group_switch_info.device_state[scene][j]){
                            all_off = false;
                        }
                        //if(scene_group_switch_info.device_level[scene][j] > max_level)
                            max_level = scene_group_switch_info.device_level[scene][j];

                        //if(scene_group_switch_info.device_color[scene][j] > max_cct)
                            max_cct = scene_group_switch_info.device_color[scene][j];        
                    }
                }  
                if(all_off){
                    device_info[0].device_state = false;
                    //device_info[0].device_level = 0;
                    process_dali_task(0, false);
                    nuos_set_state_attribute(0);
                }else{
                    device_info[0].device_state = true;
                    device_info[0].device_level = max_level;
                    device_info[0].device_val = max_cct;
                    printf("max_level:%d max_cct:%d\n", max_level, max_cct);
                    #ifndef USE_TWO_SWITCH_MODE
                    device_info[0].fan_speed = find_closest_index(max_cct);
                    device_info[0].ac_temperature = find_closest_index_2(device_info[0].device_level);
                    #endif

                    if(!device_info[0].device_state) {
                        #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
                            #ifndef USE_TWO_SWITCH_MODE
                            ledc_set_duty(LEDC_MODE, pwm_channels[0], 0);            
                            ledc_update_duty(LEDC_MODE, pwm_channels[0]);
                            #else
                            ledc_set_duty(LEDC_MODE, pwm_channels[0], 0);            
                            ledc_update_duty(LEDC_MODE, pwm_channels[0]);

                            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[1], 0));
                            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[1]));       
                            #endif             
                        #else
                            gpio_set_level(gpio_touch_led_pins[index], 0);
                        #endif
                        #ifdef USE_CCT_TIME_SYNC
                        if(device_info[0].color_or_fan_state){
                            device_info[0].color_or_fan_state = false;
                            gpio_set_level(gpio_touch_led_pins[1], 0);
                            stop_color_temp_timer();
                        }    
                        #endif 
                        nuos_set_state_attribute(0); 
                        //nuos_set_color_temp_level_attribute(0);          
                    } else {
                        #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
                            #ifndef USE_TWO_SWITCH_MODE
                            ledc_set_duty(LEDC_MODE, pwm_channels[0], device_info[0].device_level);            
                            ledc_update_duty(LEDC_MODE, pwm_channels[0]);
                            #else
                            ledc_set_duty(LEDC_MODE, pwm_channels[0], device_info[0].device_level);            
                            ledc_update_duty(LEDC_MODE, pwm_channels[0]);                    
                            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[1], map_cct_k_to_level(device_info[0].device_val)));
                            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[1]));
                            #endif
                        #else
                            gpio_set_level(gpio_touch_led_pins[index], 1);
                        #endif

                        gpio_set_level(gpio_touch_led_pins[1], device_info[0].color_or_fan_state);

                        #ifdef USE_CCT_TIME_SYNC
                        if(device_info[0].color_or_fan_state){
                            start_color_temp_timer();
                        }else{
                            stop_color_temp_timer();
                        }
                        #else
                        if(device_info[0].color_or_fan_state){
                            set_color_temp_leds(1);
                        }else{
                            set_dimming_control_leds(1);
                        }
                        #endif
                        nuos_set_state_attribute(0);
                        nuos_set_color_temp_level_attribute(0);
                        nuos_set_color_temperature_attribute(0);
                    }   
                }   
            }          
            return;
        }
        // -------------------------------------------------
        // Standard command frame
        // -------------------------------------------------
        if (is_command) {
            if (is_broadcast) {
                // printf("BROADCAST COMMAND %02X\n", b2);
            } else if (is_group) {
                // printf("GROUP COMMAND G=%u CMD=%02X\n", group, b2);
            } else if (is_short) {
                // printf("SHORT COMMAND A=%u CMD=%02X\n", addr, b2);
            }
            return;
        }
        

        // ---------------------------
        // ARC POWER CONTROL (DAPC)
        // ---------------------------
        if(b2 <= 0xFE) {
            // Broadcast
            if(b1 == 0xFE)
            {
                if(b2 == 0){
                    //printf("BROADCAST → OFF\n");
                    device_info[0].device_state = false;
                    //process_dali_task(0, false);
                }else{
                    //printf("BROADCAST → LEVEL %d\n", b2);
                    device_info[0].device_state = true;
                    device_info[0].device_level = b2;
                    //process_dali_task(0, false);        
                                 
                }
                return;
            }

            // Group command
            if(b1 & 0x80)
            {
                uint8_t group = (b1 >> 1) & 0x0F;
                if(b2 == 0)
                    printf("GROUP %d → OFF\n", group);
                else
                    printf("GROUP %d → LEVEL %d\n", group, b2);

                return;
            }

            // Short address command
            addr = (b1 >> 1) & 0x3F;

            if(b2 == 0){
                printf("DEVICE %d → OFF\n", addr);
            }else{
                printf("DEVICE %d → LEVEL %d\n", addr, b2);
            }
            return;
        }

        // ---------------------------
        // UNKNOWN
        // ---------------------------
        printf("UNKNOWN FRAME: %02X %02X\n", b1, b2);
    }

    static void receiveDaliFrame(void *arg) {
        DaliMessage msg;
        while (1) {

            if (rxFrameQueue != nullptr) {
                if(xQueueReceive(rxFrameQueue, &msg, portMAX_DELAY)== pdTRUE) {
                    interpret_frame(msg.data[0], msg.data[1]);
                }
            }else{
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
        }
    }

    extern "C" void init_dali_hw(){
       
        dali.begin(&isr_service_installed);
        if(wifi_webserver_active_flag == 0){
            rxFrameQueue = xQueueCreate(10, sizeof(DaliMessage));
            if (rxFrameQueue == nullptr) {
                return;
            } 
            dali.begin_rx(&isr_service_installed, rxFrameQueue); 
            xTaskCreate(receiveDaliFrame, "dali_task_2", TASK_STACK_SIZE_DALI_RX_FRAME, NULL, TASK_PRIORITY_DALI_RX_FRAME, NULL);
        }else{
            
        }
            
    }

    void dali_rx_intr_enabled(bool enable){
        dali.dali_rx_intr_enabled(enable);
    }

#endif



