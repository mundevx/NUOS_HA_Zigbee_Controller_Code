
 #include "app_hardware_driver.h"
 #include "app_zigbee_clusters.h"
#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_PHASE_CUT_DIMMABLE_LIGHT)
    #include "esp_log.h"
    #include "esp_err.h"
    #include <esp_log.h>
    #include <esp_check.h>

    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "freertos/semphr.h"
   
    #include "driver/ledc.h"
    #include "driver/gpio.h"

    #include "esp_timer.h"


    #define USE_NVS_STORE

    #define LEDC_TIMER              		                    LEDC_TIMER_0
    #define LEDC_MODE               		                    LEDC_LOW_SPEED_MODE

    #define LEDC_DUTY_RES          		 	                    LEDC_TIMER_8_BIT // Set duty resolution to 13 bits
    #define LEDC_DUTY               		                    100      // Set duty to 50%. ((2 ** 8) - 1) * 50% = 4095
    #define LEDC_MAX_DUTY           		                    254     //8191
    #define LEDC_FREQUENCY                                      (1000) // Frequency in Hz (5 kHz)
    #define LEDC_FADE_TIME                                      (500) // Fade time in milliseconds (1 second)


    uint8_t pwm_channels[TOTAL_ENDPOINTS]                       = { LEDC_CHANNEL_0, LEDC_CHANNEL_1 };
    int LEDC_TIMER_T[TOTAL_ENDPOINTS]                           = { LEDC_TIMER_0, LEDC_TIMER_0};    


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

    bool is_init_done = false;
    static bool state = false;
    uint8_t level = 0;
    // gpio_num_t gpio_total_init_pins[TOTAL_LEDS] = {0,0};

    #define DEFAULT_BRIGHTNESS    50
    // Dimming Parameters
    const int MIN_DELAY = 50;   // 50μs delay for full brightness
    const int MAX_DELAY = 9000; // 9000μs delay for minimum brightness (50Hz AC)
    const int PULSE_WIDTH = 50; // 50μs triac trigger pulse width

    uint8_t zigbee_level_to_percent(uint8_t level);
    uint32_t brightness_to_delay(uint8_t brightness);

    typedef struct {
        int led_pin;
        bool on;
        uint8_t current_level;  // 0-100%
        uint8_t target_level;
        volatile uint32_t firing_delay;
        esp_timer_handle_t triac_timer;
        SemaphoreHandle_t mutex;
    } light_channel_t;

    light_channel_t light1 = {0};
    light_channel_t light2 = {0};

    void IRAM_ATTR triac_timer_cb1(void *arg) {
        gpio_set_level(gpio_load_pins[0], 1);
        esp_rom_delay_us(PULSE_WIDTH);
        gpio_set_level(gpio_load_pins[0], 0);
    }

    void IRAM_ATTR triac_timer_cb2(void *arg) {
        gpio_set_level(gpio_load_pins[1], 1);
        esp_rom_delay_us(PULSE_WIDTH);
        gpio_set_level(gpio_load_pins[1], 0);
    }

    #define DIMMING_STEPS 5  // Step size for fading (adjust as needed)

    uint8_t increase_level(uint8_t level) {
        // Increase level by DIMMING_STEPS, but don't exceed maximum brightness
        if (level + DIMMING_STEPS > MAX_DIM_LEVEL_VALUE) {
            return MAX_DIM_LEVEL_VALUE;
        }
        return level + DIMMING_STEPS;
    }

    uint8_t decrease_level(uint8_t level) {
        // Decrease level by DIMMING_STEPS, but don't go below minimum
        if (level < DIMMING_STEPS) {
            return 0;
        }
        return level - DIMMING_STEPS;
    }

    void update_led(int led_pin, uint8_t current_level, bool on) {
        if (!is_init_done) return;
        
        // Find the PWM channel index for this LED pin
        int pwm_index = -1;
        for (int i = 0; i < TOTAL_LEDS; i++) {
            if (gpio_touch_led_pins[i] == led_pin) {
                pwm_index = i;
                break;
            }
        }
        if (pwm_index < 0) return;  // Pin not found
        
        if (!on || current_level == 0) {
            // Turn off LED completely
            ledc_set_duty(LEDC_MODE, pwm_channels[pwm_index], 0);
            ledc_update_duty(LEDC_MODE, pwm_channels[pwm_index]);
        } else {
            // Convert percentage to PWM duty cycle
            uint32_t duty = (current_level * LEDC_MAX_DUTY) / 100;
            // Set immediately without fading
            ledc_set_duty(LEDC_MODE, pwm_channels[pwm_index], duty);
            ledc_update_duty(LEDC_MODE, pwm_channels[pwm_index]);
        }
    }
    void update_light_channel(light_channel_t *light) {
        xSemaphoreTake(light->mutex, portMAX_DELAY);
        
        if (!light->on && light->current_level > 0) {
            // Fade out
            light->current_level = decrease_level(light->current_level);
        } else if (light->on && light->current_level < light->target_level) {
            // Fade in
            light->current_level = increase_level(light->current_level);
        }
        
        // Update hardware
        uint32_t delay = brightness_to_delay(light->current_level);
        // portENTER_CRITICAL(&light->spinlock);
        light->firing_delay = delay;
        // portEXIT_CRITICAL(&light->spinlock);
        
        // Update LED indicator
        update_led(light->led_pin, light->current_level, light->on);
        
        xSemaphoreGive(light->mutex);
    }
    void dimming_task(void *arg) {
        while (1) {
            update_light_channel(&light1);
            update_light_channel(&light2);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
    uint32_t brightness_to_delay(uint8_t brightness) {
        if (brightness <= 5) return MAX_DELAY;
        if (brightness >= 100) return MIN_DELAY;
        
        // Exponential curve calculation
        float factor = (100.0 - brightness) / 100.0;
        return MIN_DELAY + (uint32_t)((MAX_DELAY - MIN_DELAY) * pow(factor, 2.5));
    }

    void handle_onoff(light_channel_t *light, bool on) {
        xSemaphoreTake(light->mutex, portMAX_DELAY);
        
        if (on) {
            // Turn on - restore to last level
            light->on = true;
            light->target_level = light->current_level > 0 ? 
                                light->current_level : DEFAULT_BRIGHTNESS;
        } else {
            // Turn off
            light->on = false;
            light->target_level = 0;
        }
        
        xSemaphoreGive(light->mutex);
    }

    void handle_level(light_channel_t *light, uint8_t zigbee_level) {
        xSemaphoreTake(light->mutex, portMAX_DELAY);
        
        // Convert Zigbee level (0-254) to percentage (0-100)
        uint8_t level_percent = zigbee_level_to_percent(zigbee_level);
        
        if (level_percent == 0) {
            light->on = false;
        } else {
            light->on = true;
            light->target_level = level_percent;
        }
        
        xSemaphoreGive(light->mutex);
    }

    uint8_t zigbee_level_to_percent(uint8_t level) {
        if (level == 0) return 0;
        return (uint8_t)((level * 100) / 254);
}

    void light_init(light_channel_t *light, gpio_num_t triac_pin, gpio_num_t led_pin) {
        // Hardware init
        gpio_set_direction(triac_pin, GPIO_MODE_OUTPUT);
        gpio_set_direction(led_pin, GPIO_MODE_OUTPUT);
        light->led_pin = led_pin;
        
        // Timer init
        esp_timer_create_args_t timer_cfg = {
            .callback = (triac_pin == gpio_load_pins[0]) ? triac_timer_cb1 : triac_timer_cb2,
            .name = "triac_timer"
        };
        esp_timer_create(&timer_cfg, &light->triac_timer);
        
        // Sync objects
        light->mutex = xSemaphoreCreateMutex();
        // light->spinlock = portMUX_INITIALIZER_UNLOCKED;
        
        // Default state
        light->on = false;
        light->current_level = 0;
        light->target_level = DEFAULT_BRIGHTNESS;
        light->firing_delay = MAX_DELAY;
    }

    void IRAM_ATTR zero_crossing_isr(void *arg) {
        // Channel 1
        if (light1.current_level > 0) {
            esp_timer_stop(light1.triac_timer);
            esp_timer_start_once(light1.triac_timer, light1.firing_delay);
        }
        
        // Channel 2
        if (light2.current_level > 0) {
            esp_timer_stop(light2.triac_timer);
            esp_timer_start_once(light2.triac_timer, light2.firing_delay);
        }
}


    #ifdef USE_FADING
    void init_fading(){

    } 
    #endif 
    void nuos_zb_init_hardware(){
        // for(int i=0; i<TOTAL_LOADS; i++){
        //     gpio_total_init_pins[i] = gpio_load_pins[i];
        //     // gpio_set_drive_capability(gpio_load_pins[i], GPIO_DRIVE_CAP_3);
        // }
        // for(int j=0; j<TOTAL_LEDS; j++){
        //     gpio_total_init_pins[j] = gpio_touch_led_pins[j];
        //     // gpio_set_drive_capability(gpio_touch_led_pins[j], GPIO_DRIVE_CAP_3);
        // }        
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
                .timer_sel      = LEDC_TIMER_T[index],
                .intr_type      = LEDC_INTR_DISABLE,
                .gpio_num       = gpio_touch_led_pins[index],
                .duty           = level, // Set duty to 0%
                .hpoint         = 0
            };
            ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
        } 
        #ifdef USE_FADING
            init_fading();
        #endif      
        
        // Initialize lights
        light_init(&light1, gpio_load_pins[0], gpio_touch_led_pins[0]);
        light_init(&light2, gpio_load_pins[1], gpio_touch_led_pins[1]);
        
        // Create tasks
        xTaskCreate(dimming_task, "dimming_task", 4096, NULL, 8, NULL);        
        is_init_done = true;             
    }


    void nuos_on_off_led(uint8_t index, uint8_t _state){
        if(is_init_done){
            if(index < TOTAL_LEDS){
                if(_state)  {
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], LEDC_MAX_DUTY));
                }else{
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], 0));
                }            
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[index]));
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
                        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], level));
                        // ledc_set_fade_time_and_start(LEDC_MODE, pwm_channels[index], 0, LEDC_FADE_TIME, LEDC_FADE_WAIT_DONE);  
                        if(index == 0){
                            light1.on = false;
                            light1.target_level = 0;  
                        }else if(index == 1){
                            light2.on = false;
                            light2.target_level = 0;
                        }
                    }else{
                        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], device_info[index].device_level));
                        // ledc_set_fade_time_and_start(LEDC_MODE, pwm_channels[index], device_info[index].device_level, LEDC_FADE_TIME, LEDC_FADE_WAIT_DONE);
                        if(index == 0){
                            // // Turn on - restore to last level
                            // light1.on = true;
                            // light1.target_level = light1.current_level > 0 ? 
                            //                     light1.current_level : DEFAULT_BRIGHTNESS;


                            // Convert Zigbee level (0-254) to percentage (0-100)
                            uint8_t level_percent = zigbee_level_to_percent(device_info[index].device_level);
                            
                            if (level_percent == 0) {
                                light1.on = false;
                            } else {
                                light1.on = true;
                                light1.target_level = level_percent;
                            }
                        }else if(index == 1){
                            // // Turn on - restore to last level
                            // light2.on = true;
                            // light2.target_level = light2.current_level > 0 ? 
                            //                     light2.current_level : DEFAULT_BRIGHTNESS;


                            // Convert Zigbee level (0-254) to percentage (0-100)
                            uint8_t level_percent = zigbee_level_to_percent(device_info[index].device_level);
                            
                            if (level_percent == 0) {
                                light2.on = false;
                            } else {
                                light2.on = true;
                                light2.target_level = level_percent;
                            }
                        }
        

                    } 
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[index]));                    
                }
            #else
                if(!device_info[index].device_state){
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], 0));
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], 0));
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[index]));
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[index]));
                }else{
                    if(device_info[index].device_level >= MIN_DIM_LEVEL_VALUE){
                        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], device_info[index].device_level));
                        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], device_info[index].device_level));
                        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[index]));
                        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[index]));
                    }else{
                        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], MIN_DIM_LEVEL_VALUE));
                        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], MIN_DIM_LEVEL_VALUE));
                        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[index]));
                        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[index]));                   
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
            if(device_info[index].device_level <= MIN_DIM_LEVEL_VALUE){
                device_info[index].dim_up = 1;
            }else if(device_info[index].device_level >= MAX_DIM_LEVEL_VALUE){
                device_info[index].dim_up = 0;
            }
        }
    }

    static uint8_t level_backup[4] = {0, 0, 0, 0};

    void set_load(uint8_t index, uint8_t value){
        if(is_init_done){
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], value));           
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[index])); 

            // ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], value)); 
            // ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[index]));
            uint8_t level_percent = zigbee_level_to_percent(value);
            if(index == 1){
            
            light1.on = true;
            light1.target_level = level_percent;                
            }else{
            
            light2.on = true;
            light2.target_level = level_percent;

            }


        }
        #ifdef USE_NVS_STORE 
        nuos_store_data_to_nvs(index);
        #endif        
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
                if(level_backup[index] != device_info[index].device_level){
                    level_backup[index] = device_info[index].device_level;
                    printf("Set state:%d, Load:%d\n", device_info[index].device_state, level_backup[index]);
                    set_load(index, device_info[index].device_level);
                }             
            }
        }
        return false;
    }
#endif

