
#include "app_hardware_driver.h"
#include "esp_zigbee_core.h"
#include "app_zigbee_misc.h"

#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2CH_CURTAIN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN_SWITCH)



#include "esp_timer.h"
#include "esp_log.h"
#include "app_zigbee_clusters.h"

    //#define OLD_CURTAIN_BOARD
    int elapsed_sec = 0;
    static int8_t direction = 1;  // 1 = opening (0 → 100), -1 = closing (100 → 0)
    uint8_t curtain_state = 0;
    bool nuos_check_state_touch_leds();
    void init_curtain_timer();
    void pause_curtain_timer();
    static esp_timer_handle_t periodic_timer;
    static bool timer_paused = false;
    
void set_harware(uint8_t index, uint8_t is_toggle);

typedef enum {
    CURTAIN_DIR_STOPPED = 0,
    CURTAIN_DIR_OPENING,
    CURTAIN_DIR_CLOSING
} curtain_dir_t;

static curtain_dir_t curtain_dir = CURTAIN_DIR_STOPPED;
static uint8_t       target_pct  = 0;          // for GOTO command

static inline uint8_t sec_to_pct(uint32_t sec)
{
    return (sec * 100) / device_info[0].device_val;
}
static inline uint32_t pct_to_sec(uint8_t pct)
{
    return (pct * device_info[0].device_val) / 100;
}

/* New generic handler for “Go-to-percentage” */
uint8_t curtain_cmd_goto_pct(uint8_t pct)   // 0-100
{
    /* Clamp and ignore if already there */
    if (pct > 100) pct = 100;
    if (device_info[0].device_level == pct) return device_info[0].fan_speed;

    if(device_info[0].device_level > 100) device_info[0].device_level = 100;
    /* Determine direction */
    curtain_dir  = (pct > device_info[0].device_level)
                   ? CURTAIN_DIR_OPENING
                   : CURTAIN_DIR_CLOSING;
    target_pct   = pct;

    /* Sync elapsed_sec with current level */
    elapsed_sec  = pct_to_sec(device_info[0].device_level);

    /* Issue proper Zigbee motor command */
    curtain_state = (curtain_dir == CURTAIN_DIR_OPENING)
                    ? ESP_ZB_ZCL_CMD_WINDOW_COVERING_UP_OPEN
                    : ESP_ZB_ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE;

    esp_timer_start_periodic(periodic_timer, 1e6);   // 1s Tick
    timer_paused = false;
    return curtain_state;
}
void curtain_cmd_open(void)
{
    curtain_cmd_goto_pct(100);
}

void curtain_cmd_close(void)
{
    curtain_cmd_goto_pct(0);
}

void curtain_cmd_stop(void)
{
    pause_curtain_timer();
    curtain_dir = CURTAIN_DIR_STOPPED;
}

void periodic_timer_callback(void *arg)
{
    if (timer_paused || curtain_dir == CURTAIN_DIR_STOPPED) return;

    if (curtain_dir == CURTAIN_DIR_OPENING && elapsed_sec < device_info[0].device_val)
        elapsed_sec++;
    else if (curtain_dir == CURTAIN_DIR_CLOSING && elapsed_sec > 0)
        elapsed_sec--;

    device_info[0].device_level = sec_to_pct(elapsed_sec);

    nuos_report_curtain_blind_state(0, device_info[0].device_level);  
    printf("t=%u s  level=%u (target %u %%)\n", elapsed_sec, device_info[0].device_level, target_pct);
    // --- FIX: Stop when passing or matching target ---
    bool reached_limit  = (device_info[0].device_level == 0 || device_info[0].device_level == 100);
    bool reached_target = false;
    if (curtain_dir == CURTAIN_DIR_OPENING)
        reached_target = (device_info[0].device_level >= target_pct);
    else if (curtain_dir == CURTAIN_DIR_CLOSING)
        reached_target = (device_info[0].device_level <= target_pct);

    if (reached_limit || reached_target) {
        pause_curtain_timer();
        curtain_dir = CURTAIN_DIR_STOPPED;
        // Force to exact target if not at limit
        if (!reached_limit) {
            device_info[0].device_level = target_pct;
            elapsed_sec = pct_to_sec(target_pct);
            nuos_report_curtain_blind_state(0, device_info[0].device_level);          
        }
        printf("Auto-stop at %s\n", reached_limit ? "limit" : "requested percentage");
    }
}

    void init_curtain_timer(){
        const esp_timer_create_args_t periodic_timer_args = {
            .callback = &periodic_timer_callback,
            .name = "periodic"
        };
        esp_timer_create(&periodic_timer_args, &periodic_timer);
    }

    // Call this function from any event (button, command, etc.) to pause
    void pause_curtain_timer()
    {
        #ifdef TUYA_ATTRIBUTES
            if (!timer_paused) {
                esp_timer_stop(periodic_timer);
                timer_paused = true;
                printf("Timer paused at %d seconds (%d%%)\n", elapsed_sec, device_info[0].device_level);
            }
        #endif
        //set load pins
        gpio_set_level(gpio_load_pins[0], 0);
        //set load pins
        #ifdef OLD_CURTAIN_BOARD
        gpio_set_level(gpio_load_pins[1], 0);
        #endif
        nuos_store_data_to_nvs(0);
    }

    void nuos_zb_init_hardware(){
        uint32_t pins = 0; 
        for(int i=0; i<TOTAL_LEDS; i++){
            pins |= 1ULL << gpio_touch_led_pins[i];
        }  
        for(int j=0; j<TOTAL_LOADS; j++){
            pins |= 1ULL << gpio_load_pins[j];
        }         
        gpio_config_t io_conf;
        // Configure the output pin
        io_conf.intr_type = GPIO_INTR_DISABLE;       // Disable interrupt
        io_conf.mode = GPIO_MODE_OUTPUT;             // Set as output mode
        io_conf.pin_bit_mask = pins;                 // Bit mask for the output pin
        io_conf.pull_down_en = 0;                    // Disable pull-down mode
        io_conf.pull_up_en = 0;                      // Disable pull-up mode
        gpio_config(&io_conf);	

        timer_paused = true;
        elapsed_sec = (device_info[0].device_level * device_info[0].device_val) / 100;
        #ifdef TUYA_ATTRIBUTES
        init_curtain_timer();
        #endif 
    }

    static bool state = false;
    void nuos_zb_set_hardware_led_for_zb_commissioning(uint8_t is_toggle){
        #ifdef USE_RGB_LED
            nuos_toggle_rgb_led();
        #else
            if(TOTAL_LEDS >= TOTAL_LEDS_SHOW_ON_COMMISSIONING){
                if(is_toggle>0) {
                    state = !state;
                    for(int index=TOTAL_LEDS-TOTAL_LEDS_SHOW_ON_COMMISSIONING; index<TOTAL_LEDS; index++){
                        gpio_set_level(gpio_touch_led_pins[index], state);
                    }
                }else{
                    for(int i=0; i<TOTAL_ENDPOINTS; i++){
                        nuos_zb_set_hardware_curtain(i, false);
                    }
                }
            }
        #endif
    }

    void set_curtain_load(uint8_t cur_mode) {
        if(cur_mode == CURTAIN_OPEN){
            gpio_set_level(gpio_load_pins[0], 1);
            //set load pins
            gpio_set_level(gpio_load_pins[1], 1);  //open
            printf("CURTAIN_OPEN\n");
        }else if(cur_mode == CURTAIN_CLOSE){
            gpio_set_level(gpio_load_pins[0], 1);
            //set load pins
            gpio_set_level(gpio_load_pins[1], 0);  //close
            printf("CURTAIN_CLOSE\n");
        }else{
            gpio_set_level(gpio_load_pins[0], 0);  //stop
            printf("CURTAIN_STOP\n");
        }
    }

    void set_curtain_percentage(uint8_t value){
        // if(device_info[0].device_level < value){  // 100 to 0
        //     device_info[0].device_state = false;
        //     set_harware(0, false);
        // }else if(device_info[0].device_level > value){ // 0 to 100
        //     device_info[1].device_state = false;
        //     set_harware(1, false);
        // }else{
        //     //nothing to do here, curtrain already on this position
        // }
        // curtain_cmd_goto_pct(value);

        uint8_t state = curtain_cmd_goto_pct(value);
        nuos_report_curtain_blind_state(0, value); 
        if(state == 0) nuos_zb_set_hardware_curtain(0, CURTAIN_OPEN); 
        else if(state == 1) nuos_zb_set_hardware_curtain(0, CURTAIN_CLOSE); 
         

    }
    void set_harware(uint8_t index, uint8_t is_toggle){
        #ifdef OLD_CURTAIN_BOARD
        if(index == 0){
            if(device_info[0].device_state == CURTAIN_OPEN){
                //set load pins
                curtain_cmd_stop();
                printf("CURTAIN_STOP_0\n");
            }else{
                //set load pins
                device_info[0].device_state = CURTAIN_OPEN;
                gpio_set_level(gpio_load_pins[0], 1);
                printf("CURTAIN_OPEN_0\n");

                device_info[0].device_state = CURTAIN_OPEN;
                gpio_set_level(gpio_touch_led_pins[0], CURTAIN_OPEN);
                //set load pins
                gpio_set_level(gpio_load_pins[0], CURTAIN_OPEN);
                gpio_set_level(gpio_touch_led_pins[1], CURTAIN_CLOSE);
                //set load pins
                gpio_set_level(gpio_load_pins[1], CURTAIN_CLOSE);
                
                #ifdef TUYA_ATTRIBUTES
                curtain_cmd_open();  
                #endif                
            }            
        }else if(index == 1){ 
            if(device_info[0].device_state == CURTAIN_CLOSE){
                //set load pins
                curtain_cmd_stop();
                printf("CURTAIN_STOP_1\n");
            }else{
                //set load pins
                gpio_set_level(gpio_load_pins[0], 1);
                device_info[0].device_state = CURTAIN_CLOSE;
                printf("CURTAIN_CLOSE_1\n");
                gpio_set_level(gpio_touch_led_pins[0], CURTAIN_CLOSE);
                //set load pins
                gpio_set_level(gpio_load_pins[0], CURTAIN_CLOSE);
                gpio_set_level(gpio_touch_led_pins[1], CURTAIN_OPEN);
                //set load pins
                gpio_set_level(gpio_load_pins[1], CURTAIN_OPEN); 
                
                #ifdef TUYA_ATTRIBUTES
                curtain_cmd_close(); 
                #endif                
            }            
        }else{
            curtain_cmd_stop();
        }
        nuos_store_data_to_nvs(0);
        #else
        if(index == 0){
            device_info[0].fan_speed = 0;
            if(device_info[1].device_state){
                device_info[1].device_state = false;
                device_info[0].device_state = false;
                
            }
            if(device_info[0].device_state){
                device_info[0].device_state = false;
                //set load pins
                set_curtain_load(CURTAIN_STOP);
                #ifdef TUYA_ATTRIBUTES
                curtain_cmd_stop();
                #endif
            }else{
                //set load pins
                device_info[0].device_state = true;
                set_curtain_load(CURTAIN_OPEN);
                #ifdef TUYA_ATTRIBUTES
                curtain_cmd_open();  
                #endif                
            }
            
            gpio_set_level(gpio_touch_led_pins[0], 1);
            gpio_set_level(gpio_touch_led_pins[1], 0);

            nuos_store_data_to_nvs(0);
            nuos_store_data_to_nvs(1);
        }else if(index == 1){ 
            device_info[0].fan_speed = 1;
            if(device_info[0].device_state){
                device_info[0].device_state = false; 
                device_info[1].device_state = false;
            }
            
            if(device_info[1].device_state){
                device_info[1].device_state = false;
                set_curtain_load(CURTAIN_STOP);
                #ifdef TUYA_ATTRIBUTES
                curtain_cmd_stop();
                #endif
            }else{
                set_curtain_load(CURTAIN_CLOSE);
                device_info[1].device_state = true;
                #ifdef TUYA_ATTRIBUTES
                curtain_cmd_close(); 
                #endif                
            }
            
            gpio_set_level(gpio_touch_led_pins[0], 0);
            gpio_set_level(gpio_touch_led_pins[1], 1); 
            nuos_store_data_to_nvs(0);
            nuos_store_data_to_nvs(1);
        }else{
            curtain_cmd_stop();
        }
        #endif
  
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2CH_CURTAIN)
            if(index == 2){
                device_info[1].device_state = CURTAIN_OPEN;
                gpio_set_level(gpio_touch_led_pins[2], CURTAIN_OPEN);
                //set load pins
                gpio_set_level(gpio_load_pins[2], CURTAIN_OPEN);
                gpio_set_level(gpio_touch_led_pins[3], 0);
                //set load pins
                gpio_set_level(gpio_load_pins[3], 0);  
            }else if(index == 3){
                device_info[1].device_state = CURTAIN_CLOSE;
                gpio_set_level(gpio_touch_led_pins[2], CURTAIN_CLOSE);
                //set load pins
                gpio_set_level(gpio_load_pins[2], CURTAIN_CLOSE);
                gpio_set_level(gpio_touch_led_pins[3], 1);
                //set load pins
                gpio_set_level(gpio_load_pins[3], 1); 
            }
            nuos_store_data_to_nvs(1);
        #endif
        
    }

    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN_SWITCH)
        void nuos_zb_set_hardware_curtain(uint8_t index, uint8_t is_toggle){
            if(index == 0){
                if(device_info[0].fan_speed == ESP_ZB_ZCL_CMD_WINDOW_COVERING_UP_OPEN){
                    //device_info[0].device_state = ESP_ZB_ZCL_CMD_WINDOW_COVERING_UP_OPEN;
                    gpio_set_level(gpio_touch_led_pins[0], ESP_ZB_ZCL_CMD_WINDOW_COVERING_UP_OPEN);
                    //set load pins
                    gpio_set_level(gpio_load_pins[0], ESP_ZB_ZCL_CMD_WINDOW_COVERING_UP_OPEN);
                    gpio_set_level(gpio_touch_led_pins[1], ESP_ZB_ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE);
                    //set load pins
                    gpio_set_level(gpio_load_pins[1], ESP_ZB_ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE); 
                }else if(device_info[0].fan_speed == ESP_ZB_ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE){
                    //device_info[0].device_state = ESP_ZB_ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE;
                    gpio_set_level(gpio_touch_led_pins[0], ESP_ZB_ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE);
                    //set load pins
                    gpio_set_level(gpio_load_pins[0], ESP_ZB_ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE);
                    gpio_set_level(gpio_touch_led_pins[1], ESP_ZB_ZCL_CMD_WINDOW_COVERING_UP_OPEN);
                    //set load pins
                    gpio_set_level(gpio_load_pins[1], ESP_ZB_ZCL_CMD_WINDOW_COVERING_UP_OPEN);
                }else if(device_info[0].fan_speed == ESP_ZB_ZCL_CMD_WINDOW_COVERING_STOP){
                    gpio_set_level(gpio_load_pins[0], ESP_ZB_ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE);
                    gpio_set_level(gpio_load_pins[1], ESP_ZB_ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE);
                }
                nuos_store_data_to_nvs(0);
            }
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2CH_CURTAIN) 
                if(index == 1){
                    if(device_info[1].device_state == CURTAIN_OPEN){
                        device_info[1].device_state = CURTAIN_OPEN;
                        gpio_set_level(gpio_touch_led_pins[2], CURTAIN_OPEN);
                        //set load pins
                        gpio_set_level(gpio_load_pins[2], CURTAIN_OPEN);
                        gpio_set_level(gpio_touch_led_pins[3], 0);
                        //set load pins
                        gpio_set_level(gpio_load_pins[3], 0);  
                    }else if(device_info[1].device_state == CURTAIN_CLOSE){
                        device_info[1].device_state = CURTAIN_CLOSE;
                        gpio_set_level(gpio_touch_led_pins[2], CURTAIN_CLOSE);
                        //set load pins
                        gpio_set_level(gpio_load_pins[2], CURTAIN_CLOSE);
                        gpio_set_level(gpio_touch_led_pins[3], 1);
                        //set load pins
                        gpio_set_level(gpio_load_pins[3], 1); 
                    }
                    nuos_store_data_to_nvs(1);
                }    
            #endif 
            #ifdef USE_RGB_LED
                nuos_rgb_trigger_blink();
            #endif        
        }

    #else

        void nuos_zb_set_hardware_curtain(uint8_t index, uint8_t device_state){
            printf("index:%d  device_state:%d\n", index, device_state);
            if(index == 0){
                if(device_state == CURTAIN_OPEN){
                    device_info[0].device_state = false;
                    device_info[1].device_state = false;
                    #ifdef OLD_CURTAIN_BOARD
                        gpio_set_level(gpio_touch_led_pins[0], CURTAIN_OPEN);
                        //set load pins
                        gpio_set_level(gpio_load_pins[0], CURTAIN_OPEN);
                        gpio_set_level(gpio_touch_led_pins[1], CURTAIN_CLOSE);
                        //set load pins
                        gpio_set_level(gpio_load_pins[1], CURTAIN_CLOSE); 
                    #else
                        gpio_set_level(gpio_touch_led_pins[0], 1);
                        //set load pins
                        gpio_set_level(gpio_load_pins[0], 1);
                        gpio_set_level(gpio_touch_led_pins[1], 0);
                        //set load pins
                        gpio_set_level(gpio_load_pins[1], 1); 
                    #endif
                }else if(device_state == CURTAIN_CLOSE){
                    device_info[0].device_state = false;
                    device_info[1].device_state = false;
                    
                    gpio_set_level(gpio_touch_led_pins[0], 0);
                    //set load pins
                    gpio_set_level(gpio_load_pins[0], 1);
                    gpio_set_level(gpio_touch_led_pins[1], 1);
                    //set load pins
                    gpio_set_level(gpio_load_pins[1], 0);
                }
                nuos_store_data_to_nvs(0); 
                nuos_store_data_to_nvs(1); 
            }
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2CH_CURTAIN) 
                if(index == 1){
                    if(device_info[1].device_state == CURTAIN_OPEN){
                        device_info[1].device_state = CURTAIN_OPEN;
                        gpio_set_level(gpio_touch_led_pins[2], CURTAIN_OPEN);
                        //set load pins
                        gpio_set_level(gpio_load_pins[2], CURTAIN_OPEN);
                        gpio_set_level(gpio_touch_led_pins[3], 0);
                        //set load pins
                        gpio_set_level(gpio_load_pins[3], 0);  
                    }else if(device_info[1].device_state == CURTAIN_CLOSE){
                        device_info[1].device_state = CURTAIN_CLOSE;
                        gpio_set_level(gpio_touch_led_pins[2], CURTAIN_CLOSE);
                        //set load pins
                        gpio_set_level(gpio_load_pins[2], CURTAIN_CLOSE);
                        gpio_set_level(gpio_touch_led_pins[3], 1);
                        //set load pins
                        gpio_set_level(gpio_load_pins[3], 1); 
                    }
                    nuos_store_data_to_nvs(1);
                }    
            #endif 
            #ifdef USE_RGB_LED
                nuos_rgb_trigger_blink();
            #endif        
        }
    #endif
        
    // #endif

    void nuos_zb_set_leds_only(uint8_t index, uint8_t is_toggle){
        if(index == 0){
            if(device_info[0].device_state == CURTAIN_OPEN){
                device_info[0].device_state = CURTAIN_OPEN;
                gpio_set_level(gpio_touch_led_pins[0], CURTAIN_OPEN);
                gpio_set_level(gpio_touch_led_pins[1], CURTAIN_CLOSE);
            }else if(device_info[0].device_state == CURTAIN_CLOSE){
                device_info[0].device_state = CURTAIN_CLOSE;
                gpio_set_level(gpio_touch_led_pins[0], CURTAIN_CLOSE);
                gpio_set_level(gpio_touch_led_pins[1], CURTAIN_OPEN);
            }
        }
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2CH_CURTAIN) 
            if(index == 1){
                if(device_info[1].device_state == CURTAIN_OPEN){
                    device_info[1].device_state = CURTAIN_OPEN;
                    gpio_set_level(gpio_touch_led_pins[2], CURTAIN_OPEN);
                    gpio_set_level(gpio_touch_led_pins[3], 0);
                }else if(device_info[1].device_state == CURTAIN_CLOSE){
                    device_info[1].device_state = CURTAIN_CLOSE;
                    gpio_set_level(gpio_touch_led_pins[2], CURTAIN_CLOSE);
                    gpio_set_level(gpio_touch_led_pins[3], 1);
                }
            }
            
        #endif
        
    }

    void nuos_zb_set_hardware(uint8_t index, uint8_t is_toggle){
        // //set touch led pins
        if(timer3_running_flag){
            set_harware(index, is_toggle);
        }else{
            if(nuos_check_state_touch_leds()){
                for(int i=0; i<TOTAL_ENDPOINTS; i++){
                    nuos_zb_set_leds_only(i, false);
                }
            }else{
                set_harware(index, is_toggle);         
            }
        }
       
        nuos_store_data_to_nvs(index);
    }

    void nuos_on_off_led(uint8_t index, uint8_t _state){
        gpio_set_level(gpio_touch_led_pins[index], _state);
    }

    void nuos_toggle_leds(uint8_t index){
        state = !state;       
        nuos_on_off_led(index, state);
    }
    
    void nuos_set_state_touch_leds(bool state){
        for(int i=0; i<TOTAL_LEDS; i++){
            gpio_set_level(gpio_touch_led_pins[i], state);
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


    void nuos_init_hardware_dimming_up_down(uint32_t io_num){

    }
 #endif