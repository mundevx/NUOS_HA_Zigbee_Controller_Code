
#include "app_hardware_driver.h"
#include "esp_zigbee_core.h"
#include "app_zigbee_misc.h"

#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2CH_CURTAIN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN_SWITCH)



#include "esp_timer.h"
#include "esp_log.h"
#include "app_zigbee_clusters.h"

    //#define OLD_CURTAIN_BOARD
    int elapsed_sec = 0;
    int elapsed_ds = 0;  // Changed from seconds to deciseconds (100ms units)
    static int8_t direction = 1;  // 1 = opening (0 → 100), -1 = closing (100 → 0)
    uint8_t curtain_state = 0;
    bool nuos_check_state_touch_leds();
    void init_curtain_timer();
    void pause_curtain_timer();
    void curtain_cmd_stop(void);
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
// Updated conversion functions for deciseconds (100ms)
// Updated conversion functions for deciseconds (100ms)
static inline uint16_t ds_to_pct_x10(uint32_t ds)
{
    if (device_info[0].device_val == 0) return 0;
    return (ds * 1000) / (device_info[0].device_val * 10);  // Returns 0-1000 range
}

static inline uint32_t pct_to_ds(uint8_t pct)
{
    if (device_info[0].device_val == 0) return 0;
    return (pct * device_info[0].device_val * 10) / 100;
}

static inline uint8_t ds_to_pct(uint32_t ds)
{
    return ds_to_pct_x10(ds) / 10;  // Convert to 0-100 range
}
int curtain_timer_counts = 0;
bool curtain_timer_start_flag = false;
static void curtain_timer_start_task(void *pvParameters){
    while(1){
        vTaskDelay(pdMS_TO_TICKS(10));
        if(curtain_timer_start_flag){
            if(curtain_timer_counts++ > 250){  //2.5 sec
                curtain_timer_counts = 0;
                esp_timer_start_periodic(periodic_timer, 100000);   // 100ms Tick
                curtain_timer_start_flag = false;
                break;
            }
        }
    }
    vTaskDelete(NULL);
}
/* New generic handler for "Go-to-percentage" */
int curtain_cmd_goto_pct(uint8_t pct)   // 0-100
{
    /* Clamp and ignore if already there */
    if (pct > 100) pct = 100;
    
    // Convert current percentage to deciseconds for accurate starting position
    uint8_t current_pct = device_info[0].light_color_x / 10;
    if (current_pct > 100) current_pct = 100;
    
    if (current_pct == pct) {
        printf("Already at target percentage %d%%\n", pct);
        return -1;
    }

    /* Determine direction */
    curtain_dir  = (pct > current_pct) ? CURTAIN_DIR_OPENING : CURTAIN_DIR_CLOSING;
    target_pct   = pct;

    /* Sync elapsed_ds with current level */
    elapsed_ds  = pct_to_ds(current_pct);
    
    printf("Starting movement: current=%d%%, target=%d%%, direction=%s, elapsed_ds=%d\n",
           current_pct, target_pct, 
           curtain_dir == CURTAIN_DIR_OPENING ? "OPENING" : "CLOSING",
           elapsed_ds);

    /* Issue proper Zigbee motor command */
    curtain_state = (curtain_dir == CURTAIN_DIR_OPENING)
                    ? ESP_ZB_ZCL_CMD_WINDOW_COVERING_UP_OPEN
                    : ESP_ZB_ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE;

    gpio_set_level(gpio_load_pins[0], 1);
    printf("----------STARt TIMER----------\n");
    #ifdef TUYA_ATTRIBUTES
        if (!timer_paused) {
            esp_timer_stop(periodic_timer);
            timer_paused = true;
            printf("Timer paused at %d.%d seconds (%d%%)\n", 
                   elapsed_ds / 10, elapsed_ds % 10, 
                   device_info[0].light_color_x / 10);
        }
    #endif
    xTaskCreate(curtain_timer_start_task, "curtain_start", 2048, NULL, TASK_PRIORITY_ATTR, NULL);
    
    curtain_timer_counts = 0;
    curtain_timer_start_flag = true;
    timer_paused = false;
    return curtain_state;
}

void curtain_cmd_open(void)
{
    curtain_cmd_goto_pct(100);
    device_info[0].ac_mode = 100;
}

void curtain_cmd_close(void)
{
    curtain_cmd_goto_pct(0);
    device_info[0].ac_mode = 0;
}

void curtain_cmd_stop(void)
{
    pause_curtain_timer();
    curtain_dir = CURTAIN_DIR_STOPPED;
}

// extern int fix_percentage(int input_val);
void periodic_timer_callback(void *arg)
{
    if (timer_paused || curtain_dir == CURTAIN_DIR_STOPPED) return;

    // Update position every 100ms
    uint32_t max_ds = device_info[0].device_val * 10;
    
    if (curtain_dir == CURTAIN_DIR_OPENING && elapsed_ds < max_ds) {
        elapsed_ds++;
    } else if (curtain_dir == CURTAIN_DIR_CLOSING && elapsed_ds > 0) {
        elapsed_ds--;
    }

    // Convert deciseconds to percentage (scaled by 10 for light_color_x)
    device_info[0].light_color_x = ds_to_pct_x10(elapsed_ds);
    uint8_t current_pct = device_info[0].light_color_x / 10;
    
    // Report state
    if(elapsed_ds%10 == 0) //report every 1 second
    nuos_report_curtain_blind_state(0, current_pct);  
    
    printf("t=%u.%u s  level=%u%% (target %u%%)\n", 
           elapsed_ds / 10, elapsed_ds % 10, 
           current_pct, target_pct);
    
    // Calculate target in deciseconds for accurate comparison
    uint32_t target_ds = pct_to_ds(target_pct);
    
    // Improved stopping conditions
    bool reached_limit = false;
    bool reached_target = false;

    if (curtain_dir == CURTAIN_DIR_OPENING) {
        reached_limit = (elapsed_ds >= max_ds);
        reached_target = (elapsed_ds >= target_ds);
    } else if (curtain_dir == CURTAIN_DIR_CLOSING) {
        reached_limit = (elapsed_ds == 0);
        reached_target = (elapsed_ds <= target_ds);
    }

    if (reached_limit || reached_target) {
        printf("Stopping condition met: limit=%d, target=%d\n", reached_limit, reached_target);
        curtain_cmd_stop();
        
        // Snap to exact target if not at hardware limit
        if (!reached_limit) {
            elapsed_ds = target_ds;
            device_info[0].light_color_x = target_pct * 10;
            uint8_t final_pct = ds_to_pct(elapsed_ds);
            device_info[0].device_level = final_pct;
            nuos_report_curtain_blind_state(0, final_pct);
            printf("Snapped to exact target: %d%%\n", final_pct);
        }
    }
}
void pause_curtain_timer()
{
    #ifdef TUYA_ATTRIBUTES
        if (!timer_paused) {
            esp_timer_stop(periodic_timer);
            timer_paused = true;
            printf("Timer paused at %d.%d seconds (%d%%)\n", 
                   elapsed_ds / 10, elapsed_ds % 10, 
                   device_info[0].light_color_x / 10);
        }
    #endif
    // Stop hardware movement
    gpio_set_level(gpio_load_pins[0], 0);
    #ifdef OLD_CURTAIN_BOARD
    gpio_set_level(gpio_load_pins[1], 0);
    #endif
    nuos_store_data_to_nvs(0);
}

// static inline uint8_t sec_to_pct(uint32_t sec)
// {
//     return (sec * 100) / device_info[0].device_val;
// }
// static inline uint32_t pct_to_sec(uint8_t pct)
// {
//     return (pct * device_info[0].device_val) / 100;
// }


// /* New generic handler for “Go-to-percentage” */
// uint8_t curtain_cmd_goto_pct(uint8_t pct)   // 0-100
// {
//     /* Clamp and ignore if already there */
//     if (pct > 100) pct = 100;
//     if (device_info[0].device_level == pct) return device_info[0].fan_speed;

//     if(device_info[0].device_level > 100) device_info[0].device_level = 100;
//     /* Determine direction */
//     curtain_dir  = (pct > device_info[0].device_level)
//                    ? CURTAIN_DIR_OPENING
//                    : CURTAIN_DIR_CLOSING;
//     target_pct   = pct;

//     /* Sync elapsed_sec with current level */
//     elapsed_sec  = pct_to_sec(device_info[0].device_level);

//     /* Issue proper Zigbee motor command */
//     curtain_state = (curtain_dir == CURTAIN_DIR_OPENING)
//                     ? ESP_ZB_ZCL_CMD_WINDOW_COVERING_UP_OPEN
//                     : ESP_ZB_ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE;

//     esp_timer_start_periodic(periodic_timer, 1e6);   // 1s Tick
//     timer_paused = false;
//     return curtain_state;
// }
// void curtain_cmd_open(void)
// {
//     curtain_cmd_goto_pct(100);
//     device_info[0].ac_mode = 100;
// }

// void curtain_cmd_close(void)
// {
//     curtain_cmd_goto_pct(0);
//     device_info[0].ac_mode= 0;
// }

// void curtain_cmd_stop(void)
// {
//     pause_curtain_timer();
//     curtain_dir = CURTAIN_DIR_STOPPED;
// }
// // extern int fix_percentage(int input_val);
// void periodic_timer_callback(void *arg)
// {
//     if (timer_paused || curtain_dir == CURTAIN_DIR_STOPPED) return;

//     if (curtain_dir == CURTAIN_DIR_OPENING && elapsed_sec < device_info[0].device_val)
//         elapsed_sec++;
//     else if (curtain_dir == CURTAIN_DIR_CLOSING && elapsed_sec > 0)
//         elapsed_sec--;

//     device_info[0].light_color_x = sec_to_pct(elapsed_sec);
//     // device_info[0].device_level = (uint8_t)fix_percentage(sec_to_pct(elapsed_sec));
//     nuos_report_curtain_blind_state(0, device_info[0].light_color_x/10);  
//     printf("t=%u s  level=%u (target %u %%)\n", elapsed_sec, device_info[0].light_color_x, target_pct);
//     // --- FIX: Stop when passing or matching target ---
//     bool reached_limit  = (device_info[0].light_color_x == 0 || device_info[0].light_color_x == 1000);
//     bool reached_target = false;

//     //

//     if (curtain_dir == CURTAIN_DIR_OPENING)
//         reached_target = (device_info[0].light_color_x >= target_pct);
//     else if (curtain_dir == CURTAIN_DIR_CLOSING)
//         reached_target = (device_info[0].light_color_x <= target_pct);

//     if (reached_limit || reached_target) {
//         curtain_cmd_stop();
//         printf("curtain_cmd_stop   reached_limit:%d  reached_target:%d\n", reached_limit, reached_target);
//         // Force to exact target if not at limit
//         if (!reached_limit) {
//             device_info[0].device_level = target_pct/10;
//             elapsed_sec = pct_to_sec(target_pct);
//             nuos_report_curtain_blind_state(0, device_info[0].device_level);          
//         }
//         printf("Auto-stop at %s\n", reached_limit ? "limit" : "requested percentage");
//     }
// }

    void init_curtain_timer(){
        const esp_timer_create_args_t periodic_timer_args = {
            .callback = &periodic_timer_callback,
            .name = "periodic"
        };
        esp_timer_create(&periodic_timer_args, &periodic_timer);
    }

    // // Call this function from any event (button, command, etc.) to pause
    // void pause_curtain_timer()
    // {
    //     #ifdef TUYA_ATTRIBUTES
    //         if (!timer_paused) {
    //             esp_timer_stop(periodic_timer);
    //             timer_paused = true;
    //             printf("Timer paused at %d seconds (%d%%)\n", elapsed_sec, device_info[0].device_level);
    //         }
    //     #endif
    //     //set load pins
    //     gpio_set_level(gpio_load_pins[0], 0);
    //     //set load pins
    //     #ifdef OLD_CURTAIN_BOARD
    //     gpio_set_level(gpio_load_pins[1], 0);
    //     #endif
    //     nuos_store_data_to_nvs(0);
    // }

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
            printf("LOAD_CURTAIN_OPEN\n");
        }else if(cur_mode == CURTAIN_CLOSE){
            gpio_set_level(gpio_load_pins[0], 1);
            //set load pins
            gpio_set_level(gpio_load_pins[1], 0);  //close
            printf("LOAD_CURTAIN_CLOSE\n");
        }else{
            gpio_set_level(gpio_load_pins[0], 0);  //stop
            printf("LOAD_CURTAIN_STOP\n");
        }
    }

    void set_curtain_percentage(uint8_t value){

        nuos_report_curtain_blind_state(0, value);    
        int state = curtain_cmd_goto_pct( value);
        if(state != -1){
            device_info[0].device_state = 0;
            device_info[1].device_state = 0;
            nuos_zb_set_hardware_curtain(0, state);
        }
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
            gpio_set_level(gpio_touch_led_pins[0], 1);
            gpio_set_level(gpio_touch_led_pins[1], 0);
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
            


            nuos_store_data_to_nvs(0);
            nuos_store_data_to_nvs(1);
        }else if(index == 1){ 
            device_info[0].fan_speed = 1;
            if(device_info[0].device_state){
                device_info[0].device_state = false; 
                device_info[1].device_state = false;
            }
            gpio_set_level(gpio_touch_led_pins[0], 0);
            gpio_set_level(gpio_touch_led_pins[1], 1);            
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
                        gpio_set_level(gpio_touch_led_pins[1], 0);
                        //set load pins
                        set_curtain_load(CURTAIN_OPEN);
                    #endif
                }else if(device_state == CURTAIN_CLOSE){
                    device_info[0].device_state = true;
                    device_info[1].device_state = false;
                    
                    gpio_set_level(gpio_touch_led_pins[0], 0);
                    gpio_set_level(gpio_touch_led_pins[1], 1);
                    //set load pins
                    set_curtain_load(CURTAIN_CLOSE);
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