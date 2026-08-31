
#include "app_hardware_driver.h"
#include "app_zigbee_clusters.h"
#include "app_nuos_timer.h"
#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_DALI_DIRECT_SWITCH)

    #include "esp_log.h"
    #include "esp_err.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include <esp_check.h>
    #include <esp_log.h>
    #include "driver/ledc.h"
    #include "DaliCommands.h"
    #include "DALI.h"
    #include "esp_random.h"
    #include "esp_wifi.h"  // For esp_wifi_stop() and esp_wifi_start()
    #include "esp_wifi_station.h"
    // instantiate global object (adjust constructor args as needed)
    DaliCommands                                               dali(gpio_load_pins[1], gpio_load_pins[0]);
    bool is_init_done                                          = false;
    static bool state                                          = false;
    #define IS_USE_DALI_HARDWARE

    extern volatile uint16_t total_press_in_secs;
    extern "C" void nuos_set_state_touch_leds(bool state);
    extern "C" bool nuos_check_state_touch_leds();
    extern "C" void process_dali_receive_tasks(uint8_t index, bool _state_, uint8_t _level_, uint16_t color);
   // void start_dali_addressing(uint8_t startAddresses, uint8_t numAddresses);

    #define LEDC_TIMER              		                    LEDC_TIMER_0
    #define LEDC_MODE               		                    LEDC_LOW_SPEED_MODE

    #define LEDC_DUTY_RES          		 	                    LEDC_TIMER_8_BIT // Set duty resolution to 13 bits
    #define LEDC_DUTY               		                    100      // Set duty to 50%. ((2 ** 8) - 1) * 50% = 4095
    #define LEDC_MAX_DUTY           		                    254     //8191

    #define LEDC_FREQUENCY                                      (1000) // Frequency in Hz (5 kHz)
    #define LEDC_FADE_TIME                                      (500) // Fade time in milliseconds (1 second)


    #ifdef USE_COLOR_CONTROL
    ledc_channel_t pwm_channels[TOTAL_LEDS]                     = { LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    #else
    ledc_channel_t pwm_channels[TOTAL_LEDS]                = { LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    #endif

    // Add at top with other defines
    #define DALI_TASK_PRIORITY                                  6           // Higher than webserver tasks
    #define DALI_ADDRESSING_TIMEOUT_MS                          30000       // 30 second timeout
    static TaskHandle_t dali_comm_task_handle                   = NULL;
    bool _toggle_                                               = false;
    static uint8_t group_addr                                   = 0xff;

    static QueueHandle_t rxFrameQueue                           = nullptr;  // Queue for received frames (each is uint32_t)
    static bool send_command_flag                               = false;
    static bool _state_[4]                                      = {false, false, false, false};
    static uint8_t button_pressed_index                         = 0xff;
#ifdef __cplusplus
extern "C" {
#endif

extern void nuos_store_dali_scene_switch_data_to_nvs(const void *str_data);

#ifdef __cplusplus
}
#endif
    #define MAX_CCT_SCENES_VALUES    6

    uint16_t cct_values[MAX_CCT_SCENES_VALUES] = {0, MIN_CCT_VALUE, MIN_CCT_VALUE_1, MIN_CCT_VALUE_2, MIN_CCT_VALUE_3, MAX_CCT_VALUE};

    #define MAX_DIMMING_VALUES      6

    uint16_t dim_values[MAX_DIMMING_VALUES] = {0, MIN_DIM_LEVEL_VALUE, 60, 120, 190, MAX_DIM_LEVEL_VALUE};

    
    static uint16_t color_backup = 0;
    static uint8_t counts_color = 0;

    void set_color_temp_only_leds_2(uint8_t index);
    void set_color_temp_leds(uint8_t index);
    void set_dimming_control_leds(uint8_t bt_index);

    typedef scene_switch_s nvs_item_t;
    // static QueueHandle_t nvs_save_queue = NULL;
    
    uint8_t map_1_255_to_100_255(uint8_t in)
    {
        return (uint8_t)((in * dali_range_size) / 254 + dali_min_off_offset);
    }

    void set_fade_time_all_devices(uint8_t time, uint8_t rate){
        for(uint8_t index=0; index<TOTAL_ENDPOINTS; index++){
            for(int i=0; i<scene_group_switch_info.total_ids[index]; i++){
                //printf("Set fade time %d rate %d for %d\n", time, rate, scene_group_switch_info.device_ids[index][i]);
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
                //printf("Set fade time %d for %d\n", time, scene_group_switch_info.device_ids[index][i]);
                dali.set_fade_time(scene_group_switch_info.device_ids[index][i], time);
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
        }
    }
    
    void nuos_set_dali_fade_rate(uint8_t rate){
        for(uint8_t index=0; index<TOTAL_ENDPOINTS; index++){
            for(int i=0; i<scene_group_switch_info.total_ids[index]; i++){
                //printf("Set fade rate %d for %d\n", rate, scene_group_switch_info.device_ids[index][i]);
                dali.set_fade_rate(scene_group_switch_info.device_ids[index][i], rate);
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
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


    #define MAX_DEVICES 64
    #define MAX_SCENES 16
    #define MAX_GROUPS 16

    uint8_t last_dtr0 = 0;
    uint8_t last_color_dtr0 = 0;
    uint8_t last_color_dtr1 = 0;
    uint8_t scene_table[MAX_DEVICES][MAX_SCENES];
    
void interpret_frame(uint8_t b1, uint8_t b2)
{
    uint8_t addr        = 0xFF;
    uint8_t scene       = 0;
    uint8_t group       = 0xFF;
    bool is_broadcast   = false;
    bool is_group       = false;
    bool is_short       = false;
    bool is_command     = false;

    // Persistent parser context for device-type-specific commands
    static uint8_t enabled_device_type = 0xFF;   // 0x06 = DT6, 0x08 = DT8
    static uint8_t dtr0 = 0;
    static uint8_t dtr1 = 0;
    static uint8_t dtr2 = 0;

    // -------------------------------------------------
    // Decode addressing / frame type first
    // -------------------------------------------------
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

    // -------------------------------------------------
    // Special commands / parser state
    // -------------------------------------------------

    // ENABLE DEVICE TYPE
    if (b1 == 0xC1) {
        if (b2 == 0x06) {
            enabled_device_type = 0x06;
            // printf("ENABLE DEVICE TYPE 6\n");
        } else if (b2 == 0x08) {
            enabled_device_type = 0x08;
            // printf("ENABLE DEVICE TYPE 8\n");
        } else {
            enabled_device_type = 0xFF;
        }
        return;
    }

    // SET DTR0
    if (b1 == 0xA3) {
        dtr0 = b2;
        last_dtr0 = b2;
        // printf("SET_DTR0 = %u\n", dtr0);
        return;
    }

    // SET DTR1
    if (b1 == 0xC3) {
        dtr1 = b2;
        last_color_dtr0 = dtr0;
        last_color_dtr1 = dtr1;
        // printf("SET_DTR1 = %u\n", dtr1);
        return;
    }

    // Optional future use
    if (b1 == 0xC5) {
        dtr2 = b2;
        return;
    }

    // -------------------------------------------------
    // Scene recall - broadcast
    // -------------------------------------------------
    if (b1 == 0xFF && b2 >= 0x10 && b2 <= 0x1F) {
        scene = b2 - 0x10;
        printf("RECALL SCENE BROADCAST %u\n", scene);

        bool all_off[2] = {true, true};
        uint8_t max_level[2] = {MIN_DIM_LEVEL_VALUE, MIN_DIM_LEVEL_VALUE};
        uint16_t max_cct[2] = {MIN_CCT_VALUE, MIN_CCT_VALUE};

        for (int b = 0; b < 2; b++) {
            for (int j = 0; j < scene_group_switch_info.total_ids[b]; j++) {
                if (scene_group_switch_info.device_scene[scene][j] == scene) {
                    if (scene_group_switch_info.device_state[b][scene][j]) {
                        all_off[b] = false;
                    }

                    if (scene_group_switch_info.device_level[b][scene][j] > max_level[b]) {
                        max_level[b] = scene_group_switch_info.device_level[b][scene][j];
                    }

                    #ifdef USE_COLOR_CONTROL
                    if (scene_group_switch_info.device_color[b][scene][j] > max_cct[b]) {
                        max_cct[b] = scene_group_switch_info.device_color[b][scene][j];
                    }
                    #endif
                }
            }

            if (all_off[b]) {
                device_info[b].device_state = false;
                process_dali_receive_tasks(b, false, 0, max_cct[b]);
            } else {
                device_info[b].device_state = true;
                device_info[b].device_level = max_level[b];
                #ifdef USE_COLOR_CONTROL
                device_info[b].device_val = max_cct[b];
                #endif
                process_dali_receive_tasks(b, true, max_level[b], max_cct[b]);
            }
        }
        return;
    }

    // -------------------------------------------------
    // Store scene - short address only
    // -------------------------------------------------
    if (b2 >= 0x40 && b2 <= 0x4F) {
        if (!is_short) {
            return;
        }

        scene = b2 - 0x40;
        scene_table[addr][scene] = dtr0;

        ////printf("STORE SCENE %u -> Device %u Level=%u\n", scene, addr, dtr0);

        for (int b = 0; b < 2; b++) {
            for (int i = 0; i < scene_group_switch_info.total_ids[b]; i++) {
                if (addr == scene_group_switch_info.device_ids[b][i]) {
                    scene_group_switch_info.device_scene[scene][i] = scene;
                    scene_group_switch_info.device_level[b][scene][i] = dtr0;
                    #ifdef USE_COLOR_CONTROL
                    {
                        uint16_t color_temp_mirek = ((uint16_t)dtr1 << 8) | dtr0;
                        if (color_temp_mirek != 0) {
                            uint16_t kelvin_cct = 1000000UL / color_temp_mirek;
                            scene_group_switch_info.device_color[b][scene][i] = kelvin_cct;
                        }
                    }
                    #endif

                    scene_group_switch_info.device_state[b][scene][i] = (dtr0 != 0);

                    printf("=======DATA EP:%d SAVED SUCCESSFULLY=======\n", b + 1);

                    //switch_driver_gpios_intr_enabled(false);
                    //dali.dali_rx_intr_enabled(false);
                    nuos_store_dali_scene_switch_data_to_nvs(&scene_group_switch_info);
                    //dali.dali_rx_intr_enabled(true);
                    //switch_driver_gpios_intr_enabled(true);
                    break;
                }
            }
        }
        return;
    }

    // -------------------------------------------------
    // DT8 commands - check before generic DAPC fallback
    // -------------------------------------------------
#ifdef USE_COLOR_CONTROL
    if (enabled_device_type == 0x08) {
        // Your existing DT8 path
        if (b2 == 0xE7) {
            uint16_t dt8_value = ((uint16_t)dtr1 << 8) | dtr0;

            if (is_short) {
                // printf("DT8 Tc command to short addr %u, value=%u\n", addr, dt8_value);
            } else if (is_group) {
                // printf("DT8 Tc command to group %u, value=%u\n", group, dt8_value);
            } else if (is_broadcast) {
                // printf("DT8 Tc command broadcast, value=%u\n", dt8_value);
            }

            return;
        }

        // Add more DT8 opcodes here:
        // ACTIVATE, STEP COOLER, STEP WARMER, COOLEST, WARMEST, etc.
    }
#endif

    // -------------------------------------------------
    // Standard command frames (non-DAPC)
    // -------------------------------------------------
    if (is_command) {
        if (is_broadcast) {
            printf("BROADCAST COMMAND: %02X\n", b2);
            is_broadcast = false; // Clear broadcast flag to prevent misinterpretation in DAPC fallback
        } else if (is_group) {
            printf("GROUP COMMAND: Group=%u Cmd=%02X\n", group, b2);
            for(int i=0; i<MAX_TOUCH_BTNS; i++){
                if(scene_group_switch_info.group_id[i] == group){
                    printf("==> MATCHED GROUP ID %u at index %d\n", group, i);

                    if(b2 == 0){
                        device_info[i].device_state = false;
                        process_dali_receive_tasks(i, false, device_info[i].device_level, device_info[i].device_val);
                    } else {
                        device_info[i].device_state = true;
                        //device_info[i].device_level = b2;
                        process_dali_receive_tasks(i, true, device_info[i].device_level, device_info[i].device_val);
                    }
                    break;
                }
            }
            is_group  = false; // Clear group flag to prevent misinterpretation in DAPC fallback
        } else if (is_short) {
            printf("SHORT COMMAND: Addr=%u Cmd=%02X\n", addr, b2);
            is_short = false; // Clear short flag to prevent misinterpretation in DAPC fallback
        }
        return;
    }

    // -------------------------------------------------
    // ARC POWER CONTROL (DAPC) fallback
    // -------------------------------------------------
    if (b2 <= 0xFE) {
        if (is_broadcast && b1 == 0xFE) {
            if (b2 == 0) {
                printf("BROADCAST -> OFF\n");
            } else {
                printf("BROADCAST -> LEVEL %u\n", b2);
            }
            return;
        }

        if (is_short) {
            printf("DEVICE %u -> LEVEL %u\n", addr, b2);
            return;
        }
    }

    // -------------------------------------------------
    // Unknown
    // -------------------------------------------------
    printf("UNKNOWN FRAME: %02X %02X\n", b1, b2);
}
    // static void receiveDaliFrame(void *arg) {
    //     DaliMessage msg;
    //     while (1) {
    //         if (rxFrameQueue != nullptr) {
    //             if(xQueueReceive(rxFrameQueue, &msg, portMAX_DELAY)== pdTRUE) {  
    //                 interpret_frame(msg.data[0], msg.data[1]);
    //             }
    //         }else{
    //             vTaskDelay(10 / portTICK_PERIOD_MS);
    //         }
    //     }
    // }
static void receiveDaliFrame(void *arg) {
    DaliMessage msg;
    while (1) {
        if (rxFrameQueue != nullptr) {
            if(xQueueReceive(rxFrameQueue, &msg, portMAX_DELAY) == pdTRUE) {  
                // If send_command_flag was set by local TX, ignore our own loopback
                // if (send_command_flag) {
                //     send_command_flag = false;
                //     continue; 
                // }
                interpret_frame(msg.data[0], msg.data[1]);
            }
        } else {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
}
    extern "C" void nuos_zb_init_hardware(){
        for(int index=0; index<TOTAL_LEDS; index++){
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
            //if(index < TOTAL_LEDS){
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
            //}
        } 
        
        dali.begin(&isr_service_installed); 
        //init_dali_hw();
        is_init_done = true; 
    }



    extern "C" void dali_query_send(uint8_t id, uint8_t command){
        //printf("===Sending DALi Query...\n");
        dali.query(id, command); // Send your query command
    }
   
    void init_dali_hw() {

        if(wifi_webserver_active_flag == 0){
        #ifdef IS_USE_DALI_HARDWARE
            rxFrameQueue = xQueueCreate(50, sizeof(DaliMessage));
            if (rxFrameQueue == nullptr) {  
            }   
                 
            dali.begin_rx(&isr_service_installed, rxFrameQueue);  
        #endif
            if(wifi_webserver_active_flag == 0){
                xTaskCreate(receiveDaliFrame, "dali_task_2", TASK_STACK_SIZE_DALI_RX_FRAME, NULL, TASK_PRIORITY_DALI_RX_FRAME, NULL); 
            }
        }         
        is_init_done = true;  
    }


    uint8_t nuos_dali_switch_type() {
        if(scene_group_switch_info.control_type == 0) {  //individual control
            return 1; 
        }else if(scene_group_switch_info.control_type == 1) { //group control
            return 2; 
        }else if(scene_group_switch_info.control_type == 2) { //scene control
            return 3; 
        }else if(scene_group_switch_info.control_type == 3) { //broadcast control
            return 4; 
        }
        return 0;
    }     

    static int get_delay(int total_ids, uint8_t index) {
        int delay_time = 1;
        if(total_ids <=4) delay_time = 10;
        else if(total_ids >4 && total_ids <=8) delay_time = 5;
        else if(total_ids >8 && total_ids <=10) delay_time = 2;
        else delay_time = 1;
        return delay_time;
    }

    static void esp_dali_off_task(void* args) {
        uint8_t index = *(uint8_t*)args;
        for(int i=0; i<scene_group_switch_info.total_ids[index]; i++){
            dali.turn_off(scene_group_switch_info.device_ids[index][i]);
            vTaskDelay(get_delay(scene_group_switch_info.total_ids[index], index) / portTICK_PERIOD_MS);
        }
        vTaskDelete(NULL); // Delete the task after executing
    }

    static void esp_dali_on_task(void* args) {
        uint8_t index = *(uint8_t*)args;
        for(int i=0; i<scene_group_switch_info.total_ids[index]; i++){
            dali.set_dim_value(scene_group_switch_info.device_ids[index][i],
            map_1_255_to_100_255(device_info[index].device_level));
            vTaskDelay(get_delay(scene_group_switch_info.total_ids[index], index) / portTICK_PERIOD_MS);
            
        }
        vTaskDelete(NULL); // Delete the task after executing
    }

    static void esp_dali_set_scene_task(void* args) {
        uint8_t index = *(uint8_t*)args;
        for(int i=0; i<scene_group_switch_info.total_ids[index]; i++){
            dali.go_to_scene(scene_group_switch_info.device_ids[index][i], scene_group_switch_info.scene_ids[index]);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }  
        vTaskDelete(NULL); // Delete the task after executing
    }



    #ifdef USE_COLOR_CONTROL
        void set_parameter_ep_index_selected(uint8_t bt_index){
            if(bt_index < 2){
                //printf("Selected EP index set to %d\n", bt_index);
                ep_selected_index = bt_index; 
            }
        }
        uint8_t get_parameter_ep_index_selected(){
            printf("Selected EP index retrieved: %d\n", ep_selected_index);
            return ep_selected_index;
        }
        void set_parameter_toggle_bt_func_selected(uint8_t enable){
            device_info[ep_selected_index].color_or_fan_state = enable; //1-color, 0-level
        }
    #endif

uint16_t calculate_cct(uint16_t current_cct, uint8_t level)
{
    const uint16_t max_cct = 6500;

    return current_cct +
           (((uint32_t)(max_cct - current_cct) * level) / 254);
}


uint32_t get_backoff_time(){
    uint32_t backoff = 1;
    if(dali_fade_time == 0){
        backoff = 800;
    }else if(dali_fade_time == 1){
        backoff = 800 + (esp_random() % 1200);
    }else if(dali_fade_time == 2){
        backoff = 1200 + (esp_random() % 2000);
    }else if(dali_fade_time == 3){
        backoff = 2000 + (esp_random() % 3000);
    }else if(dali_fade_time == 4){
        backoff = 3000 + (esp_random() % 4000);
    }
    return backoff;
}

    // extern "C" void process_dali_tasks(uint8_t index, uint8_t is_toggle, uint8_t is_scene){
    //     static bool last_state[2] = {false, false};
    //     static uint8_t last_level[2] = {0, 0};
    //     send_command_flag = true;
    //     button_pressed_index = index;
    //     #ifdef USE_COLOR_CONTROL
    //     set_parameter_ep_index_selected(index);
    //     #endif
    //     if(index < 2){
    //         _state_[index] = (bool)device_info[index].device_state;

    //         if(is_toggle>0){
    //             device_info[index].device_state = !_state_[index];
    //         } 
    //         if(!device_info[index].device_state) {
    //             last_state[index] = false;
    //             #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
    //                 ledc_set_duty(LEDC_MODE, pwm_channels[index], 0);            
    //                 ledc_update_duty(LEDC_MODE, pwm_channels[index]);
    //             #else
    //                 gpio_set_level(gpio_touch_led_pins[index], 0);
    //             #endif
    //             dali.set_group_level(scene_group_switch_info.group_id[index], 0);
    //             //dali.set_group_off(scene_group_switch_info.group_id[index]);
    //         } else {
    //             #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
    //                 ledc_set_duty(LEDC_MODE, pwm_channels[index], device_info[index].device_level);            
    //                 ledc_update_duty(LEDC_MODE, pwm_channels[index]);
    //             #else
    //                 gpio_set_level(gpio_touch_led_pins[index], 1);
    //             #endif
    //             if(is_scene){
    //                 if(device_info[index].device_state != last_state[index]){
    //                     last_state[index] = device_info[index].device_state;
    
    //                     nuos_dali_set_group_brightness(scene_group_switch_info.group_id[index], index, device_info[index].device_level);
    //                     // vTaskDelay(get_backoff_time() / portTICK_PERIOD_MS);  
    //                     // set_dali_color_temp(index, false);
    //                 }else{  
    //                     if(last_level[index] != device_info[index].device_level){
    //                         last_level[index] = device_info[index].device_level;
    //                         nuos_dali_set_group_brightness(scene_group_switch_info.group_id[index], index, device_info[index].device_level);    
    //                     }
    //                     // vTaskDelay(get_backoff_time() / portTICK_PERIOD_MS);
    //                     // set_dali_color_temp(index, false);
    //                 }
    //             }else{

    //                 if(device_info[index].device_state != last_state[index]){
    //                     last_state[index] = device_info[index].device_state;
    //                 }
                    
    //                 if(last_level[index] != device_info[index].device_level){
    //                     last_level[index] = device_info[index].device_level;
                        
    //                 }
    //                 nuos_dali_normal_set_group_brightness(scene_group_switch_info.group_id[index], index, device_info[index].device_level);
    //                 // vTaskDelay(get_backoff_time() / portTICK_PERIOD_MS);
    //             }

    //         }                    

    //         #ifdef USE_COLOR_CONTROL
    //         set_color_temp_only_leds_2(index);
    //         #endif
    //     } else{ 
    //         if(device_info[ep_selected_index].color_or_fan_state){
    //             set_color_temp_leds(button_pressed_index);
    //         }else{
    //             set_dimming_control_leds(button_pressed_index);
    //         }  
    //     } 

    //     nuos_store_data_to_nvs(ep_selected_index);

    // }
extern "C" void process_dali_tasks(uint8_t index, uint8_t is_toggle, uint8_t is_scene){
    static bool last_state[2] = {false, false};
    static uint8_t last_level[2] = {0, 0};
    send_command_flag = true;
    button_pressed_index = index;

    #ifdef USE_COLOR_CONTROL
    set_parameter_ep_index_selected(index);
    #endif

    if(index < 2){
        _state_[index] = (bool)device_info[index].device_state;

        if(is_toggle > 0){
            device_info[index].device_state = !_state_[index];
        } 
        if(!device_info[index].device_state) {
            last_state[index] = false;
            #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
                ledc_set_duty(LEDC_MODE, pwm_channels[index], 0);            
                ledc_update_duty(LEDC_MODE, pwm_channels[index]);
            #else
                gpio_set_level(gpio_touch_led_pins[index], 0);
            #endif
            dali.set_group_level(scene_group_switch_info.group_id[index], 0);
        } else {
            #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
                ledc_set_duty(LEDC_MODE, pwm_channels[index], device_info[index].device_level);            
                ledc_update_duty(LEDC_MODE, pwm_channels[index]);
            #else
                gpio_set_level(gpio_touch_led_pins[index], 1);
            #endif
            if(is_scene){
                if(device_info[index].device_state != last_state[index]){
                    last_state[index] = device_info[index].device_state;
                    nuos_dali_set_group_brightness(scene_group_switch_info.group_id[index], index, device_info[index].device_level);
                } else {  
                    if(last_level[index] != device_info[index].device_level){
                        last_level[index] = device_info[index].device_level;
                        nuos_dali_set_group_brightness(scene_group_switch_info.group_id[index], index, device_info[index].device_level);    
                    }
                }
            } else {
                if(device_info[index].device_state != last_state[index]){
                    last_state[index] = device_info[index].device_state;
                }
                if(last_level[index] != device_info[index].device_level){
                    last_level[index] = device_info[index].device_level;
                }
                nuos_dali_normal_set_group_brightness(scene_group_switch_info.group_id[index], index, device_info[index].device_level);
            }
        }                    

        #ifdef USE_COLOR_CONTROL
        set_color_temp_only_leds_2(index);
        #endif
    } else { 
        if(device_info[ep_selected_index].color_or_fan_state){
            set_color_temp_leds(button_pressed_index);
        } else {
            set_dimming_control_leds(button_pressed_index);
        }  
    } 

   // esp_zb_lock_acquire(portMAX_DELAY);
    nuos_store_data_to_nvs(ep_selected_index);
   // esp_zb_lock_release();
}
    // extern "C" void process_dali_receive_tasks(uint8_t index, bool _state_, uint8_t _level_, uint16_t color_cct){
    //     send_command_flag = true;
    //     button_pressed_index = index;
    //     if(!_state_) {
    //         nuso_set_state_attribute_on_dali_rx(index, false);
    //         #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
    //             ledc_set_duty(LEDC_MODE, pwm_channels[index], 0);            
    //             ledc_update_duty(LEDC_MODE, pwm_channels[index]);
    //             #ifdef USE_COLOR_CONTROL
    //                 gpio_set_level(gpio_touch_led_pins[index+2], 0);
    //             #endif
    //         #else
    //             gpio_set_level(gpio_touch_led_pins[index], 0);
    //         #endif
    //     } else {
    //         nuso_set_state_attribute_on_dali_rx(index, true);
    //         nuos_set_level_attribute_on_dali_rx(index, _level_);
    //         #ifdef USE_COLOR_CONTROL
    //         nuos_set_color_temp_command(index, color_cct);
    //         #endif
    //         #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
    //             ledc_set_duty(LEDC_MODE, pwm_channels[index], _level_);            
    //             ledc_update_duty(LEDC_MODE, pwm_channels[index]);
    //             #ifdef USE_COLOR_CONTROL
    //                 if(!device_info[0].device_state && device_info[1].device_state){
    //                     set_parameter_ep_index_selected(1);
    //                     set_color_temp_only_leds_2(1);
    //                 }else if(device_info[0].device_state && !device_info[1].device_state){
    //                     set_parameter_ep_index_selected(0);
    //                     set_color_temp_only_leds_2(0);
    //                 }else if(device_info[0].device_state && device_info[1].device_state){
    //                     set_parameter_ep_index_selected(1);
    //                     set_color_temp_only_leds_2(1);
    //                 }
    //             #endif
    //         #else
    //             gpio_set_level(gpio_touch_led_pins[index], 1);
    //         #endif
    //     }                    
    //     nuos_store_data_to_nvs(index);   
    // }
extern "C" void process_dali_receive_tasks(uint8_t index, bool _state_, uint8_t _level_, uint16_t color_cct){
    send_command_flag = true;
    button_pressed_index = index;

    if (!_state_) {
        nuso_set_state_attribute_on_dali_rx(index, false);
        #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
            ledc_set_duty(LEDC_MODE, pwm_channels[index], 0);            
            ledc_update_duty(LEDC_MODE, pwm_channels[index]);
            #ifdef USE_COLOR_CONTROL
                gpio_set_level(gpio_touch_led_pins[index+2], 0);
            #endif
        #else
            gpio_set_level(gpio_touch_led_pins[index], 0);
        #endif
    } else {
        nuso_set_state_attribute_on_dali_rx(index, true);
        nuos_set_level_attribute_on_dali_rx(index, _level_);
        #ifdef USE_COLOR_CONTROL
        if ((esp_zb_bdb_dev_joined() && !wifi_webserver_active_flag)) {
            esp_zb_lock_acquire(portMAX_DELAY);
            nuos_set_color_temp_command(index, color_cct);
            esp_zb_lock_release();
        }
        #endif
        #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
            ledc_set_duty(LEDC_MODE, pwm_channels[index], _level_);            
            ledc_update_duty(LEDC_MODE, pwm_channels[index]);
            #ifdef USE_COLOR_CONTROL
                if (!device_info[0].device_state && device_info[1].device_state) {
                    set_parameter_ep_index_selected(1);
                    set_color_temp_only_leds_2(1);
                } else if (device_info[0].device_state && !device_info[1].device_state) {
                    set_parameter_ep_index_selected(0);
                    set_color_temp_only_leds_2(0);
                } else if (device_info[0].device_state && device_info[1].device_state) {
                    set_parameter_ep_index_selected(1);
                    set_color_temp_only_leds_2(1);
                }
            #endif
        #else
            gpio_set_level(gpio_touch_led_pins[index], 1);
        #endif
    }                    
    nuos_store_data_to_nvs(index);   
}
    void set_all_leds_to_original_state(){
        if(is_init_done){
            for(int i=0; i<TOTAL_LEDS; i++){
                #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
                    if(scene_group_switch_info.control_type == 0 || scene_group_switch_info.control_type == 1) {  //individual control
                        if(device_info[i].device_state) ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], device_info[i].device_level));
                        else ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], 0));
                        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[i]));
                    } else {
                        if(device_info[i].device_state) ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], MAX_DIM_LEVEL_VALUE));
                        else ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], 0));
                        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[i]));
                    }  
                #else
                    if(scene_group_switch_info.control_type == 0 || scene_group_switch_info.control_type == 1) { 
                        if(device_info[i].device_state) gpio_set_level(gpio_touch_led_pins[i], 1);
                        else gpio_set_level(gpio_touch_led_pins[i], 0);
                    }
                #endif
            }
        }
    }

    void nuos_zb_set_hardware(uint8_t index, uint8_t is_toggle) {
        if(is_init_done){  
            call_common_check_auto_off();
            process_dali_tasks(index, is_toggle, false); 
        }
    }

    void nuos_on_off_led(uint8_t index, uint8_t _state) {
        if(index < TOTAL_LEDS){
            #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
                if(!_state) ledc_set_duty(LEDC_MODE, pwm_channels[index], 0);
                else ledc_set_duty(LEDC_MODE, pwm_channels[index], MAX_DIM_LEVEL_VALUE);            
                ledc_update_duty(LEDC_MODE, pwm_channels[index]);
            #else
                gpio_set_level(gpio_touch_led_pins[index], _state);
            #endif
        }
    }

    void nuos_toggle_leds(uint8_t index){
        state = !state;       
        nuos_on_off_led(index, state);
    }
    void nuos_zb_set_hardware_led_for_zb_commissioning(uint8_t is_toggle) {
        // #ifdef USE_RGB_LED
        //     nuos_toggle_rgb_led();
        // #else
        //     if(TOTAL_LEDS >= TOTAL_LEDS_SHOW_ON_COMMISSIONING) {
        //         if(is_toggle>0) {
        //             state = !state;
        //             for(int index=TOTAL_LEDS-TOTAL_LEDS_SHOW_ON_COMMISSIONING; index<TOTAL_LEDS; index++) {
        //                 gpio_set_level(gpio_touch_led_pins[index], state);
        //             }
        //         }else{
        //             // for(int i=0; i<TOTAL_ENDPOINTS; i++) {
        //             //     nuos_zb_set_hardware(i, false);
        //             // }
        //         }
        //     }
        // #endif
    }


    uint8_t nuos_get_button_press_index(uint32_t pin) {
        for(int index=0; index<TOTAL_BUTTONS; index++){
            if(pin == gpio_touch_btn_pins[index]){
                return index;
            }
        }
        return 0;
    }

    #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
    void nuos_init_hardware_dimming_up_down(uint32_t pin){
        uint8_t index = nuos_get_button_press_index(pin);
        if(index == 0 || index == 1){
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
            if(device_info[ep_selected_index].color_or_fan_state){ 
                if(device_info[ep_selected_index].device_state){
                    if(device_info[ep_selected_index].device_val <= MIN_CCT_VALUE){
                        device_info[ep_selected_index].level_up = 1;
                    }else if(device_info[ep_selected_index].device_val >= MAX_CCT_VALUE){
                        device_info[ep_selected_index].level_up = 0;
                    }
                }   
            }else{
                if(device_info[ep_selected_index].device_state){
                    if(device_info[ep_selected_index].device_level <= MIN_DIM_LEVEL_VALUE+20){
                        device_info[ep_selected_index].dim_up = 1;
                    }else if(device_info[ep_selected_index].device_level >= MAX_DIM_LEVEL_VALUE-20){
                        device_info[ep_selected_index].dim_up = 0;
                    }
                }                
            }   
        }
        #endif
    }

    ////////////////////////////////////////////////////////////////////////////////////////////
    void set_color_temp_leds(uint8_t bt_index){
        uint8_t sindex = ep_selected_index;
        if(device_info[sindex].device_state){
            if(device_info[sindex].color_or_fan_state){
                if(bt_index == 2) {
                    if(device_info[sindex].fan_speed < MAX_CCT_SCENES_VALUES-2){
                        gpio_set_level(gpio_touch_led_pins[3], 1);
                        gpio_set_level(gpio_touch_led_pins[2], 1);
                        device_info[sindex].fan_speed++;
                        // printf("Increment color temp, speed:%d\n", device_info[sindex].fan_speed);
                    } else {
                        device_info[sindex].fan_speed = MAX_CCT_SCENES_VALUES-1;
                        gpio_set_level(gpio_touch_led_pins[3], 1);
                        gpio_set_level(gpio_touch_led_pins[2], 0);
                        // printf("Maximum color temp reached, speed:%d\n", device_info[sindex].fan_speed);
                    } 
                    device_info[sindex].device_val = cct_values[device_info[sindex].fan_speed];
                    ledc_set_duty(LEDC_MODE, pwm_channels[sindex], device_info[sindex].device_val);            
                    ledc_update_duty(LEDC_MODE, pwm_channels[sindex]);
                }else if(bt_index == 3) {
                    if(device_info[sindex].fan_speed > 2){
                        gpio_set_level(gpio_touch_led_pins[3], 1);
                        gpio_set_level(gpio_touch_led_pins[2], 1);
                        device_info[sindex].fan_speed--;
                        printf("Decrement color temp, speed:%d\n", device_info[sindex].fan_speed);
                    } else {
                        device_info[sindex].fan_speed = 1;
                        gpio_set_level(gpio_touch_led_pins[3], 0);
                        gpio_set_level(gpio_touch_led_pins[2], 1);
                        // printf("Minimum color temp reached, speed:%d\n", device_info[sindex].fan_speed);
                    }
                    device_info[sindex].device_val = cct_values[device_info[sindex].fan_speed];
                    ledc_set_duty(LEDC_MODE, pwm_channels[sindex], device_info[sindex].device_val);            
                    ledc_update_duty(LEDC_MODE, pwm_channels[sindex]); 
                }else{
                    if(device_info[sindex].fan_speed == 1){
                        gpio_set_level(gpio_touch_led_pins[3], 0);
                        gpio_set_level(gpio_touch_led_pins[2], 1);
                        // printf("Color temp at minimum, speed:%d\n", device_info[sindex].fan_speed);
                    }else if(device_info[sindex].fan_speed == MAX_CCT_SCENES_VALUES-1){
                        gpio_set_level(gpio_touch_led_pins[3], 1);
                        gpio_set_level(gpio_touch_led_pins[2], 0);
                        // printf("Color temp at maximum, speed:%d\n", device_info[sindex].fan_speed);
                    }else{
                        gpio_set_level(gpio_touch_led_pins[3], 1);
                        gpio_set_level(gpio_touch_led_pins[2], 1);
                        // printf("Color temp at middle range, speed:%d\n", device_info[sindex].fan_speed);
                    }
                    ledc_set_duty(LEDC_MODE, pwm_channels[sindex], device_info[sindex].device_val);            
                    ledc_update_duty(LEDC_MODE, pwm_channels[sindex]);
                }
                nuos_store_data_to_nvs(sindex);
            }
        } else{
            gpio_set_level(gpio_touch_led_pins[3], 0);
            gpio_set_level(gpio_touch_led_pins[2], 0);
        } 
    }

    void set_dimming_control_leds(uint8_t bt_index){
        uint8_t sindex = ep_selected_index;
        if(device_info[sindex].device_state){
            if(!device_info[sindex].color_or_fan_state){
                if(bt_index == 2) {
                    if(device_info[sindex].ac_temperature < MAX_DIMMING_VALUES-2){
                        gpio_set_level(gpio_touch_led_pins[3], 1);
                        gpio_set_level(gpio_touch_led_pins[2], 1);
                        device_info[sindex].ac_temperature++;
                    } else {
                        device_info[sindex].ac_temperature = MAX_DIMMING_VALUES-1;
                        gpio_set_level(gpio_touch_led_pins[3], 1);
                        gpio_set_level(gpio_touch_led_pins[2], 0);
                    } 
                    device_info[sindex].device_level = dim_values[device_info[sindex].ac_temperature];

                    printf("Increment level:%d\n", device_info[sindex].device_level);
                    ledc_set_duty(LEDC_MODE, pwm_channels[sindex], device_info[sindex].device_level);            
                    ledc_update_duty(LEDC_MODE, pwm_channels[sindex]);
                }else if(bt_index == 3) {
                    if(device_info[sindex].ac_temperature > 2){
                        gpio_set_level(gpio_touch_led_pins[3], 1);
                        gpio_set_level(gpio_touch_led_pins[2], 1);
                        device_info[sindex].ac_temperature--;
                    } else {
                        device_info[sindex].ac_temperature = 1;
                        gpio_set_level(gpio_touch_led_pins[3], 0);
                        gpio_set_level(gpio_touch_led_pins[2], 1);
                    }
                    device_info[sindex].device_level = dim_values[device_info[sindex].ac_temperature];
                    printf("Decrement level:%d\n", device_info[sindex].device_level);
                    ledc_set_duty(LEDC_MODE, pwm_channels[sindex], device_info[sindex].device_level);            
                    ledc_update_duty(LEDC_MODE, pwm_channels[sindex]); 

                }else{
                    //printf("ac_temperature:%d\n", device_info[sindex].ac_temperature);
                    if(device_info[sindex].ac_temperature == 1){
                        gpio_set_level(gpio_touch_led_pins[3], 0);
                        gpio_set_level(gpio_touch_led_pins[2], 1);
                    }else if(device_info[sindex].ac_temperature == MAX_DIMMING_VALUES-1){
                        gpio_set_level(gpio_touch_led_pins[3], 1);
                        gpio_set_level(gpio_touch_led_pins[2], 0);
                    }else{
                        gpio_set_level(gpio_touch_led_pins[3], 1);
                        gpio_set_level(gpio_touch_led_pins[2], 1);
                    }
                    //printf("No Change level:%d\n", device_info[sindex].device_level);
                    ledc_set_duty(LEDC_MODE, pwm_channels[sindex], device_info[sindex].device_level);            
                    ledc_update_duty(LEDC_MODE, pwm_channels[sindex]);
                }
                nuos_store_data_to_nvs(sindex);
            }
        } else{
            gpio_set_level(gpio_touch_led_pins[3], 0);
            gpio_set_level(gpio_touch_led_pins[2], 0);
        }
    }
////////////////////////////////////////////////////////////////////////////////////////////


    void set_color_temp_only_leds_2(uint8_t index){

        if(device_info[index].device_state){
            if(device_info[index].color_or_fan_state){
                if(device_info[index].fan_speed > 1 && device_info[index].fan_speed < MAX_CCT_SCENES_VALUES-1){
                    gpio_set_level(gpio_touch_led_pins[3], 1);
                    gpio_set_level(gpio_touch_led_pins[2], 1);
                }else{
                    if(device_info[index].fan_speed >= MAX_CCT_SCENES_VALUES-1) {
                        gpio_set_level(gpio_touch_led_pins[3], 1);
                        gpio_set_level(gpio_touch_led_pins[2], 0);
                    } else {
                        gpio_set_level(gpio_touch_led_pins[2], 1);
                        gpio_set_level(gpio_touch_led_pins[3], 0);
                    }
                }
            } else {
                if(device_info[index].ac_temperature > 1 && device_info[index].ac_temperature < MAX_DIMMING_VALUES-2){
                    gpio_set_level(gpio_touch_led_pins[3], 1);
                    gpio_set_level(gpio_touch_led_pins[2], 1);
                }else{
                    if(device_info[index].ac_temperature >= MAX_DIMMING_VALUES-1) {
                        gpio_set_level(gpio_touch_led_pins[3], 1);
                        gpio_set_level(gpio_touch_led_pins[2], 0);
                    } else {
                        gpio_set_level(gpio_touch_led_pins[2], 1);
                        gpio_set_level(gpio_touch_led_pins[3], 0);
                    }
                }                            
            }   
        }else{
            if(!device_info[0].device_state && !device_info[1].device_state){
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], 0));
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[index]));

                gpio_set_level(gpio_touch_led_pins[3], 0);
                gpio_set_level(gpio_touch_led_pins[2], 0);   
            }else if(!device_info[0].device_state){
                if(device_info[1].device_state)
                set_color_temp_only_leds_2(1);
            }else if(!device_info[1].device_state){
                if(device_info[0].device_state)
                set_color_temp_only_leds_2(0);
            }
        }      
    }


    int find_closest_index(uint16_t value) {
        for (int i = 0; i < MAX_CCT_SCENES_VALUES-1; i++) {
            
            if (value <= cct_values[i]) {
                printf("closest index 0:%d\n", i);
                return i; // Return previous index as the closest lower bound
            }
        }
        //printf("closest index 3:%d\n", MAX_CCT_SCENES_VALUES-1);
        return MAX_CCT_SCENES_VALUES - 1; // If greater than all, return last index
    }

    int find_closest_index_2(uint16_t value) {
        for (int i = 1; i < MAX_DIMMING_VALUES; i++) {
            if (value < dim_values[i]) {
                printf("closest index:%d\n", i-1);
                return i - 1; // Return previous index as the closest lower bound
            }
        }
        return MAX_DIMMING_VALUES - 1; // If greater than all, return last index
    }

    void set_color_to_updown_leds(uint8_t index){
        if(device_info[index].device_state){
            if(device_info[index].color_or_fan_state){
                device_info[index].fan_speed = find_closest_index(device_info[index].device_val);
            }else{
                set_dimming_control_leds(index);
                device_info[index].ac_temperature = find_closest_index_2(device_info[index].device_level);
            } 
            set_color_temp_only_leds_2(index);
        }
    }
    void convert_colors_to_index(uint8_t index, bool is_long_press){
        if(device_info[index].level_up){
            if(device_info[index].device_val + COLOR_STEPS <= (MAX_CCT_VALUE)){
                device_info[index].fan_speed = find_closest_index(device_info[index].device_val);
                if(is_long_press) device_info[index].device_val += COLOR_STEPS;
            } else {
                device_info[index].device_val = MAX_CCT_VALUE;
                device_info[index].fan_speed = MAX_CCT_SCENES_VALUES-1;               
            }
            set_color_temp_only_leds_2(index);
        }else{
            if(device_info[index].device_val - COLOR_STEPS >= MIN_CCT_VALUE){
                device_info[index].fan_speed = find_closest_index(device_info[index].device_val); 
                if(is_long_press) device_info[index].device_val -= COLOR_STEPS;                        
            }else {
                device_info[index].device_val = MIN_CCT_VALUE;
                device_info[index].fan_speed = 1;
            }
            set_color_temp_only_leds_2(index);        
        }
        
    }



   void convert_dimming_to_index(uint8_t index, bool is_long_press){
        if(device_info[index].dim_up){
            if(device_info[index].device_level + DIMMING_STEPS <= (MAX_DIM_LEVEL_VALUE)){
                device_info[index].ac_temperature = find_closest_index_2(device_info[index].device_level);
                if(is_long_press) device_info[index].device_level += DIMMING_STEPS;
            } else {
                if(is_long_press) device_info[index].device_level = MAX_DIM_LEVEL_VALUE;
                device_info[index].ac_temperature = MAX_DIMMING_VALUES-1;               
            }
            set_color_temp_only_leds_2(index);
        }else{
            if(device_info[index].device_level - DIMMING_STEPS >= MIN_DIM_LEVEL_VALUE){
                device_info[index].ac_temperature = find_closest_index_2(device_info[index].device_level); 
                if(is_long_press) device_info[index].device_level -= DIMMING_STEPS;                        
            }else {
                if(is_long_press) device_info[index].device_level = MIN_DIM_LEVEL_VALUE;
                device_info[index].ac_temperature = 1;
            }
            set_color_temp_only_leds_2(index);        
        }
        
    }

    extern "C" void set_dali_color_temp(uint8_t index, bool is_status_change){
        nuos_dali_set_group_color_temperature(scene_group_switch_info.group_id[index], index, device_info[index].device_val);
    }

    extern "C" void set_dali_level(uint8_t index){
        nuos_dali_set_group_brightness(scene_group_switch_info.group_id[index], index, device_info[index].device_level);
    }
    extern "C" void set_dali_level_to_off(uint8_t index){
        nuos_dali_set_group_brightness(scene_group_switch_info.group_id[index], index, 0);
    }
    bool nuos_set_hardware_brightness(uint32_t pin) {
        call_common_check_auto_off();
        uint8_t index = nuos_get_button_press_index(pin);
        // printf("brightness index:%d\n", index);
        if(index < 2){
            if(!device_info[index].device_state){
                device_info[index].device_state = true;
                device_info[index].device_level = MIN_DIM_LEVEL_VALUE;
                device_info[index].dim_up = 1;
            }
            if(device_info[index].device_state){
                printf("prev_brightness: %d\n", device_info[index].device_level);
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
                printf("brightness1: %d\n", device_info[index].device_level);
                process_dali_tasks(index, false, false); 
                printf("brightness2: %d\n", device_info[index].device_level);
                    
                printf("color_or_fan_state:%d\n", device_info[index].color_or_fan_state);
                // if(device_info[index].color_or_fan_state){ 
                //     convert_colors_to_index(index, false);  
                // }else{
                    convert_dimming_to_index(index, false);
                // }
                printf("brightness3: %d\n", device_info[index].device_level);
                nuos_set_level_attribute(index);
                printf("brightness4: %d\n", device_info[index].device_level);
            }
        }
        #ifdef USE_COLOR_CONTROL
        else{
                uint8_t p_index = index-2;
                if(device_info[p_index].color_or_fan_state){ 
                    convert_colors_to_index(p_index, is_long_press_brightness);
                    if(color_backup != device_info[p_index].device_val){
                        color_backup = device_info[p_index].device_val;

                        if(scene_group_switch_info.control_type == 0) { 
                            dali.set_group_color_cct(scene_group_switch_info.group_id[p_index], device_info[p_index].device_val);
                        }else{
                            #if(COMMUNICATION_MODE == COMM_MODE_ADDR_CTRL)
                            dali.set_color_temperature(global_dali_id[p_index], device_info[p_index].device_val); 
                            #elif(COMMUNICATION_MODE == COMM_MODE_GROUP_CTRL)
                            dali.set_group_color_cct(global_group_id[p_index], device_info[p_index].device_val);
                            #elif(COMMUNICATION_MODE == COMM_MODE_BROADCAST)
                            dali.set_color_temperature(dali.BROADCAST_C, device_info[p_index].device_val); 
                            #endif 
                        }
                        if(counts_color++ >= 10){
                            counts_color = 0;
                            nuos_set_color_temperature_attribute(p_index);
                            nuos_dali_set_group_color_temperature(scene_group_switch_info.group_id[p_index], p_index, device_info[p_index].device_val);
                        }                                        
                    }  
                }
                nuos_store_data_to_nvs(0);                      
                
        }
        #endif
        return false;
    }

    #endif

    extern "C" void nuos_dali_add_light_to_group(uint8_t addr, uint8_t group_id) {
        // printf("Adding addr:%d to group:%d\n", addr, group_id);
        dali.add_to_group(addr, group_id);   
    }

    extern "C" void nuos_dali_remove_light_from_group(uint8_t addr, uint8_t group_id) {
        dali.remove_from_group(addr, group_id); 
    }
  
    extern "C" void nuos_dali_set_state(uint8_t dali_id, uint8_t state) { 
        if(state == 0) dali.turn_off(dali_id);
        else dali.set_dim_value(dali_id, MAX_DIM_LEVEL_VALUE); 
    } 
    extern "C" void nuos_dali_set_brightness(uint8_t dali_id, uint8_t level) { 
        dali.set_dim_value(dali_id,  map_1_255_to_100_255(level));
    }   
    
    extern "C" void nuos_dali_set_cct_color(uint8_t did, uint16_t value) {
        dali.set_color_temperature(did, value);
    }
    // extern "C" void nuos_dali_set_rgb_color(uint8_t did, uint8_t r, uint8_t g, uint8_t b) {
    //     dali.set_color_rgb(did, r, g, b, MAX_DIM_LEVEL_VALUE, false);
    // }
    extern "C" void nuos_dali_broadcast_state(uint8_t toggle_state) { 
        if(!toggle_state) dali.send_broadcast(0b00000000);
        else dali.set_broadcast_level(MAX_DIM_LEVEL_VALUE); 
    } 

    extern "C" void nuos_dali_broadcast_level(uint8_t level) {     
        dali.set_broadcast_level(map_1_255_to_100_255(level)); 
    } 

    extern "C" void nuos_dali_toggle_group(uint8_t group_id, uint8_t index, bool toggle_state, uint8_t brightness) {  
        if(!toggle_state) dali.set_group_off(group_id);
        else dali.set_group_level(group_id, map_1_255_to_100_255(brightness));
    } 

    extern "C" void nuos_dali_set_state_group(uint8_t index, uint8_t brightness) { 
        if(!device_info[index].device_state) dali.set_group_off(scene_group_switch_info.group_id[index]);
        else {
            uint8_t bright = map_1_255_to_100_255(brightness);
            printf("Setting group %d brightness to %d\n", scene_group_switch_info.group_id[index], bright);
            dali.set_group_level(scene_group_switch_info.group_id[index], bright);
            //dali.set_cct_dimming(group_addr, bright);
        }
    }

    extern "C" void nuos_dali_add_device_to_scene(uint8_t device_id, uint8_t scene_id, uint8_t scene_level, uint16_t cct_temp) {
        if(scene_level == 0){
            dali.set_level_scene(device_id, scene_id, 0);
        }else{
            dali.set_color_scene(device_id, scene_id, scene_level, cct_temp);
        }
    }

    extern "C" void nuos_dali_rgb_add_device_to_scene(uint8_t device_id, uint8_t scene_id, uint8_t scene_level, uint8_t r, uint8_t g, uint8_t b) {
        if(scene_level == 0){
            dali.set_level_scene(device_id, scene_id, 0);
        }else{ 
            dali.set_rgb_scene(device_id, scene_id, scene_level, r, g, b);
        }
    }

    extern "C" void nuos_dali_remove_device_from_scene(uint8_t device_id, uint8_t scene_id) {
        dali.remove_from_scene(device_id, scene_id);
    }

    extern "C" void nuos_dali_add_device_state_to_scene(uint8_t device_id, uint8_t scene_id) {
        dali.add_to_scene(device_id, scene_id);
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

    extern "C" void nuos_dali_set_group_color_temperature(uint8_t group_id, uint8_t index, uint16_t value) {
        dali.set_group_color_cct(group_id, value);
        
    } 

    extern "C" void nuos_dali_set_group_rgb_temperature(uint8_t group_id, uint8_t r, uint8_t g, uint8_t b) { 
        dali.set_group_color_rgb(group_id, r, g, b, MAX_DIM_LEVEL_VALUE, false);
    } 
    

    extern "C" void nuos_dali_set_group_brightness(uint8_t group_id, uint8_t index, uint8_t value) {
        uint8_t val = map_1_255_to_100_255(value);
        dali.set_group_level(group_id, val);
    }
    extern "C" void nuos_dali_normal_set_group_brightness(uint8_t group_id, uint8_t index, uint8_t value) {
        uint8_t val = map_1_255_to_100_255(value);
        printf("Value Level:%d\n", val);
        dali.set_group_level_normal(group_id, val);
    }
    

    extern "C" void nuos_dali_add_group_to_scene(uint8_t group_id, uint8_t scene_id, uint8_t scene_level, uint16_t cct_temp) {

        dali.set_color_scene(group_id | (1<<7), scene_id, scene_level, cct_temp);
    }
   
    extern "C" void nuos_dali_remove_group_from_scene(uint8_t group_id, uint8_t scene_id) { 
        dali.remove_from_scene(group_id | (1<<7), scene_id); 
    } 

    static void esp_dali_commissioning_led_blink_task(void *pvParameters) {
        bool _toggle = false;
        while(1) {
            vTaskDelay(pdMS_TO_TICKS(500));
            for(int i=0; i<TOTAL_LEDS; i++){
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
        printf("WiFi stopped for DALI commissioning\n");
        vTaskDelay(pdMS_TO_TICKS(100));  // Let WiFi fully stop
        #endif
        switch_driver_gpios_intr_enabled(false); 
        printf("Starting DALI addressing...\n");
        // Perform DALI addressing with increased priority
        //vTaskPrioritySet(NULL, DALI_TASK_PRIORITY);
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

    extern "C" void start_dali_addressing123(uint8_t startAddresses, uint8_t numAddresses) {            
        recordsSemaphore = xSemaphoreCreateBinary();
        if (recordsSemaphore == NULL) {
            // Handle semaphore creation failure
            printf("Failed to create semaphore!\n");
            return;
        }    
        uint16_t  addr = (numAddresses & 0xff) | ((startAddresses & 0xff) << 8);
        xTaskCreate(esp_dali_init_node_task, "dali_task", 8192, &addr, TASK_PRIORITY_DALI_TASK, NULL);
        start_dali_led_commissioning_task_flag = true;

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
            if(scene_group_switch_info.control_type == 2) { //scene control
                if(scene_group_switch_info.selected_id == i){
                    nuos_zb_set_hardware(i, false);
                    break;
                }
            }else{
                nuos_zb_set_hardware(i, false);
            }
        }
    }  //end extern "C" void start_dali_addressing(uint8_t startAddresses, uint8_t numAddresses)

    extern "C" void nuos_dali_set_color_temperate(uint8_t index) {

    } 

    
    void start_dali_led_blink_task() {
        if(start_dali_led_commissioning_task_flag){
            for(int i=0; i<TOTAL_LEDS; i++){
                if(!_toggle_){
                    ledc_set_duty(LEDC_MODE, pwm_channels[i], 0);            
                    ledc_update_duty(LEDC_MODE, pwm_channels[i]);
                }else{
                    ledc_set_duty(LEDC_MODE, pwm_channels[i], 0xff);            
                    ledc_update_duty(LEDC_MODE, pwm_channels[i]);
                }
            }
            _toggle_ = !_toggle_;
        }
    }

    extern "C" void nuos_set_state_touch_leds(bool state) {
        if(is_init_done){
            for(int i=0; i<TOTAL_LEDS; i++){
                if(state){
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], MAX_DIM_LEVEL_VALUE));
                }else{
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], 0));
                }
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[i]));
            }
        }
    }

    extern "C" void nuos_set_state_touch_leds_to_original() {
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

    extern "C" bool nuos_check_state_touch_leds() {
        bool getting_on_state = false;
        if(touchLedsOffAfter1MinuteEnable) {
            for(int i=0; i<TOTAL_LEDS; i++) {
                if(device_info[i].device_state) {
                    getting_on_state = true;
                }
            }
        }
        return getting_on_state;
    }   
#endif





