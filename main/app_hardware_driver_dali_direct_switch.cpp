
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
    void start_dali_addressing(uint8_t startAddresses, uint8_t numAddresses);

    #define LEDC_TIMER              		                    LEDC_TIMER_0
    #define LEDC_MODE               		                    LEDC_LOW_SPEED_MODE

    #define LEDC_DUTY_RES          		 	                    LEDC_TIMER_8_BIT // Set duty resolution to 13 bits
    #define LEDC_DUTY               		                    100      // Set duty to 50%. ((2 ** 8) - 1) * 50% = 4095
    #define LEDC_MAX_DUTY           		                    254     //8191
    //#define LEDC_MIN_DUTY           		                    (LEDC_MAX_DUTY*10)/100     //10%

    // #define LEDC_FREQUENCY          	                        1000    // Frequency in Hertz. Set frequency at 1 KHz


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
    static bool _state_                                         = false;
    static uint8_t button_pressed_index                         = 0xff;

    uint8_t map_1_255_to_100_255(uint8_t in)
    {
        return (uint8_t)((in * dali_range_size) / 254 + dali_min_off_offset);
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
        uint8_t addr;
        uint8_t scene;

        // ---------------------------
        // SET_DTR0
        // ---------------------------
        if(b1 == 0xC3)
        {
            last_color_dtr0 = last_dtr0;
            last_color_dtr1 = b2;

            return;
        }

        // ---------------------------
        // ENABLE DEVICE TYPE
        // ---------------------------
        if(b1 == 0xC1 && b2 == 0x08)
        {
            //printf("ENABLE DEVICE TYPE 8\n");
            return;
        }

        // ---------------------------
        // SET COLOR TEMPERATURE (DT8)
        // ---------------------------
        if(b2 == 0xE7)
        {
            addr = (b1 >> 1) & 0x3F;
            //printf("SET COLOR TEMP → Device %d\n", addr);
            return;
        }

        // ---------------------------
        // STORE SCENE
        // ---------------------------
        if(b2 >= 0x40 && b2 <= 0x4F)
        {
            addr = (b1 >> 1) & 0x3F;
            scene = b2 - 0x40;

            scene_table[addr][scene] = last_dtr0;

            printf("STORE SCENE %d → Device %d  Level=%d\n",
                scene, addr, last_dtr0);
   
            for(int b=0; b<2; b++){
                //printf("---------START Loop%d  total_ids:%d--------\n", b+1, scene_group_switch_info.total_ids[b]);
                for(int i=0; i<scene_group_switch_info.total_ids[b]; i++){
                    if(addr == scene_group_switch_info.device_ids[b][i]){

                        scene_group_switch_info.device_scene[scene][i] = scene;  
                        printf("==========================last_dtr0 level[%d]:%d\n", b, last_dtr0);
                        scene_group_switch_info.device_level[b][scene][i] = last_dtr0;
                        #ifdef USE_COLOR_CONTROL  
                        uint16_t color_temp_mirek = ((last_color_dtr1 << 8) & 0xff00) | last_color_dtr0;
                        uint16_t kelvin_cct = 1000000 / color_temp_mirek;
                        //2000 to 6500
                        scene_group_switch_info.device_color[b][scene][i] = kelvin_cct;  
                        #endif
                        if(last_dtr0 == 0) scene_group_switch_info.device_state[b][scene][i]  = false;
                        else scene_group_switch_info.device_state[b][scene][i]  = true;
                        printf("=======DATA EP:%d SAVED SUCCESSFULLY=======\n", b+1);
                        nuos_store_dali_scene_switch_data_to_nvs(&scene_group_switch_info); 
                        break;
                    }
                }   
            }           
            return;
        }

        // ---------------------------
        // BROADCAST SCENE RECALL
        // ---------------------------
        if(b1 == 0xFF && b2 >= 0x10 && b2 <= 0x1F)
        {
            scene = b2 - 0x10;
            printf("RECALL SCENE BROADCAST %d\n", scene);
            //scene_group_switch_info.scene_ids[0] = scene;

            bool all_off[2] = {true, true};
            uint8_t max_level[2] = {MIN_DIM_LEVEL_VALUE, MIN_DIM_LEVEL_VALUE};
            uint16_t max_cct[2] = {MIN_CCT_VALUE, MIN_CCT_VALUE}; //default 2000K

            for(int b=0; b<2; b++){
                for(int j=0; j<scene_group_switch_info.total_ids[b]; j++){
                    printf("====>SCENE:%d\n", scene_group_switch_info.device_scene[scene][j]);
                    if(scene_group_switch_info.device_scene[scene][j] == scene){
                        if(scene_group_switch_info.device_state[b][scene][j]){
                            all_off[b] = false;
                        }
                        if(scene_group_switch_info.device_level[b][scene][j] > max_level[b]){
                            max_level[b] = scene_group_switch_info.device_level[b][scene][j];
                            
                        }
                        //printf("LEVEL:%d\n", max_level[b]);
                        #ifdef USE_COLOR_CONTROL
                        if(scene_group_switch_info.device_color[b][scene][j] > max_cct[b]){
                            max_cct[b] = scene_group_switch_info.device_color[b][scene][j];
                        }
                        #endif
                    }
                }  
                if(all_off[b]){
                    device_info[b].device_state = false;
                    process_dali_receive_tasks(b, false, 0, max_cct[b]);
                }else{
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
        // ---------------------------
        // COLOR ACTIVATE
        // ---------------------------
        if(b1 == 0xA3)
        {
            //printf("COLOR ACTIVATE\n");
            last_dtr0 = b2;
            printf("SET_DTR0 = %d\n", last_dtr0);
            return;
        }
        // ---------------------------
        // ARC POWER CONTROL (DAPC)
        // ---------------------------
        if(b2 <= 0xFE)
        {
            // Broadcast
            if(b1 == 0xFE)
            {
                if(b2 == 0){
                    printf("BROADCAST → OFF\n");
                    device_info[0].device_state = false;
                    //nuos_zb_set_hardware(0, false);
                }else{
                    printf("BROADCAST → LEVEL %d\n", b2);
                    device_info[0].device_state = true;
                    device_info[0].device_level = b2;
                    //nuos_zb_set_hardware(0, false);                    
                }
                return;
            }
            // Group command
            if(b1 & 0x80)
            {
                uint8_t group = (b1 >> 1) & 0x0F;
                // if(b2 == 0)
                //     printf("GROUP %d → OFF\n", group);
                // else
                //     printf("GROUP %d → LEVEL %d\n", group, b2);

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

    extern "C" void nuos_zb_init_hardware(){
        // Prepare and then apply the LEDC PWM timer configuration
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
        
        is_init_done = true; 
    }



    extern "C" void dali_query_send(uint8_t id, uint8_t command){
        printf("===Sending DALi Query...\n");
        dali.query(id, command); // Send your query command
    }
   
    void init_dali_hw() {

        if(wifi_webserver_active_flag == 0){
        #ifdef IS_USE_DALI_HARDWARE
            rxFrameQueue = xQueueCreate(10, sizeof(DaliMessage));
            if (rxFrameQueue == nullptr) {
                
            }   
                 
            dali.begin_rx(&isr_service_installed, rxFrameQueue);  
        #endif
            if(wifi_webserver_active_flag == 0){
                xTaskCreate(receiveDaliFrame, "dali_task_2", 4096, NULL, 23, NULL); 
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
            printf("%d off\n", scene_group_switch_info.device_ids[index][i]);
            vTaskDelay(get_delay(scene_group_switch_info.total_ids[index], index) / portTICK_PERIOD_MS);
        }
        vTaskDelete(NULL); // Delete the task after executing
    }

    static void esp_dali_on_task(void* args) {
        uint8_t index = *(uint8_t*)args;
        for(int i=0; i<scene_group_switch_info.total_ids[index]; i++){
            printf("%d on at %d\n", scene_group_switch_info.device_ids[index][i], device_info[index].device_level);
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


    extern "C" void process_dali_tasks(uint8_t index, uint8_t is_toggle){
        send_command_flag = true;
        button_pressed_index = index;
        printf("Processing DALI Tasks for index %d\n", index);
        _state_ = (bool)device_info[index].device_state;
        if(scene_group_switch_info.control_type == 0) {  //individual control
            if(is_toggle>0) device_info[index].device_state = !_state_;
            if(!device_info[index].device_state) {
                #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
                    ledc_set_duty(LEDC_MODE, pwm_channels[index], 0);            
                    ledc_update_duty(LEDC_MODE, pwm_channels[index]);

                    #ifdef USE_COLOR_CONTROL
                        ledc_set_duty(LEDC_MODE, pwm_channels[index+2], 0);            
                        ledc_update_duty(LEDC_MODE, pwm_channels[index+2]);
                    #endif
                #else
                    gpio_set_level(gpio_touch_led_pins[index], 0);
                #endif
                printf("group_id:%d\n", scene_group_switch_info.group_id[index]);
                dali.set_group_off(scene_group_switch_info.group_id[index]);
            } else {

                printf(device_info[index].device_level > 0 ? "Device level: %d\n" : "Device level is 0, treating as ON command\n", device_info[index].device_level);
                #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
                    ledc_set_duty(LEDC_MODE, pwm_channels[index], device_info[index].device_level);            
                    ledc_update_duty(LEDC_MODE, pwm_channels[index]);

                    #ifdef USE_COLOR_CONTROL
                        uint8_t color_val = map_1000(device_info[index].device_val, MIN_CCT_VALUE, MAX_CCT_VALUE, MAX_DIM_LEVEL_VALUE, MIN_DIM_LEVEL_VALUE);
                        printf("color val:%d\n", color_val);
                        ledc_set_duty(LEDC_MODE, pwm_channels[index+2], color_val);            
                        ledc_update_duty(LEDC_MODE, pwm_channels[index+2]);
                    #endif
                #else
                    gpio_set_level(gpio_touch_led_pins[index], 1);
                #endif
                printf("group_id:%d\n", scene_group_switch_info.group_id[index]);
                nuos_dali_set_group_brightness(scene_group_switch_info.group_id[index], index, device_info[index].device_level);
            }                    
        } else if(scene_group_switch_info.control_type == 1) { //broadcast control
                          
        }
        nuos_store_data_to_nvs(index);   
    }

    extern "C" void process_dali_receive_tasks(uint8_t index, bool _state_, uint8_t _level_, uint16_t color_cct){
        send_command_flag = true;
        button_pressed_index = index;
        printf("index:%d  state:%d\n", index, _state_);
        if(!_state_) {
            nuso_set_state_attribute_on_dali_rx(index, false);
            #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
                ledc_set_duty(LEDC_MODE, pwm_channels[index], 0);            
                ledc_update_duty(LEDC_MODE, pwm_channels[index]);

                #ifdef USE_COLOR_CONTROL
                    ledc_set_duty(LEDC_MODE, pwm_channels[index+2], 0);            
                    ledc_update_duty(LEDC_MODE, pwm_channels[index+2]);
                #endif
            #else
                gpio_set_level(gpio_touch_led_pins[index], 0);
            #endif

        } else {
            nuso_set_state_attribute_on_dali_rx(index, true);
            nuos_set_level_attribute_on_dali_rx(index, _level_);
            #ifdef USE_COLOR_CONTROL
            nuos_set_color_temp_command(index, color_cct);
            #endif
            #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
                ledc_set_duty(LEDC_MODE, pwm_channels[index], _level_);            
                ledc_update_duty(LEDC_MODE, pwm_channels[index]);

                #ifdef USE_COLOR_CONTROL
                    uint8_t color_val = map_1000(color_cct, MIN_CCT_VALUE, MAX_CCT_VALUE, MAX_DIM_LEVEL_VALUE, MIN_DIM_LEVEL_VALUE);
                    ledc_set_duty(LEDC_MODE, pwm_channels[index+2], color_val);            
                    ledc_update_duty(LEDC_MODE, pwm_channels[index+2]);
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
            process_dali_tasks(index, is_toggle);  
             
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

    void nuos_init_hardware_dimming_up_down(uint32_t pin) {
        uint8_t index = nuos_get_button_press_index(pin);
        //printf("hello index:%d\n", index);
        
        if(index < 2) {
            if(device_info[index].device_level <= MIN_DIM_LEVEL_VALUE+DIMMING_LAST_REACH_OFFSET){
                device_info[index].dim_up = 1;
            }else if(device_info[index].device_level >= MAX_DIM_LEVEL_VALUE-DIMMING_LAST_REACH_OFFSET){
                device_info[index].dim_up = 0;
            }
        }
        #ifdef USE_COLOR_CONTROL
        else{
            if(device_info[index-2].device_val <= MIN_CCT_VALUE+COLOR_LAST_REACH_OFFSET){
                device_info[index-2].level_up = 1;
                //printf("level up\n");
            }else if(device_info[index-2].device_val >= MAX_CCT_VALUE-COLOR_LAST_REACH_OFFSET){
                device_info[index-2].level_up = 0;
                //printf("level down\n");

            }                
        }
        #endif
        
    }

    extern "C" void set_color_temp(uint8_t index, bool is_status_change){
        //ledc_set_duty(LEDC_MODE, pwm_channels[index], map_1000(device_info[index-2].device_val, MIN_CCT_VALUE, MAX_CCT_VALUE, MIN_DIM_LEVEL_VALUE, MAX_DIM_LEVEL_VALUE));            
        //ledc_update_duty(LEDC_MODE, pwm_channels[index]);
        //nuos_set_color_temperature_attribute(index-2);
        nuos_dali_set_group_color_temperature(scene_group_switch_info.group_id[index], index, device_info[index].device_val);
    }
    bool nuos_set_hardware_brightness(uint32_t pin) {
        call_common_check_auto_off();
        uint8_t index = nuos_get_button_press_index(pin);
        printf("brightness index:%d\n", index);
        if(global_switch_state == SWITCH_PRESS_DETECTED){ 
            
            printf("global_switch_state:%d\n", global_switch_state);
            if(index < 2){
                if(!device_info[index].device_state){
                    device_info[index].device_state = true;
                    device_info[index].device_level = MIN_DIM_LEVEL_VALUE;
                    device_info[index].dim_up = 1;
                }
                if(device_info[index].device_state){

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
                    printf("brightness: %d\n", device_info[index].device_level);
                    ledc_set_duty(LEDC_MODE, pwm_channels[index], device_info[index].device_level);            
                    ledc_update_duty(LEDC_MODE, pwm_channels[index]);

                    nuos_set_level_attribute(index);

                    
                    process_dali_tasks(index, false);  
                }
            }
            #ifdef USE_COLOR_CONTROL
            else{
                
                if(!device_info[index-2].device_state){
                    device_info[index-2].device_state = true;
                    device_info[index-2].device_val = MIN_CCT_VALUE;
                    device_info[index-2].level_up = 1;
                }
     
                if(device_info[index-2].device_state){
                    if(device_info[index-2].level_up == 1){
                        if(device_info[index-2].device_val + COLOR_STEPS <= (MAX_CCT_VALUE)){
                            device_info[index-2].device_val += COLOR_STEPS;
                        } else {
                            device_info[index-2].device_val = MAX_CCT_VALUE;
                        }
                    }else{
                        if(device_info[index-2].device_val - COLOR_STEPS >= MIN_CCT_VALUE){
                            device_info[index-2].device_val -= COLOR_STEPS;  
                        }else {
                            device_info[index-2].device_val = MIN_CCT_VALUE;
                        } 
                    }
                    printf("CCT: %d\n", device_info[index-2].device_val);
                    ledc_set_duty(LEDC_MODE, pwm_channels[index+2], map_1000(device_info[index-2].device_val, MIN_CCT_VALUE, MAX_CCT_VALUE, MAX_DIM_LEVEL_VALUE, MIN_DIM_LEVEL_VALUE));            
                    ledc_update_duty(LEDC_MODE, pwm_channels[index+2]);
                    nuos_set_color_temperature_attribute(index-2);
                    nuos_dali_set_group_color_temperature(scene_group_switch_info.group_id[index-2], index-2, device_info[index-2].device_val);
                    
                } 
            }
            #endif
            
        }
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
    extern "C" void nuos_dali_set_rgb_color(uint8_t did, uint8_t r, uint8_t g, uint8_t b) {
        dali.set_color_rgb(did, r, g, b, MAX_DIM_LEVEL_VALUE);
    }
    extern "C" void nuos_dali_broadcast_state(uint8_t toggle_state) { 
        if(!toggle_state) dali.send_broadcast(0b00000000);
        else dali.set_broadcast_level(MAX_DIM_LEVEL_VALUE); 
    } 

    extern "C" void nuos_dali_broadcast_level(uint8_t level) {     
        dali.set_broadcast_level(map_1_255_to_100_255(level)); 
    } 

    extern "C" void nuos_dali_toggle_group(uint8_t group_id, uint8_t index, bool toggle_state, uint8_t brightness) {  
        if(!toggle_state) dali.set_group_off(group_id);
        else dali.set_group_on(group_id);
    } 

    extern "C" void nuos_dali_set_state_group(uint8_t group_id, bool _state) { 
        if(!_state) dali.set_group_off(group_id);
        else dali.set_group_on(group_id);
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
        dali.set_group_color_rgb(group_id, r, g, b, MAX_DIM_LEVEL_VALUE);
    } 
    

    extern "C" void nuos_dali_set_group_brightness(uint8_t group_id, uint8_t index, uint8_t value) {
        dali.set_group_level(group_id, map_1_255_to_100_255(value));
    }

    extern "C" void nuos_dali_add_group_to_scene(uint8_t group_id, uint8_t scene_id, uint8_t scene_level, uint16_t cct_temp) {

        dali.set_color_scene(group_id | (1<<7), scene_id, scene_level, cct_temp);
    }
   
    extern "C" void nuos_dali_remove_group_from_scene(uint8_t group_id, uint8_t scene_id) { 
        dali.remove_from_scene(group_id | (1<<7), scene_id); 
    } 



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

    extern "C" void start_dali_addressing(uint8_t startAddresses, uint8_t numAddresses) {            
        recordsSemaphore = xSemaphoreCreateBinary();
        if (recordsSemaphore == NULL) {
            // Handle semaphore creation failure
            printf("Failed to create semaphore!\n");
            return;
        }    
        uint16_t  addr = (numAddresses & 0xff) | ((startAddresses & 0xff) << 8);
        xTaskCreate(esp_dali_init_node_task, "dali_task", 8192, &addr, 8, NULL);
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





