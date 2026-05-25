
#include "app_hardware_driver.h"
#include "app_zigbee_clusters.h"

#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_CUSTOM)
#include "esp_zigbee_core.h"
#include "zdo/esp_zigbee_zdo_command.h"
 
#include <esp_timer.h>
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_check.h>
#include <esp_log.h>
#include "driver/ledc.h"
#include "esp_rom_sys.h"
#include "esp_wifi.h"  // For esp_wifi_stop() and esp_wifi_start()
#include "esp_wifi_station.h"

    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI)
    #include "DaliCommands.h"
        DaliCommands 										        dali(gpio_load_pins[1], gpio_load_pins[0]);

        extern "C" void nuos_dali_set_broadcast_color_rgb(uint8_t index, uint8_t r, uint8_t g, uint8_t b ,uint16_t value);
    #else
        #include <Arduino.h>
        //#include "esp_dmx.h"
        #include "dmx.h"
        // uint8_t red = 1;
        // uint8_t green = 1;
        // uint8_t blue = 1;
    #endif

    #define LEDC_TIMER              		                    LEDC_TIMER_0
    #define LEDC_MODE               		                    LEDC_LOW_SPEED_MODE

    #define LEDC_DUTY_RES          		 	                    LEDC_TIMER_8_BIT // Set duty resolution to 13 bits
    #define LEDC_DUTY               		                    100      // Set duty to 50%. ((2 ** 8) - 1) * 50% = 4095
    #define LEDC_MAX_DUTY           		                    MAX_DIM_LEVEL_VALUE     //8191
    #define LEDC_FREQUENCY          	                        5000    // Frequency in Hertz. Set frequency at 5 KHz

    #if (TOTAL_LEDS == 4)
        ledc_channel_t pwm_channels[TOTAL_LEDS]                     = { LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3};  
    #elif (TOTAL_LEDS == 3)
        ledc_channel_t pwm_channels[TOTAL_LEDS]                     = { LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2};
    #elif (TOTAL_LEDS == 2)
        ledc_channel_t pwm_channels[TOTAL_LEDS]                     = { LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    #elif (TOTAL_LEDS == 1)
        ledc_channel_t pwm_channels[TOTAL_LEDS]                     = { LEDC_CHANNEL_0};
    #endif

    bool is_init_done                                           = false;
    static bool state                                           = false;
    uint8_t level                                               = 0;

    uint8_t dmx_data[10]                                        = {0};
    uint8_t dmx_start_address                                   = 1;


    typedef struct {
        uint8_t r; // Red
        uint8_t g; // Green
        uint8_t b; // Blue
    } ColorRGB;

    // bool on_state = true;

    ColorRGB  start, end;
    extern "C"  esp_err_t nuos_set_color_rgb_mode_attribute(uint8_t index, uint8_t val_mode);
    extern void set_hardware(uint8_t index, uint8_t is_toggle);
    static uint8_t last_dali_level = 0; 
    static uint16_t last_color_temp = 0;

    // DALI limits
#define DALI_MIN MIN_DIM_LEVEL_VALUE
#define DALI_MAX MAX_DIM_LEVEL_VALUE
#define CALIBRATION_GAMMA 0.5  // Tune as needed

    extern "C" int max_of_three(int a, int b, int c) {
        int max = a;
        if (b > max) max = b;
        if (c > max) max = c;
        return max;
    }
    

    extern "C" unsigned int map_1000(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);
    void set_color_hw(uint8_t index);

    #define MAX_CCT_SCENES_VALUES    6

    uint16_t cct_values[MAX_CCT_SCENES_VALUES] = {0, MIN_CCT_VALUE, MIN_CCT_VALUE_1, MIN_CCT_VALUE_2, MIN_CCT_VALUE_3, MAX_CCT_VALUE};

    extern "C" void nuos_zb_init_hardware(){    
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
        is_init_done = true; 
    }

    void set_dali_fading() {
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI)
        #if(COMMUNICATION_MODE == COMM_MODE_ADDR_CTRL)
        for(int i=0; i<scene_group_switch_info.total_ids[0]; i++){
            dali.set_fade_rate(scene_group_switch_info.device_ids[0][i], 12);
            vTaskDelay(20 / portTICK_PERIOD_MS);
            dali.set_fade_time(scene_group_switch_info.device_ids[0][i], 1);
            vTaskDelay(20 / portTICK_PERIOD_MS);
        }
        #elif(COMMUNICATION_MODE == COMM_MODE_GROUP_CTRL)            
            dali.add_to_group(1, global_group_id[0]);              
            vTaskDelay(20 / portTICK_PERIOD_MS);                
            dali.set_group_fade_rate(global_group_id[0], 12);
            vTaskDelay(20/ portTICK_PERIOD_MS);
            dali.set_group_fade_time(global_group_id[0], 1);     
        #elif(COMMUNICATION_MODE == COMM_MODE_BROADCAST)
            dali.set_fade_rate(dali.BROADCAST_C, 12);
            vTaskDelay(5/ portTICK_PERIOD_MS); 
            dali.set_fade_time(dali.BROADCAST_C, 1);
            vTaskDelay(5 / portTICK_PERIOD_MS);
            
        #endif
        dali.set_rgbwaf_ctrl(); //taskYIELD(); 
        #endif
    }

    bool set_color_task_flag = false;
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


    void set_all_leds_to_original_state(){
        for(int i=0; i<TOTAL_LEDS; i++){
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], device_info[i].device_state));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[i]));
        }  
    }

    extern "C" void set_state(uint8_t index){
        set_state_flag = false;
        if(is_init_done){  
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
                printf("set_state   R:%d G:%d B:%d\n", dmx_data[dmx_start_address], dmx_data[dmx_start_address+1], dmx_data[dmx_start_address+2]);
                DMX::WriteAll(&dmx_data[dmx_start_address], dmx_start_address, 5);
            #else 
            // printf("===========>scene_group_switch_info.device_ids[0]:%d\n", scene_group_switch_info.device_ids[0][0]);   
            if(selected_color_mode == 0){
                  
                #if(USE_COLOR_DEVICE == COLOR_RGBW || USE_COLOR_DEVICE == COLOR_RGB_CW_WW) 
                    #if(COMMUNICATION_MODE == COMM_MODE_ADDR_CTRL) 
                    if(scene_group_switch_info.control_type != 0) { 
                        if(!device_info[3].device_state) dali.send_broadcast(dali.OFF_C);
                        else {  
                            dali.set_broadcast_level(device_info[3].device_level);
                        }      
                    }else{
                        if(!device_info[3].device_state) dali.set_group_off(scene_group_switch_info.group_id[0]);
                        else nuos_dali_set_group_brightness(scene_group_switch_info.group_id[0], 0, device_info[3].device_level);
                    }

                    #elif(COMMUNICATION_MODE == COMM_MODE_GROUP_CTRL) 
                        if(!device_info[3].device_state) dali.set_group_off(global_group_id[0]);
                        else { 
                            dali.set_group_on(global_group_id[0]);
                        }  
                    #elif(COMMUNICATION_MODE == COMM_MODE_BROADCAST)
                        if(!device_info[3].device_state) dali.send_broadcast(dali.OFF_C);
                        else {  
                            dali.set_broadcast_level(device_info[3].device_level);
                        }                    
                    #endif
                #else
                if(!device_info[4].device_state) dali.send_broadcast(dali.OFF_C);
                else {
                    #if(COMMUNICATION_MODE == COMM_MODE_ADDR_CTRL)
                        dali.set_dim_value(scene_group_switch_info.device_ids[0][0], device_info[4].device_level);
                    #elif(COMMUNICATION_MODE == COMM_MODE_GROUP_CTRL) 
                        dali.set_group_level(global_group_id[0], device_info[4].device_level);
                    #elif(COMMUNICATION_MODE == COMM_MODE_BROADCAST)
                        dali.set_broadcast_level(device_info[4].device_level);
                    #endif                    

                }
                #endif
                nuos_set_state_attribute_rgb(3);
            }else{
                #if(COMMUNICATION_MODE == COMM_MODE_ADDR_CTRL)
                
                if(scene_group_switch_info.control_type != 0) { 
                    if(!device_info[4].device_state) dali.send_broadcast(dali.OFF_C);
                    else {
                        dali.set_broadcast_level(device_info[4].device_level);
                    }
                }else{
                    printf("group_id:%d level:%d\n", scene_group_switch_info.group_id[0], device_info[4].device_level);
                    if(!device_info[4].device_state) dali.set_group_off(scene_group_switch_info.group_id[0]);
                    else nuos_dali_set_group_brightness(scene_group_switch_info.group_id[0], 0, device_info[4].device_level);
                }
                
                #elif(COMMUNICATION_MODE == COMM_MODE_GROUP_CTRL)  
                    if(!device_info[4].device_state) dali.set_group_off(global_group_id[0]);
                    else { 
                        dali.set_group_on(global_group_id[0]);
                    }  
                #elif(COMMUNICATION_MODE == COMM_MODE_BROADCAST)
                    if(!device_info[4].device_state) dali.send_broadcast(dali.OFF_C);
                    else {
                        dali.set_broadcast_level(device_info[4].device_level);
                    }
                #endif     
                nuos_set_state_attribute_rgb(4);
            }    
            #endif
        }
    }   

    extern "C" void set_dali_level(uint8_t index){
        set_level_flag = false;
        if(is_init_done){  
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
                printf("set_level   R:%d G:%d B:%d\n", dmx_data[dmx_start_address], dmx_data[dmx_start_address+1], dmx_data[dmx_start_address+2]);
                DMX::WriteAll(&dmx_data[dmx_start_address], dmx_start_address, 5);
            #else      
                if(selected_color_mode == 0){
                    #if(COMMUNICATION_MODE == COMM_MODE_ADDR_CTRL)
                    if(scene_group_switch_info.control_type != 0) { 
                        if(!device_info[3].device_state) dali.set_broadcast_level(0);
                        else{ 
                            dali.set_broadcast_level(device_info[3].device_level);
                            vTaskDelay(5 / portTICK_PERIOD_MS);
                            dali.set_broadcast_level(device_info[3].device_level);
                        } 
                    }else{
                        if(!device_info[3].device_state) dali.set_group_off(scene_group_switch_info.group_id[0]);
                        else nuos_dali_set_group_brightness(scene_group_switch_info.group_id[0], 0, device_info[3].device_level);
                    }
                    #elif(COMMUNICATION_MODE == COMM_MODE_GROUP_CTRL) 
                    if(!device_info[3].device_state) dali.set_group_level(scene_group_switch_info.device_ids[0][0], 0);
                    else dali.set_group_level(scene_group_switch_info.device_ids[0][0], device_info[3].device_level);
                    #elif(COMMUNICATION_MODE == COMM_MODE_BROADCAST)
                        if(!device_info[3].device_state) dali.set_broadcast_level(0);
                        else{ 
                            dali.set_broadcast_level(device_info[3].device_level);
                            vTaskDelay(5 / portTICK_PERIOD_MS);
                            dali.set_broadcast_level(device_info[3].device_level);
                        } 
                    #endif

                    nuos_set_color_temp_level_attribute(3); 
                                    
                }else{ 
                    // #if(COMMUNICATION_MODE == COMM_MODE_ADDR_CTRL)
                    // if(scene_group_switch_info.control_type != 0) { 
                    //     if(!device_info[4].device_state) dali.set_broadcast_level(0);
                    //     else dali.set_broadcast_level(device_info[4].device_level);
                    // }else{
                    //     printf("group_id2:%d level2:%d\n", scene_group_switch_info.group_id[0], device_info[4].device_level);
                    //     if(!device_info[4].device_state) dali.set_group_off(scene_group_switch_info.group_id[0]);
                    //     else nuos_dali_set_group_brightness(scene_group_switch_info.group_id[0], 0, device_info[4].device_level);

                    // }
                    // #elif(COMMUNICATION_MODE == COMM_MODE_GROUP_CTRL) 
                    //     if(!device_info[3].device_state) dali.set_group_level(scene_group_switch_info.device_ids[0][0], 0);
                    //     else dali.set_group_level(scene_group_switch_info.device_ids[0][0], device_info[4].device_level);
                    // #elif(COMMUNICATION_MODE == COMM_MODE_BROADCAST)
                    //     if(!device_info[4].device_state) dali.set_broadcast_level(0);
                    //     else dali.set_broadcast_level(device_info[4].device_level);
                    // #endif

                    nuos_set_level_attribute(3);                      
                }
            #endif  
             
        }
    }

    extern "C" void set_dali_color_temp(uint8_t index, bool is_brightness_change){
        set_color_flag = false;
        if(is_init_done){  
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
                DMX::WriteAll(&dmx_data[dmx_start_address], dmx_start_address, 5);
                rgb_t rgb = {dmx_data[dmx_start_address], dmx_data[dmx_start_address+1], dmx_data[dmx_start_address+2]}; // Example RGB values
                hsv_t hsv2 = rgb_to_hsv(rgb);
                                                           
                nuos_set_color_xy_attribute(4, &hsv2);
            #else  

            if(selected_color_mode == 0){
                #if(COMMUNICATION_MODE == COMM_MODE_ADDR_CTRL)
                if(scene_group_switch_info.control_type != 0) { 
                    if(mode_change_flag){
                        printf("==============Broadcast color set to 0 for all channels\n");
                        dali.set_broadcast_color_rgb( 0, 0, 0, device_info[3].device_level);
                        mode_change_flag = false; 
                        
                    }  
                    if(device_info[3].device_state){
                        dali.set_color_temperature(dali.BROADCAST_C, device_info[3].device_val);  
                    }
                }else{
                    if(mode_change_flag){
                        printf("==============Groupcast color set to 0 for all channels state:%d\n", device_info[3].device_state);
                        dali.set_group_color_rgb(scene_group_switch_info.group_id[0], 0, 0, 0, device_info[3].device_level);
                        mode_change_flag = false;
                        change_cw_ww_color_flag = true;
                        vTaskDelay(20 / portTICK_PERIOD_MS);
                    }  
                    if(device_info[3].device_state){
                        printf("color device_val:%d level:%d\n", device_info[3].device_val, device_info[3].device_level);
                        dali.set_group_color_cct(scene_group_switch_info.group_id[0], device_info[3].device_val);
                        nuos_dali_set_group_brightness(scene_group_switch_info.group_id[0], 0, device_info[3].device_level); 
                    }
                }
                #elif(COMMUNICATION_MODE == COMM_MODE_GROUP_CTRL) 
                    if(mode_change_flag){
                        dali.set_group_color_rgb(global_group_id[0], 0, 0, 0, device_info[3].device_level);
                        mode_change_flag = false; 
                    }  
                    if(device_info[3].device_state){
                        dali.set_group_color_cct(global_group_id[0], device_info[3].device_val);
                    
                        dali.set_group_level(device_info[3].device_level);  
                    }
                #elif(COMMUNICATION_MODE == COMM_MODE_BROADCAST)
                    if(mode_change_flag){
                        dali.set_broadcast_color_rgb( 0, 0, 0, device_info[3].device_level);
                        mode_change_flag = false; 
                    }  
                    if(device_info[3].device_state){
                        dali.set_color_temperature(dali.BROADCAST_C, device_info[3].device_val);
                     
                        dali.set_broadcast_level(device_info[3].device_level);  
                    }
                #endif                
                nuos_set_color_temp_attribute(3);            
            } else {
                #if(COMMUNICATION_MODE == COMM_MODE_ADDR_CTRL)
                if(scene_group_switch_info.control_type != 0) { 
                    #if(USE_COLOR_DEVICE == COLOR_RGBW || USE_COLOR_DEVICE == COLOR_RGB_CW_WW)
                        if(mode_change_flag){
                            dali.set_off_waf_channels(dali.BROADCAST_C);
                            dali.set_color_temperature(dali.BROADCAST_C, 0); 
                            mode_change_flag = false;
                        }  
                    #endif           
                    dali.set_broadcast_color_rgb(dmx_data[dmx_start_address], dmx_data[dmx_start_address+1], dmx_data[dmx_start_address+2], device_info[4].device_level); 
                }else{
                    #if(USE_COLOR_DEVICE == COLOR_RGBW || USE_COLOR_DEVICE == COLOR_RGB_CW_WW)
                        
                        if(mode_change_flag){
                            printf("==============Groupcast color set to 0 for all channels state:%d\n", device_info[4].device_state);
                            dali.set_off_waf_channels(scene_group_switch_info.group_id[0]);
                            //dali.set_color_temperature(scene_group_switch_info.group_id[0], 0);
                            //dali.set_rgbwaf_ctrl();
                            mode_change_flag = false;
                        }  
                    #endif    
                    dali.set_group_color_rgb(scene_group_switch_info.group_id[0], 
                        dmx_data[dmx_start_address], dmx_data[dmx_start_address+1], 
                        dmx_data[dmx_start_address+2], device_info[4].device_level);
                    // dali.set_off_waf_channels(scene_group_switch_info.group_id[0]);    
                   // dali.set_color_temperature(scene_group_switch_info.group_id[0], 0);
                }

                #elif(COMMUNICATION_MODE == COMM_MODE_GROUP_CTRL) 
                #if(USE_COLOR_DEVICE == COLOR_RGBW || USE_COLOR_DEVICE == COLOR_RGB_CW_WW)
                    if(mode_change_flag){
                        dali.set_off_waf_channels(global_group_id[0]);
                        mode_change_flag = false;
                    }  
                #endif           
                dali.set_group_color_rgb(global_group_id[0], dmx_data[dmx_start_address], dmx_data[dmx_start_address+1], dmx_data[dmx_start_address+2], device_info[3].device_level); 
                #elif(COMMUNICATION_MODE == COMM_MODE_BROADCAST)
                    #if(USE_COLOR_DEVICE == COLOR_RGBW || USE_COLOR_DEVICE == COLOR_RGB_CW_WW)
                        if(mode_change_flag){
                            dali.set_off_waf_channels(dali.BROADCAST_C);
                            mode_change_flag = false;
                        }  
                    #endif           
                    dali.set_broadcast_color_rgb(dmx_data[dmx_start_address], dmx_data[dmx_start_address+1], dmx_data[dmx_start_address+2], device_info[4].device_level); 
                #endif 

              
                if(!brightness_control_flag){
                    rgb_t rgb = {dmx_data[dmx_start_address], dmx_data[dmx_start_address+1], dmx_data[dmx_start_address+2]}; // Example RGB values
                    hsv_t hsv2 = rgb_to_hsv(rgb);  
                    // if(is_brightness_change){
                    hsv2.v = device_info[4].device_val; // Store brightness level for later use in brightness control 
                    //     hsv2.s = device_info[4].light_color_y;
                    // }                               
                    nuos_set_color_xy_attribute(4, &hsv2);   
                }
            }
            #endif
        }
    } 

    static QueueHandle_t rxFrameQueue = nullptr;           // Queue for received frames (each is uint32_t)

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


uint16_t rgb_to_rgb565(uint8_t r, uint8_t g, uint8_t b) {
    uint16_t r5 = r >> 3;      // Top 5 bits of R (255->31)
    uint16_t g6 = g >> 2;      // Top 6 bits of G (255->63)
    uint16_t b5 = b >> 3;      // Top 5 bits of B (255->31)
    return (r5 << 11) | (g6 << 5) | b5;
}


void rgb565_to_rgb(uint16_t rgb565, uint8_t *r, uint8_t *g, uint8_t *b) {
    *r = (uint8_t)((rgb565 >> 11) << 3);  // 5-bit R -> 8-bit (0-31 << 3 = 0-248)
    *g = (uint8_t)((rgb565 >>  5) << 2);  // 6-bit G -> 8-bit (0-63 << 2 = 0-252)
    *b = (uint8_t)((rgb565 & 0x001F) << 3); // 5-bit B -> 8-bit (0-31 << 3 = 0-248)
}
    uint8_t dali_rx_selected_color_mode = 0; // 0 for CCT, 1 for RGB
    // void interpret_frame(uint8_t b1, uint8_t b2)
    // {
    //     static uint8_t last_dtr0 = 0;
    //     static uint8_t last_dtr1 = 0;
    //     static uint8_t last_color_dtr0 = 0;
    //     static uint8_t last_color_dtr1 = 0;
    //     static uint8_t last_color_dtr2 = 0;

    //     uint8_t addr;
    //     uint16_t scene;

    //     // ---------------------------
    //     // SET_DTR0
    //     // ---------------------------
    //             // ---------------------------
    //     // COLOR ACTIVATE
    //     // ---------------------------
    //     if(b1 == dali.SET_DTR0){
    //         // printf("COLOR ACTIVATE\n");
    //         last_dtr0 = b2;
    //         //printf("SET_DTR0 = %d\n", last_dtr0);
    //         return;
    //     }

    //     if(b1 == dali.SET_DTR1){
    //         last_color_dtr0 = last_dtr0;
    //         last_color_dtr1 = b2;
    //         //last_dtr1 = b2;
    //         dali_rx_selected_color_mode = 0;
    //         return;
    //     }
    //     if(b1 == dali.SET_DTR2){
    //         last_color_dtr0 = last_dtr0;
    //         //last_color_dtr1 = last_dtr1;  
    //         last_color_dtr2 = b2;          
    //         dali_rx_selected_color_mode = 1;
    //         return;
    //     }
    //     // ---------------------------
    //     // ENABLE DEVICE TYPE
    //     // ---------------------------
    //     if(b1 == 0xC1 && b2 == 0x08){
    //         return;
    //     }
    //     // ---------------------------
    //     // SET COLOR TEMPERATURE (DT8)
    //     // ---------------------------
    //     if(b2 == 0xE7){
    //         addr = (b1 >> 1) & 0x3F;
    //         return;
    //     }

    //     // ---------------------------
    //     // STORE SCENE
    //     // ---------------------------
    //     if(b2 >= 0x40 && b2 <= 0x4F){
    //         addr = (b1 >> 1) & 0x3F;
    //         scene = b2 - 0x40;
            
    //         for(int i=0; i<scene_group_switch_info.total_ids[0]; i++){
    //             if(addr == scene_group_switch_info.device_ids[0][i]){
    //                 scene_group_switch_info.device_scene[scene][i] = scene;  
    //                 scene_group_switch_info.device_level[scene][i] = last_dtr0;
    //                 printf("dali_rx_selected_color_mode:%d\n", dali_rx_selected_color_mode);

    //                 #ifdef ENABLE_DALI_RECEIVER
    //                     scene_group_switch_info.device_color_mode[scene] = dali_rx_selected_color_mode;
    //                 #endif
    //                 if(dali_rx_selected_color_mode == 0){
    //                     scene_group_switch_info.device_color[scene][i] = last_color_dtr0;
    //                     uint16_t color_temp_mirek = ((last_color_dtr1 << 8) & 0xff00) | last_color_dtr0;
    //                     uint16_t kelvin_cct = 1000000 / color_temp_mirek;
    //                     //2000 to 6500
    //                     scene_group_switch_info.device_color[scene][i] = kelvin_cct;  
    //                     if(last_dtr0 == 0) scene_group_switch_info.device_state[scene][i]  = false;
    //                     else scene_group_switch_info.device_state[scene][i]  = true;
    //                 }else{
    //                     uint16_t r = last_color_dtr0;
    //                     uint16_t g = last_color_dtr1;
    //                     uint16_t b = last_color_dtr2;

    //                     printf("RGB values R:%d G:%d B:%d\n", r, g, b);

    //                     uint16_t rgb_color = rgb_to_rgb565(r, g, b);
    //                     printf("RGB565 value:0x%x\n", rgb_color);
    //                     scene_group_switch_info.device_color[scene][i] = rgb_color;  
    //                     if(r == 0 && g == 0 && b == 0) scene_group_switch_info.device_state[scene][i]  = false;
    //                     else scene_group_switch_info.device_state[scene][i]  = true;
    //                 }
    //                 //printf("============DATA SAVED SUCCESSFULLY============\n");
    //                 nuos_store_dali_scene_switch_data_to_nvs(&scene_group_switch_info); 
    //                 break;
    //             }
    //         }                
    //         return;
    //     }

    //     // ---------------------------
    //     // BROADCAST SCENE RECALL
    //     // ---------------------------
    //     if(b1 == 0xFF && b2 >= 0x10 && b2 <= 0x1F){
    //         scene = b2 - 0x10;
    //         printf("BROADCAST RECALL SCENE %d  TOTAL IDS:%d\n", scene, scene_group_switch_info.total_ids[0]);
    //         // scene_group_switch_info.group_id[0] = scene;
    //         bool all_off = true;
    //         uint8_t max_level = 0;
    //         uint16_t max_cct = 2000;
    //         uint16_t max_rgb = 0;

    //         if(scene_group_switch_info.total_ids[0] > 0){
    //             for(int j=0; j<scene_group_switch_info.total_ids[0]; j++){
    //                 printf("scene_id:%d  saved_scene_id:%d\n", scene, scene_group_switch_info.device_scene[scene][j]);
    //                 if(scene_group_switch_info.device_scene[scene][j] == scene){
    //                     printf("scene id state:%d\n", scene_group_switch_info.device_scene[scene][j]);
    //                     if(scene_group_switch_info.device_state[scene][j]){
    //                         all_off = false;
    //                     }
    //                     if(scene_group_switch_info.device_level[scene][j] > max_level)
    //                         max_level = scene_group_switch_info.device_level[scene][j];

    //                     if(scene_group_switch_info.device_color_mode[scene] == 0){
    //                         if(scene_group_switch_info.device_color[scene][j] > max_cct)
    //                             max_cct = scene_group_switch_info.device_color[scene][j]; 
    //                     }else{
    //                         if(scene_group_switch_info.device_color[scene][j] > max_rgb){
    //                             max_rgb = scene_group_switch_info.device_color[scene][j];
    //                             printf("max_rgb updated:0x%x\n", max_rgb);
    //                         }   
    //                     }
    //                 }
    //             }
    //             #ifdef ENABLE_DALI_RECEIVER
    //             if(scene_group_switch_info.device_color_mode[scene] != selected_color_mode){
    //                 selected_color_mode = scene_group_switch_info.device_color_mode[scene];
    //                 mode_change_flag = true;
    //                 nuos_set_color_rgb_mode_attribute(0, selected_color_mode);
    //                  store_color_mode_value(selected_color_mode);
    //             }        
   
    //             if(scene_group_switch_info.device_color_mode[scene] == 0){
    //                 // device_info[0].device_state = false;
    //                 // device_info[1].device_state = false;
    //                 // device_info[2].device_state = false;

    //                 if(all_off){
    //                     printf("ALL OFF\n");
    //                     device_info[3].device_state = false;
    //                     //device_info[3].device_level = 0;
    //                     set_hardware(3, false);
    //                     nuos_set_state_attribute_rgb(3); 
    //                 }else{
    //                     printf("DALI ON AT:%d\n", max_level);
    //                     // device_info[0].device_state = false;
    //                     // device_info[1].device_state = false;
    //                     // device_info[2].device_state = false;
    //                     device_info[3].device_state = true;
    //                     device_info[3].device_level = max_level;
    //                     device_info[3].device_val = max_cct;  //2000 to 6500

    //                     store_color_mode_value(0);

    //                     set_hardware(3, false); 
    //                     if(!device_info[3].device_state) {
    //                         ledc_set_duty(LEDC_MODE, pwm_channels[3], 0);            
    //                         ledc_update_duty(LEDC_MODE, pwm_channels[3]);
    //                         nuos_set_state_attribute_rgb(3);       
    //                     } else {
    //                         ledc_set_duty(LEDC_MODE, pwm_channels[3], device_info[3].device_level);            
    //                         ledc_update_duty(LEDC_MODE, pwm_channels[3]);
    //                         nuos_set_state_attribute_rgb(3);
    //                         nuos_set_color_temp_attribute(3); 
    //                         nuos_set_color_temp_level_attribute(3);
    //                     }   
    //                 } 
    //             }else{
    //                 printf("DALI ON AT:0x%x\n", max_rgb);
    //                 device_info[3].device_state = false;
    //                 if(all_off){
    //                     printf("ALL OFF\n");
    //                     // device_info[0].device_state = false;
    //                     // device_info[1].device_state = false;
    //                     // device_info[2].device_state = false;
    //                     device_info[4].device_state = false;
    //                     set_hardware(4, false);
    //                     nuos_set_state_attribute_rgb(4); 
    //                 }else{
    //                     rgb565_to_rgb(max_rgb, 
    //                         &device_info[0].device_level, 
    //                         &device_info[1].device_level, 
    //                         &device_info[2].device_level);

    //                     printf("r:%d g:%d b:%d\n", device_info[0].device_level, 
    //                         device_info[1].device_level, 
    //                         device_info[2].device_level);

    //                     if(device_info[0].device_level == 0) device_info[0].device_state = false;
    //                     else device_info[0].device_state = true;
    //                     if(device_info[1].device_level == 0) device_info[1].device_state = false;
    //                     else device_info[1].device_state = true;
    //                     if(device_info[2].device_level == 0) device_info[2].device_state = false;
    //                     else device_info[2].device_state = true;


    //                     for(int rgb=0; rgb<3; rgb++){
    //                         if(device_info[rgb].device_level <= MIN_DIM_LEVEL_VALUE) {
    //                             device_info[rgb].device_level = MIN_DIM_LEVEL_VALUE;
    //                         }
    //                         if(device_info[rgb].device_level == 0xff){
    //                             device_info[rgb].device_level = 0xfe;
    //                         }
    //                     } 
    //                     if(!device_info[4].device_state){
    //                         device_info[4].device_state = true;
    //                     } 
    //                     nuos_zb_set_hardware(4, false); 
    //                     nuos_set_state_attribute_rgb(4); 

    //                     rgb_t rgb = {device_info[0].device_level, device_info[1].device_level, device_info[2].device_level}; // Example RGB values
    //                     hsv_t hsv2 = rgb_to_hsv(rgb); 
    //                     hsv2.s = 1000; 
    //                     hsv2.v = device_info[4].device_val; // Store brightness level for later use in brightness control                               
    //                     nuos_set_color_xy_attribute(4, &hsv2);
    //                 }          

    //             }  
    //             #endif
    //         }          
    //         return;
    //     }



    //     // ---------------------------
    //     // ARC POWER CONTROL (DAPC)
    //     // ---------------------------
    //     if(b2 <= 0xFE) {
    //         // Broadcast
    //         if(b1 == 0xFE)
    //         {
    //             // if(b2 == 0){
    //             //     //printf("BROADCAST → OFF\n");
    //             //     device_info[0].device_state = false;
    //             //     set_hardware(0, false);
    //             // }else{
    //             //     //printf("BROADCAST → LEVEL %d\n", b2);
    //             //     device_info[0].device_state = true;
    //             //     device_info[0].device_level = b2;
    //             //     set_hardware(0, false);        
                                 
    //             // }
    //             return;
    //         }

    //         // Group command
    //         if(b1 & 0x80)
    //         {
    //             uint8_t group = (b1 >> 1) & 0x0F;
    //             if(b2 == 0)
    //                 printf("GROUP %d → OFF\n", group);
    //             else
    //                 printf("GROUP %d → LEVEL %d\n", group, b2);

    //             return;
    //         }

    //         // Short address command
    //         addr = (b1 >> 1) & 0x3F;

    //         if(b2 == 0){
    //             printf("DEVICE %d → OFF\n", addr);
    //         }else{
    //             printf("DEVICE %d → LEVEL %d\n", addr, b2);
    //         }
    //         return;
    //     }

    //     // ---------------------------
    //     // UNKNOWN
    //     // ---------------------------
    //     printf("UNKNOWN FRAME: %02X %02X\n", b1, b2);
    // }
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

    // -------------------------------------------------
    // Decode address / frame type first
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
    // DTR staging
    // -------------------------------------------------
    if (b1 == dali.SET_DTR0) {
        last_dtr0 = b2;
        return;
    }

    if (b1 == dali.SET_DTR1) {
        last_dtr1 = b2;
        last_color_dtr0 = last_dtr0;
        last_color_dtr1 = b2;
        last_color_mode = 0;
        dali_rx_selected_color_mode = 0;
        return;
    }

    if (b1 == dali.SET_DTR2) {
        last_color_dtr0 = last_dtr0;
        last_color_dtr1 = last_dtr1;
        last_color_dtr2 = b2;
        last_color_mode = 1;
        dali_rx_selected_color_mode = 1;
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

    // -------------------------------------------------
    // Store Scene (0..15)
    // -------------------------------------------------
    if (b2 >= 0x40 && b2 <= 0x4F) {
        if (!is_short) {
            return;
        }

        scene = b2 - 0x40;

        for (int i = 0; i < scene_group_switch_info.total_ids[0]; i++) {
            if (addr == scene_group_switch_info.device_ids[0][i]) {
                scene_group_switch_info.device_scene[scene][i] = scene;
                scene_group_switch_info.device_level[scene][i] = last_dtr0;

#ifdef ENABLE_DALI_RECEIVER
                scene_group_switch_info.device_color_mode[scene] = last_color_mode;
#endif

                if (last_color_mode == 0) {
                    uint16_t color_temp_mirek = ((uint16_t)last_color_dtr1 << 8) | last_color_dtr0;
                    uint16_t kelvin_cct = MIN_CCT_VALUE;

                    if (color_temp_mirek != 0) {
                        kelvin_cct = 1000000UL / color_temp_mirek;
                    }

                    scene_group_switch_info.device_color[scene][i] = kelvin_cct;
                    scene_group_switch_info.device_state[scene][i] = (last_dtr0 != 0);
                } else {
                    uint8_t r = last_color_dtr0;
                    uint8_t g = last_color_dtr1;
                    uint8_t b = last_color_dtr2;

                    uint16_t rgb565_color = rgb_to_rgb565(r, g, b);
                    scene_group_switch_info.device_color[scene][i] = rgb565_color;
                    scene_group_switch_info.device_state[scene][i] = !((r == 0) && (g == 0) && (b == 0));
                }

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

    // -------------------------------------------------
    // Broadcast Recall Scene (0..15)
    // -------------------------------------------------
    if (b1 == 0xFF && b2 >= 0x10 && b2 <= 0x1F) {
        scene = b2 - 0x10;

        // printf("BROADCAST RECALL SCENE %u TOTAL IDS:%d\n",
        //        scene, scene_group_switch_info.total_ids[0]);

        bool all_off = true;
        uint8_t max_level = 0;
        uint16_t max_cct = MIN_CCT_VALUE;
        uint16_t max_rgb565 = 0;

        if (scene_group_switch_info.total_ids[0] > 0) {
            for (int j = 0; j < scene_group_switch_info.total_ids[0]; j++) {
                if (scene_group_switch_info.device_scene[scene][j] == scene) {
                    if (scene_group_switch_info.device_state[scene][j]) {
                        all_off = false;
                    }

                    if (scene_group_switch_info.device_level[scene][j] > max_level) {
                        max_level = scene_group_switch_info.device_level[scene][j];
                    }

                    if (scene_group_switch_info.device_color_mode[scene] == 0) {
                        if (scene_group_switch_info.device_color[scene][j] > max_cct) {
                            max_cct = scene_group_switch_info.device_color[scene][j];
                        }
                    } else {
                        if (scene_group_switch_info.device_color[scene][j] > max_rgb565) {
                            max_rgb565 = scene_group_switch_info.device_color[scene][j];
                        }
                    }
                }
            }

#ifdef ENABLE_DALI_RECEIVER
            if (scene_group_switch_info.device_color_mode[scene] != selected_color_mode) {
                selected_color_mode = scene_group_switch_info.device_color_mode[scene];
                mode_change_flag = true;
                nuos_set_color_rgb_mode_attribute(0, selected_color_mode);
                store_color_mode_value(selected_color_mode);
            }

            if (scene_group_switch_info.device_color_mode[scene] == 0) {
                if (all_off) {
                    device_info[3].device_state = false;
                    set_hardware(3, false);
                    nuos_set_state_attribute_rgb(3);
                } else {
                    device_info[3].device_state = true;
                    device_info[3].device_level = max_level;
                    device_info[3].device_val = max_cct;

                    store_color_mode_value(0);
                    set_hardware(3, false);

                    if (!device_info[3].device_state) {
                        ledc_set_duty(LEDC_MODE, pwm_channels[3], 0);
                        ledc_update_duty(LEDC_MODE, pwm_channels[3]);
                        nuos_set_state_attribute_rgb(3);
                    } else {
                        ledc_set_duty(LEDC_MODE, pwm_channels[3], device_info[3].device_level);
                        ledc_update_duty(LEDC_MODE, pwm_channels[3]);
                        nuos_set_state_attribute_rgb(3);
                        nuos_set_color_temp_attribute(3);
                        nuos_set_color_temp_level_attribute(3);
                    }
                }
            } else {
                if (all_off) {
                    device_info[4].device_state = false;
                    set_hardware(4, false);
                    nuos_set_state_attribute_rgb(4);
                } else {
                    rgb565_to_rgb(max_rgb565,
                                  &device_info[0].device_level,
                                  &device_info[1].device_level,
                                  &device_info[2].device_level);

                    device_info[0].device_state = (device_info[0].device_level != 0);
                    device_info[1].device_state = (device_info[1].device_level != 0);
                    device_info[2].device_state = (device_info[2].device_level != 0);

                    for (int rgb = 0; rgb < 3; rgb++) {
                        if (device_info[rgb].device_level <= MIN_DIM_LEVEL_VALUE &&
                            device_info[rgb].device_level != 0) {
                            device_info[rgb].device_level = MIN_DIM_LEVEL_VALUE;
                        }
                        if (device_info[rgb].device_level == 0xFF) {
                            device_info[rgb].device_level = 0xFE;
                        }
                    }

                    device_info[4].device_state = true;
                    nuos_zb_set_hardware(4, false);
                    nuos_set_state_attribute_rgb(4);

                    rgb_t rgb = {
                        device_info[0].device_level,
                        device_info[1].device_level,
                        device_info[2].device_level
                    };
                    hsv_t hsv2 = rgb_to_hsv(rgb);
                    hsv2.s = 1000;
                    hsv2.v = device_info[4].device_val;
                    nuos_set_color_xy_attribute(4, &hsv2);
                }
            }
#endif
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

    // -------------------------------------------------
    // ARC POWER CONTROL (DAPC)
    // -------------------------------------------------
    if (b2 <= 0xFE) {
        if (b1 == 0xFE) {
            return;
        }

        if (is_group) {
            if (b2 == 0) {
                printf("GROUP %u -> OFF\n", group);
            } else {
                printf("GROUP %u -> LEVEL %u\n", group, b2);
            }
            return;
        }

        if (is_short) {
            if (b2 == 0) {
                printf("DEVICE %u -> OFF\n", addr);
            } else {
                printf("DEVICE %u -> LEVEL %u\n", addr, b2);
            }
            return;
        }
    }

    // -------------------------------------------------
    // Unknown frame
    // -------------------------------------------------
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

        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
            DMX::Initialize(output, LOAD_6_PIN, -1);
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI)
            dali.begin(&isr_service_installed);
            if(wifi_webserver_active_flag == 0){
                rxFrameQueue = xQueueCreate(10, sizeof(DaliMessage));
                if (rxFrameQueue == nullptr) {
                    return;
                }        
                #ifdef ENABLE_DALI_RECEIVER
                    dali.begin_rx(&isr_service_installed, rxFrameQueue);
                    xTaskCreate(receiveDaliFrame, "dali_task_2", 4096, NULL, 23, NULL);   
                #endif
                vTaskDelay(10 / portTICK_PERIOD_MS); 
            } 
        #endif
    }  

    void set_hardware(uint8_t index, uint8_t is_toggle) {     
        
        if(is_toggle>0){

            if(index < 3){
                #if(USE_COLOR_DEVICE == COLOR_RGBW || USE_COLOR_DEVICE == COLOR_RGB_CW_WW) 
                if(mode_change_flag)
                    if(device_info[index].device_state) device_info[4].device_state = true;
                #endif
                if((!device_info[0].device_state && !device_info[1].device_state && !device_info[2].device_state) || !device_info[4].device_state){
                    printf("ALL RGB OFF\n");
                    if(!device_info[4].device_state){
                      device_info[4].device_state = true;  
                    }else{
                        if(!device_info[0].device_state && !device_info[1].device_state && !device_info[2].device_state)
                            device_info[index].device_state = true; 
                    }
                }else{
                    device_info[4].device_state = true;
                }
            }     

            #if(USE_COLOR_DEVICE == COLOR_RGBW || USE_COLOR_DEVICE == COLOR_RGB_CW_WW)  
            if(!mode_change_flag)
            #else
            mode_change_flag = false;
            #endif
            device_info[index].device_state = !device_info[index].device_state;         
        }

        last_selected_color_mode = selected_color_mode;
        if(is_init_done){ 
            printf("color_mode:%d\n", selected_color_mode);
            switch(selected_color_mode){
                case 0:
                    //printf("state:%d  level:%d\n", device_info[3].device_state, device_info[3].device_level);
                    #if(USE_COLOR_DEVICE == COLOR_RGB_CW_WW || USE_COLOR_DEVICE == COLOR_RGBW)
                    if(!device_info[3].device_state){
                        device_info[4].device_state = false;
                        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[3], 0));
                        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[3]));                       
                    }else{ //if CCT state TRUE    
                        device_info[4].device_state = true;  // ALL ON     
                        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[3], device_info[3].device_level));
                        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[3]));                                   
                    } 
                    for(int j=0; j<3; j++){             //Turn OFF RGB States
                        dmx_data[dmx_start_address+j] = 0;
                        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[j], 0));
                        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[j]));
                    }             
                    nuos_store_data_to_nvs(index); 
                    nuos_store_data_to_nvs(4);
                    break;
                    #endif
                    
                case 2:    
                case 1:     
                    if((!device_info[0].device_state && !device_info[1].device_state && !device_info[2].device_state) || !device_info[4].device_state){
                        device_info[4].device_state = false;
                        for(int i=0; i<3; i++){
                            dmx_data[dmx_start_address+i] = 0;
                            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], 0));
                            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[i]));
                        } 
                        #if(USE_COLOR_DEVICE == COLOR_RGB_CW_WW || USE_COLOR_DEVICE == COLOR_RGBW)
                        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[3], 0));
                        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[3]));
                        #else
                        device_info[3].device_level = 0;
                        device_info[4].device_level = device_info[3].device_level;
                        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[3], device_info[3].device_level));
                        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[3]));                        
                        #endif
                    } else {
                        #if(USE_COLOR_DEVICE == COLOR_RGB_CW_WW || USE_COLOR_DEVICE == COLOR_RGBW)
                        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[3], 0));
                        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[3]));
                        #else
                        device_info[3].device_level = 254;
                        device_info[4].device_level = device_info[3].device_level;
                        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[3], device_info[3].device_level));
                        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[3]));                        
                        #endif 
                        for(int i=0; i<3; i++){
                            if(!device_info[i].device_state){
                                dmx_data[dmx_start_address+i] = 0;
                                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], 0));
                                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[i]));
                            } else { 
                                dmx_data[dmx_start_address+i] = device_info[i].device_level;   
                                //printf("LED%d ON at level %d\n", i, device_info[i].device_level);                           
                                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], dmx_data[dmx_start_address+i]));
                                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[i]));                
                            } 
                            nuos_store_data_to_nvs(i);
                        }                                              
                    }  
                    if(index == 4) nuos_store_data_to_nvs(4); 
                    break;
                default: break;      
            }

            // if(mode_change_flag){
            //     nuos_set_color_rgb_mode_attribute(0, selected_color_mode);
            // }
        }
    }

    void set_leds(int i, bool state){
        if(gpio_touch_led_pins[i] != -1){
            if(i == 0){
                if(state) ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], device_info[i].device_level));
                else ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], 0));
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[i]));
            }else{
                if(state) ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], device_info[i].device_level));
                else ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], 0));
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[i]));
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


    void nuos_zb_set_hardware(uint8_t index, uint8_t is_toggle){
        //set touch led pins
        call_common_check_auto_off();
        set_hardware(index, is_toggle);
    }

    void set_level_value(uint8_t _level){
        level = _level;
    }

    void nuos_on_off_led(uint8_t index, uint8_t _state){
        if(is_init_done){
            if(index < TOTAL_LEDS){
                if(gpio_touch_led_pins[index] != -1){                 
                    if(_state)  {
                        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], level));
                    }else{
                        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], 0));
                    }            
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[index]));
                }
            }
        }
    }

    extern "C" void nuos_toggle_leds(uint8_t index){
        state = !state;
        if(state) level = LEDC_MAX_DUTY;        
        nuos_on_off_led(index, level);
    }
    

    extern "C" void nuos_zb_set_hardware_led_for_zb_commissioning(uint8_t is_toggle){
        #ifdef USE_RGB_LED
            nuos_toggle_rgb_led();
        #else
            uint8_t _level = 0;
            if(TOTAL_LEDS >= TOTAL_LEDS_SHOW_ON_COMMISSIONING){
                if(is_toggle>0) {
                    state = !state; 
                    if(state) _level = LEDC_MAX_DUTY; 
                    else _level = 0;
                }
                for(int index=TOTAL_LEDS-TOTAL_LEDS_SHOW_ON_COMMISSIONING; index<TOTAL_LEDS; index++){
                    if(is_toggle==0) {
                        state = device_info[index].device_state; 
                        _level = device_info[index].device_level;
                    }
                    if(is_init_done){
                        nuos_on_off_led(index, _level);
                    }
                }
            }
        #endif
    } 


    uint8_t nuos_get_button_press_index(uint32_t pin){
        for(int index=0; index<TOTAL_BUTTONS; index++){
            if(pin == gpio_touch_btn_pins[index]){
                return index;
            }
        }
        return 0;
    }

    extern "C" void nuos_init_hardware_dimming_up_down(uint32_t pin){
        uint8_t index = nuos_get_button_press_index(pin);
        if(device_info[index].device_state){
            if(selected_color_mode == 0){
                if(!change_cw_ww_color_flag) {
                    if(device_info[index].device_level <= MIN_DIM_LEVEL_VALUE+20){
                        device_info[index].dim_up = 1;
                    }else if(device_info[index].device_level >= MAX_DIM_LEVEL_VALUE-20){
                        device_info[index].dim_up = 0;
                    }
                }else{
                    if(device_info[index].device_val <= MIN_CCT_VALUE+200){
                        device_info[index].dim_up = 1;
                    }else if(device_info[index].device_val >= MAX_CCT_VALUE-200){
                        device_info[index].dim_up = 0;
                    }
                }
            }else{
                if(device_info[index].device_level <= MIN_DIM_LEVEL_VALUE+20){
                    device_info[index].dim_up = 1;
                }else if(device_info[index].device_level >= MAX_DIM_LEVEL_VALUE-20){
                    device_info[index].dim_up = 0;
                }
            }
        }
    }


    static uint8_t level_backup[4] = {0, 0, 0, 0};
    static uint16_t val_backup[4] = {0, 0, 0, 0};

    bool nuos_set_hw_level_brightness(uint8_t index){
        #if(USE_COLOR_DEVICE == COLOR_RGB_ONLY)
        if(index < TOTAL_BUTTONS-1){
        #else
        if(index <= TOTAL_BUTTONS-1){
        #endif    
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

                if(level_backup[index] != device_info[index].device_level){
                    level_backup[index] = device_info[index].device_level;
                    #if(USE_COLOR_DEVICE == COLOR_RGB_ONLY)
                    if(index == 3)
                    device_info[4].device_level = device_info[3].device_level;
                    #endif
                    // printf("device_info[%d].device_level:%d\n", index, device_info[index].device_level);
                    return true;                      
                }             
            }
        }
        return false;
    }

    
    bool nuos_set_hw_color_temperature(uint8_t index) {

        if(index <= TOTAL_BUTTONS-1){

            if(device_info[index].device_state){
               
                if(selected_color_mode == 0){
                    if(device_info[index].dim_up == 1){
                        if(device_info[index].device_val + COLOR_STEPS <= MAX_CCT_VALUE){
                            device_info[index].device_val += COLOR_STEPS;
                        } else {
                            device_info[index].device_val = MAX_CCT_VALUE;
                        }
                    }else{
                        if(device_info[index].device_val - COLOR_STEPS >= MIN_CCT_VALUE){
                            device_info[index].device_val -= COLOR_STEPS;  
                        }else {
                            device_info[index].device_val = MIN_CCT_VALUE;
                        } 
                    }
                    printf("VALUE:%d\n", device_info[index].device_val);

                    if(val_backup[index] != device_info[index].device_val){
                        val_backup[index] = device_info[index].device_val;
                        if(gpio_touch_led_pins[index] != -1){
                            printf("Return OK\n");
                            return true;        
                        }                         
                    } 
                }else{
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
                    printf("Return OK\n");
                    return true;                    
                }            
            }
        }
        return false;
    }

    uint8_t rgb[3] = {0,0,0};
    uint32_t xxcounts = 0;
    bool nuos_set_hardware_brightness(uint32_t pin){
        //switch_driver_gpios_intr_enabled(true); 
        call_common_check_auto_off();
        uint8_t index = nuos_get_button_press_index(pin);
        brightness_control_flag = true;
        //if(global_switch_state == SWITCH_PRESS_DETECTED){ 
            if(!device_info[index].device_state){
                device_info[index].device_state = true;
                #if(USE_COLOR_DEVICE == COLOR_RGB_ONLY)
                    change_cw_ww_color_flag = false;
                    device_info[index].device_level = MIN_DIM_LEVEL_VALUE;
                #else
                    if(selected_color_mode == 0){
                        if(!change_cw_ww_color_flag) {
                            device_info[index].device_level = MIN_DIM_LEVEL_VALUE;
                        }else{
                            device_info[index].device_val = MIN_CCT_VALUE;
                        }
                    }else{
                        device_info[index].device_level = MIN_DIM_LEVEL_VALUE;
                    }
                #endif

                device_info[index].dim_up = 1;              
            } 
            #if(USE_COLOR_DEVICE == COLOR_RGB_ONLY)
            selected_color_mode = 1;
            #endif
            if(selected_color_mode == 0){
                if(!change_cw_ww_color_flag) {
                    printf("change_cw_ww_color_flag = FALSE\n");
                    if(nuos_set_hw_level_brightness(3)){
                        set_hardware(3, false);
                        set_dali_level(3);
                    }
                    if(xxcounts++ % 10 == 0){
                        nuos_set_color_temp_level_attribute(3);
                    }
                }else{
                    // printf("change_cw_ww_color_flag = TRUE\n");
                    if(nuos_set_hw_color_temperature(3)){
                        set_hardware(3, false);
                        set_dali_color_temp(0, false);
                        
                    }                
                } 
            }else{
                change_cw_ww_color_flag = false;
                if(nuos_set_hw_level_brightness(index)){
                    if(!device_info[0].device_state) dmx_data[dmx_start_address] = MIN_DIM_LEVEL_VALUE;
                    else {
                        if(device_info[0].device_level >= MIN_DIM_LEVEL_VALUE){
                            if(device_info[4].device_level >= MIN_DIM_LEVEL_VALUE){
                                dmx_data[dmx_start_address] = (uint8_t)((float)device_info[0].device_level * (device_info[4].device_level / 255.0f));
                            }
                        }
                    }

                    if(!device_info[1].device_state) dmx_data[dmx_start_address+1] = MIN_DIM_LEVEL_VALUE;
                    else {
                        if(device_info[1].device_level >= MIN_DIM_LEVEL_VALUE){
                            if(device_info[4].device_level >= MIN_DIM_LEVEL_VALUE){
                                dmx_data[dmx_start_address+1] = (uint8_t)((float)device_info[1].device_level * (device_info[4].device_level / 255.0f));
                            }
                        }
                    }

                    if(!device_info[2].device_state) dmx_data[dmx_start_address+2] = MIN_DIM_LEVEL_VALUE;
                    else {
                        if(device_info[2].device_level >= MIN_DIM_LEVEL_VALUE){
                            if(device_info[4].device_level >= MIN_DIM_LEVEL_VALUE){
                                dmx_data[dmx_start_address+2] = (uint8_t)((float)device_info[2].device_level * (device_info[4].device_level / 255.0f));
                            }
                        }
                    }

                    //device_info[4].device_level = max_of_three(dmx_data[dmx_start_address] , dmx_data[dmx_start_address+1] , dmx_data[dmx_start_address+2] );
                    set_hardware(index, false);
                    printf("R:%d G:%d B:%d\n", dmx_data[dmx_start_address], dmx_data[dmx_start_address+1], dmx_data[dmx_start_address+2]);
                    #if(USE_COLOR_DEVICE == COLOR_RGB_ONLY)
                        if(index == 3)
                        if(xxcounts % 10 == 0){
                            set_dali_level(3);
                        }
                    #else
                    device_info[4].device_level = max_of_three(dmx_data[dmx_start_address] , dmx_data[dmx_start_address+1] , dmx_data[dmx_start_address+2]);
                    if(device_info[4].device_level  < MIN_DIM_LEVEL_VALUE){
                        device_info[4].device_level = MIN_DIM_LEVEL_VALUE;
                    }                    
                    #endif                    
                    set_dali_color_temp(0, false);
                    if(xxcounts++ % 20 == 0){
                        nuos_set_level_attribute(4);

                    }
                }
            } 

        //}
        return false;
    }

    void nuos_set_state_touch_leds(bool state) {
        for(int i=0; i<TOTAL_LEDS; i++){
             set_leds(i, state);
        }
    }

    extern "C" void nuos_set_state_touch_leds_to_original() {
        for(int i=0; i<TOTAL_LEDS; i++){
            if(device_info[i].device_state){
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], device_info[i].device_level));
            }else{
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], 0));
            }
            
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[i]));
        }
    }



    //////////////////////////////////////////////////////////////////////////////////////////

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
            dali.set_broadcast_color_cct(value);
        }
    } 

    extern "C" void nuos_dali_set_group_brightness(uint8_t group_id, uint8_t index, uint8_t value){
        dali.set_group_level(group_id, value);
    }
    
    bool _toggle_ = false;
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
        //#ifdef IS_USE_DALI_HARDWARE
        //dali.set_group_color_cct(dali_nvs_stt[index].group_id, cct_values[cct_counts]); //

        dali.set_color_temperature(dali.BROADCAST_C, cct_values[cct_counts]);  
        //#endif
        nuos_store_data_to_nvs(0);
        // if(wifi_webserver_active_flag == 0){
        //     nuos_set_zigbee_attribute(index);
        // }
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
        //switch_driver_gpios_intr_enabled(false); 
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
        //switch_driver_gpios_intr_enabled(true); 
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
        xTaskCreate(esp_dali_init_node_task, "dali_task", 8192, &addr, 9, NULL);
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
            nuos_zb_set_hardware(i, false);
        }
    }    

#endif




