
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

    extern "C" int max_of_three(int a, int b, int c) {
        int max = a;
        if (b > max) max = b;
        if (c > max) max = c;
        return max;
    }
    

    extern "C" unsigned int map_1000(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);
    void set_color_hw(uint8_t index);

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
            dali.set_fade_rate(global_dali_id[0], 12);
            vTaskDelay(20 / portTICK_PERIOD_MS);
            dali.set_fade_time(global_dali_id[0], 1);
            vTaskDelay(20 / portTICK_PERIOD_MS);
        #elif(COMMUNICATION_MODE == COMM_MODE_GROUP_CTRL)            
            dali.add_to_group(1, global_group_id[0]);              
            vTaskDelay(20 / portTICK_PERIOD_MS);                
            dali.set_group_fade_rate(global_group_id[0], 12);
            vTaskDelay(20/ portTICK_PERIOD_MS);
            dali.set_group_fade_time(global_group_id[0], 1);     
        #elif(COMMUNICATION_MODE == COMM_MODE_BROADCAST)
            dali.set_fade_rate(dali.BROADCAST_C, 12);
            vTaskDelay(5/ portTICK_PERIOD_MS); //taskYIELD(); 
            dali.set_fade_time(dali.BROADCAST_C, 1);
            vTaskDelay(5 / portTICK_PERIOD_MS);  //taskYIELD();              
            // dali.set_ext_fade_time(dali.BROADCAST_C, 2);
            // vTaskDelay(5 / portTICK_PERIOD_MS); //taskYIELD(); 
            
        #endif
        dali.set_rgbwaf_ctrl(); //taskYIELD(); 
        #endif
    }

    bool set_color_task_flag = false;

    extern "C" void set_state(uint8_t index){
        set_state_flag = false;
        if(is_init_done){  
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
                printf("set_state   R:%d G:%d B:%d\n", dmx_data[dmx_start_address], dmx_data[dmx_start_address+1], dmx_data[dmx_start_address+2]);
                DMX::WriteAll(&dmx_data[dmx_start_address], dmx_start_address, 5);
            #else 
            
            //esp_zb_lock_acquire(portMAX_DELAY);     
            //printf("State ==> mode:%d\n", selected_color_mode);    
            //taskENTER_CRITICAL();    
            if(selected_color_mode == 0){
                //printf("device_info[3].device_state:%d\n", device_info[3].device_state);   
                #if(USE_COLOR_DEVICE == COLOR_RGBW || USE_COLOR_DEVICE == COLOR_RGB_CW_WW) 
                    #if(COMMUNICATION_MODE == COMM_MODE_ADDR_CTRL)
                        //taskYIELD(); 
                        if(!device_info[3].device_state) dali.turn_off(global_dali_id[0]);
                        else { 
                            dali.set_dim_value(global_dali_id[0], device_info[3].device_level);
                            //dali.turn_on_to_last_level(global_dali_id[0]);
                        } 
                    #elif(COMMUNICATION_MODE == COMM_MODE_GROUP_CTRL) 
                        taskYIELD(); 
                        if(!device_info[3].device_state) dali.set_group_off(global_group_id[0]);
                        else { 
                            //dali.set_group_on(global_group_id[0]); 
                            
                            dali.set_group_on(global_group_id[0]);
                        }  
                    #elif(COMMUNICATION_MODE == COMM_MODE_BROADCAST)
                        taskYIELD(); 
                        if(!device_info[3].device_state) dali.send_broadcast(dali.OFF_C);
                        else { 
                            //dali.set_broadcast_level(device_info[3].device_level); 
                            
                            dali.set_broadcast_level(device_info[3].device_level);
                        }                    
                    #endif

                #else
                if(!device_info[4].device_state) dali.send_broadcast(dali.OFF_C);
                else {
                    //dali.send_broadcast(dali.ON_C); 
                    #if(COMMUNICATION_MODE == COMM_MODE_ADDR_CTRL)
                        dali.set_dim_value(global_dali_id[0], device_info[4].device_level);
                    #elif(COMMUNICATION_MODE == COMM_MODE_GROUP_CTRL) 
                        dali.set_group_level(global_group_id[0], device_info[4].device_level);
                    #elif(COMMUNICATION_MODE == COMM_MODE_BROADCAST)
                        //dali.send_broadcast(0x0A); 
                        dali.set_broadcast_level(device_info[4].device_level);
                    #endif                    

                }
                #endif
                //taskYIELD(); 
                nuos_set_state_attribute_rgb(3);
            }else{
                #if(COMMUNICATION_MODE == COMM_MODE_ADDR_CTRL)
                    //taskYIELD(); 
                    if(!device_info[4].device_state){
                        dali.turn_off(global_dali_id[0]);
                        vTaskDelay(5 / portTICK_PERIOD_MS);
                        dali.turn_off(global_dali_id[0]);
                    } else { 
                        //dali.set_broadcast_level(device_info[4].device_level); 
                        dali.set_dim_value(global_dali_id[0], device_info[4].device_level);
                        vTaskDelay(5 / portTICK_PERIOD_MS);
                        dali.set_dim_value(global_dali_id[0], device_info[4].device_level);
                        //dali.turn_on_to_last_level(global_dali_id[0]);
                    } 
                #elif(COMMUNICATION_MODE == COMM_MODE_GROUP_CTRL) 
                    taskYIELD(); 
                    if(!device_info[4].device_state) dali.set_group_off(global_group_id[0]);
                    else { 
                        //dali.set_group_on(global_dali_id[0]); 
                        
                        dali.set_group_on(global_group_id[0]);
                    }  
                #elif(COMMUNICATION_MODE == COMM_MODE_BROADCAST)
                    if(!device_info[4].device_state) dali.send_broadcast(dali.OFF_C);
                    else {
                        taskYIELD(); 
                        dali.set_broadcast_level(device_info[4].device_level);
                    }
                #endif

               
                    
                nuos_set_state_attribute_rgb(4);
            }

            //taskEXIT_CRITICAL();    
            #endif
            //printf("State END\n"); 
        }
    }   

    extern "C" void set_level(uint8_t index){
        set_level_flag = false;
        if(is_init_done){  
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
                printf("set_level   R:%d G:%d B:%d\n", dmx_data[dmx_start_address], dmx_data[dmx_start_address+1], dmx_data[dmx_start_address+2]);
                DMX::WriteAll(&dmx_data[dmx_start_address], dmx_start_address, 5);
            #else   
                //printf("Level ==> mode:%d\n", selected_color_mode);  
                //esp_zb_lock_acquire(portMAX_DELAY);       
                if(selected_color_mode == 0){
                    #if(COMMUNICATION_MODE == COMM_MODE_ADDR_CTRL)
                    if(!device_info[3].device_state) dali.set_dim_value(global_dali_id[0], 0);
                    else dali.set_dim_value(global_dali_id[0], device_info[3].device_level);
                    #elif(COMMUNICATION_MODE == COMM_MODE_GROUP_CTRL) 
                    if(!device_info[3].device_state) dali.set_group_level(global_dali_id[0], 0);
                    else dali.set_group_level(global_dali_id[0], device_info[3].device_level);
                    #elif(COMMUNICATION_MODE == COMM_MODE_BROADCAST)
                        if(!device_info[3].device_state) dali.set_broadcast_level(0);
                        else{ 
                            dali.set_broadcast_level(device_info[3].device_level);
                            dali.set_broadcast_level(device_info[3].device_level);
                        } 
                    #endif

                    nuos_set_color_temp_level_attribute(3); 
                                    
                }else{ 
                    // printf("device_info[4].device_state:%d\n", device_info[4].device_state);
                    // printf("device_info[4].device_level:%d\n", device_info[4].device_level);
                    #if(COMMUNICATION_MODE == COMM_MODE_ADDR_CTRL)
                        if(!device_info[4].device_state) dali.set_dim_value(global_dali_id[0], 0);
                        else dali.set_dim_value(global_dali_id[0], device_info[4].device_level);
                    #elif(COMMUNICATION_MODE == COMM_MODE_GROUP_CTRL) 
                        if(!device_info[3].device_state) dali.set_group_level(global_dali_id[0], 0);
                        else dali.set_group_level(global_dali_id[0], device_info[4].device_level);
                    #elif(COMMUNICATION_MODE == COMM_MODE_BROADCAST)
                        if(!device_info[4].device_state) dali.set_broadcast_level(0);
                        else dali.set_broadcast_level(device_info[4].device_level);
                    #endif

                    nuos_set_level_attribute(3);                      
                }
                //esp_zb_lock_release();
            #endif  
           // printf("Level END\n");             
        }
    }

    extern "C" void set_color_temp(){
        set_color_flag = false;
        if(is_init_done){  
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
                DMX::WriteAll(&dmx_data[dmx_start_address], dmx_start_address, 5);
                rgb_t rgb = {dmx_data[dmx_start_address], dmx_data[dmx_start_address+1], dmx_data[dmx_start_address+2]}; // Example RGB values
                hsv_t hsv2 = rgb_to_hsv(rgb);
                                                           
                nuos_set_color_xy_attribute(4, &hsv2);
            #else  
            //esp_zb_lock_acquire(portMAX_DELAY);
           // printf("Color Start\n"); 
            if(selected_color_mode == 0){
                #if(COMMUNICATION_MODE == COMM_MODE_ADDR_CTRL)
                    if(mode_change_flag){
                        dali.set_color_rgb(global_dali_id[0], 0, 0, 0, device_info[3].device_level);
                        mode_change_flag = false; 
                    }  
                    if(device_info[3].device_state){
                        dali.set_color_temp(global_dali_id[0], device_info[3].device_val);
                        //taskYIELD(); 
                        dali.set_dim_value(global_dali_id[0], device_info[3].device_level);  
                    }
                #elif(COMMUNICATION_MODE == COMM_MODE_GROUP_CTRL) 
                    if(mode_change_flag){
                        dali.set_group_color_rgb(global_group_id[0], 0, 0, 0, device_info[3].device_level);
                        mode_change_flag = false; 
                    }  
                    if(device_info[3].device_state){
                        dali.set_group_color_cct(global_group_id[0], device_info[3].device_val);
                        //taskYIELD(); 
                        dali.set_group_level(device_info[3].device_level);  
                    }
                #elif(COMMUNICATION_MODE == COMM_MODE_BROADCAST)
                    if(mode_change_flag){
                        dali.set_broadcast_color_rgb( 0, 0, 0, device_info[3].device_level);
                        mode_change_flag = false; 
                    }  
                    if(device_info[3].device_state){
                        dali.set_color_temperature(dali.BROADCAST_C, device_info[3].device_val);
                        taskYIELD(); 
                        dali.set_broadcast_level(device_info[3].device_level);  
                    }
                #endif                
 
                taskYIELD(); 
                nuos_set_color_temp_attribute(3);            
            } else {


                #if(COMMUNICATION_MODE == COMM_MODE_ADDR_CTRL)
                    #if(USE_COLOR_DEVICE == COLOR_RGBW || USE_COLOR_DEVICE == COLOR_RGB_CW_WW)
                        if(mode_change_flag){
                            dali.set_off_waf_channels(global_dali_id[0]);
                            mode_change_flag = false;
                        }  
                    #endif           
                    dali.set_color_rgb(global_dali_id[0], dmx_data[dmx_start_address], dmx_data[dmx_start_address+1], dmx_data[dmx_start_address+2], device_info[4].device_level); 
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
                    taskYIELD();           
                    dali.set_broadcast_color_rgb(dmx_data[dmx_start_address], dmx_data[dmx_start_address+1], dmx_data[dmx_start_address+2], device_info[4].device_level); 
                #endif 

 
                if(!brightness_control_flag){
                    rgb_t rgb = {dmx_data[dmx_start_address], dmx_data[dmx_start_address+1], dmx_data[dmx_start_address+2]}; // Example RGB values
                    hsv_t hsv2 = rgb_to_hsv(rgb);
                    taskYIELD();                                              
                    nuos_set_color_xy_attribute(4, &hsv2);
                    taskYIELD();   
                }
            }
            //esp_zb_lock_release();
            #endif
           // printf("Color END\n"); 
        }
    } 

    extern "C" void init_dali_hw(){
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
            DMX::Initialize(output, LOAD_6_PIN, -1);
        #else(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI)
            dali.begin();
            vTaskDelay(10 / portTICK_PERIOD_MS); //taskYIELD();        
            //set_dali_fading();
        #endif    
        // --->Just for testing  start_dali_addressing()      
        // start_dali_addressing(3);
        //xTaskCreate(esp_hw_task, "Hw_task", 8192, NULL, TASK_PRIORITY_RGB, NULL);
    }  

    void set_hardware(uint8_t index, uint8_t is_toggle) {     

        if(is_toggle>0){
            #if(USE_COLOR_DEVICE == COLOR_RGBW || USE_COLOR_DEVICE == COLOR_RGB_CW_WW)  
            if(!mode_change_flag)
            #else
            mode_change_flag = false;
            #endif
            device_info[index].device_state = !device_info[index].device_state;
            if(index < 3){
                if(device_info[index].device_state)
                device_info[4].device_state = true;
            }            
        }
        last_selected_color_mode = selected_color_mode;
        // printf("mode_change_flag:%d  selected_color_mode:%d\n", mode_change_flag, selected_color_mode);
        if(is_init_done){ 
            switch(selected_color_mode){
                case 0:
                    #if(USE_COLOR_DEVICE == COLOR_RGB_CW_WW || USE_COLOR_DEVICE == COLOR_RGBW)
                    if(!device_info[3].device_state){
                        device_info[4].device_state = false;
                        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[3], 0));
                        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[3]));
                        //ledc_set_                         
                    }else{                              //if CCT state TRUE    
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
                                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], dmx_data[dmx_start_address+i]));
                                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[i]));                
                            } 
                            nuos_store_data_to_nvs(i);
                        }                                              
                    }  
                    if(index == 4)     
                    nuos_store_data_to_nvs(4); 
                    break;
                default: break;      
            }
            // printf("DS4:%d DS3:%d\n", device_info[4].device_state, device_info[3].device_state);

            if(mode_change_flag){
                nuos_set_color_rgb_mode_attribute(0, selected_color_mode);
            }
            // printf("LEVEL4!!:%d LEVEL3!!:%d\n", device_info[4].device_level, device_info[3].device_level);

            // printf("S1:%d S2:%d S3:%d\n", device_info[0].device_state, device_info[1].device_state, device_info[2].device_state);
            // printf("L1:%d L2:%d L3:%d\n", device_info[0].device_level, device_info[1].device_level, device_info[2].device_level);

            // printf("R:%d G:%d B:%d\n", dmx_data[dmx_start_address], dmx_data[dmx_start_address+1] , dmx_data[dmx_start_address+2]);
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
        if(timer3_running_flag){
            //toggle pins on button press
            set_hardware(index, is_toggle);
        }else{
            if(nuos_check_state_touch_leds()){
                for(int i=0; i<TOTAL_LEDS; i++){
                    set_leds(i, device_info[i].device_state);                       
                }
            }else{
                //toggle pins on button press
                set_hardware(index, is_toggle);               
            }
        }
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
                //uint8_t last_dim =  device_info[index].device_val;
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

                    if(val_backup[index] != device_info[index].device_val){
                        val_backup[index] = device_info[index].device_val;
                        if(gpio_touch_led_pins[index] != -1){
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
                    return true;                    
                }            
            }
        }
        return false;
    }

    uint8_t rgb[3] = {0,0,0};
    uint32_t xxcounts = 0;
    bool nuos_set_hardware_brightness(uint32_t pin){
        uint8_t index = nuos_get_button_press_index(pin);
        brightness_control_flag = true;
        if(global_switch_state == SWITCH_PRESS_DETECTED){ 
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
                    // printf("change_cw_ww_color_flag = FALSE\n");
                    if(nuos_set_hw_level_brightness(3)){
                        set_hardware(3, false);
                        set_level(3);
                    }
                    if(xxcounts++ % 10 == 0){
                        nuos_set_color_temp_level_attribute(3);
                    }
                }else{
                    if(nuos_set_hw_color_temperature(3)){
                        set_hardware(index, false);
                        set_color_temp();
                        // if(xxcounts++ % 10 == 0){
                        //     nuos_set_color_temp_attribute(3);
                        // }
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
                            set_level(3);
                        }
                    #else
                    device_info[4].device_level = max_of_three(dmx_data[dmx_start_address] , dmx_data[dmx_start_address+1] , dmx_data[dmx_start_address+2]);
                    if(device_info[4].device_level  < MIN_DIM_LEVEL_VALUE){
                        device_info[4].device_level = MIN_DIM_LEVEL_VALUE;
                    }                    
                    #endif                    
                    set_color_temp();
                    if(xxcounts++ % 20 == 0){
                        nuos_set_level_attribute(4);
                        // rgb_t rgb = {dmx_data[dmx_start_address] , dmx_data[dmx_start_address+1] , dmx_data[dmx_start_address+2]}; // Example RGB values
                        // hsv_t hsv2 = rgb_to_hsv(rgb);  
                        // nuos_set_color_xy_attribute(4, &hsv2); 
                    }
                }
            } 

        }
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

#endif




