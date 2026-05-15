#define LONG_PRESS_BRIGHTNESS_ENABLE
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "switch_driver.h"
#include "esp_timer.h"
#include "string.h"
#include "driver/gpio.h"
#include "app_hardware_driver.h"
#include "app_zigbee_misc.h"
#include "light_driver.h"
#include "nvs_flash.h"

#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI) 
    #ifdef USE_C3_ADAPTER_UART_HW
        #include "zigbee_2_uart.h"
    #endif
#endif

// ==================== Missing definitions (from original) ====================
#ifndef CLICK_ARRAY_SIZE
#define CLICK_ARRAY_SIZE 50
#endif

#ifndef BRIGHTNESS_SET_CHECKER_COUNTS
#define BRIGHTNESS_SET_CHECKER_COUNTS 20   // example value, adjust as needed
#endif

#ifndef COLOR_SET_CHECKER_COUNTS
#define COLOR_SET_CHECKER_COUNTS 10
#endif

#ifndef SETUP_LONG_PRESS_TIME_IN_SECS
#define SETUP_LONG_PRESS_TIME_IN_SECS 5
#endif

#ifndef MAX_COUNTS_FOR_SETUP_CLICK
#define MAX_COUNTS_FOR_SETUP_CLICK 6
#endif
// ==================== Configuration ====================
#define PIN_READ_TIME_MS                    1
#define DEBOUNCE_TIME_MS                    10
#define LONG_PRESS_THRESHOLD_MS             1000
#define DOUBLE_CLICK_TIMEOUT_MS             500
#define SETTINGS_CLICK_MAX_DELAY_MS         1500
// ==================== Global Variables (from original) ====================
static QueueHandle_t gpio_evt_queue = NULL;
static switch_func_pair_t *switch_func_pair;
static esp_switch_callback_t func_ptr;
static uint8_t switch_num;
static const char *TAG = "ESP_ZB_SWITCH";

// Button state machine variables
static uint32_t last_press_time = 0;
static uint32_t last_release_time = 0;
static uint32_t seconds_count_release_time = 0;
static bool long_press_active = false;
static bool waiting_for_double = false;
static uint32_t pending_button_pin = 0;
static TimerHandle_t single_double_timer = NULL;
static uint32_t press_duration = 0;

// Legacy triple‑click variables (keep as is)
uint32_t total_press_in_secs = 0;
static uint32_t double_click_detected = false;
static int click_count = 0;
static TimerHandle_t click_timer = NULL;
gpio_num_t switch_num_pressed[CLICK_ARRAY_SIZE];
bool longpress_detected = false;

// Other existing globals
bool bool_button_pressed = false;
bool bool_button_pressed_backup_ready = false;
bool is_121212, is_212121, is_122112, is_211221, is_11221122, is_22112211;
static bool toggle_status_led_long_press = false;
uint64_t brightness_count = 0;
int press_count_5s[4] = {0};
TimerHandle_t press_count_timer_handle = NULL;
volatile bool time_5sec_started_flag = false;

// ==================== Original helper functions (restored) ====================
void brightness_control_tasks(uint32_t io_num) {
    #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
        // Original implementation from user code
        #if (USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI)
            if(brightness_count % BRIGHTNESS_SET_CHECKER_COUNTS == 0){
                is_long_press_brightness = true;
                nuos_set_hardware_brightness(io_num);
            }
        #elif (USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI)
            if(scene_group_switch_info.control_type == 0 || (scene_group_switch_info.control_type == 1)){
                if(change_cw_ww_color_flag){
                    if(brightness_count % COLOR_SET_CHECKER_COUNTS == 0){
                        is_long_press_brightness = true;
                        nuos_set_hardware_brightness(io_num);
                    } 
                }else{
                    if(brightness_count % BRIGHTNESS_SET_CHECKER_COUNTS == 0){
                        is_long_press_brightness = true;
                        nuos_set_hardware_brightness(io_num);
                    }                                                
                }  
            } 
        #elif (USE_NUOS_ZB_DEVICE_TYPE == DEVICE_DALI_DIRECT_SWITCH)
            uint8_t btn_index = nuos_get_button_press_index(io_num);
            //printf("===btn_index:%d\n", btn_index);
            if(btn_index < 2){
                if(brightness_count % BRIGHTNESS_SET_CHECKER_COUNTS == 0){
                    is_long_press_brightness = true;
                    nuos_set_hardware_brightness(io_num);
                } 
            }else{
                if(brightness_count % COLOR_SET_CHECKER_COUNTS == 0){
                    is_long_press_brightness = true;
                    nuos_set_hardware_brightness(io_num);
                }                                                
            }  
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
            if(io_num == gpio_touch_btn_pins[0]){
                if(brightness_count % BRIGHTNESS_SET_CHECKER_COUNTS == 0){
                    is_long_press_brightness = true;
                    nuos_set_hardware_brightness(io_num);
                }                                            
            }else{
                if(brightness_count % COLOR_SET_CHECKER_COUNTS == 0){
                    is_long_press_brightness = true;
                    nuos_set_hardware_brightness(io_num);
                }
            }
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_ANALOG_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_PHASE_CUT_DIMMABLE_LIGHT)
                if(brightness_count % BRIGHTNESS_SET_CHECKER_COUNTS == 0){
                    is_long_press_brightness = true;
                    nuos_set_hardware_brightness(io_num);
                }
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_GROUP_SWITCH)
            // (optional brightness control)
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN)
            #ifdef TUYA_ATTRIBUTES
                // curtain specific
            #endif
        #else
            if(io_num == gpio_touch_btn_pins[3]){
                if(selected_color_mode != 0){
                    selected_color_mode = 0;
                    nuos_set_color_rgb_mode_attribute(0, selected_color_mode);
                    store_color_mode_value(selected_color_mode);
                }
                if(change_cw_ww_color_flag){
                    if(brightness_count % COLOR_SET_CHECKER_COUNTS == 0){
                        is_long_press_brightness = true;
                        nuos_set_hardware_brightness(io_num);
                    } 
                }else{
                    if(brightness_count % BRIGHTNESS_SET_CHECKER_COUNTS == 0){
                        is_long_press_brightness = true;
                        nuos_set_hardware_brightness(io_num);
                    }                                                
                }
            }else{
                if(brightness_count % BRIGHTNESS_SET_CHECKER_COUNTS == 0){
                    is_long_press_brightness = true;
                    if(selected_color_mode == 0){
                        selected_color_mode = 1;
                        nuos_set_color_rgb_mode_attribute(0, selected_color_mode);
                        store_color_mode_value(selected_color_mode);
                    }                                                
                    nuos_set_hardware_brightness(io_num);
                }
            }                                        
        #endif
        brightness_count++;
    #endif
}

void check_long_press_tasks(uint32_t sw_pressed_cnts, const uint16_t compare_time_in_secs) {
    
    if (sw_pressed_cnts == 2) {   
        if(get_button_pressed_mode() == 1){
            if(total_press_in_secs == compare_time_in_secs){
                #ifdef USE_RGB_LED
                    light_driver_set_power(false);
                #endif 
            }else if(total_press_in_secs > compare_time_in_secs){  
                // do nothing
            }else{
                #ifdef USE_RGB_LED
                    toggle_status_led_long_press = !toggle_status_led_long_press;
                    if(toggle_status_led_long_press) light_driver_set_color_RGB(0, 0, 0xff);
                    light_driver_set_power(toggle_status_led_long_press);
                #endif  
            }
        } else if(get_button_pressed_mode() == 2){
            if(total_press_in_secs == compare_time_in_secs){
                #ifdef USE_RGB_LED
                    light_driver_set_power(false);
                #endif 
            }else if(total_press_in_secs > compare_time_in_secs){  
                // do nothing
            }else{
                #ifdef USE_RGB_LED
                    toggle_status_led_long_press = !toggle_status_led_long_press;
                    if(toggle_status_led_long_press) light_driver_set_color_RGB(0x00, 0xff, 0x00);
                    light_driver_set_power(toggle_status_led_long_press);
                #endif  
            }   
        }                             
    }
}

void press_count_timer_callback(TimerHandle_t xTimer) {
    // Original implementation (example, adapt as needed)
    for (int i = 0; i < 4; i++) {
        press_count_5s[i] = 0;
    }
    time_5sec_started_flag = false;
}

// ==================== ISR Handler ====================
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    switch_driver_gpios_intr_enabled(false);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(gpio_evt_queue, (switch_func_pair_t*)arg, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// ==================== Single / Double Click Timer Callback ====================
static void single_click_timer_cb(TimerHandle_t xTimer)
{
    xTimerStop(single_double_timer, 0);
    //xTimerStop(click_timer, 0);
    if (waiting_for_double) {
        waiting_for_double = false;
        switch_func_pair_t btn_pair;
        btn_pair.pin = (gpio_num_t)pending_button_pin;
        btn_pair.func = SWITCH_ONOFF_TOGGLE_CONTROL;
        btn_pair.keypressed = SINGLE_PRESS;
        
        switch_driver_gpios_intr_enabled(false);
        if (func_ptr != NULL) {
            func_ptr(&btn_pair);
        }   
    }
    switch_driver_gpios_intr_enabled(true);
}

// ==================== Triple Click Handler (stub – replace with your original) ====================
void button_click_handler(TimerHandle_t xTimer)
{
    int local_clicks = click_count;
    if (local_clicks > CLICK_ARRAY_SIZE) local_clicks = CLICK_ARRAY_SIZE;
    click_count = 0;
    //printf("===local_clicks:%d\n", local_clicks);
    switch_driver_gpios_intr_enabled(true);

#ifdef USE_TRIPLE_CLICK
    #if (USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM ||  \
         (USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_MOTION || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_CONTACT_SWITCH ||  \
         USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_GAS_LEAK || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_LUX ||  \
         USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY))
        // No triple click action for these devices.
    #else
        //printf("click_count:%d\n", local_clicks);
        if (local_clicks == MAX_COUNTS_FOR_SETUP_CLICK) {
            // Check if all presses were on the same button 
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
                #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)

                    #ifdef USE_TWO_SWITCH_MODE
                    is_121212 = 
                        (switch_num_pressed[0] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[1] == gpio_touch_btn_pins[1]) &&
                        (switch_num_pressed[2] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[3] == gpio_touch_btn_pins[1]) &&
                        (switch_num_pressed[4] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[5] == gpio_touch_btn_pins[1]);
            
                    is_212121 = 
                        (switch_num_pressed[0] == gpio_touch_btn_pins[1]) &&
                        (switch_num_pressed[1] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[2] == gpio_touch_btn_pins[1]) &&
                        (switch_num_pressed[3] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[4] == gpio_touch_btn_pins[1]) &&
                        (switch_num_pressed[5] == gpio_touch_btn_pins[0]);

                    is_122112 = 
                        (switch_num_pressed[0] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[1] == gpio_touch_btn_pins[1]) &&
                        (switch_num_pressed[2] == gpio_touch_btn_pins[1]) &&
                        (switch_num_pressed[3] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[4] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[5] == gpio_touch_btn_pins[1]);
            
                    is_211221 = 
                        (switch_num_pressed[0] == gpio_touch_btn_pins[1]) &&
                        (switch_num_pressed[1] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[2] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[3] == gpio_touch_btn_pins[1]) &&
                        (switch_num_pressed[4] == gpio_touch_btn_pins[1]) &&
                        (switch_num_pressed[5] == gpio_touch_btn_pins[0]);

                    #else

                    is_121212 = 
                        (switch_num_pressed[0] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[1] == gpio_touch_btn_pins[2]) &&
                        (switch_num_pressed[2] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[3] == gpio_touch_btn_pins[2]) &&
                        (switch_num_pressed[4] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[5] == gpio_touch_btn_pins[2]);
            
                    is_212121 = 
                        (switch_num_pressed[0] == gpio_touch_btn_pins[2]) &&
                        (switch_num_pressed[1] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[2] == gpio_touch_btn_pins[2]) &&
                        (switch_num_pressed[3] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[4] == gpio_touch_btn_pins[2]) &&
                        (switch_num_pressed[5] == gpio_touch_btn_pins[0]);

                    is_122112 = 
                        (switch_num_pressed[0] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[1] == gpio_touch_btn_pins[2]) &&
                        (switch_num_pressed[2] == gpio_touch_btn_pins[2]) &&
                        (switch_num_pressed[3] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[4] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[5] == gpio_touch_btn_pins[2]);
            
                    is_211221 = 
                        (switch_num_pressed[0] == gpio_touch_btn_pins[2]) &&
                        (switch_num_pressed[1] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[2] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[3] == gpio_touch_btn_pins[2]) &&
                        (switch_num_pressed[4] == gpio_touch_btn_pins[2]) &&
                        (switch_num_pressed[5] == gpio_touch_btn_pins[0]);
                    #endif

                #else
                
                    is_121212 = 
                        (switch_num_pressed[0] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[1] == gpio_touch_btn_pins[2]) &&
                        (switch_num_pressed[2] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[3] == gpio_touch_btn_pins[2]) &&
                        (switch_num_pressed[4] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[5] == gpio_touch_btn_pins[2]);
            
                    is_212121 = 
                        (switch_num_pressed[0] == gpio_touch_btn_pins[2]) &&
                        (switch_num_pressed[1] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[2] == gpio_touch_btn_pins[2]) &&
                        (switch_num_pressed[3] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[4] == gpio_touch_btn_pins[2]) &&
                        (switch_num_pressed[5] == gpio_touch_btn_pins[0]);

                    is_122112 = 
                        (switch_num_pressed[0] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[1] == gpio_touch_btn_pins[2]) &&
                        (switch_num_pressed[2] == gpio_touch_btn_pins[2]) &&
                        (switch_num_pressed[3] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[4] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[5] == gpio_touch_btn_pins[2]);
            
                    is_211221 = 
                        (switch_num_pressed[0] == gpio_touch_btn_pins[2]) &&
                        (switch_num_pressed[1] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[2] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[3] == gpio_touch_btn_pins[2]) &&
                        (switch_num_pressed[4] == gpio_touch_btn_pins[2]) &&
                        (switch_num_pressed[5] == gpio_touch_btn_pins[0]);   
                #endif                 
            #else

                #ifdef USE_COLOR_CONTROL
                    is_122112 = 
                        (switch_num_pressed[0] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[1] == gpio_touch_btn_pins[2]) &&
                        (switch_num_pressed[2] == gpio_touch_btn_pins[2]) &&
                        (switch_num_pressed[3] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[4] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[5] == gpio_touch_btn_pins[2]);
            
                    is_211221 = 
                        (switch_num_pressed[0] == gpio_touch_btn_pins[2]) &&
                        (switch_num_pressed[1] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[2] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[3] == gpio_touch_btn_pins[2]) &&
                        (switch_num_pressed[4] == gpio_touch_btn_pins[2]) &&
                        (switch_num_pressed[5] == gpio_touch_btn_pins[0]);

                    is_121212 = 
                        (switch_num_pressed[0] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[1] == gpio_touch_btn_pins[2]) &&
                        (switch_num_pressed[2] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[3] == gpio_touch_btn_pins[2]) &&
                        (switch_num_pressed[4] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[5] == gpio_touch_btn_pins[2]);
            
                    is_212121 = 
                        (switch_num_pressed[0] == gpio_touch_btn_pins[2]) &&
                        (switch_num_pressed[1] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[2] == gpio_touch_btn_pins[2]) &&
                        (switch_num_pressed[3] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[4] == gpio_touch_btn_pins[2]) &&
                        (switch_num_pressed[5] == gpio_touch_btn_pins[0]);  

                #else

                    is_122112 = 
                        (switch_num_pressed[0] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[1] == gpio_touch_btn_pins[1]) &&
                        (switch_num_pressed[2] == gpio_touch_btn_pins[1]) &&
                        (switch_num_pressed[3] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[4] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[5] == gpio_touch_btn_pins[1]);
            
                    is_211221 = 
                        (switch_num_pressed[0] == gpio_touch_btn_pins[1]) &&
                        (switch_num_pressed[1] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[2] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[3] == gpio_touch_btn_pins[1]) &&
                        (switch_num_pressed[4] == gpio_touch_btn_pins[1]) &&
                        (switch_num_pressed[5] == gpio_touch_btn_pins[0]);



                    is_121212 = 
                        (switch_num_pressed[0] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[1] == gpio_touch_btn_pins[1]) &&
                        (switch_num_pressed[2] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[3] == gpio_touch_btn_pins[1]) &&
                        (switch_num_pressed[4] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[5] == gpio_touch_btn_pins[1]);
            
                    is_212121 = 
                        (switch_num_pressed[0] == gpio_touch_btn_pins[1]) &&
                        (switch_num_pressed[1] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[2] == gpio_touch_btn_pins[1]) &&
                        (switch_num_pressed[3] == gpio_touch_btn_pins[0]) &&
                        (switch_num_pressed[4] == gpio_touch_btn_pins[1]) &&
                        (switch_num_pressed[5] == gpio_touch_btn_pins[0]);  
                #endif         
            #endif

            if (is_122112 || is_211221) {
                printf("Ready for Commissioning!! %d\n", ready_commisioning_flag);
                if(ready_commisioning_flag){
                    for(int i=0; i<50; i++) {
                        esp_err_t status = esp_zb_zcl_scenes_table_clear_by_index(i);
                        if(status != ESP_OK) break;
                    }
                    setNVSStartCommissioningFlag(1);
                    setNVSCommissioningFlag(1);
                    setNVSPanicAttack(0);
                    if (esp_zb_bdb_dev_joined()) {
                        esp_zb_bdb_reset_via_local_action();
                    }
                    vTaskDelay(pdMS_TO_TICKS(100));
                    esp_zb_factory_reset();
                }
            } else if (is_121212 || is_212121) {
                printf("Ready for WiFi Webserver to be Started!!\n");
                if(!ready_commisioning_flag){
                    #if (USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
                        uint8_t button_index = nuos_get_button_press_index(switch_num_pressed[0]);
                        nuos_do_task(button_index, button_index + 1, 1);
                    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN)
                        #ifdef ENABLE_CURTAIN_TIMER_CONTROL
                        wifi_webserver_active_flag = !wifi_webserver_active_flag;
                        setNVSWebServerEnableFlag(wifi_webserver_active_flag); 
                        esp_restart();	
                        #endif
                    #else    
                    #endif 
                } else {

                    #ifdef USE_WIFI_WEBSERVER
                        wifi_webserver_active_flag = true;  
                        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI) 
                            #ifdef USE_C3_ADAPTER_UART_HW
                                const char* content = wifi_webserver_active_flag ? "{\"time\":10, \"mode\":1}" : "{\"time\":10, \"mode\":0}";
                                send_serial(content);
                            #else
                                setNVSCommissioningFlag(0);
                                setNVSWebServerEnableFlag(wifi_webserver_active_flag);                    
                                esp_restart();	                            
                            #endif
                        #else
                            setNVSCommissioningFlag(0);
                            setNVSWebServerEnableFlag(wifi_webserver_active_flag);                    
                            esp_restart();	                        
                        #endif		
                    #endif
                   
                }             
            }else{
                printf("No Key Combination found!!\n");
            }
        }else if (local_clicks == 8) {
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
            
                #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
                    #ifndef USE_TWO_SWITCH_MODE
                is_11221122 = 
                    (switch_num_pressed[0] == gpio_touch_btn_pins[0]) &&
                    (switch_num_pressed[1] == gpio_touch_btn_pins[0]) &&
                    (switch_num_pressed[2] == gpio_touch_btn_pins[2]) &&
                    (switch_num_pressed[3] == gpio_touch_btn_pins[2]) &&
                    (switch_num_pressed[4] == gpio_touch_btn_pins[0]) &&
                    (switch_num_pressed[5] == gpio_touch_btn_pins[0]) &&
                    (switch_num_pressed[6] == gpio_touch_btn_pins[2]) &&
                    (switch_num_pressed[7] == gpio_touch_btn_pins[2]);    
                is_22112211 = 
                    (switch_num_pressed[0] == gpio_touch_btn_pins[2]) &&
                    (switch_num_pressed[1] == gpio_touch_btn_pins[2]) &&
                    (switch_num_pressed[2] == gpio_touch_btn_pins[0]) &&
                    (switch_num_pressed[3] == gpio_touch_btn_pins[0]) &&
                    (switch_num_pressed[4] == gpio_touch_btn_pins[2]) &&
                    (switch_num_pressed[5] == gpio_touch_btn_pins[2]) &&
                    (switch_num_pressed[6] == gpio_touch_btn_pins[0]) &&
                    (switch_num_pressed[7] == gpio_touch_btn_pins[0]);    
                    #else
                is_11221122 = 
                    (switch_num_pressed[0] == gpio_touch_btn_pins[0]) &&
                    (switch_num_pressed[1] == gpio_touch_btn_pins[0]) &&
                    (switch_num_pressed[2] == gpio_touch_btn_pins[1]) &&
                    (switch_num_pressed[3] == gpio_touch_btn_pins[1]) &&
                    (switch_num_pressed[4] == gpio_touch_btn_pins[0]) &&
                    (switch_num_pressed[5] == gpio_touch_btn_pins[0]) &&
                    (switch_num_pressed[6] == gpio_touch_btn_pins[1]) &&
                    (switch_num_pressed[7] == gpio_touch_btn_pins[1]);    
                is_22112211 = 
                    (switch_num_pressed[0] == gpio_touch_btn_pins[1]) &&
                    (switch_num_pressed[1] == gpio_touch_btn_pins[1]) &&
                    (switch_num_pressed[2] == gpio_touch_btn_pins[0]) &&
                    (switch_num_pressed[3] == gpio_touch_btn_pins[0]) &&
                    (switch_num_pressed[4] == gpio_touch_btn_pins[1]) &&
                    (switch_num_pressed[5] == gpio_touch_btn_pins[1]) &&
                    (switch_num_pressed[6] == gpio_touch_btn_pins[0]) &&
                    (switch_num_pressed[7] == gpio_touch_btn_pins[0]); 
                    #endif
                #else
                is_11221122 = 
                    (switch_num_pressed[0] == gpio_touch_btn_pins[0]) &&
                    (switch_num_pressed[1] == gpio_touch_btn_pins[0]) &&
                    (switch_num_pressed[2] == gpio_touch_btn_pins[2]) &&
                    (switch_num_pressed[3] == gpio_touch_btn_pins[2]) &&
                    (switch_num_pressed[4] == gpio_touch_btn_pins[0]) &&
                    (switch_num_pressed[5] == gpio_touch_btn_pins[0]) &&
                    (switch_num_pressed[6] == gpio_touch_btn_pins[2]) &&
                    (switch_num_pressed[7] == gpio_touch_btn_pins[2]);    
                is_22112211 = 
                    (switch_num_pressed[0] == gpio_touch_btn_pins[2]) &&
                    (switch_num_pressed[1] == gpio_touch_btn_pins[2]) &&
                    (switch_num_pressed[2] == gpio_touch_btn_pins[0]) &&
                    (switch_num_pressed[3] == gpio_touch_btn_pins[0]) &&
                    (switch_num_pressed[4] == gpio_touch_btn_pins[2]) &&
                    (switch_num_pressed[5] == gpio_touch_btn_pins[2]) &&
                    (switch_num_pressed[6] == gpio_touch_btn_pins[0]) &&
                    (switch_num_pressed[7] == gpio_touch_btn_pins[0]);    
                
                #endif
            #else
            is_11221122 = 
                (switch_num_pressed[0] == gpio_touch_btn_pins[0]) &&
                (switch_num_pressed[1] == gpio_touch_btn_pins[0]) &&
                (switch_num_pressed[2] == gpio_touch_btn_pins[1]) &&
                (switch_num_pressed[3] == gpio_touch_btn_pins[1]) &&
                (switch_num_pressed[4] == gpio_touch_btn_pins[0]) &&
                (switch_num_pressed[5] == gpio_touch_btn_pins[0]) &&
                (switch_num_pressed[6] == gpio_touch_btn_pins[1]) &&
                (switch_num_pressed[7] == gpio_touch_btn_pins[1]);    
            is_22112211 = 
                (switch_num_pressed[0] == gpio_touch_btn_pins[1]) &&
                (switch_num_pressed[1] == gpio_touch_btn_pins[1]) &&
                (switch_num_pressed[2] == gpio_touch_btn_pins[0]) &&
                (switch_num_pressed[3] == gpio_touch_btn_pins[0]) &&
                (switch_num_pressed[4] == gpio_touch_btn_pins[1]) &&
                (switch_num_pressed[5] == gpio_touch_btn_pins[1]) &&
                (switch_num_pressed[6] == gpio_touch_btn_pins[0]) &&
                (switch_num_pressed[7] == gpio_touch_btn_pins[0]);
            #endif

            if(is_11221122 || is_22112211){
                touchLedsOffAfter1MinuteEnable = !touchLedsOffAfter1MinuteEnable;
                setNVSAllLedsOff(touchLedsOffAfter1MinuteEnable);
                #if(USE_NUOS_ZB_DEVICE_TYPE != DEVICE_GROUP_DALI)
                    bool t_state = false;
                    for (int j = 0; j < 5; j++) {
                        t_state = !t_state;
                        for (int i = 0; i < TOTAL_LEDS; i++){                         
                            nuos_on_off_led(i, t_state);
                        }
                        vTaskDelay(pdMS_TO_TICKS(100));
                    }
                    for (int i = 0; i < TOTAL_ENDPOINTS; i++) {
                        nuos_zb_set_hardware(i, false); 
                        vTaskDelay(pdMS_TO_TICKS(50));
                    } 
                #endif  
            }
        } else if (local_clicks == 5) {
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI)
            // Double click: check if any button was pressed twice
            for (int i = 0; i < TOTAL_BUTTONS; i++) {
                for (int p = 0; p < local_clicks; p++) {
                    if (switch_num_pressed[p] == gpio_touch_btn_pins[i]) {
                        // White button pressed 2 times
                        change_cw_ww_color_flag = !change_cw_ww_color_flag;
                        //printf("double_click_pressed!!\n");
                        break;
                    }
                }
            }
            #endif
        }
    #endif
#endif // USE_TRIPLE_CLICK
    
}

// ==================== Main Button Detection Task ====================
static void switch_driver_button_detected(void *arg)
{
    uint32_t io_num = 0;
    static bool start_comm_flag = false;
    uint32_t reduced_bounce_time = DEBOUNCE_TIME_MS;
    static switch_state_t switch_state = SWITCH_IDLE;
    bool evt_flag = false;
    uint32_t switch_pressed_cnts = 0;
    bool two_switch_pressed_flag = false;

    for (;;) {
        switch_func_pair_t button_func_pair;
        if (xQueueReceive(gpio_evt_queue, &button_func_pair, portMAX_DELAY)) {
            switch_driver_gpios_intr_enabled(false);
            recheckTimer();
            io_num = (uint32_t)button_func_pair.pin;
            evt_flag = true;
            longpress_detected = false;
            total_press_in_secs = 0;
            two_switch_pressed_flag = false;
            initTwoSwitchPressedPins();
#ifdef LONG_PRESS_BRIGHTNESS_ENABLE
            nuos_init_hardware_dimming_up_down(io_num);
#endif
            brightness_count = 0;
            switch_pressed_cnts = 0;
            reduced_bounce_time = DEBOUNCE_TIME_MS;

            xTimerStop(single_double_timer, 0);
            total_press_in_secs = 0;
        }

        while (evt_flag) {
            bool value = gpio_get_level((gpio_num_t)io_num);
            uint32_t current_time = esp_timer_get_time() / 1000;
            global_switch_state = switch_state;

            switch (switch_state) {
                case SWITCH_IDLE:
                    if (value == GPIO_INPUT_LEVEL_ON) {
                        last_press_time = current_time;
                        last_release_time = current_time;
                        seconds_count_release_time = current_time;
                        switch_state = SWITCH_PRESS_DETECTED;
                        long_press_active = false;
                    }
                    break;

                case SWITCH_PRESS_DETECTED:
                    if (value != GPIO_INPUT_LEVEL_ON) {
                        switch_state = SWITCH_RELEASE_DETECTED;
                        last_release_time = current_time;
                        seconds_count_release_time = current_time;
                        //total_press_in_secs = 0;
                    } else {
                        press_duration = current_time - last_press_time;
                        if (press_duration >= LONG_PRESS_THRESHOLD_MS) {
                            long_press_active = true;
                            longpress_detected = true;
                            button_func_pair.keypressed = LONG_PRESS_INC_DEC_LEVEL;
                            reduced_bounce_time = DEBOUNCE_TIME_MS;

                            if ((current_time - last_release_time) >= 2) {
                                last_release_time = current_time;

                                #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_SCENE_SWITCH)
                                    #ifdef USE_CUSTOM_SCENE
                                        button_func_pair.keypressed = SINGLE_PRESS;
                                    #endif
                                #endif
                                if (wifi_webserver_active_flag && start_commissioning) {
                                    start_comm_flag = true;
                                    start_commissioning = false;
                                }
                                
                                switch_pressed_cnts = IdentifyTwoSwitchPressed();
                                //printf("switch_pressed_cnts:%ld\n", switch_pressed_cnts);
                                if (!two_switch_pressed_flag) {
#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH || \
    (USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_MOTION || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_CONTACT_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_GAS_LEAK || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_LUX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY))
                                    if (switch_pressed_cnts >= 1) two_switch_pressed_flag = true;
#else
                                    if (switch_pressed_cnts >= 2) {
                                        two_switch_pressed_flag = true;
                                        nuos_set_rgb_led_commissioning_functionality();
                                    }
#endif
                                }
                                if (current_time - seconds_count_release_time >= 1000) {
                                    seconds_count_release_time = current_time;
                                    total_press_in_secs++;
                                    longpress_detected = true;
                                    //printf("switch_pressed_cnts:%ld\n", switch_pressed_cnts);
                                    check_long_press_tasks(switch_pressed_cnts, SETUP_LONG_PRESS_TIME_IN_SECS);
                                }

                                brightness_control_tasks(io_num);
                            }
                        }
                    }
                    break;

                case SWITCH_RELEASE_DETECTED:

                    switch_state = SWITCH_IDLE;
                    press_duration = current_time - last_press_time;
                    long_press_active = false;   // add this
                    // waiting_for_double = false;  // add this

                    if (wifi_webserver_active_flag) start_commissioning = start_comm_flag;
                    reduced_bounce_time = 10;

                    if (two_switch_pressed_flag && switch_pressed_cnts >= 2) {
                        if (total_press_in_secs < 10 || total_press_in_secs > 20)
                            ready_commisioning_flag = false;
                        // else 
                        //    ready_commisioning_flag = true; 
                    } 
                    // ---------- Single / Double Click Detection ----------
                    if (!long_press_active && press_duration < LONG_PRESS_THRESHOLD_MS) {
                        if(io_num == gpio_touch_btn_pins[0] || io_num == gpio_touch_btn_pins[1]){
                            if (waiting_for_double) {
                                xTimerStop(single_double_timer, 0);
                                waiting_for_double = false;

                                button_func_pair.pin = (gpio_num_t)io_num;
                                button_func_pair.func = SWITCH_ONOFF_TOGGLE_CONTROL;
                                #ifdef USE_COLOR_CONTROL
                                
                                    button_func_pair.keypressed = DOUBLE_PRESS;
                                    if (func_ptr != NULL) func_ptr(&button_func_pair);
                                    switch_driver_gpios_intr_enabled(true);
                            
                                #endif
                            } else {
                                pending_button_pin = io_num;
                                waiting_for_double = true;
                                xTimerReset(single_double_timer, 0);
                            }
                        }else{
                            xTimerStop(single_double_timer, 0);
                            waiting_for_double = false;

                            button_func_pair.pin = (gpio_num_t)io_num;
                            button_func_pair.func = SWITCH_ONOFF_TOGGLE_CONTROL;

                            button_func_pair.keypressed = SINGLE_PRESS;
                            if (func_ptr != NULL) func_ptr(&button_func_pair);
                            switch_driver_gpios_intr_enabled(true);
                        }
                    } else if (long_press_active && waiting_for_double) {
                        xTimerStop(single_double_timer, 0);
                        waiting_for_double = false;
                    }

                    // Triple-click recording (if needed)
#if defined(USE_TRIPLE_CLICK)
                    if (click_count < CLICK_ARRAY_SIZE && !long_press_active) {
                        switch_num_pressed[click_count] = (gpio_num_t)io_num;
                        click_count++;
                    }
                    if (click_timer) {
                        if (xTimerIsTimerActive(click_timer)) xTimerStop(click_timer, 0);
                        xTimerStart(click_timer, 0);
                    }
#endif

                    // Device-specific actions
#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RINGING_BELL_2)
                    vTaskDelay(pdMS_TO_TICKS(20));
                    nuos_zb_set_scene_switch_click(io_num, 0);
#endif
                    break;
                case SWITCH_LONG_PRESS_DETECTED:
                    switch_state = SWITCH_IDLE;
                    button_func_pair.keypressed = LONG_PRESS;
                    break;
                default:
                    switch_state = SWITCH_IDLE;
                    switch_driver_gpios_intr_enabled(true);
                    evt_flag = false;
                    break;
            }

            if (switch_state == SWITCH_IDLE) {
                actionOnTwoSwitchPressed(total_press_in_secs);
                evt_flag = false;
                //if(!waiting_for_double)
                switch_driver_gpios_intr_enabled(true);   // <-- ADD THIS LINE
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(reduced_bounce_time));
        }
    }
}

// ==================== Timer Initialisation ====================
static void create_timers_at_init(void)
{
    if (single_double_timer == NULL) {
        single_double_timer = xTimerCreate("SglDblTimer",
                                           pdMS_TO_TICKS(DOUBLE_CLICK_TIMEOUT_MS),
                                           pdFALSE,
                                           NULL,
                                           single_click_timer_cb);
        if (single_double_timer == NULL) ESP_LOGE(TAG, "Failed to create single/double timer");
    }

#if defined(USE_TRIPLE_CLICK)
    if (click_timer == NULL) {
        click_timer = xTimerCreate("ClickTimer",
                                   pdMS_TO_TICKS(SETTINGS_CLICK_MAX_DELAY_MS),
                                   pdFALSE,
                                   NULL,
                                   button_click_handler);
        if (click_timer == NULL) ESP_LOGE(TAG, "Failed to create triple-click timer");
    }
#endif

#ifdef USE_NEIGHBOUR_SCAN_KEYPRESS
    if (press_count_timer_handle == NULL) {
        press_count_timer_handle = xTimerCreate("PressCountTimer",
                                                pdMS_TO_TICKS(4000),
                                                pdTRUE,
                                                NULL,
                                                press_count_timer_callback);
        if (press_count_timer_handle == NULL) ESP_LOGE(TAG, "Failed to create press count timer");
    }
#endif
}

// ==================== GPIO Initialisation ====================
static bool switch_driver_gpio_init(switch_func_pair_t *button_func_pair, uint8_t button_num)
{
    gpio_config_t io_conf = {};
    switch_func_pair = button_func_pair;
    switch_num = button_num;
    uint64_t pin_bit_mask = 0;

    for (int i = 0; i < button_num; ++i) {
        pin_bit_mask |= (1ULL << (button_func_pair + i)->pin);
    }
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = pin_bit_mask;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    gpio_evt_queue = xQueueCreate(20, sizeof(switch_func_pair_t));
    if (gpio_evt_queue == 0) {
        ESP_LOGE(TAG, "Queue creation failed");
        return false;
    }

    xTaskCreate(switch_driver_button_detected, "button_detected",
                TASK_STACK_SIZE_SWITCH, NULL, TASK_PRIORITY_SWITCH, NULL);

    if (!isr_service_installed) {
        ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT));
        isr_service_installed = true;
    }

    for (int i = 0; i < button_num; ++i) {
        gpio_isr_handler_add((button_func_pair + i)->pin, gpio_isr_handler, (void *)(button_func_pair + i));
    }

    return true;
}

// ==================== Public API ====================
void switch_driver_gpios_intr_enabled(bool enabled)
{
    static int cnt = 0;
    //ESP_LOGI(TAG, "intr_enabled(%d) called from %p, cnt=%d", enabled, __builtin_return_address(0), ++cnt);
    for (int i = 0; i < switch_num; ++i) {
        if (enabled)
            gpio_intr_enable((switch_func_pair + i)->pin);
        else
            gpio_intr_disable((switch_func_pair + i)->pin);
    }
}

bool switch_driver_init(switch_func_pair_t *button_func_pair, uint8_t button_num, esp_switch_callback_t cb)
{
    if (!switch_driver_gpio_init(button_func_pair, button_num)) return false;
    func_ptr = cb;
    create_timers_at_init();
    return true;
}