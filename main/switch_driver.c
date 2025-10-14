

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

extern void nuos_zb_group_send_light_level_groupcast(uint16_t group_id, uint8_t level);


/**
 * @brief:
 * This example code shows how to configure light switch with attribute as well as button switch handler.
 *
 * @note:
   Currently only support toggle switch functionality available
 *
 * @note:
 * For other possible switch functions (on/off,level up/down,step up/down). User need to implement and create them by themselves
 */

// Constants
#if defined(USE_DOUBLE_PRESS)
    #define PIN_READ_TIME_MS                    1
    #define DEBOUNCE_TIME_MS                    10
    #define DOUBLE_CLICK_THRESHOLD_MS           500
#elif defined(USE_TRIPLE_CLICK)
    #define PIN_READ_TIME_MS                    1
    #define DEBOUNCE_TIME_MS                    10
    #define DOUBLE_CLICK_THRESHOLD_MS           2000
#else
    #define PIN_READ_TIME_MS                    20
    #define DEBOUNCE_TIME_MS                    20
    #define DOUBLE_CLICK_THRESHOLD_MS           500
#endif

#define TRIPLE_CLICK_MAX_DELAY_MS               2000
#define MAX_COUNTS_FOR_TRIPLE_CLICK             6

static uint64_t brightness_count                = 0;
extern char * nuos_do_task(uint8_t index, uint8_t scene_id, uint8_t erase_data);
switch_func_pair_t button_func_pair;
volatile uint16_t total_press_in_secs           = 0;
static uint32_t last_press_time                 = 0;
static uint32_t press_duration                  = 0;
static uint32_t last_release_time               = 0;
static uint32_t double_release_time             = 0;
static bool double_click_detected               = false;

// Timer handle
static TimerHandle_t click_timer                = NULL;

bool  longpress_detected                        = false;
static int click_count                          = 0;
#define CLICK_ARRAY_SIZE                        50
gpio_num_t switch_num_pressed[CLICK_ARRAY_SIZE];
/// /////////////////////////////////////////
// Variables to track switch presses in a 5-second window
int press_count_5s[4]                           = {0, 0, 0, 0}; // Assuming four switches
static TimerHandle_t press_count_timer_handle   = NULL;
volatile bool time_5sec_started_flag            = false;
/// /////////////////////////////////////////

static QueueHandle_t gpio_evt_queue             = NULL;
/* button function pair, should be defined in switch example source file */
static switch_func_pair_t *switch_func_pair;
/* call back function pointer */
static esp_switch_callback_t func_ptr;
/* which button is pressed */
static uint8_t switch_num;
static const char *TAG                          = "ESP_ZB_SWITCH";
extern esp_err_t nuos_set_color_rgb_mode_attribute(uint8_t index, uint8_t val_mode);
static void switch_driver_gpios_intr_enabled(bool enabled);
static void esp_commissioning_detect_task(void *pvParameters);
extern uint32_t timer_commissioning_ready_counts;
extern uint32_t timer_led_blink_ready_counts;
bool bool_button_pressed                        = false;
bool bool_button_pressed_backup_ready           = false;
bool is_1212                                    = false;
bool is_2121                                    = false;
bool is_122112                                  = false;
bool is_211221                                  = false;

bool is_11221122                                = false;
bool is_22112211                                = false;

static bool toggle_status_led_long_press        = false;

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    // Critical section entry
    switch_driver_gpios_intr_enabled(false); 
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(gpio_evt_queue, (switch_func_pair_t*)arg, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken); 
}

/**
 * @brief Enable GPIO (switchs refer to) isr
 *
 * @param enabled      enable isr if true.
 */
static void switch_driver_gpios_intr_enabled(bool enabled)
{
    for (int i = 0; i < switch_num; ++i) {
        if (enabled) {
            gpio_intr_enable((switch_func_pair + i)->pin);
        } else {
            gpio_intr_disable((switch_func_pair + i)->pin);
        }
    }
}

/**
 * @brief Tasks for checking the button event and debounce the switch state
 *
 * @param arg      Unused value.
 */

#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
    void task_mode_single_click(){
        if(isSceneRemoteBindingStarted){
            if(task_sequence_num == TASK_MODE_BLINK_ALL_LEDS){
                task_sequence_num = TASK_MODE_GO_TO_ACTUAL_TASK;  
            }else if(task_sequence_num == TASK_MODE_ACTUAL_TASK_BLINK_LEDS){
                task_sequence_num = TASK_MODE_BLINK_OTHER_LEDS;
            }else if(task_sequence_num == TASK_MODE_BLINK_OTHER_LEDS){
                task_sequence_num = TASK_SEND_IDENTIFY_COMMAND;
                identify_device_complete_flag = false;
            }else if(task_sequence_num == TASK_BLINK_SELECTED_LED){
                task_sequence_num = TASK_MODE_BLINK_OTHER_LEDS;
                identify_device_complete_flag = false;
            }
        }    
    }
#endif


static void esp_zb_callback(uint8_t param) {
    printf("param:%d\n", param);

}

void button_click_handler(TimerHandle_t xTimer)
{

    int local_clicks = click_count;
    if (local_clicks > CLICK_ARRAY_SIZE) local_clicks = CLICK_ARRAY_SIZE;
    click_count = 0;
#ifdef USE_TRIPLE_CLICK
    #if (USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM ||  \
         (USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_MOTION || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_CONTACT_SWITCH ||  \
         USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_GAS_LEAK || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_LUX ||  \
         USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY))
        // No triple click action for these devices.
    #else
        printf("click_count:%d\n", local_clicks);
        if (local_clicks == MAX_COUNTS_FOR_TRIPLE_CLICK) {
            // Check if all presses were on the same button
            
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
            is_1212 = 
                (switch_num_pressed[0] == gpio_touch_btn_pins[0]) &&
                (switch_num_pressed[1] == gpio_touch_btn_pins[2]) &&
                (switch_num_pressed[2] == gpio_touch_btn_pins[0]) &&
                (switch_num_pressed[3] == gpio_touch_btn_pins[2]) &&
                (switch_num_pressed[4] == gpio_touch_btn_pins[0]) &&
                (switch_num_pressed[5] == gpio_touch_btn_pins[2]);
    
            is_2121 = 
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

                is_1212 = 
                    (switch_num_pressed[0] == gpio_touch_btn_pins[0]) &&
                    (switch_num_pressed[1] == gpio_touch_btn_pins[1]) &&
                    (switch_num_pressed[2] == gpio_touch_btn_pins[0]) &&
                    (switch_num_pressed[3] == gpio_touch_btn_pins[1]) &&
                    (switch_num_pressed[4] == gpio_touch_btn_pins[0]) &&
                    (switch_num_pressed[5] == gpio_touch_btn_pins[1]);
        
                is_2121 = 
                    (switch_num_pressed[0] == gpio_touch_btn_pins[1]) &&
                    (switch_num_pressed[1] == gpio_touch_btn_pins[0]) &&
                    (switch_num_pressed[2] == gpio_touch_btn_pins[1]) &&
                    (switch_num_pressed[3] == gpio_touch_btn_pins[0]) &&
                    (switch_num_pressed[4] == gpio_touch_btn_pins[1]) &&
                    (switch_num_pressed[5] == gpio_touch_btn_pins[0]);                
            #endif

            if (is_122112 || is_211221) {
                printf("Detected sequence: %s\n", is_122112 ? "1,2,2,1,1,2" : "2,1,1,2,2,1");
                
  
                if(ready_commisioning_flag){
                    for(int i=0; i<50; i++) {
                        esp_err_t status = esp_zb_zcl_scenes_table_clear_by_index(i);
                        if(status != ESP_OK) break;
                    }
                    setNVSStartCommissioningFlag(1);
                    setNVSCommissioningFlag(1);
                    setNVSPanicAttack(0);
                    //esp_zb_scheduler_alarm_cancel(esp_zb_callback, 0);
                   
                    if (esp_zb_bdb_dev_joined()) {
                         printf("ready_commisioning_flag: 2\n");
                        esp_zb_bdb_reset_via_local_action();
                    }
                    printf("ready_commisioning_flag: 3\n");
                    vTaskDelay(pdMS_TO_TICKS(100));
                    esp_zb_factory_reset();
                }else{
                   
                }
            } else if (is_1212 || is_2121) {
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
                }               
            }
        }else if (local_clicks == 8) {
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
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
            
            // bool same_bt_pressed = true;
            // for (int p = 0; p < local_clicks; p++) {   //switch_num_pressed
            //     if (switch_num_pressed[p] != gpio_touch_btn_pins[TOTAL_BUTTONS-1]) {
            //         same_bt_pressed = false;
            //         break;
            //     }
            // }
            if(is_11221122 || is_22112211){
                //printf("mode ok!!\n");
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
        } else if (local_clicks == 2) {
            // Double click: check if any button was pressed twice
            for (int i = 0; i < TOTAL_BUTTONS; i++) {
                for (int p = 0; p < local_clicks; p++) {
                    if (switch_num_pressed[p] == gpio_touch_btn_pins[i]) {
                        // White button pressed 2 times
                        change_cw_ww_color_flag = !change_cw_ww_color_flag;
                        break;
                    }
                }
            }
        }
    #endif
#endif // USE_TRIPLE_CLICK

#ifdef USE_DOUBLE_PRESS
    // Read the last button event from the queue (non-blocking)
    if (xQueueReceive(gpio_evt_queue, &button_func_pair, 0) == pdTRUE) {
        button_func_pair.func = SWITCH_ONOFF_TOGGLE_CONTROL;
        if (local_clicks == 1) {
            if (longpress_detected) {
                button_func_pair.keypressed = LONG_PRESS;
            } else {
                button_func_pair.keypressed = SINGLE_PRESS;
            }
            longpress_detected = false;
        } else if (local_clicks == 2) {
            if (isSceneRemoteBindingStarted) {
                task_sequence_num = TASK_MODE_EXIT;
            }
            // Default to single press, but check for double press on the same button
            button_func_pair.keypressed = SINGLE_PRESS;
            for (int i = 1; i < local_clicks; i++) {
                if (switch_num_pressed[i] == switch_num_pressed[i - 1]) {
                    button_func_pair.keypressed = DOUBLE_PRESS;
                    break;
                }
            }
        }
        if (func_ptr != NULL) {
            (*func_ptr)(&button_func_pair); // Call the callback function
        }
    }
#endif // USE_DOUBLE_PRESS
    
}

static void esp_commissioning_detect_task(void *pvParameters){
    // printf("On Task esp_commissioning_detect_task\n");
    for(int j=0; j<2; j++){
        for(int i=0; i<TOTAL_ENDPOINTS; i++){
            nuos_zb_set_hardware(i, true);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        vTaskDelay(pdMS_TO_TICKS(500));    
    }
    // printf("Deleting Task esp_commissioning_detect_task\n");
    vTaskDelete(NULL); // Delete the task after executing
}

static void create_timers_at_init(void)
{
    #if defined(USE_DOUBLE_PRESS) || defined(USE_TRIPLE_CLICK)
        if (click_timer == NULL) {
            const TickType_t ticks =
    #if (USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_SCENE_SWITCH)
                pdMS_TO_TICKS(DOUBLE_CLICK_THRESHOLD_MS);
    #else
                pdMS_TO_TICKS(TRIPLE_CLICK_MAX_DELAY_MS);
    #endif
            click_timer = xTimerCreate("ClickTimer", ticks, pdFALSE, NULL, button_click_handler);
            if (click_timer == NULL) {
                ESP_LOGE(TAG, "Failed to create click_timer");
            }
        }
    #endif

    #ifdef USE_NEIGHBOUR_SCAN_KEYPRESS
        if (press_count_timer_handle == NULL) {
            press_count_timer_handle = xTimerCreate("PressCountTimer", pdMS_TO_TICKS(4000), pdTRUE, NULL, press_count_timer_callback);
            if (press_count_timer_handle == NULL) {
                ESP_LOGE(TAG, "Failed to create press_count_timer_handle");
            }
        }
    #endif
}

void check_long_press_tasks(const uint16_t compare_time_in_secs){
    if (IdentifyTwoSwitchPressed() >= 2) {   
        if(get_button_pressed_mode() == 1){
            if(total_press_in_secs == compare_time_in_secs){
                #ifdef USE_RGB_LED
                    light_driver_set_power(false);
                #endif 
                xTaskCreate(esp_commissioning_detect_task, "Comm_detect_task", 4096, NULL, 22, NULL);
            }else if(total_press_in_secs > compare_time_in_secs){  

            }else{
                ready_commisioning_flag = true;
                #ifdef USE_RGB_LED
                    toggle_status_led_long_press = !toggle_status_led_long_press;
                    if(toggle_status_led_long_press) light_driver_set_color_RGB(0, 0, 0xff);  //red
                    light_driver_set_power(toggle_status_led_long_press);
                #endif  
            }
        }                          
    }
}

void brightness_control_tasks(uint32_t io_num){
    #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
        #if (USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI)
            if(brightness_count % BRIGHTNESS_SET_CHECKER_COUNTS == 0){
                is_long_press_brightness = true;
                nuos_set_hardware_brightness(io_num);
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
            // if(brightness_count % 200 == 0){
            //     is_long_press_brightness = true;
            //     nuos_set_hardware_brightness(io_num);
            // }   
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN) 
            #ifdef TUYA_ATTRIBUTES


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
/*********************************************************************** */
static void switch_driver_button_detected(void *arg) {
    gpio_num_t io_num = GPIO_NUM_NC;
    uint32_t reduced_bounce_time = DEBOUNCE_TIME_MS;
    bool curtain_mode_bit = false;
    static switch_state_t switch_state = SWITCH_IDLE;
    bool evt_flag = false;
    bool two_switch_pressed_flag = false;
    bool switch_keep_pressed_once_flag = true;
    TickType_t last_click_time = 0;
    
    #if defined(USE_DOUBLE_PRESS) || defined(USE_TRIPLE_CLICK)
        if (click_timer == NULL) {
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_SCENE_SWITCH)
                click_timer = xTimerCreate("Click Timer", pdMS_TO_TICKS(DOUBLE_CLICK_THRESHOLD_MS), pdFALSE, (void *)0, button_click_handler);
            #else
                click_timer = xTimerCreate("Click Timer", pdMS_TO_TICKS(TRIPLE_CLICK_MAX_DELAY_MS), pdFALSE, (void *)0, button_click_handler);
            #endif
            if (click_timer == NULL) {
                ESP_LOGE("Button", "Failed to create click timer");
                return;
            }
        }
    #endif


    #ifdef USE_NEIGHBOUR_SCAN_KEYPRESS
        static TimerHandle_t press_count_timer_handle = NULL;
        if (press_count_timer_handle == NULL) {
            press_count_timer_handle = xTimerCreate("PressCountTimer", pdMS_TO_TICKS(4000), pdTRUE, (void *)0, press_count_timer_callback);
            if (press_count_timer_handle == NULL) {
                ESP_LOGE("Timer", "Failed to create press count timer");
                return;
            }
        }
    #endif

    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &button_func_pair, portMAX_DELAY)) {
            io_num = button_func_pair.pin;
            switch_driver_gpios_intr_enabled(false);
            evt_flag = true;
            longpress_detected = false;
            total_press_in_secs = 0;
            two_switch_pressed_flag = false;
            initTwoSwitchPressedPins();
            #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
               nuos_init_hardware_dimming_up_down(io_num);
            #endif
            esp_start_timer_3();

            #ifndef USE_DOUBLE_PRESS
                button_func_pair.keypressed = SINGLE_PRESS;
            #endif

            brightness_count = 0;

            reduced_bounce_time = DEBOUNCE_TIME_MS;
            switch_keep_pressed_once_flag = true;
        }

        while (evt_flag) {
            bool value = gpio_get_level(io_num);
            uint32_t current_time = esp_timer_get_time() / 1000; // ms
            global_switch_state = switch_state;

            switch (switch_state) {
                case SWITCH_IDLE:
                    if (value == GPIO_INPUT_LEVEL_ON) {
                        last_press_time = current_time;
                        last_release_time = current_time;
                        double_release_time = current_time;
                        switch_state = SWITCH_PRESS_DETECTED;
                    }
                    break;

                case SWITCH_PRESS_DETECTED:
                    switch_state = (value == GPIO_INPUT_LEVEL_ON) ? SWITCH_PRESS_DETECTED : SWITCH_RELEASE_DETECTED;
                    if (double_press_click_enable) {
                        disable_double_press_enable_counts = 0;
                    }
                    press_duration = current_time - last_press_time;
                    if (press_duration > 1000) { // Long press
                        if (switch_state == SWITCH_PRESS_DETECTED) {

                            reduced_bounce_time = DEBOUNCE_TIME_MS;
                            if ((current_time - last_release_time) >= 2) { // 2 ms
                                last_release_time = current_time;

                                #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_SCENE_SWITCH)
                                    #ifdef USE_CUSTOM_SCENE
                                        button_func_pair.keypressed = SINGLE_PRESS; 
                                    #endif   
                                #else
                                    button_func_pair.keypressed = LONG_PRESS_INC_DEC_LEVEL;                      
                                #endif
                                brightness_control_tasks(io_num);
                                if (!two_switch_pressed_flag) {
                                    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH || \
                                        (USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_MOTION || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_CONTACT_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_GAS_LEAK || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_LUX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY))
                                        if (IdentifyTwoSwitchPressed() >= 1) {
                                            two_switch_pressed_flag = true;
                                        }
                                    #else
                                        if (IdentifyTwoSwitchPressed() >= 2) {
                                            two_switch_pressed_flag = true;
                                            nuos_set_rgb_led_commissioning_functionality();
                                            printf("2 switch pressed!!\n");
                                        }
                                    #endif
                                }

                                if (current_time - double_release_time >= 1000) {
                                    double_release_time = current_time;
                                    longpress_detected = true;
                                    total_press_in_secs++;
                                    ESP_LOGI(TAG, "total_press_in_secs_1:%d\n", total_press_in_secs);
                                    check_long_press_tasks(SETUP_LONG_PRESS_TIME_IN_SECS);
                                }
                            }
                        }
                    }
                    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RINGING_BELL_2)
                        if (switch_keep_pressed_once_flag) {
                            switch_keep_pressed_once_flag = false;
                            nuos_zb_set_scene_switch_click(io_num, 1);
                        }
                    #endif
                    break;

                case SWITCH_RELEASE_DETECTED:
                    switch_state = SWITCH_IDLE;
                    reduced_bounce_time = 10;
                    if (two_switch_pressed_flag) {
                        if(total_press_in_secs < 10 || total_press_in_secs > 18){                                            
                            ready_commisioning_flag = false;
                        }
                    }
                    /* ... inside SWITCH_RELEASE_DETECTED ... */
                    #if defined(USE_DOUBLE_PRESS) || defined(USE_TRIPLE_CLICK)
                        if (click_count < CLICK_ARRAY_SIZE) {
                            switch_num_pressed[click_count] = io_num;
                            click_count++;
                        } else {
                            // Saturated — ignore further clicks until handler runs
                            ESP_LOGW(TAG, "click_count saturated, ignoring additional clicks");
                        }

                        if (click_timer != NULL) {
                            if (xTimerIsTimerActive(click_timer) != pdFALSE) {
                                xTimerStop(click_timer, 0);
                            }
                            xTimerStart(click_timer, 0);
                        } else {
                            ESP_LOGW(TAG, "click_timer is NULL when trying to start it");
                        }
                    #endif
                    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RINGING_BELL_2)
                        vTaskDelay(pdMS_TO_TICKS(20));
                        nuos_zb_set_scene_switch_click(io_num, 0);
                    #endif
                    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN_SWITCH)
                        if(longpress_detected){
                            button_func_pair.keypressed = LONG_PRESS;
                        }
                    #endif
                    break;

                case SWITCH_LONG_PRESS_DETECTED:
                    switch_state = SWITCH_IDLE;
                    button_func_pair.keypressed = LONG_PRESS;

                    break;

                default:
                    break;
            }

            if (switch_state == SWITCH_IDLE) {
                actionOnTwoSwitchPressed(total_press_in_secs);
                #ifndef USE_DOUBLE_PRESS
                    button_func_pair.func = SWITCH_ONOFF_TOGGLE_CONTROL;
                    button_func_pair.pin = io_num;
                    if (func_ptr != NULL) {
                        (*func_ptr)(&button_func_pair);
                    }
                #endif
                switch_driver_gpios_intr_enabled(true);
                evt_flag = false;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(reduced_bounce_time));
        }
    }
}

static bool switch_driver_gpio_init(switch_func_pair_t *button_func_pair, uint8_t button_num)
{
    gpio_config_t io_conf = {};
    switch_func_pair = button_func_pair;
    switch_num = button_num;
    uint64_t pin_bit_mask = 0;

    /* set up button func pair pin mask */
    for (int i = 0; i < button_num; ++i) {
        pin_bit_mask |= (1ULL << (button_func_pair + i)->pin);
    }
    /* interrupt of falling edge */
    io_conf.intr_type = GPIO_INTR_LOW_LEVEL; //GPIO_INTR_NEGEDGE
    io_conf.pin_bit_mask = pin_bit_mask;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    /* configure GPIO with the given settings */
    gpio_config(&io_conf);

    /* create a queue to handle gpio event from isr */
    gpio_evt_queue = xQueueCreate(10, sizeof(switch_func_pair_t));
    if ( gpio_evt_queue == 0) {
        ESP_LOGE(TAG, "Queue was not created and must not be used");
        return false;
    }
    /* start gpio task */
    xTaskCreate(switch_driver_button_detected, "button_detected", TASK_STACK_SIZE_SWITCH, NULL, TASK_PRIORITY_SWITCH, NULL);
    /* install gpio isr service */
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    for (int i = 0; i < button_num; ++i) {
        gpio_isr_handler_add((button_func_pair + i)->pin, gpio_isr_handler, (void *) (button_func_pair + i));
    }
    return true;
}

bool switch_driver_init(switch_func_pair_t *button_func_pair, uint8_t button_num, esp_switch_callback_t cb)
{
    if (!switch_driver_gpio_init(button_func_pair, button_num)) {
        return false;
    }
    func_ptr = cb;
    create_timers_at_init();   // <--- create timers deterministically
    return true;
}






