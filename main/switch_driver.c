#include "app_config.h"



#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_DALI_DIRECT_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"

#include "app_hardware_driver.h"
#include "app_zigbee_misc.h"
#include "light_driver.h"
#include "app_nvs_store_info.h"

#define TAG                          "SWITCH_DRV"

#define MAX_BUTTONS                  4
#define GPIO_EVT_QUEUE_LEN           32
#define TASK_STACK_SIZE_SWITCH       4096
#define TASK_PRIORITY_SWITCH         10

#define DEBOUNCE_US                  30000
#define LONG_PRESS_US                700000
#define LONG_HOLD_REPEAT_US          70000//150000
#define MULTI_CLICK_GAP_US           500000 //250000
#define COMBO_LONG_PRESS_US          800000
#define COMBO_HOLD_REPEAT_US         1000000

#define SETTING_MODE_MIN_US          10000000LL   // 10 seconds
#define SETTING_MODE_MAX_US          20000000LL   // 20 seconds
#define READY_COMMISSIONING_MS       30000        // 30 seconds

#define PATTERN_LEN                  6
#define PATTERN_GAP_TIMEOUT_US       2000000

typedef enum {
    SWITCH_EVENT_SINGLE_CLICK = 1,
    SWITCH_EVENT_DOUBLE_CLICK,
    SWITCH_EVENT_MULTI_CLICK,
    SWITCH_EVENT_LONG_PRESS,
    SWITCH_EVENT_LONG_HOLD_REPEAT,
    SWITCH_EVENT_COMBO_LONG_PRESS
} switch_event_t;

typedef struct {
    uint8_t button_id;
    gpio_num_t pin;
    switch_event_t event;
    uint8_t click_count;
    uint32_t combo_mask;
    int64_t timestamp_us;
} switch_event_info_t;

typedef struct {
    uint8_t button_id;
    gpio_num_t pin;

    bool stable_level;
    bool pressed;
    bool long_reported;

    bool setting_mode_armed;
    bool over_20s_blocked;

    int64_t last_edge_us;
    int64_t press_start_us;
    int64_t release_time_us;
    int64_t last_hold_log_us;

    uint8_t click_count;
} button_state_t;

typedef struct {
    uint8_t button_id;
    gpio_num_t pin;
    int level;
    int64_t isr_time_us;
} gpio_evt_t;

static QueueHandle_t gpio_evt_queue = NULL;
static switch_func_pair_t *switch_func_pair = NULL;
static uint8_t switch_num = 0;
extern bool isr_service_installed;
static esp_switch_callback_t func_ptr = NULL;

static button_state_t g_btn[MAX_BUTTONS];
static bool g_combo_long_reported = false;

static bool g_combo_active = false;
static uint32_t g_active_combo_mask = 0;
static int64_t g_combo_last_log_us = 0;
static int64_t g_combo_press_start_us = 0;
static bool g_combo_over_20s_blocked = false;
static bool g_combo_setting_mode_armed = false;

static uint8_t g_click_pattern[PATTERN_LEN];
static uint8_t g_click_pattern_count = 0;
static int64_t g_click_pattern_last_us = 0;

static bool g_combo_led_on = false;
bool combo_led_toggle_1 = false;
int combo_led_toggle_counts_1 = 0;
bool combo_led_toggle_2 = false;
int combo_led_toggle_counts_2 = 0;


TimerHandle_t ready_commissioning_timer = NULL;
extern bool ready_commisioning_flag;

static void clear_ready_commissioning_flag(TimerHandle_t xTimer)
{
    ready_commisioning_flag = false;
    ESP_LOGI(TAG, "ready_commisioning_flag cleared after 30s timeout");
}

static void start_ready_commissioning_window(void)
{
    ready_commisioning_flag = true;

    if (ready_commissioning_timer) {
        xTimerStop(ready_commissioning_timer, 0);
        xTimerChangePeriod(ready_commissioning_timer, pdMS_TO_TICKS(READY_COMMISSIONING_MS), 0);
        xTimerStart(ready_commissioning_timer, 0);
    }

    ESP_LOGI(TAG, "ready_commisioning_flag set for 30s");
}

static void stop_ready_commissioning_window(void)
{
    ready_commisioning_flag = false;

    if (ready_commissioning_timer) {
        xTimerStop(ready_commissioning_timer, 0);
    }

    ESP_LOGI(TAG, "ready_commisioning_flag cleared");
}

static inline bool is_valid_combo_mask(uint32_t mask)
{
    return (mask == 0x03) ||   // buttons 0 + 1
           (mask == 0x05) ||   // buttons 0 + 2
           (mask == 0x0A) ||   // buttons 1 + 3
           (mask == 0x0C);     // buttons 2 + 3
}

static void reset_click_pattern(void)
{
    memset(g_click_pattern, 0, sizeof(g_click_pattern));
    g_click_pattern_count = 0;
    g_click_pattern_last_us = 0;
}

static void push_click_pattern(uint8_t button_id, int64_t now)
{
    if (g_click_pattern_last_us != 0 &&
        (now - g_click_pattern_last_us) > PATTERN_GAP_TIMEOUT_US) {
        reset_click_pattern();
    }

    g_click_pattern_last_us = now;

    if (g_click_pattern_count < PATTERN_LEN) {
        g_click_pattern[g_click_pattern_count++] = button_id;
    } else {
        memmove(&g_click_pattern[0], &g_click_pattern[1], PATTERN_LEN - 1);
        g_click_pattern[PATTERN_LEN - 1] = button_id;
    }

    ESP_LOGI(TAG, "pattern buffer: [%d %d %d %d %d %d], count=%d",
             g_click_pattern[0], g_click_pattern[1], g_click_pattern[2],
             g_click_pattern[3], g_click_pattern[4], g_click_pattern[5],
             g_click_pattern_count);
}

static bool match_6(const uint8_t p0, const uint8_t p1, const uint8_t p2,
                    const uint8_t p3, const uint8_t p4, const uint8_t p5)
{
    if (g_click_pattern_count < PATTERN_LEN) {
        return false;
    }

    return (g_click_pattern[0] == p0 &&
            g_click_pattern[1] == p1 &&
            g_click_pattern[2] == p2 &&
            g_click_pattern[3] == p3 &&
            g_click_pattern[4] == p4 &&
            g_click_pattern[5] == p5);
}

static void detect_6_click_pattern(uint8_t button_id, int64_t now)
{
    push_click_pattern(button_id, now);

    if (g_click_pattern_count < PATTERN_LEN) {
        return;
    }
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_DALI_DIRECT_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
    if (match_6(0,1,0,1,0,1) || match_6(1,0,1,0,1,0)) {
        ESP_LOGI(TAG, "PATTERN DETECTED: 1,2,1,2,1,2");
        reset_click_pattern();

        #ifndef USE_COLOR_CONTROL
            #ifdef USE_WIFI_WEBSERVER
            if (ready_commisioning_flag) {
                // if(wifi_webserver_active_flag){
                //     wifi_webserver_active_flag = false;
                // }else{
                    wifi_webserver_active_flag = true;  
                // }
                setNVSCommissioningFlag(0);
                setNVSWebServerEnableFlag(wifi_webserver_active_flag);                    
                esp_restart();
            }			
            #endif

        #endif
        stop_ready_commissioning_window();
    }
    else if (match_6(0,1,1,0,0,1) || match_6(1,0,0,1,1,0)) {
        ESP_LOGI(TAG, "PATTERN DETECTED: 1,2,2,1,1,2");
        reset_click_pattern();
        #ifndef USE_COLOR_CONTROL
        if (ready_commisioning_flag) {
            for (int i = 0; i < 50; i++) {
                esp_err_t status = esp_zb_zcl_scenes_table_clear_by_index(i);
                if (status != ESP_OK) break;
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
        #endif
        stop_ready_commissioning_window();
    }
    else if (match_6(0,2,0,2,0,2) || match_6(2,0,2,0,2,0)) {
        ESP_LOGI(TAG, "PATTERN DETECTED: 1,3,1,3,1,3");
        reset_click_pattern();
        #if(defined(USE_COLOR_CONTROL) || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
        #ifdef USE_WIFI_WEBSERVER
        if (ready_commisioning_flag) {
            // if(wifi_webserver_active_flag){
            //     wifi_webserver_active_flag = false;
            // }else{
                wifi_webserver_active_flag = true;  
            // }
            setNVSCommissioningFlag(0);
            setNVSWebServerEnableFlag(wifi_webserver_active_flag);                    
            esp_restart();
        }			
        #endif
        #endif
        stop_ready_commissioning_window();
    }
    else if (match_6(0,2,2,0,0,2) || match_6(2,0,0,2,2,0)) {
        ESP_LOGI(TAG, "PATTERN DETECTED: 1,3,3,1,1,3");
        reset_click_pattern();
        #if(defined(USE_COLOR_CONTROL) || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
        if (ready_commisioning_flag) {
            for (int i = 0; i < 50; i++) {
                esp_err_t status = esp_zb_zcl_scenes_table_clear_by_index(i);
                if (status != ESP_OK) break;
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
        #endif
        stop_ready_commissioning_window();
    }
    #else
    if (match_6(0,2,0,2,0,2)) {
        ESP_LOGI(TAG, "PATTERN DETECTED: 1,2,1,2,1,2");
        reset_click_pattern();
        stop_ready_commissioning_window();
    }
    else if (match_6(0,1,0,1,0,1)) {
        ESP_LOGI(TAG, "PATTERN DETECTED: 1,3,1,3,1,3");
        reset_click_pattern();
        
        #ifdef USE_WIFI_WEBSERVER
        if (ready_commisioning_flag) {
            if(wifi_webserver_active_flag){
                wifi_webserver_active_flag = false;
            }else{
                wifi_webserver_active_flag = true;  
            }
            setNVSCommissioningFlag(0);
            setNVSWebServerEnableFlag(wifi_webserver_active_flag);                    
            esp_restart();
        }			
        #endif
        stop_ready_commissioning_window();
    }
    else if (match_6(1,0,1,0,1,0)) {
        ESP_LOGI(TAG, "PATTERN DETECTED: 3,1,3,1,3,1");
        reset_click_pattern();
        
        #ifdef USE_WIFI_WEBSERVER
        if (ready_commisioning_flag) {
            if(wifi_webserver_active_flag){
                wifi_webserver_active_flag = false;
            }else{
                wifi_webserver_active_flag = true;  
            }
            setNVSCommissioningFlag(0);
            setNVSWebServerEnableFlag(wifi_webserver_active_flag);                    
            esp_restart();
        }			
        #endif
        stop_ready_commissioning_window();        
    }
    else if (match_6(0,1,1,0,0,1)) {
        ESP_LOGI(TAG, "PATTERN DETECTED: 1,3,3,1,1,3");
        reset_click_pattern();

        if (ready_commisioning_flag) {
            for (int i = 0; i < 50; i++) {
                esp_err_t status = esp_zb_zcl_scenes_table_clear_by_index(i);
                if (status != ESP_OK) break;
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
        stop_ready_commissioning_window();
    }
    else if (match_6(1,0,0,1,1,0)) {
        ESP_LOGI(TAG, "PATTERN DETECTED: 3,1,1,3,3,1");
        reset_click_pattern();

        if (ready_commisioning_flag) {
            for (int i = 0; i < 50; i++) {
                esp_err_t status = esp_zb_zcl_scenes_table_clear_by_index(i);
                if (status != ESP_OK) break;
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
        stop_ready_commissioning_window();
    }
    #endif
}

static inline int find_button_index(gpio_num_t pin)
{
    for (int i = 0; i < switch_num; i++) {
        if (switch_func_pair[i].pin == pin) return i;
    }
    return -1;
}

static inline uint32_t current_pressed_mask(void)
{
    uint32_t mask = 0;
    for (int i = 0; i < switch_num; i++) {
        if (g_btn[i].pressed) {
            mask |= (1U << g_btn[i].button_id);
        }
    }
    return mask;
}

static inline int pressed_count(void)
{
    int cnt = 0;
    for (int i = 0; i < switch_num; i++) {
        if (g_btn[i].pressed) cnt++;
    }
    return cnt;
}

static uint8_t get_combo_func_from_mask(uint32_t combo_mask)
{
    if (combo_mask == ((1U << 0) | (1U << 1))) {
        return SWITCH_FACTORY_RESET_CONTROL;
    }

    if (combo_mask == ((1U << 2) | (1U << 3))) {
        return SWITCH_SCENE_CHANGE_CONTROL;
    }

    if (combo_mask == ((1U << 0) | (1U << 2))) {
        return SWITCH_FACTORY_RESET_CONTROL;
    }

    if (combo_mask == ((1U << 1) | (1U << 3))) {
        return SWITCH_SCENE_CHANGE_CONTROL;
    }

    return SWITCH_NOTHING_CONTROL;
}

static void combo_status_led_update(uint32_t combo_mask, bool enable)
{
#ifdef USE_RGB_LED
    if (enable) {
        if (combo_mask == 0x05) {
            light_driver_set_color_RGB(0x00, 0x00, 0xFF);
            light_driver_set_power(true);
            g_combo_led_on = true;
        } else if (combo_mask == 0x0A) {
            light_driver_set_color_RGB(0x00, 0xFF, 0x00);
            light_driver_set_power(true);
            g_combo_led_on = true;
        }
    } else {
        if (g_combo_led_on) {
            light_driver_set_power(false);
            g_combo_led_on = false;
        }
    }
#endif
}

static void log_combo_hold_message(uint32_t combo_mask)
{
    if (combo_mask == 0x05 || combo_mask == 0x03) {
        printf("Zigbee Comissioning Buttons Detected!!\n");
        if (combo_led_toggle_counts_1++ < 10) {
            combo_status_led_update(combo_mask, combo_led_toggle_1 = !combo_led_toggle_1);
        } else {
            if (combo_led_toggle_1) {
                combo_led_toggle_1 = false;
                combo_status_led_update(combo_mask, false);
            }
        }
    } else if (combo_mask == 0x0A || combo_mask == 0x0C) {
        printf("WiFi Webserver Buttons Detected!!\n");
        if (combo_led_toggle_counts_2++ < 10) {
            combo_status_led_update(combo_mask, combo_led_toggle_2 = !combo_led_toggle_2);
        } else {
            if (combo_led_toggle_2) {
                combo_led_toggle_2 = false;
                combo_status_led_update(combo_mask, false);
            }
        }
    } else {
        printf("Unknown combo hold, mask=0x%02" PRIx32 "\n", combo_mask);
    }
}

static void emit_event(uint8_t button_id,
                       gpio_num_t pin,
                       switch_event_t event,
                       uint8_t click_count,
                       uint32_t combo_mask,
                       int64_t ts_us)
{
    if (!func_ptr) return;

    switch_func_pair_t out = {0};
    out.id = button_id;
    out.pin = pin;
    out.keypressed = 0xFF;
    out.func = SWITCH_NOTHING_CONTROL;

    if (event == SWITCH_EVENT_COMBO_LONG_PRESS) {
        out.keypressed = LONG_PRESS;
        out.func = get_combo_func_from_mask(combo_mask);

        ESP_LOGI(TAG, "SWITCH_EVENT_COMBO_LONG_PRESS mask=0x%02" PRIx32 " func=0x%02x",
                 combo_mask, out.func);

        if (combo_mask == 0x05 || combo_mask == 0x03) {
            printf("Zigbee Comissioning Buttons Detected!!\n");
        } else if (combo_mask == 0x0A || combo_mask == 0x0C) {
            printf("WiFi Webserver Buttons Detected!!\n");
        }

        if (out.func != SWITCH_NOTHING_CONTROL) {
            func_ptr(&out);
        } else {
            printf("No function assigned for combo event!!\n");
        }
        return;
    }

    int idx = find_button_index(pin);
    if (idx < 0) {
        return;
    }

    switch (event) {
        case SWITCH_EVENT_SINGLE_CLICK:
            ESP_LOGI(TAG, "SWITCH_EVENT_SINGLE_CLICK");
            out.func = switch_func_pair[idx].single_func;
            out.keypressed = SINGLE_PRESS;
            break;

        case SWITCH_EVENT_DOUBLE_CLICK:
            ESP_LOGI(TAG, "SWITCH_EVENT_DOUBLE_CLICK");
            out.func = switch_func_pair[idx].double_func;
            out.keypressed = DOUBLE_PRESS;
            break;

        case SWITCH_EVENT_MULTI_CLICK:
            ESP_LOGI(TAG, "SWITCH_EVENT_MULTI_CLICK count=%u", click_count);
            out.func = switch_func_pair[idx].multi_func;
            out.keypressed = SINGLE_PRESS;
            break;

        case SWITCH_EVENT_LONG_PRESS:
            ESP_LOGI(TAG, "SWITCH_EVENT_LONG_PRESS");
            out.func = switch_func_pair[idx].long_func;
            out.keypressed = LONG_PRESS;
            break;

        case SWITCH_EVENT_LONG_HOLD_REPEAT:
            ESP_LOGI(TAG, "SWITCH_EVENT_LONG_HOLD_REPEAT_123");
            out.func = switch_func_pair[idx].long_func;
            out.keypressed = LONG_PRESS_INC_DEC_LEVEL;
            break;

        default:
            return;
    }

    if (out.func != SWITCH_NOTHING_CONTROL) {
        func_ptr(&out);
    } else {
        printf("No function assigned for this event!!\n");
    }
}

static inline bool is_switch_pin(gpio_num_t pin)
{
    for (int i = 0; i < switch_num; i++) {
        if (switch_func_pair[i].pin == pin) {
            return true;
        }
    }
    return false;
}

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    const switch_func_pair_t *btn = (const switch_func_pair_t *)arg;
    if (btn == NULL) {
        return;
    }

    if (!is_switch_pin(btn->pin)) {
        //printf("Interrupt on non-switch pin %d, ignoring\n", btn->pin);
        return;   // ignore any non-switch GPIO, e.g. DALI RX pin
    }
    gpio_evt_t evt = {
        .button_id = btn->id,
        .pin = btn->pin,
        .level = gpio_get_level(btn->pin),
        .isr_time_us = esp_timer_get_time()
    };

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(gpio_evt_queue, &evt, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void handle_confirmed_single_click(uint8_t button_id, gpio_num_t pin, int64_t ts_us)
{
    detect_6_click_pattern(button_id, ts_us);
    emit_event(button_id, pin, SWITCH_EVENT_SINGLE_CLICK, 1, 0, ts_us);
}

static void process_edge_event(const gpio_evt_t *evt)
{
    int idx = find_button_index(evt->pin);
    if (idx < 0) return;

    button_state_t *b = &g_btn[idx];

    if ((evt->isr_time_us - b->last_edge_us) < DEBOUNCE_US) {
        return;
    }
    b->last_edge_us = evt->isr_time_us;

    bool new_level = evt->level ? true : false;
    if (new_level == b->stable_level) {
        return;
    }

    b->stable_level = new_level;

    if (new_level == false) {
        b->pressed = true;
        b->long_reported = false;
        b->setting_mode_armed = false;
        b->over_20s_blocked = false;
        b->press_start_us = evt->isr_time_us;
        b->last_hold_log_us = 0;
        g_combo_long_reported = false;
    } else {
        int64_t held_us = evt->isr_time_us - b->press_start_us;

        b->pressed = false;
        b->release_time_us = evt->isr_time_us;
        b->last_hold_log_us = 0;

        if (held_us >= SETTING_MODE_MIN_US && held_us < SETTING_MODE_MAX_US) {
            start_ready_commissioning_window();
        }

        if (held_us >= SETTING_MODE_MAX_US) {
            b->click_count = 0;
            b->long_reported = true;
            b->setting_mode_armed = false;
            b->over_20s_blocked = false;
            ESP_LOGI(TAG, "Button %d held >20s, forcing NOTHING", b->button_id);
        } else if (!b->long_reported) {
            uint32_t ev = switch_func_pair[idx].enabled_events;

            if ((ev & BTN_EVT_SINGLE) && !(ev & BTN_EVT_DOUBLE)) {
                handle_confirmed_single_click(b->button_id, b->pin, evt->isr_time_us);
                b->click_count = 0;
            } else {
                b->click_count++;
            }
        }

        b->setting_mode_armed = false;
        b->over_20s_blocked = false;

        if (pressed_count() == 0) {
            g_combo_long_reported = false;
        }

        if (g_combo_press_start_us != 0 && pressed_count() == 0) {
            int64_t combo_held_us = evt->isr_time_us - g_combo_press_start_us;

            if (combo_held_us >= SETTING_MODE_MIN_US && combo_held_us < SETTING_MODE_MAX_US) {
                start_ready_commissioning_window();
            }

            g_combo_press_start_us = 0;
            g_combo_setting_mode_armed = false;
            g_combo_over_20s_blocked = false;
        }
    }
}

static void process_long_press_and_clicks(void)
{
    int64_t now = esp_timer_get_time();
    uint32_t cur_mask = current_pressed_mask();

    if (pressed_count() >= 2 && g_combo_press_start_us == 0) {
        int64_t oldest_press = now;
        for (int i = 0; i < switch_num; i++) {
            if (g_btn[i].pressed && g_btn[i].press_start_us < oldest_press) {
                oldest_press = g_btn[i].press_start_us;
            }
        }
        g_combo_press_start_us = oldest_press;
    }

    if (g_combo_active) {
        if (cur_mask != g_active_combo_mask || cur_mask == 0) {
            ESP_LOGI(TAG, "Combo released/changed: old=0x%02" PRIx32 " new=0x%02" PRIx32,
                     g_active_combo_mask, cur_mask);

            if (combo_led_toggle_1 || combo_led_toggle_2) {
                combo_led_toggle_1 = false;
                combo_led_toggle_2 = false;
                combo_status_led_update(g_active_combo_mask, false);
            }

            combo_led_toggle_counts_1 = 0;
            combo_led_toggle_counts_2 = 0;

            g_combo_active = false;
            g_active_combo_mask = 0;
            g_combo_long_reported = false;
            g_combo_last_log_us = 0;

            if (cur_mask == 0) {
                if (g_combo_press_start_us != 0) {
                    int64_t combo_held_us = now - g_combo_press_start_us;
                    if (combo_held_us >= SETTING_MODE_MIN_US &&
                        combo_held_us < SETTING_MODE_MAX_US) {
                        start_ready_commissioning_window();
                    }
                }
                g_combo_press_start_us = 0;
                g_combo_setting_mode_armed = false;
                g_combo_over_20s_blocked = false;
            }
        } else {
            int64_t combo_held_us = now - g_combo_press_start_us;

            if (combo_held_us >= SETTING_MODE_MAX_US) {
                g_combo_over_20s_blocked = true;
                g_combo_active = false;
                g_active_combo_mask = 0;
                g_combo_long_reported = true;
                combo_status_led_update(cur_mask, false);
                ESP_LOGI(TAG, "Combo held >20s, forcing NOTHING");
                return;
            }

            if ((now - g_combo_last_log_us) >= COMBO_HOLD_REPEAT_US) {
                g_combo_last_log_us = now;
                log_combo_hold_message(cur_mask);
            }
            return;
        }
    }

    if (!g_combo_long_reported && pressed_count() >= 2) {
        int64_t oldest_press = now;
        uint32_t combo_mask = cur_mask;

        for (int i = 0; i < switch_num; i++) {
            if (g_btn[i].pressed && g_btn[i].press_start_us < oldest_press) {
                oldest_press = g_btn[i].press_start_us;
            }
        }

        int64_t combo_held_us = now - oldest_press;

        if (combo_held_us >= SETTING_MODE_MIN_US) {
            g_combo_setting_mode_armed = true;
        }

        if (combo_held_us >= SETTING_MODE_MAX_US) {
            g_combo_over_20s_blocked = true;
            g_combo_long_reported = true;
            g_combo_active = false;
            g_active_combo_mask = 0;
            ESP_LOGI(TAG, "Combo held >20s before emit, forcing NOTHING");
            return;
        }

        if (is_valid_combo_mask(combo_mask) &&
            (combo_held_us >= COMBO_LONG_PRESS_US) &&
            !g_combo_over_20s_blocked) {
            g_combo_long_reported = true;
            g_combo_active = true;
            g_active_combo_mask = combo_mask;
            g_combo_last_log_us = now;

            emit_event(0xFF, GPIO_NUM_NC,
                       SWITCH_EVENT_COMBO_LONG_PRESS, 0, combo_mask, now);

            for (int i = 0; i < switch_num; i++) {
                if (g_btn[i].pressed) {
                    g_btn[i].long_reported = true;
                    g_btn[i].click_count = 0;
                    g_btn[i].last_hold_log_us = now;
                }
            }
            return;
        }
    }

    for (int i = 0; i < switch_num; i++) {
        button_state_t *b = &g_btn[i];

        if (b->pressed) {
            int64_t held_us = now - b->press_start_us;

            if (held_us >= SETTING_MODE_MIN_US) {
                b->setting_mode_armed = true;
            }

            if (held_us >= SETTING_MODE_MAX_US) {
                b->over_20s_blocked = true;
                continue;
            }

            if (!b->long_reported) {
                if ((now - b->press_start_us) >= LONG_PRESS_US) {
                    b->long_reported = true;
                    b->click_count = 0;
                    b->last_hold_log_us = now;

                    emit_event(b->button_id, b->pin,
                               SWITCH_EVENT_LONG_PRESS, 0, 0, now);
                }
            } else {
                if ((now - b->last_hold_log_us) >= LONG_HOLD_REPEAT_US) {
                    b->last_hold_log_us = now;

                    emit_event(b->button_id, b->pin,
                               SWITCH_EVENT_LONG_HOLD_REPEAT, 0, 0, now);
                }
            }
        }
    }

    for (int i = 0; i < switch_num; i++) {
        button_state_t *b = &g_btn[i];

        if (!b->pressed && b->click_count > 0) {
            if ((now - b->release_time_us) >= MULTI_CLICK_GAP_US) {
                uint32_t ev = switch_func_pair[i].enabled_events;

                if (b->click_count == 1) {
                    if (ev & BTN_EVT_SINGLE) {
                        handle_confirmed_single_click(b->button_id, b->pin, now);
                    }
                } else if (b->click_count == 2) {
                    if (ev & BTN_EVT_DOUBLE) {
                        emit_event(b->button_id, b->pin,
                                   SWITCH_EVENT_DOUBLE_CLICK, 2, 0, now);
                    } else if (ev & BTN_EVT_MULTI) {
                        emit_event(b->button_id, b->pin,
                                   SWITCH_EVENT_MULTI_CLICK, 2, 0, now);
                    }
                } else {
                    if (ev & BTN_EVT_MULTI) {
                        emit_event(b->button_id, b->pin,
                                   SWITCH_EVENT_MULTI_CLICK, b->click_count, 0, now);
                    }
                }

                b->click_count = 0;
            }
        }
    }
}

static void switch_driver_button_detected(void *arg)
{
    gpio_evt_t evt;

    while (1) {
        if (xQueueReceive(gpio_evt_queue, &evt, pdMS_TO_TICKS(10)) == pdTRUE) {

            process_edge_event(&evt);
        }
        // printf("looping in button_detected task...\n");
        process_long_press_and_clicks();
    }
}

static bool switch_driver_gpio_init(switch_func_pair_t *button_func_pair, uint8_t button_num)
{
    if (!button_func_pair || button_num == 0 || button_num > MAX_BUTTONS) {
        return false;
    }

    gpio_config_t io_conf = {0};
    switch_func_pair = button_func_pair;
    switch_num = button_num;

    uint64_t pin_bit_mask = 0;
    for (int i = 0; i < button_num; ++i) {
        pin_bit_mask |= (1ULL << button_func_pair[i].pin);

        g_btn[i].button_id = button_func_pair[i].id;
        g_btn[i].pin = button_func_pair[i].pin;
        g_btn[i].stable_level = true;
        g_btn[i].pressed = false;
        g_btn[i].long_reported = false;
        g_btn[i].setting_mode_armed = false;
        g_btn[i].over_20s_blocked = false;
        g_btn[i].last_edge_us = 0;
        g_btn[i].press_start_us = 0;
        g_btn[i].release_time_us = 0;
        g_btn[i].last_hold_log_us = 0;
        g_btn[i].click_count = 0;
    }

    io_conf.pin_bit_mask = pin_bit_mask;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_ANYEDGE;

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_evt_queue = xQueueCreate(GPIO_EVT_QUEUE_LEN, sizeof(gpio_evt_t));
    if (gpio_evt_queue == NULL) {
        ESP_LOGE(TAG, "Queue creation failed");
        return false;
    }

    if (ready_commissioning_timer == NULL) {
        ready_commissioning_timer = xTimerCreate(
            "ready_comm",
            pdMS_TO_TICKS(READY_COMMISSIONING_MS),
            pdFALSE,
            NULL,
            clear_ready_commissioning_flag
        );

        if (ready_commissioning_timer == NULL) {
            ESP_LOGE(TAG, "Timer creation failed");
            vQueueDelete(gpio_evt_queue);
            gpio_evt_queue = NULL;
            return false;
        }
    }

    BaseType_t ok = xTaskCreate(
        switch_driver_button_detected,
        "button_detected",
        TASK_STACK_SIZE_SWITCH,
        NULL,
        TASK_PRIORITY_SWITCH,
        NULL
    );
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Task creation failed");
        vQueueDelete(gpio_evt_queue);
        gpio_evt_queue = NULL;
        return false;
    }

    if (!isr_service_installed) {
        ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
        isr_service_installed = true;
    }

    for (int i = 0; i < button_num; ++i) {
        ESP_ERROR_CHECK(
            gpio_isr_handler_add(button_func_pair[i].pin,
                                 gpio_isr_handler,
                                 (void *)&button_func_pair[i])
        );
    }

    return true;
}

void switch_driver_gpios_intr_enabled(bool enabled)
{
    for (int i = 0; i < switch_num; ++i) {
        if (enabled) {
            gpio_intr_enable(switch_func_pair[i].pin);
        } else {
            gpio_intr_disable(switch_func_pair[i].pin);
        }
    }
}

bool switch_driver_init(switch_func_pair_t *button_func_pair,
                        uint8_t button_num,
                        esp_switch_callback_t cb)
{
    func_ptr = cb;
    return switch_driver_gpio_init(button_func_pair, button_num);
}


#else

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

bool bool_button_pressed                        = false;
bool bool_button_pressed_backup_ready           = false;
bool is_121212                                  = false;
bool is_212121                                  = false;
bool is_122112                                  = false;
bool is_211221                                  = false;
bool is_11221122                                = false;
bool is_22112211                                = false;

static bool toggle_status_led_long_press        = false;


void switch_driver_gpios_intr_enabled(bool enabled);
extern esp_err_t nuos_set_color_rgb_mode_attribute(uint8_t index, uint8_t val_mode);
#if (USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
extern char * nuos_do_task(uint8_t index, uint8_t scene_id, uint8_t erase_data);
#endif

extern bool ready_commisioning_flag;
TimerHandle_t ready_commissioning_timer = NULL;
// static void clear_ready_commissioning_flag(TimerHandle_t xTimer)
// {
//     ready_commisioning_flag = false;
//     ESP_LOGI(TAG, "ready_commisioning_flag cleared after 30s timeout");
// }

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
void switch_driver_gpios_intr_enabled(bool enabled)
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
    //printf("param:%d\n", param);

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
        //printf("click_count:%d\n", local_clicks);
        if (local_clicks == MAX_COUNTS_FOR_TRIPLE_CLICK) {
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

            if (is_122112 || is_211221) {
                //printf("Detected sequence: %s\n", is_122112 ? "1,2,2,1,1,2" : "2,1,1,2,2,1");
    
                if(ready_commisioning_flag){
                    for(int i=0; i<50; i++) {
                        esp_err_t status = esp_zb_zcl_scenes_table_clear_by_index(i);
                        if(status != ESP_OK) break;
                    }
                    setNVSStartCommissioningFlag(1);
                    setNVSCommissioningFlag(1);
                    setNVSPanicAttack(0);
                    if (esp_zb_bdb_dev_joined()) {
                         //printf("ready_commisioning_flag: 2\n");
                        esp_zb_bdb_reset_via_local_action();
                    }
                    // printf("ready_commisioning_flag: 3\n");
                    vTaskDelay(pdMS_TO_TICKS(100));
                    esp_zb_factory_reset();
                }else{
                   
                }
            } else if (is_121212 || is_212121) {
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
                        
                        #ifdef USE_C3_ADAPTER_UART_HW
                        printf("{\"mode\":%d}\n", wifi_webserver_active_flag);
                        #else
                        setNVSCommissioningFlag(0);
                        setNVSWebServerEnableFlag(wifi_webserver_active_flag);                    
                        esp_restart();	
                        #endif
                    #endif
                   
                }             
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

#ifdef USE_DOUBLE_PRESS
    switch_func_pair_t button_func_pair;
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

void check_long_press_tasks(uint32_t sw_pressed_cnts, const uint16_t compare_time_in_secs){
    if (sw_pressed_cnts == 2) {   
        if(get_button_pressed_mode() == 1){
            if(total_press_in_secs == compare_time_in_secs){
                #ifdef USE_RGB_LED
                    light_driver_set_power(false);
                #endif 
            }else if(total_press_in_secs > compare_time_in_secs){  

            }else{
                #ifdef USE_RGB_LED
                    toggle_status_led_long_press = !toggle_status_led_long_press;
                    if(toggle_status_led_long_press) light_driver_set_color_RGB(0, 0, 0xff);  //red
                    light_driver_set_power(toggle_status_led_long_press);
                #endif  
            }
        }  else if(get_button_pressed_mode() == 2){
            if(total_press_in_secs == compare_time_in_secs){
                #ifdef USE_RGB_LED
                    light_driver_set_power(false);
                #endif 
    
            }else if(total_press_in_secs > compare_time_in_secs){  

            }else{
                #ifdef USE_RGB_LED
                    toggle_status_led_long_press = !toggle_status_led_long_press;
                    if(toggle_status_led_long_press) light_driver_set_color_RGB(0x00, 0xff, 0x00);  //green
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


/* ********************************************************************** */
static void switch_driver_button_detected(void *arg) {
    gpio_num_t io_num = GPIO_NUM_NC;
    static bool start_comm_flag = false;
    uint32_t reduced_bounce_time = DEBOUNCE_TIME_MS;
    static switch_state_t switch_state = SWITCH_IDLE;
    bool evt_flag = false;
    uint32_t switch_pressed_cnts = 0;
    bool two_switch_pressed_flag = false;
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI)
        bool instant_two_switch_pressed_flag = false;
    #endif
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RINGING_BELL_2)
    bool switch_keep_pressed_once_flag = true;
    #endif

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
        switch_func_pair_t button_func_pair;
        if (xQueueReceive(gpio_evt_queue, &button_func_pair, portMAX_DELAY)) {
            //printf("Button ISR received for pin %ld\n", button_func_pair.pin);
            switch_driver_gpios_intr_enabled(false);
            recheckTimer();
            io_num = button_func_pair.pin;
            evt_flag = true;
            longpress_detected = false;
            total_press_in_secs = 0;
            two_switch_pressed_flag = false; 
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI)
            instant_two_switch_pressed_flag = false;
            #endif
            initTwoSwitchPressedPins();
            #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
               nuos_init_hardware_dimming_up_down(io_num);
            #endif
            // esp_start_timer_3();

            #ifndef USE_DOUBLE_PRESS
                button_func_pair.keypressed = SINGLE_PRESS;
            #endif

            brightness_count = 0;
            switch_pressed_cnts = 0;
            reduced_bounce_time = DEBOUNCE_TIME_MS;
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RINGING_BELL_2)
            switch_keep_pressed_once_flag = true;
            #endif
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
                    if (double_press_click_enable[0]) {
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
                                
                                if(wifi_webserver_active_flag){
                                    if(start_commissioning){
                                        start_comm_flag = true;
                                        start_commissioning = false;
                                    }
                                    
                                }
                                brightness_control_tasks(io_num);
                                switch_pressed_cnts = IdentifyTwoSwitchPressed();
                                if (!two_switch_pressed_flag) {
                                    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH || \
                                        (USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_MOTION || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_CONTACT_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_GAS_LEAK || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_LUX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY))
                                        if (switch_pressed_cnts >= 1) {
                                            two_switch_pressed_flag = true;
                                        }
                                    #else
                                        if (switch_pressed_cnts >= 2) {
                                            two_switch_pressed_flag = true;
                                            nuos_set_rgb_led_commissioning_functionality();
                                            #ifdef USE_C3_ADAPTER_UART_HW
                                            //printf("\"time\":%ld\n", switch_pressed_cnts);
                                            #else
                                            printf("2 switch pressed!!\n");
                                            #endif
                                        }
                                    #endif
                                }

                                if (current_time - double_release_time >= 1000) {
                                    double_release_time = current_time;
                                    longpress_detected = true;
                                    total_press_in_secs++;
                                    #ifdef USE_C3_ADAPTER_UART_HW
                                        printf("{\"time\":%d}\n", total_press_in_secs);
                                    #else        
                                        ESP_LOGI(TAG, "total_press_in_secs_1:%d\n", total_press_in_secs);
                                    #endif
                                    check_long_press_tasks(switch_pressed_cnts, SETUP_LONG_PRESS_TIME_IN_SECS);
                                }
                            }
                        }
                    }else{
                        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI)
                        if (IdentifyTwoSwitchPressed() >= 2) {
                            instant_two_switch_pressed_flag = true;
                            
                            #ifdef USE_C3_ADAPTER_UART_HW
                     
                            #else
                                printf("2 switch pressed!!\n");
                            #endif
                        }
                        #endif
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
                    if(wifi_webserver_active_flag){
                        start_commissioning = start_comm_flag;
                    }

                    //start_commissioning = save_is_my_device_commissionned;
                    reduced_bounce_time = 10;
                    if (two_switch_pressed_flag) {
                        if(switch_pressed_cnts >= 2){
                            if(total_press_in_secs < 10 || total_press_in_secs > 18){                                            
                                ready_commisioning_flag = false;
                            }
                        }
                    }
                    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI)
                    if(instant_two_switch_pressed_flag){
                        instant_two_switch_pressed_flag = false;
                        change_cw_ww_color_flag = !change_cw_ww_color_flag;
                    }
                    #endif
                    /* ... inside SWITCH_RELEASE_DETECTED ... */
                    #if defined(USE_DOUBLE_PRESS) || defined(USE_TRIPLE_CLICK)
                        if (click_count < CLICK_ARRAY_SIZE) {
                            if(!longpress_detected){
                                switch_num_pressed[click_count] = io_num;
                                click_count++;
                            }
                        } else {
                            // Saturated — ignore further clicks until handler runs
                            //ESP_LOGW(TAG, "click_count saturated, ignoring additional clicks");
                        }

                        if (click_timer != NULL) {
                            if (xTimerIsTimerActive(click_timer) != pdFALSE) {
                                xTimerStop(click_timer, 0);
                            }
                            xTimerStart(click_timer, 0);
                        } else {
                            //ESP_LOGW(TAG, "click_timer is NULL when trying to start it");
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


                    // ---- NEW LONG‑PRESS DURATION HANDLING ----
                    if (longpress_detected) {
                        // press_duration is already in ms from earlier calculation
                        uint32_t press_sec = press_duration / 1000;

                        if (press_sec > 20) {
                            // Ignore: do not send any event
                            //ESP_LOGW(TAG, "Long press >20s (%d s) – ignored", press_sec);
                            button_func_pair.func = SWITCH_NOTHING_CONTROL;
                            // Also prevent any later single/double click
                            longpress_detected = false;
                            click_count = 0;
                            if (click_timer) xTimerStop(click_timer, 0);
                        }
                        else if (press_sec > 10 && press_sec < 20) {
                            // Enter setting mode
                            ready_commisioning_flag = true;
                            //ESP_LOGI(TAG, "Long press %d s -> ready_commisioning_flag = true", press_sec);

                            // Start a 30s timer to auto‑clear the flag
                            // if (ready_commissioning_timer == NULL) {
                            //     ready_commissioning_timer = xTimerCreate("ready_cmsn_tmr",
                            //                                             pdMS_TO_TICKS(30000),
                            //                                             pdFALSE,
                            //                                             NULL,
                            //                                             clear_ready_commissioning_flag);
                            // }
                            // Reset the timer (stop & start) so it always gives 30s from last release
                            if (ready_commissioning_timer) {
                                xTimerStop(ready_commissioning_timer, 0);
                                xTimerStart(ready_commissioning_timer, 0);
                            }
                        }
                        // else (<=10s) – normal long press (existing behaviour)
                    }
                    break;

                case SWITCH_LONG_PRESS_DETECTED:
                    switch_state = SWITCH_IDLE;
                    button_func_pair.keypressed = LONG_PRESS;
                    break;

                default:
                    switch_state = SWITCH_IDLE;
                    switch_driver_gpios_intr_enabled(true);
                    break;
            }

            if (switch_state == SWITCH_IDLE) {
                actionOnTwoSwitchPressed(total_press_in_secs);
                #ifndef USE_DOUBLE_PRESS
                    button_func_pair.func = SWITCH_ONOFF_TOGGLE_CONTROL;
                    button_func_pair.pin = io_num;
                    if (func_ptr != NULL) {
                        (*func_ptr)(&button_func_pair); // Call the callback function
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
    io_conf.intr_type = GPIO_INTR_NEGEDGE; //GPIO_INTR_LOW_LEVEL; //
    io_conf.pin_bit_mask = pin_bit_mask;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    /* configure GPIO with the given settings */
    gpio_config(&io_conf);

    /* create a queue to handle gpio event from isr */
    gpio_evt_queue = xQueueCreate(20, sizeof(switch_func_pair_t));
    if ( gpio_evt_queue == 0) {
        //ESP_LOGE(TAG, "Queue was not created and must not be used");
        return false;
    }
    //zb_event_queue = xQueueCreate(10, sizeof(zb_button_event_t));
   // assert(zb_event_queue);
    /* start gpio task */
    xTaskCreate(switch_driver_button_detected, "button_detected", TASK_STACK_SIZE_SWITCH, NULL, TASK_PRIORITY_SWITCH, NULL);

    /* install gpio isr service */
    if (!isr_service_installed) {
        ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT));
        isr_service_installed = true;
    }
   
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

#endif