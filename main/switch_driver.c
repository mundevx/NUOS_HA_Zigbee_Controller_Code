#include "app_config.h"

#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_DALI_DIRECT_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)


#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>
#include "hal/gpio_ll.h"
#include "soc/gpio_struct.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "esp_task_wdt.h"
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
#define ACTION_QUEUE_LEN             16
#define TASK_STACK_SIZE_SWITCH       4096
#define TASK_PRIORITY_SWITCH         10
#define TASK_PRIORITY_WORKER         8 //5

#define MIN_VALID_PRESS_US           20000   // NEW: 20ms floor — real human touches never release faster than this;
                                              // anything shorter is a touch-IC glitch pulse, not a genuine click

#define DEBOUNCE_US                  50000   // CHANGED: 30ms -> 50ms; now a "quiet window", see confirm_pending_edges()
#define CLICK_LOCKOUT_US             200000  // NEW: ignore a re-press within 200ms of a release - touch pads can
                                              // chatter a full low-high-low cycle mid-touch, which otherwise reads
                                              // as a second genuine press/release and double-fires a single click
#define DOUBLE_CLICK_BOUNCE_LOCKOUT_US 80000  // NEW: shorter chatter lockout used only for buttons with
                                              // BTN_EVT_DOUBLE enabled. The full 200ms CLICK_LOCKOUT_US was
                                              // absorbing the second, intentional tap of a fast real double-click
                                              // (it can legitimately land 100-200ms after the first release),
                                              // which is why double-click was collapsing into a single click / two
                                              // separate single clicks. 80ms still filters genuine touch-IC bounce
                                              // (which settles well under 50-80ms) while letting a real double-tap
                                              // reach the click_count accumulator below.
#define SINGLE_CLICK_DEDUPE_US       350000   // NEW: drop a SINGLE_CLICK for a button that already fired one
                                              // within this window - final safety net for touch-IC glitch cycles
                                              // that run past CLICK_LOCKOUT_US, see handle_confirmed_single_click()
#define LONG_PRESS_US                700000
#define LONG_HOLD_REPEAT_US          70000
#define MULTI_CLICK_GAP_US           500000
#define COMBO_LONG_PRESS_US          800000
#define COMBO_HOLD_REPEAT_US         1000000

#define SETTING_MODE_MIN_US          10000000LL   // 10 seconds
#define SETTING_MODE_MAX_US          20000000LL   // 20 seconds
#define READY_COMMISSIONING_MS       10000        // 30 seconds

#define PATTERN_LEN                  6
#define PATTERN_GAP_TIMEOUT_US       2000000

extern bool isr_service_installed;
extern bool ready_commisioning_flag;
bool copy_ready_commisioning_flag = false;

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
    uint8_t keypressed;
    uint8_t func;
} switch_action_t;

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

    // NEW: confirm-style debounce state (per button, correctly a struct member this time)
    int64_t candidate_time_us;
    bool    candidate_level;
    bool    candidate_pending;

    // NEW: set while absorbing a chattered press/release pair within CLICK_LOCKOUT_US
    // of the last release - see confirm_pending_edges()
    bool    bounce_suppressed;

    // NEW: timestamp of the last SINGLE_CLICK actually fired for this button - see
    // handle_confirmed_single_click(). CLICK_LOCKOUT_US only guards the press side
    // (a re-press within 200ms of the prior release is absorbed as chatter); on the
    // RGB DALI board a touch-IC glitch cycle can occasionally run just past that
    // window and still produce a second full press/release cycle, which was
    // reaching the app as two separate toggles for one physical tap (ON then
    // immediately OFF). This is a second, semantic-level guard: a single click is
    // dropped if the same button already fired one more recently than
    // SINGLE_CLICK_DEDUPE_US, regardless of how the duplicate press/release made it
    // this far.
    int64_t last_single_click_us;
} button_state_t;

typedef struct {
    uint8_t button_id;
    gpio_num_t pin;
    int level;
    int64_t isr_time_us;
} gpio_evt_t;

static QueueHandle_t gpio_evt_queue         = NULL;
static QueueHandle_t switch_action_queue    = NULL;
static volatile uint32_t g_isr_queue_drops  = 0;   // NEW: raw edges lost because gpio_evt_queue was full
static switch_func_pair_t *switch_func_pair = NULL;
static uint8_t switch_num                   = 0;

static button_state_t g_btn[MAX_BUTTONS];
static esp_switch_callback_t func_ptr       = NULL;
TimerHandle_t ready_commissioning_timer     = NULL;

static uint32_t g_active_combo_mask         = 0;
static int64_t g_combo_last_log_us          = 0;
static int64_t g_combo_press_start_us       = 0;

static uint8_t g_click_pattern[PATTERN_LEN];
static uint8_t g_click_pattern_count        = 0;
static int64_t g_click_pattern_last_us      = 0;
int combo_led_toggle_counts_1               = 0;
int combo_led_toggle_counts_2               = 0;

static bool g_combo_over_20s_blocked        = false;
static bool g_combo_setting_mode_armed      = false;
static bool g_combo_active                  = false;
static bool g_combo_long_reported           = false;
static bool g_combo_led_on                  = false;
bool combo_led_toggle_1                     = false;
bool combo_led_toggle_2                     = false;

bool long_press_10sec_valid                 = false;
// REMOVED: the three stray file-scope candidate_* globals that were here before.
// They were dead code (never referenced) and are now correctly per-button members
// of button_state_t above.

static void clear_ready_commissioning_flag(TimerHandle_t xTimer)
{
    copy_ready_commisioning_flag = false;
    ready_commisioning_flag = false;
    setNVSStartCommissioningFlag(0);
    light_driver_set_power(false);
    ESP_LOGI(TAG, "copy_ready_commisioning_flag cleared after 10s timeout");
}

static void start_ready_commissioning_window(void)
{
    if(long_press_10sec_valid){ 
        ready_commisioning_flag = true;
        copy_ready_commisioning_flag = true;
        setNVSStartCommissioningFlag(1);
        if (ready_commissioning_timer) {
            xTimerStop(ready_commissioning_timer, 0);
            xTimerChangePeriod(ready_commissioning_timer, pdMS_TO_TICKS(READY_COMMISSIONING_MS), 0);
            xTimerStart(ready_commissioning_timer, 0);
        }
        ESP_LOGI(TAG, "copy_ready_commisioning_flag set for 30s");
    }
    
}

static void stop_ready_commissioning_window(void)
{
    copy_ready_commisioning_flag = false;
    ready_commisioning_flag = false;
    if (ready_commissioning_timer) {
        xTimerStop(ready_commissioning_timer, 0);
    }
    light_driver_set_power(false);
    ESP_LOGI(TAG, "copy_ready_commisioning_flag cleared");
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
                wifi_webserver_active_flag = true;  
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
        ESP_LOGI(TAG, "PATTERN DETECTED: 1,3,1,3,1,3 ready_commisioning_flag: %d", ready_commisioning_flag);
        reset_click_pattern();
        #if(defined(USE_COLOR_CONTROL) || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
        #ifdef USE_WIFI_WEBSERVER
        if (ready_commisioning_flag) {
            
            wifi_webserver_active_flag = true;  
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
            wifi_webserver_active_flag = !wifi_webserver_active_flag;
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
            wifi_webserver_active_flag = !wifi_webserver_active_flag;
            setNVSCommissioningFlag(0);
            setNVSWebServerEnableFlag(wifi_webserver_active_flag);                    
            esp_restart();
        }			
        #endif
        stop_ready_commissioning_window();        
    }
    else if (match_6(0,1,1,0,0,1) || match_6(1,0,0,1,1,0)) {
        ESP_LOGI(TAG, "PATTERN DETECTED: 1,3,3,1,1,3");
        reset_click_pattern();
        if (ready_commisioning_flag) {
            esp_zb_lock_acquire(portMAX_DELAY);
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
            esp_zb_factory_reset();
            esp_zb_lock_release();

            vTaskDelay(pdMS_TO_TICKS(100));
            esp_restart();
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
        if (combo_mask == 0x05 || combo_mask == 0x03) {
            light_driver_set_color_RGB(0x00, 0x00, 0xFF);
            light_driver_set_power(true);
            g_combo_led_on = true;
        } else if (combo_mask == 0x0A || combo_mask == 0x0C) {
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
        printf("Zigbee Comissioning Buttons(%d) Detected!!\n", combo_led_toggle_counts_1);
        if (combo_led_toggle_counts_1++ < 10) {
            combo_status_led_update(combo_mask, combo_led_toggle_1 = !combo_led_toggle_1);
        } else {
            if (combo_led_toggle_1) {
                combo_led_toggle_1 = false;
                combo_status_led_update(combo_mask, false);
            }
        }
        long_press_10sec_valid = true;
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
        // CHANGED: WiFi Webserver combo no longer arms the 10s ready-commissioning
        // window - only the Zigbee Commissioning combo (0x05/0x03) above may set this true.
        long_press_10sec_valid = false;
    } else {
        long_press_10sec_valid = false;
        printf("Unknown combo hold, mask=0x%02" PRIx32 "\n", combo_mask);
    }
}

#define TASK_PRIORITY_WORKER    5   // Must NOT be 14

static void switch_worker_task(void *arg)
{
    switch_action_t act;
    while (1) {
        if (xQueueReceive(switch_action_queue, &act, portMAX_DELAY) == pdTRUE) {
            switch_func_pair_t out = {
                .id = act.button_id,
                .pin = act.pin,
                .keypressed = act.keypressed,
                .func = act.func
            };
            
            // NUOS: while the 30s ready-commissioning window is open, suppress every
            // normal switch action (single/double/multi/long-press/combo) - only the
            // click-pattern matcher (fed directly in confirm_pending_edges()) stays live.
            if (func_ptr != NULL && !copy_ready_commisioning_flag) {
                func_ptr(&out);
            } else if (func_ptr != NULL) {
                ESP_LOGW(TAG, "action suppressed: copy_ready_commisioning_flag active (button id=%d func=0x%02x)",
                         act.button_id, act.func);
            }

            // Yield CPU after every switch action processing
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

static void emit_event(uint8_t button_id,
                       gpio_num_t pin,
                       switch_event_t event,
                       uint8_t click_count,
                       uint32_t combo_mask,
                       int64_t ts_us)
{
    printf("emit_event1\n");
    if (!switch_action_queue) return;
    printf("emit_event2\n");
    switch_action_t act;
    memset(&act, 0, sizeof(switch_action_t));
    act.button_id = button_id;
    act.pin = pin;
    act.keypressed = 0xFF;
    act.func = SWITCH_NOTHING_CONTROL;

    if (event == SWITCH_EVENT_COMBO_LONG_PRESS) {
        act.keypressed = LONG_PRESS;
        act.func = get_combo_func_from_mask(combo_mask);

        ESP_LOGI(TAG, "SWITCH_EVENT_COMBO_LONG_PRESS mask=0x%02" PRIx32 " func=0x%02x",
                 combo_mask, act.func);
        long_press_10sec_valid = false; 
        if (combo_mask == 0x05 || combo_mask == 0x03) {
            printf("Zigbee Comissioning Buttons Detected!!\n");
        } else if (combo_mask == 0x0A || combo_mask == 0x0C) {
            printf("WiFi Webserver Buttons Detected!!\n");
        }

        if (act.func != SWITCH_NOTHING_CONTROL) {
            if (xQueueSend(switch_action_queue, &act, 0) != pdTRUE) {
                ESP_LOGW(TAG, "action queue full: dropped combo event mask=0x%02" PRIx32, combo_mask);
            }
        } else {
            printf("No function assigned for combo event!!\n");
        }
        return;
    }

    int idx = find_button_index(pin);
    printf("idx:%d\n", idx);
    if (idx < 0) {
        return;
    }

    switch (event) {
        case SWITCH_EVENT_SINGLE_CLICK:
            ESP_LOGI(TAG, "SWITCH_EVENT_SINGLE_CLICK");
            act.func = switch_func_pair[idx].single_func;
            act.keypressed = SINGLE_PRESS;
            //switch_driver_gpios_intr_enabled(false); 
            break;

        case SWITCH_EVENT_DOUBLE_CLICK:
            ESP_LOGI(TAG, "SWITCH_EVENT_DOUBLE_CLICK");
            act.func = switch_func_pair[idx].double_func;
            act.keypressed = DOUBLE_PRESS;
            break;

        case SWITCH_EVENT_MULTI_CLICK:
            ESP_LOGI(TAG, "SWITCH_EVENT_MULTI_CLICK count=%u", click_count);
            act.func = switch_func_pair[idx].multi_func;
            act.keypressed = SINGLE_PRESS;
            break;

        case SWITCH_EVENT_LONG_PRESS:
            ESP_LOGI(TAG, "SWITCH_EVENT_LONG_PRESS");
            act.func = switch_func_pair[idx].long_func;
            act.keypressed = LONG_PRESS;
            break;

        case SWITCH_EVENT_LONG_HOLD_REPEAT:
            ESP_LOGI(TAG, "SWITCH_EVENT_LONG_HOLD_REPEAT_123");
            act.func = switch_func_pair[idx].long_func;
            act.keypressed = LONG_PRESS_INC_DEC_LEVEL;
            break;

        default:
            return;
    }

    if (act.func != SWITCH_NOTHING_CONTROL) {
        if (xQueueSend(switch_action_queue, &act, 0) != pdTRUE) {
            ESP_LOGW(TAG, "action queue full: dropped button id=%d event=%d", button_id, event);
        }
    } else {
        printf("No function assigned for this event!!\n");
    }

}

static inline bool IRAM_ATTR is_switch_pin(gpio_num_t pin)
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
        return;
    }
    
    // Read GPIO directly from hardware registers in IRAM without calling flash functions
    int pin_level = gpio_ll_get_level(&GPIO, btn->pin);

    gpio_evt_t evt = {
        .button_id = btn->id,
        .pin = btn->pin,
        .level = pin_level,
        .isr_time_us = esp_timer_get_time()
    };

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xQueueSendFromISR(gpio_evt_queue, &evt, &xHigherPriorityTaskWoken) != pdTRUE) {
        // Can't ESP_LOGW from IRAM ISR context - just count it, the polling
        // task below reports the running total once it changes.
        g_isr_queue_drops++;
    }

    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void handle_confirmed_single_click(uint8_t button_id, gpio_num_t pin, int64_t ts_us)
{
    int idx = find_button_index(pin);
    if (idx >= 0) {
        button_state_t *b = &g_btn[idx];
        if (b->last_single_click_us != 0 &&
            (ts_us - b->last_single_click_us) < SINGLE_CLICK_DEDUPE_US) {
            ESP_LOGW(TAG, "Button %d: suppressing duplicate single click (%lldus since last)",
                    button_id, (long long)(ts_us - b->last_single_click_us));
            return;
        }
        b->last_single_click_us = ts_us;
    }

    detect_6_click_pattern(button_id, ts_us);
    emit_event(button_id, pin, SWITCH_EVENT_SINGLE_CLICK, 1, 0, ts_us);
}

// CHANGED: process_edge_event() no longer runs the press/release state machine directly.
// It only stages the latest raw edge as a per-button "candidate". Any further edge on the
// same pin overwrites the candidate and restarts the confirmation window. The real
// press/release logic (below, in confirm_pending_edges()) only runs once the pin has been
// quiet for DEBOUNCE_US — this is what stops a single physical click, whose contact bounce
// has a gap wider than DEBOUNCE_US, from being seen as two separate press/release cycles
// (which was producing the duplicate SWITCH_EVENT_SINGLE_CLICK you saw).
static void process_edge_event(const gpio_evt_t *evt)
{
    int idx = find_button_index(evt->pin);
    if (idx < 0) return;

    button_state_t *b = &g_btn[idx];

    b->candidate_level = evt->level ? true : false;
    b->candidate_time_us = evt->isr_time_us;
    b->candidate_pending = true;
}

// NEW: promotes a candidate edge to real button state once DEBOUNCE_US has passed
// with no further edges on that pin. Contains exactly the same press/release logic
// that used to live directly in process_edge_event().
static void confirm_pending_edges(void)
{
    int64_t now = esp_timer_get_time();

    for (int idx = 0; idx < switch_num; idx++) {
        button_state_t *b = &g_btn[idx];

        if (!b->candidate_pending) {
            continue;
        }
        if ((now - b->candidate_time_us) < DEBOUNCE_US) {
            continue;   // still within quiet window, wait (a newer edge already re-armed this)
        }

        b->candidate_pending = false;

        bool new_level = b->candidate_level;
        if (new_level == b->stable_level) {
            continue;   // no real level change
        }

        b->stable_level = new_level;
        int64_t confirmed_ts = b->candidate_time_us;

        if (new_level == false) {
            // ---- Confirmed PRESS ----
            // NEW: touch pads can chatter a full low-high-low cycle mid-touch. If this
            // press lands within CLICK_LOCKOUT_US of the last release, it's almost
            // certainly that chatter, not a genuine new press - absorb it so it can't
            // start (and later close out) a second click cycle for the same touch.
            // CHANGED: buttons with BTN_EVT_DOUBLE use a shorter lockout so a real
            // double-click's second tap isn't absorbed as chatter - see
            // DOUBLE_CLICK_BOUNCE_LOCKOUT_US above.
            int64_t lockout_us = (switch_func_pair[idx].enabled_events & BTN_EVT_DOUBLE)
                                      ? DOUBLE_CLICK_BOUNCE_LOCKOUT_US
                                      : CLICK_LOCKOUT_US;
            if (b->release_time_us != 0 &&
                (confirmed_ts - b->release_time_us) < lockout_us) {
                b->bounce_suppressed = true;
                ESP_LOGW(TAG, "Button %d: absorbing bounce re-press (%lldus since release)",
                        b->button_id, (long long)(confirmed_ts - b->release_time_us));
                continue;
            }

            // NEW: defensive reset - if click_count is stale from a much earlier
            // sequence (shouldn't normally happen since the gap-flush below runs first,
            // but this guards against any race), clear it instead of accumulating into it.
            if (b->click_count > 0 &&
                (confirmed_ts - b->release_time_us) >= MULTI_CLICK_GAP_US) {
                b->click_count = 0;
            }

            b->pressed = true;
            b->long_reported = false;
            b->setting_mode_armed = false;
            b->over_20s_blocked = false;
            b->press_start_us = confirmed_ts;
            b->last_hold_log_us = 0;
            g_combo_long_reported = false;
        } else {
            // ---- Confirmed RELEASE ----
            // NEW: this release closes out a press we already decided was chatter
            // (see the PRESS branch above) - stay out of the click/long-press state
            // machine entirely, we never entered it for this cycle.
            if (b->bounce_suppressed) {
                b->bounce_suppressed = false;
                continue;
            }

            int64_t held_us = confirmed_ts - b->press_start_us;

            // NEW: reject implausibly short presses (touch-IC glitch pulses), before they can
            // increment click_count or otherwise disturb button state.
            if (held_us < MIN_VALID_PRESS_US) {
                ESP_LOGW(TAG, "Button %d: ignoring glitch press (%lldus held, min=%dus)",
                        b->button_id, (long long)held_us, MIN_VALID_PRESS_US);
                b->pressed = false;
                b->release_time_us = confirmed_ts;
                b->last_hold_log_us = 0;
                // Deliberately do NOT touch click_count, long_reported, setting_mode_armed,
                // combo state, etc. — as far as the rest of the state machine is concerned,
                // this glitch never happened.
                continue;
            }

            b->pressed = false;
            b->release_time_us = confirmed_ts;
            b->last_hold_log_us = 0;

            if (held_us >= SETTING_MODE_MIN_US && held_us < SETTING_MODE_MAX_US) {
                start_ready_commissioning_window();
                //printf("Button %d held >10s, ready_commisioning_flag set\n", b->button_id);
            }

            if (held_us >= SETTING_MODE_MAX_US) {
                b->click_count = 0;
                b->long_reported = true;
                b->setting_mode_armed = false;
                b->over_20s_blocked = false;
                ESP_LOGI(TAG, "Button %d held >20s, forcing NOTHING", b->button_id);
            } else if (!b->long_reported) {
                if (copy_ready_commisioning_flag || ready_commisioning_flag) {
                    // NUOS: commissioning window open (armed by a 10-20s long press).
                    // Every click feeds the pattern matcher immediately regardless of the
                    // button's configured single/double/multi behavior - this is what
                    // disables double-click while the window is armed, since two quick
                    // clicks now register as two separate pattern entries instead of being
                    // grouped into SWITCH_EVENT_DOUBLE_CLICK.
                    detect_6_click_pattern(b->button_id, confirmed_ts);
                    b->click_count = 0;
                } else {
                    uint32_t ev = switch_func_pair[idx].enabled_events;

                    // CHANGED: also require !(ev & BTN_EVT_MULTI) so a button configured for
                    // SINGLE+MULTI (without DOUBLE) accumulates clicks instead of always firing
                    // immediately. Doesn't change behavior for your current 4 buttons (none has
                    // MULTI enabled) but avoids a footgun if you add one later.
                    if ((ev & BTN_EVT_SINGLE) && !(ev & BTN_EVT_DOUBLE) && !(ev & BTN_EVT_MULTI)) {
                        handle_confirmed_single_click(b->button_id, b->pin, confirmed_ts);
                        b->click_count = 0;
                    } else {
                        b->click_count++;
                    }
                }
            }

            b->setting_mode_armed = false;
            b->over_20s_blocked = false;

            if (pressed_count() == 0) {
                g_combo_long_reported = false;
            }

            if (g_combo_press_start_us != 0 && pressed_count() == 0) {
                int64_t combo_held_us = confirmed_ts - g_combo_press_start_us;

                if (combo_held_us >= SETTING_MODE_MIN_US && combo_held_us < SETTING_MODE_MAX_US) {
                    start_ready_commissioning_window();
                }

                g_combo_press_start_us = 0;
                g_combo_setting_mode_armed = false;
                g_combo_over_20s_blocked = false;
            }
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
                // NEW: the commissioning window can be armed by a different button's
                // combo/long-press while this click sequence was already accumulating
                // (confirm_pending_edges() only checks the flag at each release, not
                // here at final gap-timeout). Without this check a double/multi click
                // that started just before the window opened could still fire once
                // it's open, bypassing the same suppression NUOS relies on elsewhere -
                // route it to the pattern matcher instead, consistent with
                // confirm_pending_edges().
                if (copy_ready_commisioning_flag || ready_commisioning_flag) {
                    detect_6_click_pattern(b->button_id, now);
                    b->click_count = 0;
                    continue;
                }

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

// CHANGED: added esp_task_wdt_reset() (esp_task_wdt.h was already included but unused)
// and confirm_pending_edges() call. Loop shape otherwise unchanged — it was already
// safe against watchdog starvation since xQueueReceive blocks up to 10ms per pass
// rather than busy-draining, so this is a safety net, not a fix for a live bug.
static void switch_driver_button_detected(void *arg)
{
    gpio_evt_t evt;
    static uint32_t last_reported_drops = 0;

    esp_task_wdt_add(NULL);   // NEW: register so we can feed it explicitly below

    while (1) {
        if (xQueueReceive(gpio_evt_queue, &evt, pdMS_TO_TICKS(10)) == pdTRUE) {
            process_edge_event(&evt);
        }
        confirm_pending_edges();   // NEW: promote any debounced-quiet candidates
        process_long_press_and_clicks();

        uint32_t drops = g_isr_queue_drops;
        if (drops != last_reported_drops) {
            ESP_LOGW(TAG, "gpio_evt_queue full: total raw edges dropped=%" PRIu32, drops);
            last_reported_drops = drops;
        }

        esp_task_wdt_reset();      // NEW: explicit feed
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

        // NEW: init confirm-debounce state
        g_btn[i].candidate_time_us = 0;
        g_btn[i].candidate_level = true;
        g_btn[i].candidate_pending = false;
        g_btn[i].bounce_suppressed = false;
        g_btn[i].last_single_click_us = 0;
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

    switch_action_queue = xQueueCreate(ACTION_QUEUE_LEN, sizeof(switch_action_t));
    if (switch_action_queue == NULL) {
        ESP_LOGE(TAG, "Action queue creation failed");
        vQueueDelete(gpio_evt_queue);
        gpio_evt_queue = NULL;
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
            vQueueDelete(switch_action_queue);
            gpio_evt_queue = NULL;
            switch_action_queue = NULL;
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
        vQueueDelete(switch_action_queue);
        gpio_evt_queue = NULL;
        switch_action_queue = NULL;
        return false;
    }

    BaseType_t worker_ok = xTaskCreate(
        switch_worker_task,
        "sw_worker",
        TASK_STACK_SIZE_SWITCH,
        NULL,
        TASK_PRIORITY_WORKER,
        NULL
    );
    if (worker_ok != pdPASS) {
        ESP_LOGE(TAG, "Worker task creation failed");
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

// Retain non-DALI switch driver legacy implementation below
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
static TimerHandle_t click_timer                = NULL;
bool  longpress_detected                        = false;
static int click_count                          = 0;

#define CLICK_ARRAY_SIZE                        50
gpio_num_t switch_num_pressed[CLICK_ARRAY_SIZE];
int press_count_5s[4]                           = {0, 0, 0, 0};
static TimerHandle_t press_count_timer_handle   = NULL;
volatile bool time_5sec_started_flag            = false;

static QueueHandle_t gpio_evt_queue             = NULL;
static switch_func_pair_t *switch_func_pair;
static esp_switch_callback_t func_ptr;
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

extern bool ready_commisioning_flag;

TimerHandle_t ready_commissioning_timer = NULL;

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    switch_driver_gpios_intr_enabled(false); 
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(gpio_evt_queue, (switch_func_pair_t*)arg, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken); 
}

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

static void esp_zb_callback(uint8_t param) {
}

// void button_click_handler(TimerHandle_t xTimer)
// {
//     int local_clicks = click_count;
//     if (local_clicks > CLICK_ARRAY_SIZE) local_clicks = CLICK_ARRAY_SIZE;
//     click_count = 0;

// #ifdef USE_DOUBLE_PRESS
//     switch_func_pair_t button_func_pair;
//     if (xQueueReceive(gpio_evt_queue, &button_func_pair, 0) == pdTRUE) {
//         button_func_pair.func = SWITCH_ONOFF_TOGGLE_CONTROL;
//         if (local_clicks == 1) {
//             if (longpress_detected) {
//                 button_func_pair.keypressed = LONG_PRESS;
//             } else {
//                 button_func_pair.keypressed = SINGLE_PRESS;
//             }
//             longpress_detected = false;
//         } else if (local_clicks == 2) {
//             if (isSceneRemoteBindingStarted) {
//                 task_sequence_num = TASK_MODE_EXIT;
//             }
//             button_func_pair.keypressed = SINGLE_PRESS;
//             for (int i = 1; i < local_clicks; i++) {
//                 if (switch_num_pressed[i] == switch_num_pressed[i - 1]) {
//                     button_func_pair.keypressed = DOUBLE_PRESS;
//                     break;
//                 }
//             }
//         }
//         if (func_ptr != NULL) {
//             (*func_ptr)(&button_func_pair);
//         }
//     }
// #endif
// }
void button_click_handler(TimerHandle_t xTimer)
{

    int local_clicks = click_count;
    if (local_clicks > CLICK_ARRAY_SIZE) local_clicks = CLICK_ARRAY_SIZE;
    click_count = 0;
#ifdef USE_TRIPLE_CLICK
        //printf("click_count:%d\n", local_clicks);
        if (local_clicks == MAX_COUNTS_FOR_TRIPLE_CLICK) {
            // Check if all presses were on the same button
            
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
                } else {

                    #ifdef USE_WIFI_WEBSERVER

                    
                        wifi_webserver_active_flag = true;
                        
                        //#ifdef USE_C3_ADAPTER_UART_HW
                        printf("{\"mode\":%d}\n", wifi_webserver_active_flag);
                        //#else
                        vTaskDelay(pdMS_TO_TICKS(500));
                        setNVSCommissioningFlag(0);
                        setNVSWebServerEnableFlag(wifi_webserver_active_flag);                    
                        esp_restart();	
                        //#endif
                    #endif
                   
                }             
            }
        }else if (local_clicks == 8) {
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
                pdMS_TO_TICKS(TRIPLE_CLICK_MAX_DELAY_MS);
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

// static void switch_driver_button_detected(void *arg) {
//     gpio_num_t io_num = GPIO_NUM_NC;
//     static bool start_comm_flag = false;
//     uint32_t reduced_bounce_time = DEBOUNCE_TIME_MS;
//     static switch_state_t switch_state = SWITCH_IDLE;
//     bool evt_flag = false;
//     uint32_t switch_pressed_cnts = 0;
//     bool two_switch_pressed_flag = false;

//     for (;;) {
//         switch_func_pair_t button_func_pair;
//         if (xQueueReceive(gpio_evt_queue, &button_func_pair, portMAX_DELAY)) {
//             switch_driver_gpios_intr_enabled(false);
//             recheckTimer();
//             io_num = button_func_pair.pin;
//             evt_flag = true;
//             longpress_detected = false;
//             total_press_in_secs = 0;
//             two_switch_pressed_flag = false; 
//             initTwoSwitchPressedPins();
            
//             #ifdef LONG_PRESS_BRIGHTNESS_ENABLE
//                nuos_init_hardware_dimming_up_down(io_num);
//             #endif

//             #ifndef USE_DOUBLE_PRESS
//                 button_func_pair.keypressed = SINGLE_PRESS;
//             #endif

//             brightness_count = 0;
//             switch_pressed_cnts = 0;
//             reduced_bounce_time = DEBOUNCE_TIME_MS;
//         }

//         while (evt_flag) {
//             bool value = gpio_get_level(io_num);
//             uint32_t current_time = esp_timer_get_time() / 1000;
//             global_switch_state = switch_state;
            
//             switch (switch_state) {
//                 case SWITCH_IDLE:
//                     if (value == GPIO_INPUT_LEVEL_ON) {
//                         last_press_time = current_time;
//                         last_release_time = current_time;
//                         double_release_time = current_time;
//                         switch_state = SWITCH_PRESS_DETECTED;
//                     }
//                     break;

//                 case SWITCH_PRESS_DETECTED:
//                     switch_state = (value == GPIO_INPUT_LEVEL_ON) ? SWITCH_PRESS_DETECTED : SWITCH_RELEASE_DETECTED;
//                     press_duration = current_time - last_press_time;
//                     break;

//                 case SWITCH_RELEASE_DETECTED:
//                     switch_state = SWITCH_IDLE;
//                     break;

//                 default:
//                     switch_state = SWITCH_IDLE;
//                     switch_driver_gpios_intr_enabled(true);
//                     break;
//             }

//             if (switch_state == SWITCH_IDLE) {
//                 #ifndef USE_DOUBLE_PRESS
//                     button_func_pair.func = SWITCH_ONOFF_TOGGLE_CONTROL;
//                     button_func_pair.pin = io_num;
//                     if (func_ptr != NULL) {
//                         (*func_ptr)(&button_func_pair);
//                     }
//                 #endif
//                 switch_driver_gpios_intr_enabled(true);
//                 evt_flag = false;
//                 break;
//             }
//             vTaskDelay(pdMS_TO_TICKS(reduced_bounce_time));
//         }
//     }
// }
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
        #if (USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI)
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

    #if defined(USE_DOUBLE_PRESS) || defined(USE_TRIPLE_CLICK)
        if (click_timer == NULL) {
                click_timer = xTimerCreate("Click Timer", pdMS_TO_TICKS(TRIPLE_CLICK_MAX_DELAY_MS), pdFALSE, (void *)0, button_click_handler);
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

                                    button_func_pair.keypressed = LONG_PRESS_INC_DEC_LEVEL;
                                
                                if(wifi_webserver_active_flag){
                                    if(start_commissioning){
                                        start_comm_flag = true;
                                        start_commissioning = false;
                                    }
                                    
                                }
                                brightness_control_tasks(io_num);
                                switch_pressed_cnts = IdentifyTwoSwitchPressed();
                                if (!two_switch_pressed_flag) {
                                        if (switch_pressed_cnts >= 2) {
                                            two_switch_pressed_flag = true;
                                            nuos_set_rgb_led_commissioning_functionality();
                                            #ifdef USE_C3_ADAPTER_UART_HW
                                            //printf("\"time\":%ld\n", switch_pressed_cnts);
                                            #else
                                            printf("2 switch pressed!!\n");
                                            #endif
                                        }
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
    if (gpio_evt_queue == NULL) {
        return false;
    }

    xTaskCreate(switch_driver_button_detected, "button_detected", TASK_STACK_SIZE_SWITCH, NULL, TASK_PRIORITY_SWITCH, NULL);

    if (!isr_service_installed) {
        ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT));
        isr_service_installed = true;
    }
   
    for (int i = 0; i < button_num; ++i) {
        gpio_isr_handler_add((button_func_pair + i)->pin, gpio_isr_handler, (void *)(button_func_pair + i));
    }
    return true;
}

bool switch_driver_init(switch_func_pair_t *button_func_pair, uint8_t button_num, esp_switch_callback_t cb)
{
    if (!switch_driver_gpio_init(button_func_pair, button_num)) {
        return false;
    }
    func_ptr = cb;
    create_timers_at_init();
    return true;
}

#endif