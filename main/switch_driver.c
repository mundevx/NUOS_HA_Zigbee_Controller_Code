#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "app_hardware_driver.h"

#define TAG "SWITCH_DRV"

#define MAX_BUTTONS                  4
#define GPIO_EVT_QUEUE_LEN           32
#define TASK_STACK_SIZE_SWITCH       4096
#define TASK_PRIORITY_SWITCH         10

#define DEBOUNCE_US                  30000      // 30 ms
#define LONG_PRESS_US                700000     // 700 ms
#define MULTI_CLICK_GAP_US           250000     // 250 ms
#define COMBO_LONG_PRESS_US          800000     // 800 ms

typedef enum {
    SWITCH_EVENT_SINGLE_CLICK = 1,
    SWITCH_EVENT_DOUBLE_CLICK,
    SWITCH_EVENT_MULTI_CLICK,
    SWITCH_EVENT_LONG_PRESS,
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

//typedef void (*esp_switch_callback_t)(const switch_event_info_t *evt);

typedef struct {
    uint8_t button_id;
    gpio_num_t pin;

    bool stable_level;               // 1=released, 0=pressed (pull-up button)
    bool pressed;
    bool long_reported;

    int64_t last_edge_us;
    int64_t press_start_us;
    int64_t release_time_us;

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
static uint32_t g_pressed_mask = 0;
static bool g_combo_long_reported = false;

static inline int find_button_index(gpio_num_t pin)
{
    for (int i = 0; i < switch_num; i++) {
        if (gpio_touch_btn_pins[i] == pin) return i;
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
    // example mappings
    // button IDs: 0,1,2,3

    if (combo_mask == ((1U << 0) | (1U << 1))) {
        return SWITCH_FACTORY_RESET_CONTROL;
    }
    if (combo_mask == ((1U << 2) | (1U << 3))) {
        return SWITCH_SCENE_CHANGE_CONTROL;
    }

    return SWITCH_NOTHING_CONTROL;
}

// switch_event_info_t
static void emit_event(uint8_t button_id,
                       gpio_num_t pin,
                       switch_event_t event,
                       uint8_t click_count,
                       uint32_t combo_mask,
                       int64_t ts_us)
{
    if (!func_ptr) return;
    int idx = find_button_index(pin);
    switch_event_info_t evt = {
        .button_id = button_id,
        .pin = pin,
        .event = event,
        .click_count = click_count,
        .combo_mask = combo_mask,
        .timestamp_us = ts_us
    };
    // switch_func_pair_t button_func_pair;
    // button_func_pair.id = button_id;
    // button_func_pair.pin = (gpio_num_t)pin;
    (void)evt;

    switch_func_pair_t out = {0};
    out.id = button_id;
    out.pin = pin;
    out.keypressed = 0xFF;
    out.func = 0xFF;

    // if (event == SWITCH_EVENT_COMBO_LONG_PRESS) {
    //     out.keypressed = LONG_PRESS;
    //     out.func = SWITCH_ONOFF_TOGGLE_CONTROL;
    //     func_ptr(&out);
    //     return;
    // }
if (event == SWITCH_EVENT_COMBO_LONG_PRESS) {
    out.keypressed = LONG_PRESS;
    out.func = get_combo_func_from_mask(combo_mask);

    if (out.func != SWITCH_NOTHING_CONTROL) {
        func_ptr(&out);
    }
    return;
}
    if (idx < 0) {
        return;
    }    
    switch(event){
        case SWITCH_EVENT_SINGLE_CLICK:
        printf("SWITCH_EVENT_SINGLE_CLICK\n");
        out.func = SWITCH_ONOFF_TOGGLE_CONTROL;
        out.keypressed = SINGLE_PRESS;
        break;
        case SWITCH_EVENT_DOUBLE_CLICK:
        printf("SWITCH_EVENT_DOUBLE_CLICK\n");
        out.func = SWITCH_ONOFF_TOGGLE_CONTROL;
        out.keypressed = DOUBLE_PRESS;
        break;
        case SWITCH_EVENT_MULTI_CLICK:
        printf("SWITCH_EVENT_MULTI_CLICK\n");
        out.func = SWITCH_ONOFF_TOGGLE_CONTROL;
        out.keypressed = SINGLE_PRESS;
        break;
        case SWITCH_EVENT_LONG_PRESS:
        printf("SWITCH_EVENT_LONG_PRESS\n");
        out.func = SWITCH_ONOFF_TOGGLE_CONTROL;
        out.keypressed = LONG_PRESS;
        break;
        case SWITCH_EVENT_COMBO_LONG_PRESS:
        printf("SWITCH_EVENT_COMBO_LONG_PRESS\n");
        out.func = SWITCH_ONOFF_TOGGLE_CONTROL;
        out.keypressed = LONG_PRESS;
        break;                        
        default: 
        out.keypressed = 0xff; //no key or wrong key pressed
        break;
    }
    if (out.func != 0xFF) {
        func_ptr(&out);
    }
}

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    const switch_func_pair_t *btn = (const switch_func_pair_t *)arg;

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

static void process_edge_event(const gpio_evt_t *evt)
{
    int idx = find_button_index(evt->pin);
    printf("IDX:%d\n", idx);
    if (idx < 0) return;

    button_state_t *b = &g_btn[idx];

    if ((evt->isr_time_us - b->last_edge_us) < DEBOUNCE_US) {
        return;
    }
    b->last_edge_us = evt->isr_time_us;

    bool new_level = evt->level ? true : false;   // pull-up: 1 release, 0 press
    if (new_level == b->stable_level) {
        return;
    }

    b->stable_level = new_level;

    if (new_level == false) {
        b->pressed = true;
        b->long_reported = false;
        b->press_start_us = evt->isr_time_us;
        g_pressed_mask |= (1U << b->button_id);
        g_combo_long_reported = false;
////////////////////////
    } else {
        b->pressed = false;
        b->release_time_us = evt->isr_time_us;

        if (!b->long_reported) {
            uint32_t ev = switch_func_pair[idx].enabled_events;

            // Direct single-click buttons: no waiting window
            if ((ev & BTN_EVT_SINGLE) && !(ev & BTN_EVT_DOUBLE)) {
                emit_event(b->button_id, b->pin,
                           SWITCH_EVENT_SINGLE_CLICK, 1, 0, evt->isr_time_us);
                b->click_count = 0;
            } else {
                // Timed buttons: single/double decision later
                b->click_count++;
            }
        }

        if (pressed_count() == 0) {
            g_combo_long_reported = false;
        }
    }
    // } else {
    //     b->pressed = false;
    //     g_pressed_mask &= ~(1U << b->button_id);
    //     b->release_time_us = evt->isr_time_us;

    //     if (!b->long_reported) {
    //         b->click_count++;
    //     }

    //     if (pressed_count() == 0) {
    //         g_combo_long_reported = false;
    //     }
    // }
}

static void process_long_press_and_clicks(void)
{
    int64_t now = esp_timer_get_time();

    for (int i = 0; i < switch_num; i++) {
        button_state_t *b = &g_btn[i];

        if (b->pressed && !b->long_reported) {
            if ((now - b->press_start_us) >= LONG_PRESS_US) {
                b->long_reported = true;
                b->click_count = 0;
                emit_event(b->button_id, b->pin,
                           SWITCH_EVENT_LONG_PRESS, 0, 0, now);
            }
        }

        // if (!b->pressed && b->click_count > 0) {
        //     if ((now - b->release_time_us) >= MULTI_CLICK_GAP_US) {
        //         if (b->click_count == 1) {
        //             emit_event(b->button_id, b->pin,
        //                        SWITCH_EVENT_SINGLE_CLICK, 1, 0, now);
        //         } else if (b->click_count == 2) {
        //             emit_event(b->button_id, b->pin,
        //                        SWITCH_EVENT_DOUBLE_CLICK, 2, 0, now);
        //         } else {
        //             emit_event(b->button_id, b->pin,
        //                        SWITCH_EVENT_MULTI_CLICK, b->click_count, 0, now);
        //         }
        //         b->click_count = 0;
        //     }
        // }
        if (!b->pressed && b->click_count > 0) {
            if ((now - b->release_time_us) >= MULTI_CLICK_GAP_US) {
                uint32_t ev = switch_func_pair[i].enabled_events;

                if (b->click_count == 1) {
                    if (ev & BTN_EVT_SINGLE) {
                        emit_event(b->button_id, b->pin,
                                SWITCH_EVENT_SINGLE_CLICK, 1, 0, now);
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

    if (!g_combo_long_reported && pressed_count() >= 2) {
        int64_t oldest_press = now;
        uint32_t combo_mask = current_pressed_mask();

        for (int i = 0; i < switch_num; i++) {
            if (g_btn[i].pressed && g_btn[i].press_start_us < oldest_press) {
                oldest_press = g_btn[i].press_start_us;
            }
        }

        if ((now - oldest_press) >= COMBO_LONG_PRESS_US) {
            g_combo_long_reported = true;
            emit_event(0xFF, GPIO_NUM_NC,
                       SWITCH_EVENT_COMBO_LONG_PRESS, 0, combo_mask, now);

            for (int i = 0; i < switch_num; i++) {
                if (g_btn[i].pressed) {
                    g_btn[i].long_reported = true;
                    g_btn[i].click_count = 0;
                }
            }
        }
    }
}

static void switch_driver_button_detected(void *arg)
{
    gpio_evt_t evt;

    while (1) {
        if (xQueueReceive(gpio_evt_queue, &evt, pdMS_TO_TICKS(10))) {
            process_edge_event(&evt);
        }

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
        printf("PIN:%d   - ", g_btn[i].pin);
        g_btn[i].stable_level = true;
        g_btn[i].pressed = false;
        g_btn[i].long_reported = false;
        g_btn[i].last_edge_us = 0;
        g_btn[i].press_start_us = 0;
        g_btn[i].release_time_us = 0;
        g_btn[i].click_count = 0;
    }
    printf("\n");
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

bool switch_driver_init(switch_func_pair_t *button_func_pair, uint8_t button_num, esp_switch_callback_t cb)
{
    func_ptr = cb;
    return switch_driver_gpio_init(button_func_pair, button_num);
}