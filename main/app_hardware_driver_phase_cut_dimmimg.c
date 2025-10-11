#include "app_hardware_driver.h"
 #include "app_zigbee_clusters.h"
#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_PHASE_CUT_DIMMABLE_LIGHT)
    #include "esp_log.h"
    #include "esp_err.h"
    #include <esp_log.h>
    #include <esp_check.h>

    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "freertos/semphr.h"
   
    #include "driver/ledc.h"
    #include "driver/gpio.h"

    #include "esp_timer.h"


    #define USE_NVS_STORE

    
    bool is_init_done = false;


    /*********************************************************************** */
    



    // main.c - ESP-IDF port of the two-channel phase-cut dimmer with buttons and LED indicators
// Target: ESP32-H2
// Build: idf.py build

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "sdkconfig.h"

static const char *TAG = "phase_cut_idf";

/* ---------- User pins (change to match your board) ---------- */
#define ZERO_CROSS_PIN   10
#define TRIAC_PIN_1      2
#define TRIAC_PIN_2      22
#define BUTTON_PIN_1     0
#define BUTTON_PIN_2     1
#define LED_PIN_1        27
#define LED_PIN_2        11

/* ---------- Phase cut config ---------- */
static const uint32_t AC_WAVE_MICROS = 9000U; // half-cycle microseconds ~50Hz
static const uint32_t MIN_ENABLED_R = 5U;     // percent
static const uint32_t MAX_R_CLAMP = 80U;      // percent
static const uint32_t PULSE_WIDTH_US = 50U;   // triac gate pulse width

/* ---------- Button/ramp config ---------- */
// static const uint32_t DEBOUNCE_MS = 30U;
// static const uint32_t LONG_PRESS_MS = 800U;
// static const uint32_t RAMP_INTERVAL_MS = 120U;
// static const float RAMP_STEP = 0.02f;
static const float BRIGHTNESS_MIN = 0.05f;
// static const float BRIGHTNESS_MAX = 1.0f;
static const uint8_t LED_PWM_RESOLUTION = 8;  // 8-bit
static const uint32_t LED_PWM_FREQ = 1000;    // 1kHz
static const uint8_t DEFAULT_RESTORE_BRIGHTNESS_8BIT = 128U;

/* ---------- LEDC (LEDC_LOW_SPEED_MODE for H2) ---------- */
static const ledc_mode_t LEDC_MODE = LEDC_LOW_SPEED_MODE;
static const ledc_timer_t LEDC_TIMER = LEDC_TIMER_0;
static const ledc_channel_t LEDC_CH_1 = LEDC_CHANNEL_0;
static const ledc_channel_t LEDC_CH_2 = LEDC_CHANNEL_1;

/* ---------- State (shared) ---------- */
static volatile bool zeroCrossingFlag = false;

/* Channel 1 state */
static volatile uint32_t delayForPulse_us_1 = AC_WAVE_MICROS;
static int triacGpio1 = -1;
static esp_timer_handle_t delayTimer1 = NULL;
static esp_timer_handle_t clearTimer1 = NULL;
static volatile bool stopped1 = true;
static float currentBrightness1 = 0.0f; // 0..1
// static bool logicalOn1 = false;
static uint8_t last_brightness_1 = DEFAULT_RESTORE_BRIGHTNESS_8BIT;

/* Channel 2 state */
static volatile uint32_t delayForPulse_us_2 = AC_WAVE_MICROS;
static int triacGpio2 = -1;
static esp_timer_handle_t delayTimer2 = NULL;
static esp_timer_handle_t clearTimer2 = NULL;
static volatile bool stopped2 = true;
static float currentBrightness2 = 0.0f;
// static bool logicalOn2 = false;
static uint8_t last_brightness_2 = DEFAULT_RESTORE_BRIGHTNESS_8BIT;

/* Button structures (polled in a task) */
typedef struct {
    gpio_num_t pin;
    bool active_low;
    bool last_reading;
    uint32_t last_bounce_ms;
    uint32_t press_start_ms;
    bool pressed;            // debounced state
    bool long_press_active;
    uint32_t last_ramp_ms;
} button_t;

static button_t btn1 = { .pin = BUTTON_PIN_1, .active_low = true, .last_reading = 1, .last_bounce_ms = 0, .press_start_ms = 0, .pressed = false, .long_press_active = false, .last_ramp_ms = 0 };
static button_t btn2 = { .pin = BUTTON_PIN_2, .active_low = true, .last_reading = 1, .last_bounce_ms = 0, .press_start_ms = 0, .pressed = false, .long_press_active = false, .last_ramp_ms = 0 };

/* LEDC last duty caches to avoid redundant writes */
static int led_last_duty_ch1 = -1;
static int led_last_duty_ch2 = -1;

/* ---------- Helper prototypes ---------- */
static void ledc_init_safe(void);
static void ledc_write_safe(ledc_channel_t ch, uint8_t value8);
static float map8bit_to_float(uint8_t v);
static unsigned long brightness_to_delay_us(float r);
static void set_period1(float r);
static void set_period2(float r);
static void stop_both(void);

// simple clamp helper (replacement for Arduino constrain)
static inline int clamp_i(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}


/* ---------- esp_timer callbacks ---------- */
static void delayTimerCb1(void *arg) {
    // set triac gate
    gpio_set_level(triacGpio1, 1);
    if (clearTimer1) {
        esp_timer_start_once(clearTimer1, PULSE_WIDTH_US);
    }
}
static void clearTimerCb1(void *arg) {
    gpio_set_level(triacGpio1, 0);
}

static void delayTimerCb2(void *arg) {
    gpio_set_level(triacGpio2, 1);
    if (clearTimer2) {
        esp_timer_start_once(clearTimer2, PULSE_WIDTH_US);
    }
}
static void clearTimerCb2(void *arg) {
    gpio_set_level(triacGpio2, 0);
}

/* ---------- Zero-cross ISR ---------- */
static void IRAM_ATTR zero_cross_isr(void* arg)
{
    zeroCrossingFlag = true;
    // schedule timers only if channel enabled
    if (!stopped1 && delayTimer1) {
        // esp_timer_start_once is safe to call from ISR context
        esp_timer_start_once(delayTimer1, delayForPulse_us_1);
    }
    if (!stopped2 && delayTimer2) {
        esp_timer_start_once(delayTimer2, delayForPulse_us_2);
    }
}

/* ---------- Brightness mapping ---------- */
static float map8bit_to_float(uint8_t v) {
    return (float)v / 255.0f;
}

static unsigned long brightness_to_delay_us(float r) {
    if (r < 0.0f) r = 0.0f;
    if (r > 1.0f) r = 1.0f;
    if (r > ((float)MAX_R_CLAMP / 100.0f)) r = ((float)MAX_R_CLAMP / 100.0f);
    const float gamma = 2.2f;
    float corrected = powf(r, gamma);
    unsigned long delay_us = (unsigned long)((1.0f - corrected) * AC_WAVE_MICROS);
    if (delay_us > AC_WAVE_MICROS) delay_us = AC_WAVE_MICROS;
    return delay_us;
}

/* ---------- Triac period setters ---------- */
static void set_period1(float r) {
    if (r < 0.0f) {
        r = 0.0f;
    }
    if (r > 1.0f) {
        r = 1.0f;
    }
    if (r < ((float)MIN_ENABLED_R / 100.0f) || !device_info[0].device_state) {
        stopped1 = true;
        if (delayTimer1) esp_timer_stop(delayTimer1);
        if (clearTimer1) esp_timer_stop(clearTimer1);
        gpio_set_level(triacGpio1, 0);
        currentBrightness1 = 0.0f;
        return;
    } else {
        stopped1 = false;
    }
    unsigned long delay_us = brightness_to_delay_us(r);
    delayForPulse_us_1 = delay_us;
    currentBrightness1 = r;
}

static void set_period2(float r) {
    if (r < 0.0f) {
        r = 0.0f;
    }
    if (r > 1.0f) {
        r = 1.0f;
    }
    if (r < ((float)MIN_ENABLED_R / 100.0f) || !device_info[1].device_state) {
        stopped2 = true;
        if (delayTimer2) esp_timer_stop(delayTimer2);
        if (clearTimer2) esp_timer_stop(clearTimer2);
        gpio_set_level(triacGpio2, 0);
        currentBrightness2 = 0.0f;
        return;
    } else {
        stopped2 = false;
    }
    unsigned long delay_us = brightness_to_delay_us(r);
    delayForPulse_us_2 = delay_us;
    currentBrightness2 = r;
}

static void __attribute__((unused))  stop_both(void) {
    stopped1 = true; stopped2 = true;
    if (delayTimer1) esp_timer_stop(delayTimer1);
    if (clearTimer1) esp_timer_stop(clearTimer1);
    if (delayTimer2) esp_timer_stop(delayTimer2);
    if (clearTimer2) esp_timer_stop(clearTimer2);
    gpio_set_level(triacGpio1, 0);
    gpio_set_level(triacGpio2, 0);
}

/* ---------- LEDC helpers (safe write) ---------- */
static void ledc_init_safe(void)
{
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = (ledc_timer_bit_t)LED_PWM_RESOLUTION,
        .timer_num = LEDC_TIMER,
        .freq_hz = LED_PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    ledc_channel_config_t ch1 = {
        .gpio_num = LED_PIN_1,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CH_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch1));

    ledc_channel_config_t ch2 = {
        .gpio_num = LED_PIN_2,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CH_2,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch2));

    led_last_duty_ch1 = -1;
    led_last_duty_ch2 = -1;
}

static void ledc_write_safe(ledc_channel_t ch, uint8_t value8)
{
    uint32_t maxduty = ((1u << LED_PWM_RESOLUTION) - 1u);
    uint32_t duty_scaled = (value8 * maxduty + 127) / 255; // rounded

    int *cache = NULL;
    if (ch == LEDC_CH_1) cache = &led_last_duty_ch1;
    else if (ch == LEDC_CH_2) cache = &led_last_duty_ch2;

    if (cache != NULL) {
        if ((int)duty_scaled == *cache) return;
        *cache = (int)duty_scaled;
    }

    // set + update
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, ch, duty_scaled));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, ch));
}

/* ---------- External APIs ---------- */
/* set_brightness(channel, value8) channel: 0=both,1,2 ; value: 0..255
   semantics: value==0 -> turn off channel (logicalOn=false) but store last_brightness
*/
void set_brightness(uint8_t channel, uint8_t value)
{
    if (channel > 2) return;
    if (channel == 1 || channel == 0) {
        last_brightness_1 = value;
        float mapped = map8bit_to_float(value);
        printf("mapped:%.2f\n", mapped);
        if (value == 0) {
            // device_info[0].device_state = false;
            currentBrightness1 = 0.0f;
            set_period1(0.0f);
            ledc_write_safe(LEDC_CH_1, 0);
        } else {
            currentBrightness1 = mapped;
            // if (device_info[0].device_state) {
                float apply = mapped;
                if (apply < BRIGHTNESS_MIN) apply = BRIGHTNESS_MIN;
                set_period1(apply);
                int v = (int) roundf(apply * 255.0f);
                v = clamp_i(v, 0, 255);
                ledc_write_safe(LEDC_CH_1, (uint8_t)v);
            // } else {
            //      ledc_write_safe(LEDC_CH_1, 0);
            // }
        }
    }
    if (channel == 2 || channel == 0) {
        last_brightness_2 = value;
        float mapped = map8bit_to_float(value);
        if (value == 0) {
            //device_info[1].device_state = false;
            currentBrightness2 = 0.0f;
            set_period2(0.0f);
            ledc_write_safe(LEDC_CH_2, 0);
        } else {
            currentBrightness2 = mapped;
            // if (device_info[1].device_state) {
                float apply = mapped;
                if (apply < BRIGHTNESS_MIN) apply = BRIGHTNESS_MIN;
                set_period2(apply);
                int v = (int) roundf(apply * 255.0f);
                v = clamp_i(v, 0, 255);
                ledc_write_safe(LEDC_CH_2, (uint8_t)v);
            // } else {
            //      ledc_write_safe(LEDC_CH_2, 0);
            // }
        }
    }
}

static bool state_change_1 = false, state_change_2 = false;
/* set_onoff(channel, on) - restore last_brightness when turning on */
void set_onoff(uint8_t channel, bool on)
{
    if (channel > 2) return;
    if (channel == 1 || channel == 0) {
        //device_info[0].device_state = true;
        if (on) {
            if(last_brightness_1 != device_info[0].device_level || state_change_1){
                last_brightness_1 = device_info[0].device_level;   
                state_change_1 = false;         
                uint8_t v = device_info[0].device_level; //last_brightness_1;
                if (v == 0) v = DEFAULT_RESTORE_BRIGHTNESS_8BIT;
                float mapped = map8bit_to_float(v);
                if (mapped < BRIGHTNESS_MIN) mapped = BRIGHTNESS_MIN;
                currentBrightness1 = mapped;
                set_period1(mapped);
                int v4 = (int) roundf(mapped * 255.0f);
                v4 = clamp_i(v4, 0, 255);
                ledc_write_safe(LEDC_CH_1, (uint8_t)v4);
            }
        } else {
            state_change_1 = true;
            set_period1(0.0f);
            ledc_write_safe(LEDC_CH_1, 0);
        }
    }
    if (channel == 2 || channel == 0) {
        //device_info[1].device_state = true;
        if (on) {
            if(last_brightness_2 != device_info[1].device_level || state_change_2){
                last_brightness_2 = device_info[1].device_level;
                state_change_2 = false;
                uint8_t v = device_info[1].device_level; //last_brightness_2;
                if (v == 0) v = DEFAULT_RESTORE_BRIGHTNESS_8BIT;
                float mapped = map8bit_to_float(v);
                if (mapped < BRIGHTNESS_MIN) mapped = BRIGHTNESS_MIN;
                currentBrightness2 = mapped;
                set_period2(mapped);
                int v1 = (int) roundf(mapped * 255.0f);
                v1 = clamp_i(v1, 0, 255);
                ledc_write_safe(LEDC_CH_2, (uint8_t)v1);
            }
        } else {
            state_change_2 = true;
            set_period2(0.0f);
            ledc_write_safe(LEDC_CH_2, 0);
        }
    }
}

// /* ---------- Button task ---------- */
// static void button_task(void *arg)
// {
//     (void)arg;
//     while (1) {
//         uint32_t now = (uint32_t)(esp_timer_get_time() / 1000ULL); // ms

//         // process single button
//         button_t *b = &btn1;
//         bool raw = gpio_get_level(b->pin);
//         bool is_pressed_now = b->active_low ? (raw == 0) : (raw == 1);

//         if (is_pressed_now != b->last_reading) {
//             b->last_bounce_ms = now;
//             b->last_reading = is_pressed_now;
//         }

//         if ((now - b->last_bounce_ms) > DEBOUNCE_MS) {
//             if (is_pressed_now != b->pressed) {
//                 b->pressed = is_pressed_now;
//                 if (b->pressed) {
//                     b->press_start_ms = now;
//                     b->long_press_active = false;
//                     b->last_ramp_ms = now;
//                 } else {
//                     uint32_t pressDuration = now - b->press_start_ms;
//                     if (!b->long_press_active && pressDuration >= DEBOUNCE_MS && pressDuration < LONG_PRESS_MS) {
//                         // short press -> toggle channel 1
//                         logicalOn1 = !logicalOn1;
//                         if (!logicalOn1) {
//                             set_period1(0.0f);
//                             ledc_write_safe(LEDC_CH_1, 0);
//                         } else {
//                             uint8_t v = last_brightness_1;
//                             if (v == 0) v = DEFAULT_RESTORE_BRIGHTNESS_8BIT;
//                             float mapped = map8bit_to_float(v);
//                             if (mapped < BRIGHTNESS_MIN) mapped = BRIGHTNESS_MIN;
//                             currentBrightness1 = mapped;
//                             set_period1(mapped);
//                             int v2 = (int) roundf(mapped * 255.0f);
//                             v2 = clamp_i(v2, 0, 255);
//                             ledc_write_safe(LEDC_CH_1, (uint8_t)v);
//                         }
//                     }
//                     b->long_press_active = false;
//                 }
//             } else {
//                 if (b->pressed && !b->long_press_active) {
//                     if ((now - b->press_start_ms) >= LONG_PRESS_MS) {
//                         b->long_press_active = true;
//                         logicalOn1 = true;
//                         if (currentBrightness1 < BRIGHTNESS_MIN) currentBrightness1 = BRIGHTNESS_MIN;
//                         //ledc_write_safe(LEDC_CH_1, (uint8_t)constrain(roundf(currentBrightness1*255.0f),0,255));
//     int v = (int) roundf(currentBrightness1 * 255.0f);
//     v = clamp_i(v, 0, 255);
//     ledc_write_safe(LEDC_CH_1, (uint8_t)v);                        
//                     }
//                 } else if (b->pressed && b->long_press_active) {
//                     if ((now - b->last_ramp_ms) >= RAMP_INTERVAL_MS) {
//                         b->last_ramp_ms = now;
//                         currentBrightness1 += RAMP_STEP;
//                         if (currentBrightness1 > BRIGHTNESS_MAX) currentBrightness1 = BRIGHTNESS_MIN;
//                         float apply = currentBrightness1;
//                         if (apply < BRIGHTNESS_MIN) apply = BRIGHTNESS_MIN;
//                         set_period1(apply);
//                         //last_brightness_1 = (uint8_t)constrain(roundf(apply*255.0f),0,255);
//     int v = (int) roundf(apply * 255.0f);
//     last_brightness_1 = clamp_i(v, 0, 255);
//     ledc_write_safe(LEDC_CH_1, last_brightness_1);                         
//                         ledc_write_safe(LEDC_CH_1, last_brightness_1);
//                     }
//                 }
//             }
//         }

//         // process second button (copy of above with channel 2)
//         b = &btn2;
//         raw = gpio_get_level(b->pin);
//         is_pressed_now = b->active_low ? (raw == 0) : (raw == 1);

//         if (is_pressed_now != b->last_reading) {
//             b->last_bounce_ms = now;
//             b->last_reading = is_pressed_now;
//         }

//         if ((now - b->last_bounce_ms) > DEBOUNCE_MS) {
//             if (is_pressed_now != b->pressed) {
//                 b->pressed = is_pressed_now;
//                 if (b->pressed) {
//                     b->press_start_ms = now;
//                     b->long_press_active = false;
//                     b->last_ramp_ms = now;
//                 } else {
//                     uint32_t pressDuration = now - b->press_start_ms;
//                     if (!b->long_press_active && pressDuration >= DEBOUNCE_MS && pressDuration < LONG_PRESS_MS) {
//                         // short press -> toggle channel 2
//                         logicalOn2 = !logicalOn2;
//                         if (!logicalOn2) {
//                             set_period2(0.0f);
//                             ledc_write_safe(LEDC_CH_2, 0);
//                         } else {
//                             uint8_t v = last_brightness_2;
//                             if (v == 0) v = DEFAULT_RESTORE_BRIGHTNESS_8BIT;
//                             float mapped = map8bit_to_float(v);
//                             if (mapped < BRIGHTNESS_MIN) mapped = BRIGHTNESS_MIN;
//                             currentBrightness2 = mapped;
//                             set_period2(mapped);
//                             int v5 = (int) roundf(mapped * 255.0f);
//                             v5 = clamp_i(v5, 0, 255);
//                             ledc_write_safe(LEDC_CH_2, (uint8_t)v5);
//                         }
//                     }
//                     b->long_press_active = false;
//                 }
//             } else {
//                 if (b->pressed && !b->long_press_active) {
//                     if ((now - b->press_start_ms) >= LONG_PRESS_MS) {
//                         b->long_press_active = true;
//                         logicalOn2 = true;
//                         if (currentBrightness2 < BRIGHTNESS_MIN) currentBrightness2 = BRIGHTNESS_MIN;
//                         // ledc_write_safe(LEDC_CH_2, (uint8_t)constrain(roundf(currentBrightness2*255.0f),0,255));
//     int v = (int) roundf(currentBrightness2 * 255.0f);
//     v = clamp_i(v, 0, 255);
//     ledc_write_safe(LEDC_CH_2, (uint8_t)v);                         
//                     }
//                 } else if (b->pressed && b->long_press_active) {
//                     if ((now - b->last_ramp_ms) >= RAMP_INTERVAL_MS) {
//                         b->last_ramp_ms = now;
//                         currentBrightness2 += RAMP_STEP;
//                         if (currentBrightness2 > BRIGHTNESS_MAX) currentBrightness2 = BRIGHTNESS_MIN;
//                         float apply = currentBrightness2;
//                         if (apply < BRIGHTNESS_MIN) apply = BRIGHTNESS_MIN;
//                         set_period2(apply);
//                         //last_brightness_2 = (uint8_t)constrain(roundf(apply*255.0f),0,255);
//     int v = (int) roundf(apply * 255.0f);
//     last_brightness_2 = clamp_i(v, 0, 255);
//     ledc_write_safe(LEDC_CH_2, last_brightness_2);                          
//                         // ledc_write_safe(LEDC_CH_2, last_brightness_2);
//                     }
//                 }
//             }
//         }

//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }

/* ---------- Setup / Initialization ---------- */
void init_phasecut(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_LOGI(TAG, "Starting phase-cut dimmer (ESP-IDF) ...");

    // configure triac pins
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TRIAC_PIN_1) | (1ULL << TRIAC_PIN_2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    triacGpio1 = TRIAC_PIN_1;
    triacGpio2 = TRIAC_PIN_2;
    gpio_set_level(triacGpio1, 0);
    gpio_set_level(triacGpio2, 0);

    // configure zero-cross pin and ISR (RISING)
    gpio_config_t zc_conf = {
        .pin_bit_mask = (1ULL << ZERO_CROSS_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };
    ESP_ERROR_CHECK(gpio_config(&zc_conf));
    // install ISR service
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(ZERO_CROSS_PIN, zero_cross_isr, NULL));

    // create esp_timers for triac pulses (one-shot)
    esp_timer_create_args_t dt1 = {
        .callback = &delayTimerCb1,
        .arg = NULL,
        .name = "triac_delay1"
    };
    ESP_ERROR_CHECK(esp_timer_create(&dt1, &delayTimer1));
    esp_timer_create_args_t ct1 = {
        .callback = &clearTimerCb1,
        .arg = NULL,
        .name = "triac_clear1"
    };
    ESP_ERROR_CHECK(esp_timer_create(&ct1, &clearTimer1));

    esp_timer_create_args_t dt2 = {
        .callback = &delayTimerCb2,
        .arg = NULL,
        .name = "triac_delay2"
    };
    ESP_ERROR_CHECK(esp_timer_create(&dt2, &delayTimer2));
    esp_timer_create_args_t ct2 = {
        .callback = &clearTimerCb2,
        .arg = NULL,
        .name = "triac_clear2"
    };
    ESP_ERROR_CHECK(esp_timer_create(&ct2, &clearTimer2));

    // // configure buttons as inputs with pullups
    // gpio_config_t btn_conf = {
    //     .pin_bit_mask = (1ULL << BUTTON_PIN_1) | (1ULL << BUTTON_PIN_2),
    //     .mode = GPIO_MODE_INPUT,
    //     .pull_up_en = GPIO_PULLUP_ENABLE,
    //     .pull_down_en = GPIO_PULLDOWN_DISABLE,
    //     .intr_type = GPIO_INTR_DISABLE
    // };
    // ESP_ERROR_CHECK(gpio_config(&btn_conf));

    // LEDC PWM init
    ledc_init_safe();

    // // start button task
    // BaseType_t ok = xTaskCreate(button_task, "button_task", 4*1024, NULL, tskIDLE_PRIORITY + 1, NULL);
    // if (ok != pdPASS) {
    //     ESP_LOGE(TAG, "Failed to create button_task");
    // }

    // initial values: triacs off
    // logicalOn1 = false;
    // logicalOn2 = false;
    currentBrightness1 = map8bit_to_float(device_info[0].device_level);
    currentBrightness2 = map8bit_to_float(device_info[1].device_level);
    set_period1(0.0f);
    set_period2(0.0f);

    ESP_LOGI(TAG, "Initialization complete.");
    // app_main returns; FreeRTOS tasks keep running
}


/************************************************************************************ */
    void init_fading(){

    }

    void nuos_zb_init_hardware(){
        init_phasecut();
        is_init_done = true;             
    }


    void nuos_on_off_led(uint8_t index, uint8_t _state){
        if(is_init_done){
           set_onoff(index+1, _state);
        }
    }

    static bool _state_ = false;
    void nuos_toggle_leds(uint8_t index){
        _state_ = !_state_;
        set_onoff(index+1, _state_);

    }

    // void set_level_value(uint8_t _level){
    //     set_brightness(1, _level);
    // }

    void nuos_zb_set_hardware_led_for_zb_commissioning(uint8_t is_toggle){

    }

    void set_harware(uint8_t index, uint8_t is_toggle){
        if(is_toggle>0) device_info[index].device_state = !device_info[index].device_state;
        if(is_init_done){
            
                if(!device_info[index].device_state){
                    set_onoff(index+1, false);
                    //set_brightness(uint8_t channel, uint8_t value)
                }else{
                    // if(device_info[index].device_level >= MIN_DIM_LEVEL_VALUE){
                        set_onoff(index+1, true);
                    // }else{
                    //     set_onoff(index+1, false);
                    // }
                }

        }
        
        #ifdef USE_RGB_LED
            nuos_rgb_trigger_blink();
        #endif               
    }

    void set_leds(int i, bool state){
        if(is_init_done){
            // if(i == 0){
            //     if(gpio_touch_led_pins[0] == gpio_load_pins[0]){
            //         if(state) ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[0], device_info[i].device_level));
            //         else ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[0], 0));
            //         ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[0]));
            //     }else{
            //         if(state) ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], device_info[i].device_level));
            //         else ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], 0));
            //         ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[i]));
            //     }
            // }else{
            //     if(state) ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], device_info[i].device_level));
            //     else ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], 0));
            //     ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[i]));
            // }
        }
    }
    void nuos_zb_set_hardware(uint8_t index, uint8_t is_toggle){

        //set touch led pins
        if(timer3_running_flag){
            //toggle pins on button press
            set_harware(index, is_toggle);
        }else{
            // if(nuos_check_state_touch_leds()){
            //     for(int i=0; i<TOTAL_LEDS; i++){
            //         set_leds(i, device_info[i].device_state);                       
            //     }
            // }else{
            //     //toggle pins on button press
                set_harware(index, is_toggle);               
            // }
        }

        #ifdef USE_NVS_STORE          
        nuos_store_data_to_nvs(index);
        #endif
    }

    void nuos_set_state_touch_leds(bool state){
        for(int i=0; i<TOTAL_LEDS; i++){
             set_leds(i, state);
        }
    }

    // bool nuos_check_state_touch_leds(){
    //     bool getting_on_state = false;
    //     if(touchLedsOffAfter1MinuteEnable){
    //         for(int i=0; i<TOTAL_LEDS; i++){
    //             if(device_info[i].device_state){
    //                 getting_on_state = true;
                    
    //             }
    //         }
    //     }
    //     return getting_on_state;
    // }

    void nuos_init_hardware_dimming_up_down(uint32_t pin){
        uint8_t index = nuos_get_button_press_index(pin);
        if(device_info[index].device_state){
            if(device_info[index].device_level <= MIN_DIM_LEVEL_VALUE){
                device_info[index].dim_up = 1;
            }else if(device_info[index].device_level >= MAX_DIM_LEVEL_VALUE){
                device_info[index].dim_up = 0;
            }
        }
    }

    static uint8_t level_backup[4] = {0, 0, 0, 0};

    void set_load(uint8_t index, uint8_t value){
        if(is_init_done){
            // set_onoff(index+1, false);
            set_brightness(index+1, value);
        }
        // #ifdef USE_NVS_STORE 
        // nuos_store_data_to_nvs(index);
        // #endif        
    }

    bool nuos_set_hardware_brightness(uint32_t pin){
        uint8_t index = nuos_get_button_press_index(pin);
        if(global_switch_state == SWITCH_PRESS_DETECTED){ 
            if(!device_info[index].device_state){
                device_info[index].device_state = true;
                device_info[index].device_level = MIN_DIM_LEVEL_VALUE;
                device_info[index].dim_up = 1;
            }
            if(device_info[index].device_state){
                uint8_t last_dim =  device_info[index].device_level;
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
                    //printf("Set state:%d, Load:%d\n", device_info[index].device_state, level_backup[index]);
                    set_load(index, device_info[index].device_level);
                }             
            }
        }
        return false;
    }
#endif