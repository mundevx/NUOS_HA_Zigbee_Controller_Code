#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Button event types
typedef enum {
    BUTTON_EVENT_SINGLE_CLICK,
    BUTTON_EVENT_DOUBLE_CLICK,
    BUTTON_EVENT_LONG_PRESS,
    BUTTON_EVENT_MULTI_LONG_PRESS,
} button_event_t;

// Callback type for button events
typedef void (*button_callback_t)(void *arg, uint32_t button_id, button_event_t event);

// Configuration for a single button
typedef struct {
    uint32_t pin;               // GPIO number
    uint8_t  active_level;      // 0 for active low (pulled up), 1 for active high
    uint32_t debounce_ms;       // debounce time in milliseconds (e.g., 50)
    uint32_t long_press_ms;     // time to consider a press "long" (e.g., 1000)
    uint32_t double_click_ms;   // max time between clicks for double click (e.g., 300)
    button_callback_t callback; // optional: called for events from this button
    void *callback_arg;         // user argument for the callback
} button_config_t;

// Configuration for a multi‑button long press combination
// typedef struct {
//     uint64_t button_mask;       // bit mask of button IDs (0..63) that must be pressed
//     uint32_t long_press_ms;     // duration all buttons must be held (e.g., 2000)
//     button_callback_t callback; // called when the combination is long pressed
//     void *callback_arg;
// } button_combo_t;

// Initialize the button driver.
// @param buttons      array of button_config_t
// @param num_buttons  number of entries in buttons[]
// @param combos       array of button_combo_t (can be NULL)
// @param num_combos   number of combos (0 if combos == NULL)
// @return true on success, false on failure (queue/task creation failed)
bool button_driver_init(const button_config_t *buttons, uint32_t num_buttons,
                        const button_combo_t *combos, uint32_t num_combos);

#ifdef __cplusplus
}
#endif