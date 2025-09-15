#include "app_hardware_driver.h"

static const char *TAG = "HW_DRIVER";

// Common GPIO initialization
void gpio_init_output(uint32_t pin) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(pin, 0);
}

// Common LEDC PWM initialization
void pwm_init_channel(ledc_channel_t channel, ledc_timer_t timer, gpio_num_t gpio) {
    ledc_channel_config_t ledc_channel = {
        .gpio_num = gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel,
        .timer_sel = timer,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void set_led_common(uint8_t index, bool on) {
    uint32_t pin = gpio_touch_led_pins(index); // Assume this function maps index to GPIO pin
    gpio_set_level(pin, on ? 1 : 0);
}
void set_load_common(uint8_t index, bool on) {
    uint32_t pin = gpio_load_pins[index];
    gpio_set_level(pin, on ? 1 : 0);
}
// Common NVS storage
void nuos_store_data_to_nvs_common(uint8_t index) {
    // Implementation for storing device state to NVS
    ESP_LOGD(TAG, "Storing data for device %d to NVS", index);
}