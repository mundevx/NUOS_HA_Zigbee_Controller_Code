
#include "app_hardware_driver.h"
#include "esp_timer.h"
#include "app_zigbee_misc.h"

#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH  || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)

    // uint8_t user_switch_state = SWITCH_IDLE;
    // bool switch_detected = true;
    // static esp_switch_callback_t func_ptr;
    extern esp_err_t nuos_set_scene_button_attribute(uint8_t index);

    void nuos_zb_init_hardware(){
        for(int index=0; index<TOTAL_ENDPOINTS; index++){
            gpio_reset_pin(gpio_touch_led_pins[index]);
            /* Set the GPIO as a push/pull output */
            gpio_set_direction(gpio_touch_led_pins[index], GPIO_MODE_OUTPUT);
            gpio_set_level(gpio_touch_led_pins[index], 0);

            gpio_reset_pin(gpio_load_pins[index]);
            /* Set the GPIO as a push/pull output */
            gpio_set_direction(gpio_load_pins[index], GPIO_MODE_OUTPUT);
            gpio_set_level(gpio_load_pins[index], 0);
        }
    }
       
    static bool state = false;
    void nuos_on_off_led(uint8_t index, uint8_t _state){
        gpio_set_level(gpio_touch_led_pins[index], _state);
    }

    void nuos_toggle_leds(uint8_t index){
        state = !state;       
        nuos_on_off_led(index, state);
    }

    void nuos_zb_set_hardware(uint8_t index, uint8_t is_toggle){
        // #ifdef USE_RGB_LED
        //     nuos_rgb_trigger_blink();
        // #endif 
    }

    void nuos_zb_set_scene_switch_click(uint32_t io_num, uint8_t state){
        int index = nuos_get_button_press_index(io_num);
        //set touch led pins
        gpio_set_level(gpio_touch_led_pins[index], state);
    }

    void nuos_zb_set_hardware_led_for_zb_commissioning(uint8_t is_toggle){
        #ifdef USE_RGB_LED
            nuos_toggle_rgb_led();
        #else
            if(TOTAL_LEDS >= TOTAL_LEDS_SHOW_ON_COMMISSIONING){
                if(is_toggle>0) {
                    state = !state;
                    for(int index=TOTAL_LEDS-TOTAL_LEDS_SHOW_ON_COMMISSIONING; index<TOTAL_LEDS; index++){
                        gpio_set_level(gpio_touch_led_pins[index], state);
                    }
                }else{
                    for(int i=0; i<TOTAL_LEDS; i++){
                        device_info[i].device_state = 0; 
                        nuos_zb_set_hardware(i, false);
                    }
                }
            }
        #endif
    }

    void nuos_set_state_touch_leds(bool state){

    }    
 #endif