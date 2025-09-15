
#include "app_hardware_driver.h"
#include "esp_timer.h"
#include "app_zigbee_misc.h"
#include "app_zigbee_clusters.h"
#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RINGING_BELL_2)
    extern esp_err_t nuos_set_sensor_motion_attribute(uint8_t value);

    void nuos_zb_init_hardware(){
        for(int index=0; index<TOTAL_LEDS; index++){
            gpio_reset_pin(gpio_touch_led_pins[index]);
            /* Set the GPIO as a push/pull output */
            gpio_set_direction(gpio_touch_led_pins[index], GPIO_MODE_OUTPUT);
            gpio_set_level(gpio_touch_led_pins[index], 0); 
        }

        for(int index=0; index<TOTAL_LOADS; index++){
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
        // device_info[index].device_state = true;
        // gpio_set_level(gpio_touch_led_pins[index], device_info[index].device_state);
        // gpio_set_level(gpio_load_pins[0], device_info[0].device_state);
        // if(is_my_device_commissionned){
        //     nuos_set_sensor_motion_attribute(1);  //to fast update switches on app 
        // }    
        // vTaskDelay(pdMS_TO_TICKS(500));
        // device_info[index].device_state = false;
        // gpio_set_level(gpio_touch_led_pins[index], device_info[index].device_state);
        // gpio_set_level(gpio_load_pins[0], device_info[0].device_state);
        // if(is_my_device_commissionned){
        //     nuos_set_sensor_motion_attribute(0);  //to fast update switches on app 
        // }        
        #ifdef USE_RGB_LED
            nuos_rgb_trigger_blink();
        #endif 
    }

    void nuos_zb_set_scene_switch_click(uint32_t io_num, uint8_t state){
        int index = nuos_get_button_press_index(io_num);
        //set touch led pins
        // printf("Switch index:%d  state:%d\n", index, state);
        gpio_set_level(gpio_touch_led_pins[index], state);
        gpio_set_level(gpio_load_pins[index], state);
        if(is_my_device_commissionned){
            nuos_set_sensor_motion_attribute(state);  //to fast update switches on app
        }        
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