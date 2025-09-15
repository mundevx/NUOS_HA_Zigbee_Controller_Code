
#include "app_hardware_driver.h"
#include "esp_zigbee_core.h"
#include "app_zigbee_misc.h"

#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4R_ON_OFF_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2R_ON_OFF_LIGHT)
    bool nuos_check_state_touch_leds();
    static bool state = false;
    
    void nuos_zb_init_hardware(){

        uint32_t pins = 0; 
        for(int i=0; i<TOTAL_LEDS; i++){
            pins |= 1ULL << gpio_touch_led_pins[i];
        }  
        for(int j=0; j<TOTAL_LOADS; j++){
            pins |= 1ULL << gpio_load_pins[j];
        }         
        gpio_config_t io_conf;
        // Configure the output pin
        io_conf.intr_type = GPIO_INTR_DISABLE;       // Disable interrupt
        io_conf.mode = GPIO_MODE_OUTPUT;             // Set as output mode
        io_conf.pin_bit_mask = pins;                 // Bit mask for the output pin
        io_conf.pull_down_en = 0;                    // Disable pull-down mode
        io_conf.pull_up_en = 1;                      // Disable pull-up mode
        gpio_config(&io_conf);

        // for(int index=0; index<TOTAL_LEDS; index++) {
        //     gpio_reset_pin(gpio_touch_led_pins[index]);
        //     /* Set the GPIO as a push/pull output */
        //     gpio_set_direction(gpio_touch_led_pins[index], GPIO_MODE_OUTPUT);
        //     gpio_set_level(gpio_touch_led_pins[index], 0);
        // }   

        // for(int index_2=0; index_2<TOTAL_LOADS; index_2++) {
        //     gpio_reset_pin(gpio_load_pins[index_2]);
        //     /* Set the GPIO as a push/pull output */
        //     gpio_set_direction(gpio_load_pins[index_2], GPIO_MODE_OUTPUT);
        //     gpio_set_level(gpio_load_pins[index_2], 0);
        // }               
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
                    for(int i=0; i<TOTAL_ENDPOINTS; i++){
                        nuos_zb_set_hardware(i, false);
                    }
                }
            }
        #endif
    }


    void nuos_on_off_led(uint8_t index, uint8_t _state){
        if(index < TOTAL_LEDS)
        gpio_set_level(gpio_touch_led_pins[index], _state);
    }

    void nuos_toggle_leds(uint8_t index){
        state = !state;       
        nuos_on_off_led(index, state);
    }

    void set_harware(uint8_t index, uint8_t is_toggle){
        //toggle pins on button press
        if(is_toggle>0) device_info[index].device_state = !device_info[index].device_state;
        //set led pins
        if(index < TOTAL_LEDS){
            //printf("STATE:%d\n", device_info[index].device_state);
            //gpio_set_direction(gpio_touch_led_pins[index], GPIO_MODE_OUTPUT);
            gpio_set_level(gpio_touch_led_pins[index], device_info[index].device_state);
            //set load pins
            //gpio_set_direction(gpio_load_pins[index], GPIO_MODE_OUTPUT);
            gpio_set_level(gpio_load_pins[index], device_info[index].device_state);
        } 
        printf("STARE:%d\n", device_info[index].device_state);
        #ifdef USE_RGB_LED
            nuos_rgb_trigger_blink();
        #endif          
    }

    void nuos_zb_set_hardware(uint8_t index, uint8_t is_toggle){
        //set touch led pins

        if(timer3_running_flag){
            set_harware(index, is_toggle);
        }else{
            if(nuos_check_state_touch_leds()){
                for(int i=0; i<TOTAL_LEDS; i++){
                    gpio_set_level(gpio_touch_led_pins[i], device_info[i].device_state);
                }
            }else{
                set_harware(index, is_toggle);               
            }
        }      
        nuos_store_data_to_nvs(index);
    }

    void nuos_set_state_touch_leds(bool state){
        for(int i=0; i<TOTAL_LEDS; i++){
            gpio_set_level(gpio_touch_led_pins[i], state);
        }
    }

    bool nuos_check_state_touch_leds(){
        bool getting_on_state = false;
        if(touchLedsOffAfter1MinuteEnable){
            for(int i=0; i<TOTAL_LEDS; i++){
                if(device_info[i].device_state){
                    getting_on_state = true;
                    
                }
            }
        }
        return getting_on_state;
    }
 #endif