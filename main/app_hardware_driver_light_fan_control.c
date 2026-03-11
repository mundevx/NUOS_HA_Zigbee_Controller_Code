
#include "app_hardware_driver.h"
#include "app_zigbee_misc.h"

#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN_CUSTOM)
    void nuos_set_hardware_fan_touch_leds_ctrl_only(uint8_t index);
    bool nuos_check_state_touch_leds();

	void nuos_zb_init_hardware(){

		for(int index=0; index<TOTAL_LEDS; index++){
            gpio_reset_pin(gpio_touch_led_pins[index]);
            // /* Set the GPIO as a push/pull output */
            gpio_set_direction(gpio_touch_led_pins[index], GPIO_MODE_OUTPUT);
            gpio_set_level(gpio_touch_led_pins[index], 0);
        } 

		for(int j=0; j<TOTAL_LOADS; j++){
            gpio_reset_pin(gpio_load_pins[j]);
            /* Set the GPIO as a push/pull output */
            gpio_set_direction(gpio_load_pins[j], GPIO_MODE_OUTPUT);
            gpio_set_level(gpio_load_pins[j], 0);

        } 		
	}

    static bool state = false;
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

    void set_hardware(uint8_t index, uint8_t is_toggle) {
		if(index == LIGHT_INDEX) {
            if(is_toggle>0) device_info[LIGHT_INDEX].device_state = !device_info[LIGHT_INDEX].device_state;
            //set touch led pins
            gpio_set_level(gpio_touch_led_pins[0], device_info[LIGHT_INDEX].device_state);
            //set load pins
            gpio_set_level(gpio_load_pins[0], device_info[LIGHT_INDEX].device_state);
			nuos_store_data_to_nvs(LIGHT_INDEX);
		} else if(index == FAN_INDEX) {
			if(is_toggle>0) device_info[FAN_INDEX].device_state = !device_info[FAN_INDEX].device_state;
			nuos_set_hardware_fan_ctrl(FAN_INDEX);
			//nuos_store_data_to_nvs(FAN_INDEX);
		} else {
			if(device_info[FAN_INDEX].device_state){
				if(index == 2) {
					if(device_info[FAN_INDEX].fan_speed < 4) device_info[FAN_INDEX].fan_speed++;
					device_info[FAN_INDEX].dim_up = 1;
				}else if(index == 3) {
					if(device_info[FAN_INDEX].fan_speed > 1) device_info[FAN_INDEX].fan_speed--;
					device_info[FAN_INDEX].dim_up = 0; 
				}
			}
			device_info[FAN_INDEX].device_level = fan_speed_values[device_info[FAN_INDEX].fan_speed];
			nuos_set_hardware_fan_ctrl(FAN_INDEX);
			//nuos_store_data_to_nvs(FAN_INDEX);
			printf("Fan Speed: %d, Device Level: %d\n", device_info[FAN_INDEX].fan_speed, device_info[FAN_INDEX].device_level);
		}
        #ifdef USE_RGB_LED
            nuos_rgb_trigger_blink(); 
        #endif         
    }

    void nuos_on_off_led(uint8_t index, uint8_t _state){
        gpio_set_level(gpio_touch_led_pins[index], _state);
    }

    void nuos_toggle_leds(uint8_t index){
        state = !state;       
        nuos_on_off_led(index, state);
    }

	void set_all_leds_to_original_state(){
		for(int i=0; i<TOTAL_ENDPOINTS; i++){
			if(i == LIGHT_INDEX) {
				//set touch led pins
				gpio_set_level(gpio_touch_led_pins[0], device_info[LIGHT_INDEX].device_state);
			} else if(i == FAN_INDEX) {
				nuos_set_hardware_fan_touch_leds_ctrl_only(i);
			}
		}
	}

	void nuos_zb_set_hardware(uint8_t index, uint8_t is_toggle){
        //set touch led pins
		call_common_check_auto_off();
		set_hardware(index, is_toggle);
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
			for(int i=0; i<TOTAL_ENDPOINTS; i++){
				if(device_info[i].device_state){
					getting_on_state = true;
					break;
				}
			}
		}
        return getting_on_state;
    }

	void nuos_set_hardware_fan_ctrl(uint8_t index){
        
		if(!device_info[index].device_state){
			gpio_set_level(gpio_load_pins[1], 0); //fan on-off load out
			gpio_set_level(gpio_load_pins[2], 0);
			gpio_set_level(gpio_load_pins[3], 0);

			gpio_set_level(gpio_touch_led_pins[1], 0); //fan on-off touch led
			gpio_set_level(gpio_touch_led_pins[2], 0); //fan inc touch led
			gpio_set_level(gpio_touch_led_pins[3], 0); //fan dec touch led
		}else{
			//Set LEDs
			gpio_set_level(gpio_touch_led_pins[1], 1);  //fan on-off led
			//Set GPIOs
			if(device_info[index].fan_speed == 0){
				gpio_set_level(gpio_load_pins[1], 0); //1
				gpio_set_level(gpio_load_pins[2], 0);
				gpio_set_level(gpio_load_pins[3], 0);
				//LEDs
				gpio_set_level(gpio_touch_led_pins[2], 0); //1
				gpio_set_level(gpio_touch_led_pins[3], 0);			
			}else if(device_info[index].fan_speed == 1){
				gpio_set_level(gpio_load_pins[1], 1);
				gpio_set_level(gpio_load_pins[2], 0);
				gpio_set_level(gpio_load_pins[3], 0);
				//LEDs
				gpio_set_level(gpio_touch_led_pins[2], 1);
				gpio_set_level(gpio_touch_led_pins[3], 0);
			}else if(device_info[index].fan_speed == 2){
				gpio_set_level(gpio_load_pins[1], 0);
				gpio_set_level(gpio_load_pins[2], 1);
				gpio_set_level(gpio_load_pins[3], 0);
				//LEDs
				gpio_set_level(gpio_touch_led_pins[2], 1);
				gpio_set_level(gpio_touch_led_pins[3], 1);				
			}else if(device_info[index].fan_speed == 3){
				gpio_set_level(gpio_load_pins[1], 1);
				gpio_set_level(gpio_load_pins[2], 1);
				gpio_set_level(gpio_load_pins[3], 0);
				//LEDs
				gpio_set_level(gpio_touch_led_pins[2], 1);
				gpio_set_level(gpio_touch_led_pins[3], 1);				
			}else if(device_info[index].fan_speed == 4){
				gpio_set_level(gpio_load_pins[1], 0);
				gpio_set_level(gpio_load_pins[2], 0);
				gpio_set_level(gpio_load_pins[3], 1);
				//LEDs
				gpio_set_level(gpio_touch_led_pins[2], 0);
				gpio_set_level(gpio_touch_led_pins[3], 1);			
			}
		}
		nuos_store_data_to_nvs(index);
	}

	void nuos_set_hardware_fan_touch_leds_ctrl_only(uint8_t index){
		if(device_info[index].fan_speed == 0){
			if(device_info[index].device_state == 0){
				gpio_set_level(gpio_touch_led_pins[1], 0); //fan on-off touch led
				gpio_set_level(gpio_touch_led_pins[2], 0); //fan inc touch led
				gpio_set_level(gpio_touch_led_pins[3], 0); //fan dec touch led
			}
		}else{
			if(device_info[index].device_state == 1){
			//Set LEDs
			gpio_set_level(gpio_touch_led_pins[1], 1);  //fan on-off led
			//Set GPIOs
			if(device_info[index].fan_speed == 1){
				//LEDs
				gpio_set_level(gpio_touch_led_pins[2], 1);
				gpio_set_level(gpio_touch_led_pins[3], 0);
			}else if(device_info[index].fan_speed == 2){
				//LEDs
				gpio_set_level(gpio_touch_led_pins[2], 1);
				gpio_set_level(gpio_touch_led_pins[3], 1);				
			}else if(device_info[index].fan_speed == 3){
				//LEDs
				gpio_set_level(gpio_touch_led_pins[2], 1);
				gpio_set_level(gpio_touch_led_pins[3], 1);				
			}else if(device_info[index].fan_speed == 4){
				//LEDs
				gpio_set_level(gpio_touch_led_pins[2], 0);
				gpio_set_level(gpio_touch_led_pins[3], 1);				
			}
			}
		}
	}	
#else
    #if(USE_NUOS_ZB_DEVICE_TYPE != DEVICE_IR_BLASTER)
		void nuos_set_hardware_fan_ctrl(uint8_t index){
			
		}	
	#endif
#endif	
