
#include "app_hardware_driver.h"
#include "app_zigbee_clusters.h"

#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_GAS_LEAK)

    #include <Arduino.h>
    #include "AGS02MA.h"


    AGS02MA AGS(26);

    static bool sensor_detected_flag = false;
    static bool sensor_reactivated_flag = false;
    volatile bool led_on_at_startup_flag = true;
    uint32_t led_on_at_startup_timeout = 0;
    uint32_t led_on_at_startup_timeout_value = 50; //5 seconds

    volatile bool pir_motion_disable_flag = true;
    int32_t pir_motion_disable_timeout_counts = 0;
    uint32_t pir_motion_disable_timeout_value = 600; //1 minute
    uint8_t comm_counts = 0;

    /***********************************************
     * TVOC(ppb)	Scale	Description	            Colour
        <= 220	    1	    Good	                Green
        <= 660	    3	    Moderate	            Yellow
        <= 1430	    7	    Bad	                    Orange
        <= 2200	    10	    Unhealthy	            Red
        <= 3300	    15	    Very unhealthy	        Purple
        <= 5500	    25	    Hazardous	            Deep Purple
        > 5500	    50	    Extremely Hazardous	    Deep Purple
     * 
     * 
     */

    
    static bool gas_leak_detected = false;
    static uint8_t unpair_count = 0;

    uint16_t u16_delay = 3000;
    extern "C" void sensor_task(void *pvParameters) {
		while (1) {
			vTaskDelay(pdMS_TO_TICKS(5000));
            if(is_my_device_commissionned){
                vTaskDelay(pdMS_TO_TICKS(u16_delay));
                if(sensor_reactivated_flag){
                    uint32_t value = AGS.readPPB();
                    printf("PPB:%ld\t\n", value);
                    // if(!led_on_at_startup_flag && !pir_motion_disable_flag){
                    if(value >= GAS_LEAK_SENSOR_PEAK_DETECTED_VALUE){
                        printf("Gas Leak Detected!!\n");
                        
                        if(!gas_leak_detected){
                            gas_leak_detected = true;
                            printf("RED ON\n");
                            nuos_set_zigbee_attribute(1);
                            
                            gpio_set_direction(gpio_touch_led_pins[0], GPIO_MODE_OUTPUT);
                            gpio_set_level(gpio_touch_led_pins[0], 0); //blue off

                            gpio_set_direction(gpio_touch_led_pins[1], GPIO_MODE_OUTPUT);
                            gpio_set_level(gpio_touch_led_pins[1], 1); //red on

                            gpio_set_direction(gpio_load_pins[0], GPIO_MODE_OUTPUT);
                            gpio_set_level(gpio_load_pins[0], 1);                         
                        }
                    }else{
                        gas_leak_detected = false;
                        gpio_set_direction(gpio_touch_led_pins[0], GPIO_MODE_OUTPUT);
                        gpio_set_level(gpio_touch_led_pins[0], 1); //blue on

                        gpio_set_direction(gpio_touch_led_pins[1], GPIO_MODE_OUTPUT);
                        gpio_set_level(gpio_touch_led_pins[1], 0); //red off

                        gpio_set_direction(gpio_load_pins[0], GPIO_MODE_OUTPUT);
                        gpio_set_level(gpio_load_pins[0], 0);       
                    }
                }
            }else{
                if(unpair_count++ >= 2){
                    unpair_count = 0;
                    gpio_set_direction(gpio_touch_led_pins[0], GPIO_MODE_OUTPUT);
                    gpio_set_level(gpio_touch_led_pins[0], 0);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    gpio_set_level(gpio_touch_led_pins[0], 1);
                }       
            }	
		}
    }
    extern "C" void init_buzzer_commissioning_task(){
        gpio_set_level(gpio_touch_led_pins[0], 1);
        comm_counts = getNVSSensorsCommissioningCounts();
        comm_counts++;
        delay(2000);
        gpio_set_level(gpio_touch_led_pins[0], 0);
        setNVSSensorsCommissioningCounts(comm_counts);
        delay(2000);
        gpio_set_level(gpio_touch_led_pins[0], 1);
        setNVSSensorsCommissioningCounts(0);
        // read_timeout_value();
        init_sensor_interrupt();
        if(is_my_device_commissionned){
            gpio_set_level(gpio_touch_led_pins[0], 0);
        }
    }

    extern "C" void nuos_zb_init_hardware(){
        //  ESP devices typically miss the first serial log lines after flashing.
        //  Delay somewhat to include all output.
        // delay(1000);
        for(int index=0; index<TOTAL_LEDS; index++){
            gpio_reset_pin(gpio_touch_led_pins[index]);
            // /* Set the GPIO as a push/pull output */
            gpio_set_direction(gpio_touch_led_pins[index], GPIO_MODE_OUTPUT);
            gpio_set_level(gpio_touch_led_pins[index], 1);
        }  

        gpio_reset_pin(BUZZER_PIN);
        gpio_set_direction(BUZZER_PIN, GPIO_MODE_OUTPUT);

        for(int index=0; index<TOTAL_LOADS; index++){
            gpio_reset_pin(gpio_load_pins[index]);
            /* Set the GPIO as a push/pull output */
            gpio_set_direction(gpio_load_pins[index], GPIO_MODE_OUTPUT);
            gpio_set_level(gpio_load_pins[index], 0);
        }

        gpio_set_direction(BUZZER_PIN, GPIO_MODE_OUTPUT);

        gpio_set_direction(gpio_touch_led_pins[1], GPIO_MODE_OUTPUT);
        gpio_set_level(gpio_touch_led_pins[1], 1);
        gpio_set_direction(gpio_touch_led_pins[0], GPIO_MODE_OUTPUT);
        gpio_set_level(gpio_touch_led_pins[0], 1);        

        led_on_at_startup_flag = true; 

        init_timer();
        esp_start_timer();

    }

    extern "C" void init_sensor_commissioning_task(){
        init_buzzer_commissioning_task();
        // //read last stored timeout value from nvs
        // read_timeout_value();
    }

    static bool state = false;
    extern "C" void nuos_zb_set_hardware_led_for_zb_commissioning(uint8_t is_toggle){
        // if(is_toggle>0) {
        //     state = !state;
        //     for(int index=0; index<TOTAL_LEDS; index++){
        //         gpio_set_level(gpio_touch_led_pins[index], state);
        //     }
        // }else{
        //     for(int i=0; i<TOTAL_ENDPOINTS; i++){
        //         nuos_zb_set_hardware(i, false);
        //     }
        // }
    }

    extern "C" void nuos_zb_set_hardware(uint8_t index, uint8_t is_toggle){
        // //toggle pins on button press
        if(is_toggle>0) device_info[index].device_state = !device_info[index].device_state;
        //set touch led pins
        gpio_set_direction(gpio_touch_led_pins[index], GPIO_MODE_OUTPUT);
        gpio_set_level(gpio_touch_led_pins[index], device_info[index].device_state);
        //set load pins
        gpio_set_level(gpio_load_pins[index], device_info[index].device_state);

        nuos_store_data_to_nvs(index);
    }
    void nuos_on_off_led(uint8_t index, uint8_t _state){
        gpio_set_level(gpio_touch_led_pins[index], _state);
    }

    void nuos_toggle_leds(uint8_t index){
        state = !state;       
        nuos_on_off_led(index, state);
    }
    // extern "C" void nuos_esp_motion_sensor_check_in_timer(){
    //     // // Code to execute when the timer expires
        
    //     #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_GAS_LEAK)
    //         if(led_on_at_startup_flag){
    //             if(led_on_at_startup_timeout++ >= led_on_at_startup_timeout_value){
    //                 led_on_at_startup_flag = false;
    //                 led_on_at_startup_timeout = 0;
                    
    //                 printf("BLUE ON\n");             
    //                 //turn off red led
    //                 // uint32_t value = AGS.readPPB();
    //                 // printf("PPB_START:%ld\t\n", value);
    //                 // if(value < GAS_LEAK_SENSOR_PEAK_DETECTED_VALUE){
    //                 //     gpio_set_direction(gpio_touch_led_pins[1], GPIO_MODE_OUTPUT);
    //                 //     gpio_set_level(gpio_touch_led_pins[1], 0);
    //                 //     //turn on blue led
    //                 //     gpio_set_direction(gpio_touch_led_pins[0], GPIO_MODE_OUTPUT);
    //                 //     gpio_set_level(gpio_touch_led_pins[0], 1);
    //                 // }
    //                 init_sensor_interrupt();
    //             }
    //         }
    //         if(pir_motion_disable_flag){
    //             if(pir_motion_disable_timeout_counts++ >= pir_motion_disable_timeout_value){

    //                 uint32_t value = AGS.readPPB();
    //                 printf("PPB_START:%ld\t\n", value);
    //                 if(value >= GAS_LEAK_SENSOR_PEAK_DETECTED_VALUE){
    //                     pir_motion_disable_timeout_value = 600; //1 minute
    //                     pir_motion_disable_timeout_counts = 0;
    //                     printf("Still Warming up.. Please wait\n");
    //                 }else{
    //                     pir_motion_disable_flag = false;
    //                     pir_motion_disable_timeout_counts = 0;

    //                     printf("BLUE ON\n");
    //                     //turn off red led
    //                     gpio_set_direction(gpio_touch_led_pins[1], GPIO_MODE_OUTPUT);
    //                     gpio_set_level(gpio_touch_led_pins[1], 0);
    //                     //turn on blue led
    //                     gpio_set_direction(gpio_touch_led_pins[0], GPIO_MODE_OUTPUT);
    //                     gpio_set_level(gpio_touch_led_pins[0], 1); 
    //                     sensor_reactivated_flag = true;

    //                 }


    //             }
    //             store_timeout_value();
    //         }
    //     #endif
    // }



    void i2c_scanner() {
        byte error, address;
        int nDevices = 0;
        printf("Scanning...\n");
        for (address = 1; address < 127; address++) {
            Wire.beginTransmission(address);
            error = Wire.endTransmission();
            if (error == 0) {
                printf("I2C device found at address 0x%x\n", address);
                nDevices++;
            } else if (error == 4) {
                printf("Unknown error at address 0x%x\n", address);
            }
        }

    }

    extern "C" void init_sensor_interrupt(){
        //------------------>START AGS TASK
        Wire.begin(I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
        i2c_scanner();
        bool b = AGS.begin();
        printf("BEGIN:%d\t", b);

        printf("VERSION:%d\t", AGS.getSensorVersion());

        //  pre-heating improves measurement quality
        //  can be skipped
        label:
        gpio_set_level(gpio_touch_led_pins[1], 1);
        vTaskDelay(pdMS_TO_TICKS(20000));  //warming up time 20 seconds
        
        b = AGS.setPPBMode();
        uint8_t m = AGS.getMode();
        printf("MODE:%d\t", b);
        printf("%d\t\n", m);

        uint8_t version = AGS.getSensorVersion();
        printf("VERS:%d\t\n", version);

        uint32_t value = AGS.readPPB();
        printf("PPB_START_1:%ld\t\n", value);
        if(value > GAS_LEAK_SENSOR_PEAK_DETECTED_VALUE){
            gpio_set_direction(gpio_touch_led_pins[1], GPIO_MODE_OUTPUT);
            gpio_set_level(gpio_touch_led_pins[1], 1);
            goto label;
        }else{
            gpio_set_level(gpio_touch_led_pins[1], 0);
        }
        xTaskCreate(sensor_task, "EspSensorTask", 4096, NULL, 22, NULL);
    }
    extern "C" void nuos_set_state_touch_leds(bool state){
        //turn on blue led
        gpio_set_direction(gpio_touch_led_pins[0], GPIO_MODE_OUTPUT);
        gpio_set_level(gpio_touch_led_pins[0], 1);
    }
#endif




