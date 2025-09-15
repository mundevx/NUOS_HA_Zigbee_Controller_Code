
#include "app_hardware_driver.h"
#include "app_zigbee_clusters.h"

#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_LUX)

    #include <Arduino.h>

    #include <Wire.h>
    #include <Adafruit_Sensor.h>
    #include <Adafruit_TSL2561_U.h>

    Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

    static bool sensor_detected_flag = false;
    static bool sensor_reactivated_flag = false;


    volatile bool led_on_at_startup_flag = true;
    uint32_t led_on_at_startup_timeout = 0;
    uint32_t led_on_at_startup_timeout_value = 50; //5 seconds

    volatile bool pir_motion_disable_flag = true;
    int32_t pir_motion_disable_timeout_counts = 0;
    uint32_t pir_motion_disable_timeout_value = 600; //1 minute
    uint8_t comm_counts = 0;

    
    static bool gas_leak_detected = false;
    uint16_t u16_delay = 3000;

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
    static uint8_t unpair_count = 0;
    static bool lux_sensor_error = false;  
    
    extern "C" void sensor_task(void *pvParameters) {
      
		while (1) {
            if(lux_sensor_error){
                gpio_set_level(gpio_touch_led_pins[0], 0);
                vTaskDelay(pdMS_TO_TICKS(100));
                gpio_set_level(gpio_touch_led_pins[0], 1);   
                vTaskDelay(pdMS_TO_TICKS(100));   
            }else{
                if(is_my_device_commissionned){
                    
                    /* Get a new sensor event */ 
                    sensors_event_t event;
                    tsl.getEvent(&event);
                    /* Display the results (light is measured in lux) */
                    if (event.light){
                        printf("%.1f", event.light); printf(" lux\n");

                        uint16_t lux = (uint16_t)(event.light);
                        if(device_info[0].device_val != lux){
                            device_info[0].device_val = lux;
                            nuos_set_zigbee_attribute(0);
                        }
                    }else{
                        /* If event.light = 0 lux the sensor is probably saturated
                        and no reliable data could be generated! */
                        printf("Sensor overload\n");
                    }       
                }else{
                    if(unpair_count++ >= 5){
                        unpair_count = 0;
                        gpio_set_direction(gpio_touch_led_pins[0], GPIO_MODE_OUTPUT);
                        gpio_set_level(gpio_touch_led_pins[0], 0);
                        vTaskDelay(pdMS_TO_TICKS(500));
                        gpio_set_level(gpio_touch_led_pins[0], 1);
                    }
                }                
                vTaskDelay(pdMS_TO_TICKS(2000));
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
        for(int index=0; index<TOTAL_LEDS; index++){
            gpio_reset_pin(gpio_touch_led_pins[index]);
            // /* Set the GPIO as a push/pull output */
            gpio_set_direction(gpio_touch_led_pins[index], GPIO_MODE_OUTPUT);
            gpio_set_level(gpio_touch_led_pins[index], 0);
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
        gpio_set_level(gpio_touch_led_pins[1], 0);
        gpio_set_direction(gpio_touch_led_pins[0], GPIO_MODE_OUTPUT);
        gpio_set_level(gpio_touch_led_pins[0], 0);        

        led_on_at_startup_flag = true; 

    }

    extern "C" void init_sensor_commissioning_task(){
        init_buzzer_commissioning_task();
        //read last stored timeout value from nvs
    }
    
    static bool state = false;
    void nuos_zb_set_hardware_led_for_zb_commissioning(uint8_t is_toggle){
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

    extern "C" void nuos_esp_motion_sensor_check_in_timer(){
        // // Code to execute when the timer expires
            // if(led_on_at_startup_flag){
            //     if(led_on_at_startup_timeout++ >= led_on_at_startup_timeout_value){
            //         led_on_at_startup_flag = false;
            //         led_on_at_startup_timeout = 0;
            //         printf("RED ON\n");              
            //         //turn on red led
            //         gpio_set_direction(gpio_touch_led_pins[1], GPIO_MODE_OUTPUT);
            //         gpio_set_level(gpio_touch_led_pins[1], 0);
            //         //turn off blue led
            //         gpio_set_direction(gpio_touch_led_pins[0], GPIO_MODE_OUTPUT);
            //         gpio_set_level(gpio_touch_led_pins[0], 1);

            //         
            //     }
            // }
            // if(pir_motion_disable_flag){
            //     if(pir_motion_disable_timeout_counts++ >= pir_motion_disable_timeout_value){
            //         pir_motion_disable_flag = false;
            //         pir_motion_disable_timeout_counts = 0;
            //         printf("BLUE ON\n");
            //         //turn off red led
            //         gpio_set_direction(gpio_touch_led_pins[1], GPIO_MODE_OUTPUT);
            //         gpio_set_level(gpio_touch_led_pins[1], 1);
            //         //turn on blue led
            //         gpio_set_direction(gpio_touch_led_pins[0], GPIO_MODE_OUTPUT);
            //         gpio_set_level(gpio_touch_led_pins[0], 0); 
            //         sensor_reactivated_flag = true;
            //     }
            //     store_timeout_value();
            // }
    }


    /**************************************************************************/
    /*
        Displays some basic information on this sensor from the unified
        sensor API sensor_t type (see Adafruit_Sensor for more information)
    */
    /**************************************************************************/
    extern "C" void displaySensorDetails(void)
    {
        sensor_t sensor;
        tsl.getSensor(&sensor);
        printf("------------------------------------\n");
        printf  ("Sensor:       "); printf("%s", sensor.name);
        printf  ("Driver Ver:   "); printf("%ld", sensor.version);
        printf  ("Unique ID:    "); printf("%ld", sensor.sensor_id); printf("\n");
        printf  ("Max Value:    "); printf("%f", sensor.max_value); printf(" lux\n");
        printf  ("Min Value:    "); printf("%f", sensor.min_value); printf(" lux\n");
        printf  ("Resolution:   "); printf("%f", sensor.resolution); printf(" lux\n");  
        printf("------------------------------------\n");
        delay(500);
    }

    /**************************************************************************/
    /*
        Configures the gain and integration time for the TSL2561
    */
    /**************************************************************************/
    extern "C" void configureSensor(void)
    {
        /* You can also manually set the gain or enable auto-gain support */
        tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
        // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
        tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */
        
        /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
        tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
        // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
        // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

        /* Update these values depending on what you've set above! */  
        printf("------------------------------------\n");
        printf  ("Gain:         "); printf("Auto\n");
        printf  ("Timing:       "); printf("13 ms\n");
        printf("------------------------------------\n");
    }


    extern "C" void init_sensor_interrupt(){
        // if(!start_commissioning){
            //------------------>START AGS TASK
            Wire.begin(I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
            i2c_scanner();
            lux_sensor_error = false;
            if(!tsl.begin()) {
                /* There was a problem detecting the TSL2561 ... check your connections */
                printf("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!\n");
                lux_sensor_error = true;
            }else{
                /* Display some basic information on this sensor */
                displaySensorDetails();
                /* Setup the sensor gain and integration time */
                configureSensor();
            }
            xTaskCreate(sensor_task, "EspSensorTask", 4096, NULL, 22, NULL);
        // }
    }
    extern "C" void nuos_set_state_touch_leds(bool state){

    }
#endif




