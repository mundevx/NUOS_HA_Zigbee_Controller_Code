
#include "app_hardware_driver.h"
#include "app_zigbee_clusters.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if((USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY) || (USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY_CUSTOM))
    #include <Arduino.h>
    #define TEMP_HUM_LIBRARY  2


    #if (TEMP_HUM_LIBRARY == 1)
        #include "DHTesp.h" // Click here to get the library: http://librarymanager/All#DHTesp
        #include <Ticker.h>
        DHTesp dht;
    #elif (TEMP_HUM_LIBRARY == 2)
        #include "dht11.h"
    #elif (TEMP_HUM_LIBRARY == 3)
        #include <Adafruit_Sensor.h>
        #include "dht11.h"
        #include <DHT_U.h>

        // Uncomment the type of sensor in use:
        #define DHTTYPE                 DHT11     // DHT 11 
        //#define DHTTYPE               DHT22     // DHT 22 (AM2302)
        //#define DHTTYPE               DHT21     // DHT 21 (AM2301)
        DHT                             dht(TEMP_HUMIDITY_PIN, DHTTYPE);

        SemaphoreHandle_t timerMux;
    #endif

    ////////////////////////////////////////////////  
    static bool sensor_detected_flag = false;
    static bool sensor_reactivated_flag = false;

    volatile bool led_on_at_startup_flag = true;
    uint32_t led_on_at_startup_timeout = 0;
    uint32_t led_on_at_startup_timeout_value = 50; //5 seconds

    volatile bool pir_motion_disable_flag = true;
    int32_t pir_motion_disable_timeout_counts = 0;
    uint32_t pir_motion_disable_timeout_value = 600; //1 minute
    uint8_t comm_counts = 0;


    extern "C" void sensor_task(void *pvParameters) {
        //Turn off all LEDs and mute Buzzer  
        gpio_set_direction(gpio_touch_led_pins[0], GPIO_MODE_OUTPUT);
        gpio_set_level(gpio_touch_led_pins[0], 1); //blue on
        gpio_set_direction(gpio_touch_led_pins[1], GPIO_MODE_OUTPUT);
        gpio_set_level(gpio_touch_led_pins[1], 0); //red off
        gpio_set_direction(gpio_load_pins[0], GPIO_MODE_OUTPUT);
        gpio_set_level(gpio_load_pins[0], 0);   
		while (1) {
            if(is_my_device_commissionned){
                #if (TEMP_HUM_LIBRARY == 1)  //NOT WORKING
                    TempAndHumidity newValues = dht.getTempAndHumidity(); 
                    printf(" T:%f H:%f\n", newValues.temperature, newValues.humidity);                 
                    // Check if any reads failed and exit early (to try again).
                    if (dht.getStatus() != 0) {
                        printf("DHT11 error\n");
                        device_info[0].device_val = 0;
                        nuos_set_zigbee_attribute(0);
                        device_info[1].device_val = 0;
                        nuos_set_zigbee_attribute(1);                          
                    }else{
                        
                        device_info[0].device_val = (uint16_t)(newValues.temperature*100);
                        nuos_set_zigbee_attribute(0);
                        device_info[1].device_val = (uint16_t)(newValues.humidity*100);
                        nuos_set_zigbee_attribute(1); 
                    }
                #elif (TEMP_HUM_LIBRARY == 2)  //WORKING, Currently using this library
                    uint16_t temperature = DHT11_read().temperature;
                    uint16_t humidity = DHT11_read().humidity;

                    // printf("Temperature is %d \n", temperature);
                    // printf("Humidity is %d\n", humidity);

                    uint16_t temp_mul = (temperature*100);
                    if(temperature != 0xffff){
                        if(device_info[0].device_val != temp_mul){
                            device_info[0].device_val = temp_mul;
                            nuos_set_zigbee_attribute(0);
                            vTaskDelay(pdMS_TO_TICKS(10));
                        }
                    }
                    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY)
                        uint16_t humid_mul = (humidity*100);
                        if(humidity != 0xffff) {
                            if(device_info[1].device_val != humid_mul){
                                device_info[1].device_val = humid_mul;
                                nuos_set_zigbee_attribute(1);   
                            }
                        }
                    #else
                        if(humidity != 0xffff) device_info[1].device_val = (humidity * 100);
                        nuos_set_zigbee_attribute(0);    
                    #endif                 
                #elif (TEMP_HUM_LIBRARY == 3) //NOT WORKING, ERROR in library, this statement InterruptLock lock; and also usage of delay, blocking delay
                    if (xSemaphoreTake(timerMux, portMAX_DELAY) == pdTRUE) {
                            //portENTER_CRITICAL(&timerMux);
                            // Read temperature as Celsius (the default)
                            float t = dht.readTemperature();
                            printf("Temperature: %f *C \n", t);
                            //portEXIT_CRITICAL(&timerMux);  
                            xSemaphoreGive(timerMux); 
                    }   
                #endif
            }
            vTaskDelay(pdMS_TO_TICKS(5000)); //update every 5 seconds
		}
	}

    // extern "C" void init_buzzer_commissioning_task(){
    //     vTaskDelay(pdMS_TO_TICKS(2000));
    //     gpio_set_level(BUZZER_PIN, 1);
    //     vTaskDelay(pdMS_TO_TICKS(50));
    //     gpio_set_level(BUZZER_PIN, 0);
    //     vTaskDelay(pdMS_TO_TICKS(50));
    //     gpio_set_level(BUZZER_PIN, 1);
    //     vTaskDelay(pdMS_TO_TICKS(50));
    //     gpio_set_level(BUZZER_PIN, 0);
    //     comm_counts = getNVSSensorsCommissioningCounts();
    //     comm_counts++;

    //     setNVSSensorsCommissioningCounts(comm_counts);
    //     vTaskDelay(pdMS_TO_TICKS(2000));
    //     gpio_set_level(BUZZER_PIN, 1);
    //     vTaskDelay(pdMS_TO_TICKS(100));
    //     gpio_set_level(BUZZER_PIN, 0);
    //     setNVSSensorsCommissioningCounts(0);
    // }
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
        gpio_set_level(gpio_touch_led_pins[1], 1);
        gpio_set_direction(gpio_touch_led_pins[0], GPIO_MODE_OUTPUT);
        gpio_set_level(gpio_touch_led_pins[0], 1);        

        led_on_at_startup_flag = true; 
        // init_timer();
        // esp_start_timer();
    }

    extern "C" void init_sensor_commissioning_task(){
         init_buzzer_commissioning_task();
        //read last stored timeout value from nvs
        // read_timeout_value();
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



    void nuos_on_off_led(uint8_t index, uint8_t _state){
        gpio_set_level(gpio_touch_led_pins[index], _state);
    }

    void nuos_toggle_leds(uint8_t index){
        state = !state;       
        nuos_on_off_led(index, state);
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

    extern "C" void init_sensor_task(){
        printf("dht.begin Hardware\n");
        #if (TEMP_HUM_LIBRARY == 1)
            dht.setup(TEMP_HUMIDITY_PIN, DHTesp::DHT11);
            pinMode(TEMP_HUMIDITY_PIN, INPUT);
            TempAndHumidity newValues = dht.getTempAndHumidity(); 
            printf(" T:%f H:%f\n", newValues.temperature, newValues.humidity);             
        #elif (TEMP_HUM_LIBRARY == 2)

            DHT11_init(TEMP_HUMIDITY_PIN);

        #elif (TEMP_HUM_LIBRARY == 3)
            dht.begin();
            timerMux = xSemaphoreCreateMutex();
            // Check if mutex creation was successful
            if (timerMux == NULL) {
                printf("Failed to create mutex!\n");
                // Handle error, maybe retry or exit
            }
            // pinMode(0, INPUT);
            vTaskDelay(pdMS_TO_TICKS(2000));
            float t = dht.readTemperature();
            printf("Temperature: %f *C \n", t);
        #endif
        // Serial.println("DHT initiated");
        xTaskCreate(sensor_task, "EspSensorTask", 4096, NULL, 22, NULL);
    }


    extern "C" void nuos_esp_motion_sensor_check_in_timer(){
        // // Code to execute when the timer expires
        // if(led_on_at_startup_flag){
        //     if(led_on_at_startup_timeout++ >= led_on_at_startup_timeout_value){
        //         led_on_at_startup_flag = false;
        //         led_on_at_startup_timeout = 0;
        //         printf("BLUE ON\n");                
        //         //turn off red led
        //         gpio_set_direction(gpio_touch_led_pins[1], GPIO_MODE_OUTPUT);
        //         gpio_set_level(gpio_touch_led_pins[1], 0);
        //         //turn on blue led
        //         gpio_set_direction(gpio_touch_led_pins[0], GPIO_MODE_OUTPUT);
        //         gpio_set_level(gpio_touch_led_pins[0], 1);

        //         init_sensor_task();
        //     }
        // }
    }

    extern "C" void nuos_set_state_touch_leds(bool state){

    }
#endif




