
#include "app_hardware_driver.h"
#include "esp_zigbee_core.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "ha/esp_zigbee_ha_standard.h"
#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_MOTION || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_CONTACT_SWITCH)

    static bool sensor_detected_flag = false;
    static bool sensor_reactivated_flag = false;


    volatile bool led_on_at_startup_flag = true;
    uint32_t led_on_at_startup_timeout = 0;
    uint32_t led_on_at_startup_timeout_value = 50; //5 seconds

    volatile bool pir_motion_disable_flag = true;
    int32_t pir_motion_disable_timeout_counts = 0;
    uint32_t pir_motion_disable_timeout_value = 600; //1 minute

    extern void nuos_set_zigbee_attribute(uint8_t index);


    uint8_t comm_counts = 0;
    static bool state = 0;
    
    static void sensor_task1234(void *pvParameters) {
		while (1) {
			vTaskDelay(pdMS_TO_TICKS(1000));
            if(!start_commissioning){
                if (sensor_detected_flag) {
                    nuos_set_zigbee_attribute(1);
                    sensor_detected_flag = false;
                    //esp_zb_lock_acquire(portMAX_DELAY);
                    printf("Motion Detected!!\n");
                    printf("RED ON\n");
                    gpio_set_level(gpio_touch_led_pins[0], 0); //blue off
                    gpio_set_level(gpio_touch_led_pins[1], 1); //red on
                    // gpio_set_level(BUZZER_PIN, 1);
                    // vTaskDelay(pdMS_TO_TICKS(50));
                    // gpio_set_level(BUZZER_PIN, 0);
                    //gpio_intr_disable(PIR_INPUT_PIN);
                    gpio_set_level(gpio_load_pins[0], 1);
                    pir_motion_disable_flag = true;
                    //esp_zb_lock_release(); 
                }

                if (sensor_reactivated_flag) {
                    sensor_reactivated_flag = false;
                    //esp_zb_lock_acquire(portMAX_DELAY);
                    printf("Motion Reactivated!!\n");
                    printf("BLUE1 ON\n");
                    //gpio_intr_enable(PIR_INPUT_PIN);
                    gpio_set_level(gpio_touch_led_pins[0], 1); //blue on
                    gpio_set_level(gpio_touch_led_pins[1], 0); //red off
                    gpio_set_level(gpio_load_pins[0], 0);
                    nuos_set_zigbee_attribute(0);
                    //esp_zb_lock_release();
                }
            }	
		}
	}
static void sensor_task(void *pvParameters) {
    static TickType_t last_detection_time = 0;
    const TickType_t detection_interval = pdMS_TO_TICKS(5000); // 5 seconds
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        if(!start_commissioning) {
            TickType_t current_time = xTaskGetTickCount();
            
            if (sensor_detected_flag && 
                (current_time - last_detection_time >= detection_interval)) {
                // Update the last detection time
                last_detection_time = current_time;
                
                nuos_set_zigbee_attribute(1);
                sensor_detected_flag = false;
                printf("Motion Detected!!\n");
                printf("RED ON\n");
                gpio_set_level(gpio_touch_led_pins[0], 0); // blue off
                gpio_set_level(gpio_touch_led_pins[1], 1); // red on
                gpio_set_level(gpio_load_pins[0], 1);
                pir_motion_disable_flag = true;
            }

            if (sensor_reactivated_flag) {
                sensor_reactivated_flag = false;
                printf("Motion Reactivated!!\n");
                printf("BLUE1 ON\n");
                gpio_set_level(gpio_touch_led_pins[0], 1); // blue on
                gpio_set_level(gpio_touch_led_pins[1], 0); // red off
                gpio_set_level(gpio_load_pins[0], 0);
                nuos_set_zigbee_attribute(0);
            }
        }	
    }
}
	static void pir_isr_handler(void* arg) {
		sensor_detected_flag = true;
        pir_motion_disable_timeout_counts = 0;
	}

	void init_pir_sensor_gpio(){
		sensor_detected_flag = false;

        gpio_isr_handler_add(PIR_INPUT_PIN, pir_isr_handler, (void*) PIR_INPUT_PIN);
	}

	void init_pir_sensor_task(){

		xTaskCreate(sensor_task, "EspSensorTask", 4096, NULL, 9, NULL);
	}

    void init_buzzer_commissioning_task(){
        vTaskDelay(pdMS_TO_TICKS(2000));
        //esp_zb_lock_acquire(portMAX_DELAY);
        gpio_set_level(BUZZER_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(BUZZER_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(BUZZER_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(BUZZER_PIN, 0);

        comm_counts = getNVSSensorsCommissioningCounts();
        comm_counts++;
        printf("comm_counts:%d", comm_counts);
		// setNVSCommissioningFlag(1);
        setNVSSensorsCommissioningCounts(comm_counts);
        vTaskDelay(pdMS_TO_TICKS(2000));
        gpio_set_level(BUZZER_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(BUZZER_PIN, 0);
        setNVSSensorsCommissioningCounts(0);
        //esp_zb_lock_release(); 
    }

    void nuos_zb_init_hardware(){
        uint32_t pins = 0; 
        for(int index=0; index<TOTAL_LEDS; index++){
            pins |= 1ULL << gpio_touch_led_pins[index];
        }  
        gpio_config_t io_conf;

        io_conf.intr_type = GPIO_INTR_POSEDGE; // Interrupt on rising edge (motion detected)
		io_conf.pin_bit_mask = (1ULL << PIR_INPUT_PIN);
		io_conf.mode = GPIO_MODE_INPUT;
		io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
		gpio_config(&io_conf);

        // Configure the output pin
        io_conf.intr_type = GPIO_INTR_DISABLE;       // Disable interrupt
        io_conf.mode = GPIO_MODE_OUTPUT;             // Set as output mode
        io_conf.pin_bit_mask = (1ULL << gpio_touch_led_pins[1]) | (1ULL << gpio_touch_led_pins[0]) | (1ULL << BUZZER_PIN) | (1ULL << gpio_load_pins[0]); // Bit mask for the output pin
        io_conf.pull_down_en = 1;                    // Disable pull-down mode
        io_conf.pull_up_en = 0;                      // Disable pull-up mode
        gpio_config(&io_conf);


        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(gpio_touch_led_pins[0], 1);        
        vTaskDelay(pdMS_TO_TICKS(100)); 
        gpio_set_level(gpio_touch_led_pins[1], 1);

        led_on_at_startup_flag = true; 

        // init_buzzer_commissioning_task();
        // //read last stored timeout value from nvs
        // read_timeout_value();

        init_timer();
        esp_start_timer();
 
    }

    void init_sensor_commissioning_task(){
         init_buzzer_commissioning_task();
        //read last stored timeout value from nvs
        read_timeout_value();
    }
    void nuos_zb_set_hardware_led(uint8_t index, uint8_t is_toggle){
        if(is_toggle>0) device_info[index].device_state = !device_info[index].device_state;
        gpio_set_direction(gpio_touch_led_pins[index], GPIO_MODE_OUTPUT);
        if(is_toggle == false) gpio_set_level(gpio_touch_led_pins[index], 0);
        else gpio_set_level(gpio_touch_led_pins[index], device_info[index].device_state);  
    }

    void nuos_zb_set_hardware(uint8_t index, uint8_t is_toggle){
        //toggle pins on button press
        if(is_toggle>0) device_info[index].device_state = !device_info[index].device_state;
        //set touch led pins
        esp_zb_lock_acquire(portMAX_DELAY);
        gpio_set_level(gpio_touch_led_pins[index], device_info[index].device_state);
        //set load pins
        gpio_set_level(gpio_load_pins[index], device_info[index].device_state);
        esp_zb_lock_release();   
        nuos_store_data_to_nvs(index);
    }

    
    void nuos_on_off_led(uint8_t index, uint8_t _state){
        gpio_set_level(gpio_touch_led_pins[index], _state);
    }

    void nuos_toggle_leds(uint8_t index){
        state = !state;       
        nuos_on_off_led(index, state);
    }

    void nuos_zb_set_hardware_led_for_zb_commissioning(uint8_t is_toggle){
        nuos_zb_set_hardware_led(0, is_toggle);
        nuos_zb_set_hardware_led(1, !is_toggle);
    }

    // void nuos_esp_motion_sensor_check_in_timer(){
    //         // Code to execute when the timer expires
    //         if(led_on_at_startup_flag){
    //             if(led_on_at_startup_timeout++ >= led_on_at_startup_timeout_value){
    //                 led_on_at_startup_flag = false;
    //                 led_on_at_startup_timeout = 0;
    //                 esp_zb_lock_acquire(portMAX_DELAY);
    //                 printf("BLUE ON\n");
    //                 gpio_set_level(gpio_touch_led_pins[1], 0);
    //                 //turn on blue led
    //                 gpio_set_level(gpio_touch_led_pins[0], 1);
    //                 esp_zb_lock_release(); 
    //                 init_sensor_interrupt();
    //             }
    //         }
    //         if(pir_motion_disable_flag){
    //             if(pir_motion_disable_timeout_counts++ >= pir_motion_disable_timeout_value){
    //                 pir_motion_disable_flag = false;
    //                 pir_motion_disable_timeout_counts = 0;
    //                 esp_zb_lock_acquire(portMAX_DELAY);
    //                 printf("BLUE ON\n");
    //                 //turn off red led
    //                 gpio_set_level(gpio_touch_led_pins[1], 0);
    //                 //turn on blue led
    //                 gpio_set_level(gpio_touch_led_pins[0], 1); 
    //                 esp_zb_lock_release(); 
    //                 sensor_reactivated_flag = true;
    //             }
    //             store_timeout_value();
    //         }

    // }
    void init_sensor_interrupt(){
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_MOTION || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_CONTACT_SWITCH)
            init_pir_sensor_gpio();
            init_pir_sensor_task();
        #endif
    }
    void nuos_set_state_touch_leds(bool state){

    }
#endif




