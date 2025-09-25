
#include "esp_timer.h"
#include "esp_system.h"
#include <esp_check.h>
#include <esp_log.h>
#include "app_hardware_driver.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_zigbee_core.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zb_config_platform.h"
#include "light_driver.h"
#include "app_nuos_timer.h"
#include "driver/ledc.h"
#include <esp_check.h>
#include <esp_log.h>
#include "esp_log.h"
#include "esp_err.h"
#include "app_zigbee_clusters.h"
#include <math.h>
#include <time.h>

esp_timer_handle_t my_timer_handle;
void timer_callback(void* arg);

static const char *TAG 									= "ESP_ZB_TIMERS";

bool timer_initialized_ok 								= false;
bool button_read_flag[4] 								= {false, false, false, false};
uint32_t switch_pressed_counts_to_enter_commissioning 	= 0;

/*Currently Working Roughly on it*/
uint32_t timer_commissioning_counts 					= 0;
uint32_t timer_led_blink_counts 						= 0;

static bool timer_started_flag 							= false;

bool toggle_status_led 									= false;
static bool led_state 									= false;
/* This code is used for starting timer 2 when there is clusters are not found and unmatched after restart of MCU */
esp_timer_handle_t my_timer_handle_2;
uint32_t timer_counts_enable_webserver 					= 0;
static int timer_2_counts 								= 0;

void init_timer() {
	#ifndef DONT_USE_ZIGBEE
	if(!timer_initialized_ok){
		timer_initialized_ok = true;
		// Create a timer
		const esp_timer_create_args_t timer_args = {
			.callback = &timer_callback,
			// name is optional, but may help identify the timer when debugging
			.name = "one_second_timer"
		};
		ESP_ERROR_CHECK(esp_timer_create(&timer_args, &my_timer_handle));
	}
	#endif
}

void esp_start_timer(){
	#ifndef DONT_USE_ZIGBEE
	if(!timer_started_flag){
		timer_started_flag = true;
			// Start the timer
		ESP_ERROR_CHECK(esp_timer_start_periodic(my_timer_handle, 100000)); // 100 milli second period	
	}
    timer_led_blink_counts = 0;
	timer_commissioning_counts = 0;
	#endif
}

void esp_stop_timer(){
	#ifndef DONT_USE_ZIGBEE
	if(timer_started_flag){
		timer_started_flag = false;
		if(my_timer_handle != NULL)
			esp_timer_stop(my_timer_handle);	
	}
	#endif
}


void nuos_start_commissioning(){
	printf("set_commissioning\n");
	setNVSCommissioningFlag(1);
}

void nuos_stop_commissioning(unsigned char timeout){
	printf("========start_commissioning:%d========\n", start_commissioning);
	if(!start_commissioning){
		start_commissioning = false;
		// ready_commisioning_flag  = false;
		// setNVSCommissioningFlag(0);
       
		#ifdef USE_RGB_LED
			light_driver_set_power(0);
		#else
			nuos_zb_set_hardware_led_for_zb_commissioning(false);
		#endif

		#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_MOTION || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_CONTACT_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_GAS_LEAK)
			init_timer();
			esp_start_timer();
			printf("##############START_TIMER###########\n");
		#elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_LUX)
			init_timer();
			esp_start_timer();	
			printf("##############START_TIMER###########\n");
		#elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
		// #ifndef ZB_COMMISSIONING_WITHOUT_TIMER
		init_timer();
		esp_start_timer();
		// #endif
		// 	printf("------START Timer-------\n");

		#elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
		// #ifndef ZB_COMMISSIONING_WITHOUT_TIMER
		init_timer();
		esp_start_timer();	
		// #endif	
		// #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI)
		#else
			esp_stop_timer();
			printf("##############STOP_TIMER###########\n");			
		#endif	
	}

    esp_start_timer_3();
	// nuos_reset_count_nw_steering_fault();
}


void start_wifi_webserver_timer_task(){
	#ifdef USE_WIFI_WEBSERVER
	if(wifi_webserver_active_flag > 0){
		if(wifi_webserver_active_counts >= WIFI_REMAIN_ACTIVE_IN_COUNTS){
			wifi_webserver_active_counts = 0;
			wifi_webserver_active_flag = 0;
			timer_counts_enable_webserver = 0;
            //save record in nvs
			setNVSWebServerEnableFlag(wifi_webserver_active_flag);
			//restart devices
			esp_restart();
		}else{
			if(timer_counts_enable_webserver%2 == 0){
				#ifdef USE_RGB_LED
					toggle_status_led = !toggle_status_led;
					light_driver_set_power(toggle_status_led);
				#else
					nuos_zb_set_hardware_led_for_zb_commissioning(true);
				#endif
			}			
			if(timer_counts_enable_webserver%TIME_COUNTS_FOR_1_SEC == 0){	
				wifi_webserver_active_counts++;
				printf("wifi_webserver_active_counts:%ld\n", wifi_webserver_active_counts);		
			}
            timer_counts_enable_webserver++; 
		}
	}

    #endif
}



void start_zb_commissioning_timer_task(){
	if(start_commissioning > 0){
		if(timer_led_blink_counts >= TIMER_COMMISSIONING_LED_BLINK_COUNTS){
			timer_led_blink_counts = 0;
			#ifdef USE_RGB_LED
			    toggle_status_led = !toggle_status_led;
				if(toggle_status_led) light_driver_set_color_RGB(0xff, 0, 0);  //red
				light_driver_set_power(toggle_status_led);
			#else
			    nuos_zb_set_hardware_led_for_zb_commissioning(true);
			#endif
			#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
				if(wifi_webserver_active_flag  == 0){
					printf("Commissioning....\n");
				}else{
					printf("Set AC Model....\n");
				}
			#else
				printf("Zigbee Commissioning...\n");	
			#endif
			
			if(timer_commissioning_counts >= COMMISSIONING_TIMEOUT){
				timer_commissioning_counts = 0;
				

				#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
					if(wifi_webserver_active_flag  == 0){
						nuos_stop_commissioning(1);
						
					}
					start_commissioning = false;
				#else

				    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_GAS_LEAK || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_LUX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY)
						start_commissioning = false;
						nuos_stop_commissioning(1);	
					#else
					start_commissioning = false;
					nuos_stop_commissioning(1);	
					#endif
					#ifdef USE_RGB_LED
						light_driver_set_color_RGB(0x00, 0xff, 0x00);
						light_driver_set_power(0);
						printf("stop_commissioning\n");
						light_driver_deinit_flag = true;
					#else
						nuos_set_state_touch_leds(false);
						
					#endif
					
				#endif
                //esp_zb_bdb_cancel_steering(); //no need to use this
			}
		}else{
			timer_led_blink_counts++;
            timer_commissioning_counts++;
		}
	}
}

void start_zb_devices_timer_task(){	
	#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_MOTION || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_CONTACT_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_GAS_LEAK || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_LUX)
		// nuos_esp_motion_sensor_check_in_timer();	
	#endif	
}



void timer_callback(void* arg) {
    /*START COMMISSIONING TASK*/   
	#ifndef ZB_COMMISSIONING_WITHOUT_TIMER
		#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX  || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
			if(wifi_webserver_active_flag==0){
		#endif
			start_zb_commissioning_timer_task(); 
		 #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
		 	}else{
		 		start_wifi_webserver_timer_task();
		 	}
		 #endif
	#endif
	start_zb_devices_timer_task();  
	#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
		if(find_active_neighbours_task_flag){
			led_state = !led_state;
			for(int i=0; i<TOTAL_LEDS; i++){
				if(i != save_pressed_button_index) 
					gpio_set_level(gpio_touch_led_pins[i], led_state);
			}
		}
    #endif
	
	#ifdef USE_DOUBLE_PRESS

	#else
		if(double_press_click_enable){
			if(disable_double_press_enable_counts++ > 50){ //10 sec
				double_press_click_enable = false;
			}
		}	
	#endif
	
}

void initTwoSwitchPressedPins(){
	for(int i=0; i<TOTAL_BUTTONS; i++){
        if(button_read_flag[i]){
            button_read_flag[i] = false;
        }
	}
    switch_pressed_counts_to_enter_commissioning = 0;
}

uint32_t IdentifyTwoSwitchPressed(){
	// #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_LUX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_MOTION || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_CONTACT_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_GAS_LEAK)
	// #else	
		for(int i=0; i<TOTAL_BUTTONS; i++){
			bool value = gpio_get_level(gpio_touch_btn_pins[i]);
			if(value == 0) {
				if(button_read_flag[i] == false){
					button_read_flag[i] = true;
					switch_pressed_counts_to_enter_commissioning++;
				}
			}
		}
	// #endif
	return switch_pressed_counts_to_enter_commissioning;
}

uint8_t get_button_pressed_mode(){
	bool first_button_pressed = false;
	bool second_button_pressed = false;
	bool third_button_pressed = false;
	bool fourth_button_pressed = false;

	for(int i=0; i<TOTAL_BUTTONS; i++){
        if(button_read_flag[i]){
           if(i==0){
			  first_button_pressed = true;
			  printf("first_button_pressed1\n");
		   }else if(i==1){
			  second_button_pressed = true;
			  printf("second_button_pressed\n");
		   }else if(i==2){
			  third_button_pressed = true;
			  printf("third_button_pressed\n");
		   }else if(i==3){
			  fourth_button_pressed = true;
			  printf("fourth_button_pressed\n");
		   }
        }
	}

	#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)	
		if(first_button_pressed && third_button_pressed){
			return 1;
		}else if(second_button_pressed && fourth_button_pressed){
			return 2;
		}
	#elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_MOTION || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_CONTACT_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_GAS_LEAK || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_LUX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY)					
		if(first_button_pressed){
			return 1;
		}
	#else
		if(first_button_pressed && second_button_pressed){
			return 1;
		}else if(third_button_pressed && fourth_button_pressed){
			return 2;
		}else if(second_button_pressed){
			return 0;
		}
	#endif
	return 0xff;
}


static TimerHandle_t commissioning_timeout_timer                = NULL;


void commissioning_timeout_handler(TimerHandle_t xTimer)
{
	ready_commisioning_flag = false;
	setNVSStartCommissioningFlag(0);
	if (xTimerIsTimerActive(commissioning_timeout_timer) != pdFALSE) {
		xTimerStop(commissioning_timeout_timer, 0);
	}
}


void start_commissioning_timeout_timer(){
	if(commissioning_timeout_timer == NULL)
	commissioning_timeout_timer = xTimerCreate("Comm Timer", pdMS_TO_TICKS(30000), pdFALSE, (void *)0, commissioning_timeout_handler);
	if(commissioning_timeout_timer != NULL){
		if (xTimerStart(commissioning_timeout_timer, 0) != pdPASS) {
			ESP_LOGE("Timer", "Failed to start commissining timer");
			return;
		}
	}

}

void actionOnTwoSwitchPressed(int64_t timeout) {
	bool first_button_pressed = false;
	bool second_button_pressed = false;
	bool third_button_pressed = false;
	bool fourth_button_pressed = false;
    //printf("timeout: %lld\n", timeout);
	for(int i=0; i<TOTAL_BUTTONS; i++){
        if(button_read_flag[i]){
           if(i==0){
			  first_button_pressed = true;
			  printf("first_button_pressed\n");
		   }else if(i==1){
			  second_button_pressed = true;
			  printf("second_button_pressed\n");
		   }else if(i==2){
			  third_button_pressed = true;
			  printf("third_button_pressed\n");
		   }else if(i==3){
			  fourth_button_pressed = true;
			  printf("fourth_button_pressed\n");
		   }
        }
	}


	for(int i=0; i<TOTAL_BUTTONS; i++){
        if(button_read_flag[i]){
            button_read_flag[i] = false;
        }
	}

	#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
	    
		if(timeout >= LONG_PRESS_ENABLE_WIFI_WEB_SERVER_TIME_IN_SECS && timeout < LONG_PRESS_SET_COMMISSIONING_TIME_IN_SECS){  //start web server
			if(wifi_webserver_active_flag){
				wifi_webserver_active_flag = false;
			}else{
				wifi_webserver_active_flag = true;  
			}
			printf("wifi_webserver_active_flag:%d\n", wifi_webserver_active_flag);
			setNVSCommissioningFlag(false);
			setNVSWebServerEnableFlag(wifi_webserver_active_flag);                        
			esp_restart();	
		}
	#endif
	#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
		if(timeout >= LONG_PRESS_ENABLE_WIFI_WEB_SERVER_TIME_IN_SECS && timeout < LONG_PRESS_SET_COMMISSIONING_TIME_IN_SECS){  //start web server
			if(wifi_webserver_active_flag){
				wifi_webserver_active_flag = false;
			}else{
				wifi_webserver_active_flag = true;  
			}
			setNVSCommissioningFlag(false);
			setNVSWebServerEnableFlag(wifi_webserver_active_flag);                        
			esp_restart();	
		}else if(timeout >= LONG_PRESS_SET_COMMISSIONING_TIME_IN_SECS){  //start commissioning
			printf("set_commissioning\n");
			setNVSCommissioningFlag(1);
			if (esp_zb_bdb_dev_joined()) {
				esp_zb_bdb_reset_via_local_action();
			}
			esp_zb_factory_reset();
		}

    #else
	#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
		if(timeout >= LONG_PRESS_SET_COMMISSIONING_TIME_IN_SECS){ 
	#else
		if(timeout >= MAX_TIME_TO_START_COMMISSIONING_ON_2_BUTTONS_PRESSED){ 
	#endif
		#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_MOTION || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_CONTACT_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_GAS_LEAK || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_LUX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY)					
			if(switch_pressed_counts_to_enter_commissioning == 1){
        #else
			if(switch_pressed_counts_to_enter_commissioning >= 2){  //if any switch pressed by mistake!!
	    #endif			
				switch_pressed_counts_to_enter_commissioning = 0;
				#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
		
				
					if(first_button_pressed && third_button_pressed){
						printf("set_commissioning\n");
						ready_commisioning_flag = true;
						setNVSStartCommissioningFlag(1);
						setNVSCommissioningFlag(0);
						
					}else if(second_button_pressed && fourth_button_pressed){
						#ifdef USE_WIFI_WEBSERVER	
							printf("enable_wifi_ap_mode\n");
							if(wifi_webserver_active_flag){
								wifi_webserver_active_flag = false;
							}else{
								wifi_webserver_active_flag = true;  
							}
							setNVSCommissioningFlag(0);
							setNVSWebServerEnableFlag(wifi_webserver_active_flag);                    
							esp_restart();					
						#endif
					}else{
						nuos_set_rgb_led_normal_functionality();
						//nuos_kill_rgb_led_blink_task();
					}
				#elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_MOTION || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_CONTACT_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_GAS_LEAK || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_LUX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY)					
				if(first_button_pressed){
					printf("set_commissioning\n");
					esp_zb_nvram_erase_at_start(true);
					setNVSCommissioningFlag(1);
					if (esp_zb_bdb_dev_joined()) {
						esp_zb_bdb_reset_via_local_action();
					}
					esp_zb_factory_reset();					
				}
				#else
					if(first_button_pressed && second_button_pressed){
						printf("set_commissioning\n");
                        // esp_zb_nvram_erase_at_start(true);
						ready_commisioning_flag = true;
						setNVSStartCommissioningFlag(1);
						setNVSCommissioningFlag(0);
                        
					}else if(third_button_pressed && fourth_button_pressed){
						#ifdef USE_WIFI_WEBSERVER	
							printf("enable_wifi_ap_mode\n");
							if(wifi_webserver_active_flag){
								wifi_webserver_active_flag = false;
							}else{
								wifi_webserver_active_flag = true;  
							}
							setNVSCommissioningFlag(0);
							setNVSWebServerEnableFlag(wifi_webserver_active_flag);                    
							esp_restart();					
						#endif
					}else{

					}
				#endif
				if(ready_commisioning_flag){
					start_commissioning_timeout_timer();
				}
			}
		}else{
			nuos_set_rgb_led_normal_functionality();
		}
	#endif
}


void send_zb_status_task(void* args) {
	// for(int i=0; i<TOTAL_ENDPOINTS; i++){
	// 	nuos_set_zigbee_attribute(i);
	// 	vTaskDelay(10);
	// }
    // vTaskDelete(NULL); // Delete the task after executing
}

esp_timer_handle_t my_timer_handle_3;
static int timer_3_counts = 0;
void timer_callback_3(void* arg) {  
	if(touchLedsOffAfter1MinuteEnable){
		if(timer_3_counts++ >= TOUCH_LEDS_ALL_OFF_TIMEOUT){   //1 minute count-up
			timer_3_counts = 0;
			timer3_running_flag = false;
			printf("TIMER3 STOP!!\n");
			
			nuos_set_state_touch_leds(false);
		
			esp_stop_timer_3();  
		}
	}
}
bool start_timer3_flag = false;
void init_timer_3() {
	// if(touchLedsOffAfter1MinuteEnable){
		// Create a timer
		const esp_timer_create_args_t timer_args = {
			.callback = &timer_callback_3,
			.name = "timer_3"
		};
		ESP_ERROR_CHECK(esp_timer_create(&timer_args, &my_timer_handle_3));
		start_timer3_flag = false;
	// }
}

void esp_start_timer_3(){
	if(touchLedsOffAfter1MinuteEnable){
		// Start the timer
		if(!start_timer3_flag){
			start_timer3_flag = true;
			if(my_timer_handle_3 != NULL)
			ESP_ERROR_CHECK(esp_timer_start_periodic(my_timer_handle_3, 1000000)); // 1 second period
		}
		timer_3_counts = 0;	
	}
}

void esp_stop_timer_3(){
	if(touchLedsOffAfter1MinuteEnable){
		if(start_timer3_flag){
			start_timer3_flag = false;
			if(my_timer_handle_3 != NULL)
				esp_timer_stop(my_timer_handle_3);	
		}	
	}
}

void recheckTimer(){
	
	if(!timer3_running_flag){
		timer3_running_flag = true;
		if(touchLedsOffAfter1MinuteEnable){
			esp_start_timer_3();
		}
	} 
	
	
}
/////////////////////////////////////////////////////////////////////////
#ifdef USE_CCT_TIME_SYNC
#define MIN_MIREDS 								(1000000 / MAX_CCT_VALUE)
#define MAX_MIREDS 								(1000000 / MIN_CCT_VALUE)

#define UPDATE_INTERVAL 						(10 * 60 * 1000)  		// 10 minutes in milliseconds

#define PI 										3.141592653589793

// Function to calculate daylight-saving color temperature
double calculate_CCT(double t, double CCT_min, double CCT_max, double T, double t_peak) {
    return CCT_min + (CCT_max - CCT_min) * (1 + cos((2 * PI / T) * (t - t_peak))) / 2;
}


double calculate_sinusoidal_value(double current_time, double min_val, double max_val, double period, double peak_time) {
    // Sinusoidal brightness and CCT curve based on time of day
    return min_val + (max_val - min_val) * (0.5 * (1 + cos((M_PI / period) * (current_time - peak_time))));
}

#define MIN_CCT_VALUE 2000    // Warm evening light (candle-like)
#define MAX_CCT_VALUE 6500    // Cool daylight (noon sun)
#define MIN_DIM_LEVEL 10      // Minimum brightness
#define MAX_DIM_LEVEL 254     // Maximum brightness

#define SUNRISE_START 6       // 6 AM
#define DAYLIGHT_END 18       // 6 PM

double human_centric_brightness(double current_time, double min, double max) {
    const double CYCLE = 24.0;
    const double PEAK_TIME = 12.0; // Peak at noon

    // Shift time to center cosine wave at noon
    double shifted_time = fmod(current_time - PEAK_TIME + CYCLE, CYCLE);

    // Cosine wave ranging from -1 to 1, shifted to 0-1 range
    double wave = 0.5 * (1 - cos(2 * M_PI * shifted_time / CYCLE));
    //       ^ note the minus sign here!

    return min + wave * (max - min);
}

// double human_centric_sinusoid(double current_time, double min, double max) {
//     const double CYCLE = 24.0;
//     const double PEAK_TIME = 12.0; // Peak at noon
    
//     // Shift time to center cosine wave at noon
//     double shifted_time = fmod(current_time - PEAK_TIME + CYCLE, CYCLE);
    
//     // Cosine wave ranging from -1 to 1, shifted to 0-1 range
//     double wave = 0.5 * (1 + cos(2 * M_PI * shifted_time / CYCLE));
    
//     return min + wave * (max - min);
// }


double human_centric_cct(double current_time) {
    // Two-stage piecewise function for natural transition
    if (current_time >= SUNRISE_START && current_time < DAYLIGHT_END) {
        // Daylight hours: full sinusoidal curve
        const double CYCLE = DAYLIGHT_END - SUNRISE_START;
        double normalized = (current_time - SUNRISE_START) / CYCLE;
        return MIN_CCT_VALUE + (MAX_CCT_VALUE - MIN_CCT_VALUE) * 
               (1 - cos(M_PI * normalized)) / 2;
    } else {
        // Evening/Night: maintain warm light
        return MIN_CCT_VALUE;
    }
}

void get_cct_and_brightness(uint16_t *CCT, uint8_t *brightness) {
    double T = 24;      // 24-hour cycle
    double t_peak = 12; // Peak (noon)

    // Get current time in hours (24-hour format)
    time_t rawtime;
    struct tm *timeinfo;
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    double current_time = timeinfo->tm_hour + (timeinfo->tm_min / 60.0);

    // Calculate CCT and brightness
    // *CCT = (uint16_t)calculate_sinusoidal_value(current_time, MIN_CCT_VALUE, MAX_CCT_VALUE, T, t_peak);
    // *brightness = (uint8_t)calculate_sinusoidal_value(current_time, MIN_DIM_LEVEL_VALUE, MAX_DIM_LEVEL_VALUE, T, t_peak);
	// printf("Time: %.2f hrs -> CCT: %dK, Brightness: %d%%\n", current_time, *CCT, *brightness);
    *CCT = (uint16_t)human_centric_cct(current_time);
    //*brightness = (uint8_t)human_centric_sinusoid(current_time, MIN_DIM_LEVEL_VALUE, MAX_DIM_LEVEL_VALUE);
    *brightness = (uint8_t)human_centric_brightness(current_time, MIN_DIM_LEVEL_VALUE, MAX_DIM_LEVEL_VALUE);

	printf("Time: %.2f hrs -> CCT: %dK (%.1f%% cool), Brightness: %d%%\n", 
           current_time, *CCT, 
           100.0 * (*CCT - MIN_CCT_VALUE) / (MAX_CCT_VALUE - MIN_CCT_VALUE),
           *brightness);

}

static uint32_t zigbee_time = 0; // Stores the last synced Zigbee time

// Callback when time attribute is received
void zigbee_time_received(uint32_t received_time) {
    zigbee_time = received_time + 946684800; // Convert Zigbee time to Unix time
}

// Function to calculate dynamic color temperature
uint16_t get_dynamic_color_temp() {
    if (zigbee_time == 0) {
        return MIN_MIREDS; // Default to warm light if no time sync
    }

    time_t now = zigbee_time; // Use synced time
    struct tm *tm_info = localtime(&now);
    int hour = tm_info->tm_hour;

    float factor;
    if (hour < 6) {
        factor = 0.0;  // Night - Warmest
    } else if (hour < 12) {
        factor = (hour - 6) / 6.0;  // Morning - Gradually cooler
    } else if (hour < 18) {
        factor = 1.0;  // Daylight - Coolest
    } else {
        factor = (18 - hour) / 6.0;  // Evening - Gradually warmer
    }

    uint16_t color_temp_mireds = MIN_MIREDS + factor * (MAX_MIREDS - MIN_MIREDS);
    return color_temp_mireds;
}

// Send color temperature to Zigbee light
void send_color_temperature() {
	is_long_press_brightness = false;
	nuos_set_hardware_brightness_2(1);
	nuos_zb_set_hardware(0, false);
	// if(!device_info[0].device_state){
	// 	device_info[0].device_state = true;
		set_state(0);
	// }
	set_level(0);                                              
	set_color_temp();	
	
}

static uint8_t time_sync_counts = 0;
// Timer callback: Sync time and update color temperature
void update_color_temp_timer_callback(void* arg) {
	if(set_cct_time_sync_request_flag){
		if(device_info[0].color_or_fan_state && device_info[0].device_state){
			read_zigbee_time(); // Sync time
			// uint16_t new_color_temp = get_dynamic_color_temp();
			get_cct_and_brightness(&device_info[0].device_val, &device_info[0].device_level);
			printf("NEW_CCT:%d", device_info[0].device_val);
			printf("NEW_LEVEL:%d", device_info[0].device_level);
			send_color_temperature();
		}
    }
}

// Start a periodic timer to update color temp every 10 minutes
void init_color_temp_timer() {
	// Create a timer
	const esp_timer_create_args_t timer_args = {
		.callback = &update_color_temp_timer_callback,
		// name is optional, but may help identify the timer when debugging
		.name = "color_temp_timer"
	};
	ESP_ERROR_CHECK(esp_timer_create(&timer_args, &my_timer_handle));
	ESP_ERROR_CHECK(esp_timer_start_periodic(my_timer_handle, UPDATE_INTERVAL * 100)); // 1 minute milli second period
}
void start_color_temp_timer() {
	// Start the timer
	//ESP_ERROR_CHECK(esp_timer_start_periodic(my_timer_handle, UPDATE_INTERVAL * 100)); // 1 minute milli second period	
}
void stop_color_temp_timer(){
	//esp_timer_stop(my_timer_handle);
}
#endif


