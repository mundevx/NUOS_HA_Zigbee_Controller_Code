#ifndef _APP_CONSTANTS_H_
    #define _APP_CONSTANTS_H_

    #include "app_config.h"

    // esp32h2 Hardware Tutorial Links
    //https://electronics.stackexchange.com/questions/713087/esp32-h2-doesnt-work-fine-when-powered-via-external-power-supply-and-3v3-batter
    //https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32h2/schematic-checklist.html
    //https://docs.espressif.com/projects/esptool/en/latest/esp32h2/advanced-topics/boot-mode-selection.html
    ///////////////////////////////////////////////////////////////////////////////////
    // 4t - r4
    // 2t - r3
    // c15
    #define USE_ESP32C6                                             1
    #define USE_ESP32H2                                             2
    #define USE_ESP32C6_MINI1                                       3
    #define USE_ESP32H2_MINI1                                       4
    #define USE_ESP32H2_MINI1_V2                                    5   // PCb ver. 8 july month
    #define USE_ESP32C6_MINI1_V2                                    6   // PCb ver. 8 july month
    #define USE_ESP32H2_MINI1_V3                                    7   // PCb ver. 8 july month
    #define USE_ESP32C6_MINI1_V3                                    8   // PCb ver. 8 july month
    #define USE_ESP32H2_MINI1_V4                                    9   // PCb ver. 21 sep month
    #define USE_ESP32C6_MINI1_V4                                    10  // PCb ver. 21 sep month
    #define USE_ESP32H2_MINI1_V5                                    11  // PCb ver. 15 oct month
    #define USE_ESP32C6_MINI1_V5                                    12  // PCb ver. 15 oct month
    // 2.63, watchdog timer io9 pulse ...
    ///////////////////////////////////////////////////////////////////////////////////
    //#define ALL_TOUCH_LEDS_OFF
    #define TOUCH_LEDS_ALL_OFF_TIMEOUT                              60

    #define TUYA_MAX_BRIGHTNESS                                     1000

    #define LONG_PRESS_ENABLE_WIFI_WEB_SERVER_TIME_IN_SECS          10  //05 sec --only for IR Blaster
    #define LONG_PRESS_SET_COMMISSIONING_TIME_IN_SECS               20  //15 sec --only for IR Blaster

    #define MAX_TIME_TO_START_COMMISSIONING_ON_2_BUTTONS_PRESSED    10   //in seconds 
    #define TIMER_COMMISSIONING_LED_BLINK_COUNTS                    5   //500 msec
    #define COMMISSIONING_TIMEOUT                                   600 //60 seconds

    #define MAX_CCT_VALUE                                           6500
    #define MIN_CCT_VALUE_3                                         5000
    #define MIN_CCT_VALUE_2                                         4000  
    #define MIN_CCT_VALUE_1                                         3000
    #define MIN_CCT_VALUE                                           2000
    ///////////////////////////////////////////////////////////////////////////////////
 
    #define CATEGORY_ZIGBEE_LIGHT                                   1
    #define CATEGORY_ZIGBEE_LIGHT_FAN                               2
    #define CATEGORY_ZIGBEE_CURTAIN                                 3
    #define CATEGORY_ZIGBEE_THERMOSTAT                              4
    #define CATEGORY_ZIGBEE_SCENE_SWITCH                            5
    #define CATEGORY_ZIGBEE_SENSORS                                 6 
    #define CATEGORY_ZIGBEE_DALI_LIGHT                              7

    #define CURTAIN_OPEN                                            0
    #define CURTAIN_CLOSE                                           1
    #define CURTAIN_STOP                                            2

    #define OPEN_COVER_LEVEL                                        100
    #define CLOSE_COVER_LEVEL                                       0

    #define TASK_PRIORITY_ZIGBEE                                    5
    #define TASK_PRIORITY_SWITCH                                    10
    #define TASK_PRIORITY_SCENE_RECALL                              15
    #define TASK_PRIORITY_RGB                                       20
    #define TASK_PRIORITY_ATTR                                      18
    #define TASK_PRIORITY_PRIVILEGE                                 21
    #define TASK_PRIORITY_IR_BLASTER_SEND                           21
    #define TASK_PRIORITY_BOOT_PIN_TOGGLE                           24

    #define TASK_PRIORITY_IDENTIFY                                  22
    #define TASK_PRIORITY_ZB_FIND_NODES                             32
    #define TASK_PRIORITY_ZB_SAVE_NODES                             22
    #define TASK_PRIORITY_ZB_BIND_NODES                             30
    #define TASK_PRIORITY_ZB_UNBIND_NODES                           30

    // z10. dali6 4 group,cct, rgbcw, //2mode scene
    #define TASK_STACK_SIZE_ZIGBEE                                  8192
    #define TASK_STACK_SIZE_SWITCH                                  4096
    #define TASK_STACK_SIZE_RGB                                     2048
    #define TASK_STACK_SIZE_WDT                                     2048 
    #define TASK_STACK_SIZE_COLOR                                   6800 
    #define TASK_STACK_SIZE_PRIVILEGE                               4096 
    #define TASK_STACK_SIZE_IR_BLASTER_SEND                         4096

    #define TASK_STACK_SIZE_SCENE_RECALL                            8192
    #define TASK_STACK_SIZE_IDENTIFY                                2048
    #define TASK_STACK_SIZE_ZB_FIND_NODES                           14000
    #define TASK_STACK_SIZE_SAVE_NODES                              4096
    #define TASK_STACK_SIZE_BIND_NODES                              8192
    #define TASK_STACK_SIZE_UNBIND_NODES                            8192

    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4R_ON_OFF_LIGHT)
        #define TOTAL_ENDPOINTS                                     4
        #define TOTAL_BUTTONS                                       4
        #define TOTAL_LEDS                                          4
        #define TOTAL_LOADS                                         4
        #define USE_ADD_GROUP_SCENE_CLUSTERS
        #define TOTAL_LEDS_SHOW_ON_COMMISSIONING                    4

        #define CHIP_INFO                                           USE_ESP32H2_MINI1_V5
        #ifndef DONT_USE_ZIGBEE
        #define USE_RGB_LED
        #endif     
        #define USE_ZIGBE_DEVICE_CATEGORY                           CATEGORY_ZIGBEE_LIGHT  
        #define MIN_DIM_LEVEL_VALUE                                 25 // (0 to 255)
        #define MAX_DIM_LEVEL_VALUE                                 254
        //#define CHANGE_LOAD_PIN
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2R_ON_OFF_LIGHT)
        #define TOTAL_ENDPOINTS                                     2
        #define TOTAL_BUTTONS                                       2
        #define TOTAL_LEDS                                          2
        #define TOTAL_LOADS                                         2
        #define USE_ADD_GROUP_SCENE_CLUSTERS 
        #define TOTAL_LEDS_SHOW_ON_COMMISSIONING                    2  
        #define CHIP_INFO                                           USE_ESP32H2_MINI1_V5
        #ifndef DONT_USE_ZIGBEE
        #define USE_RGB_LED
        #endif
        #define MIN_DIM_LEVEL_VALUE                                 25 // (0 to 255)
        #define MAX_DIM_LEVEL_VALUE                                 254        
        #define USE_ZIGBE_DEVICE_CATEGORY                           CATEGORY_ZIGBEE_LIGHT
        //#define CHANGE_LOAD_PIN
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2CH_CURTAIN)
        #define TOTAL_ENDPOINTS                                     2
        #define TOTAL_BUTTONS                                       4
        #define TOTAL_LEDS                                          4
        #define TOTAL_LOADS                                         4
        //#define USE_ADD_GROUP_SCENE_CLUSTERS
        #define TOTAL_LEDS_SHOW_ON_COMMISSIONING                    4
        #define CHIP_INFO                                           USE_ESP32H2_MINI1_V5
        #define MIN_DIM_LEVEL_VALUE                                 25 // (0 to 255)
        #define MAX_DIM_LEVEL_VALUE                                 254        
        #define USE_ZIGBE_DEVICE_CATEGORY                           CATEGORY_ZIGBEE_CURTAIN 
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN_SWITCH)
        #define TOTAL_ENDPOINTS                                     1
        #define TOTAL_BUTTONS                                       2
        #define TOTAL_LEDS                                          2
        #define TOTAL_LOADS                                         2
        #define USE_ADD_GROUP_SCENE_CLUSTERS 
        #define TOTAL_LEDS_SHOW_ON_COMMISSIONING                    1
        #define CHIP_INFO                                           USE_ESP32H2_MINI1_V5  
        #define USE_ZIGBE_DEVICE_CATEGORY                           CATEGORY_ZIGBEE_CURTAIN
        #define MIN_DIM_LEVEL_VALUE                                 10 // (0 to 254)
        #define MAX_DIM_LEVEL_VALUE                                 254     
        #define CHANGE_LOAD_PIN         
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN)
        #define TOTAL_ENDPOINTS                                     1
        #define TOTAL_BUTTONS                                       2
        #define TOTAL_LEDS                                          2
        #define TOTAL_LOADS                                         2
        #define USE_ADD_GROUP_SCENE_CLUSTERS 
        #define TOTAL_LEDS_SHOW_ON_COMMISSIONING                    1
        #define CHIP_INFO                                           USE_ESP32C6_MINI1_V5 //USE_ESP32H2_MINI1_V5
        #if(CHIP_INFO == USE_ESP32C6_MINI1_V2 || CHIP_INFO == USE_ESP32C6_MINI1_V3 || CHIP_INFO == USE_ESP32C6_MINI1_V4 || CHIP_INFO == USE_ESP32C6_MINI1_V5 || CHIP_INFO == USE_ESP32C6_MINI1)
            #define USE_WIFI_WEBSERVER
           
            #define WIFI_REMAIN_ACTIVE_IN_MINUTES  	                    10 //change this value

            #define TIME_PERIOD_IN_MS                                   100
            #define TIME_COUNTS_FOR_1_SEC                               (1000/TIME_PERIOD_IN_MS)

            #define WIFI_REMAIN_ACTIVE_IN_SECONDS                       (60 * WIFI_REMAIN_ACTIVE_IN_MINUTES)
            #define WIFI_REMAIN_ACTIVE_IN_COUNTS                        600
            #define TOTAL_LEDS_SHOW_ON_COMMISSIONING                    1
        
            #define ESP_WIFI_SSID      				                    "NUOS CURTAIN CONTROLLER"
            #define ESP_WIFI_PASS      				                    "NUOS1234"
        #endif  

        #ifndef DONT_USE_ZIGBEE
        #define USE_RGB_LED
        #endif  
        #define USE_ZIGBE_DEVICE_CATEGORY                           CATEGORY_ZIGBEE_CURTAIN
        #define MIN_DIM_LEVEL_VALUE                                 25 // (0 to 255)
        #define MAX_DIM_LEVEL_VALUE                                 254
        #define CHANGE_LOAD_PIN                                     
        #define TUYA_ATTRIBUTES
        #define LONG_PRESS_BRIGHTNESS_ENABLE   
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4T_ANALOG_DIMMABLE_LIGHT)
        #define TOTAL_ENDPOINTS                                     4
        #define TOTAL_BUTTONS                                       4
        #define TOTAL_LEDS                                          4
        #define TOTAL_LOADS                                         4
        //#define USE_ADD_GROUP_SCENE_CLUSTERS
        #define TOTAL_LEDS_SHOW_ON_COMMISSIONING                    1   
        #define CHIP_INFO                                           USE_ESP32H2_MINI1_V5
        #define LONG_PRESS_BRIGHTNESS_ENABLE        
        #define BRIGHTNESS_SET_CHECKER_COUNTS                       4   
        #define DIMMING_STEPS                                       1    
        #define ENABLE_PWM_DIMMING 
        #define MIN_DIM_LEVEL_VALUE                                 25 // (0 to 255)
        #define MAX_DIM_LEVEL_VALUE                                 254
        #define USE_ZIGBE_DEVICE_CATEGORY                           CATEGORY_ZIGBEE_LIGHT                                                                                        
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_ANALOG_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_PHASE_CUT_DIMMABLE_LIGHT)
        #define TOTAL_ENDPOINTS                                     2
        #define TOTAL_BUTTONS                                       2
        #define TOTAL_LEDS                                          2
        #define TOTAL_LOADS                                         2
        #define USE_ADD_GROUP_SCENE_CLUSTERS  
        #define TOTAL_LEDS_SHOW_ON_COMMISSIONING                    1  
        #define CHIP_INFO                                           USE_ESP32H2_MINI1_V5
        #ifndef DONT_USE_ZIGBEE
        #define USE_RGB_LED
        #endif       
        #define USE_FADING      
        #define FAN_INDEX                                           1
        #define LONG_PRESS_BRIGHTNESS_ENABLE        
        #define BRIGHTNESS_SET_CHECKER_COUNTS                       2//8 
        #define DIMMING_STEPS                                       1    
        #define ENABLE_PWM_DIMMING           
        #define MIN_DIM_LEVEL_VALUE                                 10 // (0 to 255)
        #define MAX_DIM_LEVEL_VALUE                                 254
        #define USE_ZIGBE_DEVICE_CATEGORY                           CATEGORY_ZIGBEE_LIGHT
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4T_COLOR_DIMMABLE_LIGHT)
        #define TOTAL_ENDPOINTS                                     4
        #define TOTAL_BUTTONS                                       4
        #define TOTAL_LEDS                                          4
        #define TOTAL_LOADS                                         4 
        #define USE_DALI 
        //#define USE_ADD_GROUP_SCENE_CLUSTERS
        #define TOTAL_LEDS_SHOW_ON_COMMISSIONING                    1 
        #define CHIP_INFO                                           USE_ESP32C6_MINI1_V4
        #define LONG_PRESS_BRIGHTNESS_ENABLE        
        #define BRIGHTNESS_SET_CHECKER_COUNTS                       5 
        #define ENABLE_PWM_DIMMING   
        #define MIN_DIM_LEVEL_VALUE                                 25 // (0 to 255)
        #define MAX_DIM_LEVEL_VALUE                                 254
        #define USE_ZIGBE_DEVICE_CATEGORY                           CATEGORY_ZIGBEE_LIGHT                
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_COLOR_DIMMABLE_LIGHT)
        #define TOTAL_ENDPOINTS                                     2
        #define TOTAL_BUTTONS                                       2
        #define TOTAL_LEDS                                          2
        #define TOTAL_LOADS                                         2
        #define USE_DALI
        //#define USE_ADD_GROUP_SCENE_CLUSTERS
        #define TOTAL_LEDS_SHOW_ON_COMMISSIONING                    1 
        #define CHIP_INFO                                           USE_ESP32C6_MINI1_V4
        #define LONG_PRESS_BRIGHTNESS_ENABLE        
        #define BRIGHTNESS_SET_CHECKER_COUNTS                       5  
        #define DIMMING_STEPS                                       20 
        #define ENABLE_PWM_DIMMING   
        #define MIN_DIM_LEVEL_VALUE                                 25 // (0 to 255)
        #define MAX_DIM_LEVEL_VALUE                                 254
        #define USE_ZIGBE_DEVICE_CATEGORY                           CATEGORY_ZIGBEE_LIGHT
                       
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN)
        #define USE_ZB_ONLY_FAN
        #ifdef USE_TUYA_BRDIGE
            #define TOTAL_ENDPOINTS                                     1
            #define FAN_INDEX                                           0
            #define LIGHT_INDEX                                         1         
        #else
            #ifdef USE_ZB_ONLY_FAN
            #define TOTAL_ENDPOINTS                                     1
            #define FAN_INDEX                                           0
            #define LIGHT_INDEX                                         1
            #else
            #define TOTAL_ENDPOINTS                                     2
            #define FAN_INDEX                                           1
            #define LIGHT_INDEX                                         0
            #endif         
        #endif
        #define TOTAL_BUTTONS                                       4
        #define TOTAL_LEDS                                          4
        #define TOTAL_LOADS                                         4  
        #define USE_ADD_GROUP_SCENE_CLUSTERS 
        #define TOTAL_LEDS_SHOW_ON_COMMISSIONING                    1      
        #define CHIP_INFO                                           USE_ESP32H2_MINI1_V5

        #ifndef DONT_USE_ZIGBEE
        #define USE_RGB_LED
        #endif     
        #define MIN_DIM_LEVEL_VALUE                                 25 // (0 to 255)
        #define MAX_DIM_LEVEL_VALUE                                 254
        #define USE_ZIGBE_DEVICE_CATEGORY                           CATEGORY_ZIGBEE_LIGHT_FAN   
        #ifdef USE_TUYA_BRDIGE
            #define TUYA_ATTRIBUTES
        #endif   
        
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN_CUSTOM)
        #define TOTAL_ENDPOINTS                                     2
        #define TOTAL_BUTTONS                                       4
        #define TOTAL_LEDS                                          4
        #define TOTAL_LOADS                                         4  
        #define USE_ADD_GROUP_SCENE_CLUSTERS 
        #define TOTAL_LEDS_SHOW_ON_COMMISSIONING                    1      
        #define CHIP_INFO                                           USE_ESP32H2_MINI1_V5
        #define LIGHT_INDEX                                         0 
        #define FAN_INDEX                                           1
        #ifndef DONT_USE_ZIGBEE
        #define USE_RGB_LED
        #endif
        #define MIN_DIM_LEVEL_VALUE                                 25 // (0 to 255)
        #define MAX_DIM_LEVEL_VALUE                                 254
        #define USE_ZIGBE_DEVICE_CATEGORY                           CATEGORY_ZIGBEE_LIGHT_FAN
          
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER)
        #define TOTAL_ENDPOINTS                                     1
        #define TOTAL_BUTTONS                                       1
        #define TOTAL_LEDS                                          1
        #define TOTAL_LOADS                                         0
        #define USE_RGB_LED

        #define USE_ADD_GROUP_SCENE_CLUSTERS
        #define CHIP_INFO                                           USE_ESP32H2_MINI1_V5
         #define TEMPERATURE_MULTIPLICATION_FACTOR                  100
        #if(CHIP_INFO == USE_ESP32C6_MINI1_V2 || CHIP_INFO == USE_ESP32C6_MINI1_V3 || CHIP_INFO == USE_ESP32C6_MINI1_V4 || CHIP_INFO == USE_ESP32C6_MINI1_V5 || CHIP_INFO == USE_ESP32C6_MINI1)
            #define USE_WIFI_WEBSERVER
           
            #define WIFI_REMAIN_ACTIVE_IN_MINUTES  	                    10 //change this value

            #define TIME_PERIOD_IN_MS                                   100
            #define TIME_COUNTS_FOR_1_SEC                               (1000/TIME_PERIOD_IN_MS)

            #define WIFI_REMAIN_ACTIVE_IN_SECONDS                       (60 * WIFI_REMAIN_ACTIVE_IN_MINUTES)
            #define WIFI_REMAIN_ACTIVE_IN_COUNTS                        600
            #define TOTAL_LEDS_SHOW_ON_COMMISSIONING                    1
        
            #define ESP_WIFI_SSID      				                    "NUOS IR BLASTER"
            #define ESP_WIFI_PASS      				                    "NUOS1234"
        #endif // 11230a
        #define MIN_DIM_LEVEL_VALUE                                 25 // (0 to 255)
        #define MAX_DIM_LEVEL_VALUE                                 254        
        //#define USE_HEATING_PIPES
        //#define USE_TEMPERATURE_MESAUREMENT
        
        #ifdef USE_TUYA_BRDIGE
            #define TUYA_ATTRIBUTES
            #define USE_LOCAL_TEMPERATURE
            #define USE_WEEKLY_TRANSITION
            //#define USE_FAN_CTRL
        #endif             
        #define USE_ZIGBE_DEVICE_CATEGORY                           CATEGORY_ZIGBEE_THERMOSTAT                                    
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
        #define TOTAL_ENDPOINTS                                     1
        #define TOTAL_BUTTONS                                       1
        #define TOTAL_LEDS                                          1
        #define TOTAL_LOADS                                         0
        //#define USE_RGB_LED
        #define USE_IR_UART_WS4_HW

        #define USE_ADD_GROUP_SCENE_CLUSTERS
        #define CHIP_INFO                                           USE_ESP32H2_MINI1_V5
        #define MIN_DIM_LEVEL_VALUE                                 25 // (0 to 255)
        #define MAX_DIM_LEVEL_VALUE                                 254      
        
        #ifndef USE_IR_UART_WS4_HW
            #if(CHIP_INFO == USE_ESP32C6_MINI1_V2 || CHIP_INFO == USE_ESP32C6_MINI1_V3 || CHIP_INFO == USE_ESP32C6_MINI1_V4 || CHIP_INFO == USE_ESP32C6_MINI1_V5 || CHIP_INFO == USE_ESP32C6_MINI1)
                #define USE_WIFI_WEBSERVER
                #define TEMPERATURE_MULTIPLICATION_FACTOR                   100
                

                #define WIFI_REMAIN_ACTIVE_IN_MINUTES  	                    10 //Change this value

                #define TIME_PERIOD_IN_MS                                   100
                #define TIME_COUNTS_FOR_1_SEC                               (1000 / TIME_PERIOD_IN_MS)

                #define WIFI_REMAIN_ACTIVE_IN_SECONDS                       (60 * WIFI_REMAIN_ACTIVE_IN_MINUTES)
                #define WIFI_REMAIN_ACTIVE_IN_COUNTS                        600
                #define TOTAL_LEDS_SHOW_ON_COMMISSIONING                    1
            
                #define ESP_WIFI_SSID      				                    "NUOS IR BLASTER"
                #define ESP_WIFI_PASS      				                    "NUOS1234"
            #endif
        #endif
        #define USE_ZIGBE_DEVICE_CATEGORY                           CATEGORY_ZIGBEE_THERMOSTAT   
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_MOTION || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_CONTACT_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_GAS_LEAK)
        #define TOTAL_ENDPOINTS                                     1     
        #define TOTAL_BUTTONS                                       1
        #define TOTAL_LEDS                                          2 
        #define TOTAL_LOADS                                         1 
        #define IS_REPORT_BATTERY_STATUS
        #define USE_ADD_GROUP_SCENE_CLUSTERS
        #define CHIP_INFO                                           USE_ESP32H2_MINI1_V5
        #define TOTAL_LEDS_SHOW_ON_COMMISSIONING                    1
        #define IS_BATTERY_POWERED
        #define USE_TEMPER_ALARM 
        #define USE_ZIGBE_DEVICE_CATEGORY                           CATEGORY_ZIGBEE_SENSORS
        #define MIN_DIM_LEVEL_VALUE                                 25 // (0 to 255)
        #define MAX_DIM_LEVEL_VALUE                                 254        
        //1A 51 64          
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RINGING_BELL_2)
        #define TOTAL_ENDPOINTS                                     1     
        #define TOTAL_BUTTONS                                       2
        #define TOTAL_LEDS                                          2 
        #define TOTAL_LOADS                                         1 
        #define IS_REPORT_BATTERY_STATUS
        //#define USE_ADD_GROUP_SCENE_CLUSTERS
        #define CHIP_INFO                                           USE_ESP32H2_MINI1_V5
        #define TOTAL_LEDS_SHOW_ON_COMMISSIONING                    1
        #define IS_BATTERY_POWERED
        #define USE_TEMPER_ALARM 
        #define USE_ZIGBE_DEVICE_CATEGORY                           CATEGORY_ZIGBEE_SENSORS
        #define USE_RGB_LED        
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH)
        #define TOTAL_ENDPOINTS                                     4
        #define TOTAL_BUTTONS                                       4
        #define TOTAL_LEDS                                          4
        #define TOTAL_LOADS                                         4   
        //#define USE_ADD_GROUP_SCENE_CLUSTERS
        #define CHIP_INFO                                           USE_ESP32C6_MINI1_V5
        #define TOTAL_LEDS_SHOW_ON_COMMISSIONING                    1       
        #define USE_RGB_LED                                         1
        // #define IS_END_DEVICE
        #define MIN_DIM_LEVEL_VALUE                                 25 // (0 to 255)
        #define MAX_DIM_LEVEL_VALUE                                 254  
        #if(CHIP_INFO == USE_ESP32C6_MINI1_V2 || CHIP_INFO == USE_ESP32C6_MINI1_V3 || CHIP_INFO == USE_ESP32C6_MINI1_V4 || CHIP_INFO == USE_ESP32C6_MINI1_V5 || CHIP_INFO == USE_ESP32C6_MINI1)
            #define USE_WIFI_WEBSERVER
            #define WIFI_REMAIN_ACTIVE_IN_MINUTES  	                    10 //change this value

            #define TIME_PERIOD_IN_MS                                   100
            #define TIME_COUNTS_FOR_1_SEC                               (1000 / TIME_PERIOD_IN_MS)

            #define WIFI_REMAIN_ACTIVE_IN_SECONDS                       (60 * WIFI_REMAIN_ACTIVE_IN_MINUTES)
            #define WIFI_REMAIN_ACTIVE_IN_COUNTS                        600


            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
                #define ESP_WIFI_SSID      				            "NUOS ZB REMOTE SCENE"
                #define ESP_WIFI_PASS      				            "NUOS1234"
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH )
                #define ESP_WIFI_SSID      				            "NUOS ZB SCENE SWITCH"
                #define ESP_WIFI_PASS      				            "NUOS1234"
                #define USE_DOUBLE_PRESS
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH )
                #define ESP_WIFI_SSID      				            "NUOS ZB GROUP SWITCH"
                #define ESP_WIFI_PASS      				            "NUOS1234"
            #endif
        #endif
        #define USE_ZIGBE_DEVICE_CATEGORY                           CATEGORY_ZIGBEE_SCENE_SWITCH
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_SCENE_SWITCH)
        #define TOTAL_ENDPOINTS                                     4
        #define TOTAL_BUTTONS                                       4
        #define TOTAL_LEDS                                          4
        #define TOTAL_LOADS                                         4   
        #define CHIP_INFO                                           USE_ESP32H2_MINI1_V5
        #define TOTAL_LEDS_SHOW_ON_COMMISSIONING                    1 
        #ifndef DONT_USE_ZIGBEE
        #define USE_RGB_LED
        #endif
        #define MIN_DIM_LEVEL_VALUE                                 25 // (0 to 255)
        #define MAX_DIM_LEVEL_VALUE                                 254    
        #define FAN_INDEX                                           1
        #define LIGHT_INDEX                                         0           
        #define USE_ZIGBE_DEVICE_CATEGORY                           CATEGORY_ZIGBEE_SCENE_SWITCH
        
        #define Mains_Powered_Scene_Switch    
        #ifdef USE_CUSTOM_SCENE
            #ifdef USE_TUYA_BRDIGE
                #define TUYA_ATTRIBUTES
            #endif
        #else
            #define IS_END_DEVICE    
        #endif
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_GROUP_SWITCH)
        #define TOTAL_ENDPOINTS                                     1
        #define TOTAL_BUTTONS                                       4
        #define TOTAL_LEDS                                          4
        #define TOTAL_LOADS                                         0   
        #define CHIP_INFO                                           USE_ESP32H2_MINI1_V3
        #define TOTAL_LEDS_SHOW_ON_COMMISSIONING                    1 

        #define USE_RGB_LED
        #define MIN_DIM_LEVEL_VALUE                                 10 // (0 to 255)
        #define MAX_DIM_LEVEL_VALUE                                 254    
        #define FAN_INDEX                                           1
        #define LIGHT_INDEX                                         0       
        #define USE_ZIGBE_DEVICE_CATEGORY                           CATEGORY_ZIGBEE_SCENE_SWITCH
        #define Mains_Powered_Scene_Switch   
        #define LONG_PRESS_BRIGHTNESS_ENABLE  
        #ifdef USE_CUSTOM_SCENE
            #ifdef USE_TUYA_BRDIGE
                #define TUYA_ATTRIBUTES
            #endif
        #else
            #define IS_END_DEVICE    
        #endif        
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY)
        #define TOTAL_ENDPOINTS                                     3     
        #define TOTAL_BUTTONS                                       0
        #define TOTAL_LEDS                                          2 
        #define TOTAL_LOADS                                         1   
        #define USE_ADD_GROUP_SCENE_CLUSTERS
        #define CHIP_INFO                                           USE_ESP32H2_MINI1_V5
        //#define USE_RGB_LED
        #define TOTAL_LEDS_SHOW_ON_COMMISSIONING                    1
        #define IS_BATTERY_POWERED
        #define USE_ZIGBE_DEVICE_CATEGORY                           CATEGORY_ZIGBEE_SENSORS
        #define MIN_DIM_LEVEL_VALUE                                 25 // (0 to 255)
        #define MAX_DIM_LEVEL_VALUE                                 254    
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY_CUSTOM)
        #define TOTAL_ENDPOINTS                                     1     
        #define TOTAL_BUTTONS                                       0
        #define TOTAL_LEDS                                          2 
        #define TOTAL_LOADS                                         1   
        //#define USE_ADD_GROUP_SCENE_CLUSTERS
        #define CHIP_INFO                                           USE_ESP32H2_MINI1_V5
        //#define USE_RGB_LED
        #define TOTAL_LEDS_SHOW_ON_COMMISSIONING                    1
        #define IS_BATTERY_POWERED
        #define USE_ZIGBE_DEVICE_CATEGORY                           CATEGORY_ZIGBEE_SENSORS 
        #define MIN_DIM_LEVEL_VALUE                                 25 // (0 to 255)
        #define MAX_DIM_LEVEL_VALUE                                 254    
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_LUX)
        #define TOTAL_ENDPOINTS                                     1     
        #define TOTAL_BUTTONS                                       0
        #define TOTAL_LEDS                                          2 
        #define TOTAL_LOADS                                         1   
        #define USE_ADD_GROUP_SCENE_CLUSTERS 
        #define CHIP_INFO                                           USE_ESP32H2_MINI1_V5
        //#define IS_BATTERY_POWERED
        #define USE_ZIGBE_DEVICE_CATEGORY                           CATEGORY_ZIGBEE_SENSORS
        #define MIN_DIM_LEVEL_VALUE                                 25 // (0 to 255)
        #define MAX_DIM_LEVEL_VALUE                                 254   
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI)
        #define TOTAL_ENDPOINTS                                     4
        #define TOTAL_BUTTONS                                       4
        #define TOTAL_LEDS                                          4
        #define TOTAL_LOADS                                         2  //dali pins Tx, Rx  
        //#define USE_ADD_GROUP_SCENE_CLUSTERS 
        #define TOTAL_LEDS_SHOW_ON_COMMISSIONING                    1      
        #define CHIP_INFO                                           USE_ESP32H2_MINI1_V5
        #define USE_RGB_LED

        #if(CHIP_INFO == USE_ESP32C6_MINI1_V2 || CHIP_INFO == USE_ESP32C6_MINI1_V3 || CHIP_INFO == USE_ESP32C6_MINI1_V4 || CHIP_INFO == USE_ESP32C6_MINI1_V5 || CHIP_INFO == USE_ESP32C6_MINI1)
        #define USE_WIFI_WEBSERVER
        //#define USE_DALI_SCENES
        #define WIFI_REMAIN_ACTIVE_IN_MINUTES  	                    10 //change this value
        
        #define TIME_PERIOD_IN_MS                                   100
        #define TIME_COUNTS_FOR_1_SEC                               (1000/TIME_PERIOD_IN_MS)

        #define WIFI_REMAIN_ACTIVE_IN_SECONDS                       (60 * WIFI_REMAIN_ACTIVE_IN_MINUTES)
        #define WIFI_REMAIN_ACTIVE_IN_COUNTS                        600



        #define ESP_WIFI_SSID      				                    "NUOS DALI TUNABLE LIGHT"
        #define ESP_WIFI_PASS      				                    "NUOS1234"

        #endif

        #define USE_DOUBLE_PRESS
        #define LONG_PRESS_BRIGHTNESS_ENABLE        
        #define BRIGHTNESS_SET_CHECKER_COUNTS                       5
        #define DIMMING_STEPS                                       2
        #define DALI_FADE_RATE                                      12
        #define DALI_FADE_TIME                                      0  /*0 for less than 1 second*/

        #define MIN_DIM_LEVEL_VALUE                                 25 // (0 to 255)
        #define MAX_DIM_LEVEL_VALUE                                 254
        #define USE_ZIGBE_DEVICE_CATEGORY                           CATEGORY_ZIGBEE_DALI_LIGHT
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
        #define TOTAL_ENDPOINTS                                     1
        #define TOTAL_BUTTONS                                       4
        #define TOTAL_LEDS                                          4
        #define TOTAL_LOADS                                         2  //dali pins Tx, Rx
        #define TOTAL_LEDS_SHOW_ON_COMMISSIONING                    4      
        #define CHIP_INFO                                           USE_ESP32H2_MINI1
        #define FAN_INDEX                                           1 
        #define ENABLE_PWM_DIMMING
        #define USE_RGB_LED
        #define USE_CCT_TIME_SYNC
        #define USE_ADD_GROUP_SCENE_CLUSTERS 
    #if(CHIP_INFO == USE_ESP32C6_MINI1_V2 || CHIP_INFO == USE_ESP32C6_MINI1_V3 || CHIP_INFO == USE_ESP32C6_MINI1_V4 || CHIP_INFO == USE_ESP32C6_MINI1_V5 || CHIP_INFO == USE_ESP32C6_MINI1)
        #define USE_WIFI_WEBSERVER
        #define WIFI_REMAIN_ACTIVE_IN_MINUTES  	                    10 //change this value
        #define TIME_PERIOD_IN_MS                                   100
        #define TIME_COUNTS_FOR_1_SEC                               (1000 / TIME_PERIOD_IN_MS)

        #define WIFI_REMAIN_ACTIVE_IN_SECONDS                       (60 * WIFI_REMAIN_ACTIVE_IN_MINUTES)
        #define WIFI_REMAIN_ACTIVE_IN_COUNTS                        600

        #define ESP_WIFI_SSID      				                    "NUOS DALI TUNABLE LIGHT"
        #define ESP_WIFI_PASS      				                    "NUOS1234"  
    #endif

        #define LONG_PRESS_BRIGHTNESS_ENABLE   
             
        #define BRIGHTNESS_SET_CHECKER_COUNTS                       20 
        #define DIMMING_STEPS                                       2 

        #define COLOR_SET_CHECKER_COUNTS                            20 
        #define COLOR_STEPS                                         200         

        #define DALI_FADE_RATE                                      12
        #define DALI_FADE_TIME                                      1

        #define MIN_DIM_LEVEL_VALUE                                 5 // (0 to 255)
        #define MAX_DIM_LEVEL_VALUE                                 254
        #define USE_ZIGBE_DEVICE_CATEGORY                           CATEGORY_ZIGBEE_DALI_LIGHT  
        #ifdef USE_TUYA_BRDIGE
            #define TUYA_ATTRIBUTES
        #endif
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI)
        #ifdef USE_INDIVIDUAL_DALI_ADDRESSING
            #define TOTAL_ENDPOINTS                                 10
            #define TOTAL_BUTTONS                                   1
            #define TOTAL_LEDS                                      1
        #else
            #define TOTAL_ENDPOINTS                                 4
            #define TOTAL_BUTTONS                                   4
            #define TOTAL_LEDS                                      4
        #endif

        #define TOTAL_LOADS                                         2  //dali pins Tx, Rx 
        
        #define USE_RGB_LED
        #ifdef USE_TUYA_BRDIGE
            #define TUYA_ATTRIBUTES
        #endif
        #define LONG_PRESS_BRIGHTNESS_ENABLE
        
        #define USE_ADD_GROUP_SCENE_CLUSTERS
        #define CHIP_INFO                                           USE_ESP32H2_MINI1_V5
        #define MIN_DIM_LEVEL_VALUE                                 20 // (0 to 255)
        #define MAX_DIM_LEVEL_VALUE                                 254   
        
        #define BRIGHTNESS_SET_CHECKER_COUNTS                       20
        #define DIMMING_STEPS                                       1
        #define TOTAL_LEDS_SHOW_ON_COMMISSIONING                    1

        #ifdef USE_INDIVIDUAL_DALI_ADDRESSING
            #define ENABLE_PWM_DIMMING
        #endif
        #if(CHIP_INFO == USE_ESP32C6_MINI1_V2 || CHIP_INFO == USE_ESP32C6_MINI1_V3 || CHIP_INFO == USE_ESP32C6_MINI1_V4 || CHIP_INFO == USE_ESP32C6_MINI1_V5 || CHIP_INFO == USE_ESP32C6_MINI1)
            #define USE_WIFI_WEBSERVER
            #define TEMPERATURE_MULTIPLICATION_FACTOR                   100
            

            #define WIFI_REMAIN_ACTIVE_IN_MINUTES  	                    10 //Change this value

            #define TIME_PERIOD_IN_MS                                   100
            #define TIME_COUNTS_FOR_1_SEC                               (1000 / TIME_PERIOD_IN_MS)

            #define WIFI_REMAIN_ACTIVE_IN_SECONDS                       (60 * WIFI_REMAIN_ACTIVE_IN_MINUTES)
            #define WIFI_REMAIN_ACTIVE_IN_COUNTS                        600
            
        
            #define ESP_WIFI_SSID      				                    "NUOS DALI GROUP LIGHT"
            #define ESP_WIFI_PASS      				                    "NUOS1234"
        #endif
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_CUSTOM)
        #define TOTAL_ENDPOINTS                                     3
        #define TOTAL_BUTTONS                                       4
        #define TOTAL_LEDS                                          4

        #define TOTAL_LOADS                                         1

        #ifdef USE_TUYA_BRDIGE
            #define TUYA_ATTRIBUTES
        #endif

        #define USE_RGB_LED
        #define USE_ADD_GROUP_SCENE_CLUSTERS 
        #define TOTAL_LEDS_SHOW_ON_COMMISSIONING                    1      
        #define CHIP_INFO                                           USE_ESP32C6_MINI1_V3

        // #if(CHIP_INFO == USE_ESP32C6_MINI1_V2 || CHIP_INFO == USE_ESP32C6_MINI1_V3 || CHIP_INFO == USE_ESP32C6_MINI1_V4 || CHIP_INFO == USE_ESP32C6_MINI1_V5 || CHIP_INFO == USE_ESP32C6_MINI1)
        //     #define USE_WIFI_WEBSERVER
        //     //#define USE_DALI_SCENES
        //     #define WIFI_REMAIN_ACTIVE_IN_MINUTES  	                    10 //change this value

        //     #define TIME_PERIOD_IN_MS                                   100
        //     #define TIME_COUNTS_FOR_1_SEC                               (1000/TIME_PERIOD_IN_MS)

        //     #define WIFI_REMAIN_ACTIVE_IN_SECONDS                       (60 * WIFI_REMAIN_ACTIVE_IN_MINUTES)
        //     #define WIFI_REMAIN_ACTIVE_IN_COUNTS                        600

        //     #define ESP_WIFI_SSID      				                    "NUOS DMX RGB LIGHT"
        //     #define ESP_WIFI_PASS      				                    "NUOS1234"  
        // #endif
        
        #define LONG_PRESS_BRIGHTNESS_ENABLE        
        #define BRIGHTNESS_SET_CHECKER_COUNTS                       5
        #define DIMMING_STEPS                                       1
  
        #define MIN_DIM_LEVEL_VALUE                                 10 // (0 to 255)
        #define MAX_DIM_LEVEL_VALUE                                 254 
        #define USE_ZIGBE_DEVICE_CATEGORY                           CATEGORY_ZIGBEE_DMX_LIGHT //

    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI)
        #define TOTAL_ENDPOINTS                                     1
        #define TOTAL_BUTTONS                                       4
        #define TOTAL_LEDS                                          4

        #define TOTAL_LOADS                                         2

        #ifdef USE_TUYA_BRDIGE
            #define TUYA_ATTRIBUTES
        #endif

        //#define USE_RGB_LED
        #define USE_ADD_GROUP_SCENE_CLUSTERS 
        #define TOTAL_LEDS_SHOW_ON_COMMISSIONING                    1      
        #define CHIP_INFO                                           USE_ESP32H2_MINI1_V5

        #define LONG_PRESS_BRIGHTNESS_ENABLE 
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)      
            #define BRIGHTNESS_SET_CHECKER_COUNTS                       20
            #define DIMMING_STEPS                                       1

            #define COLOR_SET_CHECKER_COUNTS                            15
            #define COLOR_STEPS                                         200     
            
            #define MIN_DIM_LEVEL_VALUE                                 10 // (0 to 254)
            #define MAX_DIM_LEVEL_VALUE                                 254  
        #else
            #define BRIGHTNESS_SET_CHECKER_COUNTS                       10
            #define DIMMING_STEPS                                       5

            #define COLOR_SET_CHECKER_COUNTS                            20
            #define COLOR_STEPS                                         200

            #define MIN_DIM_LEVEL_VALUE                                 20 // (0 to 254)
            #define MAX_DIM_LEVEL_VALUE                                 254  
        #endif
  

        #define USE_ZIGBE_DEVICE_CATEGORY                           CATEGORY_ZIGBEE_DMX_LIGHT
        //#define USE_DOUBLE_PRESS
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RINGING_BELL)
        #define TOTAL_ENDPOINTS                                     2
        #define TOTAL_BUTTONS                                       2
        #define TOTAL_LEDS                                          2
        #define TOTAL_LOADS                                         2   
        #define USE_ADD_GROUP_SCENE_CLUSTERS
        #define CHIP_INFO                                           USE_ESP32C6_MINI1_V3
        #define TOTAL_LEDS_SHOW_ON_COMMISSIONING                    1       
        #define USE_RGB_LED                                         1
        #define USE_ZIGBE_DEVICE_CATEGORY                           CATEGORY_ZIGBEE_SCENE_SWITCH          
    #endif

                       
#endif
 