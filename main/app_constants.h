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
    #define MIN_CCT_VALUE_4                                         5500
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
    #define TASK_PRIORITY_DALI_TASK                                 5   //9   //used when in WiFi mode and zigbee off 
    #define TASK_PRIORITY_SWITCH                                    10  //dont change it ever
    #define TASK_PRIORITY_SCENE_RECALL                              15  //Not in use
    #define TASK_PRIORITY_ATTR                                      9   //dont change it ever
    #define TASK_PRIORITY_RGB                                       18  //dont change it ever
    #define TASK_PRIORITY_PRIVILEGE                                 21  //used in case of RGB
    #define TASK_PRIORITY_IR_BLASTER_SEND                           21
    #define TASK_PRIORITY_DALI_RX_FRAME                             12   
    #define TASK_PRIORITY_BOOT_PIN_TOGGLE                           22
    #define TASK_PRIORITY_SCENE_QUEUE_TASK                          15 //not used
    #define TASK_PRIORITY_WEBSERVER                                 16  
    #define TASK_PRIORITY_UART_RECEIVER                             17  //used only for IR Blaster
    //#define TASK_PRIORITY_DALI_RX_INTR_TASK                         11
    //#define TASK_PRIORITY_DALI_RX_FRAME_QUEUE                       13


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
    #define TASK_STACK_SIZE_DALI_RX_FRAME                           4096

    #define TASK_STACK_SIZE_SCENE_RECALL                            8192
    #define TASK_STACK_SIZE_IDENTIFY                                2048
    #define TASK_STACK_SIZE_ZB_FIND_NODES                           14000
    #define TASK_STACK_SIZE_SAVE_NODES                              4096
    #define TASK_STACK_SIZE_BIND_NODES                              8192
    #define TASK_STACK_SIZE_UNBIND_NODES                            8192

    #define DALI_INVALID_ADDRESS                                    0xFF

    #define MIREDS_MIN                                              156
    #define MIREDS_MAX                                              500
    

    #define LED_RED_COLOR                                           0x02
    #define LED_GREEN_COLOR                                         0x02
    #define LED_BLUE_COLOR                                          0x02
    #define LED_ORANGE_COLOR                                        0x01

    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
        //#define USE_TWO_SWITCH_MODE
        #define TOTAL_ENDPOINTS                                     1
        #ifndef USE_TWO_SWITCH_MODE
            #define TOTAL_BUTTONS                                   4
            #define TOTAL_LEDS                                      4
        #else
            #define TOTAL_BUTTONS                                   2
            #define TOTAL_LEDS                                      2
        #endif
        #define TOTAL_LOADS                                         2  //dali pins Tx, Rx
        #define TOTAL_LEDS_SHOW_ON_COMMISSIONING                    4      
        #define CHIP_INFO                                           USE_ESP32C6_MINI1_V5
        #define FAN_INDEX                                           1 
        #define ENABLE_PWM_DIMMING
        #define USE_RGB_LED
       
        // #ifndef USE_TWO_SWITCH_MODE
        // #define USE_CCT_TIME_SYNC
        // #endif
        #define USE_ADD_GROUP_SCENE_CLUSTERS 

        #if(CHIP_INFO == USE_ESP32C6_MINI1_V2 || CHIP_INFO == USE_ESP32C6_MINI1_V3 || CHIP_INFO == USE_ESP32C6_MINI1_V4 || CHIP_INFO == USE_ESP32C6_MINI1_V5 || CHIP_INFO == USE_ESP32C6_MINI1)
            #define USE_WIFI_WEBSERVER
            #define TEMPERATURE_MULTIPLICATION_FACTOR                   100
            #define WIFI_REMAIN_ACTIVE_IN_MINUTES                       10 //Change this value

            #define TIME_PERIOD_IN_MS                                   100
            #define TIME_COUNTS_FOR_1_SEC                               (1000 / TIME_PERIOD_IN_MS)

            #define WIFI_REMAIN_ACTIVE_IN_SECONDS                       (60 * WIFI_REMAIN_ACTIVE_IN_MINUTES)
            #define WIFI_REMAIN_ACTIVE_IN_COUNTS                        600
            
            #define ESP_WIFI_SSID                                       "NUOS DALI CCT SWITCH"
            #define ESP_WIFI_PASS                                       "NUOS1234"
        #endif 

        #define LONG_PRESS_BRIGHTNESS_ENABLE   
             
        #define BRIGHTNESS_SET_CHECKER_COUNTS                       2 
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
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI)
        #define TOTAL_ENDPOINTS                                     4
        #define TOTAL_BUTTONS                                       4
        #define TOTAL_LEDS                                          4

        #define USE_C3_ADAPTER_UART_HW

        #define TOTAL_LOADS                                             2  //dali pins Tx, Rx
        #define USE_RGB_LED

        #ifdef USE_TUYA_BRDIGE
            #define TUYA_ATTRIBUTES
        #endif

        #define LONG_PRESS_BRIGHTNESS_ENABLE
        #define DIMMING_STEPS                                       5
        #define BRIGHTNESS_SET_CHECKER_COUNTS                       15

        #define COLOR_CHANGE_STEPS                                  500
        #define COLOR_SET_CHECKER_COUNTS                            20

        #define MIN_CCT_VALUE                                       2000
        #define MAX_CCT_VALUE                                       6500

        #define USE_ADD_GROUP_SCENE_CLUSTERS
        #define CHIP_INFO                                               USE_ESP32C6_MINI1_V5
        #define MIN_DIM_LEVEL_VALUE                                     10 // (0 to 255)
        #define MAX_DIM_LEVEL_VALUE                                     254

        #define TOTAL_LEDS_SHOW_ON_COMMISSIONING                        1

        #if(CHIP_INFO == USE_ESP32C6_MINI1_V2 || CHIP_INFO == USE_ESP32C6_MINI1_V3 || CHIP_INFO == USE_ESP32C6_MINI1_V4 || CHIP_INFO == USE_ESP32C6_MINI1_V5 || CHIP_INFO == USE_ESP32C6_MINI1)
            #define USE_WIFI_WEBSERVER
            #define TEMPERATURE_MULTIPLICATION_FACTOR                   100


            #define WIFI_REMAIN_ACTIVE_IN_MINUTES                       5 //Change this value

            #define TIME_PERIOD_IN_MS                                   100
            #define TIME_COUNTS_FOR_1_SEC                               (1000 / TIME_PERIOD_IN_MS)

            #define WIFI_REMAIN_ACTIVE_IN_SECONDS                       (60 * WIFI_REMAIN_ACTIVE_IN_MINUTES)
            #define WIFI_REMAIN_ACTIVE_IN_COUNTS                        300

            #ifdef DALI_DIRECT_ADDRESSING
            #define ESP_WIFI_SSID                                       "NUOS DALI DIRECT SWITCH"
            #else
            #define ESP_WIFI_SSID                                       "NUOS DALI SCENE SWITCH"
            #endif
            #define ESP_WIFI_PASS                                       "NUOS1234"
        #endif
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_DALI_DIRECT_SWITCH)

        #define USE_COLOR_CONTROL

        #define TOTAL_ENDPOINTS                                         2
        #ifdef USE_COLOR_CONTROL 
        #define TOTAL_BUTTONS                                           4
        #define TOTAL_LEDS                                              4
        #else
        #define TOTAL_BUTTONS                                           2
        #define TOTAL_LEDS                                              2
        #endif
        #define TOTAL_LOADS                                             2  //dali pins Tx, Rx 
        #define USE_RGB_LED

        #ifdef USE_TUYA_BRDIGE
            #define TUYA_ATTRIBUTES
        #endif


        #define LONG_PRESS_BRIGHTNESS_ENABLE

        #define DIMMING_STEPS                                           5
        #define BRIGHTNESS_SET_CHECKER_COUNTS                           2
        #define DIMMING_LAST_REACH_OFFSET                               50

        #define COLOR_SET_CHECKER_COUNTS                                5
        #define COLOR_STEPS                                             100 
        #define COLOR_LAST_REACH_OFFSET                                 500     

        #define USE_ADD_GROUP_SCENE_CLUSTERS
        #define CHIP_INFO                                               USE_ESP32C6_MINI1_V5
        #define MIN_DIM_LEVEL_VALUE                                     10 // (0 to 255)
        #define MAX_DIM_LEVEL_VALUE                                     254   
        
        #define TOTAL_LEDS_SHOW_ON_COMMISSIONING                        1
        #define ENABLE_PWM_DIMMING

  
        #if(CHIP_INFO == USE_ESP32C6_MINI1_V2 || CHIP_INFO == USE_ESP32C6_MINI1_V3 || CHIP_INFO == USE_ESP32C6_MINI1_V4 || CHIP_INFO == USE_ESP32C6_MINI1_V5 || CHIP_INFO == USE_ESP32C6_MINI1)
            #define USE_WIFI_WEBSERVER
            #define TEMPERATURE_MULTIPLICATION_FACTOR                   100
            

            #define WIFI_REMAIN_ACTIVE_IN_MINUTES                       5 //Change this value

            #define TIME_PERIOD_IN_MS                                   100
            #define TIME_COUNTS_FOR_1_SEC                               (1000 / TIME_PERIOD_IN_MS)

            #define WIFI_REMAIN_ACTIVE_IN_SECONDS                       (60 * WIFI_REMAIN_ACTIVE_IN_MINUTES)
            #define WIFI_REMAIN_ACTIVE_IN_COUNTS                        600
            

            #define ESP_WIFI_SSID                                       "NUOS DALI DIRECT SWITCH"
            #define ESP_WIFI_PASS                                       "NUOS1234"
        #endif                
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI)
        #define TOTAL_ENDPOINTS                                     1
        #define TOTAL_BUTTONS                                       4
        #define TOTAL_LEDS                                          4

        #define TOTAL_LOADS                                         2

        #ifdef USE_TUYA_BRDIGE
            #define TUYA_ATTRIBUTES
        #endif
        #define ENABLE_DALI_RECEIVER
        #define USE_RGB_LED
        #define USE_ADD_GROUP_SCENE_CLUSTERS
        #define TOTAL_LEDS_SHOW_ON_COMMISSIONING                    1
        #define CHIP_INFO                                           USE_ESP32C6_MINI1_V5

        #define LONG_PRESS_BRIGHTNESS_ENABLE

        #define BRIGHTNESS_SET_CHECKER_COUNTS                       1
        #define DIMMING_STEPS                                       5
        //#define DIMMING_RGB_STEPS                                   20

        #define COLOR_SET_CHECKER_COUNTS                            2
        #define COLOR_STEPS                                         200

        #define MIN_DIM_LEVEL_VALUE                                 5  // (0 to 254)
        #define MAX_DIM_LEVEL_VALUE                                 254

        #define USE_ZIGBE_DEVICE_CATEGORY                           CATEGORY_ZIGBEE_DMX_LIGHT
        #if(CHIP_INFO == USE_ESP32C6_MINI1_V2 || CHIP_INFO == USE_ESP32C6_MINI1_V3 || CHIP_INFO == USE_ESP32C6_MINI1_V4 || CHIP_INFO == USE_ESP32C6_MINI1_V5 || CHIP_INFO == USE_ESP32C6_MINI1)
            #define USE_WIFI_WEBSERVER
            #define TEMPERATURE_MULTIPLICATION_FACTOR                   100


            #define WIFI_REMAIN_ACTIVE_IN_MINUTES                       5 //Change this value

            #define TIME_PERIOD_IN_MS                                   100
            #define TIME_COUNTS_FOR_1_SEC                               (1000 / TIME_PERIOD_IN_MS)

            #define WIFI_REMAIN_ACTIVE_IN_SECONDS                       (60 * WIFI_REMAIN_ACTIVE_IN_MINUTES)
            #define WIFI_REMAIN_ACTIVE_IN_COUNTS                        600


            #define ESP_WIFI_SSID                                       "NUOS DALI RGB LIGHT"

            #define ESP_WIFI_PASS                                       "NUOS1234"
        #endif
    #endif

                       
#endif
 