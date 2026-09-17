#ifndef _APP_CONFIG_H_
    #define _APP_CONFIG_H_
    
    #define DEVICE_CCT_DALI_CUSTOM                                  1 // customized clusters for Tuya
    #define DEVICE_RGB_DALI                                         2 // SW1=DALI RED, SW2=DALI GREEN, SW3=DALI BLUE, SW4=DALI WHITE
    #define DEVICE_SCENE_DALI                                       4 // SW1=SCENE1, SW2=SCENE2, SW3=SCENE3, SW4=SCENE4
    #define DEVICE_DALI_DIRECT_SWITCH                               5 // SW1=ON/OFF, SW2=ON/OFF for 2 endpoints 2 buttons, 2 leds

    // TS004F (instead of TS0044) buttons has only Single Tap
    #define USE_NUOS_ZB_DEVICE_TYPE                                 DEVICE_DALI_DIRECT_SWITCH

    #define SETUP_LONG_PRESS_TIME_IN_SECS                           10

    //#define USE_NVS_INIT
    //#define DONT_USE_ZIGBEE                                       // For Expo or Demo Only   
    #define NEW_SDK_6
    //#define USE_OTA

    #define USE_TUYA_BRDIGE

    //#define WRITE_NVS_CONFIG
   // #define ZB_FACTORY_RESET
    #define USE_TRIPLE_CLICK
    #define USE_DOUBLE_CLICK

    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI)
        #define COLOR_RGB_ONLY                                      1   //3 channels
        #define COLOR_RGBW                                          2   //4 channels
        #define COLOR_RGB_CW_WW                                     3   //5 channels

        #define USE_COLOR_DEVICE                                    COLOR_RGB_CW_WW

        #define COMM_MODE_BROADCAST                                 1
        #define COMM_MODE_GROUP_CTRL                                2
        #define COMM_MODE_ADDR_CTRL                                 3

        #define COMMUNICATION_MODE                                  COMM_MODE_ADDR_CTRL
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
        #define COMM_MODE_BROADCAST                                 1
        #define COMM_MODE_GROUP_CTRL                                2
        #define COMM_MODE_ADDR_CTRL                                 3

        #define COMMUNICATION_MODE                                  COMM_MODE_BROADCAST   
    #endif
#endif