#ifndef _APP_CONFIG_H_
    #define _APP_CONFIG_H_
    
    #define DEVICE_4R_ON_OFF_LIGHT 					                1
    #define DEVICE_2R_ON_OFF_LIGHT 					                2
    #define DEVICE_2T_ANALOG_DIMMABLE_LIGHT 			            3
    #define DEVICE_2T_PHASE_CUT_DIMMABLE_LIGHT 			            4
    #define DEVICE_1CH_CURTAIN 					                    5
    #define DEVICE_2CH_CURTAIN 					                    6  // rarely used

    #define DEVICE_1_LIGHT_1_FAN                                    7
    #define DEVICE_IR_BLASTER                                       8
    #define DEVICE_WIRELESS_SCENE_SWITCH                            9
  
    #define DEVICE_1_LIGHT_1_FAN_CUSTOM                             10 // customized clusters for Tuya
    #define DEVICE_IR_BLASTER_CUSTOM                                11 // customized clusters for Tuya

    #define DEVICE_CCT_DALI_CUSTOM                                  12 // customized clusters for Tuya
    #define DEVICE_RGB_DALI                                         13 // SW1=DALI RED, SW2=DALI GREEN, SW3=DALI BLUE, SW4=DALI WHITE
    #define DEVICE_RGB_DMX                                          14 
    #define DEVICE_GROUP_DALI                                       15 // SW1=GROUP1 DALI LIGHTS, SW2=GROUP2 DALI LIGHTS, SW3=GROUP3 DALI LIGHTS, SW4=GROUP4 DALI LIGHTS

    #define DEVICE_RINGING_BELL_2     					            16 //
    #define DEVICE_SENSOR_MOTION                                    17
    #define DEVICE_SENSOR_CONTACT_SWITCH                            18
    #define DEVICE_SENSOR_GAS_LEAK                                  19
    #define DEVICE_SENSOR_TEMPERATURE_HUMIDITY                      20
    #define DEVICE_SENSOR_LUX                                       21 // TSL2561

    #define DEVICE_WIRELESS_REMOTE_SWITCH                           22 // SW1=DEVICES_BIND, SW2=DEVICES_BIND, SW3=DEVICES_BIND, SW4=DEVICES_BIND
    #define DEVICE_1CH_CURTAIN_SWITCH 					            23 

    #define DEVICE_WIRELESS_GROUP_SWITCH                            24 // Currently Running Remote Switch
    #define DEVICE_SCENE_DALI                                       25 // SW1=SCENE1, SW2=SCENE2, SW3=SCENE3, SW4=SCENE4
    // TS004F (instead of TS0044) buttons has only Single Tap
    #define USE_NUOS_ZB_DEVICE_TYPE                                 DEVICE_RGB_DALI

    #define SETUP_LONG_PRESS_TIME_IN_SECS                           10
    //#define USE_NVS_INIT
    //#define DONT_USE_ZIGBEE                                          // For Expo or Demo Only   
    #define NEW_SDK_6
    //#define USE_OTA

    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER)
        //#define USE_HOME_ASSISTANT
        #define USE_TUYA_BRDIGE
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI)
            #define USE_INDIVIDUAL_DALI_ADDRESSING
        #endif
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_SCENE_SWITCH ||USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_GROUP_SWITCH)
        //for local scene switch
        #define USE_CUSTOM_SCENE   
        #define USE_TUYA_BRDIGE
    #else
        #define USE_TUYA_BRDIGE
    #endif

    //#define WRITE_NVS_CONFIG
    //#define ZB_FACTORY_RESET
    #define USE_TRIPLE_CLICK

    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
        #define COLOR_RGB_ONLY                                      1   //3 channels
        #define COLOR_RGBW                                          2   //4 channels
        #define COLOR_RGB_CW_WW                                     3   //5 channels

        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
            #define USE_COLOR_DEVICE                                    COLOR_RGB_ONLY
        #else
            #define USE_COLOR_DEVICE                                    COLOR_RGB_CW_WW  

            #define COMM_MODE_BROADCAST                                 1
            #define COMM_MODE_GROUP_CTRL                                2
            #define COMM_MODE_ADDR_CTRL                                 3

            #define COMMUNICATION_MODE                                  COMM_MODE_ADDR_CTRL
        #endif
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)    
        #define COMM_MODE_BROADCAST                                 1
        #define COMM_MODE_GROUP_CTRL                                2
        #define COMM_MODE_ADDR_CTRL                                 3

        #define COMMUNICATION_MODE                                  COMM_MODE_BROADCAST   
    #endif
#endif