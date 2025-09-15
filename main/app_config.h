#ifndef _APP_CONFIG_H_
    #define _APP_CONFIG_H_
    
    #define DEVICE_4R_ON_OFF_LIGHT 					                1
    #define DEVICE_2R_ON_OFF_LIGHT 					                2
    #define DEVICE_2T_ANALOG_DIMMABLE_LIGHT 			            3
    #define DEVICE_2T_PHASE_CUT_DIMMABLE_LIGHT 			            25
    #define DEVICE_1CH_CURTAIN 					                    4
    #define DEVICE_2CH_CURTAIN 					                    5  //rarely used

    #define DEVICE_1_LIGHT_1_FAN                                    6
    #define DEVICE_IR_BLASTER                                       7

    #define DEVICE_WIRELESS_SCENE_SWITCH                            8
  
    #define DEVICE_1_LIGHT_1_FAN_CUSTOM                             9  //customized clusters for Tuya
    #define DEVICE_IR_BLASTER_CUSTOM                                10 //customized clusters for Tuya

    #define DEVICE_CCT_DALI_CUSTOM                                  11 //customized clusters for Tuya
    #define DEVICE_RGB_DALI                                         12 //SW1=DALI RED, SW2=DALI GREEN, SW3=DALI BLUE, SW4=DALI WHITE
    #define DEVICE_RGB_DMX                                          13 
    #define DEVICE_GROUP_DALI                                       14 //SW1=GROUP1 DALI LIGHTS, SW2=GROUP2 DALI LIGHTS, SW3=GROUP3 DALI LIGHTS, SW4=GROUP4 DALI LIGHTS

    #define DEVICE_RINGING_BELL_2     					            15 //
    #define DEVICE_SENSOR_MOTION                                    16
    #define DEVICE_SENSOR_CONTACT_SWITCH                            17
    #define DEVICE_SENSOR_GAS_LEAK                                  18
    #define DEVICE_SENSOR_TEMPERATURE_HUMIDITY                      19
    #define DEVICE_SENSOR_LUX                                       20 //TSL2561

    #define DEVICE_WIRELESS_REMOTE_SWITCH                           21 //SW1=DEVICES_BIND, SW2=DEVICES_BIND, SW3=DEVICES_BIND, SW4=DEVICES_BIND
    #define DEVICE_1CH_CURTAIN_SWITCH 					            22

    #define DEVICE_WIRELESS_GROUP_SWITCH                            23 //Currently Running Remote Switch

    //TS004F (instead of TS0044) buttons has only single tap
    #define USE_NUOS_ZB_DEVICE_TYPE                                 DEVICE_SENSOR_MOTION //DEVICE_WIRELESS_GROUP_SWITCH //DEVICE_CCT_DALI_CUSTOM //DEVICE_GROUP_DALI

    // #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_ANALOG_DIMMABLE_LIGHT)

    // #else
    //     //#define USE_QUEUE_CTRL
    //     //#define USE_SCENE_RECALL_QUEUE
    //     //#define USE_SCENE_STORE_QUEUE
    // #endif
    //#define USE_NVS_INIT
    //#define DONT_USE_ZIGBEE                                       // For expo or demo only
    #define NEW_SDK_6
    #define USE_OTA

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

            #define COMMUNICATION_MODE                                  COMM_MODE_BROADCAST
        #endif
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)    
        #define COMM_MODE_BROADCAST                                 1
        #define COMM_MODE_GROUP_CTRL                                2
        #define COMM_MODE_ADDR_CTRL                                 3

        #define COMMUNICATION_MODE                                  COMM_MODE_BROADCAST   
    #endif
#endif