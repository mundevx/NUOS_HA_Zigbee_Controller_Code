

// #ifndef _APP_CONFIG_H_
//     #define _APP_CONFIG_H_

    #include "app_config.h"

    #if(CHIP_INFO == USE_ESP32H2)
        #define LOAD_1_PIN                                                  GPIO_NUM_1 
        #define LOAD_2_PIN                                                  GPIO_NUM_0
        #define LOAD_3_PIN                                                  GPIO_NUM_11
        #define LOAD_4_PIN                                                  GPIO_NUM_12
        #define LOAD_5_PIN                                                  GPIO_NUM_3
        #define LOAD_6_PIN                                                  GPIO_NUM_2

        #define TOUCH_1_PIN                                                 GPIO_NUM_25 
        #define TOUCH_2_PIN                                                 GPIO_NUM_22
        #define TOUCH_3_PIN                                                 GPIO_NUM_14
        #define TOUCH_4_PIN                                                 GPIO_NUM_13
        #define TOUCH_5_PIN                                                 GPIO_NUM_5
        #define TOUCH_6_PIN                                                 GPIO_NUM_4

        #define ZCD_PIN                                                     GPIO_NUM_10

        #define DALI_TX_PIN                                                 GPIO_NUM_3
        #define DALI_RX_PIN                                                 LOAD_6_PIN   



        #define HW_TOUCH_LED_PIN_1                                          LOAD_1_PIN
        #define HW_TOUCH_LED_PIN_2                                          LOAD_2_PIN
        #define HW_TOUCH_LED_PIN_3                                          LOAD_3_PIN
        #define HW_TOUCH_LED_PIN_4                                          LOAD_4_PIN

        #define HW_LOAD_PIN_1                                               LOAD_1_PIN
        #define HW_LOAD_PIN_2                                               LOAD_5_PIN
        #define HW_LOAD_PIN_3                                               LOAD_6_PIN
        #define HW_LOAD_PIN_4                                               ZCD_PIN

        #define HW_TOUCH_BTN_PIN_1                                          TOUCH_1_PIN
        #define HW_TOUCH_BTN_PIN_2                                          TOUCH_2_PIN
        #define HW_TOUCH_BTN_PIN_3                                          TOUCH_3_PIN
        #define HW_TOUCH_BTN_PIN_4                                          TOUCH_4_PIN

    #elif(CHIP_INFO == USE_ESP32C6) 
        #define LOAD_1_PIN                                                  GPIO_NUM_1 
        #define LOAD_2_PIN                                                  GPIO_NUM_0
        #define LOAD_3_PIN                                                  GPIO_NUM_5
        #define LOAD_4_PIN                                                  GPIO_NUM_4
        #define LOAD_5_PIN                                                  GPIO_NUM_3
        #define LOAD_6_PIN                                                  GPIO_NUM_2

        #define TOUCH_1_PIN                                                 GPIO_NUM_18 
        #define TOUCH_2_PIN                                                 GPIO_NUM_19
        #define TOUCH_3_PIN                                                 GPIO_NUM_20
        #define TOUCH_4_PIN                                                 GPIO_NUM_21
        #define TOUCH_5_PIN                                                 GPIO_NUM_6
        #define TOUCH_6_PIN                                                 GPIO_NUM_7

        #define ZCD_PIN                                                     GPIO_NUM_15

        #define DALI_TX_PIN                                                 GPIO_NUM_3
        #define DALI_RX_PIN                                                 GPIO_NUM_11

        
        #define HW_TOUCH_LED_PIN_1                                          LOAD_1_PIN
        #define HW_TOUCH_LED_PIN_2                                          LOAD_2_PIN
        #define HW_TOUCH_LED_PIN_3                                          LOAD_3_PIN
        #define HW_TOUCH_LED_PIN_4                                          LOAD_4_PIN

        #define HW_LOAD_PIN_1                                               LOAD_1_PIN
        #define HW_LOAD_PIN_2                                               LOAD_5_PIN
        #define HW_LOAD_PIN_3                                               LOAD_6_PIN
        #define HW_LOAD_PIN_4                                               ZCD_PIN

        #define HW_TOUCH_BTN_PIN_1                                          TOUCH_1_PIN
        #define HW_TOUCH_BTN_PIN_2                                          TOUCH_2_PIN
        #define HW_TOUCH_BTN_PIN_3                                          TOUCH_3_PIN
        #define HW_TOUCH_BTN_PIN_4                                          TOUCH_4_PIN



    #elif(CHIP_INFO == USE_ESP32H2_MINI1) 

        #define LOAD_1_PIN                                                  GPIO_NUM_25 
        #define LOAD_2_PIN                                                  GPIO_NUM_22
        #define LOAD_3_PIN                                                  GPIO_NUM_11
        #define LOAD_4_PIN                                                  GPIO_NUM_12
        #define LOAD_5_PIN                                                  GPIO_NUM_3
        #define LOAD_6_PIN                                                  GPIO_NUM_2

        #define TOUCH_1_PIN                                                 GPIO_NUM_0 
        #define TOUCH_2_PIN                                                 GPIO_NUM_1
        #define TOUCH_3_PIN                                                 GPIO_NUM_14
        #define TOUCH_4_PIN                                                 GPIO_NUM_13
        #define TOUCH_5_PIN                                                 GPIO_NUM_5
        #define TOUCH_6_PIN                                                 GPIO_NUM_4

        #define ZCD_PIN                                                     GPIO_NUM_10

        #define DALI_TX_PIN                                                 GPIO_NUM_3
        #define DALI_RX_PIN                                                 LOAD_6_PIN   

        #define DMX_TX_PIN                                                  LOAD_6_PIN
        
        #define HW_TOUCH_LED_PIN_1                                          LOAD_1_PIN
        #define HW_TOUCH_LED_PIN_2                                          LOAD_2_PIN
        #define HW_TOUCH_LED_PIN_3                                          LOAD_3_PIN
        #define HW_TOUCH_LED_PIN_4                                          LOAD_4_PIN

        #define HW_LOAD_PIN_1                                               LOAD_1_PIN
        #define HW_LOAD_PIN_2                                               LOAD_5_PIN
        #define HW_LOAD_PIN_3                                               LOAD_6_PIN
        #define HW_LOAD_PIN_4                                               ZCD_PIN

        #define HW_TOUCH_BTN_PIN_1                                          TOUCH_1_PIN
        #define HW_TOUCH_BTN_PIN_2                                          TOUCH_2_PIN
        #define HW_TOUCH_BTN_PIN_3                                          TOUCH_3_PIN
        #define HW_TOUCH_BTN_PIN_4                                          TOUCH_4_PIN




    #elif(CHIP_INFO == USE_ESP32C6_MINI1) 
        #define LOAD_1_PIN                                                  GPIO_NUM_19 
        #define LOAD_2_PIN                                                  GPIO_NUM_18
        #define LOAD_3_PIN                                                  -1
        #define LOAD_4_PIN                                                  GPIO_NUM_7
        #define LOAD_5_PIN                                                  GPIO_NUM_3
        #define LOAD_6_PIN                                                  GPIO_NUM_2

        #define TOUCH_1_PIN                                                 GPIO_NUM_4 
        #define TOUCH_2_PIN                                                 GPIO_NUM_5
        #define TOUCH_3_PIN                                                 GPIO_NUM_1
        #define TOUCH_4_PIN                                                 GPIO_NUM_0
        #define TOUCH_5_PIN                                                 GPIO_NUM_14
        #define TOUCH_6_PIN                                                 GPIO_NUM_13

        #define ZCD_PIN                                                     GPIO_NUM_15


        #define DALI_TX_PIN                                                 GPIO_NUM_3
        #define DALI_RX_PIN                                                 LOAD_6_PIN   

        #define DMX_TX_PIN                                                  LOAD_6_PIN

        #define HW_TOUCH_LED_PIN_1                                          LOAD_1_PIN
        #define HW_TOUCH_LED_PIN_2                                          LOAD_2_PIN
        #define HW_TOUCH_LED_PIN_3                                          LOAD_3_PIN
        #define HW_TOUCH_LED_PIN_4                                          LOAD_4_PIN

        #define HW_LOAD_PIN_1                                               LOAD_1_PIN
        #define HW_LOAD_PIN_2                                               LOAD_5_PIN
        #define HW_LOAD_PIN_3                                               LOAD_6_PIN
        #define HW_LOAD_PIN_4                                               ZCD_PIN

        #define HW_TOUCH_BTN_PIN_1                                          TOUCH_1_PIN
        #define HW_TOUCH_BTN_PIN_2                                          TOUCH_2_PIN
        #define HW_TOUCH_BTN_PIN_3                                          TOUCH_3_PIN
        #define HW_TOUCH_BTN_PIN_4                                          TOUCH_4_PIN



    #elif(CHIP_INFO == USE_ESP32H2_MINI1_V2) //8 july month, wthout rgb diode circuit

        #define LOAD_1_PIN                                                  GPIO_NUM_25 
        #define LOAD_2_PIN                                                  GPIO_NUM_22
        #define LOAD_3_PIN                                                  GPIO_NUM_26
        #define LOAD_4_PIN                                                  GPIO_NUM_12
        #ifdef ENABLE_PWM_DIMMING
            #define LOAD_5_PIN                                              GPIO_NUM_18
        #else    
            #define LOAD_5_PIN                                              GPIO_NUM_3
        #endif
        #define LOAD_6_PIN                                                  GPIO_NUM_2
        

        #define TOUCH_1_PIN                                                 GPIO_NUM_0 
        #define TOUCH_2_PIN                                                 GPIO_NUM_1
        #define TOUCH_3_PIN                                                 GPIO_NUM_14
        #define TOUCH_4_PIN                                                 GPIO_NUM_13
        #define TOUCH_5_PIN                                                 GPIO_NUM_5
        #define TOUCH_6_PIN                                                 GPIO_NUM_4
        
        #define TOUCH_IR_BLASTER_PIN                                        GPIO_NUM_22

        #define ZCD_PIN                                                     GPIO_NUM_10 
        #define LED1                                                        GPIO_NUM_27

        #define TX0                                                         GPIO_NUM_17
        #define RX0                                                         GPIO_NUM_16 

        #define DALI_TX_PIN                                                 GPIO_NUM_3
        #define DALI_RX_PIN                                                 LOAD_6_PIN   

        #define DMX_TX_PIN                                                  LOAD_6_PIN

        #define HW_TOUCH_LED_PIN_1                                          LED1
        #define HW_TOUCH_LED_PIN_2                                          LOAD_2_PIN
        #define HW_TOUCH_LED_PIN_3                                          LOAD_3_PIN
        #define HW_TOUCH_LED_PIN_4                                          LOAD_4_PIN

        #define HW_LOAD_PIN_1                                               LOAD_1_PIN
        #define HW_LOAD_PIN_2                                               LOAD_5_PIN
        #define HW_LOAD_PIN_3                                               LOAD_6_PIN
        #define HW_LOAD_PIN_4                                               ZCD_PIN

        #define HW_TOUCH_BTN_PIN_1                                          TOUCH_1_PIN
        #define HW_TOUCH_BTN_PIN_2                                          TOUCH_2_PIN
        #define HW_TOUCH_BTN_PIN_3                                          TOUCH_3_PIN
        #define HW_TOUCH_BTN_PIN_4                                          TOUCH_4_PIN




        // const uint32_t gpio_touch_led_pins[MAX_PINS]                        = { LED1, LOAD_2_PIN, LOAD_3_PIN, LOAD_4_PIN};
        // const uint32_t gpio_load_pins[MAX_PINS]                             = { LOAD_1_PIN, LOAD_5_PIN, LOAD_6_PIN, ZCD_PIN};
        // const uint32_t gpio_touch_btn_pins[MAX_PINS]                        = { TOUCH_1_PIN, TOUCH_2_PIN, TOUCH_3_PIN, TOUCH_4_PIN};

    #elif(CHIP_INFO == USE_ESP32C6_MINI1_V2)  //8 july month, wthout rgb diode circuit

        #define LOAD_1_PIN                                                  GPIO_NUM_19 
        #define LOAD_2_PIN                                                  GPIO_NUM_18
        #define LOAD_3_PIN                                                  GPIO_NUM_20
        #define LOAD_4_PIN                                                  GPIO_NUM_7
        #ifdef ENABLE_PWM_DIMMING
            #define LOAD_5_PIN                                              GPIO_NUM_18
        #else    
            #define LOAD_5_PIN                                              GPIO_NUM_3
        #endif
        #define LOAD_6_PIN                                                  GPIO_NUM_2

        #define TOUCH_1_PIN                                                 GPIO_NUM_4 
        #define TOUCH_2_PIN                                                 GPIO_NUM_5
        #define TOUCH_3_PIN                                                 GPIO_NUM_1
        #define TOUCH_4_PIN                                                 GPIO_NUM_0
        #define TOUCH_5_PIN                                                 GPIO_NUM_14
        #define TOUCH_6_PIN                                                 GPIO_NUM_13
        #define TOUCH_IR_BLASTER_PIN                                        LOAD_4_PIN

        #define ZCD_PIN                                                     GPIO_NUM_15

        #define LED1                                                        GPIO_NUM_21

        #define TX0                                                         GPIO_NUM_17
        #define RX0                                                         GPIO_NUM_16 


        #define DALI_TX_PIN                                                 GPIO_NUM_3
        #define DALI_RX_PIN                                                 LOAD_6_PIN   

        #define HW_TOUCH_LED_PIN_1                                          LED1
        #define HW_TOUCH_LED_PIN_2                                          LOAD_2_PIN
        #define HW_TOUCH_LED_PIN_3                                          LOAD_3_PIN
        #define HW_TOUCH_LED_PIN_4                                          LOAD_4_PIN

        #define HW_LOAD_PIN_1                                               LOAD_1_PIN
        #define HW_LOAD_PIN_2                                               LOAD_5_PIN
        #define HW_LOAD_PIN_3                                               LOAD_6_PIN
        #define HW_LOAD_PIN_4                                               ZCD_PIN

        #define HW_TOUCH_BTN_PIN_1                                          TOUCH_1_PIN
        #define HW_TOUCH_BTN_PIN_2                                          TOUCH_2_PIN
        #define HW_TOUCH_BTN_PIN_3                                          TOUCH_3_PIN
        #define HW_TOUCH_BTN_PIN_4                                          TOUCH_4_PIN

        #define DALI_TX_PIN                                                 GPIO_NUM_3
        #define DALI_RX_PIN                                                 GPIO_NUM_2

        #define DMX_TX_PIN                                                  LOAD_6_PIN

    #elif(CHIP_INFO == USE_ESP32H2_MINI1_V3) //8 july month, with rgb diode circuit & GPIO9 as O/P is used

        #define LOAD_1_PIN                                                  GPIO_NUM_25 
        #define LOAD_2_PIN                                                  GPIO_NUM_9
        #define LOAD_3_PIN                                                  GPIO_NUM_26
        #define LOAD_4_PIN                                                  GPIO_NUM_12
        #ifdef ENABLE_PWM_DIMMING
            #define LOAD_5_PIN                                              GPIO_NUM_22
        #else    
            #define LOAD_5_PIN                                              GPIO_NUM_3
        #endif
        #define LOAD_6_PIN                                                  GPIO_NUM_2

        #define TOUCH_1_PIN                                                 GPIO_NUM_0 
        #define TOUCH_2_PIN                                                 GPIO_NUM_1
        #define TOUCH_3_PIN                                                 GPIO_NUM_14
        #define TOUCH_4_PIN                                                 GPIO_NUM_13
        #define TOUCH_5_PIN                                                 GPIO_NUM_5
        #define TOUCH_6_PIN                                                 GPIO_NUM_4
        
        #define ZCD_PIN                                                     GPIO_NUM_10 
        #define LED1                                                        GPIO_NUM_27

        #define TX0                                                         GPIO_NUM_17
        #define RX0                                                         GPIO_NUM_16 


        #define TOUCH_IR_BLASTER_PIN                                        GPIO_NUM_22
        #define DALI_TX_PIN                                                 GPIO_NUM_3
        #define DALI_RX_PIN                                                 LOAD_6_PIN   

        #define DMX_TX_PIN                                                  LOAD_6_PIN 
        //////////////////////////////////////////////////////////////////////////////////

        #define HW_TOUCH_LED_PIN_1                                          LED1
        #define HW_TOUCH_LED_PIN_2                                          LOAD_2_PIN
        #define HW_TOUCH_LED_PIN_3                                          LOAD_3_PIN
        #define HW_TOUCH_LED_PIN_4                                          LOAD_4_PIN

        #define HW_LOAD_PIN_1                                               LOAD_1_PIN
        #define HW_LOAD_PIN_2                                               LOAD_5_PIN
        #define HW_LOAD_PIN_3                                               LOAD_6_PIN
        #define HW_LOAD_PIN_4                                               ZCD_PIN

        #define HW_TOUCH_BTN_PIN_1                                          TOUCH_1_PIN
        #define HW_TOUCH_BTN_PIN_2                                          TOUCH_2_PIN
        #define HW_TOUCH_BTN_PIN_3                                          TOUCH_3_PIN
        #define HW_TOUCH_BTN_PIN_4                                          TOUCH_4_PIN

    #elif(CHIP_INFO == USE_ESP32C6_MINI1_V3)  //8 july month, with rgb diode circuit & GPIO9 as O/P is used
        #define LOAD_1_PIN                                                  GPIO_NUM_19
        #define LOAD_2_PIN                                                  GPIO_NUM_9
        #define LOAD_3_PIN                                                  GPIO_NUM_20
        #define LOAD_4_PIN                                                  GPIO_NUM_7
        #ifdef ENABLE_PWM_DIMMING
            #define LOAD_5_PIN                                              GPIO_NUM_18
        #else    
            #define LOAD_5_PIN                                              GPIO_NUM_3
        #endif
        #define LOAD_6_PIN                                                  GPIO_NUM_2

        #define TOUCH_1_PIN                                                 GPIO_NUM_4 
        #define TOUCH_2_PIN                                                 GPIO_NUM_5
        #define TOUCH_3_PIN                                                 GPIO_NUM_1
        #define TOUCH_4_PIN                                                 GPIO_NUM_0
        #define TOUCH_5_PIN                                                 GPIO_NUM_14
        #define TOUCH_6_PIN                                                 GPIO_NUM_13

        #define ZCD_PIN                                                     GPIO_NUM_15

        #define LED1                                                        GPIO_NUM_21

        #define DALI_TX_PIN                                                 GPIO_NUM_3
        #define DALI_RX_PIN                                                 LOAD_6_PIN   

        
        #define DMX_TX_PIN                                                  LOAD_6_PIN
        //////////////////////////////////////////////////////////////////////////////////

        #define HW_TOUCH_LED_PIN_1                                          LED1
        #define HW_TOUCH_LED_PIN_2                                          LOAD_2_PIN
        #define HW_TOUCH_LED_PIN_3                                          LOAD_3_PIN
        #define HW_TOUCH_LED_PIN_4                                          LOAD_4_PIN

        #define HW_LOAD_PIN_1                                               LOAD_1_PIN
        #define HW_LOAD_PIN_2                                               LOAD_5_PIN
        #define HW_LOAD_PIN_3                                               LOAD_6_PIN
        #define HW_LOAD_PIN_4                                               ZCD_PIN

        #define HW_TOUCH_BTN_PIN_1                                          TOUCH_1_PIN
        #define HW_TOUCH_BTN_PIN_2                                          TOUCH_2_PIN
        #define HW_TOUCH_BTN_PIN_3                                          TOUCH_3_PIN
        #define HW_TOUCH_BTN_PIN_4                                          TOUCH_4_PIN 
        
    #elif(CHIP_INFO == USE_ESP32H2_MINI1_V4) //21 sep month, GPIO9 as O/P is used & Load1 & Load6 are interchanged

        #define LOAD_1_PIN                                                  GPIO_NUM_2 //GPIO_NUM_25 
        #define LOAD_2_PIN                                                  GPIO_NUM_9 //GPIO_NUM_22
        #define LOAD_3_PIN                                                  GPIO_NUM_26
        #define LOAD_4_PIN                                                  GPIO_NUM_12
        #ifdef ENABLE_PWM_DIMMING
            #define LOAD_5_PIN                                              GPIO_NUM_22
        #else    
            #define LOAD_5_PIN                                              GPIO_NUM_3
        #endif
        #define LOAD_6_PIN                                                  GPIO_NUM_25 //GPIO_NUM_2

        #define TOUCH_1_PIN                                                 GPIO_NUM_0 
        #define TOUCH_2_PIN                                                 GPIO_NUM_1
        #define TOUCH_3_PIN                                                 GPIO_NUM_14
        #define TOUCH_4_PIN                                                 GPIO_NUM_13
        #define TOUCH_5_PIN                                                 GPIO_NUM_5
        #define TOUCH_6_PIN                                                 GPIO_NUM_4
        
        #define TOUCH_IR_BLASTER_PIN                                        GPIO_NUM_22

        #define ZCD_PIN                                                     GPIO_NUM_10 

        #define LED1                                                        GPIO_NUM_27
        #define LED2                                                        GPIO_NUM_9

        #define TX0                                                         GPIO_NUM_17
        #define RX0                                                         GPIO_NUM_16 

        #define DALI_TX_PIN                                                 GPIO_NUM_3
        #define DALI_RX_PIN                                                 LOAD_6_PIN   

        #define DMX_TX_PIN                                                  LOAD_6_PIN
        //////////////////////////////////////////////////////////////////////////////////

        #define HW_TOUCH_LED_PIN_1                                          LED1
        #define HW_TOUCH_LED_PIN_2                                          LOAD_2_PIN
        #define HW_TOUCH_LED_PIN_3                                          LOAD_3_PIN
        #define HW_TOUCH_LED_PIN_4                                          LOAD_4_PIN

        #define HW_LOAD_PIN_1                                               LOAD_1_PIN
        #define HW_LOAD_PIN_2                                               LOAD_5_PIN
        #define HW_LOAD_PIN_3                                               LOAD_6_PIN
        #define HW_LOAD_PIN_4                                               ZCD_PIN

        #define HW_TOUCH_BTN_PIN_1                                          TOUCH_1_PIN
        #define HW_TOUCH_BTN_PIN_2                                          TOUCH_2_PIN
        #define HW_TOUCH_BTN_PIN_3                                          TOUCH_3_PIN
        #define HW_TOUCH_BTN_PIN_4                                          TOUCH_4_PIN   
        
    #elif(CHIP_INFO == USE_ESP32C6_MINI1_V4)  //21 sep month, GPIO9 as O/P is used & Load1 & Load6 are interchanged
        #define LOAD_1_PIN                                                  GPIO_NUM_2 //GPIO_NUM_19
        #define LOAD_2_PIN                                                  GPIO_NUM_9
        #define LOAD_3_PIN                                                  GPIO_NUM_20
        #define LOAD_4_PIN                                                  GPIO_NUM_7
        #ifdef ENABLE_PWM_DIMMING
            #define LOAD_5_PIN                                              GPIO_NUM_18
        #else    
            #define LOAD_5_PIN                                              GPIO_NUM_3
        #endif
        #define LOAD_6_PIN                                                  GPIO_NUM_19 //GPIO_NUM_2

        #define TOUCH_1_PIN                                                 GPIO_NUM_4 
        #define TOUCH_2_PIN                                                 GPIO_NUM_5
        #define TOUCH_3_PIN                                                 GPIO_NUM_1
        #define TOUCH_4_PIN                                                 GPIO_NUM_0
        #define TOUCH_5_PIN                                                 GPIO_NUM_14
        #define TOUCH_6_PIN                                                 GPIO_NUM_13

        #define ZCD_PIN                                                     GPIO_NUM_15

        #define LED1                                                        GPIO_NUM_21

        #define DALI_TX_PIN                                                 GPIO_NUM_3
        #define DALI_RX_PIN                                                 LOAD_6_PIN   

        #define DMX_TX_PIN                                                  LOAD_6_PIN
        //////////////////////////////////////////////////////////////////////////////////

        #define HW_TOUCH_LED_PIN_1                                          LED1
        #define HW_TOUCH_LED_PIN_2                                          LOAD_2_PIN
        #define HW_TOUCH_LED_PIN_3                                          LOAD_3_PIN
        #define HW_TOUCH_LED_PIN_4                                          LOAD_4_PIN

        #define HW_LOAD_PIN_1                                               LOAD_1_PIN
        #define HW_LOAD_PIN_2                                               LOAD_5_PIN
        #define HW_LOAD_PIN_3                                               LOAD_6_PIN
        #define HW_LOAD_PIN_4                                               ZCD_PIN

        #define HW_TOUCH_BTN_PIN_1                                          TOUCH_1_PIN
        #define HW_TOUCH_BTN_PIN_2                                          TOUCH_2_PIN
        #define HW_TOUCH_BTN_PIN_3                                          TOUCH_3_PIN
        #define HW_TOUCH_BTN_PIN_4                                          TOUCH_4_PIN 

    #elif(CHIP_INFO == USE_ESP32H2_MINI1_V5) //15 oct month, GPIO9 as O/P is replaced by GPIO_NUM_11
        #define LOAD_1_PIN                                                  GPIO_NUM_2
        #define LOAD_2_PIN                                                  GPIO_NUM_11  //open R13, open R14 route to PIN20 & PIN17
        #define LOAD_3_PIN                                                  GPIO_NUM_26
        #define LOAD_4_PIN                                                  GPIO_NUM_12
        #ifdef ENABLE_PWM_DIMMING
            #if(USE_NUOS_ZB_DEVICE_TYPE  == DEVICE_2T_ANALOG_DIMMABLE_LIGHT) 
            #define LOAD_5_PIN                                              GPIO_NUM_22 
            #else
                #ifdef CHANGE_LOAD_PIN
                    #define LOAD_5_PIN                                              GPIO_NUM_22
                #else  
                    #define LOAD_5_PIN                                              GPIO_NUM_3
                #endif  
            #endif
           
        #else  
            #ifdef CHANGE_LOAD_PIN
            #define LOAD_5_PIN                                              GPIO_NUM_22 
            #else  
            #define LOAD_5_PIN                                              GPIO_NUM_3
            #endif
        #endif
        #define LOAD_6_PIN                                                  GPIO_NUM_25 

        #define TOUCH_1_PIN                                                 GPIO_NUM_0
        #define TOUCH_2_PIN                                                 GPIO_NUM_1
        #define TOUCH_3_PIN                                                 GPIO_NUM_14
        #define TOUCH_4_PIN                                                 GPIO_NUM_13
        #define TOUCH_5_PIN                                                 GPIO_NUM_5
        #define TOUCH_6_PIN                                                 GPIO_NUM_4


        #define UART_TX                                                     5
        #define UART_RX                                                     4     

        #define TOUCH_IR_BLASTER_PIN                                        GPIO_NUM_22

        #define ZCD_PIN                                                     GPIO_NUM_10

        #define LED1                                                        GPIO_NUM_27
        #define LED2                                                        GPIO_NUM_11

        #define TX0                                                         GPIO_NUM_17
        #define RX0                                                         GPIO_NUM_16

        #define DALI_TX_PIN                                                 LOAD_5_PIN
        #define DALI_RX_PIN                                                 LOAD_6_PIN  

        #define DMX_TX_PIN                                                  LOAD_6_PIN
        //////////////////////////////////////////////////////////////////////////////////

        #define HW_TOUCH_LED_PIN_1                                          LED1
        #define HW_TOUCH_LED_PIN_2                                          LED2     //or LOAD_2_PIN
        #define HW_TOUCH_LED_PIN_3                                          LOAD_3_PIN
        #define HW_TOUCH_LED_PIN_4                                          LOAD_4_PIN

        #define HW_LOAD_PIN_1                                               LOAD_1_PIN
        #define HW_LOAD_PIN_2                                               LOAD_5_PIN
        #define HW_LOAD_PIN_3                                               LOAD_6_PIN
        #define HW_LOAD_PIN_4                                               ZCD_PIN

        #define HW_TOUCH_BTN_PIN_1                                          TOUCH_1_PIN
        #define HW_TOUCH_BTN_PIN_2                                          TOUCH_2_PIN
        #define HW_TOUCH_BTN_PIN_3                                          TOUCH_3_PIN
        #define HW_TOUCH_BTN_PIN_4                                          TOUCH_4_PIN 
        
    #elif(CHIP_INFO == USE_ESP32C6_MINI1_V5)  ////15 oct month, GPIO9 as O/P is replaced by NC
        #define LOAD_1_PIN                                                  GPIO_NUM_2
        //#define LOAD_2_PIN                                                  GPIO_NUM_15 //short R13, open R14 route to PIN20
        #define LOAD_2_PIN                                                  GPIO_NUM_12 //short R14, open R14 route to PIN17
        #define LOAD_3_PIN                                                  GPIO_NUM_20
        #define LOAD_4_PIN                                                  GPIO_NUM_7
        #ifdef ENABLE_PWM_DIMMING
            #define LOAD_5_PIN                                              GPIO_NUM_18
        #else    
            #ifdef CHANGE_LOAD_PIN
            #define LOAD_5_PIN                                              GPIO_NUM_18 
            #else  
            #define LOAD_5_PIN                                              GPIO_NUM_3
            #endif

        #endif
        #define LOAD_6_PIN                                                  GPIO_NUM_19

        #define TOUCH_1_PIN                                                 GPIO_NUM_4
        #define TOUCH_2_PIN                                                 GPIO_NUM_5
        #define TOUCH_3_PIN                                                 GPIO_NUM_1
        #define TOUCH_4_PIN                                                 GPIO_NUM_0
        #define TOUCH_5_PIN                                                 GPIO_NUM_14
        #define TOUCH_6_PIN                                                 GPIO_NUM_13


        #define UART_TX                                                     14
        #define UART_RX                                                     13


        #define ZCD_PIN                                                     GPIO_NUM_15

        #define LED1                                                        GPIO_NUM_21
        #define LED2                                                        GPIO_NUM_12

        #define DALI_TX_PIN                                                 GPIO_NUM_3
        #define DALI_RX_PIN                                                 LOAD_6_PIN   

        #define DMX_TX_PIN                                                  LOAD_6_PIN

        #define HW_TOUCH_LED_PIN_1                                          LED1
        #define HW_TOUCH_LED_PIN_2                                          LED2
        #define HW_TOUCH_LED_PIN_3                                          LOAD_3_PIN
        #define HW_TOUCH_LED_PIN_4                                          LOAD_4_PIN

        #define HW_LOAD_PIN_1                                               LOAD_1_PIN
        #define HW_LOAD_PIN_2                                               LOAD_5_PIN
        #define HW_LOAD_PIN_3                                               LOAD_6_PIN
        #define HW_LOAD_PIN_4                                               ZCD_PIN

        #define HW_TOUCH_BTN_PIN_1                                          TOUCH_1_PIN
        #define HW_TOUCH_BTN_PIN_2                                          TOUCH_2_PIN
        #define HW_TOUCH_BTN_PIN_3                                          TOUCH_3_PIN
        #define HW_TOUCH_BTN_PIN_4                                          TOUCH_4_PIN                                         
    #endif

// #endif

/**  ESP32H2 MINI1 V2
 * 
 *   L1     IO25 (pin 25)
 *   L2     IO22 (pin 24) 
 *   L3     IO26 (pin 26) 
 *   L4     IO12 (pin 16) 
 *   L5     IO03 (pin 06) 
 *   L6     IO02 (pin 05) 
 * 
 *   T1     IO00 (pin 09) 
 *   T2     IO01 (pin 10)  
 *   T3     IO14 (pin 13)
 *   T4     IO13 (pin 12) 
 *   T5     IO05 (pin 19)  
 *   T6     IO04 (pin 18)
 
 *   ZCD    IO10 (pin 20)

 *   LED1   IO27 (pin 27)

 *   TX0    IO17 (pin 31)
 *   RX0    IO16 (pin 30)
 **/

/**  ESP32C6 MINI1 V2
 *   L1     IO01 (pin )
 *   L2     IO00 (pin ) 
 *   L3     IO05 (pin ) 
 *   L4     IO12 (pin ) 
 *   L5     IO03 (pin ) 
 *   L6     IO02 (pin ) 
 * 
 *   T1     IO00 (pin 09) 
 *   T2     IO01 (pin 10)  
 *   T3     IO14 (pin 13)
 *   T4     IO13 (pin 12) 
 *   T5     IO05 (pin 19)  
 *   T6     IO04 (pin 18)
 
 *   ZCD    IO15 (pin 20)

 *   LED1   IO27 (pin 27)

 *   TX0    IO17 (pin 31)
 *   RX0    IO16 (pin 30)
 **/
