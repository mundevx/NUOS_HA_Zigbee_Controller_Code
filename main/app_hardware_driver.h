    
    #include "app_constants.h"
    #include "pin_config.h"
    #include "driver/gpio.h"
    #include "driver/uart.h"
    #include "esp_zigbee_type.h"
    #include "switch_driver.h"
    #include "app_nvs_store_info.h"

    #define MAX_NODES                           20
    #define MAX_DST_EP                          4
    #define MAX_DALI_ADDRESSES                  64
    #define MAX_DALI_DEVICES_IN_SCENES          30
    #define MAX_TOUCH_BTNS                      4
    #define MAX_DALI_SCENES                     16

    typedef struct {
        uint16_t h; // Hue [0, 0x0168] (0 to 360 degrees)
        uint16_t s; // Saturation [0, 0x3E8] (0 to 1000)
        uint16_t v; // Value [0, 0x3E8] (0 to 1000)
    } hsv_t;

    typedef struct {
        uint8_t r; // Red [0, 255]
        uint8_t g; // Green [0, 255]
        uint8_t b; // Blue [0, 255]
    } rgb_t;
    typedef struct {
        uint8_t bytes[7]; // 56 bits = 7 bytes
    } uint56_t;

    typedef struct {
        uint8_t bytes[10]; // 80 bits = 10 bytes
    } uint80_t;

    typedef struct {
        uint8_t bytes[17]; // 136 bits = 17 bytes
    } uint136_t;

    typedef struct {
        uint8_t bytes[4]; // 32 bits = 4 bytes
    } uint32_ts;

    //used to store info during binding
	typedef struct zigbee_info_s
	{
	  esp_zb_ieee_addr_t ieee_addr;
	  uint16_t short_addr;
	  uint8_t endpoint;
	} zigbee_info_t;

    typedef struct light_control_device_ctx_t
	{
		bool device_state;
       
		uint8_t device_level;

        uint16_t device_val;

        bool color_or_fan_state;
        bool dim_up;
        bool level_up;
        uint16_t light_color_x;
        uint16_t light_color_y;
        uint8_t ac_mode;
        //ac IR

        int8_t ac_decode_type;
        uint8_t ac_temperature;
        //fan
        uint8_t fan_speed;
    } led_indicator_handle_t;

    typedef struct device_overall_params_ctx_t
	{ 
        bool state[5];
        uint8_t level[5];
        uint8_t mode;
    } device_overall_params_t;

    typedef struct attr_data_info_s {
        bool state;
        uint8_t level;
        uint32_t value; //color
        uint8_t mode; 
    }attr_data_info_t;

    typedef struct dst_ep_data_s {
        uint8_t clusters_count; 
        uint16_t cluster_id[5];        
        uint8_t dst_ep;
        uint8_t is_bind;
        uint8_t is_reporting;
        uint8_t get_cluster;
        char ep_name[40]; 
        attr_data_info_t data;
    }dst_ep_data_t;

    typedef struct dst_endpoint_info_s {
        dst_ep_data_t ep_data[MAX_DST_EP];
    }dst_endpoint_info_t;

    typedef struct dst_node_info_s {
        uint16_t short_addr;
        esp_zb_ieee_addr_t ieee_addr;
        uint16_t device_role;
        char node_name[40];
        uint8_t endpoint_counts;
        dst_endpoint_info_t dst_ep_info; //max 4 endpoints
    }dst_node_info_t;

    typedef struct stt_scene_ep_s {
        uint8_t src_ep;
        uint16_t total_records;
        dst_node_info_t dst_node_info[MAX_NODES];
    }stt_scene_ep_s;

    typedef struct stt_scene_switch_s {
        uint8_t scene_id;
        uint8_t short_addr;
        stt_scene_ep_s scene_switch_info; //4 buttons
    }stt_scene_switch_t;

    typedef struct {
        uint16_t short_;
        uint8_t dst;
        uint8_t src;
        uint8_t bind;
        uint8_t state;
        uint8_t level;
        uint16_t value;
        uint8_t check;
        char name[40];
        char g_name[40];        
    } MyItem;

    typedef struct {
        uint16_t group_id;
        uint8_t scene_id;
        uint8_t dst_ep;
        uint8_t is_on;
        uint8_t intensity;
    } zigbee_zcene_info_t;

    typedef struct {
        uint8_t index;
        uint16_t time_value;
        uint8_t command;
    }identify_response_commands_t;

    typedef struct {
        uint8_t group_id;
        uint8_t scene_id[6];
        uint8_t total_ids;
        uint8_t selected_scene_id;
        int16_t device_ids[32];
        uint8_t state;
        uint8_t brightness;
        uint32_t color_value;
    }dali_device_ids_t;

    typedef struct {
        uint8_t source_ep;
        uint8_t dst_ep;
        uint16_t short_addr;
    } zb_binding_data_t;

    typedef struct {
        uint8_t selected_id;
        uint8_t group_id[MAX_TOUCH_BTNS];
        uint8_t scene_ids[MAX_TOUCH_BTNS];
        uint8_t control_type;
        uint8_t scn_ctrl_type;
        uint8_t total_ids[MAX_TOUCH_BTNS];
        uint16_t device_ids[MAX_TOUCH_BTNS][MAX_DALI_DEVICES_IN_SCENES];  //==>20, you can increase number of dali_ids upto 64
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_DALI_DIRECT_SWITCH)
        bool device_state[2][MAX_DALI_SCENES][MAX_DALI_DEVICES_IN_SCENES];
        uint8_t device_level[2][MAX_DALI_SCENES][MAX_DALI_DEVICES_IN_SCENES];
        uint16_t device_color[2][MAX_DALI_SCENES][MAX_DALI_DEVICES_IN_SCENES];
        #else
        bool device_state[MAX_DALI_SCENES][MAX_DALI_DEVICES_IN_SCENES];
        uint8_t device_level[MAX_DALI_SCENES][MAX_DALI_DEVICES_IN_SCENES];
        uint16_t device_color[MAX_DALI_SCENES][MAX_DALI_DEVICES_IN_SCENES];
        #endif
        uint8_t device_scene[MAX_DALI_SCENES][MAX_DALI_DEVICES_IN_SCENES];
        #ifdef ENABLE_DALI_RECEIVER
            uint8_t device_color_mode[MAX_DALI_SCENES];
        #endif
    } scene_switch_s;

   typedef struct{
        int total;
        uint8_t addresses[64];
    } DaliNodeList; 

    #define SINGLE_PRESS_MAX_INTERVAL                                           20
    #define DOUBLE_PRESS_MAX_INTERVAL                                           1000
    #define LONG_PRESS_MAX_INTERVAL                                             1100

    #define SINGLE_PRESS                                                        0
    #define DOUBLE_PRESS                                                        1
    #define LONG_PRESS                                                          2
    #define LONG_PRESS_INC_DEC_LEVEL                                            3


    #define TASK_CURTAIN_CAL_INIT    0
    #define TASK_CURTAIN_CAL_START   1
    #define TASK_CURTAIN_CAL_END     2

// Default values for curtain motor
    #define DEFAULT_OFFSET_TIME                                                 2000
    #define DEFAULT_CALIBRATION_TIME                                            30000
//  https://yatyyt4jowdyyswqkog4i42helmwilyu.ui.nabu.casa/lovelace/0
////////////////////////////////////////////////////////////////////////
    #define MAX_BINDINGS                                                        40  // adjust this based on how many you expect
    #ifdef DECLARE_MAIN
        uint8_t ep_selected_index                                                  = 0;
        bool isr_service_installed                                              = false;
        bool rgb_led_blink_flag                                                 = false;
        const uint32_t blink_intervals[3]                                       = {200, 500, 1000}; // Fast, Medium, Slow

        SemaphoreHandle_t recordsSemaphore                                      = NULL;
        SemaphoreHandle_t zigbee_sem                                            = NULL;
        SemaphoreHandle_t bindingSemaphore                                      = NULL;
        SemaphoreHandle_t simple_desc_sem                                       = NULL;
        TaskHandle_t query_neighbour_task_handle                                = NULL;
        TaskHandle_t query_callback_handle                                      = NULL; 
        bool blink_leds_start_flag                                              = false;
        uint8_t scene_ep_index_mode                                             = 0;
        uint8_t scene_ep_index                                                  = 0;
        uint8_t ep_id[5]                                                        = {0xff, 0xff, 0xff, 0xff, 0xff};
        uint8_t ep_cnts                                                         = 0;
        uint8_t scene_counts                                                    = 0;
        bool single_endpoint_gateway                                            = false;
        // scene_data_attr_info_t  scene_data_attr_info[TOTAL_ENDPOINTS];
        bool touchLedsOffAfter1MinuteEnable                                     = false;
        uint8_t pressed_value                                                   = 0xff;
        int64_t switch_pressed_start_time                                       = 0;
        uint8_t switch_pressed_counts                                           = 0;
        bool switch_pressed_flag                                                = false;
        bool blink_rgb_led_flag                                                 = false;
        bool blink_rgb_led_normal_functionality_flag                            = false;
        bool blink_rgb_led_commissioning_functionality_flag                     = false;
        uint8_t uchKeypressed                                                   = 0xff;
        bool switch_detected                                                    = true;
        bool found_clusters_flag                                                = false;
        uint8_t dali_on_webpage_commissioning_counts                            = 0;
        MyItem webpageItem[MAX_NODES];
        int node_counts                                                         = 0; 
        
        uint8_t target_percentage                                               = 0;
        bool is_long_press_brightness                                           = false;
        uint8_t selected_color_mode                                             = 0xff;
        uint8_t last_selected_color_mode                                        = 0xff;
        bool mode_change_flag                                                   = false;

        //uint32_t curtain_cal_time                                               = DEFAULT_CALIBRATION_TIME;
        // Global variables for curtain settings
        //uint32_t offset_time                                                    = DEFAULT_OFFSET_TIME;

        uint8_t dali_fade_time                                                  = 0;
        uint8_t dali_fade_rate                                                  = 0;                                            
        uint8_t dali_min_off_offset                                             = 0;
        uint8_t dali_range_size                                                 = 255;

        bool double_press_click_enable[4]                                       = {false, false, false, false};
        uint16_t disable_double_press_enable_counts                             = 0;

        uint16_t cb_requests_counts                                             = 0;
        uint16_t cb_response_counts                                             = 0;
        // #ifdef USE_CCT_TIME_SYNC
        // bool set_cct_time_sync_request_flag                                     = false;
        // #endif
        bool is_reporting[MAX_DST_EP][MAX_NODES];
         
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI)
        const char* switch_ctrl_type[2] = { "Individual Control", "Broadcast Control" }; 
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_DALI_DIRECT_SWITCH)
        const char* switch_ctrl_type[2] = { "Individual Control", "Broadcast Control" };
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI)
            const char* switch_ctrl_type[2] = { "Group Control", "Scene Control"};
        #else
            const char* switch_ctrl_type[3] = { "Individual Control", "Group Control", "Scene Control" };
        #endif
        const char* scene_ctrl_type[3] = { "Broadcast Control", "Group Control", "Individual Control"};     
        //scene_switch_s scene_switch_t[4];
        scene_switch_s scene_group_switch_info;
        uint16_t global_group_id[4]                                             = { 0x1, 0x2, 0x3, 0x4 }; 
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI)
        uint8_t global_dali_id[4]                                              = { 1, 2, 3, 4 }; 
        #else
        uint8_t global_dali_id[]                                                = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
                                                                                    11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
                                                                                    21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 
                                                                                    31, 32};  
        #endif
        uint8_t global_scene_id[4]                                              = { 0x1, 0x2, 0x3, 0x4 };
        dali_device_ids_t dali_nvs_stt[4];

        dst_node_info_t dst_node_info[MAX_BINDINGS];
        int binding_count = 0;

        bool change_cw_ww_color_flag                                            = false;
        // dmx_variable_t dmx_nvs_stt;
        const uint16_t fan_speed_percentage[5]                                  = { 0, 250, 500, 750, 1000};
        const uint16_t fan_speed_values[5]                                      = { 0, 64, 128, 191, 254};


        const uint8_t ac_temp_values[17]                                        = {16, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 30};

        uint8_t global_index                                                    = 0;
        bool toggle_state_flag                                                  = false;
        bool set_hardware_flag                                                  = false;
        bool set_state_flag                                                     = false;
        bool set_level_flag                                                     = false;
        bool set_color_flag                                                     = false;
        bool brightness_control_flag                                            = false;
        stt_scene_switch_t existing_nodes_info[4];
        stt_scene_switch_t nodes_info;
        zigbee_zcene_info_t zb_scene_info[4];
        // wifi_info_handle_t wifi_info;
        //bool timer3_running_flag                                                = true;

        led_indicator_handle_t device_info[TOTAL_BUTTONS+1];

        device_overall_params_t device_info_backup;
        zigbee_info_t light_zb_info[TOTAL_ENDPOINTS+1];
        bool light_driver_deinit_flag                                           = false;
        uint8_t wifi_webserver_active_flag                                      = false;
        uint8_t global_switch_state                                             = 0;
        bool start_commissioning                                                = false;
        bool ready_commisioning_flag                                            = false;
        bool is_my_device_commissionned                                         = false;
        bool is_some_device_unavailable                                         = false;
        uint32_t wifi_webserver_active_counts                                   = 0;
        bool find_active_neighbours_task_flag                                   = false;
        uint8_t zb_network_bdb_steering_count                                   = 0;

        #if (CHIP_INFO == USE_ESP32C6)
            uint32_t I2C_MASTER_SCL_IO 				                            = GPIO_NUM_7;
            uint32_t I2C_MASTER_SDA_IO 				                            = GPIO_NUM_6;              
        #elif(CHIP_INFO == USE_ESP32H2)    
            gpio_num_t I2C_MASTER_SCL_IO 				                        = GPIO_NUM_4;
            gpio_num_t I2C_MASTER_SDA_IO 				                        = GPIO_NUM_5;
        #elif(CHIP_INFO == USE_ESP32C6_MINI1)    
            uint32_t I2C_MASTER_SCL_IO 				                            = GPIO_NUM_13;
            uint32_t I2C_MASTER_SDA_IO 				                            = GPIO_NUM_14; //   
        #elif(CHIP_INFO == USE_ESP32H2_MINI1)    
            gpio_num_t I2C_MASTER_SCL_IO 				                        = GPIO_NUM_4;
            gpio_num_t I2C_MASTER_SDA_IO 				                        = GPIO_NUM_5;  
        #elif(CHIP_INFO == USE_ESP32H2_MINI1_V2)    
            gpio_num_t I2C_MASTER_SCL_IO 				                        = GPIO_NUM_4;
            gpio_num_t I2C_MASTER_SDA_IO 				                        = GPIO_NUM_5;
        #elif(CHIP_INFO == USE_ESP32C6_MINI1_V3)    
            gpio_num_t I2C_MASTER_SCL_IO 				                        = GPIO_NUM_13;
            gpio_num_t I2C_MASTER_SDA_IO 				                        = GPIO_NUM_14;
        #elif(CHIP_INFO == USE_ESP32H2_MINI1_V3)    
            gpio_num_t I2C_MASTER_SCL_IO 				                        = GPIO_NUM_4;
            gpio_num_t I2C_MASTER_SDA_IO 				                        = GPIO_NUM_5;            
        #elif(CHIP_INFO == USE_ESP32H2_MINI1_V4)    
            gpio_num_t I2C_MASTER_SCL_IO 				                        = GPIO_NUM_4;
            gpio_num_t I2C_MASTER_SDA_IO 				                        = GPIO_NUM_5;
        #elif(CHIP_INFO == USE_ESP32C6_MINI1_V4)    
            gpio_num_t I2C_MASTER_SCL_IO 				                        = GPIO_NUM_13;
            gpio_num_t I2C_MASTER_SDA_IO 				                        = GPIO_NUM_14;  
        #elif(CHIP_INFO == USE_ESP32H2_MINI1_V5)    
            gpio_num_t I2C_MASTER_SCL_IO 				                        = GPIO_NUM_4;
            gpio_num_t I2C_MASTER_SDA_IO 				                        = GPIO_NUM_5;
        #elif(CHIP_INFO == USE_ESP32C6_MINI1_V5)    
            gpio_num_t I2C_MASTER_SCL_IO 				                        = GPIO_NUM_13;
            gpio_num_t I2C_MASTER_SDA_IO 				                        = GPIO_NUM_14;                                                                                     
        #endif
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
            // #if(TOTAL_ENDPOINTS == 1)
                const uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS]                   = {1};
                #ifdef USE_HOME_ASSISTANT
                    const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]              = {ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID};
                    char manufname[]                                            = {4, 'N', 'U', 'O', 'S'};
                    char modelid []                                             = {14, 'D', 'A', 'L', 'I', ' ', 'C', 'C', 'T', ' ', 'L', 'i', 'g', 'h', 't'};
                #else
                    const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]              = {0x010C};
                    //Tuya sgzohm9w
                    char manufname[]                                            = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 's', 'g', 'z', 'o', 'h', 'm', '9', 'w'};
                    // TS0505B
                    char modelid[]                                              = {7, 'T', 'S', '0', '5', '0', '4', 'B'};     
                #endif
            // #elif(TOTAL_ENDPOINTS == 2)
            //     const uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS]                   = {1, 2};
            //     const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                  = {0x010C, 0x010C};
            //     //Tuya whtkfqqt  iahbbjaq                                                                   
            //     //char manufname[]                                                = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'w', 'h', 't', 'k', 'f', 'q', 'q', 't'};
            //     // char manufname[]                                                = {16, '_', 'T', 'Z', '3', '2', '1', '0', '_', 'i', 'a', 'h', 'b', 'b', 'j', 'a', 'q'};
            //     // // TS0505B
            //     // char modelid[]                                                  = {7, 'T', 'S', '0', '5', '0', '4', 'B'};   
                
            //     // char manufname[]                                            = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'i', 'a', 'h', 'b', 'b', 'j', 'a', 'q'};
            //     // char modelid[]                                              = {7, 'T', 'S', '0', '5', '0', '2', 'B'};
            //     //whtkfqqt
            //     char manufname[]                                                = {16, '_', 'T', 'Z', 'E', '2', '0', '0', '_', 'w', 'h', 't', 'k', 'f', 'q', 'q', 't'};              
            //     const char modelid[]                                            = {6, 'T', 'S', '0', '6', '0', '1'}; 
            // #endif

            const gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                    = { HW_TOUCH_LED_PIN_1, HW_TOUCH_LED_PIN_3, HW_TOUCH_LED_PIN_2, HW_TOUCH_LED_PIN_4};
            const gpio_num_t gpio_touch_btn_pins[TOTAL_BUTTONS]                 = { HW_TOUCH_BTN_PIN_1, HW_TOUCH_BTN_PIN_3, HW_TOUCH_BTN_PIN_2, HW_TOUCH_BTN_PIN_4};
 
            const gpio_num_t gpio_load_pins[TOTAL_LOADS]                        = { DALI_RX_PIN, DALI_TX_PIN};
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_DALI_DIRECT_SWITCH) 

            #ifdef USE_HOME_ASSISTANT
                const uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS]                   = { 1, 2};                                                                       
                const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                  = {ESP_ZB_HA_COLOR_DIMMABLE_LIGHT_DEVICE_ID, ESP_ZB_HA_COLOR_DIMMABLE_LIGHT_DEVICE_ID};
                
                char manufname[]                                                = {16, '_', 'T', 'Z', '3', '2', '1', '0', '_', 'd', '7', 'd', '9', 'o', 'j', 'a', 'v'};
                char modelid []                                                 = {7, 'T', 'S', '0', '5', '0', '2', 'A'};
 
            #else //if tuya
                #ifdef USE_TUYA_BRDIGE 
                    #ifdef USE_COLOR_CONTROL 
                        const uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS]           = {1, 2};
                        const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]              = {ESP_ZB_HA_COLOR_DIMMABLE_LIGHT_DEVICE_ID, ESP_ZB_HA_COLOR_DIMMABLE_LIGHT_DEVICE_ID};
                        //const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]          = {ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID, ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID}; //0x010C
                        // _TZE204_mm9kyoh3
                        //char manufname[]                                        = {16, '_', 'T', 'Z', 'E', '2', '0', '4', '_', 'm', 'm', '9', 'k', 'y', 'o', 'h', '3'};
                        //_TZE204_zehdwhwb
                        // ookkkk char manufname[]                                        = {16, '_', 'T', 'Z', 'E', '2', '0', '0', '_', 'z', 'e', 'h', 'd', 'w', 'h', 'w', 'b'};
                        //_TZE204_ Tuya whtkfqqt
                        //char manufname[]                                        = {16, '_', 'T', 'Z', 'E', '2', '0', '4', '_', 'w', 'h', 't', 'k', 'f', 'q', 'q', 't'};              
                        
                        // _TZE200_fjjbhx9d  market
                        // char manufname[]                                        = {16, '_', 'T', 'Z', 'E', '2', '0', '0', '_', 'e', '3', 'o', 'i', 't', 'd', 'y', 'u'};
                        //const char modelid[]                                    = {6, 'T', 'S', '0', '6', '0', '1'};
                        
                        // //wc1lppx1 - ok - original
                        // char manufname[]                                        = {16, '_', 'T', 'Z', '3', '2', '1', '0', '_', 'w', 'c', '1', 'l', 'p', 'p', 'x', '1'};
                        // const char modelid[]                                    = {6, 'T', 'S', '1', '1', '0', '1'};  
                        
                        //d7d9ojav - testing
                        char manufname[]                                        = {16, '_', 'T', 'Z', '3', '2', '1', '0', '_', 'd', '7', 'd', '9', 'o', 'j', 'a', 'v'};
                        const char modelid[]                                    = {6, 'T', 'S', '1', '1', '0', '1'}; 

                        // // home assistant
                        // char manufname[]                                            = {16, '_', 'T', 'Z', '3', '0', '0', '0',  '_', 'w', 'c', '1', 'l', 'p', 'p', 'x', '1'};
                        // // TS0505B
                        // char modelid[]                                              = {7, 'T', 'S', '0', '5', '0', '4', 'B'}; 
                        
                        // char manufname[]                                        = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'd', '7', 'd', '9', 'o', 'j', 'a', 'v'};
                        // char modelid []                                         = {7, 'T', 'S', '0', '5', '0', '2', 'A'};
                    #else
                        //lbdxarah
                        char manufname[]                                        = {16, '_', 'T', 'Z', 'E', '2', '8', '4', '_', 'l', 'b', 'd', 'x', 'a', 'r', 'a', 'h'};
                        char modelid[]                                          = {6, 'T', 'S', '0', '6', '0', '1'};
                        const uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS]           = { 1, 2 };                                                                       
                        const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]          = {ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID, ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID};                                               
                    #endif
                #endif
            #endif

            const gpio_num_t gpio_load_pins[TOTAL_LOADS]                        = { DALI_RX_PIN, DALI_TX_PIN};
            #ifdef USE_COLOR_CONTROL 
                const gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                    = { HW_TOUCH_LED_PIN_1, HW_TOUCH_LED_PIN_3, HW_TOUCH_LED_PIN_2, HW_TOUCH_LED_PIN_4}; 
                const gpio_num_t gpio_touch_btn_pins[TOTAL_BUTTONS]                 = { HW_TOUCH_BTN_PIN_1, HW_TOUCH_BTN_PIN_3, HW_TOUCH_BTN_PIN_2, HW_TOUCH_BTN_PIN_4}; 
            #else
                const gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                    = { HW_TOUCH_LED_PIN_1, HW_TOUCH_LED_PIN_2, HW_TOUCH_LED_PIN_3, HW_TOUCH_LED_PIN_4}; 
                const gpio_num_t gpio_touch_btn_pins[TOTAL_BUTTONS]                 = { HW_TOUCH_BTN_PIN_1, HW_TOUCH_BTN_PIN_2, HW_TOUCH_BTN_PIN_3, HW_TOUCH_BTN_PIN_4}; 
            #endif
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI) 

            #ifdef USE_HOME_ASSISTANT
                const uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS]                   = { 1, 2 ,3, 4 };                                                                       
                const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                  = {ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID, 
                                                                                        ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID};
                
                char manufname[]                                                = {4, 'N', 'U', 'O', 'S'};
                char modelid []                                                 = {10, 'D', 'A', 'L', 'I', ' ', 'S', 'c', 'e', 'n', 'e'};
 
            #else //if tuya
                #ifdef USE_TUYA_BRDIGE 
                    #ifdef DALI_DIRECT_ADDRESSING 
                        //lbdxarah
                        char manufname[]                                        = {16, '_', 'T', 'Y', 'Z', 'B', '0', '1', '_', 'l', 'b', 'd', 'x', 'a', 'r', 'a', 'h'};
                        char modelid[]                                          = {6, 'T', 'S', '1', '1', '0', 'F'};

                        const uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS]           = { 1, 2 };                                                                       
                        const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]          = {ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID, ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID};
                    #else
                        //ydfvmon7   latest
                        char manufname[]                                        = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'a', 'n', 'x', 'j', 'o', 'n', 'u', '1'};
                        char modelid[]                                          = {6,'T', 'S', '0', '0', '1', '4'};

                        const uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS]           = { 1, 2 ,3, 4 };                                                                       
                        const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]          = {ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID, 
                                                                                   ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID};                        
                    #endif
                #endif
            #endif


            const gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                    = { HW_TOUCH_LED_PIN_1, HW_TOUCH_LED_PIN_2, HW_TOUCH_LED_PIN_3, HW_TOUCH_LED_PIN_4};
            const gpio_num_t gpio_load_pins[TOTAL_LOADS]                        = { DALI_RX_PIN, DALI_TX_PIN};
            const gpio_num_t gpio_touch_btn_pins[TOTAL_BUTTONS]                 = { HW_TOUCH_BTN_PIN_1, HW_TOUCH_BTN_PIN_2, HW_TOUCH_BTN_PIN_3, HW_TOUCH_BTN_PIN_4}; 


        /***********RGB DALI************* */  
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI)

                const uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS]                   = {1};

                #if(USE_COLOR_DEVICE == COLOR_RGB_ONLY)
                    const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]              = {0x0102 /*COLOR_LIGHT*/};
                #elif(USE_COLOR_DEVICE == COLOR_RGBW)
                    const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]              = {0x010D /*COLOR_LIGHT*/};
                #elif(USE_COLOR_DEVICE == COLOR_RGB_CW_WW)
                    const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]              = {0x010D /*EXTENDED_COLOR_LIGHT*/};
                #endif

            #ifdef USE_HOME_ASSISTANT
                char manufname[]                                                = {4, 'N', 'U', 'O', 'S'};
                char modelid []                                                 = {10, 'D', 'A', 'L', 'I', ' ', '1', '-', 'R', 'G', 'B'};
            #else
                    #if(USE_COLOR_DEVICE == COLOR_RGB_ONLY)
                    //r7xbuwbw
                    char manufname[]                                            = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'r', '7', 'x', 'b', 'u', 'w', 'b', 'w'};
                    //TS0503B
                    char modelid[]                                              = {7, 'T', 'S', '0', '5', '0', '3', 'B'};
                    #elif(USE_COLOR_DEVICE == COLOR_RGBW)
                    char manufname[]                                            = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'w', 'w', '5', 'y', 'e', 'n', 'a', '0'};
                    //TS0504B
                    char modelid[]                                              = {7, 'T', 'S', '0', '5', '0', '4', 'B'};
                    #elif(USE_COLOR_DEVICE == COLOR_RGB_CW_WW)

                    // xjakrdcp
                    //char manufname[]                                                = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'x', 'j', 'a', 'k', 'r', 'd', 'c', 'p'};
                    
                    // //GLEDOPTO
                    // char manufname[]                                                = {8, 'G', 'L', 'E', 'D', 'O', 'P', 'T', 'O'};
                    // char modelid[]                                                  = {9, 'G', 'L', '-', 'D', '-', '0', '0', '5', 'p'};
                    
                    // //TS0505B
                    //char modelid[]                                                  = {7, 'T', 'S', '0', '5', '0', '5', 'B'};
                    // char modelid[]                                                  = {9, 'G', 'L', '-', 'C', '-', '0', '0', '8', 'P'};

                    // // ww5yena0
                    // char manufname[]                                                = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'w', 'w', '5', 'y', 'e', 'n', 'a', '0'};
                    // //TS0505B
                    //char modelid[]                                                  = {7, 'T', 'S', '0', '5', '0', '5', 'B'};                    

                    //NUOS-RGBCW
                    // char modelid[]                                                  = {9, 'N', 'U', 'O', 'S', '-', 'R', 'G', 'B', 'W'}; 
                    char modelid[]                                              = {7, 'T', 'S', '0', '5', '0', '5', 'B'}; 

                    //s4zadi21
                    char manufname[]                                            = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 's', '4', 'z', 'a', 'd', 'i', '2', '1'};
                    // ww5yena0
                    //char manufname[]                                                = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'w', 'w', '5', 'y', 'e', 'n', 'a', '0'};

                    #endif

            #endif

                //Using Normal 4-On-ff Touch
                const gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                = { HW_TOUCH_LED_PIN_1, HW_TOUCH_LED_PIN_2, HW_TOUCH_LED_PIN_3, HW_TOUCH_LED_PIN_4};
                const gpio_num_t gpio_load_pins[TOTAL_LOADS]                    = { DALI_RX_PIN, DALI_TX_PIN};
                const gpio_num_t gpio_touch_btn_pins[TOTAL_BUTTONS]             = { HW_TOUCH_BTN_PIN_1, HW_TOUCH_BTN_PIN_2, HW_TOUCH_BTN_PIN_3, HW_TOUCH_BTN_PIN_4};

        #endif
        bool bCalMode = false;
        uint64_t start_time = 0;
        uint64_t end_time = 0;
        uint8_t vTaskMode = 0;
        bool start_dali_led_commissioning_task_flag = false;

        
        uint8_t total_rx_group_rcvd = 0;

        uint8_t gateway_type = 0;
    #else
        extern uint8_t gateway_type;
        extern uint8_t ep_selected_index;
        extern bool isr_service_installed;
        extern bool on_scene_attribute_reporting_cb_flag;
        extern bool rgb_led_blink_flag;
        extern const uint32_t blink_intervals[3];
        //extern SemaphoreHandle_t postHandlerSemaphore;
        //extern SemaphoreHandle_t xWebServerSemaphore;
        extern SemaphoreHandle_t recordsSemaphore;
        extern SemaphoreHandle_t zigbee_sem;
        extern SemaphoreHandle_t bindingSemaphore;
        extern SemaphoreHandle_t simple_desc_sem;
        extern TaskHandle_t query_neighbour_task_handle;
        extern TaskHandle_t query_callback_handle;

        extern bool blink_leds_start_flag;
        extern uint8_t scene_ep_index_mode;
        extern uint8_t scene_ep_index;

        extern uint8_t ep_id[5];
        extern uint8_t ep_cnts;
        extern uint8_t scene_counts;
        extern bool single_endpoint_gateway;
        extern bool blink_rgb_led_flag;
        extern bool blink_rgb_led_normal_functionality_flag;
        extern bool blink_rgb_led_commissioning_functionality_flag;
        extern bool switch_pressed_flag;
        extern uint8_t pressed_value;
        extern bool touchLedsOffAfter1MinuteEnable;
        extern int64_t switch_pressed_start_time;
        extern uint8_t switch_pressed_counts;
        extern uint8_t uchKeypressed;
        extern bool switch_detected;
        extern bool found_clusters_flag;

        extern uint8_t dali_on_webpage_commissioning_counts;
        extern dali_device_ids_t dali_nvs_stt[4];
        extern dst_node_info_t dst_node_info[MAX_BINDINGS];
        extern int binding_count;

        extern MyItem webpageItem[MAX_NODES];
        extern const uint16_t fan_speed_percentage[5];
        extern const uint16_t fan_speed_values[5];
        extern const uint8_t ac_temp_values[17];

        extern uint8_t global_index;
        extern bool toggle_state_flag;
        extern bool set_hardware_flag;
        extern bool set_state_flag;
        extern bool set_level_flag;
        extern bool set_color_flag;
        extern bool brightness_control_flag ;
        extern int node_counts;
        extern uint16_t global_group_id[4];
        extern scene_switch_s scene_group_switch_info;
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI)
        extern uint8_t global_dali_id[4]; 
        #else
        extern uint8_t global_dali_id[]; 
        #endif
        extern uint8_t global_scene_id[4];  
        extern bool change_cw_ww_color_flag;     
        extern stt_scene_switch_t existing_nodes_info[4];
        extern stt_scene_switch_t nodes_info;        
        extern zigbee_zcene_info_t zb_scene_info[4];
        // extern wifi_info_handle_t wifi_info;
        // bool timer3_running_flag;
        
        extern uint8_t target_percentage; 
        extern bool is_long_press_brightness;
        extern uint8_t selected_color_mode;
        extern uint8_t last_selected_color_mode;
        extern bool mode_change_flag;
        //extern uint32_t curtain_cal_time;
        //extern uint32_t offset_time;
        extern uint8_t dali_fade_time;
        extern uint8_t dali_fade_rate; 
        extern uint8_t dali_min_off_offset;
        extern uint8_t dali_range_size;

        extern bool double_press_click_enable[4];
        extern uint16_t disable_double_press_enable_counts;

        extern uint16_t cb_requests_counts;
        extern uint16_t cb_response_counts;
        // #ifdef USE_CCT_TIME_SYNC
        // extern bool set_cct_time_sync_request_flag;
        // #endif
        extern bool is_reporting[MAX_DST_EP][MAX_NODES];

        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
        extern const char* switch_ctrl_type[2]; 
        #else
        extern const char* switch_ctrl_type[2];             
        #endif
        extern const char* scene_ctrl_type[2];

        extern led_indicator_handle_t device_info[TOTAL_BUTTONS+1];

        extern device_overall_params_t device_info_backup;
        extern const uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS];
        extern const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS];
        extern char manufname[];
        extern char modelid [];
        extern char manufname1[];
        extern char manufname2[];
        extern char manufname3[];
        extern char manufname4[];

        extern char modelid1[];
        extern char modelid2[];
        extern char modelid3[];
        extern char modelid4[];

        extern const gpio_num_t gpio_touch_btn_pins[TOTAL_BUTTONS];
        extern const gpio_num_t gpio_touch_led_pins[TOTAL_LEDS];
        extern const gpio_num_t gpio_load_pins[TOTAL_LOADS];
        extern const gpio_num_t gpio_ir_led_pins[TOTAL_BUTTONS];
        extern gpio_num_t BUZZER_PIN;
        extern gpio_num_t PIR_INPUT_PIN;  
        extern gpio_num_t TEMP_HUMIDITY_PIN;
        extern const unsigned int GAS_LEAK_SENSOR_PEAK_DETECTED_VALUE;

        extern const gpio_num_t I2C_MASTER_SCL_IO;
        extern const gpio_num_t I2C_MASTER_SDA_IO;

        extern zigbee_info_t light_zb_info[TOTAL_ENDPOINTS];

        extern uint16_t ENDPOINT_CLUSTERS[TOTAL_ENDPOINTS];
        extern bool light_driver_deinit_flag;
    
        extern uint8_t wifi_webserver_active_flag;
        extern uint8_t global_switch_state;
        extern uint32_t wifi_webserver_active_counts;
        extern bool start_commissioning;
        extern bool ready_commisioning_flag;
        extern bool is_my_device_commissionned;
        extern bool is_some_device_unavailable;
        extern bool find_active_neighbours_task_flag;
        extern uint8_t zb_network_bdb_steering_count;

        extern bool bCalMode;
        extern uint64_t start_time;
        extern uint64_t end_time;
        extern uint8_t vTaskMode;
        extern bool start_dali_led_commissioning_task_flag;

    #endif     
//app_hardware_driver_on_off.c  
extern void nuos_set_hardware_fan_ctrl(uint8_t index);
extern void init_sensor_interrupt();

#ifdef __cplusplus
extern "C" {
#endif
    DaliNodeList read_dali_addressed_nodes();
    int32_t daliQueryPowerOnLevel(const uint8_t addr);
    int32_t daliQueryFadeTimeFadeRate(uint8_t addr);
    int32_t daliQueryDeviceType(uint8_t addr);
    int32_t daliQueryNextDeviceType(uint8_t addr);
    int32_t daliQueryGearFeatures(uint8_t addr);
    int32_t daliQueryDeviceInGroupA(uint8_t addr);
    int32_t daliQueryDeviceInGroupB(uint8_t addr);
    uint16_t daliQueryGear(uint8_t addr);
    void esp_dali_factory_reset_all_drivers();
    void dali_enable_query_mode();
    void dali_disable_query_mode();
    void dali_query_send(uint8_t id, uint8_t command);
    void switch_driver_gpios_intr_enabled(bool enabled);
    void set_state(uint8_t index);
    void set_dali_level(uint8_t index);
    void set_dali_color_temp(uint8_t index, bool is_color_change);
    void set_color_temp_args(uint16_t color);
    esp_err_t nuos_set_state_attribute(uint8_t index);
    void nuos_set_state_command(uint8_t dpid, uint8_t state);
    void nuos_set_level_command(uint8_t dpid, uint8_t state);
    void nuos_set_color_temp_command(uint8_t dpid, uint16_t color);
    esp_err_t nuos_set_state_attribute_rgb(uint8_t index);
    esp_err_t nuso_set_state_attribute_on_dali_rx(uint8_t index_1, bool state_);
    esp_err_t nuos_set_level_attribute_on_dali_rx(uint8_t index_1, uint8_t level);
    esp_err_t nuos_set_color_temp_level_attribute_on_dali_rx(uint8_t index, uint8_t level);
    extern void nuos_zb_init_hardware();
    extern void nuos_zb_set_hardware(uint8_t index, uint8_t value);
    extern void nuos_zb_set_hardware_led_for_zb_commissioning(uint8_t is_toggle);
    extern bool nuos_set_hardware_brightness(uint32_t pin);
    extern void nuos_init_hardware_dimming_up_down(uint32_t pin);
    extern void init_dali_hw();
    extern void set_dali_color_temp(uint8_t index, bool is_color_change);
    extern void nuos_dali_add_light_to_group(uint8_t addr, uint8_t group_id);
    extern void nuos_dali_remove_light_from_group(uint8_t addr, uint8_t group_id);
    extern void nuos_dali_toggle_group(uint8_t group_id, uint8_t index, bool toggle_state, uint8_t brightness);
    extern int start_dali_addressing(uint8_t startAddresses, uint8_t numAddresses);
    extern void nuos_dali_set_color_temperate(uint8_t index);
    extern void nuos_dali_set_group_color_temperature(uint8_t group_id, uint8_t index, uint16_t value);
    extern void nuos_dali_set_group_rgb_temperature(uint8_t group_id, uint8_t r, uint8_t g, uint8_t b);
    extern void nuos_dali_set_group_brightness(uint8_t group_id, uint8_t index, uint8_t value);
    extern void nuos_dali_normal_set_group_brightness(uint8_t group_id, uint8_t index, uint8_t value);
    
    extern void nuos_set_state_touch_leds(bool state);
    extern void nuos_set_state_touch_leds_to_original();
    extern void nuos_toggle_leds(uint8_t index);
    extern void nuos_on_off_led(uint8_t index, uint8_t _level);
    extern void set_level_value(uint8_t _level);
    extern uint8_t total_rx_group_rcvd;
    extern uart_word_length_t string_to_uart_word_length_t(const char *data_bits_str);
    extern uart_stop_bits_t string_to_uart_stop_bits_t(const char *stop_bits_str);
    extern uart_parity_t string_to_uart_parity_t(const char *parity_str);
    // extern uart_hw_flowcontrol_t string_to_uart_hw_flowcontrol_t(const char *flow_ctrl_str);
    extern void nuos_set_hw_brightness(uint8_t index);
    extern void init_sensor_commissioning_task();
    extern bool nuos_set_hardware_brightness_2(uint8_t index);
    extern void dali_set_mode(uint8_t mode);
    
    #if(USE_NUOS_ZB_DEVICE_TYPE ==  DEVICE_DALI_DIRECT_SWITCH)
    extern void nuos_dali_set_state_group(uint8_t index, uint8_t brightness);
    extern void process_dali_tasks(uint8_t index, uint8_t is_toggle, uint8_t is_scene);
    #else
    extern void nuos_dali_set_state_group(uint8_t index, bool brightness);
    extern void process_dali_tasks(uint8_t index, uint8_t is_toggle, uint8_t is_scene);
    #endif
    extern void nuos_dali_set_state(uint8_t dali_id, uint8_t state);
    extern void nuos_dali_set_brightness(uint8_t dali_id, uint8_t level);
    extern void nuos_dali_set_cct_color(uint8_t did, uint16_t value);
    extern void nuos_dali_set_rgb_color(uint8_t did, uint8_t r, uint8_t g, uint8_t b, bool mode_change_flag);
    extern void start_dali_led_blink_task();
 
    extern void nuos_dali_add_device_to_scene(uint8_t device_id, uint8_t scene_id, uint8_t scene_level, uint16_t cct_temp);
    extern void nuos_dali_remove_device_from_scene(uint8_t device_id, uint8_t scene_id);

    extern void nuos_dali_add_group_to_scene(uint8_t group_id, uint8_t scene_id, uint8_t scene_level, uint16_t cct_temp);
    extern void nuos_dali_remove_group_from_scene(uint8_t device_id, uint8_t scene_id);

    extern void nuos_dali_add_device_state_to_scene(uint8_t device_id, uint8_t scene_id);
    extern void nuos_dali_set_power_on_level(uint8_t dali_id, uint8_t level);
    extern void nuos_dali_set_fade_time_fade_rate(uint8_t dali_id, uint8_t fade_time, uint8_t fade_rate);
    extern int get_all_dali_addresses(uint8_t *foundAddresses); 

    void send_ac_model(uint8_t model_num);
    void send_fan(uint8_t fan_speed);

    extern void nuos_set_dali_fade_time(uint8_t time);
    extern void nuos_set_dali_fade_rate(uint8_t rate);
    extern void call_common_check_auto_off();
    extern void set_all_leds_to_original_state();

    extern void set_parameter_ep_index_selected(uint8_t bt_index);
    extern void set_parameter_toggle_bt_func_selected(uint8_t bt_index);
    extern uint8_t get_parameter_ep_index_selected();
    extern void nuos_dali_rgb_add_device_to_scene(uint8_t device_id, uint8_t scene_id, uint8_t scene_level, uint8_t r, uint8_t g, uint8_t b);
    extern void dali_rx_intr_enabled(bool enable);
    extern void convert_colors_to_index(uint8_t index, bool is_long_press);
    extern void set_color_to_updown_leds(uint8_t index);
#ifdef __cplusplus
}
#endif 
extern void init_fading();
extern void nuos_on_off_led(uint8_t index, uint8_t _state);
extern void nuos_zb_set_hardware_curtain(uint8_t index, uint8_t is_toggle);
extern uint8_t nuos_get_button_press_index(uint32_t pin);
extern void xy_to_rgb(unsigned int x, unsigned int y, double *r, double *g, double *b);
extern void nuos_set_level_to_rgb(uint8_t index);
extern void nuos_zb_set_scene_switch_click(uint32_t io_num, uint8_t state);
extern void nuos_zb_convert_xy_to_rgb(uint8_t index, float red_f, float blue_f, float green_f);
extern void nuos_zb_set_leds_only(uint8_t index, uint8_t is_toggle);


extern void pause_curtain_timer();
extern void resume_curtain_timer(uint8_t index);
extern void curtain_cmd_open(void);
extern void curtain_cmd_close(void);
extern void curtain_cmd_stop(void);
extern int curtain_cmd_goto_pct(uint8_t pct) ;
extern void set_curtain_percentage(uint8_t value, bool set_hw_flag);
extern void set_curtain_load(uint8_t cur_mode);
