    
    #include "app_constants.h"
    #include "pin_config.h"
    #include "driver/gpio.h"
    #include "driver/uart.h"
    #include "esp_zigbee_type.h"
    #include "switch_driver.h"
    #include "app_nvs_store_info.h"


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

        // //RGB
        // bool rgbw_state[4]; 
        // uint8_t rgbw[4];
        #if (USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI)
        #else
        bool color_or_fan_state;
        bool dim_up;
        bool level_up; 
        uint16_t light_color_x;
        uint16_t light_color_y;
        //ac IR
        uint8_t ac_mode;
        int8_t ac_decode_type;
        uint8_t ac_temperature; 
        //fan
        uint8_t fan_speed; 
        #endif 
    } led_indicator_handle_t;

    typedef struct device_overall_params_ctx_t
	{ 
        bool state[5];
        uint8_t level[5];
        uint8_t mode;
    } device_overall_params_t;

    typedef struct wifi_info_t
    {
        bool is_wifi_sta_mode;
        uint8_t ip4;
        char wifi_ssid[32];
        char wifi_pass[32];
    } wifi_info_handle_t;

    #define MAX_NODES    20
    #define MAX_DST_EP   4

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
        uint8_t selected_switch;
        uint8_t group_id;
        uint8_t scene_ids[4];
        uint8_t control_type;
        uint8_t scn_ctrl_type;
    } scene_switch_s;


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

    // typedef struct commissioning_keys_t
    // {
    //     bool start_commissioning;
    //     bool wifi_webserver_active_flag;
    //     uint8_t zb_network_bdb_steering_count;

    // } commissioning_info_handle_t;

//  https://yatyyt4jowdyyswqkog4i42helmwilyu.ui.nabu.casa/lovelace/0
////////////////////////////////////////////////////////////////////////
    #define MAX_BINDINGS                                                        40  // adjust this based on how many you expect
    #ifdef DECLARE_MAIN
        bool rgb_led_blink_flag                                                 = false;
        const uint32_t blink_intervals[3]                                       = {200, 500, 1000}; // Fast, Medium, Slow
        //SemaphoreHandle_t postHandlerSemaphore                                  = NULL;
        //SemaphoreHandle_t xWebServerSemaphore                                   = NULL;
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

        int64_t curtain_cal_time                                               = 0;

        bool double_press_click_enable                                          = false;
        uint16_t disable_double_press_enable_counts                             = 0;

        uint16_t cb_requests_counts                                             = 0;
        uint16_t cb_response_counts                                             = 0;
        #ifdef USE_CCT_TIME_SYNC
        bool set_cct_time_sync_request_flag                                     = false;
        #endif
        bool is_reporting[MAX_DST_EP][MAX_NODES];
        scene_switch_s scene_switch_t[4];
        uint16_t global_group_id[4]                                             = { 0x1, 0x2, 0x3, 0x4 };
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
        uint16_t global_dali_id[4]                                              = { 11, 12, 13, 14}; 
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI)
        uint16_t global_dali_id[4]                                              = { 10, 11, 12, 13 }; 
        #else
        uint16_t global_dali_id[32]                                             = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
                                                                                    11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
                                                                                    21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 
                                                                                    31, 32};  
        #endif
        uint8_t global_scene_id[4]                                              = { 0x1, 0x2, 0x3, 0x4 };
        dali_device_ids_t dali_nvs_stt[4];

 
        dst_node_info_t dst_node_info[MAX_BINDINGS];
        //zb_binding_data_t bindings[MAX_BINDINGS];
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
        wifi_info_handle_t wifi_info;
        bool timer3_running_flag                                                = true;

        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI)
            #ifdef USE_INDIVIDUAL_DALI_ADDRESSING
                led_indicator_handle_t device_info[TOTAL_ENDPOINTS+1];
            #else
                led_indicator_handle_t device_info[TOTAL_BUTTONS+1];
            #endif
        #else
            led_indicator_handle_t device_info[TOTAL_BUTTONS+1];
        #endif

        device_overall_params_t device_info_backup;
        zigbee_info_t light_zb_info[TOTAL_ENDPOINTS+1];
        bool light_driver_deinit_flag                                           = false;
        uint8_t wifi_webserver_active_flag                                      = false;
        uint8_t global_switch_state                                             = 0;
        bool start_commissioning                                                = false;
        bool ready_commisioning_flag                                            = false;
        bool is_my_device_commissionned                                         = false;
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
        /*************ON-OFF************* */  
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4R_ON_OFF_LIGHT)
            const uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS]                       = {1, 2 ,3, 4};
            const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                      = {ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID, 
                                                                                   ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID}; 

            #ifdef USE_HOME_ASSISTANT
                char manufname[]                                                = {7, 'L', 'I', 'G', 'H', 'T', ' ', '1'};
                char modelid []                                                 = {4, 'N', 'U', 'O', 'S'};
                // char manufname2[]                                               = {7, 'L', 'I', 'G', 'H', 'T', ' ', '2'};
                // char manufname3[]                                               = {7, 'L', 'I', 'G', 'H', 'T', ' ', '3'};
                // char manufname4[]                                               = {7, 'L', 'I', 'G', 'H', 'T', ' ', '4'};                                               
            #else
                #ifdef USE_TUYA_BRDIGE 
                //ydfvmon7   latest
                // char manufname1[]                                                = {7, 'L', 'I', 'G', 'H', 'T', ' ', '1'};
                // char manufname2[]                                                = {7, 'L', 'I', 'G', 'H', 'T', ' ', '2'};
                // char manufname3[]                                                = {7, 'L', 'I', 'G', 'H', 'T', ' ', '3'};
                // char manufname4[]                                                = {7, 'L', 'I', 'G', 'H', 'T', ' ', '4'};

                    char manufname[]                                            = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'y', 'd', 'f', 'v', 'm', 'o', 'n', '7'};
                    //char manufname[]                                            = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'j', 'f', 'j', 's', 's', '8', 'r', 'l'};
                    char modelid[]                                              = {6,'T', 'S', '0', '0', '1', '4'};

                    // char modelid1[]                                              = {8,'T', 'S', '0', '0', '1', '4', '_', '1'};
                    // char modelid2[]                                              = {8,'T', 'S', '0', '0', '1', '4', '_', '2'};
                    // char modelid3[]                                              = {8,'T', 'S', '0', '0', '1', '4', '_', '3'};
                    // char modelid4[]                                              = {8,'T', 'S', '0', '0', '1', '4', '_', '4'};

                #endif
            #endif
            const gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                    = { HW_TOUCH_LED_PIN_1, HW_TOUCH_LED_PIN_2, HW_TOUCH_LED_PIN_3, HW_TOUCH_LED_PIN_4};
            const gpio_num_t gpio_load_pins[TOTAL_LOADS]                        = { HW_LOAD_PIN_1, HW_LOAD_PIN_2, HW_LOAD_PIN_3, HW_LOAD_PIN_4};
            const gpio_num_t gpio_touch_btn_pins[TOTAL_BUTTONS]                 = { HW_TOUCH_BTN_PIN_1, HW_TOUCH_BTN_PIN_2, HW_TOUCH_BTN_PIN_3, HW_TOUCH_BTN_PIN_4}; 

        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2R_ON_OFF_LIGHT)
            uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS]                             = {1, 2};  
            const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                      = {ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID};
            #ifdef USE_HOME_ASSISTANT
                const char manufname[]                                          = {4, 'N', 'U', 'O', 'S'};
                const char modelid []                                           = {12, '2', '-', 'G', 'a', 'n', 'g', ' ', 'L', 'i', 'g', 'h', 't'};
            #else  //irptc32e  
                #ifdef USE_TUYA_BRDIGE 
                    char manufname[]                                                = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'i', 'r', 'p', 't', 'c', '3', '2', 'e'};
                    char modelid[]                                                  = {6,'T', 'S', '0', '0', '1', '2'};
                #endif                
            #endif
            const gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                     = { HW_TOUCH_LED_PIN_1, HW_TOUCH_LED_PIN_2};
            const gpio_num_t gpio_load_pins[TOTAL_LOADS]                         = { HW_LOAD_PIN_1, HW_LOAD_PIN_2};
            const gpio_num_t gpio_touch_btn_pins[TOTAL_BUTTONS]                  = { HW_TOUCH_BTN_PIN_1, HW_TOUCH_BTN_PIN_2}; 
  
        /*************CURTAIN************* */  
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2CH_CURTAIN)
            const uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS]                       = {1, 2};
            const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                      = {ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID, ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID};
            #ifdef USE_HOME_ASSISTANT
                char manufname[]                                                = {4, 'N', 'U', 'O', 'S'};
                char modelid []                                                 = {12, '2', '-', 'C', 'h', ' ', 'C', 'u', 'r', 't', 'a', 'i', 'n'};
            #else  //vmmaf6q9  
                #ifdef USE_TUYA_BRDIGE 
                    char manufname[]                                                = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'v', 'm', 'm', 'a', 'f', '6', 'q', '9'};
                    char modelid[]                                                  = {6, 'T', 'S', '0', '0', '1', '2'};
                #endif                
            #endif
            const gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                        = { HW_TOUCH_LED_PIN_1, HW_TOUCH_LED_PIN_2, HW_TOUCH_LED_PIN_3, HW_TOUCH_LED_PIN_4};
            const gpio_num_t gpio_load_pins[TOTAL_LOADS]                            = { HW_LOAD_PIN_1, HW_LOAD_PIN_2, HW_LOAD_PIN_3, HW_LOAD_PIN_4};
            const gpio_num_t gpio_touch_btn_pins[TOTAL_BUTTONS]                     = { HW_TOUCH_BTN_PIN_1, HW_TOUCH_BTN_PIN_2, HW_TOUCH_BTN_PIN_3, HW_TOUCH_BTN_PIN_4}; 

        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN_SWITCH)
            const uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS]                       = {1};
            const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                      = {ESP_ZB_HA_WINDOW_COVERING_DEVICE_ID};
            #ifdef USE_HOME_ASSISTANT
                char manufname[]                                                = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'p', 'n', '0', '1', 'n', 'z', 'e', '7'};
                char modelid[]                                                  = {6, 'T', 'S', '0', '0', '1', '1'};
            #else 
                #ifdef USE_TUYA_BRDIGE 
                    //0hjr874h
                    // char manufname[]                                            = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'h', 'l', 'd', 's', 't', 'q', 'g', 'i'};
                    // //char manufname[]                                            = {16, '_', 'T', 'Z', 'E', '2', '0', '0', '_', 'h', 'l', 'd', 's', 't', 'q', 'g', 'i'};                                          
                    // char modelid[]                                              = {6, 'S', 'M', '1', '3', '0', 'F'}; 
                    //  //kuzmsnrv
                    char manufname[]                                            = {16, '_', 'T', 'Z', '3', '0', '0', '2', '_', 'k', 'u', 'z', 'm', 's', 'n', 'r', 'v'};                                          
                    char modelid[]                                              = {6, 'T', 'S', '1', '3', '0', 'F'};

                    //hldstqgi                                               
                    // char manufname[]                                            = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'h', 'l', 'd', 's', 't', 'q', 'g', 'i'};                                          
                    // char modelid[]                                              = {6, 'T', 'S', '1', '3', '0', 'F'}; 
                    //fcfixupo 
                    //char manufname[]                                            = {16, '_', 'T', 'Z', '3', '0', '0', '2', '_', 'f', 'c', 'f', 'i', 'x', 'u', 'p', 'o'};
                    //char modelid[]                                              = {6, 'S', 'M', '1', '3', '0', 'F'}; 
                #endif                                    
            #endif    
            const gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                        = { HW_TOUCH_LED_PIN_1, HW_TOUCH_LED_PIN_2};
            const gpio_num_t gpio_load_pins[TOTAL_LOADS]                            = { HW_LOAD_PIN_1, HW_LOAD_PIN_2};
            const gpio_num_t gpio_touch_btn_pins[TOTAL_BUTTONS]                     = { HW_TOUCH_BTN_PIN_1, HW_TOUCH_BTN_PIN_2}; 

        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN)
            uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS]                                 = {1};  
            
            #ifdef USE_HOME_ASSISTANT
                const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                      = {ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID};
                const char manufname[]                                              = {4, 'N', 'U', 'O', 'S'};
                const char modelid []                                               = {12, '1', '-', 'C', 'h', ' ', 'C', 'u', 'r', 't', 'a', 'i', 'n'};
            #else  //pn01nze7  
                #ifdef USE_TUYA_BRDIGE 
                    #ifdef TUYA_ATTRIBUTES
                    const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                  = {ESP_ZB_HA_WINDOW_COVERING_DEVICE_ID};
                    //fcfixupo 
                    char manufname[]                                            = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'f', 'c', 'f', 'i', 'x', 'u', 'p', 'o'};
                    char modelid[]                                              = {6, 'T', 'S', '1', '3', '0', 'F'};                    
                    #else
                    const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                  = {ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID};
                    char manufname[]                                            = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'p', 'n', '0', '1', 'n', 'z', 'e', '7'};
                    char modelid[]                                              = {6, 'T', 'S', '0', '0', '1', '1'};                    
                    #endif
                    //char manufname[]                                            = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'p', 'n', '0', '1', 'n', 'z', 'e', '7'};
                    //char modelid[]                                              = {6, 'T', 'S', '0', '0', '1', '1'};
                    //not working...
                   //char manufname[]                                             = {16, '_', 'T', 'Z', '3', '0', '0', '2', '_', 'k', 'u', 'z', 'm', 's', 'n', 'r', 'v'};                                          
                    //char modelid[]                                              = {6, 'T', 'S', '1', '3', '0', 'F'};

                    // char manufname[]                                            = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'h', 'l', 'd', 's', 't', 'q', 'g', 'i'};                                          
                    // char modelid[]                                              = {6, 'T', 'S', '1', '3', '0', 'F'}; 


                #endif                
            #endif 
        const gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                      = { HW_TOUCH_LED_PIN_1, HW_TOUCH_LED_PIN_2};
        const gpio_num_t gpio_load_pins[TOTAL_LOADS]                          = { HW_LOAD_PIN_1, HW_LOAD_PIN_2};
        const gpio_num_t gpio_touch_btn_pins[TOTAL_BUTTONS]                   = { HW_TOUCH_BTN_PIN_1, HW_TOUCH_BTN_PIN_2}; 

        /*************ON-OFF & DIM************* */ 
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4T_ANALOG_DIMMABLE_LIGHT)
            const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                      = {ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID, ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID};
            const uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS]                       = {1, 2};

            #ifdef USE_HOME_ASSISTANT
                char manufname[]                                                = {4, 'N', 'U', 'O', 'S'};
                char modelid []                                                 = {19, '4', '-', 'G', 'a', 'n', 'g', ' ', 'D', 'i', 'm', 'm', 'e', 'r', ' ', 'L', 'i', 'g', 'h', 't'};
            #endif
            const gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                    = { HW_TOUCH_LED_PIN_1, HW_TOUCH_LED_PIN_2, HW_TOUCH_LED_PIN_3, HW_TOUCH_LED_PIN_4};
            const gpio_num_t gpio_load_pins[TOTAL_LOADS]                        = { HW_LOAD_PIN_1, HW_LOAD_PIN_2, HW_LOAD_PIN_3, HW_LOAD_PIN_4};
            const gpio_num_t gpio_touch_btn_pins[TOTAL_BUTTONS]                 = { HW_TOUCH_BTN_PIN_1, HW_TOUCH_BTN_PIN_2, HW_TOUCH_BTN_PIN_3, HW_TOUCH_BTN_PIN_4}; 

        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_ANALOG_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_PHASE_CUT_DIMMABLE_LIGHT)
            const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                      = {ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID, ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID};
            uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS]                             = {1, 2};
            #ifdef USE_HOME_ASSISTANT
                char manufname[]                                                = {4, 'N', 'U', 'O', 'S'};
                char modelid []                                                 = {19, '2', '-', 'G', 'a', 'n', 'g', ' ', 'D', 'i', 'm', 'm', 'e', 'r', ' ', 'L', 'i', 'g', 'h', 't'};
            #else  //aat0bszz  
                #ifdef USE_TUYA_BRDIGE
                    //_TZ3000_7ysdnebc (readymade)
                    //char manufname[]                                            = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', '7', 'y', 's', 'd', 'n', 'e', 'b', 'c'};
                    //60jbfyqy     ->>pagajpog
                    // char manufname[]                                            = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', '6', '0', 'j', 'b', 'f', 'y', 'q', 'y'};
                    // char modelid[]                                              = {6, 'T', 'S', '1', '1', '0', '1'};
                    char manufname[]                                            = {16, '_', 'T', 'Y', 'Z', 'B', '0', '1', '_', '6', '0', 'j', 'b', 'f', 'y', 'q', 'y'};
                    char modelid[]                                              = {6, 'T', 'S', '1', '1', '0', 'F'};

                    //aykjykoi
                    // char manufname[]                                            = {16, '_', 'T', 'Y', 'Z', 'B', '0', '1', '_', 'a', 'y', 'k', 'j', 'y', 'k', 'o', 'i'};
                    // char modelid[]                                              = {6, 'T', 'S', '1', '1', '0', 'F'};

                    //char manufname[]                                            = {16, '_', 'T', 'Z', 'E', '2', '0', '0', '_', '6', '0', 'j', 'b', 'f', 'y', 'q', 'y'};
                    //char modelid[]                                              = {6, 'T', 'S', '0', '6', '0', '1'};
                #endif                
            #endif         
             
            const gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                     = { HW_TOUCH_LED_PIN_1, HW_TOUCH_LED_PIN_2};
            const gpio_num_t gpio_load_pins[TOTAL_LOADS]                         = { HW_LOAD_PIN_1, HW_LOAD_PIN_2};
            const gpio_num_t gpio_touch_btn_pins[TOTAL_BUTTONS]                  = { HW_TOUCH_BTN_PIN_1, HW_TOUCH_BTN_PIN_2};          

        /* ************ON-OFF, DIM & COLOR************* */             
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4T_COLOR_DIMMABLE_LIGHT)
            const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                      = {ESP_ZB_HA_COLOR_DIMMABLE_LIGHT_DEVICE_ID, ESP_ZB_HA_COLOR_DIMMABLE_LIGHT_DEVICE_ID, 
                                                                                ESP_ZB_HA_COLOR_DIMMABLE_LIGHT_DEVICE_ID, ESP_ZB_HA_COLOR_DIMMABLE_LIGHT_DEVICE_ID};
            const uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS]                       = {1, 2 ,3, 4};

            #ifdef USE_HOME_ASSISTANT
                char manufname[]                                                = {4, 'N', 'U', 'O', 'S'};
                char modelid []                                                 = {18, '4', '-', 'G', 'a', 'n', 'g', ' ', 'C', 'o', 'l', 'o', 'r', ' ', 'L', 'i', 'g', 'h', 't'};
            #endif     
            const gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                    = { HW_TOUCH_LED_PIN_1, HW_TOUCH_LED_PIN_2, HW_TOUCH_LED_PIN_3, HW_TOUCH_LED_PIN_4};
            const gpio_num_t gpio_load_pins[TOTAL_LOADS]                        = { HW_LOAD_PIN_1, HW_LOAD_PIN_2, HW_LOAD_PIN_3, HW_LOAD_PIN_4};
            const gpio_num_t gpio_touch_btn_pins[TOTAL_BUTTONS]                 = { HW_TOUCH_BTN_PIN_1, HW_TOUCH_BTN_PIN_2, HW_TOUCH_BTN_PIN_3, HW_TOUCH_BTN_PIN_4}; 

        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_COLOR_DIMMABLE_LIGHT)
            const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                      = {ESP_ZB_HA_COLOR_DIMMABLE_LIGHT_DEVICE_ID, ESP_ZB_HA_COLOR_DIMMABLE_LIGHT_DEVICE_ID};
            const uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS]                       = {1, 2};

            #ifdef USE_HOME_ASSISTANT
            char manufname[]                                                    = {4, 'N', 'U', 'O', 'S'};
            char modelid []                                                     = {18, '2', '-', 'G', 'a', 'n', 'g', ' ', 'C', 'o', 'l', 'o', 'r', ' ', 'L', 'i', 'g', 'h', 't'}; 
            #endif

            const gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                    = { HW_TOUCH_LED_PIN_1, HW_LOAD_PIN_2};
            const gpio_num_t gpio_load_pins[TOTAL_LOADS]                        = { HW_LOAD_PIN_1, HW_TOUCH_LED_PIN_2};
            const gpio_num_t gpio_touch_btn_pins[TOTAL_BUTTONS]                 = { HW_TOUCH_BTN_PIN_1, HW_TOUCH_BTN_PIN_2}; 

        /*************1 LIGHT 1 FAN************* */                      
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN)    //1 Light 1 Fan
           
            
            #ifdef USE_ZB_ONLY_FAN
                const uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS]                       = {1};
                //const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                      = {ESP_ZB_HA_THERMOSTAT_DEVICE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID};
                const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                      = {ESP_ZB_HA_SMART_PLUG_DEVICE_ID};//= {ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID};
            #else
                const uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS]                       = {1, 2};
                const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                      = {ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID, ESP_ZB_HA_THERMOSTAT_DEVICE_ID};
            #endif
            

            #ifdef USE_HOME_ASSISTANT
                const char modelid[]                                                = {9, 'L', 'i', 'g', 'h', 't', ' ', 'F', 'a', 'n'};
                const char manufname[]                                              = {6, 'T', 'S', '0', '6', '0', '1'};
            #else
                

                //_TZE200_lawxy9e2
                // zs0jfagx
                //NUOS  e1qai9sl
                // NUOS cm3evd4x
                #ifdef USE_TUYA_BRDIGE
                //ddmsita6
                //-->//const char manufname[]                                          = {16, '_', 'T', 'Z', 'E', '2', '0', '0', '_', 'd', 'd', 'm', 's', 'i', 't', 'a', '6'};
                //qissu5b3
                //const char manufname[]                                          = {16, '_', 'T', 'Z', 'E', '2', '0', '0', '_', 'q', 'i', 's', 's', 'u', '5', 'b', '3'};
                #ifdef TUYA_ATTRIBUTES
                    //iloszuao 
                    //const char manufname[]                                          = {16, '_', 'T', 'Z', 'E', '2', '8', '4', '_', 'i', 'l', 'o', 's', 'z', 'u', 'a', 'o'};
                    //1dacqtcg
                    //const char manufname[]                                          = {16, '_', 'T', 'Z', 'E', '2', '8', '4', '_', '1', 'd', 'a', 'c', 'q', 't', 'c', 'g'};
                    const char modelid[]                                            = {6, 'T', 'S', '0', '6', '0', '1'};
                    // const char modelid[]                                           = {18, 'K','i','n','g',' ', 'O','f',' ',  'F','a','n','s',',', 'I','n','c','.'};
                    // const char manufname[]                                         = {16, 'H','D','C','5','2','E','a','s','t','w','i','n','d','F','a','n'};
                    //ojdbar5r
                    const char manufname[]                                          = {16, '_', 'T', 'Z', 'E', '2', '8', '4', '_', 'o', 'j', 'd', 'b', 'a', 'r', '5', 'r'};
                #else
                    //_TZ3210_lzqq3u4r
                    const char manufname[]                                          = {16, '_', 'T', 'Z', '3', '2', '1', '0', '_', 'l', 'z', 'q', 'q', '3', 'u', '4', 'r'};
                    //TS0501
                    const char modelid[]                                            = {6, 'T', 'S', '0', '5', '0', '1'};
                #endif
                #endif
            #endif
            #ifdef USE_TUYA_BRDIGE
            const gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                    = { HW_TOUCH_LED_PIN_1, HW_TOUCH_LED_PIN_3, HW_TOUCH_LED_PIN_2, HW_TOUCH_LED_PIN_4};
            const gpio_num_t gpio_load_pins[TOTAL_LOADS]                        = { HW_LOAD_PIN_1, HW_LOAD_PIN_2, HW_LOAD_PIN_3, HW_LOAD_PIN_4};
            const gpio_num_t gpio_touch_btn_pins[TOTAL_BUTTONS]                 = { HW_TOUCH_BTN_PIN_1, HW_TOUCH_BTN_PIN_3, HW_TOUCH_BTN_PIN_2, HW_TOUCH_BTN_PIN_4}; 
            #else
            const gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                    = { HW_TOUCH_LED_PIN_1, HW_TOUCH_LED_PIN_3, HW_TOUCH_LED_PIN_2, HW_TOUCH_LED_PIN_4};
            const gpio_num_t gpio_load_pins[TOTAL_LOADS]                        = { HW_LOAD_PIN_1, HW_LOAD_PIN_2, HW_LOAD_PIN_3, HW_LOAD_PIN_4};
            //const gpio_num_t gpio_touch_btn_pins[TOTAL_BUTTONS]                 = { HW_TOUCH_BTN_PIN_1, HW_TOUCH_BTN_PIN_3, HW_TOUCH_BTN_PIN_2, HW_TOUCH_BTN_PIN_4}; 
            const gpio_num_t gpio_touch_btn_pins[TOTAL_BUTTONS]                 = { HW_TOUCH_BTN_PIN_1, HW_TOUCH_BTN_PIN_3, HW_TOUCH_BTN_PIN_2, HW_TOUCH_BTN_PIN_4}; 
            
            #endif
             #ifdef USE_ZB_ONLY_FAN
                uint16_t ENDPOINT_CLUSTERS[TOTAL_ENDPOINTS]                         = {0xEF00};
             #else
                uint16_t ENDPOINT_CLUSTERS[TOTAL_ENDPOINTS]                         = {ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL};
             #endif

        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN_CUSTOM)           //1 Light 1 Fan Custom
            const uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS]                       = {1, 2};
            const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                      = {ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID, ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID};

            #ifdef USE_HOME_ASSISTANT
                const char modelid[]                                            = {9, 'L', 'i', 'g', 'h', 't', ' ', 'F', 'a', 'n'};
                const char manufname[]                                          = {4, 'N', 'U', 'O', 'S'};
            #else
                // NUOS 3j1b2aiw
                #ifdef USE_TUYA_BRDIGE 
                    char manufname[]                                            = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', '3', 'j', '1', 'b', '2', 'a', 'i', 'w'}; 
                    const char modelid[]                                        = {6, 'T', 'S', '1', '1', '0', '1'};
                #endif
            #endif  
            const gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                    = { HW_TOUCH_LED_PIN_1, HW_TOUCH_LED_PIN_3, HW_TOUCH_LED_PIN_2, HW_TOUCH_LED_PIN_4};
            const gpio_num_t gpio_load_pins[TOTAL_LOADS]                        = { HW_LOAD_PIN_1, HW_LOAD_PIN_2, HW_LOAD_PIN_3, HW_LOAD_PIN_4};
            const gpio_num_t gpio_touch_btn_pins[TOTAL_BUTTONS]                 = { HW_TOUCH_BTN_PIN_1, HW_TOUCH_BTN_PIN_3, HW_TOUCH_BTN_PIN_2, HW_TOUCH_BTN_PIN_4}; 

        /*************IR BLASTER************* */ 
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
            const uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS]                       = {1};
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
                const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                  = {ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID};
            #else
                const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                  = {ESP_ZB_HA_THERMOSTAT_DEVICE_ID};
            #endif
            #ifdef USE_HOME_ASSISTANT
                const char modelid[]                                            = {10, 'I', 'R', ' ', 'B', 'l', 'a', 's', 't', 'e', 'r'};
                const char manufname[]                                          = {4, 'N', 'U', 'O', 'S'};
            #else
                #ifdef USE_TUYA_BRDIGE 
                    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
                        ////56lfhvvn  -->testing
                        ////char manufname[]                                      = {16, '_', 'T', 'Z', 'E', '2', '0', '0', '_', '5', '6', 'l', 'f', 'h', 'v', 'v', 'n'};
                        //jl268nxf   -->working with issues
                        //char manufname[]                                        = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'j', 'l', '2', '6', '8', 'n', 'x', 'f'};
                        //cftyjrak   -->working with no issue
                        char manufname[]                                        = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'c', 'f', 't', 'y', 'j', 'r', 'a', 'k'};
                        const char modelid[]                                    = {6, 'T', 'S', '1', '1', '0', '1'};
                    #else
                        //tzzpjnbr
                        //char manufname[]                                        = {16, '_', 'T', 'Z', 'E', '2', '0', '0', '_', 't', 'z', 'z', 'p', 'j', 'n', 'b', 'r'};

                        //nwpaggmi
                        //char manufname[]                                        = {16, '_', 'T', 'Z', 'E', '2', '0', '0', '_', 'n', 'w', 'p', 'a', 'g', 'g', 'm', 'i'};

                        // //n4qc9fg7
                        // char manufname[]                                        = {16, '_', 'T', 'Z', 'E', '2', '0', '4', '_', 'n', '4', 'q', 'c', '9', 'f', 'g', '7'};//dev_id=smartplug
                        char manufname[]                                        = {16, '_', 'T', 'Z', 'E', '2', '0', '0', '_', 'n', '4', 'q', 'c', '9', 'f', 'g', '7'}; //dev_id=thermopstat
                        const char modelid[]                                    = {6, 'T', 'S', '0', '6', '0', '1'};    
                        //TS0308
                        //const char modelid[]                                    = {6, 'T', 'S', '0', '3', '0', '8'};
                    #endif
                    //NUOSIRAC
                #endif
            #endif

            const uint32_t gpio_touch_led_pins[TOTAL_LEDS]                      = { HW_TOUCH_LED_PIN_2};
            #if(TOTAL_LOADS > 0)
            const uint32_t gpio_load_pins[TOTAL_LOADS]                          = { HW_TOUCH_LED_PIN_2};
            #else
            const uint32_t gpio_load_pins[TOTAL_LOADS];
            #endif
            const uint32_t gpio_touch_btn_pins[TOTAL_BUTTONS]                   = { HW_TOUCH_LED_PIN_1 };
            const gpio_num_t gpio_ir_led_pins[TOTAL_LEDS]                       = { HW_TOUCH_LED_PIN_4 };
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
            uint16_t ENDPOINT_CLUSTERS[TOTAL_ENDPOINTS]                         = {ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL};
            #else
            uint16_t ENDPOINT_CLUSTERS[TOTAL_ENDPOINTS]                         = {ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT};
            #endif
        /*************SENSORS IAS ZONE************* */ 
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_MOTION || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_CONTACT_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_GAS_LEAK)
            const uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS]                       = {1};
            const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                      = {ESP_ZB_HA_IAS_ZONE_ID};
            #ifdef USE_HOME_ASSISTANT
                #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_MOTION)
                    char modelid[]                                              = {13, 'M', 'o', 't', 'i', 'o', 'n', ' ', 'S', 'e', 'n', 's', 'o', 'r'};
                    const char manufname[]                                      = {4, 'N', 'U', 'O', 'S'};
                #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_CONTACT_SWITCH)
                    char modelid[]                                              = {11, 'D', 'o', 'o', 'r', ' ', 'S', 'e', 'n', 's', 'o', 'r'};
                    const char manufname[]                                      = {4, 'N', 'U', 'O', 'S'};
                #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_GAS_LEAK)
                    char modelid[]                                              = {15, 'G', 'a', 's', ' ', 'L', 'e', 'a', 'k', ' ', 'S', 'e', 'n', 's', 'o', 'r'};
                    const char manufname[]                                      = {4, 'N', 'U', 'O', 'S'};
                #endif
            #else
                #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_MOTION)
                    // Tuya  m8ytq7be
                    char manufname[]                                            = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'm', '8', 'y', 't', 'q', '7', 'b', 'e'};
                    const char modelid[]                                        = {6, 'T', 'S', '0', '2', '0', '2'};
                #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_CONTACT_SWITCH) 
                    //Tuya  2qzlhsne 
                    char manufname[]                                            = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', '2', 'q', 'z', 'l', 'h', 's', 'n', 'e'};
                    const char modelid[]                                        = {6, 'T', 'S', '0', '2', '0', '2'};                                       
                #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_GAS_LEAK)
                    //Tuya  xkl3yq9f
                    char manufname[]                                            = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'x', 'k', 'l', '3', 'y', 'q', '9', 'f'};
                    const char modelid[]                                        = {6, 'T', 'S', '0', '2', '0', '2'};                                
                #endif   
            #endif
            gpio_num_t BUZZER_PIN                                               = GPIO_NUM_3; 
            gpio_num_t PIR_INPUT_PIN                                            = TOUCH_1_PIN;
            gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                          = { LOAD_4_PIN, LOAD_3_PIN};
            gpio_num_t gpio_load_pins[TOTAL_LOADS]                              = { LOAD_1_PIN}; 


            const uint32_t gpio_touch_btn_pins[TOTAL_BUTTONS]                   = { TOUCH_2_PIN};

            uint16_t ENDPOINT_CLUSTERS[TOTAL_ENDPOINTS]                         = {ESP_ZB_ZCL_CLUSTER_ID_IAS_ZONE};
            const unsigned int GAS_LEAK_SENSOR_PEAK_DETECTED_VALUE              = 1500;

        /*************SENSOR LUX************* */ 
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_LUX)
            const uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS]                       = {1};
            const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                      = {0x0106};
            #ifdef USE_HOME_ASSISTANT
                char modelid[]                                                  = {10, 'L', 'u', 'x', ' ', 'S', 'e', 'n', 's', 'o', 'r'};
                const char manufname[]                                          = {4, 'N', 'U', 'O', 'S'};
            #else
                //pisltm67
                //_TZE204_sxm7l9xa 
                //char manufname[]                                                = {16, '_', 'T', 'Z', 'E', '2', '0', '4', '_', 's', 'x', 'm', '7', 'l', '9', 'x', 'a'};
                //char manufname[]                                                = {16, '_', 'T', 'Y', 'S', 'T', '1', '1', '_', 'p', 'i', 's', 'l', 't', 'm', '6', '7'};
                //const char modelid[]                                            = {6, 'T', 'S', '0', '6', '0', '1'};
                //Tuya ikabtjsi
                char manufname[]                                                = {16, '_', 'T', 'Y', 'Z', 'B', '0', '1', '_', 'i', 'k', 'a', 'b', 't', 'j', 's', 'i'};
                const char modelid[]                                            = {6, 'T', 'S', '0', '2', '2', '2'};
                // char modelid[]                                                  = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'q', 't', 'n', 's', 'a', 'd', 'x', 'o'};
                // const char manufname[]                                          = {6, 'T', 'S', '0', '2', '0', '2'};                 
            #endif //qtnsadxo
            gpio_num_t BUZZER_PIN                                               = GPIO_NUM_3; 
            gpio_num_t PIR_INPUT_PIN                                            = TOUCH_1_PIN;
            gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                          = { LOAD_4_PIN, LOAD_3_PIN};
            gpio_num_t gpio_load_pins[TOTAL_LOADS]                              = { LOAD_1_PIN}; 


            const uint32_t gpio_touch_btn_pins[TOTAL_BUTTONS]                   = { TOUCH_2_PIN};

            // #if (CHIP_INFO == USE_ESP32C6)
            //     gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                      = { GPIO_NUM_4, GPIO_NUM_5};  //4=Blue, 5=Red
            //     gpio_num_t gpio_load_pins[TOTAL_LOADS]                          = { GPIO_NUM_1};
            //     gpio_num_t BUZZER_PIN                                           = GPIO_NUM_3;
            //     gpio_num_t PIR_INPUT_PIN                                        = GPIO_NUM_18; 
            //     gpio_num_t TEMP_HUMIDITY_PIN                                    = GPIO_NUM_0;
            // #elif (CHIP_INFO == USE_ESP32C6_MINI1)
            //     gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                      = { GPIO_NUM_7, GPIO_NUM_2};  //7=blue, NC=Red
            //     gpio_num_t gpio_load_pins[TOTAL_LOADS]                          = { GPIO_NUM_19};
            //     gpio_num_t BUZZER_PIN                                           = GPIO_NUM_3;
            //     gpio_num_t PIR_INPUT_PIN                                        = GPIO_NUM_4; 
            //     gpio_num_t TEMP_HUMIDITY_PIN                                    = GPIO_NUM_18;                           
            // #elif(CHIP_INFO == USE_ESP32H2)    
            //     gpio_num_t BUZZER_PIN                                           = GPIO_NUM_3;
            //     gpio_num_t PIR_INPUT_PIN                                        = GPIO_NUM_25;
            //     gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                      = { GPIO_NUM_12, GPIO_NUM_11};
            //     gpio_num_t gpio_load_pins[TOTAL_LOADS]                          = { GPIO_NUM_1};
            //     gpio_num_t TEMP_HUMIDITY_PIN                                    = GPIO_NUM_0;    
            // #elif(CHIP_INFO == USE_ESP32H2_MINI1)
            //     uint32_t BUZZER_PIN                                             = GPIO_NUM_3;
            //     uint32_t PIR_INPUT_PIN                                          = GPIO_NUM_0;
            //     gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                      = { GPIO_NUM_12, GPIO_NUM_11};
            //     gpio_num_t gpio_load_pins[TOTAL_LOADS]                          = { GPIO_NUM_25};
            //     uint32_t TEMP_HUMIDITY_PIN                                      = GPIO_NUM_22;              
            // #elif(CHIP_INFO == USE_ESP32H2_MINI1_V2)
            //     uint32_t BUZZER_PIN                                             = GPIO_NUM_3;
            //     uint32_t PIR_INPUT_PIN                                          = GPIO_NUM_0;
            //     gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                      = { GPIO_NUM_12, GPIO_NUM_11};
            //     gpio_num_t gpio_load_pins[TOTAL_LOADS]                          = { GPIO_NUM_25};
            //     uint32_t TEMP_HUMIDITY_PIN                                      = GPIO_NUM_22;              
            // #elif(CHIP_INFO == USE_ESP32H2_MINI1_V3)
            //     uint32_t BUZZER_PIN                                             = GPIO_NUM_3; //LOAD_5_PIN
            //     uint32_t PIR_INPUT_PIN                                          = GPIO_NUM_0; //TOUCH_1_PIN
            //     gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                      = { GPIO_NUM_12, GPIO_NUM_11}; //LOAD_4_PIN
            //     gpio_num_t gpio_load_pins[TOTAL_LOADS]                          = { GPIO_NUM_25};
            //     uint32_t TEMP_HUMIDITY_PIN                                      = GPIO_NUM_22;                                                                         
            // #endif       
            // const uint32_t gpio_touch_btn_pins[TOTAL_BUTTONS]                   = { HW_TOUCH_LED_PIN_1};     
            uint16_t ENDPOINT_CLUSTERS[TOTAL_ENDPOINTS]                         = {ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT}; 
        /*************TEMPERATURE************* */ 
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY)
            const uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS]                       = {1, 2, 3};
            const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                      = {ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID, ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID, ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID};
            #ifdef USE_HOME_ASSISTANT
                char modelid[]                                                  = {10, 'T', '&', 'H', ' ', 'S', 'e', 'n', 's', 'o', 'r'};
                const char manufname[]                                          = {4, 'N', 'U', 'O', 'S'};
            #else
                char modelid[]                                                  = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'x', 'j', '1', 'f', 'k', '2', 'v', 'v'};
                const char manufname[]                                          = {6, 'T', 'S', '0', '2', '0', '2'};                 
            #endif //   xj1fk2vv
            #if (CHIP_INFO == USE_ESP32C6)
                gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                      = {GPIO_NUM_4, GPIO_NUM_5};  //4=blue, 5=red
                gpio_num_t gpio_load_pins[TOTAL_LOADS]                          = {GPIO_NUM_1};
                gpio_num_t BUZZER_PIN                                           = GPIO_NUM_3;
                gpio_num_t PIR_INPUT_PIN                                        = GPIO_NUM_18; 
                gpio_num_t TEMP_HUMIDITY_PIN                                    = GPIO_NUM_0;
            #elif (CHIP_INFO == USE_ESP32C6_MINI1)
                gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                      = {GPIO_NUM_7, GPIO_NUM_2};  //7=blue, NC=red
                gpio_num_t gpio_load_pins[TOTAL_LOADS]                          = {GPIO_NUM_19};
                gpio_num_t BUZZER_PIN                                           = GPIO_NUM_3;
                gpio_num_t PIR_INPUT_PIN                                        = GPIO_NUM_4; 
                gpio_num_t TEMP_HUMIDITY_PIN                                    = GPIO_NUM_18;                           
            #elif(CHIP_INFO == USE_ESP32H2)    
                gpio_num_t BUZZER_PIN                                           = GPIO_NUM_3;
                gpio_num_t PIR_INPUT_PIN                                        = GPIO_NUM_25;
                gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                      = {GPIO_NUM_12, GPIO_NUM_11};
                gpio_num_t gpio_load_pins[TOTAL_LOADS]                          = {GPIO_NUM_1};
                gpio_num_t TEMP_HUMIDITY_PIN                                    = GPIO_NUM_0;    
            #elif(CHIP_INFO == USE_ESP32H2_MINI1)
                uint32_t BUZZER_PIN                                             = GPIO_NUM_3;
                uint32_t PIR_INPUT_PIN                                          = GPIO_NUM_0;
                gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                      = {GPIO_NUM_12, GPIO_NUM_11};
                gpio_num_t gpio_load_pins[TOTAL_LOADS]                          = {GPIO_NUM_25};
                uint32_t TEMP_HUMIDITY_PIN                                      = GPIO_NUM_22; 
            #elif(CHIP_INFO == USE_ESP32H2_MINI1_V2)  
                uint32_t BUZZER_PIN                                             = GPIO_NUM_3;
                uint32_t PIR_INPUT_PIN                                          = GPIO_NUM_0;
                gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                      = {GPIO_NUM_12, GPIO_NUM_11};
                gpio_num_t gpio_load_pins[TOTAL_LOADS]                          = {GPIO_NUM_25};
                uint32_t TEMP_HUMIDITY_PIN                                      = GPIO_NUM_22;                                              
            #endif
            const uint32_t gpio_touch_btn_pins[TOTAL_BUTTONS]                   = { HW_TOUCH_LED_PIN_1};
            uint16_t ENDPOINT_CLUSTERS[TOTAL_ENDPOINTS]                         = {ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG}; 

        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY_CUSTOM)
            const uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS]                       = {1};
            const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                      = {ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID};
            #ifdef USE_HOME_ASSISTANT
                char modelid[]                                                  = {10, 'T', '&', 'H', ' ', 'S', 'e', 'n', 's', 'o', 'r'};
                const char manufname[]                                          = {4, 'N', 'U', 'O', 'S'};
            #else
                //tuya elv1x5d0
                char modelid[]                                                  = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'e', 'l', 'v', '1', 'x', '5', 'd', '0'};
                //char modelid[]                                                  = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'x', 'j', '1', 'f', 'k', '2', 'v', 'v'};
                const char manufname[]                                          = {6, 'T', 'Y', '0', '2', '0', '1'};                 
            #endif //   xj1fk2vv
            #if (CHIP_INFO == USE_ESP32C6)
                gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                      = {GPIO_NUM_4, GPIO_NUM_5};  //4=blue, 5=red
                gpio_num_t gpio_load_pins[TOTAL_LOADS]                          = {GPIO_NUM_1};
                gpio_num_t BUZZER_PIN                                           = GPIO_NUM_3;
                gpio_num_t PIR_INPUT_PIN                                        = GPIO_NUM_18; 
                gpio_num_t TEMP_HUMIDITY_PIN                                    = GPIO_NUM_0;
            #elif (CHIP_INFO == USE_ESP32C6_MINI1)
                gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                      = {GPIO_NUM_7, GPIO_NUM_2};  //7=blue, NC=red
                gpio_num_t gpio_load_pins[TOTAL_LOADS]                          = {GPIO_NUM_19};
                gpio_num_t BUZZER_PIN                                           = GPIO_NUM_3;
                gpio_num_t PIR_INPUT_PIN                                        = GPIO_NUM_4; 
                gpio_num_t TEMP_HUMIDITY_PIN                                    = GPIO_NUM_18;                           
            #elif(CHIP_INFO == USE_ESP32H2)    
                gpio_num_t BUZZER_PIN                                           = GPIO_NUM_3;
                gpio_num_t PIR_INPUT_PIN                                        = GPIO_NUM_25;
                gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                      = {GPIO_NUM_12, GPIO_NUM_11};
                gpio_num_t gpio_load_pins[TOTAL_LOADS]                          = {GPIO_NUM_1};
                gpio_num_t TEMP_HUMIDITY_PIN                                    = GPIO_NUM_0;    
            #elif(CHIP_INFO == USE_ESP32H2_MINI1)
                uint32_t BUZZER_PIN                                             = GPIO_NUM_3;
                uint32_t PIR_INPUT_PIN                                          = GPIO_NUM_0;
                gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                      = {GPIO_NUM_12, GPIO_NUM_11};
                gpio_num_t gpio_load_pins[TOTAL_LOADS]                          = {GPIO_NUM_25};
                uint32_t TEMP_HUMIDITY_PIN                                      = GPIO_NUM_22; 
            #elif(CHIP_INFO == USE_ESP32H2_MINI1_V2)  
                uint32_t BUZZER_PIN                                             = GPIO_NUM_3;
                uint32_t PIR_INPUT_PIN                                          = GPIO_NUM_0;
                gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                      = {GPIO_NUM_12, GPIO_NUM_11};
                gpio_num_t gpio_load_pins[TOTAL_LOADS]                          = {GPIO_NUM_25};
                uint32_t TEMP_HUMIDITY_PIN                                      = GPIO_NUM_22;                                              
            #endif
            const uint32_t gpio_touch_btn_pins[TOTAL_BUTTONS]                   = { HW_TOUCH_LED_PIN_1};
            uint16_t ENDPOINT_CLUSTERS[3]                                       = {ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG};         
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_SCENE_SWITCH)
            const uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS]                       = { 1, 2, 3, 4 };
            
            #ifdef Mains_Powered_Scene_Switch
            const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                      = { ESP_ZB_HA_SCENE_SELECTOR_DEVICE_ID, ESP_ZB_HA_SCENE_SELECTOR_DEVICE_ID, 
                                                                                    ESP_ZB_HA_SCENE_SELECTOR_DEVICE_ID, ESP_ZB_HA_SCENE_SELECTOR_DEVICE_ID};  
            #else
            const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                      = { ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID, ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID, 
                                                                                    ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID, ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID};                                                                                
            #endif
            #ifdef USE_HOME_ASSISTANT
                 char manufname[]                                               = {16,'_', 'T', 'Z','3', '0','0', '0', '_', 'u', '3', 'a','p', 'v', 'x', 'a', 'y'};
                 const char modelid[]                                           = {6, 'T', 'S', '0', '0', '4', '4'};
            #else
                #ifdef Mains_Powered_Scene_Switch
                    //71z5chf6
                    //char manufname[]                                            = {16,'_', 'T', 'Z','3', '0','0', '2', '_', '7', '1', 'z','5', 'c', 'h', 'f', '6'};
                    char manufname[]                                            = {16,'_', 'T', 'Z','3', '0','0', '2', '_', 'u', 'n', 'y','g', '1', 'l', 'j', 'e'};
                    const char modelid[]                                        = {6, 'T', 'S', '0', '7', '2', '6'};
                #else
                    #ifdef USE_CUSTOM_SCENE     
                    //unyg1lje
                    char manufname[]                                            = {16,'_', 'T', 'Z','3', '0','0', '0', '_', 'u', 'n', 'y','g', '1', 'l', 'j', 'e'};
                    #else
                    //ur0dsft5 Tuya  
                    char manufname[]                                            = {16,'_', 'T', 'Z','3', '0','0', '0', '_', 'u', 'r', '0','d', 's', 'f', 't', '5'};
                    #endif
                    const char modelid[]                                        = {6, 'T', 'S', '0', '0', '4', '4'};
            
                #endif 
            #endif    
            const gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                    = { HW_TOUCH_LED_PIN_1, HW_TOUCH_LED_PIN_2, HW_TOUCH_LED_PIN_3, HW_TOUCH_LED_PIN_4};
            const gpio_num_t gpio_load_pins[TOTAL_LOADS]                        = { HW_LOAD_PIN_1, HW_LOAD_PIN_2, HW_LOAD_PIN_3, HW_LOAD_PIN_4};
            const gpio_num_t gpio_touch_btn_pins[TOTAL_BUTTONS]                 = { HW_TOUCH_BTN_PIN_1, HW_TOUCH_BTN_PIN_2, HW_TOUCH_BTN_PIN_3, HW_TOUCH_BTN_PIN_4}; 

            
            uint16_t ENDPOINT_CLUSTERS[TOTAL_ENDPOINTS]                         = {ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF};
        
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_GROUP_SWITCH)
            const uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS]                       = { 1, 2, 3, 4 };
            const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                      = { ESP_ZB_HA_SCENE_SELECTOR_DEVICE_ID, ESP_ZB_HA_SCENE_SELECTOR_DEVICE_ID, 
                                                                                    ESP_ZB_HA_SCENE_SELECTOR_DEVICE_ID, ESP_ZB_HA_SCENE_SELECTOR_DEVICE_ID};  

            //yqfmlhvz  (Single)
            char manufname[]                                                    = {16,'_', 'T', 'Z','3', '0','0', '2', '_', 'y', 'q', 'f', 'm', 'l', 'h', 'v', 'z'};    

            //vunmo6hb  (Double)
            //char manufname[]                                                    = {16,'_', 'T', 'Z','3', '0','0', '2', '_', 'v', 'u', 'n', 'm', 'o', '6', 'h', 'b'};
            const char modelid[]                                                = {6, 'T', 'S', '0', '7', '2', '6'};
    
            const gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                    = { HW_TOUCH_LED_PIN_1, HW_TOUCH_LED_PIN_2, HW_TOUCH_LED_PIN_3, HW_TOUCH_LED_PIN_4};
            const gpio_num_t gpio_load_pins[TOTAL_LOADS]                        = { HW_LOAD_PIN_1, HW_LOAD_PIN_2, HW_LOAD_PIN_3, HW_LOAD_PIN_4};
            const gpio_num_t gpio_touch_btn_pins[TOTAL_BUTTONS]                 = { HW_TOUCH_BTN_PIN_1, HW_TOUCH_BTN_PIN_2, HW_TOUCH_BTN_PIN_3, HW_TOUCH_BTN_PIN_4}; 

            
            uint16_t ENDPOINT_CLUSTERS[TOTAL_ENDPOINTS]                         = {ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF};
            /*************SCENE SWITCH************* */ 
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
            const uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS]                       = {1, 2, 3, 4};
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH)
            const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                      = {ESP_ZB_HA_SCENE_SELECTOR_DEVICE_ID, ESP_ZB_HA_SCENE_SELECTOR_DEVICE_ID, 
                                                                                    ESP_ZB_HA_SCENE_SELECTOR_DEVICE_ID, ESP_ZB_HA_SCENE_SELECTOR_DEVICE_ID};
            #elif (USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
            const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                      = {ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID, 
                                                                                    ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID};            
            #else
            const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                      = {ESP_ZB_HA_REMOTE_CONTROL_DEVICE_ID, ESP_ZB_HA_REMOTE_CONTROL_DEVICE_ID, 
                                                                                    ESP_ZB_HA_REMOTE_CONTROL_DEVICE_ID, ESP_ZB_HA_REMOTE_CONTROL_DEVICE_ID};
            #endif
            #ifdef USE_HOME_ASSISTANT                                                                 
                 char modelid[]                                                 = {12, 'S', 'c', 'e', 'n', 'e', ' ', 'S', 'w', 'i', 't', 'c', 'h'};
                 const char manufname[]                                         = {4, 'N', 'U', 'O', 'S'};
            #else 
            #if (USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
                //po0kwwya
                char manufname[]                                                = {16,'_', 'T', 'Z','3', '0','0', '0', '_', 'p', 'o', '0','k', 'w', 'w', 'y', 'a'};
                char modelid[]                                                  = {6, 'T','S', '0', '0', '1', '4'};              
            #else
                char manufname[]                                                = {16,'_', 'T', 'Z','3', '0','0', '0', '_', 'w', 'k', 'a','i', '4', 'g', 'a', '5'};
                char modelid[]                                                  = {6, 'T','S', '0', '0', '4', '4'};
            #endif
            #endif 
            const gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                    = { HW_TOUCH_LED_PIN_1, HW_TOUCH_LED_PIN_2, HW_TOUCH_LED_PIN_3, HW_TOUCH_LED_PIN_4};
            const gpio_num_t gpio_load_pins[TOTAL_LOADS]                        = { HW_LOAD_PIN_1, HW_LOAD_PIN_2, HW_LOAD_PIN_3, HW_LOAD_PIN_4};
            const gpio_num_t gpio_touch_btn_pins[TOTAL_BUTTONS]                 = { HW_TOUCH_BTN_PIN_1, HW_TOUCH_BTN_PIN_2, HW_TOUCH_BTN_PIN_3, HW_TOUCH_BTN_PIN_4}; 

            
            uint16_t ENDPOINT_CLUSTERS[TOTAL_ENDPOINTS]                         = {ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF};

        /*************CCT DALI************* */  
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI)
            const uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS]                       = {1, 2, 3, 4};
            const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                      = {ESP_ZB_HA_COLOR_DIMMABLE_LIGHT_DEVICE_ID, ESP_ZB_HA_COLOR_DIMMABLE_LIGHT_DEVICE_ID, 
                                                                                  ESP_ZB_HA_COLOR_DIMMABLE_LIGHT_DEVICE_ID, ESP_ZB_HA_COLOR_DIMMABLE_LIGHT_DEVICE_ID};

            #ifdef USE_HOME_ASSISTANT
                char manufname[]                                                = {4, 'N', 'U', 'O', 'S'};
                char modelid []                                                 = {14, 'D', 'A', 'L', 'I', ' ', '4', '-', 'T', 'u', 'n', 'a', 'b', 'l', 'e'};
            #else
                //Tuya sgzohm9w
                char manufname[]                                                = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 's', 'g', 'z', 'o', 'h', 'm', '9', 'w'};
                // TS0505B
                char modelid[]                                                  = {7, 'T', 'S', '0', '5', '0', '4', 'B'};                    
            #endif


            const gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                    = { HW_TOUCH_LED_PIN_1, HW_TOUCH_LED_PIN_2, HW_TOUCH_LED_PIN_3, HW_TOUCH_LED_PIN_4};
            const gpio_num_t gpio_load_pins[TOTAL_LOADS]                        = { DALI_RX_PIN, DALI_TX_PIN};
            const gpio_num_t gpio_touch_btn_pins[TOTAL_BUTTONS]                 = { HW_TOUCH_BTN_PIN_1, HW_TOUCH_BTN_PIN_2, HW_TOUCH_BTN_PIN_3, HW_TOUCH_BTN_PIN_4}; 

        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
            const uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS]                       = {1};
            #ifdef USE_HOME_ASSISTANT
                const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                  = {ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID};
                char manufname[]                                                = {4, 'N', 'U', 'O', 'S'};
                char modelid []                                                 = {14, 'D', 'A', 'L', 'I', ' ', 'C', 'C', 'T', ' ', 'L', 'i', 'g', 'h', 't'};
            #else
                const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                  = {0x010C};
                //Tuya sgzohm9w
                char manufname[]                                                = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 's', 'g', 'z', 'o', 'h', 'm', '9', 'w'};
                // TS0505B
                char modelid[]                                                  = {7, 'T', 'S', '0', '5', '0', '4', 'B'};     
            #endif

            const gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                    = { HW_TOUCH_LED_PIN_1, HW_TOUCH_LED_PIN_3, HW_TOUCH_LED_PIN_2, HW_TOUCH_LED_PIN_4};
            const gpio_num_t gpio_load_pins[TOTAL_LOADS]                        = { DALI_RX_PIN, DALI_TX_PIN};
            const gpio_num_t gpio_touch_btn_pins[TOTAL_BUTTONS]                 = { HW_TOUCH_BTN_PIN_1, HW_TOUCH_BTN_PIN_3, HW_TOUCH_BTN_PIN_2, HW_TOUCH_BTN_PIN_4}; 
           
        /*************GROUP DALI************* */  
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI)
            #ifdef USE_HOME_ASSISTANT
                #ifdef USE_INDIVIDUAL_DALI_ADDRESSING
                const uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS]                       = {1, 2 ,3, 4 , 5, 6, 7, 8, 9, 10, 11, 12 ,13, 14 , 15, 16, 17, 18, 19, 20,
                                                                                    21, 22 ,23, 24 , 25, 26, 27, 28, 29, 30, 31, 32  };            
                const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                      = { ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID, ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID, 
                                                                                        ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID, ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID,
                                                                                        ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID, ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID,
                                                                                        ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID, ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID,
                                                                                        ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID, ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID,
                                                                                        ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID, ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID, 
                                                                                        ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID, ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID,
                                                                                        ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID, ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID,
                                                                                        ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID, ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID,
                                                                                        ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID, ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID,
                                                                                        ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID, ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID, 
                                                                                        ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID, ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID,
                                                                                        ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID, ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID,
                                                                                        ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID, ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID,
                                                                                        ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID, ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID,
                                                                                        ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID, ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID
                                                                                    };
                #else 
                    const uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS]                       = { 1, 2 ,3, 4 };                                                                       
                    const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                      = {ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID, ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID, 
                                                                                        ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID, ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID};
                #endif  
            #else //if tuya
                const uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS]                       = { 1, 2 ,3, 4 };                                                                       
                const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                      = {ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID, 
                                                                                    ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID};
            #endif
                                                                     

            #ifdef USE_HOME_ASSISTANT
                char manufname[]                                                = {4, 'N', 'U', 'O', 'S'};
                char modelid []                                                 = {12, 'D', 'A', 'L', 'I', ' ', '4', '-', 'G', 'r', 'o', 'u', 'p'};
            #else
                #ifdef USE_TUYA_BRDIGE 
                //umaawtpc

                //char manufname[]                                            = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'u', 'm', 'a', 'a', 'w', 't', 'p', 'c'};
                //ydfvmon7   latest
                    char manufname[]                                            = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'y', 'd', 'f', 'v', 'm', 'o', 'n', '7'};
                    ////char manufname[]                                            = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'j', 'f', 'j', 's', 's', '8', 'r', 'l'};
                    char modelid[]                                              = {6,'T', 'S', '0', '0', '1', '4'};
                #endif
            #endif

            const gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                    = { HW_TOUCH_LED_PIN_1, HW_TOUCH_LED_PIN_2, HW_TOUCH_LED_PIN_3, HW_TOUCH_LED_PIN_4};
            const gpio_num_t gpio_load_pins[TOTAL_LOADS]                        = { DALI_RX_PIN, DALI_TX_PIN};
            const gpio_num_t gpio_touch_btn_pins[TOTAL_BUTTONS]                 = { HW_TOUCH_BTN_PIN_1, HW_TOUCH_BTN_PIN_2, HW_TOUCH_BTN_PIN_3, HW_TOUCH_BTN_PIN_4}; 

        /***********RGB DALI************* */  
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX  || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_CUSTOM)

            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_CUSTOM)
                const uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS]                   = {1, 2, 3};
                const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                  = {ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID, ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID, ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID};
            #else
                const uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS]                   = {1};

                #if(USE_COLOR_DEVICE == COLOR_RGB_ONLY)
                    const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                  = {0x0102 /*COLOR_LIGHT*/};
                #elif(USE_COLOR_DEVICE == COLOR_RGBW)
                    const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                  = {0x010D /*COLOR_LIGHT*/};
                #elif(USE_COLOR_DEVICE == COLOR_RGB_CW_WW)
                    const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                  = {0x010D /*EXTENDED_COLOR_LIGHT*/};                                                   
                #endif      
                
            #endif

            #ifdef USE_HOME_ASSISTANT
                char manufname[]                                                    = {4, 'N', 'U', 'O', 'S'};
                char modelid []                                                     = {10, 'D', 'A', 'L', 'I', ' ', '1', '-', 'R', 'G', 'B'};
            #else
                #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_CUSTOM)
                    char manufname[]                                                = {16, '_', 'T', 'Z', 'E', '2', '0', '0', '_', '6', 'o', 'a', 'r', 'u', 'g', 's', 'x'};
                    char modelid[]                                                  = {6, 'T', 'S', '0', '6', '0', '1'}; 
                #else
                    #if(USE_COLOR_DEVICE == COLOR_RGB_ONLY)
                    //r7xbuwbw
                    char manufname[]                                                = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'r', '7', 'x', 'b', 'u', 'w', 'b', 'w'};
                    //TS0503B
                    char modelid[]                                                  = {7, 'T', 'S', '0', '5', '0', '3', 'B'};
                    #elif(USE_COLOR_DEVICE == COLOR_RGBW)
                    char manufname[]                                                = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'w', 'w', '5', 'y', 'e', 'n', 'a', '0'};
                    //TS0504B
                    char modelid[]                                                  = {7, 'T', 'S', '0', '5', '0', '4', 'B'};
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
                    char modelid[]                                                  = {7, 'T', 'S', '0', '5', '0', '5', 'B'}; 

                    //s4zadi21
                    char manufname[]                                                = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 's', '4', 'z', 'a', 'd', 'i', '2', '1'};
                    // ww5yena0
                    //char manufname[]                                                = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'w', 'w', '5', 'y', 'e', 'n', 'a', '0'};

                    #endif
                
                #endif

            #endif

            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
                //Using 1-Light 1-Fan Touch
                const gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                        = { HW_TOUCH_LED_PIN_1, HW_TOUCH_LED_PIN_2, HW_TOUCH_LED_PIN_3, HW_TOUCH_LED_PIN_4};
                const gpio_num_t gpio_load_pins[TOTAL_LOADS]                            = { DALI_TX_PIN };
                const gpio_num_t gpio_touch_btn_pins[TOTAL_BUTTONS]                     = { HW_TOUCH_BTN_PIN_1, HW_TOUCH_BTN_PIN_2, HW_TOUCH_BTN_PIN_3, HW_TOUCH_BTN_PIN_4}; 
            #else
                //Using Normal 4-On-ff Touch
                const gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                        = { HW_TOUCH_LED_PIN_1, HW_TOUCH_LED_PIN_2, HW_TOUCH_LED_PIN_3, HW_TOUCH_LED_PIN_4};
                const gpio_num_t gpio_load_pins[TOTAL_LOADS]                            = { DALI_RX_PIN, DALI_TX_PIN};
                const gpio_num_t gpio_touch_btn_pins[TOTAL_BUTTONS]                     = { HW_TOUCH_BTN_PIN_1, HW_TOUCH_BTN_PIN_2, HW_TOUCH_BTN_PIN_3, HW_TOUCH_BTN_PIN_4}; 
            #endif

        /*************SCENE SWITCH************* */ 
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RINGING_BELL_2)
            const uint8_t ENDPOINTS_LIST[TOTAL_ENDPOINTS]                       = {1};
            const uint32_t ENDPOINTS_TYPE[TOTAL_ENDPOINTS]                      = {ESP_ZB_HA_IAS_ZONE_ID};
            #ifdef USE_HOME_ASSISTANT
                char modelid[]                                                  = {9, 'D', 'o', 'o', 'r', ' ', 'B', 'e', 'l', 'l'};
                const char manufname[]                                          = {4, 'N', 'U', 'O', 'S'};
            #else
                //Tuya  2qzlhsne 
                char manufname[]                                                = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', '2', 'q', 'z', 'l', 'h', 's', 'n', 'e'};
                const char modelid[]                                            = {7, 'T', 'S', '0', '2', '1', '5', 'A'};  //TS0215A                                     
            #endif
            const gpio_num_t gpio_touch_led_pins[TOTAL_LEDS]                    = { HW_TOUCH_LED_PIN_1, HW_TOUCH_LED_PIN_2};
            const gpio_num_t gpio_load_pins[TOTAL_LOADS]                        = { HW_LOAD_PIN_1}; 
            const gpio_num_t gpio_touch_btn_pins[TOTAL_BUTTONS]                 = { HW_TOUCH_BTN_PIN_1, HW_TOUCH_BTN_PIN_2}; 

            uint16_t ENDPOINT_CLUSTERS[TOTAL_ENDPOINTS]                         = {ESP_ZB_ZCL_CLUSTER_ID_IAS_ZONE};
        #endif
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN_SWITCH)
        bool curtain_cal_mode_active_flag                                       = false;
        uint8_t cal_task_mode                                                  = 0;
        bool is_bt_start_pressed                                                = false;
        bool is_bt_end_pressed                                                  = false;
        #endif

        bool bCalMode = false;
        uint64_t start_time = 0;
        uint64_t end_time = 0;
        uint8_t vTaskMode = 0;

    #else
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN_SWITCH)
        extern bool curtain_cal_mode_active_flag;
        extern uint8_t cal_task_mode;
        extern bool is_bt_start_pressed;
        extern bool is_bt_end_pressed;
        #endif
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
        extern scene_switch_s scene_switch_t[4];
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
        extern uint16_t global_dali_id[4]; 
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI)
        extern uint16_t global_dali_id[4]; 
        #else
        extern uint16_t global_dali_id[32]; 
        #endif
        extern uint8_t global_scene_id[4];  
        extern bool change_cw_ww_color_flag;     
        extern stt_scene_switch_t existing_nodes_info[4];
        extern stt_scene_switch_t nodes_info;        
        extern zigbee_zcene_info_t zb_scene_info[4];
        extern wifi_info_handle_t wifi_info;
        extern bool timer3_running_flag;
        
        extern uint8_t target_percentage; 
        extern bool is_long_press_brightness;
        extern uint8_t selected_color_mode;
        extern uint8_t last_selected_color_mode;
        extern bool mode_change_flag;
        extern int64_t curtain_cal_time;
        extern bool double_press_click_enable;
        extern uint16_t disable_double_press_enable_counts;

        extern uint16_t cb_requests_counts;
        extern uint16_t cb_response_counts;
        #ifdef USE_CCT_TIME_SYNC
        extern bool set_cct_time_sync_request_flag;
        #endif
        extern bool is_reporting[MAX_DST_EP][MAX_NODES];

        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI)
            #ifdef USE_INDIVIDUAL_DALI_ADDRESSING
            extern led_indicator_handle_t device_info[TOTAL_ENDPOINTS+1];
            #else
            extern led_indicator_handle_t device_info[TOTAL_BUTTONS+1];
            #endif
        #else
            extern led_indicator_handle_t device_info[TOTAL_BUTTONS+1];
        #endif

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

        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY_CUSTOM)
            extern uint16_t ENDPOINT_CLUSTERS[3];
        #else
            extern uint16_t ENDPOINT_CLUSTERS[TOTAL_ENDPOINTS];
        #endif
        extern bool light_driver_deinit_flag;
    
        extern uint8_t wifi_webserver_active_flag;
        extern uint8_t global_switch_state;
        extern uint32_t wifi_webserver_active_counts;
        extern bool start_commissioning;
        extern bool ready_commisioning_flag;
        extern bool is_my_device_commissionned;
        extern bool find_active_neighbours_task_flag;
        extern uint8_t zb_network_bdb_steering_count;

        extern bool bCalMode;
        extern uint64_t start_time;
        extern uint64_t end_time;
        extern uint8_t vTaskMode;

    #endif     
//app_hardware_driver_on_off.c  
#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
    #ifdef __cplusplus
    extern "C" {
    #endif
        extern void nuos_init_ir_ac(uint8_t index);
        extern void set_decode_type(int decode_type);
        extern void nuos_try_ac(int decode_type, bool state);
        extern void nuos_set_hardware_fan_ctrl(uint8_t index);
        extern void nuos_zb_set_hardware_led(uint8_t index, uint8_t is_toggle);
    #ifdef __cplusplus
    }
    #endif
#elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_MOTION || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_CONTACT_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_GAS_LEAK || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_LUX)    
    #ifdef __cplusplus
    extern "C" {
    #endif
        extern void nuos_esp_motion_sensor_check_in_timer();
        extern void init_sensor_interrupt();
        extern void nuos_set_hardware_fan_ctrl(uint8_t index);
    #ifdef __cplusplus
    }
    #endif
#else
    extern void nuos_set_hardware_fan_ctrl(uint8_t index); 
    extern void init_sensor_interrupt();
#endif

#ifdef __cplusplus
extern "C" {
#endif
    void set_state(uint8_t index);
    void set_level(uint8_t index);
    void set_color_temp();
    void set_color_temp_args(uint16_t color);
    esp_err_t nuos_set_state_attribute(uint8_t index);
    esp_err_t nuos_set_state_attribute_rgb(uint8_t index);
    extern void nuos_zb_init_hardware();
    extern void nuos_zb_set_hardware(uint8_t index, uint8_t value);
    extern void nuos_zb_set_hardware_led_for_zb_commissioning(uint8_t is_toggle);
    extern bool nuos_set_hardware_brightness(uint32_t pin);
    extern void nuos_init_hardware_dimming_up_down(uint32_t pin);
    extern void init_dali_hw();
    extern void nuos_dali_add_light_to_group(uint8_t addr, uint8_t group_id);
    extern void nuos_dali_remove_light_from_group(uint8_t addr, uint8_t group_id);
    extern void nuos_dali_toggle_group(uint8_t group_id, uint8_t index, bool toggle_state, uint8_t brightness);
    extern void start_dali_addressing(int numAddresses);
    extern void nuos_dali_set_color_temperate(uint8_t index);
    extern void nuos_dali_set_group_color_temperature(uint8_t group_id, uint8_t index, uint16_t value);
    extern void nuos_dali_set_group_brightness(uint8_t group_id, uint8_t index, uint8_t value);
    extern void nuos_set_state_touch_leds(bool state);
    extern void nuos_set_state_touch_leds_to_original();
    extern void nuos_toggle_leds(uint8_t index);
    extern void nuos_on_off_led(uint8_t index, uint8_t _level);
    extern void set_level_value(uint8_t _level);
    extern uart_word_length_t string_to_uart_word_length_t(const char *data_bits_str);
    extern uart_stop_bits_t string_to_uart_stop_bits_t(const char *stop_bits_str);
    extern uart_parity_t string_to_uart_parity_t(const char *parity_str);
    // extern uart_hw_flowcontrol_t string_to_uart_hw_flowcontrol_t(const char *flow_ctrl_str);
    extern void nuos_set_hw_brightness(uint8_t index);
    extern void init_sensor_commissioning_task();
    extern bool nuos_set_hardware_brightness_2(uint8_t index);
    extern void dali_set_mode(uint8_t mode);

#ifdef __cplusplus
}
#endif 
extern void init_fading();
#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN_SWITCH) 
extern void init_curtain_timer();
extern void pause_curtain();
extern void resume_curtain(uint8_t from, uint8_t to);

extern void nuos_mid_stop_curtain();
extern void nuos_open_curtain();
extern void nuos_close_curtain();
extern void start_calibration();
extern void stop_calibration();
extern void start_curtain_movement(uint8_t start_percent, uint8_t target_percent);
#endif
extern void nuos_on_off_led(uint8_t index, uint8_t _state);
extern void nuos_zb_set_hardware_curtain(uint8_t index, uint8_t is_toggle);
extern uint8_t nuos_get_button_press_index(uint32_t pin);
extern void xy_to_rgb(unsigned int x, unsigned int y, double *r, double *g, double *b);
extern void nuos_set_level_to_rgb(uint8_t index);
extern void nuos_zb_set_scene_switch_click(uint32_t io_num, uint8_t state);
extern void nuos_zb_convert_xy_to_rgb(uint8_t index, float red_f, float blue_f, float green_f);

extern void pause_curtain_timer();
extern void resume_curtain_timer(uint8_t index);
extern void curtain_cmd_open(void);
extern void curtain_cmd_close(void);
extern void curtain_cmd_stop(void);
extern int curtain_cmd_goto_pct(uint8_t pct) ;
