/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: LicenseRef-Included
 *
 * Zigbee HA_on_off_light Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */


#include <stdio.h>
#include <time.h>
#include "esp_timer.h"
#include "app_config.h"
#include "esp_system.h"
#include "esp_zigbee_core.h"
#include "zdo/esp_zigbee_zdo_command.h"
#include "esp_zb_controller_main.h"
#include "switch_driver.h"
#include "app_hardware_driver.h"
#include "app_zigbee_clusters.h"
#include "app_nuos_timer.h"
#include "zcl/esp_zigbee_zcl_power_config.h"
#ifdef USE_OTA
#include "esp_ota_ops.h"
#endif
#include "esp_wifi_webserver.h"
#include "app_zigbee_group_commands.h"
#include "app_zigbee_scene_commands.h"
//#include "esp_wifi.h"
// #include "zcl/zb_zcl_common.h"
#include <unistd.h> // For usleep in Linux (or vTaskDelay for ESP32)
#include <esp_zigbee_type.h>
#include "zigbee_2_uart.h"
static const char *TAG = "ZB_CLUSTERS";

extern void nuos_set_scene(esp_zb_zcl_recall_scene_message_t *message);
extern uint8_t curtain_state;
extern uint8_t dmx_data[10];
extern uint8_t dmx_start_address;
extern int max_of_three(int a, int b, int c);

#define ARRAY_LENTH(arr)                            (sizeof(arr) / sizeof(arr[0]))
#define TOTAL_BIND_REQUESTS                         (TOTAL_ENDPOINTS * 2) // For Level and On/Off
#define TIMER_PERIOD_MS                             500

uint8_t esp_zb_zcl_group_role                       = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
uint8_t esp_zb_zcl_scene_role                       = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
uint8_t esp_zb_zcl_identify_role                    = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
bool state_change_flag                              = false;
static bool last_state[TOTAL_ENDPOINTS]             = {0};
// int cnts                                            = 0;
// bool only_one_time                                  = true;
// bool call_one_time                                  = false;
uint8_t endpoint_counts                             = 0;
static int current_index                            = 0;
uint32_t timer_counter                              = 0;
// TimerHandle_t timer_handle                          = NULL;

// static QueueHandle_t timer_evt_queue                = NULL;
// volatile bool send_active_request_flag              = false;
volatile int16_t boot_pin_toggle_counts             = 0;
bool is_value_present(uint8_t arr[], uint8_t size, uint8_t value);
void bind_cb1(esp_zb_zdp_status_t zdo_status, void *user_ctx);
void bind_level_cb1(esp_zb_zdp_status_t zdo_status, void *user_ctx);

//extern void switch_driver_gpios_intr_enabled(bool enabled);

typedef struct {
    uint8_t r; // Red
    uint8_t g; // Green
    uint8_t b; // Blue
} Color;
// uint8_t last_selected_mode = 0xff;
bool change_state_flag = false;
#ifdef USE_QUEUE_CTRL 
    static QueueHandle_t attr_evt_queue                         = NULL;
#endif
static QueueHandle_t privilege_evt_queue                        = NULL;
#ifdef USE_SCENE_RECALL_QUEUE
    static QueueHandle_t scene_evt_queue                        = NULL;
#endif

#ifdef USE_SCENE_STORE_QUEUE
    static QueueHandle_t scene_store_evt_queue                  = NULL;
#endif


uint16_t cb_counts = 0;
uint8_t device_level[TOTAL_LEDS+1] = {0,0,0,0,0};
uint16_t device_cct_color[TOTAL_LEDS+1] = {0,0,0,0,0};
bool exists = false;

uint8_t color_level[2] = {0, 0};
uint32_t color_counts = 1;
uint8_t last_hue_val = 0;
uint16_t color_x, color_y;




uint8_t map_0_1000_to_0_255(uint16_t x)
{
    return (x * 255) / 1000;
}

uint16_t mapValue(uint8_t value) {
    return (value * 1000) / 255;
}

unsigned int map_1000(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

typedef struct {
    uint8_t bytes[6]; // 48 bits = 6 bytes
} uint48_t;

typedef struct {
    uint8_t bytes[9]; // 72 bits = 9 bytes
} uint72_t;

typedef struct {
    uint8_t bytes[11]; // 88 bits = 11 bytes
} uint88_t;


rgb_t hsv_to_rgb(hsv_t hsv) {
    rgb_t rgb;
    float h = hsv.h;                      // Hue is already in [0, 360]
    #ifdef USE_TUYA_BRDIGE
    float s = hsv.s / 1000.0;             // Scale saturation to [0, 1]
    float v = hsv.v / 1000.0;             // Scale value to [0, 1]
    #else
    float s = hsv.s;                      // Scale saturation to [0, 1]
    float v = hsv.v;                      // Scale value to [0, 1]
    #endif
    

    float c = v * s;                      // Chroma
    float x = c * (1 - fabs(fmod(h / 60.0, 2) - 1));  // Intermediate value
    float m = v - c;

    float r_prime, g_prime, b_prime;

    if (h >= 0 && h < 60) {
        r_prime = c; g_prime = x; b_prime = 0;
    } else if (h >= 60 && h < 120) {
        r_prime = x; g_prime = c; b_prime = 0;
    } else if (h >= 120 && h < 180) {
        r_prime = 0; g_prime = c; b_prime = x;
    } else if (h >= 180 && h < 240) {
        r_prime = 0; g_prime = x; b_prime = c;
    } else if (h >= 240 && h < 300) {
        r_prime = x; g_prime = 0; b_prime = c;
    } else {
        r_prime = c; g_prime = 0; b_prime = x;
    }

    // Convert to RGB by adding m and scaling to [0, 254]
    rgb.r = (uint8_t)((r_prime + m) * 254);
    rgb.g = (uint8_t)((g_prime + m) * 254);
    rgb.b = (uint8_t)((b_prime + m) * 254);

    return rgb;
}

hsv_t rgb_to_hsv(rgb_t rgb) {
    float rf = rgb.r / 254.0;
    float gf = rgb.g / 254.0;
    float bf = rgb.b / 254.0;

    float mx = fmaxf(fmaxf(rf, gf), bf);
    float mn = fminf(fminf(rf, gf), bf);

    hsv_t hsv;
    hsv.v = mx * 1000;
    
    float df = mx - mn;
    if (mx == mn) {
        hsv.h = 0;
        hsv.s = 0;
    } else {
        hsv.s = df / mx * 1000;

        float h;
        if (rf == mx) {
            h = (gf - bf) / df;
        } else if (gf == mx) {
            h = 2 + (bf - rf) / df;
        } else {
            h = 4 + (rf - gf) / df;
        }

        h *= 60;
        if (h < 0) {
            h += 360;
        }
        hsv.h = (uint16_t)(h * 0x0168 / 360);
    }
    return hsv;
}



// Define the APS request structure for sending a message to the end device
esp_zb_apsde_data_req_t aps_req = {
    .dst_addr_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,  // Use short addressing
    .dst_addr = {
        .addr_short = 0x0,  // Replace with the end device's short address
    },
    .dst_endpoint = 1,          // Endpoint on the end device
    .profile_id = ESP_ZB_AF_HA_PROFILE_ID,  // Home Automation Profile
    .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_BASIC,  // Basic Cluster (for testing)
    .src_endpoint = 1,          // Source endpoint on the coordinator
    .tx_options = ESP_ZB_APSDE_TX_OPT_ACK_TX,  // Enable APS ACK
    .radius = 0,                // Use default network radius
    .asdu = NULL,               // Replace with actual payload if needed
    .asdu_length = 0,           // Payload length
};

// Function to send a message and check for ACK
void send_ping_to_device() {
    esp_zb_aps_data_request(&aps_req);
    //esp_zb_apsde_data_indication_callback_t
    //esp_zb_aps_data_indication_handler_register  
}

uint8_t map_color_value(uint8_t x) {
    return (x * 2) / 3;  // Applying y = (2/3) * x
}
unsigned int map_cct(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

extern void send_on_to_xbee();


static esp_err_t zb_switch_send_recall_scene_to_light(uint8_t ep, uint16_t group_id, uint8_t scene_id)
{
    esp_zb_zcl_scenes_recall_scene_cmd_t recall_scene_cmd = {
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_GROUP_ENDP_NOT_PRESENT,
        .zcl_basic_cmd.dst_addr_u.addr_short = group_id,
        .zcl_basic_cmd.src_endpoint = ep,
        .group_id = group_id,
        .scene_id = scene_id,
    };

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_scenes_recall_scene_cmd_req(&recall_scene_cmd);
    esp_zb_lock_release();

    ESP_LOGI(TAG, "Ask the short: 0x%04x to recall Scene ID: %d of Group ID: 0x%04x",
             recall_scene_cmd.zcl_basic_cmd.dst_addr_u.addr_short, recall_scene_cmd.scene_id,
             recall_scene_cmd.group_id);
    return ESP_OK;
}


void recall_scenes_task(void* args) {
    uint8_t index = *(uint8_t*)args;
    uint16_t my_short_addr = esp_zb_get_short_address();
    for(int node_index=0; node_index<existing_nodes_info[index].scene_switch_info.total_records; node_index++){
        uint16_t dst_addr = existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].short_addr;

        if(dst_addr == my_short_addr || dst_addr == 0){

        }else{
            
            for(int ep_index = 0; ep_index <existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].endpoint_counts; ep_index++){
                uint8_t dst_ep = existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].dst_ep;
                if(existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].is_bind == 1){
                    vTaskDelay(200 / portTICK_PERIOD_MS);
                    printf("short_addr:0x%x dst_ep=%d  bind_status=%d\n", dst_addr, dst_ep,
                    existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].is_bind);
                    
                    for(uint8_t cluster_index=0; cluster_index < existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].clusters_count; cluster_index++){
                        if(existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].cluster_id[cluster_index] == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF){
                            esp_zb_zcl_on_off_cmd_t cmd_req = {0};
                            cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = dst_addr;
                            cmd_req.zcl_basic_cmd.dst_endpoint = dst_ep;
                            cmd_req.zcl_basic_cmd.src_endpoint = existing_nodes_info[index].scene_switch_info.src_ep;
                            cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;

                            if(existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].data.state)
                                cmd_req.on_off_cmd_id = ESP_ZB_ZCL_CMD_ON_OFF_ON_ID;
                            else
                                cmd_req.on_off_cmd_id = ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID;
                            esp_zb_lock_acquire(portMAX_DELAY);
                            esp_zb_zcl_on_off_cmd_req(&cmd_req);
                            esp_zb_lock_release(); 

                        } else if(existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].cluster_id[cluster_index] == ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL){
                            if(existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].data.state){

                                esp_zb_zcl_move_to_level_cmd_t cmd_level = {0};
                                cmd_level.zcl_basic_cmd.dst_addr_u.addr_short = dst_addr;
                                cmd_level.zcl_basic_cmd.dst_endpoint = dst_ep;
                                cmd_level.zcl_basic_cmd.src_endpoint = existing_nodes_info[index].scene_switch_info.src_ep;
                                cmd_level.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
                                cmd_level.level = existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].data.level;
                                cmd_level.transition_time = 0;
                                printf("LEVEL:%d\n", cmd_level.level);
                                if(existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].cluster_id[cluster_index+1] == ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL){
                                    uint8_t mode = existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].data.mode;
                                    if(mode == 2 || mode == 0){
                                        esp_zb_lock_acquire(portMAX_DELAY);
                                        esp_zb_zcl_level_move_to_level_with_onoff_cmd_req(&cmd_level);
                                        esp_zb_lock_release();
                                    }
                                }else{
                                    esp_zb_lock_acquire(portMAX_DELAY);
                                    esp_zb_zcl_level_move_to_level_with_onoff_cmd_req(&cmd_level);
                                    esp_zb_lock_release();
                                }
                            }                         
                        } else if(existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].cluster_id[cluster_index] == ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL){
                            if(existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].data.state){
                                printf("COLOR_MODE:%d\n", existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].data.mode); 
                                uint8_t mode = existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].data.mode;
                                if(mode == 0 || mode == 2){
                                    //uint16_t mapValue = map_cct(existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].data.value, MIN_CCT_VALUE, MAX_CCT_VALUE, 158, 500);
                                    printf("COLOR TEMP: %ld\n", existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].data.value);
                                    esp_zb_zcl_color_move_to_color_temperature_cmd_t cmd_req = {0};
                                    cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
                                    cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = dst_addr;
                                    cmd_req.zcl_basic_cmd.dst_endpoint = dst_ep;
                                    cmd_req.zcl_basic_cmd.src_endpoint = existing_nodes_info[index].scene_switch_info.src_ep;
                                    cmd_req.color_temperature = existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].data.value;
                                    cmd_req.transition_time = 0x0000;
                                    esp_zb_lock_acquire(portMAX_DELAY);
                                    esp_zb_zcl_color_move_to_color_temperature_cmd_req(&cmd_req);
                                    esp_zb_lock_release();  
                                }else if(mode == 1){
                                    printf("COLOR VAL: 0x%X\n", map_color_value(existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].data.value));
                                    esp_zb_zcl_color_move_to_hue_cmd_t cmd2 = {0};
                                    cmd2.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
                                    cmd2.zcl_basic_cmd.dst_addr_u.addr_short = dst_addr;
                                    cmd2.zcl_basic_cmd.dst_endpoint = dst_ep;
                                    cmd2.zcl_basic_cmd.src_endpoint = existing_nodes_info[index].scene_switch_info.src_ep;
                                    cmd2.hue = existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].data.value;
                                    cmd2.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV;
                                    cmd2.transition_time = 0;
                                    esp_zb_lock_acquire(portMAX_DELAY);
                                    esp_zb_zcl_color_move_to_hue_cmd_req(&cmd2);
                                    esp_zb_lock_release();
                                }
                            }
                        }else if(existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].cluster_id[cluster_index] == ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL){
                            /* configure report attribute command */
                            if(!is_reporting[index][node_index]){
                                is_reporting[index][node_index] = true;

                                int16_t report_change = 1; // Report on each change 
                                esp_zb_zcl_config_report_cmd_t report_cmd = {0};
                                report_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
                                report_cmd.zcl_basic_cmd.dst_addr_u.addr_short = existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].short_addr;  // Short address of Server B
                                report_cmd.zcl_basic_cmd.dst_endpoint = existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].dst_ep;
                                report_cmd.zcl_basic_cmd.src_endpoint = existing_nodes_info[index].scene_switch_info.src_ep;                                
                                report_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL;
                                
                                esp_zb_zcl_config_report_record_t records[] = {
                                    {
                                        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
                                        .attributeID = ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID,
                                        .attrType = ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM,
                                        .min_interval = 1,
                                        .max_interval = 0,
                                        .reportable_change = &report_change,
                                    },        
                                };
                                report_cmd.record_number = ARRAY_LENTH(records);
                                report_cmd.record_field = records;
                                esp_zb_zcl_config_report_cmd_req(&report_cmd);
                            }
                            
                            esp_zb_zcl_attribute_t attr_field1;
                            attr_field1.id = ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID;
                            attr_field1.data.type = ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM;
                            attr_field1.data.size = 1;
                            attr_field1.data.value = (uint8_t*)&existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].data.level;

                            esp_zb_zcl_write_attr_cmd_t write_req1;
                            memset(&write_req1, 0, sizeof(write_req1));

                            write_req1.zcl_basic_cmd.dst_addr_u.addr_short = existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].short_addr;
                            write_req1.zcl_basic_cmd.dst_endpoint = existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].dst_ep; // Target endpoint on the device
                            write_req1.zcl_basic_cmd.src_endpoint = existing_nodes_info[index].scene_switch_info.src_ep; // Source endpoint (set to 1 or 0 depending on your setup)
                            write_req1.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
                            write_req1.clusterID = ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL;
                            write_req1.attr_field = &attr_field1;
                            write_req1.attr_number = 1;
                            write_req1.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV;
                            // write_req1.dis_default_resp = 1;
                            //esp_zb_lock_acquire(portMAX_DELAY);
                            esp_err_t result = esp_zb_zcl_write_attr_cmd_req(&write_req1);
                            //esp_zb_lock_release();    
                            printf("fan_speed=%d\n", existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].data.level);
                        }else if(existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].cluster_id[cluster_index]== ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT){
                            /* configure report attribute command */
                            if(!is_reporting[index][node_index]){
                                is_reporting[index][node_index] = true;

                                int16_t report_change = 1; // Report on each change 
                                esp_zb_zcl_config_report_cmd_t report_cmd = {0};
                                report_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
                                report_cmd.zcl_basic_cmd.dst_addr_u.addr_short = existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].short_addr;  // Short address of Server B
                                report_cmd.zcl_basic_cmd.dst_endpoint = existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].dst_ep;
                                report_cmd.zcl_basic_cmd.src_endpoint = existing_nodes_info[index].scene_switch_info.src_ep;                                
                                report_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT;
                                
                                esp_zb_zcl_config_report_record_t records[] = {
                                    {
                                        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
                                        .attributeID = ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID,
                                        .attrType = ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM,
                                        .min_interval = 1,
                                        .max_interval = 0,
                                        .reportable_change = &report_change,
                                    },
                                    {
                                        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
                                        .attributeID = ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID,
                                        .attrType = ESP_ZB_ZCL_ATTR_TYPE_S16,
                                        .min_interval = 1,
                                        .max_interval = 0,
                                        .reportable_change = &report_change,
                                    },        
                                };
                                report_cmd.record_number = ARRAY_LENTH(records);
                                report_cmd.record_field = records;
                                esp_zb_zcl_config_report_cmd_req(&report_cmd);

                            }

                            uint8_t ac_mode = existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].data.level;
                            esp_zb_zcl_attribute_t attr_field;
                            attr_field.id = ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID;
                            attr_field.data.type = ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM;
                            attr_field.data.size = 1;
                            attr_field.data.value = (uint8_t*)&ac_mode;
                            esp_zb_zcl_write_attr_cmd_t write_req;
                            memset(&write_req, 0, sizeof(write_req));

                            write_req.zcl_basic_cmd.dst_addr_u.addr_short = existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].short_addr;
                            write_req.zcl_basic_cmd.dst_endpoint = existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].dst_ep; // Target endpoint on the device
                            write_req.zcl_basic_cmd.src_endpoint = existing_nodes_info[index].scene_switch_info.src_ep;
                            write_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
                            write_req.clusterID = ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT;
                            write_req.attr_field = &attr_field;
                            write_req.attr_number = 1;
                            write_req.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV;
                            // write_req.dis_default_resp = 1;
                            esp_err_t result = esp_zb_zcl_write_attr_cmd_req(&write_req);
                             printf("ac mode=%d\n", existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].data.level);
                            // printf("ac temp=%ld\n", existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].data.value);
                            if(ac_mode == 3){
                                esp_zb_zcl_attribute_t attr_field_2;
                                attr_field_2.id = ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID;
                                attr_field_2.data.type = ESP_ZB_ZCL_ATTR_TYPE_S16;
                                attr_field_2.data.size = 1;
                                uint16_t value = existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].data.value/100;
                                attr_field_2.data.value = (uint16_t*)&value;
                                esp_zb_zcl_write_attr_cmd_t write_req_2;
                                memset(&write_req_2, 0, sizeof(write_req_2));

                                write_req_2.zcl_basic_cmd.dst_addr_u.addr_short = existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].short_addr;
                                write_req_2.zcl_basic_cmd.dst_endpoint = existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].dst_ep; // Target endpoint on the device
                                write_req_2.zcl_basic_cmd.src_endpoint = existing_nodes_info[index].scene_switch_info.src_ep;
                                write_req_2.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
                                write_req_2.clusterID = ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT;
                                write_req_2.attr_field = &attr_field_2;
                                write_req_2.attr_number = 1;
                                // write_req_2.dis_default_resp = 1;
                                write_req_2.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV;
                                esp_err_t result2 = esp_zb_zcl_write_attr_cmd_req(&write_req_2);
                            }
                        }else{
                            printf("CLUSTER_ID:%d\n", existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].cluster_id[cluster_index]);
                        }
                    }
                } 
            } //for
        }
    }
    vTaskDelete(NULL); // Delete the task after executing
}

esp_err_t nuos_set_scene_button_attribute(uint8_t index){
    #ifndef DONT_USE_ZIGBEE
    printf("keypressed ==> ep=%d\n", ENDPOINTS_LIST[index]);
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
        printf("total_records=%d\n", existing_nodes_info[index].scene_switch_info.total_records);
        xTaskCreate(recall_scenes_task, "recall_scenes_task", TASK_STACK_SIZE_SCENE_RECALL, &index, TASK_PRIORITY_SCENE_RECALL, NULL);
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH)
        nuos_zb_group_send_light_onoff_groupcast(global_group_id[index], ESP_ZB_ZCL_CMD_ON_OFF_TOGGLE_ID);
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH)
        printf("keypressed ==> total_records=%d\n", nodes_bind_info[index].total_records);
        uint16_t gid = global_group_id[0];
        nuos_zb_scene_recall_scene_broadcast_request(gid, global_scene_id[index], ENDPOINTS_LIST[index]);
    #else  
        //
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_GROUP_SWITCH)

        #else

            uint8_t dst_ep = 1;  //gateway endpoint
            uint8_t addr_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
            // single_endpoint_gateway = true;
            if(!single_endpoint_gateway){
                dst_ep = ENDPOINTS_LIST[index];
            }

            #ifdef USE_CUSTOM_SCENE
                nuos_zb_set_hardware(index, true);

                if(zb_scene_info[index].scene_id == 0){
                    printf("group: 0x%x scene:%d uchKeypressed:%d\n", zb_scene_info[index].group_id, zb_scene_info[index].scene_id, uchKeypressed);
                    //nuos_set_state_attribute(index);
                    esp_zb_zcl_custom_cluster_cmd_req_t cmd_req = {
                        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
                        .cluster_id = 0xE000,
                        .custom_cmd_id = 0xFD,
                        .data.value = &index,
                        .data.size = 1,
                        .data.type = ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM,
                        #ifdef NEW_SDK_6
                            .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
                        #else
                            .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                        #endif
                        .profile_id = 0x0104,
                        .manuf_code = 0,
                        .manuf_specific = 0,
                        .dis_default_resp = 0,
                        .zcl_basic_cmd.dst_addr_u.addr_short = 0,
                        .zcl_basic_cmd.dst_endpoint = dst_ep,
                        .zcl_basic_cmd.src_endpoint = ENDPOINTS_LIST[index],
                    };
                    esp_zb_zcl_custom_cluster_cmd_req(&cmd_req);
                }else{
                    esp_err_t err = zb_switch_send_recall_scene_to_light(ENDPOINTS_LIST[index], zb_scene_info[index].group_id, zb_scene_info[index].scene_id);
                    if(err == ESP_OK){
                        printf("Recaling group: 0x%x scene:%d\n", zb_scene_info[index].group_id, zb_scene_info[index].scene_id);
                    }else{
                        //store error
                        //setNVSSceneRecallErrorFlag(err);
                        err = zb_switch_send_recall_scene_to_light(ENDPOINTS_LIST[index], zb_scene_info[index].group_id, zb_scene_info[index].scene_id);
                        if(err == ESP_OK){
                            printf("Recaling group: 0x%x scene:%d\n", zb_scene_info[index].group_id, zb_scene_info[index].scene_id);
                        }else{
                            //store error
                            //setNVSSceneRecallErrorFlag(err);
                            if(is_my_device_commissionned){
                                err = zb_switch_send_recall_scene_to_light(ENDPOINTS_LIST[index], zb_scene_info[index].group_id, zb_scene_info[index].scene_id);
                                if(err == ESP_OK){
                                    esp_zb_zcl_custom_cluster_cmd_req_t cmd_req = {
                                    .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
                                    .cluster_id = 0xE000,
                                    .custom_cmd_id = 0xFD,
                                    .data.value = &index,
                                    .data.size = 1,
                                    .data.type = ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM,
                                    #ifdef NEW_SDK_6
                                        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
                                    #else
                                        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                    #endif
                                    .profile_id = 0x0104,
                                    .manuf_code = 0,
                                    .manuf_specific = 0,
                                    .dis_default_resp = 0,
                                    .zcl_basic_cmd.dst_addr_u.addr_short = 0,
                                    .zcl_basic_cmd.dst_endpoint = dst_ep,
                                    .zcl_basic_cmd.src_endpoint = ENDPOINTS_LIST[index],
                                };
                                esp_zb_zcl_custom_cluster_cmd_req(&cmd_req);
                                }
                            }
                        }
                    }
                }
            #else
                printf("Sending Key:%d at ep:%d\n", uchKeypressed, ENDPOINTS_LIST[index]);
                esp_zb_zcl_custom_cluster_cmd_req_t cmd_req = {
                    .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
                    .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
                    .custom_cmd_id = 0xFD,
                    .data.value = &uchKeypressed,
                    .data.size = 1,
                    .data.type = ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM,
                    #ifdef NEW_SDK_6
                        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
                    #else
                        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                    #endif
                    .profile_id = 0x0104,
                    .zcl_basic_cmd.dst_addr_u.addr_short = 0,
                    .zcl_basic_cmd.dst_endpoint = dst_ep,
                    .zcl_basic_cmd.src_endpoint = ENDPOINTS_LIST[index],
                };
                esp_zb_zcl_custom_cluster_cmd_req(&cmd_req);          
            #endif
        #endif

    #endif
    #else
        #ifdef USE_CUSTOM_SCENE
        nuos_zb_set_hardware(index, true);
        #endif
    #endif
    return ESP_OK;
}

void send_report(int index, uint16_t cluster_id, uint16_t attr_id){
    #ifndef DONT_USE_ZIGBEE
    uint8_t dst_ep = 1;  //gateway endpoint
    uint8_t addr_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    // single_endpoint_gateway = true;
    if(!single_endpoint_gateway){
        dst_ep = ENDPOINTS_LIST[index];
    }
    esp_zb_zcl_report_attr_cmd_t report_cmd_2 = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = 0x0000,
            .dst_endpoint = dst_ep,
            .src_endpoint = ENDPOINTS_LIST[index],
        },
        #ifdef NEW_SDK_6
            .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
        #else
            .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        #endif
        .manuf_specific = 0,
        .manuf_code = 0,
        .dis_default_resp = 1,
        .address_mode = addr_mode,
        .clusterID = cluster_id,
        .attributeID = attr_id,
    };

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t err = esp_zb_zcl_report_attr_cmd_req(&report_cmd_2);
    if(err != ESP_OK){
        esp_zb_zcl_report_attr_cmd_req(&report_cmd_2);
    }
    esp_zb_lock_release();

    #endif
}
static uint8_t fan_level_val = 0;
uint8_t sequence_num2 = 0;

uint8_t map_0_4_to_0_254(uint8_t input) {
    return (input * 254) / 4;
}

///////////////////////////////////////////////////////////////
esp_err_t nuso_set_state_attribute_on_dali_rx(uint8_t index_1, bool state_){
    esp_zb_zcl_status_t status = ESP_OK;
    if (is_my_device_commissionned && !wifi_webserver_active_flag) {
        #ifdef USE_COLOR_CONTROL
            nuos_set_state_command(index_1, state_);
            return status;
        #endif
        status = esp_zb_zcl_set_attribute_val(
            ENDPOINTS_LIST[index_1],
            ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
            (uint8_t*)&state_,
            false
        );
        send_report(index_1, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID);
    }  
    return status;  
}





esp_err_t nuos_set_color_temp_level_attribute_on_dali_rx(uint8_t index, uint8_t level){
    esp_zb_zcl_status_t status = ESP_OK;
    #ifdef USE_TUYA_BRDIGE
    uint16_t val = map_1000(level, 0, 255, 10, 1000);
    if(val >= 1000) val = 1000;
    if (is_my_device_commissionned && !wifi_webserver_active_flag){

        #ifdef USE_COLOR_CONTROL
            nuos_set_level_command(index, level);
            return status;
        #endif

        status = esp_zb_zcl_set_attribute_val(
            ENDPOINTS_LIST[0],
            ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            0xF000,
            (uint16_t*)&val,
            false
        );
        send_report(0, ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL, 0xF000);  
    }
    #endif
    return status;
}

esp_err_t nuos_set_level_attribute_on_dali_rx(uint8_t index_1, uint8_t level){
    /* set attribute value */
    esp_zb_zcl_status_t status = ESP_OK;
    if (is_my_device_commissionned && !wifi_webserver_active_flag){
        #ifdef USE_COLOR_CONTROL
            nuos_set_level_command(index_1, level);
            return status;
        #endif
        status = esp_zb_zcl_set_attribute_val(
            ENDPOINTS_LIST[index_1],
            ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID,
            (uint8_t*)&level,
            false
        );
        send_report(index_1, ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID);
    }
    is_some_device_unavailable = false;
    return status;
}

uint8_t sequence_num_1 = 1;


// uint8_t state_data[7] = {0x0, 0x54, 0x65, 0x01, 0x00, 0x01, 0x0};
// uint8_t level_data[10] = {0x0, 0x5e, 0x67, 0x2, 0x0, 0x4, 0x0, 0x0, 0x3, 0x39};
// uint8_t color_data[10] = {0x0, 0x5f, 0x69, 0x2, 0x0, 0x4, 0x0, 0x0, 0x1, 0x17};

uint8_t state_data[7] = {0x0, 0x54, 0x01, 0x01, 0x00, 0x01, 0x0};
uint8_t level_data[10] = {0x0, 0x5e, 0x02, 0x2, 0x0, 0x4, 0x0, 0x0, 0x3, 0x39};
uint8_t color_data[10] = {0x0, 0x5f, 0x65, 0x2, 0x0, 0x4, 0x0, 0x0, 0x1, 0x17};

void nuos_set_state_command(uint8_t index, uint8_t state){
    // if(index == 0){
    //     state_data[2] = 0x65;
    // }else if(index == 1){
    //     state_data[2] = 0x66;
    // }
    if(index == 0){
        state_data[2] = 0x1;
    }else if(index == 1){
        state_data[2] = 0x7;
    }
    device_info[index].device_state = (bool)state;
    state_data[6] = state;

    state_data[1] = sequence_num_1;
    sequence_num_1++;
    
    //printf("Sending index:%d command to dpid:0x%x\n", index, state_data[2]);
    
    esp_zb_zcl_custom_cluster_cmd_req_t cmd_req = {
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .cluster_id = 0xef00,
        .custom_cmd_id = 1,
        .data.value = state_data,
        .data.size = 7,
        .data.type = ESP_ZB_ZCL_ATTR_TYPE_U56,
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
        .profile_id = 0x0104,
        .zcl_basic_cmd.dst_addr_u.addr_short = 0,
        .zcl_basic_cmd.dst_endpoint = ENDPOINTS_LIST[index],
        .zcl_basic_cmd.src_endpoint = ENDPOINTS_LIST[index],
        .dis_default_resp = 1,
        .manuf_code = 0,
        .manuf_specific = 0, 
    };
    esp_zb_lock_acquire(portMAX_DELAY);
    sequence_num_1 = esp_zb_zcl_custom_cluster_cmd_req(&cmd_req); 
    esp_zb_lock_release();
}
void nuos_set_level_command(uint8_t index, uint8_t level){

    // if(index == 0){
    //     level_data[2] = 0x67;
    // }else if(index == 1){
    //     level_data[2] = 0x68;
    // }
    if(index == 0){
        level_data[2] = 0x2;
    }else if(index == 1){
        level_data[2] = 0x8;
    }
    uint16_t mapValue = map_1000(level, MIN_DIM_LEVEL_VALUE, MAX_DIM_LEVEL_VALUE, 10, 1000);
    if(mapValue >= 1000) mapValue = 1000; 
    level_data[8] = mapValue >> 8;
    level_data[9] = mapValue & 0xFF;

    level_data[1] = sequence_num_1;
    sequence_num_1++;

    uint8_t payload[9];
    payload[0] = level_data[1];         // 1 byte sequence number
    payload[1] = level_data[2];            // DP ID (temperature setpoint)
    payload[2] = level_data[3];            // DP type (value)
    payload[3] = level_data[4];            // Function (set/report), often 0x00
    payload[4] = level_data[5];            // Data length (4 bytes)
    payload[5] = level_data[6];            // Value MSB (= 0)
    payload[6] = level_data[7];
    payload[7] = level_data[8];
    payload[8] = level_data[9];            // Value LSB (= 24)

    // Create octet string: first byte is length, then payload
    uint8_t octet_string[10];
    octet_string[0] = 9; // length byte
    memcpy(&octet_string[1], payload, 9);

    esp_zb_zcl_custom_cluster_cmd_req_t cmd_req = {
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .cluster_id = 0xef00,
        .custom_cmd_id = 1,
        .data.value = octet_string,
        .data.size = 10,
        .data.type = ESP_ZB_ZCL_ATTR_TYPE_OCTET_STRING,
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
        .profile_id = 0x0104,
        .zcl_basic_cmd.dst_addr_u.addr_short = 0,
        .zcl_basic_cmd.dst_endpoint = ENDPOINTS_LIST[index],
        .zcl_basic_cmd.src_endpoint = ENDPOINTS_LIST[index],
        .dis_default_resp = 1,
        .manuf_code = 0,
        .manuf_specific = 0, 
    };
    esp_zb_lock_acquire(portMAX_DELAY);
    sequence_num_1 = esp_zb_zcl_custom_cluster_cmd_req(&cmd_req); 
    esp_zb_lock_release();
}
void nuos_set_color_temp_command(uint8_t index, uint16_t color){
    

    uint16_t mapValue = map_1000(color, 2000, 6500, 1000, 10);
    if(mapValue >= 1000) mapValue = 1000; 
    if(mapValue <= 10) mapValue = 10; 
    // if(index == 0){
    //     color_data[2] = 0x69;
    // }else if(index == 1){
    //     color_data[2] = 0x6a;
    // }
    if(index == 0){
        color_data[2] = 0x65;
    }else if(index == 1){
        color_data[2] = 0x66;
    }
    color_data[8] = (mapValue >> 8) & 0xFF;
    color_data[9] = mapValue & 0xFF;

    color_data[1] = sequence_num_1;
    sequence_num_1++;
    
    uint8_t payload[9];
    payload[0] = color_data[1];         // 1 byte sequence number
    payload[1] = color_data[2];            // DP ID (temperature setpoint)
    payload[2] = color_data[3];            // DP type (value)
    payload[3] = color_data[4];            // Function (set/report), often 0x00
    payload[4] = color_data[5];            // Data length (4 bytes)
    payload[5] = color_data[6];            // Value MSB (= 0)
    payload[6] = color_data[7];
    payload[7] = color_data[8];
    payload[8] = color_data[9];            // Value LSB (= 24)

    // Create octet string: first byte is length, then payload
    uint8_t octet_string[10];
    octet_string[0] = 9; // length byte
    memcpy(&octet_string[1], payload, 9);

    esp_zb_zcl_custom_cluster_cmd_req_t cmd_req = {
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .cluster_id = 0xef00,
        .custom_cmd_id = 1,
        .data.value = octet_string,
        .data.size = 10,
        .data.type = ESP_ZB_ZCL_ATTR_TYPE_OCTET_STRING,
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
        .profile_id = 0x0104,
        .zcl_basic_cmd.dst_addr_u.addr_short = 0,
        .zcl_basic_cmd.dst_endpoint = ENDPOINTS_LIST[index],
        .zcl_basic_cmd.src_endpoint = ENDPOINTS_LIST[index],
        .dis_default_resp = 1,
        .manuf_code = 0,
        .manuf_specific = 0, 
    };
    esp_zb_lock_acquire(portMAX_DELAY);
    sequence_num_1 = esp_zb_zcl_custom_cluster_cmd_req(&cmd_req); 
    esp_zb_lock_release();
}


esp_err_t nuos_set_state_attribute(uint8_t index){

    esp_zb_zcl_status_t status = ESP_OK;
    #ifndef DONT_USE_ZIGBEE
    uint8_t index_1 = index;
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
        index_1 = 0;   
    #endif
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN)
        #ifdef USE_TUYA_BRDIGE
            index_1 = 0;
        #endif
    #endif    


    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN)
        #ifdef USE_TUYA_BRDIGE
        uint56_t   state_data = {0};
        state_data.bytes[0] = 0;
        state_data.bytes[1] = sequence_num2;
        state_data.bytes[2] = 5;
        state_data.bytes[3] = 1;
        state_data.bytes[4] = 0;
        state_data.bytes[5] = 1;
        state_data.bytes[6] = device_info[LIGHT_INDEX].device_state;
        esp_zb_zcl_custom_cluster_cmd_req_t cmd_req3 = {
            .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
            .cluster_id = 0xef00,
            .custom_cmd_id = 1,
            .data.value = state_data.bytes,
            .data.size = 7,
            .data.type = ESP_ZB_ZCL_ATTR_TYPE_U56,
            .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
            .profile_id = 0x0104,
            .zcl_basic_cmd.dst_addr_u.addr_short = 0,
            .zcl_basic_cmd.dst_endpoint = ENDPOINTS_LIST[0],
            .zcl_basic_cmd.src_endpoint = ENDPOINTS_LIST[0],
            .dis_default_resp = 1,
            .manuf_code = 0,
            .manuf_specific = 0, 
        };
        sequence_num2 = esp_zb_zcl_custom_cluster_cmd_req(&cmd_req3); 
        #else
            status = esp_zb_zcl_set_attribute_val(
                ENDPOINTS_LIST[index_1],
                ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
                (uint8_t*)&device_info[index].device_state,
                false
            );
            send_report(index, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID);         
        #endif 
    #else
            //printf("is_my_device_commissionned=%d wifi_webserver_active_flag=%d\n", is_my_device_commissionned, wifi_webserver_active_flag);
       if (is_my_device_commissionned && !wifi_webserver_active_flag) {
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM && TOTAL_ENDPOINTS == 2)
                nuos_set_color_temp_level_attribute_on_dali_rx(index_1, device_info[index].device_level);

            #else
                // #ifdef USE_COLOR_CONTROL
                // nuos_set_state_command(index, device_info[index].device_state);
                // return status;
                // #endif
            
                status = esp_zb_zcl_set_attribute_val(
                    ENDPOINTS_LIST[index_1],
                    ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
                    ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                    ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
                    (uint8_t*)&device_info[index].device_state,
                    false
                );
                printf("SEND_ZB_RESPONSE EP:%d STATE:%d\n", ENDPOINTS_LIST[index_1], device_info[index].device_state);
                send_report(index, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID);
            #endif
        }    
        is_some_device_unavailable = false;

    #endif    
    #endif    
    return status;
}

esp_err_t nuos_set_level_attribute(uint8_t index){
    /* set attribute value */
    esp_zb_zcl_status_t status = ESP_OK;
    #ifndef DONT_USE_ZIGBEE
    uint8_t index_1 = index;
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
        index_1 = 0;   
    #endif
    is_some_device_unavailable = false;
    if (is_my_device_commissionned && !wifi_webserver_active_flag){ 
        // #ifdef USE_COLOR_CONTROL
        // nuos_set_level_command(index, device_info[index].device_level);
        // return status;
        // #endif
        status = esp_zb_zcl_set_attribute_val(
            ENDPOINTS_LIST[index_1],
            ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID,
            (uint8_t*)&device_info[index].device_level,
            false
        );
        send_report(index_1, ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID);
    }
    
    #endif
    return status;
}

esp_err_t nuos_set_fan_attribute(uint8_t index){
    esp_err_t status = ESP_OK;
    #ifndef DONT_USE_ZIGBEE
    /* set attribute value */
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN_CUSTOM)          
        // nuos_set_fan_speed_level_attribute(FAN_INDEX);
/* set attribute value */
    //esp_zb_lock_acquire(portMAX_DELAY);
    if (is_my_device_commissionned){// && !is_some_device_unavailable) {

        status = esp_zb_zcl_set_attribute_val(
            ENDPOINTS_LIST[index],
            ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID,
            (uint8_t*)&device_info[index].device_level,
            false
        );
        //esp_zb_lock_release(); 
        send_report(index, ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID);         
   }
   is_some_device_unavailable = false;
    #else 
        // #ifndef USE_INDIVIDUAL_DALI_ADDRESSING
        //     uint8_t fan_speed = device_info[index].fan_speed;
        //     if(!device_info[index].device_state){
        //         fan_speed = 0;
        //     }else{
        //         if(device_info[index].fan_speed == 3) fan_speed = 2;
        //         else if(device_info[index].fan_speed == 4) fan_speed = 3;
        //         else fan_speed = device_info[index].fan_speed; 
        //     }  
        //     printf("fan_speed:%d\n", fan_speed);
        //     uint56_t   fan_data = {0};
        //     fan_data.bytes[0] = 0;
        //     fan_data.bytes[1] = sequence_num2;
        //     fan_data.bytes[2] = 3;
        //     fan_data.bytes[3] = 4;
        //     fan_data.bytes[4] = 0;
        //     fan_data.bytes[5] = 1;
        //     fan_data.bytes[6] = device_info[index].fan_speed - 1;
        //     esp_zb_zcl_custom_cluster_cmd_req_t cmd_req2 = {
        //         .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        //         .cluster_id = 0xef00,
        //         .custom_cmd_id = 1,
        //         .data.value = fan_data.bytes,
        //         .data.size = 7,
        //         .data.type = ESP_ZB_ZCL_ATTR_TYPE_U56,
        //         .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
        //         .profile_id = 0x0104,
        //         .zcl_basic_cmd.dst_addr_u.addr_short = 0,
        //         .zcl_basic_cmd.dst_endpoint = ENDPOINTS_LIST[0],
        //         .zcl_basic_cmd.src_endpoint = ENDPOINTS_LIST[0],
        //         .dis_default_resp = 1,
        //         .manuf_code = 0,
        //         .manuf_specific = 0, 
        //     };
        //     sequence_num2 = esp_zb_zcl_custom_cluster_cmd_req(&cmd_req2);

        //     vTaskDelay(pdMS_TO_TICKS(500));
        //     uint56_t   state_data = {0};
        //     state_data.bytes[0] = 0;
        //     state_data.bytes[1] = sequence_num2;
        //     state_data.bytes[2] = 1;
        //     state_data.bytes[3] = 1;
        //     state_data.bytes[4] = 0;
        //     state_data.bytes[5] = 1;
        //     state_data.bytes[6] = device_info[FAN_INDEX].device_state;
        //     esp_zb_zcl_custom_cluster_cmd_req_t cmd_req3 = {
        //         .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        //         .cluster_id = 0xef00,
        //         .custom_cmd_id = 1,
        //         .data.value = state_data.bytes,
        //         .data.size = 7,
        //         .data.type = ESP_ZB_ZCL_ATTR_TYPE_U56,
        //         .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
        //         .profile_id = 0x0104,
        //         .zcl_basic_cmd.dst_addr_u.addr_short = 0,
        //         .zcl_basic_cmd.dst_endpoint = ENDPOINTS_LIST[0],
        //         .zcl_basic_cmd.src_endpoint = ENDPOINTS_LIST[0],
        //         .dis_default_resp = 1,
        //         .manuf_code = 0,
        //         .manuf_specific = 0, 
        //     };
        //     sequence_num2 = esp_zb_zcl_custom_cluster_cmd_req(&cmd_req3);                
        // #endif    
    #endif
    #endif
    return status;
}



esp_err_t nuos_set_sensor_motion_attribute(uint8_t value){

    esp_zb_zcl_ias_zone_enroll_request_cmd_t cmd_req;
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_MOTION)
        cmd_req.zone_type = ESP_ZB_ZCL_IAS_ZONE_ZONETYPE_MOTION;
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_CONTACT_SWITCH)
        cmd_req.zone_type = ESP_ZB_ZCL_IAS_ZONE_ZONETYPE_CONTACT_SWITCH;
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_GAS_LEAK)
        cmd_req.zone_type = ESP_ZB_ZCL_IAS_ZONE_ZONETYPE_GAS_SENSOR;
    #else
        cmd_req.zone_type = ESP_ZB_ZCL_IAS_ZONE_ZONETYPE_CONTACT_SWITCH;    
    #endif
    
    cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = 0;
    cmd_req.zcl_basic_cmd.dst_endpoint = ENDPOINTS_LIST[0];
    cmd_req.zcl_basic_cmd.src_endpoint = ENDPOINTS_LIST[0];
    esp_zb_zcl_ias_zone_enroll_cmd_req(&cmd_req);

    esp_zb_zcl_ias_zone_status_change_notif_cmd_t resp;
    resp.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    resp.extend_status = 1;

    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_MOTION)
    resp.zone_id = 20;
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_CONTACT_SWITCH)
    resp.zone_id = 0;
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_GAS_LEAK)
    resp.zone_id = 10;
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RINGING_BELL_2)
    resp.zone_id = 0;    
    #endif
    #ifdef USE_TEMPER_ALARM
        uint8_t temper_alarm = 0;
        if(value>0)temper_alarm = 1;
        else temper_alarm = 0;
            resp.zone_status = value | (temper_alarm<<2);
    #else
        resp.zone_status = value;
    #endif
    resp.zcl_basic_cmd.dst_addr_u.addr_short = 0;
    resp.zcl_basic_cmd.dst_endpoint = ENDPOINTS_LIST[0];
    resp.zcl_basic_cmd.src_endpoint = ENDPOINTS_LIST[0];
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_ias_zone_status_change_notif_cmd_req(&resp);
    esp_zb_lock_release(); 
    #ifdef IS_REPORT_BATTERY_STATUS
        esp_zb_zcl_report_attr_cmd_t report_attr;
        report_attr.attributeID = ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID;
        report_attr.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
        report_attr.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
        report_attr.clusterID = ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG;
        report_attr.zcl_basic_cmd.dst_addr_u.addr_short = 0;
        report_attr.zcl_basic_cmd.dst_endpoint = ENDPOINTS_LIST[0];
        report_attr.zcl_basic_cmd.src_endpoint = ENDPOINTS_LIST[0];
        esp_zb_lock_acquire(portMAX_DELAY);
        esp_zb_zcl_report_attr_cmd_req(&report_attr);
        esp_zb_lock_release(); 
    #endif
    return ESP_OK;
}

esp_err_t nuos_set_sensor_temperature_attribute(uint8_t index){
    
    esp_zb_zcl_status_t status = ESP_OK;
    #ifndef DONT_USE_ZIGBEE
    //esp_zb_lock_acquire(portMAX_DELAY);
    if (is_my_device_commissionned && !wifi_webserver_active_flag){
        status = esp_zb_zcl_set_attribute_val(
            ENDPOINTS_LIST[index],
            ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
            (uint16_t*)&device_info[index].device_val,
            false
        );
        //esp_zb_lock_release(); 
        esp_zb_zcl_report_attr_cmd_t report_attr_cmd;
        report_attr_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
        report_attr_cmd.attributeID = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID;
        #ifdef NEW_SDK_6
        report_attr_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
        #else
        report_attr_cmd.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
        #endif
        report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT;
        report_attr_cmd.zcl_basic_cmd.src_endpoint = ENDPOINTS_LIST[index];
        //esp_zb_lock_acquire(portMAX_DELAY);
        esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd);
    // //esp_zb_lock_release(); 
    }
    #endif
    return status;
}

esp_err_t nuos_set_sensor_humidity_attribute(uint8_t index){
    esp_zb_zcl_status_t status = ESP_OK;
    //esp_zb_lock_acquire(portMAX_DELAY);
    if (is_my_device_commissionned && !wifi_webserver_active_flag){
        status = esp_zb_zcl_set_attribute_val(
            ENDPOINTS_LIST[index],
            ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
            (uint16_t*)&device_info[index].device_val,
            false
        );
        //esp_zb_lock_release(); 
        esp_zb_zcl_report_attr_cmd_t report_attr_cmd2;
        report_attr_cmd2.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
        report_attr_cmd2.attributeID = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID;
        #ifdef NEW_SDK_6
        report_attr_cmd2.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
        #else
        report_attr_cmd2.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
        #endif
        report_attr_cmd2.clusterID = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT;
        report_attr_cmd2.zcl_basic_cmd.src_endpoint = ENDPOINTS_LIST[index];
        //esp_zb_lock_acquire(portMAX_DELAY);
        esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd2);
        //esp_zb_lock_release();
    } 
    return status;
}

//-----------Tuya Cluster's attribute update-------------//
esp_err_t nuos_set_color_temperature_attribute(uint8_t index){
    #ifdef USE_TUYA_BRDIGE
        /* set attribute value */
        uint16_t mapValue = map_1000(device_info[index].device_val, 2000, 6500, 10, 1000);
        if(mapValue >= 1000) mapValue = 1000;
        printf("mapValue:%d\n", mapValue);
        
        esp_zb_zcl_status_t status = ESP_OK;
        if (is_my_device_commissionned && !wifi_webserver_active_flag){
            #ifdef USE_COLOR_CONTROL 
            nuos_set_color_temp_command(index, device_info[index].device_val);
            return status;
            #endif 
            status = esp_zb_zcl_set_attribute_val(
                ENDPOINTS_LIST[0],
                ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                0xE000,
                (uint16_t*)&mapValue,
                false
            );
            send_report(0, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0xE000);

        } 
    #else
        uint8_t index_1;
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
        index_1 = 0;
        #else
        index_1 = 3;
        printf("device_info[3].device_val:%d\n", device_info[index_1].device_val);
        #endif
        esp_zb_zcl_status_t status = esp_zb_zcl_set_attribute_val(
            ENDPOINTS_LIST[0],
            ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID,
            (uint16_t*)&device_info[index_1].device_val,
            false
        );
        send_report(0, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID); 
    #endif
    return status;
}


esp_err_t nuos_set_color_temp_level_attribute(uint8_t index){
    esp_zb_zcl_status_t status = ESP_OK;
    #ifdef USE_TUYA_BRDIGE
    uint16_t val = map_1000(device_info[index].device_level, MIN_DIM_LEVEL_VALUE, MAX_DIM_LEVEL_VALUE, 10, 1000);
    if(val >= 1000) val = 1000;
    if (is_my_device_commissionned && !wifi_webserver_active_flag){
        status = esp_zb_zcl_set_attribute_val(
            ENDPOINTS_LIST[0],
            ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            0xF000,
            (uint16_t*)&val,
            false
        );
        send_report(0, ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL, 0xF000);  


            // uint8_t level_data[10] = {0x0, 0x5e, 0x67, 0x2, 0x0, 0x4, 0x0, 0x0, 0x3, 0x39};

            //  if(level_data[2] == 0x65){
            //     index_1  = 1;
            // }else if(level_data[2] == 0x66){
            //     index_1  = 2;
            // }
            // level_data[1] = sequence_num_1;
            // sequence_num_1++;
            // level_data[6] = device_info[index_1].device_level;
            // esp_zb_zcl_custom_cluster_cmd_req_t cmd_req = {
            //     .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
            //     .cluster_id = 0xef00,
            //     .custom_cmd_id = 1,
            //     .data.value = level_data,
            //     .data.size = 7,
            //     .data.type = ESP_ZB_ZCL_ATTR_TYPE_U56,
            //     .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
            //     .profile_id = 0x0104,
            //     .zcl_basic_cmd.dst_addr_u.addr_short = 0,
            //     .zcl_basic_cmd.dst_endpoint = ENDPOINTS_LIST[index_1],
            //     .zcl_basic_cmd.src_endpoint = ENDPOINTS_LIST[index_1],
            //     .dis_default_resp = 1,
            //     .manuf_code = 0,
            //     .manuf_specific = 0, 
            // };
            // sequence_num_1 = esp_zb_zcl_custom_cluster_cmd_req(&cmd_req); 


    }
    #endif
    return status;
}

esp_err_t nuos_set_color_temp_mode_attribute(uint8_t index){
    // mode color light
    uint8_t val_mode = 0;
    esp_zb_zcl_status_t status = ESP_OK;
    /* set attribute value */
    #ifdef USE_TUYA_BRDIGE
    if (is_my_device_commissionned && !wifi_webserver_active_flag){
        status = esp_zb_zcl_set_attribute_val(
            ENDPOINTS_LIST[0],
            ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            0xF000,
            (uint8_t*)&val_mode,
            false
        );
        send_report(0, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0xF000);
    }
    #endif
    return status;
}

esp_err_t nuos_set_color_hue_attribute(uint8_t index, uint16_t hue){
    // mode color light
    esp_zb_zcl_status_t status = ESP_OK;
    uint8_t index_1 = index;
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
        index_1 = 0;   
    #endif    
    if (is_my_device_commissionned && !wifi_webserver_active_flag){
        /* set attribute value */
        status = esp_zb_zcl_set_attribute_val(
            ENDPOINTS_LIST[index_1],
            ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_HUE_ID,
            (uint8_t*)&hue,
            false
        );

        send_report(index_1, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_HUE_ID);
    }
    return status;
}


esp_err_t nuos_set_color_rgb_mode_attribute(uint8_t index, uint8_t val_mode){
    /* set attribute value */
    esp_zb_zcl_status_t status = ESP_OK;
    #ifdef USE_TUYA_BRDIGE
    if (is_my_device_commissionned && !wifi_webserver_active_flag){
        status = esp_zb_zcl_set_attribute_val(
            ENDPOINTS_LIST[index],
            ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            0xF000,
            (uint8_t*)&val_mode,
            false
        );

        send_report(index, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0xF000);
    }   
    #endif
    return status;
}

esp_err_t nuos_set_state_attribute_rgb(uint8_t index){
    /* set attribute value */
    esp_zb_zcl_status_t status = ESP_OK;
    if (is_my_device_commissionned && !wifi_webserver_active_flag){
        status = esp_zb_zcl_set_attribute_val(
        ENDPOINTS_LIST[0],
        ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
        (uint8_t*)&device_info[index].device_state,
        false
        );

        send_report(0, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID);
    }
    return status;
}

esp_err_t nuos_set_color_xy_attribute(uint8_t index, hsv_t* hsv){
    uint48_t val = {0};
    esp_zb_zcl_status_t status = ESP_OK;
    #ifdef USE_TUYA_BRDIGE
    if(hsv != NULL){
        val.bytes[1] = (hsv->h >> 8) & 0xff;
        val.bytes[0] = hsv->h & 0xff;

        val.bytes[3] = (hsv->s >> 8) & 0xff;
        val.bytes[2] = hsv->s & 0xff;

        val.bytes[5] = (hsv->v >> 8) & 0xff;
        val.bytes[4] = hsv->v & 0xff; 

        #if(USE_COLOR_DEVICE == COLOR_RGB_ONLY)
        hsv->v = (device_info[4].device_level*1000)/254;
        #endif          
    }else{
      
        rgb_t rgb = {device_info[0].device_level, device_info[1].device_level, device_info[2].device_level}; // Example RGB values
        hsv_t hsv_2 = rgb_to_hsv(rgb);
        val.bytes[1] = (hsv_2.h >> 8) & 0xff;
        val.bytes[0] = hsv_2.h & 0xff;

        val.bytes[3] = (hsv_2.s >> 8) & 0xff;
        val.bytes[2] = hsv_2.s & 0xff;

        val.bytes[5] = (hsv_2.v >> 8) & 0xff;
        val.bytes[4] = hsv_2.v & 0xff;
    }
    if (is_my_device_commissionned && !wifi_webserver_active_flag){

    /* set attribute value */
    //esp_zb_lock_acquire(portMAX_DELAY);
    status = esp_zb_zcl_set_attribute_val(
        ENDPOINTS_LIST[0],
        ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        0xE100,
        (uint48_t*)&val,
        false
    ); 
    //esp_zb_lock_release();
    esp_zb_zcl_report_attr_cmd_t report_cmd_2 = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = 0x0000,
            .dst_endpoint = ENDPOINTS_LIST[0],
            .src_endpoint = ENDPOINTS_LIST[0],
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL,
        .attributeID = 0xE100,
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI
    };
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_report_attr_cmd_req(&report_cmd_2);
    esp_zb_lock_release();
    }
    #else
        nuos_set_color_hue_attribute(0, hsv->h);
    #endif
    return status;
}


esp_err_t nuos_set_color_temp_attribute(uint8_t index){
    esp_zb_zcl_status_t status = ESP_OK;
    uint16_t m_level = (device_info[index].device_level*1000)/254;
    #ifdef USE_TUYA_BRDIGE
    if (is_my_device_commissionned && !wifi_webserver_active_flag){
        status = esp_zb_zcl_set_attribute_val(
            ENDPOINTS_LIST[0],
            ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            0xF000,
            (uint16_t*)&m_level,
            false
        );
        send_report(0, ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL, 0xF000);  
    
        uint16_t color_temp = map_1000(device_info[index].device_val, MIN_CCT_VALUE, MAX_CCT_VALUE, 10, 1000);
        if(color_temp >= 1000) color_temp = 1000;
        status = esp_zb_zcl_set_attribute_val(
            ENDPOINTS_LIST[0],
            ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            0xE000,
            (uint16_t*)&color_temp,
            false
        );
        send_report(0, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0xE000); 
    }
    #else

    uint16_t mapValue = map_1000(device_info[3].device_val, 2000, 6500, 158, 500);
    if(mapValue >= 500) mapValue = 500;
    printf("mapValue:%d\n", mapValue);

    status = esp_zb_zcl_set_attribute_val(
        ENDPOINTS_LIST[0],
        ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID,
        (uint16_t*)&mapValue,
        false
    );
    
    send_report(0, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID); 
    #endif 
    return status;
}


void read_zigbee_time() {

    uint16_t attributes[] = {ESP_ZB_ZCL_ATTR_TIME_LOCAL_TIME_ID};
    // esp_zb_zcl_attribute_t attr_fields;
    // attr_fields.id = ESP_ZB_ZCL_ATTR_TIME_LOCAL_TIME_ID;
    // attr_fields.data.type = ESP_ZB_ZCL_ATTR_TYPE_32BIT;
    // attr_fields.data.size = 4;
    esp_zb_zcl_read_attr_cmd_t read_cmd = {
        .zcl_basic_cmd.dst_endpoint = ENDPOINTS_LIST[0],
        .zcl_basic_cmd.src_endpoint = ENDPOINTS_LIST[0],
        .zcl_basic_cmd.dst_addr_u.addr_short = 0x0000,
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ESP_ZB_ZCL_CLUSTER_ID_TIME,
        .manuf_specific = 0,
        #ifdef USE_HOME_ASSISTANT
            .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
        #else
            .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        #endif
        // .dis_default_resp = 1,
        .manuf_code = 0,
        .attr_number = 1
    };
    read_cmd.attr_field  = attributes;
    esp_zb_zcl_read_attr_cmd_req(&read_cmd);
    printf("Sending Server Time Request\n");
}


esp_err_t nuos_set_sensor_temperature_hunidity_attribute(uint8_t index){
    esp_zb_zcl_status_t status = ESP_OK;
    status = esp_zb_zcl_set_attribute_val(
        ENDPOINTS_LIST[index],
        ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        (uint16_t*)&device_info[0].device_val,
        false
    );
    esp_zb_zcl_report_attr_cmd_t report_attr_cmd;
    report_attr_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    report_attr_cmd.zcl_basic_cmd.dst_addr_u.addr_short = 0;
    report_attr_cmd.zcl_basic_cmd.dst_endpoint = ENDPOINTS_LIST[index];
    report_attr_cmd.zcl_basic_cmd.src_endpoint = ENDPOINTS_LIST[index];
    report_attr_cmd.attributeID = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID;
    #ifdef NEW_SDK_6
    report_attr_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
    #else
    report_attr_cmd.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    #endif    
    report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT;
    //esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd);
    //esp_zb_lock_release(); 

    //esp_zb_lock_acquire(portMAX_DELAY);
    status = esp_zb_zcl_set_attribute_val(
        ENDPOINTS_LIST[index],
        ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
        (uint16_t*)&device_info[1].device_val,
        false
    );
    //esp_zb_lock_release(); 

    esp_zb_zcl_report_attr_cmd_t report_attr_cmd2;
    report_attr_cmd2.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    report_attr_cmd2.zcl_basic_cmd.dst_addr_u.addr_short = 0;
    report_attr_cmd2.zcl_basic_cmd.dst_endpoint = ENDPOINTS_LIST[index];
    report_attr_cmd2.zcl_basic_cmd.src_endpoint = ENDPOINTS_LIST[index];
    report_attr_cmd2.attributeID = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID;
    #ifdef NEW_SDK_6
    report_attr_cmd2.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
    #else
    report_attr_cmd2.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    #endif  
    report_attr_cmd2.clusterID = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT;
    //esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd2); 
    //esp_zb_lock_release();    
    return status;
}

// Convert desired lux to Zigbee MeasuredValue
uint16_t lux_to_zigbee_measuredvalue(float lux)
{
    if (lux <= 0.0f) {
        return 0; // 0 means "too low to be measured"
    }
    return (uint16_t)(10000.0f * log10f(lux) + 1.0f + 0.5f); // +0.5 for rounding
}

esp_err_t nuos_set_sensor_illuminance_attribute(uint8_t index) {
    uint16_t val = lux_to_zigbee_measuredvalue(device_info[index].device_val);
    esp_zb_zcl_status_t status = esp_zb_zcl_set_attribute_val(
        ENDPOINTS_LIST[index],
        ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_ID,
        (uint16_t*)&val,
        false
    ); 
    esp_zb_zcl_report_attr_cmd_t report_cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = 0x0000,
            .dst_endpoint = ENDPOINTS_LIST[index],
            .src_endpoint = ENDPOINTS_LIST[index],
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT,
        .attributeID = ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_ID,
        #ifdef NEW_SDK_6
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
        #else
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        #endif
    };
    esp_zb_zcl_report_attr_cmd_req(&report_cmd);
    return status;
}

esp_err_t nuos_set_ac_cool_temperature_attribute(uint8_t index){
    /* set attribute value */
    esp_zb_zcl_status_t status = ESP_OK;
    //esp_zb_lock_acquire(portMAX_DELAY);
     status = esp_zb_zcl_set_attribute_val(
        ENDPOINTS_LIST[index],
        ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID,
        (uint8_t*)&device_info[index].device_level,
        false
    );
    send_report(index, ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID); 
    //esp_zb_lock_release(); 
    return status;
}

esp_err_t nuos_set_thermostat_mode_attr(uint8_t index, uint8_t mode_val){
    esp_zb_zcl_status_t status = ESP_OK;
    status = esp_zb_zcl_set_attribute_val(
        ENDPOINTS_LIST[index],
        ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID,
        (uint8_t*)&mode_val,  //mode = cool or off
        false
    );
    send_report(index, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT, ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID);
    return status;
}

esp_err_t nuos_set_thermostat_temp_attribute(uint8_t index){
    esp_zb_zcl_status_t status = ESP_OK;
    printf("Setting AC Temperature to %d\n", device_info[index].ac_temperature);
    status = esp_zb_zcl_set_attribute_val(
        ENDPOINTS_LIST[index],
        ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID,

        (uint16_t*)&device_info[index].ac_temperature,  //temperature
        false
    );
    send_report(index, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT, ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID);
    return status;
}

esp_err_t nuos_set_thermostat_attribute(uint8_t index){
    esp_zb_zcl_status_t status = ESP_OK;
    //esp_zb_lock_acquire(portMAX_DELAY);
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)  
        nuos_set_state_attribute(index);
        vTaskDelay(pdMS_TO_TICKS(10));
        if(device_info[index].device_state){
            nuos_set_ac_cool_temperature_attribute(index);
        } 
    #else
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER)  
            //index = 1;
            if(index == 1){
                uint8_t ac_mode = 0;
                if(device_info[index].device_state){ 
                    ac_mode = ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_COOL;  //cooling 
                }else{
                    ac_mode = ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF;
                }
                printf("Set AC Mode to %d\n", ac_mode);

                status = esp_zb_zcl_set_attribute_val(
                    ENDPOINTS_LIST[index],
                    ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                    ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                    ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID,
                    (uint8_t*)&ac_mode,  //mode = cool or off
                    false
                );
                send_report(index, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT, ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID);
            }
        #endif
    #endif
    return status;
}


void nuos_send_window_covering_command(uint8_t index, uint8_t command_id){
    esp_zb_zcl_window_covering_cluster_send_cmd_req_t cmd_req;
	cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
	cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;
	cmd_req.zcl_basic_cmd.dst_endpoint = ENDPOINTS_LIST[index];
	cmd_req.zcl_basic_cmd.src_endpoint = ENDPOINTS_LIST[index];
	cmd_req.cmd_id = command_id;

	esp_zb_zcl_window_covering_cluster_send_cmd_req(&cmd_req);
	printf("Updated window covering cluster\n");
}

void nuos_report_curtain_blind_state(uint8_t index, uint8_t value){
    #ifndef DONT_USE_ZIGBEE
        printf("Sending back response!!\n");
        esp_zb_zcl_status_t status = ESP_OK;
        status = esp_zb_zcl_set_attribute_val(
            ENDPOINTS_LIST[index],
            ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID,
            &value,
            false);
        send_report(index, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID); 
    #endif    
}


/* ********************************************************************************** */

void nuos_set_zigbee_attribute(uint8_t index){
        if(is_my_device_commissionned > 0){  
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4R_ON_OFF_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2R_ON_OFF_LIGHT)
                nuos_set_state_attribute(index);
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2CH_CURTAIN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN_SWITCH)

                #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN_SWITCH)
                    nuos_report_curtain_blind_state(index, device_info[0].fan_speed);
                #else
                    #ifdef TUYA_ATTRIBUTES
                        uint8_t val = 0;
                        if(device_info[0].curtain_state == 0){
                            if(!device_info[0].device_state){
                                val = CURTAIN_STOP;
                            }else{
                                val = CURTAIN_OPEN;
                            }
                        }else if(device_info[0].curtain_state == 1){
                            if(!device_info[1].device_state){
                                val = CURTAIN_STOP;
                            }else{
                                val = CURTAIN_CLOSE;
                            }
                        }
                        nuos_report_curtain_blind_state(0, val);
                    #else
                        // if(index<2) index = 0;
                        // else index = 1;                
                        nuos_set_state_attribute(index);
                    #endif    
                #endif
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4T_ANALOG_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_ANALOG_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_PHASE_CUT_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_CUSTOM)
                nuos_set_state_attribute(index);
                
                if(device_info[index].device_state){
                    nuos_set_level_attribute(index);
                }
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN)
                if(index == FAN_INDEX){ nuos_set_fan_attribute(FAN_INDEX); }
                else if(index == LIGHT_INDEX) nuos_set_state_attribute(LIGHT_INDEX);
                else nuos_set_fan_attribute(FAN_INDEX);
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN_CUSTOM)
                if(index == LIGHT_INDEX) nuos_set_state_attribute(LIGHT_INDEX);
                else { 
                    nuos_set_state_attribute(FAN_INDEX); 
                    nuos_set_fan_attribute(FAN_INDEX);    
                }   
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)    
                nuos_set_thermostat_attribute(index);
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM) 
                nuos_set_state_attribute(0);
                if(device_info[index].device_state){
                    // printf("==========================level------------------\n");
                    //nuos_set_color_temp_mode_attribute(index);
                    #ifdef USE_TUYA_BRDIGE
                        nuos_set_color_temp_level_attribute(index);
                    #else
                        nuos_set_level_attribute(0);
                    #endif
                    nuos_set_color_temperature_attribute(0);
                }  
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
                    nuos_set_color_rgb_mode_attribute(0, selected_color_mode);
                    #ifdef USE_TUYA_BRDIGE
                    if(selected_color_mode == 0){
                        printf("STATE:%d index:%d\n", device_info[3].device_state, 3);
                        nuos_set_state_attribute_rgb(3);
                        if(device_info[3].device_state){
                            nuos_set_level_attribute(3);
                            nuos_set_color_temp_attribute(3);
                        }
                    }else if(selected_color_mode == 1){
                        // if(!device_info[0].device_state) dmx_data[dmx_start_address] = 0;
                        // else dmx_data[dmx_start_address] = device_info[0].device_level;
                        // if(!device_info[1].device_state) dmx_data[dmx_start_address+1] = 0;
                        // else dmx_data[dmx_start_address+1] = device_info[1].device_level;
                        // if(!device_info[2].device_state) dmx_data[dmx_start_address+2] = 0;
                        // else dmx_data[dmx_start_address+2] = device_info[2].device_level;                                                

                        nuos_set_state_attribute_rgb(4); 
                        if(device_info[4].device_state){  
                            rgb_t rgb = {dmx_data[dmx_start_address], dmx_data[dmx_start_address+1], dmx_data[dmx_start_address+2]}; // Example RGB values
                            
                            hsv_t hsv2 = rgb_to_hsv(rgb);
                            device_info[4].device_val = hsv2.v;
                            // device_info[4].light_color_x = hsv2.h;
                            // device_info[4].light_color_y = hsv2.s;
                            nuos_set_color_xy_attribute(4, &hsv2);
                        }
                    }
                    #else
                    if(selected_color_mode == 0){
                        nuos_set_state_attribute_rgb(3); 
                        nuos_set_level_attribute(3);                         
                        nuos_set_color_temp_attribute(3);
                    }else if(selected_color_mode == 1){
                        nuos_set_state_attribute_rgb(4); 
                        nuos_set_level_attribute(4);

                        if(!device_info[0].device_state) dmx_data[dmx_start_address] = 0;
                        else dmx_data[dmx_start_address] = device_info[0].device_level;
                        if(!device_info[1].device_state) dmx_data[dmx_start_address+1] = 0;
                        else dmx_data[dmx_start_address+1] = device_info[1].device_level;
                        if(!device_info[2].device_state) dmx_data[dmx_start_address+2] = 0;
                        else dmx_data[dmx_start_address+2] = device_info[2].device_level;                                                

                        rgb_t rgb = {dmx_data[dmx_start_address], dmx_data[dmx_start_address+1], dmx_data[dmx_start_address+2]}; // Example RGB values
                        hsv_t hsv2 = rgb_to_hsv(rgb);         
                        nuos_set_color_xy_attribute(4, &hsv2); 
                    }
                    #endif
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI)
                nuos_set_state_attribute(index);
                // if(device_info[index].device_state){
                //     nuos_set_level_attribute(index);
                // }  
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI)
            if(scene_group_switch_info.control_type == 2) { //scene control
                for(int i=0; i<TOTAL_ENDPOINTS; i++) {
                    if(i == index) {
                        device_info[i].device_state = true;
                    } else {
                        device_info[i].device_state = false;
                    }
                    nuos_set_state_attribute(i); 
                }   
            } else{
                nuos_set_state_attribute(index);
            }
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_DALI_DIRECT_SWITCH) 
                #ifdef USE_COLOR_CONTROL
                    // nuos_set_state_command(index);
                    // if(device_info[index].device_state){
                    //     nuos_set_level_command(index);
                    // } 
                    nuos_set_state_command(index, device_info[index].device_state);
                    if(device_info[index].device_state){
                        nuos_set_level_command(index, device_info[index].device_level);
                        nuos_set_color_temp_command(index, device_info[index].device_val);
                    }
                #else
                    nuos_set_state_attribute(index);
                    if(device_info[index].device_state){
                        nuos_set_level_attribute(index);
                    }
                #endif
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_MOTION || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_CONTACT_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_GAS_LEAK)   
                nuos_set_sensor_motion_attribute(index);
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY)   
                if(index == 0) nuos_set_sensor_temperature_attribute(index);
                else if(index == 1) nuos_set_sensor_humidity_attribute(index);
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY_CUSTOM)     
                nuos_set_sensor_temperature_hunidity_attribute(index);      
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_LUX)   
                nuos_set_sensor_illuminance_attribute(index);
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH)   
                nuos_set_scene_button_attribute(index);
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_GROUP_SWITCH)  
                // nuos_zb_group_send_light_onoff_groupcast(zb_scene_info[index].group_id, toggle_on_off);
                // nuos_zb_set_hardware(n_index, true);
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RINGING_BELL_2)   
                nuos_set_sensor_motion_attribute(1);
                vTaskDelay(pdMS_TO_TICKS(500));
                nuos_set_sensor_motion_attribute(0);              
            #endif
        }else{
            //printf("is_my_device_commissionned2: %d\n", is_my_device_commissionned);
        }
}


void nuos_set_zigbee_level_attribute(uint8_t index){
    //recheckTimer();
     
    //if(timer3_running_flag){
        if(is_my_device_commissionned && !is_some_device_unavailable){
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4R_ON_OFF_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2R_ON_OFF_LIGHT)
  
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2CH_CURTAIN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN_SWITCH)
                
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4T_ANALOG_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_ANALOG_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_PHASE_CUT_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_CUSTOM)
                if(device_info[index].device_state){
                    nuos_set_level_attribute(index);
                }
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN)

            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN_CUSTOM)
  
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)    

            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM) 
                if(device_info[index].device_state){
                    // printf("==========================level------------------\n");
                    //nuos_set_color_temp_mode_attribute(index);
                    #ifdef USE_TUYA_BRDIGE
                        nuos_set_color_temp_level_attribute(index);
                    #else
                        nuos_set_level_attribute(0);
                    #endif
                }  
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)

                    #ifdef USE_TUYA_BRDIGE

                    #else

                    if(selected_color_mode == 0){
                        nuos_set_level_attribute(3);                         
                    }else if(selected_color_mode == 1){     
                        nuos_set_level_attribute(4);    
                    }
                    #endif

            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI)
                // if(device_info[index].device_state){
                //     nuos_set_level_attribute(index);
                // }                  
            #endif
        }else{
            //printf("is_my_device_commissionned2: %d\n", is_my_device_commissionned);
        }
        is_some_device_unavailable = false;
    //}
}


void nuos_set_zigbee_color_attribute(uint8_t index){
    //recheckTimer();
    //if(timer3_running_flag){
        if(is_my_device_commissionned && !is_some_device_unavailable){
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4R_ON_OFF_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2R_ON_OFF_LIGHT)

            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2CH_CURTAIN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN_SWITCH)
    
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4T_ANALOG_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_ANALOG_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_PHASE_CUT_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_CUSTOM)

            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN)

            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN_CUSTOM)
  
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)    

            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM) 
                if(device_info[index].device_state){
                    nuos_set_color_temperature_attribute(0);
                }  
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
                //printf("SELECTED_MODE--------->%d\n", selected_color_mode);
                nuos_set_color_rgb_mode_attribute(0, selected_color_mode);
                // #if(USE_COLOR_DEVICE == COLOR_RGB_ONLY) 
                //     if(!device_info[0].device_state) dmx_data[dmx_start_address] = 0;
                //     else dmx_data[dmx_start_address] = device_info[0].device_level;
                //     if(!device_info[1].device_state) dmx_data[dmx_start_address+1] = 0;
                //     else dmx_data[dmx_start_address+1] = device_info[1].device_level;
                //     if(!device_info[2].device_state) dmx_data[dmx_start_address+2] = 0;
                //     else dmx_data[dmx_start_address+2] = device_info[2].device_level;                                                

                //     rgb_t rgb = {dmx_data[dmx_start_address], dmx_data[dmx_start_address+1], dmx_data[dmx_start_address+2]}; // Example RGB values
                //     hsv_t hsv2 = rgb_to_hsv(rgb);  
                //     nuos_set_state_attribute_rgb(3);                  
                //     nuos_set_color_xy_attribute(4, &hsv2);    
                // #else
                    #ifdef USE_TUYA_BRDIGE
                    if(selected_color_mode == 0){
                        nuos_set_color_temp_attribute(3);
                        nuos_set_state_attribute_rgb(4); 
                    }else if(selected_color_mode == 1){
                        if(!device_info[0].device_state) dmx_data[dmx_start_address] = 0;
                        else dmx_data[dmx_start_address] = device_info[0].device_level;
                        if(!device_info[1].device_state) dmx_data[dmx_start_address+1] = 0;
                        else dmx_data[dmx_start_address+1] = device_info[1].device_level;
                        if(!device_info[2].device_state) dmx_data[dmx_start_address+2] = 0;
                        else dmx_data[dmx_start_address+2] = device_info[2].device_level;                                                

                        rgb_t rgb = {dmx_data[dmx_start_address], dmx_data[dmx_start_address+1], dmx_data[dmx_start_address+2]}; // Example RGB values
                        hsv_t hsv2 = rgb_to_hsv(rgb);  
                        nuos_set_state_attribute_rgb(4); 
                        device_info[4].device_val = hsv2.v;   
                        // device_info[4].light_color_x = hsv2.h;
                        // device_info[4].light_color_y = hsv2.s;                
                        nuos_set_color_xy_attribute(4, &hsv2);
                    }
                    #else

                    if(selected_color_mode == 0){                       
                        nuos_set_color_temp_attribute(3); 
                    }else if(selected_color_mode == 1){
                        if(!device_info[0].device_state) dmx_data[dmx_start_address] = 0;
                        else dmx_data[dmx_start_address] = device_info[0].device_level;
                        if(!device_info[1].device_state) dmx_data[dmx_start_address+1] = 0;
                        else dmx_data[dmx_start_address+1] = device_info[1].device_level;
                        if(!device_info[2].device_state) dmx_data[dmx_start_address+2] = 0;
                        else dmx_data[dmx_start_address+2] = device_info[2].device_level;                                                

                        rgb_t rgb = {dmx_data[dmx_start_address], dmx_data[dmx_start_address+1], dmx_data[dmx_start_address+2]}; // Example RGB values
                        hsv_t hsv2 = rgb_to_hsv(rgb);          
                        nuos_set_color_xy_attribute(4, &hsv2);
                    }
                    #endif
                // #endif             
            #endif
        }else{
            //printf("is_my_device_commissionned2: %d\n", is_my_device_commissionned);
        }
        is_some_device_unavailable = false;
    //}
}

void nuos_zb_init_motion_sensor(){
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_MOTION || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_CONTACT_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_GAS_LEAK || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_LUX)
        init_sensor_commissioning_task();
    #endif 
}
/*********************************************************************************** */
esp_err_t nuos_driver_init(void)
{
    //timer3_running_flag = true;
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4R_ON_OFF_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2R_ON_OFF_LIGHT)
        for(int i=0; i<TOTAL_ENDPOINTS; i++)
           nuos_zb_set_hardware(i, 0);
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN_SWITCH) 
        #ifdef TUYA_ATTRIBUTES
            printf("===========================\n");
            printf("dev_val: %d dev_lev: %d\n", device_info[0].device_val, device_info[0].device_level);
            set_curtain_percentage(device_info[0].device_level, true);
        #else
            for(int i=0; i<TOTAL_ENDPOINTS; i++){
                nuos_zb_set_hardware_curtain(i, false); 
            }  
        #endif
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN_SWITCH)
            init_curtain_timer();
        #endif
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2CH_CURTAIN)
        for(int i=0; i<TOTAL_ENDPOINTS; i++){
            uint8_t m_index = 255;
            if(i == 0){
                if(!device_info[i].device_state) m_index = 0;
                else m_index = 1;
            }else if(i == 1){ 
                if(!device_info[i].device_state) m_index = 2;
                else m_index = 3;
            }
            if(m_index != 255 )
            nuos_zb_set_hardware(m_index, false);            
        }
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4T_ANALOG_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_ANALOG_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_PHASE_CUT_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_CUSTOM)
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_CUSTOM)
        init_dali_hw(); 
        #endif
        for(int i=0; i<TOTAL_ENDPOINTS; i++){
            nuos_zb_set_hardware(i, false);
        }
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4T_COLOR_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_COLOR_DIMMABLE_LIGHT)
        for(int i=0; i<TOTAL_ENDPOINTS; i++)
           nuos_zb_set_hardware(i, 0);      
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN)
        for(int i=0; i<TOTAL_ENDPOINTS; i++){
            if(i==LIGHT_INDEX) nuos_zb_set_hardware(LIGHT_INDEX, 0);
            else nuos_set_hardware_fan_ctrl(FAN_INDEX);  
        };
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN_CUSTOM)
        for(int i=0; i<TOTAL_ENDPOINTS; i++){
            if(i==LIGHT_INDEX) nuos_zb_set_hardware(LIGHT_INDEX, 0);
            else nuos_set_hardware_fan_ctrl(FAN_INDEX);  
        }               
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM) 
        nuos_init_ir_ac(0);
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
        init_dali_hw(); 
        nuos_zb_set_hardware(0, false); //
        set_state(0);
        // set_level();
        set_dali_color_temp(0, false); 

    #elif (USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI)
        init_dali_hw(); 
        nuos_zb_set_hardware(0, false);  
    #elif (USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI)    
        init_dali_hw();
        printf("Scene Control Type: %d Selected ID: %d\n", scene_group_switch_info.control_type, scene_group_switch_info.selected_id);
        if(scene_group_switch_info.control_type == 0 || (scene_group_switch_info.control_type == 1)){ //individual or group ctrl
            for(int i=0; i<TOTAL_ENDPOINTS; i++){
                nuos_zb_set_hardware(i, false);  
            }
        }else if(scene_group_switch_info.control_type == 2) { //scene ctrl
            nuos_zb_set_hardware(scene_group_switch_info.selected_id, false);  
        }
    #elif (USE_NUOS_ZB_DEVICE_TYPE == DEVICE_DALI_DIRECT_SWITCH)    
        init_dali_hw();
        for(int i=0; i<TOTAL_ENDPOINTS; i++){
            nuos_zb_set_hardware(i, false);  
        }
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX) 
        init_dali_hw(); 
        if(selected_color_mode == 0){
            nuos_zb_set_hardware(3, false);  
            #if(USE_NUOS_ZB_DEVICE_TYPE != DEVICE_GROUP_DALI)
            set_state(3);
            set_dali_level(3);
            #endif
        }else if(selected_color_mode == 1){
            nuos_zb_set_hardware(4, false);  
            #if(USE_NUOS_ZB_DEVICE_TYPE != DEVICE_GROUP_DALI)
            set_state(4);
            #endif
        }
        #if(USE_NUOS_ZB_DEVICE_TYPE != DEVICE_GROUP_DALI)
        set_dali_color_temp(0, false);
        #endif
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_MOTION || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_CONTACT_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_GAS_LEAK) 
        init_sensor_interrupt();
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY)
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY_CUSTOM)       
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_LUX)
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_GROUP_SWITCH)
        for(int i=0; i<TOTAL_ENDPOINTS; i++){
            if(device_info[i].device_state){
                nuos_zb_set_hardware(i, 0); 
                break;
            } 
        }
                  
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH)
        for(int i=0; i<TOTAL_ENDPOINTS; i++)
           nuos_zb_set_hardware(i, 0);          
    #endif        
    return ESP_OK;
}

/*********************************************************************************** */
esp_err_t nuos_zigbee_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message){
    esp_err_t ret = ESP_OK;
    ret = nuos_zb_group_action_handler(callback_id, message);
    if(ret == -1){
       ret = nuos_zb_scene_action_handler(callback_id, message);
    }
    #ifdef USE_OTA
    if(ret == -1){
        ret = nuos_zb_ota_action_handler(callback_id, message);
    }
    #endif
    if(ret == -1){
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
    }

    
    return ret;
}
/*********************************************************************************** */
///////////////////////////////////Binding Functions/////////////////////////////
static void bind_fan_cb(esp_zb_zdp_status_t zdo_status, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Fan Clusters Bound successfully!");  

        // zigbee_info_t* on_off_light = ((zigbee_info_t *)user_ctx);
        // ESP_LOGI(TAG, "short_addr:0x%x", on_off_light->short_addr);
        // ESP_LOGI(TAG, "endpoint:0x%x", on_off_light->endpoint); 
        if(is_my_device_commissionned){    
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_SCENE_SWITCH)
            #else
            nuos_set_zigbee_attribute(endpoint_counts);
            #endif
        }
        endpoint_counts++;
        if(endpoint_counts >= TOTAL_ENDPOINTS){ 
            endpoint_counts = 0;      
        }  


    }
}

static void bind_thermostat_cb(esp_zb_zdp_status_t zdo_status, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Thermostat Clusters Bound successfully!");  
        if(is_my_device_commissionned){ 
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_SCENE_SWITCH)
            #else
            nuos_set_zigbee_attribute(endpoint_counts);
            #endif
        }
    }
}

static void bind_time_cluster_cb(esp_zb_zdp_status_t zdo_status, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Time Clusters Bound successfully!");

    }
}

static void bind_identify_cb(esp_zb_zdp_status_t zdo_status, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Identify Cluster Bound successfully!");        
        if (user_ctx) {
            uint8_t dst_endpoint = *((uint8_t*)user_ctx); 
        }
    }
}

static void bind_cb_curtain(esp_zb_zdp_status_t zdo_status, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Curtain Clusters Bound successfully!!");
        if(is_my_device_commissionned){ 
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_SCENE_SWITCH)
            #else
            nuos_set_zigbee_attribute(endpoint_counts); 
            #endif
        }
        endpoint_counts++;
        if(endpoint_counts >= TOTAL_ENDPOINTS){ 
            endpoint_counts = 0;      
        }   
    }
}

static void bind_cb_wireless_scene(esp_zb_zdp_status_t zdo_status, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Wireless Scene Clusters Bound successfully!");
        endpoint_counts++;
        if(endpoint_counts >= TOTAL_ENDPOINTS){ 
            endpoint_counts = 0;  
        }
    }
}

static void bind_cb_sensor(esp_zb_zdp_status_t zdo_status, void *user_ctx){
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Sensor Bound successfully!");   
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_SCENE_SWITCH)
        #else  
        nuos_set_zigbee_attribute(0);
        #endif
    }
}

static void bind_cb_remote(esp_zb_zdp_status_t zdo_status, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Remote Switch Bound successfully!");        
        if (user_ctx) {    
            endpoint_counts++;
            if(endpoint_counts >= TOTAL_ENDPOINTS){  
                endpoint_counts = 0;
            }
        }
    }
}


static void bind_color_cb(esp_zb_zdp_status_t zdo_status, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Color Clusters Bound successfully!");
        if(is_my_device_commissionned){ 
            
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM) 
            if(device_info[0].device_state){
                nuos_set_color_temperature_attribute(0);
            }
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
                        
            #ifdef USE_TUYA_BRDIGE
                if(selected_color_mode == 0){
                    if(device_info[3].device_state){
                        nuos_set_color_temp_attribute(3);
                    }
                }else if(selected_color_mode == 1){
                    if(!device_info[0].device_state) dmx_data[dmx_start_address] = 0;
                    else dmx_data[dmx_start_address] = device_info[0].device_level;
                    if(!device_info[1].device_state) dmx_data[dmx_start_address+1] = 0;
                    else dmx_data[dmx_start_address+1] = device_info[1].device_level;
                    if(!device_info[2].device_state) dmx_data[dmx_start_address+2] = 0;
                    else dmx_data[dmx_start_address+2] = device_info[2].device_level;                                                

                    if(device_info[4].device_state){  
                        rgb_t rgb = {dmx_data[dmx_start_address], dmx_data[dmx_start_address+1], dmx_data[dmx_start_address+2]}; // Example RGB values
                        hsv_t hsv2 = rgb_to_hsv(rgb);  
                        device_info[4].device_val = hsv2.v;  
                        // device_info[4].light_color_x = hsv2.h;
                        // device_info[4].light_color_y = hsv2.s;                                           
                        nuos_set_color_xy_attribute(4, &hsv2);
                    }
                }
            #else
                if(selected_color_mode == 0){                     
                    nuos_set_color_temp_attribute(3);
                }else if(selected_color_mode == 1){
                    if(!device_info[0].device_state) dmx_data[dmx_start_address] = 0;
                    else dmx_data[dmx_start_address] = device_info[0].device_level;
                    if(!device_info[1].device_state) dmx_data[dmx_start_address+1] = 0;
                    else dmx_data[dmx_start_address+1] = device_info[1].device_level;
                    if(!device_info[2].device_state) dmx_data[dmx_start_address+2] = 0;
                    else dmx_data[dmx_start_address+2] = device_info[2].device_level;                                                

                    rgb_t rgb = {dmx_data[dmx_start_address], dmx_data[dmx_start_address+1], dmx_data[dmx_start_address+2]}; // Example RGB values
                    hsv_t hsv2 = rgb_to_hsv(rgb);         
                    nuos_set_color_xy_attribute(4, &hsv2); 
                }
            #endif             
        #else
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_SCENE_SWITCH)
        #else          
            nuos_set_zigbee_attribute(0);
        #endif    
        #endif  
        }  
    }
}

static void bind_level_cb(esp_zb_zdp_status_t zdo_status, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Level Clusters Bound successfully!");
        if (user_ctx) {
            zigbee_info_t* info = (zigbee_info_t*)user_ctx;
            if(is_my_device_commissionned){ 
                

                #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM) 
                    if(device_info[0].device_state){
                        #ifdef USE_TUYA_BRDIGE
                            nuos_set_color_temp_level_attribute(0);
                        #else
                            nuos_set_level_attribute(0);
                        #endif
                        nuos_set_color_temperature_attribute(0);
                    }
                #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
                        
                    #ifdef USE_TUYA_BRDIGE
                        if(selected_color_mode == 0){
                            if(device_info[3].device_state){
                                nuos_set_level_attribute(3);
                            }
                        }else if(selected_color_mode == 1){

                        }
                    #else
                    if(selected_color_mode == 0){
                        nuos_set_level_attribute(3);                         
                    }else if(selected_color_mode == 1){
                        nuos_set_level_attribute(4);
                    }
                    #endif 
                #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_MOTION || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_CONTACT_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_GAS_LEAK || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY_CUSTOM|| USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_LUX)                        
                #else
                    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_SCENE_SWITCH)
                    #else                      
                    nuos_set_level_attribute(endpoint_counts);
                    #endif
                #endif 

            }
        }         
    }
}

void check_device_platform(uint16_t short_addr) 
{
    esp_zb_zcl_read_attr_cmd_t read_cmd;
    read_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
	read_cmd.zcl_basic_cmd.src_endpoint = 1;
	read_cmd.zcl_basic_cmd.dst_endpoint = 1;
	read_cmd.zcl_basic_cmd.dst_addr_u.addr_short = short_addr;
    read_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_BASIC;

    read_cmd.manuf_specific     = 0;              /*!< Sent as manufacturer extension with code. */
    read_cmd.direction          = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV;               /*!< The command direction, refer to esp_zb_zcl_cmd_direction_t */
    read_cmd.dis_default_resp   = 1;              /*!< Disable default response for this command. */
    read_cmd.manuf_code         = 0;


	uint16_t attributes[] = {
		ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
	};
	read_cmd.attr_number = 2;//ARRAY_LENTH(attributes);
	read_cmd.attr_field = attributes;
    
    esp_zb_zcl_read_attr_cmd_req(&read_cmd);
}

void match_desc_cb(esp_zb_zdp_status_t zdo_status, uint16_t addr, uint8_t endpoint, void *user_ctx) {
    
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Device supports Tuya cluster 0xEF00 -> it's a Tuya device");
        check_device_platform(0);
    } else {
        ESP_LOGI(TAG, "No Tuya cluster -> likely standard Zigbee or other");
    }
}
void check_for_tuya_cluster(uint16_t short_addr) {
    esp_zb_zdo_match_desc_req_param_t find_req;

    uint16_t cluster_list[] = {ESP_ZB_ZCL_CLUSTER_ID_BASIC};
    find_req.num_in_clusters = 1;
    find_req.num_out_clusters = 0;
    find_req.dst_nwk_addr = short_addr;
    find_req.addr_of_interest = short_addr;
    find_req.profile_id = ESP_ZB_AF_HA_PROFILE_ID;
    find_req.cluster_list = cluster_list;
    esp_zb_zdo_match_cluster(&find_req, match_desc_cb, NULL);
}




static void bind_cb(esp_zb_zdp_status_t zdo_status, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Light Bound successfully!");        
        if (user_ctx) {
            zigbee_info_t* info = (zigbee_info_t*)user_ctx;
            uint8_t dstep = info->endpoint;
            ESP_LOGI(TAG, "dst_endpoint: %d  short:0x%x", dstep, info->short_addr);

            if(is_my_device_commissionned){ 
                #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI)
                    #ifdef USE_TUYA_BRDIGE
                        nuos_set_color_rgb_mode_attribute(0, selected_color_mode);
                        if(selected_color_mode == 0)
                            nuos_set_state_attribute(3);
                        else
                            nuos_set_state_attribute(4);
                    #else
                        if(selected_color_mode == 0){
                            nuos_set_state_attribute_rgb(3); 
                        }else if(selected_color_mode == 1){
                            nuos_set_state_attribute_rgb(4); 
                        }
                    #endif 
                #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM) 
                        if(dstep == 1)nuos_set_thermostat_attribute(0);
                #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM) 
                    nuos_set_state_attribute(0);
                #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
                        
                        #ifdef USE_TUYA_BRDIGE
                        if(selected_color_mode == 0){
                            printf("STATE:%d index:%d\n", device_info[3].device_state, 3);
                            nuos_set_state_attribute_rgb(3);
                            if(device_info[3].device_state){
                                nuos_set_level_attribute(3);
                                nuos_set_color_temp_attribute(3);
                            }
                        }else if(selected_color_mode == 1){
                            if(!device_info[0].device_state) dmx_data[dmx_start_address] = 0;
                            else dmx_data[dmx_start_address] = device_info[0].device_level;
                            if(!device_info[1].device_state) dmx_data[dmx_start_address+1] = 0;
                            else dmx_data[dmx_start_address+1] = device_info[1].device_level;
                            if(!device_info[2].device_state) dmx_data[dmx_start_address+2] = 0;
                            else dmx_data[dmx_start_address+2] = device_info[2].device_level;                                                
    
                            nuos_set_state_attribute_rgb(4); 
                            if(device_info[4].device_state){  
                                rgb_t rgb = {dmx_data[dmx_start_address], dmx_data[dmx_start_address+1], dmx_data[dmx_start_address+2]}; // Example RGB values
                                hsv_t hsv2 = rgb_to_hsv(rgb);                                             
                                nuos_set_color_xy_attribute(4, &hsv2);
                            }
                        }
                        #else
                        if(selected_color_mode == 0){
                            nuos_set_state_attribute_rgb(3); 
                            nuos_set_level_attribute(3);                         
                            nuos_set_color_temp_attribute(3);
                        }else if(selected_color_mode == 1){
                            nuos_set_state_attribute_rgb(4); 
                            nuos_set_level_attribute(4);
    
                            if(!device_info[0].device_state) dmx_data[dmx_start_address] = 0;
                            else dmx_data[dmx_start_address] = device_info[0].device_level;
                            if(!device_info[1].device_state) dmx_data[dmx_start_address+1] = 0;
                            else dmx_data[dmx_start_address+1] = device_info[1].device_level;
                            if(!device_info[2].device_state) dmx_data[dmx_start_address+2] = 0;
                            else dmx_data[dmx_start_address+2] = device_info[2].device_level;                                                
    
                            rgb_t rgb = {dmx_data[dmx_start_address], dmx_data[dmx_start_address+1], dmx_data[dmx_start_address+2]}; // Example RGB values
                            hsv_t hsv2 = rgb_to_hsv(rgb);         
                            nuos_set_color_xy_attribute(4, &hsv2); 
                        }
                        #endif 
                #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_MOTION || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_CONTACT_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_GAS_LEAK || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_LUX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY_CUSTOM)
                #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_SCENE_SWITCH) 
                #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_DALI_DIRECT_SWITCH)
                    nuos_set_state_attribute(endpoint_counts); 
                    if(device_info[endpoint_counts].device_state){
                        nuos_set_level_attribute(endpoint_counts);
                    }
                #else 
                    #if (USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI)
                        if(scene_group_switch_info.control_type == 2) {
                            if(endpoint_counts == scene_group_switch_info.selected_id) {
                                nuos_set_state_attribute(endpoint_counts); 
                            }
                        }else{
                            nuos_set_state_attribute(endpoint_counts); 
                            // #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_DALI_DIRECT_SWITCH)
                            // if(device_info[endpoint_counts].device_state){
                            //     nuos_set_level_attribute(endpoint_counts);
                            // }
                            // #endif
                        }
                    #else
                        nuos_set_state_attribute(endpoint_counts);     
                    #endif                     
                #endif
            }  
            endpoint_counts++;
            if(endpoint_counts >= TOTAL_ENDPOINTS){ 
                endpoint_counts = 0;
                printf("check_device_platform\n");
                //check_device_platform(0);
                check_for_tuya_cluster(0);
            }

            
        }
    }
}
static void unbind_cb(esp_zb_zdp_status_t zdo_status, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Light Unbind successfully!");        
        if (user_ctx) {
            uint8_t dst_endpoint = *((uint8_t*)user_ctx); 
            ESP_LOGI(TAG, " on DST_EP:%d", dst_endpoint);  
  
            endpoint_counts++;
            if(endpoint_counts >= TOTAL_ENDPOINTS){ 
                endpoint_counts = 0;     
            }
        }
    }
}

// Function to send the next bind request
void send_next_bind_request() {
    if (current_index >= TOTAL_BIND_REQUESTS) {
        // All requests done
        printf("All requests are Done!! current_index:%d\n", current_index);
        return;
    }

    int endpoint_idx = current_index / 2;
    bool is_level = (current_index % 2 == 0);
    // printf("endpoint_idx:%d\n", endpoint_idx);
    // printf("is_level:%d\n", is_level);
    esp_zb_zdo_bind_req_param_t bind_req;
    print_ieee_addr(light_zb_info[0].ieee_addr);
    memcpy(&(bind_req.src_address), light_zb_info[0].ieee_addr, sizeof(esp_zb_ieee_addr_t));
    esp_zb_ieee_address_by_short(light_zb_info[endpoint_idx].short_addr, light_zb_info[endpoint_idx].ieee_addr);
    esp_zb_get_long_address(bind_req.src_address);
    bind_req.src_endp = ENDPOINTS_LIST[endpoint_idx];
    bind_req.dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
    memcpy(bind_req.dst_address_u.addr_long, light_zb_info[0].ieee_addr, sizeof(esp_zb_ieee_addr_t));
    bind_req.dst_endp = light_zb_info[endpoint_idx].endpoint;
    bind_req.req_dst_addr = esp_zb_get_short_address();

    if (is_level) {
        bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL;
        esp_zb_zdo_device_bind_req(&bind_req, bind_level_cb1, (void *)&light_zb_info[endpoint_idx]);
    } else {
        bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
        esp_zb_zdo_device_bind_req(&bind_req, bind_cb1, (void *)&light_zb_info[endpoint_idx]);
    }
}

// Callback example
void bind_cb1(esp_zb_zdp_status_t zdo_status, void *user_ctx) {
    // Handle result...
    current_index++;
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        int endpoint_idx = current_index / 2;
        ESP_LOGI(TAG, "Light State %d Bound successfully!", endpoint_idx);  
        if(endpoint_idx < TOTAL_ENDPOINTS)
        nuos_set_state_attribute(endpoint_idx); 
    }
    
    send_next_bind_request();
}

void bind_level_cb1(esp_zb_zdp_status_t zdo_status, void *user_ctx) {
    // Handle result...
    current_index++;
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        int endpoint_idx = current_index / 2;
        ESP_LOGI(TAG, "Light Level %d Bound successfully!", endpoint_idx); 
        if(endpoint_idx < TOTAL_ENDPOINTS) 
        nuos_set_level_attribute(endpoint_idx);
    }
    send_next_bind_request();
}

// Start the process somewhere in your code
void start_binding_process() {
    current_index = 0;
    send_next_bind_request();
}

void nuos_zb_bind_cb(esp_zb_zdp_status_t zdo_status, esp_zb_ieee_addr_t ieee_addr, void *user_ctx)
{

    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI) 
        #ifdef USE_HOME_ASSISTANT
            #ifdef USE_INDIVIDUAL_DALI_ADDRESSING
                current_index = 0;
                esp_zb_zdo_bind_req_param_t bind_req;
                print_ieee_addr(ieee_addr);
                memcpy(light_zb_info[0].ieee_addr, ieee_addr, sizeof(esp_zb_ieee_addr_t));
                memcpy(&(bind_req.src_address), ieee_addr, sizeof(esp_zb_ieee_addr_t));
                esp_zb_ieee_address_by_short(light_zb_info[0].short_addr, light_zb_info[0].ieee_addr);
                esp_zb_get_long_address(bind_req.src_address);
                bind_req.src_endp = ENDPOINTS_LIST[0];
                bind_req.dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
                memcpy(bind_req.dst_address_u.addr_long, ieee_addr, sizeof(esp_zb_ieee_addr_t));
                
                bind_req.dst_endp = light_zb_info[0].endpoint;
                bind_req.req_dst_addr = esp_zb_get_short_address();

                bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL;
                ESP_LOGI(TAG, "Try to bind Light %d Level", light_zb_info[0].endpoint);            
                esp_zb_zdo_device_bind_req(&bind_req, bind_level_cb1, (void *)&light_zb_info[0]);
            #else
                for(int index=0; index<TOTAL_ENDPOINTS; index++) {
                    esp_zb_zdo_bind_req_param_t bind_req;
                    print_ieee_addr(ieee_addr);
                    memcpy(&(bind_req.src_address), ieee_addr, sizeof(esp_zb_ieee_addr_t));
                    esp_zb_ieee_address_by_short(light_zb_info[index].short_addr, light_zb_info[index].ieee_addr);
                    esp_zb_get_long_address(bind_req.src_address);
                    bind_req.src_endp = ENDPOINTS_LIST[index];
                    bind_req.dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
                    memcpy(bind_req.dst_address_u.addr_long, ieee_addr, sizeof(esp_zb_ieee_addr_t));
                    bind_req.dst_endp = light_zb_info[index].endpoint;
                    bind_req.req_dst_addr = esp_zb_get_short_address();
                    
            
                    bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL;
                    ESP_LOGI(TAG, "Try to bind Light %d Level", light_zb_info[0].endpoint);
                    esp_zb_zdo_device_bind_req(&bind_req, bind_level_cb, (void *)&light_zb_info[0]);
                    bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
                    ESP_LOGI(TAG, "Try to bind Light %d On/Off", light_zb_info[0].endpoint);
                    esp_zb_zdo_device_bind_req(&bind_req, bind_cb, (void *)&light_zb_info[0]);
                } 
            #endif
        #endif
    #else
    // print_ieee_addr(ieee_addr);
    for(int index=0; index<TOTAL_ENDPOINTS; index++) {
        esp_zb_zdo_bind_req_param_t bind_req;
        print_ieee_addr(ieee_addr);
        memcpy(&(bind_req.src_address), ieee_addr, sizeof(esp_zb_ieee_addr_t));
        esp_zb_ieee_address_by_short(light_zb_info[index].short_addr, light_zb_info[index].ieee_addr);
        esp_zb_get_long_address(bind_req.src_address);
        bind_req.src_endp = ENDPOINTS_LIST[index];
        bind_req.dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
        memcpy(bind_req.dst_address_u.addr_long, ieee_addr, sizeof(esp_zb_ieee_addr_t));
        bind_req.dst_endp = light_zb_info[index].endpoint;
        bind_req.req_dst_addr = esp_zb_get_short_address();

        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4R_ON_OFF_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2R_ON_OFF_LIGHT)
            bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
            ESP_LOGI(TAG, "Try to bind Light %d On/Off", light_zb_info[index].endpoint);
            esp_zb_zdo_device_bind_req(&bind_req, bind_cb, (void *)&light_zb_info[index]);
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2CH_CURTAIN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN_SWITCH)
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN_SWITCH)
                bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING;
                ESP_LOGI(TAG, "Try to bind Curtain %d Window Covering", light_zb_info[index].endpoint);
                esp_zb_zdo_device_bind_req(&bind_req, bind_cb_curtain, (void *)&light_zb_info[index]);
            #else
            #ifdef TUYA_ATTRIBUTES
                bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING;
                ESP_LOGI(TAG, "Try to bind Curtain %d Window Covering", light_zb_info[index].endpoint);
                esp_zb_zdo_device_bind_req(&bind_req, bind_cb_curtain, (void *)&light_zb_info[index]);
            #else
                bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
                ESP_LOGI(TAG, "Try to bind Curtain %d OnOff", light_zb_info[index].endpoint);
                esp_zb_zdo_device_bind_req(&bind_req, bind_cb_curtain, (void *)&light_zb_info[index]);    
            #endif
            #endif

                    

        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4T_ANALOG_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_ANALOG_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_PHASE_CUT_DIMMABLE_LIGHT)
            bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL;
            ESP_LOGI(TAG, "Try to bind Light %d Dimmmer", light_zb_info[index].endpoint);
            esp_zb_zdo_device_bind_req(&bind_req, bind_level_cb, (void *)&light_zb_info[index]);
            bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
            ESP_LOGI(TAG, "Try to bind Light %d On/Off", light_zb_info[index].endpoint);
            esp_zb_zdo_device_bind_req(&bind_req, bind_cb, (void *)&light_zb_info[index]);
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4T_COLOR_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_COLOR_DIMMABLE_LIGHT)
            bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL;
            ESP_LOGI(TAG, "Try to bind Light %d Color", light_zb_info[index].endpoint);
            esp_zb_zdo_device_bind_req(&bind_req, bind_color_cb, (void *)&light_zb_info[index]);
            bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL;
            ESP_LOGI(TAG, "Try to bind Light %d Level", light_zb_info[index].endpoint);
            esp_zb_zdo_device_bind_req(&bind_req, bind_level_cb, (void *)&light_zb_info[index]);
            bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
            ESP_LOGI(TAG, "Try to bind Light %d On/Off", light_zb_info[index].endpoint);
            esp_zb_zdo_device_bind_req(&bind_req, bind_cb, (void *)&light_zb_info[index]);

        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN)
            ESP_LOGI(TAG, "Try to bind Light %d On/Off", light_zb_info[index].endpoint);
            #ifdef USE_TUYA_BRDIGE
            // bind_req.cluster_id = ENDPOINT_CLUSTERS[1];
            // esp_zb_zdo_device_bind_req(&bind_req, bind_fan_cb, (void *)&light_zb_info[index]);
            bind_req.cluster_id = ENDPOINT_CLUSTERS[0];
            esp_zb_zdo_device_bind_req(&bind_req, bind_cb, (void *)&light_zb_info[index]);
            #else
            if(index == FAN_INDEX){
                bind_req.cluster_id = ENDPOINT_CLUSTERS[1];
                esp_zb_zdo_device_bind_req(&bind_req, bind_fan_cb, (void *)&light_zb_info[index]);
            }else{
                bind_req.cluster_id = ENDPOINT_CLUSTERS[0];
                esp_zb_zdo_device_bind_req(&bind_req, bind_cb, (void *)&light_zb_info[index]);
            }
            #endif 
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN_CUSTOM)

            bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
            ESP_LOGI(TAG, "Try to bind Light %d On/Off", light_zb_info[index].endpoint);
            esp_zb_zdo_device_bind_req(&bind_req, bind_cb, (void *)&light_zb_info[index]);   

            //if(index == 1){
                bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL;
                ESP_LOGI(TAG, "Try to bind Light %d Dimmmer", light_zb_info[index].endpoint);
                esp_zb_zdo_device_bind_req(&bind_req, bind_level_cb, (void *)&light_zb_info[index]);
            //}                    
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_CUSTOM)
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_CUSTOM)

            #else

            bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL;
            ESP_LOGI(TAG, "Try to bind Light %d Color", light_zb_info[index].endpoint);
            esp_zb_zdo_device_bind_req(&bind_req, bind_color_cb, (void *)&light_zb_info[index]);
            #endif
            bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL;
            ESP_LOGI(TAG, "Try to bind Light %d Level", light_zb_info[index].endpoint);
            esp_zb_zdo_device_bind_req(&bind_req, bind_level_cb, (void *)&light_zb_info[index]);
            bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
            ESP_LOGI(TAG, "Try to bind Light %d On/Off", light_zb_info[index].endpoint);
            esp_zb_zdo_device_bind_req(&bind_req, bind_cb, (void *)&light_zb_info[index]);

            #ifdef USE_CCT_TIME_SYNC
            bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_TIME;
            ESP_LOGI(TAG, "Try to bind Time Cluster");
            esp_zb_zdo_device_bind_req(&bind_req, bind_time_cluster_cb, (void *)&light_zb_info[index]);
            #endif
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI) 
            #ifdef USE_HOME_ASSISTANT
            bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL;
            ESP_LOGI(TAG, "Try to bind Light %d Level", light_zb_info[index].endpoint);
            esp_zb_zdo_device_bind_req(&bind_req, bind_level_cb, (void *)&light_zb_info[index]);
            #endif
            bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
            ESP_LOGI(TAG, "Try to bind Light %d On/Off", light_zb_info[index].endpoint);
            esp_zb_zdo_device_bind_req(&bind_req, bind_cb, (void *)&light_zb_info[index]); 
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_DALI_DIRECT_SWITCH)
            #ifdef USE_COLOR_CONTROL 
            bind_req.cluster_id = 0xEF00;
            ESP_LOGI(TAG, "Try to bind Light %d On/Off", light_zb_info[index].endpoint);
            esp_zb_zdo_device_bind_req(&bind_req, bind_cb, (void *)&light_zb_info[index]);

            // bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL;
            // ESP_LOGI(TAG, "Try to bind Light %d Color", light_zb_info[index].endpoint);
            // esp_zb_zdo_device_bind_req(&bind_req, bind_color_cb, (void *)&light_zb_info[index]);
            #endif  
            bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL;
            ESP_LOGI(TAG, "Try to bind Light %d Dimmmer", light_zb_info[index].endpoint);
            esp_zb_zdo_device_bind_req(&bind_req, bind_level_cb, (void *)&light_zb_info[index]);

            bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
            ESP_LOGI(TAG, "Try to bind Light %d On/Off", light_zb_info[index].endpoint);
            esp_zb_zdo_device_bind_req(&bind_req, bind_cb, (void *)&light_zb_info[index]);    
            
            
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI) 

            // #ifdef DALI_DIRECT_ADDRESSING 
            //     bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL;
            //     ESP_LOGI(TAG, "Try to bind Light %d Dimmmer", light_zb_info[index].endpoint);
            //     esp_zb_zdo_device_bind_req(&bind_req, bind_level_cb, (void *)&light_zb_info[index]);
            // #endif  

            bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
            ESP_LOGI(TAG, "Try to bind Light %d On/Off", light_zb_info[index].endpoint);
            esp_zb_zdo_device_bind_req(&bind_req, bind_cb, (void *)&light_zb_info[index]); 
          
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER)
            if(index == 0 || index == 1){
                bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT;
                ESP_LOGI(TAG, "Try to bind Thermostat %d", light_zb_info[index].endpoint);
                esp_zb_zdo_device_bind_req(&bind_req, bind_cb, (void *)&light_zb_info[index]);    
            }else{
                bind_req.cluster_id = 0xFC31; // Custom Cluster;
                ESP_LOGI(TAG, "Try to bind Custom Cluster %d", light_zb_info[index].endpoint);
                esp_zb_zdo_device_bind_req(&bind_req, bind_cb, (void *)&light_zb_info[index]); 
            }
            // bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
            // ESP_LOGI(TAG, "Try to bind Light %d On/Off", light_zb_info[index].endpoint);
            // esp_zb_zdo_device_bind_req(&bind_req, bind_cb, (void *)&light_zb_info[index]); 
            #ifdef USE_TEMPERATURE_MESAUREMENT
                bind_req.cluster_id = ENDPOINT_CLUSTERS[index];
                ESP_LOGI(TAG, "Try to bind Thermostat Cluster on ep= %d", light_zb_info[index].endpoint);
                esp_zb_zdo_device_bind_req(&bind_req, bind_thermostat_cb, (void *)&light_zb_info[index]);
            #endif 
            #ifdef USE_FAN_CTRL
                bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL;
                ESP_LOGI(TAG, "Try to bind Fan Cluster on ep= %d", light_zb_info[index].endpoint);
                esp_zb_zdo_device_bind_req(&bind_req, bind_fan_cb, NULL);
            #endif
            #ifdef CUSTON_FAN_CTRL
                bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL;
                ESP_LOGI(TAG, "Try to bind Light %d Dimmmer", light_zb_info[index].endpoint);
                esp_zb_zdo_device_bind_req(&bind_req, bind_level_cb, (void *)&light_zb_info[index]);
                bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
                ESP_LOGI(TAG, "Try to bind Light %d On/Off", light_zb_info[index].endpoint);
                esp_zb_zdo_device_bind_req(&bind_req, bind_cb, (void *)&light_zb_info[index]);
            #endif
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
            bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL;
            ESP_LOGI(TAG, "Try to bind Light %d Dimmmer", light_zb_info[index].endpoint);
            esp_zb_zdo_device_bind_req(&bind_req, bind_level_cb, (void *)&light_zb_info[index]);
            bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
            ESP_LOGI(TAG, "Try to bind Light %d On/Off", light_zb_info[index].endpoint);
            esp_zb_zdo_device_bind_req(&bind_req, bind_cb, (void *)&light_zb_info[index]);   
            
            
            bind_req.cluster_id = ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_SEQUENCE_ID;
            ESP_LOGI(TAG, "Try to bind Endpoint %d Fan Speed", light_zb_info[index].endpoint);
            esp_zb_zdo_device_bind_req(&bind_req, NULL, NULL); 
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_MOTION || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_CONTACT_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_GAS_LEAK)
            bind_req.cluster_id = ENDPOINT_CLUSTERS[index];
            ESP_LOGI(TAG, "Try to bind IAS Sensor Cluster on ep= %d", light_zb_info[index].endpoint);
            esp_zb_zdo_device_bind_req(&bind_req, bind_cb_sensor, (void *)&light_zb_info[index]);  

            bind_req.cluster_id = 0xFC31; // Custom Cluster;
            ESP_LOGI(TAG, "Try to bind Custom Cluster %d", light_zb_info[index].endpoint);
            esp_zb_zdo_device_bind_req(&bind_req, bind_cb, (void *)&light_zb_info[index]);     
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY)
            bind_req.cluster_id = ENDPOINT_CLUSTERS[index];
            ESP_LOGI(TAG, "Try to bind Temperature Humidity Sensor Cluster on ep= %d", light_zb_info[index].endpoint);
            esp_zb_zdo_device_bind_req(&bind_req, bind_cb_sensor, (void *)&light_zb_info[index]);  
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY_CUSTOM)
            // bind_req.cluster_id = ENDPOINT_CLUSTERS[2];
            // ESP_LOGI(TAG, "Try to bind Light %d Dimmmer", light_zb_info[index].endpoint);
            // esp_zb_zdo_device_bind_req(&bind_req, bind_level_cb, (void *)&light_zb_info[index]);   

            bind_req.cluster_id = ENDPOINT_CLUSTERS[1];
            ESP_LOGI(TAG, "Try to bind Humidity Cluster on ep= %d", light_zb_info[index].endpoint);
            esp_zb_zdo_device_bind_req(&bind_req, bind_level_cb, (void *)&light_zb_info[index]);

            bind_req.cluster_id = ENDPOINT_CLUSTERS[0];
            ESP_LOGI(TAG, "Try to bind Temperature Cluster on ep= %d", light_zb_info[index].endpoint);
            esp_zb_zdo_device_bind_req(&bind_req, bind_cb, (void *)&light_zb_info[index]);            
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_LUX)
            bind_req.cluster_id = ENDPOINT_CLUSTERS[index];
            ESP_LOGI(TAG, "Try to bind Lux Sensor Cluster on ep= %d", light_zb_info[index].endpoint);
            esp_zb_zdo_device_bind_req(&bind_req, bind_cb, (void *)&light_zb_info[index]);
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_SCENE_SWITCH|| USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_GROUP_SWITCH)
            #ifndef USE_CUSTOM_SCENE
            bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG;
            ESP_LOGI(TAG, "Try to bind Scene Switch Cluster on ep= %d", light_zb_info[index].endpoint);
            esp_zb_zdo_device_bind_req(&bind_req, bind_cb_wireless_scene, (void *)&light_zb_info[index]); 
            #endif
            bind_req.cluster_id = ENDPOINT_CLUSTERS[index];
            ESP_LOGI(TAG, "Try to bind Scene Switch Cluster on ep= %d", light_zb_info[index].endpoint);
            esp_zb_zdo_device_bind_req(&bind_req, bind_cb, (void *)&light_zb_info[index]);            
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH) 
            bind_req.cluster_id = ENDPOINT_CLUSTERS[index];
            ESP_LOGI(TAG, "Try to bind Scene Switch Cluster on ep= %d", light_zb_info[index].endpoint);
            esp_zb_zdo_device_bind_req(&bind_req, bind_cb_remote, (void *)&light_zb_info[index]);   
        #elif(USE_NUOS_ZB_DEVICE_TYPE ==  DEVICE_RINGING_BELL_2) 
            bind_req.cluster_id = ENDPOINT_CLUSTERS[index];
            ESP_LOGI(TAG, "Try to bind IAS Sensor Cluster on ep= %d", light_zb_info[index].endpoint);
            esp_zb_zdo_device_bind_req(&bind_req, bind_cb, (void *)&light_zb_info[index]);                           
        #endif

    }
    #endif
}

void nuos_zb_request_ieee_address(esp_zb_zdp_status_t zdo_status, uint16_t addr, uint8_t endpoint, void *user_ctx, esp_zb_zdo_ieee_addr_callback_t ieee_cb){
    printf("FOUND EP's': ");
    for(int index=0; index<TOTAL_ENDPOINTS; index++){
        #ifdef USE_TUYA_BRDIGE
        light_zb_info[index].endpoint = endpoint+index;
        #else
        light_zb_info[index].endpoint = endpoint+index;
        #endif
        light_zb_info[index].short_addr = addr;
        printf(" %d ", light_zb_info[index].endpoint);
    }
    printf("\n");
    /* get the light ieee address */
    esp_zb_zdo_ieee_addr_req_param_t ieee_req;
    ieee_req.addr_of_interest = addr;
    ieee_req.dst_nwk_addr = addr;
    ieee_req.request_type = 0;
    ieee_req.start_index = 0;
    esp_zb_zdo_ieee_addr_req(&ieee_req, ieee_cb, NULL);
}

void nuos_zb_find_clusters(esp_zb_zdo_match_desc_callback_t user_cb){

    esp_zb_zdo_match_desc_req_param_t find_req;
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4R_ON_OFF_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2R_ON_OFF_LIGHT)
        uint16_t cluster_list[] = {ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_ID_OTA_UPGRADE};
        find_req.num_in_clusters = 1;
        find_req.num_out_clusters = 1;
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2CH_CURTAIN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN_SWITCH)
        //#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN_SWITCH)
            uint16_t cluster_list[] = {ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, ESP_ZB_ZCL_CLUSTER_ID_OTA_UPGRADE};
        //#else     
        //    uint16_t cluster_list[] = {ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_ID_OTA_UPGRADE};
        //#endif
        find_req.num_in_clusters = 5;
        find_req.num_out_clusters = 1;
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4T_ANALOG_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_ANALOG_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_PHASE_CUT_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_CUSTOM)
        uint16_t cluster_list[] = {ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL, ESP_ZB_ZCL_CLUSTER_ID_OTA_UPGRADE};
        find_req.num_in_clusters = 2;
        find_req.num_out_clusters = 0;
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4T_COLOR_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_COLOR_DIMMABLE_LIGHT)
        uint16_t cluster_list[] = {ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_CLUSTER_ID_OTA_UPGRADE};
        find_req.num_in_clusters = 3;
        find_req.num_out_clusters = 1;
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)  
        uint16_t cluster_list[] = {ESP_ZB_ZCL_CLUSTER_ID_TIME, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_CLUSTER_ID_OTA_UPGRADE};
        find_req.num_in_clusters = 4;
        find_req.num_out_clusters = 1;    
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI)   
        uint16_t cluster_list[] = {ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL, ESP_ZB_ZCL_CLUSTER_ID_OTA_UPGRADE};
        find_req.num_in_clusters = 2;
        find_req.num_out_clusters = 1;       
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN)
        uint16_t cluster_list[] = {ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL, ESP_ZB_ZCL_CLUSTER_ID_OTA_UPGRADE};
        find_req.num_in_clusters = 2;
        find_req.num_out_clusters = 1;
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN_CUSTOM)
        uint16_t cluster_list[] = {ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL, ESP_ZB_ZCL_CLUSTER_ID_OTA_UPGRADE};
        find_req.num_in_clusters = 2;
        find_req.num_out_clusters = 1; 
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
        uint16_t cluster_list[] = {ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL, ESP_ZB_ZCL_CLUSTER_ID_OTA_UPGRADE};
        find_req.num_in_clusters = 2;
        find_req.num_out_clusters = 1;                  
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER)
        #ifdef USE_FAN_CTRL
            uint16_t cluster_list[] = {ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT, ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL, ESP_ZB_ZCL_CLUSTER_ID_OTA_UPGRADE};
            find_req.num_in_clusters = 2;
            find_req.num_out_clusters = 1;             
        #else
            uint16_t cluster_list[] = {ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT, 0xFC31, ESP_ZB_ZCL_CLUSTER_ID_OTA_UPGRADE};
            find_req.num_in_clusters = 1;
            find_req.num_out_clusters = 1;             
        #endif
 
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_MOTION || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_CONTACT_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_GAS_LEAK)
        uint16_t cluster_list[] = {ESP_ZB_ZCL_CLUSTER_ID_IAS_ZONE, 0xFC31, ESP_ZB_ZCL_CLUSTER_ID_OTA_UPGRADE};
        find_req.num_in_clusters = 2;
        find_req.num_out_clusters = 1;  
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY)
        uint16_t cluster_list[] = {ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_ID_OTA_UPGRADE};
        find_req.num_in_clusters = 1;
        find_req.num_out_clusters = 1;
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY_CUSTOM)
        uint16_t cluster_list[] = {ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_ID_OTA_UPGRADE};
        find_req.num_in_clusters = 1;
        find_req.num_out_clusters = 1;        
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_LUX)
        uint16_t cluster_list[] = {ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_ID_IAS_ZONE, ESP_ZB_ZCL_CLUSTER_ID_OTA_UPGRADE};
        find_req.num_in_clusters = 1;
        find_req.num_out_clusters = 1;   
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_GROUP_SWITCH)
        uint16_t cluster_list[] = {ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_CLUSTER_ID_OTA_UPGRADE};
        find_req.num_in_clusters = 1;
        find_req.num_out_clusters = 1;                
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH)   
        uint16_t cluster_list[] = {ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_ID_OTA_UPGRADE};
        find_req.num_in_clusters = 1;
        find_req.num_out_clusters = 1; 
    #elif(USE_NUOS_ZB_DEVICE_TYPE ==  DEVICE_RINGING_BELL_2)  
        uint16_t cluster_list[] = {ESP_ZB_ZCL_CLUSTER_ID_IAS_ZONE, ESP_ZB_ZCL_CLUSTER_ID_OTA_UPGRADE};
        find_req.num_in_clusters = 1;
        find_req.num_out_clusters = 1;   
    #elif(USE_NUOS_ZB_DEVICE_TYPE ==  DEVICE_SCENE_DALI)  
        uint16_t cluster_list[] = {ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL, ESP_ZB_ZCL_CLUSTER_ID_OTA_UPGRADE};
        find_req.num_in_clusters = 2;
        find_req.num_out_clusters = 1;
    #else    
        uint16_t cluster_list[] = {ESP_ZB_ZCL_CLUSTER_ID_OTA_UPGRADE};
        find_req.num_in_clusters = 0;
        find_req.num_out_clusters = 1;
    #endif
    
    find_req.dst_nwk_addr = 0;
    find_req.addr_of_interest = 0x0000;
    find_req.profile_id = ESP_ZB_AF_HA_PROFILE_ID;
    find_req.cluster_list = cluster_list;
    if (is_my_device_commissionned) {
        printf("Matching Clusters.....\n");
        esp_zb_zdo_match_cluster(&find_req, user_cb, NULL);
    }  
    ready_commisioning_flag = false;
    start_commissioning = false;
    is_my_device_commissionned = !start_commissioning;

    setNVSCommissioningFlag(0);  //stop commissioning flag in NVS
    setNVSStartCommissioningFlag(0);
    

    // esp_zb_bdb_cancel_formation();
	// esp_zb_bdb_close_network();

    #ifdef USE_RGB_LED
        light_driver_set_power(0);
    #else
        nuos_zb_set_hardware_led_for_zb_commissioning(false);
    #endif
}

/////////////////////////////////CLUSTERS CREATION////////////////////////////////////////////

#ifdef USE_OTA


esp_zb_attribute_list_t* get_ota_cluster(){
    // #ifdef USE_HOME_ASSISTANT
        #define ESP_OTA_CLIENT_ENDPOINT             5 
        #define OTA_UPGRADE_RUNNING_FILE_VERSION    0x01010101
        #define OTA_UPGRADE_DOWNLOADED_FILE_VERSION ESP_ZB_ZCL_OTA_UPGRADE_DOWNLOADED_FILE_VERSION_DEF_VALUE
        #define OTA_UPGRADE_MANUFACTURER            0x1001
        #define OTA_UPGRADE_IMAGE_TYPE              0x1011
        #define OTA_UPGRADE_HW_VERSION              0x0101
        #define OTA_UPGRADE_MAX_DATA_SIZE           223

        esp_zb_ota_cluster_cfg_t ota_cluster_cfg = {
            .ota_upgrade_file_version = OTA_UPGRADE_RUNNING_FILE_VERSION,
            .ota_upgrade_downloaded_file_ver = OTA_UPGRADE_DOWNLOADED_FILE_VERSION,
            .ota_upgrade_manufacturer = OTA_UPGRADE_MANUFACTURER,
            .ota_upgrade_image_type = OTA_UPGRADE_IMAGE_TYPE,
        };
        esp_zb_attribute_list_t *esp_zb_ota_client_cluster = esp_zb_ota_cluster_create(&ota_cluster_cfg);
        
        esp_zb_zcl_ota_upgrade_client_variable_t variable_config = {
            .timer_query = ESP_ZB_ZCL_OTA_UPGRADE_QUERY_TIMER_COUNT_DEF,
            .hw_version = OTA_UPGRADE_HW_VERSION,
            .max_data_size = OTA_UPGRADE_MAX_DATA_SIZE,
        };
        
        uint16_t ota_upgrade_server_addr = 0xffff;
        uint8_t ota_upgrade_server_ep = 0xff;

        ESP_ERROR_CHECK(esp_zb_ota_cluster_add_attr(esp_zb_ota_client_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_CLIENT_DATA_ID, (void *)&variable_config));
        ESP_ERROR_CHECK(esp_zb_ota_cluster_add_attr(esp_zb_ota_client_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_SERVER_ADDR_ID, (void *)&ota_upgrade_server_addr));
        ESP_ERROR_CHECK(esp_zb_ota_cluster_add_attr(esp_zb_ota_client_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_SERVER_ENDPOINT_ID, (void *)&ota_upgrade_server_ep));
        return esp_zb_ota_client_cluster;
    }
#endif

esp_zb_attribute_list_t* nuos_zb_create_basic_cluster(uint8_t index) {
    uint16_t val = 0;
    uint8_t zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE;  
    uint16_t app_version = 78;//3;  
    uint16_t hw_version = 1;     

// Use stack-allocated buffers
    // char modelid[32];
    // snprintf(modelid, sizeof(modelid), "Gang%d", index+1);

    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_GROUP_SWITCH)                                    
        uint16_t power_source = 0x03;
    #else
        uint16_t power_source = 1; // Mains Device
    #endif
     
    uint8_t device_enabled = ESP_ZB_ZCL_BASIC_DEVICE_ENABLED_DEFAULT_VALUE;
    uint8_t date_code = ESP_ZB_ZCL_BASIC_DATE_CODE_DEFAULT_VALUE;

    esp_zb_basic_cluster_cfg_t  cfg = {
        .power_source = power_source,
        .zcl_version = zcl_version
    };

    /* basic cluster create with fully customized */
    // esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_basic_cluster_create(&cfg);
    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_APPLICATION_VERSION_ID, &app_version);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &zcl_version);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &power_source);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_DEVICE_ENABLED_ID, &device_enabled);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_DATE_CODE_ID, &val);
  
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, &modelid[0]);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, &manufname[0]);   

    return esp_zb_basic_cluster;
}

esp_zb_attribute_list_t* nuos_zb_create_power_cluster(){
        /* power cluster create with fully customized */
        uint8_t bat_quantity = 1;
        uint8_t bat_size = 1;
        uint16_t bat_voltage = 31;
        uint16_t bat_percentage = 200;
        esp_zb_attribute_list_t *esp_zb_power_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG);
        esp_zb_power_config_cluster_add_attr(esp_zb_power_cluster, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, &bat_voltage);
        esp_zb_power_config_cluster_add_attr(esp_zb_power_cluster, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID, &bat_percentage);
        // esp_zb_power_config_cluster_add_attr(esp_zb_power_cluster, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_QUANTITY_ID, &bat_quantity);
        // esp_zb_power_config_cluster_add_attr(esp_zb_power_cluster, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_SIZE_ID, &bat_size);
    return esp_zb_power_cluster;
}


esp_zb_attribute_list_t* nuos_zb_create_window_covering_cluster(){
    /* window covering cluster create with fully customized */
    /* Create window covering cluster */
    esp_zb_attribute_list_t *window_attr_list = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING);
    esp_zb_window_covering_cluster_cfg_t config = {
        .covering_type = ESP_ZB_ZCL_ATTR_WINDOW_COVERING_TYPE_ROLLERSHADE,
        .covering_status = ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CONFIG_OPERATIONAL,
        .covering_mode = ESP_ZB_ZCL_ATTR_WINDOW_COVERING_TYPE_MOTOR_IS_RUNNING_IN_MAINTENANCE_MODE,
    };
    esp_zb_cluster_add_attr(window_attr_list, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_WINDOW_COVERING_TYPE_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &config.covering_type);
    esp_zb_cluster_add_attr(window_attr_list, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CONFIG_STATUS_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_8BITMAP, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &config.covering_status);
    esp_zb_cluster_add_attr(window_attr_list, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_MODE_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_8BITMAP, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &config.covering_mode);

    uint8_t current_tilt_percentage = 0;
    uint16_t installed_open_limit_tilt = 0x0000;
    uint16_t installed_closed_limit_tilt = 0xffff;
    uint16_t current_position_tilt = ESP_ZB_ZCL_WINDOW_COVERING_CURRENT_POSITION_TILT_DEFAULT_VALUE;
    uint8_t default_val = 1;

    uint16_t travel_time = 60;

    esp_zb_cluster_add_attr(window_attr_list, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_TILT_PERCENTAGE_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &current_tilt_percentage);
    esp_zb_cluster_add_attr(window_attr_list, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_INSTALLED_OPEN_LIMIT_TILT_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &installed_open_limit_tilt);
    esp_zb_cluster_add_attr(window_attr_list, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_INSTALLED_CLOSED_LIMIT_TILT_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &installed_closed_limit_tilt);
    esp_zb_cluster_add_attr(window_attr_list, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_TILT_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &current_position_tilt);                       
   
    esp_zb_cluster_add_attr(window_attr_list, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &current_tilt_percentage);
    esp_zb_cluster_add_attr(window_attr_list, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_INSTALLED_OPEN_LIMIT_LIFT_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &installed_open_limit_tilt);
    esp_zb_cluster_add_attr(window_attr_list, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_INSTALLED_CLOSED_LIMIT_LIFT_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &installed_closed_limit_tilt);
    esp_zb_cluster_add_attr(window_attr_list, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &current_position_tilt);                                

    esp_zb_cluster_add_attr(window_attr_list, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, 0xF000,
                            ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &default_val); 

    esp_zb_cluster_add_attr(window_attr_list, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, 0xF001,
                            ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &default_val); 

    esp_zb_cluster_add_attr(window_attr_list, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, 0xF002,
                            ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &default_val); 

    esp_zb_cluster_add_attr(window_attr_list, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, 0xF003,
                            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &travel_time); 
                            
                            
    // 0xF000: ("curtain_switch", t.enum8, True),  # 0: open, 1: stop, 2: close
    // 0xF001: ("accurate_calibration", t.enum8, True),  # 0: calibration started, 1: calibration finished
    // 0xF002: ("motor_steering", t.enum8, True),  # 0: default, 1: reverse
    // 0xF003: ("travel", t.uint16_t, True),  # 30 to 9000 (units of 0.1 seconds)
    return window_attr_list;
}

esp_zb_attribute_list_t* nuos_zb_create_diagnostic_cluster(){

    esp_zb_diagnostics_cluster_cfg_t  diagnostics_cfg = {
    };
    
    esp_zb_attribute_list_t *esp_zb_diagnostic_cluster =  esp_zb_diagnostics_cluster_create(&diagnostics_cfg);

    return esp_zb_diagnostic_cluster;
}




esp_zb_attribute_list_t* nuos_zb_create_time_cluster(){
    uint8_t test_attr = 0;
    uint8_t time_status = 2;
    uint8_t dst_start = 0;
    uint8_t zone_id_default = 0;
    uint16_t zone_id = 19800;
    esp_zb_attribute_list_t *esp_zb_time_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TIME);

    esp_zb_cluster_add_attr(esp_zb_time_cluster, ESP_ZB_ZCL_CLUSTER_ID_TIME, ESP_ZB_ZCL_ATTR_TIME_LOCAL_TIME_ID,
        ESP_ZB_ZCL_ATTR_TYPE_32BIT, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &test_attr);

    esp_zb_cluster_add_attr(esp_zb_time_cluster, ESP_ZB_ZCL_CLUSTER_ID_TIME, ESP_ZB_ZCL_ATTR_TIME_STANDARD_TIME_ID,
        ESP_ZB_ZCL_ATTR_TYPE_32BIT, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &test_attr);

    esp_zb_time_cluster_add_attr(esp_zb_time_cluster, ESP_ZB_ZCL_ATTR_TIME_TIME_STATUS_ID, &time_status);  


    esp_zb_cluster_add_attr(esp_zb_time_cluster, ESP_ZB_ZCL_CLUSTER_ID_TIME, ESP_ZB_ZCL_ATTR_TIME_TIME_ZONE_ID,
        ESP_ZB_ZCL_ATTR_TYPE_32BIT, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &zone_id_default);

    esp_zb_cluster_update_attr(esp_zb_time_cluster, ESP_ZB_ZCL_ATTR_TIME_TIME_ZONE_ID, &zone_id);

    // esp_zb_time_cluster_add_attr(esp_zb_time_cluster, ESP_ZB_ZCL_ATTR_TIME_TIME_ZONE_ID, &zone_id);

    return esp_zb_time_cluster;
}

esp_zb_attribute_list_t* nuos_zb_create_identify_cluster(uint8_t role){
    uint8_t test_attr = 4;
    /* identify cluster create with fully customized */
    esp_zb_identify_cluster_cfg_t identify_cfg = {
        .identify_time = 0,
	};
	//esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_identify_cluster_create(&identify_cfg);
    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
    if(role == ESP_ZB_ZCL_CLUSTER_SERVER_ROLE){
        esp_zb_identify_cluster_add_attr(esp_zb_identify_cluster, ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE, &identify_cfg.identify_time);
    }  
    return esp_zb_identify_cluster;
}

esp_zb_attribute_list_t* nuos_zb_create_group_cluster_1(){
    esp_zb_attribute_list_t *esp_zb_groups_cluster =  esp_zb_groups_cluster_create(NULL);
    return esp_zb_groups_cluster;
}

esp_zb_attribute_list_t* nuos_zb_create_scene_cluster_1(){
    esp_zb_attribute_list_t *esp_zb_scene_cluster = esp_zb_scenes_cluster_create(NULL);
    return esp_zb_scene_cluster;
}

esp_zb_attribute_list_t* nuos_zb_create_group_cluster(){
    esp_zb_groups_cluster_cfg_t group_cfg = {
        .groups_name_support_id = ESP_ZB_ZCL_GROUPS_NAME_SUPPORT_DEFAULT_VALUE
    };
    esp_zb_attribute_list_t *esp_zb_groups_cluster =  esp_zb_groups_cluster_create(&group_cfg);
    return esp_zb_groups_cluster;
}

esp_zb_attribute_list_t* nuos_zb_create_scene_cluster(){
    /* scenes cluster create with standard cluster + customized */
	esp_zb_scenes_cluster_cfg_t scene_cfg =
	{
		.scenes_count = 0,
		.current_scene = ESP_ZB_ZCL_SCENES_CURRENT_SCENE_DEFAULT_VALUE,
		.current_group = ESP_ZB_ZCL_SCENES_CURRENT_GROUP_DEFAULT_VALUE,
		.scene_valid = ESP_ZB_ZCL_SCENES_SCENE_VALID_DEFAULT_VALUE,
		.name_support = ESP_ZB_ZCL_SCENES_NAME_SUPPORT_DEFAULT_VALUE,
	};
    esp_zb_attribute_list_t *esp_zb_scene_cluster = esp_zb_scenes_cluster_create(&scene_cfg);

    
    return esp_zb_scene_cluster;
}

esp_zb_attribute_list_t* nuos_zb_create_temperature_cluster(){
    uint16_t measured_value = 2500;
    esp_zb_temperature_meas_cluster_cfg_t temperature_cfg = {
        .measured_value = measured_value,
        .max_value = 10000,
        .min_value = 0,
    };

    esp_zb_attribute_list_t *esp_zb_temperature_cluster  = esp_zb_temperature_meas_cluster_create(&temperature_cfg);
    return esp_zb_temperature_cluster;
}

esp_zb_attribute_list_t* nuos_zb_create_humidity_cluster(){
    esp_zb_humidity_meas_cluster_cfg_t humidity_cfg = {
        .measured_value = 100,
        .min_value = 0,
        .max_value = 10000,
    };
	esp_zb_attribute_list_t *esp_zb_humid_cluster = esp_zb_humidity_meas_cluster_create(&humidity_cfg);
    return esp_zb_humid_cluster;
}

esp_zb_attribute_list_t* nuos_zb_create_illuminance_cluster(){
    uint8_t sensor_id = 0;
    esp_zb_illuminance_meas_cluster_cfg_t illuminance_cfg = {
        .max_value = ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MAX_MEASURED_VALUE_MAX_VALUE,
        .min_value = ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MIN_MEASURED_VALUE_MIN_VALUE,
        .measured_value = ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_DEFAULT,
    };
    //esp_zb_attribute_list_t *esp_zb_lux_cluster = esp_zb_illuminance_meas_cluster_create(&illuminance_cfg);
    esp_zb_attribute_list_t *esp_zb_lux_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT);
    esp_zb_cluster_add_attr(esp_zb_lux_cluster, ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT, ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &illuminance_cfg.measured_value);    
    esp_zb_cluster_add_attr(esp_zb_lux_cluster, ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT, ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MIN_MEASURED_VALUE_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &illuminance_cfg.min_value);
    esp_zb_cluster_add_attr(esp_zb_lux_cluster, ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT, ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MAX_MEASURED_VALUE_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &illuminance_cfg.max_value);
    esp_zb_cluster_add_attr(esp_zb_lux_cluster, ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT, ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_LIGHT_SENSOR_TYPE_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &sensor_id);                            
                            
    return esp_zb_lux_cluster;
}

esp_zb_attribute_list_t* nuos_zb_create_ias_zone_cluster(uint8_t zone_type, uint8_t zone_id){
    /* create ias zone cluster with server parameters */
    esp_zb_ias_zone_cluster_cfg_t zone_cfg = {
        .zone_state   = 0,
        .zone_type    = zone_type,
        .zone_status  = 0,
        .ias_cie_addr = ESP_ZB_ZCL_ZONE_IAS_CIE_ADDR_DEFAULT,
        .zone_id      = zone_id,
    };
    return esp_zb_ias_zone_cluster_create(&zone_cfg);
}

esp_zb_attribute_list_t* nuos_zb_create_switch_toggle_button_cluster(){
        /* window covering cluster create with fully customized */
    esp_zb_on_off_switch_cluster_cfg_t on_off_switch_cfg = {
        .switch_type = ESP_ZB_ZCL_ON_OFF_SWITCH_CONFIGURATION_SWITCH_TYPE_TOGGLE,
        .switch_action = ESP_ZB_ZCL_ON_OFF_SWITCH_CONFIGURATION_SWITCH_ACTIONS_TYPE1
    };
    esp_zb_attribute_list_t *on_off_cfg = esp_zb_on_off_switch_config_cluster_create(&on_off_switch_cfg);
    return on_off_cfg;
}

esp_zb_attribute_list_t* nuos_zb_create_switch_push_button_cluster(){
    /* window covering cluster create with fully customized */
    esp_zb_on_off_switch_cluster_cfg_t on_off_switch_cfg = {
        .switch_type = ESP_ZB_ZCL_ON_OFF_SWITCH_CONFIGURATION_SWITCH_TYPE_MOMENTARY,
        .switch_action = ESP_ZB_ZCL_ON_OFF_SWITCH_CONFIGURATION_SWITCH_ACTIONS_TYPE1
    };
    esp_zb_attribute_list_t *on_off_cfg = esp_zb_on_off_switch_config_cluster_create(&on_off_switch_cfg);
    
    return on_off_cfg;
}

esp_zb_attribute_list_t* nuos_zb_create_fan_cluster(){
    esp_zb_fan_control_cluster_cfg_t fan_cfg = {
            .fan_mode = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_OFF,
            .fan_mode_sequence = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_SEQUENCE_LOW_MED_HIGH
    };
    esp_zb_attribute_list_t *esp_zb_fan_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL);

    esp_zb_cluster_add_attr(esp_zb_fan_cluster, ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL, ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &fan_cfg.fan_mode);
    esp_zb_cluster_add_attr(esp_zb_fan_cluster, ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL, ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_SEQUENCE_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &fan_cfg.fan_mode_sequence);

    return esp_zb_fan_cluster;
}

esp_zb_attribute_list_t* nuos_zb_create_level_cluster(uint8_t min_level, uint8_t max_level){
    uint8_t current_level = max_level;
    uint8_t move_rate = current_level;
    uint8_t on_level = current_level;
    uint16_t transition_time = 0x0000;  
    uint8_t startup_current_level = 255;

    esp_zb_attribute_list_t *esp_zb_level_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL);

    esp_zb_cluster_add_attr(esp_zb_level_cluster, ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID,
                        ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &current_level);

    esp_zb_cluster_add_attr(esp_zb_level_cluster, ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_MIN_LEVEL_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &min_level);

    esp_zb_cluster_add_attr(esp_zb_level_cluster, ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_MAX_LEVEL_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING , &max_level); 

    #ifdef  TUYA_ATTRIBUTES
        esp_zb_cluster_add_attr(esp_zb_level_cluster, ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL, 0xF000,
                                ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &current_level);                         
    #endif                         
    esp_zb_level_cluster_add_attr(esp_zb_level_cluster, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_START_UP_CURRENT_LEVEL_ID, &startup_current_level);
    return esp_zb_level_cluster;
}


esp_zb_attribute_list_t* nuos_zb_create_level_cluster_2(uint8_t min_level, uint8_t max_level){

    esp_zb_attribute_list_t *esp_zb_level_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL);
    esp_zb_level_cluster_add_attr(esp_zb_level_cluster, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID, &max_level);
    esp_zb_level_cluster_add_attr(esp_zb_level_cluster, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_MIN_LEVEL_ID, &min_level);
    esp_zb_level_cluster_add_attr(esp_zb_level_cluster, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_MAX_LEVEL_ID, &max_level);

    return esp_zb_level_cluster;
}
/*
    Key Attributes for Color Control:
    Color Mode (0x0008):

    This attribute indicates the current color mode of the light.
    It can take the following values:
    0x00: Current mode is "Current Hue and Current Saturation."
    0x01: Current mode is "Current X and Current Y" (CIE 1931 color space).
    0x02: Current mode is "Color Temperature."
    Enhanced Color Mode (0x4001):

    Similar to the Color Mode attribute but used when the device supports enhanced color capabilities.
    It can take the following values:
    0x00: Enhanced mode is "Current Hue and Current Saturation."
    0x01: Enhanced mode is "Current X and Current Y."
    0x02: Enhanced mode is "Color Temperature."
    Color Capabilities (0x400A):

    This attribute provides information about the color control capabilities of the device.
    It is a bitmask that can have the following values:
    0x0001: Supports "Hue and Saturation" mode.
    0x0002: Supports "X and Y" mode.
    0x0004: Supports "Color Temperature."
    0x0008: Supports "Enhanced Hue."      0x0B  -- 00001011    0x0B  -- 00001011
    0x0010: Supports "Color Loop."        
    0x0020: Supports "RGB."

 */
esp_zb_attribute_list_t* nuos_zb_create_color_temperature_control_cluster(){

     uint16_t current_hue = ESP_ZB_ZCL_COLOR_CONTROL_CURRENT_HUE_DEFAULT_VALUE;
     uint16_t current_sat = ESP_ZB_ZCL_COLOR_CONTROL_CURRENT_SATURATION_DEFAULT_VALUE;
     uint16_t enhanced_current_hue = ESP_ZB_ZCL_COLOR_CONTROL_ENHANCED_CURRENT_HUE_DEFAULT_VALUE;

    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI)
        const uint16_t MID_TEMP = MIN_CCT_VALUE + (MAX_CCT_VALUE - MIN_CCT_VALUE)/2;
        uint16_t min_temp = MIN_CCT_VALUE;
        uint16_t max_temp = MAX_CCT_VALUE;        
    #else 
        const uint16_t MID_TEMP = MIN_CCT_VALUE + (MAX_CCT_VALUE - MIN_CCT_VALUE)/2;  
        uint16_t min_temp = 500;//MIN_CCT_VALUE;
        uint16_t max_temp = 158;//MAX_CCT_VALUE; 
    #endif
    uint8_t loop_active = 0;
    uint16_t color_attr = 250;//MID_TEMP; //
    // It works with HA!!!
    // Use esp_zb_color_control_cluster_add_attr for this
    //esp_zb_color_control_cluster_create
    esp_zb_color_cluster_cfg_t esp_zb_color_cluster_cfg = { 
        .current_x = ESP_ZB_ZCL_COLOR_CONTROL_CURRENT_X_DEF_VALUE,                          /*!<  The current value of the normalized chromaticity value x */
        .current_y = ESP_ZB_ZCL_COLOR_CONTROL_CURRENT_Y_DEF_VALUE,                          /*!<  The current value of the normalized chromaticity value y */ 
        .color_mode = 0x02,                                                                 /*!<  The mode which attribute determines the color of the device */ 
        .options = ESP_ZB_ZCL_COLOR_CONTROL_OPTIONS_DEFAULT_VALUE,                          /*!<  The bitmap determines behavior of some cluster commands */ 
        .enhanced_color_mode = ESP_ZB_ZCL_COLOR_CONTROL_ENHANCED_COLOR_MODE_DEFAULT_VALUE,                                                        /*!<  The enhanced-mode which attribute determines the color of the device */ 
        .color_capabilities = 0x0010,                                                       /*!<  Specifying the color capabilities of the device support the color control cluster */ 
    };
    esp_zb_attribute_list_t *esp_zb_color_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL);


    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_CAPABILITIES_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_16BITMAP, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &esp_zb_color_cluster_cfg.color_capabilities);

    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_LOOP_ACTIVE_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &loop_active);

    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &color_attr);
    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMP_PHYSICAL_MIN_MIREDS_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &min_temp);
    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMP_PHYSICAL_MAX_MIREDS_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &max_temp);


    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_HUE_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &current_hue);
    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_SATURATION_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &current_sat);

    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_X_ID,
                        ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &esp_zb_color_cluster_cfg.current_x);

    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_Y_ID,
                        ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &esp_zb_color_cluster_cfg.current_y);

    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_OPTIONS_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &esp_zb_color_cluster_cfg.options);

    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_MODE_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &esp_zb_color_cluster_cfg.color_mode);

    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_ENHANCED_COLOR_MODE_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &esp_zb_color_cluster_cfg.enhanced_color_mode);

#ifdef  TUYA_ATTRIBUTES
    uint8_t tuya_mode = 0;
    uint8_t tuya_brightness = 150;
    uint16_t color_temp = MIN_CCT_VALUE;
    uint16_t zero_data = 0x0000; 

    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0xE000,
                            ESP_ZB_ZCL_ATTR_TYPE_16BIT, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &color_temp);

    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0xF000,
                            ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &tuya_mode);

    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0xF001,
                            ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &tuya_brightness);
    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0xF002,
                            ESP_ZB_ZCL_ATTR_TYPE_U48, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &zero_data);                            
    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0xF003,
                            ESP_ZB_ZCL_ATTR_TYPE_U48, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &zero_data);
                            

    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0xF007,
                            ESP_ZB_ZCL_ATTR_TYPE_U48, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &zero_data);
         
    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0xF008,
                ESP_ZB_ZCL_ATTR_TYPE_U48, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &zero_data);

    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0xF009,
                ESP_ZB_ZCL_ATTR_TYPE_U48, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &zero_data);

    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0xF00A,
                ESP_ZB_ZCL_ATTR_TYPE_U48, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &zero_data);

    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0xF00B,
                ESP_ZB_ZCL_ATTR_TYPE_U48, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &zero_data);

    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0xF00C,
                ESP_ZB_ZCL_ATTR_TYPE_U48, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &zero_data);

    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0xF00D,
                ESP_ZB_ZCL_ATTR_TYPE_BOOL, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &zero_data);

    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0xF00E,
                ESP_ZB_ZCL_ATTR_TYPE_U48, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &zero_data);

#endif

    return esp_zb_color_cluster;
}

esp_zb_attribute_list_t* nuos_zb_create_color_ctrl_cluster(){


    uint16_t current_hue = ESP_ZB_ZCL_COLOR_CONTROL_CURRENT_HUE_DEFAULT_VALUE;
    uint16_t current_sat = ESP_ZB_ZCL_COLOR_CONTROL_CURRENT_SATURATION_DEFAULT_VALUE;
    uint16_t enhanced_current_hue = ESP_ZB_ZCL_COLOR_CONTROL_ENHANCED_CURRENT_HUE_DEFAULT_VALUE;
    uint8_t loop_active = 0;

    
    uint16_t color_attr = 500;
    uint16_t min_temp = 158;//0;
    uint16_t max_temp = 500;//0xff00;
    uint16_t startup_color_temp = 17476; 


    esp_zb_color_cluster_cfg_t esp_zb_color_cluster_cfg = { 
        .current_x = ESP_ZB_ZCL_COLOR_CONTROL_CURRENT_X_DEF_VALUE,                          /*!<  The current value of the normalized chromaticity value x */
        .current_y = ESP_ZB_ZCL_COLOR_CONTROL_CURRENT_Y_DEF_VALUE,                          /*!<  The current value of the normalized chromaticity value y */ 
        .color_mode = ESP_ZB_ZCL_COLOR_CONTROL_COLOR_MODE_DEFAULT_VALUE,                                                               /*!<  The mode which attribute determines the color of the device */ 
        .options = ESP_ZB_ZCL_COLOR_CONTROL_OPTIONS_DEFAULT_VALUE,                                                                     /*!<  The bitmap determines behavior of some cluster commands */ 
        .enhanced_color_mode = ESP_ZB_ZCL_COLOR_CONTROL_ENHANCED_COLOR_MODE_DEFAULT_VALUE, 
                                                          /*!<  The enhanced-mode which attribute determines the color of the device */ 
        #if(USE_COLOR_DEVICE == COLOR_RGB_ONLY)
        .color_capabilities = 0x0008,                                                       /*!<  Specifying the color capabilities of the device support the color control cluster */ 
        #elif(USE_COLOR_DEVICE == COLOR_RGBW)
        .color_capabilities = 0x001F,                                                       /*!<  Specifying the color capabilities of the device support the color control cluster */ 
        #elif(USE_COLOR_DEVICE == COLOR_RGB_CW_WW)
        .color_capabilities = 0x003F, 
        #else                                                      /*!<  Specifying the color capabilities of the device support the color control cluster */ 
        #endif
    };

    esp_zb_attribute_list_t *esp_zb_color_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL);

    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_CAPABILITIES_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_16BITMAP, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &esp_zb_color_cluster_cfg.color_capabilities);

    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_LOOP_ACTIVE_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &loop_active);

    #if(USE_COLOR_DEVICE == COLOR_RGB_CW_WW || USE_COLOR_DEVICE == COLOR_RGBW)                                
    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &color_attr);
    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMP_PHYSICAL_MIN_MIREDS_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &min_temp);
    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMP_PHYSICAL_MAX_MIREDS_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &max_temp);

    #endif
    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_HUE_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &current_hue);
    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_SATURATION_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &current_sat);


    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_X_ID,
                        ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &esp_zb_color_cluster_cfg.current_x);

    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_Y_ID,
                        ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &esp_zb_color_cluster_cfg.current_y);

    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_OPTIONS_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &esp_zb_color_cluster_cfg.options);

    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_MODE_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &esp_zb_color_cluster_cfg.color_mode);

    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_ENHANCED_COLOR_MODE_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &esp_zb_color_cluster_cfg.enhanced_color_mode);

    esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_START_UP_COLOR_TEMPERATURE_MIREDS_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &startup_color_temp);

    #ifdef  TUYA_ATTRIBUTES                                                    
        uint8_t tuya_mode           = 1;
        uint8_t tuya_brightness     = 80;
        uint16_t color_temp         = 0x0000;
        uint16_t zero_data          = 0x0000;

        esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0xE000,
                                ESP_ZB_ZCL_ATTR_TYPE_16BIT, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &color_temp);

        esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0xE100,
                                ESP_ZB_ZCL_ATTR_TYPE_U48, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &color_temp);

        esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0xF000,
                                ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &tuya_mode);
        esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0xF001,
                                ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &tuya_brightness);
        esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0xF002,
                                ESP_ZB_ZCL_ATTR_TYPE_U48, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &zero_data);                            
        esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0xF003,
                                ESP_ZB_ZCL_ATTR_TYPE_U48, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &zero_data);
                                
        esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0xF007,
                                ESP_ZB_ZCL_ATTR_TYPE_U48, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &zero_data);
            
        esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0xF008,
                    ESP_ZB_ZCL_ATTR_TYPE_U48, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &zero_data);

        esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0xF009,
                    ESP_ZB_ZCL_ATTR_TYPE_U48, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &zero_data);

        esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0xF00A,
                    ESP_ZB_ZCL_ATTR_TYPE_U48, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &zero_data);

        esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0xF00B,
                    ESP_ZB_ZCL_ATTR_TYPE_U48, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &zero_data);

        esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0xF00C,
                    ESP_ZB_ZCL_ATTR_TYPE_U48, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &zero_data);

        esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0xF00D,
                    ESP_ZB_ZCL_ATTR_TYPE_BOOL, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &zero_data);

        esp_zb_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0xF00E,
                    ESP_ZB_ZCL_ATTR_TYPE_U48, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &zero_data);

    #endif
    return esp_zb_color_cluster;
}

esp_zb_attribute_list_t* nuos_zb_create_onoff_cluster(){
    uint8_t onoff_cfg_id = ESP_ZB_ZCL_ON_OFF_ON_OFF_DEFAULT_VALUE;
    uint8_t onoff_startup_onoff_time = 1;

    esp_zb_attribute_list_t *esp_zb_on_off_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ON_OFF);

    esp_zb_cluster_add_attr(esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
                                    ESP_ZB_ZCL_ATTR_TYPE_BOOL, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &onoff_cfg_id);
                                    
    esp_zb_cluster_add_attr(esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_ATTR_ON_OFF_START_UP_ON_OFF,
                                    ESP_ZB_ZCL_ATTR_TYPE_BOOL, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &onoff_startup_onoff_time); 
    return esp_zb_on_off_cluster;
}


esp_zb_attribute_list_t* nuos_zb_create_tuya_cluster(){

    uint16_t current_temp = 26;
    uint16_t set_temp = 27;
    uint8_t attr_onoff = 0;
    uint8_t mode = 3;


    esp_zb_attribute_list_t *esp_zb_tuya_private_cluster = esp_zb_zcl_attr_list_create(0xEF00);

    esp_zb_cluster_add_attr(esp_zb_tuya_private_cluster, 0xEF00, 0x02,
                        ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &attr_onoff);

    esp_zb_cluster_add_attr(esp_zb_tuya_private_cluster, 0xEF00, 0x03,
                            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &current_temp);  //current temp

    esp_zb_cluster_add_attr(esp_zb_tuya_private_cluster, 0xEF00, 0x04,
                            ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING , &mode); //mode


    return esp_zb_tuya_private_cluster;
}




esp_zb_attribute_list_t* nuos_zb_create_tuya_cluster_2(){
    
    uint16_t min_max_val = 0x101E;
    uint8_t bulb_type = 0;
    uint8_t scr_state = 1;
    uint8_t current_percentage = 1;
    uint8_t min_percentage = 1;
    esp_zb_attribute_list_t *esp_zb_tuya_private_cluster = esp_zb_zcl_attr_list_create(0xEE00);
    esp_zb_custom_cluster_add_custom_attr(esp_zb_tuya_private_cluster, 0xFC00, ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &min_max_val);
    esp_zb_custom_cluster_add_custom_attr(esp_zb_tuya_private_cluster, 0xFC02, ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &bulb_type);
    esp_zb_custom_cluster_add_custom_attr(esp_zb_tuya_private_cluster, 0xFC03, ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &scr_state);
    esp_zb_custom_cluster_add_custom_attr(esp_zb_tuya_private_cluster, 0xFC04, ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &current_percentage);
    esp_zb_custom_cluster_add_custom_attr(esp_zb_tuya_private_cluster, 0xFC05, ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &min_percentage);
    return esp_zb_tuya_private_cluster;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
esp_zb_cluster_list_t* server_cluster(uint8_t index){
	/* create new cluster list*/
    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_attribute_list_t *touchlink_cluster = NULL;

    esp_zb_zcl_group_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
    esp_zb_zcl_scene_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;

    if(index == 0){
        esp_zb_attribute_list_t *esp_zb_basic_cluster = nuos_zb_create_basic_cluster(index);  
        esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
        printf("Created Cluster Basic\n");
        esp_zb_attribute_list_t *esp_zb_time_cluster = nuos_zb_create_time_cluster();
        /* create cluster lists for this endpoint */
        esp_zb_cluster_list_add_time_cluster(esp_zb_cluster_list, esp_zb_time_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);  
        printf("Created Cluster Time\n"); 
        #ifdef USE_OTA
        esp_zb_cluster_list_add_ota_cluster(esp_zb_cluster_list, get_ota_cluster(), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
        printf("Created Cluster OTA\n");
        #endif
    }

    esp_zb_zcl_identify_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
    #if( USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH)
        esp_zb_zcl_identify_role = ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE;
    #endif
    esp_zb_attribute_list_t* esp_zb_identify_cluster = nuos_zb_create_identify_cluster(esp_zb_zcl_identify_role);
     #if(USE_NUOS_ZB_DEVICE_TYPE != DEVICE_WIRELESS_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE != DEVICE_WIRELESS_REMOTE_SWITCH  || USE_NUOS_ZB_DEVICE_TYPE != DEVICE_WIRELESS_GROUP_SWITCH)
         esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, esp_zb_identify_cluster, esp_zb_zcl_identify_role);
         printf("Created Cluster Identify\n");
     #endif
    esp_zb_attribute_list_t *esp_zb_on_off_cluster = nuos_zb_create_onoff_cluster();
    
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_MOTION)
        #ifdef IS_BATTERY_POWERED
            esp_zb_attribute_list_t *esp_zb_power_cluster = nuos_zb_create_power_cluster();
            esp_zb_cluster_list_add_power_config_cluster(esp_zb_cluster_list, esp_zb_power_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
        #endif
        esp_zb_attribute_list_t* esp_zb_sensor_cluster = nuos_zb_create_ias_zone_cluster(ESP_ZB_ZCL_IAS_ZONE_ZONETYPE_MOTION, 20);
        esp_zb_cluster_list_add_ias_zone_cluster(esp_zb_cluster_list, esp_zb_sensor_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
        uint8_t ss = 0;
        esp_zb_attribute_list_t *esp_zb_tuya_private_cluster = esp_zb_zcl_attr_list_create(0xFC31);
        esp_zb_custom_cluster_add_custom_attr(esp_zb_tuya_private_cluster, 0x0000, ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &ss);
        esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, esp_zb_tuya_private_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_CONTACT_SWITCH)
        esp_zb_attribute_list_t* esp_zb_sensor_cluster = nuos_zb_create_ias_zone_cluster(ESP_ZB_ZCL_IAS_ZONE_ZONETYPE_CONTACT_SWITCH, 0);
        esp_zb_cluster_list_add_ias_zone_cluster(esp_zb_cluster_list, esp_zb_sensor_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE); 
        #ifdef IS_BATTERY_POWERED
            esp_zb_attribute_list_t *esp_zb_power_cluster = nuos_zb_create_power_cluster();
            esp_zb_cluster_list_add_power_config_cluster(esp_zb_cluster_list, esp_zb_power_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
        #endif        
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_GAS_LEAK)
        esp_zb_attribute_list_t* esp_zb_sensor_cluster = nuos_zb_create_ias_zone_cluster(ESP_ZB_ZCL_IAS_ZONE_ZONETYPE_GAS_SENSOR, 10);
        esp_zb_cluster_list_add_ias_zone_cluster(esp_zb_cluster_list, esp_zb_sensor_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE); 
        #ifdef IS_BATTERY_POWERED
            esp_zb_attribute_list_t *esp_zb_power_cluster = nuos_zb_create_power_cluster();
            esp_zb_cluster_list_add_power_config_cluster(esp_zb_cluster_list, esp_zb_power_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
        #endif       
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY)
        if(index == 0){
            esp_zb_attribute_list_t *esp_zb_temperature_cluster  = nuos_zb_create_temperature_cluster();
            esp_zb_cluster_list_add_temperature_meas_cluster(esp_zb_cluster_list, esp_zb_temperature_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
        }else if(index == 1){
            esp_zb_attribute_list_t *esp_zb_humid_cluster = nuos_zb_create_humidity_cluster();
            esp_zb_cluster_list_add_humidity_meas_cluster(esp_zb_cluster_list, esp_zb_humid_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
        }else if(index == 2){
            #ifdef IS_BATTERY_POWERED
                esp_zb_attribute_list_t *esp_zb_power_cluster = nuos_zb_create_power_cluster();
                esp_zb_cluster_list_add_power_config_cluster(esp_zb_cluster_list, esp_zb_power_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
            #endif      
        }
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY_CUSTOM)

        esp_zb_attribute_list_t *esp_zb_temperature_cluster  = nuos_zb_create_temperature_cluster();
        esp_zb_cluster_list_add_temperature_meas_cluster(esp_zb_cluster_list, esp_zb_temperature_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        esp_zb_attribute_list_t *esp_zb_humid_cluster = nuos_zb_create_humidity_cluster();
        esp_zb_cluster_list_add_humidity_meas_cluster(esp_zb_cluster_list, esp_zb_humid_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        #ifdef IS_BATTERY_POWERED
            esp_zb_attribute_list_t *esp_zb_power_cluster = nuos_zb_create_power_cluster();
            esp_zb_cluster_list_add_power_config_cluster(esp_zb_cluster_list, esp_zb_power_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
        #endif      
       
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_LUX)
        esp_zb_attribute_list_t *esp_zb_lux_cluster = nuos_zb_create_illuminance_cluster();
        esp_zb_cluster_list_add_illuminance_meas_cluster(esp_zb_cluster_list, esp_zb_lux_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        esp_zb_attribute_list_t* esp_zb_sensor_cluster = nuos_zb_create_ias_zone_cluster(0x8001, 0);
        esp_zb_cluster_list_add_ias_zone_cluster(esp_zb_cluster_list, esp_zb_sensor_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);         
        #ifdef IS_BATTERY_POWERED
            esp_zb_attribute_list_t *esp_zb_power_cluster = nuos_zb_create_power_cluster();
            esp_zb_cluster_list_add_power_config_cluster(esp_zb_cluster_list, esp_zb_power_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
        #endif      
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_GROUP_SWITCH)
        /* create client role of the cluster */
        uint8_t value = 0;
        esp_zb_attribute_list_t *esp_zb_power_cluster = nuos_zb_create_power_cluster();
        esp_zb_cluster_list_add_power_config_cluster(esp_zb_cluster_list, esp_zb_power_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
     
        #ifdef USE_CUSTOM_SCENE
            uint16_t group_id = 0;
            uint16_t scene_id = 0;
            uint8_t dummy = 0;
            uint8_t mode = 1; //0-switch, 1-scene switch

            esp_zb_attribute_list_t *esp_zb_tuya_private_cluster = esp_zb_zcl_attr_list_create(0xE000);
            esp_zb_custom_cluster_add_custom_attr(esp_zb_tuya_private_cluster, 0xD004, ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &group_id);
            esp_zb_custom_cluster_add_custom_attr(esp_zb_tuya_private_cluster, 0xD005, ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &scene_id);
            esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, esp_zb_tuya_private_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE); 

            esp_zb_attribute_list_t *esp_zb_tuya_private_cluster_3 = esp_zb_zcl_attr_list_create(0xE001);
            esp_zb_custom_cluster_add_custom_attr(esp_zb_tuya_private_cluster_3, 0xD010, ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &dummy);
            esp_zb_custom_cluster_add_custom_attr(esp_zb_tuya_private_cluster_3, 0xD020, ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &mode);          
            esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, esp_zb_tuya_private_cluster_3, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);


            esp_zb_attribute_list_t *esp_zb_tuya_private_cluster_4 = esp_zb_zcl_attr_list_create(0xEF00);
            esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, esp_zb_tuya_private_cluster_4, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE); 
            
            esp_zb_attribute_list_t *esp_zb_tuya_private_cluster_5 = esp_zb_zcl_attr_list_create(0xEE00);
            esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, esp_zb_tuya_private_cluster_5, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);            
        #endif
        esp_zb_attribute_list_t *esp_zb_on_off_server_cluster = esp_zb_on_off_cluster_create(NULL);
        esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_server_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RINGING_BELL_2)
        #ifdef IS_BATTERY_POWERED
            esp_zb_attribute_list_t *esp_zb_power_cluster = nuos_zb_create_power_cluster();
            esp_zb_cluster_list_add_power_config_cluster(esp_zb_cluster_list, esp_zb_power_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
        #endif
        esp_zb_attribute_list_t* esp_zb_sensor_cluster = nuos_zb_create_ias_zone_cluster(ESP_ZB_ZCL_IAS_ZONE_ZONETYPE_CONTACT_SWITCH, 0);
        esp_zb_cluster_list_add_ias_zone_cluster(esp_zb_cluster_list, esp_zb_sensor_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);                                 
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH)
        /* create client role of the cluster */
        esp_zb_zcl_group_role = ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE;
        esp_zb_zcl_scene_role = ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE;    
        
        // esp_zb_attribute_list_t *esp_zb_switch_client_cluster = nuos_zb_create_switch_push_button_cluster();
        // esp_zb_cluster_list_add_on_off_switch_config_cluster(esp_zb_cluster_list, esp_zb_switch_client_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        // esp_zb_attribute_list_t *esp_zb_on_off_client_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ON_OFF);
        // esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_client_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

        // uint8_t value = 0;
        // esp_zb_attribute_list_t *esp_zb_power_cluster = nuos_zb_create_power_cluster();
        // esp_zb_cluster_list_add_power_config_cluster(esp_zb_cluster_list, esp_zb_power_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
     
        esp_zb_attribute_list_t *esp_zb_on_off_server_cluster = esp_zb_on_off_cluster_create(NULL);
        esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_server_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);  
        
        // esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

	#elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4R_ON_OFF_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2R_ON_OFF_LIGHT)    
		esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE); 
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2CH_CURTAIN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN_SWITCH)
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN_SWITCH)
            uint8_t backlight_switch = 0;
            uint8_t indicator_status = 0;
 
            esp_zb_attribute_list_t* esp_zb_window_covering_cluster = nuos_zb_create_window_covering_cluster();
            esp_zb_cluster_list_add_window_covering_cluster(esp_zb_cluster_list, esp_zb_window_covering_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

            uint8_t default_val = 0;
            uint16_t travel_time = 60;

            esp_zb_attribute_list_t *esp_zb_tuya_private_cluster = esp_zb_zcl_attr_list_create(0xee00);
            esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, esp_zb_tuya_private_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE); 
            
            esp_zb_attribute_list_t *esp_zb_tuya_private_cluster2 = esp_zb_zcl_attr_list_create(0xef00);
            esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, esp_zb_tuya_private_cluster2, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);             
        #else
            esp_zb_attribute_list_t *esp_zb_power_cluster = nuos_zb_create_power_cluster();
            esp_zb_cluster_list_add_power_config_cluster(esp_zb_cluster_list, esp_zb_power_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);    
            esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
            #ifdef TUYA_ATTRIBUTES 
                esp_zb_attribute_list_t* esp_zb_window_covering_cluster = nuos_zb_create_window_covering_cluster();
                esp_zb_cluster_list_add_window_covering_cluster(esp_zb_cluster_list, esp_zb_window_covering_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);             
            #endif
        #endif
         
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI)    
    
        esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
        #ifdef USE_HOME_ASSISTANT
            esp_zb_attribute_list_t *esp_zb_level_cluster = nuos_zb_create_level_cluster(0, 255);
            esp_zb_cluster_list_add_level_cluster(esp_zb_cluster_list, esp_zb_level_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
        #endif
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_DALI_DIRECT_SWITCH)
        esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        esp_zb_attribute_list_t *esp_zb_level_cluster = nuos_zb_create_level_cluster(0, 255);
        esp_zb_cluster_list_add_level_cluster(esp_zb_cluster_list, esp_zb_level_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
        
        #ifdef USE_COLOR_CONTROL 
        esp_zb_attribute_list_t *esp_zb_colour_cluster = nuos_zb_create_color_temperature_control_cluster();
        if(esp_zb_cluster_list_add_color_control_cluster(esp_zb_cluster_list, esp_zb_colour_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE) == ESP_ERR_INVALID_ARG)
        {
            ESP_LOGI(TAG, "esp_zb_cluster_list_add_color_control_cluster ESP_ERR_INVALID_ARG");
        } 
        
        esp_zb_attribute_list_t *esp_zb_tuya_private_cluster0 = esp_zb_zcl_attr_list_create(0xEE00);
        esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, esp_zb_tuya_private_cluster0, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);         
        esp_zb_attribute_list_t *esp_zb_tuya_private_cluster1 = esp_zb_zcl_attr_list_create(0xEF00);
        esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, esp_zb_tuya_private_cluster1, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
        esp_zb_attribute_list_t *esp_zb_tuya_private_cluster2 = esp_zb_zcl_attr_list_create(0xE000);
        esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, esp_zb_tuya_private_cluster2, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);   
        #endif
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI)
        esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
        // #ifdef DALI_DIRECT_ADDRESSING 
        // esp_zb_attribute_list_t *esp_zb_level_cluster = nuos_zb_create_level_cluster(0, 255);
        // esp_zb_cluster_list_add_level_cluster(esp_zb_cluster_list, esp_zb_level_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
        // #endif

    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4T_COLOR_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_COLOR_DIMMABLE_LIGHT) 
        /* level cluster create with standard cluster config*/
        esp_zb_attribute_list_t *esp_zb_colour_cluster = nuos_zb_create_color_ctrl_cluster();
        esp_zb_attribute_list_t *esp_zb_level_cluster = nuos_zb_create_level_cluster(0, 255);
		esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
		esp_zb_cluster_list_add_level_cluster(esp_zb_cluster_list, esp_zb_level_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        if(esp_zb_cluster_list_add_color_control_cluster(esp_zb_cluster_list, esp_zb_colour_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE) == ESP_ERR_INVALID_ARG)
        {
            ESP_LOGI(TAG, "esp_zb_cluster_list_add_color_control_cluster ESP_ERR_INVALID_ARG");
        } 
        //1 curtain, 2 1l1f, rgb, 
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
         /* level cluster create with standard cluster config*/
        esp_zb_attribute_list_t *esp_zb_level_cluster = nuos_zb_create_level_cluster(0x00, 255);
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
            esp_zb_attribute_list_t *esp_zb_colour_cluster = nuos_zb_create_color_ctrl_cluster();
        #else
            esp_zb_attribute_list_t *esp_zb_colour_cluster = nuos_zb_create_color_temperature_control_cluster();
        #endif
       
		esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
		esp_zb_cluster_list_add_level_cluster(esp_zb_cluster_list, esp_zb_level_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        if(esp_zb_cluster_list_add_color_control_cluster(esp_zb_cluster_list, esp_zb_colour_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE) == ESP_ERR_INVALID_ARG)
        {
            ESP_LOGI(TAG, "esp_zb_cluster_list_add_color_control_cluster ESP_ERR_INVALID_ARG");
        } 
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4T_ANALOG_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_ANALOG_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_PHASE_CUT_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_CUSTOM)   

        esp_zb_attribute_list_t *esp_zb_level_cluster = nuos_zb_create_level_cluster(1, MAX_DIM_LEVEL_VALUE);
        esp_zb_cluster_list_add_level_cluster(esp_zb_cluster_list, esp_zb_level_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
        esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

	#elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN)
		if(index == LIGHT_INDEX){
			esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);  
            printf("Created Cluster ONOFF\n");
		}else if(index == FAN_INDEX){

            // esp_zb_attribute_list_t *esp_zb_tuya_private_cluster = esp_zb_zcl_attr_list_create(0xFC31);
            // esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, esp_zb_tuya_private_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE); 

            #ifdef TUYA_ATTRIBUTES
                esp_zb_attribute_list_t *esp_zb_tuya_private_cluster1 = esp_zb_zcl_attr_list_create(0xEF00);
                esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, esp_zb_tuya_private_cluster1, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE); 
                printf("Created Cluster 0xEF00\n");

                esp_zb_attribute_list_t *esp_zb_tuya_private_cluster2 = esp_zb_zcl_attr_list_create(0xED00);
                esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, esp_zb_tuya_private_cluster2, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE); 
                printf("Created Cluster 0xED00\n");
                esp_zb_attribute_list_t *esp_zb_tuya_private_cluster3 = esp_zb_zcl_attr_list_create(0xEE00);
                esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, esp_zb_tuya_private_cluster3, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);  
                printf("Created Cluster 0xEE00\n");               
            #else
                esp_zb_attribute_list_t *esp_zb_fan_cluster = nuos_zb_create_fan_cluster();        
                esp_zb_cluster_list_add_fan_control_cluster(esp_zb_cluster_list, esp_zb_fan_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
                // esp_zb_attribute_list_t *esp_zb_level_cluster = nuos_zb_create_level_cluster(0x00, 0xfe);
                // esp_zb_cluster_list_add_level_cluster(esp_zb_cluster_list, esp_zb_level_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
                // esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
            #endif
		}
	#elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN_CUSTOM)
		esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);  
        esp_zb_attribute_list_t *esp_zb_level_cluster = nuos_zb_create_level_cluster(0, 255);
        esp_zb_cluster_list_add_level_cluster(esp_zb_cluster_list, esp_zb_level_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);   
	#elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
        /* level cluster create with standard cluster config*/ 
        esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
        esp_zb_attribute_list_t *esp_zb_level_cluster = nuos_zb_create_level_cluster(0, 64);
        esp_zb_cluster_list_add_level_cluster(esp_zb_cluster_list, esp_zb_level_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE); 

        // esp_zb_attribute_list_t *esp_zb_fan_cluster = nuos_zb_create_fan_cluster();
        // esp_zb_cluster_list_add_fan_control_cluster(esp_zb_cluster_list, esp_zb_fan_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        // esp_zb_attribute_list_t *esp_zb_tuya_private_cluster = esp_zb_zcl_attr_list_create(0xEF00);
        // // esp_zb_custom_cluster_add_custom_attr(esp_zb_tuya_private_cluster, 0x0000, ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &min_percentage);
        // esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, esp_zb_tuya_private_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE); 
        
        // esp_zb_attribute_list_t *esp_zb_tuya_private_cluster = esp_zb_zcl_attr_list_create(0xEE00);
        // uint16_t min_max_val = 0x101E;
        // uint8_t bulb_type = 0;
        // uint8_t scr_state = 1;
        // uint8_t current_percentage = 1;
        // uint8_t min_percentage = 1;
        // esp_zb_custom_cluster_add_custom_attr(esp_zb_tuya_private_cluster, 0xFC00, ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &min_max_val);
        // esp_zb_custom_cluster_add_custom_attr(esp_zb_tuya_private_cluster, 0xFC02, ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &bulb_type);
        // esp_zb_custom_cluster_add_custom_attr(esp_zb_tuya_private_cluster, 0xFC03, ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &scr_state);
        // esp_zb_custom_cluster_add_custom_attr(esp_zb_tuya_private_cluster, 0xFC04, ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &current_percentage);
        // esp_zb_custom_cluster_add_custom_attr(esp_zb_tuya_private_cluster, 0xFC05, ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &min_percentage);
        // esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, esp_zb_tuya_private_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);                
    
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER)
        uint8_t thermo_setback_id = 0xff;
        uint8_t thermo_hvac_system_type = 0x00;
        uint8_t thermo_stat_mode = ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_COOL;
        #ifdef USE_HEATING_PIPES
            uint8_t thermo_stat_ctrl_seq = ESP_ZB_ZCL_THERMOSTAT_CONTROL_SEQ_OF_OPERATION_COOLING_AND_HEATING_4_PIPES;
        #else
            uint8_t thermo_stat_ctrl_seq = ESP_ZB_ZCL_THERMOSTAT_CONTROL_SEQ_OF_OPERATION_COOLING_ONLY;
        #endif
        uint16_t thermo_setback_change_amount = 0x8000; 
        uint16_t thermo_stat_min_coolpoint = 16*TEMPERATURE_MULTIPLICATION_FACTOR;  //16 degree
        uint16_t thermo_stat_max_coolpoint = 30*TEMPERATURE_MULTIPLICATION_FACTOR;
         if(index == 0){
            //
           
            #ifdef USE_HEATING_PIPES  
                uint16_t thermo_stat_min_heatpoint = 5*TEMPERATURE_MULTIPLICATION_FACTOR;  //5 degree
                uint16_t thermo_stat_max_heatpoint = 35*TEMPERATURE_MULTIPLICATION_FACTOR;  
                uint16_t thermo_stat_default_heatpoint = 20*TEMPERATURE_MULTIPLICATION_FACTOR;
            #endif      
            uint16_t thermo_stat_default_coolpoint = 25*TEMPERATURE_MULTIPLICATION_FACTOR;
            
            uint8_t thermo_running_state = ESP_ZB_ZCL_THERMOSTAT_RUNNING_STATE_COOL_STATE_ON_BIT;
            #ifdef USE_LOCAL_TEMPERATURE
                uint16_t thermo_local_temp = ESP_ZB_ZCL_THERMOSTAT_LOCAL_TEMPERATURE_MIN_VALUE;
                uint16_t local_temp_default_val = ESP_ZB_ZCL_THERMOSTAT_LOCAL_TEMPERATURE_CALIBRATION_MIN_VALUE;
            #endif
            uint8_t pi_heating_demand = 100;
            uint8_t pi_cooling_demand = 100;
            uint8_t setpoint_change_source_id = ESP_ZB_ZCL_CMD_THERMOSTAT_SETPOINT_RAISE_LOWER;
            
            esp_zb_attribute_list_t *esp_zb_thermo_stat_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT);
            //mode
            // esp_zb_cluster_add_attr(esp_zb_thermo_stat_cluster, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT, ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID,
            //                         ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &thermo_stat_mode);

            esp_zb_cluster_add_attr(esp_zb_thermo_stat_cluster, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT, ESP_ZB_ZCL_ATTR_THERMOSTAT_CONTROL_SEQUENCE_OF_OPERATION_ID,
                                    ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &thermo_stat_ctrl_seq);

            esp_zb_cluster_add_attr(esp_zb_thermo_stat_cluster, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT, ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID,
                                    ESP_ZB_ZCL_ATTR_TYPE_S16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &thermo_stat_default_coolpoint);

            //addition
            esp_zb_thermostat_cluster_add_attr(esp_zb_thermo_stat_cluster, ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_SETBACK_ID, &thermo_setback_id);
            esp_zb_thermostat_cluster_add_attr(esp_zb_thermo_stat_cluster, ESP_ZB_ZCL_ATTR_THERMOSTAT_UNOCCUPIED_SETBACK_ID, &thermo_setback_id);
            esp_zb_thermostat_cluster_add_attr(esp_zb_thermo_stat_cluster, ESP_ZB_ZCL_ATTR_THERMOSTAT_SETPOINT_CHANGE_AMOUNT_ID, &thermo_setback_change_amount);

            esp_zb_thermostat_cluster_add_attr(esp_zb_thermo_stat_cluster, ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_SETBACK_MAX_ID, &thermo_stat_max_coolpoint);
            esp_zb_thermostat_cluster_add_attr(esp_zb_thermo_stat_cluster, ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_SETBACK_MIN_ID, &thermo_stat_min_coolpoint);

            esp_zb_thermostat_cluster_add_attr(esp_zb_thermo_stat_cluster, ESP_ZB_ZCL_ATTR_THERMOSTAT_UNOCCUPIED_SETBACK_MAX_ID, &thermo_stat_max_coolpoint);
            esp_zb_thermostat_cluster_add_attr(esp_zb_thermo_stat_cluster, ESP_ZB_ZCL_ATTR_THERMOSTAT_UNOCCUPIED_SETBACK_MIN_ID, &thermo_stat_min_coolpoint);

            esp_zb_thermostat_cluster_add_attr(esp_zb_thermo_stat_cluster, ESP_ZB_ZCL_ATTR_THERMOSTAT_MAX_COOL_SETPOINT_LIMIT_ID, &thermo_stat_max_coolpoint);
            esp_zb_thermostat_cluster_add_attr(esp_zb_thermo_stat_cluster, ESP_ZB_ZCL_ATTR_THERMOSTAT_MIN_COOL_SETPOINT_LIMIT_ID, &thermo_stat_min_coolpoint);
            //more required
            esp_zb_thermostat_cluster_add_attr(esp_zb_thermo_stat_cluster, ESP_ZB_ZCL_ATTR_THERMOSTAT_RUNNING_MODE_ID, &thermo_stat_mode);
            esp_zb_thermostat_cluster_add_attr(esp_zb_thermo_stat_cluster, ESP_ZB_ZCL_ATTR_THERMOSTAT_THERMOSTAT_RUNNING_STATE_ID, &thermo_running_state);

            esp_zb_thermostat_cluster_add_attr(esp_zb_thermo_stat_cluster, ESP_ZB_ZCL_ATTR_THERMOSTAT_HVAC_SYSTEM_TYPE_CONFIGURATION_ID, &thermo_hvac_system_type);
            esp_zb_thermostat_cluster_add_attr(esp_zb_thermo_stat_cluster, ESP_ZB_ZCL_ATTR_THERMOSTAT_SETPOINT_CHANGE_SOURCE_ID, &setpoint_change_source_id); 

            #ifdef USE_HEATING_PIPES
                esp_zb_thermostat_cluster_add_attr(esp_zb_thermo_stat_cluster, ESP_ZB_ZCL_ATTR_THERMOSTAT_MAX_HEAT_SETPOINT_LIMIT_ID, &thermo_stat_max_heatpoint);
                esp_zb_thermostat_cluster_add_attr(esp_zb_thermo_stat_cluster, ESP_ZB_ZCL_ATTR_THERMOSTAT_MIN_HEAT_SETPOINT_LIMIT_ID, &thermo_stat_min_heatpoint);
                esp_zb_thermostat_cluster_add_attr(esp_zb_thermo_stat_cluster, ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID, &thermo_stat_default_heatpoint);  
            #endif
            #ifdef USE_LOCAL_TEMPERATURE
                esp_zb_thermostat_cluster_add_attr(esp_zb_thermo_stat_cluster, ESP_ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID, &thermo_local_temp);
                esp_zb_thermostat_cluster_add_attr(esp_zb_thermo_stat_cluster, ESP_ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_CALIBRATION_ID, &local_temp_default_val);
            #endif
            #ifdef USE_WEEKLY_TRANSITION
                uint8_t thermo_start_of_week = ESP_ZB_ZCL_THERMOSTAT_START_OF_WEEK_DEFAULT_VALUE;
                uint8_t thermo_weekly_transition = 7;
                uint8_t thermo_daily_transition = 1;
                esp_zb_thermostat_cluster_add_attr(esp_zb_thermo_stat_cluster, ESP_ZB_ZCL_ATTR_THERMOSTAT_START_OF_WEEK_ID, &thermo_start_of_week);
                esp_zb_thermostat_cluster_add_attr(esp_zb_thermo_stat_cluster, ESP_ZB_ZCL_ATTR_THERMOSTAT_NUMBER_OF_WEEKLY_TRANSITIONS_ID, &thermo_weekly_transition);
                esp_zb_thermostat_cluster_add_attr(esp_zb_thermo_stat_cluster, ESP_ZB_ZCL_ATTR_THERMOSTAT_NUMBER_OF_DAILY_TRANSITIONS_ID, &thermo_daily_transition);      
            #endif
            #ifdef USE_FAN_CTRL
                esp_zb_attribute_list_t *esp_zb_fan_cluster = nuos_zb_create_fan_cluster();
                esp_zb_cluster_list_add_fan_control_cluster(esp_zb_cluster_list, esp_zb_fan_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
            #endif

            #ifdef USE_TEMPERATURE_MESAUREMENT
                esp_zb_attribute_list_t *esp_zb_temp_measurement_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT);
                esp_zb_cluster_list_add_temperature_meas_cluster(esp_zb_cluster_list, esp_zb_temp_measurement_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
            #endif

            esp_zb_cluster_list_add_thermostat_cluster(esp_zb_cluster_list, esp_zb_thermo_stat_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);       

            #ifdef TUYA_ATTRIBUTES
            uint8_t temp = 25;
            uint8_t ss = 0;
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
                temp = 0;
            esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE); 

            
            esp_zb_attribute_list_t *esp_zb_tuya_private_cluster1 = esp_zb_zcl_attr_list_create(0xEE00);
            esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, esp_zb_tuya_private_cluster1, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

            esp_zb_attribute_list_t *esp_zb_tuya_private_cluster = esp_zb_zcl_attr_list_create(0xEF00);
            esp_zb_custom_cluster_add_custom_attr(esp_zb_tuya_private_cluster, 0x0101, ESP_ZB_ZCL_ATTR_TYPE_BOOL, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &ss);
            esp_zb_custom_cluster_add_custom_attr(esp_zb_tuya_private_cluster, 0x0202, ESP_ZB_ZCL_ATTR_TYPE_S16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &temp);
            esp_zb_custom_cluster_add_custom_attr(esp_zb_tuya_private_cluster, 0x0203, ESP_ZB_ZCL_ATTR_TYPE_S16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &temp);  
            esp_zb_custom_cluster_add_custom_attr(esp_zb_tuya_private_cluster, 0x026D, ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &temp);

            //TRVMode
            esp_zb_custom_cluster_add_custom_attr(esp_zb_tuya_private_cluster, 0x4000, ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &ss);
            //ValvePosition
            esp_zb_custom_cluster_add_custom_attr(esp_zb_tuya_private_cluster, 0x4001, ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &ss);
            //EurotronicErrors
            esp_zb_custom_cluster_add_custom_attr(esp_zb_tuya_private_cluster, 0x4002, ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &ss);
            //CurrentTemperatureSetPoint
            esp_zb_custom_cluster_add_custom_attr(esp_zb_tuya_private_cluster, 0x4003, ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &temp);
            //EurotronicHostFlags
            esp_zb_custom_cluster_add_custom_attr(esp_zb_tuya_private_cluster, 0x4008, ESP_ZB_ZCL_ATTR_TYPE_U24, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &ss);
            //th_setpoint
            esp_zb_custom_cluster_add_custom_attr(esp_zb_tuya_private_cluster, 0xFFF0, ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &ss);
            //target_temp
            esp_zb_custom_cluster_add_custom_attr(esp_zb_tuya_private_cluster, 0xFFF1, ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &temp);
            esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, esp_zb_tuya_private_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);  
            #else
                
            #endif

            #endif

        }else if(index == 1){    
            esp_zb_attribute_list_t *esp_zb_thermo_stat_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT);
            //mode
            esp_zb_cluster_add_attr(esp_zb_thermo_stat_cluster, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT, ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID,
                                    ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &thermo_stat_mode);

            esp_zb_cluster_add_attr(esp_zb_thermo_stat_cluster, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT, ESP_ZB_ZCL_ATTR_THERMOSTAT_CONTROL_SEQUENCE_OF_OPERATION_ID,
                                    ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &thermo_stat_ctrl_seq);

            esp_zb_cluster_list_add_thermostat_cluster(esp_zb_cluster_list, esp_zb_thermo_stat_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE); 
        }else {
            uint8_t ss = 0;
            esp_zb_attribute_list_t *esp_zb_tuya_private_cluster = esp_zb_zcl_attr_list_create(0xFC31);
            esp_zb_custom_cluster_add_custom_attr(esp_zb_tuya_private_cluster, 0x0000, ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &ss);
            esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, esp_zb_tuya_private_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

            #ifdef CUSTON_FAN_CTRL
                uint8_t fan_current_level = 2;
                uint8_t fan_min_level = 0;
                uint8_t fan_max_level = 5;
                esp_zb_attribute_list_t* esp_zb_level_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL);
                esp_zb_level_cluster_add_attr(esp_zb_level_cluster, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID, &fan_current_level);
                esp_zb_level_cluster_add_attr(esp_zb_level_cluster, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_MIN_LEVEL_ID, &fan_min_level);
                esp_zb_level_cluster_add_attr(esp_zb_level_cluster, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_MAX_LEVEL_ID, &fan_max_level);
                esp_zb_cluster_list_add_level_cluster(esp_zb_cluster_list, esp_zb_level_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);  
                esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
            #endif
        }           
	#endif

    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
        #ifdef USE_CUSTOM_SCENE
            esp_zb_attribute_list_t* esp_zb_groups_cluster = nuos_zb_create_group_cluster();
            esp_zb_cluster_list_add_groups_cluster(esp_zb_cluster_list, esp_zb_groups_cluster, esp_zb_zcl_group_role);
            esp_zb_attribute_list_t *esp_zb_scenes_cluster = nuos_zb_create_scene_cluster();
            esp_zb_cluster_list_add_scenes_cluster(esp_zb_cluster_list, esp_zb_scenes_cluster, esp_zb_zcl_scene_role); 
        #endif     
    #else
        esp_zb_attribute_list_t* esp_zb_groups_cluster = nuos_zb_create_group_cluster();
        esp_zb_cluster_list_add_groups_cluster(esp_zb_cluster_list, esp_zb_groups_cluster, esp_zb_zcl_group_role);
        printf("Created Cluster Group\n");
        esp_zb_attribute_list_t *esp_zb_scenes_cluster = nuos_zb_create_scene_cluster();
        esp_zb_cluster_list_add_scenes_cluster(esp_zb_cluster_list, esp_zb_scenes_cluster, esp_zb_zcl_scene_role);  
        printf("Created Cluster Scene\n");
    #endif 
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM    \
        || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_ANALOG_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4R_ON_OFF_LIGHT                              \
        || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2R_ON_OFF_LIGHT)   
    touchlink_cluster = esp_zb_touchlink_commissioning_cluster_create(NULL);
        /* Add cluster */
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_touchlink_commissioning_cluster(esp_zb_cluster_list, touchlink_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    #endif

    return esp_zb_cluster_list;
}

/* ********************************************************************************** */
void nuos_init_privilege_commands(){
    #ifdef USE_TUYA_BRDIGE
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI)
            //select mode
            esp_zb_zcl_add_privilege_command(ENDPOINTS_LIST[0], ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0XF0); // mode
            //mode == 0
            esp_zb_zcl_add_privilege_command(ENDPOINTS_LIST[0], ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL, 0XF0); // brightness
            esp_zb_zcl_add_privilege_command(ENDPOINTS_LIST[0], ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0XE0); // set color temperature
            //mode == 1
            esp_zb_zcl_add_privilege_command(ENDPOINTS_LIST[0], ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0XE1); // set color hsv
            //mode == 2
            esp_zb_zcl_add_privilege_command(ENDPOINTS_LIST[0], ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0XF1); // scene
            //
            esp_zb_zcl_add_privilege_command(ENDPOINTS_LIST[0], ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0XF2); // music
            
            esp_zb_zcl_add_privilege_command(ENDPOINTS_LIST[0], ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL, 0XF0); //brightness
            esp_zb_zcl_add_privilege_command(ENDPOINTS_LIST[0], ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0xF1); //scene data
            esp_zb_zcl_add_privilege_command(ENDPOINTS_LIST[0], ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0xE0); // set color temperature
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_DALI_DIRECT_SWITCH) 
            // esp_zb_zcl_add_privilege_command(ENDPOINTS_LIST[0], 0xE000, 0Xe0);
            // esp_zb_zcl_add_privilege_command(ENDPOINTS_LIST[1], 0xE000, 0Xe0);
            esp_zb_zcl_add_privilege_command(ENDPOINTS_LIST[0], ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL, 0Xf0);
            esp_zb_zcl_add_privilege_command(ENDPOINTS_LIST[1], ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL, 0Xf0);
            // esp_zb_zcl_add_privilege_command(ENDPOINTS_LIST[0], 0xEF00, 0X00);
            // esp_zb_zcl_add_privilege_command(ENDPOINTS_LIST[1], 0xEF00, 0X00);
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER  || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN)
            // esp_zb_zcl_add_privilege_command(ENDPOINTS_LIST[0], 6, 1); //on
            // esp_zb_zcl_add_privilege_command(ENDPOINTS_LIST[0], 6, 0); //off
            // esp_zb_zcl_add_privilege_command(ENDPOINTS_LIST[0], 6, 2); //toggle 
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI)
            // esp_zb_zcl_add_privilege_command(ENDPOINTS_LIST[0], ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL, 0XF0); // brightness
            // esp_zb_zcl_add_privilege_command(ENDPOINTS_LIST[0], 0xEE00, 1);                      
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_SCENE_SWITCH)
            // esp_zb_zcl_add_privilege_command(ENDPOINTS_LIST[0], 0xE001, 0xD004);
            // esp_zb_zcl_add_privilege_command(ENDPOINTS_LIST[1], 0xE001, 0xFC);
            // esp_zb_zcl_add_privilege_command(ENDPOINTS_LIST[2], 0xE001, 0xFC);
            // esp_zb_zcl_add_privilege_command(ENDPOINTS_LIST[3], 0xE001, 0xFC);
        #else
            //Nothing to do here
        #endif
    #endif
}



esp_zb_ep_list_t * nuos_init_zb_clusters()
{
    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();
    printf("ENDPOINTS_LIST = { \n");
    for(uint8_t i=0; i<TOTAL_ENDPOINTS; i++){
        esp_zb_endpoint_config_t endpoint_config = {
            .endpoint = ENDPOINTS_LIST[i],
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = ENDPOINTS_TYPE[i],
            .app_device_version = 3,
        };
        
        esp_zb_cluster_list_t *clusters = server_cluster(i);
        esp_zb_ep_list_add_ep(esp_zb_ep_list,
    		   clusters,
    		   endpoint_config);       
       ESP_LOGI(TAG, "ENDPOINTS_LIST[%d]= %d, ENDPOINTS_TYPE = %ld", i, ENDPOINTS_LIST[i], ENDPOINTS_TYPE[i]);
    }
    printf("}\n");
    return esp_zb_ep_list;
}


// Function to interpolate between two colors
Color interpolate_color(Color start, Color end, float fraction) {
    Color result;
    result.r = (uint8_t)(start.r + (end.r - start.r) * fraction);
    result.g = (uint8_t)(start.g + (end.g - start.g) * fraction);
    result.b = (uint8_t)(start.b + (end.b - start.b) * fraction);
    return result;
}

// Function to handle LED transition
void led_transition(Color start_color, Color end_color, uint16_t transition_time_ms, uint16_t effect_duration_ms) {
    uint16_t steps = effect_duration_ms; // Number of steps for the transition
    uint16_t delay_per_step = transition_time_ms / steps;

    for (uint16_t i = 0; i < steps; i++) {
        float fraction = (float)i / (steps - 1);
        Color current_color = interpolate_color(start_color, end_color, fraction);
        device_info[0].device_level = current_color.r;
        device_info[1].device_level = current_color.g;
        device_info[2].device_level = current_color.b;
        nuos_zb_set_hardware(4, false); 
        usleep(transition_time_ms * 1000); // Sleep for delay_per_step milliseconds
    }
}

void nuos_set_privilege_command_attribute_in_queue(const esp_zb_zcl_privilege_command_message_t *message){

    uint48_t color_data;
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI)
        if (message->info.dst_endpoint == ENDPOINTS_LIST[0]) {

            if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL) {
                
                switch(message->info.command.id){
                    case 0xF0:
                        printf("MODES: %d  %d\n", selected_color_mode, *(uint8_t*)message->data);
                        last_selected_color_mode = *(uint8_t*)message->data;
                        if(selected_color_mode != last_selected_color_mode){
                            selected_color_mode = last_selected_color_mode;
                            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX) 

                                nuos_set_color_rgb_mode_attribute(0, selected_color_mode);
                                store_color_mode_value(selected_color_mode);

                                printf("Mode Changed!!\n");
                                #if(USE_COLOR_DEVICE == COLOR_RGBW || USE_COLOR_DEVICE == COLOR_RGB_CW_WW)
                                mode_change_flag = true;
                                #endif
                                
                                if(selected_color_mode == 0){
                                    device_info[3].device_state = true; 
                                    nuos_zb_set_hardware(3, false); 
                                    set_dali_color_temp(0, false);   
                                    set_dali_level(3);                                          

                                }else if(selected_color_mode == 1){
                                    device_info[4].device_state = true;

                                    for(int rgb=0; rgb<=3; rgb++){
                                        if(device_info[rgb].device_level == 0xff){
                                            device_info[rgb].device_level = 0xfe;
                                        }
                                    } 
                                    // printf("SR:%d SG:%d SB:%d\n", device_info[0].device_state, device_info[1].device_state, device_info[2].device_state);
                                    // printf("R:%d G:%d B:%d\n", device_info[0].device_level, device_info[1].device_level, device_info[2].device_level);
                                    // printf("DR:%d DG:%d DB:%d\n", dmx_data[dmx_start_address], dmx_data[dmx_start_address+1], dmx_data[dmx_start_address+2]);
                                    nuos_zb_set_hardware(4, false); 
                                    set_state(4); 
                                    set_dali_level(4); 
                                    set_dali_color_temp(0, false);               
                                }                                                      
                            #else
                                mode_change_flag = false;    
                            #endif
                        }else{
                            mode_change_flag = false;
                        }
                    break;
                    case 0xF2:  //HSVBT Music
                        // uint72_t* music_data = (uint72_t*)message->data;

                        // uint16_t hh = (music_data->bytes[3] >> 7) & 0x01; //[7]);  h
                        // uint16_t h1 = (hh << 8) | music_data->bytes[4];

                        // uint16_t ss = (music_data->bytes[5] >> 6) & 0x03; //[7:6]  s                      
                        // uint16_t s1 = (ss << 8) | music_data->bytes[6];
      
                        // uint16_t vv = (music_data->bytes[5] >> 4) & 0x03; //[5:4]  v
                        // uint16_t v1 = (vv << 8) | music_data->bytes[7];   

                        // //brightness
                        // uint16_t bb = (music_data->bytes[5] >> 2) & 0x03; //[3:2]  B
                        // uint16_t b1 = (vv << 8) | music_data->bytes[8];

                        // //color temperature
                        // uint16_t cct = music_data->bytes[5] & 0x03;        //[1:0] T
                        // uint16_t cct1 = (vv << 8) | music_data->bytes[9];

                        // uint8_t mode1 = ((music_data->bytes[1] >> 6) & 0x02) |  ((music_data->bytes[2] >> 7) & 0x01);
                        // uint8_t transition_time = (music_data->bytes[2] & 0x07); //in sec
                        // uint8_t effect_duration = (music_data->bytes[3] & 0x07); //in nsec

                        // hsv_t hsv1 = {.h=h1, .s=s1, .v=v1};      // Example HSV values
                        // rgb_t rgb1 = hsv_to_rgb(hsv1);

                        // Color start = {device_info[0].device_level, device_info[1].device_level, device_info[2].device_level};
                        
                        // start.r = device_info[0].device_level;
                        // start.g = device_info[1].device_level;
                        // start.b = device_info[2].device_level;

                        // device_info[0].device_level = rgb1.r;
                        // device_info[1].device_level = rgb1.g;
                        // device_info[2].device_level = rgb1.b;

                        // Color end = {device_info[0].device_level, device_info[1].device_level, device_info[2].device_level};

                        // if(device_info[0].device_level == 0) device_info[0].device_state = false;
                        // else device_info[0].device_state = true;
                        // if(device_info[1].device_level == 0) device_info[1].device_state = false;
                        // else device_info[1].device_state = true;
                        // if(device_info[2].device_level == 0) device_info[2].device_state = false;
                        // else device_info[2].device_state = true;
                        // if(device_info[3].device_level == 0) device_info[3].device_state = false;
                        // else device_info[3].device_state = true;
                        
                        // for(int rgb=0; rgb<=3; rgb++){
                        //     if(device_info[rgb].device_level <= MIN_DIM_LEVEL_VALUE) {
                        //         device_info[rgb].device_level = MIN_DIM_LEVEL_VALUE;
                        //         device_info[rgb].device_state = false;
                        //     }else{
                        //         device_info[rgb].device_state = true;
                        //     }
                        //     if(device_info[rgb].device_level == 0xff){
                        //         device_info[rgb].device_level = 0xfe;
                        //     }
                        // } 
                        // device_info[3].device_state = false;    
                        // nuos_zb_set_hardware(4, false); 
                        // nuos_set_color_xy_attribute(4, &hsv1);
                    break;
                    case 0xF1:  //set scene
                        uint72_t* scene_data = (uint72_t*)message->data;

                        uint16_t hhf1 = (scene_data->bytes[3] >> 7) & 0x01; //[7]);
                        uint16_t h1f1 = (hhf1 << 8) | scene_data->bytes[4];

                        uint16_t ssf1 = (scene_data->bytes[5] >> 6) & 0x03; //[7:6]                        
                        uint16_t s1f1 = (ssf1 << 8) | scene_data->bytes[6];
      
                        uint16_t vvf1 = (scene_data->bytes[5] >> 4) & 0x03; //[5:4]
                        uint16_t v1f1 = (vvf1 << 8) | scene_data->bytes[7];   

                        //brightness
                        uint16_t bbf1 = (scene_data->bytes[5] >> 2) & 0x03; //[3:2]
                        uint16_t b1f1 = (vvf1 << 8) | scene_data->bytes[8];

                        //color temperature
                        uint16_t cctf1 = scene_data->bytes[5] & 0x03;        //[1:0]
                        uint16_t cct1f1 = (vvf1 << 8) | scene_data->bytes[9];

                        uint8_t mode1f1 = ((scene_data->bytes[1] >> 6) & 0x02) |  ((scene_data->bytes[2] >> 7) & 0x01);
                        uint8_t transition_timef1 = (scene_data->bytes[2] & 0x07); //in sec
                        uint8_t effect_durationf1 = (scene_data->bytes[3] & 0x07); //in nsec

                        hsv_t hsvf1 = {.h=h1f1, .s=s1f1, .v=v1f1};      // Example HSV values
                        device_info[4].device_val = v1f1;
                        // device_info[4].light_color_x = hsvf1.h;
                        // device_info[4].light_color_y = hsvf1.s;
                        rgb_t rgbf1 = hsv_to_rgb(hsvf1);

                        Color startf1 = {device_info[0].device_level, device_info[1].device_level, device_info[2].device_level};
                        
                        startf1.r = device_info[0].device_level;
                        startf1.g = device_info[1].device_level;
                        startf1.b = device_info[2].device_level;

                        device_info[0].device_level = rgbf1.r;
                        device_info[1].device_level = rgbf1.g;
                        device_info[2].device_level = rgbf1.b;
                        device_info[4].device_level = (v1f1*254)/1000;

                        Color endf1 = {device_info[0].device_level, device_info[1].device_level, device_info[2].device_level};

                        if(device_info[0].device_level == 0) device_info[0].device_state = false;
                        else device_info[0].device_state = true;
                        if(device_info[1].device_level == 0) device_info[1].device_state = false;
                        else device_info[1].device_state = true;
                        if(device_info[2].device_level == 0) device_info[2].device_state = false;
                        else device_info[2].device_state = true;
                        if(device_info[3].device_level == 0) device_info[3].device_state = false;
                        else device_info[3].device_state = true;
                        
                        for(int rgb=0; rgb<=3; rgb++){
                            if(device_info[rgb].device_level <= MIN_DIM_LEVEL_VALUE) {
                                device_info[rgb].device_level = MIN_DIM_LEVEL_VALUE;
                                device_info[rgb].device_state = false;
                            }else{
                                device_info[rgb].device_state = true;
                            }
                            if(device_info[rgb].device_level == 0xff){
                                device_info[rgb].device_level = 0xfe;
                            }
                        } 
                        device_info[3].device_state = false;   
                        if(!device_info[4].device_state){
                            device_info[4].device_state = true;
                            change_state_flag = true;
                        }
                        
                        
                        nuos_zb_set_hardware(4, false); 
                        if(change_state_flag){
                            change_state_flag = false;
                            set_state(4);
                            set_dali_level(4);
                        }
                        set_dali_color_temp(0, true); 
                    break;

                    case 0xE1:
                        if(selected_color_mode != 1){
                            selected_color_mode = 1;
                            mode_change_flag = true;
                            store_color_mode_value(selected_color_mode);
                        }
                        memcpy(color_data.bytes, message->data, sizeof(color_data.bytes));
                        uint16_t h = color_data.bytes[1]<<8 | color_data.bytes[0];
                        uint16_t s = color_data.bytes[3]<<8 | color_data.bytes[2];
                        uint16_t v = color_data.bytes[5]<<8 | color_data.bytes[4];
 
                        #if(USE_COLOR_DEVICE == COLOR_RGB_ONLY)
                        device_info[3].device_level = (uint8_t)((v*254)/1000);
                        device_info[4].device_level = device_info[3].device_level;
                        #else
                        device_info[4].device_val = v;
                        // device_info[4].light_color_x = h;
                        // device_info[4].light_color_y = s;
                        device_info[4].device_level = map_1000(v, 10, 1000, 0, 254);
                        if(device_info[4].device_level >= 254) device_info[4].device_level = 254;
                        //printf("LEVELYZ:%d   v:%d\n", device_info[4].device_level, v);
                        #endif  
          
                        hsv_t hsv = {.h=h, .s=s, .v=v}; // Example HSV values
                        rgb_t rgb = hsv_to_rgb(hsv);
                        
                        device_info[0].device_level = rgb.r;
                        device_info[1].device_level = rgb.g;
                        device_info[2].device_level = rgb.b;

                        printf("RGB:%d  %d  %d\n", device_info[0].device_level, device_info[1].device_level, device_info[2].device_level);

                        if(device_info[0].device_level == 0) device_info[0].device_state = false;
                        else device_info[0].device_state = true;
                        if(device_info[1].device_level == 0) device_info[1].device_state = false;
                        else device_info[1].device_state = true;
                        if(device_info[2].device_level == 0) device_info[2].device_state = false;
                        else device_info[2].device_state = true;

                        // if(device_info[3].device_level == 0) device_info[3].device_state = false;
                        // else device_info[3].device_state = false;

                        for(int rgb=0; rgb<3; rgb++){

                            if(device_info[rgb].device_level <= MIN_DIM_LEVEL_VALUE) {
                                device_info[rgb].device_level = MIN_DIM_LEVEL_VALUE;
                                //device_info[rgb].device_state = false;
                            }else{
                                //device_info[rgb].device_state = true;
                            }
                            if(device_info[rgb].device_level == 0xff){
                                device_info[rgb].device_level = 0xfe;
                            }
                        } 
                        #if(USE_COLOR_DEVICE == COLOR_RGB_ONLY)
                        device_info[3].device_state = true;
                        if(!device_info[4].device_state){
                            device_info[4].device_state = true;
                            change_state_flag = true;
                        }                              
                            nuos_zb_set_hardware(4, false); 
                            if(change_state_flag || mode_change_flag){
                               change_state_flag = false;
                                set_state(4);
                            }
                            set_dali_color_temp(0, false);
                                                 
                        #else
                            device_info[3].device_state = false;
                            if(!device_info[4].device_state){
                                device_info[4].device_state = true;
                                change_state_flag = true;
                            } 
                            nuos_zb_set_hardware(4, false); 

                            if(change_state_flag || mode_change_flag){
                                change_state_flag = false;
                                set_state(4);
                            } 
                            set_dali_color_temp(0, true);                    
                        #endif     
                    break;

                    case 0xE0:
                        if(selected_color_mode == 1){
                            selected_color_mode = 0;
                            mode_change_flag = true;
                            store_color_mode_value(selected_color_mode);
                        }
                        uint16_t val = *(uint16_t*)message->data;
                        device_info[3].device_val = map_1000(val, 10, 1000, MIN_CCT_VALUE, MAX_CCT_VALUE);
                        if(device_info[3].device_val >= MAX_CCT_VALUE) device_info[3].device_val = MAX_CCT_VALUE;
                        // for(int rgb=0; rgb<3; rgb++){
                        //     device_info[rgb].device_state = false;
                        // } 
                        if(!device_info[3].device_state){
                            device_info[3].device_state = true;
                            change_state_flag = true;
                        }
                        nuos_zb_set_hardware(3, false);
                        if(change_state_flag || mode_change_flag){
                            change_state_flag = false;
                            set_state(3);
                        }
                        set_dali_level(3);
                        set_dali_color_temp(0, true);  
                                               
                    break;
                    case 0xFD:
                        printf("white_gradient:%u\n", *(uint16_t*)message->data);
                    break;
                    default: break;        
                }
            }else if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL) {
                switch(message->info.command.id){
                    case 0xF0:
                        if(selected_color_mode == 1){
                            selected_color_mode = 0;
                            mode_change_flag = true;
                            store_color_mode_value(selected_color_mode);
                        }
                        uint16_t vall = *(uint16_t*)message->data;
                        uint32_t val2 = (vall*254);
                        device_info[3].device_level = (uint8_t)(val2/1000);
                        printf("LEVEl_X:%d\n", device_info[3].device_level);
                        #if(USE_COLOR_DEVICE == COLOR_RGB_ONLY)
                        selected_color_mode = 1;
                            device_info[4].device_level = device_info[3].device_level;
                        #else
                        // for(int rgb=0; rgb<3; rgb++){
                        //     device_info[rgb].device_state = false;
                        // }                    
                        #endif   
                        if(!device_info[3].device_state){
                            device_info[3].device_state = true;
                            change_state_flag = true;
                        } 
                        nuos_zb_set_hardware(3, false);  
                        if(change_state_flag || mode_change_flag){
                            change_state_flag = false;
                            set_state(3); 
                            set_dali_color_temp(0, true);
                            //nuos_set_state_attribute(3);
                        }
                        set_dali_level(3);  
                    break;
                    default: break;                      
                }
            }
        }  
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
        if (message->info.dst_endpoint == ENDPOINTS_LIST[0]) {
            printf("message->info.command.id:%x\n", message->info.command.id);
            if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL) {
                switch(message->info.command.id){
                    case 0xf1:  //set scene
                        printf("scene_data:%u\n", *(uint16_t*)message->data);
                        uint8_t scene_data = *(uint8_t*)message->data;
                        switch(scene_data){
                            case 0:
                            device_info[0].device_val = MIN_CCT_VALUE;
                            break;
                            case 1:
                            device_info[0].device_val = MIN_CCT_VALUE_2;
                            break;
                            case 2:
                            device_info[0].device_val = MAX_CCT_VALUE;
                            break;
                            case 3:
                            device_info[0].device_val = MIN_CCT_VALUE_1;
                            break;                                                        
                            default: break; 
                        }
                        is_long_press_brightness = false;
                        device_info[0].color_or_fan_state = 1;
                        nuos_set_hardware_brightness_2(1);
                        set_dali_color_temp(0, false);
                    break;
                    case 0xe0:
                        // device_info[0].device_state = 1;
                        if(ep_cnts > TOTAL_ENDPOINTS) ep_cnts = 0;
                        // ESP_LOGI(TAG, "---------------->ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL\n<----------------"); 
                        if (ep_cnts < TOTAL_ENDPOINTS && !is_value_present(ep_id, ep_cnts, 0)) {
                            ep_id[0] = 0;
                            ep_cnts++;
                        }                      
                        printf("color_temp:%u\n", *(uint16_t*)message->data);  
                        device_info[0].device_val = map_1000(*(uint16_t*)message->data, 10, 1000, MIN_CCT_VALUE, MAX_CCT_VALUE);
                        if(device_info[0].device_val >= MAX_CCT_VALUE) device_info[0].device_val = MAX_CCT_VALUE;
                        printf("mapped_val:%u\n", device_info[0].device_val);                 
                        // nuos_zb_set_hardware(0, false); //
                        if(!device_info[0].device_state){
                            device_info[0].device_state = true;
                            state_change_flag = true;
                        }                        
                        is_long_press_brightness = false;
                        device_info[0].color_or_fan_state = 1;
                        nuos_set_hardware_brightness_2(1);
                        if(state_change_flag){
                            state_change_flag = false;
                            set_state(0);
                            set_dali_level(0);
                        }                                              
                        set_dali_color_temp(0, false);
                    break;
                    default: break;        
                }
            }else if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL) {
                switch(message->info.command.id){
                    case 0xf0:
                        if(ep_cnts > TOTAL_ENDPOINTS) ep_cnts = 0;
                        // ESP_LOGI(TAG, "---------------->ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL\n<----------------"); 
                        if (ep_cnts < TOTAL_ENDPOINTS && !is_value_present(ep_id, ep_cnts, 0)) {
                            ep_id[0] = 0;
                            ep_cnts++;
                        }                        
                        device_info[0].device_level = map_1000(*(uint16_t*)message->data, 10, 1000, MIN_DIM_LEVEL_VALUE, MAX_DIM_LEVEL_VALUE);
                        if(device_info[0].device_level >= MAX_DIM_LEVEL_VALUE) device_info[0].device_level = MAX_DIM_LEVEL_VALUE;
                        printf("LEVELX:%d\n", device_info[0].device_level);
                        if(!device_info[0].device_state){
                            device_info[0].device_state = true;
                            state_change_flag = true;
                        } 
                        device_info[0].color_or_fan_state = 0;
                        is_long_press_brightness = false;
                        nuos_set_hardware_brightness_2(0);

                        //nuos_zb_set_hardware(0, false);
                        if(state_change_flag){
                            state_change_flag = false;
                            set_state(0);
                        }
                        set_dali_level(0);
                    break;
                    default: break;        
                }
            }
        } 
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER)
        if (message->info.dst_endpoint == ENDPOINTS_LIST[0]) {
                printf("cluster:0x%x  data:%u\n", message->info.cluster, *(uint16_t*)message->data);
            // if (message->info.cluster == 0xEF00) {
            //     switch(message->info.command.id){
            //         case 0x0:
            //             printf("on-off:%u\n", *(uint16_t*)message->data);
            //         break;

            //         case 0x10:
            //             printf("setr-temp:%u\n", *(uint16_t*)message->data);
            //         break;

            //         default: break;        
            //     }
            // }
        }  
    #else
        //if (message->info.dst_endpoint == ENDPOINTS_LIST[0]) {
            printf("cluster:0x%x  data:%u\n", message->info.cluster, *(uint16_t*)message->data);
            if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
                #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN)
                    #ifdef USE_TUYA_BRDIGE
                        switch(message->info.command.id){
                            case 0x0:
                                printf("on data\n");
                                device_info[LIGHT_INDEX].device_state = false;
                                nuos_zb_set_hardware(LIGHT_INDEX, false);
                            break;
                            case 0x1:
                                printf("off data\n");
                                device_info[LIGHT_INDEX].device_state = true;
                                nuos_zb_set_hardware(LIGHT_INDEX, false);
                            break;
                            case 0x2:
                                printf("toggle data\n");
                                nuos_zb_set_hardware(LIGHT_INDEX, true);
                            break;
                            default: break;        
                        }
                    #endif
                #endif
            }else if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL) {
                #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_DALI_DIRECT_SWITCH)

                    uint16_t level_value = *(uint16_t*)message->data;
                    uint8_t dst_ep = message->info.dst_endpoint;
                    uint8_t ep_index = dst_ep-1;
                    esp_zb_zcl_status_t status = ESP_OK;

                    if (is_my_device_commissionned && !wifi_webserver_active_flag){
                        status = esp_zb_zcl_set_attribute_val(
                            ENDPOINTS_LIST[ep_index],
                            ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
                            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                            0xF000,
                            (uint8_t*)&level_value,
                            false
                        );

                        send_report(ep_index, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, 0xF000);
                    }   


                        if(ep_cnts > TOTAL_ENDPOINTS) ep_cnts = 0;
                        // ESP_LOGI(TAG, "---------------->ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL\n<----------------"); 
                        if (ep_cnts < TOTAL_ENDPOINTS && !is_value_present(ep_id, ep_cnts, 0)) {
                            ep_id[ep_index] = ep_index;
                            ep_cnts++;
                        }                        
                        device_info[ep_index].device_level = map_1000(level_value, 10, 1000, MIN_DIM_LEVEL_VALUE, MAX_DIM_LEVEL_VALUE);
                        if(device_info[ep_index].device_level >= MAX_DIM_LEVEL_VALUE) device_info[ep_index].device_level = MAX_DIM_LEVEL_VALUE;
                        printf("LEVELX:%d\n", device_info[ep_index].device_level);
                        if(!device_info[ep_index].device_state){
                            device_info[ep_index].device_state = true;
                            state_change_flag = true;
                        } 
                        nuos_zb_set_hardware(ep_index, false);
                #endif
            }
        //}                 
    #endif 

}

void nuos_set_privilege_command_attribute(const esp_zb_zcl_privilege_command_message_t *message){
    #ifdef USE_QUEUE_CTRL
    xQueueSend(privilege_evt_queue, message, 0); // Non-blocking send
    //xQueueSend(privilege_evt_queue, (esp_zb_zcl_privilege_command_message_t*)message, 0); // Non-blocking send
    #else
    nuos_set_privilege_command_attribute_in_queue(message);
    #endif
       
}


// Function to check if a value exists in the array
bool is_value_present(uint8_t arr[], uint8_t size, uint8_t value) {
    for (uint8_t i = 0; i < size; i++) {
        if (arr[i] == value) {
            return true; // Value already exists
        }
    }
    return false; // Value does not exist
}



uint8_t get_maximum_of_3_numbers(uint8_t num1, uint8_t num2, uint8_t num3){
        // Calculate the maximum
    uint8_t max = num1; // Assume num1 is the largest initially
    if (num2 > max) {
        max = num2;
    }
    if (num3 > max) {
        max = num3;
    }
    return max;
}

void scale_rgb(int *r, int *g, int *b, int overall_value) {
    int original_sum = *r + *g + *b;
    if (original_sum == 0) return; // Prevent division by zero
    
    float scale_factor = (float)overall_value / original_sum;
    
    *r = (int)(*r * scale_factor);
    *g = (int)(*g * scale_factor);
    *b = (int)(*b * scale_factor);
}




void nuos_set_attribute_cluster_2(const esp_zb_zcl_set_attr_value_message_t *message){
    uint8_t index = 0xff;
    set_state_flag = false;
    set_level_flag = false;
    set_color_flag = false;
    //recheckTimer();
    
    for(uint8_t i=0; i<TOTAL_ENDPOINTS; i++)  {
        if (message->info.dst_endpoint == ENDPOINTS_LIST[i]) {
            index = i;
            break;
        }
    }

        if (index != 0xff) {
            //send_report(index, message->info.cluster, message->attribute.id);
            if (message->info.cluster == 0xe000) {
                // ESP_LOGI(TAG, "---------------->ESP_ZB_ZCL_CLUSTER_ID_ON_OFF\n<----------------");
                if (message->attribute.id == 0xD004 && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
                    uint16_t val1 = *(uint16_t *)message->attribute.data.value;
                    printf("val1: %d\n", val1);
                }else if (message->attribute.id == 0xD005 && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
                    uint16_t val2 = *(uint16_t *)message->attribute.data.value;
                    printf("val2: %d\n", val2);
                }    
            } else if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING) { //window covering
                #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN)
                static uint8_t last_value = 0xFF;
                if (message->attribute.id == 0xF003 && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
                    // if(*(uint16_t *)message->attribute.data.value != last_value){
                    //     last_value = *(uint16_t *)message->attribute.data.value;
                        device_info[0].curtain_motor_total_time = *(uint16_t *)message->attribute.data.value;
                        printf("val1: %d\n", device_info[0].curtain_motor_total_time);
                        
                        esp_zb_zcl_set_attribute_val(message->info.dst_endpoint,
                            message->info.cluster,
                            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                            message->attribute.id,
                            &device_info[0].curtain_motor_total_time,
                            false);  

                            send_report(0, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, 0xF003);    
                            device_info[0].curtain_motor_total_time = device_info[0].curtain_motor_total_time/10;             
                            nuos_store_data_to_nvs(0);
                    // }
                }else if (message->attribute.id == 0xF001 && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM) {
                    uint8_t val2 = *(uint8_t *)message->attribute.data.value;                   
                    esp_zb_zcl_set_attribute_val(
                        message->info.dst_endpoint,
                        message->info.cluster,
                        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                        message->attribute.id,
                        &val2,
                        false); 
                        send_report(0, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, 0xF001);                      
                    switch(val2){
                        case 0:  //cal started
                        // vTaskMode = TASK_CURTAIN_CAL_START;
                        // gpio_set_level(gpio_touch_led_pins[0], 0);
                        // start_time = esp_timer_get_time();
                        // pause_curtain_timer();
                        break;
                        case 1:  //cal finished
                        // vTaskMode = TASK_CURTAIN_CAL_END;
                        // // Calculate travel time in ms
                        // device_info[0].device_val = (esp_timer_get_time() - start_time) / 1000;
                        // device_info[0].device_val = device_info[0].device_val / 1000;
                        // nuos_store_data_to_nvs(0);
                        break;
                        default: break;
                    }
                    printf("val2: %d\n", val2);
                } 
                #endif 
                                                 
            } else if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
                if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
                    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
                        device_info[index].device_state = 0;
                        nuos_set_scene_button_attribute(index);
                        nuos_set_state_attribute(index);
                    #else
                    uint8_t index_1 = index;
                    scene_counts = 0; 
                    if(ep_cnts > TOTAL_ENDPOINTS) ep_cnts = 0;
                    if (ep_cnts < TOTAL_ENDPOINTS && !is_value_present(ep_id, ep_cnts, index)) {
                        ep_id[index_1] = index;
                        ep_cnts++;
                    }
                    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
                        if(selected_color_mode == 0) index_1 = 3;
                        else index_1 = 4;
                    #endif
                    device_info[index_1].device_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : device_info[index_1].device_state;
                    // printf("_STATE_:%d  index_1:%d\n",  device_info[index_1].device_state, index_1);

                    if(last_state[index_1] != device_info[index_1].device_state){
                        last_state[index_1] = device_info[index_1].device_state;
                        state_change_flag = true;
                    }                                                  
                    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
                        if(index_1 == 0){
                            if(device_info[index_1].device_state){ 
                                device_info[index_1].ac_mode = ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_COOL;  //cooling 
                            }else{
                                device_info[index_1].ac_mode = ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF;   //off
                            }
                        }else{

                        }
                    #else

                    #endif
                   
                    //Added by Nuos
                    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN_SWITCH)
                        
                    nuos_zb_set_hardware_curtain(index_1, false);
           
                    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2CH_CURTAIN)
                        uint8_t m_index = 255;
                        if(index_1 == 0){
                            if(!device_info[index_1].device_state) m_index = 0;
                            else m_index = 1;
                        }else if(index_1 == 1){ 
                            if(!device_info[index_1].device_state) m_index = 2;
                            else m_index = 3;
                        }
                        if(m_index != 255 )
                        nuos_zb_set_hardware(m_index, false);
                    #else
                        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
                            
                            if(selected_color_mode == 0){
                                // for(int rgb=0; rgb<3; rgb++){
                                //     device_info[rgb].device_state = false;
                                // } 
                                // if(device_info[3].device_state){
                                //     device_info[4].device_state = true;
                                // }
                            }else if(selected_color_mode == 1){
                                // if(device_info[0].device_level == 0) device_info[0].device_state = false;
                                // else device_info[0].device_state = true;
                                // if(device_info[1].device_level == 0) device_info[1].device_state = false;
                                // else device_info[1].device_state = true;
                                // if(device_info[2].device_level == 0) device_info[2].device_state = false;
                                // else device_info[2].device_state = true;
                                // if(device_info[3].device_level == 0) device_info[3].device_state = false;
                                // else device_info[3].device_state = true;

                                for(int rgb=0; rgb<3; rgb++){
                                    if(device_info[rgb].device_level <= MIN_DIM_LEVEL_VALUE) {
                                        device_info[rgb].device_level = MIN_DIM_LEVEL_VALUE;
                                        device_info[rgb].device_state = false;
                                    }else{
                                        //device_info[rgb].device_state = true;
                                    }
                                    if(device_info[rgb].device_level == 0xff){
                                        device_info[rgb].device_level = 0xfe;
                                    }
                                }                                 
                            }
                            

                            nuos_zb_set_hardware(index_1, false);
                            set_state(index_1);
                            
                        #else
                        //printf("ac_mode_or_fan_speed:%d\n", device_info[index_1].ac_mode);
                        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
                        if(device_info[index_1].ac_mode == 0)
                        #endif
                            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
                            #ifdef USE_FAN_SPEED
                            if(index_1 == 0){
                            #endif
                            #endif
                                nuos_zb_set_hardware(index_1, false);
                            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
                            #ifdef USE_FAN_SPEED
                            }
                            #endif
                            #endif
                            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
                            set_state(index_1);
                            #endif
                        #endif
                    #endif
                 #ifdef USE_TUYA_BRDIGE
                    if(is_my_device_commissionned){
                        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
                            nuos_set_zigbee_attribute(index_1);
                        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
                            nuos_set_state_attribute(0);  //to fast update switches on app
                        #else
                        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
                        #ifdef USE_FAN_SPEED
                        if(index_1 == 0){
                        #endif
                        #endif
                        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI)
                            nuos_set_zigbee_attribute(index_1);
                        #else
                            nuos_set_state_attribute(index_1);  //to fast update switches on app  
                        #endif
                        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
                        #ifdef USE_FAN_SPEED
                        }
                        #endif
                        #endif
                        #endif 
                    }
                    #endif                    
                    cb_counts++;  
                    #endif                  
                    
                }
            }else if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL) {
                scene_counts = 0; 
                if(ep_cnts > TOTAL_ENDPOINTS) ep_cnts = 0;
                ESP_LOGI(TAG, "---------------->ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL   TYPE:%d\n<----------------", message->attribute.data.type); 
                if (ep_cnts < TOTAL_ENDPOINTS && !is_value_present(ep_id, ep_cnts, index)) {
                    ep_id[index] = index;
                    ep_cnts++;
                }

                if (message->attribute.id == ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID && (message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16 || message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8)) {
                    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN_CUSTOM)
                        if(index == FAN_INDEX){
                            uint8_t d_level = *(uint8_t *)message->attribute.data.value;
                            printf("App Fan level : %d \n", d_level);
                            if(d_level > 1) { //avoid double callback value
                                device_info[index].device_level = d_level;
                                device_info[index].device_state = true;
                                if(d_level <= 63){
                                    device_info[index].fan_speed = 1;
                                }else if(d_level > 64 && d_level <= 127){
                                    device_info[index].fan_speed = 2;
                                }else if(d_level > 128 && d_level <= 191){
                                    device_info[index].fan_speed = 3;
                                }else if(d_level > 191){      //near 1000
                                    device_info[index].fan_speed = 4;   //max_speed
                                }
                                
                                nuos_set_fan_attribute(index); //to fast update switches on app

                                nuos_set_hardware_fan_ctrl(index);
                            }else{

                                    if(device_info[index].fan_speed == 1){
                                        device_info[index].device_level = 63;
                                    }else if(device_info[index].fan_speed == 2 ) //d_level > 64 && d_level <= 128){
                                    {
                                        device_info[index].device_level = 127;
                                    }else if(device_info[index].fan_speed == 3 ) //d_level > 64 && d_level <= 128){
                                    {
                                        device_info[index].device_level = 191;
                                    }else if(device_info[index].fan_speed == 4 ) //d_level > 64 && d_level <= 128){
                                    {
                                        device_info[index].device_level = 254;
                                    }
                                    printf("FORCED_LEVEL:%d\n", device_info[index].device_level);
                                    //nuos_set_fan_attribute(index); //to fast update switches on app                                 
                                // if(!device_info[index].device_state){
                                //     //nuos_set_state_attribute(FAN_INDEX); 
                                //     //nuos_set_hardware_fan_ctrl(index);    
                                // }else{
                                    nuos_set_fan_attribute(index); //to fast update switches on app   
                                    nuos_set_state_attribute(FAN_INDEX);
                                    nuos_set_hardware_fan_ctrl(index); 
                                       
                                // }
                  
                            }
                            
                            ESP_LOGI(TAG, "Fan %d Speed: %d & Level:%d\n", message->info.dst_endpoint, device_info[index].fan_speed, device_info[index].device_level);
                        }else{
                            if(*(uint8_t *)message->attribute.data.value >= MIN_DIM_LEVEL_VALUE) //10% of level value
                                device_info[index].device_level = message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : device_info[index].device_level;
                            else
                                device_info[index].device_level = MIN_DIM_LEVEL_VALUE;
                            //Added by Nuos                 
                            nuos_zb_set_hardware(index, false); 
                            ESP_LOGI(TAG, "Light %d level changes to %d\n", message->info.dst_endpoint, device_info[index].device_level);
                        }
                        // ESP_LOGI(TAG, "Light %d level changes to %d", message->info.dst_endpoint, device_info[index].device_level);
                    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
                        
                        if(index == 0){
                            uint8_t level = *(uint8_t *)message->attribute.data.value;
                            if(device_info[0].device_state && level != 0){
                                ESP_LOGI(TAG, "ac device_level %d\n", level);
                                //if(level>16) level= level/16;
                                if(level > 15) level = 11;
                                if(level <= 15 ){
                                    if(level > 0){
                                        device_info[0].device_level = level; 
                                        device_info[0].ac_temperature = ac_temp_values[device_info[0].device_level];
                                        //printf("Ac Temp %d\n",  device_info[index].ac_temperature);
                                        nuos_zb_set_hardware(0, false);
//816942
                                        if(is_my_device_commissionned){ 
                                            nuos_set_ac_cool_temperature_attribute(0);
                                        } 
                                    }
                                } 
                            }   
                        } else{
                            #ifdef USE_FAN_SPEED
                            uint8_t fan_level_speed[5] = {0, 4, 8, 12, 16};
                                                   //Auto, Min, Low, med, High, Max
                            uint8_t level1 = *(uint8_t *)message->attribute.data.value;
                            ESP_LOGI(TAG, "fan speed %d\n", level1);
                            device_info[1].device_level = level1;

                            if(device_info[1].device_level > 0){
                                if(device_info[1].device_level > fan_level_speed[0] && device_info[1].device_level <= fan_level_speed[1]){
                                    device_info[0].fan_speed = 1;
                                    //break;
                                }else if(device_info[1].device_level > fan_level_speed[1] && device_info[1].device_level <= fan_level_speed[2]){
                                    device_info[0].fan_speed = 2;
                                    //break;
                                }else if(device_info[1].device_level > fan_level_speed[2] && device_info[1].device_level <= fan_level_speed[3]){
                                    device_info[0].fan_speed = 3;
                                    //break;
                                }else if(device_info[1].device_level > fan_level_speed[3] && device_info[1].device_level <= fan_level_speed[4]){
                                    device_info[0].fan_speed = 4;
                                    //break;
                                }else if(device_info[1].device_level > fan_level_speed[4]){
                                    device_info[0].fan_speed = 5;
                                }
                            }else{
                                device_info[0].fan_speed = 0;
                            }                            

                            nuos_zb_set_hardware(1, false);
                            if(is_my_device_commissionned){ 
                                nuos_set_ac_cool_temperature_attribute(1);
                            }
                            #endif
                        }                    
                    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
                        #ifdef USE_HOME_ASSISTANT
                            //nuos_set_color_rgb_mode_attribute(0, selected_color_mode);                      
                            //store_color_mode_value(selected_color_mode);   
                            uint8_t val = *(uint8_t *)message->attribute.data.value; 
                            printf("LEVEL_VAR:%d\n", val);
                            if(selected_color_mode == 0){
                                if(val != 1) {
                                    color_level[selected_color_mode] = val;
                                    // if(selected_color_mode == 0){
                                        if(color_level[selected_color_mode] != device_info[3].device_level){  
                                            if(device_info[3].device_state){             
                                                device_info[3].device_level = color_level[selected_color_mode]; 
                                                
                                                printf("LEVEL_CCT_1:%d\n", device_info[3].device_level);                       
                                                nuos_zb_set_hardware(3, false);
                                                set_dali_level(3);
                                                // global_index = 3;
                                                // toggle_state_flag = false;
                                                // set_hardware_flag = true;                                        
                                                // set_level_flag = true;
                                            }
                                        }                             
                                }
                                #ifdef USE_TUYA_BRDIGE
                                if(is_my_device_commissionned){                                  
                                    nuos_set_level_attribute(4);                                
                                }
                                #endif  
                            } else{
                                device_info[4].device_level = val;
                            }
                        #else
                            // uint8_t val = *(uint8_t *)message->attribute.data.value; 
                            // printf("LEVEL_VAR:%d    %d\n", val, device_info[4].device_level);
                            // if(val > 2){                        
                            //     if(selected_color_mode == 0){
                            //         device_info[3].device_level = val;
                            //         nuos_zb_set_hardware(3, false);
                            //         set_level(3);
                            //         set_dali_color_temp();                                 
                            //     }else{
                            //         device_info[4].device_level = val;
                            //         nuos_zb_set_hardware(4, false);
                            //         set_level(4); 
                            //         set_dali_color_temp();                               
                            //     }
                            // }
                        #endif
                    #else
                        //#ifdef USE_HOME_ASSISTANT
                        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
                            device_level[index] = *(uint8_t *)message->attribute.data.value;
                            printf("index:%d brightness:%u level:%d\n", index, device_level[0], device_info[0].device_level);
                            if(device_level[index] >= MIN_DIM_LEVEL_VALUE && device_level[index] < MAX_DIM_LEVEL_VALUE){
                                device_info[0].device_level = device_level[index];
                            
                                if(!device_info[0].device_state){
                                    device_info[0].device_state = true;
                                    state_change_flag = true;
                                }

                                is_long_press_brightness = false;
                                nuos_set_hardware_brightness_2(0);
                               
                                if(state_change_flag){
                                    state_change_flag = false;
                                    set_state(0);
                                }
                                set_dali_level(0);
                            }
                            
                        #else
                            printf("---index:%d\n", index);
                            
                            #ifdef USE_ZB_ONLY_FAN
                                if(index == FAN_INDEX){
                                    uint8_t d_level = *(uint8_t *)message->attribute.data.value;//message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : last_level;
                                    printf("LEVEl_VAL:%d", d_level);
                                    // if(d_level > 1){ //avoid double callback value
                                        device_info[FAN_INDEX].device_level = d_level;
                                        device_info[index].device_state = true;
                                        if(d_level <= 64){
                                            device_info[index].fan_speed = 1;
                                        }else if(d_level > 64 && d_level <= 128){
                                            device_info[index].fan_speed = 2;
                                        }else if(d_level > 128 && d_level <= 191){
                                            device_info[index].fan_speed = 3;
                                        }else if(d_level > 191){      //near 1000
                                            device_info[index].fan_speed = 4;   //max_speed
                                        }else{
                                            device_info[index].device_state = false;
                                        }
                                        
                                        nuos_set_fan_attribute(index); //to fast update switches on app

                                        nuos_set_hardware_fan_ctrl(index);
                                    // }else{
                                    //     //device_info[index].fan_speed = 0;
                                    // }
                                    ESP_LOGI(TAG, "Fan %d Speed: %d & Level:%d\n", message->info.dst_endpoint, device_info[index].fan_speed, device_info[index].device_level);
                                }else{
                                    if(*(uint8_t *)message->attribute.data.value >= MIN_DIM_LEVEL_VALUE) //10% of level value
                                        device_info[index].device_level = message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : device_info[index].device_level;
                                    else
                                        device_info[index].device_level = MIN_DIM_LEVEL_VALUE; //
                                    //Added by Nuos                 
                                    nuos_zb_set_hardware(index, false); 
                                    ESP_LOGI(TAG, "Light %d level changes to %d\n", message->info.dst_endpoint, device_info[index].device_level);
                                }
                            #else
                                if(device_info[index].device_state){    
                                    device_level[index] = *(uint8_t *)message->attribute.data.value;
                                    printf("LEVEL:%d\n", device_level[index]);
                                    #ifdef ENABLE_PWM_DIMMING
                                        if(device_level[index] >= 1){
                                            if(device_level[index] != device_info[index].device_level){               
                                                device_info[index].device_level = device_level[index];
                                                //Added by Nuos
                                                global_switch_state = SWITCH_PRESS_DETECTED;
                                                nuos_set_hardware_brightness(gpio_touch_btn_pins[index]);
                                                #ifdef USE_TUYA_BRDIGE
                                                if(is_my_device_commissionned) {                                  
                                                    nuos_set_level_attribute(index);                              
                                                }  
                                                #endif
                                            } 
                                        }                                  
                                        cb_counts++;
                                    #endif
                                }else{
                                    #ifdef USE_TUYA_BRDIGE
                                    if(is_my_device_commissionned){
                                        uint8_t levell = 0;
                                        esp_zb_zcl_status_t status1 = esp_zb_zcl_set_attribute_val(
                                            ENDPOINTS_LIST[index],
                                            ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
                                            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                            ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID,
                                            (uint8_t*)&levell,
                                            false
                                        );
                                        send_report(index, ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID);
                                    }
                                #endif
                            }
                            #endif
                        #endif
                    #endif
                    
				}
            }else if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL) {

                ESP_LOGI(TAG, "---------------->ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL: 0x%x<----------------\n", message->attribute.id);
                #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)   
                    
                    if (message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16)
                    {
                        #ifdef USE_TUYA_BRDIGE
                            uint16_t temp_color = *(uint16_t *)message->attribute.data.value;
                            device_info[index].device_val = temp_color;//map_cct1(temp_color, 158, 500, MIN_CCT_VALUE, MAX_CCT_VALUE);

                                    esp_zb_zcl_status_t status = esp_zb_zcl_set_attribute_val(
                                        1,
                                        ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL,
                                        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                        ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID,
                                        (uint16_t*)&temp_color,
                                        false
                                    );
                                    send_report(0, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID); 

                            printf("temp_color:%d   device_val:%d\n", temp_color, device_info[index].device_val);
                        #else
                            device_info[index].device_val = *(uint16_t *)message->attribute.data.value;
                        #endif
                        
                        if(device_cct_color[index] != device_info[index].device_val){
                            device_cct_color[index] = device_info[index].device_val;
                            //printf("index:%d   CCT_VALUE:%d\n", index, device_info[index].device_val);
                            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
                                is_long_press_brightness = false;
                                nuos_set_hardware_brightness_2(1);
                                

                                // #ifdef USE_CCT_TIME_SYNC
                               
                                // if(device_info[0].color_or_fan_state){
                                //     device_info[0].color_or_fan_state = false;
                                //     gpio_set_level(gpio_touch_led_pins[1], 0);
                                //     stop_color_temp_timer();
                                // }         
                                
                                // #endif
                            #else
                                selected_color_mode = 0;    
                            #endif
                        }
                    }else {
                        ESP_LOGW(TAG, "COLOR_TEMP cluster data: attribute(0x%x), type(0x%x)", message->attribute.id, message->attribute.data.type);
                    }
                    // if(is_my_device_commissionned){                                  
                    //     nuos_set_color_temperature_attribute(0);                                
                    // }                       
                #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
                    printf("attr_id:0x%x\n", message->attribute.id);
                    if (message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_X_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
                        #ifdef USE_HOME_ASSISTANT
                        color_x = message->attribute.data.value ? *(uint16_t *)message->attribute.data.value : device_info[4].light_color_x;
                        // device_info[4].light_color_y = *(uint16_t *)esp_zb_zcl_get_attribute(message->info.dst_endpoint, message->info.cluster,
                        //                                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_Y_ID)
                                            // ->data_p;

                        // ESP_LOGI(TAG, "Light %d color x changes to 0x%x", message->info.dst_endpoint, device_info[4].light_color_x);
                        selected_color_mode = 1;
                        // nuos_set_color_X_attribute(4, device_info[4].light_color_x);
                        #endif
                        
                    } else if (message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_Y_ID &&
                            message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
                        #ifdef USE_HOME_ASSISTANT        
                        color_y = message->attribute.data.value ? *(uint16_t *)message->attribute.data.value : device_info[4].light_color_y;
                        // device_info[index].light_color_x = *(uint16_t *)esp_zb_zcl_get_attribute(message->info.dst_endpoint, message->info.cluster,
                        //                                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_X_ID)
                        
                        if(device_info[4].light_color_x != color_x || device_info[4].light_color_y != color_y){
                            device_info[4].light_color_y = color_y;
                            device_info[4].light_color_x  = color_x;
                            // ESP_LOGI(TAG, "Light %d color y changes to 0x%x", message->info.dst_endpoint, device_info[4].light_color_y);
                            //user defined
                            float red_f = 0, green_f = 0, blue_f = 0, color_x, color_y;
                            color_x = (float)device_info[4].light_color_x / 65535;
                            color_y = (float)device_info[4].light_color_y / 65535;
                            /* assume color_Y is full light level value 1  (0-1.0) */
                            float color_X = color_x / color_y;
                            float color_Z = (255 - color_x - color_y) / color_y;
                            float brightness = 1.0f;
                            xyToRgb(device_info[4].light_color_x, device_info[4].light_color_y, brightness, &device_info[0].device_level, 
                            &device_info[1].device_level, &device_info[2].device_level);
                            // printf("red_f:0x%x  green_f:0x%x  blue_f:0x%x\n", device_info[0].device_level, device_info[1].device_level, device_info[2].device_level);
                            selected_color_mode = 1; 
                            // //Added by Nuos 

                            if(device_info[0].device_level == 0) device_info[0].device_state = false;
                            else device_info[0].device_state = true;
                            if(device_info[1].device_level == 0) device_info[1].device_state = false;
                            else device_info[1].device_state = true;
                            if(device_info[2].device_level == 0) device_info[2].device_state = false;
                            else device_info[2].device_state = true;
                            if(device_info[3].device_level == 0) device_info[3].device_state = false;
                            else device_info[3].device_state = true;
                            device_info[4].device_state = false;
                            for(int rgb=0; rgb<3; rgb++){
                                if(device_info[rgb].device_level <= MIN_DIM_LEVEL_VALUE) {
                                    device_info[rgb].device_level = MIN_DIM_LEVEL_VALUE;
                                    device_info[rgb].device_state = false;
                                }else{
                                    device_info[rgb].device_state = true;
                                    device_info[4].device_state = true;
                                }
                                if(device_info[rgb].device_level == 0xff){
                                    device_info[rgb].device_level = 0xfe;
                                }
                            } 
            
                            
                            device_info[4].device_state = true;

                            device_info[3].device_state = false;
                          
                            nuos_zb_set_hardware(4, false); 
                            set_state(4);
                            set_dali_color_temp(0, false);
                            // global_index = 4;
                            // toggle_state_flag = false;
                            // set_hardware_flag = true;
                            //set_hardware_flag = true;
                            //set_state_flag = true;
                            //set_color_flag = true;                       
                            // nuos_set_color_Y_attribute(4, device_info[4].light_color_y);
                            // nuos_set_level_attribute(4);                            
                        }
                        
                        #endif
                       
                    }else if (message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16){

                        device_info[3].device_val = *(uint16_t *)message->attribute.data.value;
                        if(selected_color_mode == 1){
                            selected_color_mode = 0;
                            #if(USE_COLOR_DEVICE == COLOR_RGBW || USE_COLOR_DEVICE == COLOR_RGB_CW_WW)
                            mode_change_flag = true;
                            #endif

                            nuos_set_color_rgb_mode_attribute(0, selected_color_mode);
                            store_color_mode_value(selected_color_mode);

                            printf("Mode Changed!!\n");
                        }
                        
                        if(device_cct_color[3] != device_info[3].device_val){
                            device_cct_color[3] = device_info[3].device_val;
                            selected_color_mode = 0;
                        }

                        device_info[3].device_state = true;
                        //#ifdef USE_HOME_ASSISTANT
                        if(device_info[3].device_val <= 500){
                            device_info[3].device_val = map_1000(device_info[3].device_val, 158, 500, MIN_CCT_VALUE, MAX_CCT_VALUE);
                        }
                        for(int rgb=0; rgb<3; rgb++){
                            device_info[rgb].device_state = false;
                        } 

                        // global_index = 3;
                        // toggle_state_flag = false;
                        // set_hardware_flag = true;
                        // set_level_flag = true;
                        // set_color_flag = true;
                        nuos_zb_set_hardware(3, false);
                        //set_state(3);
                        set_dali_level(3);
                        set_dali_color_temp(0, true);
                        
                    }else if(message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_ENHANCED_COLOR_MODE_ID)  { 
                        selected_color_mode = *(uint8_t *)message->attribute.data.value; 
                        if(selected_color_mode == 2) selected_color_mode = 0;

                        if(last_selected_color_mode != selected_color_mode){
                            last_selected_color_mode = selected_color_mode;
                            //printf("Mode Changed!!\n");
                            #if(USE_COLOR_DEVICE == COLOR_RGBW || USE_COLOR_DEVICE == COLOR_RGB_CW_WW)
                            mode_change_flag = true;
                            nuos_set_color_rgb_mode_attribute(0, selected_color_mode);
                            // printf("nuos_set_color_rgb_mode_attribute\n");
                            store_color_mode_value(selected_color_mode);
                            printf("Mode Changed!!\n"); 
                            #endif                                                       
                        }else{
                            mode_change_flag = false;
                        }                     
                        
                    }else if(message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_HUE_ID)  {
                        // //#ifdef USE_HOME_ASSISTANT
                        uint8_t hue = *(uint8_t *)message->attribute.data.value; 
                        //if(color_counts%2 == 0){
                        //ESP_LOGW(TAG, "HUE(%d) ", hue);   
                        #ifdef USE_HOME_ASSISTANT
                        uint8_t new_hue = (3 * hue)/2;
                        #else
                        uint8_t new_hue = hue;
                        #endif
                        //
                        //if(last_hue_val != new_hue){
                            last_hue_val = new_hue;
                           // ESP_LOGW(TAG, "new_hue(%d) level:%d\n", new_hue, device_info[4].device_level);
   

                            int level_1000 = (device_info[4].device_level * 1000) / 255;

                            hsv_t hsvX = {new_hue, 1000, level_1000};
                            rgb_t rgbX = hsv_to_rgb(hsvX); 
                            device_info[0].device_level = rgbX.r;
                            device_info[1].device_level = rgbX.g;
                            device_info[2].device_level = rgbX.b;
                            ESP_LOGW(TAG, "R(%d) G(%d) B(%d) ", rgbX.r, rgbX.g, rgbX.b);
                            if(selected_color_mode == 0){
                                selected_color_mode = 1;
                                #if(USE_COLOR_DEVICE == COLOR_RGBW || USE_COLOR_DEVICE == COLOR_RGB_CW_WW)
                                mode_change_flag = true;
                                #endif

                                nuos_set_color_rgb_mode_attribute(0, selected_color_mode);
                                store_color_mode_value(selected_color_mode);

                                printf("Mode Changed!!\n");
    
                            }else{
                                selected_color_mode = 1;
                            }
                            

                            device_info[4].device_level = max_of_three(device_info[0].device_level, device_info[1].device_level, device_info[2].device_level);

                            if(device_info[0].device_level == 0) {device_info[0].device_state = false; device_info[0].device_level = MIN_DIM_LEVEL_VALUE;}
                            else device_info[0].device_state = true;
                            if(device_info[1].device_level == 0) {device_info[1].device_state = false; device_info[1].device_level = MIN_DIM_LEVEL_VALUE;}
                            else device_info[1].device_state = true;
                            if(device_info[2].device_level == 0) { device_info[2].device_state = false; device_info[2].device_level = MIN_DIM_LEVEL_VALUE;}
                            else device_info[2].device_state = true;

                            device_info[3].device_state = false;

                            
                            if(!device_info[0].device_state && !device_info[1].device_state && !device_info[2].device_state){
                                device_info[4].device_state = false;
                               // printf("All OFF RGB\n");
                            }else{
                                device_info[4].device_state = true;
                            }
                           // printf("STATE RGB:%d\n", device_info[4].device_state);
                            for(int rgb=0; rgb<3; rgb++){
                                if(device_info[rgb].device_level < MIN_DIM_LEVEL_VALUE) {
                                    device_info[rgb].device_level = MIN_DIM_LEVEL_VALUE;
                                    //device_info[rgb].device_state = false;
                                }else{
                                    //device_info[rgb].device_state = true;
                                }
                                if(device_info[rgb].device_level == 0xff){
                                    device_info[rgb].device_level = 0xfe;
                                }
                            } 

                            // device_info[0].device_state = true;
                            // global_index = 4;
                            // toggle_state_flag = false;
                            // set_hardware_flag = true;
                            // set_level_flag = true;
                            // set_color_flag = true;
                            nuos_zb_set_hardware(4, false);
                            //set_state(4);
                            set_dali_level(4);
                            set_dali_color_temp(0, false);
                            // #ifdef USE_TUYA_BRDIGE      
                            // nuos_set_color_xy_attribute(4, &hsvX);
                            // #endif                            
                        //} 
                        //#endif                                                          
                    }else if(message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_SATURATION_ID)  {
                        // uint8_t sat = *(uint8_t *)message->attribute.data.value; 
                        // ESP_LOGW(TAG, "SAT(%d)", sat);
                    } else {
                        ESP_LOGW(TAG, "Color control cluster data: attribute(0x%x), type(0x%x)", message->attribute.id, message->attribute.data.type);
                    }
                    #ifdef USE_TUYA_BRDIGE
                    // if(is_my_device_commissionned){
                    //     nuos_set_zigbee_attribute(index); //to fast update switches on app
                    // }
                    #endif
                #endif
            }else if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL) {
                //ESP_LOGI(TAG, "---------------->ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL<----------------");
                #ifndef USE_INDIVIDUAL_DALI_ADDRESSING
				if (message->attribute.id == ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM) {
					device_info[index].fan_speed = message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : device_info[index].fan_speed;
					ESP_LOGI(TAG, "Fan speed  set to %d", device_info[index].fan_speed);
                    //Added by Nuos 
                    if(device_info[index].fan_speed == 3) device_info[index].fan_speed = 4;

                    if(device_info[index].fan_speed == 0) device_info[index].device_state = 0;
                    else device_info[index].device_state = 1;
                    nuos_set_hardware_fan_ctrl(index);
				}else if (message->attribute.id == ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_SEQUENCE_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM) {
					ESP_LOGW(TAG, "ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_SEQUENCE_ID");
				} else {
					ESP_LOGW(TAG, "On/Off cluster data: attribute(0x%x), type(0x%x)", message->attribute.id, message->attribute.data.type);
				}
                if(is_my_device_commissionned){ 
                    nuos_set_zigbee_attribute(index); //to fast update switches on app 
                }
                #endif
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER)    
            }else if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT) {
                //ESP_LOGI(TAG, "---------------->ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT<----------------");
                scene_counts = 0; 
                if(ep_cnts > TOTAL_ENDPOINTS) ep_cnts = 0;
                ESP_LOGI(TAG, "---------------->ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT   TYPE:%d\n<----------------", message->attribute.data.type); 
                if (ep_cnts < TOTAL_ENDPOINTS && !is_value_present(ep_id, ep_cnts, index)) {
                    ep_id[index] = index;
                    ep_cnts++;
                }                
                #ifndef USE_INDIVIDUAL_DALI_ADDRESSING
                if (message->attribute.id == ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM) {
                    device_info[1].ac_mode = message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : 0;
                    ESP_LOGI(TAG, "ac mode %d  index:%d", device_info[1].ac_mode, index);
                    if(device_info[1].ac_mode == 0) device_info[1].device_state = false;
                    else device_info[1].device_state = true;

                    device_info[0].ac_mode  = device_info[1].ac_mode;
                    device_info[0].device_state  = device_info[1].device_state;
                    //Added by Nuos 
                    nuos_zb_set_hardware(1, false);
                    if(is_my_device_commissionned){ 
                        nuos_set_thermostat_attribute(1);
                    }

                }else if (message->attribute.id == ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_S16) {
                    uint16_t val = message->attribute.data.value ? *(int16_t *)message->attribute.data.value : 0;
                    device_info[0].ac_temperature = val / 100;
                    ESP_LOGI(TAG, "ac temp2 %d  index:%d", device_info[0].ac_temperature, index);

                    device_info[1].ac_temperature = device_info[0].ac_temperature;

                    //Added by Nuos 
                    nuos_zb_set_hardware(0, false);

                    if(is_my_device_commissionned){ 
                        nuos_set_thermostat_temp_attribute(0);
                    }
                } else {
                    ESP_LOGW(TAG, "Thermostat cluster data: attribute(0x%x), type(0x%x)",
                            message->attribute.id, message->attribute.data.type);
                }
                #endif
        #endif
            }else if(message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_BASIC){
                if(message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING){
                    if (message->attribute.id == ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID) {

                    }else if (message->attribute.id == ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID) {

                    }
                    printf("Attribute value (string): %.*s\n", *(uint8_t *)message->attribute.data.value,
                           (char *)message->attribute.data.value + 1);                    
                }


            }else if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY) {
                if (message->attribute.id == ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID) {
                    //Added by Nuos
                    uint16_t val = *(int16_t *)message->attribute.data.value;
                    // printf("Val:%d\n", val);
                    for(int k=0; k<TOTAL_ENDPOINTS; k++){                         
                        if(ENDPOINTS_LIST[k] == message->info.dst_endpoint){
                            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI)
                                mode_change_flag = false;
                                selected_color_mode = 0;
                                nuos_zb_set_hardware(3, true);
                                set_state(3);
                                nuos_set_zigbee_attribute(3);
                            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
                                mode_change_flag = false;
                                nuos_zb_set_hardware(3, true);
                                set_state(3);
                                //set_level(3);
                                nuos_set_zigbee_attribute(3);
                            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
                                device_info[k].device_level = 254;
                                is_long_press_brightness = false;
                                nuos_zb_set_hardware(k, 1);
                                set_state(k);
                                if(device_info[k].device_state)
                                set_dali_level(k);     
                                nuos_set_zigbee_attribute(k);  
                            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN)          
                                //set_curtain_percentage(device_info[0].device_level, true);  
                                if(val==0) {
                                    nuos_zb_set_leds_only(0, false);  
                                }else{
                                    nuos_zb_set_leds_only(0, true);  
                                }
                                                 
                            #else
                                #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN)
                                if(!device_info[0].device_state) device_info[0].device_state= true;
                                else device_info[0].device_state = false;
                                #endif
                                nuos_zb_set_hardware(k, true);    // toggle leds
                                nuos_set_zigbee_attribute(k);     // zigbee attributes
                            #endif
                           
                        }
                    }
                }
            }
        }
        //switch_driver_gpios_intr_enabled(true);
}


void nuos_set_attribute_cluster_3(const esp_zb_zcl_report_attr_message_t *message){
    uint8_t index = 0xff;
    set_state_flag = false;
    set_level_flag = false;
    set_color_flag = false;
    //recheckTimer();
    
    for(uint8_t i=0; i<TOTAL_ENDPOINTS; i++)  {
        if (message->dst_endpoint == ENDPOINTS_LIST[i]) {
            index = i;
            break;
        }
    }
    
    // #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
    //     device_info[0].color_or_fan_state = false;
    // #endif
        if (index != 0xff) {
            //send_report(index, message->cluster, message->attribute.id);
            if (message->cluster == 0xe000) {
                // ESP_LOGI(TAG, "---------------->ESP_ZB_ZCL_CLUSTER_ID_ON_OFF\n<----------------");
                if (message->attribute.id == 0xD004 && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
                    uint16_t val1 = *(uint16_t *)message->attribute.data.value;
                    printf("val1: %d\n", val1);
                }else if (message->attribute.id == 0xD005 && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
                    uint16_t val2 = *(uint16_t *)message->attribute.data.value;
                    printf("val2: %d\n", val2);
                }    
                // zb_scene_info

               // writeSceneInfoStruct(&scene_info, message);
            } else if (message->cluster == ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING) { //window covering
                #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN)
                static uint8_t last_value = 0xFF;
                if (message->attribute.id == 0xF003 && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
                    // if(*(uint16_t *)message->attribute.data.value != last_value){
                    //     last_value = *(uint16_t *)message->attribute.data.value;
                        device_info[0].curtain_motor_total_time = *(uint16_t *)message->attribute.data.value;
                        printf("val1: %d\n", device_info[0].curtain_motor_total_time);
                        
                        esp_zb_zcl_set_attribute_val(message->dst_endpoint,
                            message->cluster,
                            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                            message->attribute.id,
                            &device_info[0].curtain_motor_total_time,
                            false);  

                            send_report(0, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, 0xF003);    
                            device_info[0].curtain_motor_total_time = device_info[0].curtain_motor_total_time/10;             
                            nuos_store_data_to_nvs(0);
                    // }
                }else if (message->attribute.id == 0xF001 && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM) {
                    uint8_t val2 = *(uint8_t *)message->attribute.data.value;                   
                    esp_zb_zcl_set_attribute_val(
                        message->dst_endpoint,
                        message->cluster,
                        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                        message->attribute.id,
                        &val2,
                        false); 
                        send_report(0, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, 0xF001);                      
                    switch(val2){
                        case 0:  //cal started
                        // vTaskMode = TASK_CURTAIN_CAL_START;
                        // gpio_set_level(gpio_touch_led_pins[0], 0);
                        // start_time = esp_timer_get_time();
                        // pause_curtain_timer();
                        break;
                        case 1:  //cal finished
                        // vTaskMode = TASK_CURTAIN_CAL_END;
                        // // Calculate travel time in ms
                        // device_info[0].device_val = (esp_timer_get_time() - start_time) / 1000;
                        // device_info[0].device_val = device_info[0].device_val / 1000;
                        // nuos_store_data_to_nvs(0);
                        break;
                        default: break;
                    }
                    printf("val2: %d\n", val2);
                } 
                #endif 
                                                 
            } else if (message->cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
                if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
                    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
                        device_info[index].device_state = 0;
                        nuos_set_scene_button_attribute(index);
                        nuos_set_state_attribute(index);
                    #else
                    uint8_t index_1 = index;
                    scene_counts = 0; 
                    if(ep_cnts > TOTAL_ENDPOINTS) ep_cnts = 0;
                    if (ep_cnts < TOTAL_ENDPOINTS && !is_value_present(ep_id, ep_cnts, index)) {
                        ep_id[index_1] = index;
                        ep_cnts++;
                    }
                    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
                        if(selected_color_mode == 0) index_1 = 3;
                        else index_1 = 4;
                    #endif
                    device_info[index_1].device_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : device_info[index_1].device_state;
                    printf("_STATE_:%d  index_1:%d\n",  device_info[index_1].device_state, index_1);

                    if(last_state[index_1] != device_info[index_1].device_state){
                        last_state[index_1] = device_info[index_1].device_state;
                        state_change_flag = true;
                    }                                                  
                    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
                        if(index_1 == 0){
                            if(device_info[index_1].device_state){ 
                                device_info[index_1].ac_mode = ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_COOL;  //cooling 
                            }else{
                                device_info[index_1].ac_mode = ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF;   //off
                            }
                        }else{

                        }
                    #else

                    #endif
                   
                    //Added by Nuos
                    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN_SWITCH)
                        
                    nuos_zb_set_hardware_curtain(index_1, false);
           
                    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2CH_CURTAIN)
                        uint8_t m_index = 255;
                        if(index_1 == 0){
                            if(!device_info[index_1].device_state) m_index = 0;
                            else m_index = 1;
                        }else if(index_1 == 1){ 
                            if(!device_info[index_1].device_state) m_index = 2;
                            else m_index = 3;
                        }
                        if(m_index != 255 )
                        nuos_zb_set_hardware(m_index, false);
                    #else
                        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
                            
                            if(selected_color_mode == 0){
                                // for(int rgb=0; rgb<3; rgb++){
                                //     device_info[rgb].device_state = false;
                                // } 
                            }else if(selected_color_mode == 1){
                                // if(device_info[0].device_level == 0) device_info[0].device_state = false;
                                // else device_info[0].device_state = true;
                                // if(device_info[1].device_level == 0) device_info[1].device_state = false;
                                // else device_info[1].device_state = true;
                                // if(device_info[2].device_level == 0) device_info[2].device_state = false;
                                // else device_info[2].device_state = true;
                                // if(device_info[3].device_level == 0) device_info[3].device_state = false;
                                // else device_info[3].device_state = true;

                                for(int rgb=0; rgb<3; rgb++){
                                    if(device_info[rgb].device_level <= MIN_DIM_LEVEL_VALUE) {
                                        device_info[rgb].device_level = MIN_DIM_LEVEL_VALUE;
                                        device_info[rgb].device_state = false;
                                    }else{
                                        //device_info[rgb].device_state = true;
                                    }
                                    if(device_info[rgb].device_level == 0xff){
                                        device_info[rgb].device_level = 0xfe;
                                    }
                                }                                 
                            }
                            

                            nuos_zb_set_hardware(index_1, false);
                            set_state(index_1);
                            
                        #else
                        //printf("ac_mode_or_fan_speed:%d\n", device_info[index_1].ac_mode);
                        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
                        if(device_info[index_1].ac_mode == 0)
                        #endif
                            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
                            #ifdef USE_FAN_SPEED
                            if(index_1 == 0){
                            #endif
                            #endif
                                nuos_zb_set_hardware(index_1, false);
                            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
                            #ifdef USE_FAN_SPEED
                            }
                            #endif
                            #endif
                            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
                            set_state(index_1);
                            #endif
                        #endif
                    #endif
                 #ifdef USE_TUYA_BRDIGE
                    if(is_my_device_commissionned){
                        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
                            nuos_set_zigbee_attribute(index_1);
                        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
                            nuos_set_state_attribute(0);  //to fast update switches on app
                        #else
                        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
                        #ifdef USE_FAN_SPEED
                        if(index_1 == 0){
                        #endif
                        #endif
                        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI)
                            nuos_set_zigbee_attribute(index_1);
                        #else
                            nuos_set_state_attribute(index_1);  //to fast update switches on app  
                        #endif
                        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
                        #ifdef USE_FAN_SPEED
                        }
                        #endif
                        #endif
                        #endif 
                    }
                    #endif                    
                    cb_counts++;  
                    #endif                  
                    
                }
            }else if (message->cluster == ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL) {
                scene_counts = 0; 
                if(ep_cnts > TOTAL_ENDPOINTS) ep_cnts = 0;
                ESP_LOGI(TAG, "---------------->ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL   TYPE:%d\n<----------------", message->attribute.data.type); 
                if (ep_cnts < TOTAL_ENDPOINTS && !is_value_present(ep_id, ep_cnts, index)) {
                    ep_id[index] = index;
                    ep_cnts++;
                }

                if (message->attribute.id == ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID && (message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16 || message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8)) {
                    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN_CUSTOM)
                        if(index == FAN_INDEX){
                            uint8_t d_level = *(uint8_t *)message->attribute.data.value;
                            printf("App Fan level : %d \n", d_level);
                            if(d_level > 1) { //avoid double callback value
                                device_info[index].device_level = d_level;
                                device_info[index].device_state = true;
                                if(d_level <= 63){
                                    device_info[index].fan_speed = 1;
                                }else if(d_level > 64 && d_level <= 127){
                                    device_info[index].fan_speed = 2;
                                }else if(d_level > 128 && d_level <= 191){
                                    device_info[index].fan_speed = 3;
                                }else if(d_level > 191){      //near 1000
                                    device_info[index].fan_speed = 4;   //max_speed
                                }
                                
                                nuos_set_fan_attribute(index); //to fast update switches on app

                                nuos_set_hardware_fan_ctrl(index);
                            }else{

                                    if(device_info[index].fan_speed == 1){
                                        device_info[index].device_level = 63;
                                    }else if(device_info[index].fan_speed == 2 ) //d_level > 64 && d_level <= 128){
                                    {
                                        device_info[index].device_level = 127;
                                    }else if(device_info[index].fan_speed == 3 ) //d_level > 64 && d_level <= 128){
                                    {
                                        device_info[index].device_level = 191;
                                    }else if(device_info[index].fan_speed == 4 ) //d_level > 64 && d_level <= 128){
                                    {
                                        device_info[index].device_level = 254;
                                    }
                                    printf("FORCED_LEVEL:%d\n", device_info[index].device_level);
                                    //nuos_set_fan_attribute(index); //to fast update switches on app                                 
                                // if(!device_info[index].device_state){
                                //     //nuos_set_state_attribute(FAN_INDEX); 
                                //     //nuos_set_hardware_fan_ctrl(index);    
                                // }else{
                                    nuos_set_fan_attribute(index); //to fast update switches on app   
                                    nuos_set_state_attribute(FAN_INDEX);
                                    nuos_set_hardware_fan_ctrl(index); 
                                       
                                // }
                  
                            }
                            
                            ESP_LOGI(TAG, "Fan %d Speed: %d & Level:%d\n", message->dst_endpoint, device_info[index].fan_speed, device_info[index].device_level);
                        }else{
                            if(*(uint8_t *)message->attribute.data.value >= MIN_DIM_LEVEL_VALUE) //10% of level value
                                device_info[index].device_level = message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : device_info[index].device_level;
                            else
                                device_info[index].device_level = MIN_DIM_LEVEL_VALUE;
                            //Added by Nuos                 
                            nuos_zb_set_hardware(index, false); 
                            ESP_LOGI(TAG, "Light %d level changes to %d\n", message->dst_endpoint, device_info[index].device_level);
                        }
                        // ESP_LOGI(TAG, "Light %d level changes to %d", message->dst_endpoint, device_info[index].device_level);
                    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
                        
                        if(index == 0){
                            uint8_t level = *(uint8_t *)message->attribute.data.value;
                            if(device_info[0].device_state && level != 0){
                                ESP_LOGI(TAG, "ac device_level %d\n", level);
                                if(level <= 15 ){
                                    if(level > 0){
                                        device_info[0].device_level = level; 
                                        device_info[0].ac_temperature = ac_temp_values[device_info[0].device_level];
                                        nuos_zb_set_hardware(0, false);
                                        if(is_my_device_commissionned){ 
                                            nuos_set_ac_cool_temperature_attribute(0);
                                        } 
                                    }
                                } 
                            }   
                        } else{
                            #ifdef USE_FAN_SPEED
                            uint8_t fan_level_speed[5] = {0, 4, 8, 12, 16};
                                                   //Auto, Min, Low, med, High, Max
                            uint8_t level1 = *(uint8_t *)message->attribute.data.value;
                            ESP_LOGI(TAG, "fan speed %d\n", level1);
                            device_info[1].device_level = level1;

                            if(device_info[1].device_level > 0){
                                if(device_info[1].device_level > fan_level_speed[0] && device_info[1].device_level <= fan_level_speed[1]){
                                    device_info[0].fan_speed = 1;
                                    //break;
                                }else if(device_info[1].device_level > fan_level_speed[1] && device_info[1].device_level <= fan_level_speed[2]){
                                    device_info[0].fan_speed = 2;
                                    //break;
                                }else if(device_info[1].device_level > fan_level_speed[2] && device_info[1].device_level <= fan_level_speed[3]){
                                    device_info[0].fan_speed = 3;
                                    //break;
                                }else if(device_info[1].device_level > fan_level_speed[3] && device_info[1].device_level <= fan_level_speed[4]){
                                    device_info[0].fan_speed = 4;
                                    //break;
                                }else if(device_info[1].device_level > fan_level_speed[4]){
                                    device_info[0].fan_speed = 5;
                                }
                            }else{
                                device_info[0].fan_speed = 0;
                            }                            

                            nuos_zb_set_hardware(1, false);
                            if(is_my_device_commissionned){ 
                                nuos_set_ac_cool_temperature_attribute(1);
                            }
                            #endif
                        }                    
                    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
                        #ifdef USE_HOME_ASSISTANT
                            //nuos_set_color_rgb_mode_attribute(0, selected_color_mode);                      
                            //store_color_mode_value(selected_color_mode);   
                            uint8_t val = *(uint8_t *)message->attribute.data.value; 
                            // printf("LEVEL_VAR:%d\n", val);
                            if(selected_color_mode == 0){
                                if(val != 1) {
                                    color_level[selected_color_mode] = val;
                                    // if(selected_color_mode == 0){
                                        if(color_level[selected_color_mode] != device_info[3].device_level){  
                                            if(device_info[3].device_state){             
                                                device_info[3].device_level = color_level[selected_color_mode]; 
                                                
                                                printf("LEVEL_CCT_1:%d\n", device_info[3].device_level);                       
                                                nuos_zb_set_hardware(3, false);
                                                set_dali_level(3);
                                                // global_index = 3;
                                                // toggle_state_flag = false;
                                                // set_hardware_flag = true;                                        
                                                // set_level_flag = true;
                                            }
                                        }                             
                                }
                                #ifdef USE_TUYA_BRDIGE
                                if(is_my_device_commissionned){                                  
                                    nuos_set_level_attribute(4);                                
                                }
                                #endif  
                            } else{
                                device_info[4].device_level = val;
                            }
                        #else
                            uint8_t val = *(uint8_t *)message->attribute.data.value; 
                            // printf("LEVEL_VAR:%d    %d\n", val, device_info[4].device_level);
                            // if(val > 2){                        
                            //     if(selected_color_mode == 0){
                            //         device_info[3].device_level = val;
                            //         nuos_zb_set_hardware(3, false);
                            //         set_dali_level(3);
                            //         set_dali_color_temp(3, false);                                 
                            //     }else{
                            //         device_info[4].device_level = val;
                            //         nuos_zb_set_hardware(4, false);
                            //         set_dali_level(4); 
                            //         set_dali_color_temp(4, false);                               
                            //     }
                            // }
                        #endif
                    #else
                        //#ifdef USE_HOME_ASSISTANT
                        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
                            device_level[index] = *(uint8_t *)message->attribute.data.value;
                            // printf("index:%d brightness:%u level:%d\n", index, device_level[0], device_info[0].device_level);
                            if(device_level[index] >= MIN_DIM_LEVEL_VALUE && device_level[index] < MAX_DIM_LEVEL_VALUE){
                                device_info[0].device_level = device_level[index];
                            
                                if(!device_info[0].device_state){
                                    device_info[0].device_state = true;
                                    state_change_flag = true;
                                }

                                is_long_press_brightness = false;
                                nuos_set_hardware_brightness_2(0);
                               
                                if(state_change_flag){
                                    state_change_flag = false;
                                    set_state(0);
                                }
                                set_dali_level(0);
                            }
                            
                        #else
                            // printf("---index:%d\n", index);
                            #ifdef USE_ZB_ONLY_FAN
                                if(index == FAN_INDEX){
                                    uint8_t d_level = *(uint8_t *)message->attribute.data.value;//message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : last_level;
                                    printf("LEVEl_VAL:%d", d_level);
                                    // if(d_level > 1){ //avoid double callback value
                                        device_info[FAN_INDEX].device_level = d_level;
                                        device_info[index].device_state = true;
                                        if(d_level <= 64){
                                            device_info[index].fan_speed = 1;
                                        }else if(d_level > 64 && d_level <= 128){
                                            device_info[index].fan_speed = 2;
                                        }else if(d_level > 128 && d_level <= 191){
                                            device_info[index].fan_speed = 3;
                                        }else if(d_level > 191){      //near 1000
                                            device_info[index].fan_speed = 4;   //max_speed
                                        }else{
                                            device_info[index].device_state = false;
                                        }
                                        
                                        nuos_set_fan_attribute(index); //to fast update switches on app

                                        nuos_set_hardware_fan_ctrl(index);
                                    // }else{
                                    //     //device_info[index].fan_speed = 0;
                                    // }
                                    ESP_LOGI(TAG, "Fan %d Speed: %d & Level:%d\n", message->dst_endpoint, device_info[index].fan_speed, device_info[index].device_level);
                                }else{
                                    if(*(uint8_t *)message->attribute.data.value >= MIN_DIM_LEVEL_VALUE) //10% of level value
                                        device_info[index].device_level = message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : device_info[index].device_level;
                                    else
                                        device_info[index].device_level = MIN_DIM_LEVEL_VALUE; //
                                    //Added by Nuos                 
                                    nuos_zb_set_hardware(index, false); 
                                    ESP_LOGI(TAG, "Light %d level changes to %d\n", message->dst_endpoint, device_info[index].device_level);
                                }
                            #else
                                if(device_info[index].device_state){    
                                    device_level[index] = *(uint8_t *)message->attribute.data.value;
                                    // if(device_level[index] < MIN_DIM_LEVEL_VALUE){
                                    //     device_level[index] = MIN_DIM_LEVEL_VALUE;
                                    // }
                                    printf("LEVEL:%d\n", device_level[index]);
                                    #ifdef ENABLE_PWM_DIMMING
                                        //ESP_LOGI(TAG, "======> Callback %d EP%d level = %d <======" , cb_counts, message->info.dst_endpoint, device_info[index].device_level);
                                        if(device_level[index] >= MIN_DIM_LEVEL_VALUE){
                                            if(device_level[index] != device_info[index].device_level){               
                                                device_info[index].device_level = device_level[index];
                                                //Added by Nuos
                                                global_switch_state = SWITCH_PRESS_DETECTED;
                                                nuos_set_hardware_brightness(gpio_touch_btn_pins[index]);
                                                #ifdef USE_TUYA_BRDIGE
                                                if(is_my_device_commissionned) {                                  
                                                    nuos_set_level_attribute(index);                              
                                                }  
                                                #endif
                                            } 
                                        }                                  
                                        cb_counts++;
                                    #endif
                                }else{
                                    #ifdef USE_TUYA_BRDIGE
                                    if(is_my_device_commissionned){
                                        uint8_t levell = 0;
                                        esp_zb_zcl_status_t status1 = esp_zb_zcl_set_attribute_val(
                                            ENDPOINTS_LIST[index],
                                            ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
                                            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                            ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID,
                                            (uint8_t*)&levell,
                                            false
                                        );
                                        send_report(index, ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID);
                                    }
                                #endif
                            }
                            #endif
                        #endif
                    #endif
                    
				}
            }else if (message->cluster == ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL) {

                ESP_LOGI(TAG, "---------------->ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL: 0x%x<----------------\n", message->attribute.id);
                #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)   
                    
                    if (message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16)
                    {
                        #ifdef USE_TUYA_BRDIGE
                            uint16_t temp_color = *(uint16_t *)message->attribute.data.value;
                            device_info[index].device_val = temp_color;//map_cct1(temp_color, 158, 500, MIN_CCT_VALUE, MAX_CCT_VALUE);

                                    esp_zb_zcl_status_t status = esp_zb_zcl_set_attribute_val(
                                        1,
                                        ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL,
                                        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                        ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID,
                                        (uint16_t*)&temp_color,
                                        false
                                    );
                                    send_report(0, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID); 

                            printf("temp_color:%d   device_val:%d\n", temp_color, device_info[index].device_val);
                        #else
                            device_info[index].device_val = *(uint16_t *)message->attribute.data.value;
                        #endif
                        
                        if(device_cct_color[index] != device_info[index].device_val){
                            device_cct_color[index] = device_info[index].device_val;
                            //printf("index:%d   CCT_VALUE:%d\n", index, device_info[index].device_val);
                            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
                                is_long_press_brightness = false;
                                nuos_set_hardware_brightness_2(1);
                                

                                // #ifdef USE_CCT_TIME_SYNC
                               
                                // if(device_info[0].color_or_fan_state){
                                //     device_info[0].color_or_fan_state = false;
                                //     gpio_set_level(gpio_touch_led_pins[1], 0);
                                //     stop_color_temp_timer();
                                // }         
                                
                                // #endif
                            #else
                                selected_color_mode = 0;    
                            #endif
                        }
                    }else {
                        ESP_LOGW(TAG, "COLOR_TEMP cluster data: attribute(0x%x), type(0x%x)", message->attribute.id, message->attribute.data.type);
                    }
                    // if(is_my_device_commissionned){                                  
                    //     nuos_set_color_temperature_attribute(0);                                
                    // }                       
                #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
                    printf("attr_id:0x%x\n", message->attribute.id);
                    if (message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_X_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
                        #ifdef USE_HOME_ASSISTANT
                        color_x = message->attribute.data.value ? *(uint16_t *)message->attribute.data.value : device_info[4].light_color_x;
                        // device_info[4].light_color_y = *(uint16_t *)esp_zb_zcl_get_attribute(message->info.dst_endpoint, message->info.cluster,
                        //                                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_Y_ID)
                                            // ->data_p;

                        // ESP_LOGI(TAG, "Light %d color x changes to 0x%x", message->info.dst_endpoint, device_info[4].light_color_x);
                        selected_color_mode = 1;
                        // nuos_set_color_X_attribute(4, device_info[4].light_color_x);
                        #endif
                        
                    } else if (message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_Y_ID &&
                            message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
                        #ifdef USE_HOME_ASSISTANT        
                        color_y = message->attribute.data.value ? *(uint16_t *)message->attribute.data.value : device_info[4].light_color_y;
                        // device_info[index].light_color_x = *(uint16_t *)esp_zb_zcl_get_attribute(message->info.dst_endpoint, message->info.cluster,
                        //                                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_X_ID)
                        
                        if(device_info[4].light_color_x != color_x || device_info[4].light_color_y != color_y){
                            device_info[4].light_color_y = color_y;
                            device_info[4].light_color_x  = color_x;
                            // ESP_LOGI(TAG, "Light %d color y changes to 0x%x", message->info.dst_endpoint, device_info[4].light_color_y);
                            //user defined
                            float red_f = 0, green_f = 0, blue_f = 0, color_x, color_y;
                            color_x = (float)device_info[4].light_color_x / 65535;
                            color_y = (float)device_info[4].light_color_y / 65535;
                            /* assume color_Y is full light level value 1  (0-1.0) */
                            float color_X = color_x / color_y;
                            float color_Z = (255 - color_x - color_y) / color_y;
                            float brightness = 1.0f;
                            xyToRgb(device_info[4].light_color_x, device_info[4].light_color_y, brightness, &device_info[0].device_level, 
                            &device_info[1].device_level, &device_info[2].device_level);
                            // printf("red_f:0x%x  green_f:0x%x  blue_f:0x%x\n", device_info[0].device_level, device_info[1].device_level, device_info[2].device_level);
                            selected_color_mode = 1; 
                            // //Added by Nuos 

                            if(device_info[0].device_level == 0) device_info[0].device_state = false;
                            else device_info[0].device_state = true;
                            if(device_info[1].device_level == 0) device_info[1].device_state = false;
                            else device_info[1].device_state = true;
                            if(device_info[2].device_level == 0) device_info[2].device_state = false;
                            else device_info[2].device_state = true;
                            if(device_info[3].device_level == 0) device_info[3].device_state = false;
                            else device_info[3].device_state = true;
                            device_info[4].device_state = false;
                            for(int rgb=0; rgb<3; rgb++){
                                if(device_info[rgb].device_level <= MIN_DIM_LEVEL_VALUE) {
                                    device_info[rgb].device_level = MIN_DIM_LEVEL_VALUE;
                                    device_info[rgb].device_state = false;
                                }else{
                                    device_info[rgb].device_state = true;
                                    device_info[4].device_state = true;
                                }
                                if(device_info[rgb].device_level == 0xff){
                                    device_info[rgb].device_level = 0xfe;
                                }
                            } 
            
                            
                            device_info[4].device_state = true;

                            device_info[3].device_state = false;
                          
                            nuos_zb_set_hardware(4, false); 
                            set_state(4);
                            set_dali_color_temp(0, false);
                            // global_index = 4;
                            // toggle_state_flag = false;
                            // set_hardware_flag = true;
                            //set_hardware_flag = true;
                            //set_state_flag = true;
                            //set_color_flag = true;                       
                            // nuos_set_color_Y_attribute(4, device_info[4].light_color_y);
                            // nuos_set_level_attribute(4);                            
                        }
                        
                        #endif
                       
                    }else if (message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16){

                        device_info[3].device_val = *(uint16_t *)message->attribute.data.value;
                        if(selected_color_mode == 1){
                            selected_color_mode = 0;
                            #if(USE_COLOR_DEVICE == COLOR_RGBW || USE_COLOR_DEVICE == COLOR_RGB_CW_WW)
                            mode_change_flag = true;
                            #endif

                            nuos_set_color_rgb_mode_attribute(0, selected_color_mode);
                            store_color_mode_value(selected_color_mode);

                            printf("Mode Changed!!\n");
                        }
                        
                        if(device_cct_color[3] != device_info[3].device_val){
                            device_cct_color[3] = device_info[3].device_val;
                            selected_color_mode = 0;
                        }

                        device_info[3].device_state = true;
                        //#ifdef USE_HOME_ASSISTANT
                        if(device_info[3].device_val <= 500){
                            device_info[3].device_val = map_1000(device_info[3].device_val, 158, 500, MIN_CCT_VALUE, MAX_CCT_VALUE);
                        }
                        for(int rgb=0; rgb<3; rgb++){
                            device_info[rgb].device_state = false;
                        } 

                        // global_index = 3;
                        // toggle_state_flag = false;
                        // set_hardware_flag = true;
                        // set_level_flag = true;
                        // set_color_flag = true;
                        nuos_zb_set_hardware(3, false);
                        //set_state(3);
                        set_dali_level(3);
                        set_dali_color_temp(0, true);
                        
                    }else if(message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_ENHANCED_COLOR_MODE_ID)  { 
                        selected_color_mode = *(uint8_t *)message->attribute.data.value; 
                        if(selected_color_mode == 2) selected_color_mode = 0;
                        //printf("en_color_mode:%d\n", selected_color_mode);
                        if(last_selected_color_mode != selected_color_mode){
                            last_selected_color_mode = selected_color_mode;
                            //printf("Mode Changed!!\n");
                            #if(USE_COLOR_DEVICE == COLOR_RGBW || USE_COLOR_DEVICE == COLOR_RGB_CW_WW)
                            mode_change_flag = true;
                            nuos_set_color_rgb_mode_attribute(0, selected_color_mode);
                            // printf("nuos_set_color_rgb_mode_attribute\n");
                            store_color_mode_value(selected_color_mode);
                            printf("Mode Changed!!\n"); 
                            #endif                                                       
                        }else{
                            mode_change_flag = false;
                        }                     
                        
                    }else if(message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_HUE_ID)  {
                        // //#ifdef USE_HOME_ASSISTANT
                        uint8_t hue = *(uint8_t *)message->attribute.data.value; 
                        //if(color_counts%2 == 0){
                        //ESP_LOGW(TAG, "HUE(%d) ", hue);   
                        #ifdef USE_HOME_ASSISTANT
                        uint8_t new_hue = (3 * hue)/2;
                        #else
                        uint8_t new_hue = hue;
                        #endif
                        //
                        //if(last_hue_val != new_hue){
                            last_hue_val = new_hue;
                           // ESP_LOGW(TAG, "new_hue(%d) level:%d\n", new_hue, device_info[4].device_level);
   

                            int level_1000 = (device_info[4].device_level * 1000) / 255;
                            
                            hsv_t hsvX = {new_hue, 1000, level_1000};
                            device_info[4].device_level = hsvX.v;
                            rgb_t rgbX = hsv_to_rgb(hsvX); 
                            device_info[0].device_level = rgbX.r;
                            device_info[1].device_level = rgbX.g;
                            device_info[2].device_level = rgbX.b;
                            ESP_LOGW(TAG, "R(%d) G(%d) B(%d) ", rgbX.r, rgbX.g, rgbX.b);
                            if(selected_color_mode == 0){
                                selected_color_mode = 1;
                                #if(USE_COLOR_DEVICE == COLOR_RGBW || USE_COLOR_DEVICE == COLOR_RGB_CW_WW)
                                mode_change_flag = true;
                                #endif

                                nuos_set_color_rgb_mode_attribute(0, selected_color_mode);
                                store_color_mode_value(selected_color_mode);

                                printf("Mode Changed!!\n");
    
                            }else{
                                selected_color_mode = 1;
                            }
                            

                            device_info[4].device_level = max_of_three(device_info[0].device_level, device_info[1].device_level, device_info[2].device_level);

                            if(device_info[0].device_level == 0) {device_info[0].device_state = false; device_info[0].device_level = MIN_DIM_LEVEL_VALUE;}
                            else device_info[0].device_state = true;
                            if(device_info[1].device_level == 0) {device_info[1].device_state = false; device_info[1].device_level = MIN_DIM_LEVEL_VALUE;}
                            else device_info[1].device_state = true;
                            if(device_info[2].device_level == 0) { device_info[2].device_state = false; device_info[2].device_level = MIN_DIM_LEVEL_VALUE;}
                            else device_info[2].device_state = true;

                            device_info[3].device_state = false;

                            
                            if(!device_info[0].device_state && !device_info[1].device_state && !device_info[2].device_state){
                                device_info[4].device_state = false;
                               // printf("All OFF RGB\n");
                            }else{
                                device_info[4].device_state = true;
                            }
                           // printf("STATE RGB:%d\n", device_info[4].device_state);
                            for(int rgb=0; rgb<3; rgb++){
                                if(device_info[rgb].device_level < MIN_DIM_LEVEL_VALUE) {
                                    device_info[rgb].device_level = MIN_DIM_LEVEL_VALUE;
                                    //device_info[rgb].device_state = false;
                                }else{
                                    //device_info[rgb].device_state = true;
                                }
                                if(device_info[rgb].device_level == 0xff){
                                    device_info[rgb].device_level = 0xfe;
                                }
                            } 

                            // device_info[0].device_state = true;
                            // global_index = 4;
                            // toggle_state_flag = false;
                            // set_hardware_flag = true;
                            // set_level_flag = true;
                            // set_color_flag = true;
                            nuos_zb_set_hardware(4, false);
                            //set_state(4);
                            set_dali_level(4);
                            set_dali_color_temp(0, true);
                            // #ifdef USE_TUYA_BRDIGE      
                            // nuos_set_color_xy_attribute(4, &hsvX);
                            // #endif                            
                        //} 
                        //#endif                                                          
                    }else if(message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_SATURATION_ID)  {
                        // uint8_t sat = *(uint8_t *)message->attribute.data.value; 
                        // ESP_LOGW(TAG, "SAT(%d)", sat);
                    } else {
                        ESP_LOGW(TAG, "Color control cluster data: attribute(0x%x), type(0x%x)", message->attribute.id, message->attribute.data.type);
                    }
                    #ifdef USE_TUYA_BRDIGE
                    // if(is_my_device_commissionned){
                    //     nuos_set_zigbee_attribute(index); //to fast update switches on app
                    // }
                    #endif
                #endif
            }else if (message->cluster == ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL) {
                scene_counts = 0; 
                if(ep_cnts > TOTAL_ENDPOINTS) ep_cnts = 0;
                ESP_LOGI(TAG, "---------------->ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL   TYPE:%d\n<----------------", message->attribute.data.type); 
                if (ep_cnts < TOTAL_ENDPOINTS && !is_value_present(ep_id, ep_cnts, index)) {
                    ep_id[index] = index;
                    ep_cnts++;
                }
                #ifndef USE_INDIVIDUAL_DALI_ADDRESSING
				if (message->attribute.id == ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM) {
					device_info[index].fan_speed = message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : device_info[index].fan_speed;
					ESP_LOGI(TAG, "Fan speed  set to %d", device_info[index].fan_speed);
                    //Added by Nuos 
                    if(device_info[index].fan_speed == 3) device_info[index].fan_speed = 4;

                    if(device_info[index].fan_speed == 0) device_info[index].device_state = 0;
                    else device_info[index].device_state = 1;
                    nuos_set_hardware_fan_ctrl(index);
				}else if (message->attribute.id == ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_SEQUENCE_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM) {
					ESP_LOGW(TAG, "ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_SEQUENCE_ID");
				} else {
					ESP_LOGW(TAG, "On/Off cluster data: attribute(0x%x), type(0x%x)", message->attribute.id, message->attribute.data.type);
				}
                if(is_my_device_commissionned){ 
                    nuos_set_zigbee_attribute(index); //to fast update switches on app 
                }
                #endif
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER)    
            }else if (message->cluster == ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT) {
                scene_counts = 0; 
                if(ep_cnts > TOTAL_ENDPOINTS) ep_cnts = 0;
                ESP_LOGI(TAG, "---------------->ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT   TYPE:%d\n<----------------", message->attribute.data.type); 
                if (ep_cnts < TOTAL_ENDPOINTS && !is_value_present(ep_id, ep_cnts, index)) {
                    ep_id[index] = index;
                    ep_cnts++;
                }
                #ifndef USE_INDIVIDUAL_DALI_ADDRESSING
                if (message->attribute.id == ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM) {
                    device_info[1].ac_mode = message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : 0;
                    ESP_LOGI(TAG, "ac mode %d  index:%d", device_info[1].ac_mode, index);
                    if(device_info[1].ac_mode == 0) device_info[1].device_state = false;
                    else device_info[1].device_state = true;

                    device_info[0].ac_mode  = device_info[1].ac_mode;
                    device_info[0].device_state  = device_info[1].device_state;
                    //Added by Nuos 
                    nuos_zb_set_hardware(1, false);
                    if(is_my_device_commissionned){ 
                        nuos_set_thermostat_attribute(1);
                    }

                }else if (message->attribute.id == ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_S16) {
                    uint16_t val = message->attribute.data.value ? *(int16_t *)message->attribute.data.value : 0;
                    device_info[0].ac_temperature = val / 100;
                    ESP_LOGI(TAG, "ac temp2 %d  index:%d", device_info[0].ac_temperature, index);

                    device_info[1].ac_temperature = device_info[0].ac_temperature;

                    //Added by Nuos 
                    nuos_zb_set_hardware(0, false);

                    if(is_my_device_commissionned){ 
                        nuos_set_thermostat_temp_attribute(0);
                    }
                } else {
                    ESP_LOGW(TAG, "Thermostat cluster data: attribute(0x%x), type(0x%x)",
                            message->attribute.id, message->attribute.data.type);
                }
                #endif
        #endif
            }else if(message->cluster == ESP_ZB_ZCL_CLUSTER_ID_BASIC){
                if(message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING){
                    if (message->attribute.id == ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID) {

                    }else if (message->attribute.id == ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID) {

                    }
                    printf("Attribute value (string): %.*s\n", *(uint8_t *)message->attribute.data.value,
                           (char *)message->attribute.data.value + 1);                    
                }


            }else if (message->cluster == ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY) {
                if (message->attribute.id == ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID) {
                    //Added by Nuos
                    uint16_t val = *(int16_t *)message->attribute.data.value;
                    // printf("Val:%d\n", val);
                    for(int k=0; k<TOTAL_ENDPOINTS; k++){                         
                        if(ENDPOINTS_LIST[k] == message->dst_endpoint){
                            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI)
                                mode_change_flag = false;
                                selected_color_mode = 0;
                                nuos_zb_set_hardware(3, true);
                                set_state(3);
                                nuos_set_zigbee_attribute(3);
                            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
                                mode_change_flag = false;
                                nuos_zb_set_hardware(3, true);
                                set_state(3);
                                //set_dali_level(3);
                                nuos_set_zigbee_attribute(3);
                            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
                                device_info[k].device_level = 254;
                                is_long_press_brightness = false;
                                nuos_zb_set_hardware(k, 1);
                                set_state(k);
                                if(device_info[k].device_state)
                                set_dali_level(k);     
                                nuos_set_zigbee_attribute(k);  
                            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN)          
                                //set_curtain_percentage(device_info[0].device_level, true);  
                                if(val==0) {
                                    nuos_zb_set_leds_only(0, false);  
                                }else{
                                    nuos_zb_set_leds_only(0, true);  
                                }
                                                 
                            #else
                                #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN)
                                if(!device_info[0].device_state) device_info[0].device_state= true;
                                else device_info[0].device_state = false;
                                #endif
                                nuos_zb_set_hardware(k, true);    // toggle leds
                                nuos_set_zigbee_attribute(k);     // zigbee attributes
                            #endif
                           
                        }
                    }
                }
            }
        }
        //switch_driver_gpios_intr_enabled(true);
}

void nuos_set_attribute_cluster(const esp_zb_zcl_set_attr_value_message_t *message){
    #ifdef USE_QUEUE_CTRL 
        esp_zb_zcl_set_attr_value_message_t local_copy = *message; // Copy the struct after declaration
        if (xQueueSend(attr_evt_queue, &local_copy, portMAX_DELAY) != pdPASS) {
            ESP_LOGW(TAG, "Attribute event queue full, message dropped");
        }
    #else
    nuos_set_attribute_cluster_2(message);
    #endif
}

void nuos_set_scene_group_cluster(const esp_zb_zcl_recall_scene_message_t *message){
    #ifdef USE_SCENE_RECALL_QUEUE
        esp_zb_zcl_recall_scene_message_t local_copy = *message; // Copy the struct after declaration
        if (xQueueSend(scene_evt_queue, &local_copy, portMAX_DELAY) != pdPASS) {
            ESP_LOGW(TAG, "Scene event queue full, message dropped");
        }
    #else
    nuos_set_scene((esp_zb_zcl_recall_scene_message_t*)message);
    #endif 
}


void nuos_set_scene_store_cluster(const esp_zb_zcl_store_scene_message_t *message){
    #ifdef USE_SCENE_STORE_QUEUE
        esp_zb_zcl_store_scene_message_t local_copy = *message; // Copy the struct after declaration
        if (xQueueSend(scene_store_evt_queue, &local_copy, 0) != pdPASS) {
            ESP_LOGW(TAG, "Scene event queue full, message dropped");
        }
    #else
        nuos_set_store_scene((esp_zb_zcl_store_scene_message_t*)message);
    #endif 
}

esp_zb_zcl_privilege_command_message_t privilege_message;

#ifdef USE_QUEUE_CTRL 
esp_zb_zcl_set_attr_value_message_t queue_message;
static void attribute_task(void *pvParameters){
    while(1){
        vTaskDelay(pdMS_TO_TICKS(10));
        //if (xQueueReceive(attr_evt_queue, &queue_message, portMAX_DELAY)) {
        if (xQueueReceive(attr_evt_queue, &queue_message, 0)) {
            nuos_set_attribute_cluster_2(&queue_message);
        }
        // taskYIELD();
    }
}
#endif

#ifdef USE_SCENE_RECALL_QUEUE

static void set_scene_task(void *pvParameters){
    esp_zb_zcl_recall_scene_message_t queue_scene_message;
    while(1){ 
        vTaskDelay(pdMS_TO_TICKS(10));
        if (xQueueReceive(scene_evt_queue, &queue_scene_message, portMAX_DELAY)) {
            nuos_set_scene(&queue_scene_message);
        }
        taskYIELD();
    }
}
#endif

#ifdef USE_SCENE_STORE_QUEUE
static void scene_store_task(void *pvParameters){
    esp_zb_zcl_store_scene_message_t queue_scene_store_message;
    while(1){ 
        vTaskDelay(pdMS_TO_TICKS(10));
        if (xQueueReceive(scene_store_evt_queue, &queue_scene_store_message, portMAX_DELAY)) {
            nuos_set_store_scene(&queue_scene_store_message);
        }
        taskYIELD();
    }
}
#endif


#ifdef USE_TUYA_BRDIGE
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER  || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
    #ifdef USE_QUEUE_CTRL 
        static void privilege_task(void *pvParameters){
            while(1){
                vTaskDelay(pdMS_TO_TICKS(2));
                //if (xQueueReceive(privilege_evt_queue, &privilege_message, portMAX_DELAY)) {
                if (xQueueReceive(privilege_evt_queue, &privilege_message, portMAX_DELAY)) {
                    nuos_set_privilege_command_attribute_in_queue(&privilege_message);
                    //vTaskDelay(pdMS_TO_TICKS(10));
                } 
                taskYIELD();
                
            }
        }
        #endif
    #endif
#endif
int cccnntt = 0;
int disable_cnts = 0;
static void send_active_request_task(void* args){
    static bool pin_state = false;
    #ifdef USE_IR_UART_WS4_HW
    uart_init();
    #endif
    while(1){
        vTaskDelay(10 / portTICK_PERIOD_MS);

        // if(cccnntt++ >= 500){
        //     cccnntt = 0;
        //     disable_cnts = 0;
        //     dali_query_send(0x01, 0xA0);
        // }
        // if(disable_cnts++ >= 10){
        //     disable_cnts = 0;
        //     dali_disable_query_mode();
        // }

        #if(CHIP_INFO == USE_ESP32H2_MINI1_V5 || CHIP_INFO == USE_ESP32C6_MINI1_V5)
        if(boot_pin_toggle_counts++ >= 50){
            boot_pin_toggle_counts = 0;
            pin_state = !pin_state;
            gpio_set_level(GPIO_NUM_9, pin_state);
        }

        nuos_check_and_start_timer_for_touch_leds_off_after_1_minute();
        
        #endif
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_SCENE_SWITCH)
        if(timer_counter++ >= 15000){ //5 minutes or 60 seconds
            timer_counter = 0;
            printf("Sending Active request...\n");
            // Check if the device is joined
            if (is_my_device_commissionned) {
                // printf("BDB device joined!!\n");
                uint16_t attributes[] = {ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID};
                esp_zb_zcl_read_attr_cmd_t read_cmd = {
                    .zcl_basic_cmd.dst_endpoint = ENDPOINTS_LIST[0],
                    .zcl_basic_cmd.src_endpoint = ENDPOINTS_LIST[0],
                    .zcl_basic_cmd.dst_addr_u.addr_short = 0x0000,
                    .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
                    .clusterID = ESP_ZB_ZCL_CLUSTER_ID_BASIC,
                    .manuf_specific = 0,
                    #ifdef USE_HOME_ASSISTANT
                        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
                    #else
                        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
                    #endif
                    .manuf_code = 0,
                    .attr_number = 1
                    };
                read_cmd.attr_field  = attributes;
        
                esp_zb_zcl_read_attr_cmd_req(&read_cmd);
            }
        }    
        #endif   
    }
}

void nuos_start_bootloader() {
    #if(CHIP_INFO == USE_ESP32H2_MINI1_V5 || CHIP_INFO == USE_ESP32C6_MINI1_V5)
    gpio_set_level(GPIO_NUM_9, 0);
    #endif
    xTaskCreate(send_active_request_task, "active_req_task", 4096, NULL, TASK_PRIORITY_BOOT_PIN_TOGGLE, NULL);
}

bool nuos_init_sequence(){

    nuos_start_bootloader();
    #if(CHIP_INFO == USE_ESP32H2_MINI1_V5 || CHIP_INFO == USE_ESP32C6_MINI1_V5)
        gpio_set_level(GPIO_NUM_9, 1);
    #endif  
    #ifndef USE_NVS_INIT
        nuos_init_nvs();
    #endif
    #ifdef USE_WIFI_WEBSERVER
     //Added by Nuos   
    wifi_webserver_active_flag = getNVSWebServerEnableFlag();
    #else
    wifi_webserver_active_flag = false;
    #endif
    //Added by Nuos 
    start_commissioning = getNVSCommissioningFlag();
    is_my_device_commissionned = !start_commissioning;
    ready_commisioning_flag = getNVSStartCommissioningFlag();
    if(ready_commisioning_flag){
        printf("Clearing Start Commissioning Flag from NVS\n");
        ready_commisioning_flag = false;
        setNVSStartCommissioningFlag(0);
        nuos_write_default_value();
    }
    //Added by Nuos 
    #ifdef USE_RGB_LED
        nuos_init_rgb_led();
        nuos_blink_rgb_led_init_task();
    #else
        nuos_init_rgb_led();    
    #endif
    
    #ifdef USE_QUEUE_CTRL 
        attr_evt_queue = xQueueCreate(10, sizeof(esp_zb_zcl_set_attr_value_message_t));
        if ( attr_evt_queue == 0) {
            ESP_LOGE(TAG, "Attr Queue was not created and must not be used");
        }    
        xTaskCreate(attribute_task, "attribute_task", TASK_STACK_SIZE_COLOR, NULL, TASK_PRIORITY_ATTR, NULL);

        #ifdef USE_TUYA_BRDIGE
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER  || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
            
            privilege_evt_queue  = xQueueCreate(10, sizeof(esp_zb_zcl_privilege_command_message_t));
            if ( privilege_evt_queue == 0) {
                ESP_LOGE(TAG, "Privilege Attr Queue was not created and must not be used");
            } 
            
            xTaskCreate(privilege_task, "privilege_task", TASK_STACK_SIZE_PRIVILEGE, NULL, TASK_PRIORITY_PRIVILEGE, NULL);
            
            #endif

            #ifdef USE_SCENE_RECALL_QUEUE
                scene_evt_queue = xQueueCreate(10, sizeof(esp_zb_zcl_recall_scene_message_t));
                if ( scene_evt_queue == 0) {
                    ESP_LOGE(TAG, "Attr Queue was not created and must not be used");
                }    
                xTaskCreate(set_scene_task, "scene_task", TASK_STACK_SIZE_PRIVILEGE, NULL, 18, NULL);
            
            #endif 

            #ifdef USE_SCENE_STORE_QUEUE
                scene_store_evt_queue = xQueueCreate(10, sizeof(esp_zb_zcl_recall_scene_message_t));
                if ( scene_store_evt_queue == 0) {
                    ESP_LOGE(TAG, "Attr Queue was not created and must not be used");
                }    
                xTaskCreate(scene_store_task, "scene_store_task", 4098, NULL, 16, NULL);
            
            #endif 
            
        #endif 
    #endif
    #ifdef USE_WIFI_WEBSERVER
        if(wifi_webserver_active_flag > 0){ //This line, Added by NUOS 
            nuos_get_data_from_nvs();
            setNVSWebServerEnableFlag(0); 
            #ifndef ZB_FACTORY_RESET
                #ifdef USE_WIFI_WEBSERVER
                    nuos_init_wifi_webserver();
                #endif 
                return false;             
            #else
                return true; 
            #endif 
        } 
    #endif
    return true;
}
