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

#define DECLARE_MAIN
#include <stdlib.h>
#include <time.h>
#include "esp_system.h"
#include <esp_zigbee_core.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "ha/esp_zigbee_ha_standard.h"
// #include "zcl/zb_zcl_common.h"
#include "esp_zb_controller_main.h"
#include "switch_driver.h"
#include "app_hardware_driver.h"
#include "app_zigbee_clusters.h"
#include "esp_netif.h"
#include "esp_event.h"
#ifdef USE_OTA
#include "esp_ota_ops.h"
#if CONFIG_ZB_DELTA_OTA
#include "esp_delta_ota_ops.h"
#endif
#include "esp_timer.h"
#include <sys/time.h>
#include "zcl/esp_zigbee_zcl_common.h"
#include "app_zigbee_clusters.h"
#include "app_zigbee_group_commands.h"

// bool stack_initialised = false;

// static bool on_startup_getting_link_failure = true;

    #define OTA_ELEMENT_HEADER_LEN              6       /* OTA element format header size include tag identifier and length field */

    /**
     * @name Enumeration for the tag identifier denotes the type and format of the data within the element
     * @anchor esp_ota_element_tag_id_t
     */
    typedef enum esp_ota_element_tag_id_e {
        UPGRADE_IMAGE                               = 0x0000,           /*!< Upgrade image */
    } esp_ota_element_tag_id_t;


    static const esp_partition_t *s_ota_partition = NULL;
    static esp_ota_handle_t s_ota_handle = 0;
    static bool s_tagid_received = false;
    
    #define OTA_UPGRADE_QUERY_INTERVAL (1 * 10) // 1 minutes

#endif

// #if defined ZB_ED_ROLE
//     #error Define ZB_COORDINATOR_ROLE in idf.py menuconfig to compile light switch source code.
// #endif

static const char *TAG = "ZB_MAIN_FILE";

typedef struct {
    uint8_t endpoint;
    uint8_t *data;
    uint16_t length;
} zigbee_response_t;

QueueHandle_t response_queue;


typedef struct light_bulb_device_params_s {
    esp_zb_ieee_addr_t ieee_addr;
    uint8_t  endpoint;
    uint16_t short_addr;
} light_bulb_device_params_t;

#if(TOTAL_BUTTONS == 4)
    static switch_func_pair_t button_func_pair[] = {
        {gpio_touch_btn_pins[0], SWITCH_ONOFF_TOGGLE_CONTROL},
        {gpio_touch_btn_pins[1], SWITCH_ONOFF_TOGGLE_CONTROL},
        {gpio_touch_btn_pins[2], SWITCH_ONOFF_TOGGLE_CONTROL},
        {gpio_touch_btn_pins[3], SWITCH_ONOFF_TOGGLE_CONTROL}
    };
#elif(TOTAL_BUTTONS == 2)
    static switch_func_pair_t button_func_pair[] = {
        {gpio_touch_btn_pins[0], SWITCH_ONOFF_TOGGLE_CONTROL},
        {gpio_touch_btn_pins[1], SWITCH_ONOFF_TOGGLE_CONTROL}
    };
#elif(TOTAL_BUTTONS == 1)
    static switch_func_pair_t button_func_pair[] = {
        {gpio_touch_btn_pins[0], SWITCH_ONOFF_TOGGLE_CONTROL}
    }; 
#elif(TOTAL_BUTTONS == 0)
    static switch_func_pair_t button_func_pair[] = {
    };       
#endif

int get_random_number(int min, int max) {
    srandom((unsigned) time(NULL));
    // Use rand() to generate a random number
    return (rand() % (max - min + 1)) + min;
}

#ifdef USE_CCT_TIME_SYNC
void print_current_time(){
    // Print updated time
    time_t now = time(NULL);
    struct tm *t = localtime(&now);
    printf("Date (%d/%d/%d)\n", t->tm_mday, t->tm_mon + 1, t->tm_year + 1900);
    printf("Time (%02d:%02d:%02d)\n", t->tm_hour, t->tm_min, t->tm_sec);


}


void set_time(int year, int mon, int day, int hour, int min, int sec){

    struct timeval tv;
    struct tm tm_info = {0};

    // ✅ Set time: March 7, 2025, 15:30:11
    tm_info.tm_year = year - 1900; // tm_year is years since 1900, so 2025 -> 125
    tm_info.tm_mon = mon;            // March (0-based, Jan = 0, so March = 2)
    tm_info.tm_mday = day;           // Day of the month
    tm_info.tm_hour = hour;
    tm_info.tm_min = min;
    tm_info.tm_sec = sec;

    time_t unix_time = mktime(&tm_info);
    if (unix_time == -1) {
        printf("Error: mktime() failed to convert time.\n");
    }

    printf("Unix timestamp: %lld\n", unix_time);  // Debugging line

    tv.tv_sec = unix_time;
    tv.tv_usec = 0;

    // ✅ Set system time
    settimeofday(&tv, NULL);

    // ✅ Get updated time
    time_t now = time(NULL);
    struct tm *t = localtime(&now);
    printf("Date (%d/%d/%d)\n", t->tm_mday, t->tm_mon + 1, t->tm_year + 1900);
    printf("Time (%02d:%02d:%02d)\n", t->tm_hour, t->tm_min, t->tm_sec);
}
#endif
static void zb_buttons_handler(switch_func_pair_t *button_func_pair)
{
    if (button_func_pair->func == SWITCH_ONOFF_TOGGLE_CONTROL) {
        uint32_t* io_index = &button_func_pair->pin;
        uint32_t button_index = *io_index;
        printf("button_index:%ld", button_index);
        //for curtain only
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN_SWITCH)
            #ifdef TUYA_ATTRIBUTES
            if(bCalMode){
                //printf("bCalMode:%d vtaskMode:%d \n", bCalMode, vTaskMode);
                if(vTaskMode == TASK_CURTAIN_CAL_INIT){
                    if(button_index == 0){  //if 1st switch pressed
                        //printf("INIT\n");
                        vTaskMode = TASK_CURTAIN_CAL_START;
                        gpio_set_level(gpio_touch_led_pins[0], 0);
                        start_time = esp_timer_get_time();
                        pause_curtain_timer();
                         device_info[0].device_state = CURTAIN_OPEN;
                        nuos_zb_set_hardware_curtain(0, false);
                        // uint8_t calibration_mode = ESP_ZB_ZCL_ATTR_WINDOW_COVERING_TYPE_RUN_IN_CALIBRATION_MODE;
                        // esp_zb_zcl_set_attribute_val(1,
                        //     ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
                        //     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                        //     ESP_ZB_ZCL_ATTR_WINDOW_COVERING_WINDOW_COVERING_TYPE_ID,
                        //     &calibration_mode,
                        //     false);
                        // uint8_t cal_started = 0;
                        // esp_zb_zcl_set_attribute_val(1,
                        //     ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
                        //     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                        //     0xF001,
                        //     &cal_started,
                        //     false);                            
                    }
                }else if(vTaskMode == TASK_CURTAIN_CAL_START){
                    if(button_index == 1){  //if 2nd switch pressed
                        vTaskMode = TASK_CURTAIN_CAL_END;
                        device_info[0].device_val = (esp_timer_get_time() - start_time) / 1000;
                        device_info[0].device_val = device_info[0].device_val / 1000;
                        device_info[0].device_state = CURTAIN_CLOSE;
                        nuos_zb_set_hardware_curtain(1, false);
                        // uint8_t calibration_mode = ESP_ZB_ZCL_ATTR_WINDOW_COVERING_TYPE_MOTOR_IS_RUNNING_IN_MAINTENANCE_MODE;
                        // esp_zb_zcl_set_attribute_val(1,
                        //     ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
                        //     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                        //     ESP_ZB_ZCL_ATTR_WINDOW_COVERING_WINDOW_COVERING_TYPE_ID,
                        //     &calibration_mode,
                        //     false);    
                        
                        // uint8_t cal_started = 1;
                        // esp_zb_zcl_set_attribute_val(1,
                        //     ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
                        //     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                        //     0xF001,
                        //     &cal_started,
                        //     false);  

                        // esp_zb_zcl_status_t state = esp_zb_zcl_set_attribute_val(
                        // ENDPOINTS_LIST[0],
                        // ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
                        // ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                        // 0xF003,
                        // &device_info[0].device_val,
                        // false);                        
                    }
                }
            }
            #endif 
        #endif
        uchKeypressed = button_func_pair->keypressed;
        switch(uchKeypressed){
            case SINGLE_PRESS:
                nuos_switch_single_click_task(button_index);
                break;
            case DOUBLE_PRESS:
                nuos_switch_double_click_task(button_index);
                break;
            case LONG_PRESS:
            printf("LONGPRESSED1!!\n");
                nuos_switch_long_press_task(button_index); 
                break;
            case LONG_PRESS_INC_DEC_LEVEL:
                nuos_switch_long_press_brightness_task(button_index);
                break;    
            default: 
                printf("Invalid Switch Pressed!!\n");
                break;    
        }
        button_func_pair->keypressed = 0xff; //no key pressed
    }
}

static esp_err_t deferred_driver_init(void)
{
    // //Added by Nuos  commented on 04-02-2025
    ESP_RETURN_ON_FALSE(switch_driver_init(button_func_pair, PAIR_SIZE(button_func_pair), zb_buttons_handler), ESP_FAIL, TAG,
                        "Failed to initialize switch driver");

    //Added by Nuos                    
    nuos_driver_init(); 
    //Added by Nuos 
    nuos_zb_init_motion_sensor();                
 
    //Added by Nuos 
    init_timer_3(); 
    esp_start_timer_3();
    //Added by Nuos
    nuos_check_nvs_start_commissioning();

    #ifdef USE_CCT_TIME_SYNC
        init_color_temp_timer(); 
    #endif
    init_nvs_for_zb_devices();
    //don't uncomment below line,keep it commented!!
    // for(int i=0; i<15; i++) {
    // 	esp_err_t status = esp_zb_zcl_scenes_table_clear_by_index(i);
    // }
    for(int j=0; j<TOTAL_ENDPOINTS; j++) {
        esp_zb_zcl_scenes_table_show(ENDPOINTS_LIST[j]);
    }
    return ESP_OK;
}

static void simple_desc_cb(esp_zb_zdp_status_t zdo_status, esp_zb_af_simple_desc_1_1_t *simple_desc, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Simple desc response: status(%d), device_id(%d), app_version(%d), profile_id(0x%x), endpoint_ID(%d)", zdo_status,
                 simple_desc->app_device_id, simple_desc->app_device_version, simple_desc->app_profile_id, simple_desc->endpoint);

        for (int i = 0; i < (simple_desc->app_input_cluster_count + simple_desc->app_output_cluster_count); i++) {
            ESP_LOGI(TAG, "Cluster ID list: 0x%x", *(simple_desc->app_cluster_list + i));
        }
    }
}

static void ep_cb(esp_zb_zdp_status_t zdo_status, uint8_t ep_count, uint8_t *ep_id_list, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {

        ESP_LOGI(TAG, "Active endpoint response: status(%d) and endpoint count(%d)", zdo_status, ep_count);
        for (int i = 0; i < ep_count; i++) {
            ESP_LOGI(TAG, "Endpoint ID List: %d", ep_id_list[i]);
            if(ep_id_list[i] != 242){
                /* get the node simple descriptor */
                esp_zb_zdo_simple_desc_req_param_t simple_desc_req;
                simple_desc_req.addr_of_interest = 0;
                simple_desc_req.endpoint = ep_id_list[i];
                esp_zb_zdo_simple_desc_req(&simple_desc_req, simple_desc_cb, NULL);
            }
        }
    }
}



static void ieee_cb(esp_zb_zdp_status_t zdo_status, esp_zb_zdo_ieee_addr_rsp_t *resp, void *user_ctx){
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        esp_zb_zdo_ieee_addr_rsp_t *ieee_resp = (esp_zb_zdo_ieee_addr_rsp_t *)resp;
		nuos_zb_bind_cb(zdo_status, resp->ieee_addr, NULL);       
    }
}

static void user_find_cb(esp_zb_zdp_status_t zdo_status, uint16_t addr, uint8_t endpoint, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Found Light Cluster from Gateway with EP:%d", endpoint);
        found_clusters_flag = true; 

        if (endpoint == 255) {
            // This is gateway with no dynamic clusters created like Alexa and Espressif zigbee gateway,
            // that has only 1 gateway endpoint
            single_endpoint_gateway = true;
            // Endpoint not found, request simple descriptor
            esp_zb_zdo_simple_desc_req_param_t simple_desc_req;
            simple_desc_req.addr_of_interest = addr;
            simple_desc_req.endpoint = 1;
            esp_zb_zdo_simple_desc_req(&simple_desc_req, simple_desc_cb, NULL);
        } else {
            single_endpoint_gateway = false;
            // Proceed to bind
            nuos_zb_request_ieee_address(zdo_status, addr, endpoint, user_ctx, ieee_cb);
        }

    }
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee commissioning");
}


static void zb_zdo_match_desc_handler(esp_zb_zdp_status_t zdo_status, uint16_t addr, uint8_t endpoint, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        #ifdef USE_OTA
        esp_zb_ota_upgrade_client_query_interval_set(ESP_OTA_CLIENT_ENDPOINT, OTA_UPGRADE_QUERY_INTERVAL);
        esp_zb_ota_upgrade_client_query_image_req(addr, endpoint);
        ESP_LOGI(TAG, "Query OTA upgrade from server endpoint: %d after %d seconds", endpoint, OTA_UPGRADE_QUERY_INTERVAL);
        #endif
    } else {
        ESP_LOGW(TAG, "No OTA Server found");
    }
}

static bool network_steering_mode_flag = false;
// void esp_zb_app_signal_handler111(esp_zb_app_signal_t *signal_struct)
// {
//     uint32_t *p_sg_p     = signal_struct->p_app_signal;
//     esp_err_t err_status = signal_struct->esp_err_status;
//     esp_zb_app_signal_type_t sig_type = *p_sg_p;
//     esp_zb_zdo_signal_device_annce_params_t *dev_annce_params = NULL;
//     switch (sig_type) {
//     case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
//         ESP_LOGI(TAG, "Initialize Zigbee stack");
//         esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
//         break;
//     case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
//     case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
//         if (err_status == ESP_OK) {
//             ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
//             ESP_LOGI(TAG, "Device started up in%s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : " non");
//             if (esp_zb_bdb_is_factory_new()) {
//                 esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_TOUCHLINK_TARGET);
//             } else {
//                 ESP_LOGI(TAG, "Device rebooted");
//                 ESP_LOGI(TAG, "My Short Address : 0x%x", esp_zb_get_short_address());
//             }
//         } else {
//             ESP_LOGW(TAG, "%s failed with status: %s, retrying", esp_zb_zdo_signal_to_string(sig_type),
//                      esp_err_to_name(err_status));
//             esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
//         }
//         break;
//     case ESP_ZB_BDB_SIGNAL_TOUCHLINK_TARGET:
//         ESP_LOGI(TAG, "Touchlink target is ready, awaiting commissioning");

//         break;
//     case ESP_ZB_BDB_SIGNAL_TOUCHLINK_NWK:
//         if (err_status == ESP_OK) {
//             esp_zb_ieee_addr_t extended_pan_id;
//             esp_zb_get_extended_pan_id(extended_pan_id);
//             ESP_LOGI(TAG,
//                      "Commissioning successfully, network information (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, "
//                      "Channel:%d, Short Address: 0x%04hx)",
//                      extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4], extended_pan_id[3], extended_pan_id[2],
//                      extended_pan_id[1], extended_pan_id[0], esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
//         }
//         break;
//     case ESP_ZB_BDB_SIGNAL_TOUCHLINK_TARGET_FINISHED:
//         if (err_status == ESP_OK) {
//             ESP_LOGI(TAG, "Touchlink target commissioning finished");
//         } else {
//             ESP_LOGI(TAG, "Touchlink target commissioning failed");
//         }
//         break;
//     case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
//         dev_annce_params = (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(p_sg_p);
//         ESP_LOGI(TAG, "New device commissioned or rejoined (short: 0x%04hx)", dev_annce_params->device_short_addr);
//         break;
//     case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
//         if (err_status == ESP_OK) {
//             if (*(uint8_t *)esp_zb_app_signal_get_params(p_sg_p)) {
//                 ESP_LOGI(TAG, "Network(0x%04hx) is open for %d seconds", esp_zb_get_pan_id(), *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p));
//             } else {
//                 ESP_LOGW(TAG, "Network(0x%04hx) closed, devices joining not allowed.", esp_zb_get_pan_id());
//             }
//         }
//         break;
//     default:
//         ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status));
//         break;
//     }
// }
bool joining_signal_received = false;

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{ 
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    esp_zb_zdo_signal_device_annce_params_t *dev_annce_params = NULL;
    esp_zb_zdo_signal_leave_params_t *leave_params = NULL;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        // stack_initialised = true;
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        //esp_zb_nwk_set_link_status_period(300); 
        network_steering_mode_flag = false;
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network steering...\n");
                #ifdef USE_RGB_LED
                    light_driver_set_power(false);
                    if(start_commissioning>0){
                        printf("Commissioning Flag True\n");
                        //nuos_start_rgb_led_blink_task(0);
                    }                   
                #endif
 
                    if(getNVSPanicAttack() > 0){
                        setNVSPanicAttack(0);
                        
                        ready_commisioning_flag = false;
                        setNVSStartCommissioningFlag(0);
                        setNVSCommissioningFlag(1);
                        start_commissioning = true;
                        is_my_device_commissionned = !start_commissioning;
                        nvs_flash_erase();                      
                    }else{
                        esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 5000);
                        network_steering_mode_flag = true;  
                    }
            } else {
                ESP_LOGI(TAG, "Device rebooted");
                ESP_LOGI(TAG, "My Short Address : 0x%x", esp_zb_get_short_address());
                network_steering_mode_flag = false;

                nuos_zb_find_clusters(user_find_cb);
                // esp_zb_rejoin_network(true);
                if (!esp_zb_bdb_dev_joined()) {
                    printf("BDB not joined!!.....\n");
                    esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 2000);
                }
                #ifdef USE_FADING
                init_fading();  
                #endif
            }
          
        } else {
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
            
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
        }
        break;
    // case ESP_ZB_BDB_SIGNAL_TOUCHLINK_TARGET:
    //     ESP_LOGI(TAG, "Touchlink target is ready, awaiting commissioning");
    //     break;
    // case ESP_ZB_BDB_SIGNAL_TOUCHLINK_NWK:
    //     if (err_status == ESP_OK) {
    //         esp_zb_ieee_addr_t extended_pan_id;
    //         esp_zb_get_extended_pan_id(extended_pan_id);
    //         ESP_LOGI(TAG,
    //                  "Commissioning successfully, network information (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, "
    //                  "Channel:%d, Short Address: 0x%04hx)",
    //                  extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4], extended_pan_id[3], extended_pan_id[2],
    //                  extended_pan_id[1], extended_pan_id[0], esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            
    //         // esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
    //     }
    //     break;
    // case ESP_ZB_BDB_SIGNAL_TOUCHLINK_TARGET_FINISHED:
    //     if (err_status == ESP_OK) {
    //         ESP_LOGI(TAG, "Touchlink target commissioning finished");
    //     } else {
    //         ESP_LOGI(TAG, "Touchlink target commissioning failed");
    //     }
    //     // esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
    //     break;        
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            //esp_zb_zdo_pim_set_long_poll_interval(ED_KEEP_ALIVE);
            //Added by Nuos  
            joining_signal_received = true;
            nuos_zb_find_clusters(user_find_cb);
        } else {
            joining_signal_received = false;
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 5000);
            
            
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
        dev_annce_params = (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        ESP_LOGI(TAG, "New device commissioned or rejoined (short: 0x%04hx)", dev_annce_params->device_short_addr);
        break;

    case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
        if (err_status == ESP_OK) {
            if (*(uint8_t *)esp_zb_app_signal_get_params(p_sg_p)) {
                ESP_LOGI(TAG, "Network(0x%04hx) is open for %d seconds", esp_zb_get_pan_id(), *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p));
                joining_signal_received = true;
            } else {
                ESP_LOGW(TAG, "Network(0x%04hx) closed, devices joining not allowed.", esp_zb_get_pan_id());
                joining_signal_received = false;
            }
        }
        break;

    case ESP_ZB_ZDO_SIGNAL_LEAVE:         
        // Attempt to rejoin the network
        esp_zb_bdb_commissioning_mode_mask_t mode_leave = esp_zb_get_bdb_commissioning_mode();
        if(mode_leave == 0) esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING); 
        leave_params = (esp_zb_zdo_signal_leave_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        //log_v("Signal to leave the network, leave type: %d", leave_params);
        if (leave_params->leave_type == ESP_ZB_NWK_LEAVE_TYPE_RESET) {  // Leave without rejoin -> Factory reset
            printf("Leave without rejoin, factory reset the device\n");
            if(is_my_device_commissionned){
                is_my_device_commissionned = false;
                ready_commisioning_flag = true;
                setNVSStartCommissioningFlag(0);
                setNVSCommissioningFlag(1);          
                esp_zb_factory_reset();
            }
        } else {  // Leave with rejoin -> Rejoin the network, only reboot the device
          printf("Leave with rejoin, only reboot the device\n");
          //esp_restart();
          //esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } 			
        break;
         
    case ESP_ZB_ZDO_DEVICE_UNAVAILABLE:
        // Attempt to rejoin the network
        esp_zb_bdb_commissioning_mode_mask_t mode_unavailable = esp_zb_get_bdb_commissioning_mode();
        printf("SIGNAL:UNAVAILABLE = %d\n", mode_unavailable);
        if(mode_unavailable == 0) esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        //is_my_device_commissionned = false;        
        break;
    default:
         //ZB_NWK_COMMAND_STATUS_NO_ROUTE_AVAILABLE 
        //ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status));
        uint8_t param = *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p);
        printf("%s, status: 0x%x\n", esp_zb_zdo_signal_to_string(sig_type), param);
        if (param == 0x09U) {       
            taskYIELD();  
        }        
        break;
    }
}

// uint16_t cb1_counts = 0;
// extern void send_report(int index, uint16_t cluster_id, uint16_t attr_id);
static esp_err_t zb_write_attribute_handler(const esp_zb_zcl_cmd_write_attr_resp_message_t *message)
{
    esp_err_t ret = ESP_OK;
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
     ESP_LOGI(TAG, "Received message A: endpoint(%d), cluster(0x%x)", 
             message->info.dst_endpoint, message->info.cluster);

    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_GROUP_SWITCH) 
        if(zigbee_sem != NULL) xSemaphoreGive(zigbee_sem);
    #endif

    return ret;
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message B: endpoint(%d), cluster(0x%x), attribute(0x%x)", 
            message->info.dst_endpoint, message->info.cluster, message->attribute.id);
    
    nuos_set_attribute_cluster(message);
    return ret;
}

static esp_err_t zb_basic_reset_factory_handler(const esp_zb_zcl_basic_reset_factory_default_message_t *message){
    esp_err_t ret = ESP_OK;
    
   ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
   ESP_RETURN_ON_FALSE( message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, 
                        ESP_ERR_INVALID_ARG, TAG, 
                        "Received message: error status(%d)",
                        message->info.status);
                       
                  
   // ESP_LOGI(TAG, "Received message: status_code(0x%x), resp_to_cmd(0x%x)", message->status_code, message->resp_to_cmd);
    ESP_LOGI(TAG, "cluster(0x%x), dst_ep(%d)", message->info.cluster, message->info.dst_endpoint);  
    esp_zb_bdb_reset_via_local_action();  
    esp_zb_factory_reset();
   // ESP_LOGI(TAG, "dst_address(%d), src_endpoint(%d)", message->info.dst_endpoint, message->info.src_endpoint); 
   return ret;
}

static esp_err_t zb_cmd_default_handler(const esp_zb_zcl_cmd_default_resp_message_t *message){
     esp_err_t ret = ESP_OK;
     
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    // ESP_LOGI(TAG, "Received message: status_code(0x%x), resp_to_cmd(0x%x)", message->status_code, message->resp_to_cmd);
    // ESP_LOGI(TAG, "cluster(%d), dst_address(0x%x)", message->info.cluster, message->info.dst_address);  
    // ESP_LOGI(TAG, "dst_address(%d), src_endpoint(%d)", message->info.dst_endpoint, message->info.src_endpoint); 
    return ret;
}

static esp_err_t zb_cmd_ias_zone_handler(const esp_zb_zcl_ias_zone_enroll_response_message_t *message){
     esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    ESP_LOGI(TAG, "Received message C: zone_id(0x%x), response_code(0x%x)", message->zone_id, message->response_code);
    ESP_LOGI(TAG, "cluster(%d), dst_endpoint(0x%x)", message->info.cluster, message->info.dst_endpoint);   
    
    return ret;
}

static esp_err_t zb_cmd_identify_effect_handler(const esp_zb_zcl_identify_effect_message_t *message){
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    ESP_LOGI(TAG, "Received message D: effect_id(0x%x), effect_variant(0x%x)", message->effect_id, message->effect_variant);
    ESP_LOGI(TAG, "cluster(%d), dst_endpoint(0x%x)", message->info.cluster, message->info.dst_endpoint);
    //Added by Nuos   
    nuos_zb_indentify_indication(message->info.cluster, message->info.dst_endpoint, message->effect_id);
    return ret;
}


uint32_t count = 3;
uint8_t last_cmd_id = 0;

static esp_err_t zb_cmd_privilege_command_handler(const esp_zb_zcl_custom_cluster_command_message_t *message){
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
   
    ESP_LOGI(TAG, "cluster(0x%x), command(0x%x), dst_ep(%d)", message->info.cluster, message->info.command.id, message->info.dst_endpoint);
    ESP_LOGI(TAG, "count(%ld)", count++);
    //data is coming 3 times, so avoid it.
    // if((last_cmd_id != message->info.command.id || message->info.command.id == 0xe1)){
    //     last_cmd_id =  message->info.command.id;
        //nuos_set_privilege_command_attribute(message); 
    // }
    return ret;
}


esp_err_t sequence_num = 0;
uint8_t data_cmd_0x10[2] = {0};
uint8_t data_cmd_0x01[5] = {0};
uint8_t data_cmd_0xfc[17] = {0};
uint8_t data_cmd_0x08[4] = {0};
uint8_t data_cmd_0x08_3[3] = {0};

#define ARRAY_LENTH(arr) (sizeof(arr) / sizeof(arr[0]))
void send_config_report(int index, uint16_t cluster_id, uint16_t attr_id, uint8_t attr_type, uint16_t short_addr){
    /* Send "configure report attribute" command to the bound sensor */

    int16_t report_change = 1; // Report on each change 
    esp_zb_zcl_config_report_cmd_t report_cmd = {0};
    report_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_GROUP_ENDP_NOT_PRESENT;
    report_cmd.zcl_basic_cmd.dst_addr_u.addr_short = short_addr;  // Short address of Server B
    report_cmd.zcl_basic_cmd.dst_endpoint = 0;  // Endpoint of Server B
    report_cmd.zcl_basic_cmd.src_endpoint = 1;  // Client A's endpoint
    report_cmd.clusterID = cluster_id;  // Example: On/Off cluster
    
    esp_zb_zcl_config_report_record_t records[] = {
        {
            .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
            .attributeID = attr_id,
            .attrType = attr_type,
            .min_interval = 1,
            .max_interval = 0,
            .reportable_change = &report_change,
        },        
    };
    
    report_cmd.record_number = ARRAY_LENTH(records);
    report_cmd.record_field = records;
    
    esp_zb_zcl_config_report_cmd_req(&report_cmd);
  
}


static esp_err_t zb_cmd_custom_cluster_handler(const esp_zb_zcl_custom_cluster_command_message_t *message){
  

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
                        
                        
    ESP_LOGI(TAG, "cluster(0x%x), dst_endpoint(%d), src_endpoint(%d)\n", message->info.cluster, message->info.dst_endpoint, message->info.src_endpoint);
    // ESP_LOGI(TAG, "command(0x%x) size(%d)\n", message->info.command.id, message->size);

    ESP_LOGI(TAG, "fc(0x%x) manuf_code(0x%x)\n", message->info.header.fc, message->info.header.manuf_code);

    // 0x0 0x32 0x1 0x4 0x0 0x1 0x0 0x42 0x1a 0x0 0x0 0x0 0xf 0x0 0x0
    // 0x0 0x33 0x1 0x4 0x0 0x1 0x0 
    // 0x0 0x34 0x2 0x2 0x0 0x4 0x0 0x42 0x1a 0x0
    // 0x0 0x34 0x2 0x2 0x0 0x4 0x0 0x0 0x0 0x64
    #ifdef TUYA_ATTRIBUTES
    uint56_t thermostat_data = {0};
    uint80_t thermostat_temp = {0};
    printf("\n"); 
     
    if(message->info.command.id == 0xfc) {  //set scene
        printf("SIZE:%d\n", message->data.size);
        if(message->data.size == 17 || (message->data.size == 18)){
            if(message->data.size == 18){
                memcpy(data_cmd_0xfc, (uint8_t*)&message->data.value[1], sizeof(data_cmd_0xfc));
            }else{
                memcpy(data_cmd_0xfc, message->data.value, sizeof(data_cmd_0xfc));
            }
            
            for(int i=0; i<message->data.size; i++){
                printf("0x%x ", data_cmd_0xfc[i]);
            }
            printf("\n");
            uint8_t index = nuos_get_endpoint_index(data_cmd_0xfc[3]);
            zb_scene_info[index].dst_ep =  data_cmd_0xfc[3];  //scene switch button endpoint
            zb_scene_info[index].group_id = (((data_cmd_0xfc[6] << 8) & 0xff00) | data_cmd_0xfc[7]);
            zb_scene_info[index].scene_id = (((data_cmd_0xfc[10] << 8) & 0xff00) | data_cmd_0xfc[11]);
            writeSceneInfoStruct(index, (zigbee_zcene_info_t*)&zb_scene_info[index]);
            printf("ep: %d | group: 0x%x |  scene:%d\n", zb_scene_info[index].dst_ep, zb_scene_info[index].group_id, zb_scene_info[index].scene_id);

            
            esp_zb_zcl_custom_cluster_cmd_resp_t resp = {0};
            resp.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
            resp.cluster_id = message->info.cluster; // Same as incoming
            resp.profile_id = ESP_ZB_AF_HA_PROFILE_ID; // Or your profile
            resp.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI; // Server to Client
            // Set destination to sender of original command
            resp.zcl_basic_cmd.dst_addr_u.addr_short = 0;
            resp.zcl_basic_cmd.dst_endpoint = message->info.dst_endpoint;
            resp.zcl_basic_cmd.src_endpoint = message->info.src_endpoint; // Your endpoint
            resp.custom_cmd_id = 2; // Your response command ID
            // Prepare payload
            resp.data.type = ESP_ZB_ZCL_ATTR_TYPE_ARRAY;
            resp.data.size = message->data.size;
            resp.data.value = data_cmd_0xfc;
            // Manufacturer fields if needed
            resp.manuf_code = 0;
            resp.manuf_specific = 0;
            // Send the response
            esp_zb_zcl_custom_cluster_cmd_resp(&resp);       

        }
    }else if(message->info.command.id == 0xe1){  //set scene
        uint32_ts data_cmd_0x04 = {0};
        if(message->data.size == 4){
            memcpy(data_cmd_0x04.bytes, message->data.value, sizeof(data_cmd_0x04.bytes));
            for(int i=0; i<message->data.size; i++){
                printf("0x%x ", data_cmd_0x04.bytes[i]);
            }
            printf("\n");  

            uint8_t index = nuos_get_endpoint_index(data_cmd_0x04.bytes[0]); 

            zb_scene_info[index].dst_ep =  data_cmd_0x04.bytes[0];
            
            zb_scene_info[index].group_id = ((data_cmd_0x04.bytes[2] << 8) | data_cmd_0x04.bytes[3]);
            zb_scene_info[index].scene_id = data_cmd_0x04.bytes[1];
            printf("group: 0x%x\n", zb_scene_info[index].group_id);
            writeSceneInfoStruct(index, (zigbee_zcene_info_t*)&zb_scene_info[index]);

            printf("ep: %d | group: 0x%x |  scene:%d\n", zb_scene_info[index].dst_ep, zb_scene_info[index].group_id, zb_scene_info[index].scene_id);
            
           

            esp_zb_zcl_custom_cluster_cmd_req_t cmd_req = {
                .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
                .cluster_id = message->info.cluster,
                .custom_cmd_id = 1,
                .data.value = data_cmd_0x04.bytes,
                .data.size = message->data.size,
                .data.type = ESP_ZB_ZCL_ATTR_TYPE_U32,
                .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
                .profile_id = 0x0104,
                .zcl_basic_cmd.dst_addr_u.addr_short = 0,
                .zcl_basic_cmd.dst_endpoint = message->info.dst_endpoint,
                .zcl_basic_cmd.src_endpoint = message->info.src_endpoint,
                .dis_default_resp = 1,
                .manuf_code = 0,
                .manuf_specific = 0, 
            };
            sequence_num = esp_zb_zcl_custom_cluster_cmd_req(&cmd_req); 

            esp_zb_zcl_set_attribute_val(
                message->info.dst_endpoint,
                0xE000,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                0xD004,
                (uint16_t*)&data_cmd_0x04.bytes[1],
                false
            );
            esp_zb_zcl_report_attr_cmd_t report_cmd_2 = {
                .zcl_basic_cmd = {
                    .dst_addr_u.addr_short = 0x0000,
                    .dst_endpoint = message->info.dst_endpoint,
                    .src_endpoint = message->info.src_endpoint,
                },
                #ifdef NEW_SDK_6
                    .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
                #else
                    .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                #endif
                
                .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
                .clusterID = 0xE000,
                .attributeID = 0xD004,
            };
            esp_zb_zcl_report_attr_cmd_req(&report_cmd_2);    
            esp_zb_zcl_set_attribute_val(
                message->info.dst_endpoint,
                0xE000,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                0xD005,
                (uint8_t*)&data_cmd_0x04.bytes[3],
                false
            );

            esp_zb_zcl_report_attr_cmd_t report_cmd_3= {
                .zcl_basic_cmd = {
                    .dst_addr_u.addr_short = 0x0000,
                    .dst_endpoint = message->info.dst_endpoint,
                    .src_endpoint = message->info.src_endpoint,
                },
                #ifdef NEW_SDK_6
                    .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
                #else
                    .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                #endif
                
                .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
                .clusterID = 0xE000,
                .attributeID = 0xD005,
            };
            esp_zb_zcl_report_attr_cmd_req(&report_cmd_3);

            // nuos_zigbee_group_query_all_groups(zb_scene_info[index].group_id);
            //send_config_report(index, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, ESP_ZB_ZCL_ATTR_TYPE_BOOL, zb_scene_info[index].group_id);
        }        
    }else if(message->info.command.id == 0x01){
        if(message->data.size == 5){
            memcpy(data_cmd_0x01, message->data.value, sizeof(data_cmd_0x01));
            for(int i=0; i<message->data.size; i++){
                printf("0x%x ", data_cmd_0x01[i]);
            }
            printf("\n");  

            esp_zb_zcl_custom_cluster_cmd_req_t cmd_req = {
                .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
                .cluster_id = message->info.cluster,
                .custom_cmd_id = 1,
                .data.value = message->data.value,
                .data.size = message->data.size,
                .data.type = ESP_ZB_ZCL_ATTR_TYPE_U40,
                .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
                .profile_id = 0x0104,
                .zcl_basic_cmd.dst_addr_u.addr_short = 0,
                .zcl_basic_cmd.dst_endpoint = message->info.dst_endpoint,
                .zcl_basic_cmd.src_endpoint = message->info.src_endpoint,
                .dis_default_resp = 1,
                .manuf_code = 0,
                .manuf_specific = 0, 
            };
            sequence_num = esp_zb_zcl_custom_cluster_cmd_req(&cmd_req); 

        }
    }else if(message->info.command.id == 0x08){
        if(message->data.size == 4){
            memcpy(data_cmd_0x08, message->data.value, sizeof(data_cmd_0x08));
            for(int i=0; i<message->data.size; i++){
                printf("0x%x ", data_cmd_0x08[i]);
            }
            printf("\n");  

            esp_zb_zcl_custom_cluster_cmd_req_t cmd_req = {
                .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
                .cluster_id = message->info.cluster,
                .custom_cmd_id = 1,
                .data.value = message->data.value,
                .data.size = message->data.size,
                .data.type = ESP_ZB_ZCL_ATTR_TYPE_U32,
                .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
                .profile_id = 0x0104,
                .zcl_basic_cmd.dst_addr_u.addr_short = 0,
                .zcl_basic_cmd.dst_endpoint = message->info.dst_endpoint,
                .zcl_basic_cmd.src_endpoint = message->info.src_endpoint,
                .dis_default_resp = 1,
                .manuf_code = 0,
                .manuf_specific = 0, 
            };
            sequence_num = esp_zb_zcl_custom_cluster_cmd_req(&cmd_req); 
        }else{
            if(message->data.size == 3){
                memcpy(data_cmd_0x08_3, message->data.value, sizeof(data_cmd_0x08_3));
                for(int i=0; i<message->data.size; i++){
                    printf("0x%x ", data_cmd_0x08_3[i]);
                }
                printf("\n");  

                esp_zb_zcl_custom_cluster_cmd_req_t cmd_req = {
                    .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
                    .cluster_id = message->info.cluster,
                    .custom_cmd_id = 1,
                    .data.value = message->data.value,
                    .data.size = message->data.size,
                    .data.type = ESP_ZB_ZCL_ATTR_TYPE_U24,
                    .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
                    .profile_id = 0x0104,
                    .zcl_basic_cmd.dst_addr_u.addr_short = 0,
                    .zcl_basic_cmd.dst_endpoint = message->info.dst_endpoint,
                    .zcl_basic_cmd.src_endpoint = message->info.src_endpoint,
                    .dis_default_resp = 1,
                    .manuf_code = 0,
                    .manuf_specific = 0, 
                };
                sequence_num = esp_zb_zcl_custom_cluster_cmd_req(&cmd_req); 
            }else if(message->data.size == 5){
                memcpy(data_cmd_0x01, message->data.value, sizeof(data_cmd_0x01));
                for(int i=0; i<message->data.size; i++){
                    printf("0x%x ", data_cmd_0x01[i]);
                }
                printf("\n");  

                esp_zb_zcl_custom_cluster_cmd_req_t cmd_req = {
                    .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
                    .cluster_id = message->info.cluster,
                    .custom_cmd_id = 1,
                    .data.value = message->data.value,
                    .data.size = message->data.size,
                    .data.type = ESP_ZB_ZCL_ATTR_TYPE_U40,
                    .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
                    .profile_id = 0x0104,
                    .zcl_basic_cmd.dst_addr_u.addr_short = 0,
                    .zcl_basic_cmd.dst_endpoint = message->info.dst_endpoint,
                    .zcl_basic_cmd.src_endpoint = message->info.src_endpoint,
                    .dis_default_resp = 1,
                    .manuf_code = 0,
                    .manuf_specific = 0, 
                };
                sequence_num = esp_zb_zcl_custom_cluster_cmd_req(&cmd_req); 

            }  
        }        
    } else if(message->info.command.id == 0x10){
        if(message->data.size == 2){
            memcpy(data_cmd_0x10, message->data.value, sizeof(data_cmd_0x10));
            for(int i=0; i<message->data.size; i++){
                printf("0x%x ", data_cmd_0x10[i]);
            }
            printf("\n");  
            esp_zb_zcl_custom_cluster_cmd_req_t cmd_req = {
                .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
                .cluster_id = message->info.cluster,
                .custom_cmd_id = 1,
                .data.value = message->data.value,
                .data.size = message->data.size,
                .data.type = ESP_ZB_ZCL_ATTR_TYPE_U8,
                .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
                .profile_id = 0x0104,
                .zcl_basic_cmd.dst_addr_u.addr_short = 0,
                .zcl_basic_cmd.dst_endpoint = ENDPOINTS_LIST[0],
                .zcl_basic_cmd.src_endpoint = ENDPOINTS_LIST[0],
                .dis_default_resp = 1,
                .manuf_code = 0,
                .manuf_specific = 0, 
            };
            sequence_num = esp_zb_zcl_custom_cluster_cmd_req(&cmd_req);                        
        }

    }else if(message->info.command.id == 0x00){
        
        scene_counts = 0; 
        if(ep_cnts > TOTAL_ENDPOINTS) ep_cnts = 0;
 
        if (ep_cnts < TOTAL_ENDPOINTS && !is_value_present(ep_id, ep_cnts, 0)) {
            ep_id[0] = 0;
            ep_cnts++;
        }

        if(message->data.size == 7){  //set thermostat state   
            memcpy(thermostat_data.bytes, message->data.value, sizeof(thermostat_data.bytes));
            for(int i=0; i<message->data.size; i++){
                printf("0x%x ", thermostat_data.bytes[i]);
            }
            printf("\n");                  
            if(thermostat_data.bytes[2] == 3 && thermostat_data.bytes[3] == 4){

                #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN)
                device_info[FAN_INDEX].device_state = true;
                
                if(device_info[FAN_INDEX].fan_speed < thermostat_data.bytes[6]){
                    device_info[FAN_INDEX].fan_speed = thermostat_data.bytes[6];
                    nuos_zb_set_hardware(2, false);
                } else {
                    device_info[FAN_INDEX].fan_speed = thermostat_data.bytes[6];
                    nuos_zb_set_hardware(3, false);
                }
                #endif
            }else if(thermostat_data.bytes[2] == 1 && thermostat_data.bytes[3] == 1){  //set fan state
                #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN)
                device_info[FAN_INDEX].device_state = thermostat_data.bytes[6];
                nuos_zb_set_hardware(FAN_INDEX, false);
                #endif
                printf("ON OFF\n");
            }else if(thermostat_data.bytes[2] == 0x0f && thermostat_data.bytes[3] == 1){
                device_info[0].device_state = thermostat_data.bytes[6];
                printf("Fan Light\n");                
            }else if(thermostat_data.bytes[2] == 5 && thermostat_data.bytes[3] == 1){  //set light state
                #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN)
                device_info[LIGHT_INDEX].device_state = thermostat_data.bytes[6];
                nuos_zb_set_hardware(LIGHT_INDEX, false);
                #endif
                
            }
            esp_zb_zcl_custom_cluster_cmd_req_t cmd_req = {
                .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
                .cluster_id = message->info.cluster,
                .custom_cmd_id = 1,
                .data.value = thermostat_data.bytes,
                .data.size = message->data.size,
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
            sequence_num = esp_zb_zcl_custom_cluster_cmd_req(&cmd_req);          
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER)
                if(thermostat_data.bytes[0] == 0) nuos_set_thermostat_mode_attr(0, 0);
                else nuos_set_thermostat_mode_attr(0, 3);
            #endif
        }else if(message->data.size == 10){ // uint8_t arrays[10] = {0x00, 0x18, 0x18, 0x02, 0x00, 0x04, 0x00, 0x00, 0x00, 26};
            memcpy(thermostat_temp.bytes, message->data.value, sizeof(thermostat_temp.bytes));
            for(int i=0; i<message->data.size; i++){
                printf("0x%x ", thermostat_temp.bytes[i]);
            }
            printf("\n");  




            uint8_t payload[9];
            payload[0] = thermostat_temp.bytes[1];         // 1 byte sequence number
            payload[1] = thermostat_temp.bytes[2];            // DP ID (temperature setpoint)
            payload[2] = thermostat_temp.bytes[3];            // DP type (value)
            payload[3] = thermostat_temp.bytes[4];            // Function (set/report), often 0x00
            payload[4] = thermostat_temp.bytes[5];            // Data length (4 bytes)
            payload[5] = thermostat_temp.bytes[6];            // Value MSB (= 0)
            payload[6] = thermostat_temp.bytes[7];
            payload[7] = thermostat_temp.bytes[8];
            payload[8] = thermostat_temp.bytes[9];            // Value LSB (= 24)

            // Create octet string: first byte is length, then payload
            uint8_t octet_string[10];
            octet_string[0] = 9; // length byte
            memcpy(&octet_string[1], payload, 9);

            esp_zb_zcl_custom_cluster_cmd_req_t cmd_req = {
                .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
                .cluster_id = message->info.cluster,
                .custom_cmd_id = 1,
                .data.value = octet_string,
                .data.size = 10,
                .data.type = ESP_ZB_ZCL_ATTR_TYPE_OCTET_STRING,
                .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
                .profile_id = 0x0104,
                .zcl_basic_cmd.dst_addr_u.addr_short = 0,
                .zcl_basic_cmd.dst_endpoint = 1,
                .zcl_basic_cmd.src_endpoint = 1,
                .dis_default_resp = 1,
                .manuf_code = 0,
                .manuf_specific = 0, 
            };
            sequence_num = esp_zb_zcl_custom_cluster_cmd_req(&cmd_req);

            if(thermostat_temp.bytes[2] == 50 && thermostat_temp.bytes[3] == 2){ 
                #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER)
                printf("THERMOSTAT TEMP\n");
                device_info[0].device_val = thermostat_temp.bytes[9];
                nuos_set_thermostat_temp_attribute(0);
                #endif
            }else if(thermostat_temp.bytes[2] == 4 && thermostat_temp.bytes[3] == 2){ 
                printf("FAN SPEED\n");
                #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN)
                uint8_t f_speed = thermostat_temp.bytes[9]/25;
                if(device_info[FAN_INDEX].fan_speed < f_speed){
                    device_info[FAN_INDEX].fan_speed = f_speed;
                    nuos_zb_set_hardware(2, false);
                } else {
                    device_info[FAN_INDEX].fan_speed = f_speed;
                    nuos_zb_set_hardware(3, false);
                }   
                #endif             
            }   
        }   
    } else {
        printf("default\n");
    }
    #endif 
    return ESP_OK;
}



// static uint16_t last_cmd = 0;
extern bool is_running, was_paused, timer_stop_flag;
static uint8_t last_percentage_val = 255;

#define ADD_HOURS    9
#define ADD_MINUTES  30

time_t add_9h30m_to_time(time_t current_time)
{
    // Convert hours and minutes to seconds and add to current time
    time_t added_seconds = (ADD_HOURS * 3600) + (ADD_MINUTES * 60);
    return current_time + added_seconds;
}

// void send_window_covering_command(uint16_t dst_addr, uint8_t dst_ep, uint8_t src_ep, uint8_t command, void *payload)
// {
//     esp_zb_zcl_window_covering_cluster_send_cmd_req_t cmd_req = {0};
//     cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = dst_addr; // Coordinator
//     cmd_req.zcl_basic_cmd.dst_endpoint = dst_ep; // Coordinator's endpoint (often 1, but check your gateway)
//     cmd_req.zcl_basic_cmd.src_endpoint = src_ep; // Your device's endpoint
//     cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
//     cmd_req.cmd_id = command;
//     cmd_req.value = payload;
//     esp_zb_zcl_window_covering_cluster_send_cmd_req(&cmd_req);  
// }

static esp_err_t zb_cmd_window_covering_handler(const esp_zb_zcl_window_covering_movement_message_t *message){

    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "COMMAND(%d) CLUSTER(%d) DST_Ep(%d)\n", message->command, message->info.cluster, message->info.dst_endpoint);
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN_SWITCH)
        #ifdef TUYA_ATTRIBUTES
        if(message->command < 3){
            uint8_t cmd_val = message->command;
            if(message->command == 0) cmd_val = 1;
            else if(message->command == 1) cmd_val = 0;
            esp_zb_zcl_set_attribute_val(message->info.dst_endpoint,
                ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                0xF000 ,
                &cmd_val,
                false);      
            send_report(0, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, 0xF000);    
        }

        scene_counts = 0; 
        if(ep_cnts > TOTAL_ENDPOINTS) ep_cnts = 0;
        if (ep_cnts < TOTAL_ENDPOINTS && !is_value_present(ep_id, ep_cnts, 0)) {
            ep_id[0] = 0;
            ep_cnts++;
        }            
        switch (message->command) {
            case ESP_ZB_ZCL_CMD_WINDOW_COVERING_UP_OPEN:
                if(device_info[0].fan_speed != message->command){
                    device_info[0].fan_speed = message->command;  

                    ESP_LOGI(TAG, "Curtain OPEN : %d", device_info[0].fan_speed);
                    device_info[0].device_state = 0;   
                    device_info[1].device_state = 0; 
                    //nuos_report_curtain_blind_state(0, 100);                   
                    nuos_zb_set_hardware(0, false);
                }
                break;        
            case  ESP_ZB_ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE:
                if(device_info[0].fan_speed != message->command){
                    device_info[0].fan_speed = message->command;  
                    ESP_LOGI(TAG, "Curtain CLOSE : %d", device_info[0].fan_speed);
                    device_info[1].device_state = 0;
                    device_info[0].device_state = 0;    
                    //nuos_report_curtain_blind_state(0, 0);        
                    nuos_zb_set_hardware(1, false);
                }
                break;        
            case ESP_ZB_ZCL_CMD_WINDOW_COVERING_STOP:
                if(device_info[0].fan_speed != message->command){
                    device_info[0].fan_speed = message->command;                  
                    ESP_LOGI("WINDOW", "Curtain STOP\n");  
                    curtain_cmd_stop();                   
                    nuos_zb_set_hardware(2, false);
                }
                break;

            case ESP_ZB_ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_PERCENTAGE:  
                ESP_LOGI("WINDOW", "Go To Lift Percentage: %d%%\n", message->payload.percentage_lift_value);
                nuos_report_curtain_blind_state(0, message->payload.percentage_lift_value);    

                uint8_t state = curtain_cmd_goto_pct( message->payload.percentage_lift_value);
                if(state == 0) device_info[0].device_state = 1;
                else if(state == 1) device_info[1].device_state = 1;
                nuos_zb_set_hardware_curtain(0, state);                              
                break;
            default:
                break;
        }
        #endif
    #endif
    return ret;
}



static esp_err_t zb_cmd_read_attribute_handler(const esp_zb_zcl_cmd_read_attr_resp_message_t *message){

    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    #ifdef USE_CCT_TIME_SYNC
    if (message && message->variables) {
        esp_zb_zcl_read_attr_resp_variable_t *variable = message->variables;
        if(message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_TIME){  
            #ifdef USE_CCT_TIME_SYNC                        
            while (variable) {
                if (variable->attribute.id == ESP_ZB_ZCL_ATTR_TIME_LOCAL_TIME_ID || variable->attribute.id == ESP_ZB_ZCL_ATTR_TIME_STANDARD_TIME_ID) { // Assuming 0x0000 is the Time attribute ID
                    printf("On Attribute ZB_ZCL_ATTR_TIME_LOCAL_TIME_ID Callback\n");
                    if (variable->attribute.data.value) {
                        #ifdef USE_HOME_ASSISTANT
                        time_t t = *(uint32_t *)variable->attribute.data.value + 946684800;
                        #else
                        uint32_t tt = *(uint32_t *)variable->attribute.data.value;
                        time_t usa_t = tt + 946684800;
                        #endif
                        printf("Converted time: %s", ctime(&usa_t));

                        time_t new_time = add_9h30m_to_time(usa_t);
                        struct tm *time_info = localtime(&new_time);
                        printf("Updated Time: %s", asctime(time_info));
                    
                        printf("Time=> %d:%d:%d\n", time_info->tm_hour, time_info->tm_min, time_info->tm_sec);
                        // Extract hour, minute, and second
                        int hour = time_info->tm_hour;
                        int minute = time_info->tm_min;
                        int second = time_info->tm_sec;

                        int year = time_info->tm_year+1900;
                        int month = time_info->tm_mon;
                        int mday = time_info->tm_mday;
                        int ist_hour = hour;

                        // Print the extracted values
                        printf("Year: %d\n", year);
                        printf("Month: %d\n", month);
                        printf("Day: %d\n", mday);                            
                        printf("Hour: %d\n", ist_hour);
                        printf("Minute: %d\n", minute);
                        printf("Second: %d\n", second);
    
                        set_time(year, month, mday, ist_hour, minute, second);
   
                        set_cct_time_sync_request_flag = true;
                    } else {
                        printf("Invalid attribute value (NULL)\n");
                    }
                }
                variable = variable->next;
            }
            #endif
        }
    } else {
        printf("Invalid message or empty attribute response\n");
    }
    #else
        set_remote_scene_devices_attribute_values((void*)message);
    #endif
    return ret;                    

}



static esp_err_t zb_configure_report_resp_handler(const esp_zb_zcl_cmd_config_report_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    if(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS){
        esp_zb_zcl_config_report_resp_variable_t *variable = message->variables;
        if(variable == NULL){
            ESP_LOGI(TAG, "Configure report response: status(%d), cluster(0x%x)", message->info.status, message->info.cluster);
        }else{
            ESP_LOGI(TAG, "Configure report response: status(%d), cluster(0x%x), attribute(0x%x)", message->info.status, message->info.cluster,
                 variable->attribute_id);  
                 #ifdef USE_CCT_TIME_SYNC 
                 if(variable->attribute_id == 0x07){
                    read_zigbee_time(); 
                 }
                 #endif
            }             
    }
    return ESP_OK;
}

static esp_err_t zb_attribute_reporting_handler(const esp_zb_zcl_report_attr_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->status);
    ESP_LOGI(TAG, "Received report from address(0x%x) src endpoint(%d) to dst endpoint(%d) cluster(0x%x)", message->src_address.u.short_addr,
             message->src_endpoint, message->dst_endpoint, message->cluster);

    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH  || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH) 
    #else
    for(int i=0; i<TOTAL_ENDPOINTS; i++){
        if(ENDPOINTS_LIST[i] == message->dst_endpoint){
            nuos_set_zigbee_attribute(i);   
        }
    }
    #endif
    return ESP_OK;
}


#ifdef USE_OTA

static esp_err_t esp_element_ota_data(uint32_t total_size, const void *payload, uint16_t payload_size, void **outbuf, uint16_t *outlen)
{
    static uint16_t tagid = 0;
    void *data_buf = NULL;
    uint16_t data_len;

    if (!s_tagid_received) {
        uint32_t length = 0;
        if (!payload || payload_size <= OTA_ELEMENT_HEADER_LEN) {
            ESP_RETURN_ON_ERROR(ESP_ERR_INVALID_ARG, TAG, "Invalid element format");
        }

        tagid  = *(const uint16_t *)payload;
        length = *(const uint32_t *)(payload + sizeof(tagid));
        if ((length + OTA_ELEMENT_HEADER_LEN) != total_size) {
            ESP_RETURN_ON_ERROR(ESP_ERR_INVALID_ARG, TAG, "Invalid element length [%ld/%ld]", length, total_size);
        }

        s_tagid_received = true;

        data_buf = (void *)(payload + OTA_ELEMENT_HEADER_LEN);
        data_len = payload_size - OTA_ELEMENT_HEADER_LEN;
    } else {
        data_buf = (void *)payload;
        data_len = payload_size;
    }

    switch (tagid) {
        case UPGRADE_IMAGE:
            *outbuf = data_buf;
            *outlen = data_len;
            break;
        default:
            ESP_RETURN_ON_ERROR(ESP_ERR_INVALID_ARG, TAG, "Unsupported element tag identifier %d", tagid);
            break;
    }

    return ESP_OK;
}

static esp_err_t zb_ota_upgrade_status_handler(esp_zb_zcl_ota_upgrade_value_message_t message)
{
    static uint32_t total_size = 0;
    static uint32_t offset = 0;
    static int64_t start_time = 0;
    esp_err_t ret = ESP_OK;

    if (message.info.status == ESP_ZB_ZCL_STATUS_SUCCESS) {
        switch (message.upgrade_status) {
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_START:
            ESP_LOGI(TAG, "-- OTA upgrade start");
            start_time = esp_timer_get_time();
            s_ota_partition = esp_ota_get_next_update_partition(NULL);
            assert(s_ota_partition);
            #if CONFIG_ZB_DELTA_OTA
                ret = esp_delta_ota_begin(s_ota_partition, 0, &s_ota_handle);
            #else
                ret = esp_ota_begin(s_ota_partition, 0, &s_ota_handle);
            #endif
            ESP_RETURN_ON_ERROR(ret, TAG, "Failed to begin OTA partition, status: %s", esp_err_to_name(ret));
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_RECEIVE:
            total_size = message.ota_header.image_size;
            offset += message.payload_size;
            ESP_LOGI(TAG, "-- OTA Client receives data: progress [%ld/%ld]", offset, total_size);
            if (message.payload_size && message.payload) {
                uint16_t payload_size = 0;
                void    *payload = NULL;
                ret = esp_element_ota_data(total_size, message.payload, message.payload_size, &payload, &payload_size);
                ESP_RETURN_ON_ERROR(ret, TAG, "Failed to element OTA data, status: %s", esp_err_to_name(ret));
#if CONFIG_ZB_DELTA_OTA
                ret = esp_delta_ota_write(s_ota_handle, payload, payload_size);
#else
                ret = esp_ota_write(s_ota_handle, (const void *)payload, payload_size);
#endif
                ESP_RETURN_ON_ERROR(ret, TAG, "Failed to write OTA data to partition, status: %s", esp_err_to_name(ret));
            }
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_APPLY:
            ESP_LOGI(TAG, "-- OTA upgrade apply");
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_CHECK:
            ret = offset == total_size ? ESP_OK : ESP_FAIL;
            offset = 0;
            total_size = 0;
            s_tagid_received = false;
            ESP_LOGI(TAG, "-- OTA upgrade check status: %s", esp_err_to_name(ret));
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_FINISH:
            ESP_LOGI(TAG, "-- OTA Finish");
            ESP_LOGI(TAG, "-- OTA Information: version: 0x%lx, manufacturer code: 0x%x, image type: 0x%x, total size: %ld bytes, cost time: %lld ms,",
                     message.ota_header.file_version, message.ota_header.manufacturer_code, message.ota_header.image_type,
                     message.ota_header.image_size, (esp_timer_get_time() - start_time) / 1000);
#if CONFIG_ZB_DELTA_OTA
            ret = esp_delta_ota_end(s_ota_handle);
#else
            ret = esp_ota_end(s_ota_handle);
#endif
            ESP_RETURN_ON_ERROR(ret, TAG, "Failed to end OTA partition, status: %s", esp_err_to_name(ret));
            ret = esp_ota_set_boot_partition(s_ota_partition);
            ESP_RETURN_ON_ERROR(ret, TAG, "Failed to set OTA boot partition, status: %s", esp_err_to_name(ret));
            ESP_LOGW(TAG, "Prepare to restart system");
            esp_restart();
            break;
        default:
            ESP_LOGI(TAG, "OTA status: %d", message.upgrade_status);
            break;
        }
    }
    return ret;
}

static esp_err_t zb_ota_upgrade_query_image_resp_handler(esp_zb_zcl_ota_upgrade_query_image_resp_message_t message)
{
    esp_err_t ret = ESP_OK;
    if (message.info.status == ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Queried OTA image from address: 0x%04hx, endpoint: %d", message.server_addr.u.short_addr, message.server_endpoint);
        ESP_LOGI(TAG, "Image version: 0x%lx, manufacturer code: 0x%x, image size: %ld", message.file_version, message.manufacturer_code,
                 message.image_size);
    }
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Approving OTA image upgrade");
    } else {
        ESP_LOGI(TAG, "Rejecting OTA image upgrade, status: %s", esp_err_to_name(ret));
    }
    return ret;
}
#endif

static bool response_pending = false;
// Global variable to store the response data
static esp_zb_zcl_custom_cluster_cmd_req_t g_cmd_req;

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
        case ESP_ZB_CORE_CMD_WRITE_ATTR_RESP_CB_ID:
            ret = zb_write_attribute_handler((esp_zb_zcl_cmd_write_attr_resp_message_t *)message);
        break;    
        case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
            ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
            break;
        case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID:
            ret = zb_cmd_default_handler((esp_zb_zcl_cmd_default_resp_message_t *)message);
            break; 
        case ESP_ZB_CORE_BASIC_RESET_TO_FACTORY_RESET_CB_ID:
            ret = zb_basic_reset_factory_handler((esp_zb_zcl_basic_reset_factory_default_message_t *)message); 
        case ESP_ZB_CORE_IAS_ZONE_ENROLL_RESPONSE_VALUE_CB_ID:
            //Added by Nuos 
            ret = zb_cmd_ias_zone_handler((esp_zb_zcl_ias_zone_enroll_response_message_t *)message);
            break; 
        case ESP_ZB_CORE_IDENTIFY_EFFECT_CB_ID:
            //Added by Nuos 
            ret = zb_cmd_identify_effect_handler((esp_zb_zcl_identify_effect_message_t *)message);
            break; 
        case ESP_ZB_CORE_CMD_PRIVILEGE_COMMAND_REQ_CB_ID:
           // ret = zb_cmd_privilege_command_handler((esp_zb_zcl_privilege_command_message_t *)message);
            break;  
        case ESP_ZB_CORE_CMD_REPORT_CONFIG_RESP_CB_ID:
            ret = zb_configure_report_resp_handler((esp_zb_zcl_cmd_config_report_resp_message_t *)message);   
            break; 
        case ESP_ZB_CORE_CMD_CUSTOM_CLUSTER_REQ_CB_ID:
            ret = zb_cmd_custom_cluster_handler((esp_zb_zcl_custom_cluster_command_message_t *)message);
            break;    
        case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID:
            ret = zb_cmd_read_attribute_handler((esp_zb_zcl_cmd_read_attr_resp_message_t *)message);
            break;
        case ESP_ZB_CORE_WINDOW_COVERING_MOVEMENT_CB_ID:
            ret = zb_cmd_window_covering_handler((esp_zb_zcl_window_covering_movement_message_t *)message);
            break;
        case ESP_ZB_CORE_REPORT_ATTR_CB_ID:
            ret = zb_attribute_reporting_handler((esp_zb_zcl_report_attr_message_t *)message);
            break; 
        #ifdef USE_OTA
        case ESP_ZB_CORE_OTA_UPGRADE_VALUE_CB_ID:
            ret = zb_ota_upgrade_status_handler(*(esp_zb_zcl_ota_upgrade_value_message_t *)message);
            break;
        case ESP_ZB_CORE_OTA_UPGRADE_QUERY_IMAGE_RESP_CB_ID:
            ret = zb_ota_upgrade_query_image_resp_handler(*(esp_zb_zcl_ota_upgrade_query_image_resp_message_t *)message);
            break;
        #endif 
        default:
            //Added by Nuos 
            nuos_zigbee_action_handler(callback_id, message);
            if (response_pending) {
                esp_zb_zcl_custom_cluster_cmd_req(&g_cmd_req);
                response_pending = false;
            }
            break;
    }
    return ret;
}

#if(CHIP_INFO == USE_ESP32H2_MINI1_V5 || CHIP_INFO == USE_ESP32C6_MINI1_V5)    
    uint16_t send_live_request_count = 0; 
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_GROUP_SWITCH)
        static void ieee_cb_2(esp_zb_zdp_status_t zdo_status, esp_zb_zdo_ieee_addr_rsp_t *resp, void *user_ctx) {
            if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
                // ESP_LOGI(TAG, "IEEE address: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", resp->ieee_addr[7], resp->ieee_addr[6],
                //         resp->ieee_addr[5], resp->ieee_addr[4], resp->ieee_addr[3], resp->ieee_addr[2], resp->ieee_addr[1],
                //         resp->ieee_addr[0]);
            }
        }
    #endif

#endif


#define TOUCHLINK_TARGET_TIMEOUT                    60 /* The timeout for the Touchlink target, measured in seconds */

/*
#define TUYA_CLUSTER_ID 				0xEF00  // Most Tuya devices use this custom cluster

bool zb_apsde_data_indication_handler(esp_zb_apsde_data_ind_t ind)
{
    bool processed 				= false;
	static uint16_t last_src 	= 0;
	static uint8_t last_seq 	= 0xFF;

	if (ind.src_short_addr == last_src && ind.asdu[1] == last_seq) {
		// Duplicate, ignore
		return false;
	}
	last_src = ind.src_short_addr;
	last_seq = ind.asdu[1];

    if (ind.status != 0x00) {
        ESP_LOGE("APSDE INDICATION", "Invalid status of APSDE-DATA indication, error code: %d", ind.status);
        return false;
    }

    // Only process messages for endpoint 1, Home Automation profile, and On/Off or Tuya clusters
    if (ind.dst_endpoint == 0 || ind.profile_id != ESP_ZB_AF_HA_PROFILE_ID ||
        (ind.cluster_id != TUYA_CLUSTER_ID)) {
        ESP_LOGW("APSDE INDICATION", "Invalid data!!"); 
        return false;
    }
    // Print basic info
    ESP_LOGI("APSDE INDICATION",
             "Received APSDE-DATA %s request with a length of %lu from endpoint %d, source address 0x%04hx to "
             "endpoint %d, destination address 0x%04hx",
             ind.dst_addr_mode == 0x01 ? "group" : "unicast", ind.asdu_length, ind.src_endpoint,
             ind.src_short_addr, ind.dst_endpoint, ind.dst_short_addr);
    ESP_LOG_BUFFER_HEX_LEVEL("APSDE INDICATION", ind.asdu, ind.asdu_length, ESP_LOG_INFO);

    // Check payload length for ZCL header
    if (ind.asdu_length < 3) {
        ESP_LOGW("APSDE INDICATION", "ASDU too short for ZCL command");
        return false;
    }

    uint8_t frame_control 	= ind.asdu[0];
    uint8_t seq_number   	= ind.asdu[1];
    uint8_t command_id   	= ind.asdu[2];
	uint8_t data 			= ind.asdu[3];
    // Tuya devices: command_id is typically 0x00 (set data), 0x01 (get data), 0x02 (report), 0x03 (status report), etc.
    switch (command_id) {
        case 0x00: // Tuya set data request
            ESP_LOGI("APSDE INDICATION", "Tuya Set Data Request (0x00) received");
            // Parse and handle Tuya set data here
            processed = true;
            break;
        case 0x01: // Tuya get data request
            ESP_LOGI("APSDE INDICATION", "Tuya Get Data Request (0x01) received");
            // Parse and handle Tuya get data here
            processed = true;
            break;
        case 0x02: // Tuya report data
            ESP_LOGI("APSDE INDICATION", "Tuya Report Data (0x02) received");
            // Parse and handle Tuya report data here
            processed = true;
            break;
        case 0x03: // Tuya status report
            ESP_LOGI("APSDE INDICATION", "Tuya Status Report (0x03) received");
            // Parse and handle Tuya status report here
            processed = true;
            break;
        case 0x04: // Tuya command response
            ESP_LOGI("APSDE INDICATION", "Tuya Command Response (0x04) received");
            // Parse and handle Tuya command response here
            processed = true;
            break;
        case 0x05: // Tuya heartbeat
            ESP_LOGI("APSDE INDICATION", "Tuya Heartbeat (0x05) received");
            // Parse and handle Tuya heartbeat here
            processed = true;
            break;
        case 0x06: // Tuya device info
            ESP_LOGI("APSDE INDICATION", "Tuya Device Info (0x06) received");
            // Parse and handle Tuya device info here
            processed = true;
            break;
        case 0xfd: // Tuya wireless scene switch
            ESP_LOGI("APSDE INDICATION", "Tuya Device Info (0xfd) received");
            // Parse and handle Tuya device info here
            processed = true;
			if(ind.asdu_length == 4){
				switch(data){
					case 0:  //single press
					printf("SINGLE KEY_%d PRESSED\n!!", ind.src_endpoint);
					break;
					case 1:  //double press
					printf("DOUBLE KEY_%d PRESS\n!!", ind.src_endpoint);
					break;
					case 2:  //long press
					printf("LONG KEY_%d PRESS\n!!", ind.src_endpoint);
					break;	
					default:
					break;									
				}
			}
            break;			
        // Add more cases as needed for your Tuya devices
        default:
            ESP_LOGW("APSDE INDICATION", "Unknown Tuya/cluster command ID: 0x%02X", command_id);
            // Optionally print payload for debugging
            ESP_LOG_BUFFER_HEX_LEVEL("APSDE INDICATION", ind.asdu, ind.asdu_length, ESP_LOG_INFO);
            break;
    }
// Prepare the response data but don't send it here
            uint8_t payload[ind.asdu_length-1];
            for(int i=0; i<ind.asdu_length-1; i++){
                payload[i] = ind.asdu[i+1];
            }
            
            uint8_t octet_string[ind.asdu_length];
            octet_string[0] = ind.asdu_length-1;
            memcpy(&octet_string[1], payload, ind.asdu_length-1);
            
            // Store the response data in a global variable
            g_cmd_req = (esp_zb_zcl_custom_cluster_cmd_req_t){
                .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
                .cluster_id = TUYA_CLUSTER_ID,
                .custom_cmd_id = 1,
                .data.value = octet_string,
                .data.size = ind.asdu_length,
                .data.type = ESP_ZB_ZCL_ATTR_TYPE_OCTET_STRING,
                .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
                .profile_id = 0x0104,
                .zcl_basic_cmd.dst_addr_u.addr_short = ind.src_short_addr, // Respond to the source
                .zcl_basic_cmd.dst_endpoint = ind.src_endpoint, // Respond to the source endpoint
                .zcl_basic_cmd.src_endpoint = ind.dst_endpoint, // Use the destination endpoint as source
                .dis_default_resp = 1,
                .manuf_code = 0,
                .manuf_specific = 0, 
            };
            response_pending = true;
    return processed;
}*/



static void esp_zb_task(void *pvParameters)
{
    //  ESP_ERROR_CHECK(esp_zb_io_buffer_size_set(64)); 
    // esp_zb_scheduler_queue_size_set(150); 
    printf("---------esp_zb_task---------\n");
    const uint16_t table_size = 32;
    // esp_zb_zcl_scenes_table_init(table_size);
    ESP_ERROR_CHECK(esp_zb_aps_src_binding_table_size_set(table_size));
    // ESP_ERROR_CHECK(esp_zb_aps_dst_binding_table_size_set(table_size));
    // ESP_ERROR_CHECK(esp_zb_overall_network_size_set(32)); // Coordinator + 31 devices

    /* initialize Zigbee stack */
    #ifdef IS_END_DEVICE
        esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    #else
        esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZR_CONFIG();
    #endif
    printf("---------esp_zb_task End---------\n"); 
    esp_zb_init(&zb_nwk_cfg);
    ESP_ERROR_CHECK(esp_zb_aps_src_binding_table_size_set(table_size));
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM    \
        || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_ANALOG_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4R_ON_OFF_LIGHT                              \
        || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2R_ON_OFF_LIGHT)   
        // esp_zb_set_channel_mask(ESP_ZB_TOUCHLINK_CHANNEL_MASK);
        // esp_zb_set_rx_on_when_idle(true);
        // esp_zb_zdo_touchlink_target_set_timeout(TOUCHLINK_TARGET_TIMEOUT);
    #endif
    // Wait for stack to be ready before starting network steering
    //Added by Nuos 
    esp_zb_device_register(nuos_init_zb_clusters());
    nuos_init_privilege_commands(); 
    // esp_zb_aps_data_indication_handler_register(zb_apsde_data_indication_handler); 
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);

    #ifdef IS_END_DEVICE
        #ifdef IS_BATTERY_POWERED
            esp_zb_set_rx_on_when_idle(false);
        #endif
    #endif
    #ifdef ZB_FACTORY_RESET
        //esp_zb_nvram_erase_at_start(true);
        nvs_flash_erase();
        esp_zb_factory_reset();
    #endif   
    ESP_ERROR_CHECK(esp_zb_start(false));  
    esp_zb_stack_main_loop();   
}

void app_main(void) {
    //Init GPIOs pin
    nuos_zb_init_hardware();
    #if(CHIP_INFO == USE_ESP32H2_MINI1_V5 || CHIP_INFO == USE_ESP32C6_MINI1_V5)
        gpio_reset_pin(GPIO_NUM_9);
        gpio_set_direction(GPIO_NUM_9, GPIO_MODE_OUTPUT);
        gpio_set_level(GPIO_NUM_9, true);
    #endif     
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG()
    };
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else if (ret != ESP_OK) {
        ESP_ERROR_CHECK(ret);
    } 
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    #ifdef USE_WIFI_WEBSERVER
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());
    #endif
    //Added by NUOS
    if(nuos_init_sequence()){
        #ifdef DONT_USE_ZIGBEE
        ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "Failed" : "Successful");
        #else
        xTaskCreate(esp_zb_task, "Zigbee_main", TASK_STACK_SIZE_ZIGBEE, NULL, TASK_PRIORITY_ZIGBEE, NULL);
        #endif
    }else{
        ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "Failed" : "Successful");
    }
}
