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
#include "app_constants.h"
#include "esp_zigbee_core.h"
#include "esp_zb_controller_main.h"
#include "switch_driver.h"
#include "app_hardware_driver.h"
#include "app_zigbee_clusters.h"
#include "app_nuos_timer.h"
#include "zcl/esp_zigbee_zcl_power_config.h"
#include "esp_wifi_webserver.h"
#include "app_zigbee_group_commands.h"
#include "app_zigbee_scene_commands.h"
#include "app_zigbee_query_nodes.h"
#ifdef USE_OTA
#include "esp_ota_ops.h"
#include "esp_timer.h"
#endif
#include "esp_check.h"
#include "esp_log.h"
#include "esp_err.h"

static const char *TAG                          = "ZB_MISC";

//////////////////////////////////

static uint8_t identify_src_sp                  = 3;
uint8_t button_index                            = 0; 

// Function to print IEEE address
void print_ieee_addr(const esp_zb_ieee_addr_t addr) {
    // Ensure it's a valid pointer
    if (!addr) {
        ESP_LOGE(TAG, "Invalid IEEE address pointer");
        return;
    }
    
    // Print each byte of the IEEE address
    char ieee_addr_str[3 * 8];
    for (int i = 0; i < 8; ++i) {
        sprintf(&ieee_addr_str[i * 3], "%02X:", addr[i]);
    }
    // Remove the trailing colon
    ieee_addr_str[3 * 8 - 1] = '\0';

    //ESP_LOGI(TAG, "IEEE Address: %s", ieee_addr_str);
}


void query_identify_command(uint16_t dst_addr, uint8_t dst_endpoint){
    esp_zb_zcl_identify_query_cmd_t query_req;
    esp_zb_ieee_addr_t ieee_addr;
    esp_zb_ieee_address_by_short(dst_addr, ieee_addr);
    esp_zb_get_long_address(query_req.zcl_basic_cmd.dst_addr_u.addr_long);
    query_req.zcl_basic_cmd.dst_addr_u.addr_short = dst_addr;
    query_req.zcl_basic_cmd.dst_endpoint = dst_endpoint;
    query_req.zcl_basic_cmd.src_endpoint = dst_endpoint;
    memcpy(query_req.zcl_basic_cmd.dst_addr_u.addr_long, ieee_addr, sizeof(esp_zb_ieee_addr_t));
    query_req.address_mode = ESP_ZB_APS_ADDR_MODE_64_ENDP_PRESENT;
    uint8_t seq_no = esp_zb_zcl_identify_query_cmd_req(&query_req);
    ESP_LOGI(TAG, "Query Identify Sequence:%d dst_addr:0x%x", seq_no, dst_addr);
}
void send_identify_trigger_effect_command(uint16_t dst_addr, uint8_t dst_endpoint){
    // query_identify_command(dst_addr, dst_endpoint);
    esp_zb_zcl_identify_trigger_effect_cmd_t cmd_req;
    esp_zb_ieee_addr_t ieee_addr;
    esp_zb_ieee_address_by_short(dst_addr, ieee_addr);
    esp_zb_get_long_address(cmd_req.zcl_basic_cmd.dst_addr_u.addr_long);

    cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_64_ENDP_PRESENT;
    cmd_req.effect_id = ESP_ZB_ZCL_IDENTIFY_EFFECT_ID_BLINK;
    cmd_req.effect_variant = 0;
    cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = dst_addr;
    memcpy(cmd_req.zcl_basic_cmd.dst_addr_u.addr_long, ieee_addr, sizeof(esp_zb_ieee_addr_t));
    print_ieee_addr(ieee_addr);
    cmd_req.zcl_basic_cmd.dst_endpoint = dst_endpoint;
    cmd_req.zcl_basic_cmd.src_endpoint = dst_endpoint;
    esp_zb_zcl_identify_trigger_effect_cmd_req(&cmd_req);
}

void send_identify_command(uint16_t dst_addr, uint8_t src_endpoint, uint8_t dst_endpoint, uint16_t identify_time) {
/*
Blink (0): The light turns on and off once.
Breathe (1): The light turns on and off over 1 second, repeated 15 times.
Okay (2): For colored lights, it turns green for 1 second; non-colored lights flash twice. This effect can indicate a successful request to the device, like confirming a successful binding.
Channel Change (11): Colored lights turn orange for 8 seconds; non-colored lights go to maximum brightness for 0.5 seconds and then to minimum brightness for 7.5 seconds.

The `effectid` parameter can include two special values:
254 (0xfe): This value ends the effect after the current breathe cycle.
255 (0xff): This value stops the effect as soon as possible.
*/
    esp_zb_zcl_identify_cmd_t cmd_req;
    esp_zb_ieee_addr_t ieee_addr;
    esp_zb_ieee_address_by_short(dst_addr, ieee_addr);
    esp_zb_get_long_address(cmd_req.zcl_basic_cmd.dst_addr_u.addr_long);

    cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_64_ENDP_PRESENT;
    cmd_req.identify_time = identify_time;
    cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = dst_addr;
    memcpy(cmd_req.zcl_basic_cmd.dst_addr_u.addr_long, ieee_addr, sizeof(esp_zb_ieee_addr_t));
    // print_ieee_addr(ieee_addr);
    cmd_req.zcl_basic_cmd.dst_endpoint = dst_endpoint;
    cmd_req.zcl_basic_cmd.src_endpoint = src_endpoint;
    uint8_t ret = esp_zb_zcl_identify_cmd_req(&cmd_req);
    ESP_LOGI(TAG, "Identify command sent to address:0x%04x, endpoint:%d with sequence:%d", dst_addr, dst_endpoint, ret);

}

esp_zb_zdo_bind_req_param_t bind_req[5];
SemaphoreHandle_t bind_sem = NULL;
uint8_t bind_retry_counts = 0;
static void bind_cb_2(esp_zb_zdp_status_t zdo_status, void *user_ctx)
{
    if (user_ctx) {
        esp_zb_zdo_bind_req_param_t *bind_req = (esp_zb_zdo_bind_req_param_t*)user_ctx;
    }
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Node Bound successfully!");        
        if (user_ctx) {
            //esp_zb_zdo_bind_req_param_t *bind_req = (esp_zb_zdo_bind_req_param_t*)user_ctx;
            printf("========BINDING DATA==========\n");
            printf("src_ep: %u\n", bind_req->src_endp);
            printf("dst_short_addr: 0x%x  dst_ep: %u  cluster_id: %u\n", bind_req->dst_address_u.addr_short, bind_req->dst_endp, bind_req->cluster_id);
            printf("==============================\n");

            /* configure report attribute command */
            esp_zb_zcl_config_report_cmd_t report_cmd;
            bool report_change = 0;


            esp_zb_zcl_config_report_record_t records[2];

            report_cmd.zcl_basic_cmd.dst_addr_u.addr_short = bind_req->dst_address_u.addr_short;
            report_cmd.zcl_basic_cmd.dst_endpoint = bind_req->dst_endp;
            report_cmd.zcl_basic_cmd.src_endpoint = bind_req->src_endp;
            report_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
            report_cmd.clusterID = bind_req->cluster_id;
            /////////****************************************** *//////////
            esp_zb_zcl_read_attr_cmd_t read_req;
            read_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
            read_req.zcl_basic_cmd.src_endpoint = bind_req->src_endp;
            read_req.zcl_basic_cmd.dst_endpoint = bind_req->dst_endp;
            read_req.zcl_basic_cmd.dst_addr_u.addr_short = bind_req->dst_address_u.addr_short;
            read_req.clusterID = bind_req->cluster_id;

            if(bind_req->cluster_id == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF){
                uint16_t attributes1[] = {ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID};
                read_req.attr_number = 1;//ARRAY_LENTH(attributes1);
                read_req.attr_field = attributes1;       

            }else if(bind_req->cluster_id == ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL){
                uint16_t attributes2[] = {ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID};
                read_req.attr_number = 1;//ARRAY_LENTH(attributes2);
                read_req.attr_field = attributes2;
            }else if(bind_req->cluster_id == ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL){
                uint16_t attributes3[] = {ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID};
                read_req.attr_number = 1;//ARRAY_LENTH(attributes3);
                read_req.attr_field = attributes3;
                // esp_zb_zcl_config_report_record_t records3[] = {
                // {ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID, ESP_ZB_ZCL_ATTR_TYPE_U16, 0, 30, &report_change}};     
                esp_zb_zcl_config_report_record_t records3[] = {
                    {
                        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
                        .attributeID = ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID,
                        .attrType = ESP_ZB_ZCL_ATTR_TYPE_U16,
                        .min_interval = 0,
                        .max_interval = 30,
                        .reportable_change = &report_change
                    },
                };                           
                memcpy(records, records3, sizeof(esp_zb_zcl_config_report_record_t));
            }else if(bind_req->cluster_id == ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL){
                uint16_t attributes4[] = {ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID};
                read_req.attr_number = 1;//ARRAY_LENTH(attributes4);
                read_req.attr_field = attributes4;

                // esp_zb_zcl_config_report_record_t records4[] = {
                // {ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV, ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID, ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM, 0, 30, &report_change}};
                esp_zb_zcl_config_report_record_t records4[] = {
                    {
                        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
                        .attributeID = ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID,
                        .attrType = ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM,
                        .min_interval = 0,
                        .max_interval = 30,
                        .reportable_change = &report_change
                    },
                };                
                memcpy(records, records4, sizeof(esp_zb_zcl_config_report_record_t));  
            }else if(bind_req->cluster_id == ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT){
                uint16_t attributes5[] = {ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID, ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID};
                read_req.attr_number = 2;//ARRAY_LENTH(attributes5);
                read_req.attr_field = attributes5;

                // esp_zb_zcl_config_report_record_t records5[] = {
                // {ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV, ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID, ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM, 0, 30, &report_change},
                // {ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV, ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID, ESP_ZB_ZCL_ATTR_TYPE_S16, 0, 30, &report_change}};  
                esp_zb_zcl_config_report_record_t records5[] = {
                    {
                        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
                        .attributeID = ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID,
                        .attrType = ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM,
                        .min_interval = 0,
                        .max_interval = 30,
                        .reportable_change = &report_change
                    },
                    {
                        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
                        .attributeID = ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID,
                        .attrType = ESP_ZB_ZCL_ATTR_TYPE_S16,
                        .min_interval = 0,
                        .max_interval = 30,
                        .reportable_change = &report_change
                    },                    
                };                 
                memcpy(records, records5, sizeof(esp_zb_zcl_config_report_record_t)*2);                  
            }

            // if(bind_req->cluster_id == ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT || bind_req->cluster_id == ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL || bind_req->cluster_id == ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL){
            //     report_cmd.record_number = sizeof(records) / sizeof(esp_zb_zcl_config_report_record_t);
            //     report_cmd.record_field = records;
            //     esp_zb_zcl_config_report_cmd_req(&report_cmd);
            // } 
            cb_requests_counts++;
            esp_zb_zcl_read_attr_cmd_req(&read_req);
            
            
        }

        if(bind_sem != NULL)
        xSemaphoreGive(bind_sem);        
    }else {
        ESP_LOGE(TAG, "Binding failed with status: 0x%x", zdo_status);
        if(bind_retry_counts++ > 3){
            bind_retry_counts = 0;
            if(bind_sem != NULL)
            xSemaphoreGive(bind_sem);               
        }else{    
            ESP_LOGI(TAG, "Retry to bind Light\n");
            //esp_zb_zdo_device_bind_req(&bind_req, bind_cb_2, bind_req);    
        }  
    }

}

static void unbind_cb_2(esp_zb_zdp_status_t zdo_status, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Node UnBound successfully!");        
        if (user_ctx) {


            // esp_zb_zdo_bind_req_param_t *unbind_req = (esp_zb_zdo_bind_req_param_t*)user_ctx;
                // Now you can access the members of bind_req
            // printf("Source address: %llx\n", unbind_req->src_address); // Assuming IEEE address is 64-bit
            // printf("Source endpoint: %u\n", unbind_req->src_endp);
            // printf("Cluster ID: %u\n", unbind_req->cluster_id);
            // printf("Destination address mode: %u\n", unbind_req->dst_addr_mode);
            // printf("Request destination address: %u\n", unbind_req->req_dst_addr);
            // Access other fields as needed

        }

        cb_response_counts++;
        //if(cb_response_counts >= cb_requests_counts){
           // xSemaphoreGive(postHandlerSemaphore);
           //xTaskNotifyGive(query_neighbour_task_handle);
        //}

    }
}


// void nuos_read_attr_read_request(uint8_t btn_index) {
//     memset(bind_req, 0, sizeof(bind_req));  // Clear all elements to zero
//     esp_zb_ieee_addr_t src_ieee_addr;

//     for(int node_index=0; node_index<existing_nodes_info[btn_index].scene_switch_info.total_records; node_index++){
//         dst_node_info_t *dst_node_info = existing_nodes_info[btn_index].scene_switch_info.dst_node_info[node_index];

//         if(dst_node_info->dst_ep_info.clusters_count > 0 && dst_node_info->dst_ep_info.clusters_count < 5){
//             for(int cluster_index=0; cluster_index<dst_node_info->dst_ep_info.clusters_count; cluster_index++){
//                 if(dst_node_info->dst_ep_info.ep_data[ep_index].is_bind) {
//                     ESP_LOGI(TAG, "Already bind Light\n");
//                     printf("========SEND CMD READ ATTR DATA==========\n");
//                     printf("src_ep: %u dst_ep: %u\n", bind_req[cluster_index].src_endp, dst_node_info->dst_ep_info.ep_data[ep_index].dst_ep);
//                     printf("dst_short_addr: 0x%x cluster_id: %x\n", dst_node_info->short_addr, dst_node_info->dst_ep_info.cluster_id[cluster_index]);
//                     printf("==============================\n");

//                     esp_zb_zcl_read_attr_cmd_t read_req;
//                     read_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
//                     read_req.zcl_basic_cmd.src_endpoint = ENDPOINTS_LIST[btn_index];
//                     read_req.zcl_basic_cmd.dst_endpoint = dst_node_info->dst_ep_info.ep_data[ep_index].dst_ep;
//                     read_req.zcl_basic_cmd.dst_addr_u.addr_short = dst_node_info->short_addr;
//                     read_req.clusterID = dst_node_info->dst_ep_info.cluster_id[cluster_index];
//                     read_req.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV;
//                     read_req.dis_default_resp = 0;
//                     read_req.manuf_code = 0;
//                     read_req.manuf_specific = 0;
//                     if(bind_req[cluster_index].cluster_id == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF){
//                         uint16_t attributes1 = ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID;
//                         read_req.attr_number = 1;
//                         read_req.attr_field = &attributes1;  
//                     }else if(bind_req[cluster_index].cluster_id == ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL){
//                         uint16_t attributes2[] = {ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID};
//                         read_req.attr_number = 1;
//                         read_req.attr_field = attributes2;
//                     }else if(bind_req[cluster_index].cluster_id == ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL){
//                         uint16_t attributes3[] = {0xE100};
//                         read_req.attr_number = 1;
//                         read_req.attr_field = attributes3; 
//                     }else if(bind_req[cluster_index].cluster_id == ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL){
//                         uint16_t attributes4[] = {ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID};
//                         read_req.attr_number = 1;
//                         read_req.attr_field = attributes4;     
//                     }else if(bind_req[cluster_index].cluster_id == ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT){
//                         uint16_t attributes5[] = {ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID, ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID};
//                         read_req.attr_number = 2;
//                         read_req.attr_field = attributes5;             
//                     }
//                     cb_requests_counts++;
//                     esp_zb_zcl_read_attr_cmd_req(&read_req);      
//                 }  
//             } //for
//         }else{
//             printf("cluster_counts:%d\n", dst_node_info->dst_ep_info.clusters_count);
//         }
//     }
// }



void nuos_simple_binding_request(uint8_t btn_index, uint8_t ep_index, dst_node_info_t *dst_node_info){

    memset(bind_req, 0, sizeof(bind_req));  // Clear all elements to zero
    esp_zb_ieee_addr_t src_ieee_addr;
    
    if(dst_node_info->dst_ep_info.ep_data[ep_index].clusters_count > 0 && dst_node_info->dst_ep_info.ep_data[ep_index].clusters_count < 5){
        for(int cluster_index=0; cluster_index<dst_node_info->dst_ep_info.ep_data[ep_index].clusters_count; cluster_index++){
            //query_callback_handle = xTaskGetCurrentTaskHandle();
          
            // bind_req[cluster_index].req_dst_addr = dst_node_info->short_addr;
            // esp_zb_ieee_addr_t ieee_addr;
    
            // esp_zb_ieee_address_by_short(dst_node_info->short_addr, ieee_addr);
    
            // print_ieee_addr(ieee_addr);
            // /* populate the src information of the binding */
            // memcpy(bind_req[cluster_index].src_address, ieee_addr, sizeof(esp_zb_ieee_addr_t));
            // bind_req[cluster_index].src_endp = dst_node_info->dst_ep_info.ep_data[ep_index].dst_ep;
            // bind_req[cluster_index].cluster_id = dst_node_info->dst_ep_info.cluster_id[cluster_index];
    
            // /* populate the dst information of the binding */
            // bind_req[cluster_index].dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
            // esp_zb_get_long_address(bind_req[cluster_index].dst_address_u.addr_long);
            // bind_req[cluster_index].dst_endp = ENDPOINTS_LIST[btn_index];
            // ESP_LOGI(TAG, "Request remote device to bind us");

            ///////////////////////////////////////////////////////////////////////////////////            
            esp_zb_ieee_address_by_short(dst_node_info->short_addr, src_ieee_addr);
            esp_zb_get_long_address(bind_req[cluster_index].src_address);
            bind_req[cluster_index].src_endp = ENDPOINTS_LIST[btn_index];
                    
            bind_req[cluster_index].dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
            memcpy(bind_req[cluster_index].dst_address_u.addr_long, src_ieee_addr, sizeof(esp_zb_ieee_addr_t));
            bind_req[cluster_index].dst_endp = dst_node_info->dst_ep_info.ep_data[ep_index].dst_ep;
            bind_req[cluster_index].dst_address_u.addr_short = dst_node_info->short_addr;        
            bind_req[cluster_index].req_dst_addr = esp_zb_get_short_address(); /* TODO: Send bind request to self */            
            
            bind_req[cluster_index].cluster_id = dst_node_info->dst_ep_info.ep_data[ep_index].cluster_id[cluster_index];
            dst_node_info->dst_ep_info.ep_data[ep_index].is_bind = false;
            if(!dst_node_info->dst_ep_info.ep_data[ep_index].is_bind) {                 
                cb_requests_counts++;
                ESP_LOGI(TAG, "Try to bind Light\n");
                bind_retry_counts = 0;
                esp_zb_zdo_device_bind_req(&bind_req[cluster_index], bind_cb_2, (void *)&bind_req[cluster_index]);               
            }else{
                ESP_LOGI(TAG, "Already bind Light\n");
                printf("========SEND CMD READ ATTR DATA==========\n");
                printf("src_ep: %u dst_ep: %u\n", bind_req[cluster_index].src_endp, dst_node_info->dst_ep_info.ep_data[ep_index].dst_ep);
                printf("dst_short_addr: 0x%x cluster_id: %x\n", dst_node_info->short_addr, dst_node_info->dst_ep_info.ep_data[ep_index].cluster_id[cluster_index]);
                printf("==============================\n");

                esp_zb_zcl_read_attr_cmd_t read_req;
                read_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
                read_req.zcl_basic_cmd.src_endpoint = ENDPOINTS_LIST[btn_index];
                read_req.zcl_basic_cmd.dst_endpoint = dst_node_info->dst_ep_info.ep_data[ep_index].dst_ep;
                read_req.zcl_basic_cmd.dst_addr_u.addr_short = dst_node_info->short_addr;
                read_req.clusterID = dst_node_info->dst_ep_info.ep_data[ep_index].cluster_id[cluster_index]; //dst_node_info->dst_ep_info.ep_data[ep_index].clusters_count
                read_req.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV;
                read_req.dis_default_resp = 0;
                read_req.manuf_code = 0;
                read_req.manuf_specific = 0;
                if(bind_req[cluster_index].cluster_id == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF){
                    uint16_t attributes1 = ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID;
                    read_req.attr_number = 1;
                    read_req.attr_field = &attributes1;  
                }else if(bind_req[cluster_index].cluster_id == ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL){
                    uint16_t attributes2[] = {ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID};
                    read_req.attr_number = 1;
                    read_req.attr_field = attributes2;
                }else if(bind_req[cluster_index].cluster_id == ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL){
                    uint16_t attributes3[] = {ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_HUE_ID, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID};
                    read_req.attr_number = 2;
                    read_req.attr_field = attributes3; 
                }else if(bind_req[cluster_index].cluster_id == ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL){
                    uint16_t attributes4[] = {ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID};
                    read_req.attr_number = 1;
                    read_req.attr_field = attributes4;     
                }else if(bind_req[cluster_index].cluster_id == ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT){
                    uint16_t attributes5[] = {ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID, ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID};
                    read_req.attr_number = 2;
                    read_req.attr_field = attributes5;             
                }
                // else if(bind_req[cluster_index].cluster_id == 0xEF00){
                //     uint16_t attributes6[] = {0xE000, 0xE100, 0xF000};
                //     read_req.attr_number = 3;
                //     read_req.attr_field = attributes6;             
                // }
                cb_requests_counts++;
                esp_zb_zcl_read_attr_cmd_req(&read_req);
                vTaskDelay(pdMS_TO_TICKS(20));
                //send_zb_to_read_basic_attribute_values(ENDPOINTS_LIST[btn_index], dst_node_info->dst_ep_info.ep_data[ep_index].dst_ep, dst_node_info->short_addr);                
            }  
        } //for
    }else{
        printf("cluster_counts:%d\n", dst_node_info->dst_ep_info.ep_data[ep_index].clusters_count);
    }
}

void nuos_simple_unbinding_request(uint8_t btn_index, uint8_t ep_index, dst_node_info_t *dst_node_info) {
    // Clear the entire array
    memset(bind_req, 0, sizeof(bind_req));  // Clear all elements to zero
    printf("dst_short_addr: 0x%x,  clusters_count: %d\n", dst_node_info->short_addr, dst_node_info->dst_ep_info.ep_data[ep_index].clusters_count);
    esp_zb_ieee_addr_t src_ieee_addr;
    if(dst_node_info->dst_ep_info.ep_data[ep_index].clusters_count > 0 && dst_node_info->dst_ep_info.ep_data[ep_index].clusters_count < 5){
        for(int cluster_index=0; cluster_index<dst_node_info->dst_ep_info.ep_data[ep_index].clusters_count; cluster_index++){
            // esp_zb_ieee_address_by_short(dst_node_info->short_addr, src_ieee_addr);
            // esp_zb_get_long_address(bind_req[cluster_index].src_address);
            // bind_req[cluster_index].src_endp = ENDPOINTS_LIST[btn_index]; //
                    
            // bind_req[cluster_index].dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
            // memcpy(bind_req[cluster_index].dst_address_u.addr_long, src_ieee_addr, sizeof(esp_zb_ieee_addr_t));
            // bind_req[cluster_index].dst_endp = dst_node_info->dst_ep_info.ep_data[ep_index].dst_ep;
            // bind_req[cluster_index].dst_address_u.addr_short = dst_node_info->short_addr;        
            // bind_req[cluster_index].req_dst_addr = esp_zb_get_short_address(); /* TODO: Send bind request to self */            
            
            // bind_req[cluster_index].cluster_id = dst_node_info->dst_ep_info.cluster_id[cluster_index];

            bind_req[cluster_index].req_dst_addr = dst_node_info->short_addr;
            esp_zb_ieee_addr_t ieee_addr;
    
            esp_zb_ieee_address_by_short(dst_node_info->short_addr, ieee_addr);
    
            print_ieee_addr(ieee_addr);
            /* populate the src information of the binding */
            memcpy(bind_req[cluster_index].src_address, ieee_addr, sizeof(esp_zb_ieee_addr_t));
            bind_req[cluster_index].src_endp = dst_node_info->dst_ep_info.ep_data[ep_index].dst_ep;
            bind_req[cluster_index].cluster_id = dst_node_info->dst_ep_info.ep_data[ep_index].cluster_id[cluster_index];
    
            /* populate the dst information of the binding */
            bind_req[cluster_index].dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
            esp_zb_get_long_address(bind_req[cluster_index].dst_address_u.addr_long);
            bind_req[cluster_index].dst_endp = ENDPOINTS_LIST[btn_index];
            ESP_LOGI(TAG, "Request remote device to bind us");


            printf("cluster_id: 0x%x\n", dst_node_info->dst_ep_info.ep_data[ep_index].cluster_id[cluster_index]);
            ESP_LOGI(TAG, "Try to Unbind Light\n");

            cb_requests_counts++;
            esp_zb_zdo_device_unbind_req(&bind_req[cluster_index], unbind_cb_2, (void *)&bind_req[cluster_index]);


        }
        // uint8_t btn_index = get_button_index(src_endpoint); 
        if(btn_index != 0xff){
            uint8_t node_index = get_node_index(btn_index, dst_node_info->short_addr);
            if(node_index != 0xff){
                ESP_LOGI(TAG, "clear_remote_scene_devices_attribute_bind_values\n");
                clear_remote_scene_devices_attribute_bind_values(btn_index, node_index, ep_index, dst_node_info->short_addr);
            }else{
                printf("getting node_index:%d\n", node_index);
            }
        } else{
            printf("getting btn_index:%d\n", btn_index);
        }       
    }else{
        printf("cluster_counts:%d\n", dst_node_info->dst_ep_info.ep_data[ep_index].clusters_count);
    }

}




void nuos_attribute_get_request(uint8_t btn_index, uint8_t ep_index, dst_node_info_t *dst_node_info, uint8_t cl_index){
    if(dst_node_info->dst_ep_info.ep_data[ep_index].clusters_count > 0 && dst_node_info->dst_ep_info.ep_data[ep_index].clusters_count < 5){
        for(int cluster_index=0; cluster_index<dst_node_info->dst_ep_info.ep_data[ep_index].clusters_count; cluster_index++){
            if(cl_index == cluster_index || cl_index == 0xff){
                if(dst_node_info->dst_ep_info.ep_data[ep_index].is_bind) {                 
                    //ESP_LOGI(TAG, "Already bind Light\n");
                    printf("========SENDING CMD READ ATTR DATA==========\n");
                    esp_zb_zcl_read_attr_cmd_t read_req;
                    read_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
                    read_req.zcl_basic_cmd.src_endpoint = ENDPOINTS_LIST[btn_index];
                    read_req.zcl_basic_cmd.dst_endpoint = dst_node_info->dst_ep_info.ep_data[ep_index].dst_ep;
                    read_req.zcl_basic_cmd.dst_addr_u.addr_short = dst_node_info->short_addr;
                    read_req.clusterID = dst_node_info->dst_ep_info.ep_data[ep_index].cluster_id[cluster_index]; //dst_node_info->dst_ep_info.ep_data[ep_index].clusters_count
                    read_req.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV;
                    read_req.dis_default_resp = 0;
                    read_req.manuf_code = 0;
                    read_req.manuf_specific = 0;
                    if(dst_node_info->dst_ep_info.ep_data[ep_index].cluster_id[cluster_index] == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF){
                        uint16_t attributes1 = ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID;
                        read_req.attr_number = 1;
                        read_req.attr_field = &attributes1;  
                    }else if(dst_node_info->dst_ep_info.ep_data[ep_index].cluster_id[cluster_index]  == ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL){
                        uint16_t attributes2[] = {ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID};
                        read_req.attr_number = 1;
                        read_req.attr_field = attributes2;
                    }else if(dst_node_info->dst_ep_info.ep_data[ep_index].cluster_id[cluster_index]  == ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL){
                        uint16_t attributes3[] = {
                            ESP_ZB_ZCL_ATTR_COLOR_CONTROL_ENHANCED_COLOR_MODE_ID, 
                            ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_HUE_ID, 
                            ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID};
                        // ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_MODE_ID   //ESP_ZB_ZCL_ATTR_COLOR_CONTROL_ENHANCED_COLOR_MODE_ID
                        read_req.attr_number = 3;
                        read_req.attr_field = attributes3; 
                    }else if(dst_node_info->dst_ep_info.ep_data[ep_index].cluster_id[cluster_index] == ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL){
                        uint16_t attributes4[] = {ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID};
                        read_req.attr_number = 1;
                        read_req.attr_field = attributes4;     
                    }else if(dst_node_info->dst_ep_info.ep_data[ep_index].cluster_id[cluster_index] == ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT){
                        uint16_t attributes5[] = {ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID, ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID};
                        read_req.attr_number = 2;
                        read_req.attr_field = attributes5;             
                    }
                    cb_requests_counts++;

                    // query_neighbour_task_handle = xTaskGetCurrentTaskHandle();
                    esp_zb_zcl_read_attr_cmd_req(&read_req);  
                    vTaskDelay(pdMS_TO_TICKS(80));
                    // uint32_t is_notified= ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10000));
                    // if (is_notified > 0) {
                    //     ESP_LOGI("Zigbee", "esp_zb_zdo_active_ep_req Notified OK!!");
                    // } else {
                    //     ESP_LOGI("Zigbee", "esp_zb_zdo_active_ep_req Notified Timeout!!");
                    // } 
                } 
            } 
        } //for
    }else{
        printf("cluster_counts:%d\n", dst_node_info->dst_ep_info.ep_data[ep_index].clusters_count);
    }
}


uint8_t nuos_get_button_press_index(uint32_t pin){
    for(int index=0; index<TOTAL_BUTTONS; index++){
        if(pin == gpio_touch_btn_pins[index]){
            #ifdef USE_ZB_ONLY_FAN
            if(index < 2){
                return index==0?1:0;
            }else{
                return index;
            }
            #else
            return index;
            #endif
        }
    }
    return 0xff;
}

uint8_t nuos_get_endpoint_index(uint8_t dst_ep){
    for(int index=0; index<TOTAL_BUTTONS; index++){
        if(dst_ep == ENDPOINTS_LIST[index]){
            return index;
        }
    }
    return 0;
}

bool nuos_init_wifi_webserver(){
    #ifdef USE_WIFI_WEBSERVER
        return nuos_init_webserver();
    #else
        return true;
    #endif
}

void nuos_zb_reset_nvs_start_commissioning(){
    if(start_commissioning == 0){
        setNVSCommissioningFlag(1);
        esp_zb_factory_reset();
    }
}
void nuos_zb_nlme_status_indication(){
    #ifdef USE_RGB_LED
        if(light_driver_deinit_flag){
            light_driver_deinit_flag = false;
            light_driver_init(LIGHT_DEFAULT_OFF);
        }
        //light_driver_set_color_RGB(0xff, 0, 0); //
    #endif
}

void nuos_zb_indentify_indication(uint16_t cluster_id, uint8_t dst_endpoint, uint8_t effect_id){
    identify_response_commands_t identify_stt;
    if(dst_endpoint <= TOTAL_BUTTONS){
        identify_stt.index = nuos_get_endpoint_index(dst_endpoint);
        if(identify_stt.index != 0xff){
            switch(effect_id){
                case ESP_ZB_ZCL_IDENTIFY_EFFECT_ID_BLINK:
                identify_stt.time_value = 1000;
                identify_stt.command = 1;
                xTaskCreate(identify_cluster_task, "identify_cluster_task", TASK_STACK_SIZE_IDENTIFY, &identify_stt, TASK_PRIORITY_IDENTIFY, NULL);
                break;

                case ESP_ZB_ZCL_IDENTIFY_EFFECT_ID_BREATHE:
                identify_stt.time_value = 1000;
                identify_stt.command = 15;
                xTaskCreate(identify_cluster_task, "identify_cluster_task", TASK_STACK_SIZE_IDENTIFY, &identify_stt, TASK_PRIORITY_IDENTIFY, NULL);                
                break;

                case ESP_ZB_ZCL_IDENTIFY_EFFECT_ID_OKAY:
                identify_stt.time_value = 1000;
                identify_stt.command = 2;
                xTaskCreate(identify_cluster_task, "identify_cluster_task", TASK_STACK_SIZE_IDENTIFY, &identify_stt, TASK_PRIORITY_IDENTIFY, NULL);                
                break;

                case ESP_ZB_ZCL_IDENTIFY_EFFECT_ID_CHANNEL_CHANGE:
                break;

                case ESP_ZB_ZCL_IDENTIFY_EFFECT_ID_FINISH_EFFECT:
                break;

                case ESP_ZB_ZCL_IDENTIFY_EFFECT_ID_STOP:
                break;

                default:
                break;
            }
        }
        
    }
}
void identify_cluster_task(void* args) {
    identify_response_commands_t* identify_stt = ((identify_response_commands_t *)args);
    printf("identify_cluster_task-->index:%d  comand:%d", identify_stt->index, identify_stt->command);
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_MOTION || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_CONTACT_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_GAS_LEAK)
        gpio_set_direction(BUZZER_PIN, GPIO_MODE_OUTPUT);
        gpio_set_direction(gpio_touch_led_pins[0], GPIO_MODE_OUTPUT);
        gpio_set_level(gpio_touch_led_pins[0], 1);
        gpio_set_direction(gpio_touch_led_pins[1], GPIO_MODE_OUTPUT);
        gpio_set_level(gpio_touch_led_pins[1], 1);
        gpio_set_level(BUZZER_PIN, 1);
        TickType_t xDelay = pdMS_TO_TICKS(10);
        vTaskDelay(xDelay);
        gpio_set_direction(BUZZER_PIN, GPIO_MODE_OUTPUT);
        gpio_set_direction(gpio_touch_led_pins[0], GPIO_MODE_OUTPUT);
        gpio_set_level(gpio_touch_led_pins[0], 0);
        gpio_set_direction(gpio_touch_led_pins[1], GPIO_MODE_OUTPUT);
        gpio_set_level(gpio_touch_led_pins[1], 0);
        gpio_set_level(BUZZER_PIN, 0);
        xDelay = pdMS_TO_TICKS(1000);
        vTaskDelay(xDelay);
    #else
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4R_ON_OFF_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2R_ON_OFF_LIGHT)
            printf("identify_stt->index:%d", identify_stt->index);
            device_info[identify_stt->index].device_state = true;    
            nuos_zb_set_hardware(identify_stt->index, false);
            const TickType_t xDelay = pdMS_TO_TICKS(identify_stt->time_value);
            vTaskDelay(xDelay);
            device_info[identify_stt->index].device_state = false;  
            nuos_zb_set_hardware(identify_stt->index, false);
            vTaskDelay(xDelay);
        #endif
    #endif
    vTaskDelete(NULL); // Delete the task after executing
}


//////////////////reporting functions////////////////////////////////////////
void setup_reporting_for_thermostat_system_mode_id() {
    esp_zb_zcl_config_report_cmd_t report_cmd;
        bool report_change = 0;
        report_cmd.zcl_basic_cmd.dst_addr_u.addr_short = 0;
        report_cmd.zcl_basic_cmd.dst_endpoint = 1;
        report_cmd.zcl_basic_cmd.src_endpoint = 1;
        report_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
        report_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT;

        esp_zb_zcl_config_report_record_t records[] = {
            {
                .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
                .attributeID = ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID,
                .attrType = ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM,
                .min_interval = 0,
                .max_interval = 30,
                .reportable_change = &report_change
            },
        };


    report_cmd.record_number = sizeof(records) / sizeof(esp_zb_zcl_config_report_record_t);
    report_cmd.record_field = records;
    esp_zb_zcl_config_report_cmd_req(&report_cmd);

    // /* Config the reporting info  */
    // esp_zb_zcl_reporting_info_t reporting_info = {
    //     .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
    //     .ep = 1,
    //     .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
    //     .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    //     .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
    //     .u.send_info.min_interval = 1,
    //     .u.send_info.max_interval = 0,
    //     .u.send_info.def_min_interval = 1,
    //     .u.send_info.def_max_interval = 0,
    //     .u.send_info.delta.u16 = 100,
    //     .attr_id = ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID,
    //     .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    // };
    // esp_zb_zcl_update_reporting_info(&reporting_info);
}


void setup_reporting_for_fan_attributes() {
    esp_zb_zcl_config_report_cmd_t report_cmd;
        unsigned int report_change = 0;
        report_cmd.zcl_basic_cmd.dst_addr_u.addr_short = 0;
        report_cmd.zcl_basic_cmd.dst_endpoint = 2;
        report_cmd.zcl_basic_cmd.src_endpoint = 2;
        report_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
        report_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL;

        esp_zb_zcl_config_report_record_t records[] = {
            {
                .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
                .attributeID = ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID,
                .attrType = ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM,
                .min_interval = 0,
                .max_interval = 30,
                .reportable_change = &report_change
            },
        };

    report_cmd.record_number = sizeof(records) / sizeof(esp_zb_zcl_config_report_record_t);
    report_cmd.record_field = records;

    esp_zb_zcl_config_report_cmd_req(&report_cmd);

        /* Config the reporting info  */
    // esp_zb_zcl_reporting_info_t reporting_info = {
    //     .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
    //     .ep = 2,
    //     .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL,
    //     .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    //     .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
    //     .u.send_info.min_interval = 1,
    //     .u.send_info.max_interval = 0,
    //     .u.send_info.def_min_interval = 1,
    //     .u.send_info.def_max_interval = 0,
    //     .u.send_info.delta.u16 = 100,
    //     .attr_id = ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID,
    //     .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    // };
    // esp_zb_zcl_update_reporting_info(&reporting_info);
}



#ifdef USE_OTA
static const esp_partition_t *s_ota_partition = NULL;
static esp_ota_handle_t s_ota_handle = 0;


esp_err_t zb_ota_upgrade_status_handler(esp_zb_zcl_ota_upgrade_value_message_t messsage)
{
    static uint32_t total_size = 0;
    static uint32_t offset = 0;
    static int64_t start_time = 0;
    esp_err_t ret = ESP_OK;
    if (messsage.info.status == ESP_ZB_ZCL_STATUS_SUCCESS) {
        switch (messsage.upgrade_status) {
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_START:
            ESP_LOGI(TAG, "-- OTA upgrade start");
            start_time = esp_timer_get_time();
            s_ota_partition = esp_ota_get_next_update_partition(NULL);
            assert(s_ota_partition);
            ret = esp_ota_begin(s_ota_partition, OTA_WITH_SEQUENTIAL_WRITES, &s_ota_handle);
            ESP_RETURN_ON_ERROR(ret, TAG, "Failed to begin OTA partition, status: %s", esp_err_to_name(ret));
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_RECEIVE:
            total_size = messsage.ota_header.image_size;
            offset += messsage.payload_size;
            ESP_LOGI(TAG, "-- OTA Client receives data: progress [%ld/%ld]", offset, total_size);
            if (messsage.payload_size && messsage.payload) {
                ret = esp_ota_write(s_ota_handle, (const void *)messsage.payload, messsage.payload_size);
                ESP_RETURN_ON_ERROR(ret, TAG, "Failed to write OTA data to partition, status: %s", esp_err_to_name(ret));
            }
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_APPLY:
            ESP_LOGI(TAG, "-- OTA upgrade apply");
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_CHECK:
            ret = offset == total_size ? ESP_OK : ESP_FAIL;
            ESP_LOGI(TAG, "-- OTA upgrade check status: %s", esp_err_to_name(ret));
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_FINISH:
            ESP_LOGI(TAG, "-- OTA Finish");
            ESP_LOGI(TAG,
                     "-- OTA Information: version: 0x%lx, manufactor code: 0x%x, image type: 0x%x, total size: %ld bytes, cost time: %lld ms,",
                     messsage.ota_header.file_version, messsage.ota_header.manufacturer_code, messsage.ota_header.image_type,
                     messsage.ota_header.image_size, (esp_timer_get_time() - start_time) / 1000);
            ret = esp_ota_end(s_ota_handle);
            ESP_RETURN_ON_ERROR(ret, TAG, "Failed to end OTA partition, status: %s", esp_err_to_name(ret));
            ret = esp_ota_set_boot_partition(s_ota_partition);
            ESP_RETURN_ON_ERROR(ret, TAG, "Failed to set OTA boot partition, status: %s", esp_err_to_name(ret));
            ESP_LOGW(TAG, "Prepare to restart system");
            esp_restart();
            break;
        default:
            ESP_LOGI(TAG, "OTA status: %d", messsage.upgrade_status);
            break;
        }
    }
    return ret;
}

esp_err_t nuos_zb_ota_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message){
    esp_err_t ret = -1;
    switch (callback_id) {
        case ESP_ZB_CORE_OTA_UPGRADE_VALUE_CB_ID:
        ret = zb_ota_upgrade_status_handler(*(esp_zb_zcl_ota_upgrade_value_message_t *)message);
        break;   

        default: break;
    }
    return ret; 
}

#endif
void nuos_toggle_rgb_led(){
    static bool toggle_state = 0;
    toggle_state = !toggle_state;
    #ifdef USE_RGB_LED
    light_driver_set_power(toggle_state);
    #endif
}
int task_sequence_num                           = -1;
uint8_t save_pressed_button_index               = 0;
uint8_t node_counts_sequence                    = 0;
uint8_t node_ep_counts_sequence                 = 0;
#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_SCENE_SWITCH)
extern esp_err_t nuos_set_scene_button_attribute(uint8_t index);
#endif
uint32_t current_speed_index = 0;
//static int timer_cnts = 0;



static bool one_time_scan_flag = true;
static uint8_t cccounts = 0;
uint8_t cc_nts = 0;

#ifdef USE_RGB_LED
// Declare the task handle globally
TaskHandle_t rgb_task_handle = NULL;
static void rgb_task(void *pvParameters){
    while(1){
        //vTaskDelay(pdMS_TO_TICKS(10));
        // Block and wait for notification to blink the LED
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if(blink_rgb_led_flag){
            blink_rgb_led_flag = false;
            light_driver_set_power(1);
            vTaskDelay(pdMS_TO_TICKS(50));
            light_driver_set_power(0);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        if(blink_rgb_led_normal_functionality_flag){
            blink_rgb_led_normal_functionality_flag = false;
             light_driver_set_power(1);
             vTaskDelay(pdMS_TO_TICKS(50));
             light_driver_set_power(0);
        }
        if(blink_rgb_led_commissioning_functionality_flag){
            blink_rgb_led_commissioning_functionality_flag = false;
             light_driver_set_power(1);
             vTaskDelay(pdMS_TO_TICKS(10));
             light_driver_set_power(0);
        }        
    }   
}

/////////////////////////////////////////////////
// typedef struct {
//     uint32_t toggle_interval_ms;
// } rgb_task_params_t;

// static void rgb_task(void *pvParameters) {
//     rgb_task_params_t *params = (rgb_task_params_t *)pvParameters;
//     bool led_state = false;

//     while (1) {
//         led_state = !led_state;
//         light_driver_set_power(led_state ? 1 : 0);
//         vTaskDelay(pdMS_TO_TICKS(params->toggle_interval_ms));
//     }
// }



/// @brief ////////////////////////////////////////////////////////////////


// static void rgb_task(void *pvParameters) {
//     // Blinking intervals (in milliseconds) for 3 speeds
    
//     //rgb_task_params_t *params = (rgb_task_params_t *)pvParameters;                     // Start with first speed
//     bool led_state = false;
//     //current_speed_index = *(uint32_t *)pvParameters; 
//     // Track time for speed switching
//     //TickType_t last_speed_change_time = xTaskGetTickCount();

//     while (1) {

//         // Check if 1 minute (60,000ms) has passed since last speed change
//         // if ((xTaskGetTickCount() - last_speed_change_time) * portTICK_PERIOD_MS >= 60000) {
//         //     current_speed_index = (current_speed_index + 1) % 3; // Cycle through 0,1,2
//         //     last_speed_change_time = xTaskGetTickCount();         // Reset timer
//         // }
//         if(rgb_led_blink_flag){
//             // Toggle LED and delay with current speed
//             led_state = !led_state;
//             light_driver_set_power(led_state ? 1 : 0);
//         }
//         vTaskDelay(pdMS_TO_TICKS(blink_intervals[current_speed_index]));
//     }
// }


void nuos_blink_rgb_led_init_task() {
    rgb_led_blink_flag = false;
    current_speed_index = 0;
    xTaskCreate(rgb_task, "rgb_task", TASK_STACK_SIZE_RGB, NULL, TASK_PRIORITY_RGB, &rgb_task_handle);
}

void nuos_start_rgb_led_blink_task(uint32_t index){
    if(wifi_webserver_active_flag > 0){
        light_driver_set_color_RGB(0xff, 0xa5, 0); //orange color for IR STA mode functionality
    }else{
        if(start_commissioning > 0){
            light_driver_set_color_RGB(0xff, 0, 0); //red color for commissioning functionality
        }else{
            light_driver_set_color_RGB(0, 0xff, 0); //green color for normal functionality
        }
    }
    rgb_led_blink_flag = true;
    current_speed_index = index;
}
void nuos_kill_rgb_led_blink_task(){
    
    rgb_led_blink_flag = false;
    light_driver_set_power(0);
    if(rgb_task_handle != NULL)
    vTaskDelete(rgb_task_handle);
    rgb_task_handle = NULL;
    light_driver_set_power(0);
}

// Function to trigger the blink
void nuos_rgb_trigger_blink() {
    //nuos_start_rgb_led_blink_task(0);
    // light_driver_set_power(1);
    // vTaskDelay(pdMS_TO_TICKS(50));
    // light_driver_set_power(0);
}
#endif

void nuos_set_rgb_led_normal_functionality(){ 
    blink_rgb_led_normal_functionality_flag = true;
    #ifdef USE_RGB_LED
    light_driver_set_color_RGB(0, 0xff, 0);
    if(rgb_task_handle != NULL) xTaskNotifyGive(rgb_task_handle); // Notify the rgb_task to run 
    #endif   
}
void nuos_set_rgb_led_commissioning_functionality(){
    blink_rgb_led_commissioning_functionality_flag = true;
    #ifdef USE_RGB_LED
    if(rgb_task_handle != NULL) xTaskNotifyGive(rgb_task_handle); // Notify the rgb_task to run 
    #endif  
}
void nuos_init_rgb_led(){
    light_driver_init(LIGHT_DEFAULT_OFF);
    vTaskDelay(pdMS_TO_TICKS(10));
    #ifdef USE_RGB_LED
       
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
            if(wifi_webserver_active_flag > 0){
                light_driver_set_color_RGB(0xff, 0xa5, 0); //orange color for IR STA mode functionality
            }else{
                if(start_commissioning > 0){
                    light_driver_set_color_RGB(0xff, 0, 0); //red color for commissioning functionality
                }else{
                    light_driver_set_color_RGB(0, 0xff, 0); //green color for normal functionality
                }
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        #else
            if(start_commissioning > 0){
                light_driver_set_color_RGB(0xff, 0, 0); //red color for zigbee commissioning functionality
            }else{
                light_driver_set_color_RGB(0, 0xff, 0); //green color for normal functionality
            }  
        #endif  
        light_driver_set_power(false);
    #else
        // gpio_reset_pin(GPIO_NUM_8);  
        // gpio_set_direction(GPIO_NUM_8, GPIO_MODE_OUTPUT); //
        // gpio_set_level(GPIO_NUM_8, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        light_driver_set_power(false);         
    #endif
}


void nuos_switch_single_click_task(uint32_t io_num) {

    button_index = nuos_get_button_press_index(io_num);
    printf("SINGLE CLICK Detected!!: index:%d\n", button_index);
    brightness_control_flag = false;
    #ifndef DONT_USE_ZIGBEE
    if(!is_my_device_commissionned){
        start_commissioning = true;
        #ifdef ZB_COMMISSIONING_WITHOUT_TIMER
            nuos_start_mode_change_task();
        #else //start commissioning timer
            init_timer();
            esp_start_timer();    
        #endif
    }
    #endif
    
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN_SWITCH)
    if(curtain_cal_mode_active_flag){
        if(button_index == 0){
            is_bt_start_pressed = true;
        }else if(button_index == 1){
            is_bt_end_pressed = true;
        } 
    } else{  
    #endif    
    
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_SCENE_SWITCH) 
        nuos_set_scene_button_attribute(button_index);
    #else
        // Added by Nuos
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
            if(button_index == 0 || button_index == 1){
                nuos_zb_set_hardware(button_index, true);
                #ifdef USE_CCT_TIME_SYNC
                if(button_index == 0) set_state(button_index);
                #else
                set_state(button_index);
                #endif
            }else{
                nuos_zb_set_hardware(button_index, false);
                set_color_temp();
            }
        #else
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI)
                if(button_index == TOTAL_BUTTONS-1){
                    #if(USE_COLOR_DEVICE == COLOR_RGB_ONLY)
                    selected_color_mode = 1;
                    #else
                    selected_color_mode = 0;
                    #endif
                 
                }else{
                    selected_color_mode = 1;
                }
                if(last_selected_color_mode != selected_color_mode){
                    last_selected_color_mode = selected_color_mode;
                    mode_change_flag = true;  
                }
                store_color_mode_value(selected_color_mode); 
                #if(USE_COLOR_DEVICE == COLOR_RGB_ONLY)
                if(button_index == 3){
                    device_info[4].device_state = !device_info[4].device_state;
                }
                #endif                       
                global_index = button_index; 

                nuos_zb_set_hardware(button_index, true);
                #if(USE_COLOR_DEVICE == COLOR_RGB_ONLY)
                    set_color_temp();
                    set_level(3);                
                #else
                // set_color_flag = true;
                
                // set_level_flag = true;
                set_color_temp();
                if(button_index < 3){
                    set_state(4);
                    set_level(4);
                }else{
                    set_state(3);
                    set_level(3);
                }

                
                #endif
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_MOTION || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_CONTACT_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_GAS_LEAK || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_LUX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY)    
            #else
                nuos_zb_set_hardware(button_index, true);  
            #endif    
        #endif 
        #ifndef DONT_USE_ZIGBEE 
        if((is_my_device_commissionned == true) && (wifi_webserver_active_flag == false)){
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
                // nuos_set_zigbee_attribute(0); 
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RINGING_BELL_2)

            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI)
                // nuos_set_zigbee_attribute(0); 
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX) 
                nuos_set_zigbee_attribute(0);
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)   
            // if(one_time_scan_flag){ if(cccounts++ >= 20) { cccounts = 0; one_time_scan_flag = false; nuos_do_task(button_index, button_index+1, 1);}}
                nuos_set_zigbee_attribute(button_index);     
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_MOTION || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_CONTACT_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_GAS_LEAK || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_LUX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_TEMPERATURE_HUMIDITY)                   
            #else
                #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN)
                #ifdef TUYA_ATTRIBUTES

                #else
                nuos_set_zigbee_attribute(button_index);
                #endif
                #else
                nuos_set_zigbee_attribute(button_index);
                #endif
            #endif
        }
        #endif
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN_SWITCH)
        }
    #endif
    #endif
}

void nuos_switch_double_click_task(uint32_t io_num){
    //printf("DOUBLE CLICK Detected on EP:%d\n", ENDPOINTS_LIST[button_index]);
    button_index = nuos_get_button_press_index(io_num);
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_SCENE_SWITCH) 
        nuos_set_scene_button_attribute(button_index);
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH) 

    #else
        double_press_click_enable = !double_press_click_enable;
    #endif
}


void nuos_switch_long_press_task(uint32_t io_num){
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN_SWITCH)
        #ifdef TUYA_ATTRIBUTES
        //pause_curtain_timer();
        printf("Stop Motor...");
        nuos_zb_set_hardware(2, false);
        #endif
    #endif
}

void nuos_switch_long_press_brightness_task(uint32_t io_num){
    //printf("LONG PRESS BRIGHTNESS Detected!!\n");
    button_index = nuos_get_button_press_index(io_num);
    #ifdef LONG_PRESS_BRIGHTNESS_ENABLE 
    #ifndef DONT_USE_ZIGBEE 
       if((is_my_device_commissionned == true) && (wifi_webserver_active_flag == false)){
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
            switch(button_index){
                case 0:
                    #ifdef USE_TUYA_BRDIGE
                        nuos_set_color_temp_level_attribute(0); //
                    #else                    
                        nuos_set_level_attribute(0);
                    #endif
                break;
                case 2:
                case 3:
                    nuos_set_color_temperature_attribute(0); //
                break;                    
            }
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI)
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI)
                if(last_selected_color_mode != selected_color_mode){
                    last_selected_color_mode = selected_color_mode;
                    mode_change_flag = true;
                    store_color_mode_value(selected_color_mode);
                }
                
            #endif    
            nuos_set_zigbee_attribute(0);
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI)

        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_GROUP_SWITCH)

        #else
            nuos_set_level_attribute(button_index);
        #endif
        }
    #endif
    #else

    #endif
}

void blink_single_ep_led(){
    for(int k=0; k<TOTAL_LEDS; k++){
        gpio_set_level(gpio_touch_led_pins[k], 0);
    }
    for(int i=0; i<TOTAL_LEDS; i++){
        if(i == save_pressed_button_index){
            gpio_set_level(gpio_touch_led_pins[i], 1);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            gpio_set_level(gpio_touch_led_pins[i], 0);
            vTaskDelay(500 / portTICK_PERIOD_MS);                        
        }
    }  
}

bool isSceneRemoteBindingStarted = false;
bool identify_device_complete_flag = false;
extern uint16_t total_press_in_secs;

void mode_change_task(void* args) {
    static int ep_cnts = 0;
        
    uint8_t binding_node_ep_counts_sequence = 0;
    uint8_t binding_node_counts_sequence = 0;

    while(isSceneRemoteBindingStarted){
        vTaskDelay(10 / portTICK_PERIOD_MS);     
        switch(task_sequence_num){
            case TASK_MODE_BLINK_ALL_LEDS:
                for(int i=0; i<TOTAL_LEDS; i++){
                    gpio_set_level(gpio_touch_led_pins[i], 1);
                }
                vTaskDelay(50 / portTICK_PERIOD_MS);
                for(int i=0; i<TOTAL_LEDS; i++){
                    gpio_set_level(gpio_touch_led_pins[i], 0);
                }     
                vTaskDelay(50 / portTICK_PERIOD_MS);      
            break;
            case TASK_MODE_GO_TO_ACTUAL_TASK:
                task_sequence_num = TASK_MODE_ACTUAL_TASK_BLINK_LEDS;
                nuos_find_active_nodes(save_pressed_button_index,  node_counts, 1);
                break;
            case TASK_MODE_ACTUAL_TASK_BLINK_LEDS:
                for(int i=0; i<TOTAL_LEDS; i++){
                    gpio_set_level(gpio_touch_led_pins[i], 1);
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                    gpio_set_level(gpio_touch_led_pins[i], 0);
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                }                         
            break;

            case TASK_MODE_BLINK_OTHER_LEDS:
                for(int i=0; i<TOTAL_LEDS; i++){
                    if(i != save_pressed_button_index){
                        gpio_set_level(gpio_touch_led_pins[i], 1);
                    }
                }
                vTaskDelay(100 / portTICK_PERIOD_MS);
                for(int i=0; i<TOTAL_LEDS; i++){
                    if(i != save_pressed_button_index){
                        gpio_set_level(gpio_touch_led_pins[i], 0);
                    }
                }     
                vTaskDelay(100 / portTICK_PERIOD_MS);                 
            break;  
            case TASK_SEND_IDENTIFY_COMMAND:     
                dst_node_info_t dst_node_info = existing_nodes_info[save_pressed_button_index].scene_switch_info.dst_node_info[node_counts_sequence];   
                ep_cnts = dst_node_info.endpoint_counts;
                if(node_counts_sequence < node_counts){
                    printf("short_addr:0x%x\n", existing_nodes_info[save_pressed_button_index].short_addr);
                    
                   // if(existing_nodes_info[node_counts_sequence].short_addr == 0x5300 || existing_nodes_info[node_counts_sequence].short_addr == 0x647e){

                    vTaskDelay(100 / portTICK_PERIOD_MS);
                    send_identify_command(dst_node_info.short_addr, ENDPOINTS_LIST[save_pressed_button_index],
                        
                            dst_node_info.dst_ep_info.ep_data[node_ep_counts_sequence].dst_ep,  5);   
                         

                    existing_nodes_info[save_pressed_button_index].scene_switch_info.src_ep = ENDPOINTS_LIST[save_pressed_button_index];
                    //}
                    task_sequence_num = TASK_BLINK_SELECTED_LED;
                }else{
                    task_sequence_num = TASK_MODE_EXIT;
                }
                binding_node_ep_counts_sequence = node_ep_counts_sequence;
                binding_node_counts_sequence = node_counts_sequence;

                node_ep_counts_sequence++;
                if(node_ep_counts_sequence >= ep_cnts){
                    node_counts_sequence++;
                    node_ep_counts_sequence = 0;
                }
                identify_device_complete_flag = true; 
                printf("node_counts_sequence:%d\n", node_counts_sequence);
                printf("node_ep_counts_sequence:%d\n", node_ep_counts_sequence);
            break;

            case TASK_BLINK_SELECTED_LED:
                blink_single_ep_led(); 
            break;

            case TASK_SEND_BINDING_REQUEST:
                for(int i=0; i<TOTAL_LEDS; i++){
                    if(i != save_pressed_button_index){
                        gpio_set_level(gpio_touch_led_pins[i], 1);
                    }
                }             
                printf("---BINDING---\n");

                printf("binding_node_counts_sequence:%d\n", binding_node_counts_sequence);
                printf("binding_node_ep_counts_sequence:%d\n", binding_node_ep_counts_sequence);

                task_sequence_num = TASK_ALL_LED_ON;                               
            break;
            case TASK_SEND_UNBINDING_REQUEST:
                printf("---UNBINDING---\n");
                for(int i=0; i<TOTAL_LEDS; i++){
                    if(i != save_pressed_button_index){
                        gpio_set_level(gpio_touch_led_pins[i], 1);
                    }
                } 
                task_sequence_num = TASK_ALL_LED_OFF;                              
            break;
            case TASK_ALL_LED_ON:
                for(int k=0; k<TOTAL_LEDS; k++){
                    gpio_set_level(gpio_touch_led_pins[k], 1);
                }
            break;
            case TASK_ALL_LED_OFF:
                for(int k=0; k<TOTAL_LEDS; k++){
                    gpio_set_level(gpio_touch_led_pins[k], 0);
                }   
            break;                                  
            case TASK_MODE_EXIT:
                isSceneRemoteBindingStarted = false;
                vTaskDelete(NULL);
            break;
            default: break;
        }
    }
}



// Function to convert XY color values to RGB
void xyToRgb(uint16_t x, uint16_t y, float brightness, uint8_t *r, uint8_t *g, uint8_t *b) {
    // Convert 0-65535 range to 0-1 range for XY values
    float x_normalized = x / 65535.0f;
    float y_normalized = y / 65535.0f;
    float z_normalized = 1.0f - x_normalized - y_normalized;

    // Calculate XYZ values
    float Y = brightness; // brightness
    float X = (Y / y_normalized) * x_normalized;
    float Z = (Y / y_normalized) * z_normalized;

    // Convert to RGB using sRGB D65 conversion
    float r_float = X * 1.656492f - Y * 0.354851f - Z * 0.255038f;
    float g_float = -X * 0.707196f + Y * 1.655397f + Z * 0.036152f;
    float b_float = X * 0.051713f - Y * 0.121364f + Z * 1.011530f;

    // Normalize RGB values
    if (r_float > b_float && r_float > g_float && r_float > 1.0f) {
        g_float /= r_float;
        b_float /= r_float;
        r_float = 1.0f;
    } else if (g_float > b_float && g_float > r_float && g_float > 1.0f) {
        r_float /= g_float;
        b_float /= g_float;
        g_float = 1.0f;
    } else if (b_float > r_float && b_float > g_float && b_float > 1.0f) {
        r_float /= b_float;
        g_float /= b_float;
        b_float = 1.0f;
    }

    // Apply gamma correction and convert to 0-255 range
    *r = (r_float <= 0.0031308f) ? (uint8_t)(12.92f * r_float * 255.0f) : (uint8_t)((1.055f * powf(r_float, 1.0f / 2.4f) - 0.055f) * 255.0f);
    *g = (g_float <= 0.0031308f) ? (uint8_t)(12.92f * g_float * 255.0f) : (uint8_t)((1.055f * powf(g_float, 1.0f / 2.4f) - 0.055f) * 255.0f);
    *b = (b_float <= 0.0031308f) ? (uint8_t)(12.92f * b_float * 255.0f) : (uint8_t)((1.055f * powf(b_float, 1.0f / 2.4f) - 0.055f) * 255.0f);

    // Clip RGB values to 0-255 range
    *r = *r < 0 ? 0 : *r > 255 ? 255 : *r;
    *g = *g < 0 ? 0 : *g > 255 ? 255 : *g;
    *b = *b < 0 ? 0 : *b > 255 ? 255 : *b;
}


// Function to convert RGB (0-255) to XY color values (0-65535)
void rgbToXy(uint8_t r, uint8_t g, uint8_t b, uint16_t *x, uint16_t *y) {
    // Convert RGB values to the 0-1 range
    float r_normalized = r / 255.0f;
    float g_normalized = g / 255.0f;
    float b_normalized = b / 255.0f;

    // Apply inverse gamma correction
    r_normalized = (r_normalized > 0.04045f) ? powf((r_normalized + 0.055f) / 1.055f, 2.4f) : (r_normalized / 12.92f);
    g_normalized = (g_normalized > 0.04045f) ? powf((g_normalized + 0.055f) / 1.055f, 2.4f) : (g_normalized / 12.92f);
    b_normalized = (b_normalized > 0.04045f) ? powf((b_normalized + 0.055f) / 1.055f, 2.4f) : (b_normalized / 12.92f);

    // Convert to XYZ using the sRGB D65 conversion matrix
    float X = r_normalized * 0.4124564f + g_normalized * 0.3575761f + b_normalized * 0.1804375f;
    float Y = r_normalized * 0.2126729f + g_normalized * 0.7151522f + b_normalized * 0.0721750f;
    float Z = r_normalized * 0.0193339f + g_normalized * 0.1191920f + b_normalized * 0.9503041f;

    // Convert XYZ to xy
    float x_normalized = X / (X + Y + Z);
    float y_normalized = Y / (X + Y + Z);

    // Scale xy to the 0-65535 range
    *x = (uint16_t)(x_normalized * 65535.0f);
    *y = (uint16_t)(y_normalized * 65535.0f);
}


void rgbToHsv(uint8_t r, uint8_t g, uint8_t b, float *h, float *s, float *v) {
    // Convert RGB values from 0-255 to 0-1 range
    float r_float = r / 255.0f;
    float g_float = g / 255.0f;
    float b_float = b / 255.0f;

    // Find the maximum and minimum values among R, G, and B
    float max_val = fmaxf(r_float, fmaxf(g_float, b_float));
    float min_val = fminf(r_float, fminf(g_float, b_float));
    float delta = max_val - min_val;

    // Calculate Hue (H)
    if (delta == 0.0f) {
        *h = 0.0f; // Undefined hue when the color is grayscale
    } else if (max_val == r_float) {
        *h = 60.0f * fmodf((g_float - b_float) / delta, 6.0f);
        if (*h < 0.0f) {
            *h += 360.0f;
        }
    } else if (max_val == g_float) {
        *h = 60.0f * ((b_float - r_float) / delta + 2.0f);
    } else if (max_val == b_float) {
        *h = 60.0f * ((r_float - g_float) / delta + 4.0f);
    }

    // Calculate Saturation (S)
    if (max_val == 0.0f) {
        *s = 0.0f; // No saturation if max_val is 0 (black)
    } else {
        *s = delta / max_val;
    }

    // Calculate Value (V)
    *v = max_val;
}