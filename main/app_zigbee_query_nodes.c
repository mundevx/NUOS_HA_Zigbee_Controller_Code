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
#include "app_hardware_driver.h"
#include <stdio.h>
#include <string.h>
#include "esp_zigbee_core.h"
#include "app_zigbee_query_nodes.h"
#include "app_zigbee_misc.h"
#include "freertos/event_groups.h"
#include "zcl/esp_zigbee_zcl_command.h"

static const char *TAG                          = "ESP_ZB_QUERY_NODES";

uint8_t src_endpoint_index                      = 0;
uint8_t total_neighbour_records_devices_count   = 0;
char str_name[]                                 = {'d', 'e', 'v', 'i', 'c', 'e', '0', '0', '0', '\0'};

// Define a semaphore to synchronize task completion
extern SemaphoreHandle_t recordsSemaphore;
extern TaskHandle_t query_neighbour_task_handle;


// Define the event group
static EventGroupHandle_t event_group;
#define ARRAY_LENTH(arr) (sizeof(arr) / sizeof(arr[0]))
const char* zb_node_names[17] = {"Zigbee OnOff Light", "Zigbee Dimmable Light", "Zigbee RGB Light", "Zigbee 1-Light 1-Fan", "Curtain Switch", 
            "Wireless Scene Switch", 
            "Zigbee Lux Sensor", "Zigbee Temperature Sensor", "Zigbee Humidity Sensor", "Zigbee IAS Sensor", 
            "Zigbee IR Blaster", "Unknown", "Zigbee CCT Light", "Unknown Sensor", "Unknown Light", "Unknown Device", "Unknown Switch"};    

void start_led_blinking(uint8_t mode, uint8_t src_index){
    blink_leds_start_flag = true;
    scene_ep_index_mode = mode;
    scene_ep_index = src_index;
}

void stop_led_blinking() {
    scene_ep_index_mode = 0;
}

bool nuos_is_zigbee_busy(){
    return blink_leds_start_flag;
}

void nuos_led_blink_task(bool toggle){
    if(blink_leds_start_flag){
        if(scene_ep_index_mode == 0){
            gpio_set_level(gpio_touch_led_pins[scene_ep_index], 0);
            blink_leds_start_flag = false;
        }else if(scene_ep_index_mode == 1){
            gpio_set_level(gpio_touch_led_pins[scene_ep_index], toggle);
        }else if(scene_ep_index_mode == 2){
            gpio_set_level(gpio_touch_led_pins[scene_ep_index], toggle);
            vTaskDelay(pdMS_TO_TICKS(10));
            gpio_set_level(gpio_touch_led_pins[scene_ep_index], toggle);
            vTaskDelay(pdMS_TO_TICKS(10));
            gpio_set_level(gpio_touch_led_pins[scene_ep_index], toggle);
            vTaskDelay(pdMS_TO_TICKS(10));
            gpio_set_level(gpio_touch_led_pins[scene_ep_index], toggle);
        }else if(scene_ep_index_mode == 3){
            gpio_set_level(gpio_touch_led_pins[scene_ep_index], toggle);
            gpio_set_level(gpio_touch_led_pins[scene_ep_index], toggle);
            gpio_set_level(gpio_touch_led_pins[scene_ep_index], toggle);
            gpio_set_level(gpio_touch_led_pins[scene_ep_index], toggle);
        }
    }
}  

#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
    uint8_t get_button_index(uint8_t endpoint_id){
        for(int i=0; i<TOTAL_ENDPOINTS; i++){
            if(ENDPOINTS_LIST[i] == endpoint_id){
                return i;
            }
        }
        return 0xff;
    }

    uint8_t get_node_index(uint8_t btn_index, uint16_t short_addr){
        for(int j=0; j<MAX_NODES; j++){
            if(existing_nodes_info[btn_index].scene_switch_info.dst_node_info[j].short_addr == short_addr){
                return j;
            }
        }
        return 0xff;
    }

    uint8_t get_ep_index(uint8_t btn_index, uint8_t node_index, uint8_t dst_ep){
        for(int k=0; k<existing_nodes_info[btn_index].scene_switch_info.dst_node_info[node_index].endpoint_counts; k++) {
            if(existing_nodes_info[btn_index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[k].dst_ep == dst_ep){   //dst_endpoint[j]
                return k;
            }
        }
        return 0xff;
    }


    // void do_remote_scene_bindings(zb_binding_data_t *bindings, int count) {
    //     for (int i = 0; i < count; i++) {
    //         zb_binding_data_t *b = &bindings[i];
    
    //         printf("Processing binding %d:\n", i);
    //         printf("  Source EP : %d\n", b->source_ep);
    //         printf("  Dest EP   : %d\n", b->dst_ep);
    //         printf("  Short Addr: 0x%04X\n", b->short_addr);
    
    //         // Example: use this data in a Zigbee binding request
    //         // You could replace this with your actual Zigbee API call, e.g.:
    //         // esp_zb_zdo_bind_req(b->short_addr, b->dst_ep, ..., b->source_ep);
    
    //         // Here’s just a stub:
    //         // zigbee_bind_endpoint(b->source_ep, b->short_addr, b->dst_ep);
    //     }
    // }

    void do_remote_scene_bindings(uint8_t btn_index, uint8_t dst_ep, uint16_t short_addr, bool is_binding){
        uint8_t src_ep = ENDPOINTS_LIST[btn_index];
        //printf("src_ep:%d,  btn_index:%d\n", src_ep, btn_index);
        if(btn_index != 0xff){
            uint8_t node_index = get_node_index(btn_index, short_addr);
            if(node_index != 0xff){
                uint8_t ep_index = get_ep_index(btn_index, node_index, dst_ep);
                //printf("dst_ep:%d,  ep_index:%d\n", dst_ep, ep_index);
                if(ep_index != 0xff){
                    dst_node_info_t dst_node_info = existing_nodes_info[btn_index].scene_switch_info.dst_node_info[node_index];
                    existing_nodes_info[btn_index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].is_bind = 1;
                    existing_nodes_info[btn_index].scene_switch_info.src_ep = src_ep;
                    
                    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
                        nuos_attribute_get_request(btn_index, ep_index, &dst_node_info, 0xff);
                    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH)  
                        uint16_t gid = 0;
                        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH)
                            gid = global_group_id[0];
                        #else
                            gid = global_group_id[main_ep_bind_index];
                        #endif
                        //nuos_zb_group_send_add_group_request_to_self(gid, dst_ep, src_ep);
                        nuos_zb_group_send_add_group_request_to_dst(gid, src_ep, dst_ep, short_addr);                
                    #endif
                    // save_nodes_info_to_nvs(btn_index);
                }else{
                    printf("ERROR: No DST ENDPOINT FOUND!!\n");
                }
            }else{
                printf("ERROR: No NODE INDEX FOUND!!\n");
            } 
        }else{
            printf("ERROR: No SRC ENDPOINT FOUND!!\n");
        }
    }

    void send_zb_to_read_basic_attribute_values(uint8_t src_endpoint, uint8_t dst_endpoint, uint16_t addr_short){
        /* Read peer Manufacture Name & Model Identifier */
        esp_zb_zcl_read_attr_cmd_t read_req;
        read_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
        read_req.zcl_basic_cmd.src_endpoint = src_endpoint;
        read_req.zcl_basic_cmd.dst_endpoint = dst_endpoint;
        read_req.zcl_basic_cmd.dst_addr_u.addr_short = addr_short;
        read_req.clusterID = ESP_ZB_ZCL_CLUSTER_ID_BASIC;

        // uint16_t attributes[] = {
        //     ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
        //     ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
        // };
        uint16_t attributes[] = {
            ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
        };
        read_req.attr_number = ARRAY_LENTH(attributes);
        read_req.attr_field = attributes;

        esp_zb_zcl_read_attr_cmd_req(&read_req);
    }

    void clear_remote_scene_devices_attribute_bind_values(uint8_t btn_index, uint8_t node_index, uint8_t ep_index, uint16_t short_addr){
        attr_data_info_t stt_attr_data = {0,0,0};
        existing_nodes_info[btn_index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].is_bind = 0;
        update_attr_data_in_nvs(short_addr, btn_index, ep_index, &stt_attr_data);
    }

    void set_remote_scene_devices_attribute_values(void * void_message){

        // attr_data_info_t stt_attr_data = {0,0,0}; 
        esp_zb_zcl_cmd_read_attr_resp_message_t *message = (esp_zb_zcl_cmd_read_attr_resp_message_t *)void_message;

        printf("--------READ ATTRIBUTES COMMAND HANDLER-----------\n");
        printf("dst_short_addr: 0x%x  dst_ep: %u  cluster_id: %u\n", message->info.src_address.u.short_addr, message->info.src_endpoint, message->info.cluster);
        
        uint32_t count = 0; 
        uint8_t btn_index = get_button_index(message->info.dst_endpoint); 
        uint8_t node_index = get_node_index(btn_index, message->info.src_address.u.short_addr);
        if(node_index != 0xff){
            uint8_t ep_index = get_ep_index(btn_index, node_index, message->info.src_endpoint);
            if(ep_index != 0xff){
                esp_zb_zcl_read_attr_resp_variable_t *variable = message->variables;
                while (variable != NULL) {
                    // vTaskDelay(pdMS_TO_TICKS(10));
                    if(count++ >= 300) break;  //wait for 3 sec
                    if (variable->status == ESP_ZB_ZCL_STATUS_SUCCESS) {
                        switch (variable->attribute.data.type) {
                            
                            case ESP_ZB_ZCL_ATTR_TYPE_BOOL:
                                printf("Attribute value (bool): %u\n", *(bool *)variable->attribute.data.value);
                                if(message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF){
                                    existing_nodes_info[btn_index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].data.state = *(bool *)variable->attribute.data.value;
                                }
                                break;                            
                            case ESP_ZB_ZCL_ATTR_TYPE_U8:
                                printf("Attribute value (uint8): %u\n", *(uint8_t *)variable->attribute.data.value);
                                if(message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL){
                                    existing_nodes_info[btn_index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].data.level = *(uint8_t *)variable->attribute.data.value;
                                }else if(message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL){
                                    existing_nodes_info[btn_index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].data.value = *(uint8_t *)variable->attribute.data.value;
                                }
                                break;
                            case ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM:
                                printf("Attribute value (uint8bit_enum): %u  Cluster:0x%x\n", *(uint8_t *)variable->attribute.data.value, message->info.cluster);
                                if(message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL){
                                    existing_nodes_info[btn_index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].data.level = *(uint16_t *)variable->attribute.data.value;
                                }else if(message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT){
                                    existing_nodes_info[btn_index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].data.level = *(uint16_t *)variable->attribute.data.value;
                                }else if(message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL || message->info.cluster == 0x010C || message->info.cluster == 0x010D){
                                    existing_nodes_info[btn_index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].data.mode = *(uint8_t *)variable->attribute.data.value;
                                }
                                //ESP_ZB_ZCL_ATTR_COLOR_CONTROL_ENHANCED_COLOR_MODE_ID, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_HUE_ID, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID
                                break;
                            case ESP_ZB_ZCL_ATTR_TYPE_U16:
                                printf("Attribute value (uint16): %u\n", *(uint16_t *)variable->attribute.data.value);
                                if(message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL){
                                    existing_nodes_info[btn_index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].data.value = *(uint16_t *)variable->attribute.data.value;
                                }else if(message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL){
                                    existing_nodes_info[btn_index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].data.level = *(uint8_t *)variable->attribute.data.value;
                                }
                                break;
                            case ESP_ZB_ZCL_ATTR_TYPE_S16:
                                printf("Attribute value (S16): %u\n", *(uint16_t *)variable->attribute.data.value);
                                if(message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT){
                                    if (variable->attribute.id == ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID) {
                                        existing_nodes_info[btn_index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].data.value = *(uint16_t *)variable->attribute.data.value;
                                    }
                                }
                                break;    
                            case ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING:
                                printf("Attribute value (string): %.*s\n", *(uint8_t *)variable->attribute.data.value,
                                    (char *)variable->attribute.data.value + 1);
                                
                                if (variable->attribute.id == ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID) {
                                    // Assuming variable->attribute.data.value points to a Zigbee string attribute
                                    uint8_t string_length = variable->attribute.data.size-1; // First byte indicates length
                                    // Copy the string, skipping the first byte
                                    //node_name
                                    memcpy(existing_nodes_info[btn_index].scene_switch_info.dst_node_info[node_index].node_name, (char *)variable->attribute.data.value + 1, string_length);
                                    // Ensure null termination for proper C string handling
                                    existing_nodes_info[btn_index].scene_switch_info.dst_node_info[node_index].node_name[string_length] = '\0';  // Add null terminator
                                }
                                // if (variable->attribute.id == ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID) {
                                //     // Assuming variable->attribute.data.value points to a Zigbee string attribute
                                //     uint8_t string_length = variable->attribute.data.size-1; // First byte indicates length
                                //     // Copy the string, skipping the first byte
                                    
                                //     memcpy(existing_nodes_info[btn_index].scene_switch_info.dst_node_info[node_index].node_name, (char *)variable->attribute.data.value + 1, string_length);
                                //     // Ensure null termination for proper C string handling
                                //     new_device.model_id[string_length] = '\0';  // Add null terminator
                                // }
                                break;
                                default: 
                                break;
                        }
                        variable = variable->next;   
                    }else{
                       if(count > 0 && count< 2){
                            printf("variable->status:%d\n", variable->status);
                       } 
                    }
                }
                printf("--------------------------------------------------\n");
                update_attr_data_in_nvs(message->info.src_address.u.short_addr, btn_index, ep_index, NULL);
                if(query_neighbour_task_handle != NULL)
                xTaskNotifyGive(query_neighbour_task_handle);
            }
        }
        cb_response_counts++; 
        if(cb_response_counts >= cb_requests_counts){
            //xSemaphoreGive(postHandlerSemaphore); 
        }
        printf("======================END==============================\n");       
    }
#else
    void set_remote_scene_devices_attribute_values(void * void_message) {
        
    }  
#endif

void waitForAllRecords() {
    if (recordsSemaphore != NULL) {
        // Wait for the semaphore to be given by the records task
        xSemaphoreTake(recordsSemaphore, portMAX_DELAY);
    }
}

uint8_t check_is_record_present(uint16_t short_addr){
    for(int i=0; i<MAX_NODES; i++){  
        if(nodes_info.scene_switch_info.dst_node_info[i].short_addr == short_addr){
            printf("FOUND SHORT ADDRESS:0x%x\n", short_addr);
            return i;
        }
    }
    
    return nodes_info.scene_switch_info.total_records;
}

void send_zcl_read_attr_request(uint16_t short_addr, uint16_t cluster_id, uint16_t attr_id, uint8_t dst_ep) {
    esp_zb_zcl_read_attr_cmd_t read_req = {0};

    // Initialize basic ZCL command information
    read_req.zcl_basic_cmd.src_endpoint = 0;
    read_req.zcl_basic_cmd.dst_endpoint = dst_ep;
    read_req.zcl_basic_cmd.dst_addr_u.addr_short = short_addr;
    // Set APS addressing mode
    read_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    // Set cluster ID
    read_req.clusterID = cluster_id;
    // Set attribute fields
    read_req.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;

    read_req.attr_number = 1;
    read_req.attr_field = &attr_id;

    read_req.dis_default_resp = 1; // Disable default response
    read_req.manuf_specific = 0;  // Not manufacturer-specific
    read_req.manuf_code = 0; // No manufacturer code

    // Send the ZCL Read Attributes Request
    esp_err_t err = esp_zb_zcl_read_attr_cmd_req(&read_req);
    if (err != ESP_OK) {
        ESP_LOGE("ZCL", "Failed to send read attribute request, error: %d", err);
    } else {
        ESP_LOGI("ZCL", "Sent Read Attribute Request for Cluster: 0x%04X, Attribute: 0x%04X", cluster_id, attr_id);
    }
}

void task_get_attribute_value(void* args) {
    uint16_t endpoint = *((uint16_t *)args);
    uint8_t state = *(uint8_t*)esp_zb_zcl_get_attribute(endpoint, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
        ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE, ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID)->data_p;

        esp_zb_zcl_read_attr_cmd_t read_req = {
        .zcl_basic_cmd = {
            .src_endpoint = 1,  // Your source endpoint
            .dst_endpoint = endpoint,  // Target endpoint
            .dst_addr_u.addr_short = 0,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
    };
    esp_zb_zcl_read_attr_cmd_req(&read_req);

    ESP_LOGI(TAG, " DEVICE STATE: %d", state);                                                
    vTaskDelete(NULL); // Delete the task after executing
}

#define REQUEST_TIMEOUT_MS          5000 // Timeout period (in milliseconds)
#define MAX_RETRY_COUNTS            3
uint8_t retry_counts                = 0;


typedef struct temp_index_s {
    uint8_t n_index;
    uint8_t ep_index;
}temp_index_t;


//temp_index_t t_index = {0, 0};

typedef struct {
    uint16_t short_addr;
    uint8_t ep_count;
    uint8_t current_ep_index;
    uint8_t ep_list[6];
    TaskHandle_t waiting_task;
    SemaphoreHandle_t completion_sem;
    temp_index_t t_index;
} descriptor_context_t;


typedef struct {
    uint16_t short_addr;
    uint8_t node_index;
    uint8_t current_ep_index;
    uint8_t bind_req_index;
    uint8_t clusters_count;
    uint8_t button_index;
    esp_zb_zdo_bind_req_param_t bind_req[5];
    //esp_zb_zdo_bind_req_param_t *bind_req; // ✅ Pointer for dynamic allocation
    size_t bind_req_size;
    TaskHandle_t waiting_task;
    SemaphoreHandle_t completion_sem;
    dst_node_info_t dst_node_info;
} descriptor_binding_context_t;


static void process_next_endpoint(descriptor_context_t *ctx);

static void simple_desc_cb(esp_zb_zdp_status_t zdo_status, esp_zb_af_simple_desc_1_1_t *simple_desc, void *user_ctx)
{
    uint8_t clusters_count = 0; 
    //temp_index_t* temp_index = (temp_index_t*)user_ctx;  

    descriptor_context_t *ctx = (descriptor_context_t *)user_ctx;
    
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Simple desc response: status(%d), device_id(0x%x), app_version(%d), profile_id(0x%x), endpoint_ID(%d)", zdo_status,
                 simple_desc->app_device_id, simple_desc->app_device_version, simple_desc->app_profile_id, simple_desc->endpoint);
        
        
        for (int i = 0; i < (simple_desc->app_input_cluster_count + simple_desc->app_output_cluster_count); i++) {
            uint16_t cluster = *(simple_desc->app_cluster_list + i);   
            ESP_LOGI(TAG, "CLUSTER: 0x%x", cluster);  

            taskYIELD(); // Allow other tasks to run
            switch(simple_desc->app_device_id){
				case ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID:        //OnOff Switch   
					switch(cluster){  
						case ESP_ZB_ZCL_CLUSTER_ID_ON_OFF:
                            nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].dst_ep_info.ep_data[ctx->t_index.ep_index].cluster_id[clusters_count++] = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
                            strcpy(nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].node_name, zb_node_names[4]);
							break;
                        default:                       
                        break;
					}
					break;
                case ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID:         // OnOff Light Device 
                    switch(cluster){
                        case ESP_ZB_ZCL_CLUSTER_ID_ON_OFF:
                        nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].dst_ep_info.ep_data[ctx->t_index.ep_index].cluster_id[clusters_count++] = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
                        strcpy(nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].node_name, zb_node_names[0]);            
                            break;
                        case ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL:
                        nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].dst_ep_info.ep_data[ctx->t_index.ep_index].cluster_id[clusters_count++] = ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL;
                        strcpy(nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].node_name, zb_node_names[1]);            
                            break;                        
                        case ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL:
                        nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].dst_ep_info.ep_data[ctx->t_index.ep_index].cluster_id[clusters_count++] = ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL;
                        strcpy(nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].node_name, zb_node_names[2]);            
                            break;                            
                        default:                        
                        break;
                    }
                    break;
                    
                case ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID:       //Dimmer Light
                    switch(cluster){
                        case ESP_ZB_ZCL_CLUSTER_ID_ON_OFF:
                        nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].dst_ep_info.ep_data[ctx->t_index.ep_index].cluster_id[clusters_count++] = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
                        if(nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].endpoint_counts == 1){
                            strcpy(nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].node_name, zb_node_names[10]);
                        }else{
                            strcpy(nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].node_name, zb_node_names[1]);
                        }
                                       
                        break;                        
                    case ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL:
                        nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].dst_ep_info.ep_data[ctx->t_index.ep_index].cluster_id[clusters_count++] = ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL;
                        if(nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].endpoint_counts == 1){
                            strcpy(nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].node_name, zb_node_names[10]);
                        }else{
                            strcpy(nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].node_name, zb_node_names[1]);
                        }            
                        break;
                    default:                       
                        break;
                    }
                    break;
                    
                case 0x010C: //CCT Light
                    switch(cluster){
                        case ESP_ZB_ZCL_CLUSTER_ID_ON_OFF:
                        nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].dst_ep_info.ep_data[ctx->t_index.ep_index].cluster_id[clusters_count++] = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
                        strcpy(nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].node_name, zb_node_names[12]);                
                        break;                        
                        case ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL:
                        nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].dst_ep_info.ep_data[ctx->t_index.ep_index].cluster_id[clusters_count++] = ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL;
                        strcpy(nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].node_name, zb_node_names[12]);                
                        break;                        
                        case ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL:
                        nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].dst_ep_info.ep_data[ctx->t_index.ep_index].cluster_id[clusters_count++] = ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL;
                        strcpy(nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].node_name, zb_node_names[12]);                
                        break;   
                        // case 0xEF00:
                        // nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].dst_ep_info.cluster_id[clusters_count++] = 0xE100;
                        // strcpy(nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].node_name, zb_node_names[12]);                
                        // break;                              
                        default:                        
                        break;
                    }  
                    break; 

                case 0x010D:
                case ESP_ZB_HA_COLOR_DIMMABLE_LIGHT_DEVICE_ID: //RGB Light
                    switch(cluster){
                        case ESP_ZB_ZCL_CLUSTER_ID_ON_OFF:
                        nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].dst_ep_info.ep_data[ctx->t_index.ep_index].cluster_id[clusters_count++] = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
                        strcpy(nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].node_name, zb_node_names[2]);                
                            break;                        
                        case ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL:
                        nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].dst_ep_info.ep_data[ctx->t_index.ep_index].cluster_id[clusters_count++] = ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL;
                        strcpy(nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].node_name, zb_node_names[2]);                
                            break;                        
                        case ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL:
                        nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].dst_ep_info.ep_data[ctx->t_index.ep_index].cluster_id[clusters_count++] = ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL;
                        strcpy(nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].node_name, zb_node_names[2]);                
                            break;   
                            
                        default:                        
                        break;
                    }
                    break;

                case ESP_ZB_HA_THERMOSTAT_DEVICE_ID:  		   //1Light-1Fan
                    switch(cluster){
                        case ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL:
                        nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].dst_ep_info.ep_data[ctx->t_index.ep_index].cluster_id[clusters_count++] = ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL;
                        strcpy(nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].node_name, zb_node_names[3]);               
                            break;
                        case ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT:
                        nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].dst_ep_info.ep_data[ctx->t_index.ep_index].cluster_id[clusters_count++] = ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT;
                        strcpy(nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].node_name, zb_node_names[10]);                                      
                            break;                            
                        default:                       
                        break;
                    }
                    break;

                case 0x0106: 								   //Illuminance Sensor
                    switch(cluster){
                        case ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT:
                        nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].dst_ep_info.ep_data[ctx->t_index.ep_index].cluster_id[clusters_count++] = ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT;
                        strcpy(nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].node_name, zb_node_names[6]);                                        
                            break;
                        default:                        
                        break;
                    }
                    break;

                case ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID:   //Temperature & Humidity Sensor
                    switch(cluster){
                        case ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT:
                        nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].dst_ep_info.ep_data[ctx->t_index.ep_index].cluster_id[clusters_count++] = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT;
                        strcpy(nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].node_name, zb_node_names[7]);                                 
                            break;
                        case ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT:
                        nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].dst_ep_info.ep_data[ctx->t_index.ep_index].cluster_id[clusters_count++] = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT;
                        strcpy(nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].node_name, zb_node_names[8]);                                        
                            break;
                        default:                       
                        break;
                    }
                    break;   
                case ESP_ZB_HA_IAS_ZONE_ID:  				   //contact switch, motion sensor, gas leak
                    switch(cluster){
                        case ESP_ZB_ZCL_CLUSTER_ID_IAS_ZONE:  
                        nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].dst_ep_info.ep_data[ctx->t_index.ep_index].cluster_id[clusters_count++] = ESP_ZB_ZCL_CLUSTER_ID_IAS_ZONE;
                        strcpy(nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].node_name, zb_node_names[9]);                                  
                            break;
                        default:                      
                        break;
                    }
                    break;
                case ESP_ZB_HA_SCENE_SELECTOR_DEVICE_ID:       //zigbee scene switch (Not Tested yet)
                    nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].dst_ep_info.ep_data[ctx->t_index.ep_index].cluster_id[clusters_count++] = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
                    strcpy(nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].node_name, zb_node_names[5]);
                    break;
                case ESP_ZB_HA_REMOTE_CONTROL_DEVICE_ID:       //wireless remote switch (Actually running on site)
                    switch(cluster){
                        case ESP_ZB_ZCL_CLUSTER_ID_ON_OFF:
                        nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].dst_ep_info.ep_data[ctx->t_index.ep_index].cluster_id[clusters_count++] = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
                        strcpy(nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].node_name, zb_node_names[5]);                                                                              
                        break;
                        default:                        
                        break;
                    }
                    break;
                case ESP_ZB_HA_HEATING_COOLING_UNIT_DEVICE_ID: //zigbee Thermostat (Not Tested yet)
                    switch(cluster){
                        case ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT:
                        nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].dst_ep_info.ep_data[ctx->t_index.ep_index].cluster_id[clusters_count++] = ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT;
                        strcpy(nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].node_name, zb_node_names[10]);                         
                        break;
                        default:                     
                        break;
                    }
                    break;
                default: 
                    nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].dst_ep_info.ep_data[ctx->t_index.ep_index].cluster_id[clusters_count++] = ESP_ZB_ZCL_CLUSTER_ID_DIAGNOSTICS;
                    strcpy(nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].node_name, zb_node_names[11]); 
                    break;
            }
        }
        nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].dst_ep_info.ep_data[ctx->t_index.ep_index].get_cluster = 1;
        nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].dst_ep_info.ep_data[ctx->t_index.ep_index].clusters_count = clusters_count;

        ctx->t_index.ep_index++;
        ctx->current_ep_index++;
    }else {
        //no clusters found
        ESP_LOGE(TAG, "simple_desc_cb failed with status: 0x%x", zdo_status);  
        nodes_info.scene_switch_info.dst_node_info[ctx->t_index.n_index].dst_ep_info.ep_data[ctx->t_index.ep_index].get_cluster = 2; //error
        if(retry_counts++ > MAX_RETRY_COUNTS){
            retry_counts = 0;
            ESP_LOGE(TAG, "RETRY FAILED!!"); 
            ctx->t_index.ep_index++;
            ctx->current_ep_index++;            
        }else{
            ESP_LOGE(TAG, "TRYING AGAIN...Keep Patience!"); 
        }
    }
    // Trigger next endpoint processing
    process_next_endpoint(ctx);
}

static void process_next_endpoint(descriptor_context_t *ctx) {
    if (ctx->current_ep_index >= ctx->ep_count) {
        // All endpoints processed
        xSemaphoreGive(ctx->completion_sem);
        return;
    }

    uint8_t endpoint = ctx->ep_list[ctx->current_ep_index];
    esp_zb_zdo_simple_desc_req_param_t req = {
        .addr_of_interest = ctx->short_addr,
        .endpoint = endpoint
    };
    taskYIELD(); // Allow other tasks to run
    esp_zb_zdo_simple_desc_req(&req, simple_desc_cb, ctx); 
}

uint8_t ep_cb_retry_counts = 0;
/* 2F90 - near dinning
   442C - curtain 
   5E22 - On Off Bell
   82D0 - Fan
   83AC - dim
   1l1F - Master bedroom - 0x9A50*/
   
static void ep_cb(esp_zb_zdp_status_t zdo_status, uint8_t ep_count, uint8_t *ep_id_list, void *user_ctx)
{
    #define MAX_EP_CB_RETRIES           3
    descriptor_context_t *ctx = (descriptor_context_t *)user_ctx;
    if (ctx == NULL) {
        ESP_LOGE(TAG, "ep_cb received NULL ctx! Aborting.");
        if(ctx->completion_sem == NULL) ESP_LOGE(TAG, "OOPs Its NULL");
        xSemaphoreGive(ctx->completion_sem);
        return;
    }    
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {

        if(ep_count == 0) { 
            ESP_LOGE(TAG, "ep_count == 0");
            xSemaphoreGive(ctx->completion_sem); 
            return;
        }

        uint8_t n_index = check_is_record_present(ctx->short_addr);
        if(n_index != 0xff){
            ESP_LOGI(TAG, "Active endpoint response: status(%d) ep_count(%d) short_addr(0x%x)", zdo_status, ep_count, ctx->short_addr);
        }

        ESP_LOGI(TAG, "ep_cb: ctx=%p, short_addr=0x%04x, n_index=%d", ctx, ctx ? ctx->short_addr : 0, n_index);
        uint8_t l_ep_counts = 0;
        
        char* name = {0};    

        if(ctx->completion_sem == NULL) ESP_LOGE(TAG, "OOPs Its NULL");
        if(n_index == 0xff){ESP_LOGE(TAG, "n_index == 0xff");  xSemaphoreGive(ctx->completion_sem); return;} 
        nodes_info.scene_switch_info.dst_node_info[n_index].endpoint_counts = ep_count;
        ESP_LOGE(TAG, "==========>ep_count:%d\n", ep_count);
        for (int jk = 0; jk < ep_count; jk++) {
            if(ep_id_list[jk] == 242) {
                nodes_info.scene_switch_info.dst_node_info[n_index].endpoint_counts = ep_count - 1;
                break;
            }
        }   
        
        for (int i = 0; i < ep_count; i++) {
            if(ep_id_list[i] != 242) {
                if(n_index != 0xff){
                    nodes_info.scene_switch_info.dst_node_info[n_index].dst_ep_info.ep_data[l_ep_counts].dst_ep = ep_id_list[i];
                    char buf[4];
                    sprintf(buf, "%02d", (l_ep_counts));
                    strncpy(&str_name[6], buf, 3);
                    strncpy(nodes_info.scene_switch_info.dst_node_info[n_index].dst_ep_info.ep_data[l_ep_counts].ep_name, str_name, strlen(str_name));
                    l_ep_counts++;                                             
                } 
            } 
        }

        if(n_index != 0xff){
            ctx->ep_count = nodes_info.scene_switch_info.dst_node_info[n_index].endpoint_counts;
            ESP_LOGE(TAG, "==========>ep_count2:%d ep_count3:%d\n", l_ep_counts, ctx->ep_count);
            ctx->ep_list[0] = ep_id_list[0];
            ctx->ep_list[1] = ep_id_list[1];
            ctx->ep_list[2] = ep_id_list[2];
            ctx->ep_list[3] = ep_id_list[3];
            ctx->current_ep_index = 0;
            ctx->t_index.n_index = n_index; 
            ctx->t_index.ep_index = 0;
            retry_counts = 0;
            // Start processing first endpoint
            process_next_endpoint(ctx);
        }
    } else {
        //MAX_EP_CB_RETRIES
        ESP_LOGE(TAG, "ep_cb failed with status: 0x%x", zdo_status);
        if(ep_cb_retry_counts++ > 3){
            ep_cb_retry_counts = 0;
            xSemaphoreGive(ctx->completion_sem);
        } else{
            esp_zb_zdo_active_ep_req_param_t active_ep_req = {
                .addr_of_interest = ctx->short_addr
            };
            
            esp_zb_zdo_active_ep_req(&active_ep_req, ep_cb, ctx);
        }         
        
    }
}

uint8_t depth[30];
uint8_t lqi[30];

int is_short_address_present(uint16_t short_addr){
    for(int btn_index=0; btn_index<TOTAL_ENDPOINTS;btn_index++){
        for(int node_index=0; node_index < existing_nodes_info[btn_index].scene_switch_info.total_records; node_index++){
            if(existing_nodes_info[btn_index].scene_switch_info.dst_node_info[node_index].short_addr == short_addr){
                return node_index;
            }
        }
    }
    return -1;
}

static int total_nodes = 0;

bool is_short_address_already_exists(uint16_t short_addr, int l_node_cnts){
    if(l_node_cnts == 0) return false;
    if(existing_nodes_info[src_endpoint_index].scene_switch_info.dst_node_info[src_endpoint_index].short_addr == 0) return true;
    for(int i=0; i<l_node_cnts; i++){
        if(existing_nodes_info[src_endpoint_index].scene_switch_info.dst_node_info[i].short_addr == short_addr){
            return true;
        } 
    }
    return false;
}

bool is_short_address_local_struct_already_exists(uint16_t short_addr, int l_node_cnts){
    for(int i=0; i<l_node_cnts; i++){
        //ESP_LOGW(TAG, "SHORT_ADDR: 0x%x\n", nodes_info.scene_switch_info.dst_node_info[i].short_addr);
        if(nodes_info.scene_switch_info.dst_node_info[i].short_addr == short_addr){
            return true;
        } 
    }
    return false;
}

static void lqi_callback(const esp_zb_zdo_mgmt_lqi_rsp_t *rsp, void *user_ctx) {
    if (!rsp || rsp->status != ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGW(TAG, "LQI request failed or invalid response");
        return;
    }

    ESP_LOGI(TAG, "Total Neighbors: %d | Received: %d | Start Index: %d",
             rsp->neighbor_table_entries, rsp->neighbor_table_list_count, rsp->start_index);

    total_nodes = *(int*)user_ctx;
    printf("TOTAL_NODES:%d\n", total_nodes);
    for (int i = 0; i < rsp->neighbor_table_list_count; i++) {
        esp_zb_zdo_neighbor_table_list_record_t *neighbor = &rsp->neighbor_table_list[i];
        if(neighbor->network_addr != 0 && neighbor->network_addr != esp_zb_get_short_address()){
            if(!is_short_address_already_exists(neighbor->network_addr, total_nodes)){
                ESP_LOGI(TAG, "Neighbor %d - Short Address: 0x%04X, Device Type: %02X", i, neighbor->network_addr, neighbor->device_type);
                nodes_info.scene_switch_info.dst_node_info[node_counts].short_addr = neighbor->network_addr;
                memcpy(nodes_info.scene_switch_info.dst_node_info[node_counts].ieee_addr, neighbor->extended_addr, sizeof(esp_zb_ieee_addr_t));
                nodes_info.scene_switch_info.dst_node_info[node_counts].device_role = neighbor->device_type;
                node_counts++;  
            } else{
                ESP_LOGI(TAG, "Short Address: 0x%04X already exists!!\n", neighbor->network_addr);
            }  
        }            
    }

    // 🔹 Request next batch if there are more neighbors to fetch
    if ((rsp->start_index + rsp->neighbor_table_list_count) < rsp->neighbor_table_entries) {
        esp_zb_zdo_mgmt_lqi_req_param_t next_req;
        next_req.dst_addr = 0x0000;  // Query Coordinator
        next_req.start_index = rsp->start_index + rsp->neighbor_table_list_count;

        ESP_LOGI(TAG, "Requesting next batch starting at index: %d", next_req.start_index);
        esp_zb_zdo_mgmt_lqi_req(&next_req, lqi_callback, &total_nodes);
    }else{
        xTaskNotifyGive(query_neighbour_task_handle);
    }
}
  
void send_lqi_request(uint16_t target_short_addr, uint8_t erase_data) {
    esp_zb_zdo_mgmt_lqi_req_param_t lqi_req;
    lqi_req.dst_addr = target_short_addr; // Usually 0x0000 for Coordinator
    lqi_req.start_index = 0;  // Start from the first entry
    if(erase_data != 0) node_counts = 0;
    esp_zb_zdo_mgmt_lqi_req(&lqi_req, lqi_callback, &node_counts);
}
        
SemaphoreHandle_t completion_sem = NULL;

static void find_zigbee_nodes_task(void* args) {
    uint8_t erase_data = *(uint8_t*)args;
    printf("Scanning All Nodes on Network... SRC_EP=%d\n", ENDPOINTS_LIST[src_endpoint_index]);
    query_neighbour_task_handle = xTaskGetCurrentTaskHandle();
    esp_zb_nwk_neighbor_info_t nbr_info;
    esp_zb_nwk_info_iterator_t iterator = ESP_ZB_NWK_INFO_ITERATOR_INIT;
    esp_err_t err = 0;
    
    nodes_info.scene_id = global_scene_id[src_endpoint_index];
    nodes_info.scene_switch_info.src_ep = ENDPOINTS_LIST[src_endpoint_index];    
    printf("NODE COUNTS1:%d\n", node_counts);
    send_lqi_request(0x0000, erase_data);
    printf("NODE COUNTS2:%d\n", node_counts);
    uint32_t is_notified= ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(12000));
    if (is_notified > 0) {
        ESP_LOGI("Zigbee", "esp_zb_zdo_active_ep_req Notified OK!!");
    } else {
        ESP_LOGI("Zigbee", "esp_zb_zdo_active_ep_req Notified Timeout!!");
    }      
    int l_node_counts = node_counts;

    printf("NODE COUNTS2:%d\n", node_counts);

    while ((esp_zb_nwk_get_next_neighbor(&iterator, &nbr_info)) == ESP_OK) {
        // Process neighbor info
        if(nbr_info.short_addr != 0 && nbr_info.short_addr != esp_zb_get_short_address()){
            //if(erase_data == 0){
            if(!is_short_address_local_struct_already_exists(nbr_info.short_addr, l_node_counts)) {
                nodes_info.scene_switch_info.dst_node_info[node_counts].short_addr = nbr_info.short_addr;
                memcpy(nodes_info.scene_switch_info.dst_node_info[node_counts].ieee_addr, nbr_info.ieee_addr, sizeof(esp_zb_ieee_addr_t));
                nodes_info.scene_switch_info.dst_node_info[node_counts].device_role = nbr_info.device_type;
                node_counts++;
            }
        }
    }

    printf("NODE COUNTS3:%d\n", node_counts);
    nodes_info.scene_switch_info.total_records = node_counts;
    printf("----------NEIGHBOURS RECORD FOUND----------------\n");
    printf("total_nodes:%d\n", node_counts);

    for(int node_index=0; node_index<node_counts; node_index++){
        esp_zb_zdo_active_ep_req_param_t active_ep_req;
        
        active_ep_req.addr_of_interest = nodes_info.scene_switch_info.dst_node_info[node_index].short_addr;
        printf(".....................%d/%d.........................\n", node_index+1, node_counts);
        printf("short_addr_query: 0x%04x, src_ep:%d\n", active_ep_req.addr_of_interest, ENDPOINTS_LIST[src_endpoint_index]);

        uint8_t node_already_exists = 0;
        if(erase_data == 0){
            if(is_short_address_local_struct_already_exists(active_ep_req.addr_of_interest, node_counts)){
                node_already_exists = 1;
            }
        }

        if(node_already_exists == 0){
            // Create semaphore
            completion_sem = xSemaphoreCreateBinary();
            if (completion_sem == NULL) {
                ESP_LOGE(TAG, "Failed to create semaphore");
                continue;
            }

            // Create a context
            descriptor_context_t *ctx = malloc(sizeof(descriptor_context_t));
            if (ctx == NULL) {
                ESP_LOGE(TAG, "Failed to allocate memory for context");
                vSemaphoreDelete(completion_sem);
                continue;
            }
            ctx->short_addr = nodes_info.scene_switch_info.dst_node_info[node_index].short_addr;
            ctx->completion_sem = completion_sem;
            // Start discovery process
            esp_zb_zdo_active_ep_req_param_t active_ep_req = {
                .addr_of_interest = ctx->short_addr
            };
            ep_cb_retry_counts = 0;
            esp_zb_zdo_active_ep_req(&active_ep_req, ep_cb, ctx);

            // Wait for completion (with 15s timeout)
            if (xSemaphoreTake(ctx->completion_sem, pdMS_TO_TICKS(20000))) {
                ESP_LOGI(TAG, "Discovery completed successfully");
            } else {
                ESP_LOGE(TAG, "Discovery timed out");
            }
            vSemaphoreDelete(completion_sem);
            free(ctx);
            // Optional: delay between requests to prevent flooding the stack
            //vTaskDelay(pdMS_TO_TICKS(50));
            taskYIELD(); // Allow other tasks to run
        }              
    }
    //
    printf("-------------------------------------------------\n");
    //vTaskDelay(pdMS_TO_TICKS(2000));
    // Signal that record processing is complete
    if(recordsSemaphore != NULL) xSemaphoreGive(recordsSemaphore);
    vTaskDelete(NULL); // Delete the task after executing
}


static void save_nodes_task(void* args) {
    // Save nodes_info to NVS
    printf("---->NODE_COUNTS:%d\n", node_counts);
    if(node_counts > 0){
        uint16_t total = store_new_nodes(&nodes_info, src_endpoint_index);
        printf("---->NODE_COUNTS_2:%d, TOTAL:%d\n", node_counts, total);
    }
    vTaskDelete(NULL); // Delete the task after executing
}

uint8_t nuos_find_active_nodes(uint8_t src_index, uint8_t total_nodes, uint8_t erase_data){
    // Allocate memory to pass the value safely
    find_active_neighbours_task_flag = true;
    src_endpoint_index = src_index;
    total_neighbour_records_devices_count = 0;
    start_led_blinking(2, src_index);
    recordsSemaphore = xSemaphoreCreateBinary();

    if (recordsSemaphore == NULL) {
        // Handle semaphore creation failure
        ESP_LOGE(TAG, "Failed to create semaphore");
        return ESP_FAIL;
    }
    node_counts = total_nodes;  
    printf("---->NODE_COUNTS_1:%d\n", node_counts);
    xTaskCreate(find_zigbee_nodes_task, "find_node_task", TASK_STACK_SIZE_ZB_FIND_NODES, &erase_data, TASK_PRIORITY_ZB_FIND_NODES, NULL); 

    
    if (recordsSemaphore != NULL) {
        // Wait for the semaphore to be given by the records task
        xSemaphoreTake(recordsSemaphore, portMAX_DELAY);
    }

    
    printf("----------------TOTAL_NODES_FOUND:%d---------------\n", node_counts); //
    //taskYIELD();

    // recordsSemaphore = xSemaphoreCreateBinary();

    // if (recordsSemaphore == NULL) {
    //     // Handle semaphore creation failure
    //     ESP_LOGE(TAG, "Failed to create semaphore");
    //     return ESP_FAIL;
    // }

   // xTaskCreate(save_nodes_task, "save_nodes_task", TASK_STACK_SIZE_SAVE_NODES, &node_counts, TASK_PRIORITY_ZB_SAVE_NODES, NULL); //    
    if(node_counts > 0){
        uint16_t total = store_new_nodes(&nodes_info, src_endpoint_index);
        printf("---->NODE_COUNTS_2:%d, TOTAL:%d\n", node_counts, total);
    }
    // if (recordsSemaphore != NULL) {
    //     // Wait for the semaphore to be given by the records task
    //     xSemaphoreTake(recordsSemaphore, portMAX_DELAY); //
    // }

    //if(total_nodes == 0)
    //xTaskCreate(save_nodes_task, "save_nodes_task", TASK_STACK_SIZE_SAVE_NODES, NULL, TASK_PRIORITY_ZB_SAVE_NODES, NULL); 
    // // Save nodes_info to NVS
    // if(node_counts > 0){
    //     uint16_t total = store_new_nodes(&nodes_info, src_index);
    //     printf("---->NODE_COUNTS_2:%d, TOTAL:%d\n", node_counts, total); 
    // }  
    //esp_stop_timer();  //stop showing led touchpad blinking
    find_active_neighbours_task_flag = false;
    //task_sequence_num = TASK_MODE_BLINK_OTHER_LEDS;
    task_sequence_num = TASK_MODE_EXIT;
    stop_led_blinking();
    return node_counts;
}

/* ------------------------------------------------------------------- */
static void bind_cb_3(esp_zb_zdp_status_t zdo_status, void *user_ctx)
{  
    descriptor_binding_context_t *ctx = (descriptor_binding_context_t *)user_ctx;
    if (ctx == NULL) {
        ESP_LOGE(TAG, "bind_cb_3 received NULL ctx! Aborting.");
        if(ctx->completion_sem == NULL) ESP_LOGE(TAG, "OOPs Its NULL");
        xSemaphoreGive(ctx->completion_sem);
        return;
    }

    if (user_ctx == NULL) {
        return;
    }else{
        // printf("============>bind_req_index:%d\n", ctx->bind_req_index); 
        // printf("SRC_EP: %d DST_EP: %d SHORT: 0x%x\n", ctx->bind_req[ctx->bind_req_index].src_endp, ctx->bind_req[ctx->bind_req_index].dst_endp, ctx->bind_req[ctx->bind_req_index].dst_address_u.addr_short);
        // printf("CLUSTER_ID: 0x%x   src_endpoint_index:%d\n", ctx->bind_req[ctx->bind_req_index].cluster_id, src_endpoint_index); 
        // printf("req_dst_addr: 0x%x\n", ctx->bind_req[ctx->bind_req_index].req_dst_addr); 
        // printf("node_index: 0x%x\n", ctx->node_index);        
    }
    
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        printf("Binding Success Ok!!\n"); // dst_ep_data_t

        existing_nodes_info[src_endpoint_index].scene_switch_info.dst_node_info[ctx->node_index].dst_ep_info.ep_data[ctx->bind_req_index].is_bind = 1;
        existing_nodes_info[src_endpoint_index].scene_switch_info.src_ep = ctx->bind_req[ctx->bind_req_index].src_endp;
        
        update_binding_data_in_nvs(existing_nodes_info[src_endpoint_index].scene_switch_info.dst_node_info[ctx->node_index].short_addr,
            src_endpoint_index, ctx->current_ep_index, 1);           

    }else{
        existing_nodes_info[src_endpoint_index].scene_switch_info.dst_node_info[ctx->node_index].dst_ep_info.ep_data[ctx->bind_req_index].is_bind = 0;
        printf("Binding Failed!! 0x%x\n", zdo_status);
    }
    //printf("bind_req_index:%d clusters_count:%d\n", ctx->bind_req_index, ctx->clusters_count);
    if(ctx->bind_req_index >= ctx->clusters_count-1) {
        //printf("End of Cluster Counts!!\n");
        uint8_t node_i = ctx->node_index;
        uint8_t ep_i = ctx->current_ep_index;
        nuos_attribute_get_request(src_endpoint_index, ep_i, &existing_nodes_info[src_endpoint_index].scene_switch_info.dst_node_info[node_i], 0xff);
        if(completion_sem != NULL){
            xSemaphoreGive(ctx->completion_sem);
        }else{
            ESP_LOGI("Zigbee", "completion_sem = NULL");
        }            
    }else{
        ctx->bind_req_index++;
        //if(nodes_info.scene_switch_info.dst_node_info[ctx->node_index].dst_ep_info.ep_data[ctx->current_ep_index].is_bind == 0){
            esp_zb_zdo_device_bind_req(&ctx->bind_req[ctx->bind_req_index], bind_cb_3, ctx);
        //}
        //printf("==========bind_req_index:%d==========n", ctx->bind_req_index);
                
    }
}

static void bind_cb_4(esp_zb_zdp_status_t zdo_status, void *user_ctx)
{  
    descriptor_binding_context_t *ctx = (descriptor_binding_context_t *)user_ctx;
    if (ctx == NULL) {
        ESP_LOGE(TAG, "bind_cb_4 received NULL ctx! Aborting.");
        if(ctx->completion_sem == NULL) ESP_LOGE(TAG, "OOPs Its NULL");
        xSemaphoreGive(ctx->completion_sem);
        return;
    }

    if (user_ctx) {
        printf("============>bind_req_index:%d\n", ctx->bind_req_index); 
        printf("SRC_EP: %d DST_EP: %d SHORT: 0x%x\n", ctx->bind_req[ctx->bind_req_index].src_endp, ctx->bind_req[ctx->bind_req_index].dst_endp, ctx->bind_req[ctx->bind_req_index].dst_address_u.addr_short);
        printf("CLUSTER_ID: 0x%x   src_endpoint_index:%d\n", ctx->bind_req[ctx->bind_req_index].cluster_id, src_endpoint_index); 
        printf("req_dst_addr: 0x%x\n", ctx->bind_req[ctx->bind_req_index].req_dst_addr); 
        printf("node_index: 0x%x\n", ctx->node_index); 
        
    }
    
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        printf("Binding Success Ok!!\n"); // dst_ep_data_t
        existing_nodes_info[src_endpoint_index].scene_switch_info.dst_node_info[ctx->node_index].dst_ep_info.ep_data[ctx->bind_req_index].is_bind = 0;
        existing_nodes_info[src_endpoint_index].scene_switch_info.src_ep = ctx->bind_req[ctx->bind_req_index].src_endp;
        update_binding_data_in_nvs(existing_nodes_info[src_endpoint_index].scene_switch_info.dst_node_info[ctx->node_index].short_addr,
            src_endpoint_index, ctx->current_ep_index, 1);

    }else{
        existing_nodes_info[src_endpoint_index].scene_switch_info.dst_node_info[ctx->node_index].dst_ep_info.ep_data[ctx->bind_req_index].is_bind = 0;
        printf("Binding Failed!! 0x%x\n", zdo_status);
    }
    if(ctx->bind_req_index >= ctx->clusters_count-1) {
        if(completion_sem != NULL){
            xSemaphoreGive(ctx->completion_sem);
        }            
    }else{
        ctx->bind_req_index++;
        if(nodes_info.scene_switch_info.dst_node_info[ctx->node_index].dst_ep_info.ep_data[ctx->current_ep_index].is_bind == 1){
            esp_zb_zdo_device_unbind_req(&ctx->bind_req[ctx->bind_req_index], bind_cb_4, ctx);
        }
        printf("==========unbind_req_index:%d==========n", ctx->bind_req_index);       
    }
}

void bind_zigbee_nodes_task(void* args) {
    dst_node_info_t* d_node_info = (dst_node_info_t*)args;

    //printf("--------------------endpoint_counts:%d--------------------\n", d_node_info[0].endpoint_counts);
    for(int n_index=0; n_index<node_counts; n_index++){
        if(d_node_info[n_index].short_addr != 0){
            //printf("--------------------short_addr:%d--------------------\n", d_node_info[n_index].short_addr);  
            for(int j=0; j<d_node_info[n_index].endpoint_counts; j++){ 
                if(d_node_info[n_index].dst_ep_info.ep_data[j].is_bind == 1){
                    printf("==================================\n"); 
                    printf("BINDING FOUND AT SHORT_ADDR: 0x%x EP:%d\n", 
                        d_node_info[n_index].short_addr, 
                        d_node_info[n_index].dst_ep_info.ep_data[j].dst_ep);
                    
                    //printf("clusters_count: %d\n", d_node_info[n_index].dst_ep_info.ep_data[j].clusters_count); 
                    
                    // Create semaphore
                    completion_sem = xSemaphoreCreateBinary();
                    if (completion_sem == NULL) {
                        ESP_LOGE(TAG, "Failed to create semaphore");
                        continue;
                    }

                    // Create a context
                    descriptor_binding_context_t *ctx = malloc(sizeof(descriptor_binding_context_t));
                    if (ctx == NULL) {
                        ESP_LOGE(TAG, "Failed to allocate memory for context");
                        vSemaphoreDelete(completion_sem);
                        continue;
                    }
                    ctx->node_index = n_index;
                    ctx->short_addr = d_node_info[n_index].short_addr;
                    ctx->completion_sem = completion_sem;
                    // Start Binding process
                    ep_cb_retry_counts = 0;
                    ctx->bind_req_index = 0;
                    ctx->current_ep_index = j;
                    
                    ctx->button_index = nodes_info.scene_id;
                    ctx->clusters_count = d_node_info[ctx->node_index].dst_ep_info.ep_data[j].clusters_count;
                    if(d_node_info[ctx->node_index].dst_ep_info.ep_data[j].clusters_count > 0 && d_node_info[ctx->node_index].dst_ep_info.ep_data[j].clusters_count < 5){
                        //ESP_LOGI(TAG, "===========>Send Bind Light Request...NODE_INDEX:%d\n", ctx->node_index);
                        size_t required_size = d_node_info[ctx->node_index].dst_ep_info.ep_data[j].clusters_count;

                        ctx->bind_req_size = required_size; // Save the allocated size
                        for(int k=0; k<d_node_info[ctx->node_index].dst_ep_info.ep_data[j].clusters_count; k++){

                            esp_zb_ieee_addr_t src_ieee_addr;
                            esp_zb_ieee_address_by_short(d_node_info[ctx->node_index].short_addr, src_ieee_addr);
                            esp_zb_get_long_address(ctx->bind_req[k].src_address);
                            ctx->bind_req[k].src_endp = ENDPOINTS_LIST[src_endpoint_index];
                                    
                            ctx->bind_req[k].dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
                            memcpy(ctx->bind_req[k].dst_address_u.addr_long, src_ieee_addr, sizeof(esp_zb_ieee_addr_t));
                            ctx->bind_req[k].dst_endp = d_node_info[ctx->node_index].dst_ep_info.ep_data[j].dst_ep;
                            ctx->bind_req[k].dst_address_u.addr_short = d_node_info[ctx->node_index].short_addr;        
                            ctx->bind_req[k].req_dst_addr = esp_zb_get_short_address();            
                            ctx->bind_req[k].cluster_id = d_node_info[ctx->node_index].dst_ep_info.ep_data[j].cluster_id[k];

   
                        } //for
                        ep_cb_retry_counts = 0;
                        
                        if(d_node_info[ctx->node_index].dst_ep_info.ep_data[ctx->current_ep_index].is_bind == 1){
                            printf("======>SEND NODE[%d] BINDING REQUEST------------------------", ctx->node_index);
                            memcpy(&ctx->dst_node_info, &d_node_info[ctx->node_index], sizeof(dst_node_info_t));
                            esp_zb_zdo_device_bind_req(&ctx->bind_req[ctx->bind_req_index], bind_cb_3, ctx);
                            //nuos_attribute_get_request(src_endpoint_index, j, &d_node_info[ctx->node_index], k);
                        }   
                    }
                    // Wait for completion (with 15s timeout)
                    if (xSemaphoreTake(ctx->completion_sem, pdMS_TO_TICKS(20000))) {
                        ESP_LOGI(TAG, "Discovery completed successfully\n");
                    } else {
                        ESP_LOGE(TAG, "Discovery timed out\n");
                    }
                    if (ctx) {
                        vSemaphoreDelete(completion_sem);                      
                        free(ctx);
                    } 
                    
                }
            }
        } else{
            ESP_LOGE(TAG, "Src Short == 0\n");
        }   
    }
    if(recordsSemaphore != NULL) xSemaphoreGive(recordsSemaphore);
    vTaskDelete(NULL); // Delete the task after executing
}


void unbind_zigbee_nodes_task(void* args) {
    dst_node_info_t* d_node_info = (dst_node_info_t*)args;

    //printf("--------------------endpoint_counts:%d--------------------\n", d_node_info[0].endpoint_counts);
    for(int n_index=0; n_index<node_counts; n_index++){
        if(d_node_info[n_index].short_addr != 0){
            //printf("--------------------short_addr:%d--------------------\n", d_node_info[n_index].short_addr);  
            for(int j=0; j<d_node_info[n_index].endpoint_counts; j++){ 
                if(d_node_info[n_index].dst_ep_info.ep_data[j].is_bind == 1){
                    printf("==================================\n"); 
                    printf("BINDING FOUND AT SHORT_ADDR: 0x%x EP:%d\n", 
                        d_node_info[n_index].short_addr, 
                        d_node_info[n_index].dst_ep_info.ep_data[j].dst_ep);
                    
                    //printf("clusters_count: %d\n", d_node_info[n_index].dst_ep_info.ep_data[j].clusters_count); 
                    
                    // Create semaphore
                    completion_sem = xSemaphoreCreateBinary();
                    if (completion_sem == NULL) {
                        ESP_LOGE(TAG, "Failed to create semaphore");
                        continue;
                    }

                    // Create a context
                    descriptor_binding_context_t *ctx = malloc(sizeof(descriptor_binding_context_t));
                    if (ctx == NULL) {
                        ESP_LOGE(TAG, "Failed to allocate memory for context");
                        vSemaphoreDelete(completion_sem);
                        continue;
                    }
                    ctx->node_index = n_index;
                    ctx->short_addr = d_node_info[n_index].short_addr;
                    ctx->completion_sem = completion_sem;
                    // Start Binding process
                    ep_cb_retry_counts = 0;
                    ctx->bind_req_index = 0;
                    ctx->current_ep_index = j;
                    
                    ctx->button_index = nodes_info.scene_id;
                    ctx->clusters_count = d_node_info[ctx->node_index].dst_ep_info.ep_data[j].clusters_count;
                    if(d_node_info[ctx->node_index].dst_ep_info.ep_data[j].clusters_count > 0 && d_node_info[ctx->node_index].dst_ep_info.ep_data[j].clusters_count < 5){
                        //ESP_LOGI(TAG, "===========>Send Bind Light Request...NODE_INDEX:%d\n", ctx->node_index);
                        size_t required_size = d_node_info[ctx->node_index].dst_ep_info.ep_data[j].clusters_count;

                        ctx->bind_req_size = required_size; // Save the allocated size
                        for(int k=0; k<d_node_info[ctx->node_index].dst_ep_info.ep_data[j].clusters_count; k++){

                            esp_zb_ieee_addr_t src_ieee_addr;
                            esp_zb_ieee_address_by_short(d_node_info[ctx->node_index].short_addr, src_ieee_addr);
                            esp_zb_get_long_address(ctx->bind_req[k].src_address);
                            ctx->bind_req[k].src_endp = ENDPOINTS_LIST[src_endpoint_index];
                                    
                            ctx->bind_req[k].dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
                            memcpy(ctx->bind_req[k].dst_address_u.addr_long, src_ieee_addr, sizeof(esp_zb_ieee_addr_t));
                            ctx->bind_req[k].dst_endp = d_node_info[ctx->node_index].dst_ep_info.ep_data[j].dst_ep;
                            ctx->bind_req[k].dst_address_u.addr_short = d_node_info[ctx->node_index].short_addr;        
                            ctx->bind_req[k].req_dst_addr = esp_zb_get_short_address();            
                            ctx->bind_req[k].cluster_id = d_node_info[ctx->node_index].dst_ep_info.ep_data[j].cluster_id[k];

    
                        } //for
                        ep_cb_retry_counts = 0;
                        
                        if(d_node_info[ctx->node_index].dst_ep_info.ep_data[ctx->current_ep_index].is_bind == 1){
                            printf("======>SEND NODE[%d] BINDING REQUEST------------------------", ctx->node_index);
                            memcpy(&ctx->dst_node_info, &d_node_info[ctx->node_index], sizeof(dst_node_info_t));
                            esp_zb_zdo_device_unbind_req(&ctx->bind_req[ctx->bind_req_index], bind_cb_4, ctx);
                        }   
                    }
                    // Wait for completion (with 15s timeout)
                    if (xSemaphoreTake(ctx->completion_sem, pdMS_TO_TICKS(20000))) {
                        ESP_LOGI(TAG, "Discovery completed successfully\n");
                    } else {
                        ESP_LOGE(TAG, "Discovery timed out\n");
                    }
                    if (ctx) {
                        vSemaphoreDelete(completion_sem);                      
                        free(ctx);
                    } 
                    
                }
            }
        } else{
            ESP_LOGE(TAG, "Src Short == 0\n");
        }   
    }
    if(recordsSemaphore != NULL) xSemaphoreGive(recordsSemaphore);
    vTaskDelete(NULL); // Delete the task after executing
}



uint8_t nuos_unbind_task(uint8_t src_index, void * data){
    src_endpoint_index = src_index;
    recordsSemaphore = xSemaphoreCreateBinary();
    start_led_blinking(3, src_index);
    if (recordsSemaphore == NULL) {
        // Handle semaphore creation failure
        ESP_LOGE(TAG, "Failed to create semaphore");
        return ESP_FAIL;
    }
    xTaskCreate(unbind_zigbee_nodes_task, "unbind_node_task", TASK_STACK_SIZE_UNBIND_NODES, data, TASK_PRIORITY_ZB_UNBIND_NODES, NULL); 
    if (recordsSemaphore != NULL) {
        // Wait for the semaphore to be given by the records task
        xSemaphoreTake(recordsSemaphore, portMAX_DELAY);
    }
    stop_led_blinking();
    return 0;
}

uint8_t nuos_bind_task(uint8_t src_index, void * data){
    src_endpoint_index = src_index;
    recordsSemaphore = xSemaphoreCreateBinary();
    start_led_blinking(3, src_index);
    if (recordsSemaphore == NULL) {
        // Handle semaphore creation failure
        ESP_LOGE(TAG, "Failed to create semaphore");
        return ESP_FAIL;
    }
    xTaskCreate(bind_zigbee_nodes_task, "bind_node_task", TASK_STACK_SIZE_BIND_NODES, data, TASK_PRIORITY_ZB_BIND_NODES, NULL); 
    if (recordsSemaphore != NULL) {
        // Wait for the semaphore to be given by the records task
        xSemaphoreTake(recordsSemaphore, portMAX_DELAY);
    }
    stop_led_blinking();
    return 0;
}