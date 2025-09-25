/*
 * webserver.c
 *
 *  Created on: 21-Feb-2024
 *      Author: procu
 */
#include "app_constants.h"
//#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
#ifdef USE_WIFI_WEBSERVER
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>


#include "esp_event.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_wifi_webserver.h"
#include "cJSON.h"
#include "app_nvs_store_info.h"
#include "app_hardware_driver.h"
#include "nvs_flash.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "app_zigbee_group_commands.h"
#include "app_zigbee_scene_commands.h"
#include "app_zigbee_misc.h"
#include "app_zigbee_query_nodes.h"

#define MAX_HTTP_RECV_BUFFER 			4096

static const char *TAG                  = "WEBSERVER";
extern wifi_info_handle_t wifi_info;
static int decode_type                  = 0;
bool state                              = false;
cJSON *ac_model                         = NULL;
cJSON *ac_index                         = NULL; 
cJSON *selected_items                   = NULL;
cJSON *option_selected                  = NULL;
cJSON *dbind                            = NULL;
cJSON *dstate                           = NULL;
cJSON *dlevel                           = NULL;
cJSON *dcolor                           = NULL;
cJSON *dcheck                           = NULL;

extern SemaphoreHandle_t bind_sem;
uint8_t node_index, ep_index;
void query_all_groups_task(void* args);
void remove_scene_task(void* args);

void remove_duplicates(int* array, int size, int* result, int* result_size);

int map(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH)
    void add_scene_task(void* args);
    void view_scene_table_task(void* args);
#endif

char input_str[12];
const int size = 20;//sizeof(dali_nvs_stt[0].device_ids) / sizeof(dali_nvs_stt[0].device_ids[0]);
// Allocate memory for the result arrays (same size as the original array)
int temp_result[20];
int temp_result_size;

int final_result[20];
int final_result_size;

void parse_json(const char *json_string) {
    // Parse JSON string
    cJSON *root = cJSON_Parse(json_string);
    if (root == NULL) {
        printf("Error parsing JSON: %s\n", cJSON_GetErrorPtr());
        return;
    }
    wifi_webserver_active_counts = 0;
    // Extract values from JSON object
    cJSON *fxn = cJSON_GetObjectItem(root, "fxn");
    if (fxn == NULL){  printf("------------ERROR returning back----------\n");  cJSON_Delete(root); return;}

    printf("Function: %s\n", fxn->valuestring);
    int fxnInt = atoi( fxn->valuestring);
    switch(fxnInt){
        case 1:
            cJSON *wifi_ssid = cJSON_GetObjectItem(root, "ssid");
            cJSON *wifi_pswd = cJSON_GetObjectItem(root, "password");
           
            if (wifi_ssid == NULL || wifi_pswd == NULL) {
                printf("Missing JSON keys wifi_ssid && wifi_pswd\n");
                cJSON_Delete(root);
                return;
            }
          
            printf("wifi_ssid: %s\n", wifi_ssid->valuestring);
            printf("wifi_pswd: %s\n", wifi_pswd->valuestring);

            memset(&wifi_info, 0, sizeof(wifi_info));
            
            wifi_info.is_wifi_sta_mode = true;
            strncpy(wifi_info.wifi_ssid, wifi_ssid->valuestring, strlen(wifi_ssid->valuestring));
            strncpy(wifi_info.wifi_pass, wifi_pswd->valuestring, strlen(wifi_pswd->valuestring));

            cJSON *ip4 = cJSON_GetObjectItem(root, "ip4");
            if (ip4 == NULL) {
                printf("Missing JSON keys ip4\n");
                wifi_info.ip4 = 110;
            }else{
                wifi_info.ip4 = (uint8_t)atoi(ip4->valuestring);
            }  
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)

            nuos_store_wifi_info_data_to_nvs();
            #endif
            esp_restart();
        break;
        case 6: 
            cJSON *data = cJSON_GetObjectItem(root, "data");
            if (data == NULL) {
                printf("Missing JSON keys data\n");
                cJSON_Delete(root);
                return;
            }        
            if(data->valueint == 0) wifi_info.is_wifi_sta_mode = 1;
            else wifi_info.is_wifi_sta_mode = 0;
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)

            nuos_store_wifi_info_data_to_nvs();
            #endif
            esp_restart();
        break;        
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
        case 2:
            ac_model = cJSON_GetObjectItem(root, "ac_model");
            ac_index = cJSON_GetObjectItem(root, "ac_index");
            if (ac_model == NULL || ac_index == NULL) {
                printf("Missing JSON keys\n");
                cJSON_Delete(root);
                return;
            }
            set_decode_type(ac_index->valueint);
            printf("AC Model: %s\n", ac_model->valuestring);
        break;

        case 3:   //try
            ac_model = cJSON_GetObjectItem(root, "ac_model");
            ac_index = cJSON_GetObjectItem(root, "ac_index");
            if (ac_index == NULL) {
                printf("Missing JSON keys\n");
                cJSON_Delete(root);
                return;
            }
            decode_type = ac_index->valueint;         
            state = !state; //toggle ac
            if(!state) printf("Turning OFF: %s\n", ac_model->valuestring);
            else printf("Turning ON: %s\n", ac_model->valuestring);
            set_decode_type(decode_type);
            nuos_try_ac(decode_type, state);
        break;

        case 4:  //cancel
            setNVSWebServerEnableFlag(false);
            nuos_store_wifi_info_data_to_nvs();
            esp_restart();
        break;
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
        case 15:
            selected_items = cJSON_GetObjectItem(root, "selected_items");
            for (int p = 0 ; p < cJSON_GetArraySize(selected_items); p++)
            {
                cJSON * subitem = cJSON_GetArrayItem(selected_items, p);
                cJSON *error = cJSON_GetObjectItem(subitem, "error");
                if (error != NULL) {
                    printf("ERROR getting details\n");
                    cJSON_Delete(root);
                    return;
                }
                cJSON *shortaddr = cJSON_GetObjectItem(subitem, "short");
                cJSON *dstep = cJSON_GetObjectItem(subitem, "dst");
                dbind = cJSON_GetObjectItem(subitem, "bind");
                dstate = cJSON_GetObjectItem(subitem, "state");
                dcheck = cJSON_GetObjectItem(subitem, "check");

                uint8_t _is_state = (uint8_t)dstate->valueint;
                uint8_t _is_bind = (uint8_t)dbind->valueint;
                uint16_t _short_addr = (uint16_t)shortaddr->valueint;
                uint8_t _dst_ep = (uint16_t)dstep->valueint; 
                uint8_t is_check = (uint8_t)dcheck->valueint;

                if(is_check && _is_state){ 
                    send_identify_command(_short_addr, 1, _dst_ep, 6); 
                }
            
            }                
        break;
        case 16:
            option_selected = cJSON_GetObjectItem(root, "option");
            int main_ep_bind_index = option_selected->valueint;   
            uint8_t switch_id = main_ep_bind_index + 1;       
            selected_items = cJSON_GetObjectItem(root, "selected_items");
            for (int p = 0 ; p < cJSON_GetArraySize(selected_items); p++)
            {
                cJSON * subitem = cJSON_GetArrayItem(selected_items, p);
                cJSON *error = cJSON_GetObjectItem(subitem, "error");
                if (error != NULL) {
                    printf("ERROR getting details\n");
                    cJSON_Delete(root);
                    return;
                }
                cJSON *shortaddr = cJSON_GetObjectItem(subitem, "short");
                cJSON *dstep = cJSON_GetObjectItem(subitem, "dst");
                dbind = cJSON_GetObjectItem(subitem, "bind");
                dstate = cJSON_GetObjectItem(subitem, "state");
                dlevel = cJSON_GetObjectItem(subitem, "level");

                dcheck = cJSON_GetObjectItem(subitem, "check");

                uint8_t _is_state = (uint8_t)dstate->valueint;
                uint8_t _is_bind = (uint8_t)dbind->valueint;
                uint16_t _short_addr = (uint16_t)shortaddr->valueint;
                uint8_t _dst_ep = (uint8_t)dstep->valueint; 
                uint8_t is_check = (uint8_t)dcheck->valueint;
                uint8_t _level = (uint8_t)dlevel->valueint;

                nuos_set_scene_devices(main_ep_bind_index, _is_state, _level, switch_id, _dst_ep, _short_addr);
                
            }                
        break;
        case 11:  //remote scene binding
            //"option":"scene1"  
            option_selected = cJSON_GetObjectItem(root, "option");
            main_ep_bind_index = option_selected->valueint;   
            switch_id = main_ep_bind_index + 1;  //wireless switch endpoint   

            selected_items = cJSON_GetObjectItem(root, "selected_items");

            //memset(&nodes_info, 0, sizeof(stt_scene_switch_t));
            memcpy(&nodes_info, &existing_nodes_info[main_ep_bind_index], sizeof(stt_scene_switch_t));
            node_counts = existing_nodes_info[main_ep_bind_index].scene_switch_info.total_records;
            for(int i=0; i<node_counts; i++){
                for(int j=0; j<nodes_info.scene_switch_info.dst_node_info[i].endpoint_counts; j++){                                     
                    nodes_info.scene_switch_info.dst_node_info[i].dst_ep_info.ep_data[j].is_bind = 0;
                }
            }
            for (int p = 0 ; p < cJSON_GetArraySize(selected_items); p++){
                cJSON * subitem = cJSON_GetArrayItem(selected_items, p);
                cJSON *error = cJSON_GetObjectItem(subitem, "error");
                if (error != NULL) {
                    printf("ERROR getting details\n");
                    cJSON_Delete(root);
                    return;
                }
                cJSON *shortaddr = cJSON_GetObjectItem(subitem, "short");
                cJSON *dstep = cJSON_GetObjectItem(subitem, "dst");
                dbind = cJSON_GetObjectItem(subitem, "bind");
                dstate = cJSON_GetObjectItem(subitem, "state");
                dlevel = cJSON_GetObjectItem(subitem, "level");
                dcolor = cJSON_GetObjectItem(subitem, "color");
                dcheck = cJSON_GetObjectItem(subitem, "check");

                uint8_t _is_bind = (uint8_t)dbind->valueint;
                uint8_t _is_state = (uint8_t)dstate->valueint;
                uint8_t is_check = (uint8_t)dcheck->valueint;
                uint16_t short_addr = (uint16_t)shortaddr->valueint;
                uint8_t dst_ep = (uint8_t)dstep->valueint;

                printf("short_addr:0x%x  dst_ep:%d\n", short_addr, dst_ep);
                if(_is_bind && _is_state) {                  
                    uint8_t src_ep = ENDPOINTS_LIST[main_ep_bind_index];
                    printf("src_ep:%d,  main_ep_bind_index:%d\n", src_ep, main_ep_bind_index);
                    if(main_ep_bind_index != 0xff){
                        uint8_t node_index = get_node_index(main_ep_bind_index, short_addr);
                        if(node_index != 0xff){
                            uint8_t ep_index = get_ep_index(main_ep_bind_index, node_index, dst_ep);
                            printf("dst_ep:%d,  ep_index:%d\n", dst_ep, ep_index);
                            if(ep_index != 0xff) {
                                // nodes_info.scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].dst_ep = dst_ep;
                                // nodes_info.scene_switch_info.dst_node_info[node_index].short_addr = short_addr;
                                nodes_info.scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].is_bind = 1;
                                binding_count++;
                                printf("Binding_count:%d\n", binding_count);
                                    
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
            }

            if (binding_count > 0) {
                printf("----nuos_bind_task----\n");
                nuos_bind_task(main_ep_bind_index, nodes_info.scene_switch_info.dst_node_info);               
            }             
        break;
        case 21: //remote scene Un-binding
            //"option":"scene1"  
            option_selected = cJSON_GetObjectItem(root, "option");
            main_ep_bind_index = option_selected->valueint;   
            switch_id = main_ep_bind_index + 1;  //wireless switch endpoint   
            selected_items = cJSON_GetObjectItem(root, "selected_items");
            //memset(&nodes_info, 0, sizeof(stt_scene_switch_t));
            memcpy(&nodes_info, &existing_nodes_info[main_ep_bind_index], sizeof(stt_scene_switch_t));
            node_counts = existing_nodes_info[main_ep_bind_index].scene_switch_info.total_records;
            for(int i=0; i<node_counts; i++){
                for(int j=0; j<nodes_info.scene_switch_info.dst_node_info[i].endpoint_counts; j++){                                     
                    nodes_info.scene_switch_info.dst_node_info[i].dst_ep_info.ep_data[j].is_bind = 0;
                }
            }
            for (int p = 0 ; p < cJSON_GetArraySize(selected_items); p++){
                cJSON * subitem = cJSON_GetArrayItem(selected_items, p);
                cJSON *error = cJSON_GetObjectItem(subitem, "error");
                if (error != NULL) {
                    printf("ERROR getting details\n");
                    cJSON_Delete(root);
                    return;
                }
                cJSON *shortaddr = cJSON_GetObjectItem(subitem, "short");
                cJSON *dstep = cJSON_GetObjectItem(subitem, "dst");
                dbind = cJSON_GetObjectItem(subitem, "bind");
                dstate = cJSON_GetObjectItem(subitem, "state");
                dlevel = cJSON_GetObjectItem(subitem, "level");
                dcolor = cJSON_GetObjectItem(subitem, "color");
                dcheck = cJSON_GetObjectItem(subitem, "check");

                uint8_t _is_bind = (uint8_t)dbind->valueint;
                uint8_t _is_state = (uint8_t)dstate->valueint;
                uint8_t is_check = (uint8_t)dcheck->valueint;
                uint16_t short_addr = (uint16_t)shortaddr->valueint;
                uint8_t dst_ep = (uint8_t)dstep->valueint;

                printf("short_addr:0x%x  dst_ep:%d\n", short_addr, dst_ep);
                if(_is_bind && _is_state) {                  
                    uint8_t src_ep = ENDPOINTS_LIST[main_ep_bind_index];
                    printf("src_ep:%d,  main_ep_bind_index:%d\n", src_ep, main_ep_bind_index);
                    if(main_ep_bind_index != 0xff){
                        uint8_t node_index = get_node_index(main_ep_bind_index, short_addr);
                        if(node_index != 0xff){
                            uint8_t ep_index = get_ep_index(main_ep_bind_index, node_index, dst_ep);
                            printf("dst_ep:%d,  ep_index:%d\n", dst_ep, ep_index);
                            if(ep_index != 0xff) {
                                //nodes_info
                                // nodes_info.scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].dst_ep = dst_ep;
                                // nodes_info.scene_switch_info.dst_node_info[node_index].short_addr = short_addr;
                                nodes_info.scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].is_bind = 0;
                                binding_count++;
                                printf("Binding_count:%d\n", binding_count);
                                //======> write here code
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
            }
            if (binding_count > 0) {
                printf("----nuos_unbind_task----\n");
                nuos_unbind_task(main_ep_bind_index, nodes_info.scene_switch_info.dst_node_info);               
            }    
        break;  

        case 22:  //clear all records
            clear_all_records_in_nvs();
        break;
        case 23:  //set scene selected devices's attributes
            option_selected = cJSON_GetObjectItem(root, "option");
            main_ep_bind_index = option_selected->valueint;   
            switch_id = main_ep_bind_index + 1;  //wireless switch endpoint   
            
            for(int node_index=0; node_index<existing_nodes_info[main_ep_bind_index].scene_switch_info.total_records; node_index++){
                for(int ep_index = 0; ep_index <existing_nodes_info[main_ep_bind_index].scene_switch_info.dst_node_info[node_index].endpoint_counts; ep_index++){
                    if(existing_nodes_info[main_ep_bind_index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].is_bind == 1){
                        do_remote_scene_bindings(
                            main_ep_bind_index, 
                            existing_nodes_info[main_ep_bind_index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].dst_ep, 
                            existing_nodes_info[main_ep_bind_index].scene_switch_info.dst_node_info[node_index].short_addr, 
                            true);
                    }
                }
            }

        break;




        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH)
            case 12:
                //{"fxn":"12","option":1,"data":{"index":1,"name":"SWITCH_2","intensity":50,"isOn":false}}
                cJSON *data = cJSON_GetObjectItem(root, "data");
                cJSON *dataIndex = cJSON_GetObjectItem(data, "index");
                cJSON *dataIntensity = cJSON_GetObjectItem(data, "intensity");
                cJSON *dataIsOn = cJSON_GetObjectItem(data, "isOn");

                // Ensure 'isOn' is a boolean
                bool isOn = false;
                if (cJSON_IsBool(dataIsOn)) {
                    isOn = cJSON_IsTrue(dataIsOn);
                    printf("isOn: %s\n", isOn ? "true" : "false");
                } else {
                    printf("Error: 'isOn' is not a boolean\n");
                }

                uint8_t index = (uint8_t)dataIndex->valueint;
                uint8_t level = atoi(dataIntensity->valuestring);
                
                printf("index:%d level:%d isOn:%d\n", index, level, isOn);
                zb_scene_info[index].group_id = global_group_id[0];
                zb_scene_info[index].scene_id = global_scene_id[index];
                zb_scene_info[index].is_on = (uint8_t)isOn;
                zb_scene_info[index].intensity = level;
                zb_scene_info[index].dst_ep = 1;
                
                xTaskCreate(add_scene_task, "add_scene_task", 8192, &index, 25, NULL); 
                //xTaskCreate(query_all_groups_task, "query_group_task", 4096, &global_group_id[index], 25, NULL); 
                //xTaskCreate(view_scene_table_task, "view_scene_table_task", 4096, &index, 26, NULL);    
            break; 
            case 13:
                //{"fxn":"12","option":1,"data":{"index":1,"name":"SWITCH_2","intensity":50,"isOn":false}}
                data = cJSON_GetObjectItem(root, "data");
                dataIndex = cJSON_GetObjectItem(data, "index");
                dataIntensity = cJSON_GetObjectItem(data, "intensity");
                dataIsOn = cJSON_GetObjectItem(data, "isOn");

                // Ensure 'isOn' is a boolean
                isOn = false;
                if (cJSON_IsBool(dataIsOn)) {
                    isOn = cJSON_IsTrue(dataIsOn);
                    printf("isOn: %s\n", isOn ? "true" : "false");
                } else {
                    printf("Error: 'isOn' is not a boolean\n");
                }

                index = (uint8_t)dataIndex->valueint;
                level = atoi(dataIntensity->valuestring);
                
                printf("index:%d level:%d isOn:%d\n", index, level, isOn);
                zb_scene_info[index].group_id = global_group_id[0];
                zb_scene_info[index].scene_id = global_scene_id[index];
                zb_scene_info[index].is_on = (uint8_t)isOn;
                zb_scene_info[index].intensity = level;
                zb_scene_info[index].dst_ep = 1;
                xTaskCreate(remove_scene_task, "remove_scene_task", 8192, &index, 25, NULL); 
            break;                   
        #endif 
    #else
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI )
            case 10:
                cJSON *sindex = cJSON_GetObjectItem(root, "index");
                if (sindex == NULL) {
                    printf("Missing JSON keys index\n");
                    cJSON_Delete(root);
                    return;
                } 

                int index = sindex->valueint;
                printf("sindex: %d\n", index);

                // Get the values array
                cJSON *valuesArray = cJSON_GetObjectItem(root, "values");
                if (!cJSON_IsArray(valuesArray))
                {
                    ESP_LOGE(TAG, "Values is not an array");
                    cJSON_Delete(root);
                    return;
                }
                dali_nvs_stt[index].group_id = global_group_id[index];
                // Iterate over the array and print the values
                int arraySize = cJSON_GetArraySize(valuesArray);
                for (int i = 0; i < arraySize; ++i)
                {
                    cJSON *item = cJSON_GetArrayItem(valuesArray, i);
                    if (cJSON_IsNumber(item))
                    {
                        int value = item->valueint;
                        
                        temp_result[i] = value;

                        ESP_LOGI(TAG, "Value at index %d: %d", i, temp_result[i]);
                        
                    }
                }
                // Remove zeros
                int j = 0;
                for (int i = 0; i < size; ++i) {
                    if (temp_result[i] != 0) {
                        final_result[j++] = temp_result[i];
                    }
                }
                final_result_size = j;
                //remove 
                for (int i = 0; i < dali_nvs_stt[index].total_ids; ++i) {
                    nuos_dali_remove_light_from_group(dali_nvs_stt[index].device_ids[i], dali_nvs_stt[index].group_id);
                    vTaskDelay(5);
                }
                // Remove duplicates
                j=0;
                for (int i = 0; i < final_result_size; ++i) {
                    bool is_duplicate = false;
                    for (int k = 0; k < j; ++k) {
                        if (dali_nvs_stt[index].device_ids[k] == final_result[i]) {
                            is_duplicate = true;
                            break;
                        }
                    }
                    if (!is_duplicate) {
                        dali_nvs_stt[index].device_ids[j++] = final_result[i];
                    }
                }
                dali_nvs_stt[index].total_ids = j;

                // Print the final result array
                printf("Array without zeros and duplicates: ");
                for (int i = 0; i < dali_nvs_stt[index].total_ids; ++i) {
                    printf("%d ", dali_nvs_stt[index].device_ids[i]);
                    nuos_dali_add_light_to_group(dali_nvs_stt[index].device_ids[i], dali_nvs_stt[index].group_id);
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                }
                printf("\n");
                for (int p = dali_nvs_stt[index].total_ids; p < 20; p++) {
                    dali_nvs_stt[index].device_ids[p] = 0;
                }
                
                nuos_store_dali_data_to_nvs(index);
                break;

            case 11:  //Toggle DALI Group
                sindex = cJSON_GetObjectItem(root, "index");
                if (sindex == NULL) {
                    printf("Missing JSON keys index\n");
                    cJSON_Delete(root);
                    return;
                }     
                printf("sindex: %d\n", sindex->valueint);
                index = sindex->valueint;
                if(dali_nvs_stt[index].state == 0) dali_nvs_stt[index].state = 1;
                else dali_nvs_stt[index].state = 0;
                nuos_dali_toggle_group(dali_nvs_stt[index].group_id, index, dali_nvs_stt[index].state, dali_nvs_stt[index].brightness);
                nuos_store_dali_data_to_nvs(index);
                break;
                
            case 17:  //Set Intensity of DALI Group
                sindex = cJSON_GetObjectItem(root, "index");
                if (sindex == NULL) {
                    printf("Missing JSON keys index\n");
                    cJSON_Delete(root);
                    return;
                }   
                index = sindex->valueint;  
                printf("sindex: %d\n", index);
                cJSON *svalue = cJSON_GetObjectItem(root, "value");
                if (svalue == NULL) {
                    printf("Missing JSON keys sinput\n");
                    cJSON_Delete(root);
                    return;
                }
                uint8_t brightness =  svalue->valueint;
                printf("svalue: %d\n", svalue->valueint);

                
                #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
                    device_info[0].device_state = true;
                    device_info[1].device_state = true;
                    device_info[2].device_state = true;
                    device_info[3].device_state = true;                
                    device_info[3].device_level = (uint8_t)map(brightness, 0, 100, 0, MAX_DIM_LEVEL_VALUE);
                    device_info[3].device_val  = (uint8_t)map(brightness, 0, 1000, 0, 1000);
                    nuos_zb_set_hardware(3, false);
                    nuos_set_hw_brightness(3);
                #else
                    dali_nvs_stt[index].brightness = (uint8_t)map(brightness, 0, 100, 0, 254); 
                    nuos_dali_set_group_brightness(dali_nvs_stt[index].group_id, index, dali_nvs_stt[index].brightness);
                    nuos_store_dali_data_to_nvs(index);
                #endif
                
                break;             
        #endif
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI)
            case 30:
                device_info[0].device_state = true;
                device_info[1].device_state = true;
                device_info[2].device_state = true;
                device_info[3].device_state = true;

                //{favcolor: "#d31717", fxn: "30"}
                cJSON *favcolor_item  = cJSON_GetObjectItem(root, "favcolor");

                if (!cJSON_IsString(favcolor_item)) {
                    printf("Invalid favcolor value\n");
                    return;
                }

                const char* hex_color = favcolor_item->valuestring;

                // Validate hex color string
                if (strlen(hex_color) != 7 || hex_color[0] != '#') {
                    printf("Invalid hex color format\n");
                    return;
                }
                int r, g, b;
                sscanf(hex_color + 1, "%02x%02x%02x", &r, &g, &b);
                // Convert hexadecimal to decimal
                device_info[0].device_level = (uint8_t)r;
                device_info[1].device_level = (uint8_t)g;
                device_info[2].device_level = (uint8_t)b;

                printf("RED:    0x%x\n", device_info[0].device_level);
                printf("GREEN:    0x%x\n", device_info[1].device_level);
                printf("BLUE:    0x%x\n", device_info[2].device_level);

                nuos_zb_set_hardware(3, false);

            break;
            case 31:
                cJSON *sbaudrate = cJSON_GetObjectItem(root, "baud_rate");
                if (sbaudrate == NULL) {
                    printf("Missing JSON keys index\n");
                    cJSON_Delete(root);
                    return;
                }   
                uint32_t baudrate = atoi(sbaudrate->valuestring);

                cJSON *sdatabits = cJSON_GetObjectItem(root, "data_bits");
                if (sdatabits == NULL) {
                    printf("Missing JSON keys index\n");
                    cJSON_Delete(root);
                    return;
                }   
                // uint32_t databits = sdatabits->valuestring;

                cJSON *sparity = cJSON_GetObjectItem(root, "parity");
                if (sparity == NULL) {
                    printf("Missing JSON keys index\n");
                    cJSON_Delete(root);
                    return;
                }   
                // uint32_t parity = sparity->valuestring;  

                cJSON *sstopbits = cJSON_GetObjectItem(root, "stop_bits");
                if (sstopbits == NULL) {
                    printf("Missing JSON keys index\n");
                    cJSON_Delete(root);
                    return;
                }   
                // uint32_t stopbits = sstopbits->valuestring;

                cJSON *sflowctrl = cJSON_GetObjectItem(root, "flow_ctrl");
                if (sflowctrl == NULL) {
                    printf("Missing JSON keys index\n");
                    cJSON_Delete(root);
                    return;
                }   
                // uint32_t flowctrl = sflowctrl->valuestring; 

                uart_word_length_t databits = string_to_uart_word_length_t(sdatabits->valuestring);
                uart_stop_bits_t stopbits = string_to_uart_stop_bits_t(sstopbits->valuestring);
                uart_parity_t parity = string_to_uart_parity_t(sparity->valuestring);
                uart_hw_flowcontrol_t flowctrl = string_to_uart_hw_flowcontrol_t(sflowctrl->valuestring);
                
                uart_stt.data_bits = databits;
                uart_stt.stop_bits = stopbits;
                uart_stt.parity = parity;
                uart_stt.flow_ctrl = flowctrl;
                writeUartStruct(0, (uart_config_t*)&uart_stt);

                if(baudrate == 0){ //enable default
                    dmx_nvs_stt.set_default_flag = true;
                } else{
                    dmx_nvs_stt.set_default_flag = false;
                }
                dmx_nvs_stt.uart_baudrate = baudrate;
                writeDmxStruct(0, (dmx_variable_t*)&dmx_nvs_stt);
                esp_restart();

            break;

            case 32: //{fxn: "32", index: 0, value: "17"}
                cJSON *sdmxstartch = cJSON_GetObjectItem(root, "value");
                if (sdmxstartch == NULL) {
                    printf("Missing JSON keys index\n");
                    cJSON_Delete(root);
                    return;
                }   
                dmx_nvs_stt.dmx_start_address = (uint8_t)atoi(sdmxstartch->valuestring);
                writeDmxStruct(0, (dmx_variable_t*)&dmx_nvs_stt);
                esp_restart(); 
            break;

            case 34: //{fxn: "32", index: 0, value: "17"}
                cJSON *sdmxgpiopin = cJSON_GetObjectItem(root, "value");
                if (sdmxgpiopin == NULL) {
                    printf("Missing JSON keys index\n");
                    cJSON_Delete(root);
                    return;
                }   
                dmx_nvs_stt.dmx_gpio_pin = (uint8_t)atoi(sdmxgpiopin->valuestring);
                writeDmxStruct(0, (dmx_variable_t*)&dmx_nvs_stt);
                esp_restart();
            break;            
        #endif
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
            case 16:  //Set Color Temperature of DALI Group
                sindex = cJSON_GetObjectItem(root, "index");
                if (sindex == NULL) {
                    printf("Missing JSON keys index\n");
                    cJSON_Delete(root);
                    return;
                }     
                index = sindex->valueint;
                printf("sindex: %d\n", index);
                svalue = cJSON_GetObjectItem(root, "value");
                if (svalue == NULL) {
                    printf("Missing JSON keys sinput\n");
                    cJSON_Delete(root);
                    return;
                }
                printf("svalue: %d\n", svalue->valueint);
                dali_nvs_stt[index].color_value = (uint16_t)svalue->valueint;
                nuos_dali_set_group_color_temperature(dali_nvs_stt[index].group_id, index, dali_nvs_stt[index].color_value);
                nuos_store_dali_data_to_nvs(index);
                break;
        #endif
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI)
            case 12:  //Set Intensity of DALI Group
                sindex = cJSON_GetObjectItem(root, "max_addr");
                if (sindex == NULL) {
                    printf("Missing JSON keys index\n");
                    cJSON_Delete(root);
                    return;
                }   
                int max_address = atoi(sindex->valuestring);

                printf("max_addr: %d\n", max_address);
                setNVSDaliNodesCommissioningCounts(max_address);
                
                start_dali_addressing(max_address);

                break; 
        #endif
    #endif
        
        default: break;
    }
    // Clean up cJSON object
    cJSON_Delete(root);
}

void query_all_groups_task(void* args) {
    uint16_t group_id = *((uint16_t *)args);
    //printf("group_id:%d", group_id);
    nuos_zigbee_group_query_all_groups(group_id);
    vTaskDelete(NULL); // Delete the task after executing
}

#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH)
    void add_scene_task(void* args) {
        uint8_t index = *((uint8_t *)args);
        printf("index:%d", index);
        esp_zb_lock_acquire(portMAX_DELAY);
        nuos_zb_scene_add_scene_broadcast_request(zb_scene_info[index].group_id, zb_scene_info[index].scene_id, ENDPOINTS_LIST[index], 
        zb_scene_info[index].dst_ep, zb_scene_info[index].is_on, zb_scene_info[index].intensity);

        // nuos_zb_scene_add_scene_unicast_request(zb_scene_info[index].group_id, zb_scene_info[index].scene_id, 
        // ENDPOINTS_LIST[index], 
        // zb_scene_info[index].dst_ep,
        // 0x808a, 
        // zb_scene_info[index].is_on, zb_scene_info[index].intensity);


        // nuos_zb_scene_add_scene_groupcast_request(zb_scene_info[index].group_id, zb_scene_info[index].scene_id, 
        // ENDPOINTS_LIST[index], 
        // zb_scene_info[index].dst_ep,
        // zb_scene_info[index].is_on, zb_scene_info[index].intensity);

        // nuos_zb_scene_store_scene_unicast_request(zb_scene_info[index].group_id, zb_scene_info[index].scene_id, ENDPOINTS_LIST[index], 
        // zb_scene_info[index].dst_ep, 0x808a);

        nuos_zb_scene_store_scene_broadcast_request(zb_scene_info[index].group_id, zb_scene_info[index].scene_id, ENDPOINTS_LIST[index], zb_scene_info[index].dst_ep);
        // nuos_zb_scene_store_scene_unicast_request(zb_scene_info[index].group_id, zb_scene_info[index].scene_id, ENDPOINTS_LIST[index], 
        // zb_scene_info[index].dst_ep, 0x808a);
        esp_zb_lock_release();
        vTaskDelete(NULL); // Delete the task after executing
    }


    void remove_scene_task(void* args) {
        uint8_t index = *((uint8_t *)args);
        printf("index:%d", index);
        esp_zb_lock_acquire(portMAX_DELAY);
        nuos_zb_scene_remove_scene_broadcast_request(zb_scene_info[index].group_id, zb_scene_info[index].scene_id, ENDPOINTS_LIST[index], 
        zb_scene_info[index].dst_ep);
        esp_zb_lock_release();
        vTaskDelete(NULL); // Delete the task after executing
    }


    void view_scene_table_task(void* args) {
        uint8_t index = *((uint8_t *)args);
        printf("index:%d", index);
        esp_zb_zcl_scenes_table_show(ENDPOINTS_LIST[index]);  
        vTaskDelete(NULL); // Delete the task after executing
    }
#endif




char* replaceSubstring(const char* original, const char* toReplace, const char* replaceWith) {
    char* result;
    char* insertPoint;
    const char* temp;
    int lenFront, count;

    // Count the number of times the old substring occurs in the original string
    temp = original;
    for (count = 0; (temp = strstr(temp, toReplace)); ++count) {
        temp += strlen(toReplace);
    }

    // Allocate memory for the new string
    result = (char*)malloc(strlen(original) + count * (strlen(replaceWith) - strlen(toReplace)) + 1);
    if(result == NULL) {
        printf("Failed to allocate memory\n");
        return NULL;
    }

    // Replace the substring
    temp = original;
    insertPoint = result;
    while (count--) {
        lenFront = strstr(temp, toReplace) - temp;
        memcpy(insertPoint, temp, lenFront);
        insertPoint += lenFront;
        memcpy(insertPoint, replaceWith, strlen(replaceWith));
        insertPoint += strlen(replaceWith);
        temp += lenFront + strlen(toReplace);
    }
    // Copy the rest of the string
    strcpy(insertPoint, temp);
    return result;
}


// Global semaphore
// SemaphoreHandle_t xWebServerSemaphore = xSemaphoreCreateMutex();


const char resp1[] = "Post data received successfully";
const char resp2[] = "Error Getting Responses!!";
// Handler to process the form data sent via POST request
esp_err_t submit_post_handler(httpd_req_t *req) {
    char content[MAX_HTTP_RECV_BUFFER];
    int total_len = req->content_len;
    int received = 0;
    int ret;
    printf("total_len:%d\n", total_len);
    if (total_len >= MAX_HTTP_RECV_BUFFER) {
        // Content too large
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    while (received < total_len) {
        ret = httpd_req_recv(req, content + received, total_len);
        if (ret <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                httpd_resp_send_408(req);
            } else {
                httpd_resp_send_500(req);
            }
            return ESP_FAIL;
        }
        received += ret;
    }


    content[received] = '\0'; // Null-terminate the content

 
    // Find and remove all occurrences of %23
	char *pos;
	while ((pos = strstr(content, "%23")) != NULL) {
		memmove(pos, pos + 3, strlen(pos + 3) + 1);
	}
   
    ESP_LOGI("content", " : %s", content);
    parse_json(content); 
    httpd_resp_send(req, resp1, HTTPD_RESP_USE_STRLEN);
    // xWebServerSemaphore = xSemaphoreCreateMutex();

    // if (xSemaphoreTake(xWebServerSemaphore, pdMS_TO_TICKS(100))) {
    //     // Process request
    //     parse_json(content);         
    //     xSemaphoreGive(xWebServerSemaphore);
 
    //     httpd_resp_send(req, resp1, HTTPD_RESP_USE_STRLEN);      
    // } else {
    //    // httpd_resp_send_500(req);
    //    httpd_resp_send(req, resp2, HTTPD_RESP_USE_STRLEN); 
    // }


    cb_requests_counts = 0;
    cb_response_counts = 0;
    
    return ESP_OK;
}


esp_err_t http_get_handler(httpd_req_t *req) {
    printf("On http get :%d\n", strlen(webpage));
    httpd_resp_send(req, webpage, strlen(webpage));
    return ESP_OK;
}

// esp_err_t http_get_handler(httpd_req_t *req) {
//     ESP_LOGI(TAG, "HTTP GET Request");
    
//     // 1. Send headers first
//     httpd_resp_set_type(req, "text/html");
    
//     // 2. Send content in chunks
//     const size_t chunk_size = 1024; // Optimal for ESP32
//     size_t offset = 0;
    
//     while (offset < sizeof(webpage)) {
//         size_t send_len = MIN(chunk_size, sizeof(webpage) - offset);
//         esp_err_t ret = httpd_resp_send_chunk(req, webpage + offset, send_len);
        
//         if (ret != ESP_OK) {
//             ESP_LOGE(TAG, "Send failed at offset %d: %s", offset, esp_err_to_name(ret));
//             httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Send Error");
//             return ESP_FAIL;
//         }
        
//         offset += send_len;
//         taskYIELD(); // Allow other tasks to run
//         //vTaskDelay(10 / portTICK_PERIOD_MS);
//     }
    
//     // 3. Finalize response
//     return httpd_resp_send_chunk(req, NULL, 0);
// }

#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)

    char* prepare_json(uint8_t index){
        int total_item = 0;
        for(int node_index=0; node_index<node_counts; node_index++){
            for(int ep_index=0; ep_index<existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].endpoint_counts; ep_index++){   
                //if(existing_nodes_info[i].src_endpoint[j] == ENDPOINTS_LIST[index])  {
                    webpageItem[total_item].short_ = existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].short_addr;
                    webpageItem[total_item].dst =  existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].dst_ep;
                    webpageItem[total_item].src =  existing_nodes_info[index].scene_switch_info.src_ep;
                    webpageItem[total_item].bind = existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].is_bind;
                    webpageItem[total_item].state = existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].data.state;
                    webpageItem[total_item].level = existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].data.level;
                    webpageItem[total_item].value = existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].data.value;
                    webpageItem[total_item].check = 0;
                    strcpy(webpageItem[total_item].name, existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].ep_name);
                    strcpy(webpageItem[total_item].g_name, existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].node_name);
                    total_item++;
               // }               
               
            }
        } 
        // static void callback(){

        // }
        // for(int node_index=0; node_index<node_counts; node_index++){
        //     for(int ep_index=0; ep_index<existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].endpoint_counts; ep_index++){   
        //         if(existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[ep_index].is_bind == 1){
        //             send_request(callback);
        //         }
        //     }
        // }    
        //start task and wait for all record to be finished
        cJSON *json = cJSON_CreateArray();
        for (size_t i = 0; i < total_item; i++) {
            // Start creating a cJSON object
            cJSON *item_json = cJSON_CreateObject();
            // Add fields to the cJSON object
            cJSON_AddNumberToObject(item_json, "short", webpageItem[i].short_);
            cJSON_AddStringToObject(item_json, "g_name", webpageItem[i].g_name);
            cJSON_AddStringToObject(item_json, "name", webpageItem[i].name);
            cJSON_AddNumberToObject(item_json, "dst", webpageItem[i].dst);
            cJSON_AddNumberToObject(item_json, "src", webpageItem[i].src);
            cJSON_AddNumberToObject(item_json, "bind", webpageItem[i].bind);
            cJSON_AddNumberToObject(item_json, "state", webpageItem[i].state);
            cJSON_AddNumberToObject(item_json, "level", webpageItem[i].level);
            cJSON_AddNumberToObject(item_json, "color", webpageItem[i].value);
            cJSON_AddNumberToObject(item_json, "check", webpageItem[i].check);
            cJSON_AddItemToArray(json, item_json);
        }
        const char *response = cJSON_Print(json);
        cJSON_Delete(json);
        return response;
    }
    extern SemaphoreHandle_t recordsSemaphore;

    char * nuos_do_task(uint8_t index, uint8_t scene_id, uint8_t erase_data){
        char *response  = "[]";
        if(scene_id != 0xff){
            int total_item = 0;
            //if (httpd_query_key_value(query, "erase_data", value_data, sizeof(value_data)) == ESP_OK) { 
                
                printf("erase_data:%d\n", erase_data);
                if(erase_data == 1){
                    clear_all_records_in_nvs();

                    memset(&nodes_info, 0, sizeof(stt_scene_switch_t));
                    memset(existing_nodes_info, 0, sizeof(existing_nodes_info));

                }else{
                    printf("total_records:%d\n", existing_nodes_info[index].scene_switch_info.total_records);
                    memcpy(&nodes_info, &existing_nodes_info[index], sizeof(stt_scene_switch_t));
                    node_counts = existing_nodes_info[index].scene_switch_info.total_records;
                }
            //}                            
            int total_node_counts = nuos_find_active_nodes(index, node_counts, erase_data);
            if(erase_data == 1){
                memcpy(&existing_nodes_info[index], &nodes_info, sizeof(stt_scene_switch_t));
            }
            ESP_LOGI(TAG, "-------------total_node_counts=%d-----------\n", total_node_counts);
            if(total_node_counts == 0){
                response = prepare_json(index);
                for(int i=0; i<existing_nodes_info[index].scene_switch_info.total_records; i++){
                    printf("----------------NODE_ADDRESS:0x%x---------------\n", existing_nodes_info[index].scene_switch_info.dst_node_info[i].short_addr);
                    printf("ep_counts:%d\n", existing_nodes_info[index].scene_switch_info.dst_node_info[i].endpoint_counts);
                    printf("node_name:%s\n" , existing_nodes_info[index].scene_switch_info.dst_node_info[i].node_name);
                    
                    for(int j=0; j<existing_nodes_info[index].scene_switch_info.dst_node_info[i].endpoint_counts; j++){
                        printf("........ATTRIBUTE VALUES........\n");
                        printf("dst_ep:%d\n", existing_nodes_info[index].scene_switch_info.dst_node_info[i].dst_ep_info.ep_data[j].dst_ep);
                        
                        for(uint8_t k=0; k<existing_nodes_info[index].scene_switch_info.dst_node_info[i].dst_ep_info.ep_data[j].clusters_count; k++){

                            printf("cluster_id:%d\n", existing_nodes_info[index].scene_switch_info.dst_node_info[i].dst_ep_info.ep_data[j].cluster_id[k]);


                            if(existing_nodes_info[index].scene_switch_info.dst_node_info[i].dst_ep_info.ep_data[j].cluster_id[k] == 6){
                                printf("state:%d\n", existing_nodes_info[index].scene_switch_info.dst_node_info[i].dst_ep_info.ep_data[j].data.state);
                            }else if(existing_nodes_info[index].scene_switch_info.dst_node_info[i].dst_ep_info.ep_data[j].cluster_id[k] == 8){
                                printf("level:%d\n", existing_nodes_info[index].scene_switch_info.dst_node_info[i].dst_ep_info.ep_data[j].data.level);                                                
                            }else if(existing_nodes_info[index].scene_switch_info.dst_node_info[i].dst_ep_info.ep_data[j].cluster_id[k] == 768){
                                printf("color:0x%lx\n", existing_nodes_info[index].scene_switch_info.dst_node_info[i].dst_ep_info.ep_data[j].data.value);                                                
                            }
                            
                        }
                    }
                    printf("................................\n");
                }

            }else{
                
                for(int node_index=0; node_index<total_node_counts; node_index++){
                    ESP_LOGI(TAG, "-------------endpoint_counts=%d-----------\n", existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].endpoint_counts);
                    for(int j=0; j<existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].endpoint_counts; j++){
                        webpageItem[total_item].short_ = existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].short_addr;
                        webpageItem[total_item].dst =  existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[j].dst_ep;
                        webpageItem[total_item].bind = existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[j].is_bind;
                        webpageItem[total_item].state = existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[j].data.state;
                        webpageItem[total_item].level = existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[j].data.level;
                        webpageItem[total_item].value = existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[j].data.value;

                        webpageItem[total_item].src = existing_nodes_info[index].scene_switch_info.src_ep;
                        strcpy(webpageItem[total_item].name, existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].dst_ep_info.ep_data[j].ep_name);
                        strcpy(webpageItem[total_item].g_name, existing_nodes_info[index].scene_switch_info.dst_node_info[node_index].node_name);
                        total_item++;
                    }
                }
                
                //start task and wait for all record to be finished
                cJSON *json = cJSON_CreateArray();
                for (size_t i = 0; i < total_item; i++) {
                    // Start creating a cJSON object
                    cJSON *item_json = cJSON_CreateObject();
                    // Add fields to the cJSON object
                    cJSON_AddNumberToObject(item_json, "short", webpageItem[i].short_);
                    cJSON_AddStringToObject(item_json, "g_name", webpageItem[i].g_name);
                    cJSON_AddStringToObject(item_json, "val", webpageItem[i].value);
                    cJSON_AddStringToObject(item_json, "name", webpageItem[i].name);
                    cJSON_AddNumberToObject(item_json, "dst", webpageItem[i].dst);
                    cJSON_AddNumberToObject(item_json, "src", webpageItem[i].src);
                    cJSON_AddNumberToObject(item_json, "bind", webpageItem[i].bind);
                    cJSON_AddNumberToObject(item_json, "state", webpageItem[i].state);
                    cJSON_AddNumberToObject(item_json, "level", webpageItem[i].level);
                    cJSON_AddNumberToObject(item_json, "color", webpageItem[i].value);
                    cJSON_AddNumberToObject(item_json, "check", webpageItem[i].check);
                    cJSON_AddItemToArray(json, item_json);
                }
                response = cJSON_Print(json);
                printf("response: %s", response);
                cJSON_Delete(json);
            }                          
        }
        return response;
    }
    esp_err_t http_get_items_handler(httpd_req_t *req) {

        char query[128];
        char value_fxn[32];
        char value_data[32];
        uint8_t scene_id = 0xff;
        int erase_data = 0;

        char *response  = "[]";
        int index = 0;
        // Retrieve the query string from the URL
        if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
            ESP_LOGI(TAG, "Found URL query => %s", query);
            // Retrieve specific query parameter value
            if (httpd_query_key_value(query, "fxn", value_fxn, sizeof(value_fxn)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => fxn=%s", value_fxn);
                int fxn = atoi(value_fxn);  
                if(fxn == 0){
                    if(wifi_info.is_wifi_sta_mode){
                        #ifdef USE_WIFI_WEBSERVER
                        wifi_scan();

                        httpd_resp_send(req, jsonWiFiScanListStr, HTTPD_RESP_USE_STRLEN); //
                        //printf("jsonWiFiScanListStr:%s\n", jsonWiFiScanListStr);
                        //strcpy(response, jsonWiFiScanListStr);
                        return ESP_OK;
                        #endif
                    }
                }else if(fxn == 1){
                    if (httpd_query_key_value(query, "scene_id", value_data, sizeof(value_data)) == ESP_OK) {
                        ESP_LOGI(TAG, "Found URL query parameter => scene_id=%s", value_data);
                        index = atoi(value_data);
                        scene_id = index+1; 
                        if (httpd_query_key_value(query, "erase_data", value_data, sizeof(value_data)) == ESP_OK) { 
                            erase_data = atoi(value_data);
                            response = nuos_do_task(index, scene_id, erase_data);
                        }  
                        
                    }                
                }else if(fxn == 2){
                    if (httpd_query_key_value(query, "scene_id", value_data, sizeof(value_data)) == ESP_OK) {
                        ESP_LOGI(TAG, "Found URL query parameter => scene_id=%s", value_data);
                        index = atoi(value_data);
                        node_counts = existing_nodes_info[index].scene_switch_info.total_records;
                        ESP_LOGI(TAG, "total_records=%d", node_counts);
                        response = prepare_json(index);

                        for(int i=0; i<existing_nodes_info[index].scene_switch_info.total_records; i++){
                            printf("----------------NODE_ADDRESS:0x%x---------------\n", existing_nodes_info[index].scene_switch_info.dst_node_info[i].short_addr);
                            printf("endpoint_counts:%d\n", existing_nodes_info[index].scene_switch_info.dst_node_info[i].endpoint_counts);
                            printf("node_name:%s\n" , existing_nodes_info[index].scene_switch_info.dst_node_info[i].node_name);
                            
                            for(int j=0; j<existing_nodes_info[index].scene_switch_info.dst_node_info[i].endpoint_counts; j++){
                                for(uint8_t k=0; k<existing_nodes_info[index].scene_switch_info.dst_node_info[i].dst_ep_info.ep_data[j].clusters_count; k++){                                
                                    printf("cluster_id:%d\n", existing_nodes_info[index].scene_switch_info.dst_node_info[i].dst_ep_info.ep_data[j].cluster_id[k]);
                                    printf("........ATTRIBUTE VALUES........\n");
                                    
                                    printf("dst_ep:%d  state:%d  level:%d\n", existing_nodes_info[index].scene_switch_info.dst_node_info[i].dst_ep_info.ep_data[j].dst_ep,
                                    existing_nodes_info[index].scene_switch_info.dst_node_info[i].dst_ep_info.ep_data[j].data.state,
                                    existing_nodes_info[index].scene_switch_info.dst_node_info[i].dst_ep_info.ep_data[j].data.level);
                                }
                            }
                            printf("................................\n");
                        }                        
                    } 
                }
            }
        }

        httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
        
        return ESP_OK;
    }
#endif

void start_webserver() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;  // Increase the stack size
    httpd_handle_t server = NULL;
    httpd_uri_t index_uri = {
        .uri       = "/index",
        .method    = HTTP_GET,
        .handler   = http_get_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t submit_uri = {
        .uri       = "/action",
        .method    = HTTP_POST,
        .handler   = submit_post_handler,
        .user_ctx  = NULL
    };
    // server.on("/action", HTTP_POST, [](AsyncWebServerRequest *request){
    //     bool valid = true;
        
    //     // Process 4x5 grid
    //     for(int row = 1; row <= 4; row++) {
    //       for(int col = 1; col <= 5; col++) {
    //         String paramName = "input" + String(row) + "_" + String(col);
    //         if(request->hasParam(paramName, true)) {
    //           int value = request->getParam(paramName, true)->value().toInt();
              
    //           // Validate range
    //           if(value < 0 || value > 10) {
    //             valid = false;
    //             break;
    //           }
              
    //           // Process valid value
    //           Serial.printf("Received %s: %d\n", paramName.c_str(), value);
    //         }
    //       }
    //       if(!valid) break;
    //     }
    
    //     if(valid) {
    //       request->send(200, "text/plain", "Values accepted");
    //     } else {
    //       request->send(400, "text/plain", "Invalid values (0-10 only)");
    //     }
    //   });

    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
        httpd_uri_t items_uri = {
            .uri       = "/items",
            .method    = HTTP_GET,
            .handler   = http_get_items_handler,
            .user_ctx  = NULL
        };
    #endif
    if (httpd_start(&server, &config) == ESP_OK) {
    	httpd_register_uri_handler(server, &index_uri);
		httpd_register_uri_handler(server, &submit_uri);
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
            httpd_register_uri_handler(server, &items_uri);
        #endif
    }
}

void remove_duplicates(int* array, int size, int* result, int* result_size) {
    int j = 0;
    for (int i = 0; i < size; ++i) {
        bool is_duplicate = false;
        for (int k = 0; k < j; ++k) {
            if (result[k] == array[i]) {
                is_duplicate = true;
                break;
            }
        }
        if (!is_duplicate) {
            result[j++] = array[i];
        }
    }
    *result_size = j;
}
#endif


