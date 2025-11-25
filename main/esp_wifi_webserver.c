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
cJSON *offset_json                      = NULL;
cJSON *calibration_json                 = NULL;

#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI)
    extern void process_dali_tasks(uint8_t index, uint8_t is_toggle);
#endif
uint8_t node_index, ep_index;
void query_all_groups_task(void* args);
void remove_scene_task(void* args);

void remove_duplicates(int* array, int size, int* result, int* result_size);

#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH)
    void add_scene_task(void* args);
    void view_scene_table_task(void* args);
#endif

char input_str[12];
const int size = MAX_DALI_ADDRESSES;//sizeof(dali_nvs_stt[0].device_ids) / sizeof(dali_nvs_stt[0].device_ids[0]);
// Allocate memory for the result arrays (same size as the original array)
int temp_result[MAX_DALI_ADDRESSES];
int temp_result_size;

int final_result[MAX_DALI_ADDRESSES];
int final_result_size;

uint8_t tmp_selected_ids[64] = {0};

int map(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH)
    void add_scene_task(void* args);
    void view_scene_table_task(void* args);
#endif

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

    

    int fxnInt = 0;//atoi( fxn->valuestring);
    if(cJSON_IsNumber(fxn)){
        fxnInt = fxn->valueint;    
    }else if(cJSON_IsString(fxn)) {
        if( fxn->valuestring != NULL )
            fxnInt = atoi( fxn->valuestring);
    }
    printf("Function: %d\n", fxnInt);
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
    #elif (USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN)
        case 60: // Set curtain offset time (expecting seconds, convert to ms)
            offset_json = cJSON_GetObjectItem(root, "offset");
            if (offset_json == NULL) {
                printf("Missing JSON keys index\n");
                cJSON_Delete(root);
                return;
            } 
            // Convert seconds to milliseconds
            device_info[0].curtain_motor_start_offset = (uint8_t)(offset_json->valueint/10);
            
            ESP_LOGI(TAG, "Curtain offset set via JSON: %ds (%.1fms)", 
                        device_info[0].curtain_motor_start_offset, offset_json->valueint);
 
            calibration_json = cJSON_GetObjectItem(root, "calibration");
            if (calibration_json == NULL) {
                printf("Missing JSON keys index\n");
                cJSON_Delete(root);
                return;
            }             //12000/100
            // Convert seconds to milliseconds
            device_info[0].curtain_motor_total_time = (uint32_t)(calibration_json->valueint/1000);
            
            ESP_LOGI(TAG, "Curtain calibration set via JSON: %ds (%dms)", 
                        device_info[0].curtain_motor_total_time, (int)calibration_json->valueint);
            nuos_store_data_to_nvs(0);            
            break;

            
    #else
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI )
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
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI)
            case 12:  //Set Intensity of DALI Group
                //{fxn: 12, start_addr: 0, max_addr: 15}
                sindex = cJSON_GetObjectItem(root, "max_addr");
                if (sindex == NULL) {
                    printf("Missing JSON keys max_addr\n");
                    cJSON_Delete(root);
                    return;
                }   

                int max_address = sindex->valueint;
                int start_address = 0;

                cJSON *sstartAddr = cJSON_GetObjectItem(root, "start_addr");
                if (sstartAddr == NULL) {
                    printf("Missing JSON keys start_addr\n");
                    cJSON_Delete(root);
                    start_address = 20;
                }else{
                    start_address = sstartAddr->valueint;
                }

                printf("start_address:%d, max_addr: %d\n", start_address, max_address);
                //setNVSDaliNodesStartAddrCounts(start_address);
                setNVSDaliNodesCommissioningCounts(max_address);
                
                start_dali_addressing(start_address, max_address);
                break; 
        #endif
        
    #endif
case 40:
            //{"fxn":"40","group_id":1,"scene_ids":[1,2,3,4],"control_type":2,"scn_ctrl_type":1}
            scene_group_switch_info.group_id = cJSON_GetObjectItem(root, "group_id")->valueint;
            cJSON *scenes1 = cJSON_GetObjectItem(root, "scene_ids");
            for (int i = 0; i < cJSON_GetArraySize(scenes1); ++i) {
                scene_group_switch_info.scene_ids[i] = cJSON_GetArrayItem(scenes1, i)->valueint;
            }
            scene_group_switch_info.control_type = cJSON_GetObjectItem(root, "control_type")->valueint;
            scene_group_switch_info.scn_ctrl_type = cJSON_GetObjectItem(root, "scn_ctrl_type")->valueint;
            nuos_store_dali_scene_switch_data_to_nvs(&scene_group_switch_info);
        break;

        case 42:
            //{fxn: "42", index: 1, id: "scene2"}
            cJSON * sindex6 = cJSON_GetObjectItem(root, "index");
            if (sindex6 == NULL) {
                printf("Missing JSON keys index\n");
                cJSON_Delete(root);
                return;
            }  
            
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI)
            int index33 = sindex6->valueint;
            if(scene_group_switch_info.control_type == 2){
                process_dali_tasks(index33, false);
            }else{
                process_dali_tasks(index33, true);
            }
            #endif
        break;
        
        case 41:
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI)
            cJSON * sindex22 = cJSON_GetObjectItem(root, "input_id");
            if (sindex22 == NULL) {
                printf("Missing JSON keys index\n");
                cJSON_Delete(root);
                return;
            } 

            int index22 = sindex22->valueint;
            //printf("sindex: %d\n", index);

            // Get the values array
            cJSON *valuesArray2 = cJSON_GetObjectItem(root, "selected_ids");
            if (!cJSON_IsArray(valuesArray2))
            {
                ESP_LOGE(TAG, "Values is not an array");
                cJSON_Delete(root);
                return;
            }
            // dali_nvs_stt[index].group_id = scene_group_switch_info.scene_ids[index];
            // Iterate over the array and print the values
            int arraySize1 = cJSON_GetArraySize(valuesArray2);      
            int jj = 0;  // Remove invalids
            for (int i = 0; i < arraySize1; ++i)
            {
                cJSON *item1 = cJSON_GetArrayItem(valuesArray2, i);
                if (cJSON_IsNumber(item1))
                {
                    int value = item1->valueint;
                    
                    temp_result[i] = value;
                    if (temp_result[i] != DALI_INVALID_ADDRESS) {
                        final_result[jj++] = temp_result[i];
                    }
                    ESP_LOGI(TAG, "Value at index %d: %d", i, temp_result[i]);
                    
                }
            }
            final_result_size = jj;
            //remove 
            if(scene_group_switch_info.control_type == 1 ){
                for (int i = 0; i < scene_group_switch_info.total_ids[index22]; ++i) {
                    nuos_dali_remove_light_from_group(scene_group_switch_info.device_ids[index22][i], scene_group_switch_info.scene_ids[index22]);
                    vTaskDelay(5);
                }
            }
            // Remove duplicates
            jj=0;
            for (int i = 0; i < final_result_size; ++i) {
                bool is_duplicate = false;
                for (int k = 0; k < jj; ++k) {
                    if (scene_group_switch_info.device_ids[index22][k] == final_result[i]) {
                        is_duplicate = true;
                        break;
                    }
                }
                if (!is_duplicate) {
                    scene_group_switch_info.device_ids[index22][jj++] = final_result[i];
                }
            }

            scene_group_switch_info.total_ids[index22] = jj;
            
            // Print the final result array
            printf("Array without zeros and duplicates: ");
            for (int i = 0; i < scene_group_switch_info.total_ids[index22]; ++i) {
                printf("%d ", scene_group_switch_info.device_ids[index22][i]);
                if(scene_group_switch_info.control_type == 1 ){
                    nuos_dali_add_light_to_group(scene_group_switch_info.device_ids[index22][i], scene_group_switch_info.scene_ids[index22]);
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                }
            }
            printf("\n");
            for (int p = scene_group_switch_info.total_ids[index22]; p < 64; p++) {
                scene_group_switch_info.device_ids[index22][p] = DALI_INVALID_ADDRESS;
            }

            nuos_store_dali_scene_switch_data_to_nvs(&scene_group_switch_info); 
            #endif         
        break;  
        
        



        /////////////////////////////////////////////////////////////////////

        // ---- Add near other cases in parse_json() ----
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI)

        case 301:  //toggle group id
        {
            // Immediate toggle / longpress command for a DALI address
            // payload: { fxn:53, dali_id: <int>, state: 0|1|2, group_id: <int> }
            cJSON *dali_id = cJSON_GetObjectItem(root, "dali_id");
            cJSON *g_val  = cJSON_GetObjectItem(root, "val");
            cJSON *g_id    = cJSON_GetObjectItem(root, "group_id");
            cJSON *s_id    = cJSON_GetObjectItem(root, "scene_id");
            if (!g_val) { cJSON_Delete(root); return; }
            int state = g_val->valueint;

            if (g_id) {
                int group = g_id ? g_id->valueint : 0;
                printf("Immediate toggle state:%d group:%d\n", state, group);
                nuos_dali_set_state_group(group, state);
            }else{
                if (dali_id) {
                    
                    int did = dali_id ? dali_id->valueint : 0;
                    printf("Immediate toggle state:%d did:%d\n", state, did);
                    nuos_dali_set_state(did, state);
                }
            }
        }
        break;
        case 302:  //brightness control group id
        {
            // Immediate toggle / longpress command for a DALI address
            // payload: { fxn:53, dali_id: <int>, state: 0|1|2, group_id: <int> }
            cJSON *dali_id = cJSON_GetObjectItem(root, "dali_id");
            cJSON *sbright  = cJSON_GetObjectItem(root, "val");
            cJSON *g_id    = cJSON_GetObjectItem(root, "group_id");
            if (!sbright) { cJSON_Delete(root); return; }
            int level = sbright->valueint;
            if (g_id) {
                int group = g_id ? g_id->valueint : 0;
                printf("Immediate toggle level:%d group:%d\n", level, group);
                uint8_t val = map(level, 0, 100, 0, 255);
                nuos_dali_set_group_brightness(group, 0, val);
            }else{
                if (dali_id) {
                    
                    int did = dali_id ? dali_id->valueint : 0;
                    uint8_t val = map(level, 0, 100, 0, 255);
                    printf("Immediate toggle level:%d did:%d\n", val, did);
                    nuos_dali_set_brightness(did, val);
                }                
            }
        }
        break;
        case 303:  //CCT control group id
        {
            // Immediate toggle / longpress command for a DALI address
            // payload: { fxn:53, dali_id: <int>, state: 0|1|2, group_id: <int> }
            cJSON *dali_id = cJSON_GetObjectItem(root, "dali_id");
            cJSON *sVal  = cJSON_GetObjectItem(root, "val");
            cJSON *g_id    = cJSON_GetObjectItem(root, "group_id");
            if (!sVal) { cJSON_Delete(root); return; }
            int cct_val = sVal->valueint;
            if (g_id) {
                int group = g_id ? g_id->valueint : 0;
                printf("Immediate toggle level:%d group:%d\n", cct_val, group);
                nuos_dali_set_group_color_temperature(group, 0, cct_val);
            }else{
                
                if (dali_id) {
                    
                    int did = dali_id ? dali_id->valueint : 0;
                    printf("Immediate toggle cct:%d did:%d\n", cct_val, did);
                    nuos_dali_set_cct_color(did, cct_val);
                } 
            }
        }
        break;
        case 53:
        {
            // Immediate toggle / longpress command for a DALI address
            // payload: { fxn:53, dali_id: <int>, state: 0|1|2, group_id: <int> }
            cJSON *dali_id = cJSON_GetObjectItem(root, "dali_id");
            cJSON *sstate  = cJSON_GetObjectItem(root, "state");
            cJSON *g_id    = cJSON_GetObjectItem(root, "group_id");
            if (!dali_id || !sstate) { cJSON_Delete(root); return; }
            int id = dali_id->valueint;
            int state = sstate->valueint;
            int group = g_id ? g_id->valueint : 0;
            printf("Immediate toggle dali_id:%d state:%d group:%d\n", id, state, group);

            // Implement immediate action (toggle or select)
            // Example: if state==1 turn ON device id, if state==0 turn OFF, if state==2 treat as longpress selection
            if (state == 1) {
                // call DALI on function (example)
                nuos_dali_set_state(id, 1);
            } else if (state == 0) {
                nuos_dali_set_state(id, 0);
            } else if (state == 2) {
                // long press selection (UI only), store in temporary buffer until explicit Save (fxn 61)
                // For convenience, we can mark in RAM array `tmp_selected_ids` (create globally)
                // Example:
                tmp_selected_ids[id] = 1; // set flagged for membership
            }
        }
        break;

        case 60:
        {
            // Save Tab2 (DALI address / Tab2 values)
            // payload example: { fxn:60, selected_address: 10 }
            cJSON *sel = cJSON_GetObjectItem(root, "selected_address");
            if (!sel) { cJSON_Delete(root); return; }
            int sel_addr = sel->valueint;
            printf("Saving Tab2 selected_address=%d\n", sel_addr);

            // Persist to NVS / existing store function
            // Provide your own implementation: e.g., write_dali_selected_address_to_nvs(sel_addr);
            //nuos_store_selected_dali_address_to_nvs(sel_addr); // implement this helper

            if(sel_addr > MAX_DALI_ADDRESSES) sel_addr = MAX_DALI_ADDRESSES;
            setNVSDaliNodesCommissioningCounts(sel_addr);
            //start_dali_addressing(sel_addr);               
        } //MunishK
        break;

        case 61:
        {
            // Save Group membership or single longpress event
            // payload: { fxn:61, action: "save_group" | "single", group_id: <int>, dali_ids: [..] }
            cJSON *action = cJSON_GetObjectItem(root, "action");
            if (!action) { cJSON_Delete(root); return; }
            const char *act = action->valuestring;
            cJSON *group = cJSON_GetObjectItem(root, "group_id");
            int group_id = group ? group->valueint : 0;

            if (strcmp(act, "save_group") == 0) {
                cJSON *ids = cJSON_GetObjectItem(root, "dali_ids");
                if (!ids || !cJSON_IsArray(ids)) { cJSON_Delete(root); return; }
                // copy ids to your persistent array and store to NVS
                int total = cJSON_GetArraySize(ids);
                int idx = 0;
                for (int i = 0; i < total; i++) {
                    final_result[idx++] = cJSON_GetArrayItem(ids, i)->valueint; // reuse array
                }
                // Save to NVS: implement nuos_store_group_membership_to_nvs(group_id, final_result, idx)
                nuos_store_group_membership_to_nvs(group_id, final_result, idx);
                printf("Saved group %d membership, count=%d\n", group_id, idx);
            } else if (strcmp(act, "single") == 0) {
                // single longpress selection persisted
                cJSON *dali_id = cJSON_GetObjectItem(root, "dali_id");
                cJSON *state = cJSON_GetObjectItem(root, "state"); // 2 expected
                if (!dali_id || !state) { cJSON_Delete(root); return; }
                int id = dali_id->valueint;
                int st = state->valueint;
                // e.g., toggle in a small NVS list representing "selected for last group"
                nuos_toggle_tmp_selection(id, group_id, st); // implement helper that marks selection in RAM/NVS
            }
        }
        break;

        case 62:
        {
            // Save Scene membership (Tab4)
            // payload: { fxn:62, action:'save_scene', group_id: <int>, dali_ids: [..] }
            cJSON *ids = cJSON_GetObjectItem(root, "dali_ids");
            cJSON *g = cJSON_GetObjectItem(root, "group_id");
            int group_id = g ? g->valueint : 0;
            if (!ids || !cJSON_IsArray(ids)) { cJSON_Delete(root); return; }

            int total = cJSON_GetArraySize(ids);
            int idx = 0;
            for (int i = 0; i < total; i++) {
                final_result[idx++] = cJSON_GetArrayItem(ids, i)->valueint;
            }
            // Persist: implement nuos_store_scene_membership_to_nvs(group_id, final_result, idx)
            nuos_store_scene_membership_to_nvs(group_id, final_result, idx);
            printf("Saved scene membership for group %d count=%d\n", group_id, idx);
        }
        break;

case 51:
case 201: // Group add/remove via fxn in POST body
        {
            // Expected JSON: { "fxn":201, "action":"add"|"remove", "group_id": <0..15>, "dali_ids": [0,1,2,...] }
            cJSON *action = cJSON_GetObjectItem(root, "action");
            cJSON *group = cJSON_GetObjectItem(root, "group_id");
            cJSON *ids   = cJSON_GetObjectItem(root, "dali_ids");

            if (!action || !group || !ids || !cJSON_IsString(action) || !cJSON_IsArray(ids)) {
                ESP_LOGW(TAG, "fxn201: invalid payload");
                break;
            }


            int gid = 0;
            if (group && cJSON_IsString(group)) {
                gid = atoi(group->valuestring);
            }else{
                gid = group->valueint;
            }
            if (gid < 0 || gid >= GROUP_COUNT) {
                ESP_LOGW(TAG, "fxn201: group_id out of range %d", gid);
                break;
            }

            uint8_t buf[DALI_ADDR_COUNT] = {0};
            if (read_group_blob(gid, buf) != ESP_OK) {
                // initialize zero if missing
                memset(buf, 0, sizeof(buf));
            }
            const char *act = action->valuestring;
            // if(!cJSON_IsString(act)){
            //     break;                
            // }
            cJSON *it = NULL;
            if (strcmp(act, "add") == 0) {
                cJSON_ArrayForEach(it, ids) {
                    if (cJSON_IsNumber(it)) {
                        int id = it->valueint;
                        if (id >= 0 && id < DALI_ADDR_COUNT) {
                            buf[id] = 1;
                            nuos_dali_add_light_to_group(id, gid);
                        }
                    }
                }
            } else if (strcmp(act, "remove") == 0) {
                cJSON_ArrayForEach(it, ids) {
                    if (cJSON_IsNumber(it)) {
                        int id = it->valueint;
                        if (id >= 0 && id < DALI_ADDR_COUNT) {
                            buf[id] = 0;
                            nuos_dali_remove_light_from_group(id, gid);
                        }
                    }
                }
            } else {
                ESP_LOGW(TAG, "fxn201: unknown action '%s'", act);
                break;
            }

            if (write_group_blob(gid, buf) != ESP_OK) {
                ESP_LOGE(TAG, "fxn201: failed to write group %d to NVS", gid);
            } else {
                ESP_LOGI(TAG, "fxn201: group %d updated (action=%s)", gid, act);
            }
        }
        break;

        case 202: // Scene add/remove via fxn in POST body
        {
        // Expected JSON: { "fxn":202, "action":"add"|"remove", "group_id": <0..15>, "scenes": [0,1,2,...] }
        // Attempt to parse new device-array format first
            cJSON *devices = cJSON_GetObjectItem(root, "devices");
            cJSON *action = cJSON_GetObjectItem(root, "action");      // must be "add" or "remove"
            cJSON *scene_id = cJSON_GetObjectItem(root, "scene_id");
            int sid = (scene_id && cJSON_IsNumber(scene_id)) ? scene_id->valueint : 0;

            if (devices && cJSON_IsArray(devices)) {
                // Validate action string
                const char *act = NULL;
                if (action && cJSON_IsString(action)) {
                    act = action->valuestring;
                } else {
                    ESP_LOGW(TAG, "fxn202(devices): missing or invalid 'action' (expected 'add' or 'remove')");
                    // You can choose to return/break here; currently we will skip processing
                    break;
                }

                bool is_add = (strcmp(act, "add") == 0);
                bool is_remove = (strcmp(act, "remove") == 0);
                if (!is_add && !is_remove) {
                    ESP_LOGW(TAG, "fxn202(devices): unknown action '%s' (expected 'add' or 'remove')", act);
                    break;
                }

                ESP_LOGI(TAG, "fxn202: received devices payload (scene_id=%d, action=%s)", sid, act);

                cJSON *dev = NULL;
                int processed = 0;
                cJSON_ArrayForEach(dev, devices) {
                    if (!cJSON_IsObject(dev)) {
                        ESP_LOGW(TAG, "fxn202(devices): skipping non-object array entry");
                        continue;
                    }

                    cJSON *j_dali       = cJSON_GetObjectItem(dev, "dali_id");
                    cJSON *j_power      = cJSON_GetObjectItem(dev, "power");
                    cJSON *j_brightness = cJSON_GetObjectItem(dev, "brightness");
                    cJSON *j_cct        = cJSON_GetObjectItem(dev, "cct");

                    if (!j_dali || !cJSON_IsNumber(j_dali)) {
                        ESP_LOGW(TAG, "fxn202(devices): missing or invalid 'dali_id' - skipping entry");
                        continue;
                    }

                    int did = j_dali->valueint;
                    if (did < 0 || did >= 64) {
                        ESP_LOGW(TAG, "fxn202(devices): dali_id out of range %d - skipping", did);
                        continue;
                    }

                    int power = (j_power && cJSON_IsNumber(j_power)) ? j_power->valueint : 0;
                    int brightness = (j_brightness && cJSON_IsNumber(j_brightness)) ? j_brightness->valueint : 0;
                    int cct = (j_cct && cJSON_IsNumber(j_cct)) ? j_cct->valueint : 0;

                    if (is_add) {
                        ESP_LOGI(TAG, "fxn202(devices): ADD device %d to scene %d (p=%d, br=%d, cct=%d)", did, sid, power, brightness, cct);
       
                            if(power==0) brightness = 0; //this is cruicial
                            nuos_dali_add_device_to_scene((uint8_t)did, (uint8_t)sid, brightness, cct);

                        // Optionally apply device settings now:
                        // nuos_dali_set_device_power(did, power);
                        // nuos_dali_set_device_level(did, brightness);
                        // nuos_dali_set_device_cct(did, cct);
                    } else { // remove
                        ESP_LOGI(TAG, "fxn202(devices): REMOVE device %d from scene %d", did, sid);
                        nuos_dali_remove_device_from_scene((uint8_t)did, (uint8_t)sid);
                    }
                    processed++;
                } // end foreach device

                ESP_LOGI(TAG, "fxn202(devices): processed %d device entries (action=%s)", processed, act);
                esp_err_t perr = persist_devices_from_cjson_array(devices, sid, act);
                if (perr != ESP_OK) {
                    ESP_LOGW(TAG, "persist_devices_from_cjson_array failed: %s", esp_err_to_name(perr));
                } else {
                    ESP_LOGI(TAG, "persist_devices_from_cjson_array success");
                    // update in-memory global store too:
                    if (g_device_scene_store) free(g_device_scene_store);
                    g_device_scene_store = NULL;
                    g_device_scene_count = 0;
                    device_scene_t *tmp = NULL;
                    size_t tmp_count = 0;
                    if (load_device_scene_store(&tmp, &tmp_count) == ESP_OK && tmp_count > 0) {
                        g_device_scene_store = tmp;
                        g_device_scene_count = tmp_count;
                    }
                }               
                break; // handled devices case; skip legacy group handler
            }
            // after processing devices loop:
            /********************* */

        }//munish
        break;
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

    cb_requests_counts = 0;
    cb_response_counts = 0;
    
    return ESP_OK;
}

// #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN)
// Modify your http_get_handler to serve the embedded HTML
esp_err_t http_get_handler(httpd_req_t *req) {
    // Add these external declarations for the embedded HTML
    extern const char index_html_start[] asm("_binary_index_html_start");
    extern const char index_html_end[]   asm("_binary_index_html_end");
    // Calculate HTML size from embedded binary
    size_t html_size = index_html_end - index_html_start;
    
    // Set proper UTF-8 headers
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Content-Type", "text/html; charset=utf-8");
    return httpd_resp_send(req,  (const char*)index_html_start, html_size);
}
// #else
// esp_err_t http_get_handler(httpd_req_t *req) {
//     printf("On http get :%d\n", strlen(webpage));
//     httpd_resp_send(req, webpage, strlen(webpage));
//     return ESP_OK;
// }

// #endif

#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)

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
                    cJSON_AddNumberToObject(item_json, "val", webpageItem[i].value);
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
                        httpd_resp_send(req, jsonWiFiScanListStr, HTTPD_RESP_USE_STRLEN);
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
                } else if(fxn == 10){  //read already saved dali addresses
                    cJSON *json = cJSON_CreateArray();
                    for (size_t i = 0; i < dali_nvs_stt[0].total_ids; i++) {
                        // Start creating a cJSON object
                        //if(dali_nvs_stt[0].device_ids[i] != DALI_INVALID_ADDRESS){
                            cJSON *item_json = cJSON_CreateObject();
                            // Add fields to the cJSON object
                            cJSON_AddNumberToObject(item_json, "Id", dali_nvs_stt[0].device_ids[i]);
                            cJSON_AddNumberToObject(item_json, "state", dali_nvs_stt[0].state);
                            cJSON_AddNumberToObject(item_json, "toggle", dali_nvs_stt[0].state);
                            cJSON_AddItemToArray(json, item_json);
                        //}
                    }                                             
                    response = cJSON_Print(json);
                    cJSON_Delete(json);
                }else if(fxn == 12){  // read new dali addresses
                        uint8_t foundAddresses[64];
                        dali_nvs_stt[0].total_ids = get_all_dali_addresses(foundAddresses);

                        cJSON *json = cJSON_CreateArray();
                        for (size_t i = 0; i < dali_nvs_stt[0].total_ids; i++) {
                            // Start creating a cJSON object
                            cJSON *item_json = cJSON_CreateObject();
                            // Add fields to the cJSON object
                            cJSON_AddNumberToObject(item_json, "Id", foundAddresses[i]>>1);
                            cJSON_AddNumberToObject(item_json, "state", 0);
                            cJSON_AddNumberToObject(item_json, "toggle", 0);
                            cJSON_AddItemToArray(json, item_json);
                        }
                                                
                        for (int p = 0; p < dali_nvs_stt[0].total_ids; p++) {
                            dali_nvs_stt[0].device_ids[p] = foundAddresses[p]>>1;
                        }       

                        for (int p = dali_nvs_stt[0].total_ids; p < MAX_DALI_ADDRESSES; p++) {
                            dali_nvs_stt[0].device_ids[p] = DALI_INVALID_ADDRESS;
                        }  
                        nuos_store_dali_data_to_nvs(index); 
                   
                        response = cJSON_Print(json);
                        cJSON_Delete(json);

                }else if(fxn == 11){
                    if (httpd_query_key_value(query, "id", value_data, sizeof(value_data)) == ESP_OK) {
                        int id = atoi(value_data);
                        if (httpd_query_key_value(query, "state", value_data, sizeof(value_data)) == ESP_OK) {
                            int state = atoi(value_data);
                            printf("id:%d  state:%d\n", id, state);
                            nuos_dali_set_state(id, state);
                        }
                    }
                }else if(fxn == 15){
                    cJSON *item_json = cJSON_CreateObject();
                    // Add fields to the cJSON object
                    cJSON_AddNumberToObject(item_json, "group_id", scene_group_switch_info.group_id);
                    // cJSON *scene_ids_array = cJSON_CreateIntArray(
                    //     (const char*[]){
                    //     scene_group_switch_info.scene_ids[0], 
                    //     scene_group_switch_info.scene_ids[1], 
                    //     scene_group_switch_info.scene_ids[2], 
                    //     scene_group_switch_info.scene_ids[3]}, 4);


                    // cJSON *scene_ids_array = cJSON_CreateStringArray(
                    //     (const char*[]){
                    //         scene_group_switch_info.scene_ids[0],
                    //         scene_group_switch_info.scene_ids[1],
                    //         scene_group_switch_info.scene_ids[2],
                    //         scene_group_switch_info.scene_ids[3]
                    //     }, 4);
                    /* corrected: create int array and use cJSON_CreateIntArray */
                    int scene_ids[4];
                    scene_ids[0] = scene_group_switch_info.scene_ids[0];
                    scene_ids[1] = scene_group_switch_info.scene_ids[1];
                    scene_ids[2] = scene_group_switch_info.scene_ids[2];
                    scene_ids[3] = scene_group_switch_info.scene_ids[3];

                    cJSON *scene_ids_array = cJSON_CreateIntArray(scene_ids, 4);

                    cJSON_AddItemToObject(item_json, "scene_ids", scene_ids_array);
                    cJSON_AddStringToObject(item_json, "control_type", switch_ctrl_type[scene_group_switch_info.control_type]);
                    cJSON_AddStringToObject(item_json, "scn_ctrl_type", scene_ctrl_type[scene_group_switch_info.scn_ctrl_type]);
                                              
                    response = cJSON_Print(item_json);
                    cJSON_Delete(item_json);
                }else if(fxn == 16){
                    cJSON *item_json = cJSON_CreateArray();
                    for (int i = 0; i < 4; i++) {
                        cJSON *arr = cJSON_CreateArray();
                        for (int j = 0; j < scene_group_switch_info.total_ids[i]; j++) {
                            //if (scene_group_switch_info.device_ids[i][j]) {
                                cJSON_AddItemToArray(arr, cJSON_CreateNumber(scene_group_switch_info.device_ids[i][j])); // 1-based button IDs
                            //}
                        }
                        cJSON_AddItemToArray(item_json, arr);
                    }
                    response = cJSON_Print(item_json);
                    cJSON_Delete(item_json);
                    if (!response) {
                        httpd_resp_send_500(req);
                        return ESP_FAIL;
                    }                    
                }else if (fxn == 20) {
                    // Return saved Tab2 selection
                    cJSON *obj = cJSON_CreateObject();
                    int saved_addr = nuos_read_selected_dali_address_from_nvs(); // implement
                    cJSON_AddNumberToObject(obj, "selected_address", saved_addr);
                    const char *resp = cJSON_Print(obj);
                    cJSON_Delete(obj);
                    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
                    free((void*)resp);
                    return ESP_OK;

                } else if (fxn == 21) {
                    // Return saved group membership for given group_id (or default group if not provided)
                    // client calls: /items?fxn=21&group_id=3
                    char value_group[32];
                    int group_id = 0;
                    if (httpd_query_key_value(query, "group_id", value_group, sizeof(value_group)) == ESP_OK) {
                        group_id = atoi(value_group);
                    }
                    if (group_id < 0) group_id = 0;
                    if (group_id >= GROUP_COUNT) group_id = 0;

                    // Try to use high-level helper if available; otherwise read raw blob
                    int ids[DALI_ADDR_COUNT];
                    int ids_count = 0;
                    memset(ids, 0, sizeof(ids));

                    // If you have a helper: nuos_read_group_membership_from_nvs(group_id, ids, &ids_count);
                    // If not, use read_group_blob(..) to read the bitmap and compute ids
                    uint8_t blob[DALI_ADDR_COUNT] = {0};
                    if (read_group_blob(group_id, blob) == ESP_OK) {
                        printf("---> READ NVS OK\n");
                        for (int i = 0; i < DALI_ADDR_COUNT; ++i) {
                            if (blob[i]) {
                                printf("---> i:%d\n", i);
                                ids[ids_count++] = i;
                            }
                        }
                    } else {
                        printf("---> fallback: attempt to call wrapper and set ids_count appropriately\n");
                        // fallback: attempt to call wrapper and set ids_count appropriately
                        // nuos_read_group_membership_from_nvs(group_id, ids, &ids_count);
                    }

                    cJSON *obj = cJSON_CreateObject();
                    cJSON_AddNumberToObject(obj, "group_id", group_id);
                    cJSON *arr = cJSON_CreateIntArray(ids, ids_count);
                    cJSON_AddItemToObject(obj, "selected_ids", arr);
                    const char *resp = cJSON_Print(obj);
                    cJSON_Delete(obj);
                    printf("%s\n", resp);
                    httpd_resp_set_type(req, "application/json");
                    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
                    free((void*)resp);
                    return ESP_OK;

                } else if (fxn == 22) {
                    // Return saved scenes for a given group
                    // client calls: /items?fxn=22&group_id=3
                    char value_group2[32];
                    int group_id2 = 0;
                    if (httpd_query_key_value(query, "group_id", value_group2, sizeof(value_group2)) == ESP_OK) {
                        group_id2 = atoi(value_group2);
                    }
                    if (group_id2 < 0) group_id2 = 0;
                    if (group_id2 >= GROUP_COUNT) group_id2 = 0;

                    // Read scene blob for this group (SCENE_COUNT slots)
                    uint8_t scene_blob[SCENE_COUNT] = {0};
                    int scene_ids[SCENE_COUNT];
                    int scene_count = 0;
                    memset(scene_ids, 0, sizeof(scene_ids));

                    if (read_scene_blob(group_id2, scene_blob) == ESP_OK) {
                        for (int i = 0; i < SCENE_COUNT; ++i) {
                            if (scene_blob[i]) {
                                scene_ids[scene_count++] = i;
                            }
                        }
                    } else {
                        // fallback: try wrapper if present
                        // nuos_read_scene_membership_from_nvs(group_id2, scene_ids, &scene_count);
                    }

                    cJSON *obj2 = cJSON_CreateObject();
                    cJSON_AddNumberToObject(obj2, "group_id", group_id2);
                    cJSON *sarr = cJSON_CreateIntArray(scene_ids, scene_count);
                    cJSON_AddItemToObject(obj2, "selected_scenes", sarr);
                    const char *resp2 = cJSON_Print(obj2);
                    cJSON_Delete(obj2);

                    httpd_resp_set_type(req, "application/json");
                    httpd_resp_send(req, resp2, HTTPD_RESP_USE_STRLEN);
                    free((void*)resp2);
                    return ESP_OK;
                }                
            }
        }
        httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
#endif

// Handler for curtain values API - now using /items endpoint
esp_err_t curtain_values_get_handler(httpd_req_t *req) {
    char json_response[100];
    #if (USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN)
    snprintf(json_response, sizeof(json_response), 
             "{\"offset\":%d,\"calibration\":%d}", 
             device_info[0].curtain_motor_start_offset*10, device_info[0].curtain_motor_total_time*1000);
    #endif  
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json_response, strlen(json_response));
}

void start_webserver() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;  // Increase the stack size
    httpd_handle_t server = NULL;
    httpd_uri_t index_uri = {
        .uri       = "/",
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
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI)
        httpd_uri_t items_uri = {
            .uri       = "/items",
            .method    = HTTP_GET,
            .handler   = http_get_items_handler,
            .user_ctx  = NULL
        };
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN)    
        httpd_uri_t items_uri = {
            .uri       = "/items",
            .method    = HTTP_GET,
            .handler   = curtain_values_get_handler,
            .user_ctx  = NULL
        };    
    #endif
    if (httpd_start(&server, &config) == ESP_OK) {
    	httpd_register_uri_handler(server, &index_uri);
		httpd_register_uri_handler(server, &submit_uri);
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI)
            httpd_register_uri_handler(server, &items_uri);
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN) 
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


