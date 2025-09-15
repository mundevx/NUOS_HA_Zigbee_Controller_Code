

#include "app_hardware_driver.h"
#include "esp_zigbee_core.h"
#include "esp_err.h"
#include "esp_check.h"
#include "app_zigbee_group_commands.h"
#include "app_zigbee_scene_commands.h"
#include "esp_zigbee_core.h"
#include "zdo/esp_zigbee_zdo_command.h"
#include "app_zigbee_clusters.h"

static const char *TAG = "ESP_ZB_SCENE_COMMANDS";

extern void nuos_set_attribute_cluster_2(const esp_zb_zcl_set_attr_value_message_t *message);
extern esp_err_t nuos_set_state_attribute(uint8_t index);
extern esp_err_t nuos_set_color_temp_level_attribute(uint8_t index);
extern esp_err_t nuos_set_fan_attribute(uint8_t index);
extern void xyToRgb(uint16_t x, uint16_t y, float brightness, uint8_t *r, uint8_t *g, uint8_t *b);
extern void nuos_set_scene_group_cluster(const esp_zb_zcl_recall_scene_message_t *message);

void nuos_zb_scene_query_all_scenes_request(uint16_t group_id, uint8_t src_endpoint)
{
    esp_zb_zcl_scenes_get_scene_membership_cmd_t cmd_req;
    // memset(&cmd_req, 0, sizeof(cmd_req));
    cmd_req.group_id = group_id;
    cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_GROUP_ENDP_NOT_PRESENT;
    cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = group_id;
    cmd_req.zcl_basic_cmd.dst_endpoint = 0; // Groupcast does not use endpoint
    cmd_req.zcl_basic_cmd.src_endpoint = src_endpoint;
    esp_zb_zcl_scenes_get_scene_membership_cmd_req(&cmd_req);
}


esp_zb_zcl_scenes_extension_field_t nuos_zb_scene_set_onoff_cluster_extension_field(uint8_t on_off_value, uint16_t cluster_id){
    // 1 for ON, 0 for OFF
    esp_zb_zcl_scenes_extension_field_t on_off_extension_field = {
        .cluster_id = cluster_id, // Cluster ID for On/Off cluster
        .length = sizeof(on_off_value),
        .extension_field_attribute_value_list = &on_off_value,
        .next = NULL // Initially, no next extension field
    };
    return on_off_extension_field;
}

esp_zb_zcl_scenes_extension_field_t nuos_zb_scene_set_onoff_level_cluster_extension_field(uint8_t on_off_value, uint8_t brightness_value){
    // brightness level (0-255)
    esp_zb_zcl_scenes_extension_field_t field1 = nuos_zb_scene_set_onoff_cluster_extension_field(on_off_value, 0x06);
    esp_zb_zcl_scenes_extension_field_t level_control_extension_field = {
        .cluster_id = 0x0008, // Cluster ID for Level Control cluster
        .length = sizeof(brightness_value),
        .extension_field_attribute_value_list = &brightness_value,
        .next = &field1 // Link to the On/Off extension field
    };
    return level_control_extension_field;
}


void nuos_zb_scenes_store_scene_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep, uint16_t dst_addr_short){
    esp_zb_zcl_scenes_store_scene_cmd_t cmd_req = {
        .zcl_basic_cmd.dst_addr_u.addr_short = dst_addr_short,
        .zcl_basic_cmd.dst_endpoint = dst_ep,
        .zcl_basic_cmd.src_endpoint = src_ep,
        .group_id = group_id,
        .scene_id = scene_id,
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
    };
    //esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_scenes_store_scene_cmd_req(&cmd_req);
    //esp_zb_lock_release(); 
}


void nuos_zb_scenes_remove_scene_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep, uint16_t dst_addr_short, uint16_t address_mode){
    esp_zb_zcl_scenes_remove_scene_cmd_t cmd_req;
    cmd_req.group_id = group_id;
    cmd_req.scene_id = scene_id;
    cmd_req.address_mode = address_mode;
    cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = dst_addr_short;
    cmd_req.zcl_basic_cmd.dst_endpoint = dst_ep;
    cmd_req.zcl_basic_cmd.src_endpoint = src_ep;    
    esp_zb_zcl_scenes_remove_scene_cmd_req(&cmd_req);
}

typedef union colorControlExtensionField_u
{
   struct colorControlExtensionField_s
   {
        uint8_t mode;
        uint8_t r;
        uint8_t g;
        uint8_t b;
        uint8_t bright;
        uint16_t colorTemp;
   }fields;

   uint8_t raw[7];
}colorControlExtensionField_t;

colorControlExtensionField_t colorControlExtensionField = {0};


void nuos_zb_scenes_add_scene_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep, uint16_t dst_addr_short, 
                                        uint8_t on_off_value, uint8_t brightness_value, uint16_t colorx, uint16_t colory) {

    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2R_ON_OFF_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4R_ON_OFF_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2CH_CURTAIN)
        esp_zb_zcl_scenes_extension_field_t on_off_extension_field = {
            .cluster_id = 0x0006, // Cluster ID for On/Off cluster
            .length = sizeof(uint8_t),
            .extension_field_attribute_value_list = &on_off_value,
            .next = NULL // Link to the Level Control extension field 
        };
        esp_zb_zcl_scenes_add_scene_cmd_t cmd = {
            .zcl_basic_cmd.dst_addr_u.addr_short = dst_addr_short,
            .zcl_basic_cmd.dst_endpoint = dst_ep,
            .zcl_basic_cmd.src_endpoint = 0,
            .group_id = group_id,
            .scene_id = scene_id,
            .transition_time = 0,
            .extension_field = &on_off_extension_field,
        };        
        // esp_zb_zcl_scenes_add_scene_cmd_req(&cmd);  
        
        esp_zb_zcl_scenes_table_store(dst_ep, group_id, scene_id, 0x0000,
                                            &on_off_extension_field);        
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_ANALOG_DIMMABLE_LIGHT)
        esp_zb_zcl_scenes_extension_field_t level_control_extension_field = {
            .cluster_id = 0x0008, // Cluster ID for Level Control Cluster
            .length = sizeof(uint8_t),
            .extension_field_attribute_value_list = &brightness_value,
            .next = NULL //Initially, No Next Extension Field
        };
        esp_zb_zcl_scenes_extension_field_t on_off_extension_field = {
            .cluster_id = 0x0006, // Cluster ID for On/Off cluster
            .length = sizeof(uint8_t),
            .extension_field_attribute_value_list = &on_off_value,
            .next = &level_control_extension_field // Link to the Level Control Extension Field
        };
        esp_zb_zcl_scenes_add_scene_cmd_t cmd = {
            .zcl_basic_cmd.dst_addr_u.addr_short = dst_addr_short,
            .zcl_basic_cmd.dst_endpoint = dst_ep,
            .zcl_basic_cmd.src_endpoint = 0,
            .group_id = group_id,
            .scene_id = scene_id,
            .transition_time = 0,
            .extension_field = &on_off_extension_field,
        };        
        // esp_zb_zcl_scenes_add_scene_cmd_req(&cmd);     
        esp_zb_zcl_scenes_table_store(dst_ep, group_id, scene_id, 0x0000,
                                            &on_off_extension_field);          
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM) 
        uint16_t colorxy[] = {colorx, colory};
        if(on_off_value == 0){
            esp_zb_zcl_scenes_extension_field_t on_off_extension_field = {
                .cluster_id = 0x0006, // Cluster ID for On/Off cluster
                .length = sizeof(uint8_t),
                .extension_field_attribute_value_list = &on_off_value,
                .next = NULL // Link to the Level Control extension field
            }; 
            esp_zb_zcl_scenes_add_scene_cmd_t cmd = {
                .zcl_basic_cmd.dst_addr_u.addr_short = dst_addr_short,
                .zcl_basic_cmd.dst_endpoint = dst_ep,
                .zcl_basic_cmd.src_endpoint = 0,
                .group_id = group_id,
                .scene_id = scene_id,
                .transition_time = 0,
                .extension_field = &on_off_extension_field,
            };
            //esp_zb_lock_acquire(portMAX_DELAY);
            esp_zb_zcl_scenes_add_scene_cmd_req(&cmd); 
            //esp_zb_lock_release();              
        }else{
            
            esp_zb_zcl_scenes_extension_field_t color_control_extension_field = {
                .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, // Cluster ID for Level Control cluster
                .length = sizeof(uint32_t),
                .extension_field_attribute_value_list = &colorxy,
                .next = NULL //Initially, no next extension field
            };    
            esp_zb_zcl_scenes_extension_field_t level_control_extension_field = {
                .cluster_id = 0x0008, // Cluster ID for Level Control cluster
                .length = sizeof(uint8_t),
                .extension_field_attribute_value_list = &brightness_value,
                .next = &color_control_extension_field //Link to the Color Control extension field
            };
            esp_zb_zcl_scenes_extension_field_t on_off_extension_field = {
                .cluster_id = 0x0006, // Cluster ID for On/Off cluster
                .length = sizeof(uint8_t),
                .extension_field_attribute_value_list = &on_off_value,
                .next = &level_control_extension_field // Link to the Level Control extension field
            };
            esp_zb_zcl_scenes_add_scene_cmd_t cmd = {
                .zcl_basic_cmd.dst_addr_u.addr_short = dst_addr_short,
                .zcl_basic_cmd.dst_endpoint = dst_ep,
                .zcl_basic_cmd.src_endpoint = 0,
                .group_id = group_id,
                .scene_id = scene_id,
                .transition_time = 0,
                .extension_field = &on_off_extension_field,
            };
            //esp_zb_lock_acquire(portMAX_DELAY);
            esp_zb_zcl_scenes_add_scene_cmd_req(&cmd); 
            //esp_zb_lock_release();              
        }
    #endif
}

void nuos_zb_scenes_add_scene_curtain_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep, uint16_t dst_addr_short, 
                                        uint8_t curtain_state, uint8_t lift_percentage) {


            esp_zb_zcl_scenes_extension_field_t lift_percentage_extension_field = {
                .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, // Cluster ID for lift percentage Control cluster
                .length = sizeof(uint8_t),
                .extension_field_attribute_value_list = &lift_percentage,
                .next = NULL //Link to the Color Control extension field
            };
            esp_zb_zcl_scenes_add_scene_cmd_t cmd = {
                .zcl_basic_cmd.dst_addr_u.addr_short = dst_addr_short,
                .zcl_basic_cmd.dst_endpoint = dst_ep,
                .zcl_basic_cmd.src_endpoint = 0,
                .group_id = group_id,
                .scene_id = scene_id,
                .transition_time = 0,
                .extension_field = &lift_percentage_extension_field,
            };
            // //esp_zb_lock_acquire(portMAX_DELAY);
            //esp_zb_zcl_scenes_add_scene_cmd_req(&cmd); 
            // //esp_zb_lock_release(); 
            esp_zb_zcl_scenes_table_store(dst_ep, group_id, scene_id, 0x0000,
                                                &lift_percentage_extension_field);

}

void nuos_zb_scenes_add_scene_color_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep, uint16_t dst_addr_short, 
                                        uint8_t on_off_value, uint8_t brightness_value, uint8_t r, uint16_t g, uint16_t b, 
                                        uint8_t mode, uint16_t colorTemp) {

    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2R_ON_OFF_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4R_ON_OFF_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2CH_CURTAIN)
        esp_zb_zcl_scenes_extension_field_t on_off_extension_field = {
            .cluster_id = 0x0006, // Cluster ID for On/Off cluster
            .length = sizeof(uint8_t),
            .extension_field_attribute_value_list = &on_off_value,
            .next = NULL // Link to the Level Control extension field 
        };
            esp_zb_zcl_scenes_add_scene_cmd_t cmd = {
                .zcl_basic_cmd.dst_addr_u.addr_short = dst_addr_short,
                .zcl_basic_cmd.dst_endpoint = dst_ep,
                .zcl_basic_cmd.src_endpoint = 0,
                .group_id = group_id,
                .scene_id = scene_id,
                .transition_time = 0,
                .extension_field = &on_off_extension_field,
            };
            //esp_zb_lock_acquire(portMAX_DELAY);
            esp_zb_zcl_scenes_add_scene_cmd_req(&cmd); 
            //esp_zb_lock_release();         
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_ANALOG_DIMMABLE_LIGHT)
        esp_zb_zcl_scenes_extension_field_t level_control_extension_field = {
            .cluster_id = 0x0008, // Cluster ID for Level Control Cluster
            .length = sizeof(uint8_t),
            .extension_field_attribute_value_list = &brightness_value,
            .next = NULL //Initially, No Next Extension Field
        };
        esp_zb_zcl_scenes_extension_field_t on_off_extension_field = {
            .cluster_id = 0x0006, // Cluster ID for On/Off cluster
            .length = sizeof(uint8_t),
            .extension_field_attribute_value_list = &on_off_value,
            .next = &level_control_extension_field // Link to the Level Control Extension Field
        };
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI) 

  
            colorControlExtensionField.fields.mode = mode;
        
            colorControlExtensionField.fields.colorTemp = colorTemp; 

            colorControlExtensionField.fields.r = r;
            colorControlExtensionField.fields.g = g;
            colorControlExtensionField.fields.b = b;
           
            colorControlExtensionField.fields.bright = brightness_value;

            esp_zb_zcl_scenes_extension_field_t color_control_extension_field = {
                .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, // Cluster ID for Level Control cluster
                .length = sizeof(colorControlExtensionField_t),
                .extension_field_attribute_value_list = &colorControlExtensionField,
                .next = NULL //Initially, no next extension field
            };
            printf("brightness_value:%d\n", brightness_value);    
            esp_zb_zcl_scenes_extension_field_t level_control_extension_field = {
                .cluster_id = 0x0008, // Cluster ID for Level Control cluster
                .length = sizeof(uint8_t),
                .extension_field_attribute_value_list = &brightness_value,
                .next = &color_control_extension_field //Link to the Color Control extension field
            };
            esp_zb_zcl_scenes_extension_field_t on_off_extension_field = {
                .cluster_id = 0x0006, // Cluster ID for On/Off cluster
                .length = sizeof(uint8_t),
                .extension_field_attribute_value_list = &on_off_value,
                .next = &level_control_extension_field // Link to the Level Control extension field
            }; 

            esp_zb_zcl_scenes_add_scene_cmd_t cmd = {
                .zcl_basic_cmd.dst_addr_u.addr_short = dst_addr_short,
                .zcl_basic_cmd.dst_endpoint = dst_ep,
                .zcl_basic_cmd.src_endpoint = 0,
                .group_id = group_id,
                .scene_id = scene_id,
                .transition_time = 0,
                .extension_field = &on_off_extension_field,
            };
            //esp_zb_lock_acquire(portMAX_DELAY);
            esp_zb_zcl_scenes_add_scene_cmd_req(&cmd); 
            //esp_zb_lock_release();            
  
     
    #endif
  
}

void nuos_zb_scenes_add_scene_request_2(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep, uint16_t dst_addr_short, 
                                        uint8_t on_off_value, uint8_t brightness_value, uint16_t colorx, uint16_t colory, uint8_t on_off_value_2, uint8_t brightness_value_2) {

    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2R_ON_OFF_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4R_ON_OFF_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2CH_CURTAIN)
        esp_zb_zcl_scenes_extension_field_t on_off_extension_field = {
            .cluster_id = 0x0006, // Cluster ID for On/Off cluster
            .length = sizeof(uint8_t),
            .extension_field_attribute_value_list = &on_off_value,
            .next = NULL // Link to the Level Control extension field 
        };
    esp_zb_zcl_scenes_add_scene_cmd_t cmd = {
        .zcl_basic_cmd.dst_addr_u.addr_short = dst_addr_short,
        .zcl_basic_cmd.dst_endpoint = dst_ep,
        .zcl_basic_cmd.src_endpoint = 0,
        .group_id = group_id,
        .scene_id = scene_id,
        .transition_time = 0,
        .extension_field = &on_off_extension_field,
    };
    //esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_scenes_add_scene_cmd_req(&cmd); 
    //esp_zb_lock_release();         
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_ANALOG_DIMMABLE_LIGHT)
        if(on_off_value == 0) brightness_value = 0;
        if(on_off_value_2 == 0) brightness_value_2 = 0;
        
        uint8_t _value[] = {brightness_value, brightness_value_2};
        uint8_t _state[] = {on_off_value, on_off_value_2};
        esp_zb_zcl_scenes_extension_field_t level_control_extension_field = {
            .cluster_id = 0x0008, // Cluster ID for Level Control Cluster
            .length = 2*sizeof(uint8_t),
            .extension_field_attribute_value_list = _value,
            .next = NULL //Initially, No Next Extension Field
        };
        esp_zb_zcl_scenes_extension_field_t on_off_extension_field = {
            .cluster_id = 0x0006, // Cluster ID for On/Off cluster
            .length = 2*sizeof(uint8_t),
            .extension_field_attribute_value_list = _state,
            .next = &level_control_extension_field // Link to the Level Control Extension Field
        };  
        esp_zb_zcl_scenes_table_store(1, 
                                group_id, scene_id, 
                                0x0000,
                                &on_off_extension_field);
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI) 
        uint16_t colorxy[] = {colorx, colory};
        esp_zb_zcl_scenes_extension_field_t color_control_extension_field = {
            .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, // Cluster ID for Level Control cluster
            .length = sizeof(uint32_t),
            .extension_field_attribute_value_list = &colorxy,
            .next = NULL //Initially, no next extension field
        };    
        esp_zb_zcl_scenes_extension_field_t level_control_extension_field = {
            .cluster_id = 0x0008, // Cluster ID for Level Control cluster
            .length = sizeof(uint8_t),
            .extension_field_attribute_value_list = &brightness_value,
            .next = &color_control_extension_field //Initially, no next extension field
        };
        esp_zb_zcl_scenes_extension_field_t on_off_extension_field = {
            .cluster_id = 0x0006, // Cluster ID for On/Off cluster
            .length = sizeof(uint8_t),
            .extension_field_attribute_value_list = &on_off_value,
            .next = &level_control_extension_field // Link to the Level Control extension field
        };   
    esp_zb_zcl_scenes_add_scene_cmd_t cmd = {
        .zcl_basic_cmd.dst_addr_u.addr_short = dst_addr_short,
        .zcl_basic_cmd.dst_endpoint = dst_ep,
        .zcl_basic_cmd.src_endpoint = 0,
        .group_id = group_id,
        .scene_id = scene_id,
        .transition_time = 0,
        .extension_field = &on_off_extension_field,
    };
    //esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_scenes_add_scene_cmd_req(&cmd); 
    //esp_zb_lock_release();            
    #endif

 
}

void nuos_zb_scene_recall_scene_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, 
                                        uint8_t dst_ep, uint16_t dst_addr_short, esp_zb_zcl_address_mode_t address_mode)
{
    printf("--------DST_EP:%d---------\n", dst_ep);
	esp_zb_zcl_scenes_recall_scene_cmd_t cmd_req;
	cmd_req.group_id = group_id;
	cmd_req.scene_id = scene_id;
	cmd_req.address_mode = address_mode;
	cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = dst_addr_short;
	cmd_req.zcl_basic_cmd.dst_endpoint = dst_ep;
	cmd_req.zcl_basic_cmd.src_endpoint = src_ep;
    // esp_zb_lock_acquire(portMAX_DELAY);
	esp_zb_zcl_scenes_recall_scene_cmd_req(&cmd_req);
    // esp_zb_lock_release(); 
}

void nuos_zb_scene_add_scene_to_self_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep, 
                                                uint8_t value_on_off, uint8_t value_level, uint16_t colorx, uint16_t colory){

    nuos_zb_scenes_add_scene_request(group_id, scene_id, src_ep, dst_ep, esp_zb_get_short_address(), value_on_off, value_level, colorx, colory);
}

void nuos_zb_scene_add_scene_unicast_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep, uint16_t dst_addr_short, 
                                                uint8_t value_on_off, uint8_t value_level){
    // nuos_zb_scenes_add_scene_request(group_id, scene_id, src_ep, dst_ep, dst_addr_short, value_on_off, value_level); //unicast cast working
}
void nuos_zb_scene_add_scene_broadcast_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep, uint8_t value_on_off, uint8_t value_level){
    // nuos_zb_scenes_add_scene_request(group_id, scene_id, src_ep, dst_ep, 0xffff, value_on_off, value_level); //broadcast working
}
void nuos_zb_scene_add_scene_groupcast_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep, uint8_t value_on_off, uint8_t value_level){
    // nuos_zb_scenes_add_scene_request(group_id, scene_id, 0, dst_ep, group_id, value_on_off, value_level); //groupcast not working
}



void nuos_zb_scene_remove_scene_unicast_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep, uint16_t dst_addr_short){
        nuos_zb_scenes_remove_scene_request(group_id, scene_id, src_ep, dst_ep, dst_addr_short,  ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT);
}
void nuos_zb_scene_remove_scene_broadcast_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep){
    	nuos_zb_scenes_remove_scene_request(group_id, scene_id, src_ep, dst_ep, 0xffff, ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT); //broadcast working
}
void nuos_zb_scene_remove_scene_groupcast_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep){
    	nuos_zb_scenes_remove_scene_request(group_id, scene_id, 0, dst_ep, group_id, ESP_ZB_APS_ADDR_MODE_16_GROUP_ENDP_NOT_PRESENT); //groupcast not working
}


void nuos_zb_scene_store_scene_unicast_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep, uint16_t dst_addr_short){
        nuos_zb_scenes_store_scene_request(group_id, scene_id, src_ep, dst_ep, dst_addr_short);
}
void nuos_zb_scene_store_scene_broadcast_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep){
    	nuos_zb_scenes_store_scene_request(group_id, scene_id, src_ep, dst_ep, 0xffff); //broadcast not working
}
void nuos_zb_scene_store_scene_groupcast_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep){
    	nuos_zb_scenes_store_scene_request(group_id, scene_id, 0, dst_ep, group_id);    //groupcast not working
}


void nuos_zb_scene_recall_scene_unicast_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep, uint16_t dst_addr_short){
    nuos_zb_scene_recall_scene_request(group_id, scene_id, src_ep, dst_ep, dst_addr_short, ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT);
}

void nuos_zb_scene_recall_scene_broadcast_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep){
    nuos_zb_scene_recall_scene_request(group_id, scene_id, src_ep, 0xff, 0xffff,ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT);
}

void nuos_zb_scene_recall_scene_groupcast_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep){
    nuos_zb_scene_recall_scene_request(group_id, scene_id, src_ep, 0, group_id, ESP_ZB_APS_ADDR_MODE_16_GROUP_ENDP_NOT_PRESENT);
}

/***********************ACTION HANDLERS**********************/
esp_err_t zb_add_scene_resp_handler(const esp_zb_zcl_scenes_operate_scene_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    ESP_LOGI(TAG, "Add Scene Response: GROUP_ID(%d), SCENE_ID(%d)", message->group_id, message->scene_id);
    ESP_LOGI(TAG, "Add Scene Response: CLUSTER(%d), COMMAND(%d)", message->info.cluster, message->info.command.id);
    ESP_LOGI(TAG, "Add Scene Response: DST_ADDR(0x%x), DST_EP(%d), SRC_EP(%d)", 
                    message->info.dst_address, message->info.dst_endpoint, message->info.src_endpoint);
    // #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH) 
    //     //nuos_zb_scene_store_scene_broadcast_request(message->group_id, message->scene_id, message->info.src_endpoint, message->info.dst_endpoint);
    //     nuos_zb_scene_store_scene_unicast_request(message->group_id, message->scene_id, message->info.src_endpoint, 
    //     message->info.dst_endpoint, message->info.dst_address);
    // #endif
    return ESP_OK;
}

esp_err_t zb_view_scene_resp_handler(const esp_zb_zcl_scenes_view_scene_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    ESP_LOGI(TAG, "View Scene Response: GROUP_ID(%d), SCENE_ID(%d)", message->group_id, message->scene_id);
    ESP_LOGI(TAG, "View Scene Response: TRANSITION_TIME(%d)", message->transition_time);

    esp_zb_zcl_scenes_extension_field_t *field_set = message->field_set;
    while (field_set) {
        ESP_LOGI(TAG, "View Scene response: ex_cluster_id(%d), ex_length(%d)", field_set->cluster_id, field_set->length);
        field_set = field_set->next;
    }
    ESP_LOGI(TAG, "View Scene Response: CLUSTER(%d), COMMAND(%d)", message->info.cluster, message->info.command.id);
    ESP_LOGI(TAG, "View Scene Response: DST_ADDR(0x%x), DST_EP(%d), SRC_EP(%d)", message->info.dst_address, message->info.dst_endpoint, message->info.src_endpoint);
    return ESP_OK;
}

esp_err_t zb_get_scene_membership_resp_handler(const esp_zb_zcl_scenes_get_scene_membership_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                       message->info.status);

    ESP_LOGI(TAG, "GET Scene Membership Response: CAPACITY(%d), GROUP_ID(%d), SCENE_ID(%d)", message->capacity, message->group_id, message->scene_count);

    uint8_t *scene_list = message->scene_list;
    for(int i=0; i<message->scene_count; i++){
        ESP_LOGI(TAG, "scene_list[%d]: %d", i, scene_list[i]);
    }
    ESP_LOGI(TAG, "GET Scene Membership Response: CLUSTER(%d), COMMAND(%d)", message->info.cluster, message->info.command.id);
    ESP_LOGI(TAG, "GET Scene Membership Response: DST_ADDR(0x%x), DST_EP(%d), SRC_EP(%d)", message->info.dst_address, message->info.dst_endpoint, message->info.src_endpoint);

    return ESP_OK;
}

uint8_t total_cnts = 0;


esp_err_t nuos_set_store_scene(esp_zb_zcl_store_scene_message_t* message){
    printf("ep_cnts:%d \n", ep_cnts);

    uint8_t device_state_1=255, device_state_2=255;
    scene_counts = 0;

    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_ANALOG_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN_CUSTOM)
       
        if(ep_cnts<=TOTAL_ENDPOINTS){
            total_cnts = ep_cnts;
            for(int i=0; i<ep_cnts; i++){
                printf("ep_id[%d]: %d \n", i, ep_id[i]);
            } 

            if(ep_cnts--){      
                //uint8_t index = message->info.dst_endpoint-1;
                uint8_t index = ep_id[scene_counts];
                if(index == 255) index = scene_counts;
                printf("---->ep_index: %d %d  dst:%d \n", index, ENDPOINTS_LIST[index], message->info.dst_endpoint);
                if(index > 0){
                    device_state_1 = device_info[index-1].device_state;
                    device_state_2 = device_info[index].device_state;
                    nuos_zb_scenes_add_scene_request_2(message->group_id, message->scene_id, 0, ENDPOINTS_LIST[index-1], esp_zb_get_short_address(), 
                                                            device_info[index-1].device_state, device_info[index-1].device_level, 
                                                            device_info[index-1].light_color_x, device_info[index-1].light_color_y,
                                                            device_info[index].device_state, device_info[index].device_level);
                }else{
                    nuos_zb_scenes_add_scene_request_2(message->group_id, message->scene_id, 0, ENDPOINTS_LIST[index], esp_zb_get_short_address(), 
                                                            device_info[0].device_state, device_info[0].device_level, 
                                                            device_info[0].light_color_x, device_info[0].light_color_y,
                                                            device_info[1].device_state, device_info[1].device_level);
                }                                  
                scene_counts++;
            }     
        }
        if(ep_cnts == 0 || ep_cnts > TOTAL_ENDPOINTS){
            for(int j=0; j<total_cnts; j++){
                ep_id[j] = 255;
            }    
            total_cnts = 0;
            ep_cnts = 0;
        }   
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM) 

            uint8_t index = message->info.dst_endpoint-1;
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
            selected_color_mode = 0;
            nuos_zb_scenes_add_scene_color_request(message->group_id, message->scene_id, 0, ENDPOINTS_LIST[index], esp_zb_get_short_address(), 
                                            device_info[0].device_state, device_info[0].device_level, 
                                            device_info[1].device_level, device_info[1].device_level, device_info[2].device_level, 
                                            selected_color_mode, device_info[0].device_val);
            #else
            uint8_t brightness_val = device_info[4].device_level;
           
            if(selected_color_mode == 0)  brightness_val = device_info[3].device_level;

             printf("BRIGHTNESS_VAL:%d\n", brightness_val);

            nuos_zb_scenes_add_scene_color_request(message->group_id, message->scene_id, 0, ENDPOINTS_LIST[index], esp_zb_get_short_address(), 
                                            device_info[4].device_state, brightness_val, 
                                            device_info[0].device_level, device_info[1].device_level, device_info[2].device_level, 
                                            selected_color_mode, device_info[3].device_val);
            #endif


     #else

        uint8_t index = message->info.dst_endpoint-1;

        #ifndef USE_INDIVIDUAL_DALI_ADDRESSING
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN_SWITCH)
                #ifdef TUYA_ATTRIBUTES
                nuos_zb_scenes_add_scene_curtain_request(message->group_id, message->scene_id, 
                                        0, ENDPOINTS_LIST[index], 
                                        esp_zb_get_short_address(),
                                        device_info[0].fan_speed, device_info[0].device_level);
                #else
                nuos_zb_scenes_add_scene_request(message->group_id, message->scene_id, 
                        0, ENDPOINTS_LIST[index], 
                        esp_zb_get_short_address(), 
                        device_info[index].device_state, device_info[index].device_level, 
                        device_info[index].light_color_x, device_info[index].light_color_y);                        
                #endif
            #else
                nuos_zb_scenes_add_scene_request(message->group_id, message->scene_id, 
                                        0, ENDPOINTS_LIST[index], 
                                        esp_zb_get_short_address(), 
                                        device_info[index].device_state, device_info[index].device_level, 
                                        device_info[index].light_color_x, device_info[index].light_color_y);
            #endif
        #endif
    #endif
    return ESP_OK;
}

esp_err_t zb_get_scene_store_resp_handler(const esp_zb_zcl_store_scene_message_t *message)
{
    
    
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message"); 

    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    // printf("zb_get_scene_store_resp_handler\n");
    printf("=================> %d <=================\n", message->info.dst_endpoint);
    // printf("group_id: %d  scene_id: %d  cluster: %d\n", message->group_id, message->scene_id, message->info.cluster);

    nuos_set_scene_store_cluster(message);
    
    return ESP_OK;
}
extern void set_load(uint8_t index, uint8_t v);


extern uint16_t device_cct_color[TOTAL_ENDPOINTS];
void control_zb_devices(uint8_t index_1, uint16_t cluster_id, void* value){
    recheckTimer();
    printf("cluster_id:0x%x\n", cluster_id);
    if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI)
            index_1 = 4;   
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
            index_1 = 4;   
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
            index_1 = 0;               
        #endif
        device_info[index_1].device_state = *(bool*)value;    
        printf("index_1:%d  device_state :%d\n", index_1, device_info[index_1].device_state);
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
            if(device_info[index_1].device_state){ 
                device_info[index_1].ac_mode = ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_COOL;  //cooling 
            }else{
                device_info[index_1].ac_mode = ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF;   //off
                nuos_zb_set_hardware(index_1, false);
            }
            
            // ESP_LOGI(TAG, "ac mode %d", device_info[index_1].ac_mode);
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2CH_CURTAIN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN_SWITCH)
            nuos_zb_set_hardware_curtain(index_1, false);
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
            if(!device_info[index_1].device_state){
                device_info[3].device_state = false;
                nuos_zb_set_hardware(index_1, false);
                #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
                    set_state(index_1);
                #else
                    set_state(index_1);
                #endif
           }
        #else
            nuos_zb_set_hardware(index_1, false);
            //set_load(index_1);
        #endif
        if(is_my_device_commissionned){
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
                nuos_set_state_attribute(index_1);

            #else
                nuos_set_state_attribute(index_1);
            #endif       
        }
    }else if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL) {
        uint8_t d_level = *(uint8_t*)value;
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN_CUSTOM)
            if(index_1 == FAN_INDEX){
                if(d_level > 1){ //avoid double callback value
                    device_info[FAN_INDEX].device_level = d_level;
                    device_info[index_1].device_state = true;
                    if(d_level <= 64){
                        device_info[index_1].fan_speed = 1;
                    }else if(d_level > 64 && d_level <= 128){
                        device_info[index_1].fan_speed = 2;
                    }else if(d_level > 128 && d_level <= 191){
                        device_info[index_1].fan_speed = 3;
                    }else if(d_level > 191){      //near 1000
                        device_info[index_1].fan_speed = 4;   //max_speed
                    }else{
                        device_info[index_1].device_state = false;
                    }
                    
                    nuos_set_fan_attribute(index_1); //to fast update switches on app

                    nuos_set_hardware_fan_ctrl(index_1);
                }
            }else{
                if(d_level >= MIN_DIM_LEVEL_VALUE) //10% of level value
                    device_info[index_1].device_level = d_level;
                else
                    device_info[index_1].device_level = MIN_DIM_LEVEL_VALUE; 
                //Added by Nuos                 
                nuos_zb_set_hardware(index_1, false); 
            }
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
            uint8_t level = *(uint8_t *)value;
            device_info[0].device_level = level;
            ESP_LOGI(TAG, "ac device_level %d\n", device_info[index_1].device_level);
            if(device_info[0].device_state){
                if(device_info[0].device_level <= 15){
                    if(device_info[0].device_level > 0){
                        device_info[0].ac_temperature = ac_temp_values[device_info[0].device_level];
                        printf("Ac Temp %d\n",  device_info[0].ac_temperature);
                        nuos_zb_set_hardware(0, false);
                        if(is_my_device_commissionned){ 
                            nuos_set_ac_cool_temperature_attribute(index_1);
                        }                            
                    }
                } 
            }                        
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
        uint8_t level = *(uint8_t *)value;
        // device_info[index_1].device_level = map_1000(level, 0, 1000, MIN_DIM_LEVEL_VALUE, MAX_DIM_LEVEL_VALUE);
        // if(is_my_device_commissionned){                                  
        //     nuos_set_level_attribute(index_1);                            
        // }         
        #else
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI)

            #else
            #ifdef ENABLE_PWM_DIMMING
                if(device_info[index_1].device_state){
                    device_info[index_1].device_level = d_level;
                    if(is_my_device_commissionned) {                                  
                        nuos_set_level_attribute(index_1);                              
                    }  
                    set_load(index_1, d_level);
                }
            #endif
            #endif
        #endif
    }else if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL) {
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
        // printf("index_1:%d My device_state:%d \n", index_1, device_info[index_1].device_state);
        if(device_info[index_1].device_state){ 
            colorControlExtensionField_t* colorControlExtensionField = (colorControlExtensionField_t*)value;
            // printf("selected_color_mode:%d \n", colorControlExtensionField->raw[0]);
            uint16_t val = colorControlExtensionField->fields.colorTemp;
            device_info[0].device_val = val;//map_1000(val, 0, 1000, MIN_CCT_VALUE, MAX_CCT_VALUE);
            device_info[0].device_level = colorControlExtensionField->fields.bright;
            // printf("My device_level:%d device_val:%d\n", device_info[0].device_level, device_info[0].device_val);
            device_info[0].color_or_fan_state = true;
            
            is_long_press_brightness = false;
            nuos_set_hardware_brightness_2(1); 
            //nuos_zb_set_hardware(0, false);
            // printf("My device_level1:%d device_val1:%d\n", device_info[0].device_level, device_info[0].device_val);
            set_color_temp();
            set_level(0);               
        }
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
            colorControlExtensionField_t* colorControlExtensionField = (colorControlExtensionField_t*)value;
            printf("index_1:%d selected_color_mode:%d state:%d\n", index_1, selected_color_mode, device_info[index_1].device_state);            
            if(device_info[index_1].device_state){
                //set color
                if(colorControlExtensionField->raw[0] == 0){
                    if(selected_color_mode != 0){
                        mode_change_flag = true;
                        last_selected_color_mode = selected_color_mode;
                    }                   
                    selected_color_mode = colorControlExtensionField->raw[0];
                    nuos_set_color_rgb_mode_attribute(0, selected_color_mode);
                    store_color_mode_value(selected_color_mode);
                    device_info[3].device_state = true;                   
                    //if(device_info[3].device_state){
                        uint16_t val = colorControlExtensionField->fields.colorTemp;
                        device_info[3].device_val = val;//map_1000(val, 0, 1000, MIN_CCT_VALUE, MAX_CCT_VALUE);
                        device_info[3].device_level = colorControlExtensionField->fields.bright;
                    //}
                    for(int rgb=0; rgb<3; rgb++){
                        device_info[rgb].device_state = false;
                    } 
                    if(device_info[4].device_state){
                        device_info[3].device_state = true;
                        nuos_zb_set_hardware(3, false);
                        //set_state(3); 
                        set_color_temp();
                        set_level(3);
                    }                      
                }else if(colorControlExtensionField->raw[0] == 1){
                    if(selected_color_mode != 1){
                        mode_change_flag = true;
                        last_selected_color_mode = selected_color_mode;
                    }
                    selected_color_mode = colorControlExtensionField->raw[0];
                    nuos_set_color_rgb_mode_attribute(0, selected_color_mode);
                    store_color_mode_value(selected_color_mode);             

                    device_info[0].device_level = colorControlExtensionField->raw[1];
                    device_info[1].device_level = colorControlExtensionField->raw[2];
                    device_info[2].device_level = colorControlExtensionField->raw[3];

                    printf("RED:%d GREEN:%d BLUE:%d\n", device_info[0].device_level, device_info[1].device_level, device_info[2].device_level); 
                    
                    device_info[4].device_level = colorControlExtensionField->fields.bright; 
                    if(device_info[4].device_level == 0) device_info[4].device_level = 254;
                    printf("BRIGHTNESS:%d\n", device_info[4].device_level);

                    rgb_t rgb = {device_info[0].device_level, device_info[1].device_level, device_info[2].device_level}; // Example RGB values
                    hsv_t hsv2 = rgb_to_hsv(rgb);

                    device_info[0].device_level = rgb.r;
                    device_info[1].device_level = rgb.g;
                    device_info[2].device_level = rgb.b;

                    if(device_info[0].device_level == 0) device_info[0].device_state = false;
                    else device_info[0].device_state = true;
                    if(device_info[1].device_level == 0) device_info[1].device_state = false;
                    else device_info[1].device_state = true;
                    if(device_info[2].device_level == 0) device_info[2].device_state = false;
                    else device_info[2].device_state = true;
                    if(device_info[3].device_level == 0) device_info[3].device_state = false;
                    else device_info[3].device_state = true;

                    for(int rgb=0; rgb<3; rgb++){
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
                    
                    #if(USE_COLOR_DEVICE == COLOR_RGB_ONLY)
                    uint8_t index = 4;
                    device_info[3].device_state = true;
                    #else
                    uint8_t index = 4;
                    device_info[3].device_state = false;
                    #endif
                    if(device_info[index].device_state){
                        store_color_mode_value(selected_color_mode);
                        nuos_zb_set_hardware(4, false); 
                        //set_state(4);
                        set_color_temp();
                        set_level(4);                        
                    }            
                }
            }
        #endif
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN)    
        }else if (cluster_id == 0x101) { //light state
            device_info[LIGHT_INDEX].device_state = *(bool*)value;
            nuos_zb_set_hardware(LIGHT_INDEX, false);
            device_info[FAN_INDEX].device_state = false;  //make fan off
            nuos_zb_set_hardware(FAN_INDEX, false);                
        }else if (cluster_id == 0x100) { //fan state
            device_info[FAN_INDEX].device_state = *(bool*)value;
            nuos_zb_set_hardware(FAN_INDEX, false);
        }else if (cluster_id == 0x1) { // attr= 1, light off, fan off/fanon
            device_info[LIGHT_INDEX].device_state = false;
            nuos_zb_set_hardware(LIGHT_INDEX, false);
            device_info[FAN_INDEX].device_state = false;
            nuos_zb_set_hardware(FAN_INDEX, false);        
        }else if (cluster_id == 0x4) { //fan speed
            device_info[FAN_INDEX].fan_speed = *(uint8_t*)value;
            nuos_zb_set_hardware(FAN_INDEX, false);
    #else
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN_SWITCH)
        }else if (cluster_id == 5) {
            device_info[0].fan_speed = *(uint8_t*)value;
            nuos_zb_set_hardware_curtain(0, false);
            if(device_info[0].fan_speed == ESP_ZB_ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE){
                nuos_report_curtain_blind_state(0, 0);
            }else if(device_info[0].fan_speed == ESP_ZB_ZCL_CMD_WINDOW_COVERING_UP_OPEN){
                nuos_report_curtain_blind_state(1, 100); 
            }else if(device_info[0].fan_speed == ESP_ZB_ZCL_CMD_WINDOW_COVERING_STOP){
                nuos_report_curtain_blind_state(1, 100); 
            }
        #endif    
    #endif    
    }else if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL) {

    }else if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT) {

    }else if (cluster_id == 5) {
        uint8_t data = *(uint8_t*)value;
        printf("data5:%d\n", data);
    }else if (cluster_id == 0x0301) {
        uint8_t data = *(uint8_t*)value;
        printf("data301:%d\n", data);
    }else if(cluster_id == ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING){
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN)
            #ifdef TUYA_ATTRIBUTES
                uint8_t data = *(uint8_t*)value; //percentage value
                printf("data102:%d\n", data);
                //nuos_zb_set_hardware(index_1, false);
                set_curtain_percentage(data);
            #endif
        #endif
    }
}

void nuos_set_scene(esp_zb_zcl_recall_scene_message_t *message){
// printf("cmd_id: %d  direction: %d is_common: %d\n", message->command.id, message->command.direction, message->command.is_common);
    uint8_t index_1 = message->info.dst_endpoint-1;
    esp_zb_zcl_scenes_extension_field_t* ext_list = message->field_set;

    // printf("command: %d, cluster:x%0x\n", message->command.id, message->info.cluster);
    while (ext_list != NULL) {
        //taskYIELD();
        printf("Length: %d  %d\n", ext_list->length, index_1);
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_ANALOG_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_PHASE_CUT_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN_CUSTOM )
             for (int i = 0; i < ext_list->length; i++) {
                printf("index:%d,  CLUSTER_ID: 0x%x ATTR: 0x%02X\n", index_1, ext_list->cluster_id, ext_list->extension_field_attribute_value_list[i]);
                if(i==index_1){
                    control_zb_devices(index_1, ext_list->cluster_id, &ext_list->extension_field_attribute_value_list[i]);                  
                }
                index_1++;              
             }
            index_1 = 0;//message->info.dst_endpoint-1;
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN)
             for (int i = 0; i < ext_list->length; i++) {
                printf("index:%d,  CLUSTER_ID: 0x%x ATTR: 0x%02X\n", index_1, ext_list->cluster_id, ext_list->extension_field_attribute_value_list[i]);
                // if(i==index_1){
                     control_zb_devices(index_1, ext_list->cluster_id, &ext_list->extension_field_attribute_value_list[i]);                  
                // }
                // index_1++;              
             }
            //index_1 = 0;//message->info.dst_endpoint-1;            
        #else
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI) 
                printf("CLUSTER_ID: 0x%x\n", ext_list->cluster_id);
                #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
                control_zb_devices(0, ext_list->cluster_id, &ext_list->extension_field_attribute_value_list[0]); 
                #elif(USE_COLOR_DEVICE == COLOR_RGB_ONLY)
                control_zb_devices(4, ext_list->cluster_id, &ext_list->extension_field_attribute_value_list[0]); 
                #else
                control_zb_devices(4, ext_list->cluster_id, &ext_list->extension_field_attribute_value_list[0]); 
                #endif
            #else    
                control_zb_devices(index_1, ext_list->cluster_id, &ext_list->extension_field_attribute_value_list[0]);
            #endif    
        #endif
        ext_list = ext_list->next;
    }
    printf("End....\n");
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_GROUP_SWITCH)
    #ifdef USE_CUSTOM_SCENE
    if(ext_list == NULL){
        for(int i=0; i<TOTAL_ENDPOINTS; i++){
            if((zb_scene_info[i].group_id ==  message->group_id) && (zb_scene_info[i].scene_id == message->scene_id)){
                nuos_zb_set_hardware(i, false);
            }
        }
        
    }
    #endif
    #endif

}


esp_err_t zb_get_scene_recall_resp_handler(const esp_zb_zcl_recall_scene_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    printf("*************************************\n");
    // printf("group_id: %d  scene_id: %d\n", message->group_id, message->scene_id);
    // printf("cluster: %d  dst_endpoint: %d\n", message->info.cluster, message->info.dst_endpoint); 
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER  || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
        nuos_set_scene(message);
    #else
        nuos_set_scene_group_cluster(message);
    #endif
    return ESP_OK;
}

esp_err_t nuos_zb_scene_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message){
    esp_err_t ret = -1;
    //printf("On Scene Recall!!\n");
    switch (callback_id) {
        case ESP_ZB_CORE_SCENES_STORE_SCENE_CB_ID:
            ret = zb_get_scene_store_resp_handler ((esp_zb_zcl_store_scene_message_t *)message);
            break;
        case ESP_ZB_CORE_CMD_OPERATE_SCENE_RESP_CB_ID:
            ret =  zb_add_scene_resp_handler((esp_zb_zcl_scenes_operate_scene_resp_message_t *)message);
            break;
        case ESP_ZB_CORE_CMD_VIEW_SCENE_RESP_CB_ID:
            ret = zb_view_scene_resp_handler((esp_zb_zcl_scenes_view_scene_resp_message_t *)message);
            break;
        case ESP_ZB_CORE_CMD_GET_SCENE_MEMBERSHIP_RESP_CB_ID:
            ret = zb_get_scene_membership_resp_handler((esp_zb_zcl_scenes_get_scene_membership_resp_message_t *)message);
            break;
        case ESP_ZB_CORE_SCENES_RECALL_SCENE_CB_ID:
        	ret = zb_get_scene_recall_resp_handler((esp_zb_zcl_recall_scene_message_t *)message);
        	break;
        default:
         
        break;
    }
    return ret;
}