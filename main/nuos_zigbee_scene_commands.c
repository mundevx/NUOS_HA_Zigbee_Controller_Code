

#include "app_hardware_driver.h"
#include "esp_zigbee_core.h"
#include "esp_err.h"
#include "esp_check.h"
#include "app_zigbee_group_commands.h"
#include "app_zigbee_scene_commands.h"
#include "esp_zigbee_core.h"
#include "zdo/esp_zigbee_zdo_command.h"
#include "app_zigbee_clusters.h"
#include "esp_random.h"
#include "esp_timer.h"

static const char *TAG = "ESP_ZB_SCENE_COMMANDS";

extern void nuos_set_attribute_cluster_2(const esp_zb_zcl_set_attr_value_message_t *message);
extern esp_err_t nuos_set_state_attribute(uint8_t index);
extern esp_err_t nuos_set_color_temp_level_attribute(uint8_t index);
extern esp_err_t nuos_set_fan_attribute(uint8_t index);
extern void xyToRgb(uint16_t x, uint16_t y, float brightness, uint8_t *r, uint8_t *g, uint8_t *b);
extern void nuos_set_scene_group_cluster(const esp_zb_zcl_recall_scene_message_t *message);
extern void set_load(uint8_t index, uint8_t v);
extern uint16_t device_cct_color[TOTAL_ENDPOINTS];
uint8_t total_cnts = 0;
static void scene_print_task(void *pvParameters);



/* Queue handle */
static QueueHandle_t scene_queue = NULL;

typedef enum {
    CMD_SET_DALI_SET_STATE,
    CMD_SET_DALI_SET_LEVEL,
    CMD_SET_DALI_COLOR_TEMP,
    CMD_SET_DALI_COLOR_RGB
} zb_cmd_type_t;
typedef struct {
    zb_cmd_type_t type;
    uint8_t       index;
    bool          state;
    uint8_t       level;
    uint16_t      color_temp;
    uint8_t       r;
    uint8_t       g;
    uint8_t       b;
    uint8_t       color_mode;
    uint8_t       reserved[3]; // Padding to make the structure size a multiple of 4 bytes
} zb_cmd_t;

uint8_t rejected_index = 255;

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

typedef union colorCCTExtensionField_u
{
   struct colorCCTExtensionField_s
   {
        uint16_t colorTemp;
        // uint16_t colorTemp2;
   }fields;

   uint8_t raw[4];
}colorCCTExtensionField_t;

colorCCTExtensionField_t colorCCTExtensionField = {0};

void nuos_zb_scenes_add_scene_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep, uint16_t dst_addr_short, 
                                        uint8_t on_off_value, uint8_t brightness_value, uint16_t colorx, uint16_t colory) {

    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_DALI_DIRECT_SWITCH|| USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2R_ON_OFF_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_4R_ON_OFF_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2CH_CURTAIN)
        // esp_zb_zcl_scenes_extension_field_t on_off_extension_field = {
        //     .cluster_id = 0x0006, // Cluster ID for On/Off cluster
        //     .length = sizeof(uint8_t),
        //     .extension_field_attribute_value_list = &on_off_value,
        //     .next = NULL // Link to the Level Control extension field 
        // };
        // esp_zb_zcl_scenes_add_scene_cmd_t cmd = {
        //     .zcl_basic_cmd.dst_addr_u.addr_short = dst_addr_short,
        //     .zcl_basic_cmd.dst_endpoint = dst_ep,
        //     .zcl_basic_cmd.src_endpoint = src_ep,
        //     .group_id = group_id,
        //     .scene_id = scene_id,
        //     .transition_time = 0,
        //     .extension_field = &on_off_extension_field,
        // };        
        // esp_zb_zcl_scenes_add_scene_cmd_req(&cmd); 
        
        if(brightness_value > 254) brightness_value = 254;
        if(on_off_value == 0) brightness_value = 0;

        esp_zb_zcl_scenes_extension_field_t on_off_extension_field = {
            .cluster_id = 0x0006, // Cluster ID for On/Off cluster
            .length = sizeof(uint8_t),
            .extension_field_attribute_value_list = &on_off_value,
            // .next = &level_control_extension_field // Link to the Level Control Extension Field
        };

        if(brightness_value > 0) {
            esp_zb_zcl_scenes_extension_field_t level_control_extension_field = {
                .cluster_id = 0x0008, // Cluster ID for Level Control Cluster
                .length = sizeof(uint8_t),
                .extension_field_attribute_value_list = &brightness_value,
                .next = NULL //Initially, No Next Extension Field
            };
            on_off_extension_field.next = &level_control_extension_field;  
        }else{
            on_off_extension_field.next = NULL;
        }
        esp_zb_zcl_scenes_add_scene_cmd_t cmd = {
            .zcl_basic_cmd.dst_addr_u.addr_short = dst_addr_short,
            .zcl_basic_cmd.dst_endpoint = dst_ep,
            .zcl_basic_cmd.src_endpoint = 0,
            .group_id = group_id,
            .scene_id = scene_id,
            .transition_time = 0,
            .extension_field = &on_off_extension_field,
        };        
        esp_zb_zcl_scenes_add_scene_cmd_req(&cmd); 

    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER)

        esp_zb_zcl_scenes_extension_field_t level_control_extension_field = {
            .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT, // Cluster ID for Level Control Cluster
            .length = sizeof(uint8_t),
            .extension_field_attribute_value_list = &brightness_value,
            .next = NULL //Initially, No Next Extension Field
        };
        esp_zb_zcl_scenes_extension_field_t on_off_extension_field = {
            .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT, // Cluster ID for On/Off cluster
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
        esp_zb_zcl_scenes_add_scene_cmd_req(&cmd); 
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_ANALOG_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_PHASE_CUT_DIMMABLE_LIGHT)
        
        if(brightness_value > 64) brightness_value = 64;
        if(on_off_value == 0) brightness_value = 0;
        
      
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
        esp_zb_zcl_scenes_add_scene_cmd_req(&cmd); 

        // esp_zb_zcl_scenes_table_store(dst_ep, group_id, scene_id, 0x0000,
        //                                     &on_off_extension_field);          
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
                .extension_field_attribute_value_list = (uint8_t*)&colorxy,
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


            printf("curtain value:%d\n", lift_percentage);                                
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
            esp_zb_zcl_scenes_add_scene_cmd_req(&cmd); 
            // //esp_zb_lock_release(); 
            //esp_zb_zcl_scenes_table_store(dst_ep, group_id, scene_id, 0x0000,
                                                //&lift_percentage_extension_field);

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
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_ANALOG_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_PHASE_CUT_DIMMABLE_LIGHT)
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
                .extension_field_attribute_value_list = (uint8_t*)&colorControlExtensionField,
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
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_ANALOG_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_PHASE_CUT_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_DALI_DIRECT_SWITCH)
        if(brightness_value > 254) brightness_value = 254;
        if(brightness_value_2 > 254) brightness_value_2 = 254;        
    
        if(on_off_value == 0) brightness_value = 0;
        if(on_off_value_2 == 0) brightness_value_2 = 0;

        uint8_t _value[] = {brightness_value, brightness_value_2};
        uint8_t _state[] = {on_off_value, on_off_value_2};

        #ifdef USE_COLOR_CONTROL
        uint16_t colorxy[] = {colorx, colory};
        esp_zb_zcl_scenes_extension_field_t color_control_extension_field = {
            .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, // Cluster ID for Level Control cluster
            .length = sizeof(uint32_t),
            .extension_field_attribute_value_list = (uint8_t*)&colorxy,
            .next = NULL //Initially, no next extension field
        }; 
        esp_zb_zcl_scenes_extension_field_t level_control_extension_field = {
            .cluster_id = 0x0008, // Cluster ID for Level Control Cluster
            .length = 2*sizeof(uint8_t),
            .extension_field_attribute_value_list = _value,
            .next = &color_control_extension_field
        }; 
        #else
        esp_zb_zcl_scenes_extension_field_t level_control_extension_field = {
            .cluster_id = 0x0008, // Cluster ID for Level Control Cluster
            .length = 2*sizeof(uint8_t),
            .extension_field_attribute_value_list = _value,
            .next = NULL //Initially, No Next Extension Field
        };
        #endif
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
            .extension_field_attribute_value_list = (uint8_t*)&colorxy,
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
typedef struct {
    uint8_t endpoint_1;
    uint8_t value_1;
} custom_onoff_multi_ep_field_t;
custom_onoff_multi_ep_field_t custom_onoff_field;
custom_onoff_multi_ep_field_t custom_level_field;
typedef struct {
    uint8_t endpoint_1;
    uint8_t value_1;
    uint8_t endpoint_2;
    uint8_t value_2;
} custom_onoff_multi_ep_32_field_t;
custom_onoff_multi_ep_32_field_t custom_onoff_32_field;
custom_onoff_multi_ep_32_field_t custom_level_32_field;
void nuos_zb_scenes_add_scene_request_3(uint16_t group_id, uint8_t scene_id, 
                                        uint8_t attributes_count, uint8_t dst_ep_index, uint16_t dst_addr_short, 
                                        uint8_t on_off_value, uint8_t brightness_value, uint16_t cct_val,  
                                        uint8_t on_off_value_2, uint8_t brightness_value_2, uint16_t cct_val_2) {
         
        if(brightness_value > 254) brightness_value = 254;
        if(brightness_value_2 > 254) brightness_value_2 = 254;        
    
        if(on_off_value == 0) brightness_value = 0;
        if(on_off_value_2 == 0) brightness_value_2 = 0;

        uint8_t _value[] = {brightness_value, brightness_value_2};
        uint8_t _state[] = {on_off_value, on_off_value_2};

        #ifdef USE_COLOR_CONTROL
        uint16_t cct_array[] = {cct_val, cct_val_2};
        

        esp_zb_zcl_scenes_extension_field_t color_control_extension_field = {
            .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, // Cluster ID for Level Control cluster
            .length = attributes_count*sizeof(uint16_t),
            .extension_field_attribute_value_list = (uint8_t*)&cct_array,
            .next = NULL //Initially, no next extension field
        }; 

        esp_zb_zcl_scenes_extension_field_t level_control_extension_field = {
            .cluster_id = 0x0008, // Cluster ID for Level Control Cluster
            // .length = attributes_count*sizeof(uint8_t),
            // .extension_field_attribute_value_list = _value,
            // .next = &color_control_extension_field
        };
        if(attributes_count == 2){
            custom_level_32_field.endpoint_1 = 1;
            custom_level_32_field.endpoint_2 = 2; 
            custom_level_32_field.value_1 = _value[0];
            custom_level_32_field.value_2 = _value[1];
            level_control_extension_field.length = sizeof(custom_onoff_multi_ep_32_field_t);
            level_control_extension_field.extension_field_attribute_value_list = (uint8_t*)&custom_level_32_field;
            level_control_extension_field.next = &color_control_extension_field;
        }else{
            custom_level_field.endpoint_1 = ENDPOINTS_LIST[dst_ep_index];
            custom_level_field.value_1 = _value[dst_ep_index];
            level_control_extension_field.length = sizeof(custom_onoff_multi_ep_field_t);
            level_control_extension_field.extension_field_attribute_value_list = (uint8_t*)&custom_level_field;
            level_control_extension_field.next = &color_control_extension_field;
        }        
        if(cct_val == 0 && cct_val_2 == 0){
            level_control_extension_field.next = NULL;
        }else{
            level_control_extension_field.next = &color_control_extension_field;
        }
        #else
        esp_zb_zcl_scenes_extension_field_t level_control_extension_field = {
            .cluster_id = 0x0008, // Cluster ID for Level Control Cluster
            // .length = attributes_count*sizeof(uint8_t),
            // .extension_field_attribute_value_list = _value,
            // .next = &color_control_extension_field
        };
        if(attributes_count == 2){
            custom_level_32_field.endpoint_1 = 1;
            custom_level_32_field.endpoint_2 = 2; 
            custom_level_32_field.value_1 = _value[0];
            custom_level_32_field.value_2 = _value[1];
            level_control_extension_field.length = sizeof(custom_onoff_multi_ep_32_field_t);
            level_control_extension_field.extension_field_attribute_value_list = (uint8_t*)&custom_level_32_field;
            level_control_extension_field.next = NULL;
        }else{
            custom_level_field.endpoint_1 = ENDPOINTS_LIST[dst_ep_index];
            custom_level_field.value_1 = _value[dst_ep_index];
            level_control_extension_field.length = sizeof(custom_onoff_multi_ep_field_t);
            level_control_extension_field.extension_field_attribute_value_list = (uint8_t*)&custom_level_field;
            level_control_extension_field.next = NULL;
        }        
        #endif
        
        esp_zb_zcl_scenes_extension_field_t on_off_extension_field = {
            .cluster_id = 0x0006, // Cluster ID for On/Off cluster
        };  

        if(attributes_count == 2){
            custom_onoff_32_field.endpoint_1 = 1;
            custom_onoff_32_field.endpoint_2 = 2; 
            custom_onoff_32_field.value_1 = _state[0];
            custom_onoff_32_field.value_2 = _state[1];
            on_off_extension_field.length = sizeof(custom_onoff_multi_ep_32_field_t);
            on_off_extension_field.extension_field_attribute_value_list = (uint8_t*)&custom_onoff_32_field;
            on_off_extension_field.next = &level_control_extension_field;
        }else{
            custom_onoff_field.endpoint_1 = ENDPOINTS_LIST[dst_ep_index];
            custom_onoff_field.value_1 = _state[dst_ep_index];
            printf("endpoint_ :%d , value_ :%d\n", custom_onoff_field.endpoint_1, custom_onoff_field.value_1);
            on_off_extension_field.length = sizeof(custom_onoff_multi_ep_field_t);
            on_off_extension_field.extension_field_attribute_value_list = (uint8_t*)&custom_onoff_field;
            on_off_extension_field.next = &level_control_extension_field;
        }


        if(brightness_value == 0 && brightness_value_2 == 0){
            on_off_extension_field.next = NULL;
        }else{
            on_off_extension_field.next = &level_control_extension_field;
        }
        esp_zb_zcl_scenes_table_store(1, 
                                group_id, scene_id, 
                                0x0000,
                                &on_off_extension_field);

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


esp_err_t nuos_set_store_scene(esp_zb_zcl_store_scene_message_t* message){
    printf("ep_cnts:%d \n", ep_cnts);

    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_DALI_DIRECT_SWITCH)
        uint8_t new_ep_id[2] = {0, 0};
        //#ifdef USE_COLOR_CONTROL
        
        static uint32_t prev_ms = 0;
        static uint8_t saved_ep_cnts = 0;
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);  // ms
        
        if((now_ms - prev_ms) < 2000 && prev_ms != 0){ //if 2nd command is 
            if(saved_ep_cnts > ep_cnts){
                saved_ep_cnts = 0;
                return ESP_OK;
            }else{
                saved_ep_cnts = ep_cnts;
            }
            
        } else {
            saved_ep_cnts = ep_cnts;
        }
        prev_ms = now_ms;
        //#endif
    #endif
    scene_counts = 0;
    if(ep_cnts == 0) return ESP_OK;

    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_ANALOG_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_PHASE_CUT_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_DALI_DIRECT_SWITCH)
        
        
        
        if(ep_cnts<=TOTAL_ENDPOINTS){
            //#ifdef USE_COLOR_CONTROL
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_DALI_DIRECT_SWITCH)
                total_cnts = ep_cnts;
                uint8_t Ecnts = 0;
                for(int i=0; i<TOTAL_ENDPOINTS; i++){
                    printf("ep_id[%d]: %d \n", i, ep_id[i]);
                    if(ep_id[i] != 255) new_ep_id[Ecnts++] = ep_id[i];
                } 
                uint8_t index = new_ep_id[scene_counts];
                if(index == 255) index = scene_counts;
            #else

                uint8_t index = message->info.dst_endpoint-1;
            #endif
            if(ep_cnts--){      
                // uint8_t index = new_ep_id[scene_counts];
                // if(index == 255) index = scene_counts;
                // printf("---->ep_index: %d %d  dst:%d \n", index, ENDPOINTS_LIST[index], message->info.dst_endpoint);

                #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_DALI_DIRECT_SWITCH)
                    //#ifdef USE_COLOR_CONTROL
                        nuos_zb_scenes_add_scene_request_3(message->group_id, message->scene_id, total_cnts, index, esp_zb_get_short_address(), 
                                                                device_info[0].device_state, device_info[0].device_level, device_info[0].device_val,
                                                                device_info[1].device_state, device_info[1].device_level, device_info[1].device_val);
                    //#else
                        // if(index > 0){
                        //     nuos_zb_scenes_add_scene_request_2(message->group_id, message->scene_id, 0, ENDPOINTS_LIST[index-1], esp_zb_get_short_address(), 
                        //                                             device_info[index-1].device_state, device_info[index-1].device_level, 
                        //                                             device_info[index-1].light_color_x, device_info[index-1].light_color_y,
                        //                                             device_info[index].device_state, device_info[index].device_level);
                        // }else{
                        //     nuos_zb_scenes_add_scene_request_2(message->group_id, message->scene_id, 0, ENDPOINTS_LIST[index], esp_zb_get_short_address(), 
                        //                                             device_info[0].device_state, device_info[0].device_level, 
                        //                                             device_info[0].light_color_x, device_info[0].light_color_y,
                        //                                             device_info[1].device_state, device_info[1].device_level);
                        // } 
                    //#endif
                #else
                    if(index > 0){
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
                #endif                                 
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
             uint8_t level[3] = {device_info[0].device_level, device_info[1].device_level, device_info[2].device_level};
            if(!device_info[0].device_state) level[0] = 0;
            if(!device_info[1].device_state) level[1] = 0;
            if(!device_info[2].device_state) level[2] = 0;

            nuos_zb_scenes_add_scene_color_request(message->group_id, message->scene_id, 0, ENDPOINTS_LIST[index], esp_zb_get_short_address(), 
                                            device_info[4].device_state, brightness_val, 
                                            level[0], level[1], level[2], 
                                            selected_color_mode, device_info[3].device_val);
            #endif

    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
        #ifdef USE_FAN_SPEED
            if(ep_cnts<=TOTAL_ENDPOINTS){
                total_cnts = ep_cnts;
                for(int i=0; i<ep_cnts; i++){
                    printf("ep_id[%d]: %d \n", i, ep_id[i]);
                } 

                if(ep_cnts--){      
                    uint8_t index = ep_id[scene_counts];
                    if(index == 255) index = scene_counts;
                    printf("---->ep_index: %d %d  dst:%d \n", index, ENDPOINTS_LIST[index], message->info.dst_endpoint);
                    if(index > 0){
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
        #else
        uint8_t index = message->info.dst_endpoint-1;
        nuos_zb_scenes_add_scene_request(message->group_id, message->scene_id, 
                                0, ENDPOINTS_LIST[index], 
                                esp_zb_get_short_address(), 
                                device_info[index].device_state, device_info[index].device_level, 
                                device_info[index].light_color_x, device_info[index].light_color_y);
        #endif  
        
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER)
        printf("message->group_id:%d  scene_id:%d\n", message->group_id, message->scene_id);
        uint8_t index = message->info.dst_endpoint-1;
        nuos_zb_scenes_add_scene_request(message->group_id, message->scene_id, 
                0, ENDPOINTS_LIST[index], 
                esp_zb_get_short_address(), 
                device_info[1].device_state, device_info[0].ac_temperature, 
                device_info[0].light_color_x, device_info[0].light_color_y);     
    #else

        uint8_t index = message->info.dst_endpoint-1;
        printf("STORE SCENE REQ DST_EP:%d  index:%d\n", message->info.dst_endpoint, index);
        #ifndef USE_INDIVIDUAL_DALI_ADDRESSING
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN_SWITCH)
                #ifdef TUYA_ATTRIBUTES
                nuos_zb_scenes_add_scene_curtain_request(message->group_id, message->scene_id, 
                                        0, ENDPOINTS_LIST[index], 
                                        esp_zb_get_short_address(),
                                        device_info[0].curtain_state, device_info[0].device_level);
                #else
                nuos_zb_scenes_add_scene_request(message->group_id, message->scene_id, 
                        0, ENDPOINTS_LIST[index], 
                        esp_zb_get_short_address(), 
                        device_info[index].device_state, device_info[index].device_level, 
                        device_info[index].light_color_x, device_info[index].light_color_y);                        
                #endif
            #else
 
            /////////////////////////////////////////////////////////// /                           
                #ifdef DALI_DIRECT_ADDRESSING                        
                if(ep_cnts<=TOTAL_ENDPOINTS){
                    total_cnts = ep_cnts;
                    for(int i=0; i<ep_cnts; i++){
                        printf("ep_id[%d]: %d \n", i, ep_id[i]);
                    } 

                    if(ep_cnts--){      
                        uint8_t index = ep_id[scene_counts];
                        if(index == 255) index = scene_counts;
                        printf("---->ep_index: %d %d  dst:%d \n", index, ENDPOINTS_LIST[index], message->info.dst_endpoint);
                        if(index > 0){
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
                #else
                //printf("---->ep_index: %d %d  dst:%d \n", index, ENDPOINTS_LIST[index], message->info.dst_endpoint);
                printf("message->group_id:%d  scene_id:%d\n", message->group_id, message->scene_id);
                nuos_zb_scenes_add_scene_request(message->group_id, message->scene_id, 
                        0, ENDPOINTS_LIST[index], 
                        esp_zb_get_short_address(), 
                        device_info[index].device_state, device_info[index].device_level, 
                        device_info[index].light_color_x, device_info[index].light_color_y);                
                #endif
            /////////////////////////////////////////////////////////////                            
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
    printf("=================> %d <=================\n", message->info.dst_endpoint); 
    nuos_set_scene_store_cluster(message);
    return ESP_OK;
}


void control_zb_devices(uint8_t index_1, uint16_t cluster_id, void* value){
    if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
        uint8_t s_val = *(uint8_t*)value;
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI)
            index_1 = 4;   
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
            index_1 = 4;   
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
            index_1 = 0;               
        #endif
        

        device_info[index_1].device_state = *(bool*)value;    
        printf("index_1:%d  device_state :%d\n", index_1, device_info[index_1].device_state);
            
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
            if(index_1 == 0){
                if(device_info[index_1].device_state){ 
                    device_info[index_1].ac_mode = ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_COOL;  //Cooling 
                }else{
                    device_info[index_1].ac_mode = ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF;   //Off
                    nuos_zb_set_hardware(index_1, false);
                }
            }
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN  || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN_SWITCH)
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

        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
            if(!device_info[index_1].device_state){
                device_info[3].device_state = false;
                nuos_zb_set_hardware(index_1, false);
                #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
                    set_state(index_1);
                #else
                    set_state(index_1);
                #endif
           }
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)     
            nuos_zb_set_hardware(index_1, false);
            set_state(index_1);
            
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI)  
            nuos_zb_set_hardware(index_1, false);             
        #else

            nuos_zb_set_hardware(index_1, false);
         
        #endif
        if(is_my_device_commissionned){
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
            if(index_1 == 0) {  
                nuos_set_state_attribute(index_1);
            }
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN)
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI)  
                nuos_set_zigbee_attribute(index_1);
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2CH_CURTAIN)
                nuos_set_state_attribute(index_1);              
            #else
                nuos_set_state_attribute(index_1);
            #endif       
        }
    }else if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL) {
        uint8_t d_level = *(uint8_t*)value;
        printf("=====device_level: %d \n", d_level);
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
                }else{
                    device_info[index_1].device_state = false;
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
        #ifdef USE_FAN_SPEED
            device_info[index_1].device_level = level;
            if(index_1 == 0){
                if(device_info[0].device_state){
                    if(device_info[0].device_level <= 15){
                        if(device_info[0].device_level > 0){
                            device_info[0].ac_temperature = ac_temp_values[device_info[0].device_level];
                            nuos_zb_set_hardware(0, false);
                            if(is_my_device_commissionned){ 
                                nuos_set_ac_cool_temperature_attribute(index_1);
                            }                            
                        }
                    } 
                }                
            }else{
                const uint8_t fan_level_speed[5] = {0, 4, 8, 12, 16};
                                        //Auto, Min, Low, med, High, Max
                // for(int i=0; i<5; i++){
                    if(device_info[1].device_level > 0){
                        if(device_info[1].device_level >= fan_level_speed[0] && device_info[1].device_level < fan_level_speed[1]){
                            device_info[0].fan_speed = 1;
                            //break;
                        }else if(device_info[1].device_level >= fan_level_speed[1] && device_info[1].device_level < fan_level_speed[2]){
                            device_info[0].fan_speed = 2;
                            //break;
                        }else if(device_info[1].device_level >= fan_level_speed[2] && device_info[1].device_level < fan_level_speed[3]){
                            device_info[0].fan_speed = 3;
                            //break;
                        }else if(device_info[1].device_level >= fan_level_speed[3] && device_info[1].device_level < fan_level_speed[4]){
                            device_info[0].fan_speed = 4;
                            //break;
                        }else if(device_info[1].device_level >= fan_level_speed[4]){
                            device_info[0].fan_speed = 5;
                        }
                    }else{
                        device_info[0].fan_speed = 0;
                    }
                // }
                printf("level:%d,   fan_speed:%d \n", device_info[1].device_level, device_info[0].fan_speed);
                nuos_zb_set_hardware(1, false);
                if(is_my_device_commissionned){ 
                    nuos_set_ac_cool_temperature_attribute(1);
                }              
            }
        #else

            device_info[0].device_level = level;
            if(device_info[0].device_state){
                if(device_info[0].device_level <= 15){
                    if(device_info[0].device_level > 0){
                        device_info[0].ac_temperature = ac_temp_values[device_info[0].device_level];
                        nuos_zb_set_hardware(0, false);
                        if(is_my_device_commissionned){ 
                            nuos_set_ac_cool_temperature_attribute(index_1);
                        }                            
                    }
                } 
            }
        #endif    
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
        uint8_t level = *(uint8_t *)value;
        if(level > 0){
            device_info[index_1].device_level = level;
            nuos_set_color_temp_level_attribute(index_1);
            // set_dali_level(0);  
            
        }
        #else
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI)
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI)
                #ifdef DALI_DIRECT_ADDRESSING
                    #ifdef ENABLE_PWM_DIMMING
                        if(device_info[index_1].device_state){
                            device_info[index_1].device_level = d_level;
                            if(is_my_device_commissionned) {                                  
                                nuos_set_level_attribute(index_1);                              
                            }
                            nuos_zb_set_hardware(index_1, false);
                        }
                    #endif
                #endif
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_DALI_DIRECT_SWITCH)
                #ifdef ENABLE_PWM_DIMMING
                    if(d_level > 0){
                        device_info[index_1].device_level = d_level;
                        nuos_set_level_attribute(index_1);
                        set_color_to_updown_leds(index_1);
                        //set_dali_level(index_1); 
                    }
                #endif    
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
    }else if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL) {

    }else if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT) {
        uint8_t val = *(uint8_t *)value;
        printf("====SCENE val:%d \n", val);    
    }else if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL) {
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
        // printf("index_1:%d My device_state:%d \n", index_1, device_info[index_1].device_state);
        if(device_info[index_1].device_state){ 
            colorControlExtensionField_t* colorControlExtensionField = (colorControlExtensionField_t*)value;
            // printf("selected_color_mode:%d \n", colorControlExtensionField->raw[0]);
            uint16_t val = colorControlExtensionField->fields.colorTemp;
            device_info[0].device_val = val;
            device_info[0].device_level = colorControlExtensionField->fields.bright;
            printf("My device_level:%d device_val:%d\n", device_info[0].device_level, device_info[0].device_val);
            device_info[0].color_or_fan_state = true;
            
            is_long_press_brightness = false;
            nuos_set_hardware_brightness_2(1); 
            set_dali_color_temp(0, false);
            //set_dali_level(0);  
            //nuos_set_state_attribute(0);          
        }else{
            nuos_zb_set_hardware(index_1, false);
            set_state(index_1);
        }
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_DALI_DIRECT_SWITCH)

        #ifdef USE_COLOR_CONTROL
        uint16_t cct_val = *(uint16_t *)value;
        if(device_info[index_1].device_state){
            device_info[index_1].device_val = cct_val;
            set_color_to_updown_leds(index_1);
            set_dali_color_temp(index_1, false);
            nuos_set_color_temperature_attribute(index_1);    
        }
        #endif

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
                    // for(int rgb=0; rgb<3; rgb++){
                    //     device_info[rgb].device_state = false;
                    // } 
                    //if(device_info[4].device_state){
                        device_info[3].device_state = true;
                        nuos_zb_set_hardware(3, false);
                        //set_state(3); 
                        set_dali_color_temp(0, false);
                        set_dali_level(3);
                    //}                      
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
                    //hsv_t hsv2 = rgb_to_hsv(rgb);

                    device_info[0].device_level = rgb.r;
                    device_info[1].device_level = rgb.g;
                    device_info[2].device_level = rgb.b;

                    if(device_info[0].device_level == 0) device_info[0].device_state = false;
                    else device_info[0].device_state = true;
                    if(device_info[1].device_level == 0) device_info[1].device_state = false;
                    else device_info[1].device_state = true;
                    if(device_info[2].device_level == 0) device_info[2].device_state = false;
                    else device_info[2].device_state = true;
                    // if(device_info[3].device_level == 0) device_info[3].device_state = false;
                    // else device_info[3].device_state = true;

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
                    uint8_t index = 4;
                    device_info[3].device_state = true;
                    #else
                    //uint8_t index = 4;
                    //device_info[3].device_state = false;
                    #endif
                    //if(device_info[index].device_state){
                        store_color_mode_value(selected_color_mode);
                        nuos_zb_set_hardware(4, false); 
                        //set_state(4);
                        set_dali_color_temp(0, false);
                        set_dali_level(4);                        
                    //}            
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

    }else if (cluster_id == 5) {
        uint8_t data = *(uint8_t*)value;
        printf("data5:%d\n", data);
    }else if (cluster_id == 0x0301) {
        uint8_t data = *(uint8_t*)value;
        printf("data301:%d\n", data);
    }else if(cluster_id == ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING){
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN)
            #ifdef TUYA_ATTRIBUTES
                device_info[0].device_level = *(uint8_t*)value; //percentage value
                device_info[0].fan_speed = 0xff;
                printf("data102:%d\n", device_info[0].device_level);
                set_curtain_percentage(device_info[0].device_level, true);
            #endif
        #endif
    }
}

int random_20_to_100(void)
{
    return 50 + (esp_random() % 81);  // 20..100 inclusive
}

int random_50_to_200(void)
{
    return 50 + (esp_random() % 151);  // 50..200 inclusive
}
int random_100_to_500(void)
{
    return 100 + (esp_random() % 401);  // 100..500 inclusive
}
typedef enum
{
    SCENE_MODE_SINGLE_ENDPOINT = 0,
    SCENE_MODE_MULTI_ENDPOINT
} scene_storage_mode_t;

typedef struct
{
    uint8_t endpoint;
    uint16_t cluster_id;
    uint8_t onoff;
    uint8_t level;
    uint16_t color;
 //----------------------------------------------------------
    // RGB DEVICE SUPPORT
    //----------------------------------------------------------
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)

    uint8_t color_mode;

    uint8_t r;
    uint8_t g;
    uint8_t b;

    uint16_t color_temp;

    #endif
    bool valid;
} scene_ep_state_t;

#define MAX_SCENE_EPS   8

typedef struct
{
    scene_storage_mode_t mode;

    uint8_t total_eps;

    scene_ep_state_t ep[MAX_SCENE_EPS];

} decoded_scene_t;


uint8_t get_endpoint_index(uint8_t endpoint){
    for(int i=0; i<TOTAL_ENDPOINTS; i++){
        if(ENDPOINTS_LIST[i] == endpoint){
            return i;
        }
    }
    return 255;
}

bool decode_scene_extension_fields(
        esp_zb_zcl_scenes_extension_field_t *ext_list,
        uint8_t current_endpoint,
        decoded_scene_t *scene)
{

    if (!scene)
        return false;
        
    if(ext_list == NULL)
        return false;
    
    memset(scene, 0, sizeof(decoded_scene_t));
    //----------------------------------------------------------
    // Default
    //----------------------------------------------------------
    scene->mode = SCENE_MODE_SINGLE_ENDPOINT;
    scene->total_eps = 1;
    //----------------------------------------------------------
    // AUTO DETECT MODE
    //----------------------------------------------------------

    esp_zb_zcl_scenes_extension_field_t *tmp = ext_list;

    while (tmp)
    {
        switch (tmp->cluster_id)
        {
            //--------------------------------------------------
            // ONOFF / LEVEL
            //--------------------------------------------------
            case 0x0006:
            case 0x0008:
            {
                if (tmp->length > 2)
                {
                    scene->mode = SCENE_MODE_MULTI_ENDPOINT;
                    uint8_t eps = tmp->length / 2;

                    if (eps > scene->total_eps)
                    {
                        scene->total_eps = eps;
                    }


                    // if (tmp->length > scene->total_eps)
                    // {
                    //     scene->total_eps = tmp->length;
                    // }
                }
            }
            break;

            //--------------------------------------------------
            // COLOR
            //--------------------------------------------------
            case 0x0300:
            {
                #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
                //------------------------------------------------------
                // RGB device:
                // 7 bytes = 1 endpoint
                //------------------------------------------------------
                if(tmp->length > 7)
                {
                    scene->mode = SCENE_MODE_MULTI_ENDPOINT;

                    uint8_t eps = tmp->length / 7;

                    if(eps > scene->total_eps)
                    {
                        scene->total_eps = eps;
                    }
                }
                else
                {
                    //--------------------------------------------------
                    // Single RGB endpoint
                    //--------------------------------------------------

                    scene->mode = SCENE_MODE_SINGLE_ENDPOINT;

                    scene->total_eps = 1;
                }

                #else

                //------------------------------------------------------
                // Normal color temperature device
                //------------------------------------------------------

                if (tmp->length > 2)
                {
                    scene->mode = SCENE_MODE_MULTI_ENDPOINT;

                    uint8_t eps = tmp->length / 2;

                    if (eps > scene->total_eps)
                    {
                        scene->total_eps = eps;
                    }
                }

                #endif
            }
            break;

            default:
                break;
        }

        tmp = tmp->next;
    }

    //----------------------------------------------------------
    // SINGLE ENDPOINT MODE
    //----------------------------------------------------------

    if (scene->mode == SCENE_MODE_SINGLE_ENDPOINT)
    {
        scene->ep[0].endpoint = current_endpoint;
        scene->ep[0].valid = true;
    }
    else
    {
        //------------------------------------------------------
        // MULTI ENDPOINT MODE
        //------------------------------------------------------

        for (uint8_t i = 0; i < scene->total_eps; i++)
        {
            scene->ep[i].endpoint = i + 1;
            scene->ep[i].valid = true;
        }
    }

    //----------------------------------------------------------
    // DECODE
    //----------------------------------------------------------

    while (ext_list)
    {
        uint16_t cluster_id = ext_list->cluster_id;
        uint8_t *data = ext_list->extension_field_attribute_value_list;
        uint8_t len = ext_list->length;

        switch (cluster_id)
        {

            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_DALI_DIRECT_SWITCH)
            case 0x0006:   // On/Off
            {
                if (scene->mode == SCENE_MODE_MULTI_ENDPOINT)
                {
                    uint8_t pair_count = len / 2;

                    for (uint8_t ep = 0; ep < (scene->total_eps) && ep < pair_count; ep++)
                    {
                        uint8_t base = ep * 2;

                        scene->ep[ep].cluster_id = cluster_id;
                        scene->ep[ep].endpoint   = ENDPOINTS_LIST[get_endpoint_index(data[base])];
                        scene->ep[ep].onoff      = data[base + 1];
                    }
                }
                else
                {
                    scene->ep[0].cluster_id = cluster_id;
                    scene->ep[0].endpoint   = ENDPOINTS_LIST[get_endpoint_index(data[0])];
                    scene->ep[0].onoff      = data[1];
                }
            }
            break;

            case 0x0008:   // Level
            {
                if (scene->mode == SCENE_MODE_MULTI_ENDPOINT)
                {
                    uint8_t pair_count = len / 2;

                    for (uint8_t ep = 0; ep < scene->total_eps && ep < pair_count; ep++)
                    {
                        uint8_t base = ep * 2;

                        scene->ep[ep].cluster_id = cluster_id;
                        scene->ep[ep].endpoint   = ENDPOINTS_LIST[get_endpoint_index(data[base])];
                        scene->ep[ep].level      = data[base + 1];
                    }
                } else {
                    scene->ep[0].cluster_id = cluster_id;
                    scene->ep[0].endpoint   = ENDPOINTS_LIST[get_endpoint_index(data[0])];
                    scene->ep[0].level      = data[1];
                }
            }
            break;   
            #else
            
            // --------------------------------------------------
            // ONOFF
            // --------------------------------------------------
            case 0x0006:
            {
                if (scene->mode == SCENE_MODE_MULTI_ENDPOINT)
                {
                    for (uint8_t ep = 0;
                        ep < scene->total_eps && ep < len;
                        ep++)
                    {
                        scene->ep[ep].cluster_id = cluster_id;

                        scene->ep[ep].onoff = data[ep];
                    }
                }
                else
                {
                    scene->ep[0].cluster_id = cluster_id;

                    scene->ep[0].onoff = data[0];
                }
            }
            break;

            //--------------------------------------------------
            // LEVEL
            //--------------------------------------------------
            case 0x0008:
            {
                if (scene->mode == SCENE_MODE_MULTI_ENDPOINT)
                {
                    for (uint8_t ep = 0;
                        ep < scene->total_eps && ep < len;
                        ep++)
                    {
                        scene->ep[ep].cluster_id = cluster_id;

                        scene->ep[ep].level = data[ep];
                    }
                }
                else
                {
                    scene->ep[0].cluster_id = cluster_id;

                    scene->ep[0].level = data[0];
                }
            }
            break;            

            #endif

            //--------------------------------------------------
            // COLOR
            //--------------------------------------------------
            case 0x0300:
            {

                #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)

                //------------------------------------------------------
                // RGB DEVICE FORMAT
                // 7 bytes per endpoint
                //------------------------------------------------------

                uint8_t total_colors = len / 7;

                for (uint8_t ep = 0;
                    ep < scene->total_eps &&
                    ep < total_colors;
                    ep++)
                {
                    uint16_t offset = ep * 7;

                    colorControlExtensionField_t color;

                    memcpy(color.raw,
                        &data[offset],
                        7);

                    scene->ep[ep].cluster_id = cluster_id;

                    scene->ep[ep].color_mode =
                        color.fields.mode;

                    scene->ep[ep].r =
                        color.fields.r;

                    scene->ep[ep].g =
                        color.fields.g;

                    scene->ep[ep].b =
                        color.fields.b;

                    //--------------------------------------------------
                    // USE BRIGHT AS LEVEL
                    //--------------------------------------------------

                    scene->ep[ep].level =
                        color.fields.bright;

                    scene->ep[ep].color_temp =
                        color.fields.colorTemp;

                    scene->ep[ep].valid = true;
                }

            #else
                if (scene->mode == SCENE_MODE_MULTI_ENDPOINT)
                {
                    uint8_t total_colors = len / 2;

                    for (uint8_t ep = 0;
                        ep < scene->total_eps &&
                        ep < total_colors;
                        ep++)
                    {
                        scene->ep[ep].cluster_id = cluster_id;

                        scene->ep[ep].color =
                            ((uint16_t)data[(ep * 2) + 1] << 8) |
                            data[(ep * 2)];
                    }
                }
                else
                {
                    if (len >= 2)
                    {
                        scene->ep[0].cluster_id = cluster_id;

                        scene->ep[0].color =
                            ((uint16_t)data[1] << 8) |
                            data[0];
                    }
                }
            #endif    
            }
            break;

            default:
                break;
        }

        ext_list = ext_list->next;
    }

    return true;
}


void print_decoded_scene(decoded_scene_t *scene)
{
    if (!scene)
        return;

    printf("\n========================================\n");
    printf("Scene Mode : %s\n",
           scene->mode == SCENE_MODE_MULTI_ENDPOINT ?
           "MULTI_ENDPOINT" :
           "SINGLE_ENDPOINT");

    printf("Total Endpoints : %d\n", scene->total_eps);
    printf("========================================\n");

    for (uint8_t i = 0; i < scene->total_eps; i++)
    {
        if (!scene->ep[i].valid)
            continue;

        printf("\nEndpoint : %d\n",
               scene->ep[i].endpoint);

        printf("----------------------------------------\n");

        printf("ON/OFF : %s (%d)\n",
               scene->ep[i].onoff ? "ON" : "OFF",
               scene->ep[i].onoff);

        if(scene->ep[i].onoff == 255) return;       
        if(scene->ep[i].onoff) {  
            printf("LEVEL  : %d (0x%02X)\n",
                scene->ep[i].level,
                scene->ep[i].level);
        
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)

                printf("MODE   : %d\n",
                    scene->ep[i].color_mode);

                printf("R      : %d\n",
                    scene->ep[i].r);

                printf("G      : %d\n",
                    scene->ep[i].g);

                printf("B      : %d\n",
                    scene->ep[i].b);

                printf("CT     : %d\n",
                    scene->ep[i].color_temp);

                if(scene->ep[i].color_temp <2000 || scene->ep[i].color_temp > 6500){
                    scene->ep[i].color_temp = 2000;
                    printf("COLOR TEMP VALUE OUT OF RANGE\n");
                }

            #else

            printf("COLOR  : 0x%04X (%d)\n",
                scene->ep[i].color,
                scene->ep[i].color);

            #endif
        }
               
        uint8_t index_1 = 255;
        for(int j=0; j<TOTAL_ENDPOINTS; j++){
            if(ENDPOINTS_LIST[j] == scene->ep[i].endpoint){ 
                index_1 = j;
                printf("Matched Endpoint:%d, index:%d\n", scene->ep[i].endpoint, index_1);
                break; 
            }
        }
        
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_ANALOG_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_2T_PHASE_CUT_DIMMABLE_LIGHT || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN_CUSTOM )
            if(index_1 != 255){
                device_info[index_1].device_state = scene->ep[i].onoff;
                if(device_info[index_1].device_state){
                    device_info[index_1].device_level = scene->ep[i].level;
                    nuos_zb_set_hardware(index_1, false);
                    nuos_set_state_attribute(index_1);
                    nuos_set_level_attribute(index_1);
                } else{
                    nuos_zb_set_hardware(index_1, false);
                    nuos_set_state_attribute(index_1);
                } 
            }  
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
        if(index_1 < 255){
            static bool prev_onoff = false;

            device_info[0].device_state = scene->ep[i].onoff;
            if(device_info[0].device_state){ 

                device_info[0].device_val = scene->ep[i].color_temp;
                device_info[0].device_level = scene->ep[i].level;

                printf("My device_level:%d device_val:%d\n", device_info[0].device_level, device_info[0].device_val);
                nuos_zb_set_hardware(0, false);
                is_long_press_brightness = false;
                nuos_set_hardware_brightness_2(1);
                nuos_set_state_attribute(0);

                 if(prev_onoff != scene->ep[i].onoff){
                    prev_onoff = scene->ep[i].onoff;
                    device_info[0].device_state = scene->ep[i].onoff;

                    set_dali_color_temp(0, false);

                    if(dali_fade_time == 0){
                        vTaskDelay(50 / portTICK_PERIOD_MS);
                    }else if(dali_fade_time == 1){
                        vTaskDelay(700 / portTICK_PERIOD_MS);
                    }else if(dali_fade_time == 2){
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                    }else if(dali_fade_time == 3){
                        vTaskDelay(1400 / portTICK_PERIOD_MS);
                    }else if(dali_fade_time == 4){
                        vTaskDelay(2000 / portTICK_PERIOD_MS);
                    }else{
                        vTaskDelay(3000 / portTICK_PERIOD_MS);
                    }
                    set_dali_level(0);
                }else{
                    device_info[0].device_val = scene->ep[i].color_temp;
                    device_info[0].device_level = scene->ep[i].level;
                    set_dali_level(0); 
                    nuos_zb_set_hardware(0, false);
                    is_long_press_brightness = false;
                    nuos_set_hardware_brightness_2(1);
                    set_dali_color_temp(0, false);
                    
                }

                nuos_set_color_temperature_attribute(0);  
                
            }else{
                prev_onoff = false;
                nuos_zb_set_hardware(0, false);
                set_state(0);
                nuos_set_state_attribute(0);
            }
        }

        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_DALI_DIRECT_SWITCH)
            if(index_1 != 255){
                device_info[index_1].device_state = scene->ep[i].onoff;
                if(device_info[index_1].device_state){
                    #ifdef USE_COLOR_CONTROL
                    device_info[index_1].device_val = scene->ep[i].color;
                    printf("DALI DIRECT SWITCH - COLOR TEMP VALUE:%d index_1:%d\n", device_info[index_1].device_val,index_1);
                    #endif
                
                    if(scene->ep[i].level == 0) scene->ep[i].level = MIN_DIM_LEVEL_VALUE;

                    device_info[index_1].device_level = scene->ep[i].level;
                    // nuos_zb_set_hardware(index_1, false);
                    process_dali_tasks(index_1, false, true);
                    set_color_to_updown_leds(index_1);
                    nuos_set_state_attribute(index_1);

                    // zb_cmd_t cmd;
                    // cmd.type = CMD_SET_DALI_SET_STATE;
                    // cmd.index = index_1;
                    // cmd.state = device_info[index_1].device_state;
                    // cmd.level = device_info[index_1].device_level;  
                    // cmd.color_temp = scene->ep[i].color;

                    // /* Send decoded scene to queue */
                    // if (xQueueSend(scene_queue, &cmd, 0) != pdTRUE) {
                    //     ESP_LOGW(TAG, "Scene queue full");
                    // }

                    // /* Send decoded scene to queue */
                    // if (xQueueSend(scene_queue, &cmd, random_20_to_100 () / portTICK_PERIOD_MS) != pdTRUE) {
                    //     ESP_LOGW(TAG, "Scene queue full");
                    // }
                    nuos_set_level_attribute(index_1);
                    //nuos_dali_set_group_brightness(scene_group_switch_info.group_id[index_1], index_1, device_info[index_1].device_level);
                    #ifdef USE_COLOR_CONTROL
                    // cmd.type = CMD_SET_DALI_COLOR_TEMP;
                    // /* Send decoded scene to queue */
                    // if (xQueueSend(scene_queue, &cmd, 0) != pdTRUE) {
                    //     ESP_LOGW(TAG, "Scene queue full");
                    // }
                    // set_dali_color_temp(index_1, false);
                    nuos_set_color_temperature_attribute(index_1);
                    #endif
                } else{
                    nuos_zb_set_hardware(index_1, false);
                    nuos_set_state_attribute(index_1);
                } 
            }
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
            if(index_1 == 0){
                device_info[index_1].device_state = scene->ep[i].onoff;
                if(device_info[index_1].device_state){
                    device_info[index_1].device_level = scene->ep[i].level;
                    nuos_zb_set_hardware(index_1, false);
                    nuos_set_state_attribute(index_1);
                    nuos_set_level_attribute(index_1);
                    device_info[index_1].device_val = scene->ep[i].color;
                    set_dali_color_temp(index_1, false);
                    nuos_set_color_temperature_attribute(index_1);
                } else{
                    nuos_zb_set_hardware(index_1, false);
                    nuos_set_state_attribute(index_1);
                } 
            }  
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI)
            if(index_1 != 255){
                device_info[index_1].device_state = scene->ep[i].onoff;
                #ifdef DALI_DIRECT_ADDRESSING
                    #ifdef ENABLE_PWM_DIMMING
                        if(device_info[index_1].device_state){
                            device_info[index_1].device_level = scene->ep[i].level;                                 
                            nuos_set_level_attribute(index_1);                              
                            nuos_zb_set_hardware(index_1, false);
                        }
                    #endif
                #endif
            }
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
            printf("color_mode:%d\n", scene->ep[i].color_mode);
            if(scene->ep[i].color_mode == 0){
                index_1 = 3;
                if(selected_color_mode != 0){
                    selected_color_mode = scene->ep[i].color_mode;
                    mode_change_flag = true;
                    last_selected_color_mode = selected_color_mode;
                    nuos_set_color_rgb_mode_attribute(0, selected_color_mode);
                    store_color_mode_value(selected_color_mode);
                }                   
                
            }else if(scene->ep[i].color_mode == 1){
                index_1 = 4;
                if(selected_color_mode != 1){
                    selected_color_mode = scene->ep[i].color_mode;
                    mode_change_flag = true;
                    last_selected_color_mode = selected_color_mode;
                    nuos_set_color_rgb_mode_attribute(0, selected_color_mode);
                    store_color_mode_value(selected_color_mode);
                }
            }
            
            device_info[index_1].device_state = scene->ep[i].onoff;
            printf("index_1:%d state:%d\n", index_1, device_info[index_1].device_state);
            if(device_info[index_1].device_state){
                //set color
                if(scene->ep[i].color_mode == 0){
                    device_info[3].device_state = true;                   
                    uint16_t val = scene->ep[i].color_temp;
                    device_info[3].device_val = val;//map_1000(val, 0, 1000, MIN_CCT_VALUE, MAX_CCT_VALUE);
                    device_info[3].device_level = scene->ep[i].level;
    
                    device_info[3].device_state = true;
                    nuos_zb_set_hardware(3, false);
                    set_dali_color_temp(0, false);
                    set_dali_level(3);
                          
                }else if(scene->ep[i].color_mode == 1){

                    //selected_color_mode = scene->ep[i].color_mode;
                    // nuos_set_color_rgb_mode_attribute(0, selected_color_mode);
                    // store_color_mode_value(selected_color_mode);             

                    device_info[0].device_level = scene->ep[i].r;
                    device_info[1].device_level = scene->ep[i].g;
                    device_info[2].device_level = scene->ep[i].b;

                    printf("RED:%d GREEN:%d BLUE:%d\n", device_info[0].device_level, device_info[1].device_level, device_info[2].device_level); 
                    
                    device_info[4].device_level = scene->ep[i].level; 
                    if(device_info[4].device_level == 0) device_info[4].device_level = 254;
                    printf("BRIGHTNESS:%d\n", device_info[4].device_level);

                    rgb_t rgb = {device_info[0].device_level, device_info[1].device_level, device_info[2].device_level}; // Example RGB values

                    device_info[0].device_level = rgb.r;
                    device_info[1].device_level = rgb.g;
                    device_info[2].device_level = rgb.b;

                    if(device_info[0].device_level == 0) device_info[0].device_state = false;
                    else device_info[0].device_state = true;
                    if(device_info[1].device_level == 0) device_info[1].device_state = false;
                    else device_info[1].device_state = true;
                    if(device_info[2].device_level == 0) device_info[2].device_state = false;
                    else device_info[2].device_state = true;


                    for(int rgb=0; rgb<3; rgb++){
                        if(device_info[rgb].device_level <= MIN_DIM_LEVEL_VALUE) {
                            device_info[rgb].device_level = MIN_DIM_LEVEL_VALUE;
                        }
                        if(device_info[rgb].device_level == 0xff){
                            device_info[rgb].device_level = 0xfe;
                        }
                    } 
                    
                    #if(USE_COLOR_DEVICE == COLOR_RGB_ONLY)
                    uint8_t index = 4;
                    device_info[3].device_state = true;
                    #else
                    #endif
                    //store_color_mode_value(selected_color_mode);
                    nuos_zb_set_hardware(4, false); 
                    set_dali_color_temp(0, false);
                    set_dali_level(4);                                  
                }
            }else{
                nuos_zb_set_hardware(index_1, false);
                set_state(index_1);
                nuos_set_state_attribute(index_1);
            } 

        #else
        
        
        #endif
    }
    printf("\n========================================\n");
}




/* =========================================================
   Queue Init
   ========================================================= */
void scene_queue_init(void)
{
    /* Create queue for 20 scene objects */
    scene_queue = xQueueCreate(20, sizeof(zb_cmd_t));

    if (scene_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create scene queue");
        return;
    }

    /* Create task */
    xTaskCreate(
        scene_print_task,
        "scene_print_task",
        4096,
        NULL,
        TASK_PRIORITY_SCENE_QUEUE_TASK,
        NULL
    );

    ESP_LOGI(TAG, "Scene queue and task created");
}




/* =========================================================
   Scene Print Task
   ========================================================= */
static void scene_print_task(void *pvParameters)
{
    //decoded_scene_t rx_scene;
    zb_cmd_t cmd;
    while (1) {

        /* Wait forever for scene data */
        if (xQueueReceive(scene_queue, &cmd, portMAX_DELAY) == pdTRUE) {

            printf("\nReceived scene from queue:\n");
            // /* Random delay here instead of callback */
            /* Print received scene */
            //print_decoded_scene(&rx_scene);

            switch (cmd.type) {
                case CMD_SET_DALI_SET_STATE: {
                    printf("CMD_SET_DALI_SET_STATE : %d\n", cmd.index);
                    // Call your functions here, in this task context

                    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_DALI_DIRECT_SWITCH)
                    nuos_zb_set_hardware(cmd.index, false);
                    set_color_to_updown_leds(cmd.index);
                    set_dali_level(cmd.index);
                    //nuos_set_state_attribute(cmd.index);
                    //nuos_set_level_attribute(cmd.index);
                    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
        

                    nuos_zb_set_hardware(0, false);
                    is_long_press_brightness = false;
                    nuos_set_hardware_brightness_2(1);
                    if(cmd.level == 0) cmd.level = MIN_DIM_LEVEL_VALUE;
                    device_info[cmd.index].device_level = cmd.level;
                    nuos_dali_set_state_group(cmd.index, device_info[cmd.index].device_level);
                    set_dali_level(0); 
                    nuos_set_state_attribute(0);
                    nuos_set_color_temp_level_attribute(0);
                    #endif
                    break;
                }
                case CMD_SET_DALI_SET_LEVEL: {
                    // Call your functions here, in this task context
                    nuos_dali_set_group_brightness(scene_group_switch_info.group_id[cmd.index], cmd.index, device_info[cmd.index].device_level);
                    nuos_set_level_attribute(cmd.index);
                    break;
                }
                case CMD_SET_DALI_COLOR_TEMP: {
                    printf("CMD_SET_DALI_COLOR_TEMP : %d\n", cmd.index);
                    // Call your functions here, in this task context
                    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_DALI_DIRECT_SWITCH)
                    set_dali_color_temp(cmd.index, cmd.color_temp);
                    nuos_set_color_temperature_attribute(cmd.index);

                    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
                    set_dali_color_temp(0, false);
                    
                    nuos_set_color_temperature_attribute(0);  
                    #endif
                       
                    break;
                }                
                default:
                printf("Unknown command type received: %d\n", cmd.type);
                    break;
            }
        }
    }
}

void nuos_set_scene(esp_zb_zcl_recall_scene_message_t *message){
        decoded_scene_t local_scene;
        if(decode_scene_extension_fields(
            message->field_set,
            message->info.dst_endpoint,
            &local_scene)) {
                //vTaskDelay(random_50_to_200 () / portTICK_PERIOD_MS);
                print_decoded_scene(&local_scene);
                // /* Send decoded scene to queue */
                // if (xQueueSend(scene_queue, &local_scene, random_20_to_100 () / portTICK_PERIOD_MS) != pdTRUE) {
                //     ESP_LOGW(TAG, "Scene queue full");
                // }

            }
}



void nuos_set_scene_OK(esp_zb_zcl_recall_scene_message_t *message){

    uint8_t index_1 = message->info.dst_endpoint-1;
    esp_zb_zcl_scenes_extension_field_t* ext_list = message->field_set;
    while (ext_list != NULL) {
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
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
            #ifdef USE_FAN_SPEED
                for (int i = 0; i < ext_list->length; i++) {
                    printf("index:%d,  CLUSTER_ID: 0x%x ATTR: 0x%02X\n", index_1, ext_list->cluster_id, ext_list->extension_field_attribute_value_list[i]);
                    if(i==index_1){
                        control_zb_devices(index_1, ext_list->cluster_id, &ext_list->extension_field_attribute_value_list[i]);                  
                    }
                    index_1++;              
                }
                index_1 = 0;            
            #else
            control_zb_devices(index_1, ext_list->cluster_id, &ext_list->extension_field_attribute_value_list[0]);
            #endif 
        #else
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI) 
                printf("CLUSTER_ID: 0x%x\n", ext_list->cluster_id);
                #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
                control_zb_devices(0, ext_list->cluster_id, &ext_list->extension_field_attribute_value_list[0]); 

                if(device_info[0].device_state){
                    vTaskDelay(500 / portTICK_PERIOD_MS); 
                    set_dali_level(0); 
                }
                #elif(USE_COLOR_DEVICE == COLOR_RGB_ONLY)
                control_zb_devices(4, ext_list->cluster_id, &ext_list->extension_field_attribute_value_list[0]); 
                #else
                control_zb_devices(4, ext_list->cluster_id, &ext_list->extension_field_attribute_value_list[0]); 
                #endif
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI)
                #ifdef DALI_DIRECT_ADDRESSING
                    for (int i = 0; i < ext_list->length; i++) {
                        printf("index:%d,  CLUSTER_ID: 0x%x ATTR: 0x%02X\n", index_1, ext_list->cluster_id, ext_list->extension_field_attribute_value_list[i]);
                        if(i==index_1){
                            control_zb_devices(index_1, ext_list->cluster_id, &ext_list->extension_field_attribute_value_list[i]);                  
                        }
                        index_1++;              
                    }
                    index_1 = 0;//message->info.dst_endpoint-1;  
                #else
                    control_zb_devices(index_1, ext_list->cluster_id, &ext_list->extension_field_attribute_value_list[0]);
                #endif
            #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_DALI_DIRECT_SWITCH)
                
                // #ifdef USE_COLOR_CONTROL
                uint8_t surma_8[2] = {0,0};
                uint16_t surma_16 = 0;
                
                for (int i = 0; i < ext_list->length; i++) { 
   
                    if(ext_list->cluster_id == 0x0300){
                        if(i == 0){
                            surma_8[0] = (uint8_t)ext_list->extension_field_attribute_value_list[i];
                        }else if(i == 1) {
                            surma_8[1] = (uint8_t)ext_list->extension_field_attribute_value_list[i];
                            surma_16 = (surma_8[1] << 8) | surma_8[0];
                            control_zb_devices(0, ext_list->cluster_id, &surma_16);
                            
                            if(device_info[0].device_state){
                                vTaskDelay(500 / portTICK_PERIOD_MS); 
                                set_dali_level(0); 
                            }
                        }else if(i == 2){
                            surma_8[0] = (uint8_t)ext_list->extension_field_attribute_value_list[i];
                        }else if(i == 3) {
                            surma_8[1] = (uint8_t)ext_list->extension_field_attribute_value_list[i];
                            surma_16 = (surma_8[1] << 8) | surma_8[0];
                            control_zb_devices(1, ext_list->cluster_id, &surma_16);


                            if(device_info[1].device_state){
                                vTaskDelay(500 / portTICK_PERIOD_MS); 
                                set_dali_level(1); 
                            }
                        }
                    }else{
   
                        uint8_t _index = i / 2;
                        if(ext_list->cluster_id == 0x0006 || ext_list->cluster_id == 0x0008){
                            if(_index == 0){ 
                                uint8_t _pass_index_first = get_endpoint_index(ext_list->extension_field_attribute_value_list[0]);
                                control_zb_devices(_pass_index_first, ext_list->cluster_id, &ext_list->extension_field_attribute_value_list[1]);
                            }else if(_index == 1){ 
                                uint8_t _pass_index_second = get_endpoint_index(ext_list->extension_field_attribute_value_list[2]);
                                control_zb_devices(_pass_index_second, ext_list->cluster_id, &ext_list->extension_field_attribute_value_list[3]);
                            
                            }
                        } 
                    }
                }    
                // #else
                // control_zb_devices(index_1, ext_list->cluster_id, &ext_list->extension_field_attribute_value_list[0]);
                // #endif
            #else  
                control_zb_devices(i, ext_list->cluster_id, &ext_list->extension_field_attribute_value_list[i]);
            #endif    
        #endif
        ext_list = ext_list->next;
    }
    // printf("End....\n");
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
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER  || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
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