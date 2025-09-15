

#include "app_hardware_driver.h"

#include "esp_zigbee_core.h"
#include "esp_err.h"
#include "esp_check.h"

static const char *TAG = "ESP_ZB_GROUP_COMMANDS";
/********************LOW LEVEl Functions************************/
void _nuos_zb_group_send_add_group_request(uint16_t group_id, uint8_t src_ep, uint8_t dst_ep, uint16_t dst_short_addr, esp_zb_zcl_address_mode_t address_mode){
    esp_zb_zcl_groups_add_group_cmd_t cmd_req;
    cmd_req.group_id = group_id;
    cmd_req.address_mode = address_mode;
    cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = dst_short_addr;
    cmd_req.zcl_basic_cmd.dst_endpoint = dst_ep;
    cmd_req.zcl_basic_cmd.src_endpoint = src_ep;
    esp_zb_zcl_groups_add_group_cmd_req(&cmd_req);
}

void _nuos_zb_group_send_remove_group_request(uint16_t group_id, uint8_t src_ep, uint8_t dst_ep, uint16_t dst_short_addr, esp_zb_zcl_address_mode_t address_mode){
    esp_zb_zcl_groups_add_group_cmd_t cmd_req;
    cmd_req.group_id = group_id;
    cmd_req.address_mode = address_mode;
    cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = dst_short_addr;
    cmd_req.zcl_basic_cmd.dst_endpoint = dst_ep;
    cmd_req.zcl_basic_cmd.src_endpoint = src_ep;
    esp_zb_zcl_groups_remove_group_cmd_req(&cmd_req);
}
/********************HIGH LEVEl Functions***********************/
void nuos_zigbee_group_query_all_groups(uint16_t group_id)
{
    // Define the ZCL basic command and other required parameters
    esp_zb_zcl_groups_get_group_membership_cmd_t cmd;
    // memset(&cmd, 0, sizeof(cmd));
    // Initialize the basic command information
    cmd.zcl_basic_cmd.dst_addr_u.addr_short = group_id;
    cmd.zcl_basic_cmd.dst_endpoint = 0; // Groupcast does not use endpoint
    cmd.zcl_basic_cmd.src_endpoint = 1;
    // Set the command ID and sequence number
    cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_GROUP_ENDP_NOT_PRESENT;
    cmd.group_count = 0; // Request all groups by setting group_count to 0
    // Send the command
    //esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_groups_get_group_membership_cmd_req(&cmd);
    //esp_zb_lock_release();
}

/// @brief //////////ADD GROUP
/// @param group_id 
/// @param src_ep 
/// @param dst_ep 
/// @param dst_short_addr 
void nuos_zb_group_send_add_group_request_to_dst(uint16_t group_id, uint8_t src_ep, uint8_t dst_ep, uint16_t dst_short_addr){
    _nuos_zb_group_send_add_group_request(group_id, src_ep, dst_ep, dst_short_addr, ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT);
}
void nuos_zb_group_send_add_group_request_to_broadcast(uint16_t group_id, uint8_t src_ep, uint8_t dst_ep){
    _nuos_zb_group_send_add_group_request(group_id, src_ep, dst_ep, 0xFFFF, ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT);
}
void nuos_zb_group_send_add_group_request_to_self(uint16_t group_id, uint8_t dst_ep, uint8_t src_ep){
    _nuos_zb_group_send_add_group_request(group_id, dst_ep, src_ep, esp_zb_get_short_address(), ESP_ZB_APS_ADDR_MODE_16_GROUP_ENDP_NOT_PRESENT);
}

/// @brief //////////REMOVE GROUP
/// @param group_id 
/// @param src_ep 
/// @param dst_ep 
/// @param dst_short_addr 
void nuos_zb_group_send_remove_group_request_to_dst(uint16_t group_id, uint8_t src_ep, uint8_t dst_ep, uint16_t dst_short_addr){
    _nuos_zb_group_send_remove_group_request(group_id, src_ep, dst_ep, dst_short_addr, ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT);
}
void nuos_zb_group_send_remove_group_request_to_broadcast(uint16_t group_id, uint8_t src_ep, uint8_t dst_ep){
    _nuos_zb_group_send_remove_group_request(group_id, src_ep, dst_ep, 0xFFFF, ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT);
}
void nuos_zb_group_send_remove_group_request_to_self(uint16_t group_id, uint8_t dst_ep, uint8_t src_ep){
    _nuos_zb_group_send_remove_group_request(group_id, dst_ep, src_ep, esp_zb_get_short_address(), ESP_ZB_APS_ADDR_MODE_16_GROUP_ENDP_NOT_PRESENT);
}
unsigned int map_cct1(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void nuos_zb_group_send_light_onoff_groupcast(uint16_t group_id, uint8_t  on_off_cmd_id){
    esp_zb_zcl_on_off_cmd_t cmd_req;
    cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = group_id;
    cmd_req.zcl_basic_cmd.dst_endpoint = 0;
    //cmd_req.zcl_basic_cmd.src_endpoint = 0xff;
    cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_GROUP_ENDP_NOT_PRESENT;
    cmd_req.on_off_cmd_id = on_off_cmd_id;
    esp_zb_lock_acquire(portMAX_DELAY);
	esp_zb_zcl_on_off_cmd_req(&cmd_req);
    esp_zb_lock_release();
}

void nuos_zb_group_send_light_level_groupcast(uint16_t group_id, uint8_t level){
    esp_zb_zcl_move_to_level_cmd_t cmd_level;
    cmd_level.zcl_basic_cmd.dst_addr_u.addr_short = group_id;
    cmd_level.zcl_basic_cmd.dst_endpoint = 0;
    cmd_level.address_mode = ESP_ZB_APS_ADDR_MODE_16_GROUP_ENDP_NOT_PRESENT;
    cmd_level.level = level;
    cmd_level.transition_time = 0xffff;
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_level_move_to_level_with_onoff_cmd_req(&cmd_level);
    esp_zb_lock_release();
}


void zigbee_cluster_color_temperature_update(uint16_t group_id, uint16_t temp){
    uint16_t mapValue = temp;
    mapValue = map_cct1(temp, MIN_CCT_VALUE, MAX_CCT_VALUE, 500, 158);
    esp_zb_zcl_color_move_to_color_temperature_cmd_t cmd_req;
    cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = group_id;
    cmd_req.zcl_basic_cmd.dst_endpoint = 0;
    cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_GROUP_ENDP_NOT_PRESENT;  
    cmd_req.color_temperature = mapValue;
    cmd_req.transition_time = 0;
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_color_move_to_color_temperature_cmd_req(&cmd_req);
    esp_zb_lock_release();

}
/***********************ACTION HANDLERS**********************/
static esp_err_t zb_group_get_membership_handler(const esp_zb_zcl_groups_get_group_membership_resp_message_t* message) {
	//ESP_LOGW(TAG, "---------On group_req:%d, ep%d membership-----------", group_rx_counts, message->info.src_endpoint);
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_LOGI(TAG, "Received report from group_count(%d) src_addr(0x%x) ", message->group_count, message->info.src_address.u.short_addr);
//    ESP_LOGI(TAG, " address(0x%x) src endpoint(%d) to dst endpoint(%d) cluster(0x%x)", message->info.dst_address ,message->info.src_endpoint,
//    		message->info.dst_endpoint, message->info.cluster);

    if(message->info.dst_address == esp_zb_get_short_address()){
    	ESP_LOGI(TAG, "short address matches!!");
    }
    for(int i=0; i<message->group_count; i++){
    	ESP_LOGI(TAG, "group_id[%d]: %d", i, message->group_id[i]);

//    	save_groups_id(message->group_id[i]);
//    	ESP_LOGI(TAG, "cmd_id(%d), cmd_dir(%d), cmd_common(%d)", message->info.command.id, message->info.command.direction, message->info.command.is_common);
//    	ESP_LOGI(TAG, "manufcode(0x%x), tsn(%d), fc(%d)", message->info.header.manuf_code,
//    			message->info.header.tsn, message->info.header.fc);
    }
    ESP_LOGW(TAG, "-----------------------END-------------------------");
    return ESP_OK;
}

static esp_err_t zb_operate_Group_resp_handler(const esp_zb_zcl_groups_operate_group_resp_message_t *message)
{
    esp_err_t ret = ESP_OK;
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message2: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message2g: group_id(0x%x), cluster(0x%x), dst_ep(%d), src_ep(%d)", message->group_id, message->info.cluster, message->info.dst_endpoint,
              message->info.src_endpoint);

    _nuos_zb_group_send_add_group_request(message->group_id, message->info.src_endpoint, 0xff, esp_zb_get_short_address(), ESP_ZB_APS_ADDR_MODE_16_GROUP_ENDP_NOT_PRESENT);
    return ret;
}

static esp_err_t zb_view_Group_resp_handler(const esp_zb_zcl_groups_view_group_resp_message_t *message)
{
    esp_err_t ret = ESP_OK;
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message3: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message3g: group_id(0x%x), cluster(0x%x), dst_ep(%d), src_ep(%d)", message->group_id, message->info.cluster, message->info.dst_endpoint,
              message->info.src_endpoint);

    return ret;
}



esp_err_t nuos_zb_group_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message){
    esp_err_t ret = -1;
    switch (callback_id) {
	case ESP_ZB_CORE_CMD_VIEW_GROUP_RESP_CB_ID:
		ret = zb_view_Group_resp_handler((esp_zb_zcl_groups_view_group_resp_message_t*)message);
		break;
    case ESP_ZB_CORE_CMD_OPERATE_GROUP_RESP_CB_ID:
		ret = zb_operate_Group_resp_handler((esp_zb_zcl_groups_operate_group_resp_message_t*)message);
		break;
    case ESP_ZB_CORE_CMD_GET_GROUP_MEMBERSHIP_RESP_CB_ID:
        ret = zb_group_get_membership_handler((esp_zb_zcl_groups_get_group_membership_resp_message_t*)message);
        break;
    default:
        // ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}