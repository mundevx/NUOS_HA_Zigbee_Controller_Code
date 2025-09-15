
#ifndef _NUOS_ZIGBEE_GROUP_COMMANDS
#define _NUOS_ZIGBEE_GROUP_COMMANDS

    #include "esp_zigbee_core.h"
    void nuos_zigbee_group_query_all_groups(uint16_t group_id);

    void nuos_zb_group_send_add_group_request_to_dst(uint16_t group_id, uint8_t src_ep, uint8_t dst_ep, uint16_t dst_short_addr);
    void nuos_zb_group_send_add_group_request_to_broadcast(uint16_t group_id, uint8_t src_ep, uint8_t dst_ep);
    void nuos_zb_group_send_add_group_request_to_self(uint16_t group_id, uint8_t dst_ep, uint8_t src_ep);

    void nuos_zb_group_send_remove_group_request_to_dst(uint16_t group_id, uint8_t src_ep, uint8_t dst_ep, uint16_t dst_short_addr);
    void nuos_zb_group_send_remove_group_request_to_broadcast(uint16_t group_id, uint8_t src_ep, uint8_t dst_ep);
    void nuos_zb_group_send_remove_group_request_to_self(uint16_t group_id, uint8_t dst_ep, uint8_t src_ep);

    void nuos_zb_group_send_light_onoff_groupcast(uint16_t group_id, uint8_t  on_off_cmd_id);
    void nuos_zb_group_send_light_level_groupcast(uint16_t group_id, uint8_t  level);

    esp_err_t nuos_zb_group_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message);
    void zigbee_cluster_color_temperature_update(uint16_t group_id, uint16_t temp);
#endif
    