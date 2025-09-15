
#ifndef _NUOS_ZIGBEE_SCENE_COMMANDS
#define _NUOS_ZIGBEE_SCENE_COMMANDS
    void nuos_zb_scene_query_all_scenes_request(uint16_t group_id, uint8_t src_endpoint);
    void nuos_zb_scene_add_scene_unicast_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep, uint16_t dst_addr_short, uint8_t value_on_off, uint8_t value_level);
    void nuos_zb_scene_add_scene_broadcast_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep, uint8_t value_on_off, uint8_t value_level);
    void nuos_zb_scene_add_scene_groupcast_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep, uint8_t value_on_off, uint8_t value_level);
    //void nuos_zb_scene_add_scene_to_self_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep, uint8_t value_on_off, uint8_t value_level);
    
    void nuos_zb_scene_recall_scene_unicast_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep, uint16_t dst_addr_short);
    void nuos_zb_scene_recall_scene_broadcast_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep);
    void nuos_zb_scene_recall_scene_groupcast_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep);

    void nuos_zb_scene_store_scene_unicast_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep, uint16_t dst_addr_short);
    void nuos_zb_scene_store_scene_broadcast_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep);
    void nuos_zb_scene_store_scene_groupcast_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep);

    esp_err_t nuos_zb_scene_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message);


    void nuos_zb_scene_remove_scene_unicast_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep, uint16_t dst_addr_short);
    void nuos_zb_scene_remove_scene_broadcast_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep);
    void nuos_zb_scene_remove_scene_groupcast_request(uint16_t group_id, uint8_t scene_id, uint8_t src_ep, uint8_t dst_ep);

    esp_err_t nuos_set_store_scene(esp_zb_zcl_store_scene_message_t* message);
    void set_curtain_percentage(uint8_t value);
#endif
    