#ifndef _APP_ZIGBEE_QUERY_NODES_H_
#define _APP_ZIGBEE_QUERY_NODES_H_
    #include "ha/esp_zigbee_ha_standard.h"
    void nuos_zb_find_clusters(esp_zb_zdo_match_desc_callback_t user_cb);
    uint8_t nuos_find_active_nodes(uint8_t src_endpoint, uint8_t total_nodes, uint8_t erase_data);  
    void do_remote_scene_bindings(uint8_t src_ep, uint8_t dst_ep, uint16_t short_addr, bool is_binding);  
    void set_remote_scene_devices_attribute_values(void * void_message);
    void send_zb_to_read_basic_attribute_values(uint8_t src_endpoint, uint8_t dst_endpoint, uint16_t addr_short);
    void clear_remote_scene_devices_attribute_bind_values(uint8_t btn_index, uint8_t node_index, uint8_t ep_index, uint16_t short_addr);

    uint8_t get_button_index(uint8_t endpoint_id);
    uint8_t get_node_index(uint8_t btn_index, uint16_t short_addr);
    uint8_t get_ep_index(uint8_t btn_index, uint8_t node_index, uint8_t dst_ep);
    void send_zcl_read_attr_request(uint16_t short_addr, uint16_t cluster_id, uint16_t attr_id, uint8_t dst_ep);
    uint8_t nuos_bind_task(uint8_t src_index, void * data);
    uint8_t nuos_unbind_task(uint8_t src_index, void * data);
    extern void start_led_blinking(uint8_t mode, uint8_t src_index);
    extern void stop_led_blinking();
    extern void nuos_led_blink_task(bool toggle);
    extern bool nuos_is_zigbee_busy();
#endif