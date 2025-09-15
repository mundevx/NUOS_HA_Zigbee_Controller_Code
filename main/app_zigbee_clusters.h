#ifndef _APP_ZIGBEE_CLUSTER_H_
#define _APP_ZIGBEE_CLUSTER_H_
    #include "ha/esp_zigbee_ha_standard.h"
    #include "app_zigbee_query_nodes.h"
    #include "app_zigbee_misc.h"
    extern esp_err_t nuos_driver_init(void);
    extern esp_zb_ep_list_t * nuos_init_zb_clusters();
    
    #ifdef __cplusplus
    extern "C" {
    #endif
    #ifdef USE_CCT_TIME_SYNC
    void read_zigbee_time();
    void set_time();
    #endif
    extern void nuos_set_zigbee_attribute(uint8_t index);
    extern esp_err_t nuos_set_thermostat_temp_attribute(uint8_t index);
    
    void nuos_set_privilege_command_attribute(const esp_zb_zcl_privilege_command_message_t *message);
    esp_err_t nuos_set_color_temp_level_attribute(uint8_t index);
    esp_err_t nuos_set_level_attribute(uint8_t index);
    esp_err_t nuos_set_color_temperature_attribute(uint8_t index);
    esp_err_t nuos_set_color_rgb_mode_attribute(uint8_t index, uint8_t val_mode);
    hsv_t rgb_to_hsv(rgb_t rgb);
    rgb_t hsv_to_rgb(hsv_t hsv);
    esp_err_t nuos_set_color_xy_attribute(uint8_t index, hsv_t* hsv);
    esp_err_t nuos_set_color_temp_attribute(uint8_t index);
    unsigned int map_1000(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max); 
    void nuos_report_curtain_blind_state(uint8_t index, uint8_t value);   
    #ifdef __cplusplus
    }
    #endif
    extern void nuos_zb_init_motion_sensor();
    extern void nuos_zb_find_clusters(esp_zb_zdo_match_desc_callback_t user_cb);
    extern void nuos_zb_bind_cb(esp_zb_zdp_status_t zdo_status, esp_zb_ieee_addr_t ieee_addr, void *user_ctx);
    extern void nuos_zb_request_ieee_address(esp_zb_zdp_status_t zdo_status, 
    uint16_t addr, uint8_t endpoint, 
    void *user_ctx, esp_zb_zdo_ieee_addr_callback_t ieee_cb);
    extern void send_active_request();
    esp_err_t nuos_zigbee_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message);
    void setup_reporting_for_thermostat_system_mode_id() ;
    void nuos_set_attribute_cluster(const esp_zb_zcl_set_attr_value_message_t *message);
    void nuos_set_scene_group_cluster(const esp_zb_zcl_recall_scene_message_t *message);
    void nuos_set_scene_store_cluster(const esp_zb_zcl_store_scene_message_t *message);
    
    esp_err_t nuos_set_sensor_temperature_hunidity_attribute(uint8_t index);
    esp_err_t nuos_set_ac_cool_temperature_attribute(uint8_t index);
    esp_err_t nuos_set_tuya_onoff(uint8_t index, void* data);
    esp_err_t nuos_set_tuya_temp(uint8_t index, uint16_t data);
    esp_err_t nuos_set_thermostat_mode_attr(uint8_t index, uint8_t mode_val);
    bool is_value_present(uint8_t arr[], uint8_t size, uint8_t value);
    void nuos_set_scene_devices(uint8_t index, bool _is_state, uint8_t dlevel, uint8_t switch_id, uint8_t _dst_ep, uint16_t _short_addr);
    void send_report(int index, uint16_t cluster_id, uint16_t attr_id);
    void nuos_start_bootloader();    
    void nuos_init_privilege_commands();
    bool nuos_init_sequence();    
#endif