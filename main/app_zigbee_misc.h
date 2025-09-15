#ifndef _APP_ZIGBEE_NUOS_MISC_H_
#define _APP_ZIGBEE_NUOS_MISC_H_
    #include "ha/esp_zigbee_ha_standard.h"
    void query_identify_command(uint16_t dst_addr, uint8_t dst_endpoint);
    void send_identify_trigger_effect_command(uint16_t dst_addr, uint8_t dst_endpoint);
    void send_identify_command(uint16_t dst_addr, uint8_t src_endpoint, uint8_t dst_endpoint, uint16_t identify_time);
    void query_identify_command(uint16_t dst_addr, uint8_t dst_endpoint);
    void identify_cluster_task(void* args);
    void send_identify_trigger_effect(uint16_t dst_addr, uint8_t dst_endpoint);

    void nuos_simple_binding_request(uint8_t src_endpoint, uint8_t ep_index, dst_node_info_t *dst_node_info);
    void nuos_simple_unbinding_request(uint8_t src_endpoint, uint8_t ep_index, dst_node_info_t *dst_node_info);
    void nuos_attribute_get_request(uint8_t btn_index, uint8_t ep_index, dst_node_info_t *dst_node_info, uint8_t cl_index);
     
    bool nuos_init_wifi_webserver();
    void config_reporting();
    void setup_reporting_for_fan_attributes();
    
    void nuos_zb_nlme_status_indication();
    void nuos_zb_indentify_indication(uint16_t cluster_id, uint8_t dst_endpoint, uint8_t effect_id);
    void nuos_zb_reset_nvs_start_commissioning();

    void print_ieee_addr(const esp_zb_ieee_addr_t addr) ;
    esp_err_t nuos_zb_ota_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message);

    void nuos_read_attr_read_request(uint8_t btn_index, uint8_t ep_index, dst_node_info_t *dst_node_info);
    #ifdef __cplusplus
    extern "C" {
    #endif
    void nuos_init_rgb_led();
    void nuos_blink_rgb_led(const uint32_t delay);
    void nuos_toggle_rgb_led();
    void nuos_set_rgb_led_normal_functionality();
    void nuos_set_rgb_led_commissioning_functionality();
    void nuos_switch_single_click_task(uint32_t io_num);
    void nuos_switch_double_click_task(uint32_t io_num);
    void nuos_switch_long_press_task(uint32_t io_num); 
    void nuos_switch_long_press_brightness_task(uint32_t io_num);
    uint8_t nuos_get_endpoint_index(uint8_t dst_ep);

    void xyToRgb(uint16_t x, uint16_t y, float brightness, uint8_t *r, uint8_t *g, uint8_t *b);
    void rgbToXy(uint8_t r, uint8_t g, uint8_t b, uint16_t *x, uint16_t *y);   
    void rgbToHsv(uint8_t r, uint8_t g, uint8_t b, float *h, float *s, float *v);
    void nuos_rgb_trigger_blink();  
    #ifdef __cplusplus
    }
    #endif   

    #define TASK_MODE_BLINK_ALL_LEDS                0
    #define TASK_MODE_GO_TO_ACTUAL_TASK             1
    #define TASK_MODE_BLINK_OTHER_LEDS              2
    #define TASK_SEND_IDENTIFY_COMMAND              3
    #define TASK_BLINK_SELECTED_LED                 4
    #define TASK_SEND_BINDING_REQUEST               5
    #define TASK_SEND_UNBINDING_REQUEST             6
    #define TASK_MODE_EXIT                          7
    #define TASK_ALL_LED_ON                         8
    #define TASK_ALL_LED_OFF                        9
    #define TASK_MODE_ACTUAL_TASK_BLINK_LEDS        10

    extern int task_sequence_num;
    extern uint8_t save_pressed_button_index; 
    extern uint8_t dec_node_counts;
    extern void mode_change_task(void* args);
    extern void mode_change_color_task(void* args);
    extern bool isSceneRemoteBindingStarted;
    extern bool identify_device_complete_flag;
    extern void nuos_blink_rgb_led_init_task();
    extern void nuos_start_rgb_led_blink_task(uint32_t index);
    extern void nuos_kill_rgb_led_blink_task();
    
#endif