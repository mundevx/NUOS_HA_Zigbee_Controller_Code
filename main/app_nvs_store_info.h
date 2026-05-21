
#ifndef _APP_NVS_STORE_INFO_H_
#define _APP_NVS_STORE_INFO_H_
#include "cJSON.h"


    #define GROUP_COUNT             16
    #define SCENE_COUNT             16
    #define DALI_ADDR_COUNT         64
    /* Compact device->scene entry */
    typedef struct {
        uint8_t dali_id;     // 0..63
        uint8_t scene_id;    // 0..(SCENE_COUNT-1)
        uint8_t included;    // 0 or 1
        uint8_t power;       // 0 or 1
        uint8_t brightness;  // 0..100  (store 0..100)
        uint16_t cct;        // color temperature in K (2000..6500)
        uint8_t color_mode;   // 0 = cct, 1 = rgb
        uint8_t r;
        uint8_t g;
        uint8_t b;
    } device_scene_t;
    /* Global in-memory store (loaded at boot) */
    extern device_scene_t *g_device_scene_store;
    extern size_t g_device_scene_count;

    #ifdef __cplusplus
    extern "C" {
    #endif

        void nuos_store_data_to_nvs(uint8_t index);
        void nuos_read_data_from_nvs(uint8_t index);
        void store_timeout_value();
        void read_timeout_value();
        void setNVSCommissioningFlag(uint8_t value);
        uint8_t getNVSCommissioningFlag();
        uint8_t getNVSSensorsCommissioningCounts();
        void setNVSSensorsCommissioningCounts(uint8_t value);
        void nuos_get_data_from_nvs();
        void setNVSWebServerEnableFlag(uint8_t value);
        uint8_t getNVSWebServerEnableFlag();
        void nuos_store_dali_data_to_nvs(uint8_t index);

        void setNVSDaliNodesCommissioningCounts(uint8_t value);
        uint8_t getNVSDaliNodesCommissioningCounts();

        void writeUartStruct(uint8_t index, void * strt);

        void readUartStruct(uint8_t index, void* my_data, size_t* length);

        void writeDmxStruct(uint8_t index, void * strt);
        void readDmxStruct(uint8_t index, void* my_data, size_t* length);

        void store_color_mode_value(uint8_t mode);
        uint8_t read_color_mode_value();
        
        extern void nuos_store_dali_scene_switch_data_to_nvs(const void *str_data);
        
        extern void nuos_read_dali_scene_switch_data_from_nvs(void* str_data, size_t* length);

        extern esp_err_t persist_devices_from_cjson_array(cJSON *devices, int scene_id, const char *action);
        extern esp_err_t load_device_scene_store(device_scene_t **out_arr, size_t *out_count);
        extern esp_err_t save_device_scene_store(const device_scene_t *arr, size_t count);
        extern esp_err_t clear_all_groups_and_scenes_in_nvs(void);
        extern esp_err_t clear_scene_in_nvs(int group_id);
        extern esp_err_t clear_group_in_nvs(int group_id);
        extern esp_err_t write_scene_blob(int group_id, uint8_t buffer[SCENE_COUNT]);
        extern esp_err_t read_scene_blob(int group_id, uint8_t buffer[SCENE_COUNT]);
        extern esp_err_t write_group_blob(int group_id, uint8_t buffer[DALI_ADDR_COUNT]);
        extern esp_err_t read_group_blob(int group_id, uint8_t buffer[DALI_ADDR_COUNT]);
        extern void group_key_name(int group_id, char *out, size_t outlen);
        extern void scene_key_name(int group_id, char *out, size_t outlen);
        extern void nuos_toggle_tmp_selection(int dali_id, int group_id, int state);
        extern int nuos_read_scene_membership_from_nvs(int scene_index, int *out_ids, int *out_count);
        extern void nuos_store_scene_membership_to_nvs(int group_id, int *ids, int count);
    #ifdef __cplusplus
    }
    #endif

    
    extern void writeSceneInfoStruct(uint8_t index, void * strt);
    extern void readSceneInfoStruct(uint8_t index, void* my_data, size_t* length);

    extern void writeKeyValueToNVRAM(const char* key, int32_t value);
    extern int32_t readKeyValueFromNVRAM(const char* key);
    
    
    extern uint8_t getNVSSceneRecallErrorFlag();
    extern void setNVSSceneRecallErrorFlag(uint8_t value);

    extern void nuos_store_wifi_info_data_to_nvs();
    extern void nuos_read_wifi_info_data_from_nvs();
    extern void nuos_check_nvs_start_commissioning();
    extern void nuos_write_neighbour_bind_info_nvs(uint8_t index);
    extern esp_err_t nuos_read_neighbour_bind_info_nvs(uint8_t index);

    extern size_t store_new_nodes(void* new_node, uint8_t index);
    extern size_t load_nodes_info_from_nvs(uint8_t index) ;
    extern void save_nodes_info_to_nvs(uint8_t index);
    // extern esp_err_t update_attr_data_in_nvs(uint16_t target_short_addr, uint8_t scene_index, uint8_t ep_index, void* new_attr_data);
    extern void prepare_array_string(int itemCount, char* Items);
    extern void prepare_selected_array_string(int itemCount, char* Items);
    extern void nuos_enable_ap_mode();
    extern void setNVSZbNwSteeringCrashCounts(uint8_t value);
    void clear_all_records_in_nvs();
    bool getNVSAllLedsOff();
    void setNVSAllLedsOff(uint8_t value);
    void nuos_init_nvs();
    // void store_curtain_cal_time(uint32_t time);
    // uint32_t read_curtain_cal_time();
    // esp_err_t update_binding_data_in_nvs(uint16_t target_short_addr, uint8_t scene_index, uint8_t ep_index, void* new_binding_data);
    esp_err_t update_binding_data_in_nvs(uint16_t target_short_addr, uint8_t scene_index, uint8_t ep_index, uint8_t binding_val);
    // void save_specific_ep_data(int scene_index, int node_index, int ep_index);
    void init_nvs_for_zb_devices();

    extern uint8_t getNVSStartCommissioningFlag();
    extern void setNVSStartCommissioningFlag(uint8_t value);
    extern void nuos_write_default_value();

    extern void setNVSPanicAttack(uint8_t value);
    extern uint8_t getNVSPanicAttack();

    extern void nuos_store_group_membership_to_nvs(int group_id, int *ids, int count);

    extern int nuos_read_selected_dali_address_from_nvs(void);

    void setNVSDaliNodesStartAddrCounts(uint8_t value);
    uint8_t getNVSDaliNodesStartAddrCounts();

    void read_motion_disable_timeout_value();
    void store_motion_disable_timeout_value(int32_t val);

    
    void read_motion_auto_enable_value();
    void store_motion_auto_enable_value(int32_t val);

    extern uint8_t getNVSDaliMinDimOffset();
    extern void setNVSDaliMinDimOffset(uint8_t value);

    extern uint8_t getNVSDaliFadeTime();
    extern void setNVSDaliFadeTime(uint8_t value);
    extern uint8_t getNVSDaliFadeRate();
    extern void setNVSDaliFadeRate(uint8_t value);

#endif
    