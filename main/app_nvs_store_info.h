
#ifndef _APP_NVS_STORE_INFO_H_
#define _APP_NVS_STORE_INFO_H_

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
    extern esp_err_t update_attr_data_in_nvs(uint16_t target_short_addr, uint8_t scene_index, uint8_t ep_index, void* new_attr_data);
    extern void prepare_array_string(int itemCount, char* Items);
    extern void prepare_selected_array_string(int itemCount, char* Items);
    extern void nuos_enable_ap_mode();
    extern void setNVSZbNwSteeringCrashCounts(uint8_t value);
    void clear_all_records_in_nvs();
    bool getNVSAllLedsOff();
    void setNVSAllLedsOff(uint8_t value);
    void nuos_init_nvs();
    void store_curtain_cal_time(uint64_t time);
    int64_t read_curtain_cal_time();
    // esp_err_t update_binding_data_in_nvs(uint16_t target_short_addr, uint8_t scene_index, uint8_t ep_index, void* new_binding_data);
    esp_err_t update_binding_data_in_nvs(uint16_t target_short_addr, uint8_t scene_index, uint8_t ep_index, uint8_t binding_val);
    void save_specific_ep_data(int scene_index, int node_index, int ep_index);
    void init_nvs_for_zb_devices();

    extern uint8_t getNVSStartCommissioningFlag();
    extern void setNVSStartCommissioningFlag(uint8_t value);
    extern void nuos_write_default_value();

    extern void setNVSPanicAttack(uint8_t value);
    extern uint8_t getNVSPanicAttack();

#endif
    