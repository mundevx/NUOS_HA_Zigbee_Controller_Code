
#include "nvs_flash.h"

#include "esp_system.h"
#include "esp_zigbee_core.h"
// #include "app_nuos_timer.h"
// #include "esp_wifi_station.h"
#include "app_nvs_store_info.h"
#include <stdio.h>
#include <string.h>
#include "esp_check.h"
#include "esp_log.h"
#include "esp_err.h"
#ifdef USE_RGB_LED
#include "light_driver.h"
#endif
#include "app_hardware_driver.h"

static const char *TAG = "ESP_ZB_NVS_STORE";
static const char *TAG_NVS = "TAG_NVS"; 

extern wifi_info_handle_t wifi_info;

extern volatile bool pir_motion_disable_flag;
extern int32_t pir_motion_disable_timeout_counts;
extern int32_t pir_motion_disable_timeout_value;
extern int32_t motion_auto_enable_flag;
extern uint8_t last_cmd_id;
nvs_handle_t my_handle;
const char* nvram_struct_device_overall_keys = "ep_overall";

const char* nvram_struct_scene_sw_key = "scene_sw";
// const char* nvram_struct_keys[5] = {"ep1", "ep2", "ep3", "ep4", "all"};
// const char* nvram_struct_dali_keys[4] = {"dali1", "dali2", "dali3", "dali4"};


const char* nvram_struct_keys[32] = {"ep1", "ep2", "ep3", "ep4", "ep5", "ep6", "ep7", "ep8", "ep9", "ep10",
                                    "ep11", "ep12", "ep13", "ep14", "ep15", "ep16", "ep17", "ep18", "ep19", "ep20",
                                    "ep21", "ep22", "ep23", "ep24", "ep25", "ep26", "ep27", "ep28", "ep29", "ep30",
                                    "ep31", "ep32" };

const char* nvram_struct_dali_keys[32] = {"dali1", "dali2", "dali3", "dali4", "dali5", "dali6", "dali7", "dali8", "dali9", "dali10", 
                                        "dali11", "dali12", "dali13", "dali14", "dali15", "dali16", "dali17", "dali18", "dali19", "dali20", 
                                        "dali21", "dali22", "dali23", "dali24", "dali25", "dali26", "dali27", "dali28", "dali29", "dali30", 
                                        "dali31", "dali32" };


const char* nvram_struct_scene_keys[4] = {"scn1", "scn2", "scn3", "scn4"};
// const char* nvram_struct_bind_info_keys[4] = {"zb_bind1", "zb_bind2", "zb_bind3", "zb_bind4"};

const char* nvram_struct_uart_keys = "stt_uart";
const char* nvram_mode_keys = "mode"; 

const char* nvram_cal_keys = "curtain_cal";
const char* nvram_commissioning_keys = "com_key";
const char* nvram_start_commissioning_keys = "s_com_key";
const char* nvram_webserver_keys = "webserver_key";
const char* nvram_zb_steering_keys = "zb_init_key";
const char* nvram_sensor_commissioning_keys = "sen_com_key";
const char* nvram_wifi_info_keys = "wifi_info";
const char* nvram_struct_existing_info_key = "e_info_key";
const char* nvram_dali_commissioning_key = "dali_commiss";
const char* nvram_dali_start_addr_key = "dali_start";
const char* nvram_all_leds_off_keys = "leds_off_key";
const char* nvram_ip_addr_keys = "ip_key";
const char* nvram_scene_err_keys = "err_key";
const char* nvram_dali_off_keys = "dali_off_key";
const char* nvram_dali_ft_keys = "dali_ft_key";
const char* nvram_dali_fr_keys = "dali_fr_key";
const char* nvram_nvs_panic_attack = "panic";

#define MAX_DALI_FADE_TIME                  10
#define DEFAULT_DALI_FADE_TIME              1

#define MAX_DALI_FADE_RATE                  254
#define DEFAULT_DALI_FADE_RATE              12

void writeEpStruct(uint8_t index, void * strt);
void readEpStruct(uint8_t index, void* my_data, size_t* length);

#ifdef ZB_COMMISSIONING_WITHOUT_TIMER
extern void nuos_start_mode_change_task();
#endif

#ifdef __cplusplus
    extern "C" {
#endif
extern void init_arduino_eeprom(uint8_t log_index, uint16_t restart_reason);
#ifdef __cplusplus
    }
#endif

#define ZIGBEE_NVS_PARTITION "storage"
void get_nvs_dali_scene_switch_webpage_data();
esp_err_t clear_all_groups_and_scenes_in_nvs(void);
void nuos_read_dali_scene_switch_data_to_nvs(void* str_data, size_t* length);


void nuos_init_nvs(){
    esp_err_t err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return; 
}
void writeKeyValueToNVRAM(const char* key, int32_t value) {
    #ifdef USE_NVS_INIT
    esp_err_t err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return;
    #endif
    ESP_ERROR_CHECK(nvs_set_i32(my_handle, key, value));
    ESP_ERROR_CHECK(nvs_commit(my_handle));
    #ifdef USE_NVS_INIT
    nvs_close(my_handle);
    #endif
}

int32_t readKeyValueFromNVRAM(const char* key){
	int32_t my_data = 0;
    esp_err_t err;
    #ifdef USE_NVS_INIT
    err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;
    #endif
    err = nvs_get_i32(my_handle, key, &my_data);
    if (err == ESP_OK) {

    }
    #ifdef USE_NVS_INIT
	nvs_close(my_handle);
    #endif
	return my_data;
}


uint8_t getNVSSceneRecallErrorFlag(){
	return (uint8_t)readKeyValueFromNVRAM(nvram_scene_err_keys);
}
void setNVSSceneRecallErrorFlag(uint8_t value){
	writeKeyValueToNVRAM(nvram_scene_err_keys, value);
}

uint8_t getNVSDaliMinDimOffset(){
	return (uint8_t)readKeyValueFromNVRAM(nvram_dali_off_keys);
}
void setNVSDaliMinDimOffset(uint8_t value){
	writeKeyValueToNVRAM(nvram_dali_off_keys, value);
}

uint8_t getNVSDaliFadeTime(){
	return (uint8_t)readKeyValueFromNVRAM(nvram_dali_ft_keys);
}
void setNVSDaliFadeTime(uint8_t value){
	writeKeyValueToNVRAM(nvram_dali_ft_keys, value);
}

uint8_t getNVSDaliFadeRate(){
	return (uint8_t)readKeyValueFromNVRAM(nvram_dali_fr_keys);
}
void setNVSDaliFadeRate(uint8_t value){
	writeKeyValueToNVRAM(nvram_dali_fr_keys, value);
}

uint8_t getNVSCommissioningFlag(){
	return (uint8_t)readKeyValueFromNVRAM(nvram_commissioning_keys);
}
void setNVSCommissioningFlag(uint8_t value){
	writeKeyValueToNVRAM(nvram_commissioning_keys, value);
}

uint8_t getNVSStartCommissioningFlag(){
	return (uint8_t)readKeyValueFromNVRAM(nvram_start_commissioning_keys);
}
void setNVSStartCommissioningFlag(uint8_t value){
    
	writeKeyValueToNVRAM(nvram_start_commissioning_keys, value);
}

void setNVSSensorsCommissioningCounts(uint8_t value){
	writeKeyValueToNVRAM(nvram_sensor_commissioning_keys, value);
	if(value == 5){
        esp_zb_nvram_erase_at_start(true);
        setNVSCommissioningFlag(1);
        for(int i=0; i<50; i++) {
            esp_err_t status = esp_zb_zcl_scenes_table_clear_by_index(i);
            if(status != ESP_OK) break;
        }

        esp_zb_factory_reset();
	} 
}
uint8_t getNVSSensorsCommissioningCounts(){
	uint8_t comm_counts = (uint8_t)readKeyValueFromNVRAM(nvram_sensor_commissioning_keys);
    printf("comm_counts:%d\n", comm_counts);
	if(comm_counts >= 4){
        setNVSSensorsCommissioningCounts(0);
		setNVSCommissioningFlag(1); 
	}
	return comm_counts;
}


void setNVSZbNwSteeringCrashCounts(uint8_t value){
    writeKeyValueToNVRAM(nvram_zb_steering_keys, value);
}
uint8_t getNVSZbNwSteeringCrashCounts(){
	return (uint8_t)readKeyValueFromNVRAM(nvram_zb_steering_keys);
}


void setNVSWebServerEnableFlag(uint8_t value){
    writeKeyValueToNVRAM(nvram_webserver_keys, value);
}
uint8_t getNVSWebServerEnableFlag(){
    #ifdef USE_WIFI_WEBSERVER
	return (uint8_t)readKeyValueFromNVRAM(nvram_webserver_keys);
    #else
    return 0;
    #endif
}


void setNVSPanicAttack(uint8_t value){
    writeKeyValueToNVRAM(nvram_nvs_panic_attack, value);
}
uint8_t getNVSPanicAttack(){
	return (uint8_t)readKeyValueFromNVRAM(nvram_nvs_panic_attack);
}

#ifdef USE_WIFI_WEBSERVER
    void writeEpWifiStruct(wifi_info_handle_t *strt){
        #ifdef USE_NVS_INIT
        ESP_ERROR_CHECK(nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle));
        #endif
        nvs_set_blob(my_handle, nvram_wifi_info_keys, strt, sizeof(wifi_info_handle_t));
        ESP_ERROR_CHECK(nvs_commit(my_handle));
        #ifdef USE_NVS_INIT
        nvs_close(my_handle);
        #endif
    }

    void readEpWifiStruct(wifi_info_handle_t* my_data, size_t* length){
        #ifdef USE_NVS_INIT
        ESP_ERROR_CHECK(nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle));
        #endif
        esp_err_t err = nvs_get_blob(my_handle, nvram_wifi_info_keys, my_data, length);
        #ifdef USE_NVS_INIT
        nvs_close(my_handle);
        #endif
    }

    void nuos_store_wifi_info_data_to_nvs(){
        writeEpWifiStruct(&wifi_info);
    }

    void nuos_read_wifi_info_data_from_nvs(){
        // Clear the struct
        memset(&wifi_info, 0, sizeof(wifi_info_handle_t));
        size_t required_size = sizeof(wifi_info_handle_t);
        readEpWifiStruct(&wifi_info, &required_size);

        printf("----------Stored Wifi Info-------------\n");
        printf("wifi_ssid:%s \n", wifi_info.wifi_ssid);
        printf("wifi_pass:%s \n", wifi_info.wifi_pass);
        printf("wifi_info:%d \n", wifi_info.ip4);
        printf("---------------------------------------\n");
    }
#endif

void store_motion_auto_enable_value(int32_t val) {
    // Open
   esp_err_t err;
   #ifdef USE_NVS_INIT
   err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
   if (err == ESP_OK) {
    #endif
       // Write
       motion_auto_enable_flag = val;
       err = nvs_set_i32(my_handle, "enable_auto", motion_auto_enable_flag);
       // Commit
       err = nvs_commit(my_handle);
    #ifdef USE_NVS_INIT
       // Close
       nvs_close(my_handle);
   }
   #endif
}

void read_motion_auto_enable_value() {
    // Open
	esp_err_t err;
    #ifdef USE_NVS_INIT
    err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
	if (err == ESP_OK) {
    #endif    
		err = nvs_get_i32(my_handle, "enable_auto", &motion_auto_enable_flag);
    #ifdef USE_NVS_INIT
		// Close
		nvs_close(my_handle);
	}
    #endif
}

void store_motion_disable_timeout_value(int32_t val) {
    // Open
   esp_err_t err;
   #ifdef USE_NVS_INIT
   err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
   if (err == ESP_OK) {
    #endif
       // Write
       pir_motion_disable_timeout_value = val;
       err = nvs_set_i32(my_handle, "disable_timeout", pir_motion_disable_timeout_value);
       // Commit
       err = nvs_commit(my_handle);
    #ifdef USE_NVS_INIT
       // Close
       nvs_close(my_handle);
   }
   #endif
}

void read_motion_disable_timeout_value() {
    // Open
	esp_err_t err;
    #ifdef USE_NVS_INIT
    err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
	if (err == ESP_OK) {
    #endif    
		err = nvs_get_i32(my_handle, "disable_timeout", &pir_motion_disable_timeout_value);
    #ifdef USE_NVS_INIT
		// Close
		nvs_close(my_handle);
	}
    #endif
}

void store_timeout_value() {
    // Open
//    esp_err_t err = nvs_open_from_partition("nvs", ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
   esp_err_t err;
   #ifdef USE_NVS_INIT
   err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
   if (err == ESP_OK) {
    #endif
       // Write
       err = nvs_set_i32(my_handle, "timeout", pir_motion_disable_timeout_counts);
       // Commit
       err = nvs_commit(my_handle);
    #ifdef USE_NVS_INIT
       // Close
       nvs_close(my_handle);
   }
   #endif
}

void read_timeout_value() {
    // Open
	esp_err_t err;
    #ifdef USE_NVS_INIT
    err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
	if (err == ESP_OK) {
    #endif    
		err = nvs_get_i32(my_handle, "timeout", &pir_motion_disable_timeout_counts);
    #ifdef USE_NVS_INIT
		// Close
		nvs_close(my_handle);
	}
    #endif
    
	if(pir_motion_disable_timeout_counts >= pir_motion_disable_timeout_value){
		pir_motion_disable_timeout_counts = 0;
	}
}

void nuos_store_data_to_nvs(uint8_t index){
	writeEpStruct(index, (led_indicator_handle_t*)&device_info[index]);
}


void store_color_mode_value(uint8_t mode) {
    // Open
   esp_err_t err;
   #ifdef USE_NVS_INIT
   err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
   if (err == ESP_OK) {
    #endif
       // Write
       err = nvs_set_i32(my_handle, nvram_mode_keys, mode);
       // Commit
       err = nvs_commit(my_handle);
       #ifdef USE_NVS_INIT
       // Close
       nvs_close(my_handle);
   }
   #endif
}


uint8_t read_color_mode_value() {
    uint8_t mode = 1;
    // Open
	esp_err_t err;
    #ifdef USE_NVS_INIT
    err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
	if (err == ESP_OK) {
        #endif
		err = ESP_OK;//nvs_get_i32(my_handle, nvram_mode_keys, &mode);
        #ifdef USE_NVS_INIT
		// Close
		nvs_close(my_handle);
	}
    #endif
    return mode;
}



// void store_curtain_cal_time(uint32_t time) {
//     // Open
//    esp_err_t err;
//    #ifdef USE_NVS_INIT
//    err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
//    if (err == ESP_OK) {
//     #endif
    
//        // Write
//        err = nvs_set_i32(my_handle, nvram_cal_keys, time);
//        // Commit
//        err = nvs_commit(my_handle);
//        #ifdef USE_NVS_INIT
//        // Close
//        nvs_close(my_handle);
//    }
//    #endif
// }

// int32_t read_curtain_cal_time() {
//     int32_t time = 0;
//     // Open
// 	esp_err_t err;
//     #ifdef USE_NVS_INIT
//     err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
// 	if (err == ESP_OK) {
//         #endif
// 		err = nvs_get_i32(my_handle, nvram_cal_keys, &time);
//         #ifdef USE_NVS_INIT
// 		// Close
// 		nvs_close(my_handle);
// 	}
//     #endif
//     return time;
// }

bool getNVSAllLedsOff(){
	return (bool)readKeyValueFromNVRAM(nvram_all_leds_off_keys);
}
void setNVSAllLedsOff(uint8_t value){
	writeKeyValueToNVRAM(nvram_all_leds_off_keys, value);
}

void nuos_write_default_value(){
	for(int index=0; index<TOTAL_ENDPOINTS; index++){
        device_info[index].device_state = 1;
        device_info[index].device_level = MAX_DIM_LEVEL_VALUE;
        #ifndef USE_INDIVIDUAL_DALI_ADDRESSING
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN)
        
        #else
		device_info[index].light_color_x = 0x2300;
		device_info[index].light_color_y = 0x9100;
        #endif
        device_info[index].dim_up = 1;
        #endif
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN_CUSTOM)
            device_info[index].device_level = 128;
            device_info[index].fan_speed = 2; //medium
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER)      
            device_info[index].ac_temperature = 23;
            device_info[index].ac_decode_type = 16; //15-coolix 
            device_info[index].ac_mode = 3; //cool
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI)  
            if(index == 3){
                device_info[index].device_state = 1;
                device_info[index].device_level = MAX_DIM_LEVEL_VALUE;
                device_info[index].device_val = 2000;
            }                  
        #else
            device_info[index].device_level = MAX_DIM_LEVEL_VALUE;            
        #endif
		writeEpStruct(index, (led_indicator_handle_t*)&device_info[index]);
	}	
}
void write_nvs_configuration(){
	setNVSCommissioningFlag(1);
    ready_commisioning_flag = true;

    store_motion_disable_timeout_value(60);

    store_motion_auto_enable_value(1);

    setNVSStartCommissioningFlag(1);
	setNVSSensorsCommissioningCounts(0);

    setNVSDaliMinDimOffset(100);

    printf("write_nvs_configuration\n");
    setNVSWebServerEnableFlag(1);
    printf("setNVSWebServerEnableFlag\n");
    setNVSAllLedsOff(0);
    printf("setNVSAllLedsOff\n");
    store_color_mode_value(0);
    printf("store_color_mode_value\n");
    uint8_t nvs_store_max = TOTAL_ENDPOINTS;
    #ifdef USE_ZB_ONLY_FAN
        nvs_store_max = TOTAL_ENDPOINTS+1;
    #endif
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
        nvs_store_max = TOTAL_BUTTONS;
    #endif
    printf("START\n");
	for(int index=0; index<nvs_store_max; index++){
        printf("index:%d ", index);
        device_info[index].device_state = 1;
        device_info[index].device_level = MAX_DIM_LEVEL_VALUE;
        #ifndef USE_INDIVIDUAL_DALI_ADDRESSING
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN)
        #else
		device_info[index].light_color_x = 0;
		device_info[index].light_color_y = 0;
        #endif
        device_info[index].dim_up = 1;
        #endif
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1_LIGHT_1_FAN_CUSTOM)
            device_info[index].device_level = 128;
            device_info[index].fan_speed = 2; //medium
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER)      
            device_info[index].ac_temperature = 23;
            device_info[index].ac_decode_type = 16; //15-coolix 
            device_info[index].ac_mode = 3; //cool
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI)  
            if(index == 3){
                device_info[index].device_state = 1;
                device_info[index].device_level = MAX_DIM_LEVEL_VALUE;
                device_info[index].device_val = 2000;
            }                  
        #else
            device_info[index].device_level = MAX_DIM_LEVEL_VALUE;            
        #endif
		writeEpStruct(index, (led_indicator_handle_t*)&device_info[index]);
	}	
	
	#ifdef USE_WIFI_WEBSERVER
        nuos_read_wifi_info_data_from_nvs();
        printf("wifi_ssid:%s \n", wifi_info.wifi_ssid);
        printf("wifi_pass:%s \n", wifi_info.wifi_pass);
        printf("wifi_info:%d \n", wifi_info.ip4);
        if(strlen(wifi_info.wifi_ssid) == 0){
            strcpy(wifi_info.wifi_ssid, "NUOS HOME Automation");
            strcpy(wifi_info.wifi_pass, "NUOS@FCSA");
            wifi_info.ip4 = 119;
            wifi_info.is_wifi_sta_mode = 0;
            writeEpWifiStruct(&wifi_info);	
        }
		#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)

        #else
        wifi_info.is_wifi_sta_mode = 0;
        #endif
		strcpy(wifi_info.wifi_ssid, "NUOS HOME Automation");
		strcpy(wifi_info.wifi_pass, "NUOS@FCSA");
        wifi_info.ip4 = 119;
		writeEpWifiStruct(&wifi_info);	
	#endif
}

void nuos_enable_ap_mode(){
	wifi_info.is_wifi_sta_mode = 0;
    nuos_store_wifi_info_data_to_nvs();
}

void nuos_check_nvs_start_commissioning(){
	#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_MOTION) 
		getNVSSensorsCommissioningCounts();
    #endif
}


void writeEpStruct(uint8_t index, void * strt){
	//ESP_ERROR_CHECK(nvs_open_from_partition("nvs", ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle));
    esp_err_t err;
    #ifdef USE_NVS_INIT
    err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return;
    #endif
	nvs_set_blob(my_handle, nvram_struct_keys[index], (led_indicator_handle_t*)strt, sizeof(led_indicator_handle_t));
    ESP_ERROR_CHECK(nvs_commit(my_handle));
    #ifdef USE_NVS_INIT
    nvs_close(my_handle);
    #endif
}

void readEpStruct(uint8_t index, void* my_data, size_t* length){
	//ESP_ERROR_CHECK(nvs_open_from_partition("nvs", ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle));
    esp_err_t err;
    #ifdef USE_NVS_INIT
    err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return;
    #endif
	err = nvs_get_blob(my_handle, nvram_struct_keys[index], my_data, length);
    #ifdef USE_NVS_INIT
	nvs_close(my_handle);
    #endif
}

void writeDaliStruct(uint8_t index, void * strt){
	// ESP_ERROR_CHECK(nvs_open_from_partition("nvs", ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle));
    esp_err_t err;
    #ifdef USE_NVS_INIT
    err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return;
    #endif
	nvs_set_blob(my_handle, nvram_struct_dali_keys[index], (dali_device_ids_t*)strt, sizeof(dali_device_ids_t));
    ESP_ERROR_CHECK(nvs_commit(my_handle));
    #ifdef USE_NVS_INIT
    nvs_close(my_handle);
    #endif
}

void readDaliStruct(uint8_t index, void* my_data, size_t* length){
	// ESP_ERROR_CHECK(nvs_open_from_partition("nvs", ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle));
    esp_err_t err;
    #ifdef USE_NVS_INIT
    err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return;
    #endif
	err = nvs_get_blob(my_handle, nvram_struct_dali_keys[index], my_data, length);
    #ifdef USE_NVS_INIT
	nvs_close(my_handle);
    #endif
}



void writeSceneInfoStruct(uint8_t index, void * strt){
	// ESP_ERROR_CHECK(nvs_open_from_partition("nvs", ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle));
    esp_err_t err;
    #ifdef USE_NVS_INIT
    err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return;
    #endif
	nvs_set_blob(my_handle, nvram_struct_scene_keys[index], (zigbee_zcene_info_t*)strt, sizeof(zigbee_zcene_info_t));
    ESP_ERROR_CHECK(nvs_commit(my_handle));
    #ifdef USE_NVS_INIT
    nvs_close(my_handle);
    #endif
}

void readSceneInfoStruct(uint8_t index, void* my_data, size_t* length){
	// ESP_ERROR_CHECK(nvs_open_from_partition("nvs", ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle));
    esp_err_t err;
    #ifdef USE_NVS_INIT
    err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return;
    #endif
	err = nvs_get_blob(my_handle, nvram_struct_scene_keys[index], my_data, length);
    #ifdef USE_NVS_INIT
	nvs_close(my_handle);
    #endif
}




void writeUartStruct(uint8_t index, void * strt){
	// ESP_ERROR_CHECK(nvs_open_from_partition("nvs", ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle));
    esp_err_t err;
    #ifdef USE_NVS_INIT
    err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return;
    #endif
	nvs_set_blob(my_handle, nvram_struct_uart_keys, (uart_config_t*)strt, sizeof(uart_config_t));
    ESP_ERROR_CHECK(nvs_commit(my_handle));
    #ifdef USE_NVS_INIT
    nvs_close(my_handle);
    #endif
}

void readUartStruct(uint8_t index, void* my_data, size_t* length){
	// ESP_ERROR_CHECK(nvs_open_from_partition("nvs", ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle));
    esp_err_t err = ESP_OK;
    #ifdef USE_NVS_INIT
    err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return;
    #endif
    
	err = nvs_get_blob(my_handle, nvram_struct_uart_keys, my_data, length);
    #ifdef USE_NVS_INIT
	nvs_close(my_handle);
    #endif
}


void setNVSDaliNodesCommissioningCounts(uint8_t value){
	writeKeyValueToNVRAM(nvram_dali_commissioning_key, value);
}
uint8_t getNVSDaliNodesCommissioningCounts(){
	uint8_t comm_counts = (uint8_t)readKeyValueFromNVRAM(nvram_dali_commissioning_key);
    printf("comm_counts:%d", comm_counts);
	return comm_counts;
}


void setNVSDaliNodesStartAddrCounts(uint8_t value){
	writeKeyValueToNVRAM(nvram_dali_start_addr_key, value);
}
uint8_t getNVSDaliNodesStartAddrCounts(){
	uint8_t comm_counts = (uint8_t)readKeyValueFromNVRAM(nvram_dali_start_addr_key);
    printf("comm_counts:%d", comm_counts);
	return comm_counts;
}


void nuos_store_dali_data_to_nvs(uint8_t index){
    writeDaliStruct(index, (dali_device_ids_t*)&dali_nvs_stt[index]);
}
/////////////////////////////////////////////////////////////////////////////////////////////


#if (USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN)
#define NVS_KEY_OFFSET "offset_time"
#define NVS_KEY_CALIBRATION "calibration_time"


#endif

void save_nodes_info_to_nvs(uint8_t index) {
    esp_err_t err;
    #ifdef USE_NVS_INIT
    // Open NVS handle
    err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return;
    }
    #endif
    // Loop through each struct in the array and write individually
    // for (size_t i = 0; i < count; i++) {
    char key[16];  // Key for each struct in NVS
    snprintf(key, sizeof(key), "scene_%d", index);  // Generate unique key

    size_t size = sizeof(stt_scene_switch_t);
    err = nvs_set_blob(my_handle, key, &existing_nodes_info[index], size);
    if (err != ESP_OK) {
        printf("Error writing scene switch %zu to NVS: %s\n", index, esp_err_to_name(err));
    } else {
        printf("Scene switch %zu written to NVS successfully.\n", index);
    }
    // }

    // Commit all changes
    err = nvs_commit(my_handle);
    if (err != ESP_OK) {
        printf("Error committing changes to NVS: %s\n", esp_err_to_name(err));
    }
    #ifdef USE_NVS_INIT
    // Close NVS handle
    nvs_close(my_handle);
    #endif
}

void clear_all_records_in_nvs(){
    memset(existing_nodes_info, 0, sizeof(existing_nodes_info));
    memset(&nodes_info, 0, sizeof(nodes_info));
    existing_nodes_info[0].scene_switch_info.total_records = 0;
    existing_nodes_info[1].scene_switch_info.total_records = 0; 
    existing_nodes_info[2].scene_switch_info.total_records = 0; 
    existing_nodes_info[3].scene_switch_info.total_records = 0;      
    for(int i=0; i<TOTAL_ENDPOINTS; i++){
        save_nodes_info_to_nvs(i);
    }
    node_counts = 0;
   
}

size_t load_nodes_info_from_nvs(uint8_t index) {
    // Validate index bounds first
    if (index >= TOTAL_ENDPOINTS) { // Replace MAX_SCENES with your actual array size
        printf("Invalid index: %u\n", index);
        return 0;
    }

    esp_err_t err;
    #ifdef USE_NVS_INIT
    err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READONLY, &my_handle);
    if (err != ESP_OK) {
        printf("Error opening NVS: %s\n", esp_err_to_name(err));
        return 0;
    }
    #endif

    char key[16];
    snprintf(key, sizeof(key), "scene_%d", index);

    // Initialize the target struct to avoid garbage values
    memset(&existing_nodes_info[index], 0, sizeof(stt_scene_switch_t));

    size_t required_size = sizeof(stt_scene_switch_t);
    err = nvs_get_blob(my_handle, key, &existing_nodes_info[index], &required_size);
    if (required_size != sizeof(stt_scene_switch_t)) {
        printf("Size mismatch! Expected %u, got %u\n", sizeof(stt_scene_switch_t), required_size);
        return 0;
    }
    if (err == ESP_ERR_NVS_INVALID_LENGTH) {
        printf("Stored data for scene %u is larger than expected!\n", index);
        #ifdef USE_NVS_INIT
        nvs_close(my_handle);
        #endif
        return 0;
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        printf("Scene %u not found in NVS.\n", index);
        #ifdef USE_NVS_INIT
        nvs_close(my_handle);
        #endif
        return 0;
    } else if (err != ESP_OK) {
        printf("Error reading scene %u: %s\n", index, esp_err_to_name(err));
        #ifdef USE_NVS_INIT
        nvs_close(my_handle);
        #endif
        return 0;
    }

    #ifdef USE_NVS_INIT
    nvs_close(my_handle);
    #endif

    // Validate the loaded data
    if (existing_nodes_info[index].scene_switch_info.total_records > MAX_NODES) {
        printf("Corrupted data: total_records exceeds MAX_NODES!\n");
        return 0;
    }

    printf("Loaded scene %u with %u records.\n", index, existing_nodes_info[index].scene_switch_info.total_records);
    return existing_nodes_info[index].scene_switch_info.total_records;
}


size_t store_new_nodes(void* n_node, uint8_t index) {
    esp_err_t err;
    stt_scene_switch_t *new_node = (stt_scene_switch_t *)n_node;

    #ifdef USE_NVS_INIT
    err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error opening NVS: %s\n", esp_err_to_name(err));
        return 0;
    }
    #endif

    for (int src_ep_index = 0; src_ep_index < TOTAL_ENDPOINTS; src_ep_index++) {
        // Read existing nodes for current src_ep_index
        char key[16];
        snprintf(key, sizeof(key), "scene_%d", src_ep_index);
        size_t required_size = sizeof(stt_scene_switch_t);
        err = nvs_get_blob(my_handle, key, &existing_nodes_info[src_ep_index], &required_size);
        if (err != ESP_OK) {
            memset(&existing_nodes_info[src_ep_index], 0, sizeof(stt_scene_switch_t)); // Initialize if not found
        }

        new_node->scene_id = global_scene_id[src_ep_index];
        new_node->scene_switch_info.src_ep = ENDPOINTS_LIST[src_ep_index];
        existing_nodes_info[src_ep_index].scene_switch_info.total_records = 0;//new_node->scene_switch_info.total_records;
        printf("---->src_ep_index:%d\n", src_ep_index);
        
        for (size_t j = 0; j < new_node->scene_switch_info.total_records; j++) {
            uint16_t current_short_addr = new_node->scene_switch_info.dst_node_info[j].short_addr;
            //if (current_short_addr == 0) { printf("current_short_addr:%d\n", current_short_addr); continue;}
            if (current_short_addr != 0) {
                bool found = false;
                // Check existing records
                for (size_t l = 0; l < existing_nodes_info[src_ep_index].scene_switch_info.total_records; l++) {
                    if (existing_nodes_info[src_ep_index].scene_switch_info.dst_node_info[l].short_addr == current_short_addr) {
                        // Update existing entry
                        memcpy(&existing_nodes_info[src_ep_index].scene_switch_info.dst_node_info[l],
                            &new_node->scene_switch_info.dst_node_info[j],
                            sizeof(dst_node_info_t));
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    if (existing_nodes_info[src_ep_index].scene_switch_info.total_records >= MAX_NODES) {
                        printf("No space for new entry.\n");
                        // continue;
                    }else{
                        // Add new entry
                        size_t l = existing_nodes_info[src_ep_index].scene_switch_info.total_records;
                        memcpy(&existing_nodes_info[src_ep_index].scene_switch_info.dst_node_info[l],
                            &new_node->scene_switch_info.dst_node_info[j],
                            sizeof(dst_node_info_t));
                        existing_nodes_info[src_ep_index].scene_switch_info.total_records++;
                    }

                }                
            }

            taskYIELD(); // Yield to prevent watchdog trigger
        }

        // Write updated data back to NVS after processing all entries
        size_t size = sizeof(stt_scene_switch_t);
        err = nvs_set_blob(my_handle, key, &existing_nodes_info[src_ep_index], size);
        if (err != ESP_OK) {
            printf("NVS write failed: %s\n", esp_err_to_name(err));
        }
        taskYIELD();
    }

    err = nvs_commit(my_handle);
    #ifdef USE_NVS_INIT
    nvs_close(my_handle);
    #endif
    printf("---->NVS updated successfully!!\n");
    return existing_nodes_info[index].scene_switch_info.total_records;
}


// esp_err_t update_attr_data_in_nvs(uint16_t target_short_addr, uint8_t scene_index, uint8_t ep_index, void* new_attr_data) {
//     esp_err_t err;

//     #ifdef USE_NVS_INIT
//     // Open NVS in read-write mode
//     err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
//     if (err != ESP_OK) {
//         printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
//         return err;
//     }
//     #endif
//     // Prepare key for accessing the specific scene
//     char key[16];
//     snprintf(key, sizeof(key), "scene_%d", scene_index);
//     // Search for the target node by short_addr in dst_node_info
//     bool found = false;
//     for (size_t i = 0; i < existing_nodes_info[scene_index].scene_switch_info.total_records; i++) {
//         if (existing_nodes_info[scene_index].scene_switch_info.dst_node_info[i].short_addr == target_short_addr) {
//             found = true;
//             // Update the attr_data_info_t in the specified endpoint (ep_index)
//             if (ep_index < MAX_DST_EP) {
//                 if(new_attr_data != NULL)
//                 memcpy(&existing_nodes_info[scene_index].scene_switch_info.dst_node_info[i].dst_ep_info.ep_data[ep_index].data, (attr_data_info_t*)new_attr_data, sizeof(attr_data_info_t));
                
//                 printf("**************UPDATE ATTRIBUTES IN NVS****************\n");
//                 printf("short_addr:0x%x, ep_index:%d\n", target_short_addr, ep_index);

//                 printf("STATE:%d, LEVEL:%d\n", 
//                 existing_nodes_info[scene_index].scene_switch_info.dst_node_info[i].dst_ep_info.ep_data[ep_index].data.state, 
//                 existing_nodes_info[scene_index].scene_switch_info.dst_node_info[i].dst_ep_info.ep_data[ep_index].data.level);
//                 printf("MODE:%d, COLOR:%ld\n", 
//                     existing_nodes_info[scene_index].scene_switch_info.dst_node_info[i].dst_ep_info.ep_data[ep_index].data.mode, 
//                     existing_nodes_info[scene_index].scene_switch_info.dst_node_info[i].dst_ep_info.ep_data[ep_index].data.value);                
//                 printf("******************************************************\n");
//             } else {
//                 printf("Invalid ep_index!\n");
//                 #ifdef USE_NVS_INIT
//                 nvs_close(my_handle);
//                 #endif
//                 return ESP_ERR_INVALID_ARG;
//             }
//             break;
//         }
//     }

//     if (!found) {
//         printf("Node with short_addr %d not found.\n", target_short_addr);
//         #ifdef USE_NVS_INIT
//         nvs_close(my_handle);
//         #endif
//         return ESP_ERR_NOT_FOUND;
//     }

//     // Write the updated scene_switch back to NVS
//     err = nvs_set_blob(my_handle, key, &existing_nodes_info[scene_index], sizeof(stt_scene_switch_t));
//     if (err != ESP_OK) {
//         printf("Error updating scene switch in NVS: %s\n", esp_err_to_name(err));
//         #ifdef USE_NVS_INIT
//         nvs_close(my_handle);
//         #endif
//         return err;
//     }

//     // Commit the changes
//     err = nvs_commit(my_handle);
//     if (err != ESP_OK) {
//         printf("Error committing changes to NVS: %s\n", esp_err_to_name(err));
//     }
//     #ifdef USE_NVS_INIT
//     // Close NVS handle
//     nvs_close(my_handle);
//     #endif
//     return ESP_OK;
// }


// void save_specific_ep_data(int scene_index, int node_index, int ep_index)
// {

//     dst_ep_data_t *ep_data = &existing_nodes_info[scene_index]
//                                  .scene_switch_info
//                                  .dst_node_info[node_index]
//                                  .dst_ep_info
//                                  .ep_data[ep_index];
//     // Unique key per ep_data
//     // Prepare key for accessing the specific scene
//     char key[16];
//     snprintf(key, sizeof(key), "scene_%d", scene_index);

//     esp_err_t err = nvs_set_blob(my_handle, key, ep_data, sizeof(dst_ep_data_t));
//     if (err == ESP_OK) {
//         nvs_commit(my_handle);
//         printf("Saved ep_data[%d] of node[%d] in scene[%d]\n", ep_index, node_index, scene_index);
//     } else {
//         printf("Save failed: %s\n", esp_err_to_name(err));
//     }

//     //nvs_close(nvs_handle);
// }

esp_err_t update_binding_data_in_nvs(uint16_t target_short_addr, uint8_t scene_index, uint8_t ep_index, uint8_t binding_val) {
    esp_err_t err;

    #ifdef USE_NVS_INIT
    // Open NVS in read-write mode
    err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return err;
    }
    #endif
    // Prepare key for accessing the specific scene
    char key[16];
    snprintf(key, sizeof(key), "scene_%d", scene_index);
    // Search for the target node by short_addr in dst_node_info
    bool found = false;
    for (size_t i = 0; i < existing_nodes_info[scene_index].scene_switch_info.total_records; i++) {
        if (existing_nodes_info[scene_index].scene_switch_info.dst_node_info[i].short_addr == target_short_addr) {
            found = true;
            // Update the attr_data_info_t in the specified endpoint (ep_index)
            if (ep_index < MAX_DST_EP) {
                //if(new_binding_data != NULL)
                //memcpy(&existing_nodes_info[scene_index].scene_switch_info.dst_node_info[i].dst_ep_info.ep_data, (dst_ep_data_t*)new_binding_data, sizeof(dst_ep_data_t));
                existing_nodes_info[scene_index].scene_switch_info.dst_node_info[i].dst_ep_info.ep_data[ep_index].is_bind = binding_val;
                printf("**************UPDATE EP DATA IN NVS****************\n");
                printf("short_addr:%d, ep_index:%d\n", target_short_addr, ep_index);

                // printf("STATE:%d, LEVEL:%d\n", 
                // existing_nodes_info[scene_index].scene_switch_info.dst_node_info[i].dst_ep_info.ep_data[ep_index].data.state, 
                // existing_nodes_info[scene_index].scene_switch_info.dst_node_info[i].dst_ep_info.ep_data[ep_index].data.level);
                printf("******************************************************\n");
            } else {
                printf("Invalid ep_index!\n");
                #ifdef USE_NVS_INIT
                nvs_close(my_handle);
                #endif
                return ESP_ERR_INVALID_ARG;
            }
            break;
        }
    }

    if (!found) {
        printf("Node with short_addr %d not found.\n", target_short_addr);
        #ifdef USE_NVS_INIT
        nvs_close(my_handle);
        #endif
        return ESP_ERR_NOT_FOUND;
    }

    // Write the updated scene_switch back to NVS
    err = nvs_set_blob(my_handle, key, &existing_nodes_info[scene_index], sizeof(stt_scene_switch_t));
    if (err != ESP_OK) {
        printf("Error updating scene switch in NVS: %s\n", esp_err_to_name(err));
        #ifdef USE_NVS_INIT
        nvs_close(my_handle);
        #endif
        return err;
    }

    // Commit the changes
    err = nvs_commit(my_handle);
    if (err != ESP_OK) {
        printf("Error committing changes to NVS: %s\n", esp_err_to_name(err));
    }
    #ifdef USE_NVS_INIT
    // Close NVS handle
    nvs_close(my_handle);
    #endif
    return ESP_OK;
}

bool panic_toggle = false;

void nuos_get_data_from_nvs() { 
    touchLedsOffAfter1MinuteEnable = getNVSAllLedsOff();
    // printf("touchLedsOffAfter1MinuteEnable:%d\n", touchLedsOffAfter1MinuteEnable);
    #ifdef WRITE_NVS_CONFIG
    	write_nvs_configuration();
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
            clear_all_records_in_nvs();
        #else
            setNVSDaliNodesCommissioningCounts(0);
            for (size_t p = 0; p < 4; p++) {
                memset(&dali_nvs_stt[p], 0, sizeof(dali_nvs_stt[p]));
                nuos_store_dali_data_to_nvs(p);
            }    
        #endif
    #endif
    
	#if( (USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI) || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
		
            
        #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_GROUP_SWITCH)
            #ifdef USE_CUSTOM_SCENE
                for(int index=0; index<TOTAL_ENDPOINTS; index++){
                    size_t required_length = sizeof(zigbee_zcene_info_t);
                    readSceneInfoStruct(index, (zigbee_zcene_info_t*)&zb_scene_info[index], &required_length);
                    
                    printf("group_id[%d]:%d\n", index, zb_scene_info[index].group_id);
                    printf("scene_id[%d]:%d\n", index, zb_scene_info[index].scene_id);

                    printf("-----------------------------------\n");                                                 
                }
            #endif
        #else
            #ifdef USE_WIFI_WEBSERVER
            nuos_read_wifi_info_data_from_nvs();
            #endif
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI)
            dali_on_webpage_commissioning_counts = getNVSDaliNodesCommissioningCounts();
            // dali_on_webpage_commissioning_counts = getNVSDaliNodesStartAddrCounts();
            #ifdef USE_INDIVIDUAL_DALI_ADDRESSING
            int index=0;    
            #else
            for(int index=0; index<TOTAL_ENDPOINTS; index++){
            #endif    
                size_t required_length = sizeof(dali_device_ids_t);
                readDaliStruct(index, (dali_device_ids_t*)&dali_nvs_stt[index], &required_length);
                
                printf("dev_id0:%d\n", dali_nvs_stt[index].device_ids[0]);
                printf("dev_id1:%d\n", dali_nvs_stt[index].device_ids[1]);
                printf("dev_id2:%d\n", dali_nvs_stt[index].device_ids[2]);
                printf("dev_id3:%d\n", dali_nvs_stt[index].device_ids[3]);

                printf("-----------------------------------\n");
                // dali_nvs_stt[index].device_ids[0] =index+1;
                // dali_nvs_stt[index].device_ids[1] =index+2;
                if(dali_nvs_stt[index].brightness > 254){
                    dali_nvs_stt[index].brightness = 25 /*MIN_DIM_LEVEL_VALUE*/;
                }
                if(dali_nvs_stt[index].color_value > 254){
                    dali_nvs_stt[index].color_value = 254;
                }
                if(dali_nvs_stt[index].group_id > 254 || dali_nvs_stt[index].group_id == 0){
                    dali_nvs_stt[index].group_id = 1;
                }
                if(dali_nvs_stt[index].selected_scene_id > 254){
                    dali_nvs_stt[index].selected_scene_id = 1;
                }     
                if(dali_nvs_stt[index].scene_id[0] > 254){
                    dali_nvs_stt[index].scene_id[0] = 1;
                }  
                if(dali_nvs_stt[index].total_ids > 254){
                    dali_nvs_stt[index].total_ids = 0;
                } 
            #ifndef USE_INDIVIDUAL_DALI_ADDRESSING
            }
            #endif
            
            #endif
        #endif
	#endif

    uint8_t nvs_store_max = TOTAL_ENDPOINTS;
    #ifdef USE_ZB_ONLY_FAN
        nvs_store_max = TOTAL_ENDPOINTS+1;
    #endif
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
        nvs_store_max = TOTAL_BUTTONS+1;
    #endif
  
    zb_network_bdb_steering_count = getNVSZbNwSteeringCrashCounts();
    printf("zb_network_bdb_steering_count:%d\n", zb_network_bdb_steering_count);
 	esp_reset_reason_t reset_reason = esp_reset_reason();
    // Print the reset reason
    switch (reset_reason) {
        case ESP_RST_POWERON:
            printf("Reset reason        : Power-on reset %d\n", reset_reason);
            break;
        case ESP_RST_SW:
            printf("Reset reason        : Software reset via esp_restart()\n");
            break;
        case ESP_RST_PANIC:
            printf("Reset reason        : Software reset due to exception/panic\n");
            panic_toggle = !panic_toggle;
            setNVSWebServerEnableFlag(0);
            setNVSPanicAttack(1);
            break;
        case ESP_RST_INT_WDT:
            printf("Reset reason        : Reset due to interrupt watchdog timer\n");
            break;
        case ESP_RST_TASK_WDT:
            printf("Reset reason        : Task watchdog timer reset\n");
            break;
        case ESP_RST_BROWNOUT:
            printf("Reset reason        : Brownout reset (software or hardware)\n");
            break;
        default:
            printf("Reset reason        : Unknown\n");
            break;
    }  


    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI)  
        //clear_all_groups_and_scenes_in_nvs();
        get_nvs_dali_scene_switch_webpage_data();
    #endif
    
    printf("------------>READ NVS DATA<---------------\n");
    memset(device_info, 0, sizeof(device_info));
    for(int index=0; index<nvs_store_max; index++){
        size_t required_length = sizeof(led_indicator_handle_t);
        readEpStruct(index, (led_indicator_handle_t*)&device_info[index], &required_length);
        printf("-------------------%d---------------------\n", index);
        printf("device_state        : %d\n", device_info[index].device_state);
        #if (USE_ZIGBE_DEVICE_CATEGORY == CATEGORY_ZIGBEE_LIGHT)
            if(device_info[index].device_level < MIN_DIM_LEVEL_VALUE){
                device_info[index].device_level = MIN_DIM_LEVEL_VALUE;
            }
            printf("brightness          : %d\n", device_info[index].device_level);
            printf("light_color_x       : %d\n", device_info[index].light_color_x);
            printf("light_color_y       : %d\n", device_info[index].light_color_y);         
        #elif (USE_ZIGBE_DEVICE_CATEGORY == CATEGORY_ZIGBEE_LIGHT_FAN) 
        printf("fan_speed           : %d\n", device_info[index].fan_speed);
        printf("device_va           : %d\n", device_info[index].device_val);
        #elif (USE_ZIGBE_DEVICE_CATEGORY == CATEGORY_ZIGBEE_CURTAIN) 
            
            printf("curtain_percentage        : %d\n", device_info[index].device_level);
            if(device_info[index].device_level > 100){
                device_info[index].device_level = 100;
            }
            #if (USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN)
            printf("curtain_motor_start_offset        : %d\n", device_info[index].curtain_motor_start_offset);
            if(device_info[index].curtain_motor_start_offset > 1000){
                device_info[index].curtain_motor_start_offset = 0;
            }
            
            printf("curtain_motor_total_time        : %d\n", device_info[index].curtain_motor_total_time);
            if(device_info[index].curtain_motor_total_time > 120){
                device_info[index].curtain_motor_total_time = 0;
            }      
            if(device_info[index].curtain_state > 2) {
                device_info[index].curtain_state = 2; //stop
            }  
            #endif              
        #elif (USE_ZIGBE_DEVICE_CATEGORY == CATEGORY_ZIGBEE_THERMOSTAT)
        printf("ac_mode             : %d\n", device_info[index].ac_mode);
        printf("ac_decode_type      : %d\n", device_info[index].ac_decode_type);
        printf("ac_temperature      : %d\n", device_info[index].ac_temperature);
        #elif (USE_ZIGBE_DEVICE_CATEGORY == CATEGORY_ZIGBEE_SCENE_SWITCH) 

        #elif (USE_ZIGBE_DEVICE_CATEGORY == CATEGORY_ZIGBEE_SENSORS) 

        #elif (USE_ZIGBE_DEVICE_CATEGORY == CATEGORY_ZIGBEE_DALI_LIGHT)
        printf("brightness          : %d\n", device_info[index].device_level);
        printf("color_state         : %d\n", device_info[index].color_or_fan_state);
        printf("color_value         : %d\n", device_info[index].device_val);
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI)
        printf("color               : %d\n", device_info[index].device_level);
        printf("brightness          : %d\n", device_info[index].device_val); 
        #endif                             
    }

    #if (USE_ZIGBE_DEVICE_CATEGORY == CATEGORY_ZIGBEE_CURTAIN) 
        if(device_info[0].fan_speed > 1){
            device_info[0].fan_speed = 1;
        }
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SENSOR_MOTION)  
        read_motion_disable_timeout_value(); 
        if(pir_motion_disable_timeout_value<60 || pir_motion_disable_timeout_value>900) 
            pir_motion_disable_timeout_value = 0;

        read_motion_auto_enable_value();
        
    #else

      
    #endif

    #ifdef USE_WIFI_WEBSERVER
    dali_min_off_offset = getNVSDaliMinDimOffset();
    dali_range_size = 254 - dali_min_off_offset;


    dali_fade_time = getNVSDaliFadeTime();
    if(dali_fade_time > MAX_DALI_FADE_TIME){
        dali_fade_time = DEFAULT_DALI_FADE_TIME;
    }

    dali_fade_rate = getNVSDaliFadeRate();
    if(dali_fade_rate > MAX_DALI_FADE_RATE){
        dali_fade_rate = DEFAULT_DALI_FADE_RATE;
    }
    #endif
    selected_color_mode = read_color_mode_value();
    last_selected_color_mode = selected_color_mode;
    
    for(int mk=0; mk<TOTAL_ENDPOINTS; mk++){
        for(int rs=0; rs<MAX_NODES; rs++){
            is_reporting[mk][rs] = false;
        }
    }
    printf("start_commissioning:%d\n", start_commissioning);
    if(wifi_webserver_active_flag>0 || start_commissioning>0){
        #ifdef ZB_COMMISSIONING_WITHOUT_TIMER
            nuos_start_mode_change_task();
        #else
            init_timer();
            esp_start_timer();    
        #endif
    }
}

void get_nvs_dali_scene_switch_webpage_data(){
    size_t required_length = sizeof(scene_switch_s);
    nuos_read_dali_scene_switch_data_to_nvs((scene_switch_s*)&scene_group_switch_info, &required_length);
}

void init_nvs_for_zb_devices(){
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)

    memset(existing_nodes_info, 0, sizeof(existing_nodes_info));  
      
    for(int i=0; i<TOTAL_ENDPOINTS; i++){
        load_nodes_info_from_nvs(i);
        printf("----------->TOTAL_NODES:0x%0x<-------------\n", existing_nodes_info[i].scene_switch_info.total_records); 
        for(int k=0; k<existing_nodes_info[i].scene_switch_info.total_records; k++){
            // printf("SHORT_ADDR:0x%0x\n", existing_nodes_info[i].scene_switch_info.dst_node_info[k].short_addr);
            // printf("NODE_NAME:%s\n", existing_nodes_info[i].scene_switch_info.dst_node_info[k].node_name);
            // printf("EP_COUNTS:%0d\n", existing_nodes_info[i].scene_switch_info.dst_node_info[k].endpoint_counts);
            for(int j=0; j<existing_nodes_info[i].scene_switch_info.dst_node_info[k].endpoint_counts; j++){
                //printf("EP[%d]  NAME = %s IS_BIND = %d\n", existing_nodes_info[i].scene_switch_info.dst_node_info[k].dst_ep_info.ep_data[j].dst_ep, 
                // (char*)existing_nodes_info[i].scene_switch_info.dst_node_info[k].dst_ep_info.ep_data[j].ep_name,
                //     existing_nodes_info[i].scene_switch_info.dst_node_info[k].dst_ep_info.ep_data[j].is_bind); 
            }                   
        }
    }        
#endif 
}

void nuos_store_dali_scene_switch_data_to_nvs(void* str_data){
    esp_err_t err;
    #ifdef USE_NVS_INIT
    err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return;
    #endif
    nvs_set_blob(my_handle, nvram_struct_scene_sw_key, (scene_switch_s*)str_data, sizeof(scene_switch_s));
    ESP_ERROR_CHECK(nvs_commit(my_handle));
    #ifdef USE_NVS_INIT
    nvs_close(my_handle);
    #endif
}

void nuos_read_dali_scene_switch_data_to_nvs(void* str_data, size_t* length){
    // ESP_ERROR_CHECK(nvs_open_from_partition("nvs", ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle));
    esp_err_t err;
    #ifdef USE_NVS_INIT
    err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return;
    #endif
    err = nvs_get_blob(my_handle, nvram_struct_scene_sw_key, str_data, length);
    #ifdef USE_NVS_INIT
    nvs_close(my_handle);
    #endif
}










// Store scene membership as JSON string in NVS key "scene_<group_id>"
void nuos_store_scene_membership_to_nvs(int group_id, int *ids, int count)
{
    if (count <= 0) {
        ESP_LOGI(TAG_NVS, "nuos_store_scene_membership_to_nvs: nothing to store for group %d", group_id);
        // Still write an empty array to NVS (optional)
    }

    cJSON *arr = cJSON_CreateArray();
    if (!arr) {
        ESP_LOGE(TAG_NVS, "cJSON_CreateArray failed");
        return;
    }

    for (int i = 0; i < count; ++i) {
        cJSON_AddItemToArray(arr, cJSON_CreateNumber(ids[i]));
    }

    char *buf = cJSON_PrintUnformatted(arr);
    cJSON_Delete(arr);

    if (!buf) {
        ESP_LOGE(TAG_NVS, "cJSON_PrintUnformatted failed");
        return;
    }
    esp_err_t err = ESP_OK;
    #ifdef USE_NVS_INIT
    err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_NVS, "nvs_open failed: %s", esp_err_to_name(err));
        free(buf);
    }
    #endif

    char key[32];
    snprintf(key, sizeof(key), "scene_%d", group_id);
    err = nvs_set_str(my_handle, key, buf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_NVS, "nvs_set_str(%s) failed: %s", key, esp_err_to_name(err));
    } else {
        err = nvs_commit(my_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG_NVS, "nvs_commit failed: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG_NVS, "Saved scene membership for group %d (count=%d)", group_id, count);
        }
    }
    #ifdef USE_NVS_INIT
    nvs_close(my_handle);
    #endif
    free(buf);
}

// Read scene membership from NVS key "scene_<scene_index>".
// Returns 1 on success (and sets *out_count), 0 on not-found or error.
int nuos_read_scene_membership_from_nvs(int scene_index, int *out_ids, int *out_count)
{
    if (!out_ids || !out_count) return 0;

    char key[32];
    snprintf(key, sizeof(key), "scene_%d", scene_index);

    esp_err_t err = ESP_OK;
    #ifdef USE_NVS_INIT
    err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG_NVS, "nvs_open readonly failed for key %s: %s", key, esp_err_to_name(err));
        *out_count = 0;
        return 0;
    }
    #endif
    size_t required = 0;
    err = nvs_get_str(my_handle, key, NULL, &required);
    if (err != ESP_OK || required == 0) {
        // no data
        #ifdef USE_NVS_INIT
        nvs_close(my_handle);
        #endif
        *out_count = 0;
        return 0;
    }

    char *buf = malloc(required);
    if (!buf) {
        ESP_LOGE(TAG_NVS, "malloc failed");
        #ifdef USE_NVS_INIT
        nvs_close(my_handle);
        #endif
        *out_count = 0;
        return 0;
    }

    err = nvs_get_str(my_handle, key, buf, &required);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_NVS, "nvs_get_str failed: %s", esp_err_to_name(err));
        free(buf);
        #ifdef USE_NVS_INIT
        nvs_close(my_handle);
        #endif
        *out_count = 0;
        return 0;
    }

    cJSON *arr = cJSON_Parse(buf);
    free(buf);
    if (!arr || !cJSON_IsArray(arr)) {
        ESP_LOGW(TAG_NVS, "no valid JSON array stored for key %s", key);
        if (arr) cJSON_Delete(arr);
        #ifdef USE_NVS_INIT
        nvs_close(my_handle);
        #endif
        *out_count = 0;
        return 0;
    }

    int len = cJSON_GetArraySize(arr);
    int idx = 0;
    for (int i = 0; i < len; ++i) {
        cJSON *item = cJSON_GetArrayItem(arr, i);
        if (cJSON_IsNumber(item)) {
            out_ids[idx++] = item->valueint;
        }
        // ignore non-number items
    }
    *out_count = idx;

    cJSON_Delete(arr);
    #ifdef USE_NVS_INIT
    nvs_close(my_handle);
    #endif
    ESP_LOGI(TAG_NVS, "Loaded scene membership scene_index=%d count=%d", scene_index, *out_count);
    return 1;
}

// Toggle / add / remove a temporary selection list for a group: key "tmp_grp_<group_id>".
// state: 2 => toggle (add if not present, remove if present)
//        1 => add (ensure present)
//        0 => remove (ensure absent)
void nuos_toggle_tmp_selection(int dali_id, int group_id, int state)
{
    
    char key[32];
    snprintf(key, sizeof(key), "tmp_grp_%d", group_id);
    esp_err_t err = ESP_OK;
    #ifdef USE_NVS_INIT
    err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_NVS, "nvs_open failed: %s", esp_err_to_name(err));
        return;
    }
    #endif
    // Read existing JSON string
    size_t required = 0;
    err = nvs_get_str(my_handle, key, NULL, &required);
    char *buf = NULL;
    cJSON *arr = NULL;

    if (err == ESP_OK && required > 0) {
        buf = malloc(required);
        if (buf) {
            if (nvs_get_str(my_handle, key, buf, &required) == ESP_OK) {
                arr = cJSON_Parse(buf);
            }
            free(buf);
        }
    }

    if (!arr) {
        // create empty array if none existed
        arr = cJSON_CreateArray();
        if (!arr) {
            ESP_LOGE(TAG_NVS, "cJSON_CreateArray failed");
            #ifdef USE_NVS_INIT
            nvs_close(my_handle);
            #endif
            return;
        }
    }

    // Build current list into a C array for easier manipulation
    int len = cJSON_GetArraySize(arr);
    int *tmp = malloc(sizeof(int) * (len + 8)); // allow some extra
    if (!tmp) {
        ESP_LOGE(TAG_NVS, "malloc tmp failed");
        cJSON_Delete(arr);
        #ifdef USE_NVS_INIT
        nvs_close(my_handle);
        #endif
        return;
    }

    int count = 0;
    for (int i = 0; i < len; ++i) {
        cJSON *it = cJSON_GetArrayItem(arr, i);
        if (cJSON_IsNumber(it)) tmp[count++] = it->valueint;
    }

    // Helper to find index
    int found_index = -1;
    for (int i = 0; i < count; ++i) {
        if (tmp[i] == dali_id) { found_index = i; break; }
    }

    if (state == 2) { // toggle
        if (found_index >= 0) {
            // remove
            for (int i = found_index; i < count - 1; ++i) tmp[i] = tmp[i+1];
            count--;
        } else {
            // add
            tmp[count++] = dali_id;
        }
    } else if (state == 1) { // ensure present
        if (found_index < 0) tmp[count++] = dali_id;
    } else { // state == 0 ensure absent
        if (found_index >= 0) {
            for (int i = found_index; i < count - 1; ++i) tmp[i] = tmp[i+1];
            count--;
        }
    }

    // Recreate cJSON array
    cJSON_Delete(arr);
    arr = cJSON_CreateArray();
    if (!arr) {
        ESP_LOGE(TAG_NVS, "cJSON_CreateArray failed (2)");
        free(tmp);
        #ifdef USE_NVS_INIT
        nvs_close(my_handle);
        #endif
        return;
    }

    for (int i = 0; i < count; ++i) {
        cJSON_AddItemToArray(arr, cJSON_CreateNumber(tmp[i]));
    }
    free(tmp);

    char *out = cJSON_PrintUnformatted(arr);
    cJSON_Delete(arr);
    if (!out) {
        ESP_LOGE(TAG_NVS, "cJSON_PrintUnformatted failed");
        #ifdef USE_NVS_INIT
        nvs_close(my_handle);
        #endif
        return;
    }

    err = nvs_set_str(my_handle, key, out);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_NVS, "nvs_set_str(%s) failed: %s", key, esp_err_to_name(err));
    } else {
        err = nvs_commit(my_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG_NVS, "nvs_commit failed: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG_NVS, "tmp selection updated for group %d (count=%d)", group_id, (int)strlen(out)); // count is not exact here, just log
        }
    }

    free(out);
    #ifdef USE_NVS_INIT
    nvs_close(my_handle);
    #endif
}




// Helper: key names
void group_key_name(int group_id, char *out, size_t outlen) {
    snprintf(out, outlen, "group_%d", group_id);
}
void scene_key_name(int group_id, char *out, size_t outlen) {
    snprintf(out, outlen, "scene_%d", group_id);
}

// Read group blob (64 bytes)
esp_err_t read_group_blob(int group_id, uint8_t buffer[DALI_ADDR_COUNT]) {
    esp_err_t err = ESP_OK;
    #ifdef USE_NVS_INIT
    err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_NVS, "nvs_open failed: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }
    #endif
    char key[32];
    group_key_name(group_id, key, sizeof(key));
    size_t required = DALI_ADDR_COUNT;
    err = nvs_get_blob(my_handle, key, buffer, &required);
    if (err == ESP_OK && required != DALI_ADDR_COUNT) {
        err = ESP_ERR_INVALID_SIZE;
    }
    #ifdef USE_NVS_INIT
    nvs_close(my_handle);
    #endif
    return err;
}

// Write group blob
esp_err_t write_group_blob(int group_id, uint8_t buffer[DALI_ADDR_COUNT]) {
    esp_err_t err = ESP_OK;
    #ifdef USE_NVS_INIT
    err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_NVS, "nvs_open failed: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }
    #endif
    char key[32];
    group_key_name(group_id, key, sizeof(key));
    err = nvs_set_blob(my_handle, key, buffer, DALI_ADDR_COUNT);
    if (err == ESP_OK) nvs_commit(my_handle);
    #ifdef USE_NVS_INIT
    nvs_close(my_handle);
    #endif
    return err;
}

// Read scene blob (16 bytes)
esp_err_t read_scene_blob(int group_id, uint8_t buffer[SCENE_COUNT]) {
    esp_err_t err = ESP_OK;
    #ifdef USE_NVS_INIT
    err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_NVS, "nvs_open failed: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }
    #endif
    char key[32];
    scene_key_name(group_id, key, sizeof(key));
    size_t required = SCENE_COUNT;
    err = nvs_get_blob(my_handle, key, buffer, &required);
    if (err == ESP_OK && required != SCENE_COUNT) {
        err = ESP_ERR_INVALID_SIZE;
    }
    #ifdef USE_NVS_INIT
    nvs_close(my_handle);
    #endif
    return err;
}

// Write scene blob
esp_err_t write_scene_blob(int group_id, uint8_t buffer[SCENE_COUNT]) {
    esp_err_t err = ESP_OK;
    #ifdef USE_NVS_INIT
    err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_NVS, "nvs_open failed: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }
    #endif
    char key[32];
    scene_key_name(group_id, key, sizeof(key));
    err = nvs_set_blob(my_handle, key, buffer, SCENE_COUNT);
    if (err == ESP_OK) nvs_commit(my_handle);
    #ifdef USE_NVS_INIT
    nvs_close(my_handle);
    #endif
    return err;
}



/* ========== NVS CLEAR HELPERS ========== */
/* Place near other NVS helpers in webserver.c (or app_nvs_store_info.c) */


/* GROUP_COUNT should be defined in your project (16 normally) */
/* SCENE_COUNT likewise if you need per-scene clearing; we only use GROUP_COUNT here */

esp_err_t clear_group_in_nvs(int group_id)
{
    if (group_id < 0 || group_id >= GROUP_COUNT) return ESP_ERR_INVALID_ARG;

    esp_err_t err = ESP_OK;
    #ifdef USE_NVS_INIT
    err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_NVS, "nvs_open failed: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }
    #endif

    char key[32];
    snprintf(key, sizeof(key), "group_%d", group_id);

    err = nvs_erase_key(my_handle, key);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "clear_group_in_nvs: key %s not found (ok)", key);
        err = ESP_OK;
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "clear_group_in_nvs: nvs_erase_key(%s) failed: %s", key, esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "clear_group_in_nvs: erased %s", key);
    }

    if (err == ESP_OK) {
        err = nvs_commit(my_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "clear_group_in_nvs: nvs_commit failed: %s", esp_err_to_name(err));
        }
    }

    #ifdef USE_NVS_INIT
    nvs_close(my_handle);
    #endif
    return err;
}

esp_err_t clear_scene_in_nvs(int group_id)
{
    if (group_id < 0 || group_id >= GROUP_COUNT) return ESP_ERR_INVALID_ARG;

    esp_err_t err = ESP_OK;
    #ifdef USE_NVS_INIT
    err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_NVS, "nvs_open failed: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }
    #endif

    char key[32];
    snprintf(key, sizeof(key), "scene_%d", group_id);

    err = nvs_erase_key(my_handle, key);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "clear_scene_in_nvs: key %s not found (ok)", key);
        err = ESP_OK;
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "clear_scene_in_nvs: nvs_erase_key(%s) failed: %s", key, esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "clear_scene_in_nvs: erased %s", key);
    }

    if (err == ESP_OK) {
        err = nvs_commit(my_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "clear_scene_in_nvs: nvs_commit failed: %s", esp_err_to_name(err));
        }
    }

    #ifdef USE_NVS_INIT
    nvs_close(my_handle);
    #endif
    return err;
}

/**
 * Clears all group_<n> and scene_<n> keys in NVS namespace NVS_NAMESPACE_DALI
 * for n = 0 .. GROUP_COUNT-1.
 *
 * Returns ESP_OK on success (all keys erased or not found).
 */
esp_err_t clear_all_groups_and_scenes_in_nvs(void)
{
    esp_err_t err = ESP_OK;
    #ifdef USE_NVS_INIT
    err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_NVS, "nvs_open failed: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }
    #endif

    esp_err_t last_err = ESP_OK;
    for (int i = 0; i < GROUP_COUNT; ++i) {
        char gkey[32];
        snprintf(gkey, sizeof(gkey), "group_%d", i);
        err = nvs_erase_key(my_handle, gkey);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Cleared NVS key: %s", gkey);
        } else if (err == ESP_ERR_NVS_NOT_FOUND) {
            // not an error — treat as success
            err = ESP_OK;
        } else {
            ESP_LOGE(TAG, "Failed erase %s: %s", gkey, esp_err_to_name(err));
            last_err = err;
        }

        char skey[32];
        snprintf(skey, sizeof(skey), "scene_%d", i);
        err = nvs_erase_key(my_handle, skey);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Cleared NVS key: %s", skey);
        } else if (err == ESP_ERR_NVS_NOT_FOUND) {
            err = ESP_OK;
        } else {
            ESP_LOGE(TAG, "Failed erase %s: %s", skey, esp_err_to_name(err));
            last_err = err;
        }
    }

    // Commit changes (if any)
    err = nvs_commit(my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "clear_all_groups_and_scenes_in_nvs: nvs_commit failed: %s", esp_err_to_name(err));
        last_err = err;
    } else {
        ESP_LOGI(TAG, "clear_all_groups_and_scenes_in_nvs: commit OK");
    }

    #ifdef USE_NVS_INIT
    nvs_close(my_handle);
    #endif
    return last_err;
}



/* Global in-memory store (loaded at boot) */
device_scene_t *g_device_scene_store = NULL;
size_t g_device_scene_count = 0;

/* NVS namespace & key */
#define NVS_KEY_STORE "device_store"

// Write an array of device_scene_t to NVS as a blob
esp_err_t save_device_scene_store(const device_scene_t *arr, size_t count)
{
    if (!arr || count == 0) {
        return ESP_FAIL;
    }

    esp_err_t err = ESP_OK;
    #ifdef USE_NVS_INIT
    err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_NVS, "nvs_open failed: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }
    #endif
    size_t blob_size = count * sizeof(device_scene_t);
    err = nvs_set_blob(my_handle, NVS_KEY_STORE, arr, blob_size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "save_device_scene_store: nvs_set_blob failed: %s", esp_err_to_name(err));
        nvs_close(my_handle);
        return err;
    }

    err = nvs_commit(my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "save_device_scene_store: nvs_commit failed: %s", esp_err_to_name(err));
    }
    #ifdef USE_NVS_INIT
    nvs_close(my_handle);
    #endif
    ESP_LOGI(TAG, "Saved %d device_scene entries to NVS (size=%u bytes)", (int)count, (unsigned)blob_size);
    return err;
}

// Load device store from NVS into a newly allocated buffer (caller should free via free())
esp_err_t load_device_scene_store(device_scene_t **out_arr, size_t *out_count)
{
    if (!out_arr || !out_count) return ESP_ERR_INVALID_ARG;
    *out_arr = NULL;
    *out_count = 0;

    esp_err_t err = ESP_OK;
    #ifdef USE_NVS_INIT
    err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_NVS, "nvs_open failed: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }
    #endif

    // Query required size
    size_t required = 0;
    err = nvs_get_blob(my_handle, NVS_KEY_STORE, NULL, &required);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        // not stored yet
        #ifdef USE_NVS_INIT
        nvs_close(my_handle);
        #endif
        ESP_LOGI(TAG, "No device scene store (key not present)");
        return ESP_OK;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "load_device_scene_store: nvs_get_blob(size) failed: %s", esp_err_to_name(err));
        #ifdef USE_NVS_INIT
        nvs_close(my_handle);
        #endif
        return err;
    }

    if (required % sizeof(device_scene_t) != 0) {
        ESP_LOGW(TAG, "load_device_scene_store: blob size %u not multiple of entry size", (unsigned)required);
        #ifdef USE_NVS_INIT
        nvs_close(my_handle);
        #endif
        return ESP_ERR_INVALID_SIZE;
    }

    device_scene_t *buf = malloc(required);
    if (!buf) {
        #ifdef USE_NVS_INIT
        nvs_close(my_handle);
        #endif
        ESP_LOGE(TAG, "load_device_scene_store: malloc failed");
        return ESP_ERR_NO_MEM;
    }

    size_t read = required;
    err = nvs_get_blob(my_handle, NVS_KEY_STORE, buf, &read);
    #ifdef USE_NVS_INIT
    nvs_close(my_handle);
    #endif
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "load_device_scene_store: nvs_get_blob(read) failed: %s", esp_err_to_name(err));
        free(buf);
        return err;
    }

    size_t count = read / sizeof(device_scene_t);
    *out_arr = buf;
    *out_count = count;
    ESP_LOGI(TAG, "Loaded %u device_scene entries from NVS", (unsigned)count);
    return ESP_OK;
}



// Convert cJSON array "devices" into device_scene_t array and save.
// Returns ESP_OK on success.
esp_err_t persist_devices_from_cjson_array(cJSON *devices, int scene_id, const char *action)
{
    if (!devices || !cJSON_IsArray(devices)) return ESP_ERR_INVALID_ARG;

    // Build a temporary list (we don't want duplicates; you can choose policy)
    device_scene_t *arr = malloc(64 * sizeof(device_scene_t)); // worst-case upper bound
    if (!arr) return ESP_ERR_NO_MEM;
    size_t count = 0;

    cJSON *dev = NULL;
    cJSON_ArrayForEach(dev, devices) {
        if (!cJSON_IsObject(dev)) continue;
        cJSON *j_dali = cJSON_GetObjectItem(dev, "dali_id");
        if (!j_dali || !cJSON_IsNumber(j_dali)) continue;
        int did = j_dali->valueint;
        if (did < 0 || did >= 64) continue;

        // optional: read fields if present
        int power = 0, brightness = 50, cct = 3500;
        cJSON *j_power = cJSON_GetObjectItem(dev, "power");
        if (j_power && cJSON_IsNumber(j_power)) power = j_power->valueint;
        cJSON *j_brightness = cJSON_GetObjectItem(dev, "brightness");
        if (j_brightness && cJSON_IsNumber(j_brightness)) brightness = j_brightness->valueint;
        cJSON *j_cct = cJSON_GetObjectItem(dev, "cct");
        if (j_cct && cJSON_IsNumber(j_cct)) cct = j_cct->valueint;

        // If action == "add", set included=1, scene_id=provided scene
        // If action == "remove", set included=0 (and you may choose to keep scene_id field)
        uint8_t included_flag = 1;
        uint8_t sid = (uint8_t) scene_id;
        if (action && strcmp(action, "remove") == 0) {
            included_flag = 0;
        }

        arr[count].dali_id = (uint8_t) did;
        arr[count].scene_id = sid;
        arr[count].included = included_flag;
        arr[count].power = (uint8_t)(power ? 1 : 0);
        arr[count].brightness = (uint8_t)((brightness < 0) ? 0 : (brightness > 100 ? 100 : brightness));
        arr[count].cct = (uint16_t)cct;
        count++;
        if (count >= 64) break;
    }

    // If you want to merge with existing store (update existing entries) rather than replace,
    // you'd load existing store and update matching dali_id entries. For now this function replaces.
    esp_err_t err = save_device_scene_store(arr, count);
    free(arr);
    return err;
}

void nuos_store_group_membership_to_nvs(int group_id, int *ids, int count) {
    esp_err_t err = ESP_OK;
    cJSON *arr = cJSON_CreateIntArray(ids, count);
    char *s = cJSON_PrintUnformatted(arr);
    cJSON_Delete(arr);

    #ifdef USE_NVS_INIT
    err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return;
    else {
    #endif  
        char key[16]; 
        snprintf(key, sizeof(key), "grp_%d", group_id);
        nvs_set_str(my_handle, key, s);
        nvs_commit(my_handle);
        
    #ifdef USE_NVS_INIT    
    }
    nvs_close(my_handle);
    #endif
    free(s);
}

int nuos_read_selected_dali_address_from_nvs(void) {
    int32_t val = -1;
    esp_err_t err = ESP_OK;
    #ifdef USE_NVS_INIT
    err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return val;
    #endif
        nvs_get_i32(my_handle, "dali_sel", &val);

    #ifdef USE_NVS_INIT
    nvs_close(my_handle);
    #endif
    return val;
}