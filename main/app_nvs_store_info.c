
#include "nvs_flash.h"
#include "app_hardware_driver.h"
#include "esp_system.h"
#include "esp_zigbee_core.h"
#include "app_nuos_timer.h"
#include "esp_wifi_station.h"
#include "app_nvs_store_info.h"
#include <stdio.h>
#include <string.h>
#include "esp_check.h"
#include "esp_log.h"
#include "esp_err.h"
#ifdef USE_RGB_LED
#include "light_driver.h"
#endif
#include "esp_zigbee_core.h"

//
// #include <Wire.h>
// #include <I2C_eeprom.h>
//  //I2C_eeprom ee(0x51);
// #define DEVICEADDRESS (0x51)

// #define EE24LC01MAXBYTES 1024/8

// I2C_eeprom eeprom(DEVICEADDRESS, EE24LC01MAXBYTES);
static const char *TAG = "ESP_ZB_NVS_STORE";

extern wifi_info_handle_t wifi_info;

extern volatile bool pir_motion_disable_flag;
extern int32_t pir_motion_disable_timeout_counts;
extern uint32_t pir_motion_disable_timeout_value;
extern uint8_t last_cmd_id;
nvs_handle_t my_handle;
const char* nvram_struct_device_overall_keys = "ep_overall";
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

// const char* nvram_dmx_ch_key = "dmx_ch";


const char* LOG_INDEX_KEY = "LOG_INDEX";
const char* nvram_commissioning_keys = "com_key";
const char* nvram_start_commissioning_keys = "s_com_key";
const char* nvram_webserver_keys = "webserver_key";
const char* nvram_zb_steering_keys = "zb_init_key";
const char* nvram_sensor_commissioning_keys = "sen_com_key";
const char* nvram_wifi_info_keys = "wifi_info";
const char* nvram_struct_existing_info_key = "e_info_key";
const char* nvram_dali_commissioning_key = "dali_commiss";
const char* nvram_all_leds_off_keys = "leds_off_key";
const char* nvram_ip_addr_keys = "ip_key";
const char* nvram_scene_err_keys = "err_key";
const char* nvram_nvs_panic_attack = "panic";

void writeEpStruct(uint8_t index, void * strt);
void readEpStruct(uint8_t index, void* my_data, size_t* length);
esp_err_t save_reset_log_to_nvs(esp_reset_reason_t reset_reason) ;
// Function to read and print all logs in chronological order
esp_err_t read_all_reset_logs(void);
esp_err_t write_all_reset_logs_default(esp_reset_reason_t default_reason);

extern void nuos_start_mode_change_task();
#ifdef __cplusplus
    extern "C" {
#endif
extern void init_arduino_eeprom(uint8_t log_index, uint16_t restart_reason);
#ifdef __cplusplus
    }
#endif

#define ZIGBEE_NVS_PARTITION "storage"

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
    #if( (USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI) || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
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
#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
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



void store_curtain_cal_time(uint64_t time) {
    // Open
   esp_err_t err;
   #ifdef USE_NVS_INIT
   err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
   if (err == ESP_OK) {
    #endif
    
       // Write
       err = nvs_set_i64(my_handle, nvram_cal_keys, time);
       // Commit
       err = nvs_commit(my_handle);
       #ifdef USE_NVS_INIT
       // Close
       nvs_close(my_handle);
   }
   #endif
}

int64_t read_curtain_cal_time() {
    int64_t time = 0;
    // Open
	esp_err_t err;
    #ifdef USE_NVS_INIT
    err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
	if (err == ESP_OK) {
        #endif
		err = nvs_get_i64(my_handle, nvram_cal_keys, &time);
        #ifdef USE_NVS_INIT
		// Close
		nvs_close(my_handle);
	}
    #endif
    return time;
}

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
		device_info[index].light_color_x = 0x2300;
		device_info[index].light_color_y = 0x9100;
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
    setNVSStartCommissioningFlag(1);
	setNVSSensorsCommissioningCounts(0);
    //writeKeyValueToNVRAM(LOG_INDEX_KEY, 0);
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
		device_info[index].light_color_x = 0;
		device_info[index].light_color_y = 0;
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
	
	#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI  || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
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


void nuos_store_dali_data_to_nvs(uint8_t index){
    writeDaliStruct(index, (dali_device_ids_t*)&dali_nvs_stt[index]);
}
/////////////////////////////////////////////////////////////////////////////////////////////

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

size_t load_nodes_info_from_nvs12(uint8_t index) {
    esp_err_t err;
    #ifdef USE_NVS_INIT
    // Open NVS handle
    err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READONLY, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return 0;
    }
    #endif
    // Generate unique key based on index
    char key[16];
    snprintf(key, sizeof(key), "scene_%d", index);

    size_t required_size = sizeof(stt_scene_switch_t);

    // Try to get the blob from NVS
    err = nvs_get_blob(my_handle, key, &existing_nodes_info[index], &required_size);
    if (err == ESP_OK) {
        printf("Scene switch %u read from NVS successfully.\n", index);
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        printf("Scene switch %u not found in NVS.\n", index);
        #ifdef USE_NVS_INIT
        nvs_close(my_handle);
        #endif
        return 0;  // Key not found, return 0 records
    } else {
        printf("Error reading scene switch %u from NVS: %s\n", index, esp_err_to_name(err));
        #ifdef USE_NVS_INIT
        nvs_close(my_handle);
        #endif
        return 0;
    }
    #ifdef USE_NVS_INIT
    // Close NVS handle
    nvs_close(my_handle);
    #endif
    printf("TOTAL_RECORDS: %d\n", existing_nodes_info[index].scene_switch_info.total_records);
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

size_t store_new_nodes1234(void* n_node, uint8_t index) {
    esp_err_t err;
    #ifdef USE_NVS_INIT
    // Open NVS handle for read/write
    err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return 0;
    }
    #endif
    stt_scene_switch_t *new_node = (stt_scene_switch_t *)n_node;

    // Read existing nodes from NVS
    size_t required_size = sizeof(existing_nodes_info);
    char key[16];
    snprintf(key, sizeof(key), "scene_%d", index);
    err = nvs_get_blob(my_handle, key, existing_nodes_info, &required_size);
    if (err == ESP_OK) {
        printf("Existing nodes read from NVS successfully.\n");
    } else {
        printf("No existing nodes found in NVS or error occurred: %s\n", esp_err_to_name(err));
    }

    printf("new_node->scene_switch_info.total_records:%d\n", new_node->scene_switch_info.total_records);

    // Iterate over new nodes
    for (int src_ep_index = 0; src_ep_index < TOTAL_ENDPOINTS; src_ep_index++) {
        new_node->scene_id = global_scene_id[src_ep_index];
        new_node->scene_switch_info.src_ep = ENDPOINTS_LIST[src_ep_index];
        
        for (size_t j = 0; j < new_node->scene_switch_info.total_records; j++) {
            taskYIELD();
            uint16_t current_short_addr = new_node->scene_switch_info.dst_node_info[j].short_addr;
            if (current_short_addr != 0) {
                snprintf(key, sizeof(key), "scene_%d", src_ep_index);
                bool found = false;

                // Check for existence in the current storage
                for (size_t l = 0; l < existing_nodes_info[src_ep_index].scene_switch_info.total_records; l++) {
                    
                    if (existing_nodes_info[src_ep_index].scene_switch_info.dst_node_info[l].short_addr == current_short_addr) {
                        existing_nodes_info[src_ep_index].scene_switch_info.dst_node_info[l].endpoint_counts = new_node->scene_switch_info.dst_node_info[j].endpoint_counts;

                        memcpy(&existing_nodes_info[src_ep_index].scene_switch_info.dst_node_info[l].dst_ep_info, &new_node->scene_switch_info.dst_node_info[j].dst_ep_info, sizeof(dst_endpoint_info_t));
                        strcpy(existing_nodes_info[src_ep_index].scene_switch_info.dst_node_info[l].node_name, new_node->scene_switch_info.dst_node_info[j].node_name);
                        existing_nodes_info[src_ep_index].scene_switch_info.total_records++;
                        // Write the new struct to NVS
                        size_t size = sizeof(stt_scene_switch_t);
                        err = nvs_set_blob(my_handle, key, &existing_nodes_info[src_ep_index], size);
                        if (err != ESP_OK) {
                            printf("Error modifying record to NVS: %s\n", esp_err_to_name(err));
                        } else {
                            printf("Record modified successfully to NVS.\n");
                        }
                        taskYIELD();
                        found = true;
                        break;
                    }
                }

                // If the node wasn't found, insert it as a new entry
                if (!found) {
                    printf("Adding new entry for short_addr %d...\n", current_short_addr);
                    // Look for an available slot in existing records
                    for (size_t l = 0; l < MAX_NODES; l++) {
                        if (existing_nodes_info[src_ep_index].scene_switch_info.dst_node_info[l].short_addr == 0) {
                            // Find the first available slot
                            memcpy(&existing_nodes_info[src_ep_index].scene_switch_info.dst_node_info[l], &new_node->scene_switch_info.dst_node_info[j], sizeof(dst_node_info_t));
                            existing_nodes_info[src_ep_index].scene_switch_info.dst_node_info[l].endpoint_counts = new_node->scene_switch_info.dst_node_info[j].endpoint_counts;
                            existing_nodes_info[src_ep_index].scene_switch_info.src_ep = new_node->scene_switch_info.src_ep;
                            existing_nodes_info[src_ep_index].scene_id = global_scene_id[src_ep_index];
                            existing_nodes_info[src_ep_index].scene_switch_info.total_records++; // Increment the count of records
                            printf("Added new entry for short_addr %d at index %d.\n", current_short_addr, l);
                            taskYIELD();
                            break; // Exit the loop after adding
                        }
                    }

                    // Write the new struct to NVS
                    size_t size = sizeof(stt_scene_switch_t);
                    err = nvs_set_blob(my_handle, key, &existing_nodes_info[src_ep_index], size);
                    if (err != ESP_OK) {
                        printf("Error adding new scene switch to NVS: %s\n", esp_err_to_name(err));
                    } else {
                        printf("New scene switch added successfully to NVS.\n");
                    }
                }
            }
        }
    }

    // Commit changes to NVS
    err = nvs_commit(my_handle);
    if (err != ESP_OK) {
        printf("Error committing changes to NVS: %s\n", esp_err_to_name(err));
    }
    #ifdef USE_NVS_INIT
    // Close NVS handle
    nvs_close(my_handle);
    #endif
    return (existing_nodes_info[index].scene_switch_info.total_records);
}

esp_err_t update_attr_data_in_nvs(uint16_t target_short_addr, uint8_t scene_index, uint8_t ep_index, void* new_attr_data) {
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
                if(new_attr_data != NULL)
                memcpy(&existing_nodes_info[scene_index].scene_switch_info.dst_node_info[i].dst_ep_info.ep_data[ep_index].data, (attr_data_info_t*)new_attr_data, sizeof(attr_data_info_t));
                
                printf("**************UPDATE ATTRIBUTES IN NVS****************\n");
                printf("short_addr:0x%x, ep_index:%d\n", target_short_addr, ep_index);

                printf("STATE:%d, LEVEL:%d\n", 
                existing_nodes_info[scene_index].scene_switch_info.dst_node_info[i].dst_ep_info.ep_data[ep_index].data.state, 
                existing_nodes_info[scene_index].scene_switch_info.dst_node_info[i].dst_ep_info.ep_data[ep_index].data.level);
                printf("MODE:%d, COLOR:%ld\n", 
                    existing_nodes_info[scene_index].scene_switch_info.dst_node_info[i].dst_ep_info.ep_data[ep_index].data.mode, 
                    existing_nodes_info[scene_index].scene_switch_info.dst_node_info[i].dst_ep_info.ep_data[ep_index].data.value);                
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


void save_specific_ep_data(int scene_index, int node_index, int ep_index)
{
    // if (scene_index >= 4 || node_index >= MAX_NODES || ep_index >= MAX_DST_EP) {
    //     printf("Invalid index!\n");
    //     return;
    // }

    dst_ep_data_t *ep_data = &existing_nodes_info[scene_index]
                                 .scene_switch_info
                                 .dst_node_info[node_index]
                                 .dst_ep_info
                                 .ep_data[ep_index];

    // Open NVS
    // nvs_handle_t nvs_handle;
    // esp_err_t err = nvs_open(ZIGBEE_NVS_PARTITION, NVS_READWRITE, &nvs_handle);
    // if (err != ESP_OK) {
    //     printf("NVS open failed: %s\n", esp_err_to_name(err));
    //     return;
    // }

    // Unique key per ep_data
    // Prepare key for accessing the specific scene
    char key[16];
    snprintf(key, sizeof(key), "scene_%d", scene_index);

    esp_err_t err = nvs_set_blob(my_handle, key, ep_data, sizeof(dst_ep_data_t));
    if (err == ESP_OK) {
        nvs_commit(my_handle);
        printf("Saved ep_data[%d] of node[%d] in scene[%d]\n", ep_index, node_index, scene_index);
    } else {
        printf("Save failed: %s\n", esp_err_to_name(err));
    }

    //nvs_close(nvs_handle);
}

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
    printf("touchLedsOffAfter1MinuteEnable:%d\n", touchLedsOffAfter1MinuteEnable);
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
            nuos_read_wifi_info_data_from_nvs();
            #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI)
            dali_on_webpage_commissioning_counts = getNVSDaliNodesCommissioningCounts();
            
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
        
        // #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)

        //     memset(existing_nodes_info, 0, sizeof(existing_nodes_info));  
              
        //     // for(int i=0; i<TOTAL_ENDPOINTS; i++){
        //     //     load_nodes_info_from_nvs(i);
        //     //     printf("----------->TOTAL_NODES:0x%0x<-------------\n", existing_nodes_info[i].scene_switch_info.total_records); 
        //     //     for(int k=0; k<existing_nodes_info[i].scene_switch_info.total_records; k++){
        //     //         // printf("SHORT_ADDR:0x%0x\n", existing_nodes_info[i].scene_switch_info.dst_node_info[k].short_addr);
        //     //         // printf("NODE_NAME:%s\n", existing_nodes_info[i].scene_switch_info.dst_node_info[k].node_name);
        //     //         // printf("EP_COUNTS:%0d\n", existing_nodes_info[i].scene_switch_info.dst_node_info[k].endpoint_counts);
        //     //         for(int j=0; j<existing_nodes_info[i].scene_switch_info.dst_node_info[k].endpoint_counts; j++){
        //     //             //printf("EP[%d]  NAME = %s IS_BIND = %d\n", existing_nodes_info[i].scene_switch_info.dst_node_info[k].dst_ep_info.ep_data[j].dst_ep, 
        //     //             // (char*)existing_nodes_info[i].scene_switch_info.dst_node_info[k].dst_ep_info.ep_data[j].ep_name,
        //     //             //     existing_nodes_info[i].scene_switch_info.dst_node_info[k].dst_ep_info.ep_data[j].is_bind); 
        //     //         }                   
        //     //     }
        //     // }        
        // #endif 
 
	#endif

    //getNVSCommissioningFlag();
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
    ///////////////////////////////////////////////////////////////////////////////////    
    printf("------------>READ NVS DATA<---------------\n");
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
            printf("device_level        : %d\n", device_info[index].device_level);
            if(device_info[index].device_level > 100){
                device_info[index].device_level = 100;
            }
        #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN_SWITCH)
            last_cmd_id = device_info[index].fan_speed;
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
        curtain_cal_time = read_curtain_cal_time() / 100;
        printf("Read cal_time_percentage:%lld\n", curtain_cal_time);
        if(device_info[0].fan_speed > 1){
            device_info[0].fan_speed = 1;
        }
        // if(device_info[0].fan_speed == CURTAIN_OPEN){
        //     device_info[0].device_state = false;
        //     device_info[1].device_state = true;
        // }else if(device_info[0].fan_speed == CURTAIN_CLOSE){
        //     device_info[1].device_state = false;
        //     device_info[0].device_state = true;
        // }
        
    #endif
    selected_color_mode = read_color_mode_value();
    last_selected_color_mode = selected_color_mode;
    //printf("selected_color_mode:%d\n", selected_color_mode);
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