
/* WiFi station Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "app_constants.h"
#include "app_hardware_driver.h"
//#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX  || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_CUSTOM|| USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
#ifdef USE_WIFI_WEBSERVER
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
// #include "protocol_examples_common.h"
#include "esp_coexist.h"
#include "esp_http_server.h"
#include "esp_wifi_station.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "app_nvs_store_info.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/ip4_addr.h"
// #include "esp_netif.h"


static const char *TAG = "WIFI_STATION";
wifi_config_t wifi_config;
esp_netif_ip_info_t ip_info;
/* The examples use WiFi configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_STA_SSID      			"NUOS Home Automation"
#define EXAMPLE_ESP_WIFI_STA_PASS      			"NUOS@FCSA"

#define EXAMPLE_ESP_MAXIMUM_RETRY  				15
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD 		3
#define ESP_WIFI_SAE_MODE                 		2
#define EXAMPLE_H2E_IDENTIFIER              	""

#define EXAMPLE_ESP_WIFI_CHANNEL   				1
#define EXAMPLE_MAX_STA_CONN       				5

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT 						BIT0
#define WIFI_FAIL_BIT      						BIT1


#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
	#define ESP_WIFI_SAE_MODE 					WPA3_SAE_PWE_HUNT_AND_PECK
	#define EXAMPLE_H2E_IDENTIFIER 				""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
	#define ESP_WIFI_SAE_MODE 					WPA3_SAE_PWE_HASH_TO_ELEMENT
	#define EXAMPLE_H2E_IDENTIFIER 				CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
	#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
	#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif
#if CONFIG_ESP_WIFI_AUTH_OPEN
	#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
	#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
	#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
	#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
	#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
	#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
	#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
	#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
wifi_ap_record_t* ap_list;
char jsonWiFiScanListStr[3000];

unsigned int ip_parts[4] = {0, 0, 0, 0};

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

const char* get_signal_level(int rssi) {
	int u8_rssi = -1 * rssi;
    if (u8_rssi >= 40 && u8_rssi <= 70) {
        return "Excellent";
    } else if (u8_rssi >= 71 && u8_rssi <= 80) {
        return "Good";
    } else {
        return "Poor/Bad";
    }
}

extern SemaphoreHandle_t recordsSemaphore;

// void monitor_coex_status() {
//     while(1) {
//         esp_coex_status_t status;
//         esp_coex_get_status(&status);
//         ESP_LOGI("COEX", "WiFi Active: %d, BLE Active: %d, Zigbee Active: %d",
//                status.wifi_active, status.ble_active, status.zigbee_active);
//         vTaskDelay(pdMS_TO_TICKS(2000));
//     }
// }



static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{

	if (event_id == WIFI_EVENT_SCAN_DONE) {

		uint16_t ap_count;
		esp_wifi_scan_get_ap_num(&ap_count);

		if (ap_count > 0) {
			ap_list = (wifi_ap_record_t*)malloc(sizeof(wifi_ap_record_t) * ap_count);
			if (ap_list) {
				esp_wifi_scan_get_ap_records(&ap_count, ap_list);
				int jsonStrSize = sizeof(jsonWiFiScanListStr);
				strcpy(jsonWiFiScanListStr, "["); // Initialize the JSON string with opening bracket
				for (int i = 0; i < ap_count; i++) {
					// -40 dBm to -70 dBm
					//Signal Strength: -71 dBm to -80 dBm
					//Signal Strength: Below -80 dBm
					//printf("SSID: %s, Signal: %s \n", (char*)ap_list[i].ssid, get_signal_level(ap_list[i].rssi));
		            const char* ssid = (char*)ap_list[i].ssid;
		            const char* signalLevel = (char*) get_signal_level(ap_list[i].rssi);
		            char itemStr[128];
					sprintf(itemStr, "{\"name\": \"%s\", \"range\": \"%s\"}", ssid, signalLevel);
					// Check if adding the next item exceeds the buffer size
					if (strlen(jsonWiFiScanListStr) + strlen(itemStr) + 2 < jsonStrSize) {
						strcat(jsonWiFiScanListStr, itemStr);
						if (i < ap_count - 1) {
							strcat(jsonWiFiScanListStr, ", ");
						}
					} else {
						//printf("Error: JSON string buffer is too small.\n");
						return;
					}
				}
				strcat(jsonWiFiScanListStr, "]"); // Close the JSON array
               // printf("JSON String: %s\n", jsonWiFiScanListStr);
				free(ap_list);
				// if(recordsSemaphore != NULL) xSemaphoreGive(recordsSemaphore);
			}
		}
	}else{
		if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
			esp_wifi_connect();
		} else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
			if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
				esp_wifi_connect();
				s_retry_num++;
				ESP_LOGI(TAG, "retry to connect to the AP");
			} else {
				xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
				wifi_info.is_wifi_sta_mode = 0;
				#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)

				nuos_store_wifi_info_data_to_nvs();
				#endif
				esp_restart();
			}
			ESP_LOGI(TAG,"connect to the AP fail");
			//Blink LED
		} else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
			ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;

			ip_info = event->ip_info;
			ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
            uint32_t ip_addr = event->ip_info.ip.addr; 
			ip_parts[3] = (ip_addr >> 24) & 0xFF;
			ip_parts[2] = (ip_addr >> 16) & 0xFF;
			ip_parts[1] = (ip_addr >> 8) & 0xFF;
			ip_parts[0] = ip_addr & 0xFF;

			esp_wifi_get_config(WIFI_IF_STA, &wifi_config);
			ESP_LOGI(TAG, "got ssid:%s", wifi_config.sta.ssid);
			ESP_LOGI(TAG, "got pass:%s", wifi_config.sta.password);
			
			s_retry_num = 0;
			xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);

			 // Start task after WiFi initialization
			 //xTaskCreate(monitor_coex_status, "coex_monitor", 2048, NULL, 12, NULL);
		}
	}
}

// Global variable to store base IP of the subnet
void wifi_init_softap(void)
{
    // ESP_ERROR_CHECK(esp_netif_init());
    // ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = ESP_WIFI_SSID,
            .ssid_len = strlen(ESP_WIFI_SSID),
            .channel = 6, /*EXAMPLE_ESP_WIFI_CHANNEL,*/
            .password = ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
#ifdef CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT
            .authmode = WIFI_AUTH_WPA3_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
#else /* CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT */
            .authmode = WIFI_AUTH_WPA2_PSK,
#endif
            .pmf_cfg = {
                    .required = true,
            },
        },
    };
    if (strlen(ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config)); 

	#if CONFIG_ESP_COEX_SW_COEXIST_ENABLE
		ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));
		esp_coex_wifi_i154_enable();
	#else
		ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
	#endif
	//esp_wifi_set_max_tx_power(8);
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             ESP_WIFI_SSID, ESP_WIFI_PASS, EXAMPLE_ESP_WIFI_CHANNEL);
}

#define IP4_ADDR(ipaddr, a, b, c, d) ((ipaddr)->addr = ((uint32_t)(d) << 24) | ((uint32_t)(c) << 16) | ((uint32_t)(b) << 8) | (uint32_t)(a))

   
void set_wifi_static(unsigned int ip_last_number){
		// Use sscanf to parse the IP address string into four integers
		// if (sscanf(ip_address, "%u.%u.%u.%u", &ip_parts[0], &ip_parts[1], &ip_parts[2], &ip_parts) != 4) {
		// 	fprintf(stderr, "Invalid IP address format\n");
		// }

		// Get the Wi-Fi station netif handle
	    esp_netif_t *sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
	    if (!sta_netif) {
	        printf("Error getting STA netif handle\n");
	        return;
	    }

	    //Set static IP address, gateway, and netmask
	    esp_netif_ip_info_t ip_info;
	    IP4_ADDR(&ip_info.ip, ip_parts[0], ip_parts[1], ip_parts[2], ip_last_number);     // Set your desired static IP address
	    IP4_ADDR(&ip_info.gw, ip_parts[0], ip_parts[1], ip_parts[2], 1);       // Set your gateway address
	    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0); // Set your netmask

	    //Set the static IP information for the station interface
	    esp_netif_dhcpc_stop(sta_netif); // Stop DHCP client
	    esp_netif_set_ip_info(sta_netif, &ip_info);
}

void set_wifi_additional_config_settings(){
	//		// Additional configuration settings if needed
		wifi_config.sta.threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD;
		wifi_config.sta.sae_pwe_h2e = ESP_WIFI_SAE_MODE;

		strncpy((char *)wifi_config.sta.sae_h2e_identifier, EXAMPLE_H2E_IDENTIFIER, sizeof(wifi_config.sta.sae_h2e_identifier) - 1);
		wifi_config.sta.sae_h2e_identifier[sizeof(wifi_config.sta.sae_h2e_identifier) - 1] = '\0';

		// Attempt to configure Wi-Fi
		esp_err_t wifi_err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
		if (wifi_err != ESP_OK) {
		    // Handle the error
		    printf("Failed to set Wi-Fi configuration: %d\n", wifi_err);
		} else {
		    // Continue with Wi-Fi initialization
		}
}


void wifi_init_sta()
{
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
	// ESP_ERROR_CHECK(esp_netif_init());
    // ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();

    // Create mutex here if needed
    // if (xWebServerSemaphore == NULL) {
    //     xWebServerSemaphore = xSemaphoreCreateMutex();
    // }

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	// cfg.static_rx_buf_num = 16;  // Default: 10
	// cfg.dynamic_rx_buf_num = 32; // Default: 32	
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	esp_event_handler_instance_t instance_any_id;
	esp_event_handler_instance_t instance_got_ip;


		s_wifi_event_group = xEventGroupCreate();
		ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
															ESP_EVENT_ANY_ID,
															&event_handler,
															NULL,
															&instance_any_id));
		ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
															IP_EVENT_STA_GOT_IP,
															&event_handler,
															NULL,
															&instance_got_ip));
        	wifi_config_t wifi_config = {
        		.sta = {
        			.ssid = "",
        			.password = "",
        			/* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (pasword len => 8).
        			 * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
        			 * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
        			 * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
        			 */
       			// .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
       			// .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
       			// .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
        		},
        	};


		// Populate the structure with string variables
		strncpy((char *)wifi_config.sta.ssid, wifi_info.wifi_ssid, sizeof(wifi_info.wifi_ssid) - 1);
		wifi_config.sta.ssid[sizeof(wifi_config.sta.ssid) - 1] = '\0';

		strncpy((char *)wifi_config.sta.password, wifi_info.wifi_pass, sizeof(wifi_info.wifi_pass) - 1);
		wifi_config.sta.password[sizeof(wifi_info.wifi_pass) - 1] = '\0';

		printf("-------wifi_ssid: %s size:%d\n", wifi_config.sta.ssid, sizeof(wifi_info.wifi_ssid));
		printf("-------wifi_pass: %s size:%d\n", wifi_config.sta.password, sizeof(wifi_info.wifi_pass));

        //add additional config info
		ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
		ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
		#if CONFIG_ESP_COEX_SW_COEXIST_ENABLE
			ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));
			esp_coex_wifi_i154_enable();
		#else
				ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
		#endif
		

		//set_wifi_tx_power(20);
		ESP_ERROR_CHECK(esp_wifi_start());



		// Add after esp_wifi_start():
		esp_wifi_set_bandwidth(ESP_IF_WIFI_STA, WIFI_BW_HT20); // Limit to 20MHz bandwidth		
		// ESP_LOGI(TAG, "wifi_init_sta finished.");
        //add static wifi address
		#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
			uint8_t wifi_connected_timeout = 0;
			while (true) {
				// Check if the WIFI_CONNECTED_BIT is set
				EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT,
													pdFALSE, pdFALSE, portMAX_DELAY);

				if (bits & WIFI_CONNECTED_BIT) {
					ESP_LOGI("WIFI_CHECK", "Wi-Fi is connected.");
					break;
				} else {
					ESP_LOGI("WIFI_CHECK", "Wi-Fi is not connected.");
				}

				// Delay to avoid spamming the log
				vTaskDelay(pdMS_TO_TICKS(1000));
				if(wifi_connected_timeout++ >= 15) {break;}
				
			}
			if(wifi_info.ip4 > 0) set_wifi_static(wifi_info.ip4);
		#endif
		/* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
		   number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
		EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
				WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
				pdFALSE,
				pdFALSE,
				portMAX_DELAY);
		/* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
		 * happened. */
		if (bits & WIFI_CONNECTED_BIT) {
			ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
					wifi_info.wifi_ssid, wifi_info.wifi_pass);
		} else if (bits & WIFI_FAIL_BIT) {
			ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
					wifi_info.wifi_ssid, wifi_info.wifi_pass);
			// //start in AP mode
			wifi_info.is_wifi_sta_mode = 0;
			#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)

			nuos_store_wifi_info_data_to_nvs();
			#endif
			esp_restart();
		} else {
			ESP_LOGE(TAG, "UNEXPECTED EVENT");
		}
		vEventGroupDelete(s_wifi_event_group);
}


void stop_wifi() {
	//CONFIG_ESP_COEX_SW_COEXIST_ENABLE
	//CONFIG_ESP_ZB_MEM_ALLOC_MODE_INTERNAL=y
//CONFIG_ESP_ZB_ZCL_BUFFER_SIZE=1024
    esp_err_t ret = esp_wifi_disconnect();
    if (ret != ESP_OK) {
        ESP_LOGE("WIFI", "WiFi disconnect failed: %s", esp_err_to_name(ret));
    }

	ret = esp_wifi_stop();
    if (ret != ESP_OK) {
        ESP_LOGE("WIFI", "WiFi stop failed: %s", esp_err_to_name(ret));
    }

    ret = esp_wifi_deinit();
    if (ret != ESP_OK) {
        ESP_LOGE("WIFI", "WiFi deinit failed: %s", esp_err_to_name(ret));
    }
}
void set_wifi_tx_power(int power) {
    // power is in 1/4 dBm units, e.g., 80 for 20 dBm
    esp_err_t ret = esp_wifi_set_max_tx_power(power);
    if (ret != ESP_OK) {
        printf("Failed to set WiFi TX power: %s\n", esp_err_to_name(ret));
    }
}

void wifi_scan() {
	esp_wifi_scan_start(NULL, true);
	
    //vTaskDelay(3000); 
	// recordsSemaphore = xSemaphoreCreateBinary(); //
	// if (recordsSemaphore == NULL) {
	// 	// Handle semaphore creation failure
	// 	ESP_LOGE(TAG, "Failed to create semaphore");
	// }
	// xSemaphoreTake(recordsSemaphore, 4000);  	
}

void wifi_pause(){
	if (xEventGroupGetBits(s_wifi_event_group) & WIFI_CONNECTED_BIT) {
        esp_wifi_stop();
        vTaskDelay(pdMS_TO_TICKS(50)); // Allow radio to stabilize
    }
}
void wifi_restart(){
    if (xEventGroupGetBits(s_wifi_event_group) & WIFI_CONNECTED_BIT) {
        esp_wifi_start();
        xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(5000));
    }
}
// void wifi_restart() {
// 	// Restart Wi-Fi
// 	esp_wifi_start();

// 	// Wait for Wi-Fi to reconnect
// 	EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT,
// 		pdFALSE, pdFALSE, pdMS_TO_TICKS(5000));

// 	if (bits & WIFI_CONNECTED_BIT) {
// 		ESP_LOGI("WiFi", "Reconnected successfully");
// 	} else {
// 		ESP_LOGE("WiFi", "Failed to reconnect!");
// 	}
// }

void start_wifi_scanning(){

	//xTaskCreate(wifi_scan_task, "wifi_scan_main", 2048, NULL, 20, NULL);
    // Start scanning
    wifi_scan_config_t scan_config = {
        .ssid = 0,
        .bssid = 0,
        .channel = 0,
        .show_hidden = true
    };
    ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_config, true));
}

void change_wifi_config(char* SECOND_SSID, char*SECOND_PASSWORD){
	if(wifi_info.is_wifi_sta_mode){
		stop_wifi();
		wifi_init_sta();
	}
}

bool wifi_station_main(void)
{
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
    printf("wifi_info.is_wifi_sta_mode: %d\n", wifi_info.is_wifi_sta_mode);
	if(wifi_info.is_wifi_sta_mode){
		wifi_init_sta();
		wifi_scan();
    }else{
    	wifi_init_softap();
    }
	#else
	    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN)
        wifi_init_sta();
		#else
		wifi_init_softap();
		#endif
	#endif
	return wifi_info.is_wifi_sta_mode;
}

#endif