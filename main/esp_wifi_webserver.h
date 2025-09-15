
#ifndef MAIN_WEBSERVER_H_
	#define MAIN_WEBSERVER_H_
	#ifdef USE_WIFI_WEBSERVER
	    #include "esp_wifi_station.h"
		extern char webpage[33000];
		extern void prepare_html_complete_string();
		void start_webserver();
		bool nuos_init_webserver();
		extern void nuos_set_scene_devices(uint8_t index, bool state, uint8_t level, uint8_t src_ep, uint8_t dst_ep, uint16_t short_addr);
		extern  char * nuos_do_task(uint8_t index, uint8_t scene_id, uint8_t erase_data);
	#endif	
#endif /* MAIN_WIFI_STATION_H_ */
