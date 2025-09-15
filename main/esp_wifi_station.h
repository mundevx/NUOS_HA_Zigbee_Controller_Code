
#ifndef MAIN_WIFI_STATION_H_
	#define MAIN_WIFI_STATION_H_
	#ifdef USE_WIFI_WEBSERVER


		extern char jsonWiFiScanListStr[3000];
	
		extern void wifi_init_softap(void);
		extern void wifi_scan();
		extern bool wifi_station_main(void);
		extern void wifi_init_sta();
		extern void start_wifi_scanning();
		extern void wifi_restart();
		extern void wifi_pause();
	#endif	
#endif /* MAIN_WIFI_STATION_H_ */
