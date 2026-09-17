idf_component_register(
    SRCS 
    "esp_zb_controller_main.c"
    "app_hardware_driver_dali_rgb.cpp"
    "app_hardware_driver_dali_direct_switch.cpp"
    "app_hardware_driver_dali_tunable_light_tuya.cpp"
    "app_hardware_driver_dali_scene_switch.cpp"
    "app_nvs_store_info.c"
    "app_zigbee_clusters.c"
    "switch_driver.c"
    "app_nuos_timer.c"
    "parson.c"
    "esp_wifi_station.c"
   
    "app_zigbee_misc.c"
    "esp_wifi_webserver.c"
    "nuos_zigbee_scene_commands.c" 
    "zigbee_2_uart.c"
   
    "DaliCommands.cpp"
    "DALI.cpp"
    "dali_receiver.cpp"
    SRC_DIRS "." "../../../common/zcl_utility/src"
    INCLUDE_DIRS "." "../../../common/zcl_utility/include" 
    EMBED_FILES "web_files/index_curtain.html" "web_files/index_dali_master.html" "web_files/index_dali_scn.html" "web_files/index_dali_switch.html" "web_files/index_dali_direct_addr.html"
    REQUIRES light_driver espressif__esp-zboss-lib mbedtls nvs_flash esp_timer app_update esp_http_server cjson esp_wifi
)

