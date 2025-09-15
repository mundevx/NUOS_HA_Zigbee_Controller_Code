
#include "app_hardware_driver.h"
//#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_1CH_CURTAIN || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
#ifdef USE_WIFI_WEBSERVER
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "esp_event.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_wifi_station.h"
#include "esp_wifi_webserver.h"
#include "cJSON.h"
#include "eeprom.h"

typedef const char*  String;
char webpage[33000];
char buffer[15000];

extern wifi_info_handle_t wifi_info;
extern int map(int x, int in_min, int in_max, int out_min, int out_max);
extern char* prepare_json(uint8_t index);

/*
remove this line
input { padding: 10px; border: 1px solid #ccc; border-radius: 5px; outline: none; font-size: 16px; transition: border-color 0.3s; } 

*/
//////////////////////////////////////////////////////////////////////
String HTML_START = "<!DOCTYPE html><html>";
/********************HEADER+TITLE**********************/
#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
	String HEAD_TITLE ="<head> <meta charset=\"UTF-8\"> <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\"> <title>Select AC Models (NUOS)</title> </head>";
#elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
	String HEAD_TITLE ="<head> <meta charset=\"UTF-8\"> <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\"> <title>Select Scene Devices (NUOS)</title> </head>";
#elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI)
	String HEAD_TITLE ="<head> <meta charset=\"UTF-8\"> <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\"> <title>Set DALI Group Devices</title> </head>";
#endif
/********************STYLE**********************/
String STYLE_START = "<style>";
// String STYLE ="body { display: flex; flex-direction: column; align-items: center; justify-content: flex-start; min-height: 100vh; margin: 0; padding-top: 20px; } .grid-container { display: grid; grid-template-columns: repeat(2, 1fr); grid-template-rows: auto auto; row-gap: 30px; column-gap: 60px; width: 100%; max-width: auto; } .form-container { background-color: white; padding: 20px; border-radius: 8px; box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1); width: 560px; } .form { display: flex; flex-direction: column; } .form-row { display: flex; justify-content: space-between; margin-bottom: 10px; } .form-row input[type=\"number\"] { flex: 1; margin: 0 5px; padding: 10px; border: 1px solid #ddd; border-radius: 4px; } .form-group { margin-bottom: 5px; } .h1 { text-align: center; color: #333; margin-bottom: 15px; } .h2 { background: linear-gradient(to right, #6dd5ed, #2193b0); color: white; padding: 10px 10px; border-radius: 8px 8px 0 0; text-align: center; margin: -20px -20px 20px -20px; } .h3 { font-size: 12px; font-style: italic; } label { display: block; margin-bottom: 5px; } .input[type=\"number\"] { width: 100%; padding: 10px 15px; margin: 8px 0; box-sizing: border-box; border: 2px solid #FF3D00; border-radius: 4px; background-color: #FAFAFA; transition: border-color 0.3s, box-shadow 0.3s; box-shadow: 0 0px 2px rgba(0, 50, 0, 0.6); } .input::placeholder { color: #BDBDBD; opacity: 1; } input:hover { border-color: #66afe9; } input:focus { border-color: #4d90fe; box-shadow: 0 0 5px rgba(81, 203, 238, 1); } .submit-button { background: linear-gradient(to right, #2193b0, #2193b0); color: white; border: none; padding: 10px 15px; text-align: center; text-decoration: none; display: inline-block; font-size: 16px; border-radius: 4px; cursor: pointer; width: 100%; transition-duration: 0.4s; box-shadow: 0 2px 4px rgba(0, 150, 150, 0.6); } .stylish-line { border: 0; height: 1px; background-image: linear-gradient(to right, #ffffff, #ffffff); margin: 20px 0; box-shadow: 0 1px 4px rgba(0, 0, 100, 0.6); } .tabs-container { display: flex; background-color: #fff; border-radius: 8px; overflow: hidden; box-shadow: 0 0 10px rgba(0, 0, 0, 0.1); } .tab { flex: 1; padding: 15px; text-align: center; cursor: pointer; transition: background-color 0.3s, color 0.3s; } .tab:hover { background-color: #f0f0f0; } .tab.active { background-color: #3498db; color: #fff; } .stylish-button { background-color: #2196F3; border: none; color: white; padding: 10px 15px; text-align: center; text-decoration: none; display: inline-block; font-size: 16px; margin: 4px 2px; cursor: pointer; border-radius: 4px; transition-duration: 0.4s; } .stylish-button:hover { background-color: white; color: #2196F3; border: 2px solid #2196F3; } table { border-collapse: collapse; width: 100%; margin-top: 20px; } th, td { border: 1px solid #ddd; padding: 8px; text-align: left; } th { background-color: #f2f2f2; } tr:nth-child(even) { background-color: #f9f9f9; } .card1 { display: flex; flex-wrap: wrap; justify-content: space-between; } .container1 { flex-basis: calc(48% - 20px); margin-bottom: 20px; box-sizing: border-box; margin-right: 20px; } .container { display: flex; flex-direction: column; align-items: center; width: 100%; } .card { background-color: #fff; border-radius: 8px; box-shadow: 0 0 10px rgba(0, 0, 0, 0.1); padding: 20px; margin-top: 20px; display: none; } .tabs-container { display: flex; background-color: #fff; border-radius: 8px; overflow: hidden; box-shadow: 0 0 10px rgba(0, 0, 0, 0.1); } .card.active { display: block; } .wifi-network { display: flex; align-items: center; margin-bottom: 10px; padding: 10px; border: 1px solid #ccc; border-radius: 8px; } .wifi-range-icon { width: 24px; height: 24px; margin-left: 10px; background-color: #3498db; border-radius: 50%; }";
#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
String STYLE = "body { display: flex; flex-direction: column; align-items: center; justify-content: flex-start; min-height: 100vh; margin: 0; padding-top: 20px; } .grid-container { display: grid; grid-template-columns: repeat(2, 1fr); grid-template-rows: auto auto; row-gap: 30px; column-gap: 60px; width: 100%; max-width: auto; } .form-container { background-color: white; padding: 20px; border-radius: 8px; box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1); width: 560px; } .form { display: flex; flex-direction: column; } .form-row { display: flex; justify-content: space-between; margin-bottom: 10px; } .form-row input[type=\"number\"] { flex: 1; margin: 0 5px; padding: 10px; border: 1px solid #ddd; border-radius: 4px; } .form-group { margin-bottom: 5px; } .h1 { text-align: center; color: #333; margin-bottom: 15px; } .h2 { background: linear-gradient(to right, #6dd5ed, #2193b0); color: white; padding: 10px 10px; border-radius: 8px 8px 0 0; text-align: center; margin: -20px -20px 20px -20px; } .h3 { font-size: 12px; font-style: italic; } label { display: block; margin-bottom: 5px; } input[type=\"number\"], select { width: 100%; height: 40px; margin-bottom: 20px; padding: 10px; font-size: 16px; border: 1px solid #ccc; border-radius: 5px; } .input::placeholder { color: #BDBDBD; opacity: 1; } input { padding: 10px; border: 1px solid #ccc; border-radius: 5px; outline: none; font-size: 16px; transition: border-color 0.3s; } input:hover { border-color: #66afe9; } input:focus { border-color: #4d90fe; box-shadow: 0 0 5px rgba(81, 203, 238, 1); } .submit-button { background: linear-gradient(to right, #2193b0, #2193b0); color: white; border: none; padding: 10px 15px; text-align: center; text-decoration: none; display: inline-block; font-size: 16px; border-radius: 4px; cursor: pointer; width: 100%; transition-duration: 0.4s; box-shadow: 0 2px 4px rgba(0, 150, 150, 0.6); } .stylish-line { border: 0; height: 1px; background-image: linear-gradient(to right, #ffffff, #ffffff); margin: 20px 0; box-shadow: 0 1px 4px rgba(0, 0, 100, 0.6); } .tabs-container { display: flex; background-color: #fff; border-radius: 8px; overflow: hidden; box-shadow: 0 0 10px rgba(0, 0, 0, 0.1); } .tab { flex: 1; padding: 15px; text-align: center; cursor: pointer; transition: background-color 0.3s, color 0.3s; } .tab:hover { background-color: #f0f0f0; } .tab.active { background-color: #3498db; color: #fff; } .stylish-button { background-color: #2196F3; border: none; color: white; padding: 10px 15px; text-align: center; text-decoration: none; display: inline-block; font-size: 16px; margin: 4px 2px; cursor: pointer; border-radius: 4px; transition-duration: 0.4s; } .stylish-button:hover { background-color: white; color: #2196F3; border: 2px solid #2196F3; } input[type=\"number\"] { width: 50px; height: 30px; text-align: center; } table { border-collapse: collapse; width: 100%; margin-top: 20px; } th, td { border: 1px solid #ddd; padding: 8px; text-align: left; } th { background-color: #f2f2f2; } tr:nth-child(even) { background-color: #f9f9f9; } .card1 { display: flex; flex-wrap: wrap; justify-content: space-between; } .container1 { flex-basis: calc(48% - 20px); margin-bottom: 20px; box-sizing: border-box; margin-right: 20px; } .container { display: flex; flex-direction: column; align-items: center; width: 100%; } .card { background-color: #fff; border-radius: 8px; box-shadow: 0 0 10px rgba(0, 0, 0, 0.1); padding: 20px; margin-top: 20px; display: none; } .tabs-container { display: flex; background-color: #fff; border-radius: 8px; overflow: hidden; box-shadow: 0 0 10px rgba(0, 0, 0, 0.1); } .card.active { display: block; } .wifi-network { display: flex; align-items: center; margin-bottom: 10px; padding: 10px; border: 1px solid #ccc; border-radius: 8px; } .wifi-range-icon { width: 24px; height: 24px; margin-left: 10px; background-color: #3498db; border-radius: 50%; }";
#elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH)
String STYLE ="body { display: flex; flex-direction: column; align-items: center; justify-content: flex-start; min-height: 100vh; margin: 0; padding-top: 20px; } .grid-container { display: grid; grid-template-columns: repeat(2, 1fr); grid-template-rows: auto auto; row-gap: 30px; column-gap: 60px; width: 100%; max-width: auto; } .form-container { background-color: white; padding: 20px; border-radius: 8px; box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1); width: 560px; } .form { display: flex; flex-direction: column; } .form-row { display: flex; justify-content: space-between; margin-bottom: 10px; } .form-row input[type=\"number\"] { flex: 1; margin: 0 5px; padding: 10px; border: 1px solid #ddd; border-radius: 4px; } .form-group { margin-bottom: 5px; } .h1 { text-align: center; color: #333; margin-bottom: 15px; } .h2 { background: linear-gradient(to right, #6dd5ed, #2193b0); color: white; padding: 10px 10px; border-radius: 8px 8px 0 0; text-align: center; margin: -20px -20px 20px -20px; } .h3 { font-size: 12px; font-style: italic; } label { display: block; margin-bottom: 5px; } input[type=\"number\"] { width: 100%; padding: 10px 15px; margin: 8px 0; box-sizing: border-box; border: 2px solid #FF3D00; border-radius: 4px; background-color: #FAFAFA; transition: border-color 0.3s, box-shadow 0.3s; box-shadow: 0 0px 2px rgba(0, 50, 0, 0.6); } input::placeholder { color: #BDBDBD; opacity: 1; } input { padding: 10px; border: 1px solid #ccc; border-radius: 5px; outline: none; font-size: 16px; transition: border-color 0.3s; } input:hover { border-color: #66afe9; } input:focus { border-color: #4d90fe; box-shadow: 0 0 5px rgba(81, 203, 238, 1); } .submit-button { background: linear-gradient(to right, #2193b0, #2193b0); color: white; border: none; padding: 10px 15px; text-align: center; text-decoration: none; display: inline-block; font-size: 16px; border-radius: 4px; cursor: pointer; width: 100%; transition-duration: 0.4s; box-shadow: 0 2px 4px rgba(0, 150, 150, 0.6); } .stylish-line { border: 0; height: 1px; background-image: linear-gradient(to right, #ffffff, #ffffff); margin: 20px 0; box-shadow: 0 1px 4px rgba(0, 0, 100, 0.6); } .tabs-container { display: flex; background-color: #fff; border-radius: 8px; overflow: hidden; box-shadow: 0 0 10px rgba(0, 0, 0, 0.1); } .tab { flex: 1; padding: 15px; text-align: center; cursor: pointer; transition: background-color 0.3s, color 0.3s; } .tab:hover { background-color: #f0f0f0; } .tab.active { background-color: #3498db; color: #fff; } .stylish-button { background-color: #2196F3; border: none; color: white; padding: 10px 15px; text-align: center; text-decoration: none; display: inline-block; font-size: 16px; margin: 4px 2px; cursor: pointer; border-radius: 4px; transition-duration: 0.4s; } .stylish-button:hover { background-color: white; color: #2196F3; border: 2px solid #2196F3; } table { border-collapse: collapse; width: 100%; margin-top: 20px; } th, td { border: 1px solid #ddd; padding: 8px; text-align: left; } th { background-color: #f2f2f2; } tr:nth-child(even) { background-color: #f9f9f9; } .container { display: flex; flex-direction: column; align-items: center; width: 100%; } .card { background-color: #fff; border-radius: 8px; box-shadow: 0 0 10px rgba(0, 0, 0, 0.1); padding: 20px; margin-top: 20px; display: none; } .card.active { display: block; }";
#else
String STYLE = "body { display: flex; flex-direction: column; align-items: center; justify-content: flex-start; min-height: 100vh; margin: 0; padding-top: 20px; } .grid-container { display: grid; grid-template-columns: repeat(2, 1fr); grid-template-rows: auto auto; row-gap: 30px; column-gap: 60px; width: 100%; max-width: auto; } .form-container { background-color: white; padding: 20px; border-radius: 8px; box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1); width: 560px; } .form { display: flex; flex-direction: column; } .form-row { display: flex; justify-content: space-between; margin-bottom: 10px; } .form-row input[type=\"number\"] { flex: 1; margin: 0 5px; padding: 10px; border: 1px solid #ddd; border-radius: 4px; } .form-group { margin-bottom: 5px; } .h1 { text-align: center; color: #333; margin-bottom: 15px; } .h2 { background: linear-gradient(to right, #6dd5ed, #2193b0); color: white; padding: 10px 10px; border-radius: 8px 8px 0 0; text-align: center; margin: -20px -20px 20px -20px; } .h3 { font-size: 12px; font-style: italic; } label { display: block; margin-bottom: 5px; } .input[type=\"number\"] { width: 100%; padding: 10px 15px; margin: 8px 0; box-sizing: border-box; border: 2px solid #FF3D00; border-radius: 4px; background-color: #FAFAFA; transition: border-color 0.3s, box-shadow 0.3s; box-shadow: 0 0px 2px rgba(0, 50, 0, 0.6); } .input::placeholder { color: #BDBDBD; opacity: 1; } input { padding: 10px; border: 1px solid #ccc; border-radius: 5px; outline: none; font-size: 16px; transition: border-color 0.3s; } input:hover { border-color: #66afe9; } input:focus { border-color: #4d90fe; box-shadow: 0 0 5px rgba(81, 203, 238, 1); } .submit-button { background: linear-gradient(to right, #2193b0, #2193b0); color: white; border: none; padding: 10px 15px; text-align: center; text-decoration: none; display: inline-block; font-size: 16px; border-radius: 4px; cursor: pointer; width: 100%; transition-duration: 0.4s; box-shadow: 0 2px 4px rgba(0, 150, 150, 0.6); } .stylish-line { border: 0; height: 1px; background-image: linear-gradient(to right, #ffffff, #ffffff); margin: 20px 0; box-shadow: 0 1px 4px rgba(0, 0, 100, 0.6); } .tabs-container { display: flex; background-color: #fff; border-radius: 8px; overflow: hidden; box-shadow: 0 0 10px rgba(0, 0, 0, 0.1); } .tab { flex: 1; padding: 15px; text-align: center; cursor: pointer; transition: background-color 0.3s, color 0.3s; } .tab:hover { background-color: #f0f0f0; } .tab.active { background-color: #3498db; color: #fff; } .stylish-button { background-color: #2196F3; border: none; color: white; padding: 10px 15px; text-align: center; text-decoration: none; display: inline-block; font-size: 16px; margin: 4px 2px; cursor: pointer; border-radius: 4px; transition-duration: 0.4s; } .stylish-button:hover { background-color: white; color: #2196F3; border: 2px solid #2196F3; } input[type=\"number\"] { width: 50px; height: 30px; text-align: center; } table { border-collapse: collapse; width: 100%; margin-top: 20px; } th, td { border: 1px solid #ddd; padding: 8px; text-align: left; } th { background-color: #f2f2f2; } tr:nth-child(even) { background-color: #f9f9f9; } .card1 { display: flex; flex-wrap: wrap; justify-content: space-between; } .container1 { flex-basis: calc(48% - 20px); margin-bottom: 20px; box-sizing: border-box; margin-right: 20px; } .container { display: flex; flex-direction: column; align-items: center; width: 100%; } .card { background-color: #fff; border-radius: 8px; box-shadow: 0 0 10px rgba(0, 0, 0, 0.1); padding: 20px; margin-top: 20px; display: none; } .tabs-container { display: flex; background-color: #fff; border-radius: 8px; overflow: hidden; box-shadow: 0 0 10px rgba(0, 0, 0, 0.1); } .card.active { display: block; } .wifi-network { display: flex; align-items: center; margin-bottom: 10px; padding: 10px; border: 1px solid #ccc; border-radius: 8px; } .wifi-range-icon { width: 24px; height: 24px; margin-left: 10px; background-color: #3498db; border-radius: 50%; }";
#endif
String STYLE_SELECT_AC_MODEL ="select { width: 100%; padding: 10px; margin-bottom: 20px; border: 1px solid #ccc; border-radius: 4px; background-color: #fff; font-size: 16px; } button { width: 100%; padding: 10px; border: none; border-radius: 4px; background-color: #28a745; color: white; font-size: 16px; cursor: pointer; } button:hover { background-color: #218838; }";
String STYLE_WIRELESS_SWITCH ="#item-container { display: grid; grid-template-columns: repeat(4, 1fr); gap: 10px; } .group-container { border: 1px solid #ccc; padding: 10px; display: flex; flex-direction: column; } .group-title { font-weight: bold; margin-bottom: 10px; } .item-button { padding: 10px; margin: 5px 0; background-color: #aaa; border: 1px solid #044; cursor: pointer; } .item-button.selected { background-color: #7b7; color: #fff; }";
#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH)
	String STYLE_SCENE_SWITCH =".card2 { background: white; border-radius: 8px; box-shadow: 0 0px 10px rgba(0, 0, 0, 0.1); padding: 20px; width:90%; margin-top: 20px; text-align: center; } .container2 { margin: 10px 0px 10px 0px; } .intensity-bar { display: flex; align-items: center; justify-content: center; } .intensity-bar input[type=range] { -webkit-appearance: none; appearance: none; width: 200px; height: 10px; background: #ddd; outline: none; border-radius: 5px; transition: background 0.3s; } .intensity-bar input[type=range]::-webkit-slider-thumb { -webkit-appearance: none; appearance: none; width: 20px; height: 20px; background: #66bb6a; cursor: pointer; border-radius: 50%; transition: background 0.3s; } .intensity-bar input[type=range]::-moz-range-thumb { width: 20px; height: 20px; background: #66bb6a; cursor: pointer; border-radius: 50%; transition: background 0.3s; } .intensity-bar span { margin-left: 10px; font-weight: bold; color: #555; } .switch { display: flex; align-items: center; justify-content: center; } .switch label { position: relative; display: inline-block; width: 50px; height: 24px; margin-left: 10px; } .switch input { opacity: 0; width: 0; height: 0; } .slider { position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0; background-color: #ccc; transition: .4s; border-radius: 24px; } .slider:before { position: absolute; content: \"\"; height: 20px; width: 20px; left: 2px; bottom: 2px; background-color: white; transition: .4s; border-radius: 50%; } input:checked + .slider { background-color: #66bb6a; } input:checked + .slider:before { transform: translateX(26px); } .add-scene-button { display: inline-block; padding: 10px 20px; font-size: 16px; color: white; background-color: #007bff; border: none; border-radius: 5px; cursor: pointer; transition: background-color 0.3s; } .add-scene-button:hover { background-color: #0056b3; } select:focus { border-color: #007bff; outline: none; }";
#endif
#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI)
	String STYLE_COLOR_TEMP =".slider-container { width: calc(100% - 20px); margin: 0px; position: relative; } .slider-container input[type=\"range\"] { width: 100%; height: 10px; -webkit-appearance: none; background: linear-gradient(to right, rgb(255, 138, 18), rgb(220, 229, 255)); border: 1px solid #ccc; border-radius: 5px; outline: none; margin: 10px 0; } .slider-container input[type=\"range\"]::-webkit-slider-thumb { -webkit-appearance: none; appearance: none; width: 25px; height: 25px; background-color: #4CAF50; border-radius: 50%; cursor: pointer; position: relative; z-index: 2; } .scale { display: flex; justify-content: space-between; margin-top: 10px; position: relative; z-index: 1; padding: 0 10px; } .tick { height: 5px; width: 1px; background-color: #000; position: absolute; bottom: -5px; } .label { position: absolute; top: 20px; transform: translateX(-50%); font-size: 50%; }";
	String STYLE_INTENSITY =".slider-container-1 { width: calc(100% - 20px); margin: 10px auto; position: relative; } .slider-1 { -webkit-appearance: none; appearance: none; width: 100%; height: 10px; border-radius: 5px; background: #d3d3d3; outline: none; opacity: 0.7; -webkit-transition: .2s; transition: opacity .2s; } .slider-1::-webkit-slider-thumb { -webkit-appearance: none; appearance: none; width: 20px; height: 20px; border-radius: 50%; background: #4CAF50; cursor: pointer; position: relative; z-index: 1; } .thumb-label { position: absolute; top: -30px; left: 50%; transform: translateX(-50%); background-color: #4CAF50; color: white; padding: 5px; border-radius: 5px; pointer-events: none; font-size: 12px; display: none; z-index: 0; }";
	String STYLE_SCENE_SELECTOR =".button-container { display: flex; justify-content: center; margin: 10px 0px 20px 6px; } .button-selector { background-color: #f0f0f0; border: 2px solid #ccc; border-radius: 5px; padding: 10px 15px; margin-right: 10px; cursor: pointer; transition: background-color 0.3s ease, color 0.3s ease; } .button-selector.selected { background-color: #4CAF50; color: white; }";
	String STYLE_SETTINGS =".row { margin-bottom: 10px; display: flex; align-items: center; } .color-input-container { display: flex; align-items: center; } .color-input-label { margin-right: 10px; } .color-input { border: 1px solid #ccc; border-radius: 3px; padding: 5px; margin-left: 10px; } .submit-btn { background-color: #4CAF50; color: white; border: none; border-radius: 3px; padding: 10px 0; text-align: center; text-decoration: none; display: inline-block; font-size: 16px; cursor: pointer; transition: background-color 0.3s ease; width: 90%; margin: 0 auto; } .submit-btn:hover { background-color: #45a049; } .delete-btn { background-color: #f44336; color: white; border: none; border-radius: 3px; padding: 10px 0; text-align: center; text-decoration: none; display: inline-block; font-size: 14px; cursor: pointer; transition: background-color 0.3s ease; width: 90%; margin: 0 0px; } .delete-btn:hover { background-color: #d32f2f; }";

    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
	String STYLE_RGB_DMX =".container-3 { text-align: center; background-color: white; padding: 20px; border-radius: 10px; box-shadow: 0 0 10px rgba(0,0,0,0.1); }";
    String STYLE_RGB_DMX_FORM_ROW =".form-row-1 { display: flex; justify-content: space-between; align-items: center; }";
	String STYLE_RGB_DMX_FORM_ROW_1 =".color-picker { height: 50px; width: 90px; border-radius: 5%; background-color: #dddddd; padding: 18; }";
	#endif
#endif

String STYLE_LOADER_SPINNER ="#loading-spinner { border: 5px solid #f3f3f3; border-top: 5px solid #3498db; border-radius: 50%; width: 40px; height: 40px; animation: spin 1s linear infinite; } @keyframes spin { 0% { transform: rotate(0deg); } 100% { transform: rotate(360deg); } }";
String STYLE_DIALOG_BOX ="#overlay { position: fixed; top: 0; left: 0; width: 100%; height: 100%; background-color: rgba(0, 0, 0, 0.5); display: none; z-index: 1; } #confirmationDialog { position: fixed; top: 50%; left: 50%; transform: translate(-50%, -50%); padding: 20px; width: 300px; background-color: #fff; border-radius: 8px; box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2); z-index: 2; display: none; text-align: center; animation: fadeIn 0.3s; } @keyframes fadeIn { from { opacity: 0; } to { opacity: 1; } } #confirmationDialog p { font-size: 16px; color: #333; margin-bottom: 20px; } .dialog-btn { padding: 10px 20px; border: none; border-radius: 4px; font-size: 14px; cursor: pointer; margin: 0 10px; } .dialog-btn-yes { background-color: #28a745; color: #fff; } .dialog-btn-yes:hover { background-color: #218838; } .dialog-btn-no { background-color: #dc3545; color: #fff; } .dialog-btn-no:hover { background-color: #c82333; }";
String STYLE_BUTTON_SPINNER =".spinner-icon { width: 16px; height: 16px; margin-left: 8px; vertical-align: middle; animation: spin 0.8s linear infinite; } .spinner-icon.hidden { display: none; } button:disabled { opacity: 0.6; cursor: not-allowed; }";
String STYLE_IP_ADDR =".ip-input-group { display: flex; justify-content: center; align-items: center; gap: 0.5rem; margin-bottom: 1.5rem; } .ip-segment { width: 50px; padding: 0.5rem; font-size: 1em; text-align: center; border: 1px solid #ddd; border-radius: 4px; transition: border-color 0.3s ease; } .ip-segment:focus { outline: none; border-color: #5b9bd5; } .dot { font-size: 1.2em; color: #333; vertical-align: middle; }";

String STYLE_CHECK_BOX =".checkbox-container { display: flex; flex-direction: row; align-items: center; margin-bottom: 8px; } .checkbox-label { font-size: 14px; font-weight: bold; margin-bottom: 4px; } .group-checkbox { width: 18px; height: 18px; cursor: pointer; }";
String STYLE_END = "</style>";
/********************BODY**********************/
String BODY_START = "<body>";

#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
	String TAB_DISPLAY ="<div class=\"tabs-container\">  <div class=\"tab\" onclick=\"openTab(event, 'Tab2')\">Select IR AC Model</div> </div>";
#elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
	String TAB_DISPLAY ="<div class=\"tabs-container\"> <div class=\"tab\" onclick=\"openTab(event, 'Tab1')\">Set WiFi Credentials</div> <div class=\"tab\" onclick=\"openTab(event, 'Tab2')\">Wirelss Remote Switch Settings</div> </div>";
#elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH)
	String TAB_DISPLAY ="<div class=\"tabs-container\"> <div class=\"tab\" onclick=\"openTab(event, 'Tab1')\">Set WiFi Credentials</div> <div class=\"tab\" onclick=\"openTab(event, 'Tab2')\">Group Switch Settings</div> </div>";
#elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH)
	String TAB_DISPLAY ="<div class=\"tabs-container\"> <div class=\"tab\" onclick=\"openTab(event, 'Tab1')\">Set WiFi Credentials</div> <div class=\"tab\" onclick=\"openTab(event, 'Tab2')\">Scene Switch Settings</div> </div>";
#elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI)
	#ifdef USE_DALI_SCENES
		String TAB_DISPLAY ="<div class=\"tabs-container\"> <div class=\"tab\" onclick=\"openTab(event, 'Tab1')\">Set WiFi Credentials</div> <div class=\"tab\" onclick=\"openTab(event, 'Tab2')\">Set Dali IDs to Group</div> <div class=\"tab\" onclick=\"openTab(event, 'Tab3')\">Assign Dali Addresses</div> <div class=\"tab\" onclick=\"openTab(event, 'Tab4')\">Settings</div> </div>";
	#else
	    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
		String TAB_DISPLAY ="<div class=\"tabs-container\"> <div class=\"tab\" onclick=\"openTab(event, 'Tab2')\">Set Color</div> <div class=\"tab\" onclick=\"openTab(event, 'Tab3')\">DMX Settings</div> </div>";
		#else
		String TAB_DISPLAY ="<div class=\"tabs-container\"> <div class=\"tab\" onclick=\"openTab(event, 'Tab2')\">Set Dali IDs to Group</div> <div class=\"tab\" onclick=\"openTab(event, 'Tab3')\">Assign Dali Addresses</div> </div>";
		#endif
	#endif   
#endif

#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
	String TAB2_START = "<div id=\"Tab2\" class=\"card\"> <div class=\"container\"> <h1>Select Your AC Model</h1>";
	String TAB2_BODY ="<form name=\"acForm\" action=\"/action\" method=\"post\" onsubmit=\"return false;\">"; 
	String TAB2_SELECT ="<select id=\"ac-model\" name=\"ac_model\"> <option value=\"\">--Select a model--</option> <option value=\"RC5\">RC5</option> <option value=\"RC6\">RC6</option> <option value=\"NEC\">NEC</option> <option value=\"SONY\">SONY</option> <option value=\"PANASONIC\">PANASONIC</option> <option value=\"JVC\">JVC</option> <option value=\"SAMSUNG\">SAMSUNG</option> <option value=\"WHYNTER\">WHYNTER</option> <option value=\"AIWA_RC_T501\">AIWA_RC_T501</option> <option value=\"LG\">LG</option> <option value=\"SANYO\">SANYO</option> <option value=\"MITSUBISHI\">MITSUBISHI</option> <option value=\"DISH\">DISH</option> <option value=\"SHARP\">SHARP</option> <option value=\"COOLIX\">COOLIX</option> <option value=\"DAIKIN\">DAIKIN</option> <option value=\"DENOM\">DENOM</option> <option value=\"KELVINATOR\">KELVINATOR</option> <option value=\"SHERWOOD\">SHERWOOD</option> <option value=\"MITSUBISHI_AC\">MITSUBISHI_AC</option> <option value=\"RCMM\">RCMM</option> <option value=\"SANYO_LC7461\">SANYO_LC7461</option> <option value=\"RC5X\">RC5X</option> <option value=\"GREE\">GREE</option> <option value=\"PRONTO\">PRONTO</option> <option value=\"NEC_LIKE\">NEC_LIKE</option> <option value=\"AGRO\">AGRO</option> <option value=\"TROTEC\">TROTEC</option> <option value=\"NIKAI\">NIKAI</option> <option value=\"RAW\">RAW</option> <option value=\"GLOBALCACHE\">GLOBALCACHE</option> <option value=\"TOSHIBA_AC\">TOSHIBA</option> <option value=\"FUJITSU_AC\">FUJITSU</option> <option value=\"MIDEA\">MIDEA</option> <option value=\"MAGIQUEST\">MAGIQUEST</option> <option value=\"LASERTAG\">LASERTAG</option> <option value=\"CARRIER_AC\">CARRIER_AC</option> <option value=\"HAIER_AC\">HAIER_AC</option> <option value=\"MITSUBISHI2\">MITSUBISHI2</option> <option value=\"HITACHI_AC\">HITACHI_AC</option> <option value=\"HITACHI_AC1\">HITACHI_AC1</option> <option value=\"HITACHI_AC2\">HITACHI_AC2</option> <option value=\"GICABLE\">GICABLE</option> <option value=\"HAIER_AC_YRW02\">HAIER_AC_YRW02</option> <option value=\"WHIRLPOOL_AC\">WHIRLPOOL_AC</option> <option value=\"SAMSUNG_AC\">SAMSUNG_AC</option> <option value=\"LUTRON\">LUTRON</option> <option value=\"ELECTRA_AC\">ELECTRA_AC</option> <option value=\"PANASONIC_AC\">PANASONIC</option> <option value=\"PIONEER\">PIONEER</option> <option value=\"LG2\">LG2</option> <option value=\"MWM\">MWM</option> <option value=\"DAIKIN2\">DAIKIN2</option> <option value=\"VESTEL_AC\">VESTEL_AC</option> <option value=\"TECO\">TECO</option> <option value=\"SAMSUNG36\">SAMSUNG36</option> <option value=\"TCL112AC\">TCL112AC</option> <option value=\"LEGOPF\">LEGOPF</option> <option value=\"MITSUBISHI_HEAVY_88\">MITSUBISHI_HEAVY_88</option> <option value=\"MITSUBISHI_HEAVY_152\">MITSUBISHI_HEAVY_152</option> <option value=\"DAIKIN216\">DAIKIN216</option> <option value=\"SHARP_AC\">SHARP_AC</option> <option value=\"GOODWEATHER\">GOODWEATHER</option> <option value=\"INAX\">INAX</option> <option value=\"DAIKIN160\">DAIKIN160</option> <option value=\"NEOCLIMA\">NEOCLIMA</option> <option value=\"DAIKIN176\">DAIKIN176</option> <option value=\"DAIKIN128\">DAIKIN128</option> <option value=\"AMCOR\">AMCOR</option> <option value=\"DAIKIN152\">DAIKIN152</option> <option value=\"MITSUBISHI136\">MITSUBISHI136</option> <option value=\"MITSUBISHI112\">MITSUBISHI112</option> <option value=\"HITACHI_AC424\">HITACHI_AC424</option> <option value=\"SONY_38K\">SONY_38K</option> <option value=\"EPSON\">EPSON</option> <option value=\"SYMPHONY\">SYMPHONY</option> <option value=\"HITACHI_AC3\">HITACHI_AC3</option> <option value=\"DAIKIN64\">DAIKIN64</option> <option value=\"AIRWELL\">AIRWELL</option> <option value=\"DELONGHI_AC\">DELONGHI_AC</option> <option value=\"DOSHISHA\">DOSHISHA</option> <option value=\"MULTIBRACKETS\">MULTIBRACKETS</option> <option value=\"CARRIER_AC40\">CARRIER_AC40</option> <option value=\"CARRIER_AC64\">CARRIER_AC64</option> <option value=\"HITACHI_AC344\">HITACHI_AC344</option> <option value=\"CORONA_AC\">CORONA_AC</option> <option value=\"MIDEA24\">MIDEA24</option> <option value=\"ZEPEAL\">ZEPEAL</option> <option value=\"SANYO_AC\">SANYO_AC</option> <option value=\"VOLTAS\">VOLTAS</option> <option value=\"METZ\">METZ</option> <option value=\"TRANSCOLD\">TRANSCOLD</option> <option value=\"TECHNIBEL_AC\">TECHNIBEL</option> <option value=\"MIRAGE\">MIRAGE</option> <option value=\"ELITESCREENS\">ELITESCREENS</option> <option value=\"PANASONIC_AC32\">PANASONIC_AC32</option> <option value=\"MILESTAG2\">MILESTAG2</option> <option value=\"ECOCLIM\">ECOCLIM</option> <option value=\"XMP\">XMP</option> <option value=\"TRUMA\">TRUMA</option> <option value=\"HAIER_AC176\">HAIER_AC176</option> <option value=\"TEKNOPOINT\">TEKNOPOINT</option> <option value=\"KELON\">KELON</option> <option value=\"TROTEC_3550\">TROTEC_3550</option> <option value=\"SANYO_AC88\">SANYO_AC88</option> <option value=\"BOSE\">BOSE</option> <option value=\"ARRIS\">ARRIS</option> <option value=\"RHOSS\">RHOSS</option> <option value=\"AIRTON\">AIRTON</option> <option value=\"COOLIX48\">COOLIX48</option> <option value=\"HITACHI_AC264\">HITACHI_AC264</option> <option value=\"KELON168\">KELON168</option> <option value=\"HITACHI_AC296\">HITACHI_AC296</option> <option value=\"DAIKIN200\">DAIKIN200</option> <option value=\"HAIER_AC160\">HAIER_AC160</option> <option value=\"CARRIER_AC128\">CARRIER_AC128</option> <option value=\"TOTO\">BOSE</option> <option value=\"CLIMABUTLER\">CLIMABUTLER</option> <option value=\"TCL96AC\">TCL96AC</option> <option value=\"BOSCH144\">BOSCH144</option> <option value=\"SANYO_AC152\">SANYO_AC152</option> <option value=\"DAIKIN312\">DAIKIN312</option> <option value=\"GORENJE\">GORENJE</option> <option value=\"WOWWEE\">WOWWEE</option> <option value=\"CARRIER_AC84\">CARRIER_AC84</option> <option value=\"YORK\">YORK</option> </select>";
	String TAB2_BUTTON ="<div style=\"display: flex; justify-content: space-between; width: 100%;\"> <button type=\"button\" style=\"background-color: blue; color: white;\" onclick=\"validateAcModel('3')\">Try</button> <button type=\"button\" style=\"background-color: orange; color: white; margin: 0 10px;\" onclick=\"validateAcModel('2')\">Save</button> <button type=\"button\" style=\"background-color: red; color: white;\" onclick=\"validateAcModel('4')\">Disable WiFi</button> </div>";
	String TAB2_END = " </form></div></div>";
#elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
	String TAB1_START ="<div id=\"Tab1\" data-tab=\"Tab1\" class=\"card\"> <div class=\"card1\">";
	String SHOW_ALL_WIFI_NETWORKS = "<div class=\"container1\"> <h3>Available Wi-Fi Networks</h3> <div id=\"wifiContainer\"></div> <button type=\"button\" class=\"stylish-button\" onclick=\"fetchInitialData()\">Refresh Networks</button> <br> <button id=\"apModeButton\" onclick=\"toggleButton()\">AP mode enabled</button></div>";
	String START_WIFI_FORM_CONTAINER = "<div class=\"container1\"> <h3>Wi-Fi Settings</h3> <form id=\"wifiForm\" action=\"/action\" method=\"post\" onsubmit=\"submitData(event, this, 0, 1)\">";
	String SET_WIFI_SSID_VALUE = "<div class=\"form-group\"> <label for=\"ssid\">Network SSID:</label> <input type=\"text\" id=\"ssid\" value=\"[ssid]\" name=\"ssid\" required> </div> <br>";
	String SET_WIFI_PASS_VALUE = "<div class=\"form-group\"> <label for=\"password\">Password:</label> <input type=\"password\" id=\"password\" value=\"[pass]\" name=\"password\" required> </div> <br>";
	String SET_WIFI_IP_VALUE ="<label for=\"ip\">Enter IP Address</label> <div class=\"ip-input-group\"> <label for=\"ip1\">192</label> <span class=\"dot\">.</span> <label for=\"ip2\">168</label> <span class=\"dot\">.</span> <label for=\"ip3\">xxx</label> <span class=\"dot\">.</span> <input type=\"number\" class=\"ip-input\" id=\"ip4\" name=\"ip4\" min=\"0\" max=\"255\" required> </div><br>";
	String SET_WIFI_INFO_END = "<button type=\"submit\" class=\"submit-button\">Submit</button> </form> </div>";
	String TAB1_END = "</div></div>";
	
	String TAB2_START = "<div id=\"Tab2\" data-tab=\"Tab2\" class=\"card\"> <div class=\"container\"> <h1>Scene Switch Settings</h1>";
	String TAB2_BODY ="<button id=\"fetch-button\" onclick=\"showConfirmationDialog()\">Fetch All Zigbee Devices</button>";
	String TAB2_LOAD_SPINNER ="<div id=\"loading-spinner\" style=\"display: none;\"></div>";
	String TAB2_DIALOG_BOX ="<div id=\"overlay\"></div> <div id=\"confirmationDialog\"> <p>Do you want to reload all Zigbee Nodes?<br> It will erase all your NVS binding data</p> <button class=\"dialog-btn dialog-btn-yes\" onclick=\"reloadNodes(1)\">Yes</button> <button class=\"dialog-btn dialog-btn-no\" onclick=\"reloadNodes(0)\">No</button> </div>";
	String TAB2_SELECT = "<br> <div id=\"item-container\"></div>";
	String TAB2_IDENTIFY_BUTTON ="<br> <button id=\"submit-button\" onclick=\"submitSelectedButtons('20')\" style=\"background-color: blue; color: white;\">Identify Zigbee Devices</button>";
	String TAB2_BUTTON ="<table> <tr> <td><button id=\"submit-button-1\" onclick=\"submitSelectedButtons('11')\" style=\"background-color: orange; color: white;\">";
	#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
		String  TAB2_BUTTON_NAME = "Bind Devices To Switch<svg id=\"spinnerIcon1\" class=\"spinner-icon hidden\" xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 24 24\" fill=\"none\" stroke=\"currentColor\" stroke-width=\"2\"> <circle cx=\"12\" cy=\"12\" r=\"10\" stroke-opacity=\"0.25\"></circle> <path d=\"M12 2 A10 10 0 0 1 22 12\" stroke-linecap=\"round\"></path> </svg>";
		String TAB2_DROPDOWN ="<label for=\"dropdownMenu\">Choose an option:</label> <select id=\"dropdownMenu\" name=\"dropdownMenu\" onchange=\"saveSelectedValue('2', 0)\"> <option value=\"scene1\">SCENE_1</option> <option value=\"scene2\">SCENE_2</option> <option value=\"scene3\">SCENE_3</option> <option value=\"scene4\">SCENE_4</option> </select> <input type=\"hidden\" id=\"selectedValue\" name=\"selectedValue\" value=\"\">";
	#elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH)
		String  TAB2_BUTTON_NAME = "Add Devices To Group";
		String TAB2_DROPDOWN ="<label for=\"dropdownMenu\">Choose an option:</label> <select id=\"dropdownMenu\" name=\"dropdownMenu\" onchange=\"saveSelectedValue('2', 0)\"> <option value=\"scene1\">SWITCH_1 GROUP_1</option> <option value=\"scene2\">SWITCH_2 GROUP_2</option> <option value=\"scene3\">SWITCH_3 GROUP_3</option> <option value=\"scene4\">SWITCH_4 GROUP_4</option> </select> <input type=\"hidden\" id=\"selectedValue\" name=\"selectedValue\" value=\"\">";
	#elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH)
		String  TAB2_BUTTON_NAME = "Add Devices To Group";
		String TAB2_DROPDOWN ="<label for=\"dropdownMenu\">Choose an option:</label> <select id=\"dropdownMenu\" name=\"dropdownMenu\" onchange=\"saveSelectedValue('2', 0)\"> <option value=\"scene1\">SWITCH_1 SCENE_1</option> <option value=\"scene2\">SWITCH_2 SCENE_2</option> <option value=\"scene3\">SWITCH_3 SCENE_3</option> <option value=\"scene4\">SWITCH_4 SCENE_4</option> </select> <input type=\"hidden\" id=\"selectedValue\" name=\"selectedValue\" value=\"\">";	
	#endif

	String  TAB2_BUTTON_END = "</button></td>";

	String TAB2_UNBIND_BUTTON = "<td><button id=\"submit-button-2\" onclick=\"submitSelectedButtons('21')\" style=\"background-color: orange; color: white;\">";
	#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
		String  TAB2_UNBIND_BUTTON_NAME = "Unbind Devices From Switch<svg id=\"spinnerIcon2\" class=\"spinner-icon hidden\" xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 24 24\" fill=\"none\" stroke=\"currentColor\" stroke-width=\"2\"> <circle cx=\"12\" cy=\"12\" r=\"10\" stroke-opacity=\"0.25\"></circle> <path d=\"M12 2 A10 10 0 0 1 22 12\" stroke-linecap=\"round\"></path> </svg>";
	#elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH)
		String  TAB2_UNBIND_BUTTON_NAME = "Remove Devices From Group";
	#elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH)
		String  TAB2_UNBIND_BUTTON_NAME = "Remove Devices From Group";
	#endif
	#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
	String TAB2_UNBIND_BUTTON_END = "</button></td>  <td> <button id=\"submit-button-4\" onclick=\"submitSelectedButtons('23')\" style=\"background-color: orange; color: white;\">Refresh States<svg id=\"spinnerIcon4\" class=\"spinner-icon hidden\" xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 24 24\" fill=\"none\" stroke=\"currentColor\" stroke-width=\"2\"> <circle cx=\"12\" cy=\"12\" r=\"10\" stroke-opacity=\"0.25\"></circle> <path d=\"M12 2 A10 10 0 0 1 22 12\" stroke-linecap=\"round\"></path> </svg></button></td>  <td><button id=\"submit-button-3\" onclick=\"submitSelectedButtons('22')\" style=\"background-color: red; color: white;\">Clear All Records<svg id=\"spinnerIcon3\" class=\"spinner-icon hidden\" xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 24 24\" fill=\"none\" stroke=\"currentColor\" stroke-width=\"2\"> <circle cx=\"12\" cy=\"12\" r=\"10\" stroke-opacity=\"0.25\"></circle> <path d=\"M12 2 A10 10 0 0 1 22 12\" stroke-linecap=\"round\"></path> </svg></button></td>  </tr> </table></div>";
    #else
	String TAB2_UNBIND_BUTTON_END = "</button></td> </tr> </table></div>";
	#endif
	#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH)
	String TAB2_SCENE_BODY ="<div class=\"card2\"> <label for=\"sceneSelect\">Select Scene</label> <table> <tr> <td> <div class=\"container2 intensity-bar\"> <label for=\"intensityRange\">Brightness</label> <input type=\"range\" id=\"intensityRange\" min=\"0\" max=\"255\" value=\"0\" oninput=\"updateIntensityValue()\"> <span id=\"intensityValue\">0</span> </div> </td> <td> <div class=\"container2 switch\"> <label for=\"onOffSwitch\">On/Off</label> <label class=\"switch\"> <input type=\"checkbox\" id=\"onOffSwitch\" onchange=\"updateSwitchState()\"> <span class=\"slider\"></span> </label> </div> </td> </tr> </table> <div class=\"container\"> <table> <tr> <td><button class=\"add-scene-button\" onclick=\"sendSelectedScene('12')\" style=\"background-color: orange; color: white;\">Add Scene</button></td> <td><button class=\"add-scene-button\" onclick=\"sendSelectedScene('13')\" style=\"background-color: orange; color: white;\">Remove Scene</button></td> </tr> </table> </div> </div>";
    #endif
	String TAB2_END = "</div>";

#elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI)

	String TAB2_START ="<div id=\"Tab2\" class=\"card\">";
	// String CARD_GRID_START ="<div style=\"display: inline-block;\"> <h1>Set RGB Light</h1> <div class=\"grid-container\">";
    #if(TOTAL_ENDPOINTS <=4 && TOTAL_ENDPOINTS >= 1)
		#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)	
			String DEVICE1_SET_DMX_CH ="<h3>Select DMX Start Channel</h3> <select id=\"colorSelect\" name=\"colorSelect\"> <option value=\"\" disabled>-- Select --</option> </select>";
			String DEVICE1_COLOR_RGB ="<form action=\"/action\" method=\"post\" onsubmit=\"submitData(event, this, 0, 30)\"> <h3>Select your favorite color:</h3> <div class=\"form-row-1\"> <input type=\"color\" id=\"favcolor\" name=\"favcolor\" value=\"#ff0000\" class=\"color-picker\"> <input type=\"submit\" value=\"Submit\" class=\"stylish-button\"> </div> </form>";
			String DEVICE1_SET_GPIO_PIN ="<h3>Select GPIO Pin</h3> <select id=\"pinSelect\" name=\"pinSelect\"> <option value=\"\" disabled>-- Select --</option> </select>";
			String DEVICE1_SET_INTENSITY ="<h3>Set Brightness</h3><div class=\"slider-container-1\"> <input type=\"range\" min=\"1\" max=\"100\" value=\"";
			String DEVICE1_SET_INTENSITY_CONTINUE ="\" class=\"slider-1\" id=\"stepSlider1\"> <div class=\"thumb-label\">1</div> </div>";
		#else
			String DEVICE1_FORM_START  ="<div class=\"form-container\"> <h2>Set Group1 Lights</h2>";
			String DEVICE1_SCENE ="<div class=\"button-container\"> <div class=\"button-selector\" id=\"scene1_1\" onclick=\"selectScene(0, 1)\">Scene1</div> <div class=\"button-selector\" id=\"scene1_2\" onclick=\"selectScene(0, 2)\">Scene2</div> <div class=\"button-selector\" id=\"scene1_3\" onclick=\"selectScene(0, 3)\">Scene3</div> <div class=\"button-selector\" id=\"scene1_4\" onclick=\"selectScene(0, 4)\">Scene4</div> <div class=\"button-selector\" id=\"scene1_5\" onclick=\"selectScene(0, 5)\">Scene5</div> <div class=\"button-selector\" id=\"scene1_6\" onclick=\"selectScene(0, 6)\">Scene6</div> </div>";
			//char* DEVICE1_FORM = "<form id=\"inputForm0\" > <table> <tbody> <script> for (let row = 1; row <= 4; row++) { document.write(\"<tr>\"); for (let col = 1; col <= 5; col++) { document.write(`<td><input type=\"number\" id=\"input0_${row}${col}\" name=\"input${row}_${col}\" value=\"0\"></td>`); } document.write(\"</tr>\"); } </script> </tbody> </table> <br><hr class=\"stylish-line\"> <button type=\"button\" class=\"submit-button\" onclick=\"submitForm('10', 0)\">Assign Lights ID To Group1</button> <br><br> <button type=\"button\" id=\"getButton1\" class=\"submit-button\" onclick=\"handlePOST('11', 0, 0)\">Toggle All Lights In Group1</button><input type=\"number\" id=\"index\" name=\"index\" value=\"1\"  style=\"display: none;\"></form>";
			char* DEVICE1_FORM_S ="<form id=\"inputForm0\"> <table> <tbody> <script> const values1 = [";
			String  DEVICE1_FORM = "]; let index1 = 0; for (let row = 1; row <= 4; row++) { document.write(\"<tr>\"); for (let col = 1; col <= 5; col++) { document.write(`<td><input type=\"number\" id=\"input0_${row}${col}\" name=\"input${row}_${col}\" value=\"${values1[index1]}\"></td>`); index1++;} document.write(\"</tr>\"); } </script> </tbody> </table> <br><hr class=\"stylish-line\"> <button type=\"button\" class=\"submit-button\" onclick=\"submitForm('10', 0)\">Assign Lights ID To Group1</button> <br><br> <button type=\"button\" class=\"submit-button\" onclick=\"handlePOST('11', 0, 0)\">Toggle All Lights In Group1</button> </form>";
			String DEVICE1_COLOR_TEMP ="<div class=\"slider-container\"> <input type=\"range\" min=\"0\" max=\"9\" step=\"1\" value=\"";
			String DEVICE1_COLOR_TEMP_CONTINUE = "\" class=\"slider\" id=\"colorSlider2\"> <div class=\"scale\"> <div class=\"tick\" style=\"left: 2;\"></div> <div class=\"label\" style=\"left: 2%;\">2000K</div> <div class=\"tick\" style=\"left: 13.1%;\"></div> <div class=\"label\" style=\"left: 13.1%;\">2500K</div> <div class=\"tick\" style=\"left: 23.2%;\"></div> <div class=\"label\" style=\"left: 23.2%;\">3000K</div> <div class=\"tick\" style=\"left: 33.9%;\"></div> <div class=\"label\" style=\"left: 33.9%;\">3500K</div> <div class=\"tick\" style=\"left: 44.4%;\"></div> <div class=\"label\" style=\"left: 44.4%;\">4000K</div> <div class=\"tick\" style=\"left: 55.5%;\"></div> <div class=\"label\" style=\"left: 55.5%;\">4500K</div> <div class=\"tick\" style=\"left: 65.6%;\"></div> <div class=\"label\" style=\"left: 65.6%;\">5000K</div> <div class=\"tick\" style=\"left: 76.7%;\"></div> <div class=\"label\" style=\"left: 76.7%;\">5500K</div> <div class=\"tick\" style=\"left: 86.8%;\"></div> <div class=\"label\" style=\"left: 86.8%;\">6000K</div> <div class=\"tick\" style=\"left: 98%;\"></div> <div class=\"label\" style=\"left: 98%;\">6500K</div> </div> </div>";
			String DEVICE1_SET_INTENSITY ="<div class=\"slider-container-1\"> <input type=\"range\" min=\"1\" max=\"100\" value=\"";
			String DEVICE1_SET_INTENSITY_CONTINUE ="\" class=\"slider-1\" id=\"stepSlider1\"> <div class=\"thumb-label\">1";
			String DEVICE1_DIV_END = "</div>";
		#endif
    #endif
    #if(TOTAL_ENDPOINTS <=4 && TOTAL_ENDPOINTS >= 2)
		String DEVICE2_FORM_START  ="<div class=\"form-container\"> <h2>Set Group2 Lights</h2>";
		String DEVICE2_SCENE ="<div class=\"button-container\"> <div class=\"button-selector\" id=\"scene2_1\" onclick=\"selectScene(1, 1)\">Scene1</div> <div class=\"button-selector\" id=\"scene2_2\" onclick=\"selectScene(1, 2)\">Scene2</div> <div class=\"button-selector\" id=\"scene2_3\" onclick=\"selectScene(1, 3)\">Scene3</div> <div class=\"button-selector\" id=\"scene2_4\" onclick=\"selectScene(1, 4)\">Scene4</div> <div class=\"button-selector\" id=\"scene2_5\" onclick=\"selectScene(1, 5)\">Scene5</div> <div class=\"button-selector\" id=\"scene2_6\" onclick=\"selectScene(1, 6)\">Scene6</div> </div>";
		//char* DEVICE2_FORM = "<form id=\"inputForm1\"> <table> <tbody> <script> for (let row = 1; row <= 4; row++) { document.write(\"<tr>\"); for (let col = 1; col <= 5; col++) { document.write(`<td><input type=\"number\" id=\"input1_${row}${col}\" name=\"input${row}_${col}\" value=\"0\"></td>`); } document.write(\"</tr>\"); } </script> </tbody> </table> <br><hr class=\"stylish-line\"> <button type=\"button\" class=\"submit-button\" onclick=\"submitForm('10', 1)\">Assign Lights ID To Group2</button> <br><br> <button type=\"button\" id=\"getButton1\" class=\"submit-button\" onclick=\"handlePOST('11', 1, 0)\">Toggle All Lights In Group1</button><input type=\"number\" id=\"index\" name=\"index\" value=\"1\"  style=\"display: none;\"></form>";
		char* DEVICE2_FORM_S ="<form id=\"inputForm1\"> <table> <tbody> <script> const values2 = ["; 
		String DEVICE2_FORM ="]; let index2 = 0; for (let row = 1; row <= 4; row++) { document.write(\"<tr>\"); for (let col = 1; col <= 5; col++) { document.write(`<td><input type=\"number\" id=\"input1_${row}${col}\" name=\"input${row}_${col}\" value=\"${values2[index2]}\"></td>`); index2++;} document.write(\"</tr>\"); } </script> </tbody> </table> <br><hr class=\"stylish-line\"> <button type=\"button\" class=\"submit-button\" onclick=\"submitForm('10', 1)\">Assign Lights ID To Group2</button> <br><br> <button type=\"button\" class=\"submit-button\" onclick=\"handlePOST('11', 1, 0)\">Toggle All Lights In Group2</button> </form>";
		
		String DEVICE2_COLOR_TEMP ="<div class=\"slider-container\"> <input type=\"range\" min=\"0\" max=\"9\" step=\"1\" value=\"";
		String DEVICE2_COLOR_TEMP_CONTINUE ="\" class=\"slider\" id=\"colorSlider2\"> <div class=\"scale\"> <div class=\"tick\" style=\"left: 2;\"></div> <div class=\"label\" style=\"left: 2%;\">2000K</div> <div class=\"tick\" style=\"left: 13.1%;\"></div> <div class=\"label\" style=\"left: 13.1%;\">2500K</div> <div class=\"tick\" style=\"left: 23.2%;\"></div> <div class=\"label\" style=\"left: 23.2%;\">3000K</div> <div class=\"tick\" style=\"left: 33.9%;\"></div> <div class=\"label\" style=\"left: 33.9%;\">3500K</div> <div class=\"tick\" style=\"left: 44.4%;\"></div> <div class=\"label\" style=\"left: 44.4%;\">4000K</div> <div class=\"tick\" style=\"left: 55.5%;\"></div> <div class=\"label\" style=\"left: 55.5%;\">4500K</div> <div class=\"tick\" style=\"left: 65.6%;\"></div> <div class=\"label\" style=\"left: 65.6%;\">5000K</div> <div class=\"tick\" style=\"left: 76.7%;\"></div> <div class=\"label\" style=\"left: 76.7%;\">5500K</div> <div class=\"tick\" style=\"left: 86.8%;\"></div> <div class=\"label\" style=\"left: 86.8%;\">6000K</div> <div class=\"tick\" style=\"left: 98%;\"></div> <div class=\"label\" style=\"left: 98%;\">6500K</div> </div> </div>";
		String DEVICE2_SET_INTENSITY ="<div class=\"slider-container-1\"> <input type=\"range\" min=\"1\" max=\"100\" value=\"";
		String DEVICE2_SET_INTENSITY_CONTINUE ="\" class=\"slider-1\" id=\"stepSlider2\"> <div class=\"thumb-label\">1";
		String DEVICE2_DIV_END = "</div>";
    #endif
	 
    #if(TOTAL_ENDPOINTS <=4 && TOTAL_ENDPOINTS >= 3)
		String DEVICE3_FORM_START  ="<div class=\"form-container\"> <h2>Set Group3 Lights</h2>";
		String DEVICE3_SCENE ="<div class=\"button-container\"> <div class=\"button-selector\" id=\"scene3_1\" onclick=\"selectScene(2, 1)\">Scene1</div> <div class=\"button-selector\" id=\"scene3_2\" onclick=\"selectScene(2, 2)\">Scene2</div> <div class=\"button-selector\" id=\"scene3_3\" onclick=\"selectScene(2, 3)\">Scene3</div> <div class=\"button-selector\" id=\"scene3_4\" onclick=\"selectScene(2, 4)\">Scene4</div> <div class=\"button-selector\" id=\"scene3_5\" onclick=\"selectScene(2, 5)\">Scene5</div> <div class=\"button-selector\" id=\"scene3_6\" onclick=\"selectScene(2, 6)\">Scene6</div> </div>";
		char* DEVICE3_FORM_S ="<form id=\"inputForm2\"> <table> <tbody> <script> const values3 = [";
		String DEVICE3_FORM ="]; let index3 = 0; for (let row = 1; row <= 4; row++) { document.write(\"<tr>\"); for (let col = 1; col <= 5; col++) { document.write(`<td><input type=\"number\" id=\"input2_${row}${col}\" name=\"input${row}_${col}\" value=\"${values3[index3]}\"></td>`); index3++;} document.write(\"</tr>\"); } </script> </tbody> </table> <br><hr class=\"stylish-line\"> <button type=\"button\" class=\"submit-button\" onclick=\"submitForm('10', 2)\">Assign Lights ID To Group3</button> <br><br> <button type=\"button\" class=\"submit-button\" onclick=\"handlePOST('11', 2, 0)\">Toggle All Lights In Group3</button> </form>";		
		String DEVICE3_COLOR_TEMP ="<div class=\"slider-container\"> <input type=\"range\" min=\"0\" max=\"9\" step=\"1\" value=\"";
		String DEVICE3_COLOR_TEMP_CONTINUE ="\" class=\"slider\" id=\"colorSlider3\"> <div class=\"scale\"> <div class=\"tick\" style=\"left: 2;\"></div> <div class=\"label\" style=\"left: 2%;\">2000K</div> <div class=\"tick\" style=\"left: 13.1%;\"></div> <div class=\"label\" style=\"left: 13.1%;\">2500K</div> <div class=\"tick\" style=\"left: 23.2%;\"></div> <div class=\"label\" style=\"left: 23.2%;\">3000K</div> <div class=\"tick\" style=\"left: 33.9%;\"></div> <div class=\"label\" style=\"left: 33.9%;\">3500K</div> <div class=\"tick\" style=\"left: 44.4%;\"></div> <div class=\"label\" style=\"left: 44.4%;\">4000K</div> <div class=\"tick\" style=\"left: 55.5%;\"></div> <div class=\"label\" style=\"left: 55.5%;\">4500K</div> <div class=\"tick\" style=\"left: 65.6%;\"></div> <div class=\"label\" style=\"left: 65.6%;\">5000K</div> <div class=\"tick\" style=\"left: 76.7%;\"></div> <div class=\"label\" style=\"left: 76.7%;\">5500K</div> <div class=\"tick\" style=\"left: 86.8%;\"></div> <div class=\"label\" style=\"left: 86.8%;\">6000K</div> <div class=\"tick\" style=\"left: 98%;\"></div> <div class=\"label\" style=\"left: 98%;\">6500K</div> </div> </div>";
		String DEVICE3_SET_INTENSITY ="<div class=\"slider-container-1\"> <input type=\"range\" min=\"1\" max=\"100\" value=\"";
		String DEVICE3_SET_INTENSITY_CONTINUE ="\" class=\"slider-1\" id=\"stepSlider3\"> <div class=\"thumb-label\">1";
		String DEVICE3_DIV_END = "</div>";
    #endif

	#if(TOTAL_ENDPOINTS == 4)
		String DEVICE4_FORM_START  ="<div class=\"form-container\"> <h2>Set Group4 Lights</h2>";
		String DEVICE4_SCENE ="<div class=\"button-container\"> <div class=\"button-selector\" id=\"scene4_1\" onclick=\"selectScene(3, 1)\">Scene1</div> <div class=\"button-selector\" id=\"scene4_2\" onclick=\"selectScene(3, 2)\">Scene2</div> <div class=\"button-selector\" id=\"scene4_3\" onclick=\"selectScene(3, 3)\">Scene3</div> <div class=\"button-selector\" id=\"scene4_4\" onclick=\"selectScene(3, 4)\">Scene4</div> <div class=\"button-selector\" id=\"scene4_5\" onclick=\"selectScene(3, 5)\">Scene5</div> <div class=\"button-selector\" id=\"scene4_6\" onclick=\"selectScene(3, 6)\">Scene6</div> </div>";
		//char* DEVICE4_FORM = "<form id=\"inputForm3\"> <table> <tbody> <script> for (let row = 1; row <= 4; row++) { document.write(\"<tr>\"); for (let col = 1; col <= 5; col++) { document.write(`<td><input type=\"number\" id=\"input3_${row}${col}\" name=\"input${row}_${col}\" value=\"0\"></td>`); } document.write(\"</tr>\"); } </script> </tbody> </table> <br><hr class=\"stylish-line\"> <button type=\"button\" class=\"submit-button\" onclick=\"submitForm('10', 3)\">Assign Lights ID To Group4</button> <br><br> <button type=\"button\" id=\"getButton1\" class=\"submit-button\" onclick=\"handlePOST('11', 3, 0)\">Toggle All Lights In Group1</button><input type=\"number\" id=\"index\" name=\"index\" value=\"1\"  style=\"display: none;\"></form>";
		char* DEVICE4_FORM_S ="<form id=\"inputForm3\"> <table> <tbody> <script> const values4 = [";
		String DEVICE4_FORM = "]; let index4 = 0; for (let row = 1; row <= 4; row++) { document.write(\"<tr>\"); for (let col = 1; col <= 5; col++) { document.write(`<td><input type=\"number\" id=\"input3_${row}${col}\" name=\"input${row}_${col}\" value=\"${values4[index4]}\"></td>`); index4++;} document.write(\"</tr>\"); } </script> </tbody> </table> <br><hr class=\"stylish-line\"> <button type=\"button\" class=\"submit-button\" onclick=\"submitForm('10', 3)\">Assign Lights ID To Group4</button> <br><br> <button type=\"button\" class=\"submit-button\" onclick=\"handlePOST('11', 3, 0)\">Toggle All Lights In Group4</button> </form>";
		String DEVICE4_COLOR_TEMP ="<div class=\"slider-container\"> <input type=\"range\" min=\"0\" max=\"9\" step=\"1\" value=\"";
		String DEVICE4_COLOR_TEMP_CONTINUE ="\" class=\"slider\" id=\"colorSlider4\"> <div class=\"scale\"> <div class=\"tick\" style=\"left: 2;\"></div> <div class=\"label\" style=\"left: 2%;\">2000K</div> <div class=\"tick\" style=\"left: 13.1%;\"></div> <div class=\"label\" style=\"left: 13.1%;\">2500K</div> <div class=\"tick\" style=\"left: 23.2%;\"></div> <div class=\"label\" style=\"left: 23.2%;\">3000K</div> <div class=\"tick\" style=\"left: 33.9%;\"></div> <div class=\"label\" style=\"left: 33.9%;\">3500K</div> <div class=\"tick\" style=\"left: 44.4%;\"></div> <div class=\"label\" style=\"left: 44.4%;\">4000K</div> <div class=\"tick\" style=\"left: 55.5%;\"></div> <div class=\"label\" style=\"left: 55.5%;\">4500K</div> <div class=\"tick\" style=\"left: 65.6%;\"></div> <div class=\"label\" style=\"left: 65.6%;\">5000K</div> <div class=\"tick\" style=\"left: 76.7%;\"></div> <div class=\"label\" style=\"left: 76.7%;\">5500K</div> <div class=\"tick\" style=\"left: 86.8%;\"></div> <div class=\"label\" style=\"left: 86.8%;\">6000K</div> <div class=\"tick\" style=\"left: 98%;\"></div> <div class=\"label\" style=\"left: 98%;\">6500K</div> </div> </div>";
		String DEVICE4_SET_INTENSITY ="<div class=\"slider-container-1\"> <input type=\"range\" min=\"1\" max=\"100\" value=\"";
		String DEVICE4_SET_INTENSITY_CONTINUE ="\" class=\"slider-1\" id=\"stepSlider4\"> <div class=\"thumb-label\">1";
		String DEVICE4_DIV_END = "</div>";
    #endif
	String CARD_GRID_END = "</div></div>";
	String TAB2_END = "</div>";

	String TAB3_START ="<div id=\"Tab3\" class=\"card\">";
	#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
		String TAB3_BODY ="<h1>UART Configuration Form</h1> <form id=\"uartConfigForm\" action=\"/action\" method=\"post\" onsubmit=\"submitData(event, this, 0, 31)\"> <div> <label for=\"baudRate\">Baud Rate:</label> <input type=\"number\" id=\"baudRate\" name=\"baud_rate\" value=\"250000\" min=\"300\" max=\"9216000\" step=\"1\"> </div> <div> <label for=\"dataBits\">Data Bits:</label> <select id=\"dataBits\" name=\"data_bits\"> <option value=\"UART_DATA_5_BITS\">5 bits</option> <option value=\"UART_DATA_6_BITS\">6 bits</option> <option value=\"UART_DATA_7_BITS\">7 bits</option> <option value=\"UART_DATA_8_BITS\" selected>8 bits</option> <option value=\"UART_DATA_BITS_MAX\">9 bits</option> </select> </div> <div> <label for=\"parity\">Parity:</label> <select id=\"parity\" name=\"parity\"> <option value=\"UART_PARITY_DISABLE\" selected>No Parity</option> <option value=\"UART_PARITY_EVEN\">Even Parity</option> <option value=\"UART_PARITY_ODD\">Odd Parity</option> </select> </div> <div> <label for=\"stopBits\">Stop Bits:</label> <select id=\"stopBits\" name=\"stop_bits\"> <option value=\"UART_STOP_BITS_1\">1 bit</option> <option value=\"UART_STOP_BITS_1_5\">1.5 bits</option> <option value=\"UART_STOP_BITS_2\" selected>2 bits</option> </select> </div> <div> <label for=\"flowControl\">Flow Control:</label> <select id=\"flowControl\" name=\"flow_ctrl\"> <option value=\"UART_HW_FLOWCTRL_DISABLE\" selected>Disable</option> <option value=\"UART_HW_FLOWCTRL_RTS\">RTS Only</option> <option value=\"UART_HW_FLOWCTRL_CTS\">CTS Only</option> <option value=\"UART_HW_FLOWCTRL_CTS_RTS\">RTS+CTS</option> <option value=\"UART_HW_FLOWCTRL_MAX\">Max</option> </select> </div> <input type=\"submit\" value=\"Apply Configuration\"> </form>";
	#else
	String TAB3_START_DALI ="<form action=\"/action\" method=\"post\" onsubmit=\"submitData(event, this, 0, 12)\"> <div class=\"form-group\"> <h3>Max Lights to Assign Address</h3> <button type=\"submit\" class=\"stylish-button\" style=\"width: 300px;\">Start Assign Dali Addresses</button> <input type=\"number\" id=\"max_addr\" name=\"max_addr\" value=\"";
	String TAB3_START_DALI_CONTINUE="\" placeholder=\"Total Addr\" min=\"0\" max=\"63\" style=\"width: 120px;\"> </div> </form>";
	String TAB3_SHOW_DALI ="<div> <form action=\"/action_rx\" method=\"get\" onsubmit=\"handleSubmit(event)\" id=\"myFormGet\"> <div class=\"form-group\"> <button type=\"submit\" class=\"stylish-button\" style=\"width: 300px;\">Show All DALI Assigned IDs</button> <input class=\"hidden\" type=\"number\" id=\"query\" name=\"max_addr\" value=\"10\" placeholder=\"Get All\" min=\"0\" max=\"63\" style=\"width: 120px; display: none;\"> </div> </form> <div id=\"tableContainer\"></div> </div>";
	#endif
	String TAB3_END = "</div>";

	String TAB4_START ="<div id=\"Tab4\" class=\"card\">";
	String TAB4_BODY ="<div id=\"group1Card\"> <label for=\"groupSelector\">Select Group:</label> <form id=\"SettingsForm\" action=\"/action\" method=\"post\" onsubmit=\"submitData(event, this, 1, 13)\"> <select id=\"groupSelector\" id=\"index\" name=\"index\"> <option value=\"1\">Group1</option> <option value=\"2\">Group2</option> <option value=\"3\">Group3</option> <option value=\"4\">Group4</option> </select> <div class=\"row\"> <label>Scene1</label> <input type=\"checkbox\" id=\"scene1_sta\" name=\"scene1_sta\"> <input type=\"range\" min=\"0\" max=\"254\" value=\"254\" id=\"scene1_val\" name=\"scene1_val\"> <input type=\"number\" min=\"2000\" max=\"6500\" value=\"2000\" step=\"100\" id=\"scene1_int\" name=\"scene1_int\"> <input type=\"color\" value=\"#ffffff\" class=\"color-input\" id=\"scene1_col\" name=\"scene1_col\"> </div> <!-- Additional Scene Rows --> <div class=\"row\"> <label>Scene2</label> <input type=\"checkbox\" id=\"scene2_sta\" name=\"scene2_sta\"> <input type=\"range\" min=\"0\" max=\"254\" value=\"254\" id=\"scene2_val\" name=\"scene2_val\"> <input type=\"number\" min=\"2000\" max=\"6500\" value=\"2000\" step=\"100\" id=\"scene2_int\" name=\"scene2_int\"> <input type=\"color\" value=\"#ffffff\" class=\"color-input\" id=\"scene2_col\" name=\"scene2_col\"> </div> <div class=\"row\"> <label>Scene3</label> <input type=\"checkbox\" id=\"scene3_sta\" name=\"scene3_sta\"> <input type=\"range\" min=\"0\" max=\"254\" value=\"254\" id=\"scene3_val\" name=\"scene3_val\"> <input type=\"number\" min=\"2000\" max=\"6500\" value=\"2000\" step=\"100\" id=\"scene3_int\" name=\"scene3_int\"> <input type=\"color\" value=\"#ffffff\" class=\"color-input\" id=\"scene3_col\" name=\"scene3_col\"> </div> <div class=\"row\"> <label>Scene4</label> <input type=\"checkbox\" id=\"scene4_sta\" name=\"scene4_sta\"> <input type=\"range\" min=\"0\" max=\"254\" value=\"254\" id=\"scene4_val\" name=\"scene4_val\"> <input type=\"number\" min=\"2000\" max=\"6500\" value=\"2000\" step=\"100\" id=\"scene4_int\" name=\"scene4_int\"> <input type=\"color\" value=\"#ffffff\" class=\"color-input\" id=\"scene4_col\" name=\"scene4_col\"> </div> <div class=\"row\"> <label>Scene5</label> <input type=\"checkbox\" id=\"scene5_sta\" name=\"scene5_sta\"> <input type=\"range\" min=\"0\" max=\"254\" value=\"254\" id=\"scene5_val\" name=\"scene5_val\"> <input type=\"number\" min=\"2000\" max=\"6500\" value=\"2000\" step=\"100\" id=\"scene5_int\" name=\"scene5_int\"> <input type=\"color\" value=\"#ffffff\" class=\"color-input\" id=\"scene5_col\" name=\"scene5_col\"> </div> <div class=\"row\"> <label>Scene6</label> <input type=\"checkbox\" id=\"scene6_sta\" name=\"scene6_sta\"> <input type=\"range\" min=\"0\" max=\"254\" value=\"254\" id=\"scene6_val\" name=\"scene6_val\"> <input type=\"number\" min=\"2000\" max=\"6500\" value=\"2000\" step=\"100\" id=\"scene6_int\" name=\"scene6_int\"> <input type=\"color\" value=\"#ffffff\" class=\"color-input\" id=\"scene6_col\" name=\"scene6_col\"> </div> <button class=\"submit-btn\">Submit</button><br> </form> <br> <button class=\"delete-btn\" id=\"deleteGroup\" >Delete Group</button><br> <br> <button class=\"delete-btn\" id=\"deleteScenes\" >Delete All Scenes</button><br> <br> <button class=\"delete-btn\" id=\"deleteAllDevices\" >Delete All Devices in this Group</button><br> </div>";
	String TAB4_END = "</div>";

	/////////////SCRIPTS///////////
	String SCRIPT_HANDLE_SUBMIT ="function handleSubmit(event) { event.preventDefault(); const query = document.getElementById('query').value; fetch(`/action_rx?query=${encodeURIComponent(query)}`) .then(response => response.text()) .then(data => { console.log(data); const jsonObj = JSON.parse(data); let tableHTML = jsonToTable(jsonObj); document.getElementById('tableContainer').innerHTML = tableHTML; }) .catch(error => { console.error('Error:', error); }); } document.getElementById('myFormGet').addEventListener('submit', handleSubmit);";

	String SCRIPT_HANDLE_POST ="function handlePOST(_fxn, _index, _value){ const jsonData = { fxn: _fxn, index: _index, value: _value }; const xhr = new XMLHttpRequest(); xhr.open(\"POST\", \"/action\", true); xhr.setRequestHeader(\"Content-Type\", \"application/json\"); xhr.onreadystatechange = function() { if (xhr.readyState === XMLHttpRequest.DONE) { if (xhr.status === 200) { console.log(\"Request successful\"); } else { console.error(\"Request failed\"); } } }; xhr.send(JSON.stringify(jsonData)); }";
	
	String SCRIPT_SHOW_TABLE = "function jsonToTable(jsonObj) { const headers = Object.keys(jsonObj[0]); let table = `<table><tr>`; headers.forEach(header => table += `<th>${header}</th>`); table += `</tr>`; jsonObj.forEach(row => { table += `<tr>`; headers.forEach(header => table += `<td>${row[header]}</td>`); table += `</tr>`; }); table += `</table>`; return table; }";

	String SCRIPT_COLOR_TEMPERATURE ="const sliders = document.querySelectorAll('.slider'); const labels = document.querySelectorAll('.label'); sliders.forEach((slider, index) => { slider.addEventListener('input', function() { const value = this.value; const color = getColor(value); this.style.background = `linear-gradient(to right, rgb(${color.r}, ${color.g}, ${color.b}), rgb(220, 229, 255))`; temp = 2000 + (value * 500); console.log(temp); handlePOST('16', index, temp); updateLabels(value, index); }); }); function updateLabels(value, index) { const labelsArray = Array.from(labels); const startIndex = index * 10; const endIndex = startIndex + 10; labelsArray.slice(startIndex, endIndex).forEach((label, innerIndex) => { const percentage = 2 + (innerIndex * 96) / 9; label.style.left = `${percentage}%`; }); } function getColor(value) { const warmWhite = { r: 255, g: 138, b: 18 }; const coolWhite = { r: 220, g: 229, b: 255 }; const ratio = (value * 1000) / 6500; const r = Math.round((1 - ratio) * warmWhite.r + ratio * coolWhite.r); const g = Math.round((1 - ratio) * warmWhite.g + ratio * coolWhite.g); const b = Math.round((1 - ratio) * warmWhite.b + ratio * coolWhite.b); return { r, g, b }; }";

	String SCRIPT_SET_INTENSITY ="document.addEventListener('DOMContentLoaded', function() { const slider = document.getElementById('stepSlider1'); const sliders_intensity = document.querySelectorAll(\".slider-container-1\"); sliders_intensity.forEach((sliderContainer, _index) => { const slider1 = sliderContainer.querySelector(\".slider-1\"); const thumbLabel = sliderContainer.querySelector(\".thumb-label\"); slider1.addEventListener(\"input\", function() { const _value = parseInt(this.value); const thumb = this.querySelector(\".slider-1::-webkit-slider-thumb\"); thumbLabel.textContent = _value; thumbLabel.style.display = \"block\"; thumbLabel.style.left = `calc(${_value * 100 / parseInt(this.max)}% - ${thumbLabel.offsetWidth / 2}px)`; handlePOST('17', _index, _value); }); }); slider.dispatchEvent(new Event('change')); });";
    String SCRIPT_UPDATE_INTENSITY ="function updateSlider(newValue) { const slider = document.getElementById('stepSlider1'); slider.value = newValue; slider.dispatchEvent(new Event('change')); }";
	String SCRIPT_UPDATE_INTENSITY_VAL_START ="updateSlider(";
	String SCRIPT_UPDATE_INTENSITY_VAL_SET ="20";
	String SCRIPT_UPDATE_INTENSITY_VAL_END =");";

	String SCRIPT_SCENES_TAB ="document.getElementById('deleteGroup').addEventListener('click', function() { console.error('deleteGroup'); handlePOST('50', 0, 0); }); document.getElementById('deleteScenes').addEventListener('click', function() { console.error('deleteScenes'); handlePOST('50', 1, 0); }); document.getElementById('deleteAllDevices').addEventListener('click', function() { console.error('deleteAllDevices'); handlePOST('50', 2, 0); });";

	String SCRIPT_SCENES_SELECTOR ="const maxButtons = 6; function selectScene(_index, _scene) { const buttons = document.querySelectorAll('.button-selector'); buttons.forEach(button => { const num = scene.toString(); if (button.id === `scene${index+1}_${scene}`) { button.classList.add('selected'); } else { button.classList.remove('selected'); } }); handlePOST('18', _index, _scene); }";
    String SCRIPT_CHECK_DUPLICATES ="function checkDuplicates(input, index){}";

    String SCRIPT_SUBMIT_FORM ="function submitForm(fxn, formIndex) { let inputData = { \"fxn\": fxn, \"index\": formIndex, \"values\": [] }; for (let row = 1; row <= 4; row++) { for (let col = 1; col <= 5; col++) { let inputId = `input${formIndex}_${row}${col}`; let inputValue = document.getElementById(inputId).value; inputData.values.push(parseInt(inputValue)); } }  let jsonData = JSON.stringify(inputData); fetch('/action', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: jsonData }) .then(response => response.json()) .then(data => { console.log('Success:', data); }) .catch((error) => { console.error('Error:', error); }); }";

	String SCRIPT_FORM_ARRAY_VALUES ="const newValues = [ {\"input1_1\":1, \"input1_2\":2, \"input1_3\":3, \"input1_4\":4, \"input1_5\":5, \"input2_1\":0, \"input2_2\":0, \"input2_3\":0, \"input2_4\":0, \"input2_5\":0, \"input3_1\":0, \"input3_2\":0, \"input3_3\":0, \"input3_4\":0, \"input3_5\":0, \"input4_1\":0, \"input4_2\":0, \"input4_3\":0, \"input4_4\":0, \"input4_5\":0}, {\"input1_1\":1, \"input1_2\":2, \"input1_3\":3, \"input1_4\":4, \"input1_5\":5, \"input2_1\":0, \"input2_2\":0, \"input2_3\":0, \"input2_4\":0, \"input2_5\":0, \"input3_1\":0, \"input3_2\":0, \"input3_3\":0, \"input3_4\":0, \"input3_5\":0, \"input4_1\":0, \"input4_2\":0, \"input4_3\":0, \"input4_4\":0, \"input4_5\":0},{\"input1_1\":1, \"input1_2\":2, \"input1_3\":3, \"input1_4\":4, \"input1_5\":5, \"input2_1\":0, \"input2_2\":0, \"input2_3\":0, \"input2_4\":0, \"input2_5\":0, \"input3_1\":0, \"input3_2\":0, \"input3_3\":0, \"input3_4\":0, \"input3_5\":0, \"input4_1\":0, \"input4_2\":0, \"input4_3\":0, \"input4_4\":0, \"input4_5\":0}, {\"input1_1\":1, \"input1_2\":2, \"input1_3\":3, \"input1_4\":4, \"input1_5\":5, \"input2_1\":0, \"input2_2\":0, \"input2_3\":0, \"input2_4\":0, \"input2_5\":0, \"input3_1\":0, \"input3_2\":0, \"input3_3\":0, \"input3_4\":0, \"input3_5\":0, \"input4_1\":0, \"input4_2\":0, \"input4_3\":0, \"input4_4\":0, \"input4_5\":0} ];";
    String SCRIPT_ARRAY_START = "const newValues =";
	String SCRIPT_UPDATE_IP_FIELDS ="function updateInputFields(formIndex, values) { for (let row = 1; row <= 4; row++) { for (let col = 1; col <= 5; col++) { let inputId = `input${formIndex}_${row}${col}`; let valueKey = `input${row}_${col}`; document.getElementById(inputId).value = values[valueKey] || 0; } } }";

#elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH)

#endif
/******************SCRIPT************************/
String SCRIPT_START ="<script>";
String SCRIPT_SHOW_WIFI_NWKS ="function showWifiNetworks(networks) { var wifiContainer = document.getElementById(\"wifiContainer\"); wifiContainer.innerHTML = ''; networks.forEach(function(network) { var wifiNetwork = document.createElement(\"div\"); wifiNetwork.classList.add(\"wifi-network\"); var radioBtn = document.createElement(\"input\"); radioBtn.type = \"radio\"; radioBtn.name = \"wifiOptions\"; radioBtn.value = network.name; radioBtn.addEventListener(\"change\", function() { document.getElementById(\"ssid\").value = this.value; }); var wifiName = document.createElement(\"span\"); wifiName.textContent = network.name; var wifiRangeIcon = document.createElement(\"div\"); wifiRangeIcon.classList.add(\"wifi-range-icon\"); wifiRangeIcon.style.backgroundColor = getRangeColor(network.range); wifiNetwork.appendChild(radioBtn); wifiNetwork.appendChild(wifiName); wifiNetwork.appendChild(wifiRangeIcon); wifiContainer.appendChild(wifiNetwork); }); } function getRangeColor(range) { if (range === 'Excellent') { return '#3498db'; } else if (range === 'Good') { return '#2ecc71'; } else { return '#e74c3c'; } } var availableNetworks = [ { name: 'Network1', range: 'Excellent' }, { name: 'Network2', range: 'Good' }, { name: 'Network3', range: 'Fair' }, ]; ";
String SCRIPT_FETCH_INITIAL_DATA ="function convertToAvailableNetworks(data) { const formattedData = data.map(item => { return { name: item.name, range: item.range }; }); return formattedData; } function fetchInitialData() { fetch(`/items?fxn=0`) .then(response => { if (!response.ok) { throw new Error('Network response was not ok'); } return response.json(); }) .then(data => { console.log('Received data:', data); const availableNetworks = convertToAvailableNetworks(data); showWifiNetworks(availableNetworks); }) .catch(error => { console.error('Error during fetch operation:', error); }); } ";

String SCRIPT_OPEN_TAB ="function openTab(event, tabId) { event = event || window.event; var target = event.target || event.srcElement; var tabContents = document.querySelectorAll('.card'); tabContents.forEach(function(content) { content.classList.remove('active'); }); var tabs = document.querySelectorAll('.tab'); tabs.forEach(function(tab) { tab.classList.remove('active'); }); document.getElementById(tabId).classList.add('active'); target.classList.add('active'); }";

String SCRIPT_SUBMIT_DATA ="function submitData(event, formElement, index, fxn) { event.preventDefault(); let formData = new FormData(formElement); formData.append('fxn', fxn); let jsonData = {}; for (let [key, value] of formData.entries()) { jsonData[key] = value; } var json_data = JSON.stringify(jsonData); console.log(json_data); var xhr = new XMLHttpRequest(); xhr.open(\"POST\", \"/action\", true); xhr.setRequestHeader(\"Content-Type\", \"application/json\"); xhr.onreadystatechange = function() { if (xhr.readyState === XMLHttpRequest.DONE) { if (xhr.status === 200) { console.log(\"Request successful\"); } else { console.error(\"Request failed\"); } } }; xhr.send(json_data); }";

#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
	String SCRIPT_SUBMIT_AC_MODEL ="function validateAcModel(fxn) { var form = document.forms[\"acForm\"]; var selectElement = form[\"ac_model\"]; var selectedIndex = selectElement.selectedIndex; var selectedValue = selectElement.value; var data = { \"fxn\": fxn, \"ac_index\": selectedIndex, \"ac_model\": selectedValue }; var json_data = JSON.stringify(data); console.log(json_data);  var xhr = new XMLHttpRequest(); xhr.open(\"POST\", \"/action\", true);xhr.setRequestHeader(\"Content-Type\", \"application/json\");xhr.onreadystatechange = function() {if (xhr.readyState === XMLHttpRequest.DONE) {if (xhr.status === 200) {console.log(\"Request successful\");} else {console.error(\"Request failed\");}}};xhr.send(json_data); }";
	String SCRIPT_SELECT_AC_MODEL ="function selectACModel(index) { var acModelDropdown = document.getElementById(\"ac-model\"); if (index >= 0 && index < acModelDropdown.options.length) { acModelDropdown.selectedIndex = index; } else { console.error(\"Invalid index: \" + index); } validateAcModel(); }";
#elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
    String SCRIPT_GLOBAL_ARRAY = "var toggle_val=0; let selected_index= 0; let globalItemsArray = ";
    String SCRIPT_SEND_SELECTED_ITEMS ="function sendSelectedItem(_fxn, itemName) { const jsonData = { fxn: _fxn, name: itemName }; const xhr = new XMLHttpRequest(); xhr.open(\"POST\", \"/action\", true); xhr.setRequestHeader(\"Content-Type\", \"application/json\"); xhr.onreadystatechange = function() { if (xhr.readyState === XMLHttpRequest.DONE) { if (xhr.status === 200) { console.log(\"Request successful\"); } else { console.error(\"Request failed\"); } } }; xhr.send(JSON.stringify(jsonData)); }";

	//String SCRIPT_FETCH_ITEMS =" function fetchItems(fxn, scene_id, erase_data) { const fetchButton = document.getElementById('fetch-button'); fetchButton.disabled = true; fetchButton.style.backgroundColor = 'gray'; document.getElementById('loading-spinner').style.display = 'block'; try { console.log('On Fetch'); const response =  fetch(`/items?fxn=${fxn}&scene_id=${scene_id}&erase_data=${erase_data}`); const items =  response.json(); globalItemsArray.length = 0; items.forEach(item => { globalItemsArray.push(item); }); document.getElementById('loading-spinner').style.display = 'none'; fetchButton.disabled = false; fetchButton.style.backgroundColor = ''; if(erase_data==1) globalItemsArray={}; displayItems(globalItemsArray, scene_id); } catch (error) { console.error('Error fetching items:', error); } }";
	//String SCRIPT_FETCH_ITEMS ="async function fetchItems(fxn, scene_id, erase_data) { const fetchButton = document.getElementById('fetch-button'); fetchButton.disabled = true; fetchButton.style.backgroundColor = 'gray'; document.getElementById('loading-spinner').style.display = 'block'; const fetchWithTimeout = (url, timeout = 10000) => { return Promise.race([ fetch(url).then(response => response.json()), new Promise((_, reject) => setTimeout(() => reject(new Error('Request timed out')), timeout) ) ]); }; try { console.log('On Fetch'); const items = await fetchWithTimeout(`/items?fxn=${fxn}&scene_id=${scene_id}&erase_data=${erase_data}`); globalItemsArray.length = 0; items.forEach(item => globalItemsArray.push(item)); document.getElementById('loading-spinner').style.display = 'none'; fetchButton.disabled = false; fetchButton.style.backgroundColor = ''; if (erase_data == 1) globalItemsArray = []; displayItems(globalItemsArray, scene_id); } catch (error) { console.error('Error fetching items:', error); alert('Failed to fetch items. Please try again.'); document.getElementById('loading-spinner').style.display = 'none'; fetchButton.disabled = false; fetchButton.style.backgroundColor = ''; } }";
	String SCRIPT_FETCH_ITEMS ="async function fetchItems(fxn, scene_id, erase_data) { const fetchButton = document.getElementById('fetch-button'); fetchButton.disabled = true; fetchButton.style.backgroundColor = 'gray'; document.getElementById('loading-spinner').style.display = 'block'; const fetchWithTimeout = (url, timeout = 180000) => { return Promise.race([ fetch(url).then(response => response.json()), new Promise((_, reject) => setTimeout(() => reject(new Error('Request timed out')), timeout) ) ]); }; try { console.log('On Fetch'); const items = await fetchWithTimeout(`/items?fxn=${fxn}&scene_id=${scene_id}&erase_data=${erase_data}`); globalItemsArray.length = 0; items.forEach(item => globalItemsArray.push(item)); document.getElementById('loading-spinner').style.display = 'none'; fetchButton.disabled = false; fetchButton.style.backgroundColor = ''; if (erase_data == 1) globalItemsArray = []; displayItems(globalItemsArray, scene_id); } catch (error) { console.error('Error fetching items:', error); alert('Failed to fetch items. Please try again.'); document.getElementById('loading-spinner').style.display = 'none'; fetchButton.disabled = false; fetchButton.style.backgroundColor = ''; } }";
	//String SCRIPT_FETCH_ITEMS ="async function fetchItems(fxn, scene_id, erase_data) { const fetchButton = document.getElementById('fetch-button'); const loadingSpinner = document.getElementById('loading-spinner'); try { fetchButton.disabled = true; fetchButton.style.backgroundColor = 'gray'; loadingSpinner.style.display = 'block'; if (typeof fxn !== 'string' || !scene_id) { throw new Error('Invalid parameters'); } const params = new URLSearchParams({ fxn: fxn, scene_id: scene_id, erase_data: erase_data ? '1' : '0' }); const url = new URL(`/items`, window.location.origin); url.search = params.toString(); console.log('Fetching from:', url.toString()); const response = await fetch(url, { method: 'GET', headers: { 'Content-Type': 'application/json', }, credentials: 'include' // If using cookies }); if (!response.ok) { const errorBody = await response.text(); throw new Error(`HTTP error! status: ${response.status} - ${errorBody}`); } const items = await response.json(); globalItemsArray = erase_data ? [] : [...globalItemsArray]; globalItemsArray.push(...items); displayItems(globalItemsArray, scene_id); } catch (error) { console.error('Fetch error:', { error: error.message, stack: error.stack, timestamp: new Date().toISOString() }); alert(`Error: ${error.message}\\nPlease check the console for details.`); } finally { fetchButton.disabled = false; fetchButton.style.backgroundColor = ''; loadingSpinner.style.display = 'none'; } }";


	String SCRIPT_FETCH_SELECTED_ITEMS ="function saveSelectedValue(fxn, erase_data) { var dropdown = document.getElementById(\"dropdownMenu\"); var selectedValue = dropdown.options[dropdown.selectedIndex].value; document.getElementById(\"selectedValue\").value = selectedValue; selected_id = dropdown.selectedIndex; fetchItems(fxn, dropdown.selectedIndex, erase_data); }";
	String SCRIPT_CONFIRMATION_DIALOG ="function showConfirmationDialog() { document.getElementById(\"overlay\").style.display = \"block\"; document.getElementById(\"confirmationDialog\").style.display = \"block\"; } function closeDialog() { document.getElementById(\"overlay\").style.display = \"none\"; document.getElementById(\"confirmationDialog\").style.display = \"none\"; } function reloadNodes(confirm) { closeDialog(); saveSelectedValue('1', confirm); }";
	
	//String SCRIPT_SET_BUTTONS ="function submitSelectedButtons(fxn) { const button1 = document.getElementById('submit-button-1'); const button2 = document.getElementById('submit-button-2'); const button3 = document.getElementById('submit-button-3'); const button4 = document.getElementById('submit-button-4'); const spinnerIcon1 = document.getElementById('spinnerIcon1'); const spinnerIcon2 = document.getElementById('spinnerIcon2'); const spinnerIcon3 = document.getElementById('spinnerIcon3'); const spinnerIcon4 = document.getElementById('spinnerIcon4'); if (fxn == '11') { spinnerIcon1.classList.remove('hidden'); button1.disabled = true; } else if (fxn == '21') { spinnerIcon2.classList.remove('hidden'); button2.disabled = true; } else if (fxn == '22') { spinnerIcon3.classList.remove('hidden'); button3.disabled = true; } else if (fxn == '23') { spinnerIcon4.classList.remove('hidden'); button4.disabled = true; } const selectedItemsText = Array.from(document.querySelectorAll('.bind-checkbox:checked')).map(checkbox => { const groupContainer = checkbox.closest('.group-container'); const shortAddressHex = groupContainer ? JSON.parse(groupContainer.title).short : null; return { short: shortAddressHex}; }); console.log(\"selectedItemsText:\", JSON.stringify(selectedItemsText, null, 2)); const selectedShorts = selectedItemsText .map(item => item.short ? parseInt(item.short, 16) : null) .filter(num => num !== null); console.log(\"Converted selected shorts to numbers:\", selectedShorts); let jsonData = { fxn: fxn, option: document.getElementById(\"dropdownMenu\").selectedIndex }; if (fxn !== '22' && fxn !== '23') { const selectedItems = globalItemsArray.filter(item => { const itemShort = Number(item.short); console.log(`Checking item: short=${itemShort} (Hex: 0x${itemShort.toString(16).toUpperCase()})`); return selectedShorts.includes(itemShort) && item.state === 1; }); console.log(\"Filtered selectedItems:\", selectedItems); jsonData.selected_items = selectedItems; } var json_data = JSON.stringify(jsonData); const response = fetch(\"/action\", { method: \"POST\", headers: { \"Content-Type\": \"application/json\" }, body: json_data }); if (response.ok) { console.log(\"Request successful\"); alert('Request Successful!!'); } else { console.error(\"Request failed\"); alert('Request Incomplete, Try Again!!'); } if (fxn == '11') { spinnerIcon1.classList.add('hidden'); button1.disabled = false; } else if (fxn == '21') { spinnerIcon2.classList.add('hidden'); button2.disabled = false; } else if (fxn == '22') { spinnerIcon3.classList.add('hidden'); button3.disabled = false; } else if (fxn == '23') { spinnerIcon4.classList.add('hidden'); button4.disabled = false; } }";
	
	String SCRIPT_SET_BUTTONS ="async function submitSelectedButtons(fxn) { const button1 = document.getElementById('submit-button-1'); const button2 = document.getElementById('submit-button-2'); const button3 = document.getElementById('submit-button-3'); const button4 = document.getElementById('submit-button-4'); const spinnerIcon1 = document.getElementById('spinnerIcon1'); const spinnerIcon2 = document.getElementById('spinnerIcon2'); const spinnerIcon3 = document.getElementById('spinnerIcon3'); const spinnerIcon4 = document.getElementById('spinnerIcon4'); if (fxn == '11') { spinnerIcon1.classList.remove('hidden'); button1.disabled = true; } else if (fxn == '21') { spinnerIcon2.classList.remove('hidden'); button2.disabled = true; } else if (fxn == '22') { spinnerIcon3.classList.remove('hidden'); button3.disabled = true; } else if (fxn == '23') { spinnerIcon4.classList.remove('hidden'); button4.disabled = true; } let selectedItemsText; if (fxn === '11' || fxn === '21') { selectedItemsText = Array.from(document.querySelectorAll('.bind-checkbox:checked')).map(checkbox => { const groupContainer = checkbox.closest('.group-container'); const shortAddressHex = groupContainer ? JSON.parse(groupContainer.title).short : null; return { short: shortAddressHex }; }); } else { selectedItemsText = Array.from(document.querySelectorAll('.identify-checkbox:checked')).map(checkbox => { const groupContainer = checkbox.closest('.group-container'); const shortAddressHex = groupContainer ? JSON.parse(groupContainer.title).short : null; return { short: shortAddressHex }; }); } console.log(\"selectedItemsText:\", JSON.stringify(selectedItemsText, null, 2)); const selectedShorts = selectedItemsText .map(item => item.short ? parseInt(item.short, 16) : null) .filter(num => num !== null); console.log(\"Converted selected shorts to numbers:\", selectedShorts); let jsonData = { fxn: fxn, option: document.getElementById(\"dropdownMenu\").selectedIndex }; if (fxn !== '22' && fxn !== '23') { const selectedItems = globalItemsArray.filter(item => { const itemShort = Number(item.short); console.log(`Checking item: short=${itemShort} (Hex: 0x${itemShort.toString(16).toUpperCase()})`); const isMatch = selectedShorts.includes(itemShort); const isActive = item.state === 1; console.log(`Matching item short=${itemShort}: match=${isMatch}, state=${item.state} → include=${isMatch && isActive}`); console.log(\"itemShort type:\", typeof itemShort, \"value:\", itemShort); console.log(\"selectedShorts:\", selectedShorts, \"types:\", selectedShorts.map(s => typeof s)); return selectedShorts.includes(itemShort) && item.state === 1; }); console.log(\"globalItemsArray\", globalItemsArray);  console.log(\"Filtered selectedItems:\", selectedItems); jsonData.selected_items = selectedItems; } var json_data = JSON.stringify(jsonData); const response = await fetch(\"/action\", { method: \"POST\", headers: { \"Content-Type\": \"application/json\" }, body: json_data }); if (response.ok) { console.log(\"Request successful\"); alert('Request Successful!!'); } else { console.error(\"Request failed\"); alert('Request Incomplete, Try Again!!'); } if (fxn == '11') { spinnerIcon1.classList.add('hidden'); button1.disabled = false; } else if (fxn == '21') { spinnerIcon2.classList.add('hidden'); button2.disabled = false; } else if (fxn == '22') { spinnerIcon3.classList.add('hidden'); button3.disabled = false; } else if (fxn == '23') { spinnerIcon4.classList.add('hidden'); button4.disabled = false; } }";

	
	String SCRIPT_SET_BINDING ="function submitSelectedItems(fxn, buttonElement) { const button1 = document.getElementById('submit-button-1'); const button2 = document.getElementById('submit-button-2'); const button3 = document.getElementById('submit-button-3'); const button4 = document.getElementById('submit-button-4'); const spinnerIcon1 = document.getElementById('spinnerIcon1'); const spinnerIcon2 = document.getElementById('spinnerIcon2'); const spinnerIcon3 = document.getElementById('spinnerIcon3'); const spinnerIcon4 = document.getElementById('spinnerIcon4'); if (fxn == '11') { spinnerIcon1.classList.remove('hidden'); button1.disabled = true; } else if (fxn == '21') { spinnerIcon2.classList.remove('hidden'); button2.disabled = true; } else if (fxn == '22') { spinnerIcon3.classList.remove('hidden'); button3.disabled = true; } else if (fxn == '23') { spinnerIcon4.classList.remove('hidden'); button4.disabled = true; } try { const shortKey = Number(buttonElement.getAttribute(\"data-short\")); const itemName = buttonElement.getAttribute(\"data-name\"); console.log(`Clicked button for item: Short=${shortKey}, Name=${itemName}`); const selectedItems = globalItemsArray .filter(item => item.short === shortKey) .map(item => ({ short: item.short, g_name: item.g_name, name: item.name, dst: item.dst, src: item.src, bind: item.bind, state: item.state, level: item.level, color: item.color, check: item.check })); console.log('Filtered selectedItems:', selectedItems); var dropdown = document.getElementById(\"dropdownMenu\"); const jsonData = { fxn: fxn, option: dropdown.selectedIndex, selected_items: selectedItems }; var json_data = JSON.stringify(jsonData); console.log(json_data); if (fxn === '22') { const userConfirmed = confirm(\"Are you sure you want to submit these items?\"); if (!userConfirmed) { console.log(\"User canceled the submission.\"); return; } } const response = fetch(\"/action\", { method: \"POST\", headers: { \"Content-Type\": \"application/json\" }, body: json_data }); if (response.ok) { console.log(\"Request successful\"); alert('Request Successful!!'); } else { console.error(\"Request failed\"); alert('Request Incomplete, Try Again!!'); } if (fxn == '11') { spinnerIcon1.classList.add('hidden'); button1.disabled = false; } else if (fxn == '21') { spinnerIcon2.classList.add('hidden'); button2.disabled = false; } else if (fxn == '22') { spinnerIcon3.classList.add('hidden'); button3.disabled = false; } else if (fxn == '23') { spinnerIcon4.classList.add('hidden'); button4.disabled = false; } } catch (error) { console.error(\"An error occurred:\", error); } }";

	String SCRIPT_GROUP_ITEMS ="function groupItemsByShort(items) { return items.reduce((groups, item) => { const shortKey = item.short; if (!groups[shortKey]) { groups[shortKey] = []; } groups[shortKey].push(item); return groups; }, {}); }";
	String SCRIPT_DISPLAY_ITEMS ="function displayItems(items) { if (!Array.isArray(items)) { console.error('Invalid items array'); return; } const groupedItems = groupItemsByShort(items); console.log(groupedItems); const container = document.getElementById('item-container'); if (!container) { console.error('Container element not found'); return; } container.innerHTML = ''; for (const shortKey in groupedItems) { const groupContainer = document.createElement('div'); groupContainer.className = 'group-container'; const shortAddressHex = `0x${parseInt(shortKey, 10).toString(16).toUpperCase()}`; groupContainer.title = JSON.stringify({ short: shortAddressHex }); const checkboxContainer = document.createElement('div'); checkboxContainer.className = 'checkbox-container'; const identifyCheckbox = document.createElement('input'); identifyCheckbox.type = 'checkbox'; identifyCheckbox.className = 'identify-checkbox'; identifyCheckbox.id = `identify-checkbox-${shortKey}`; const checkboxLabel = document.createElement('label'); checkboxLabel.className = 'checkbox-label'; checkboxLabel.textContent = 'Identify'; checkboxLabel.setAttribute('for', `identify-checkbox-${shortKey}`); checkboxContainer.appendChild(identifyCheckbox); checkboxContainer.appendChild(checkboxLabel); const bindCheckbox = document.createElement('input'); bindCheckbox.type = 'checkbox'; bindCheckbox.className = 'bind-checkbox'; bindCheckbox.id = `bind-checkbox-${shortKey}`; const bindLabel = document.createElement('label'); bindLabel.className = 'checkbox-label'; bindLabel.textContent = 'Bind'; bindLabel.setAttribute('for', `bind-checkbox-${shortKey}`); checkboxContainer.appendChild(bindCheckbox); checkboxContainer.appendChild(bindLabel); const allBound = groupedItems[shortKey].every(item => item.bind === 1); console.log(\"allBound:\", allBound); if(allBound) bindCheckbox.checked = 1; else bindCheckbox.checked = 0; identifyCheckbox.onclick = () => { const isChecked = identifyCheckbox.checked; if(isChecked) bindCheckbox.checked = false; groupedItems[shortKey].forEach(item => { const index = globalItemsArray.findIndex(el => el.name === item.name && el.short === item.short); if (isChecked) { if (index !== -1) { globalItemsArray[index].check = 1; } else { globalItemsArray.push({ ...item, check: 1 }); } } else { if (index !== -1) { globalItemsArray[index].check = 0; } else { globalItemsArray.push({ ...item, check: 0 }); } } }); groupContainer.querySelectorAll('.item-button').forEach(button => { button.style.border = isChecked ? '3px solid blue' : '1px solid gray'; }); console.log(`Group ${shortAddressHex} is ${isChecked ? \"selected\" : \"deselected\"}`); console.log(\"Global Items Array:\", globalItemsArray); }; bindCheckbox.onclick = () => { const isChecked = bindCheckbox.checked; if(isChecked) identifyCheckbox.checked = false; console.log(`Bind checkbox for ${shortAddressHex} is ${isChecked ? \"checked\" : \"unchecked\"}`); groupedItems[shortKey].forEach(item => { const index = globalItemsArray.findIndex(el => el.name === item.name && el.short === item.short); if (isChecked) { if (index !== -1) { globalItemsArray[index].bind = 1; } else { globalItemsArray.push({ ...item, bind: 1 }); } } else { if (index !== -1) { globalItemsArray[index].bind = 0; } else { globalItemsArray.push({ ...item, bind: 0 }); } } }); groupContainer.querySelectorAll('.item-button').forEach(button => { button.style.border = isChecked ? '3px solid orange' : '1px solid gray'; }); console.log(`Group ${shortAddressHex} is ${isChecked ? \"selected\" : \"deselected\"}`); }; const groupNameElement = document.createElement('h3'); groupNameElement.className = 'group-name'; groupNameElement.textContent = groupedItems[shortKey][0]?.g_name || 'Unnamed Device'; groupContainer.appendChild(groupNameElement); groupContainer.appendChild(checkboxContainer); groupedItems[shortKey].forEach(item => { const button = document.createElement('button'); button.className = 'item-button'; button.textContent = item.name; button.setAttribute('data-short', item.short); button.setAttribute('data-name', item.name); const index = globalItemsArray.findIndex(el => el.name === item.name && el.short === item.short); let level = index !== -1 ? globalItemsArray[index].level || 0 : 0; if(level>0){ button.style.position = \"relative\"; button.style.overflow = \"hidden\"; button.style.display = \"flex\"; button.style.alignItems = \"center\"; button.style.justifyContent = \"space-between\"; button.style.padding = \"5px 10px\"; const progressBar = document.createElement('div'); progressBar.className = 'progress-bar'; progressBar.style.width = `${(level / 255) * 100}%`; button.appendChild(progressBar); const slider = document.createElement('input'); slider.type = 'range'; slider.min = '0'; slider.max = '255'; slider.value = level; slider.className = 'progress-slider'; button.appendChild(slider); const updateProgress = () => { level = parseInt(slider.value, 10); progressBar.style.width = `${(level / 255) * 100}%`; if (index !== -1) { globalItemsArray[index].level = level; } else { globalItemsArray.push({ ...item, level }); } console.log(`${item.name} level set to ${level}`); }; slider.oninput = updateProgress; } if (index !== -1) { if (globalItemsArray[index].bind) { button.classList.add('selected'); console.log(\"Binding: OK\"); } } button.onclick = () => { button.classList.toggle('selected'); const index = globalItemsArray.findIndex(el => el.name === item.name && el.short === item.short); if (button.classList.contains('selected')) { if (index !== -1) { globalItemsArray[index].state = 1; } else { globalItemsArray.push({ ...item, state: 1 }); } if(globalItemsArray[index].check === 1){ submitSelectedButtons('15'); } console.log(`${item.name} is selected`); } else { if (index !== -1) { globalItemsArray[index].state = 0; } else { globalItemsArray.push({ ...item, state: 0 }); } console.log(`${item.name} is deselected`); } if(globalItemsArray[index].check === 0 && globalItemsArray[index].bind == 0){ submitSelectedItems('16', button); } }; groupContainer.appendChild(button); }); container.appendChild(groupContainer); } }";

	String SCRIPT_GET_SHORT_CODES =" function getShortCodes() { let shortCodes = {}; globalItemsArray.forEach(item => { if (!shortCodes[item.name]) { shortCodes[item.name] = []; } shortCodes[item.name].push(item.short); }); return shortCodes; }";

	#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH ||  USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH )
		String SCRIPT_SCENE_ITEMS ="const sceneItems = [ { index: 0, name: 'SCENE_1', intensity: 50, isOn: false }, { index: 1, name: 'SCENE_2', intensity: 50, isOn: false }, { index: 2, name: 'SCENE_3', intensity: 50, isOn: false }, { index: 3, name: 'SCENE_4', intensity: 50, isOn: false } ];";
    	String SCRIPT_SCENE_UPDATE_CONTROLS ="function updateControls(selectedSceneIndex) { const scene = sceneItems[selectedSceneIndex]; console.log(scene); document.getElementById('intensityRange').value = scene.intensity; document.getElementById('intensityValue').textContent = scene.intensity; document.getElementById('onOffSwitch').checked = scene.isOn; }";
		String SCRIPT_SCENE_UPDATE_INTENSITY ="function updateIntensityValue() { const selectedSceneIndex = document.getElementById(\"dropdownMenu\").selectedIndex; const intensity = document.getElementById('intensityRange').value; document.getElementById('intensityValue').textContent = intensity; console.log(selectedSceneIndex); const scene = sceneItems[selectedSceneIndex]; scene.intensity = intensity; }";
		String SCRIPT_SCENE_UPDATE_SWITCH ="function updateSwitchState() { const selectedSceneIndex = document.getElementById(\"dropdownMenu\").selectedIndex; const isOn = document.getElementById('onOffSwitch').checked; const scene = sceneItems[selectedSceneIndex]; scene.isOn = isOn; }";
		String SCRIPT_SCENE_SEND_SELECTED ="function sendSelectedScene(fxn) { const selectedSceneIndex = document.getElementById(\"dropdownMenu\").selectedIndex; const selectedScene = sceneItems[selectedSceneIndex]; const jsonData = { fxn: fxn, option:selectedSceneIndex, data: selectedScene }; var json_data = JSON.stringify(jsonData); const replacedString = json_data.replace(/\\\\\"/g, '\"'); console.log(replacedString); var xhr = new XMLHttpRequest(); xhr.open(\"POST\", \"/action\", true); xhr.setRequestHeader(\"Content-Type\", \"application/json\"); xhr.onreadystatechange = function() { if (xhr.readyState === XMLHttpRequest.DONE) { if (xhr.status === 200) { console.log(\"Request successful\"); } else { console.error(\"Request failed\"); } } }; xhr.send(json_data); }";
		String SCRIPT_SCENE_CALL_OPEN_CONTROLS ="updateControls(selected_index);";
	#endif
    String SCRIPT_UPDATE_IP_1 = "const defaultIP = ";
	String SCRIPT_UPDATE_IP_2 ="; document.addEventListener(\"DOMContentLoaded\", function() { document.getElementById(\"ip4\").value = defaultIP; });";
#else
    String SCRIPT_SET_DMX_CHANNEL ="document.getElementById('colorSelect').addEventListener('change', function() { const selectedValue = this.value; console.log(`Newly selected value: ${selectedValue}`); handlePOST('32', 0, selectedValue); });";
    String SCRIPT_SET_DMX_CH ="function setInitialValue(value) { document.getElementById('colorSelect').innerHTML = ''; const select = document.getElementById('colorSelect'); const selectOption = document.createElement('option'); selectOption.value = ''; selectOption.textContent = '-- Select --'; select.appendChild(selectOption); Array.from({length: 101}, (_, i) => `<option value=\"${i}\">${i}</option>` ).forEach(option => { select.insertAdjacentHTML('beforeend', option); }); const initialOption = select.querySelector(`option[value=\"${value}\"]`); if (initialOption) { initialOption.selected = true; } } setInitialValue(";
	String SCRIPT_SET_DMX_CH_VAL = "1";
	String SCRIPT_SET_DMX_CH_END = ");";

	String SCRIPT_SET_DMX_GPIO_PIN ="document.getElementById('pinSelect').addEventListener('change', function() { const selectedValue = this.value; console.log(`Newly selected value: ${selectedValue}`); handlePOST('34', 0, selectedValue); });";
	String SCRIPT_SET_DMX_PIN_FXN ="function setPinInitialValue(value) { document.getElementById('pinSelect').innerHTML = ''; const select = document.getElementById('pinSelect'); const selectOption = document.createElement('option'); selectOption.value = ''; selectOption.textContent = '-- Select --'; select.appendChild(selectOption); Array.from({length: 30}, (_, i) => `<option value=\"${i}\">${i}</option>` ).forEach(option => { select.insertAdjacentHTML('beforeend', option); }); const initialOption = select.querySelector(`option[value=\"${value}\"]`); if (initialOption) { initialOption.selected = true; } }  setPinInitialValue(";
	String SCRIPT_SET_DMX_PIN_VAL = "1";
	String SCRIPT_SET_DMX_PIN_END = ");";

	String SCRIPT_SET_DMX_UPDATE_COLOR_FXN ="function updateColorPicker(value) { const colorInput = document.getElementById('favcolor'); colorInput.value = value; colorInput.dispatchEvent(new Event('change')); }";
	String SCRIPT_SET_DMX_UPDATE_COLOR_START ="updateColorPicker('";
    String SCRIPT_SET_DMX_UPDATE_COLOR_VAL ="#ffffff";
	String SCRIPT_SET_DMX_UPDATE_COLOR_END ="');"; 
#endif

//String SCRIPT_SET_WIFI_AP_TOGGLE_MODE ="function toggleButton() { var button = document.getElementById('apModeButton'); if (button.innerHTML === 'AP mode enabled') { button.innerHTML = 'AP mode disabled'; toggle_val = 0; button.style.backgroundColor = 'red'; button.style.color = 'white'; } else { button.innerHTML = 'AP mode enabled'; toggle_val = 1; button.style.backgroundColor = 'green'; button.style.color = 'white';} const jsonData = { fxn: '6', data: toggle_val }; const xhr = new XMLHttpRequest(); xhr.open(\"POST\", \"/action\", true); xhr.setRequestHeader(\"Content-Type\", \"application/json\"); xhr.onreadystatechange = function() { if (xhr.readyState === XMLHttpRequest.DONE) { if (xhr.status === 200) { console.log(\"Request successful\"); } else { console.error(\"Request failed\"); } } }; xhr.send(JSON.stringify(jsonData)); }";
String SCRIPT_SET_WIFI_AP_TOGGLE_MODE ="function toggleButton() { var button = document.getElementById('apModeButton'); if (toggle_val == 0) { button.innerHTML = 'STA mode enabled<br>(Zigbee Enabled)'; toggle_val = 1; button.style.backgroundColor = 'green'; button.style.color = 'white'; } else { button.innerHTML = 'AP mode enabled<br>(Zigbee Disabled)'; toggle_val = 0; button.style.backgroundColor = 'red'; button.style.color = 'white'; } const jsonData = { fxn: '6', data: toggle_val }; const xhr = new XMLHttpRequest(); xhr.open(\"POST\", \"/action\", true); xhr.setRequestHeader(\"Content-Type\", \"application/json\"); xhr.onreadystatechange = function() { if (xhr.readyState === XMLHttpRequest.DONE) { if (xhr.status === 200) { console.log(\"Request successful\"); } else { console.error(\"Request failed\"); } } }; xhr.send(JSON.stringify(jsonData)); }";
String SCRIPT_SET_WIFI_AP_SET_MODE ="function setButton(state) { var button = document.getElementById('apModeButton'); if (state == 0) { button.innerHTML = 'AP mode disabled'; button.style.backgroundColor = 'red'; button.style.color = 'white'; } else if (state == 1) { button.innerHTML = 'AP mode enabled'; button.style.backgroundColor = 'green'; button.style.color = 'white'; } }";
String SCRIPT_SET_WIFI_AP_BUTTON_VALUE_START =" window.onload = function() {";
String SCRIPT_SET_WIFI_AP_BUTTON_VALUE_END ="selectACModel(";
String SCRIPT_SET_WIFI_AP_BUTTON_VALUE_END2 ="); for (let i = 0; i < newValues.length; i++) { updateInputFields(i, newValues[i]); } };";

String SCRIPT_END = "</script>";

String BODY_END = "</body>";
String HTML_END = "</html>";

String MyDisplayTab1 ="window.onload = function() {  displayItems(globalItemsArray, selected_index); let tabElement = document.querySelector(\"[data-tab='Tab1']\"); if (tabElement) { openTab(this, 'Tab1'); }";
String MyDisplayTab2 ="window.onload = function() {  displayItems(globalItemsArray, selected_index); let tabElement = document.querySelector(\"[data-tab='Tab2']\"); if (tabElement) { openTab(this, 'Tab2'); }";

String  MyDisplayItems0 = "var button = document.getElementById('apModeButton'); toggle_val = 0; button.innerHTML = 'AP mode enabled<br>(Zigbee Disabled)'; button.style.backgroundColor = 'red'; button.style.color = 'white'; };";
String  MyDisplayItems1 = "var button = document.getElementById('apModeButton'); toggle_val = 1; button.innerHTML = 'STA mode enabled<br>(Zigbee Enabled)'; button.style.backgroundColor = 'green'; button.style.color = 'white'; };";

const char *ssid_old_value = "[ssid]";
const char *pass_old_value = "[pass]";

#if(USE_NUOS_ZB_DEVICE_TYPE != DEVICE_1CH_CURTAIN)
void replace_substring(char *html_template, const char *old_value, const char *new_value) {
    char *pos = strstr(html_template, old_value);
    if (pos != NULL) {
        // Create a new buffer to store the modified string
        memset(buffer, 0, sizeof(buffer));
        
        // Copy the part before the old_value
        strncpy(buffer, html_template, pos - html_template);
        
        // Append the new_value
        strcat(buffer, new_value);
        
        // Append the part after the old_value
        strcat(buffer, pos + strlen(old_value));
        
        // Copy the modified string back to the original template
        strcpy(html_template, buffer);
    }
}


void init_html_start_string(){
	webpage[0] = 0;
	strcat(&webpage[0], HTML_START);
	strcat(webpage, HEAD_TITLE);
	/*STYLE*/
	strcat(webpage, STYLE_START);
	strcat(webpage, STYLE);

	#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
		strcat(webpage, STYLE_SELECT_AC_MODEL);
	#elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
	    strcat(webpage, STYLE_SELECT_AC_MODEL);
		strcat(webpage, STYLE_WIRELESS_SWITCH);
		#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH)
			strcat(webpage, STYLE_SCENE_SWITCH);
		#endif
	#elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
		#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI  || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
			strcat(webpage, STYLE_COLOR_TEMP);
			strcat(webpage, STYLE_INTENSITY);
		#endif
		#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI)
			strcat(webpage, STYLE_INTENSITY);
		#endif
		#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
			strcat(webpage, STYLE_RGB_DMX);
			strcat(webpage, STYLE_RGB_DMX_FORM_ROW); 
			strcat(webpage, STYLE_RGB_DMX_FORM_ROW_1); 
			strcat(webpage, STYLE_INTENSITY);
		#endif
		#ifdef USE_DALI_SCENES
			strcat(webpage, STYLE_SCENE_SELECTOR);
			strcat(webpage, STYLE_SETTINGS);
		#endif
    #endif
	strcat(webpage, STYLE_LOADER_SPINNER);
	strcat(webpage, STYLE_DIALOG_BOX); 
	strcat(webpage, STYLE_BUTTON_SPINNER);  

	strcat(webpage, STYLE_IP_ADDR);
	strcat(webpage, STYLE_CHECK_BOX); 
	strcat(webpage, STYLE_END);
	strcat(webpage, BODY_START); 
	strcat(webpage, TAB_DISPLAY);
}

#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
void prepare_html_start_wifi_string(){
	strcat(webpage, TAB1_START);
	strcat(webpage, SHOW_ALL_WIFI_NETWORKS);
	strcat(webpage, START_WIFI_FORM_CONTAINER);
	/*Save here ssid and password functionalities*/
	strcat(webpage, SET_WIFI_SSID_VALUE);
	strcat(webpage, SET_WIFI_PASS_VALUE);
	strcat(webpage, SET_WIFI_IP_VALUE);

	replace_substring(webpage, ssid_old_value, wifi_info.wifi_ssid);
	replace_substring(webpage, pass_old_value, wifi_info.wifi_pass);
}

void prepare_html_end_wifi_string(){
	strcat(webpage, SET_WIFI_INFO_END);
	strcat(webpage, TAB1_END);
}
#endif
void prepare_html_tab3_string(){
	#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI  || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_CUSTOM || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI_CUSTOM)
		#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
			strcat(webpage, TAB3_START);
			strcat(webpage, TAB3_BODY); 
		#else
			strcat(webpage, TAB3_START);
			strcat(webpage, TAB3_START_DALI);

			char str[4];
			itoa(dali_on_webpage_commissioning_counts, str, 10);
			strcat(webpage, str);
			strcat(webpage, TAB3_START_DALI_CONTINUE);
			strcat(webpage, TAB3_SHOW_DALI);
		#endif
		strcat(webpage, TAB3_END);
	#endif
}


void prepare_html_tab4_string(){
	#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI  || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_CUSTOM  || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI)
        #ifdef USE_DALI_SCENES
			strcat(webpage, TAB4_START);
			strcat(webpage, TAB4_BODY);
			strcat(webpage, TAB4_END);
		#endif

	#endif
}

void prepare_html_tab2_string(){
	strcat(webpage, TAB2_START);
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
		strcat(webpage, TAB2_BODY);  	 
		strcat(webpage, TAB2_SELECT); 
		strcat(webpage, TAB2_BUTTON);
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
		strcat(webpage, TAB2_DROPDOWN);
		strcat(webpage, TAB2_BODY);  
		strcat(webpage, TAB2_LOAD_SPINNER);	 
		strcat(webpage, TAB2_DIALOG_BOX);	  
		strcat(webpage, TAB2_SELECT); 
		#if( USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH) 
			strcat(webpage, TAB2_IDENTIFY_BUTTON);
		#endif
		strcat(webpage, TAB2_BUTTON);
		#if( USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH) 
			strcat(webpage, TAB2_BUTTON_NAME);
			strcat(webpage, TAB2_BUTTON_END);
			strcat(webpage, TAB2_UNBIND_BUTTON);
			strcat(webpage, TAB2_UNBIND_BUTTON_NAME);
			strcat(webpage, TAB2_UNBIND_BUTTON_END);
		#endif
		#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH)
			strcat(webpage, TAB2_SCENE_BODY);
		#endif
	#else
		// strcat(webpage, CARD_GRID_START);
        char str[6];
		#if(TOTAL_ENDPOINTS <=4 && TOTAL_ENDPOINTS >= 1)
		    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DMX)
				strcat(webpage, DEVICE1_SET_DMX_CH);
                strcat(webpage, DEVICE1_SET_GPIO_PIN);
				strcat(webpage, DEVICE1_COLOR_RGB);
				strcat(webpage, DEVICE1_SET_INTENSITY);
				itoa(device_info[3].device_val, str, 10);
				strcat(webpage, str);			
				strcat(webpage, DEVICE1_SET_INTENSITY_CONTINUE);		
			#else
				strcat(webpage, DEVICE1_FORM_START);
				#ifdef USE_DALI_SCENES
					strcat(webpage, DEVICE1_SCENE);
				#endif
				strcat(webpage, DEVICE1_FORM_S);
				for(int i=0; i<20; i++){
					itoa(dali_nvs_stt[0].device_ids[i], str, 10);
					strcat(webpage, str);
					if(i<19)				
					strcat(webpage, ",");
				}				
				strcat(webpage, DEVICE1_FORM);

				#if(USE_NUOS_ZB_DEVICE_TYPE != DEVICE_GROUP_DALI)
					strcat(webpage, DEVICE1_COLOR_TEMP);
					itoa(dali_nvs_stt[0].color_value, str, 10);
					strcat(webpage, str);
					strcat(webpage, DEVICE1_COLOR_TEMP_CONTINUE);
				#endif
				strcat(webpage, DEVICE1_SET_INTENSITY);
				itoa(dali_nvs_stt[0].brightness, str, 10);
				strcat(webpage, str);			
				strcat(webpage, DEVICE1_SET_INTENSITY_CONTINUE);
				strcat(webpage, DEVICE1_DIV_END);
				strcat(webpage, CARD_GRID_END);
			#endif
        #endif
		#if(TOTAL_ENDPOINTS <=4 && TOTAL_ENDPOINTS >= 2)
			strcat(webpage, DEVICE2_FORM_START);
			#ifdef USE_DALI_SCENES
				strcat(webpage, DEVICE2_SCENE);
			#endif
			strcat(webpage, DEVICE2_FORM_S);
			for(int i=0; i<20; i++){
				itoa(dali_nvs_stt[1].device_ids[i], str, 10);
				strcat(webpage, str);
				if(i<19)				
				strcat(webpage, ",");
			}			
			strcat(webpage, DEVICE2_FORM);
            #if(USE_NUOS_ZB_DEVICE_TYPE != DEVICE_GROUP_DALI)
				strcat(webpage, DEVICE2_COLOR_TEMP);
				itoa(dali_nvs_stt[1].color_value, str, 10);
				strcat(webpage, str);
				strcat(webpage, DEVICE2_COLOR_TEMP_CONTINUE);
            #endif 
			strcat(webpage, DEVICE2_SET_INTENSITY);
			itoa(dali_nvs_stt[1].brightness, str, 10);
			strcat(webpage, str);			
			strcat(webpage, DEVICE2_SET_INTENSITY_CONTINUE);
			strcat(webpage, DEVICE2_DIV_END);
			strcat(webpage, CARD_GRID_END);
        #endif
		#if(TOTAL_ENDPOINTS <=4 && TOTAL_ENDPOINTS >= 3)
			strcat(webpage, DEVICE3_FORM_START);
			#ifdef USE_DALI_SCENES
				strcat(webpage, DEVICE3_SCENE);
			#endif
			strcat(webpage, DEVICE3_FORM_S);
			for(int i=0; i<20; i++){
				itoa(dali_nvs_stt[2].device_ids[i], str, 10);
				strcat(webpage, str);
				if(i<19)				
				strcat(webpage, ",");
			}			
			strcat(webpage, DEVICE3_FORM);

			#if(USE_NUOS_ZB_DEVICE_TYPE != DEVICE_GROUP_DALI)
				strcat(webpage, DEVICE3_COLOR_TEMP);
				itoa(dali_nvs_stt[2].color_value, str, 10);
				strcat(webpage, str);
				strcat(webpage, DEVICE3_COLOR_TEMP_CONTINUE);
            #endif
			strcat(webpage, DEVICE3_SET_INTENSITY);
			itoa(dali_nvs_stt[2].brightness, str, 10);
			strcat(webpage, str);			
			strcat(webpage, DEVICE3_SET_INTENSITY_CONTINUE);
			strcat(webpage, DEVICE3_DIV_END);
			strcat(webpage, CARD_GRID_END);
        #endif

		#if(TOTAL_ENDPOINTS == 4)
			strcat(webpage, DEVICE4_FORM_START);
			#ifdef USE_DALI_SCENES
				strcat(webpage, DEVICE4_SCENE);
			#endif
			strcat(webpage, DEVICE4_FORM_S);
			for(int i=0; i<20; i++){
				itoa(dali_nvs_stt[3].device_ids[i], str, 10);
				strcat(webpage, str);
				if(i<19)				
				strcat(webpage, ",");
			}
			
			strcat(webpage, DEVICE4_FORM);

			#if(USE_NUOS_ZB_DEVICE_TYPE != DEVICE_GROUP_DALI)
				strcat(webpage, DEVICE4_COLOR_TEMP);
				itoa(dali_nvs_stt[3].color_value, str, 10);
				strcat(webpage, str);
				strcat(webpage, DEVICE4_COLOR_TEMP_CONTINUE);
            #endif    
			strcat(webpage, DEVICE4_SET_INTENSITY);
			itoa(dali_nvs_stt[3].brightness, str, 10);
			strcat(webpage, str);			
			strcat(webpage, DEVICE4_SET_INTENSITY_CONTINUE);
			strcat(webpage, DEVICE4_DIV_END);
			strcat(webpage, CARD_GRID_END);
        #endif
		
	#endif 
	strcat(webpage, TAB2_END);
}



char* convert_to_json() {
    cJSON* json_array = cJSON_CreateArray();
    for (int i = 0; i < 4; ++i) {
        cJSON* json_obj = cJSON_CreateObject();
        cJSON_AddNumberToObject(json_obj, "input1_1", dali_nvs_stt[i].device_ids[0]);
        cJSON_AddNumberToObject(json_obj, "input1_2", dali_nvs_stt[i].device_ids[1]);
        cJSON_AddNumberToObject(json_obj, "input1_3", dali_nvs_stt[i].device_ids[2]);
        cJSON_AddNumberToObject(json_obj, "input1_4", dali_nvs_stt[i].device_ids[3]);
        cJSON_AddNumberToObject(json_obj, "input1_5", dali_nvs_stt[i].device_ids[4]);

        cJSON_AddNumberToObject(json_obj, "input2_1", dali_nvs_stt[i].device_ids[5]);
        cJSON_AddNumberToObject(json_obj, "input2_2", dali_nvs_stt[i].device_ids[6]);
        cJSON_AddNumberToObject(json_obj, "input2_3", dali_nvs_stt[i].device_ids[7]);
        cJSON_AddNumberToObject(json_obj, "input2_4", dali_nvs_stt[i].device_ids[8]);
        cJSON_AddNumberToObject(json_obj, "input2_5", dali_nvs_stt[i].device_ids[9]);

        cJSON_AddNumberToObject(json_obj, "input3_1", dali_nvs_stt[i].device_ids[10]);
        cJSON_AddNumberToObject(json_obj, "input3_2", dali_nvs_stt[i].device_ids[11]);
        cJSON_AddNumberToObject(json_obj, "input3_3", dali_nvs_stt[i].device_ids[12]);
        cJSON_AddNumberToObject(json_obj, "input3_4", dali_nvs_stt[i].device_ids[13]);
        cJSON_AddNumberToObject(json_obj, "input3_5", dali_nvs_stt[i].device_ids[14]);

        cJSON_AddNumberToObject(json_obj, "input4_1", dali_nvs_stt[i].device_ids[15]);
        cJSON_AddNumberToObject(json_obj, "input4_2", dali_nvs_stt[i].device_ids[16]);
        cJSON_AddNumberToObject(json_obj, "input4_3", dali_nvs_stt[i].device_ids[17]);
        cJSON_AddNumberToObject(json_obj, "input4_4", dali_nvs_stt[i].device_ids[18]);
        cJSON_AddNumberToObject(json_obj, "input4_5", dali_nvs_stt[i].device_ids[19]);

        cJSON_AddItemToArray(json_array, json_obj);
    }

    char* json_string = cJSON_Print(json_array);
	
    cJSON_Delete(json_array);
    return json_string;
}
void rgbToHexString(uint8_t r, uint8_t g, uint8_t b, char* buffer) {
    snprintf(buffer, 11, "#%02X%02X%02X", r, g, b);
}
void prepare_html_script_string(){
	strcat(webpage, SCRIPT_START);

	#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
		strcat(webpage, SCRIPT_GLOBAL_ARRAY);
		node_counts = existing_nodes_info[0].scene_switch_info.total_records;
        String response = prepare_json(0); 
		strcat(webpage, response);
		strcat(webpage, ";");	
		strcat(webpage, SCRIPT_UPDATE_IP_1);
		char str1[5];
		itoa(wifi_info.ip4, str1, 10);
		strcat(webpage, str1);		
		strcat(webpage, SCRIPT_UPDATE_IP_2);

		strcat(webpage, SCRIPT_OPEN_TAB);
		strcat(webpage, SCRIPT_SUBMIT_DATA);

	#endif



	#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
		strcat(webpage, SCRIPT_SUBMIT_AC_MODEL);
		strcat(webpage, SCRIPT_SELECT_AC_MODEL);

		strcat(webpage, SCRIPT_OPEN_TAB);
		strcat(webpage, SCRIPT_SUBMIT_DATA);	
       // strcat(webpage, SCRIPT_OPEN_TAB2_BY_USER);
    #elif(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
		strcat(webpage, SCRIPT_SHOW_WIFI_NWKS);
		strcat(webpage, SCRIPT_FETCH_INITIAL_DATA);
		strcat(webpage, SCRIPT_SET_WIFI_AP_TOGGLE_MODE);
		strcat(webpage, SCRIPT_SET_WIFI_AP_SET_MODE);

		strcat(webpage, SCRIPT_FETCH_ITEMS);
		strcat(webpage, SCRIPT_FETCH_SELECTED_ITEMS);
		strcat(webpage, SCRIPT_CONFIRMATION_DIALOG); 
		strcat(webpage, SCRIPT_SEND_SELECTED_ITEMS);
		strcat(webpage, SCRIPT_GET_SHORT_CODES);
		strcat(webpage, SCRIPT_GROUP_ITEMS);
		strcat(webpage, SCRIPT_DISPLAY_ITEMS);
		
		strcat(webpage, SCRIPT_SET_BINDING);
		strcat(webpage, SCRIPT_SET_BUTTONS);
		#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH)
			strcat(webpage, SCRIPT_SCENE_ITEMS);
			strcat(webpage, SCRIPT_SCENE_UPDATE_CONTROLS);
			strcat(webpage, SCRIPT_SCENE_UPDATE_INTENSITY);
			strcat(webpage, SCRIPT_SCENE_UPDATE_SWITCH);
			strcat(webpage, SCRIPT_SCENE_SEND_SELECTED);
			strcat(webpage, SCRIPT_SCENE_CALL_OPEN_CONTROLS);
		#endif

		strcat(webpage, SCRIPT_OPEN_TAB);
		strcat(webpage, SCRIPT_SUBMIT_DATA);

		if(wifi_info.is_wifi_sta_mode == 0){	
			strcat(webpage,  MyDisplayTab1);
			strcat(webpage,  MyDisplayItems0);	
		}else{
			strcat(webpage,  MyDisplayTab2);
			strcat(webpage,  MyDisplayItems1);
	    }
    #else
		#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_RGB_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_DALI || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_CCT_CUSTOM)
			strcat(webpage, SCRIPT_ARRAY_START); 
			strcat(webpage, convert_to_json());
			strcat(webpage, ";");
			strcat(webpage, SCRIPT_UPDATE_IP_FIELDS); 

			strcat(webpage, SCRIPT_HANDLE_SUBMIT);
			strcat(webpage, SCRIPT_HANDLE_POST);
			strcat(webpage, SCRIPT_SUBMIT_FORM);
			
			strcat(webpage, SCRIPT_SHOW_TABLE);
			strcat(webpage, SCRIPT_COLOR_TEMPERATURE);
		#endif

		#ifdef USE_DALI_SCENES
			strcat(webpage, SCRIPT_SCENES_TAB);
			strcat(webpage, SCRIPT_SCENES_SELECTOR);
		#endif

		strcat(webpage, SCRIPT_OPEN_TAB);
		strcat(webpage, SCRIPT_SUBMIT_DATA);
		strcat(webpage, SCRIPT_SUBMIT_FORM);
		///////////////////////
		#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_DALI)
		strcat(webpage, SCRIPT_HANDLE_POST);
		strcat(webpage, SCRIPT_SET_INTENSITY);
		strcat(webpage, SCRIPT_UPDATE_INTENSITY);
		strcat(webpage, SCRIPT_UPDATE_INTENSITY_VAL_START);
		strcat(webpage, SCRIPT_UPDATE_INTENSITY_VAL_SET);
		strcat(webpage, SCRIPT_UPDATE_INTENSITY_VAL_END);

		strcat(webpage, SCRIPT_SHOW_TABLE);
		//////////////////////////////////////
		
		#endif
		//strcat(webpage, SCRIPT_OPEN_TAB2_BY_USER);

	#endif

	#if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_IR_BLASTER_CUSTOM)
		strcat(webpage, SCRIPT_SET_WIFI_AP_BUTTON_VALUE_START);	
		strcat(webpage, SCRIPT_SET_WIFI_AP_BUTTON_VALUE_END);
		char str1[4];
		itoa(device_info[0].ac_decode_type, str1, 10);
		strcat(webpage, str1);
		strcat(webpage, SCRIPT_SET_WIFI_AP_BUTTON_VALUE_END2);
		printf("--------->is_wifi_sta_mode11:%d\n", wifi_info.is_wifi_sta_mode);
        printf("--------->ac_decode_type11:%d\n", device_info[0].ac_decode_type);
       // strcat(webpage, SCRIPT_OPEN_TAB2_BY_USER);
	#endif

	strcat(webpage, SCRIPT_END);

	strcat(webpage, BODY_END);
	strcat(webpage, HTML_END);
    printf("\n\n");
	printf(webpage);
	printf("\n\n");
}

void prepare_html_end_string(){
	strcat(webpage, BODY_END);
	strcat(webpage, HTML_END);
}

void prepare_html_complete_string(){
	init_html_start_string();
    #if(USE_NUOS_ZB_DEVICE_TYPE == DEVICE_SCENE_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_GROUP_SWITCH || USE_NUOS_ZB_DEVICE_TYPE == DEVICE_WIRELESS_REMOTE_SWITCH)
	prepare_html_start_wifi_string();
    prepare_html_end_wifi_string();
    #endif
	prepare_html_tab2_string();

	prepare_html_tab3_string();
	prepare_html_tab4_string();

	prepare_html_script_string();
	prepare_html_end_string();

	
}
#endif
bool nuos_init_webserver(){
	#ifdef USE_WIFI_WEBSERVER
		bool station_mode = wifi_station_main();
		#ifdef USE_WEB_LOG_MODE
        	start_curtain_webserver(); 
		#else
			prepare_html_complete_string();
			start_webserver();
		#endif
		return station_mode;
	#else
		return true;
	#endif
}

#endif