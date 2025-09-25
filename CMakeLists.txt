# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/../../common/light_driver
    # C:\esp\esp-zigbee-sdk\examples\esp_zigbee_HA_sample\HA_on_off_light\components\arduino
    # C:\esp\esp-zigbee-sdk\examples\esp_zigbee_HA_sample\HA_on_off_light\components\IRremoteESP8266
    )
    
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(on_off_light_bulb)
# Add web files to embedded binary data
# embed_txt("web_files/incurtaindex.html" "text/html")
# add_subdirectory(main)