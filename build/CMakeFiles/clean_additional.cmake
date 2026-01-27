# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "")
  file(REMOVE_RECURSE
  "bootloader\\bootloader.bin"
  "bootloader\\bootloader.elf"
  "bootloader\\bootloader.map"
  "config\\sdkconfig.cmake"
  "config\\sdkconfig.h"
  "esp-idf\\esptool_py\\flasher_args.json.in"
  "esp-idf\\mbedtls\\x509_crt_bundle"
  "flash_app_args"
  "flash_bootloader_args"
  "flash_project_args"
  "flasher_args.json"
  "index_curtain.html.S"
  "index_dali_direct_addr.html.S"
  "index_dali_master.html.S"
  "index_dali_scn.html.S"
  "index_dali_switch.html.S"
  "ldgen_libraries"
  "ldgen_libraries.in"
  "on_off_light_bulb.bin"
  "on_off_light_bulb.map"
  "project_elf_src_esp32h2.c"
  "x509_crt_bundle.S"
  )
endif()
