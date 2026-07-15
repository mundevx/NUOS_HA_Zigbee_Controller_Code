#ifndef __DALICOMP_H__
#define __DALICOMP_H__

#include <stdint.h>
#include "DALI.h"
#include "driver/gpio.h"
#include "dali_receiver.h"

class DaliCommands {
public:
    typedef enum {
        SET_DTR0 = 0xA3,
        SET_DTR1 = 0xC3,
        SET_DTR2 = 0xC5,
        ACTIVATE_DT8 = 0xE2,        // 226
        SET_COLOR_TEMP = 0xE7,       // 231
        COLOR_ACTIVATE = 0xE2,
        ENABLE_DEVICE_TYPE = 0xC1,
        SET_RGB_DIMMING_CURVE = 0xD5,    // 213
        SET_X = 224,
        SET_Y = 225,
        SET_TEMP_TEMPC = 0xE7,       // 231 <---working
        SET_TEMP_TEMPC_COOLER = 0xE8,    // 232
        SET_TEMP_TEMPC_WARMER = 0xE9,    // 233
        SET_TEMP_RGB_DIM = 0xEB,     // 235 <---working
        SET_TEMP_WAF_DIM = 0xEC,     // 236 <---working
        SET_TEMP_RGB_WAF_DIM = 0xED,     // 237
        SET_TEMP_PRIMARY_DIM = 0xEA,     // 234
        SET_TEMP_WHITE_CH = 0xCF,    // 207 white channel
    } DT8_COMMANDS;

    typedef enum {
        SHORT_POWER = 0,
        GROUP_ADDRESS = 1,
    } COMMANDS_DATA;

    typedef enum {
        OFF = 0x00,
        ON_AND_STEP_UP = 0x08,
        GO_TO_LAST_ACTIVE_LEVEL = 0x0A,
        DALI_CMD_RESET = 32,
        DALI_CMD_STORE_LEVEL = 33,
        DALI_CMD_STORE_DTR_SA = 128,
        DALI_CMD_STORE_DTR_MAX_LEVEL = 42,
        DALI_CMD_STORE_DTR_MIN_LEVEL = 43,
        DALI_CMD_STORE_DTR_PWR_ON_LEVEL = 45,
        DALI_CMD_STORE_DTR_FADE_TIME = 46,
        DALI_CMD_STORE_DTR_FADE_RATE = 47,
        DALI_CMD_STORE_DTR = 163,
    } STANDARD_COMMANDS;

    typedef enum {
        GO_TO_SCENE = 0x10,
        SET_SCENE = 0x40,
        SET_FADE_TIME = 0x2E,        // 46
        SET_FADE_RATE = 0x2F,        // 47
        SET_EXT_FADE_TIME = 0x30,    // 48
        SET_POWER_ON_LEVEL = 0x2D,   // 45
        SET_MIN_LEVEL = 0x2B,
        REMOVE_FROM_SCENE = 0x50,
        REMOVE_FROM_GROUP = 0x70,
        STORE_DTR_AS_SCENE = 0x40,
        ADD_TO_GROUP = 0x60,
        SET_SHORT_ADDR = 0x80,
        SET_MAX_LEVEL = 0x2A,
        PING = 262,
    } GROUP_COMMANDS;



    typedef enum {
        BROADCAST_DP = 0b11111110,
        BROADCAST_C = 0b11111111,
        ON_DP = 0b11111110,
        OFF_DP = 0b00000000,
        ON_C = 0b00000101,
        OFF_C = 0b00000000,
        QUERY_STATUS = 0b10010000,
        RESET = 0b00100000,
    } SPECIAL_COMMANDS;

    // Constructor
    explicit DaliCommands(gpio_num_t txPin, gpio_num_t rxPin);

    // Basic control functions
    void begin(bool* is_isr);
    void begin_rx(bool* is_isr, QueueHandle_t rxFrameQueue);

    int initNodes(const uint8_t* addresses, uint8_t numAddresses);
    int commissionNewNodes();
    void turn_off(uint8_t nodeAddress);
    void turn_on_to_max(uint8_t nodeAddress);
    void turn_on_to_last_level(uint8_t nodeAddress);
    void factory_reset(uint8_t nodeNumber);
    void set_dim_value(uint8_t nodeAddress, uint8_t value);
    
    void dali_rx_intr_enabled(bool enabled);
    // Color control functions
    bool set_color_temp(uint8_t addr, uint16_t kelvin);
    void set_color_temperature(uint8_t addr, uint16_t temp);
    void set_color_rgb(uint8_t addr, uint8_t r, uint8_t g, uint8_t b, uint8_t dim, bool mode_change_flag);
    void set_color_rgb_2(uint8_t addr, uint8_t r, uint8_t g, uint8_t b, uint8_t dim, bool mode_change_flag);
    bool set_rgb_2(uint8_t addr, uint8_t r, uint8_t g, uint8_t b, uint8_t dim, bool mode_change_flag);
    bool set_rgb_3(uint8_t addr, uint8_t r, uint8_t g, uint8_t b, uint8_t dim, bool mode_change_flag);
    bool set_rgb_3X(uint8_t addr, uint8_t r, uint8_t g, uint8_t b, uint8_t dim, bool mode_change_flag);
    void set_rgb_32(uint8_t addr, uint8_t r, uint8_t g, uint8_t b, uint8_t dim);
    void set_color_cct_waf_dim(uint8_t addr, uint8_t dim);
    void set_cct_channel_4_5_dim(uint8_t addr, uint8_t warm_dim, uint8_t cool_dim);
    bool set_off_waf_channels(uint8_t addr);
    bool set_rgb_WAF(uint8_t addr, uint8_t dim);
    void set_color_rgb_WAF(uint8_t addr, uint8_t dim);

    // Device parameter functions
    void set_power_on_level(uint8_t addr, uint8_t power_on_level);
    void set_min_level(uint8_t addr, uint8_t value);
    void set_max_level(uint8_t addr, uint8_t value);
    void set_fade_time(uint8_t addr, uint8_t time);
    void set_fade_rate(uint8_t addr, uint8_t rate);
    void set_ext_fade_time(uint8_t addr, uint8_t time);
    void set_rgbwaf_ctrl();
    void set_mode(uint8_t addr, uint8_t mode);
    void set_primary_dim_level(uint8_t addr, uint16_t white_dim);

    // Command sending functions
    bool send_command_special(uint8_t opcode, uint8_t address);
    bool send_command_standard(uint8_t opcode, uint8_t address);
    void send_command_special32(uint8_t opcode1, uint8_t address1,
                               uint8_t opcode2, uint8_t address2);
    // void send_command_standard32(uint8_t opcode1, uint8_t address1,
    //                             uint8_t opcode2, uint8_t address2);

    // bool send_command_special_standard32(uint8_t opcode1, uint8_t address1,
    //                                        uint8_t opcode2, uint8_t address2);                            

    // Broadcast functions
    void set_broadcast_fade_rate(uint8_t rate);
    void set_broadcast_fade_time(uint8_t time);
    void send_broadcast(uint8_t status);
    void set_broadcast_level(uint8_t value);
    void set_broadcast_color_rgb(uint8_t r, uint8_t g, uint8_t b, uint8_t dim, bool ss);
    void set_broadcast_color_rgb_2(uint8_t r, uint8_t g, uint8_t b, uint8_t dim);
    void set_broadcast_color_cct(uint16_t color_temp_kelvin);
    void set_cct_dimming(uint8_t addr, uint8_t dim);

    // Group functions
    bool add_to_group(uint8_t addr, uint8_t group);
    void remove_from_group(uint8_t addr, uint8_t group);
    void set_group_fade_time(uint8_t addr, uint8_t time);
    void set_group_fade_rate(uint8_t addr, uint8_t rate);
    void set_group_power_on_level(uint8_t addr, uint8_t power_on_level);
    void set_group_off(uint8_t addr);
    void set_group_on(uint8_t addr);
    uint8_t get_group_addr(uint8_t group_number);
    void set_group_level(uint8_t group_addr, uint8_t value);
    void set_group_level_normal(uint8_t group_id, uint8_t value);
    void set_group_color_cct(uint8_t group_addr, uint16_t color_temp_kelvin);
    void set_group_color_rgb(uint8_t group_id, uint8_t r, uint8_t g, uint8_t b, uint8_t dim, bool mode_change_flag);
    void set_group_color_rgb_normal(uint8_t group_id, uint8_t r, uint8_t g, uint8_t b, uint8_t dim, bool mode_change_flag);
    // Scene functions
    void set_scene(uint8_t addr, uint8_t scene, uint8_t level);
    void set_color_scene(uint8_t addr, uint8_t scene, uint8_t scene_level , uint16_t temp);
    void set_rgb_scene(uint8_t addr, uint8_t scene, uint8_t scene_level , uint8_t r, uint8_t g, uint8_t b);
    void set_level_scene(uint8_t addr, uint8_t scene , uint8_t scene_level);

    void remove_from_scene(uint8_t addr, uint8_t scene);
    void go_to_scene(uint8_t addr, uint8_t scene);
    void add_to_scene(uint8_t addr, uint8_t scene);
    void go_to_group_scene(uint8_t group_id, uint8_t scene);    
    // Scanning function
    int scanAssignedShortAddresses(uint8_t* foundAddresses, uint8_t maxAddresses);

    void disableRxInterrupt();
    void enableRxInterrupt();

    void query(uint8_t shortAddress, uint8_t queryCommand);
    void enable_query_mode();
    void disable_query_mode();
    void set_group_cc_primary_level(uint8_t group_id, uint16_t cct_color);
 
    bool send_command_special_ret_retry(uint8_t opcode, uint8_t address, uint8_t* retryCount);

    bool send_command_standard_ret_retry(uint8_t opcode, uint8_t address, uint8_t* retryCount);

    bool send_command_special_normal(uint8_t opcode, uint8_t address);

    bool send_command_standard_normal(uint8_t opcode, uint8_t address);

    
    int readExistingDrivers(uint8_t *addressList, int maxDevices);
    bool waitForResponse();

    bool clearShortAddress(uint8_t shortAddr);
    bool resetDriver(uint8_t shortAddr);
    int32_t queryGear(uint8_t shortAddr, uint8_t query_cmd);
    int32_t queryPowerOnLevel(uint8_t shortAddr);
    int32_t queryFadeTimeFadeRate(uint8_t shortAddr);
    int32_t queryDeviceType(uint8_t shortAddr);
    int32_t queryNextDeviceType(uint8_t shortAddr);
    int32_t queryDeviceInGroupA(uint8_t shortAddr);
    int32_t queryDeviceInGroupB(uint8_t shortAddr);
    int32_t queryGearFeatures(uint8_t shortAddr);
    private:
        gpio_num_t txPin;
        gpio_num_t rxPin;
        DALI daliCore;
        dali_rx::Receiver receiver;
    };

#endif // __DALICOMP_H__