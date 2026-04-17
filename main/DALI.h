// #ifndef __DALI_H__
// #define __DALI_H__

// #include <stdint.h>
// #include "driver/gpio.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_rom_sys.h"
// #include "esp_timer.h"
// #include "esp_log.h"

// class DALI {
// private:
//     enum {
//         COMMAND_BROADCAST = 0b11111111,
//         COMMAND_GRUPO = 0b10000001,
//         RESET = 0b00100000,
//         INITIALISE = 0b10100101,
//         RANDOMISE = 0b10100111,
//         SEARCHADDRH = 0b10110001,
//         SEARCHADDRM = 0b10110011,
//         SEARCHADDRL = 0b10110101,
//         COMPARE = 0b10101001,
//         WITHDRAW = 0b10101011,
//         TERMINATE = 0b10100001,
//         SHORT_POWER = 0b00000000,
//         PROGRAM_SHORT_ADDRESS = 0b10110111,
//         VERIFY_SHORT_ADDRESS = 0b10111001,
//         CONTROL_BROADCAST = 0b11111110,
//     };

// public:
//     DALI(); // Default constructor
//     DALI(gpio_num_t txPin, gpio_num_t rxPin);

// public:
//     void begin();
//     int initNodes(const uint8_t* addresses, uint8_t numAddresses);
//     int initNodes(uint8_t address);
//     void turnOff(uint8_t nodeAddress);
//     void setMax(uint8_t nodeAddress);
//     void setValue(uint8_t nodeAddress, uint8_t value);
//     void sendCommandPublic(uint8_t command, uint8_t data);
//     void sendCommandPublic32(uint8_t command1, uint8_t data1, uint8_t command2, uint8_t data2);
//     void task_delayMicroseconds(uint32_t microseconds);
//     void task_delay(uint32_t milliseconds);
//     int scanAssignedShortAddresses(uint8_t* foundAddresses, uint8_t maxAddresses);

// private:
//     void sendZero(void);
//     void sendOne(void);
//     bool isBusIdle();
//     void sendCommand(uint8_t command, uint8_t data);
//     void sendCommand32(uint8_t command1, uint8_t data1, uint8_t command2, uint8_t data2);
//     bool sendSearchAddr(uint32_t addr);
//     bool sendProgramShortAddr(uint8_t nodeNumber);
//     void withdrawNode(uint32_t addr);

// private:
//     gpio_num_t txPin;
//     gpio_num_t rxPin;
//     static const char *TAG;
// };

// #endif // __DALI_H__

#ifndef __DALI_H__
#define __DALI_H__
#include "driver/gptimer.h"
#include <stdint.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_log.h"

class DALI {
private:
    enum {
        COMMAND_BROADCAST = 0b11111111,
        COMMAND_GRUPO = 0b10000001,
        RESET = 0b00100000,
        INITIALISE = 0b10100101,
        RANDOMISE = 0b10100111,
        SEARCHADDRH = 0b10110001,
        SEARCHADDRM = 0b10110011,
        SEARCHADDRL = 0b10110101,
        COMPARE = 0b10101001,
        WITHDRAW = 0b10101011,
        TERMINATE = 0b10100001,
        SHORT_POWER = 0b00000000,
        PROGRAM_SHORT_ADDRESS = 0b10110111,
        VERIFY_SHORT_ADDRESS = 0b10111001,
        CONTROL_BROADCAST = 0b11111110,
    };

public:

    #define IS_INVERTED

    #ifdef IS_INVERTED
    #define DALI_HIGH                   0
    #define DALI_LOW                    1
    #else
    #define DALI_HIGH                   1
    #define DALI_LOW                    0
    #endif

    DALI();                          // Default constructor
    DALI(gpio_num_t txPin, gpio_num_t rxPin);
    ~DALI();                          // Destructor for cleanup
    void begin(bool* is_isr_handler);
    
    int initNodes(const uint8_t* addresses, uint8_t numAddresses);
    int initNodes(uint8_t address);
    void turnOff(uint8_t nodeAddress);
    void setMax(uint8_t nodeAddress);
    void setValue(uint8_t nodeAddress, uint8_t value);
    void sendCommandPublic(uint8_t command, uint8_t data);
    void sendCommandPublic32(uint8_t command1, uint8_t data1, uint8_t command2, uint8_t data2);
    void task_delayMicroseconds(uint32_t microseconds);
    void task_delay(uint32_t milliseconds);
    int scanAssignedShortAddresses(uint8_t* foundAddresses, uint8_t maxAddresses);

    // Receiver control
    void query(uint8_t shortAddress, uint8_t queryCommand);
    // QueueHandle_t rxFrameQueue;           // Queue for received frames (each is uint32_t)

    void disableRxInterrupt();
    void enableRxInterrupt();

    // Bus-busy / collision statistics (readable by application)
    struct BusStats {
        uint32_t collisions;        // frames aborted mid-tx due to collision
        uint32_t busyWaits;         // times we had to wait for bus idle before tx
        uint32_t retryExhausted;    // frames dropped after all retries failed
    };
    // const BusStats& getBusStats() const { return busStats_; }
    // void clearBusStats()               { busStats_ = {}; }

    // Tuning knobs (set before begin() or at runtime)
    uint8_t  txMaxRetries   = 5;      // max send attempts per frame
    uint32_t txBackoffMinTE = 2;      // minimum back-off in TE units (~416 µs each)
    uint32_t txBackoffMaxTE = 8;      // maximum back-off in TE units

private:
    bool sendZero(void);
    bool sendOne(void);
    bool sendZeroWithCheck();
    bool sendOneWithCheck();
  
    // Returns false if a collision was detected; aborts TX immediately.
    bool sendCommandWithRetry(uint8_t command, uint8_t data);
    bool sendCommand32WithRetry(uint8_t command1, uint8_t data1,
                                uint8_t command2, uint8_t data2);

    bool sendCommand(uint8_t command, uint8_t data);
    void sendCommand32(uint8_t command1, uint8_t data1, uint8_t command2, uint8_t data2);
    bool sendSearchAddr(uint32_t addr);
    bool sendProgramShortAddr(uint8_t nodeNumber);
    void withdrawNode(uint32_t addr);

    // Bus-busy helpers
    bool isBusBusy();
    bool waitForBusFree(uint8_t maxRetries, uint32_t retryDelayMs);

    gpio_num_t txPin;
    gpio_num_t rxPin;
    // BusStats   busStats_ = {};
    static const char *TAG;

};

#endif // __DALI_H__