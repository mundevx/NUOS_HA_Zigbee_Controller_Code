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

#define TASK_PRIORITY_DALI_RX_INTR_TASK   11
#define TASK_PRIORITY_DALI_RX_FRAME_QUEUE 13



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
    bool sendCommandPublic(uint8_t command, uint8_t data);
    void sendCommandPublic32(uint8_t command1, uint8_t data1, uint8_t command2, uint8_t data2);
    void task_delayMicroseconds(uint32_t microseconds);
    void task_delay(uint32_t milliseconds);
    int scanAssignedShortAddresses(uint8_t* foundAddresses, uint8_t maxAddresses);

    // Receiver control
    void query(uint8_t shortAddress, uint8_t queryCommand);
    // QueueHandle_t rxFrameQueue;           // Queue for received frames (each is uint32_t)

    void disableRxInterrupt();
    void enableRxInterrupt();

    void IRAM_ATTR markBusActivityFromISR();
    uint32_t getBusActivityCounter();
    bool isBusIdle();
    bool sendCommandWithRetry(uint8_t command, uint8_t data);
    void daliSetFrameDelay(uint16_t delay_us);
private:
    volatile bool tx_in_progress_ = false;
    bool sendZero(void);
    bool sendOne(void);
    void releaseBus();
    // bool sendCommand32WithRetry(uint8_t command1, uint8_t data1,
    //                             uint8_t command2, uint8_t data2);
    bool sendCommand32Raw(uint8_t command1, uint8_t data1, uint8_t command2, uint8_t data2);                            

    bool sendCommand(uint8_t command, uint8_t data);
    bool sendCommand32(uint8_t command1, uint8_t data1, uint8_t command2, uint8_t data2);
    bool sendSearchAddr(uint32_t addr);
    bool sendProgramShortAddr(uint8_t nodeNumber);
    void withdrawNode(uint32_t addr);

    
    bool waitBusIdleStable(uint32_t stable_us, uint32_t timeout_us);
    volatile bool bus_busy_ = false;
    volatile int64_t last_bus_activity_us_ = 0;
    portMUX_TYPE bus_mux_ = portMUX_INITIALIZER_UNLOCKED;

    static constexpr uint32_t DALI_HALF_BIT_US = 416;
    static constexpr uint32_t DALI_BIT_US = 2 * DALI_HALF_BIT_US;      // ~832 us
    static constexpr uint32_t DALI_FRAME_BITS = 17;                    // 1 start + 16 data
    static constexpr uint32_t DALI_FORWARD_FRAME_US = DALI_FRAME_BITS * DALI_BIT_US; // ~14144 us
    static constexpr uint32_t DALI_POST_TX_IDLE_US = 3700;             // your existing stop/settle
    static constexpr uint32_t DALI_COMPLETE_FRAME_US = DALI_FORWARD_FRAME_US + DALI_POST_TX_IDLE_US;
    static constexpr uint32_t DALI_BUS_IDLE_MIN_US = 5000;             // choose 9TE or your required value
    
    bool sendCommandRaw(uint8_t command, uint8_t data);       // add this
    void setTxInProgress(bool state);

    gpio_num_t txPin;
    gpio_num_t rxPin;
    // BusStats   busStats_ = {};
    static const char *TAG;
    //bool sendHalfBit(bool tx_level);
    bool sendHalfBit(bool txLevel);

};

#endif // __DALI_H__