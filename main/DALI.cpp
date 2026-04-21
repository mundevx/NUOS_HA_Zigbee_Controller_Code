/*
Copyright (c) 2019 Boot&Work Corp., S.L. All rights reserved
Converted to ESP-IDF for ESP32-H2

This library is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
*/

#include "DALI.h"
// At top with other includes
#include "esp_random.h"
#include "esp_log.h"

const char *DALI::TAG               = "DALI";

// ////////////////////////////////////////////////////////////////////////////////////////////////////
DALI::DALI() : txPin(GPIO_NUM_NC),  rxPin(GPIO_NUM_NC) {}

// ////////////////////////////////////////////////////////////////////////////////////////////////////
DALI::DALI(gpio_num_t txPin, gpio_num_t rxPin) : txPin(txPin), rxPin(rxPin) {}

////////////////////////////////////////////////////////////////////////////////////////////////////
DALI::~DALI() {
   
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void DALI::task_delay(uint32_t milliseconds) {
    vTaskDelay(milliseconds / portTICK_PERIOD_MS);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void DALI::task_delayMicroseconds(uint32_t microseconds) {
    esp_rom_delay_us(microseconds);
}
 

void DALI::begin(bool* is_isr_handler) {
// Configure TX pin as output
    gpio_config_t io_conf_tx = {};
    io_conf_tx.intr_type = GPIO_INTR_DISABLE;
    io_conf_tx.mode = GPIO_MODE_OUTPUT;
    io_conf_tx.pin_bit_mask = (1ULL << txPin);
    io_conf_tx.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf_tx.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf_tx);

    // // Configure RX pin as input
    // gpio_config_t io_conf_rx = {};
    // io_conf_rx.intr_type = GPIO_INTR_DISABLE;
    // io_conf_rx.mode = GPIO_MODE_INPUT;
    // io_conf_rx.pin_bit_mask = (1ULL << rxPin);
    // io_conf_rx.pull_down_en = GPIO_PULLDOWN_DISABLE;
    // io_conf_rx.pull_up_en = GPIO_PULLUP_DISABLE;
    // gpio_config(&io_conf_rx);

    // Set initial TX state (idle: release bus)
    gpio_set_level(txPin, DALI_HIGH);   // Idle = HIGH (depending on polarity)
}


// Send a '0' bit (low-high) with collision check on the high phase
bool DALI::sendZeroWithCheck() {
    // First half: drive low (active) – safe, no collision possible
    gpio_set_level(txPin, DALI_LOW);   // DALI_LOW = 1 (active)
    esp_rom_delay_us(416);

    // Second half: release to high (inactive) – check if bus actually goes high
    gpio_set_level(txPin, DALI_HIGH);  // DALI_HIGH = 0 (released)
    esp_rom_delay_us(20);              // settle time
    int bus_level = gpio_get_level(rxPin);
    // Because of inversion: DALI_HIGH=0 means bus released (logic 1)
    // If another master is pulling low (DALI_LOW=1), rxPin will be 1, which is NOT DALI_HIGH.
    if (bus_level != DALI_HIGH) {
        ESP_LOGW(TAG, "Collision in sendZero: bus level %d, expected %d", bus_level, DALI_HIGH);
        return false;
    }
    esp_rom_delay_us(396);  // complete the half-bit
    return true;
}

// Send a '1' bit (high-low) with collision check on the first high phase
bool DALI::sendOneWithCheck() {
    // First half: release bus (high) – collision possible here
    gpio_set_level(txPin, DALI_HIGH);
    esp_rom_delay_us(20);
    if (gpio_get_level(rxPin) != DALI_HIGH) {
        ESP_LOGW(TAG, "Collision in sendOne (first half)");
        return false;
    }
    esp_rom_delay_us(396);

    // Second half: drive low – safe
    gpio_set_level(txPin, DALI_LOW);
    esp_rom_delay_us(416);
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool DALI::sendZero(void) {
    gpio_set_level(txPin, DALI_LOW);
    task_delayMicroseconds(416);
    gpio_set_level(txPin, DALI_HIGH);
    task_delayMicroseconds(416);
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool DALI::sendOne(void) {
    gpio_set_level(txPin, DALI_HIGH);
    task_delayMicroseconds(416);
    gpio_set_level(txPin, DALI_LOW);
    task_delayMicroseconds(416);
    return true;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
bool DALI::isBusBusy() {
    #ifdef IS_INVERTED
        // Inverted: idle = HIGH, busy = LOW
        return (gpio_get_level(rxPin) == DALI_LOW);
    #else
        // Non-inverted: idle = LOW, busy = HIGH
        return (gpio_get_level(rxPin) == DALI_HIGH);
    #endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// bool DALI::waitForBusFree(uint8_t maxTries, uint32_t delayUs) {
//     for (uint8_t attempt = 0; attempt < maxTries; attempt++) {
//         if (!isBusBusy()) {
//             return true;
//         }
//         ESP_LOGW(TAG, "DALI bus busy, retry %d/%d", attempt + 1, maxTries);
//         task_delayMicroseconds(delayUs);
//     }
//     ESP_LOGE(TAG, "DALI bus still busy after %d tries", maxTries);
//     return false;
// }
////////////////////////////////////////////////////////////////////////////////////////////////////
#define MAX_RETRIES 2
bool DALI::sendCommand(uint8_t command, uint8_t data) {
    // Disable RX interrupt to avoid processing our own edges
    //disableRxInterrupt();
    // if (!isBusBusy()) {
    //     return true;
    // }
    uint16_t info = (uint16_t)((command << 8) | data);
    sendOne();   // Start bit
    for (uint8_t i = 0; i < 16; i++) {
        if (info & 0x8000)
            sendOne();
        else
            sendZero();
        info <<= 1;
    } 
    gpio_set_level(txPin, DALI_LOW);
    task_delayMicroseconds(2700); 
    //enableRxInterrupt();
    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void DALI::sendCommand32(uint8_t command1, uint8_t data1, uint8_t command2, uint8_t data2) {
    //task_delay(5);

    uint16_t cd1 = (uint16_t)((command1 << 8) | data1);
    uint16_t cd2 = (uint16_t)((command2 << 8) | data2);
    uint32_t info = (uint32_t)((cd1 << 16) | cd2);

    sendOne();   // Start bit

    for (uint8_t i = 0; i < 32; i++) {
        if (info & 0x80000000)
            sendOne();
        else
            sendZero();
        info <<= 1;

        if (i == 15) {
            gpio_set_level(txPin, DALI_LOW);
            task_delayMicroseconds(1700);
            task_delay(1);
            sendOne();
        }
    }

    gpio_set_level(txPin, DALI_LOW);
    task_delayMicroseconds(2700);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool DALI::sendSearchAddr(uint32_t addr) {
    sendCommand(SEARCHADDRH, (addr >> 16) & 0xFF);
    sendCommand(SEARCHADDRM, (addr >> 8) & 0xFF);
    sendCommand(SEARCHADDRL, addr & 0xFF);
    sendCommand(COMPARE, 0);

    for (uint32_t n = 0; n < 50000; n++) {
    #ifdef IS_INVERTED
        if (!gpio_get_level(rxPin))
    #else
        if (gpio_get_level(rxPin))
    #endif
        {
            task_delay(20);
            return true;
        }
        task_delayMicroseconds(1);
    }
    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void DALI::withdrawNode(uint32_t addr) {
    sendCommand(SEARCHADDRH, (addr >> 16) & 0xFF);
    sendCommand(SEARCHADDRM, (addr >> 8) & 0xFF);
    sendCommand(SEARCHADDRL, addr & 0xFF);
    sendCommand(WITHDRAW, 0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
int DALI::initNodes(const uint8_t* addresses, uint8_t numAddresses) {
    uint32_t searchLower;
    uint32_t searchDifference;
    uint32_t searchTop;
    int ret = 0;

    // Reset modules
    sendCommand(COMMAND_BROADCAST, RESET);
    task_delay(10);
    sendCommand(COMMAND_BROADCAST, RESET);
    task_delay(300);
   
    // Terminate any nodes in configuration
    sendCommand(TERMINATE, 0);
    task_delay(100);
   
    // Initialize modules
    sendCommand(INITIALISE, 0);
    task_delay(10);
    sendCommand(INITIALISE, 0);
    task_delay(200);
   
    // Randomize node addresses
    sendCommand(RANDOMISE, 0);
    task_delay(10);
    sendCommand(RANDOMISE, 0);
    task_delay(200);
   
    while (1) {
        searchLower = 0;
        searchDifference = 0xFFFFFF;
        searchTop = 0xFFFFFF;
       
        while (searchDifference > 1) {
            while (sendSearchAddr(searchDifference + searchLower)) {
                searchTop = searchDifference + searchLower;
                if (!searchDifference)
                    break;
                searchDifference >>= 1;
            }
           
            if (searchDifference == 0xFFFFFF)
                return ret;
           
            searchLower += searchDifference;
            searchDifference = searchTop - searchLower;
        }
       
        if (sendSearchAddr(searchDifference + searchLower)) {
            if (!sendProgramShortAddr(*addresses++))
                return 0;
           
            withdrawNode(searchDifference + searchLower);
           
            if ((sendSearchAddr(searchDifference + searchLower) |
                 (searchDifference + searchLower)) == 0xFFFFFF)
                return 0;
           
            ret++;
            printf("Assigned DALI address: %d\n", ret - 1);
            if (ret == numAddresses)
                return ret;
        }
    }
    return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void DALI::setValue(uint8_t nodeNumber, uint8_t value) {
    if (value == 0) {
        return turnOff(nodeNumber);
    } else if (value == 255) {
        setMax(nodeNumber);
    } else {
        sendCommand(SHORT_POWER | ((nodeNumber << 1) & 0x7e), value);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void DALI::turnOff(uint8_t nodeNumber) {
    sendCommand(SHORT_POWER | ((nodeNumber << 1) & 0x7e) | 0x01, 0x00);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void DALI::setMax(uint8_t nodeNumber) {
    sendCommand(SHORT_POWER | ((nodeNumber << 1) & 0x7e) | 0x01, 0x05);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool DALI::sendProgramShortAddr(uint8_t nodeNumber) {
    nodeNumber &= 0x3F;
    sendCommand(PROGRAM_SHORT_ADDRESS, 1 | (nodeNumber << 1));
    sendCommand(VERIFY_SHORT_ADDRESS, 1 | (nodeNumber << 1));

    for (uint32_t n = 0; n < 50000; n++) {
    #ifdef IS_INVERTED
        if (!gpio_get_level(rxPin))
    #else
        if (gpio_get_level(rxPin))
    #endif
        {
            task_delay(20);
            return true;
        }
        task_delayMicroseconds(1);
    }
    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void DALI::sendCommandPublic(uint8_t command, uint8_t data) {
    sendCommand(command, data);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void DALI::sendCommandPublic32(uint8_t command1, uint8_t data1,
                                uint8_t command2, uint8_t data2) {
    sendCommand32(command1, data1, command2, data2);
}

void DALI::disableRxInterrupt() {
    if (rxPin != GPIO_NUM_NC) {
        gpio_intr_disable(rxPin);
    }
}

void DALI::enableRxInterrupt() {
    if (rxPin != GPIO_NUM_NC) {
        gpio_intr_enable(rxPin);
    }
}


void DALI::query(uint8_t address, uint8_t queryCommand) {
    
    // Get the upper bit
    uint8_t mask = address & 0x80;
    // Change address to have 1 in LSb to signify 'standard command'
    uint8_t new_address = mask | ((address << 1) + 1);
    sendCommandPublic(new_address, queryCommand);  
}

#define QUERY_STATUS_2 0xBB

int DALI::scanAssignedShortAddresses(uint8_t* foundAddresses, uint8_t maxAddresses) {
    int found = 0;
 
    for (uint8_t addr = 0; addr < 64; ++addr) {
        vTaskDelay(1 / portTICK_PERIOD_MS);
       
        // Send QUERY_STATUS command to short address
        sendCommand(addr, QUERY_STATUS_2);
       
        // Wait for response
        for (uint32_t n = 0; n < 100000; n++) {
    #ifdef IS_INVERTED
            if (!gpio_get_level(rxPin))
    #else
            if (gpio_get_level(rxPin))
    #endif
            {
                foundAddresses[found++] = addr;
                ESP_LOGD(TAG, "Found address: %d", addr >> 1);
                break;
            }
            task_delayMicroseconds(1);
        }
    }

    return found;

}

