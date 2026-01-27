/*
Copyright (c) 2019 Boot&Work Corp., S.L. All rights reserved
Converted to ESP-IDF for ESP32-H2

This library is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
*/

#include "DALI.h"

#define IS_INVERTED

#ifdef IS_INVERTED
#define DALI_HIGH 0
#define DALI_LOW 1
#else
#define DALI_HIGH 1
#define DALI_LOW 0
#endif

const char *DALI::TAG = "DALI";

////////////////////////////////////////////////////////////////////////////////////////////////////
DALI::DALI() : txPin(GPIO_NUM_NC), rxPin(GPIO_NUM_NC) {}

////////////////////////////////////////////////////////////////////////////////////////////////////
DALI::DALI(gpio_num_t txPin, gpio_num_t rxPin) : txPin(txPin), rxPin(rxPin) {}

////////////////////////////////////////////////////////////////////////////////////////////////////
void DALI::task_delay(uint32_t milliseconds) {
    vTaskDelay(milliseconds / portTICK_PERIOD_MS);
    //esp_rom_delay_us(milliseconds*1000);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void DALI::task_delayMicroseconds(uint32_t microseconds) {
    esp_rom_delay_us(microseconds);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void DALI::begin() {
    // Configure TX pin as output
    gpio_config_t io_conf_tx = {};
    io_conf_tx.intr_type = GPIO_INTR_DISABLE;
    io_conf_tx.mode = GPIO_MODE_OUTPUT;
    io_conf_tx.pin_bit_mask = (1ULL << txPin);
    io_conf_tx.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf_tx.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf_tx);

    // Configure RX pin as input
    gpio_config_t io_conf_rx = {};
    io_conf_rx.intr_type = GPIO_INTR_DISABLE;
    io_conf_rx.mode = GPIO_MODE_INPUT;
    io_conf_rx.pin_bit_mask = (1ULL << rxPin);
    io_conf_rx.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf_rx.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf_rx);

    // Set initial TX state
    gpio_set_level(txPin, DALI_LOW);
   
    ESP_LOGI(TAG, "DALI initialized - TX: GPIO%d, RX: GPIO%d", txPin, rxPin);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void DALI::sendZero(void) {
    gpio_set_level(txPin, DALI_LOW);
    task_delayMicroseconds(416);
    gpio_set_level(txPin, DALI_HIGH);
    task_delayMicroseconds(416);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void DALI::sendOne(void) {
    gpio_set_level(txPin, DALI_HIGH);
    task_delayMicroseconds(416);
    gpio_set_level(txPin, DALI_LOW);
    task_delayMicroseconds(416);
}

bool DALI::isBusIdle() {
    // Sample RX pin multiple times to confirm stable idle state (HIGH = idle)
    for (int i = 0; i < 10; i++) {
        if (gpio_get_level(rxPin) != DALI_HIGH) {
            return false;  // Bus busy (LOW detected)
        }
        task_delayMicroseconds(10);  // Small debounce delay
    }
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void DALI::sendCommand(uint8_t command, uint8_t data) {
    uint16_t info = (uint16_t)((command << 8) | data);
    #ifdef IS_WAIT_FOR_BUS_IDLE
    // Wait for bus idle (max 100ms timeout)
    uint32_t timeout = 10000;  // ~100ms at 10us ticks
    while (!isBusIdle() && timeout--) {
        task_delayMicroseconds(10);
    }
    if (timeout == 0) {
        ESP_LOGE(TAG, "Bus idle timeout in sendCommand(0x%02X, 0x%02X)", command, data);
        return;  // Abort to avoid collision
    }   
    #endif
    // Send start bit
    sendOne();
   
    // Send 16 bits
    for (uint8_t i = 0; i < 16; i++) {
        if (info & 0x8000)
            sendOne();
        else
            sendZero();
        info <<= 1;
    }
   
    // Stop bit
    gpio_set_level(txPin, DALI_LOW);
    task_delayMicroseconds(5000);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void DALI::sendCommand32(uint8_t command1, uint8_t data1, uint8_t command2, uint8_t data2) {
    task_delay(5);
   
    uint16_t cd1 = (uint16_t)((command1 << 8) | data1);
    uint16_t cd2 = (uint16_t)((command2 << 8) | data2);
    uint32_t info = (uint32_t)((cd1 << 16) | cd2);
   
    // Send start bit
    sendOne();
   
    // Send 32 bits
    for (uint8_t i = 0; i < 32; i++) {
        if (info & 0x80000000)
            sendOne();
        else
            sendZero();
        info <<= 1;
       
        // Mid-command stop
        if (i == 15) {
            gpio_set_level(txPin, DALI_LOW);
            task_delayMicroseconds(1700);
            task_delay(1);
            sendOne();
        }
    }
   
    // Final stop bit
    gpio_set_level(txPin, DALI_LOW);
    task_delayMicroseconds(1700);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool DALI::sendSearchAddr(uint32_t addr) {
    // Send address high
    sendCommand(SEARCHADDRH, (addr >> 16) & 0xFF);
    // task_delay(1);
   
    // Send address mid
    sendCommand(SEARCHADDRM, (addr >> 8) & 0xFF);
    // task_delay(1);
   
    // Send address low
    sendCommand(SEARCHADDRL, addr & 0xFF);
    // task_delay(1);
   
    // Send compare command
    sendCommand(COMPARE, 0);
   
    // Wait for response
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
    // Send address high
    sendCommand(SEARCHADDRH, (addr >> 16) & 0xFF);
    // Send address mid
    sendCommand(SEARCHADDRM, (addr >> 8) & 0xFF);
    // Send address low
    sendCommand(SEARCHADDRL, addr & 0xFF);
    // Send withdraw command
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
   
    // Send program short address command
    sendCommand(PROGRAM_SHORT_ADDRESS, 1 | (nodeNumber << 1));
   
    // Send verify short address command
    sendCommand(VERIFY_SHORT_ADDRESS, 1 | (nodeNumber << 1));
   
    // Wait for response
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

////////////////////////////////////////////////////////////////////////////////////////////////////
#define QUERY_STATUS 0xBB

int DALI::scanAssignedShortAddresses(uint8_t* foundAddresses, uint8_t maxAddresses) {
    int found = 0;
   
    for (uint8_t addr = 0; addr < 64; ++addr) {
        vTaskDelay(1 / portTICK_PERIOD_MS);
       
        // Send QUERY_STATUS command to short address
        sendCommand(addr, QUERY_STATUS);
       
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