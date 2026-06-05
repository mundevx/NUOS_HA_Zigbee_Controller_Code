#include "DALI.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "esp_random.h"

#define DALI_RETRY_COUNTS   50
#define DALI_FRAME_DELAY_US 3700

volatile uint32_t bus_activity_counter = 0;

uint16_t u16_frame_delay_us = DALI_FRAME_DELAY_US; // Default inter-frame delay, can be adjusted by daliSetFrameDelay()
const char *DALI::TAG = "DALI";
SemaphoreHandle_t dali_mutex;

DALI::DALI() : txPin(GPIO_NUM_NC), rxPin(GPIO_NUM_NC) {}
DALI::DALI(gpio_num_t txPin, gpio_num_t rxPin) : txPin(txPin), rxPin(rxPin) {}
DALI::~DALI() {}

void DALI::task_delay(uint32_t milliseconds) {
    vTaskDelay(milliseconds / portTICK_PERIOD_MS);
}

void DALI::task_delayMicroseconds(uint32_t microseconds) {
    esp_rom_delay_us(microseconds);
}

void DALI::begin(bool* is_isr_handler) {
    gpio_config_t io_conf_tx = {};
    io_conf_tx.intr_type = GPIO_INTR_DISABLE;
    io_conf_tx.mode = GPIO_MODE_OUTPUT_OD;
    io_conf_tx.pin_bit_mask = (1ULL << txPin);
    io_conf_tx.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf_tx.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf_tx);

    gpio_set_level(txPin, DALI_LOW);

    taskENTER_CRITICAL(&bus_mux_);
    bus_busy_ = false;
    last_bus_activity_us_ = esp_timer_get_time();
    taskEXIT_CRITICAL(&bus_mux_);
    dali_mutex = xSemaphoreCreateMutex();
}

// void IRAM_ATTR DALI::markBusActivityFromISR() {
//     taskENTER_CRITICAL_ISR(&bus_mux_);
//     if (!tx_in_progress_) {          // ← ignore our own loopback
//         bus_busy_ = true;
//         last_bus_activity_us_ = esp_timer_get_time();
//     }
//     taskEXIT_CRITICAL_ISR(&bus_mux_);
// }

void IRAM_ATTR DALI::markBusActivityFromISR()
{
    taskENTER_CRITICAL_ISR(&bus_mux_);

    if (!tx_in_progress_)
    {
        bus_busy_ = true;
        last_bus_activity_us_ = esp_timer_get_time();
        bus_activity_counter++;
    }

    taskEXIT_CRITICAL_ISR(&bus_mux_);
}
uint32_t DALI::getBusActivityCounter()
{
    uint32_t cnt;

    taskENTER_CRITICAL(&bus_mux_);
    cnt = bus_activity_counter;
    taskEXIT_CRITICAL(&bus_mux_);

    return cnt;
}
bool DALI::isBusIdle() {
    int64_t now = esp_timer_get_time();
    int64_t last_us;
    bool    busy;

    // Only task context calls this
    taskENTER_CRITICAL(&bus_mux_);
    busy    = bus_busy_;
    last_us = last_bus_activity_us_;
    taskEXIT_CRITICAL(&bus_mux_);

    // Only use time in non‑critical section
    if (busy && ((now - last_us) >= DALI_BUS_IDLE_MIN_US)) {
        // Defer the actual mutation
        taskENTER_CRITICAL(&bus_mux_);
        // Only clear if no one else updated in the meantime
        if (bus_busy_ && ((esp_timer_get_time() - last_bus_activity_us_) >= DALI_BUS_IDLE_MIN_US)) {
            bus_busy_ = false;
        }
        taskEXIT_CRITICAL(&bus_mux_);
    }

    return !bus_busy_;
}


bool DALI::sendHalfBit(bool txLevel)
{
    gpio_set_level(txPin, txLevel);

    // wait for line settle
    esp_rom_delay_us(150);

    int rxLevel = gpio_get_level(rxPin);

    // collision detect
    if (txLevel == DALI_HIGH && rxLevel == DALI_LOW) {
        //ESP_EARLY_LOGW(TAG, "DALI collision");
        gpio_set_level(txPin, DALI_LOW);
        // releaseBus();
        return false;
    }

    esp_rom_delay_us(266);

    return true;
}
bool DALI::sendZero()
{
    if (!sendHalfBit(DALI_LOW)) return false;
    if (!sendHalfBit(DALI_HIGH)) return false;
    return true;
}

bool DALI::sendOne()
{
    if (!sendHalfBit(DALI_HIGH)) return false;
    if (!sendHalfBit(DALI_LOW)) return false;
    return true;
}
// bool DALI::sendZero(void) {
//     gpio_set_level(txPin, DALI_LOW);
//     task_delayMicroseconds(416);
//     gpio_set_level(txPin, DALI_HIGH);
//     task_delayMicroseconds(416);
//     return true;
// }

// bool DALI::sendOne(void) {
//     gpio_set_level(txPin, DALI_HIGH);
//     task_delayMicroseconds(416);
//     gpio_set_level(txPin, DALI_LOW);
//     task_delayMicroseconds(416);
//     return true;
// }

// bool DALI::sendCommandRaw(uint8_t command, uint8_t data) {
//     uint16_t info = (uint16_t)((command << 8) | data);

//     if (!sendOne()) {
//         return false;
//     }

//     for (uint8_t i = 0; i < 16; i++) {

//         bool ok;

//         if (info & 0x8000)
//             ok = sendOne();
//         else
//             ok = sendZero();

//         if (!ok) {
//             printf("Collision detected\\n");
//             gpio_set_level(txPin, DALI_LOW); 
//             releaseBus();
//             task_delayMicroseconds(3700);
//             // But SIMULTANEOUSLY, the loopback edge arrives at RX ISR:
//             markBusActivityFromISR(); // bus_busy_ = true  ← sets true again!
//             return false;
//         }

//         info <<= 1;
//     }
//     gpio_set_level(txPin, DALI_LOW); 
//     releaseBus();  // Add this line!
//     task_delayMicroseconds(3700);
//     // But SIMULTANEOUSLY, the loopback edge arrives at RX ISR:
//     //markBusActivityFromISR(); // bus_busy_ = true  ← sets true again!

//     return true;
// }
bool DALI::sendCommandRaw(uint8_t command, uint8_t data) {
    uint16_t info = (uint16_t)((command << 8) | data);
    // Block loopback ISR for entire TX + settle window
    if (rxPin != GPIO_NUM_NC) gpio_intr_disable(rxPin);

    if (!sendOne()) {
        releaseBus();
        
        if (rxPin != GPIO_NUM_NC) gpio_intr_enable(rxPin);
        return false;
    }

    for (uint8_t i = 0; i < 16; i++) {
        bool ok = (info & 0x8000) ? sendOne() : sendZero();
        if (!ok) {
            releaseBus();
            if (rxPin != GPIO_NUM_NC) gpio_intr_enable(rxPin);
            return false;
        }
        info <<= 1;
    }

    gpio_set_level(txPin, DALI_LOW);
    releaseBus();
    //task_delayMicroseconds(3700);     // loopback arrives here — ISR disabled ✅
    if (rxPin != GPIO_NUM_NC) gpio_intr_enable(rxPin);

    return true;
}

// bool DALI::sendCommand(uint8_t command, uint8_t data) {
//     uint16_t info = (uint16_t)((command << 8) | data);
//     sendOne();   // Start bit

//     for (uint8_t i = 0; i < 16; i++) {
//         if (info & 0x8000)
//             sendOne();
//         else
//             sendZero();
//         info <<= 1;
//     }

//     gpio_set_level(txPin, DALI_LOW);
//     task_delayMicroseconds(2700);
//     return true;
// }
bool DALI::sendCommand(uint8_t command, uint8_t data) {
    for (int retry = 0; retry < DALI_RETRY_COUNTS; retry++) {
        if (isBusIdle()) {

            uint32_t backoff =
                1 + (esp_random() % 10);

            vTaskDelay(pdMS_TO_TICKS(backoff));

            if (isBusIdle()) {
                return sendCommandRaw(command, data);
            }
        }
        esp_rom_delay_us(DALI_COMPLETE_FRAME_US);
    }
    printf("DALI bus busy, retry limit reached\\n");
    return false;
}
// bool acquire_dt8_lock()
// {
//     uint32_t timeout = esp_timer_get_time() + 500000;

//     while(esp_timer_get_time() < timeout)
//     {
//         if(send_lock_request())
//         {
//             return true;
//         }

//         vTaskDelay(pdMS_TO_TICKS(10));
//     }

//     return false;
// }
void DALI::releaseBus()
{
    
    taskENTER_CRITICAL(&bus_mux_);
    bus_busy_ = false;
    last_bus_activity_us_ = esp_timer_get_time();
    taskEXIT_CRITICAL(&bus_mux_);
    task_delayMicroseconds(u16_frame_delay_us);
}


void DALI::daliSetFrameDelay(uint16_t delay_us)
{
    
    u16_frame_delay_us = delay_us;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
bool DALI::sendCommandWithRetry(uint8_t command, uint8_t data) {
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
    //task_delayMicroseconds(3700);
    return false;
}


bool DALI::sendCommand32Raw(uint8_t command1, uint8_t data1, uint8_t command2, uint8_t data2) {
    uint16_t cd1 = (uint16_t)((command1 << 8) | data1);
    uint16_t cd2 = (uint16_t)((command2 << 8) | data2);
    uint32_t info = (uint32_t)((cd1 << 16) | cd2);

    if (!sendOne()) {
        return false;
    }

    for (uint8_t i = 0; i < 32; i++) {

        bool ok;

        if (info & 0x80000000)
            ok = sendOne();
        else
            ok = sendZero();

        if (!ok) {
            printf("Collision detected\\n");
            releaseBus();
            //task_delayMicroseconds(3700);
            return false;
        }

        info <<= 1;

        if (i == 15) {
            gpio_set_level(txPin, DALI_LOW);
            task_delayMicroseconds(3700);
            if (!sendOne()) {
                return false;
            }
        }
    }
    gpio_set_level(txPin, DALI_LOW); 
    releaseBus();  // Add this line!
    //task_delayMicroseconds(3700);

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////
bool DALI::sendCommand32(uint8_t command1, uint8_t data1, uint8_t command2, uint8_t data2) {

    // uint16_t cd1 = (uint16_t)((command1 << 8) | data1);
    // uint16_t cd2 = (uint16_t)((command2 << 8) | data2);
    // uint32_t info = (uint32_t)((cd1 << 16) | cd2);

    // sendOne();   // Start bit

    // for (uint8_t i = 0; i < 32; i++) {
    //     if (info & 0x80000000)
    //         sendOne();
    //     else
    //         sendZero();
    //     info <<= 1;

    //     if (i == 15) {
    //         gpio_set_level(txPin, DALI_LOW);
    //         task_delayMicroseconds(3700);
    //         task_delay(1);
    //         sendOne();
    //     }
    // }

    // gpio_set_level(txPin, DALI_LOW);
    // task_delayMicroseconds(3700);
    for (int retry = 0; retry < DALI_RETRY_COUNTS; retry++) {
        // if (isBusIdle()) {
        //     task_delay(2);  // Wait 2ms to ensure bus idle
            if (isBusIdle()) {
                bool result = sendCommand32Raw(command1, data1, command2, data2);
                //task_delay(20);  // FIX: Add 10ms inter-frame delay after send
                return result;
            }

        esp_rom_delay_us(DALI_COMPLETE_FRAME_US);
    }
    //printf("DALI bus busy, retry limit reached\\n");
    return false;
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
    // sendCommand(COMMAND_BROADCAST, RESET);
    // task_delay(10);
    // sendCommand(COMMAND_BROADCAST, RESET);
    // task_delay(300);
   
    // Terminate any nodes in configuration
    sendCommand(TERMINATE, 0);
    task_delay(100);
   
    // Initialize modules
    sendCommand(INITIALISE, 0);
    task_delay(10);
    sendCommand(INITIALISE, 0);
    task_delay(200);
   
    // Randomize node addresses
    // sendCommand(RANDOMISE, 0);
    // task_delay(10);
    // sendCommand(RANDOMISE, 0);
    // task_delay(200);
   
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
bool DALI::sendCommandPublic(uint8_t command, uint8_t data) {
    return sendCommand(command, data);
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
    //(addr << 1) | 0x01
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

