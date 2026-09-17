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

void DALI::sendZeroNormal(void) {
    gpio_set_level(txPin, DALI_LOW);
    task_delayMicroseconds(416);
    gpio_set_level(txPin, DALI_HIGH);
    task_delayMicroseconds(416);
}

void DALI::sendOneNormal(void) {
    gpio_set_level(txPin, DALI_HIGH);
    task_delayMicroseconds(416);
    gpio_set_level(txPin, DALI_LOW);
    task_delayMicroseconds(416);
}

void DALI::sendBit(bool bit) {
    if (bit) {
        sendZeroNormal();
    } else {
        sendOneNormal();
    }
}

bool DALI::sendCommandRaw(uint8_t command, uint8_t data) {
    uint16_t info = (uint16_t)((command << 8) | data);
    //tx_in_progress_ = true;
    // Block loopback ISR for entire TX + settle window
    if (rxPin != GPIO_NUM_NC) gpio_intr_disable(rxPin);

    taskENTER_CRITICAL(&bus_mux_);
    if (!sendOne()) {
       // tx_in_progress_ = false;
        taskEXIT_CRITICAL(&bus_mux_);
        releaseBus();

        if (rxPin != GPIO_NUM_NC) gpio_intr_enable(rxPin);
        return false;
    }



    for (uint8_t i = 0; i < 16; i++) {
        bool ok = (info & 0x8000) ? sendOne() : sendZero();
        if (!ok) {
           // tx_in_progress_ = false;
            taskEXIT_CRITICAL(&bus_mux_);
            releaseBus();
            if (rxPin != GPIO_NUM_NC) gpio_intr_enable(rxPin);
            return false;
        }
        info <<= 1;
    }

    gpio_set_level(txPin, DALI_LOW);
    taskEXIT_CRITICAL(&bus_mux_);
    releaseBus();
    //task_delayMicroseconds(3700);     // loopback arrives here — ISR disabled ✅
    if (rxPin != GPIO_NUM_NC) gpio_intr_enable(rxPin);

    return true;
}

bool DALI::sendCommand(uint8_t command, uint8_t data) {
    for (int retry = 0; retry < DALI_RETRY_COUNTS; retry++) {
        if (isBusIdle()) {
            uint32_t backoff = 1 + (esp_random() % 10);
            // uint32_t backoff = 2 + (esp_random() % 25);
            vTaskDelay(pdMS_TO_TICKS(backoff));

            if (isBusIdle()) {
                return sendCommandRaw(command, data);
            }
        }
        // CHANGED: was esp_rom_delay_us(DALI_COMPLETE_FRAME_US) - a busy-wait that never
        // yields to FreeRTOS. With DALI_RETRY_COUNTS=50 and a ~17.8ms frame time, a stuck
        // bus could burn ~890ms of pure CPU time with no scheduler yield, starving the
        // IDLE task and tripping the task watchdog (seen as a crash inside sendCommand).
        // vTaskDelay waits roughly the same real time but actually yields the core.
        vTaskDelay(pdMS_TO_TICKS((DALI_COMPLETE_FRAME_US + 999) / 1000));
    }
    printf("DALI bus busy, retry limit reached\\n");
    return false;
}

void DALI::sendCommandNormal(uint8_t command, uint8_t data) {
    uint16_t info = (uint16_t)((command << 8) | data);
    taskENTER_CRITICAL(&bus_mux_);
    sendOneNormal();

    for (uint8_t i = 0; i < 16; i++) {
        if (info & 0x8000)
            sendOneNormal();
        else
            sendZeroNormal();
        info <<= 1;
    }

    gpio_set_level(txPin, DALI_LOW);
    taskEXIT_CRITICAL(&bus_mux_);
    task_delayMicroseconds(6700);
}

void DALI::releaseBus()
{
    
    taskENTER_CRITICAL(&bus_mux_);
    bus_busy_ = false;
    last_bus_activity_us_ = esp_timer_get_time();
    taskEXIT_CRITICAL(&bus_mux_);
    task_delayMicroseconds(u16_frame_delay_us);
}


void DALI::daliSetFrameDelay(uint16_t delay_us){

    u16_frame_delay_us = delay_us;
}

bool DALI::sendCommandWithRetry(uint8_t command, uint8_t data) {
    uint16_t info = (uint16_t)((command << 8) | data);
    taskENTER_CRITICAL(&bus_mux_);
    sendOne();   // Start bit
    for (uint8_t i = 0; i < 16; i++) {
        if (info & 0x8000)
            sendOne();
        else
            sendZero();
        info <<= 1;
    } 
    gpio_set_level(txPin, DALI_LOW);
    taskEXIT_CRITICAL(&bus_mux_);
    //task_delayMicroseconds(3700);
    return false;
}


bool DALI::sendCommand32Raw(uint8_t command1, uint8_t data1, uint8_t command2, uint8_t data2) {
    uint16_t cd1 = (uint16_t)((command1 << 8) | data1);
    uint16_t cd2 = (uint16_t)((command2 << 8) | data2);
    uint32_t info = (uint32_t)((cd1 << 16) | cd2);

    taskENTER_CRITICAL(&bus_mux_);
    if (!sendOne()) {
        taskEXIT_CRITICAL(&bus_mux_);
        return false;
    }
// 1 EP 36
// 240.46, 24046

//UPS 2 END POINT
    for (uint8_t i = 0; i < 32; i++) {
        bool ok;
        if (info & 0x80000000)
            ok = sendOne();
        else
            ok = sendZero();

        if (!ok) {
            printf("Collision detected\\n");
            taskEXIT_CRITICAL(&bus_mux_);
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
    taskEXIT_CRITICAL(&bus_mux_);
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

        // CHANGED: same watchdog-starvation fix as sendCommand() above - yield instead
        // of busy-waiting the CPU while polling for the bus to go idle.
        vTaskDelay(pdMS_TO_TICKS((DALI_COMPLETE_FRAME_US + 999) / 1000));
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


int DALI::getNextFreeShortAddress() {
    for (int addr = 0; addr < 64; addr++) {
        if (!isShortAddressUsed(addr))
            return addr;
    }
    return -1;
}

#define QUERY_CONTROL_GEAR_PRESENT   0x91

bool DALI::isShortAddressUsed(uint8_t shortAddr) {
    uint8_t command = (shortAddr << 1) | 0x01;
    sendCommand(command, QUERY_CONTROL_GEAR_PRESENT);

    for (uint32_t i = 0; i < 50000; i++) {
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


int DALI::commissionNewNodes() {
    uint32_t searchLower, searchUpper, searchCurrent;
    int assigned = 0;

    if (xSemaphoreTake(dali_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "commissionNewNodes: Could not acquire dali_mutex");
        return 0;
    }

    sendCommand(TERMINATE, 0);
    task_delay(100);

    sendCommand(INITIALISE, 0xFF);
    task_delay(20);
    sendCommand(INITIALISE, 0xFF);
    task_delay(300);

    sendCommand(RANDOMISE, 0);
    task_delay(20);
    sendCommand(RANDOMISE, 0);
    task_delay(300);

    while (1) {
        searchLower = 0x000000;
        searchUpper = 0xFFFFFF;

        if (!sendSearchAddr(searchUpper)) {
            ESP_LOGI(TAG, "No unaddressed DALI devices found responding to search.");
            break;
        }

        while (searchLower < searchUpper) {
            searchCurrent = searchLower + ((searchUpper - searchLower) / 2);

            if (sendSearchAddr(searchCurrent)) {
                searchUpper = searchCurrent;
            } else {
                searchLower = searchCurrent + 1;
            }
        }

        uint32_t foundRandomAddr = searchLower;

        int shortAddr = getNextFreeShortAddress();
        if (shortAddr < 0) {
            ESP_LOGE(TAG, "No free short addresses left (Bus full at 64 devices).");
            break;
        }

        sendSearchAddr(foundRandomAddr);

        if (!sendProgramShortAddr(shortAddr)) {
            ESP_LOGE(TAG, "Failed to program short address %d", shortAddr);
            break; 
        }

        ESP_LOGI(TAG, "Assigned Short Address %d to RandomAddr 0x%06lX", shortAddr, foundRandomAddr);

        withdrawNode(foundRandomAddr);
        assigned++;
    }

    sendCommand(TERMINATE, 0);
    xSemaphoreGive(dali_mutex);

    ESP_LOGI(TAG, "DALI Addressing Completed. Total assigned: %d", assigned);
    return assigned;
}

void DALI::factoryResetDaliDrivers(){

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
   
    //Randomize node addresses
    sendCommand(RANDOMISE, 0);
    task_delay(10);
    sendCommand(RANDOMISE, 0);
    task_delay(200);
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
bool DALI::sendCommandNormalPublic(uint8_t command, uint8_t data) {
    sendCommandNormal(command, data);
    return true;
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

bool DALI::waitBusIdleStable(uint32_t stable_us, uint32_t timeout_us) {
    int64_t deadline = esp_timer_get_time() + timeout_us;
    while (esp_timer_get_time() < deadline) {
        if (isBusIdle()) {
            esp_rom_delay_us(stable_us);
            if (isBusIdle()) return true;
        }
        esp_rom_delay_us(100);
    }
    return false;
}

#define QUERY_STATUS_2 0xBB

int DALI::scanAssignedShortAddresses(uint8_t* foundAddresses, uint8_t maxAddresses) {
    int found = 0;
    for (uint8_t addr = 0; addr < 64; ++addr) {
        vTaskDelay(1 / portTICK_PERIOD_MS);
        sendCommand(addr, QUERY_STATUS_2);
        for (uint32_t n = 0; n < 100000; n++) {
#ifdef IS_INVERTED
            if (!gpio_get_level(rxPin))
#else
            if ( gpio_get_level(rxPin))
#endif
            { foundAddresses[found++] = addr; break; }
            task_delayMicroseconds(1);
        }
    }
    return found;
}

#define QUERY_STATUS                0x90
#define QUERY_CONTROL_GEAR_PRESENT  0x91

bool DALI::waitForResponse() {
    for (uint32_t i = 0; i < 50000; i++) {
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

#define DALI_BACKWARD_TE_US      416
#define DALI_RESPONSE_TIMEOUT_US 12000

static inline int IRAM_ATTR fastRead(gpio_num_t pin) {
    int level = gpio_get_level(pin);
#ifdef IS_INVERTED
    level = !level;
#endif
    return level;
}

int IRAM_ATTR DALI::waitForResponseValue(uint8_t *value) {
    uint8_t data = 0;

    int64_t start = esp_timer_get_time();

    while ((esp_timer_get_time() - start) < DALI_RESPONSE_TIMEOUT_US) {
        if (fastRead(rxPin))
            break;
    }

    if ((esp_timer_get_time() - start) >= DALI_RESPONSE_TIMEOUT_US)
        return -1;

    int64_t t0 = esp_timer_get_time();

    for (int bit = 7; bit >= 0; bit--) {
        while (esp_timer_get_time() < (t0 + 3 * DALI_BACKWARD_TE_US / 2 +
                                       (7 - bit) * 2 * DALI_BACKWARD_TE_US));
        int first = fastRead(rxPin);

        while (esp_timer_get_time() < (t0 + 5 * DALI_BACKWARD_TE_US / 2 +
                                       (7 - bit) * 2 * DALI_BACKWARD_TE_US));
        int second = fastRead(rxPin);

        if ((first == 0) && (second == 1)) {
            data |= (1 << bit);
        } else if ((first == 1) && (second == 0)) {
            // bit = 0
        } else {
            return -2;
        }
    }
    
    *value = data;
    return 0;
}

int32_t DALI::queryPowerOnLevel(uint8_t shortAddr) {
    return queryGear(shortAddr, QUERY_POWER_ON_LEVEL);
}
int32_t DALI::queryFadeTimeFadeRate(uint8_t shortAddr) {
    return queryGear(shortAddr, QUERY_FADE_TIME_FADE_RATE);
}
int32_t DALI::queryDeviceType(uint8_t shortAddr) {
    return queryGear(shortAddr, QUERY_DEVICE_TYPE);
}
int32_t DALI::queryNextDeviceType(uint8_t shortAddr) {
    return queryGear(shortAddr, QUERY_NEXT_DEVICE_TYPE);
}
int32_t DALI::queryGearFeatures(uint8_t shortAddr) {
    return queryGear(shortAddr, QUERY_GEAR_FEATURES);
}
int32_t DALI::queryDeviceInGroupA(uint8_t shortAddr) {
    return queryGear(shortAddr, QUERY_GROUPS_0_TO_7);
}
int32_t DALI::queryDeviceInGroupB(uint8_t shortAddr) {
    return queryGear(shortAddr, QUERY_GROUPS_8_TO_15);
}

int DALI::readExistingDrivers(uint8_t *addressList, int maxDevices) {
    int count = 0;
    for (uint8_t shortAddr = 0; shortAddr < 64; shortAddr++) {
        uint8_t daliAddr = (shortAddr << 1) | 0x01;
        sendCommand(daliAddr, QUERY_CONTROL_GEAR_PRESENT);

        if (waitForResponse()) {
            if (count < maxDevices)
                addressList[count] = shortAddr;
            count++;
        }
        task_delay(10);
    }
    printf("Total Devices Found = %d\n", count);
    return count;
}

int32_t DALI::queryGear(uint8_t shortAddr, uint8_t query_cmd) {
    query(shortAddr, query_cmd);
    return -1;
}

bool DALI::resetDriver(uint8_t shortAddr) {
    if (shortAddr > 63)
        return false;

    sendCommand(0xff, RESET);
    task_delay(10);
    sendCommand(0xff, RESET);
    task_delay(300);

    sendCommand(TERMINATE, 0);
    task_delay(100);

    sendCommand(INITIALISE, 0);
    task_delay(10);
    sendCommand(INITIALISE, 0);
    task_delay(200);

    sendCommand(RANDOMISE, 0);
    task_delay(10);
    sendCommand(RANDOMISE, 0);
    task_delay(200);
    return true;
}

void DALI::sendData(uint8_t value) {}

#define DALI_CMD_SET_DTR           0xA3
#define DALI_CMD_STORE_DTR_AS_SA   128
#define DALI_MASK_ADDRESS          0xFF

bool DALI::clearShortAddress(uint8_t shortAddr) {
    if (shortAddr > 63) {
        ESP_LOGE(TAG, "clearShortAddress: Invalid short address %d", shortAddr);
        return false;
    }

    uint8_t addressedByte = (shortAddr << 1) | 0x01;

    if (xSemaphoreTake(dali_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        ESP_LOGW(TAG, "clearShortAddress: Could not acquire dali_mutex");
        return false;
    }

    if (!sendCommand(DALI_CMD_SET_DTR, DALI_MASK_ADDRESS)) {
        ESP_LOGW(TAG, "clearShortAddress: Failed to set DTR to 0xFF");
        xSemaphoreGive(dali_mutex);
        return false;
    }

    task_delayMicroseconds(u16_frame_delay_us);

    if (!sendCommand(addressedByte, DALI_CMD_STORE_DTR_AS_SA)) {
        ESP_LOGW(TAG, "clearShortAddress: Failed to send first STORE_DTR command");
        xSemaphoreGive(dali_mutex);
        return false;
    }

    task_delayMicroseconds(u16_frame_delay_us);

    if (!sendCommand(addressedByte, DALI_CMD_STORE_DTR_AS_SA)) {
        ESP_LOGW(TAG, "clearShortAddress: Failed to send second STORE_DTR command");
        xSemaphoreGive(dali_mutex);
        return false;
    }

    xSemaphoreGive(dali_mutex);
    ESP_LOGI(TAG, "Successfully cleared short address %d", shortAddr);
    return true;
}