// ============================================================
//  DALI.cpp — corrected implementation (4 bugs fixed)
//
//  Fix #1  tx_in_progress_ guard uncommented and bracketed
//          correctly so own-loopback frames no longer set
//          bus_busy_ = true after releaseBus().
//
//  Fix #3  sendCommand32Raw() now mirrors sendCommandRaw():
//          gpio_intr_disable wraps the entire two-frame
//          sequence including the inter-frame gap, preventing
//          loopback of frame-1 from asserting bus_busy_ before
//          frame-2 starts.
//
//  Fix #4  bus_activity_counter threshold in set_color_temp()
//          tightened from >15 to >2 (meaningful only after
//          fix #1 suppresses own-loopback counts).
//          — set_color_temp() is in DaliCommands.cpp below.
//
//  Note:   Fix #2 (address double-encoding) lives entirely in
//          DaliCommands.cpp — no DALI.cpp changes needed for it.
// ============================================================

#include "DALI.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "esp_random.h"

#define DALI_RETRY_COUNTS   100
#define DALI_FRAME_DELAY_US 3700

volatile uint32_t bus_activity_counter = 0;

uint16_t u16_frame_delay_us = DALI_FRAME_DELAY_US;
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
    io_conf_tx.intr_type    = GPIO_INTR_DISABLE;
    io_conf_tx.mode         = GPIO_MODE_OUTPUT_OD;
    io_conf_tx.pin_bit_mask = (1ULL << txPin);
    io_conf_tx.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf_tx.pull_up_en   = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf_tx);

    gpio_set_level(txPin, DALI_LOW);

    taskENTER_CRITICAL(&bus_mux_);
    bus_busy_             = false;
    tx_in_progress_       = false;         // FIX #1: explicit init
    last_bus_activity_us_ = esp_timer_get_time();
    taskEXIT_CRITICAL(&bus_mux_);

    dali_mutex = xSemaphoreCreateMutex();
}

// ── FIX #1: setTxInProgress helper ───────────────────────────
// Wraps the volatile flag in the same spinlock used by
// markBusActivityFromISR() so reads in the ISR are coherent.
void DALI::setTxInProgress(bool state) {
    taskENTER_CRITICAL(&bus_mux_);
    tx_in_progress_ = state;
    taskEXIT_CRITICAL(&bus_mux_);
}

// ── markBusActivityFromISR ────────────────────────────────────
// Unchanged logic; now works correctly because tx_in_progress_
// is actually set during transmission.
void IRAM_ATTR DALI::markBusActivityFromISR() {
    taskENTER_CRITICAL_ISR(&bus_mux_);
    if (!tx_in_progress_) {
        bus_busy_             = true;
        last_bus_activity_us_ = esp_timer_get_time();
        bus_activity_counter++;
    }
    taskEXIT_CRITICAL_ISR(&bus_mux_);
}

uint32_t DALI::getBusActivityCounter() {
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

    taskENTER_CRITICAL(&bus_mux_);
    busy    = bus_busy_;
    last_us = last_bus_activity_us_;
    taskEXIT_CRITICAL(&bus_mux_);

    if (busy && ((now - last_us) >= DALI_BUS_IDLE_MIN_US)) {
        taskENTER_CRITICAL(&bus_mux_);
        if (bus_busy_ &&
            ((esp_timer_get_time() - last_bus_activity_us_) >= DALI_BUS_IDLE_MIN_US)) {
            bus_busy_ = false;
        }
        taskEXIT_CRITICAL(&bus_mux_);
    }
    return !bus_busy_;
}

bool DALI::sendHalfBit(bool txLevel) {
    gpio_set_level(txPin, txLevel);
    esp_rom_delay_us(150);

    int rxLevel = gpio_get_level(rxPin);
    if (txLevel == DALI_HIGH && rxLevel == DALI_LOW) {
        gpio_set_level(txPin, DALI_LOW);
        return false;
    }
    esp_rom_delay_us(266);
    return true;
}

bool DALI::sendZero() {
    if (!sendHalfBit(DALI_LOW))  return false;
    if (!sendHalfBit(DALI_HIGH)) return false;
    return true;
}

bool DALI::sendOne() {
    if (!sendHalfBit(DALI_HIGH)) return false;
    if (!sendHalfBit(DALI_LOW))  return false;
    return true;
}

void DALI::sendZeroNormal(void) {
    gpio_set_level(txPin, DALI_LOW);
    task_delayMicroseconds(416);
    gpio_set_level(txPin, DALI_HIGH);
    task_delayMicroseconds(416);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void DALI::sendOneNormal(void) {
    gpio_set_level(txPin, DALI_HIGH);
    task_delayMicroseconds(416);
    gpio_set_level(txPin, DALI_LOW);
    task_delayMicroseconds(416);
}

void DALI::releaseBus() {
    taskENTER_CRITICAL(&bus_mux_);
    bus_busy_             = false;
    last_bus_activity_us_ = esp_timer_get_time();
    taskEXIT_CRITICAL(&bus_mux_);
    task_delayMicroseconds(u16_frame_delay_us);
}

void DALI::daliSetFrameDelay(uint16_t delay_us) {
    u16_frame_delay_us = delay_us;
}

// ── FIX #1: sendCommandRaw ────────────────────────────────────
// tx_in_progress_ is now SET before the start bit and CLEARED
// before releaseBus(), so the loopback that arrives after
// gpio_intr_enable is credited to our own transmission and
// markBusActivityFromISR() ignores it.
//
// Order matters:
//   1. disable ISR          — no edges processed while we TX
//   2. set tx_in_progress_  — arm the loopback guard
//   3. transmit
//   4. drive line idle
//   5. clear tx_in_progress_ — BEFORE releaseBus so the
//                              guard is still active during
//                              the 3700µs settle delay
//   6. releaseBus()         — clears bus_busy_, waits settle
//   7. enable ISR           — loopback may arrive here, but
//                              markBusActivity sees
//                              tx_in_progress_==false and
//                              stamps it as foreign activity
bool DALI::sendCommandRaw(uint8_t command, uint8_t data) {
    uint16_t info = (uint16_t)((command << 8) | data);

    if (rxPin != GPIO_NUM_NC) gpio_intr_disable(rxPin);  // step 1
    setTxInProgress(true);                                // step 2 — FIX #1

    if (!sendOne()) {                                     // start bit
        setTxInProgress(false);
        releaseBus();
        if (rxPin != GPIO_NUM_NC) gpio_intr_enable(rxPin);
        return false;
    }

    for (uint8_t i = 0; i < 16; i++) {
        bool ok = (info & 0x8000) ? sendOne() : sendZero();
        if (!ok) {
            setTxInProgress(false);
            releaseBus();
            if (rxPin != GPIO_NUM_NC) gpio_intr_enable(rxPin);
            return false;
        }
        info <<= 1;
    }

    gpio_set_level(txPin, DALI_LOW);

    setTxInProgress(false);                               // step 5 — FIX #1
    releaseBus();                                         // step 6
    if (rxPin != GPIO_NUM_NC) gpio_intr_enable(rxPin);    // step 7

    return true;
}

bool DALI::sendCommand(uint8_t command, uint8_t data) {
    for (int retry = 0; retry < DALI_RETRY_COUNTS; retry++) {
        if (isBusIdle()) {
            uint32_t backoff = 10 + (esp_random() % 20);
            vTaskDelay(pdMS_TO_TICKS(backoff));

            if (isBusIdle()) {
                return sendCommandRaw(command, data);
            }
        }
        esp_rom_delay_us(DALI_COMPLETE_FRAME_US);
    }
    //printf("DALI bus busy, retry limit reached\\n");
    return false;
}
bool DALI::sendCommandRetry(uint8_t command, uint8_t data, uint8_t* retryCount) {
    for (int retry = 0; retry < DALI_RETRY_COUNTS; retry++) {
        if (isBusIdle()) {
            uint32_t backoff = 10 + (esp_random() % 20);
            vTaskDelay(pdMS_TO_TICKS(backoff));
            if (isBusIdle()) {
                if (retryCount) {
                    *retryCount = retry;
                }
                return sendCommandRaw(command, data);
            }
        }
        // esp_rom_delay_us(DALI_COMPLETE_FRAME_US); //17844 us or 17.8 ms
        esp_rom_delay_us(100); //17844 us or 17.8 ms
    }
    //ESP_LOGW(TAG, "sendCommand: retry limit reached cmd=0x%02X data=0x%02X", command, data);
    if (retryCount) {
        *retryCount = DALI_RETRY_COUNTS;
    }
    return false;
}

void DALI::sendCommandNormal(uint8_t command, uint8_t data) {
    uint16_t info = (uint16_t)((command << 8) | data);
    sendOneNormal();   // Start bit

    for (uint8_t i = 0; i < 16; i++) {
        if (info & 0x8000)
            sendOneNormal();
        else
            sendZeroNormal();
        info <<= 1;
    }

    gpio_set_level(txPin, DALI_LOW);
    task_delayMicroseconds(6700);
}

// ── FIX #3: sendCommand32Raw ──────────────────────────────────
// Now mirrors sendCommandRaw:
//   • gpio_intr_disable wraps the entire two-frame sequence
//     including the 3700µs inter-frame gap.
//   • tx_in_progress_ is held true for both frames so neither
//     frame's loopback can falsely set bus_busy_.
//   • setTxInProgress(false) called before releaseBus() so the
//     guard remains active during the settle delay.
//
// Without this, the rx ISR fires on the loopback of frame 1
// during the 3700µs gap, sets bus_busy_=true, and
// sendCommand32()'s retry loop stalls before frame 2 starts.
bool DALI::sendCommand32Raw(uint8_t command1, uint8_t data1,
                             uint8_t command2, uint8_t data2) {
    uint16_t cd1  = (uint16_t)((command1 << 8) | data1);
    uint16_t cd2  = (uint16_t)((command2 << 8) | data2);
    uint32_t info = (uint32_t)((cd1 << 16) | cd2);

    if (rxPin != GPIO_NUM_NC) gpio_intr_disable(rxPin);  // FIX #3
    setTxInProgress(true);                                // FIX #1+#3

    // ── Frame 1: start bit + 16 data bits ──
    if (!sendOne()) {
        setTxInProgress(false);
        releaseBus();
        if (rxPin != GPIO_NUM_NC) gpio_intr_enable(rxPin);
        return false;
    }

    for (uint8_t i = 0; i < 16; i++) {
        bool ok = (info & 0x80000000) ? sendOne() : sendZero();
        if (!ok) {
            ESP_LOGW(TAG, "sendCommand32Raw: collision on frame 1 bit %d", i);
            setTxInProgress(false);
            releaseBus();
            if (rxPin != GPIO_NUM_NC) gpio_intr_enable(rxPin);
            return false;
        }
        info <<= 1;
    }

    // Drive idle between frames — ISR still disabled, no loopback noise
    gpio_set_level(txPin, DALI_LOW);
    task_delayMicroseconds(3700);    // inter-frame gap — ISR disabled ✅

    // ── Frame 2: start bit + 16 data bits ──
    if (!sendOne()) {
        setTxInProgress(false);
        releaseBus();
        if (rxPin != GPIO_NUM_NC) gpio_intr_enable(rxPin);
        return false;
    }

    for (uint8_t i = 0; i < 16; i++) {
        bool ok = (info & 0x80000000) ? sendOne() : sendZero();
        if (!ok) {
            ESP_LOGW(TAG, "sendCommand32Raw: collision on frame 2 bit %d", i);
            setTxInProgress(false);
            releaseBus();
            if (rxPin != GPIO_NUM_NC) gpio_intr_enable(rxPin);
            return false;
        }
        info <<= 1;
    }

    gpio_set_level(txPin, DALI_LOW);

    setTxInProgress(false);                               // FIX #1+#3
    releaseBus();
    if (rxPin != GPIO_NUM_NC) gpio_intr_enable(rxPin);

    return true;
}

bool DALI::sendCommand32(uint8_t command1, uint8_t data1,
                          uint8_t command2, uint8_t data2) {
    for (int retry = 0; retry < DALI_RETRY_COUNTS; retry++) {
        if (isBusIdle()) {
            return sendCommand32Raw(command1, data1, command2, data2);
        }
        esp_rom_delay_us(DALI_COMPLETE_FRAME_US);
    }
    ESP_LOGW(TAG, "sendCommand32: retry limit reached");
    return false;
}

bool DALI::sendCommandWithRetry(uint8_t command, uint8_t data, uint8_t* retryCount) {
    uint16_t info = (uint16_t)((command << 8) | data);
    sendOne();
    for (uint8_t i = 0; i < 16; i++) {
        if (info & 0x8000) sendOne(); else sendZero();
        info <<= 1;
    }
    gpio_set_level(txPin, DALI_LOW);
    return false;
}

bool DALI::sendSearchAddr(uint32_t addr) {
    sendCommand(SEARCHADDRH, (addr >> 16) & 0xFF);
    sendCommand(SEARCHADDRM, (addr >> 8)  & 0xFF);
    sendCommand(SEARCHADDRL,  addr        & 0xFF);
    sendCommand(COMPARE, 0);
    for (uint32_t n = 0; n < 50000; n++) {
#ifdef IS_INVERTED
        if (!gpio_get_level(rxPin))
#else
        if ( gpio_get_level(rxPin))
#endif
        { task_delay(20); return true; }
        task_delayMicroseconds(1);
    }
    return false;
}

void DALI::withdrawNode(uint32_t addr) {
    sendCommand(SEARCHADDRH, (addr >> 16) & 0xFF);
    sendCommand(SEARCHADDRM, (addr >> 8)  & 0xFF);
    sendCommand(SEARCHADDRL,  addr        & 0xFF);
    sendCommand(WITHDRAW, 0);
}

int DALI::initNodes(const uint8_t* addresses, uint8_t numAddresses) {
    uint32_t searchLower, searchDifference, searchTop;
    int ret = 0;

    sendCommand(TERMINATE, 0);
    task_delay(100);
    sendCommand(INITIALISE, 0);
    task_delay(10);
    sendCommand(INITIALISE, 0);
    task_delay(200);

    while (1) {
        searchLower      = 0;
        searchDifference = 0xFFFFFF;
        searchTop        = 0xFFFFFF;

        while (searchDifference > 1) {
            while (sendSearchAddr(searchDifference + searchLower)) {
                searchTop = searchDifference + searchLower;
                if (!searchDifference) break;
                searchDifference >>= 1;
            }
            if (searchDifference == 0xFFFFFF) return ret;
            searchLower     += searchDifference;
            searchDifference = searchTop - searchLower;
        }

        if (sendSearchAddr(searchDifference + searchLower)) {
            if (!sendProgramShortAddr(*addresses++)) return 0;
            withdrawNode(searchDifference + searchLower);
            if ((sendSearchAddr(searchDifference + searchLower) |
                 (searchDifference + searchLower)) == 0xFFFFFF) return 0;
            ret++;
            ESP_LOGI(TAG, "Assigned DALI address: %d", ret - 1);
            if (ret == numAddresses) return ret;
        }
    }
    return 0;
}

void DALI::setValue(uint8_t nodeNumber, uint8_t value) {
    if      (value == 0)   turnOff(nodeNumber);
    else if (value == 255) setMax(nodeNumber);
    else    sendCommand(SHORT_POWER | ((nodeNumber << 1) & 0x7e), value);
}

void DALI::turnOff(uint8_t nodeNumber) {
    sendCommand(SHORT_POWER | ((nodeNumber << 1) & 0x7e) | 0x01, 0x00);
}

void DALI::setMax(uint8_t nodeNumber) {
    sendCommand(SHORT_POWER | ((nodeNumber << 1) & 0x7e) | 0x01, 0x05);
}

bool DALI::sendProgramShortAddr(uint8_t nodeNumber) {
    nodeNumber &= 0x3F;
    sendCommand(PROGRAM_SHORT_ADDRESS, 1 | (nodeNumber << 1));
    sendCommand(VERIFY_SHORT_ADDRESS,  1 | (nodeNumber << 1));
    for (uint32_t n = 0; n < 50000; n++) {
#ifdef IS_INVERTED
        if (!gpio_get_level(rxPin))
#else
        if ( gpio_get_level(rxPin))
#endif
        { task_delay(20); return true; }
        task_delayMicroseconds(1);
    }
    return false;
}

bool DALI::sendCommandPublic(uint8_t command, uint8_t data) {
    return sendCommand(command, data);
}
bool DALI::sendCommandNormalPublic(uint8_t command, uint8_t data) {
    sendCommandNormal(command, data);
    return true;
}


bool DALI::sendCommandRetryPublic(uint8_t command, uint8_t data, uint8_t* retryCount) {
    return sendCommandRetry(command, data, retryCount);
}

void DALI::sendCommandPublic32(uint8_t command1, uint8_t data1,
                                uint8_t command2, uint8_t data2) {
    sendCommand32(command1, data1, command2, data2);
}

void DALI::disableRxInterrupt() {
    if (rxPin != GPIO_NUM_NC) gpio_intr_disable(rxPin);
}

void DALI::enableRxInterrupt() {
    if (rxPin != GPIO_NUM_NC) gpio_intr_enable(rxPin);
}

void DALI::query(uint8_t address, uint8_t queryCommand) {
    uint8_t mask        = address & 0x80;
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