// // ============================================================
// //  DALI.cpp — corrected implementation (4 bugs fixed)
// //
// //  Fix #1  tx_in_progress_ guard uncommented and bracketed
// //          correctly so own-loopback frames no longer set
// //          bus_busy_ = true after releaseBus().
// //
// //  Fix #3  sendCommand32Raw() now mirrors sendCommandRaw():
// //          gpio_intr_disable wraps the entire two-frame
// //          sequence including the inter-frame gap, preventing
// //          loopback of frame-1 from asserting bus_busy_ before
// //          frame-2 starts.
// //
// //  Fix #4  bus_activity_counter threshold in set_color_temp()
// //          tightened from >15 to >2 (meaningful only after
// //          fix #1 suppresses own-loopback counts).
// //          — set_color_temp() is in DaliCommands.cpp below.
// //
// //  Note:   Fix #2 (address double-encoding) lives entirely in
// //          DaliCommands.cpp — no DALI.cpp changes needed for it.
// // ============================================================

// #include "DALI.h"
// #include <stdint.h>
// #include <stdbool.h>
// #include "driver/gpio.h"
// #include "esp_timer.h"
// #include "esp_rom_sys.h"
// #include "esp_log.h"
// #include "esp_random.h"




// #define DALI_RETRY_COUNTS   100
// #define DALI_FRAME_DELAY_US 3700

// volatile uint32_t bus_activity_counter = 0;

// uint16_t u16_frame_delay_us = DALI_FRAME_DELAY_US;
// const char *DALI::TAG = "DALI";
// SemaphoreHandle_t dali_mutex;

// DALI::DALI() : txPin(GPIO_NUM_NC), rxPin(GPIO_NUM_NC) {}
// DALI::DALI(gpio_num_t txPin, gpio_num_t rxPin) : txPin(txPin), rxPin(rxPin) {}
// DALI::~DALI() {}

// void DALI::task_delay(uint32_t milliseconds) {
//     vTaskDelay(milliseconds / portTICK_PERIOD_MS);
// }

// void DALI::task_delayMicroseconds(uint32_t microseconds) {
//     esp_rom_delay_us(microseconds);
// }

// void DALI::begin(bool* is_isr_handler) {
//     gpio_config_t io_conf_tx = {};
//     io_conf_tx.intr_type    = GPIO_INTR_DISABLE;
//     io_conf_tx.mode         = GPIO_MODE_OUTPUT_OD;
//     io_conf_tx.pin_bit_mask = (1ULL << txPin);
//     io_conf_tx.pull_down_en = GPIO_PULLDOWN_DISABLE;
//     io_conf_tx.pull_up_en   = GPIO_PULLUP_DISABLE;
//     gpio_config(&io_conf_tx);

//     gpio_set_level(txPin, DALI_LOW);

//     taskENTER_CRITICAL(&bus_mux_);
//     bus_busy_             = false;
//     tx_in_progress_       = false;         // FIX #1: explicit init
//     last_bus_activity_us_ = esp_timer_get_time();
//     taskEXIT_CRITICAL(&bus_mux_);

//     dali_mutex = xSemaphoreCreateMutex();
// }

// // ── FIX #1: setTxInProgress helper ───────────────────────────
// // Wraps the volatile flag in the same spinlock used by
// // markBusActivityFromISR() so reads in the ISR are coherent.
// void DALI::setTxInProgress(bool state) {
//     taskENTER_CRITICAL(&bus_mux_);
//     tx_in_progress_ = state;
//     taskEXIT_CRITICAL(&bus_mux_);
// }

// // ── markBusActivityFromISR ────────────────────────────────────
// // Unchanged logic; now works correctly because tx_in_progress_
// // is actually set during transmission.
// void IRAM_ATTR DALI::markBusActivityFromISR() {

//     //taskENTER_CRITICAL_ISR(&bus_mux_);
//     portENTER_CRITICAL_ISR(&bus_mux_);
//     if (!tx_in_progress_) {
//         bus_busy_             = true;
//         last_bus_activity_us_ = esp_timer_get_time();
//         bus_activity_counter++;
//     }
//     //taskEXIT_CRITICAL_ISR(&bus_mux_);
//     portEXIT_CRITICAL_ISR(&bus_mux_);
// }

// // void IRAM_ATTR DALI::markBusActivityFromISR() {
// //     taskENTER_CRITICAL_ISR(&bus_mux_);
// //     if (!tx_in_progress_) {
// //         bus_busy_             = true;
// //         last_bus_activity_us_ = esp_timer_get_time();
// //         bus_activity_counter++;
// //     }
// //     taskEXIT_CRITICAL_ISR(&bus_mux_);
// // }


// uint32_t DALI::getBusActivityCounter() {
//     uint32_t cnt;
//     taskENTER_CRITICAL(&bus_mux_);
//     cnt = bus_activity_counter;
//     taskEXIT_CRITICAL(&bus_mux_);
//     return cnt;
// }

// bool DALI::isBusIdle() {
//     int64_t now = esp_timer_get_time();
//     int64_t last_us;
//     bool    busy;

//     taskENTER_CRITICAL(&bus_mux_);
//     busy    = bus_busy_;
//     last_us = last_bus_activity_us_;
//     taskEXIT_CRITICAL(&bus_mux_);

//     if (busy && ((now - last_us) >= DALI_BUS_IDLE_MIN_US)) {
//         taskENTER_CRITICAL(&bus_mux_);
//         if (bus_busy_ &&
//             ((esp_timer_get_time() - last_bus_activity_us_) >= DALI_BUS_IDLE_MIN_US)) {
//             bus_busy_ = false;
//         }
//         taskEXIT_CRITICAL(&bus_mux_);
//     }
//     return !bus_busy_;
// }

// bool DALI::sendHalfBit(bool txLevel) {
//     gpio_set_level(txPin, txLevel);
//     esp_rom_delay_us(150);

//     int rxLevel = gpio_get_level(rxPin);
//     if (txLevel == DALI_HIGH && rxLevel == DALI_LOW) {
//         gpio_set_level(txPin, DALI_LOW);
//         return false;
//     }
//     esp_rom_delay_us(266);
//     return true;
// }

// bool DALI::sendZero() {
//     if (!sendHalfBit(DALI_LOW))  return false;
//     if (!sendHalfBit(DALI_HIGH)) return false;
//     return true;
// }

// bool DALI::sendOne() {
//     if (!sendHalfBit(DALI_HIGH)) return false;
//     if (!sendHalfBit(DALI_LOW))  return false;
//     return true;
// }



// void DALI::sendZeroNormal(void) {
//     gpio_set_level(txPin, DALI_LOW);
//     task_delayMicroseconds(416);
//     gpio_set_level(txPin, DALI_HIGH);
//     task_delayMicroseconds(416);
// }

// ////////////////////////////////////////////////////////////////////////////////////////////////////
// void DALI::sendOneNormal(void) {
//     gpio_set_level(txPin, DALI_HIGH);
//     task_delayMicroseconds(416);
//     gpio_set_level(txPin, DALI_LOW);
//     task_delayMicroseconds(416);
// }


// void DALI::sendBit(bool bit)
// {
//     if (bit){
//         // Manchester '1' = LOW -> HIGH
//         sendZeroNormal();
//     }else{
//         // Manchester '0' = HIGH -> LOW
//         sendOneNormal();
//     }
// }

// void DALI::releaseBus() {
//     taskENTER_CRITICAL(&bus_mux_);
//     bus_busy_             = false;
//     last_bus_activity_us_ = esp_timer_get_time();
//     taskEXIT_CRITICAL(&bus_mux_);
//     task_delayMicroseconds(u16_frame_delay_us);
// }

// void DALI::daliSetFrameDelay(uint16_t delay_us) {
//     u16_frame_delay_us = delay_us;
// }

// // ── FIX #1: sendCommandRaw ────────────────────────────────────
// // tx_in_progress_ is now SET before the start bit and CLEARED
// // before releaseBus(), so the loopback that arrives after
// // gpio_intr_enable is credited to our own transmission and
// // markBusActivityFromISR() ignores it.
// //
// // Order matters:
// //   1. disable ISR          — no edges processed while we TX
// //   2. set tx_in_progress_  — arm the loopback guard
// //   3. transmit
// //   4. drive line idle
// //   5. clear tx_in_progress_ — BEFORE releaseBus so the
// //                              guard is still active during
// //                              the 3700µs settle delay
// //   6. releaseBus()         — clears bus_busy_, waits settle
// //   7. enable ISR           — loopback may arrive here, but
// //                              markBusActivity sees
// //                              tx_in_progress_==false and
// //                              stamps it as foreign activity
// bool DALI::sendCommandRaw(uint8_t command, uint8_t data) {
//     uint16_t info = (uint16_t)((command << 8) | data);

//     if (rxPin != GPIO_NUM_NC) gpio_intr_disable(rxPin);  // step 1
//     setTxInProgress(true);                                // step 2 — FIX #1

//     if (!sendOne()) {                                     // start bit
//         setTxInProgress(false);
//         releaseBus();
//         if (rxPin != GPIO_NUM_NC) gpio_intr_enable(rxPin);
//         return false;
//     }

//     for (uint8_t i = 0; i < 16; i++) {
//         bool ok = (info & 0x8000) ? sendOne() : sendZero();
//         if (!ok) {
//             setTxInProgress(false);
//             releaseBus();
//             if (rxPin != GPIO_NUM_NC) gpio_intr_enable(rxPin);
//             return false;
//         }
//         info <<= 1;
//     }

//     gpio_set_level(txPin, DALI_LOW);

//     setTxInProgress(false);                               // step 5 — FIX #1
//     releaseBus();                                         // step 6
//     if (rxPin != GPIO_NUM_NC) gpio_intr_enable(rxPin);    // step 7

//     return true;
// }

// bool DALI::sendCommand(uint8_t command, uint8_t data) {
//     for (int retry = 0; retry < DALI_RETRY_COUNTS; retry++) {
//         if (isBusIdle()) {
//             uint32_t backoff = 10 + (esp_random() % 20);
//             vTaskDelay(pdMS_TO_TICKS(backoff));

//             if (isBusIdle()) {
//                 return sendCommandRaw(command, data);
//             }
//         }
//         esp_rom_delay_us(DALI_COMPLETE_FRAME_US);
//     }
//     return false;
// }
// bool DALI::sendCommandRetry(uint8_t command, uint8_t data, uint8_t* retryCount) {
//     for (int retry = 0; retry < DALI_RETRY_COUNTS; retry++) {
//         if (isBusIdle()) {
//             uint32_t backoff = 10 + (esp_random() % 20);
//             vTaskDelay(pdMS_TO_TICKS(backoff));
//             if (isBusIdle()) {
//                 if (retryCount) {
//                     *retryCount = retry;
//                 }
//                 return sendCommandRaw(command, data);
//             }
//         }
//         // esp_rom_delay_us(DALI_COMPLETE_FRAME_US); //17844 us or 17.8 ms
//         esp_rom_delay_us(100); //17844 us or 17.8 ms
//     }
//     //ESP_LOGW(TAG, "sendCommand: retry limit reached cmd=0x%02X data=0x%02X", command, data);
//     if (retryCount) {
//         *retryCount = DALI_RETRY_COUNTS;
//     }
//     return false;
// }

// void DALI::sendCommandNormal(uint8_t command, uint8_t data) {
//     uint16_t info = (uint16_t)((command << 8) | data);
//     sendOneNormal();   // Start bit

//     for (uint8_t i = 0; i < 16; i++) {
//         if (info & 0x8000)
//             sendOneNormal();
//         else
//             sendZeroNormal();
//         info <<= 1;
//     }

//     gpio_set_level(txPin, DALI_LOW);
//     task_delayMicroseconds(6700);
// }

// // ── FIX #3: sendCommand32Raw ──────────────────────────────────
// // Now mirrors sendCommandRaw:
// //   • gpio_intr_disable wraps the entire two-frame sequence
// //     including the 3700µs inter-frame gap.
// //   • tx_in_progress_ is held true for both frames so neither
// //     frame's loopback can falsely set bus_busy_.
// //   • setTxInProgress(false) called before releaseBus() so the
// //     guard remains active during the settle delay.
// //
// // Without this, the rx ISR fires on the loopback of frame 1
// // during the 3700µs gap, sets bus_busy_=true, and
// // sendCommand32()'s retry loop stalls before frame 2 starts.
// bool DALI::sendCommand32Raw(uint8_t command1, uint8_t data1,
//                              uint8_t command2, uint8_t data2) {
//     uint16_t cd1  = (uint16_t)((command1 << 8) | data1);
//     uint16_t cd2  = (uint16_t)((command2 << 8) | data2);
//     uint32_t info = (uint32_t)((cd1 << 16) | cd2);

//     if (rxPin != GPIO_NUM_NC) gpio_intr_disable(rxPin);  // FIX #3
//     setTxInProgress(true);                                // FIX #1+#3

//     // ── Frame 1: start bit + 16 data bits ──
//     if (!sendOne()) {
//         setTxInProgress(false);
//         releaseBus();
//         if (rxPin != GPIO_NUM_NC) gpio_intr_enable(rxPin);
//         return false;
//     }

//     for (uint8_t i = 0; i < 16; i++) {
//         bool ok = (info & 0x80000000) ? sendOne() : sendZero();
//         if (!ok) {
//             ESP_LOGW(TAG, "sendCommand32Raw: collision on frame 1 bit %d", i);
//             setTxInProgress(false);
//             releaseBus();
//             if (rxPin != GPIO_NUM_NC) gpio_intr_enable(rxPin);
//             return false;
//         }
//         info <<= 1;
//     }

//     // Drive idle between frames — ISR still disabled, no loopback noise
//     gpio_set_level(txPin, DALI_LOW);
//     task_delayMicroseconds(3700);    // inter-frame gap — ISR disabled ✅

//     // ── Frame 2: start bit + 16 data bits ──
//     if (!sendOne()) {
//         setTxInProgress(false);
//         releaseBus();
//         if (rxPin != GPIO_NUM_NC) gpio_intr_enable(rxPin);
//         return false;
//     }

//     for (uint8_t i = 0; i < 16; i++) {
//         bool ok = (info & 0x80000000) ? sendOne() : sendZero();
//         if (!ok) {
//             ESP_LOGW(TAG, "sendCommand32Raw: collision on frame 2 bit %d", i);
//             setTxInProgress(false);
//             releaseBus();
//             if (rxPin != GPIO_NUM_NC) gpio_intr_enable(rxPin);
//             return false;
//         }
//         info <<= 1;
//     }

//     gpio_set_level(txPin, DALI_LOW);

//     setTxInProgress(false);                               // FIX #1+#3
//     releaseBus();
//     if (rxPin != GPIO_NUM_NC) gpio_intr_enable(rxPin);

//     return true;
// }

// bool DALI::sendCommand32(uint8_t command1, uint8_t data1,
//                           uint8_t command2, uint8_t data2) {
//     for (int retry = 0; retry < DALI_RETRY_COUNTS; retry++) {
//         if (isBusIdle()) {
//             return sendCommand32Raw(command1, data1, command2, data2);
//         }
//         esp_rom_delay_us(DALI_COMPLETE_FRAME_US);
//     }
//     ESP_LOGW(TAG, "sendCommand32: retry limit reached");
//     return false;
// }

// bool DALI::sendCommandWithRetry(uint8_t command, uint8_t data, uint8_t* retryCount) {
//     uint16_t info = (uint16_t)((command << 8) | data);
//     sendOne();
//     for (uint8_t i = 0; i < 16; i++) {
//         if (info & 0x8000) sendOne(); else sendZero();
//         info <<= 1;
//     }
//     gpio_set_level(txPin, DALI_LOW);
//     return false;
// }

// // bool DALI::sendSearchAddr(uint32_t addr) {
// //     sendCommand(SEARCHADDRH, (addr >> 16) & 0xFF);
// //     sendCommand(SEARCHADDRM, (addr >> 8)  & 0xFF);
// //     sendCommand(SEARCHADDRL,  addr        & 0xFF);
// //     sendCommand(COMPARE, 0);
// //     for (uint32_t n = 0; n < 50000; n++) {
// // #ifdef IS_INVERTED
// //         if (!gpio_get_level(rxPin))
// // #else
// //         if ( gpio_get_level(rxPin))
// // #endif
// //         { task_delay(20); return true; }
// //         task_delayMicroseconds(1);
// //     }
// //     return false;
// // }
// bool DALI::sendSearchAddr(uint32_t addr) {
//     // Distribute the 24-bit address space across DTR registers with explicit pacing gaps
//     if (!sendCommand(SEARCHADDRH, (addr >> 16) & 0xFF)) return false;
//     task_delayMicroseconds(u16_frame_delay_us);
    
//     if (!sendCommand(SEARCHADDRM, (addr >> 8)  & 0xFF)) return false;
//     task_delayMicroseconds(u16_frame_delay_us);
    
//     if (!sendCommand(SEARCHADDRL,  addr        & 0xFF)) return false;
//     task_delayMicroseconds(u16_frame_delay_us);
    
//     // Send the final activation query
//     if (!sendCommand(COMPARE, 0)) return false;

//     // The driver must reply within 11.5ms (the standard DALI backward window)
//     // We sample cleanly up to ~25ms to account for circuit optocoupler lag
//     for (uint32_t n = 0; n < 25000; n++) {
// #ifdef IS_INVERTED
//         if (!gpio_get_level(rxPin))
// #else
//         if (gpio_get_level(rxPin))
// #endif
//         { 
//             // Device detected! Pause briefly to let its response finish transmitting
//             task_delay(15); 
//             return true; 
//         }
//         esp_rom_delay_us(1); // Standardized microsecond poll
//     }
//     return false;
// }
// void DALI::withdrawNode(uint32_t addr) {
//     sendCommand(SEARCHADDRH, (addr >> 16) & 0xFF);
//     sendCommand(SEARCHADDRM, (addr >> 8)  & 0xFF);
//     sendCommand(SEARCHADDRL,  addr        & 0xFF);
//     sendCommand(WITHDRAW, 0);
//     task_delay(20);
// }

// int DALI::getNextFreeShortAddress()
// {
//     for(int addr=0; addr<64; addr++)
//     {
//         if(!isShortAddressUsed(addr))
//             return addr;
//     }

//     return -1;
// }

// #define QUERY_CONTROL_GEAR_PRESENT   0x91

// bool DALI::isShortAddressUsed(uint8_t shortAddr)
// {
//     uint8_t command = (shortAddr << 1) | 0x01;

//     sendCommand(command, QUERY_CONTROL_GEAR_PRESENT);

//     for(uint32_t i=0;i<50000;i++)
//     {
// #ifdef IS_INVERTED
//         if(!gpio_get_level(rxPin))
// #else
//         if(gpio_get_level(rxPin))
// #endif
//         {
//             task_delay(20);
//             return true;
//         }

//         task_delayMicroseconds(1);
//     }

//     return false;
// }

// int DALI::commissionNewNodes()
// {
//     uint32_t searchLower, searchUpper, searchCurrent;
//     int assigned = 0;

//     //ESP_LOGI(TAG, "Starting DALI Addressing Sequence (Unaddressed Gear Only)...");

//     // Protect the entire loop execution from interleaved commands
//     if (xSemaphoreTake(dali_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
//         ESP_LOGE(TAG, "commissionNewNodes: Could not acquire dali_mutex");
//         return 0;
//     }

//     // -------------------------------------------------------------
//     // 1. Initialise Mode: 0xFF targets ONLY unaddressed devices
//     // -------------------------------------------------------------
//     sendCommand(TERMINATE, 0);
//     task_delay(100);

//     sendCommand(INITIALISE, 0xFF);  // 0xFF = Gear without a short address
//     task_delay(20);
//     sendCommand(INITIALISE, 0xFF);  // Standard requires sending twice
//     task_delay(300);

//     // -------------------------------------------------------------
//     // 2. Generate Random 24-bit Addresses
//     // -------------------------------------------------------------
//     sendCommand(RANDOMISE, 0);
//     task_delay(20);
//     sendCommand(RANDOMISE, 0);      // Standard requires sending twice
//     task_delay(300);

//     // -------------------------------------------------------------
//     // 3. Search Loop
//     // -------------------------------------------------------------
//     while (1)
//     {
//         // Reset full 24-bit search space boundaries
//         searchLower = 0x000000;
//         searchUpper = 0xFFFFFF;

//         // Verify if at least one unaddressed device responds to the ceiling address
//         if (!sendSearchAddr(searchUpper)) {
//             ESP_LOGI(TAG, "No unaddressed DALI devices found responding to search.");
//             break;
//         }

//         // ---------------------------------------------------------
//         // 4. Binary Search Sequence (Guaranteed to finish in 24 iterations)
//         // ---------------------------------------------------------
//         while (searchLower < searchUpper)
//         {
//             // Calculate midpoint safely avoiding overflow
//             searchCurrent = searchLower + ((searchUpper - searchLower) / 2);

//             if (sendSearchAddr(searchCurrent)) {
//                 // Device(s) exist at or below searchCurrent; tighten upper bound
//                 searchUpper = searchCurrent;
//             } else {
//                 // No devices below searchCurrent; shift search space up
//                 searchLower = searchCurrent + 1;
//             }
//         }

//         // searchLower now holds the exact, lowest 24-bit random address found
//         uint32_t foundRandomAddr = searchLower;

//         // ---------------------------------------------------------
//         // 5. Short Address Assignment
//         // ---------------------------------------------------------
//         int shortAddr = getNextFreeShortAddress();
//         if (shortAddr < 0) {
//             ESP_LOGE(TAG, "No free short addresses left (Bus full at 64 devices).");
//             break;
//         }

//         // Target the specific device by setting the search address register to its match
//         sendSearchAddr(foundRandomAddr);

//         // Program and verify short address
//         if (!sendProgramShortAddr(shortAddr)) {
//             ESP_LOGE(TAG, "Failed to program short address %d", shortAddr);
//             break; 
//         }

//         ESP_LOGI(TAG, "Assigned Short Address %d to RandomAddr 0x%06lX", shortAddr, foundRandomAddr);

//         // ---------------------------------------------------------
//         // 6. Squelch Found Gear
//         // ---------------------------------------------------------
//         // withdrawNode isolates the device matching the search address 
//         // out of the INITIALISE state so it no longer answers binary searches.
//         withdrawNode(foundRandomAddr);
//         assigned++;
//     }

//     // -------------------------------------------------------------
//     // 7. Cleanup & Release Bus Control
//     // -------------------------------------------------------------
//     sendCommand(TERMINATE, 0);
//     xSemaphoreGive(dali_mutex);

//     ESP_LOGI(TAG, "DALI Addressing Completed. Total assigned: %d", assigned);
//     return assigned;
// }

// int DALI::initNodes(const uint8_t* addresses, uint8_t numAddresses) {
//     uint32_t searchLower, searchDifference, searchTop;
//     int ret = 0;
//     #if 0
//         sendCommand(TERMINATE, 0);
//         task_delay(100);
//         sendCommand(INITIALISE, 0);
//         task_delay(10);
//         sendCommand(INITIALISE, 0);
//         task_delay(200);
//     #else
//         // Reinicia los modulos
//         sendCommand(COMMAND_BROADCAST, RESET);
//         task_delay(10);
//         sendCommand(COMMAND_BROADCAST, RESET);
//         task_delay(300);

//         // Termina con todos los nodos que puedan estar en configuración
//         sendCommand(TERMINATE, 0);
//         task_delay(100);

//         // Reinicia los modulos
//         sendCommand(INITIALISE, 0);
//         task_delay(10);
//         sendCommand(INITIALISE, 0);
//         task_delay(200);

//         // Pone a una dirección aleatoria los nodos
//         sendCommand(RANDOMISE, 0);
//         task_delay(10);
//         sendCommand(RANDOMISE, 0);
//         task_delay(200);
//     #endif

//     while (1) {
//         searchLower      = 0;
//         searchDifference = 0xFFFFFF;
//         searchTop        = 0xFFFFFF;

//         while (searchDifference > 1) {
//             while (sendSearchAddr(searchDifference + searchLower)) {
//                 searchTop = searchDifference + searchLower;
//                 if (!searchDifference) break;
//                 searchDifference >>= 1;
//             }
//             if (searchDifference == 0xFFFFFF) return ret;
//             searchLower     += searchDifference;
//             searchDifference = searchTop - searchLower;
//         }

//         if (sendSearchAddr(searchDifference + searchLower)) {
//             if (!sendProgramShortAddr(*addresses++)) return 0;
//             withdrawNode(searchDifference + searchLower);
//             if ((sendSearchAddr(searchDifference + searchLower) |
//                  (searchDifference + searchLower)) == 0xFFFFFF) return 0;
//             ret++;
//             ESP_LOGI(TAG, "Assigned DALI address: %d", ret - 1);
//             if (ret == numAddresses) return ret;
//         }
//     }
//     return 0;
// }

// void DALI::setValue(uint8_t nodeNumber, uint8_t value) {
//     if      (value == 0)   turnOff(nodeNumber);
//     else if (value == 255) setMax(nodeNumber);
//     else    sendCommand(SHORT_POWER | ((nodeNumber << 1) & 0x7e), value);
// }

// void DALI::turnOff(uint8_t nodeNumber) {
//     sendCommand(SHORT_POWER | ((nodeNumber << 1) & 0x7e) | 0x01, 0x00);
// }

// void DALI::setMax(uint8_t nodeNumber) {
//     sendCommand(SHORT_POWER | ((nodeNumber << 1) & 0x7e) | 0x01, 0x05);
// }

// bool DALI::sendProgramShortAddr(uint8_t nodeNumber) {
//     nodeNumber &= 0x3F;
//     sendCommand(PROGRAM_SHORT_ADDRESS, 1 | (nodeNumber << 1));
//     sendCommand(VERIFY_SHORT_ADDRESS,  1 | (nodeNumber << 1));
//     for (uint32_t n = 0; n < 50000; n++) {
// #ifdef IS_INVERTED
//         if (!gpio_get_level(rxPin))
// #else
//         if ( gpio_get_level(rxPin))
// #endif
//         { task_delay(20); return true; }
//         task_delayMicroseconds(1);
//     }
//     return false;
// }

// bool DALI::sendCommandPublic(uint8_t command, uint8_t data) {
//     return sendCommand(command, data);
// }
// bool DALI::sendCommandNormalPublic(uint8_t command, uint8_t data) {
//     sendCommandNormal(command, data);
//     return true;
// }


// bool DALI::sendCommandRetryPublic(uint8_t command, uint8_t data, uint8_t* retryCount) {
//     return sendCommandRetry(command, data, retryCount);
// }

// void DALI::sendCommandPublic32(uint8_t command1, uint8_t data1,
//                                 uint8_t command2, uint8_t data2) {
//     sendCommand32(command1, data1, command2, data2);
// }

// void DALI::disableRxInterrupt() {
//     if (rxPin != GPIO_NUM_NC) gpio_intr_disable(rxPin);
// }

// void DALI::enableRxInterrupt() {
//     if (rxPin != GPIO_NUM_NC) gpio_intr_enable(rxPin);
// }

// void DALI::query(uint8_t address, uint8_t queryCommand) {
//     uint8_t mask        = address & 0x80;
//     uint8_t new_address = mask | ((address << 1) + 1);
//     printf("new_addr: 0x%x query:0x%x\n", new_address, queryCommand);
//     sendCommandPublic(new_address, queryCommand);
// }

// bool DALI::waitBusIdleStable(uint32_t stable_us, uint32_t timeout_us) {
//     int64_t deadline = esp_timer_get_time() + timeout_us;
//     while (esp_timer_get_time() < deadline) {
//         if (isBusIdle()) {
//             esp_rom_delay_us(stable_us);
//             if (isBusIdle()) return true;
//         }
//         esp_rom_delay_us(100);
//     }
//     return false;
// }

// #define QUERY_STATUS_2 0xBB

// int DALI::scanAssignedShortAddresses(uint8_t* foundAddresses, uint8_t maxAddresses) {
//     int found = 0;
//     for (uint8_t addr = 0; addr < 64; ++addr) {
//         vTaskDelay(1 / portTICK_PERIOD_MS);
//         sendCommand(addr, QUERY_STATUS_2);
//         for (uint32_t n = 0; n < 100000; n++) {
// #ifdef IS_INVERTED
//             if (!gpio_get_level(rxPin))
// #else
//             if ( gpio_get_level(rxPin))
// #endif
//             { foundAddresses[found++] = addr; break; }
//             task_delayMicroseconds(1);
//         }
//     }
//     return found;
// }



// #define QUERY_STATUS                0x90
// #define QUERY_CONTROL_GEAR_PRESENT  0x91


// bool DALI::waitForResponse()
// {
//     for (uint32_t i = 0; i < 50000; i++)
//     {
// #ifdef IS_INVERTED
//         if (!gpio_get_level(rxPin))
// #else
//         if (gpio_get_level(rxPin))
// #endif
//         {
//             task_delay(20);
//             return true;
//         }

//         task_delayMicroseconds(1);
//     }

//     return false;
// }




// // Assuming rxPin is stored as a gpio_num_t type in your DALI class instance
// // e.g., gpio_num_t rxPin = GPIO_NUM_4;

// // bool DALI::waitForResponseValue(uint8_t *outputByte) {
// //     uint8_t decoded = 0;
    
// //     // 1. Wait for the START BIT transition
// //     // esp_timer_get_time() returns microseconds since boot as an int64_t
// //     int64_t wait_start = esp_timer_get_time();
// //     bool start_bit_detected = false;
    
// //     while ((esp_timer_get_time() - wait_start) < 11500) { // Standard DALI 11.5ms window
// // #ifdef IS_INVERTED
// //         if (gpio_get_level(rxPin) == 0) // Bus dropped from idle High to active Low
// // #else
// //         if (gpio_get_level(rxPin) == 1)
// // #endif
// //         {
// //             start_bit_detected = true;
// //             break;
// //         }
// //     }
    
// //     if (!start_bit_detected) {
// //         return false; // Timeout: Device did not reply
// //     }

// //     // DALI timing constants: Half-bit window (TE) is ~417 microseconds
// //     const uint32_t TE_US = 417; 

// //     // 2. Step past the start bit (Manchester logical '1' transition)
// //     esp_rom_delay_us(TE_US - 50); 
    
// //     // 3. Sample 8 Data Bits
// //     for (int8_t bit = 7; bit >= 0; bit--) {
// //         // Sample first half of bit cell
// //         esp_rom_delay_us(TE_US / 2);
// //         int first_half = gpio_get_level(rxPin);
        
// //         // Sample second half of bit cell
// //         esp_rom_delay_us(TE_US);
// //         int second_half = gpio_get_level(rxPin);
        
// //         // Manchester decoding: Low-to-High = '1', High-to-Low = '0'
// // #ifdef IS_INVERTED
// //         first_half = !first_half;
// //         second_half = !second_half;
// // #endif

// //         if (first_half == 0 && second_half == 1) {
// //             decoded |= (1 << bit);
// //         } else if (first_half == 1 && second_half == 0) {
// //             decoded &= ~(1 << bit);
// //         } else {
// //             return false; // Manchester violation (no transition in the middle)
// //         }
        
// //         esp_rom_delay_us(TE_US / 2); // Transition to the next bit cell boundary
// //     }
    
// //     // Write back the decoded value and return success
// //     *outputByte = decoded;
// //     return true;
// // }

// // #include "esp_timer.h"
// // #include "hal/gpio_ll.h"
// // #include "soc/gpio_reg.h"
// // #include "freertos/FreeRTOS.h"

// // int IRAM_ATTR DALI::waitForResponseValue(uint8_t *outputByte)
// // {
// //     constexpr int TE_US = 417;
// //     constexpr int START_TIMEOUT_US = 12000;

// //     uint8_t decoded = 0;

// //     // Fast GPIO read
// //     auto readPin = [this]() -> int
// //     {
// // #if CONFIG_IDF_TARGET_ESP32C6
// //        // uint32_t in = gpio_get_level(rxPin);
// //         int in = gpio_ll_get_level(&GPIO, rxPin);
// // #else
// //         int in = gpio_ll_get_level(&GPIO, rxPin);
// // #endif
// //         return (in >> rxPin) & 1;
// //     };

// // #ifdef IS_INVERTED
// //     int prev = !readPin();
// // #else
// //     int prev = readPin();
// // #endif

// //     int64_t timeout = esp_timer_get_time();

// //     //------------------------------------------------------
// //     // Wait for start-bit transition
// //     //------------------------------------------------------
// //     while ((esp_timer_get_time() - timeout) < START_TIMEOUT_US)
// //     {
// // #ifdef IS_INVERTED
// //         int now = !readPin();
// // #else
// //         int now = readPin();
// // #endif

// //         // Rising edge = beginning of start bit
// //         if (prev == 0 && now == 1)
// //         {
// //             break;
// //         }

// //         prev = now;
// //     }

// //     if ((esp_timer_get_time() - timeout) >= START_TIMEOUT_US)
// //         return -1;

// //     //------------------------------------------------------
// //     // Timestamp exactly after edge detection
// //     //------------------------------------------------------
// //     int64_t t0 = esp_timer_get_time();

// //     portDISABLE_INTERRUPTS();

// //     bool framingError = false;

// //     //------------------------------------------------------
// //     // Sample 8 Manchester bits
// //     //------------------------------------------------------
// //     for (int bit = 7; bit >= 0; bit--)
// //     {
// //         // Center of first half
// //         while (esp_timer_get_time() < (t0 + 1042 + (7 - bit) * 2 * TE_US));

// // #ifdef IS_INVERTED
// //         int first = !readPin();
// // #else
// //         int first = readPin();
// // #endif

// //         // Center of second half
// //         while (esp_timer_get_time() < (t0 + 1042 + TE_US + (7 - bit) * 2 * TE_US));

// // #ifdef IS_INVERTED
// //         int second = !readPin();
// // #else
// //         int second = readPin();
// // #endif

// //         if (first == 0 && second == 1)
// //         {
// //             decoded |= (1 << bit);
// //         }
// //         else if (first == 1 && second == 0)
// //         {
// //             // bit = 0
// //             //decoded &= ~(1 << bit);
// //         }
// //         else
// //         {
// //             framingError = true;
// //             break;
// //         }
// //     }

// //     portENABLE_INTERRUPTS();

// //     if (framingError)
// //         return -2;

// //     *outputByte = decoded;

// //     return 0;
// // }


// #define DALI_BACKWARD_TE_US      416
// #define DALI_RESPONSE_TIMEOUT_US 12000

// static inline int IRAM_ATTR fastRead(gpio_num_t pin)
// {
//     int level = gpio_get_level(pin);

// #ifdef IS_INVERTED
//     level = !level;
// #endif

//     return level;
// }

// int IRAM_ATTR DALI::waitForResponseValue(uint8_t *value)
// {
//     uint8_t data = 0;

// #ifdef IS_INVERTED
//     int idle = 0;
// #else
//     int idle = 1;
// #endif

//     //---------------------------------------------------------
//     // Wait for start bit
//     //---------------------------------------------------------

//     int64_t start = esp_timer_get_time();

//     while ((esp_timer_get_time() - start) < DALI_RESPONSE_TIMEOUT_US)
//     {

//         if (fastRead(rxPin))
//             break;

//     }

//     if ((esp_timer_get_time() - start) >= DALI_RESPONSE_TIMEOUT_US)
//         return -1;

//     //---------------------------------------------------------
//     // First edge detected
//     //---------------------------------------------------------

//     int64_t t0 = esp_timer_get_time();

//    // portDISABLE_INTERRUPTS();

//     for (int bit = 7; bit >= 0; bit--)
//     {
//         //-----------------------------------------------------
//         // Sample first half
//         //-----------------------------------------------------

//         while (esp_timer_get_time() < (t0 + 3 * DALI_BACKWARD_TE_US / 2 +
//                                        (7 - bit) * 2 * DALI_BACKWARD_TE_US));


//         int first = fastRead(rxPin);

//         //-----------------------------------------------------
//         // Sample second half
//         //-----------------------------------------------------

//         while (esp_timer_get_time() < (t0 + 5 * DALI_BACKWARD_TE_US / 2 +
//                                        (7 - bit) * 2 * DALI_BACKWARD_TE_US));


//         int second = fastRead(rxPin);


//         if ((first == 0) && (second == 1))
//         {
//             data |= (1 << bit);
//         }
//         else if ((first == 1) && (second == 0))
//         {
//             // bit = 0
//         }
//         else
//         {
//            // portENABLE_INTERRUPTS();
//             return -2;
//         }
//     }

//     //portENABLE_INTERRUPTS();
    
//     *value = data;

//     return 0;
// }

// int32_t DALI::queryPowerOnLevel(uint8_t shortAddr) {
//     return queryGear(shortAddr, QUERY_POWER_ON_LEVEL);
// }
// int32_t DALI::queryFadeTimeFadeRate(uint8_t shortAddr) {
//     return queryGear(shortAddr, QUERY_FADE_TIME_FADE_RATE);
// }
// int32_t DALI::queryDeviceType(uint8_t shortAddr) {
//     return queryGear(shortAddr, QUERY_DEVICE_TYPE);
// }
// int32_t DALI::queryNextDeviceType(uint8_t shortAddr) {
//     return queryGear(shortAddr, QUERY_NEXT_DEVICE_TYPE);
// }
// int32_t DALI::queryGearFeatures(uint8_t shortAddr) {
//     return queryGear(shortAddr, QUERY_GEAR_FEATURES);
// }

// int32_t DALI::queryDeviceInGroupA(uint8_t shortAddr) {
//     return queryGear(shortAddr, QUERY_GROUPS_0_TO_7);
// }
// int32_t DALI::queryDeviceInGroupB(uint8_t shortAddr) {
//     return queryGear(shortAddr, QUERY_GROUPS_8_TO_15);
// }
// int DALI::readExistingDrivers(uint8_t *addressList, int maxDevices)
// {
//     int count = 0;

//     // printf("Scanning DALI short addresses...\n");

//     for (uint8_t shortAddr = 0; shortAddr < 64; shortAddr++)
//     {
//         uint8_t daliAddr = (shortAddr << 1) | 0x01;

//         // Send query
//         sendCommand(daliAddr, QUERY_CONTROL_GEAR_PRESENT);

//         // Wait for backward frame
//         if (waitForResponse())
//         {
//             //printf("Device found at Short Address %d\n", shortAddr);

//             if (count < maxDevices)
//                 addressList[count] = shortAddr;

//             count++;
//         }

//         task_delay(10);
//     }

//     printf("Total Devices Found = %d\n", count);

//     return count;
// }


// // Change your function return type to an explicit int32_t to separate errors from 255
// int32_t DALI::queryGear(uint8_t shortAddr, uint8_t query_cmd) {
//     //uint8_t daliAddr = (shortAddr << 1) | 0x01; 
//     //sendCommand(daliAddr, query_cmd);
//     query(shortAddr,  query_cmd);
//     //uint8_t responseValue = 0;//waitForResponseValue();
//     // Pass the clean uint8_t pointer directly to your ESP-IDF decoder
//     // if (waitForResponseValue(&responseValue) == 0) {
//     //     // Explicitly return the clean 0-255 unsigned byte cast into the wider integer
//     //     return (int32_t)responseValue; 
//     // }
//     return -1; // Return -1 strictly for "Hardware Timeout / No Device Present"
// }


// bool DALI::resetDriver(uint8_t shortAddr)
// {
//     if (shortAddr > 63)
//         return false;

//     // Forward address frame for short address
//     uint8_t daliAddr = (shortAddr << 1) | 0x01;

//     // // RESET must be sent twice within 100 ms
//     // sendCommand(daliAddr, RESET);
//     // task_delay(20);

//     // sendCommand(daliAddr, RESET);
//     // task_delay(150);
//         // Reinicia los modulos
//     sendCommand(0xff, RESET);
//     task_delay(10);
//     sendCommand(0xff, RESET);
//     task_delay(300);

//     // Termina con todos los nodos que puedan estar en configuración
//     sendCommand(TERMINATE, 0);
//     task_delay(100);

//     // Reinicia los modulos
//     sendCommand(INITIALISE, 0);
//     task_delay(10);
//     sendCommand(INITIALISE, 0);
//     task_delay(200);

//     // Pone a una dirección aleatoria los nodos
//     sendCommand(RANDOMISE, 0);
//     task_delay(10);
//     sendCommand(RANDOMISE, 0);
//     task_delay(200);
//     return true;
// }

// void DALI::sendData(uint8_t value)
// {

// }

// #define DALI_CMD_SET_DTR           0xA3  // Special command 257 (1010 0011)
// #define DALI_CMD_STORE_DTR_AS_SA   128   // Configuration command 128 (1000 0000)
// #define DALI_MASK_ADDRESS          0xFF  // 0xFF clears/deletes the short address

// /**
//  * @brief Removes/clears the short address from a specific DALI device.
//  * @param shortAddr The short address to clear (0 to 63).
//  * @return true if commands were transmitted successfully, false otherwise.
//  */
// bool DALI::clearShortAddress(uint8_t shortAddr) {
//     if (shortAddr > 63) {
//         ESP_LOGE(TAG, "clearShortAddress: Invalid short address %d", shortAddr);
//         return false;
//     }

//     // 1. Prepare the targeted address byte for Command 128.
//     // Standard format for a short address command: 0AAAAAA1
//     uint8_t addressedByte = (shortAddr << 1) | 0x01;

//     // Mutex guard to protect back-to-back transmission sequence from other threads
//     if (xSemaphoreTake(dali_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
//         ESP_LOGW(TAG, "clearShortAddress: Could not acquire dali_mutex");
//         return false;
//     }

//     // 2. Set DTR0 to 0xFF (MASK value)
//     // Special commands use their own raw formatting, sendCommand handles bus idling
//     if (!sendCommand(DALI_CMD_SET_DTR, DALI_MASK_ADDRESS)) {
//         ESP_LOGW(TAG, "clearShortAddress: Failed to set DTR to 0xFF");
//         xSemaphoreGive(dali_mutex);
//         return false;
//     }

//     // Small delay between different logical commands to prevent bus collisions
//     task_delayMicroseconds(u16_frame_delay_us);

//     // 3. Send STORE DTR AS SHORT ADDRESS (Twice required by DALI standard)
//     // First transmission
//     if (!sendCommand(addressedByte, DALI_CMD_STORE_DTR_AS_SA)) {
//         ESP_LOGW(TAG, "clearShortAddress: Failed to send first STORE_DTR command");
//         xSemaphoreGive(dali_mutex);
//         return false;
//     }

//     task_delayMicroseconds(u16_frame_delay_us);

//     // Second transmission (must happen within 100ms)
//     if (!sendCommand(addressedByte, DALI_CMD_STORE_DTR_AS_SA)) {
//         ESP_LOGW(TAG, "clearShortAddress: Failed to send second STORE_DTR command");
//         xSemaphoreGive(dali_mutex);
//         return false;
//     }

//     xSemaphoreGive(dali_mutex);
//     ESP_LOGI(TAG, "Successfully cleared short address %d", shortAddr);
//     return true;
// }

// ============================================================
//  DALI.cpp — corrected implementation
// ============================================================

#include "DALI.h"
#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
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

    portENTER_CRITICAL(&bus_mux_);
    bus_busy_             = false;
    tx_in_progress_       = false;
    last_bus_activity_us_ = esp_timer_get_time();
    portEXIT_CRITICAL(&bus_mux_);

    dali_mutex = xSemaphoreCreateMutex();
}

void DALI::setTxInProgress(bool state) {
    portENTER_CRITICAL(&bus_mux_);
    tx_in_progress_ = state;
    portEXIT_CRITICAL(&bus_mux_);
}

void IRAM_ATTR DALI::markBusActivityFromISR() {
    portENTER_CRITICAL_ISR(&bus_mux_);
    if (!tx_in_progress_) {
        bus_busy_             = true;
        last_bus_activity_us_ = esp_timer_get_time();
        bus_activity_counter++;
    }
    portEXIT_CRITICAL_ISR(&bus_mux_);
}

uint32_t DALI::getBusActivityCounter() {
    uint32_t cnt;
    portENTER_CRITICAL(&bus_mux_);
    cnt = bus_activity_counter;
    portEXIT_CRITICAL(&bus_mux_);
    return cnt;
}

bool DALI::isBusIdle() {
    int64_t now = esp_timer_get_time();
    int64_t last_us;
    bool    busy;

    portENTER_CRITICAL(&bus_mux_);
    busy    = bus_busy_;
    last_us = last_bus_activity_us_;
    //portEXIT_CRITICAL(&bus_mux_);

    if (busy && ((now - last_us) >= DALI_BUS_IDLE_MIN_US)) {
        //portENTER_CRITICAL(&bus_mux_);
        if (bus_busy_ &&
            ((esp_timer_get_time() - last_bus_activity_us_) >= DALI_BUS_IDLE_MIN_US)) {
            bus_busy_ = false;
        }
        //portEXIT_CRITICAL(&bus_mux_);
    }
    portEXIT_CRITICAL(&bus_mux_);
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

void DALI::releaseBus() {
    portENTER_CRITICAL(&bus_mux_);
    bus_busy_             = false;
    last_bus_activity_us_ = esp_timer_get_time();
    portEXIT_CRITICAL(&bus_mux_);
    task_delayMicroseconds(u16_frame_delay_us);
}

void DALI::daliSetFrameDelay(uint16_t delay_us) {
    u16_frame_delay_us = delay_us;
}

bool DALI::sendCommandRaw(uint8_t command, uint8_t data) {
    uint16_t info = (uint16_t)((command << 8) | data);

    if (rxPin != GPIO_NUM_NC) gpio_intr_disable(rxPin);
    setTxInProgress(true);

    if (!sendOne()) {
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

    setTxInProgress(false);
    releaseBus();
    if (rxPin != GPIO_NUM_NC) gpio_intr_enable(rxPin);

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
        esp_rom_delay_us(100);
    }
    if (retryCount) {
        *retryCount = DALI_RETRY_COUNTS;
    }
    return false;
}

void DALI::sendCommandNormal(uint8_t command, uint8_t data) {
    uint16_t info = (uint16_t)((command << 8) | data);
    sendOneNormal();

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

bool DALI::sendCommand32Raw(uint8_t command1, uint8_t data1,
                            uint8_t command2, uint8_t data2) {
    uint16_t cd1  = (uint16_t)((command1 << 8) | data1);
    uint16_t cd2  = (uint16_t)((command2 << 8) | data2);
    uint32_t info = (uint32_t)((cd1 << 16) | cd2);

    if (rxPin != GPIO_NUM_NC) gpio_intr_disable(rxPin);
    setTxInProgress(true);

    // Frame 1
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

    gpio_set_level(txPin, DALI_LOW);
    task_delayMicroseconds(3700);

    // Frame 2
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

    setTxInProgress(false);
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
    if (!sendCommand(SEARCHADDRH, (addr >> 16) & 0xFF)) return false;
    task_delayMicroseconds(u16_frame_delay_us);
    
    if (!sendCommand(SEARCHADDRM, (addr >> 8)  & 0xFF)) return false;
    task_delayMicroseconds(u16_frame_delay_us);
    
    if (!sendCommand(SEARCHADDRL,  addr        & 0xFF)) return false;
    task_delayMicroseconds(u16_frame_delay_us);
    
    if (!sendCommand(COMPARE, 0)) return false;

    for (uint32_t n = 0; n < 25000; n++) {
#ifdef IS_INVERTED
        if (!gpio_get_level(rxPin))
#else
        if (gpio_get_level(rxPin))
#endif
        { 
            task_delay(15); 
            return true; 
        }
        esp_rom_delay_us(1);
    }
    return false;
}

void DALI::withdrawNode(uint32_t addr) {
    sendCommand(SEARCHADDRH, (addr >> 16) & 0xFF);
    sendCommand(SEARCHADDRM, (addr >> 8)  & 0xFF);
    sendCommand(SEARCHADDRL,  addr        & 0xFF);
    sendCommand(WITHDRAW, 0);
    task_delay(20);
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

int DALI::initNodes(const uint8_t* addresses, uint8_t numAddresses) {
    uint32_t searchLower, searchDifference, searchTop;
    int ret = 0;

    sendCommand(COMMAND_BROADCAST, RESET);
    task_delay(10);
    sendCommand(COMMAND_BROADCAST, RESET);
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
    printf("new_addr: 0x%x query:0x%x\n", new_address, queryCommand);
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