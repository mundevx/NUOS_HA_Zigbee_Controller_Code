#pragma once

#include <cstdint>
#include <functional>
#include "esp_err.h"
#include "esp_timer.h"
#include "driver/gpio.h"

namespace dali_rx {

/**
 * @brief DALI message receiver using pin change interrupts and a periodic timer.
 *
 * Listens on a GPIO pin, measures pulse widths, decodes Manchester‑encoded
 * half‑bits, and delivers complete messages via a callback.
 */
class Receiver {
public:
    /** Callback type: receives raw message bytes and length (1–3). */
    using callback_t = std::function<void(const uint8_t* data, size_t len)>;

    Receiver();
    ~Receiver();

    /**
     * @brief Start the receiver.
     * @param rx_pin      GPIO number for DALI input.
     * @param cb          Function called when a valid message is received.
     * @param inverted    True if bus low is logic 1 (active‑high), false if bus low is logic 0 (active‑low).
     * @return ESP_OK on success.
     */
    esp_err_t begin(gpio_num_t rx_pin, callback_t cb, bool inverted=false, bool* is_isr= nullptr);

    /** Stop receiver, free resources. */
    void end();

private:
    // Receiver state machine
    enum class RxState {
        IDLE,
        START,
        BIT
    };

    static void IRAM_ATTR gpio_isr_handler(void* arg);
    static void IRAM_ATTR timer_callback(void* arg);

    void handle_pin_change();
    void push_halfbit(uint8_t bit);

    // Configuration
    gpio_num_t      pin_;
    bool            inverted_;
    callback_t      callback_;

    // State
    RxState         state_;
    uint8_t         msg_[3];               // received bytes (max 24 bits)
    int16_t         halfbit_cnt_;           // number of half‑bits received (including start)
    uint32_t        last_change_us_;        // timestamp of last pin change
    bool            last_level_;             // last raw level (0/1)

    // Idle timer
    esp_timer_handle_t  idle_timer_;
    uint8_t             idle_te_cnt_;        // counts TE periods while bus is idle
    portMUX_TYPE        spinlock_;           // critical section guard
};

} // namespace dali