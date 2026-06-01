#pragma once

#include <cstdint>
#include <functional>
#include "esp_err.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "dali_receiver.h"
class DALI;   // forward declaration
namespace dali_rx {

struct RxEdgeEvent {
    uint32_t t_us;
    bool bus_low;
};
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
    // New method to set query mode (true = expecting 8-bit response)
    void set_query_mode(bool query_mode) {
        portENTER_CRITICAL(&spinlock_);
        query_mode_ = query_mode;
        portEXIT_CRITICAL(&spinlock_);
    }
    void dali_rx_intr_enabled(bool enabled);
    void attachBusMonitor(DALI* dali) { dali_ = dali; }
    Receiver(DALI* dali) : dali_(dali) {}
    
private:

    DALI* dali_ = nullptr;
    // Receiver state machine
    enum class RxState {
        IDLE,
        START,
        BIT
    };

    static void IRAM_ATTR gpio_isr_handler(void* arg);
    static void IRAM_ATTR timer_callback(void* arg);
    void IRAM_ATTR handle_pin_change_isr();
    void rx_task();

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
    // portMUX_TYPE        spinlock_;           // critical section guard

    bool query_mode_;  // true = expecting 8-bit response frame

    QueueHandle_t edge_queue_ = nullptr;
    portMUX_TYPE spinlock_ = portMUX_INITIALIZER_UNLOCKED;
    //enum class RxState : uint8_t { IDLE, START, BIT };
    //RxState state_ = IDLE;
    void process_edge(uint32_t now, bool bus_low);
};

} // namespace dali