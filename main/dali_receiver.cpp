#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dali_receiver.h"
#include "esp_log.h"
#include "DALI.h"

static const char* TAG = "DaliReceiver";
extern bool isr_service_installed;
// DALI timing (1200 baud, half‑bit period TE)
#define DALI_BAUD 1200
#define DALI_TE ((1000000+(DALI_BAUD))/(2*(DALI_BAUD)))  //417us
#define DALI_TE_MIN (80*DALI_TE)/100  
#define DALI_TE_MAX (120*DALI_TE)/100  
#define DALI_IS_TE(x) ((DALI_TE_MIN)<=(x) && (x)<=(DALI_TE_MAX))
#define DALI_IS_2TE(x) ((2*(DALI_TE_MIN))<=(x) && (x)<=(2*(DALI_TE_MAX)))

#define DALI_IS_BUS_LOW() (gpio_get_level(GPIO_NUM_19)==0)
//using namespace dali_rx;
namespace dali_rx {

Receiver::Receiver()
    : pin_(GPIO_NUM_NC)
    , inverted_(false)
    , state_(RxState::IDLE)
    , halfbit_cnt_(0)
    , last_change_us_(0)
    , last_level_(false)
    , idle_timer_(nullptr)
    , idle_te_cnt_(0)
    , query_mode_(false)  // Initialize query mode to false (16-bit by default)
{
    spinlock_ = portMUX_INITIALIZER_UNLOCKED;
    memset(msg_, 0, sizeof(msg_));
}

Receiver::~Receiver() {
    end();
}

esp_err_t Receiver::begin(gpio_num_t rx_pin, callback_t cb, bool inverted, bool* is_isr) {
    if (rx_pin == GPIO_NUM_NC) {
        return ESP_ERR_INVALID_ARG;
    }

    pin_ = rx_pin;
    inverted_ = inverted;
    callback_ = cb;

    // Configure GPIO
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = 1ULL << pin_;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Install GPIO ISR service if not already done
        if (!isr_service_installed) {
            ESP_ERROR_CHECK(gpio_install_isr_service(0));
            isr_service_installed = true;
        }
    

    // Add ISR handler
    ESP_ERROR_CHECK(gpio_isr_handler_add(pin_, gpio_isr_handler, this));

    // Create a periodic timer to count idle TE periods (fires every TE)
    const esp_timer_create_args_t timer_args = {
        .callback = timer_callback,
        .arg = this,
        //.dispatch_method = ESP_TIMER_ISR,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "dali_idle_timer",
        .skip_unhandled_events = true,
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &idle_timer_));
    ESP_ERROR_CHECK(esp_timer_start_periodic(idle_timer_, DALI_TE));

    ESP_LOGI(TAG, "Receiver started on GPIO %d", pin_);

    edge_queue_ = xQueueCreate(64, sizeof(RxEdgeEvent));

    xTaskCreate(
        [](void* arg) {
            static_cast<Receiver*>(arg)->rx_task();
        },
        "dali_rx_task",
        4096,
        this,
        TASK_PRIORITY_DALI_RX_INTR_TASK,
        nullptr
    );
    return ESP_OK;
}


void Receiver::dali_rx_intr_enabled(bool enabled)
{
    if (enabled) {
        gpio_intr_enable(pin_);
    } else {
        gpio_intr_disable(pin_);
    }
}

void Receiver::end() {
    if (idle_timer_) {
        esp_timer_stop(idle_timer_);
        esp_timer_delete(idle_timer_);
        idle_timer_ = nullptr;
    }
    if (pin_ != GPIO_NUM_NC) {
        gpio_isr_handler_remove(pin_);
        gpio_reset_pin(pin_);
        pin_ = GPIO_NUM_NC;
    }
}

// void IRAM_ATTR Receiver::gpio_isr_handler(void* arg) {
//     auto* self = static_cast<Receiver*>(arg);
//     self->handle_pin_change();
// }



void IRAM_ATTR Receiver::gpio_isr_handler(void* arg) {
    auto* self = static_cast<Receiver*>(arg);
    self->handle_pin_change_isr();
}
void IRAM_ATTR Receiver::timer_callback(void* arg) {
    auto* self = static_cast<Receiver*>(arg);
    uint8_t msg_copy[3] = {0};
    size_t copy_len = 0;

    portENTER_CRITICAL(&self->spinlock_);
    
    if (self->idle_te_cnt_ < 0xff) {
        self->idle_te_cnt_++;
    }

    if (self->state_ == RxState::BIT && self->idle_te_cnt_ > 4) {
        size_t bits = (self->halfbit_cnt_ + 1) >> 1;
        
        if ((bits & 0x07) == 0) {
            size_t len = bits >> 3;
            if (len >= 1 && len <= 3 && self->callback_) {
                memcpy(msg_copy, self->msg_, len);
                copy_len = len;
            }
        }
        self->state_ = RxState::IDLE;
    }
    
    portEXIT_CRITICAL(&self->spinlock_);

    // Invoke the callback OUTSIDE the critical section so xQueueSend is safe
    if (copy_len > 0 && self->callback_) {
        self->callback_(msg_copy, copy_len);
    }
}
// void IRAM_ATTR Receiver::timer_callback123(void* arg) {
//     auto* self = static_cast<Receiver*>(arg);
    
//     // Change portENTER_CRITICAL_ISR to portENTER_CRITICAL
//     portENTER_CRITICAL(&self->spinlock_);
    
//     if (self->idle_te_cnt_ < 0xff) {
//         self->idle_te_cnt_++;
//     }

//     if (self->state_ == RxState::BIT && self->idle_te_cnt_ > 4) {
//         size_t bits = (self->halfbit_cnt_ + 1) >> 1;
        
//         if ((bits & 0x07) == 0) {
//             size_t len = bits >> 3;
//             if (len >= 1 && len <= 3 && self->callback_) {
//                 self->callback_(self->msg_, len);
//             }
//         }
//         self->state_ = RxState::IDLE;
//     }
    
//     // Change portEXIT_CRITICAL_ISR to portEXIT_CRITICAL
//     portEXIT_CRITICAL(&self->spinlock_);
// }

//ok
// void IRAM_ATTR Receiver::timer_callback(void* arg) {
//     auto* self = static_cast<Receiver*>(arg);
//     portENTER_CRITICAL_ISR(&self->spinlock_);
//     if (self->idle_te_cnt_ < 0xff) {
//         self->idle_te_cnt_++;
//     }

//     // Check for end of message: at least 2 TE idle while in BIT state
//     if (self->state_ == RxState::BIT && self->idle_te_cnt_ > 4) {
//         // Convert half‑bits to bytes
//         size_t bits = (self->halfbit_cnt_ + 1) >> 1;   // number of full bits (incl. start)
        
//         // Auto-detect frame length - accept 8-bit, 16-bit, or 24-bit frames
//         if ((bits & 0x07) == 0) {  // Multiple of 8 bits
//             size_t len = bits >> 3;  // Convert bits to bytes
//             // DALI frames can be: 
//             // - 8 bits (backward frame/response)
//             // - 16 bits (forward frame/command)
//             // - 24 bits (special cases)
//             if (len >= 1 && len <= 3 && self->callback_) {
//                 // Callback must be ISR‑safe – it runs in timer ISR context
//                 self->callback_(self->msg_, len);
//             }
//         }
//         self->state_ = RxState::IDLE;
//     }
//     portEXIT_CRITICAL_ISR(&self->spinlock_);
// }
// void IRAM_ATTR Receiver::timer_callback(void* arg) {
//     auto* self = static_cast<Receiver*>(arg);
//     portENTER_CRITICAL_ISR(&self->spinlock_);
//     if (self->idle_te_cnt_ < 0xff) {
//         self->idle_te_cnt_++;
//     }

//     // Check for end of message: at least 2 TE idle while in BIT state
//     if (self->state_ == RxState::BIT && self->idle_te_cnt_ > 4) {
//         // Convert half‑bits to bytes
//         size_t bits = (self->halfbit_cnt_ + 1) >> 1;   // number of full bits (incl. start)
//         if ((bits & 0x07) == 0) {                      // multiple of 8 bits
//             size_t len = bits >> 3;                     // bytes (max 3)
//             if (self->callback_) {
//                 // Callback must be ISR‑safe – it runs in timer ISR context
//                 self->callback_(self->msg_, len);
//             }
//         }
//         self->state_ = RxState::IDLE;
//     }
//     portEXIT_CRITICAL_ISR(&self->spinlock_);
// }

// void IRAM_ATTR Receiver::handle_pin_change() {
//     uint32_t now = esp_timer_get_time();

//     if (dali_) {
//         dali_->markBusActivityFromISR();
//     }

//     portENTER_CRITICAL_ISR(&spinlock_);

//     int level = gpio_get_level(pin_);
//     bool bus_low = (inverted_ ? !level : level);

//     idle_te_cnt_ = 0;

//     if (bus_low == last_level_) {
//         portEXIT_CRITICAL_ISR(&spinlock_);
//         return;
//     }

//     uint32_t dt = now - last_change_us_;
//     last_change_us_ = now;
//     last_level_ = bus_low;

//     switch (state_) {
//         case RxState::IDLE:
//             if (bus_low) {
//                 state_ = RxState::START;
//             }
//             break;

//         case RxState::START:
//             if (bus_low || !DALI_IS_TE(dt)) {
//                 state_ = RxState::IDLE;
//             } else {
//                 halfbit_cnt_ = -1;
//                 for (auto& b : msg_) b = 0;
//                 state_ = RxState::BIT;
//             }
//             break;

//         case RxState::BIT:
//             if (DALI_IS_TE(dt)) {
//                 push_halfbit(bus_low);
//             } else if (DALI_IS_2TE(dt)) {
//                 push_halfbit(bus_low);
//                 push_halfbit(bus_low);
//             } else {
//                 state_ = RxState::IDLE;
//             }
//             break;

//         default:
//             break;
//     }

//     portEXIT_CRITICAL_ISR(&spinlock_);
// }
void IRAM_ATTR Receiver::handle_pin_change_isr() {
    BaseType_t hp_task_woken = pdFALSE;

    RxEdgeEvent ev;
    ev.t_us = (uint32_t)esp_timer_get_time();
    // int level = gpio_get_level(pin_);
    // ev.bus_low = inverted_ ? !level : level;

    if (dali_) {
        dali_->markBusActivityFromISR();
    }


    // RxEdgeEvent ev{ now, bus_low };

    // xQueueSendFromISR(edge_queue_, &ev, &hp_task_woken);
    // RxEdgeEvent ev;
    // ev.t_us = now;
    //ev.bus_low = false;   // fill later in task, or use a low-level read here

    xQueueSendFromISR(edge_queue_, &ev, &hp_task_woken);
    if (hp_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}
void IRAM_ATTR Receiver::push_halfbit(uint8_t bit) {
    // Invert because bus low = logical 1 (DALI active low)
    bit = (~bit) & 1;

    // Half‑bits are assembled into bytes: even index starts a new bit cell
    if ((halfbit_cnt_ & 1) == 0) {
        uint8_t idx = halfbit_cnt_ >> 4;      // byte index (0,1,2)
        if (idx < 3) {
            msg_[idx] = (msg_[idx] << 1) | bit;
        }
    }
    halfbit_cnt_++;
}



void Receiver::rx_task() {
    RxEdgeEvent ev;
    while (true) {
        if (xQueueReceive(edge_queue_, &ev, portMAX_DELAY) == pdTRUE) {
            int level = gpio_get_level(pin_);
            ev.bus_low = inverted_ ? !level : level;
            process_edge(ev.t_us, ev.bus_low);
        }
    }
}

void Receiver::process_edge(uint32_t now, bool bus_low) {
    portENTER_CRITICAL(&spinlock_);

    idle_te_cnt_ = 0;

    if (bus_low == last_level_) {
        portEXIT_CRITICAL(&spinlock_);
        return;
    }

    uint32_t dt = now - last_change_us_;
    last_change_us_ = now;
    last_level_ = bus_low;

    switch (state_) {
        case RxState::IDLE:
            if (bus_low) {
                state_ = RxState::START;
            }
            break;

        case RxState::START:
            if (bus_low || !DALI_IS_TE(dt)) {
                state_ = RxState::IDLE;
            } else {
                halfbit_cnt_ = -1;
                for (auto& b : msg_) b = 0;
                state_ = RxState::BIT;
            }
            break;

        case RxState::BIT:
            if (DALI_IS_TE(dt)) {
                push_halfbit(bus_low);
            } else if (DALI_IS_2TE(dt)) {
                push_halfbit(bus_low);
                push_halfbit(bus_low);
            } else {
                state_ = RxState::IDLE;
            }
            break;

        default:
            break;
    }

    portEXIT_CRITICAL(&spinlock_);
}

} // namespace dali



