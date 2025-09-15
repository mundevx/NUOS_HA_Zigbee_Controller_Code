

#ifndef cJSON_H
#define cJSON_H
#ifdef USE_IR_UART_WS4_HW
void uart_init();
    #ifdef __cplusplus
    extern "C" {
    #endif
    void send_serial(const char* data);

    #ifdef __cplusplus
    }
    #endif
#endif
#endif // cJSON_H
