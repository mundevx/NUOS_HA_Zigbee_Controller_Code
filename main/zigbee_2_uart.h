#ifndef CJSON_H
#define CJSON_H

    #ifdef USE_IR_UART_WS4HW
    void uart_init(void);
    #endif

    #ifdef __cplusplus
    extern "C" {
    #endif

        void send_serial(const char *data);

    #ifdef __cplusplus
    }
    #endif

#endif  // CJSON_H
