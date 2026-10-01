#ifndef NUOS_ZIGBEE_2_UART_H
#define NUOS_ZIGBEE_2_UART_H

#ifdef __cplusplus
extern "C" {
#endif

void uart_init(void);
void send_serial(const char *data);

#ifdef __cplusplus
}
#endif

#endif  // NUOS_ZIGBEE_2_UART_H
