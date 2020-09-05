#ifndef UART_H
#define UART_H

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

#include <termios.h>

#ifdef UART_C
    #define EXTERN
#else
    #define EXTERN extern
#endif //UART_C

//debug
//#define DEBUG

#ifdef DEBUG
    EXTERN void debug_print(const char *s);
    EXTERN void debug_println2(const char *s, int i);
    #define PRINT(s) debug_print(s)
    #define PRINTLN2(s,i) debug_println2(s,i)
#else
    #define PRINT(s)
    #define PRINTLN2(s,i)
#endif //DEBUG

//prototypes
EXTERN size_t uart_data_available(void);
EXTERN int uart_get(void *buf, int size);
EXTERN void uart_send(void *buf, int size);
EXTERN int uart_init(const char *portname, int speed);
EXTERN void uart_close();

#ifdef __cplusplus
}
#endif //__cplusplus
#endif //UART_H

