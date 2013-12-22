/*
 * Simple include file for our utility routines
 */

#include "uart.h"
#include "clock.h"
#include "term.h"
#include "sdio.h"

#ifndef NULL
#define NULL (void *)(0x0000)
#endif

extern int console; // System console for the application

void bb_setup(int32_t baud);
void led_setup(void);

void sd_ident(void);
void sdio_explorer(void);

char *stime(uint32_t);

// dump functions
uint8_t *dump_line(int console, uint8_t *addr, uint8_t *base);
uint8_t *dump_page(int chan, uint8_t *addr, uint8_t *base);

enum EDIT_EVENT { EDIT_CANCEL, EDIT_ACCEPT, EDIT_PREV, EDIT_NEXT };
enum EDIT_EVENT edit_number(int, int , int , uint32_t * , int , int );

#ifndef CONSOLE_UART
#define CONSOLE_UART    USART6
#endif
