/*
 * Copyright (c) 2013 - Chuck McManis, all rights reserved.
 *
 * These are some dead simple, polled, debug APIs for using the USART2 port
 * as a diagnostic console on PD5, nad PD6. I use the Black Magic ride along
 * UART to monitor this port. These are well tested and self contained so 
 * generally plugging them in is 'easy' and you can then use these (and a 
 * bunch of calls to debug_puts() :-)) to figure out what is going on in your
 * code if you can't run it under GDB.
 */
#include <stdint.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include "uart.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "task.h"

#if configCHECK_FOR_STACK_OVERFLOW
void vApplicationStackOverflowHook(void);

void
vApplicationStackOverflowHook(void)
{
    for (;;);
}
#endif

void vAssertCalled(const char * const pcFileName, unsigned long ulLine)
{
    volatile unsigned long ulSetToNonZeroInDebuggerToContinue = 0;

    /* Parameters are not used. */
    (void) ulLine;
    (void) pcFileName;

    taskENTER_CRITICAL();
    {
        while (ulSetToNonZeroInDebuggerToContinue == 0) {
            /* Use the debugger to set ulSetToNonZeroInDebuggerToContinue to a
               non zero value to step out of this function to the point that raised
               this assert(). */
            __asm volatile( "NOP");
            __asm volatile( "NOP");
        }
    }
    taskEXIT_CRITICAL();
}


int debug_console;
/*
 * Hard Defined, USART2, PD5 & PD6, 38400 baud. Compatible
 * with the Blackmagic Debug Probe ACM1 port.
 */
void
debug_init(void) {
    debug_console = uart_init(PD5, PD6, 115200);
}

/*
 * Get a character, optionally wait for it. (no function key support)
 */
char
debug_getc(int wait) {
    return uart_getc(debug_console, wait);
}

/*
 * Put a character
 */
void
debug_putc(char c) {
    uart_putc(debug_console, c);
}

/*
 * Write out a string.
 */
void
debug_puts(const char *s) {
    uart_puts(debug_console, s);
}

static const char *w_space = "[Press Space]\n";
/*
 * Write out the string '[Press Space]' and wait
 * for the space key to be pressed.
 */
void
debug_wait() {
    debug_puts(w_space);
    while (debug_getc(0) != ' ');
    return;
}
