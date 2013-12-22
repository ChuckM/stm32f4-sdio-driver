/*
 * Copyright (C) 2013 Chuck McManis <cmcmanis@mcmanis.com>
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/sdio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/f4/flash.h>
#include <stdint.h>
#include "bb.h"

/*
 * Do chip initialization to set up the chip and peripherals
 *
 * Not sure I like this design pattern, basically we call "bb_setup()" 
 * which is a global, and then this code sets up all the things that
 * the bb "library" is going to be using. That said, I don't like the
 * fairly #ifdef laden style of Atmel or ST Micro either so I'm living
 * with this for now.
 */
void
bb_setup(int32_t baud) {
    clock_init(1000);
    console = uart_init(PC6, PC7, baud);
    sdio_init();
}


/*
 * stime(uint32_t)
 *
 * Convert a number representing milliseconds into a 'time' string
 * of HHH:MM:SS.mmm where HHH is hours, MM is minutes, SS is seconds
 * and .mmm is fractions of a second.
 */
char *
stime(uint32_t t) {
    static char time_string[14];
    uint16_t msecs = t % 1000;
    uint8_t secs = (t / 1000) % 60;
    uint8_t mins = (t / 60000) % 60;
    uint16_t hrs = (t /3600000);

    // HH:MM:SS.mmm\0
    // 0123456789abc
    time_string[0] = (hrs % 100) + '0';
    time_string[1] = (hrs / 10) % 10 + '0';
    time_string[2] = hrs % 10 + '0';
    time_string[3] = ':';
    time_string[4] = (mins / 10)  % 10 + '0';
    time_string[5] = mins % 10 + '0';
    time_string[6] = ':';
    time_string[7] = (secs / 10)  % 10 + '0';
    time_string[8] = secs % 10 + '0';
    time_string[9] = '.';
    time_string[10] = (msecs / 100) % 10 + '0';
    time_string[11] = (msecs / 10) % 10 + '0';
    time_string[12] = msecs % 10 + '0';
    time_string[13] = 0;
    return &time_string[0];
}

/*
 * Edit a number in place, needs work.
 * So ideally it should return a 'what happened' not a 
 * number, so we'll change it to that.
 */
enum EDIT_EVENT 
edit_number(int chan, int row, int col, uint32_t *orig, int base, int size) {
#if 0
    XXX Fix this 
    int cursor = 0;
    char buf[33];
    char c;
    int width = 8;

    ntoa(*orig, FMT_BASE_16, buf, base);
    while (1) {
        move_cursor(console, row, col);
        text_color(console, WHITE);
        uart_puts(console, buf);
        move_cursor(console, row, col+cursor);
        c = uart_getc(console, 1);
        if ((c >= '0') && (c <= '9')) {
            *(buf + cursor) = c;
            cursor = (cursor + 1) & 0x7;
        } else if (((c >= 'a') && (c <= 'f')) ||
                   ((c >= 'A') && (c <= 'F'))) {
            *(buf + cursor) = c & 0x4f;
            cursor = (cursor + 1) % width;
        } else if (c == 27) {
            move_cursor(console, row, col);
            text_color(console, DEFAULT);
            uart_puts(console, buf);
            return EDIT_CANCEL;
        } else if (c == 13 ) {
            *orig = aton(buf, base);
            move_cursor(console, row, col);
            text_color(console, DEFAULT);
            uart_puts(console, buf);
            return EDIT_ACCEPT;
        } else if ( c == 196 ) {
            cursor = (cursor > 0) ? cursor - 1 : width - 1;
        } else if ( c == 195 ) {
            cursor = (cursor + 1) % width;
        }
    }
#endif
    return EDIT_ACCEPT;
}
