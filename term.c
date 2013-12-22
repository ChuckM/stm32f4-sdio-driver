/*
 * Copyright (c) 2008 - Chuck McManis, All Rights Reserved
 *
 * These are some 'curses' like functions which have come in
 * handy.
 */

#include <stdint.h>
#include "uart.h"
#include "term.h"
extern int console;

#define CSI "\033["

/*
 * Send the escape sequence to position the cursor
 */
void
move_cursor(int chan, int row, int col) {
    if ((row <= 0) || (col <= 0) || (row > 88) || (col > 200)) {
        return;
    }
    uart_puts(chan, CSI);
    uart_putnum(chan, FMT_BASE_10, row);
    uart_putc(chan, ';');
    uart_putnum(chan, FMT_BASE_10, col);
    uart_putc(chan, 'H');
}

static const char *__screen_clear = CSI "2J";

/* Clear the screen */
void
clear_screen(int chan) {
    uart_puts(chan, __screen_clear);
}

static const char * __clear_eol = CSI "2j";
void
clear_eol(int chan) {
    uart_puts(chan, __clear_eol);
}

static const char *__screen_colors[6] = {
    CSI "31;40;1m",    // RED on black
    CSI "33;40;1m",    // YELLOW on black
    CSI "37;40;1m",    // WHITE on black
    CSI "32;40;1m",    // GREEN on black
    CSI "34;40;1m",    // BLUE on black
    CSI "0m",          // DEFAULT on black
};


/* Set Text Color */
void
text_color(int chan, enum TEXT_COLOR color) {
    switch (color) {
        case RED:
            return uart_puts(chan, __screen_colors[0]);
        case YELLOW:
            return uart_puts(chan, __screen_colors[1]);
        case WHITE:
            return uart_puts(chan, __screen_colors[2]);
        case GREEN:
            return uart_puts(chan, __screen_colors[3]);
        case BLUE:
            return uart_puts(chan, __screen_colors[4]);
        default:
            return uart_puts(chan, __screen_colors[5]);
    }
}

