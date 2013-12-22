/*
 * Some functions for dumping out the contents of memory in
 * hexadecimal.
 */

#include <stdint.h>
#include "bb.h"

/* Convienience defines for uart_putnum */
#define FMT_BYTE    FMT_BASE_16 | FMT_LEADING_ZERO | FMT_SIZE_BYTE
#define FMT_LONG    FMT_BASE_16 | FMT_LEADING_ZERO | FMT_SIZE_LONG

/*
 * Dump out the memory contents at address 'addr'
 * This is 'classic' hex dump format
 *
 * Note that the 'base' parameter allows you to
 * have the addresses be shown relative to the 
 * the address passed. This lets the SD card code
 * display them as 0x000 - 0x1ff rather than the
 * address of the buffer they are held in.
 */
uint8_t *
dump_line(int c, uint8_t *addr, uint8_t *base) {
    uint8_t *line_addr;
    uint8_t b;
    uint32_t tmp;
    int i;

    line_addr = addr;
    text_color(c, YELLOW);
    tmp = (uint32_t)line_addr - (uint32_t) base;
    uart_putnum(c, FMT_LONG, tmp);
    uart_puts(c, " | ");
    text_color(c, GREEN);
    for (i = 0; i < 16; i++) {
        uart_putnum(c, FMT_BYTE, *(line_addr+i));
        uart_putc(c, ' ');
        if (i == 7) {
            uart_puts(c, "  ");
        }
    }
    text_color(c, WHITE);
    uart_puts(c, "| ");
    for (i = 0; i < 16; i++) {
        b = *line_addr++;
        uart_putc(c, ((b > 126) || (b < 32)) ? '.' : (char) b);
    }
    text_color(c, DEFAULT);
    uart_puts(c, "\n");
    return line_addr;
}


uint8_t *
dump_page(int chan, uint8_t *addr, uint8_t *base) {
    int i;
    for (i = 0; i < 16; i++) {
        addr = dump_line(chan, addr, base);
    }
    return addr;
}

