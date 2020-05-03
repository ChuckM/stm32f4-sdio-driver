/*
 * Simple monitor for the EMBEST Baseboard + STM32F4-Discovery
 * board. 
 */

#include <stdint.h>
#include <libopencm3/stm32/sdio.h>
#include <libopencm3/cm3/scb.h>
#include "FreeRTOS.h"
#include "task.h"
#include "bb.h"
#include "debug.h"

/* 
 * Not sure I love this design, but this integer is the 'console'
 * port. That is the port where all the serial I/O is to be sent
 * or 'standard out' in UNIX/Linux parlance. It is defined as a global
 * here because it is impractical to be sending all around, much like
 * the file descriptor for standard out is global for the same reason.
 * It also means that if someone or something stomps on it then your
 * code will stop writing to the serial port you expected.
 */
int console = 0;

const char *greet = "\nARM Baseboard Monitor v0.01\nEnter Command, ? or H for help.\n";

void do_cmd(int);
void taskMain(void *x);
#define is_hex(c) ((((c) >= '0') && ((c) <= '9')) ||\
                   (((c) >= 'a') && ((c) <= 'f')) ||\
                   (((c) >= 'A') && ((c) <= 'F')))

#define CMD_READ_SD     1
#define CMD_WRITE_SD    2
#define CMD_IDENT_SD    3
void do_cmd(int cmd) {
    switch (cmd) {
        case CMD_READ_SD:
        case CMD_WRITE_SD:
        case CMD_IDENT_SD:
            break;
        default:
            uart_puts(console, (const char *)"Unrecognized command\n");
    }
}

char buf[256];


#define valid_addr(x) \
    ((((uint32_t) x >= 0x10000000) && ((uint32_t) x < 0x10010000)) ||\
     (((uint32_t) x >= 0x20000000) && ((uint32_t) x < 0x20020000)) ||\
     (((uint32_t) x >= 0x08000000) && ((uint32_t) x < 0x08100000)))

#define MIN_ADDR (uint8_t *)(0x20000000)
void
taskMain(void *x)
{
    (void)x;
    char c;
    uint8_t *addr;

    sdio_init();
    text_color(console, DEFAULT);
    clear_screen(console);
    move_cursor(console, 1,1);
    uart_puts(console, (const char *)greet);
    uart_puts(console, "Endian test : ");
    uart_putnum(console, FMT_HEX_CONSTANT, 0xaabbccdd);
    uart_puts(console, "\n");
    uart_puts(console, "Greeting is at : 0x");
    uart_putnum(console, FMT_HEX_CONSTANT, (uint32_t)(greet));
    uart_puts(console, "\n");
    addr = MIN_ADDR;
    move_cursor(console, 13, 1);
    addr = dump_page(console, addr, NULL);
    addr = dump_page(console, addr, NULL);
    /* really should go into command line mode here */
    while(1) {
        move_cursor(console, 11, 1);
        uart_puts(console, "Enter Command:                                ");
        move_cursor(console, 11, 16);
        c = uart_getc(console, 1);
        switch (c) {
            case 'd':
                uart_puts(console, "dump (address) :");
                addr = (uint8_t *)uart_getnum(console);
                addr = (valid_addr(addr)) ? addr : MIN_ADDR;
                move_cursor(console, 13, 1);
                addr = dump_page(console, addr, NULL);
                addr = dump_page(console, addr, NULL);
                break;
            case '\r':
                uart_puts(console, "\n");
                move_cursor(console, 13, 1);
                addr = dump_page(console, addr, NULL);
                addr = dump_page(console, addr, NULL);
                break;
            case 'a':
                uart_puts(console, "address: ");
                addr = (uint8_t *)uart_getnum(console);
                addr = (valid_addr(addr)) ? addr : MIN_ADDR;
                break;
            case 's':
                sdio_explorer();
                break;
            default:
                uart_puts(console, "?\n");
        }
    }
}

int
main(void)
{
    scb_set_priority_grouping(SCB_AIRCR_PRIGROUP_GROUP16_NOSUB);  // Required by FreeRTOS

    // setup 115,200 baud
    bb_setup(115200);
    debug_init();
    debug_puts("\nBBMON: Debug Channel\n");

    xTaskCreate(taskMain,      "Main",        512, NULL, 1, NULL);
    vTaskStartScheduler();
    return 0;
}

