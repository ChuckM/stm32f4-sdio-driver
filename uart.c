/*
 * Copyright (c) 2013 - Chuck McManis, all rights reserved.
 *
 * A slightly more complex UART/USART driver in order to
 * facilitate multiple character streams. When debugging
 * this example I found it useful to have a couple of
 * streams going. Note that debugging it under GDB is problematic
 * as GDB (at least in SWD mode) doesn't seem to allow interrupts
 * to happen while it is running.
 */
#include <stdint.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include "uart.h"
#include "clock.h"
#include "debug.h"

#ifndef NULL
#define NULL (char *)(0)
#endif

/* Until libopencm3 gets defines for the STM32F42[79] which have
 * two additional UARTS
 */

#ifndef UART7
#include "stm32f42x_uart.h"
#endif

/* This defines how many channels we can have configured/opened at once */
#ifndef MAX_UART_CHANNELS
#define MAX_UART_CHANNELS 2
#endif

/* These are the attributes we can fetch with pin_map() */
enum PIN_ATTRIBUTE {
    USART, APB_REG, APB_ENA, BIT, GPIO_ENA, IRQ, AF, GPIO
};

/*
 * Forward Prototypes
 */
static void common_usart_isr(uint32_t, int);

/* This function is at the bottom of the file which helps map info from the PIN
 * to the various constants needed by the initialization function.
 */
static uint32_t uart_pin_map(enum UART_PORT_PIN pin, enum PIN_ATTRIBUTE attr);

/*
 * We hold buffers for all possible ports that can
 * consume more ram than you would like, if so feel
 * free to change this define. Recommend you keep it at
 * least at 16 though.
 */
#define UART_BUF_SIZE    32

static volatile char recv_buf[MAX_UART_CHANNELS][UART_BUF_SIZE];
static volatile char xmit_buf[MAX_UART_CHANNELS][UART_BUF_SIZE];
static volatile uint16_t nxt_recv_ndx[MAX_UART_CHANNELS];
static volatile uint16_t cur_recv_ndx[MAX_UART_CHANNELS];
static volatile uint16_t nxt_xmit_ndx[MAX_UART_CHANNELS];
static volatile uint16_t cur_xmit_ndx[MAX_UART_CHANNELS];

#define NUARTS  8   // Total possible uarts;

/* This maps each UART to the channel it was assigned to */
static int uart_to_channel_map[NUARTS];
/* This maps each channel to the UART serving it */
static int channel_to_uart_map[MAX_UART_CHANNELS];

/* This is the order of UARTs in the uart_to_channel_map */
static const uint32_t UART_MAP[] = {
    USART1,
    USART2,
    USART3,
    UART4,
    UART5,
    USART6,
    UART7,
    UART8
};

/*
 * Common USART/UART transmit and receive interrupts are
 * collated to here. Each serial port simply redirects to here
 * while passing in its BASE address in peripheral space and the
 * channel # it has been assigned. The channel number mapping is
 * established at initialization time.
 */
void
common_usart_isr(uint32_t usart, int channel) {
    if (USART_SR(usart) & USART_SR_RXNE) {
        recv_buf[channel][nxt_recv_ndx[channel]] = USART_DR(usart);
        nxt_recv_ndx[channel] = (nxt_recv_ndx[channel] + 1) % UART_BUF_SIZE;
    }
    if (USART_SR(usart) & USART_SR_TXE) {
        if (nxt_xmit_ndx[channel] == cur_xmit_ndx[channel]) {
            usart_disable_tx_interrupt(usart);  // nothing to send
        } else {
            USART_DR(usart) = xmit_buf[channel][cur_xmit_ndx[channel]];
            cur_xmit_ndx[channel] = (cur_xmit_ndx[channel] + 1) % UART_BUF_SIZE;
        }
    }
}

/* Individual ISR's for the serial ports just redirect */
void usart1_isr() {
    common_usart_isr(USART1, uart_to_channel_map[0]);
}

void usart2_isr() {
    common_usart_isr(USART2, uart_to_channel_map[1]);
}

void usart3_isr() {
    common_usart_isr(USART3, uart_to_channel_map[2]);
}

void uart4_isr() {
    common_usart_isr(UART4_BASE, uart_to_channel_map[3]);
}

void uart5_isr() {
    common_usart_isr(UART5_BASE, uart_to_channel_map[4]);
}

void usart6_isr() {
    common_usart_isr(USART6, uart_to_channel_map[5]);
}

void uart7_isr() {
    common_usart_isr(UART7, uart_to_channel_map[6]);
}

void uart8_isr() {
    common_usart_isr(UART8, uart_to_channel_map[7]);
}

/* Next available channel to configure */
static int nxt_channel = 0;

/*
 * uart_init(tx pin, rx pin, baudrate);
 *
 * Initialize a UART that will talk on the tx/rx pin pair at a given baudrate
 * Curently only 8n1 format, no-hw flow control, only.
 *
 * Returns channel unumber (0 - MAX_UART_CHANNELS) or -1 on error
 */
int
uart_init(enum UART_PORT_PIN tx, enum UART_PORT_PIN rx, int baudrate) {
    uint32_t my_uart = uart_pin_map(tx, USART);
    int i;

    if (uart_pin_map(rx, USART) != my_uart) {
        /* Both pins are not connected to same serial port */
        return -1;
    }
    if (nxt_channel >= MAX_UART_CHANNELS) {
        /* Need more channel configured */
        return -2;
    }

    /* Enable Clock for the USART/UART involved */
    rcc_peripheral_enable_clock((uint32_t *)uart_pin_map(tx, APB_REG),
                                uart_pin_map(tx, APB_ENA));

    /* Enable Clock for the GPIOs we are using */
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, uart_pin_map(tx, GPIO_ENA));
    if (uart_pin_map(rx,GPIO_ENA) != uart_pin_map(tx,GPIO_ENA)) {
        rcc_peripheral_enable_clock(&RCC_AHB1ENR, uart_pin_map(rx, GPIO_ENA));
    }

    /* GPIO pins */

    /* Both AF Mode */
    gpio_mode_setup(uart_pin_map(tx, GPIO), GPIO_MODE_AF, GPIO_PUPD_NONE,
                                            uart_pin_map(tx, BIT));
    gpio_mode_setup(uart_pin_map(rx, GPIO), GPIO_MODE_AF, GPIO_PUPD_NONE,
                                            uart_pin_map(rx, BIT));
    gpio_set_af(uart_pin_map(tx, GPIO),
                uart_pin_map(tx, AF),
                uart_pin_map(tx, BIT));

    gpio_set_af(uart_pin_map(rx, GPIO),
                uart_pin_map(rx, AF),
                uart_pin_map(rx, BIT));

    /* Transmit pin set to an output */
    gpio_set_output_options(uart_pin_map(tx, GPIO), GPIO_OTYPE_PP,
        GPIO_OSPEED_25MHZ, uart_pin_map(tx, BIT));


    /* Set up UART parameters */
    usart_set_baudrate(my_uart, baudrate);
    usart_set_databits(my_uart, 8);
    usart_set_stopbits(my_uart, USART_STOPBITS_1);
    usart_set_mode(my_uart, USART_MODE_TX_RX);
    usart_set_parity(my_uart, USART_PARITY_NONE);
    usart_set_flow_control(my_uart, USART_FLOWCONTROL_NONE);
    usart_enable(my_uart);

    nxt_recv_ndx[nxt_channel] = cur_recv_ndx[nxt_channel] = 0;
    nxt_xmit_ndx[nxt_channel] = cur_xmit_ndx[nxt_channel] = 0;
    /*
     * This was done to try to get it to run under GDB with the Black
     * Magic debug probe but it didn't have any effect (interrupts
     * are still masked)
     */
    nvic_set_priority(uart_pin_map(tx, IRQ), 0); // highest priority
    nvic_enable_irq(uart_pin_map(tx, IRQ));
    USART_DR(my_uart) = 0;
    usart_enable_rx_interrupt(my_uart);

    /* Now create two mappings, channel => usart, and usart => channel */
    channel_to_uart_map[nxt_channel] = my_uart;
    for (i = 0; i < NUARTS; i++) {
        if (UART_MAP[i] == my_uart) {
            uart_to_channel_map[i] = nxt_channel;
            break;
        }
    }
    nxt_channel++;
    return (nxt_channel - 1);
}

/*
 * This is a small hack which takes function keys (arrow keys, etc)
 * and returns a 'character' for them. It waits briefly to (2mS) to see
 * if another character is going to follow an ESC and if it does then
 * it trys to collect the entire escape sequence which can be translated
 * into a function key (or special key).
 */
static uint8_t fkey_buf[5];
static int fkey_ndx;

static uint8_t map_function_key(void);

/*
 * Ok, if we got a function key sequence we look here to map it
 * into an 8 bit return value. It was simply $[<char> we take the
 * character returned and turn on bit 7.
 */
static uint8_t
map_function_key(void) {
    if (fkey_ndx == 1) {
        return 0x80 | fkey_buf[1];
    }
    return 0xff;
}

/* --- "high" level APIs (channel based) ---
 *
 * uart_getc(channel, wait)
 *
 * If wait is non-zero the function blocks until a character
 * is available. If wait is 0 then the function returns immediately
 * with '0' as the value. (yes this means it can't read 'nul' bytes
 * interactively)
 */
char
uart_getc(int channel, int wait) {
    char res;

    if ((cur_recv_ndx[channel] == nxt_recv_ndx[channel]) && ! wait) {
        return '\000';
    }
    while (cur_recv_ndx[channel] == nxt_recv_ndx[channel]) {
        __asm__("NOP");
    }
    res = recv_buf[channel][cur_recv_ndx[channel]];
    cur_recv_ndx[channel] = (cur_recv_ndx[channel] + 1) % UART_BUF_SIZE;
    /* check for function key */
    /* as written this code depends on the function key
     * string coming in at full speed and there being a
     * slight delay with the next character, but that might
     * not be a good assumption. We'll see if it is good enough
     * for now.
     *
     * In testing 2 mS sleeps to be sure another character isn't
     * coming in seems to be ok, also you don't normally type the CSI
     * sequence (^[ [) on your own. you can "fool" the input driver
     * if you do into trying to interpret a function key sequence with
     * unpredictable results.
     */
    fkey_ndx = 0;
    if (res == 27) {
        msleep(2);
        if ((cur_recv_ndx[channel] != nxt_recv_ndx[channel]) &&
            (recv_buf[channel][cur_recv_ndx[channel]] == 91)) {
            for (fkey_ndx = 0; fkey_ndx < 5; fkey_ndx++) {
                fkey_buf[fkey_ndx] = recv_buf[channel][cur_recv_ndx[channel]];
                cur_recv_ndx[channel] = (cur_recv_ndx[channel] + 1) %
                                                            UART_BUF_SIZE;
                msleep(2);
                if (cur_recv_ndx[channel] == nxt_recv_ndx[channel]) {
                    break; // no more characters
                }
            }
        }
    }
    return (fkey_ndx == 0) ? res : map_function_key();
}

/*
 * char *uart_gets()
 *
 * This is the 'old school' getstring function. It allows for basic
 * editng (delete or backspace) and returns on the receipt of <CR>.
 * It null terminates the string as it builds it and returns if it
 * runs out of space, it also returns if the <CR> is typed, and sends
 * ^G (BELL) if the input tries to delete before the start of the
 * buffer.
 */
char *
uart_gets(int chan, char *buf, int len) {
    char *line_buf = buf;
    char c;

    *line_buf = '\000';
    // read until you see a <CR>
    while (1) {
        c = uart_getc(chan, 1);
        if (c == '\r') {
            uart_putc(chan, '\r');
            uart_putc(chan, '\n');
            break;
        }
        // if you see a ^H or ^? (<DEL>) delete last character
        if ((c == '\010') || (c == '\177')) {
            if (line_buf > buf) {
                uart_puts(chan, "\010 \010"); // back up one space
                if (line_buf > buf) {
                    line_buf--;
                }
                *line_buf = '\000';
            } else {
                uart_putc(chan, '\a');  // beep the console
            }
        } else {
            uart_putc(chan, c);
            *line_buf = c;
            line_buf++;
            // if you're at the end of the buffer just return
            if (line_buf < (buf + len)) {
                *line_buf = '\000';
            } else {
                --line_buf;
                *line_buf = '\000';
                break;
            }
        }
    }
    return buf;
}

/*
 * uart_putc
 *
 * This pushes a character into the transmit buffer for
 * the channel and turns on TX interrupts (which will fire
 * because initially the register will be empty.) If the
 * ISR sends out the last character it turns off the transmit
 * interrupt flag, so that it won't keep firing on an empty
 * transmit buffer.
 */
void
uart_putc(int chan, char c) {
    uint32_t usart = channel_to_uart_map[chan];

    /* block if the transmit buffer is full */
    while (((nxt_xmit_ndx[chan] + 1) % UART_BUF_SIZE) == cur_xmit_ndx[chan]) {
        __asm__("NOP");
    }
    xmit_buf[chan][nxt_xmit_ndx[chan]] = c;
    nxt_xmit_ndx[chan] = (nxt_xmit_ndx[chan] + 1) % UART_BUF_SIZE;
    usart_enable_tx_interrupt(usart);
}

/*
 * uart_puts(char *string)
 *
 * Write a NUL terminated string to the UART. This routine writes
 * the characters in order (adding LF to CR). Note that line cleanup
 * is done here and not in putc so that you can send just \r or what
 * ever you want through putc without worrying about it.
 */
void
uart_puts(int chan, const char *s) {
    if (s == NULL)
        return;

    while (*s) {
        if (*s == '\n') {
            uart_putc(chan, '\r');
        }
        uart_putc(chan, *s++);
    }
}

/*
 * UART Pin Mapping
 *
 * To faciliate a very flexible USART/UART configuration utility
 * I needed a way to encode the relationship of various pins to
 * their USART registers, there GPIO pins, the various AHB and APB
 * registers. There are probably a zillion ways to do this but I
 * chose to create code which has embedded in it the notion of what
 * goes where. So that I could query it with a simple API.
 *
 * The function uart_pin_map takes a UART_PORT_PIN and can return
 * information about it like which GPIO it needes enabled, what flag
 * in the APB register it needs on, etc. It does not care if you use
 * the TX pin out of one GPIO port and the RX pin out of another.
 *
 * The only ambiguity is UART4/USART3 which share TX and RX pins
 * and differ only by their Alternate function value (7 or 8). So
 * we don't support UART4 on those pins (only USART3).
 */

/*
 * Prototypes for helper functions
 */

static uint32_t pin_to_bit(enum UART_PORT_PIN pin);
static uint32_t pin_to_enable(enum UART_PORT_PIN pin);
static uint32_t pin_uart_attr(enum UART_PORT_PIN pin, int attr);
static uint32_t pin_to_af(enum UART_PORT_PIN pin);
static uint32_t pin_to_gpio(enum UART_PORT_PIN pin);

/* A really simple one, this maps the name (like PA0) to the bit number
 * define like GPIO0.
 */
static uint32_t
pin_to_bit(enum UART_PORT_PIN pin) {
    switch (pin) {
        case PA0:
        case PE0: return GPIO0;
        case PA1:
        case PE1: return GPIO1;
        case PA2:
        case PD2: return GPIO2;
        case PA3: return GPIO3;
        case PD5: return GPIO5;
        case PB6:
        case PC6:
        case PD6:
        case PF6: return GPIO6;
        case PB7:
        case PC7:
        case PE7:
        case PF7: return GPIO7;
        case PD8:
        case PE8: return GPIO8;
        case PA9:
        case PD9:
        case PG9: return GPIO9;
        case PB10:
        case PA10:
        case PC10: return GPIO10;
        case PB11:
        case PC11: return GPIO11;
        case PC12: return GPIO12;
        case PG14: return GPIO14;
        default: return 0xfffffff;
    }
}

/* Return base register for each GPIO based on PIN name */
static uint32_t
pin_to_gpio(enum UART_PORT_PIN pin) {
    switch (pin) {
        case PA0:
        case PA1:
        case PA2:
        case PA3:
        case PA9:
        case PA10:
            return GPIOA;
        case PB6:
        case PB7:
        case PB10:
        case PB11:
            return GPIOB;
        case PC6:
        case PC7:
        case PC10:
        case PC11:
        case PC12:
            return GPIOC;
        case PD2:
        case PD5:
        case PD6:
        case PD8:
        case PD9:
            return GPIOD;
        case PE0:
        case PE1:
        case PE7:
        case PE8:
            return GPIOE;
        case PF6:
        case PF7:
            return GPIOF;
        case PG9:
        case PG14:
            return GPIOG;
        default: return 0xfffffff;
    }
}

/*
 * This maps a pin to its GPIO clock, since you have to enable
 * the GPIO clock for the port this takes care of that for us.
 */
static uint32_t
pin_to_enable(enum UART_PORT_PIN pin) {
    switch (pin) {
        case PA0:
        case PA1:
        case PA2:
        case PA3:
        case PA9:
        case PA10: return RCC_AHB1ENR_IOPAEN;

        case PB6:
        case PB7:
        case PB10:
        case PB11: return RCC_AHB1ENR_IOPBEN;

        case PC6:
        case PC7:
        case PC10:
        case PC11:
        case PC12: return RCC_AHB1ENR_IOPCEN;

        case PD2:
        case PD5:
        case PD6:
        case PD8:
        case PD9: return RCC_AHB1ENR_IOPDEN;

        case PE0:
        case PE1:
        case PE7:
        case PE8: return RCC_AHB1ENR_IOPEEN;

        case PF6:
        case PF7: return RCC_AHB1ENR_IOPFEN;

        case PG9:
        case PG14: return RCC_AHB1ENR_IOPGEN;
        default: return 0xfffffff;
    }
}

/*
 * A number of attributes common to each USART/UART such as
 * their ENABLE flag in the APB register, their NVIC interrupt
 * vector, their 'home' APB register (1 or 2) and their base
 * address.
 */
static uint32_t
pin_uart_attr(enum UART_PORT_PIN pin, int attr) {
    switch (pin) {
        case PA0:
        case PA1:
            switch (attr) {
                case 1: return RCC_APB1ENR_UART4EN;
                case 2: return (uint32_t) &RCC_APB1ENR;
                case 3: return NVIC_UART4_IRQ;
                default: return UART4;
            }

        case PA2:
        case PA3:
        case PD5:
        case PD6:
            switch (attr) {
                case 1: return RCC_APB1ENR_USART2EN;
                case 2: return (uint32_t) &RCC_APB1ENR;
                case 3: return NVIC_USART2_IRQ;
                default: return USART2;
            }

        case PA9:
        case PA10:
        case PB6:
        case PB7:
            switch (attr) {
                case 1: return RCC_APB2ENR_USART1EN;
                case 2: return (uint32_t) &RCC_APB2ENR;
                case 3: return NVIC_USART1_IRQ;
                default: return USART1;
            }

        case PB10:
        case PB11:
        case PC10:
        case PC11:
        case PD8:
        case PD9:
            switch (attr) {
                case 1: return RCC_APB1ENR_USART3EN;
                case 2: return (uint32_t) &RCC_APB1ENR;
                case 3: return NVIC_USART3_IRQ;
                default: return USART3;
            }

        case PC6:
        case PC7:
        case PG9:
        case PG14:
            switch (attr) {
                case 1: return RCC_APB2ENR_USART6EN;
                case 2: return (uint32_t) &RCC_APB2ENR;
                case 3: return NVIC_USART6_IRQ;
                default: return USART6;
            }

        case PC12:
        case PD2:
            switch (attr) {
                case 1: return RCC_APB1ENR_UART5EN;
                case 2: return (uint32_t) &RCC_APB1ENR;
                case 3: return NVIC_UART5_IRQ;
                default: return UART5;
            }

        case PE0:
        case PE1:
            switch (attr) {
                case 1: return RCC_APB1ENR_UART8EN;
                case 2: return (uint32_t) &RCC_APB1ENR;
                case 3: return NVIC_UART8_IRQ;
                default: return UART8;
            }

        case PE7:
        case PE8:
        case PF6:
        case PF7:
            switch (attr) {
                case 1: return RCC_APB1ENR_UART7EN;
                case 2: return (uint32_t) &RCC_APB1ENR;
                case 3: return NVIC_UART7_IRQ;
                default: return UART7;
            }

        default: return 0xfffffff;
    }
}

/* This one then maps which Alternate function flag is needed for a pin
 * to make it a UART pin
 */
static uint32_t
pin_to_af(enum UART_PORT_PIN pin) {
    switch (pin) {
        case PA0:
        case PA1:
        case PC6:
        case PC7:
        case PC12:
        case PD2:
        case PE0:
        case PE1:
        case PE7:
        case PE8:
        case PF6:
        case PF7:
        case PG9:
        case PG14:
            return GPIO_AF8;

        case PA2:
        case PA3:
        case PA9:
        case PA10:
        case PB6:
        case PB7:
        case PB10:
        case PB11:
        case PC10:
        case PC11:
        case PD5:
        case PD6:
        case PD8:
        case PD9:
            return GPIO_AF7;
        default: return 0xfffffff;
    }
}

/*
 * pin_map
 *
 * This code encodes the relationship between pins and various chip
 * registers, the USART, the AHB bus, the Alternate function code,
 * Etc. Call it with a pin and an attribute and get back an answer.
 *
 * See the UART initialization code for an example of its use.
 */
static uint32_t
uart_pin_map(enum UART_PORT_PIN pin, enum PIN_ATTRIBUTE attr) {
    switch (attr) {
        case USART:
            return pin_uart_attr(pin, 0);
        case APB_ENA:
            return pin_uart_attr(pin, 1);
        case APB_REG:
            return pin_uart_attr(pin, 2);
        case IRQ:
            return pin_uart_attr(pin, 3);
        case BIT:
            return pin_to_bit(pin);
        case GPIO_ENA:
            return pin_to_enable(pin);
        case AF:
            return pin_to_af(pin);
        default:
            return pin_to_gpio(pin);
    }
}

/*
 * Convert a number to a string in the desired
 * base.
 *
 * Assumes:
 *      - buf has enough space (up to 35 bytes for
 *        a 32 bit number in binary)
 * Verifies:
 *      - base is one of 2, 8, 10, or 16
 * Creates:
 *      - null string on error ("")
 *
 * This got a bit out of hand but I wanted just *one* number printing
 * function rather than several. 
 *
 * Notes from my notebook:
 * Number consists of:
 *
 *      PREFIX|NUMBER|SUFFIX
 *     |------ width -------|
 *     |------ field width -----|

 * Prefix is one of "", "-", "0x", "0b", "0", "-0x" "-0b", "-0"
 * Suffix is one of "", ".0"
 *
 * The independent variable is the leading zero question.
 * So when LEADING ZERO is set you pad the width of Number to fill
 * the width, up to 32 digits of precision.
 *
 * With LEFT adjust you pad right, without it you pad left.
 *
 * Examples:
 *   0x12345000 - hex + alternate form
 *     1234abcd - just hex 
 *           0C - hex + width of 1 byte + leading 0
 *         10.0 - decimal + alternate form
 *      0b11011 - Binary + alternate form
 *   0b00011011 - Binary + alternate form + leading 0 + 1 byte
 *      0100377 - octal + alternate form + 2 bytes
 * 037777777777 - octal equivalent of -1
 *         -1.0 - decimal + sign + alternate form (adds the .0)
 *
 * So I'm still going back and forth on the width. It would be handy
 * to make it just 1, 2, or 4 for 1 byte value, 2 byte value, 4 byte
 * value with 8 reserved for 8 byte value. As that is the common use
 * case. And it would only apply to how many digits of the number were
 * considered and not the field width.
 */
void
ntoa(uint32_t val, uint16_t fmt, char *buf) {
    char *d;
    uint32_t mask;
    int base, size, width;
    int sign_bit = 0;

    d = buf;
    *d = '\000';
    d++;
    // defaults if not specified
    mask = 0xffffffff;
    base = 10;
    width = 0;
    size = fmt & FMT_SIZE_MASK;
    /* sigh how to make this cleaner? */
    switch (fmt & FMT_BASE_MASK) {
        case FMT_BASE_2:
            base = 2;
            switch (size) {
                case 1:
                    mask = 0xff;
                    width = 8;
                    break;  
                case 2:
                    mask = 0xffff;
                    width = 16;
                    break;
                case 4:
                    mask = 0xffffffff;
                    width = 32;
                    break;
                default:
                    width = 0;
                    break;
            }
            break;
        case FMT_BASE_8:
            base = 8;
            switch (size) {
                case 1:
                    mask = 0xff;
                    width = 3;
                    break;  
                case 2:
                    mask = 0xffff;
                    width = 6;
                    break;
                case 4:
                    mask = 0xffffffff;
                    width = 11;
                    break;
                default:
                    width = 0;
                    break;
            }
            break;
        case FMT_BASE_16:
            base = 16;
            switch (size) {
                case 1:
                    mask = 0xff;
                    width = 2;
                    break;  
                case 2:
                    mask = 0xffff;
                    width = 4;
                    break;
                case 4:
                    mask = 0xffffffff;
                    width = 8;
                    break;
                default:
                    width = 0;
                    break;
            }
            break;
        case FMT_BASE_10:
            base = 10;
            switch (size) {
                case 1:
                    mask = 0xff;
                    width = 3;
                    sign_bit = ((val & 0x80) != 0);
                    break;  
                case 2:
                    mask = 0xffff;
                    width = 5;
                    sign_bit = ((val & 0x8000) != 0);
                    break;
                case 4:
                    mask = 0xffffffff;
                    width = 10;
                    sign_bit = ((val & 0x80000000) != 0);
                    break;
                default:
                    width = 0;
                    sign_bit = ((val & 0x80000000) != 0);
                    break;
            }
            if (fmt & FMT_ALTERNATE_FORM) {
                *d++ = '0';
                *d++ = '.';
            }
            break;
        default:
            break;
    }
    /* this assumes if you send it a byte, it gets promoted and sign extended */
    if ((fmt & (FMT_SIGNED | FMT_BASE_MASK)) == (FMT_SIGNED | FMT_BASE_10)) {
        val = (val & 0x80000000) ? -val : val;
    }
    val = val & mask;

    do {
        *d++ = ((val % base) < 10) ? (val % base) + '0' :
                                                 (val % base) + '7';
        val /= base;
        width--;
    } while (val > 0);

    if (fmt & FMT_LEADING_ZERO) {
        while (width > 0) {
            *d++ = '0';
            width--;
        }
    }
    if (fmt & FMT_ALTERNATE_FORM) {
        switch (fmt & FMT_BASE_MASK) {
            case FMT_BASE_16:
                *d++ = 'x';
                *d++ = '0';
                break;
            case FMT_BASE_8:
                *d++ = '0';
                break;
            case FMT_BASE_2:
                *d++ = 'b';
                *d++ = '0';
            default:
                break;
        }
    }
    if (sign_bit && (fmt & FMT_SIGNED)) {
        *d++ = '-';
        width--;
    }

    /*
     * So how often do you get to use an interview question in your
     * code? 
     * At this point the string is in buf, only backwards so reverse it
     * in place.
     */
    d--;    // Point d at last character
    do {
        *d ^= *buf;
        *buf ^= *d;
        *d ^= *buf;
    } while (++buf < --d);
    return;
}

/*
 * Convert a string to a number in the desired
 * 
 * Assumes:
 *      - Stops at first non-legal digit in guessed
 *        base or NUL character.
 *      - only does signed input on decimal numbers
 * Returns:
 *      - 0 even on failure.
 */
uint32_t
aton(char *buf) {
    uint32_t res = 0;
    int     base = 10;
    int     sign_bit = 0;
    uint8_t digit;

    /* check for different base indicators */
    if (((*buf >= '1') && (*buf <= '9')) || (*buf == '-')) {
        if (*buf == '-') {
            sign_bit = 1;
            buf++;
        }
        base = 10;
    } else if ((*buf) == '0') {
        buf++;
        if (*buf == 'b') {
            base = 2;
            buf++;
        } else if (*buf == 'x') {
            base = 16;
            buf++;
        } else if ((*buf >= '0') && (*buf <= '7')) {
            base = 8;
        } else if (*buf == '\000') {
            return 0;
        }
    }
    while (*buf != '\000') {
        digit = (*buf > '9') ? ((*buf & 0xdf) - '7') : (*buf - '0');
        if (((base == 2) && (digit > 1)) ||
            ((base == 8) && (digit > 7)) ||
            ((base == 10) && (digit > 9)) ||
            ((base == 16) && (digit > 15))) {
            return (sign_bit) ? -res : res;
        }
        res *= base;
        res += digit;
        buf++;
    }
    return (sign_bit) ? -res : res;
}


/*
 * Print a number given various formatting options.
 * NB: No floating point yet. 
 *
 */
void
uart_putnum(int chan, uint16_t fmt, uint32_t num) {
    char buf[48];

    ntoa(num, fmt, buf);
    uart_puts(chan, buf);
    if (fmt & FMT_NEWLINE) {
        uart_puts(chan, "\n");
    }
}

/*
 * uart_getnum
 *
 * Read a number in from the specified channel. Accepts base
 * decorators (like 0x) to distinguish hexadecimal numbers from
 * decimal ones. See aton for the list.
 */
uint32_t
uart_getnum(int chan) {
    char buf[48];

    uart_gets(chan, buf, 48);
    return aton(buf);
}
