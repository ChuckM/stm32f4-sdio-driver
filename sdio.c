/*
 * sdio.c
 *
 * SDIO Bus Driver layer. This code sends commands and drives
 * the SDIO peripheral on the STM32F4xx, there is a layer above
 * this, the SD Card driver, which uses this driver to talk to
 * SD Cards. The SPI driver can also talk to SD Cards, hence the
 * split at this layer.
 *
 * Note that the simple implementation for the SDIO driver runs
 * in a 'polled' mode. This is easier to explain and debug and
 * sufficient for the first few projects. A more sophisticated
 * version with DMA and interrupts will follow.
 *
 * Could be part of the libopencm3 project if they wanted it.
 */

#include <stdint.h>
#include <libopencm3/stm32/sdio.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include "term.h"
#include "uart.h"
#include "sdio.h"
#include "debug.h"
#include "clock.h"
#include "FreeRTOS.h"
#include "task.h"

/*
 * Some Global defines, collected here.
 */


/* shorthand for debug_puts which used to be so that we could
 * turn it on/off with #define DEBUG(x) to nothing. But with
 * putnum that doesn't work as nicely as I would like.
 */
#define DEBUG(x) debug_puts((x))

/*
 * The logging feature. Useful during debugging but consumes
 * (28 * SDIO_LOGSIZE)+4 bytes of RAM to hold the log and the
 * index into the log.  Default size is 10 entries (see below)
 */
#define SDIO_LOGGING

/* The input-output buffers */
static TaskHandle_t xTaskToNotify = 0;
static int selected_rca = 0;

void
sdio_isr() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if( SDIO_STA & SDIO_STA_DBCKEND ) {
        SDIO_ICR |= SDIO_ICR_DATAENDC | SDIO_ICR_DBCKENDC;
        SDIO_MASK &= ~(SDIO_MASK_DATAENDIE | SDIO_MASK_DBCKENDIE);
        if( xTaskToNotify )
            vTaskNotifyGiveFromISR( xTaskToNotify, &xHigherPriorityTaskWoken );
    }
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/*
 * The Embest board ties PB15 to 'card detect' which is useful
 * for aborting early, and detecting card swap. Needs porting
 * for other implementations.
 */
#define SDIO_HAS_CARD_DETECT

/*
 * Not defined by default
 */
#ifndef NULL
#define NULL (void *)(0x00000000)
#endif

/*
 * Helper defines to pull out various bit fields of the CSD for
 * the size calculation.
 */
#define SDIO_CSD_VERSION(x) sdio_bit_slice(x->csd, 128, 127, 126)
#define SDIO_CSD1_CSIZE_MULT(x) sdio_bit_slice(x->csd, 128, 49, 47)
#define SDIO_CSD1_RBLKLEN(x) sdio_bit_slice(x->csd, 128, 83, 80)
#define SDIO_CSD1_CSIZE(x) sdio_bit_slice(x->csd, 128, 73, 62)
#define SDIO_CSD2_CSIZE(x) sdio_bit_slice(x->csd, 128, 69, 48)

/*
 * Conveniently swaps the bytes in a long around
 * used by the SCR code.
 */
#define byte_swap(val) \
        __asm__("rev %[swap], %[swap]" : [swap] "=r" (val) : "0" (val));

/*
 * sdio_bus
 *
 * Set the bus width and the clock speed for the
 * SDIO bus.
 *
 * Returns 0 on success
 *      -1 illegal bit specification
 *      -2 illegal clock specification
 */
int
sdio_bus(int bits, enum SDIO_CLOCK_DIV freq) {
    int clkreg = 0;

    switch (bits) {
        case 1:
            clkreg |= SDIO_CLKCR_WIDBUS_1;
            break;
        case 4:
            clkreg |= SDIO_CLKCR_WIDBUS_4;
            break;
        default:
            return -1;
    }
    switch (freq) {
        case SDIO_24MHZ:
            break;
        case SDIO_16MHZ:
            clkreg |= 1;
            break;
        case SDIO_12MHZ:
            clkreg |= 2;
            break;
        case SDIO_8MHZ:
            clkreg |= 8;
            break;
        case SDIO_4MHZ:
            clkreg |= 10;
            break;
        case SDIO_1MHZ:
            clkreg |= 46;
            break;
        case SDIO_400KHZ:
            clkreg |= 118;
            break;
        default:
            return -2;
    }
    clkreg |= SDIO_CLKCR_CLKEN;
    SDIO_CLKCR = clkreg;
    return 0;
}

static void dma_init(void)
{
    /* SDIO uses DMA controller 2 Stream 3 or 6 Channel 4. */
    rcc_periph_clock_enable(RCC_DMA2);
    dma_stream_reset(DMA2, DMA_STREAM3);
    dma_set_priority(DMA2, DMA_STREAM3, DMA_SxCR_PL_LOW);
    dma_set_memory_size(DMA2, DMA_STREAM3, DMA_SxCR_MSIZE_32BIT);
    dma_set_peripheral_size(DMA2, DMA_STREAM3, DMA_SxCR_PSIZE_32BIT);
    dma_enable_memory_increment_mode(DMA2, DMA_STREAM3);
    dma_set_peripheral_flow_control(DMA2, DMA_STREAM3);
    dma_set_peripheral_burst(DMA2, DMA_STREAM3, DMA_SxCR_PBURST_INCR4);
    dma_set_memory_burst(DMA2, DMA_STREAM3, DMA_SxCR_MBURST_INCR4);
    dma_enable_fifo_mode(DMA2, DMA_STREAM3);
    dma_set_fifo_threshold(DMA2, DMA_STREAM3, DMA_SxFCR_FTH_4_4_FULL);
    dma_set_peripheral_address(DMA2, DMA_STREAM3, (uint32_t) &SDIO_FIFO);
    dma_channel_select(DMA2, DMA_STREAM3, DMA_SxCR_CHSEL_4);
}

/*
 * Set up the GPIO pins and peripheral clocks for the SDIO
 * system. The code should probably take an option card detect
 * pin, at the moment it uses the one used by the Embest board.
 */
void
sdio_init(void)
{
    /* Enable clocks for SDIO and DMA2 */
    dma_init();
    rcc_periph_clock_enable(RCC_SDIO);

    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_GPIOD);

	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12 | GPIO15);

    /* Setup GPIO Pins for SDIO:
        PC8 - PC11 - DAT0 thru DAT3
              PC12 - CLK
               PD2 - CMD
    */
    // All SDIO lines are push-pull, 25Mhz
	gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ,
                            GPIO12 );

    // All SDIO lines are push-pull, 25Mhz
	gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ,
                            GPIO8 | GPIO9 | GPIO10 | GPIO11 );

    // D0 - D3 enable pullups (bi-directional)
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP,
                    GPIO8 | GPIO9 | GPIO10 | GPIO11);
    // CLK line no pullup
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE,  GPIO12);

	gpio_set_af(GPIOC, GPIO_AF12,
                GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12);
    gpio_set_af(GPIOD, GPIO_AF12, GPIO2);

    /* GPIOD setup */
	gpio_set_output_options(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO2);
	gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO2);

#ifdef SDIO_HAS_CARD_DETECT
    /* SDIO Card Detect pin on the Embest Baseboard */
    /*     PB15 as a hacked Card Detect (active LOW for card present) */
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO15);
#endif

    nvic_set_priority(NVIC_SDIO_IRQ, 11<<4);
    nvic_enable_irq(NVIC_SDIO_IRQ);

    selected_rca = 0;
    xTaskToNotify = xTaskGetCurrentTaskHandle();
    xTaskNotifyGive( xTaskToNotify ); // SDIO is available
}

/*
 * Reset the state of the SDIO bus and peripheral. This code tries
 * to reset the bus *AND* the card if one is plugged in. The bus
 * can be reset by software but the card is reset by powering it down.
 *
 * The SDIO_POWER_STATE tells the code which state to leave the bus in,
 * powered up or powered down.
 *
 * If the state is POWER_ON, then the bus is reset to 400Khz, 1 bit wide
 * which is what he spec requires. Once the type and capabilities of the
 * card have been determined, it can be upgraded.
 */
void
sdio_reset(enum SDIO_POWER_STATE state) {

    /* Step 1 power off the interface */
    SDIO_POWER = SDIO_POWER_PWRCTRL_PWROFF;
    /* reset the SDIO peripheral interface */
    rcc_peripheral_reset(&RCC_APB2RSTR, RCC_APB2RSTR_SDIORST);
    rcc_peripheral_clear_reset(&RCC_APB2RSTR, RCC_APB2RSTR_SDIORST);
    if (state == SDIO_POWER_ON) {
        SDIO_POWER = SDIO_POWER_PWRCTRL_PWRON;
        sdio_bus(1, SDIO_400KHZ); // required by the spec
    }
}

/*
 * The error message catalog.
 */
static const char *__sdio_error_msgs[] = {
    "Success",
    "Command Timeout",              // -1
    "Command CRC Failure",          // -2
    "Soft Timeout (No Response)",   // -3
    "Data CRC Failure",             // -4
    "RX FIFO Overrun",              // -5
    "TX FIFO Underrun",             // -6
    "Unsupported Card"              // -7
};

#define SDIO_ESUCCESS    0
#define SDIO_ECTIMEOUT  -1
#define SDIO_ECCRCFAIL  -2
#define SDIO_ENORESP    -3
#define SDIO_EDCRCFAIL  -4
#define SDIO_ERXOVERR   -5
#define SDIO_ETXUNDER   -6
#define SDIO_EBADCARD   -7
#define SDIO_EUNKNOWN   -8

/*
 * Return a text string description of the error code.
 */
const char *
sdio_errmsg(int err) {
    return (err <= SDIO_EUNKNOWN) ? (const char *) "Unknown Error" :
                                __sdio_error_msgs[0-err];
}

#ifdef SDIO_LOGGING

/*
 * SDIO_LOGSIZE determines the impact on RAM at 28 * LOG_SIZE + 4 bytes.
 */
#define SDIO_LOGSIZE 10

static struct log_entry {
    uint32_t arg;
    uint32_t resp[4];
    int err;
    uint8_t cmd;
} sdio_log_buffer[SDIO_LOGSIZE];

static int current_sdio_log_entry = 0;


/* This is a helper function which keeps a 'log' of the last "n"
 * commands and their results, sent through the command API. This
 * can be helpful when debugging why something didn't work, you go
 * back and look at the previous n commands and their results.
 */
static void
sdio_log(uint8_t cmd, uint32_t arg, int err)
{
    sdio_log_buffer[current_sdio_log_entry].cmd = cmd;
    sdio_log_buffer[current_sdio_log_entry].arg = arg;
    sdio_log_buffer[current_sdio_log_entry].err = err;
    sdio_log_buffer[current_sdio_log_entry].resp[0] = SDIO_RESP1;
    sdio_log_buffer[current_sdio_log_entry].resp[1] = SDIO_RESP2;
    sdio_log_buffer[current_sdio_log_entry].resp[2] = SDIO_RESP3;
    sdio_log_buffer[current_sdio_log_entry].resp[3] = SDIO_RESP4;
    current_sdio_log_entry = ((current_sdio_log_entry + 1) % SDIO_LOGSIZE);
}

/*
 * Send a log entry out to the indicated UART
 * Note this creates a dependency on the uart driver I wrote.
 * When logging is not enabled, the SDIO code is independent.
 */
static void
sdio_dump_log_entry(int u, struct log_entry *l) {
    int i;

    uart_puts(u, "CMD: ");
    uart_putnum(u, FMT_BASE_10, l->cmd);
    uart_puts(u, ", ARG: ");
    uart_putnum(u, FMT_HEX_LONG | FMT_ALTERNATE_FORM, l->arg);
    switch (l->cmd) {
        case 9:
        case 2:
            uart_puts(u, ", RESP: ");
            for (i = 0; i < 4; i++) {
                uart_putnum(u, FMT_HEX_CONSTANT, l->resp[i]);
                uart_puts(u, ", ");
            }
            break;
        case 0:
            uart_puts(u, ", ");
            break;
        default:
            uart_puts(u, ", RESP: ");
            uart_putnum(u, FMT_HEX_CONSTANT, l->resp[0]);
            uart_puts(u, ", ");
        }
    uart_puts(u, "Error: '");
    uart_puts(u, sdio_errmsg(l->err));
    uart_puts(u, "'\n");
}
#endif

/* 
 * sdio_bit_slice - helper function
 *
 * A number of the things the SDIO returns are in bit
 * fields. This code is designed to slice out a range
 * of bits and return them as a value (up to 32 bits
 * worth).
 */
uint32_t
sdio_bit_slice(uint32_t a[], int bits, int msb, int lsb) {
    uint32_t t;
    int i;

    if (((msb >= bits) || (msb < 0)) ||
        (lsb > msb) ||
        ((lsb < 0) || (lsb >= bits))) {
        DEBUG("Bad Slice values.\n");
        return 0;
    }
    t = 0;
    for (i = msb; i > lsb; i--) {
        t |= (a[((bits-1) - i)/32] >> (i % 32)) & 0x1;
        t <<= 1;
    }
    t |= (a[((bits-1) - lsb)/32] >> (lsb % 32)) & 0x1;
    return t;
}

/*
 * A convienence define. These are the flags we care about when
 * sending a command. During command processing SDIO_STA_CMDACT
 * will be set.
 */
#define COMMAND_FLAGS   (SDIO_STA_CMDSENT |\
                         SDIO_STA_CCRCFAIL |\
                         SDIO_STA_CMDREND |\
                         SDIO_STA_CTIMEOUT)

/*
 * Send a command over the SDIO bus.
 * Passed a command (8 bit value) and an argument (32 bit value)
 * This command figures out if the command will return a short (32 bit)
 * or long (64 bit) response. It is up to the calling program to pull
 * data from the long response commands.
 * Passed:
 *          cmd - Command to execute
 *          arg - Argument to pass to the command
 *          buf - pointer to a long aligned buffer if data
 *          len - expected length of buffer (in bytes)
 */
int
sdio_command(uint32_t cmd, uint32_t arg)
{
    uint32_t    tmp_val;
    int         error = 0;

    tmp_val = SDIO_CMD & ~0x7ff;            // Read pre-existing state
    tmp_val |= (cmd & SDIO_CMD_CMDINDEX_MASK);   // Put the Command in
    tmp_val |= SDIO_CMD_CPSMEN;                 // We'll be running CPSM

    switch(cmd) {
        case 0:
            tmp_val |= SDIO_CMD_WAITRESP_NO_0;
            break;
        case 2:
        case 9:
            tmp_val |= SDIO_CMD_WAITRESP_LONG;
            break;
        default:
            tmp_val |= SDIO_CMD_WAITRESP_SHORT; // the common case
            break;
    }
    /* If a data transaction is in progress, wait for it to finish */
#if 0
    if ((cmd != 12) && (SDIO_STA & (SDIO_STA_RXACT | SDIO_STA_TXACT))) {
        // XXX: This should be an error, we don't have multithread
        tmp_val |= SDIO_CMD_WAITPEND;
    }
#endif

    /*
     * EXECUTE:
     *    o Reset all status bits
     *    o Put ARG into SDIO ARG
     *    o reset the error indicator
     *    o Enable all interrupts.
     *    o Do the command
     */
    SDIO_ICR = 0x7ff;           // Reset everything that isn't bolted down.
    SDIO_ARG = arg;
    SDIO_CMD = tmp_val;
    /*
     * In a polled mode we should be able to just read the status bits
     * directly.
     */
    tmp_val = 0;
    do {
        tmp_val |= (SDIO_STA & 0x7ff);
    } while ((SDIO_STA & SDIO_STA_CMDACT) || (! tmp_val));;
    SDIO_ICR = tmp_val;

    /*
     * Compute the error here. Which can be one of
     * -- Success (either CMDSENT or CMDREND depending on response)
     * -- Timeout (based on CTIMEOUT)
     * -- No Response (based on no response in the time alloted)
     * -- CRC Error (based on CCRCFAIL)
     */
    if (! tmp_val) {
        error = SDIO_ENORESP;
    } else if (tmp_val & SDIO_STA_CCRCFAIL) {
        error = SDIO_ECCRCFAIL;
    } else if (tmp_val & (SDIO_STA_CMDREND | SDIO_STA_CMDSENT)) {
        error = SDIO_ESUCCESS;
    } else if (tmp_val & SDIO_STA_CTIMEOUT) {
        error = SDIO_ECTIMEOUT;
    } else {
        error = SDIO_EUNKNOWN;
    }

#ifdef SDIO_LOGGING
    // Note the result in our short log
    sdio_log(cmd, arg, error);
#endif
    return error;
}


/* our static data buffer we use for data movement commands */
static uint32_t data_buf[129];
static int data_len;

/*
 * Helper function - sdio_select
 *
 * This function "selects" a card using CMD7, note that if
 * you select card 0 that deselects the card (RCA is not allowed
 * to be 0)
 */
static int
sdio_select(int rca) {
    int err=0;

    if( rca != selected_rca ) {
        err = sdio_command(7, rca << 16);
        selected_rca = rca;
        if ((rca == 0) && (err == SDIO_ECTIMEOUT)) {
            return 0;   // "cheat" a timeout selecting 0 is a successful deselect
        }
    }
    return err;
}

/*
 * Helper function - sdio_scr
 *
 * Unlike the CID and CSD functions this function transfers data
 * so it needs to use the DPSM.
 *
 * Note that data over the wire is byte swapped so we swap it back
 * to "fix" it.
 *
 * Note when this return 0 the first two longs in the data_buf are
 * the SCR register.
 */

static int
sdio_scr(SDIO_CARD c) {
    int err;
    uint32_t    tmp_reg;
    int ndx;

    /* Select the card */
    err = sdio_select(c->rca);
    if (! err) {
        /* Set the Block Size */
        err = sdio_command(16, 8);
        if (! err) {
            /* APPCMD (our RCA) */
            err = sdio_command(55, c->rca << 16);
            if (! err) {
                SDIO_DTIMER = 0xffffffff;
                SDIO_DLEN = 8;
                SDIO_DCTRL = SDIO_DCTRL_DBLOCKSIZE_3 |
                             SDIO_DCTRL_DTDIR |
                             SDIO_DCTRL_DTEN;
                /* ACMD51 - Send SCR */
                err = sdio_command(51, 0);
                if (! err) {
                    data_len = 0;
                    do {
                        tmp_reg = SDIO_STA;
                        if (tmp_reg & SDIO_STA_RXDAVL) {
                            data_buf[data_len++] = SDIO_FIFO;
                        }
                    } while (tmp_reg & SDIO_STA_RXACT);
                    if ((tmp_reg & SDIO_STA_DBCKEND) == 0) {
                        if (tmp_reg & SDIO_STA_DCRCFAIL) {
                            err = SDIO_EDCRCFAIL;
                        } else if (tmp_reg & SDIO_STA_RXOVERR) {
                            err = SDIO_ERXOVERR;
                        } else {
                            err = SDIO_EUNKNOWN; // XXX: unknown error
                        }
                    }
                    if (! err) {
                        for (ndx = 0; ndx < 2; ndx++) {
                            byte_swap(data_buf[ndx]);
                            c->scr[ndx] = data_buf[ndx];
                        }
                    }
                }
            }
        }
    }
    (void) sdio_select(0);
    return err;
}

/*
 * Read a Block from our Card
 *
 * NB: There is a possibly useless test in this code, during the read
 * phase it allows that the SDIO card might try to send more than 512
 * bytes (128 32 bit longs) and allows it to do so, constantly over
 * writing the last long in the just-in-case-over-long-by-1 data buffer.
 * To compromise the system you would need a borked or custom crafted
 * sdio card which did that.
 */
int
sdio_readblock(SDIO_CARD c, uint32_t lba, uint8_t *buf) {
    int err;
    uint32_t tmp_reg;
    uint32_t addr = lba;
    uint8_t *t;
    int ndx;

    if (! SDIO_CARD_CCS(c)) {
        addr = lba * 512; // non HC cards use byte address
    }
    err = sdio_select(c->rca);
    if (! err) {
        err = sdio_command(16, 512);
        if (!err) {
            SDIO_DTIMER = 0xffffffff;
            SDIO_DLEN = 512;
            SDIO_DCTRL = SDIO_DCTRL_DBLOCKSIZE_9 |
                         SDIO_DCTRL_DTDIR |
                         SDIO_DCTRL_DTEN;
            err = sdio_command(17, addr);
            if (! err) {
                data_len = 0;
                do {
                    tmp_reg = SDIO_STA;
                    if (tmp_reg & SDIO_STA_RXDAVL) {
                        data_buf[data_len] = SDIO_FIFO;
                        if (data_len < 128) {
                            ++data_len;
                        }
                    }
                } while (tmp_reg & SDIO_STA_RXACT);
                if ((tmp_reg & SDIO_STA_DBCKEND) == 0) {
                    if (tmp_reg & SDIO_STA_DCRCFAIL) {
                        err = SDIO_EDCRCFAIL;
                    } else if (tmp_reg & SDIO_STA_RXOVERR) {
                        err = SDIO_ERXOVERR;
                    } else {
                        err = SDIO_EUNKNOWN; // Unknown Error!
                    }
                } else {
                    /* Data received, byte swap and put in user
                     * supplied buffer.
                     */
#if 0
                    for (ndx = 0; ndx < data_len; ndx++) {
                        byte_swap(data_buf[ndx]);
                    }
#endif
                    t = (uint8_t *)(data_buf);
                    /* copy out to the user buffer */
                    for (ndx = 0; ndx < 512; ndx ++) {
                        *buf = *t;
                        buf++;
                        t++;
                    }
                }
            }
        }
    }
    // deselect the card
    (void) sdio_select(0);
    return err;
}

/*
 * Write a Block from our Card
 */
int
sdio_writeblock(SDIO_CARD c, uint32_t lba, uint8_t *buf) {
    int err;
    uint32_t tmp_reg;
    uint32_t addr = lba;
    uint8_t *t;
    int ndx;

    if (! SDIO_CARD_CCS(c)) {
        addr = lba * 512; // non HC cards use byte address
    }
    
    /*
     * Copy buffer to our word aligned buffer. Nominally you
     * can just use the passed in buffer and cast it to a
     * uint32_t * but that can cause issues if it isn't 
     * aligned.
     */
    t = (uint8_t *)(data_buf);
    for (ndx = 0; ndx < 512; ndx ++) {
        *t = *buf;
        buf++;
        t++;
    }
    err = sdio_select(c->rca);
    if (! err) {
        /* Set Block Size to 512 */
        err = sdio_command(16, 512);
        if (!err) {
            SDIO_DTIMER = 0xffffffff;
            SDIO_DLEN = 512;
            SDIO_DCTRL = SDIO_DCTRL_DBLOCKSIZE_9 |
                         SDIO_DCTRL_DTEN;
            err = sdio_command(24, addr);
            if (! err) {
                data_len = 0;
                portDISABLE_INTERRUPTS();
                while(data_len < 128)
                    if (SDIO_STA & SDIO_STA_TXFIFOF)
                        portENABLE_INTERRUPTS();
                    else
                    {
                        portDISABLE_INTERRUPTS();
                        SDIO_FIFO = data_buf[data_len++];
                    }

                portENABLE_INTERRUPTS();
                while (SDIO_STA & SDIO_STA_TXACT);
                tmp_reg = SDIO_STA;
                if ((tmp_reg & SDIO_STA_DBCKEND) == 0) {
                    if (tmp_reg & SDIO_STA_DCRCFAIL) {
                        err = SDIO_EDCRCFAIL;
                    } else if (tmp_reg & SDIO_STA_TXUNDERR) {
                        err = SDIO_ETXUNDER;
                    } else {
                        err = SDIO_EUNKNOWN; // Unknown Error!
                    }
                }
            }
        }
    }
    // deselect the card
    (void) sdio_select(0);
    return err;
}

/*
 * sdio-status - Get Card Status page
 *
 * This function fetches the SD Card Status page and
 * copies it into the CARD structure.
 */
int
sdio_status(SDIO_CARD c) {
    uint32_t tmp_reg;
    int ndx;
    int err;

    err = sdio_select(c->rca);
    if (! err) {
        err = sdio_command(16, 64);
        if (! err) {
            err = sdio_command(55, c->rca << 16);
            if (! err) {
                SDIO_DTIMER = 0xffffffff;
                SDIO_DLEN = 64;
                SDIO_DCTRL = SDIO_DCTRL_DBLOCKSIZE_6 |
                             SDIO_DCTRL_DTDIR |
                             SDIO_DCTRL_DTEN;
                /* ACMD13 - Send Status Reg */
                err = sdio_command(13, 0);
                if (! err) {
                    data_len = 0;
                    do {
                        tmp_reg = SDIO_STA;
                        if (tmp_reg & SDIO_STA_RXDAVL) {
                            data_buf[data_len] = SDIO_FIFO;
                            if (data_len < 128) {
                                ++data_len;
                            }
                        }
                    } while (tmp_reg & SDIO_STA_RXACT);
                    if ((tmp_reg & SDIO_STA_DBCKEND) == 0) {
                        if (tmp_reg & SDIO_STA_DCRCFAIL) {
                            err = SDIO_EDCRCFAIL;
                        } else if (tmp_reg & SDIO_STA_RXOVERR) {
                            err = SDIO_ERXOVERR;
                        } else {
                            err = SDIO_EUNKNOWN; // Unknown Error!
                        }
                    } else {
                        for (ndx = 0; ndx < 16; ndx++) {
                            byte_swap(data_buf[ndx]);
                            c->status[ndx] = data_buf[ndx];
                        }
                    }
                    (void) sdio_select(0);
                }
            }
        }
    }
    return err;
}

static struct SDIO_CARD_DATA __sdio_card_data;
#define MAX_RETRIES 5

/*
 * sdio_open - Prepare to use SDIO card
 *
 * This function resets the SDIO bus and identifies the
 * card (if any) that is plugged in. If there is no card
 * present, or an error in figuring out what the card is
 * (for example its an old MMC card) the function returns
 * NULL. If it fails and you have logging enabled you can
 * look at the last few commands sent.
 */
SDIO_CARD
sdio_open(void) {
    int err;
    int i;
    uint8_t *t;
    uint32_t tmp_reg;
    SDIO_CARD res = &__sdio_card_data;

    // basically bset(0, __sdio_card_data)
    t = (uint8_t *) &__sdio_card_data;
    for (i = 0; i < (int) sizeof(__sdio_card_data); i++) {
        *t++ = 0;
    }
    sdio_reset(SDIO_POWER_ON);
    err = sdio_command(0, 0);
    if (!err) {
        err = sdio_command(8, 0x1aa);
        if (!err) {
            // Woot! We support CMD8 so we're a v2 card at least */
            tmp_reg = SDIO_RESP1;
            __sdio_card_data.props = 1;
            i = 0;
            err = sdio_command(5, 0);
            if (! err) {
                // It is an SDIO card which is unsupported!
                err = SDIO_EBADCARD;
                return NULL;
            }
            do {
                err = sdio_command(55, 0); // broadcast ACMD
                if (err) {
                    break;   // try again
                }
                // Testing Card Busy, Voltage match, and capacity
                err = sdio_command(41, 0xc0100000);
                if (err != -2) {            // Expect CCRCFAIL here
                    break;               // try again
                }
                tmp_reg = SDIO_RESP1; // what did the card send?
                if ((tmp_reg & 0x80000000) == 0) {
                    continue;               // still powering up
                }
                res->ocr = tmp_reg;           // Ok OCR is valid
                break;
            } while (++i < MAX_RETRIES);
            if (res->ocr) {
                err = sdio_command(2, 0);
                if (! err) {
                    res->cid[0] = SDIO_RESP1;
                    res->cid[1] = SDIO_RESP2;
                    res->cid[2] = SDIO_RESP3;
                    res->cid[3] = SDIO_RESP4;
                    err = sdio_command(3, 0);   // get the RCA
                    if (! err) {
                        tmp_reg = SDIO_RESP1;
                        res->rca = (tmp_reg >> 16) & 0xffff;
                        if (! res->rca) {
                            /*
                             * If the card says '0' tell it to pick
                             * we assume this will work because the
                             * previous send RCA worked and the card
                             * should be in the ident state if it is
                             * functioning correctly.
                             */
                            (void) sdio_command(3, 0); // try again
                            tmp_reg = SDIO_RESP1;
                            res->rca = (tmp_reg >> 16) & 0xffff;
                        }
                        err = sdio_command(9, res->rca << 16);
                        if (! err) {
                            res->csd[0] = SDIO_RESP1;
                            res->csd[1] = SDIO_RESP2;
                            res->csd[2] = SDIO_RESP3;
                            res->csd[3] = SDIO_RESP4;
                            err = sdio_scr(res); // Capture the SCR
                            if (! err) {
                                /* All SD Cards support 4 bit bus and 24Mhz */
                                err = sdio_select(res->rca);
                                if (! err) {
                                    err = sdio_command(55, res->rca << 16);
                                    if (! err) {
                                        err = sdio_command(6, 2);
                                        if (! err) {
                                            sdio_bus(4, SDIO_24MHZ);
                                            (void) sdio_select(0);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    /* Compute the size of the card based on fields in the CSD
     * block. There are two kinds, V1 or V2. 
     * In the V1 Case :
     *     Size = 1<<BLOCK_LEN * 1<<(MULT+2) * (C_SIZE+1) bytes.
     * In the V2 Case :
     *     Size = (C_SIZE + 1) * 512K bytes.
     * But for our structure we want the size in 512 byte "blocks"
     * since that is the addressing unit we're going to export so
     * we compute the size / 512 as the "size" for the structure.
     */

    if (! err) {
        res->size = 0;
        switch (SDIO_CSD_VERSION(res)) {
            case 0:
                tmp_reg = ((1 << (SDIO_CSD1_CSIZE_MULT(res) + 2)) *
                           ( 1 << SDIO_CSD1_RBLKLEN(res))) >> 9;
                res->size = tmp_reg * (SDIO_CSD1_CSIZE(res) + 1);
                break;
            case 1:
                res->size = (SDIO_CSD2_CSIZE(res)+1) << 10; 
                break;
            default:
                res->size = 0; // Bug if its not CSD V1 or V2
        }
    }
    return (err == 0) ? res : NULL;
}

void
sdio_print_log(int dev, int nrec) {
#ifdef SDIO_LOGGING
    int ent = current_sdio_log_entry;   // global for the log
    int i;
    int delta = 0;
    for (i = 0; i < nrec; i++, delta--) {
        ent = (ent) ? ent - 1 : SDIO_LOGSIZE - 1;
        text_color(dev, YELLOW);
        uart_putnum(dev, FMT_BASE_10 | FMT_SIGNED, delta);
        uart_puts(dev, ": ");
        text_color(dev, DEFAULT);
        sdio_dump_log_entry(dev, &sdio_log_buffer[ent]);
    }
#else
    uart_puts(dev, "Sorry Logging was not compiled in.\n");
    return;
#endif
}

/*
 * Read a Block from our card using interrupts
 *
 */
int
sdio_readblock_dma(SDIO_CARD c, uint32_t lba, uint8_t *buf, uint8_t wait) {
    int err;
    uint32_t addr = lba;

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY );
    while( SDIO_STA & SDIO_STA_RXACT);

    if (! SDIO_CARD_CCS(c)) {
        addr = lba * 512; // non HC cards use byte address
    }
    if( (err = sdio_select(c->rca)) )
        return err;
    if( (err = sdio_command(16, 512)) )
        return err;

    dma_clear_interrupt_flags(DMA2, DMA_STREAM3, DMA_TCIF | DMA_HTIF | DMA_TEIF | DMA_DMEIF | DMA_FEIF);
    dma_set_memory_address(DMA2, DMA_STREAM3, (uint32_t) buf);
    dma_set_transfer_mode(DMA2, DMA_STREAM3, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
    dma_enable_stream(DMA2, DMA_STREAM3);

    SDIO_DTIMER = 0xffffffff;
    SDIO_DLEN = 512;
    SDIO_MASK |= SDIO_MASK_DBCKENDIE | SDIO_MASK_DATAENDIE | SDIO_MASK_RXOVERRIE | SDIO_MASK_DCRCFAILIE;
    SDIO_DCTRL = SDIO_DCTRL_DBLOCKSIZE_9 | SDIO_DCTRL_DTDIR | SDIO_DCTRL_DMAEN | SDIO_DCTRL_DTEN;

    if( (err = sdio_command(17, addr)) )
        return err;
    
    if( wait )
        return sdio_wait_for_completion();
    return err;
}

/*
 * Write a Block to our card using interrupts
 */
int
sdio_writeblock_dma(SDIO_CARD c, uint32_t lba, uint8_t *buf, uint8_t wait) {
    int err;
    uint32_t addr = lba;

    if (! SDIO_CARD_CCS(c)) {
        addr = lba * 512; // non HC cards use byte address
    }
    
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY );
    while( SDIO_STA & SDIO_STA_TXACT);
    if( (err = sdio_select(c->rca)) )
        return err;
    /* Set Block Size to 512 */
    if( (err = sdio_command(16, 512)) )
        return err;

    dma_clear_interrupt_flags(DMA2, DMA_STREAM3, DMA_TCIF | DMA_HTIF | DMA_TEIF | DMA_DMEIF | DMA_FEIF);
    dma_set_memory_address(DMA2, DMA_STREAM3, (uint32_t) buf);
    dma_set_transfer_mode(DMA2, DMA_STREAM3, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);
    dma_enable_stream(DMA2, DMA_STREAM3);

    if( (err = sdio_command(24, addr)) )
        return err;

    SDIO_DTIMER = 0xffffffff;
    SDIO_DLEN = 512;
    SDIO_MASK |= SDIO_MASK_DBCKENDIE | SDIO_MASK_DATAENDIE | SDIO_MASK_TXUNDERRIE | SDIO_MASK_DCRCFAILIE;
    SDIO_DCTRL = SDIO_DCTRL_DBLOCKSIZE_9 | SDIO_DCTRL_DTEN | SDIO_DCTRL_DMAEN;

    if( wait )
        return sdio_wait_for_completion();

    return err;
}

int sdio_wait_for_completion() {
    int err=0;
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY );
    while( SDIO_STA & (SDIO_STA_TXACT|SDIO_STA_RXACT) );
    uint32_t tmp_reg = SDIO_STA;
    if (tmp_reg & SDIO_STA_DCRCFAIL) {
        err = SDIO_EDCRCFAIL;
    } else if (tmp_reg & SDIO_STA_TXUNDERR) {
        err = SDIO_ETXUNDER;
    } else if (tmp_reg & SDIO_STA_RXOVERR) {
        err = SDIO_ERXOVERR;
    } else if(tmp_reg) {
        err = SDIO_EUNKNOWN; // Unknown Error!
    }
    SDIO_ICR = 0x7ff;           // Reset everything that isn't bolted down.
    // deselect the card
    (void) sdio_select(0);
    xTaskNotifyGive( xTaskToNotify ); // SDIO is available
    return err;
}
