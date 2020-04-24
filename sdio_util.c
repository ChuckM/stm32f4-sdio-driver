/*
 * sdio.c - code to access the SD card on the Base Board
 *
 * In theory this is in two parts, the low level driver part and
 * the higher level "storage layer" part. Sort of like UNIX in that
 * way.
 */

#include <stdint.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/sdio.h>
#include <libopencm3/stm32/gpio.h>
#include "bb.h"
#include "debug.h"
#ifndef NULL
#define NULL (void *)(0)
#endif

#define USE_NEW_SDIO
/* conveniently swaps the bytes in a long around */
#define byte_swap(val) \
        asm("rev %[swap], %[swap]" : [swap] "=r" (val) : "0" (val));

#define DEBUG(x) debug_puts(x)

/* Prototypes */

/* These are functions to show status on the terminal */
void show_sdio_clock(int, int);
void show_sdio_status(int, int);
void show_sdio_sdstatus(int, int);
void debug_sdio_sdstatus(uint32_t buf[]);
void show_sdio_cid(int, int, uint32_t []);
void show_sdio_csd(int, int, uint32_t []);
void show_sdio_power(int row, int col);
void show_sdio_carddetect(int row, int col);
void show_sdiolog(int row, int col);
void show_sdio_scr(int row, int col, uint32_t scr);

/* these are the actually useful functions */
void debug_status(uint32_t);
static void en_dis(char *, uint32_t, char *, char *);
void sdio_log(uint8_t cmd, uint32_t arg, int err, const char *resp);
int sd_getsdstatus(int rca);

char debug_buf[64];

/*
 * Capabilities of the card we've detected
 */
struct sd_caps {
    int valid;      // Set 1 when structure is valid
    int f8_support; // Supports CMD8
    int sdio;       // Is SDIO Card
    int v2;         // V2.x
    int lv;         // Low Voltage (1.8V) card
    int ok_volts;   // Voltage compatible with host
    int hc;         // High capacity
    uint16_t rca;   // Relative Card Address
    uint32_t cid[4];    // CID values
    uint32_t csd[4];    // CSD values
    uint32_t size;      // Size in K Bytes of our SD Card
    uint32_t blocks;    // Size in blocks of our SD Card
    uint32_t scr;       // lower 32 of SCR
} card_caps;

struct form_data {
    int nflags;
    struct {
        int row_offset;
        int col_offset;
        char *label;
        uint32_t flag;
        char *affirmative;
        char *negative;
    } flags[];
};

const struct form_data clock = {
    5,
    {
    {3, 1, "Divider Bypass : ", SDIO_CLKCR_BYPASS, 0, 0},
    {4, 5, "SDIO Clock : ", SDIO_CLKCR_CLKEN, 0, 0},
    {5, 5, "Power Save : ", SDIO_CLKCR_PWRSAV, 0, 0},
    {6, 0, "HW Flow Control : ", SDIO_CLKCR_HWFC_EN, 0, 0},
    {7, 4, "Clock Phase : ", SDIO_CLKCR_NEGEDGE, "Falling Edge", "Rising Edge"}
    }
};


static void
en_dis(char *tag, uint32_t bits, char *en, char *ds) {
    uart_puts(console, tag);
    if (bits) {
        uart_puts(console, (en == 0) ? "ENABLED  " : en);
    } else {
        uart_puts(console, (ds == 0) ? "DISABLED" : ds);
    }
}


uint8_t blk_read_buf[512];


/*
 * show_sdio_clock - put up information about what the clock
 * register is configured to do.
 */
static char *bus_widths[4] = { "1 bit", "4 bit", "8 bit", "*BAD*" };

void
show_sdio_clock(int row, int col) {
    uint32_t reg = SDIO_CLKCR;
    int freq;
    int i;
    i = (reg & 0x7f); // should use the shifts
    freq = 48000000 / (i + 2);
    move_cursor(console, row, col);
    text_color(console, YELLOW);
    uart_puts(console, "SDIO Clock Settings");
    text_color(console, DEFAULT);
    move_cursor(console, row + 1, col + 6);
    uart_puts(console, "Bus Width : ");
    i = (reg & (0x3 << SDIO_CLKCR_WIDBUS_SHIFT)) >> SDIO_CLKCR_WIDBUS_SHIFT;
    uart_puts(console, bus_widths[i]);
    move_cursor(console, row + 2, col+1);
    uart_puts(console, "SDIO Frequency : ");
    if (freq > 1000000) {
        i = freq % 1000000;
        freq = freq / 1000000;
        uart_putnum(console, FMT_BASE_10, freq);
        uart_puts(console, "Mhz   ");
    } else {
        i = freq % 1000;
        freq = freq / 1000;
        uart_putnum(console, FMT_BASE_10, freq);
        uart_puts(console, "Khz   ");
    }
    for (i = 0; i < clock.nflags; i++) {
        move_cursor(console, row + clock.flags[i].row_offset,
                    col + clock.flags[i].col_offset);
        en_dis(clock.flags[i].label,
               (clock.flags[i].flag & reg) != 0,
                clock.flags[i].affirmative,
                clock.flags[i].negative);

    }
}


const struct {
    uint32_t bit;
    const char *msg;
} status_flags[] = {
    { SDIO_STA_CEATAEND, "CE-ATA Command Complete!" },
    { SDIO_STA_SDIOIT,   "SDIO Interrupt Received!" },
    { SDIO_STA_RXDAVL,   "Data Available in Rx FIFO!" },
    { SDIO_STA_TXDAVL,   "Data Available in Tx FIFO!" },
    { SDIO_STA_RXFIFOE,  "Rx FIFO Empty!" },
    { SDIO_STA_TXFIFOE,  "Tx FIFO Empty!" },
    { SDIO_STA_RXFIFOF,  "Rx FIFO Full!" },
    { SDIO_STA_TXFIFOF,  "Tx FIFO Full!" },
    { SDIO_STA_RXFIFOHF, "Rx FIFO Half Full!" },
    { SDIO_STA_TXFIFOHE, "Tx FIFO Half Empty!" },
    { SDIO_STA_RXACT,    "Rx in Progress!" },
    { SDIO_STA_TXACT,    "Tx in Progress!" },
    { SDIO_STA_CMDACT,   "Command Transfer in Progress!" },
    { SDIO_STA_DBCKEND,  "Data Block Sent/Recieved, CRC OK!" },
    { SDIO_STA_STBITERR, "Start bit not detected on data in wide bus!" },
    { SDIO_STA_DATAEND,  "Data end!" },
    { SDIO_STA_CMDSENT,  "Command Sent (no response req'd)!" },
    { SDIO_STA_CMDREND,  "Command Response Receved (CRC OK!)" },
    { SDIO_STA_RXOVERR,  "Rx FIFO Overrun error!" },
    { SDIO_STA_TXUNDERR, "Tx FIFO Underrun Error!" },
    { SDIO_STA_DTIMEOUT, "Data Timeout!" },
    { SDIO_STA_CTIMEOUT, "Command Response Timeout!" },
    { SDIO_STA_DCRCFAIL, "Data CRC Fail!" },
    { SDIO_STA_CCRCFAIL, "Command CRC Fail!" },
    { 0, 0 }
};



void
show_sdio_status(int row, int col) {
    uint32_t reg = SDIO_STA;
    int ndx = 0;
    int i = 0;
    move_cursor(console, row, col+2);
    text_color(console, YELLOW);
    uart_puts(console, "SDIO Status Register : ");
    text_color(console, DEFAULT);
    if (reg == 0) {
        uart_puts(console, "No Status Active!");
    } else {
        while (status_flags[ndx].msg != 0) {
            if (reg & status_flags[ndx].bit) {
                uart_puts(console, status_flags[ndx].msg);
                move_cursor(console, row+i, col+23);
                i++;
            }
            ndx++;
        }
    }
}

void
debug_status(uint32_t bits) {
    uint32_t reg = bits;
    int ndx = 0;
    DEBUG("\n    SDIO Status Register : ");
    ntoa(bits, FMT_BINARY_LONG, debug_buf);
    DEBUG(debug_buf);
    DEBUG("\n");
    if (reg == 0) {
        DEBUG("       - No Status Active!\n");
    } else {
        while (status_flags[ndx].msg != 0) {
            if (reg & status_flags[ndx].bit) {
                DEBUG("       - ");
                DEBUG(status_flags[ndx].msg);
                DEBUG("\n");
            }
            ndx++;
        }
    }
}

void show_sdio_response(int, int);

/* XXX the response registers should be an array */
void
show_sdio_response(int row, int col) {
    uint32_t resp[4];
    char buf[16];
    int i;
    resp[0] = SDIO_RESP1;
    resp[1] = SDIO_RESP2;
    resp[2] = SDIO_RESP3;
    resp[3] = SDIO_RESP4;
    move_cursor(console, row, col+9);
    text_color(console, YELLOW);
    uart_puts(console, "SDIO Response : ");
    text_color(console, DEFAULT);
    ntoa(resp[0], FMT_HEX_LONG | FMT_ALTERNATE_FORM, buf);
    uart_puts(console, buf);
    text_color(console, YELLOW);
    uart_puts(console, " <- [SHORT] ");
    text_color(console, DEFAULT);
    for (i = 1; i < 4; i++) {
        move_cursor(console, row + i, col + 25);
        ntoa(resp[i], FMT_HEX_LONG | FMT_ALTERNATE_FORM, buf);
        uart_puts(console, buf);
    }
}

/*
 * Error flags for RESP1 Register
 */

#define RESP1_ADDR_OUT_OF_RANGE        ((uint32_t)0x80000000)
#define RESP1_ADDR_MISALIGNED          ((uint32_t)0x40000000)
#define RESP1_BLOCK_LEN_ERR            ((uint32_t)0x20000000)
#define RESP1_ERASE_SEQ_ERR            ((uint32_t)0x10000000)
#define RESP1_BAD_ERASE_PARAM          ((uint32_t)0x08000000)
#define RESP1_WRITE_PROT_VIOLATION     ((uint32_t)0x04000000)
#define RESP1_LOCK_UNLOCK_FAILED       ((uint32_t)0x01000000)
#define RESP1_COM_CRC_FAILED           ((uint32_t)0x00800000)
#define RESP1_ILLEGAL_CMD              ((uint32_t)0x00400000)
#define RESP1_CARD_ECC_FAILED          ((uint32_t)0x00200000)
#define RESP1_CC_ERROR                 ((uint32_t)0x00100000)
#define RESP1_GENERAL_UNKNOWN_ERROR    ((uint32_t)0x00080000)
#define RESP1_STREAM_READ_UNDERRUN     ((uint32_t)0x00040000)
#define RESP1_STREAM_WRITE_OVERRUN     ((uint32_t)0x00020000)
#define RESP1_CID_CSD_OVERWRIETE       ((uint32_t)0x00010000)
#define RESP1_WP_ERASE_SKIP            ((uint32_t)0x00008000)
#define RESP1_CARD_ECC_DISABLED        ((uint32_t)0x00004000)
#define RESP1_ERASE_RESET              ((uint32_t)0x00002000)
#define RESP1_AKE_SEQ_ERROR            ((uint32_t)0x00000008)
#define RESP1_ALL_ERRORS               ((uint32_t)0xFDFFE008)

#if 0

static const struct {
    uint32_t bit;   // error bit in R1 Register
    char    *msg;   // Text version of error
} r1_errors[] = {
        { RESP1_ADDR_OUT_OF_RANGE, "Address out of range" },
        { RESP1_ADDR_MISALIGNED, "Address misaligned" },
        { RESP1_BLOCK_LEN_ERR, "Block length error" },
        { RESP1_ERASE_SEQ_ERR, "Erase sequence error" },
        { RESP1_BAD_ERASE_PARAM, "Erase parameter error" },
        { RESP1_WRITE_PROT_VIOLATION, "Write protect violation" },
        { RESP1_LOCK_UNLOCK_FAILED, "Lock / Unlock failure" },
        { RESP1_COM_CRC_FAILED, "Com CRC failed" },
        { RESP1_ILLEGAL_CMD, "Illegal command" },
        { RESP1_CARD_ECC_FAILED, "Card ECC failure" },
        { RESP1_CC_ERROR, "CC error" },
        { RESP1_GENERAL_UNKNOWN_ERROR, "General (unknown) error" },
        { RESP1_STREAM_READ_UNDERRUN, "Stream read underrun" },
        { RESP1_STREAM_WRITE_OVERRUN, "Stream write overrun" },
        { RESP1_CID_CSD_OVERWRIETE, "CID / CSD Overwrite" },
        { RESP1_WP_ERASE_SKIP, "Write protect Erase skipped" },
        { RESP1_CARD_ECC_DISABLED, "Card ECC disabled" },
        { RESP1_ERASE_RESET, "Erase reset" },
        { RESP1_AKE_SEQ_ERROR, "AKE sequence error" },
        { 0, 0},   // must be last
};


/*
 * Return an appropriate error message based on an R1 response
 * Passed the response code
 * Returns const char * to error message
 */
const char *
sdio_resp_error_msg(uint8_t cmd, uint32_t resp) {
    int i;

    // Command ended successfully, check the response
    if ((resp & 0xff) != cmd) {
        return sdio_error_strings[0];   // Success
    }

    for (i = 0; r1_errors[i].bit != 0; i++) {
        if (resp & r1_errors[i].bit) {
            return r1_errors[i].msg;
        }
    }
    return sdio_error_strings[5]; // Did not recognize the error bit
}
#endif

void
show_sdio_power(int row, int col) {
    move_cursor(console, row, col+2);
    text_color(console, YELLOW);
    uart_puts(console, "SDIO Power: ");
    switch ( SDIO_POWER & 0x3 ) {
        case 0:
            text_color(console, RED);
            uart_puts(console, "OFF");
            text_color(console, DEFAULT);
            break;
        case 1:
            uart_puts(console, "RSV1");
            break;
        case 2:
            uart_puts(console, "RSV2");
            break;
        case 3:
            text_color(console, GREEN);
            uart_puts(console, "ON");
            text_color(console, DEFAULT);
            break;
    }
}

void
show_sdio_carddetect(int row, int col) {
    move_cursor(console, row, col);
    text_color(console, YELLOW);
    uart_puts(console, "Card Present: ");
    text_color(console, DEFAULT);
    if (gpio_get(GPIOB, GPIO15)) {
        text_color(console, RED);
        uart_puts(console, "NOT DETECTED");
    } else {
        uart_puts(console, "Detected");
    }
    text_color(console, DEFAULT);
}

uint8_t blk_write_buf[512];

SDIO_CARD my_card;


/*
 * SDIO Explorer
 *
 * So this becomes a simple way to 'play around' with the SDIO
 * registers in the chip to see what they do.
 */
void
sdio_explorer(void) {
    int err, i;
    char c;
    uint8_t *ss, *ds;
    uint32_t blk;
    uint8_t *addr;

    clear_screen(console);
    my_card = sdio_open();
    DEBUG("Last 10 SDIO commands:\n");
    sdio_print_log(debug_console, 10);
    if (my_card == NULL) {
        DEBUG("Open failed!\n");
        show_sdio_power(1, 40);
        show_sdio_clock(1, 1);
        show_sdio_carddetect(2, 40);
        debug_wait();
        return;
    }
    DEBUG("Card size is : ");
    uart_putnum(debug_console, FMT_BASE_10, my_card->size);
    DEBUG(" blocks. (");
    uart_putnum(debug_console, FMT_BASE_10, my_card->size >> 1);
    DEBUG("K bytes)\n");
    DEBUG("Reading a block from the card\n");
    err = sdio_readblock(my_card, 0, blk_read_buf);
    if (! err) {
        DEBUG("Success!\n");
        addr = dump_page(debug_console, blk_read_buf, blk_read_buf);
        dump_page(debug_console, addr, blk_read_buf);
    }
    err = sdio_status(my_card);
    if (err) {
        DEBUG("Error reading SD Card Status : ");
        DEBUG(sdio_errmsg(err));
        DEBUG("\n");
    } else {
        debug_sdio_sdstatus(my_card->status);
    }

    show_sdio_power(1, 40);
    show_sdio_carddetect(2, 40);
    show_sdio_clock(1, 1);
#if 0
    show_sdio_csd(17, 1, my_card->csd);
    show_sdio_cid(3, 32, my_card->cid);
    show_sdio_scr(32, 42, my_card->scr[0]);
#endif

    move_cursor(console, 10, 1);
    text_color(console, YELLOW);
    uart_puts(console, "Card Size: ");
    text_color(console, GREEN);
    uart_putnum(console, FMT_BASE_10, my_card->size);
    uart_puts(console, " blocks.");
    while (1) {
        move_cursor(console, 11, 1);
        uart_puts(console, "[R]ead, [W]rite, or e[X]it:        ");
        move_cursor(console, 11, 29);
        c = uart_getc(console, 1);
        if ((c == 'x') || (c == 'X')) {
            uart_puts(console, "Exit");
            return;
        }
        if ((c == 'r') || (c == 'R')) {
            uart_puts(console, "Read");
            move_cursor(console, 12, 1);
            uart_puts(console, "Enter Block # :               ");
            move_cursor(console, 12, 17);
            blk = uart_getnum(console);
            if (blk >= my_card->size) {
                move_cursor(console, 13, 1);
                uart_puts(console, "Block number out of range!");
                continue;
            }
            err = sdio_readblock(my_card, blk, blk_read_buf);
            move_cursor(console, 13, 1);
            uart_puts(console, "Result : ");
            uart_puts(console, sdio_errmsg(err));
            move_cursor(console, 15, 1);
            addr = dump_page(console, blk_read_buf, blk_read_buf);
            dump_page(console, addr, blk_read_buf);
        } else if ((c == 'w') || (c == 'W')) {
            uart_puts(console, "Write");
            move_cursor(console, 12, 1);
            uart_puts(console, "Enter Block # :               ");
            move_cursor(console, 12, 17);
            blk = uart_getnum(console);
            for (i = 0; i < 512; i++) {
                *(blk_read_buf+i) = i & 0xff;
            }
            ss = (uint8_t *) "This is Block # ";
            ds = blk_read_buf;
            while (*ss != '\000') {
                *ds = *ss;
                ds++; ss++;
            }
            ntoa(blk, FMT_BASE_10, (char *) ds);
            err = sdio_writeblock(my_card, blk, blk_read_buf);
            move_cursor(console, 13, 1);
            uart_puts(console, "Result : ");
            uart_puts(console, sdio_errmsg(err));
        }
    }
}

/*
 * Put up the SD Card's CID data
 */
void
show_sdio_cid(int row, int col, uint32_t cid[]) {
    int i;
    char c;
    uint32_t t;
    char buf[10];

    move_cursor(console, row, col);
    text_color(console, YELLOW);
    uart_puts(console, "SD Card CID Data:");
    col += 2;
    move_cursor(console, row+1, col+6);
    uart_puts(console, "Manufacturer: ");
    text_color(console, DEFAULT);
    i = (cid[0] >> 24) & 0xff;
    uart_putnum(console, FMT_BASE_10, i);
    move_cursor(console, row+2, col);
    text_color(console, YELLOW);
    uart_puts(console, "OEM/Application ID: ");
    text_color(console, DEFAULT);
    c = (char) ((cid[0] >> 16) & 0xff);
    uart_putc(console, c);
    c = (char) ((cid[0] >> 8) & 0xff);
    uart_putc(console, c);
    text_color(console, YELLOW);
    move_cursor(console, row+3, col+6);
    uart_puts(console, "Product Name: ");
    text_color(console, DEFAULT);
    uart_putc(console, (char) (cid[0] & 0xff));
    for (i = 0; i < 4; i++) {
        uart_putc(console, (char)((cid[1] >> 8 * (3-i)) & 0xff));
    }
    text_color(console, YELLOW);
    move_cursor(console, row+4, col+10);
    uart_puts(console, "Revision: ");
    text_color(console, DEFAULT);
    c = (char)((cid[2] >> 28) & 0xf) + '0';
    uart_putc(console, c);
    uart_putc(console, '.');
    c = (char)((cid[2] >> 24) & 0xf) + '0';
    uart_putc(console, c);
    move_cursor(console, row+5, col+15);
    text_color(console, YELLOW);
    uart_puts(console, "S/N: ");
    text_color(console, DEFAULT);
    t = ((cid[2] << 8) & 0xffffff00) | (cid[3] >> 24 & 0xff);
    ntoa(t, FMT_HEX_LONG, buf);
    uart_puts(console, buf);
    text_color(console, YELLOW);
    move_cursor(console, row+6, col+6);
    uart_puts(console, "Manufactured: ");
    text_color(console, DEFAULT);
    i = (cid[3] >> 8) & 0xf;
    uart_putnum(console, FMT_BASE_10, i);
    uart_puts(console, "/");
    i = ((cid[3] >> 12) & 0xff) + 2000;
    uart_putnum(console, FMT_BASE_10, i);
}

static char *__csd_struct(int n) {
    switch (n) {
        case 0:
            return "CSD Version 1.0";
        case 1:
            return "CSD Version 2.0";
        default:
            break;
    }
    return "Unknown";
}

static char *taac_time_constants[16][8] =  {
    { },
    {   "1ns", "10ns", "100ns",   "1us", "10us", "100us",   "1ms", "10ms" },
    { "1.2ns", "12ns", "120ns", "1.2us", "12us", "120us", "1.2ms", "12ms" },
    { "1.3ns", "13ns", "130ns", "1.3us", "13us", "130us", "1.3ms", "13ms" },
    { "1.5ns", "15ns", "150ns", "1.5us", "15us", "150us", "1.5ms", "15ms" },
    {   "2ns", "20ns", "200ns",   "2us", "20us", "200us",   "2ms", "20ms" },
    { "2.5ns", "25ns", "250ns", "2.5us", "25us", "250us", "2.5ms", "25ms" },
    {   "3ns", "30ns", "300ns",   "3us", "30us", "300us",   "3ms", "30ms" },
    { "3.5ns", "35ns", "350ns", "3.5us", "35us", "350us", "3.5ms", "35ms" },
    {   "4ns", "40ns", "400ns",   "4us", "40us", "400us",   "4ms", "40ms" },
    { "4.5ns", "45ns", "450ns", "4.5us", "45us", "450us", "4.5ms", "45ms" },
    {   "5ns", "50ns", "500ns",   "5us", "50us", "500us",   "5ms", "50ms" },
    { "5.5ns", "55ns", "550ns", "5.5us", "55us", "550us", "5.5ms", "55ms" },
    {   "6ns", "60ns", "600ns",   "6us", "60us", "600us",   "6ms", "60ms" },
    {   "7ns", "70ns", "700ns",   "7us", "70us", "700us",   "7ms", "70ms" },
    {   "8ns", "80ns", "800ns",   "8us", "80us", "800us",   "8ms", "80ms" }
};
static char *__taac_val(int n) {
        int r, c;
        r = n & 0x3;
        c = (n >> 3) & 0xf;
        return taac_time_constants[c][r];
}

static char *tran_time_constants[16][8] =  {
    { },
    { "100kbps", "1Mbps", "10Mbps", "100Mbps", NULL, NULL, NULL, NULL },
    { "120kbps", "1.2Mbps", "12Mbps", "120Mbps", NULL, NULL, NULL, NULL },
    { "130kbps", "1.3Mbps", "13Mbps", "130Mbps", NULL, NULL, NULL, NULL },
    { "150kbps", "1.5Mbps", "15Mbps", "150Mbps", NULL, NULL, NULL, NULL },
    { "200kbps", "2Mbps", "20Mbps", "200Mbps", NULL, NULL, NULL, NULL },
    { "250kbps", "2.5Mbps", "25Mbps", "250Mbps", NULL, NULL, NULL, NULL },
    { "300kbps", "3Mbps", "30Mbps", "300Mbps", NULL, NULL, NULL, NULL },
    { "350kbps", "3.5Mbps", "35Mbps", "350Mbps", NULL, NULL, NULL, NULL },
    { "400kbps", "4Mbps", "40Mbps", "400Mbps", NULL, NULL, NULL, NULL },
    { "450kbps", "4.5Mbps", "45Mbps", "450Mbps", NULL, NULL, NULL, NULL },
    { "500kbps", "5Mbps", "50Mbps", "500Mbps", NULL, NULL, NULL, NULL },
    { "550kbps", "5.5Mbps", "55Mbps", "550Mbps", NULL, NULL, NULL, NULL },
    { "600kbps", "6Mbps", "60Mbps", "600Mbps", NULL, NULL, NULL, NULL },
    { "700kbps", "7Mbps", "70Mbps", "700Mbps", NULL, NULL, NULL, NULL },
    { "800kbps", "8Mbps", "80Mbps", "800Mbps", NULL, NULL, NULL, NULL },
};
static char *__tran_val(int n) {
        int r, c;
        r = n & 0x3;
        c = (n >> 3) & 0xf;
        return tran_time_constants[c][r];
}
static char *__dec_val(int n) {
    static char buf[40];
    ntoa(n, FMT_BASE_10, buf);
    return buf;
}

static char *__hex_val(int n) {
    static char buf[40];
    ntoa(n, FMT_BASE_16 | FMT_ALTERNATE_FORM, buf);
    return buf;
}

static char *__bin_val(int n) {
    static char buf[40];
    ntoa(n, FMT_BASE_2 | FMT_ALTERNATE_FORM, buf);
    return buf;
}

static char *__blk_len(int n) {
    static char buf[40];
    int i;
    i = 1 << n;
    ntoa(i, FMT_BASE_10, buf);
    return buf;
}

static char *__sm_val(int n) {
    static char buf[40];
    int i;
    i = 1 << (n+2);
    ntoa(i, FMT_BASE_10, buf);
    return buf;
}

static char *__yn_val(int n) {
    return (n) ? "Yes" : "No";
}

static char *__en_val(int n) {
    return (n) ? "Enabled" : "Disabled";
}

static char *__curr_min_val(int n) {
    switch (n & 0x7) {
        case 0:
            return "0.5mA";
        case 1:
            return "1mA";
        case 2:
            return "5mA";
        case 3:
            return "10mA";
        case 4:
            return "25mA";
        case 5:
            return "35mA";
        case 6:
            return "60mA";
        case 7:
            return "100mA";
    }
    return "";
}

static char *__curr_max_val(int n) {
    switch (n & 0x7) {
        case 0:
            return "1mA";
        case 1:
            return "5mA";
        case 2:
            return "10mA";
        case 3:
            return "25mA";
        case 4:
            return "35mA";
        case 5:
            return "45mA";
        case 6:
            return "80mA";
        case 7:
            return "200mA";
    }
    return "";
}

struct {
    const char *name;
    int msb,lsb;
    char * (*fmt)(int);
} v1_vals[] = {
    { "     CSD Structure", 127, 126, __csd_struct },
    { "              TAAC", 119, 112, __taac_val },
    { "              NSAC", 111, 104, __dec_val },
    { "        TRAN_SPEED", 103, 96, __tran_val },
    { "               CCC", 95, 84, __bin_val },
    { "       READ_BL_LEN", 83, 80, __blk_len },
    { "   READ_BL_PARTIAL", 79, 79, __yn_val },       // Single bit
    { "WRITE_BLK_MISALIGN", 78, 78, __yn_val},        // Single bit
    { " READ_BLK_MISALIGN", 77, 77, __yn_val},        // Single bit
    { "           DSR_IMP", 76, 76, __yn_val},        // Single bit
    { "            C_SIZE", 73, 62, __dec_val},
    { "    VDD_R_CURR_MIN", 61, 59, __curr_min_val},
    { "    VDD_R_CURR_MAX", 58, 56, __curr_max_val},
    { "    VDD_W_CURR_MIN", 55, 53, __curr_min_val},
    { "    VDD_W_CURR_MAX", 52, 50, __curr_max_val},
    { "       C_SIZE_MULT", 49, 47, __sm_val},
    { "      ERASE_BLK_EN", 46, 46, __en_val},        // Single bit
    { "       SECTOR_SIZE", 45, 39, __dec_val},
    { "       WP_GRP_SIZE", 38, 32, __dec_val},
    { "     WP_GRP_ENABLE", 31, 31, __en_val},        // Single bit
    { "        R2W_FACTOR", 28, 26, __bin_val},
    { "      WRITE_BL_LEN", 25, 22, __blk_len},
    { "  WRITE_BL_PARTIAL", 21, 21, __yn_val},        // Single bit
    { "   FILE_FORMAT_GRP", 15, 15, __yn_val},        // Single bit
    { "              COPY", 14, 14, __yn_val},        // Single bit
    { "PERM_WRITE_PROTECT", 13, 13, __yn_val},        // Single bit
    { " TMP_WRITE_PROTECT", 12, 12, __yn_val},        // Single bit
    { "       FILE_FORMAT", 11, 10, __dec_val},
    { "               CRC", 7, 1, __hex_val},
    { NULL, 0, 0, NULL}
}, v2_vals[] = {
    { "      CSD Structure", 127, 126, NULL },
    { "              TAAC*", 119, 112, NULL },
    { "              NSAC*", 111, 104, NULL },
    { "        TRAN_SPEED*", 103, 96 , NULL },
    { "                CCC", 95, 84, NULL  },
    { "       READ_BL_LEN*", 83, 80, NULL  },
    { "   READ_BL_PARTIAL*", 79, 79, NULL  },
    { "WRITE_BLK_MISALIGN*", 78, 78, NULL },
    { " READ_BLK_MISALIGN*", 77, 77, NULL },
    { "            DSR_IMP", 76, 76, NULL  },
    { "             C_SIZE", 69, 48, NULL },
    { "     ERASE_BLK_LEN*", 46, 46, NULL  },
    { "       SECTOR_SIZE*", 45, 39, NULL  },
    { "       WP_GRP_SIZE*", 38, 32, NULL  },
    { "     WP_GRP_ENABLE*", 31, 31, NULL  },
    { "        R2W_FACTOR*", 28, 26, NULL  },
    { "      WRITE_BL_LEN*", 25, 22, NULL  },
    { "  WRITE_BL_PARTIAL*", 21, 21, NULL  },
    { "   FILE_FORMAT_GRP*", 15, 15, NULL  },
    { "               COPY", 14, 14, NULL  },
    { " PERM_WRITE_PROTECT", 13, 13, NULL },
    { "  TMP_WRITE_PROTECT", 12, 12, NULL  },
    { "       FILE_FORMAT*", 11, 10, NULL  },
    { "                CRC", 7, 1, NULL },
    { NULL, 0, 0, NULL}
};

// 0b  1 1101 1000 1110 1101 1011 0111    READ_BL_PARTIAL    (79,79)
// 0b  1 1101 1000 1110 1101 1011 0111 1  WRITE_BLK_MISALIGN (78,78)

void debug_csd_v1(uint32_t csd[]);
void debug_csd_v1(uint32_t csd[]) {
    int i;
    char    buf[40];
    DEBUG("CSD V1 DUMP:\n");
    DEBUG("   127             ---             96\n");
    DEBUG("    +-------+-------+-------+-------+\n    ");
    ntoa(csd[0], FMT_BASE_2 | FMT_LEADING_ZERO | 4, buf);
    DEBUG(buf);
    DEBUG(" -- ");
    ntoa(csd[0], FMT_BASE_16 | FMT_LEADING_ZERO | 4, buf);
    DEBUG(buf);
    DEBUG("\n    +-------+-------+-------+-------+\n    ");
    ntoa(csd[1], FMT_BASE_2 | FMT_LEADING_ZERO | 4, buf);
    DEBUG(buf);
    DEBUG(" -- ");
    ntoa(csd[1], FMT_BASE_16 | FMT_LEADING_ZERO | 4, buf);
    DEBUG(buf);
    DEBUG("\n    +-------+-------+-------+-------+\n    ");
    ntoa(csd[2], FMT_BASE_2 | FMT_LEADING_ZERO | 4, buf);
    DEBUG(buf);
    DEBUG(" -- ");
    ntoa(csd[2], FMT_BASE_16 | FMT_LEADING_ZERO | 4, buf);
    DEBUG(buf);
    DEBUG("\n    +-------+-------+-------+-------+\n    ");
    ntoa(csd[3], FMT_BASE_2 | FMT_LEADING_ZERO | 4, buf);
    DEBUG(buf);
    DEBUG(" -- ");
    ntoa(csd[3], FMT_BASE_16 | FMT_LEADING_ZERO | 4, buf);
    DEBUG(buf);
    DEBUG("\n    +-------+-------+-------+-------+\n");

    for (i = 0; v1_vals[i].name != NULL; i++) {
        DEBUG("  ");
        DEBUG(v1_vals[i].name);
        DEBUG(" : ");
        ntoa(sdio_bit_slice(csd, 128, v1_vals[i].msb, v1_vals[i].lsb), FMT_BASE_10, buf);
        DEBUG(buf);
        DEBUG(" -- ");
        ntoa(sdio_bit_slice(csd, 128, v1_vals[i].msb, v1_vals[i].lsb),
            FMT_BASE_2 | FMT_ALTERNATE_FORM, buf);
        DEBUG(buf);
        DEBUG("\n");
    }
}

void debug_csd_v2(uint32_t csd[]);
void debug_csd_v2(uint32_t csd[]) {
    int i;
    char    buf[40];
    DEBUG("CSD V2 DUMP:\n");
    DEBUG("   127             ---             96\n");
    DEBUG("    +-------+-------+-------+-------+\n    ");
    ntoa(csd[0], FMT_BASE_2 | FMT_LEADING_ZERO | 4, buf);
    DEBUG(buf);
    DEBUG(" -- ");
    ntoa(csd[0], FMT_BASE_16 | FMT_LEADING_ZERO | 4, buf);
    DEBUG(buf);
    DEBUG("\n    +-------+-------+-------+-------+\n    ");
    ntoa(csd[1], FMT_BASE_2 | FMT_LEADING_ZERO | 4, buf);
    DEBUG(buf);
    DEBUG(" -- ");
    ntoa(csd[1], FMT_BASE_16 | FMT_LEADING_ZERO | 4, buf);
    DEBUG(buf);
    DEBUG("\n    +-------+-------+-------+-------+\n    ");
    ntoa(csd[2], FMT_BASE_2 | FMT_LEADING_ZERO | 4, buf);
    DEBUG(buf);
    DEBUG(" -- ");
    ntoa(csd[2], FMT_BASE_16 | FMT_LEADING_ZERO | 4, buf);
    DEBUG(buf);
    DEBUG("\n    +-------+-------+-------+-------+\n    ");
    ntoa(csd[3], FMT_BASE_2 | FMT_LEADING_ZERO | 4, buf);
    DEBUG(buf);
    DEBUG(" -- ");
    ntoa(csd[3], FMT_BASE_16 | FMT_LEADING_ZERO | 4, buf);
    DEBUG(buf);
    DEBUG("\n    +-------+-------+-------+-------+\n");

    for (i = 0; v2_vals[i].name != NULL; i++) {
        DEBUG("  ");
        DEBUG(v2_vals[i].name);
        DEBUG(" : ");
        ntoa(sdio_bit_slice(csd, 128, v2_vals[i].msb, v2_vals[i].lsb), FMT_BASE_10, buf);
        DEBUG(buf);
        DEBUG(" -- ");
        ntoa(sdio_bit_slice(csd, 128, v2_vals[i].msb, v2_vals[i].lsb),
            FMT_BASE_2 | FMT_ALTERNATE_FORM, buf);
        DEBUG(buf);
        DEBUG("\n");
    }
}


void
show_sdio_csd(int row, int col, uint32_t csd[]) {
    int i;
    int k;
    int ver = 0;
    uint32_t c_size, c_size_mult, r_blk_len;
    char buf[32];

    move_cursor(console, row, col);
    text_color(console, YELLOW);
    uart_puts(console, "CSD Information:  ");
    k = (csd[0] >> 30) & 0x3;
    switch (k) {
        case 0:
            uart_puts(console, "Version 1.0");
            ver = 1;
            break;
        case 1:
            uart_puts(console, "Version 2.0");
            ver = 2;
            break;
        default:
            uart_puts(console, "Unknown Version");
            break;
    }
    /* compute capacity */
    if (ver == 1) {
        debug_csd_v1(csd);
        DEBUG("Ver 1 CSD\n");
        c_size = sdio_bit_slice(csd, 128, 73, 62);
        c_size_mult = sdio_bit_slice(csd, 128, 49, 47);
        c_size_mult = 1 << (c_size_mult + 2);
        r_blk_len = sdio_bit_slice(csd, 128, 83, 80);
        r_blk_len = 1 << (r_blk_len);
        card_caps.size = (c_size + 1) * ((c_size_mult * r_blk_len) >> 10);
        for (i = 0; v1_vals[i].name != NULL; i++) {
            move_cursor(console, row+1+i, col);
            text_color(console, YELLOW);
            uart_puts(console, v1_vals[i].name);
            uart_puts(console, ": ");
            text_color(console, GREEN);
            uart_puts(console, v1_vals[i].fmt(
                    sdio_bit_slice(csd, 128, v1_vals[i].msb, v1_vals[i].lsb)));
//            ntoa(sdio_bit_slice(csd, 128,
//                v1_vals[i].msb, v1_vals[i].lsb), v1_vals[i].fmt, buf);
//            uart_puts(console, buf);
        }
        row = row + 1 + i;
    } else if (ver == 2) {
        debug_csd_v2(csd);
        DEBUG("Ver 2 CSD\n");
        c_size = sdio_bit_slice(csd, 128, 69, 48);
        card_caps.size = (c_size+1) * 512; // Adjust to # of 512 byte blocks
    }
    card_caps.blocks = card_caps.size << 1; // twice as many blocks
    move_cursor(console, row++, col);
    text_color(console, YELLOW);
    uart_puts(console, "Card size: ");
    ntoa(card_caps.size, FMT_BASE_10, buf);
    text_color(console, GREEN);
    uart_puts(console, buf);
    uart_puts(console, "K bytes.");
    for (i = 0; i < 4; i++) {
        move_cursor(console, row + i, col+2);
        text_color(console, YELLOW);
        uart_puts(console, "[ ");
        uart_putnum(console, FMT_BASE_10, i);
        uart_puts(console, " ] : ");
        text_color(console, GREEN);
        uart_putnum(console, FMT_HEX_LONG | FMT_ALTERNATE_FORM, csd[i]);
    }
}

#define SD_SCR_STRUCT(x)   (sdio_bit_slice(x, 64, 63, 60))
#define SD_SCR_SPEC(x)     (sdio_bit_slice(x, 64, 59, 56))
#define SD_SCR_DATA(x)     (sdio_bit_slice(x, 64, 55, 55))
#define SD_SCR_SECURITY(x) (sdio_bit_slice(x, 64, 54, 52))
#define SD_SCR_BUSWID(x)   (sdio_bit_slice(x, 64, 51, 48))
#define SD_SCR_SPEC3(x)    (sdio_bit_slice(x, 64, 47, 47))
#define SD_SCR_EXSEC(x)    (sdio_bit_slice(x, 64, 46, 43))
#define SD_SCR_SPEC4(x)    (sdio_bit_slice(x, 64, 42, 42))
#define SD_SCR_CMDS(x)     (sdio_bit_slice(x, 64, 35, 32))

void
show_sdio_scr(int row, int col, uint32_t scr) {
    uint32_t s[2];

    s[0] = scr;
    s[1] = 0;
    move_cursor(console, row, col);
    text_color(console, YELLOW);
    uart_puts(console, " -- SCR Register -- ");
    move_cursor(console, row+1, col+12);
    uart_puts(console, "SCR Structure: ");
    text_color(console, GREEN);
    switch (SD_SCR_STRUCT(s)) {
        case 0:
            uart_puts(console, "Version 1.01-4.00");
            break;
        default:
            uart_puts(console, "?Reserved?");
    }
    text_color(console, YELLOW);
    move_cursor(console, row+2, col+13);
    uart_puts(console, "SPEC Version: ");
    text_color(console, GREEN);
    uart_putnum(console, SD_SCR_SPEC(s), FMT_BASE_10);
    text_color(console, YELLOW);
    move_cursor(console, row+3, col+9);
    uart_puts(console, "Data After Erase: ");
    text_color(console, GREEN);
    uart_puts(console, (SD_SCR_DATA(s) != 0) ? "1" : "0");
    text_color(console, YELLOW);
    move_cursor(console, row+4, col+4);
    uart_puts(console, "CPRM Security Support: ");
    text_color(console, GREEN);
    switch (SD_SCR_SECURITY(s)) {
        case 0:
            uart_puts(console, "None");
            break;
        case 1:
            uart_puts(console, "Not Used");
            break;
        case 2:
            uart_puts(console, "SDSC Security (V 1.01)");
            break;
        case 3:
            uart_puts(console, "SDHC Security (V 2.00)");
            break;
        case 4:
            uart_puts(console, "SDXC Security (V 3.xx)");
            break;
        default:
            uart_puts(console, "?Reserved?");
            break;
    }
    text_color(console, YELLOW);
    move_cursor(console, row+5, col+15);
    uart_puts(console, "Bus Widths: ");
    text_color(console, GREEN);
    uart_puts(console, (SD_SCR_BUSWID(s) == 5) ? "1b, 4b" : "?Unk?");
    text_color(console, YELLOW);
    move_cursor(console, row+6, col+16);
    uart_puts(console, "SD Spec 3: ");
    text_color(console, GREEN);
    uart_puts(console, (SD_SCR_SPEC3(s)) ? "True" : "False");
    text_color(console, YELLOW);
    move_cursor(console, row+7, col+16);
    uart_puts(console, "SD Spec 4: ");
    text_color(console, GREEN);
    uart_puts(console, (SD_SCR_SPEC4(s)) ? "True" : "False");
    text_color(console, YELLOW);
    move_cursor(console, row+8, col+8);
    uart_puts(console, "Extended Security: ");
    text_color(console, GREEN);
    uart_puts(console, (SD_SCR_EXSEC(s)) ? "True" : "False");
    text_color(console, YELLOW);
    move_cursor(console, row+9, col);
    uart_puts(console, "Addt'l Commands Supported: ");
    text_color(console, GREEN);
    if (SD_SCR_CMDS(s)) {
        if (SD_SCR_CMDS(s) & 0x8) {
            uart_puts(console, "CMD58/59, ");
        }
        if (SD_SCR_CMDS(s) & 0x4) {
            uart_puts(console, "CMD48/49, ");
        }
        if (SD_SCR_CMDS(s) & 0x2) {
            uart_puts(console, "CMD23, ");
        }
        if (SD_SCR_CMDS(s) & 0x1) {
            uart_puts(console, "CMD20");
        }
    } else {
        uart_puts(console, "None");
    }
}


/*
 * Dump out the SD Card status into the debug console
 */
void
debug_sdio_sdstatus(uint32_t buf[]) {
    int i;

    DEBUG("SD Card Status:\n");
    DEBUG("Bus Width: ");
    switch(sdio_bit_slice(buf, 512, 511, 510)) {
        case 0:
            DEBUG("1 bit\n");
            break;
        default:
            DEBUG("Reserved\n");
            break;
        case 2:
            DEBUG("4 bits\n");
    }
    DEBUG("Secure Mode: ");
    DEBUG((sdio_bit_slice(buf, 512, 509, 509) ? "Yes\n" : "No\n"));
    DEBUG("Card Type: ");
    DEBUG((sdio_bit_slice(buf, 512, 495, 480) ? "SD ROM\n" : "SD Card\n"));
    DEBUG("Size of Protected Area: ");
    uart_putnum(debug_console, FMT_BASE_10, sdio_bit_slice(buf, 512, 479, 448));
    DEBUG("\nSpeed Class: ");
    switch (sdio_bit_slice(buf, 512, 447,440)) {
        case 0: DEBUG("Class 0\n");
                break;
        case 1: DEBUG("Class 2\n");
                break;
        case 2: DEBUG("Class 4\n");
                break;
        case 3: DEBUG("Class 6\n");
                break;
        default:
                DEBUG("Reserved\n");
    }
    i = sdio_bit_slice(buf, 512, 439, 432);
    DEBUG("Performance Move: ");
    if ((i > 0) && (i < 255)) {
        uart_putnum(debug_console, FMT_BASE_10, i);
        DEBUG("MB/sec\n");
    } else {
        DEBUG((i) ? "Inf\n" : "Undef\n");
    }
    DEBUG("AU Size: ");
    i = sdio_bit_slice(buf, 512, 431, 428);
    if (i == 0) {
        DEBUG("Undef\n");
    } else if (i < 10) {
        uart_putnum(debug_console, FMT_BASE_10, 8 << i);
        DEBUG("kB\n");
    } else {
        DEBUG("Reserved\n");
    }
    return;
}

