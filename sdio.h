/* this define lets the init code know that you are using a GPIO as a card
 * detect pin */
#define SDIO_HAS_CARD_DETECT

enum SDIO_CLOCK_DIV {
    SDIO_24MHZ = 0,
    SDIO_16MHZ,
    SDIO_12MHZ,
    SDIO_8MHZ,
    SDIO_4MHZ,
    SDIO_1MHZ,
    SDIO_400KHZ
};

enum SDIO_POWER_STATE {
    SDIO_POWER_ON,
    SDIO_POWER_OFF
};

#define SDIO_CARD_CCS(c)     (((c)->ocr & 0x40000000) != 0)
#define SDIO_CARD_UHS2(c)   (((c)->ocr & 0x40000000) != 0)
#define SDIO_CARD_LVOK(c)   (((c)->ocr & 0x01000000) != 0)

typedef struct SDIO_CARD_DATA {
    uint32_t    props;
    uint32_t    ocr;
    uint32_t    cid[4];
    uint32_t    csd[4];
    uint32_t    scr[2];
    uint32_t    status[16];
    uint32_t    size;
    uint16_t    rca;
} * SDIO_CARD;


int sdio_bus(int bits, enum SDIO_CLOCK_DIV freq);
void sdio_init(void);
void sdio_reset(enum SDIO_POWER_STATE state);
SDIO_CARD sdio_open(void);
int sdio_command(uint32_t cmd, uint32_t arg);
int sdio_readblock(SDIO_CARD, uint32_t lba, uint8_t *buf);
int sdio_readblock_dma(SDIO_CARD, uint32_t lba, uint8_t *buf, uint8_t wait);
int sdio_writeblock(SDIO_CARD, uint32_t lba, uint8_t *buf);
int sdio_writeblock_dma(SDIO_CARD, uint32_t lba, uint8_t *buf, uint8_t wait);
int sdio_wait_for_completion(void);
int sdio_status(SDIO_CARD);
void sdio_print_log(int console, int number_of_entries);
const char *sdio_errmsg(int err);
uint32_t sdio_bit_slice(uint32_t a[], int bits, int msb, int lsb);

