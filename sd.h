enum SD_CLOCK_DIV {
    CLOCK_24MHZ = 0,
    CLOCK_16MHZ,
    CLOCK_12MHZ,
    CLOCK_8MHZ,
    CLOCK_4MHZ,
    CLOCK_1MHZ,
    CLOCK_400KHZ
};

int sd_bus(int bits, enum SD_CLOCK_DIV freq);

