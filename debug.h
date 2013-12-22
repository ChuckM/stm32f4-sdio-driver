extern int debug_console;
/* Function prototypes */
void debug_init(void);
char debug_getc(int wait);
void debug_putc(char c);
void debug_puts(const char *s);
void debug_wait(void);
