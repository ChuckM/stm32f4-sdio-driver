#ifndef UART7
void uart7_isr(void);
void uart8_isr(void);

#define UART7_BASE  (PERIPH_BASE_APB1 + 0x07800)
#define UART8_BASE  (PERIPH_BASE_APB1 + 0x07c00)
#define UART7       UART7_BASE
#define UART8       UART8_BASE

#define RCC_APB1ENR_UART7EN (0x1 << 30)
#define RCC_APB1ENR_UART8EN (0x1 << 31)

#define NVIC_UART7_IRQ  82
#define NVIC_UART8_IRQ  83
#endif

