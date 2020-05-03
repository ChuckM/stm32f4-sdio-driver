/*
 * Copyright (C) 2013 Chuck McManis <cmcmanis@mcmanis.com>
 *
 * Simple clock setup/driver for the STM32F4-Discovery board.
 * libopencm3 does the heavy lifting, it just sets up SysTick
 * and the desired clock rate (168Mhz in this case)
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>
#include <stdint.h>
#include "clock.h"
#include "FreeRTOS.h"
#include "task.h"

/*
 * Comment this #define out if you don't want the BLUE led on
 * the board blinking at 2Hz
 */
#define SYSTICK_HEARTBEAT

/*
 * SysTick support routines
 */

/* monotonically increasing number of milliseconds from reset
 * overflows every 49 days if you're wondering
 */
volatile uint32_t system_millis;

/* sleep for delay milliseconds */
void msleep(uint32_t delay) {
    uint32_t wake = system_millis + delay;
    while (wake > system_millis) ;
}

void vApplicationTickHook(void)
{
    if ((xTaskGetTickCount() % 500) == 0)
        gpio_toggle(GPIOA, GPIO6);
}

/* return the time */
uint32_t mtime() {
    return xTaskGetTickCount();
}

/* Set STM32 to 168 MHz. */
void
clock_init(int systick_rate)
{
       rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
#ifdef SYSTICK_HEARTBEAT
        rcc_periph_clock_enable(RCC_GPIOD);
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO15);
#endif
}
