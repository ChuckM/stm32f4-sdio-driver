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

/* Called when systick fires */
void
sys_tick_handler(void) {
    system_millis++;
#ifdef SYSTICK_HEARTBEAT
    /* Generate a 2Hz heart beep blink */
    if ((system_millis % 500) == 0) {
        gpio_toggle(GPIOD, GPIO15); // BLUE led
    }
#endif
}

/* sleep for delay milliseconds */
void msleep(uint32_t delay) {
    uint32_t wake = system_millis + delay;
    while (wake > system_millis) ;
}

/* return the time */
uint32_t mtime() {
    return system_millis;
}

/* Set up a timer to create 1mS ticks. */
static void
systick_setup(int tick_rate) {
    /* clock rate / 1000 to get 1mS interrupt rate */
    systick_set_reload((168000000) / tick_rate);
    STK_CTRL = 0x07;
#if 0
    /* libopencm3 bug, the above assign does all three of these
     * steps, but systick_set_clocksource fails to set the clock
     * source to AHB because it doesn't correctly shift the value
     */
    systick_set_clocksource(1);
    systick_counter_enable();
    systick_interrupt_enable();
#endif
}

/* Set STM32 to 168 MHz. */
void
clock_init(int systick_rate)
{
	rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);
#ifdef SYSTICK_HEARTBEAT
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPDEN);
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO15);
#endif
    if (systick_rate) {
        systick_setup(systick_rate);
    }
}
