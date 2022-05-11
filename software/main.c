/*  Copyright (C) 2022  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
/* Beginnings of firmware for RugRover robot. */
#include <app_error.h>
#include <nrf_delay.h>
#include <nrf_gpio.h>
#include <nrf_drv_gpiote.h>

static void gpioteHandler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);

int main(void)
{
    /* Toggle LEDs. */
    const uint32_t leds[2] = { 19, 20 };
    for (size_t i = 0 ; i < 2 ; i++) {
        nrf_gpio_cfg_output(leds[i]);
    }

    /* Interrupt on button 3 presses. */
    const uint32_t button3Pin = 15;
    nrf_drv_gpiote_in_config_t gpioteConfig =
    {
        .sense = NRF_GPIOTE_POLARITY_HITOLO,
        .pull = NRF_GPIO_PIN_PULLUP,
        .is_watcher = false,
        .hi_accuracy = false
    };
    uint32_t errorCode = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(errorCode);
    errorCode = nrf_drv_gpiote_in_init(button3Pin, &gpioteConfig, gpioteHandler);
    APP_ERROR_CHECK(errorCode);
    nrf_drv_gpiote_in_event_enable(button3Pin, true);

    while (true)
    {
        //*(volatile uint32_t*)0xFFFFFFFF; // = 0xbaadf00d;

        for (int i = 0; i < 2; i++)
        {
            nrf_gpio_pin_toggle(leds[i]);
            nrf_delay_ms(500);
        }
    }
}

// ISR for button 3 press running at priority level of 2 (higher priority than MRI).
static void gpioteHandler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    // Crash when button 3 is pressed.
    *(volatile uint32_t*)0xFFFFFFFF; // = 0xbaadf00d;
}
