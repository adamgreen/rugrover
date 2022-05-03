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
#include <nrf_delay.h>
#include <nrf_gpio.h>

int main(void)
{
    /* Toggle LEDs. */
    const uint32_t leds[2] = { 19, 20 };
    for (size_t i = 0 ; i < 2 ; i++) {
        nrf_gpio_cfg_output(leds[i]);
    }
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
