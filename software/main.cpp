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
/* Beginnings of firmware for RugRover robot.
   The hardware pin connections are:
   nRF52 Pin Number     Pin Name            Pin Description
          0.24          OLED_SCK_PIN        OLED Board - CLK
          0.25          OLED_MOSI_PIN       OLED Board - MOS
          0.26          OLED_DC_PIN         OLED Board - DC
          0.27          OLED_CS_PIN         OLED Board - CS
          0.2           OLED_RESET_PIN      OLED Board - RES
*/
#include <stdio.h>
#include <nrf_delay.h>
#include <nrf_gpio.h>
#include <core/mri.h>
#include <OLED_SH1106.h>

// The SPIM peripheral instance to use for communicating with the OLED. Note that SPIM instances share the same
// resources as TWI (I2C).
#define OLED_SPI_INSTANCE   0

// Pins connected to the SPI signals used by the SH1106 OLED driver.
#define OLED_SCK_PIN    24
#define OLED_MOSI_PIN   25
#define OLED_CS_PIN     27
#define OLED_DC_PIN     26
#define OLED_RESET_PIN  2

// Dimensions of the 1.3" OLED screen.
#define OLED_WIDTH      128
#define OLED_HEIGHT     64
// The Pololu's OLED screen's columns are centered on the 132 segment drivers.
#define OLED_COLUMN_OFFSET  ((OLED_SH1106_MAX_WIDTH-OLED_WIDTH)/2)
// The OLED screen's rows use up all of the 64 common drivers.
#define OLED_ROW_OFFSET     0



// SPIM instance used for driving the OLED.
static const nrf_drv_spi_t  g_spiOLED = NRF_DRV_SPI_INSTANCE(OLED_SPI_INSTANCE);

// OLED Screen.
static OLED_SH1106          g_OLED(OLED_WIDTH, OLED_HEIGHT, OLED_COLUMN_OFFSET, OLED_ROW_OFFSET,
                                   &g_spiOLED, OLED_SCK_PIN, OLED_MOSI_PIN, OLED_DC_PIN);



static void enteringDebuggerHook(void* pvContext);
static void leavingDebuggerHook(void* pvContext);



int main(void)
{
    // Looks like I can just hardwire CS low and RESET high on the OLED. This will free up 2 pins.
    nrf_gpio_pin_clear(OLED_CS_PIN);
    nrf_gpio_cfg_output(OLED_CS_PIN);
    nrf_gpio_pin_set(OLED_RESET_PIN);
    nrf_gpio_cfg_output(OLED_RESET_PIN);

    g_OLED.init();
    g_OLED.setRotation(2);
    g_OLED.setBrightness(16);
    g_OLED.print("Hello World\n");
    for (int i = 2 ; i <= 7 ; i++)
    {
        g_OLED.printf("Line %d\n", i);
    }
    g_OLED.print("1234567890123456789012345\n");
    g_OLED.refresh();

    /* Toggle LEDs. */
    const uint32_t leds[2] = { 19, 20 };
    for (size_t i = 0 ; i < 2 ; i++) {
        nrf_gpio_cfg_output(leds[i]);
    }

    bool invert = true;
    mriSetDebuggerHooks(enteringDebuggerHook, leavingDebuggerHook, NULL);
    while (true)
    {
        for (int i = 0; i < 2; i++)
        {
            nrf_gpio_pin_toggle(leds[i]);
            nrf_delay_ms(500);

            g_OLED.invertDisplay(invert);
            invert = !invert;
        }
    }
}



// Break into debugger if any errors/asserts are detected at runtime.
// This handler is called when an application error is encountered in BLE stack or application code.
void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    { __asm volatile ("bkpt #0"); }
}


/* Flag set to flag that MRI hooks should be disabled when writing to stdout. */
extern volatile int g_mriDisableHooks;

static void enteringDebuggerHook(void* pvContext)
{
    if (g_mriDisableHooks)
    {
        return;
    }

    // Turn both LEDs off when entering debugger.
    nrf_gpio_pin_set(19);
    nrf_gpio_pin_set(20);
}

static void leavingDebuggerHook(void* pvContext)
{
    if (g_mriDisableHooks)
    {
        return;
    }
}


// Required to get C++ code to build.
extern "C" void __cxa_pure_virtual()
{
    ASSERT ( 0 );
    abort();
}
