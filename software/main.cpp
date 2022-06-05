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
   nRF52 Pin Number     Pin Name                Pin Description
          0.24          OLED_SCK_PIN            OLED Board - CLK
          0.25          OLED_MOSI_PIN           OLED Board - MOS
          0.26          OLED_DC_PIN             OLED Board - DC
          0.27          OLED_CS_PIN             OLED Board - CS
          0.2           OLED_RESET_PIN          OLED Board - RES
          0.8           LMOTOR_ENCODER_A_PIN    Left Motor Encoder A
          0.9           LMOTOR_ENCODER_B_PIN    Left Motor Encoder B
          0.10          RMOTOR_ENCODER_A_PIN    Right Motor Encoder A
          0.11          RMOTOR_ENCODER_B_PIN    Right Motor Encoder B
*/
#include <stdio.h>
#include <app_util_platform.h>
#include <nrf_delay.h>
#include <nrf_gpio.h>
#include <core/mri.h>
#include "OLED_SH1106/OLED_SH1106.h"
#include "QuadratureDecoder/QuadratureDecoderHardware.h"
#include "QuadratureDecoder/QuadratureDecoderSoftware.h"

// The SPIM peripheral instance to use for communicating with the OLED. Note that SPIM instances share the same
// resources as TWI (I2C).
#define OLED_SPI_INSTANCE   0

// Pins connected to the SPI signals used by the SH1106 OLED driver.
#define OLED_SCK_PIN    24
#define OLED_MOSI_PIN   25
#define OLED_CS_PIN     27
#define OLED_DC_PIN     26
#define OLED_RESET_PIN  2

// Pins connected to the motor quadrature encoders.
#define LMOTOR_ENCODER_A_PIN    8
#define LMOTOR_ENCODER_B_PIN    9
#define RMOTOR_ENCODER_A_PIN    10
#define RMOTOR_ENCODER_B_PIN    11

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

// Quadrature encoders.
QuadratureDecoderHardware   g_leftEncoder(LMOTOR_ENCODER_A_PIN, LMOTOR_ENCODER_B_PIN);
QuadratureDecoderSoftware   g_rightEncoder(RMOTOR_ENCODER_A_PIN, RMOTOR_ENCODER_B_PIN);


static void verifyUICRBits();
static void enteringDebuggerHook(void* pvContext);
static void leavingDebuggerHook(void* pvContext);



int main(void)
{
    verifyUICRBits();

    // Looks like I can just hardwire CS low and RESET high on the OLED. This will free up 2 pins.
    nrf_gpio_pin_clear(OLED_CS_PIN);
    nrf_gpio_cfg_output(OLED_CS_PIN);
    nrf_gpio_pin_set(OLED_RESET_PIN);
    nrf_gpio_cfg_output(OLED_RESET_PIN);

    // Init the OLED used for debug output.
    g_OLED.init();
    g_OLED.setRotation(2);
    g_OLED.setBrightness(16);
    g_OLED.refresh();

    // Init the wheel encoder.
    bool result;
    nrf_drv_gpiote_init();

    // UNDONE: Making sure that I can debug my GPIOTE based encoder ISR.
    NVIC_SetPriority(GPIOTE_IRQn, _PRIO_APP_LOWEST);

    result = g_leftEncoder.init();
    ASSERT ( result );
    result = g_rightEncoder.init();
    ASSERT ( result );

    // UNDONE: Can remove these later as they are used for early bringup testing.
    // Initialize GPIO pins for LEDs that are flashed to show progress.
    const uint32_t leds[2] = { 19, 20 };
    for (size_t i = 0 ; i < 2 ; i++) {
        nrf_gpio_cfg_output(leds[i]);
    }

    int32_t prevLeftCount = g_leftEncoder.getCount();
    int32_t prevRightCount = g_rightEncoder.getCount();
    mriSetDebuggerHooks(enteringDebuggerHook, leavingDebuggerHook, NULL);
    while (true)
    {
        for (int i = 0; i < 2; i++)
        {
            // Flash the LEDs to show progress.
            nrf_gpio_pin_toggle(leds[i]);
            nrf_delay_ms(125);

            // Retrieve left and right encoders.
            int32_t leftCount = g_leftEncoder.getCount();
            int32_t rightCount = g_rightEncoder.getCount();

            // Calculate ticks/s rate.
            int32_t leftRate = (leftCount - prevLeftCount) * 8;
            int32_t rightRate = (rightCount - prevRightCount) * 8;
            prevLeftCount = leftCount;
            prevRightCount = rightCount;

            // Display current left and right encoder counts.
            g_OLED.clearScreen();
//            g_OLED.printf("%ld\n%ld", leftCount, rightCount);
            g_OLED.printf("%ld\n%ld", leftRate, rightRate);
            g_OLED.refresh();
        }
    }
}

static void verifyUICRBits()
{
    // Can't set UICR bits directly from this code as SoftDevice is already running for mriblue_boot.
    // Just verify that they have the correct results here instead. "make flash_softdevice" should be setup to set it
    // using nrfjprog.

    /* Configure NFCT pins as GPIOs if NFCT is not to be used in your code. If CONFIG_NFCT_PINS_AS_GPIOS is not defined,
       two GPIOs (see Product Specification to see which ones) will be reserved for NFC and will not be available as
       normal GPIOs. */
    #if defined (CONFIG_NFCT_PINS_AS_GPIOS)
        if ((NRF_UICR->NFCPINS & UICR_NFCPINS_PROTECT_Msk) == (UICR_NFCPINS_PROTECT_NFC << UICR_NFCPINS_PROTECT_Pos))
        {
            // Run "make flash_softdevice" to set correctly.
            ASSERT ( false );
        }
    #endif

    /* Configure GPIO pads as pPin Reset pin if Pin Reset capabilities desired. If CONFIG_GPIO_AS_PINRESET is not
      defined, pin reset will not be available. One GPIO (see Product Specification to see which one) will then be
      reserved for PinReset and not available as normal GPIO. */
    #if defined (CONFIG_GPIO_AS_PINRESET)
        if (((NRF_UICR->PSELRESET[0] & UICR_PSELRESET_CONNECT_Msk) != (UICR_PSELRESET_CONNECT_Connected << UICR_PSELRESET_CONNECT_Pos)) ||
            ((NRF_UICR->PSELRESET[1] & UICR_PSELRESET_CONNECT_Msk) != (UICR_PSELRESET_CONNECT_Connected << UICR_PSELRESET_CONNECT_Pos)))
        {
            // Run "make flash_softdevice" to set correctly.
            ASSERT ( false );
        }
    #endif
}


// Break into debugger if any errors/asserts are detected at runtime.
// This handler is called when an application error is encountered in BLE stack or application code.
void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    { __asm volatile ("bkpt #0"); }
}


// Flag set to signal that MRI hooks should be disabled when writing to stdout.
extern volatile int g_mriDisableHooks;

static void enteringDebuggerHook(void* pvContext)
{
    if (g_mriDisableHooks)
    {
        return;
    }

    // UNDONE: Can remove this later.
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
