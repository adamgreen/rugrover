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
    ------------------------------------------------------------------------
          0.8           LMOTOR_ENCODER_A_PIN    Left Motor Encoder A
          0.9           LMOTOR_ENCODER_B_PIN    Left Motor Encoder B
          0.10          RMOTOR_ENCODER_A_PIN    Right Motor Encoder A
          0.11          RMOTOR_ENCODER_B_PIN    Right Motor Encoder B
    ------------------------------------------------------------------------
          0.31          LMOTOR_EN_PIN           Left Motor Driver - EN
          0.21          LMOTOR_PWM1_PIN         Left Motor Driver - PWM1
          0.5           LMOTOR_PWM2_PIN         Left Motor Driver - PWM2
          0.15          LMOTOR_DIAG_PIN         Left Motor Driver - DIAG
          0.3           LMOTOR_OCM_PIN          Left Motor Driver - OCM
          0.12          RMOTOR_EN_PIN           Right Motor Driver - EN
          0.6           RMOTOR_PWM1_PIN         Right Motor Driver - PWM1
          0.7           RMOTOR_PWM2_PIN         Right Motor Driver - PWM2
          0.16          RMOTOR_DIAG_PIN         Right Motor Driver - DIAG
          0.4           RMOTOR_OCM_PIM          Right Motor Driver - OCM

*/
#include <stdio.h>
#include <app_util_platform.h>
#include <nrf_delay.h>
#include <nrf_gpio.h>
#include <core/mri.h>
#include "OLED_SH1106/OLED_SH1106.h"
#include "QuadratureDecoder/QuadratureDecoderHardware.h"
#include "QuadratureDecoder/QuadratureDecoderSoftware.h"
#include "DualTB9051FTGDrivers/DualTB9051FTGDrivers.h"
#include "SAADCScanner/SAADCScanner.h"



// Blink this LED (LED4 on nRF52-DK) to show progress.
#define BLINK_LED_PIN   20

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

// Pins connected to the motor drivers.
#define LMOTOR_EN_PIN       31
#define LMOTOR_PWM1_PIN     21
#define LMOTOR_PWM2_PIN     5
#define LMOTOR_DIAG_PIN     15
#define LMOTOR_OCM_PIN      3
#define RMOTOR_EN_PIN       12
#define RMOTOR_PWM1_PIN     6
#define RMOTOR_PWM2_PIN     7
#define RMOTOR_DIAG_PIN     16
#define RMOTOR_OCM_PIN      4

// Should the motor's idea of forward be reversed?
#define LMOTOR_REVERSE      true
#define RMOTOR_REVERSE      false

// The frequency to be used for the motor PWM signal.
#define MOTOR_FREQUENCY     20000

// Don't allow motor current to go over 1.5A.
#define MOTOR_MAX_CURRENT_mA    1500


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

// Analog to digitial converter object.
SAADCScanner                g_adc(NRF_SAADC_RESOLUTION_12BIT, _PRIO_APP_LOWEST);

// PWM peripheral instance used for the motor drivers.
nrf_drv_pwm_t               g_PWM = NRF_DRV_PWM_INSTANCE(0);

// Motor drivers.
DualTB9051FTGDrivers        g_motors(LMOTOR_EN_PIN, LMOTOR_PWM1_PIN, LMOTOR_PWM2_PIN,
                                     LMOTOR_DIAG_PIN, LMOTOR_OCM_PIN, LMOTOR_REVERSE,
                                     RMOTOR_EN_PIN, RMOTOR_PWM1_PIN, RMOTOR_PWM2_PIN,
                                     RMOTOR_DIAG_PIN, RMOTOR_OCM_PIN, RMOTOR_REVERSE,
                                     &g_adc,
                                     &g_PWM, MOTOR_FREQUENCY, MOTOR_MAX_CURRENT_mA,_PRIO_APP_LOWEST);



static void verifyUICRBits();
static void hangOnError(const char* pErrorMessage);
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

    // The software quadrature decoder uses GPIOTE so initialize it.
    nrf_drv_gpiote_init();

    // UNDONE: Making sure that I can debug my GPIOTE based encoder ISR. Maybe make it higher prio later?
    NVIC_SetPriority(GPIOTE_IRQn, _PRIO_APP_LOWEST);

    // Init the wheel encoders.
    bool result;
    result = g_leftEncoder.init();
    ASSERT ( result );
    result = g_rightEncoder.init();
    ASSERT ( result );

    // Initialize the ADC object which scans all of the configured ADC channels as often as possible (~200kHz).
    result = g_adc.init();
    ASSERT ( result );

    // Initialize and enable the dual channel motor controller.
    result = g_motors.init();
    ASSERT ( result );
    g_motors.enable(true);

    // Initialize GPIO pin for LED that is flashed to show progress.
    nrf_gpio_cfg_output(BLINK_LED_PIN);

    // Used to calculate encoder tick deltas.
    int32_t prevLeftCount = g_leftEncoder.getCount();
    int32_t prevRightCount = g_rightEncoder.getCount();

    // Far enough along that it is safe for MRI to call our hooks to disable motors when entering the debugger.
    mriSetDebuggerHooks(enteringDebuggerHook, leavingDebuggerHook, NULL);

    // Enter main action loop.
    const uint32_t delayms = 10;
    const int32_t powerLimit = 50;
    const int32_t powerStep = 10;
    const uint32_t powerDelayms = 2000;
    int32_t power = 0;
    int32_t powerInc = powerStep;
    uint32_t powerIter = 0;
    while (true)
    {
        // Check to see if either motor controller has detected a fault (over voltage, over current, over temp, etc).
        if (g_motors.hasEitherMotorEncounteredFault())
        {
            hangOnError("Motor fault detected!");
        }

        // Check for motor currents measured by ADC that are too high (software over current detection).
        if (g_motors.hasEitherMotorDetectedCurrentOverload())
        {
            hangOnError("Motor over current detected!");
        }

        // Get the latest analog motor current readings.
        DualTB9051FTGDrivers::CurrentReadings currentReadings = g_motors.getCurrentReadings();

        // Every so often the motor power should be changed so that it oscillates back and forth between the set
        // positive and negative limits.
        if (++powerIter >= powerDelayms/delayms)
        {
            // Update motor power.
            power += powerInc;
            if (abs(power) > powerLimit)
            {
                powerInc = -powerInc;
                power += 2*powerInc;
            }

            // Toggle the LED to show progress.
            nrf_gpio_pin_toggle(BLINK_LED_PIN);

            powerIter = 0;
        }
        g_motors.setPower(power, power);

        // Retrieve left and right encoder tick counts.
        int32_t leftCount = g_leftEncoder.getCount();
        int32_t rightCount = g_rightEncoder.getCount();

        // Calculate encoder ticks/s rate.
        int32_t leftRate = (leftCount - prevLeftCount) * (1000/delayms);
        int32_t rightRate = (rightCount - prevRightCount) * (1000/delayms);
        prevLeftCount = leftCount;
        prevRightCount = rightCount;

        // Display PWM rate, encoder rate, motor current.
        g_OLED.clearScreen();
        g_OLED.printf("    PWM: %ld\n", power);
        g_OLED.printf("Current: %ld, %lu\n", currentReadings.leftCurrent_mA, currentReadings.rightCurrent_mA);
        g_OLED.printf("Encoder: %ld, %ld\n", leftRate, rightRate);
//      g_OLED.printf("Encoder: %ld, %ld\n", leftCount, rightCount);
        g_OLED.refresh();

        // UNDONE: Simulating a 100Hz PID loop.
        nrf_delay_ms(delayms);
    }
}

static void verifyUICRBits()
{
    // UNDONE: Verify that the bits are clear when they are supposed to be as well.

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

static void hangOnError(const char* pErrorMessage)
{
    printf("%s\r\n", pErrorMessage);

    g_OLED.clearScreen();
    g_OLED.printf("%s", pErrorMessage);
    g_OLED.refresh();

    g_motors.enable(false);
    while (true)
    {
        // Loop here forever if a motor fault has been detected.
    }
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

    // Stop the motors when entering the debugger so that the robot doesn't run off.
    g_motors.enable(false);
}

static void leavingDebuggerHook(void* pvContext)
{
    if (g_mriDisableHooks)
    {
        return;
    }

    // Can re-enable the motors now.
    g_motors.enable(true);
}


// Required to get C++ code to build.
extern "C" void __cxa_pure_virtual()
{
    ASSERT ( 0 );
    abort();
}
