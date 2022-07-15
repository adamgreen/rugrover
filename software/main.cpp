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
#include "DifferentialDrive/DifferentialDrive.h"



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

// The number of encoder ticks per revolution of the wheel.
#define MOTOR_TICKS_PER_REV 1250

// The frequency to be used for the motor PWM signal.
#define MOTOR_FREQUENCY     20000

// Don't allow motor current to go over 1.5A.
#define MOTOR_MAX_CURRENT_mA    1500

// PID contants used for the motors.
#define MOTOR_PID_Kc            1.0f
#define MOTOR_PID_Ti            0.0f
#define MOTOR_PID_Td            0.0f

// Don't allow the PID controller to command the motor power past this percentage value (0 - 100).
#define MOTOR_PID_MAX_PERCENT   80


// Dimensions of the 1.3" OLED screen.
#define OLED_WIDTH      128
#define OLED_HEIGHT     64
// The Pololu's OLED screen's columns are centered on the 132 segment drivers.
#define OLED_COLUMN_OFFSET  ((OLED_SH1106_MAX_WIDTH-OLED_WIDTH)/2)
// The OLED screen's rows use up all of the 64 common drivers.
#define OLED_ROW_OFFSET     0



// Enumeration of routines which can be run from the main loop.
enum DebugRoutines
{
    DEBUG_PID = 1,
    DEBUG_MAX
};



// SPIM instance used for driving the OLED.
static const nrf_drv_spi_t  g_spiOLED = NRF_DRV_SPI_INSTANCE(OLED_SPI_INSTANCE);

// OLED Screen.
static OLED_SH1106          g_OLED(OLED_WIDTH, OLED_HEIGHT, OLED_COLUMN_OFFSET, OLED_ROW_OFFSET,
                                   &g_spiOLED, OLED_SCK_PIN, OLED_MOSI_PIN, OLED_DC_PIN);

// Analog to digitial converter object.
static SAADCScanner         g_adc(NRF_SAADC_RESOLUTION_12BIT, _PRIO_APP_LOWEST);

// PWM peripheral instance used for the motor drivers.
static nrf_drv_pwm_t        g_PWM = NRF_DRV_PWM_INSTANCE(0);

// Closed loop PID control of the robot's two motors.
static DifferentialDrive    g_drive(LMOTOR_EN_PIN, LMOTOR_PWM1_PIN, LMOTOR_PWM2_PIN,
                                     LMOTOR_DIAG_PIN, LMOTOR_OCM_PIN, LMOTOR_REVERSE,
                                     LMOTOR_ENCODER_A_PIN, LMOTOR_ENCODER_B_PIN,
                                     RMOTOR_EN_PIN, RMOTOR_PWM1_PIN, RMOTOR_PWM2_PIN,
                                     RMOTOR_DIAG_PIN, RMOTOR_OCM_PIN, RMOTOR_REVERSE,
                                     RMOTOR_ENCODER_A_PIN, RMOTOR_ENCODER_B_PIN,
                                     MOTOR_TICKS_PER_REV,
                                     MOTOR_PID_Kc, MOTOR_PID_Ti, MOTOR_PID_Td, MOTOR_PID_MAX_PERCENT,
                                     &g_adc,
                                     &g_PWM, MOTOR_FREQUENCY, MOTOR_MAX_CURRENT_mA);

// If this global gets set to non-zero from debugger then enter debug menu.
static int           g_dbg = 0;

// Which debug routine should be executed.
static DebugRoutines g_dbgRoutine = DEBUG_PID;



static void testPidRoutine();
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
    g_OLED.setTextColor(1, 0);
    g_OLED.refresh();

    // The software quadrature decoder uses GPIOTE so initialize it.
    nrf_drv_gpiote_init();

    // UNDONE: Making sure that I can debug my GPIOTE based encoder ISR. Maybe make it higher prio later?
    NVIC_SetPriority(GPIOTE_IRQn, _PRIO_APP_LOWEST);

    // Initialize the ADC object which scans all of the configured ADC channels as often as possible (~200kHz).
    bool result;
    result = g_adc.init();
    ASSERT ( result );

    // Initialize and enable the closed loop different drive system.
    result = g_drive.init();
    ASSERT ( result );
    g_drive.enable();

    // Initialize GPIO pin for LED that is flashed to show progress.
    nrf_gpio_cfg_output(BLINK_LED_PIN);

    // Far enough along that it is safe for MRI to call our hooks to disable motors when entering the debugger.
    mriSetDebuggerHooks(enteringDebuggerHook, leavingDebuggerHook, NULL);

    while (true)
    {
        g_dbg = 0;
        switch (g_dbgRoutine)
        {
            case DEBUG_PID:
                testPidRoutine();
                break;
            default:
                break;
        }

        while (true)
        {
            uint32_t choice = 0;
            char     buffer[16];

            printf("\n\nDebug Menu\n");
            printf("1. Test PID code\n");
            printf("\nSelect: ");
            fgets(buffer, sizeof(buffer), stdin);
            choice = strtoul(buffer, NULL, 10);
            if (choice < 1 || choice >= DEBUG_MAX)
            {
                printf("Invalid selection\n");
            }
            else
            {
                g_dbgRoutine = (DebugRoutines)choice;
                break;
            }
        }
    }
}

static void testPidRoutine()
{
    // Enter main action loop.
    const uint32_t delayms = 10;
    const uint32_t delay_us = delayms * 1000;
    const int32_t powerLimit = 50;
    const int32_t powerStep = 10;
    const uint32_t powerDelayms = 2000;
    int32_t power = 0;
    int32_t powerInc = powerStep;
    uint32_t powerIter = 0;
    uint32_t count = 0;

    // Setup the fixed heading text on the OLED.
    g_OLED.clearScreen();
    g_OLED.printf("    PWM:\n");
    g_OLED.printf("    RPM:\n");
    g_OLED.printf("     mA:\n");
    g_OLED.printf(" Max mA:\n");

    uint32_t prevTime = micros();
    uint32_t prevDumpTime = prevTime;
    while (true)
    {
        // Delay specified amount of time between iterations of the loop.
        uint32_t currTime;
        uint32_t elapsedTime;
        do
        {
            if (g_dbg)
            {
                return;
            }

            currTime = micros();
            elapsedTime = currTime - prevTime;
        } while (elapsedTime < delay_us);
        prevTime = currTime;

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
        // UNDONE: Fixing for now.
        //power = 20;

        DifferentialDrive::Values drivePower = { .left = power, .right = power };
        g_drive.setPower(drivePower);

        DifferentialDrive::DriveStats stats = g_drive.getStats();
        // Check to see if either motor controller has detected a fault (over voltage, over current, over temp, etc).
        if (stats.faultDetected != DifferentialDrive::NEITHER)
        {
            hangOnError("Motor fault detected!");
        }
        // Check for motor currents measured by ADC that are too high (software over current detection).
        if (stats.overcurrentDetected != DifferentialDrive::NEITHER)
        {
            hangOnError("Motor over current detected!");
        }

        // Display PWM rate, encoder rate, motor current.
        g_OLED.setCursor(8*6, 0*8);
            g_OLED.printf("% 4ld", power);
        g_OLED.setCursor(8*6, 1*8);
            g_OLED.printf("% 4ld,% 4ld", (int32_t)stats.velocityActual.left, (int32_t)stats.velocityActual.right);
        g_OLED.setCursor(8*6, 2*8);
            g_OLED.printf("% 4ld,% 4ld", stats.current_mA.left, stats.current_mA.right);
        g_OLED.setCursor(8*6, 3*8);
            g_OLED.printf("% 4ld,% 4ld\n", stats.maxCurrent_mA.left, stats.maxCurrent_mA.right);
        g_OLED.refresh();

        // Dump frame count once every 10 seconds.
        count++;
        currTime = micros();
        elapsedTime = currTime - prevDumpTime;
        if (elapsedTime >= 10*1000000)
        {
            printf("%lu\n", count);
            prevDumpTime = currTime;
            count = 0;
        }
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

    g_drive.disable();
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
    g_drive.disable();
}

static void leavingDebuggerHook(void* pvContext)
{
    if (g_mriDisableHooks)
    {
        return;
    }

    // Can re-enable the motors now.
    g_drive.enable();
}


// Required to get C++ code to build.
extern "C" void __cxa_pure_virtual()
{
    ASSERT ( 0 );
    abort();
}
