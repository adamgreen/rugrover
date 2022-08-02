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
#include <math.h>
#include <app_util_platform.h>
#include <nrf_delay.h>
#include <nrf_gpio.h>
#include <core/mri.h>
#include <OLED_SH1106/OLED_SH1106.h>
#include <DifferentialDrive/DifferentialDrive.h>
#include <Navigate/Navigate.h>



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

// The next 3 values are found through the calibration process and will be different for each bot and can even
// change over time.
// * The distance between the wheels in mm.
#define WHEEL_BASELINE      128.1868f
// * The diameter of the left wheel in mm.
#define LEFT_WHEEL_DIAMETER 80.65f
// * The diameter of the right wheel as a ratio of the left wheel (will be found during calibration).
#define RIGHT_WHEEL_RATIO   1.0f

// The calculated diameter of the right wheel based on left wheel and preceding ratio.
#define RIGHT_WHEEL_DIAMETER    (RIGHT_WHEEL_RATIO * LEFT_WHEEL_DIAMETER)

// The frequency to be used for the motor PWM signal.
#define MOTOR_PWM_FREQUENCY 20000

// Don't allow motor current to go over 2.0A.
#define MOTOR_MAX_CURRENT_mA    2000

/* PID contants used for the motors.
        ΔPV=15.500001
        ΔCO=30.000000
        Kp=0.516667
        Tp=60028 us
        ϴp=10004 us
        Aggressive Kc=9.678014
        Moderate Kc=1.480167
        Conservative Kc=0.156291
        Ti=0.065030 s
        Td=0.004617 s
*/
#define SPEED_PID_Kc            1.480167f
#define SPEED_PID_Ti            0.065030f
#define SPEED_PID_Td            0.004617f

/* Contants used for the navigation heading PI loop.
    slope1 = 3.014416
    slope2 = -3.071422
    CO1 = 0.250000
    CO2 = -0.250000
    Kp* = 12.171677
    Өp = 0.112236
    Tc = 0.336708
    Kc = 0.320254
    Ti = 0.785653
*/
#define HEADING_PID_Kc            0.320254f
#define HEADING_PID_Ti            0.801193f
#define HEADING_PID_Td            0.785653f

/* Constants used for the navigation distance PI loop.
    slope1 = 113.845070
    slope2 = -117.282158
    CO1 = 0.250000
    CO2 = -0.250000
    Kp* = 462.254456
    Өp = 0.130037
    Tc = 0.390112
    Kc = 0.007278
    Ti = 0.910261
*/
#define DISTANCE_PID_Kc            0.007278f
#define DISTANCE_PID_Ti            0.910261f
#define DISTANCE_PID_Td            0.0f

// PID Frequency in Hz.
#define MOTOR_PID_FREQUENCY     100

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
    DEBUG_NAVIGATE = 1,
    DEBUG_PID = 2,
    DEBUG_PID_SPEED_CALIBRATE = 3,
    DEBUG_PID_HEADING_CALIBRATE = 4,
    DEBUG_PID_DISTANCE_CALIBRATE = 5,
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
                                     SPEED_PID_Kc, SPEED_PID_Ti, SPEED_PID_Td, MOTOR_PID_MAX_PERCENT,
                                     &g_adc,
                                     &g_PWM, MOTOR_PWM_FREQUENCY, MOTOR_PID_FREQUENCY, MOTOR_MAX_CURRENT_mA);

// Navigation module.
static Navigate             g_navigate(&g_drive, MOTOR_PID_FREQUENCY, MOTOR_TICKS_PER_REV,
                                       LEFT_WHEEL_DIAMETER, RIGHT_WHEEL_DIAMETER, WHEEL_BASELINE,
                                       HEADING_PID_Kc, HEADING_PID_Ti, HEADING_PID_Td,
                                       DISTANCE_PID_Kc, DISTANCE_PID_Ti, DISTANCE_PID_Td);

// If this global gets set to non-zero from debugger then enter debug menu.
static int           g_dbg = 0;

// Which debug routine should be executed.
static DebugRoutines g_dbgRoutine = DEBUG_MAX;



static uint32_t displayDebugMenuAndGetOption(const char* pOptionsString, uint32_t maxOption);
static void testNavigateRoutine();
static float getFloatOption(const char* pDescription, float defaultVal);
static void testPidRoutine();
static void sleep_us(uint32_t* pPrevTime, uint32_t delay);
static void testPidSpeedCalibrateRoutine();
static void testPidHeadingCalibrateRoutine();
static void testPidDistanceCalibrateRoutine();
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

    // The software quadrature decoder uses GPIOTE so initialize it.
    nrf_drv_gpiote_init();

    // Run software quadrature decoder GPIOTE ISR at the highest app priority.
    NVIC_SetPriority(GPIOTE_IRQn, _PRIO_APP_HIGH);

    // Initialize the ADC object which scans all of the configured ADC channels as often as possible (~200kHz).
    bool result;
    result = g_adc.init();
    ASSERT ( result );

    // Initialize the closed loop different drive system.
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
            case DEBUG_NAVIGATE:
                testNavigateRoutine();
            case DEBUG_PID:
                testPidRoutine();
                break;
            case DEBUG_PID_SPEED_CALIBRATE:
                testPidSpeedCalibrateRoutine();
                break;
            case DEBUG_PID_HEADING_CALIBRATE:
                testPidHeadingCalibrateRoutine();
                break;
            case DEBUG_PID_DISTANCE_CALIBRATE:
                testPidDistanceCalibrateRoutine();
                break;
            default:
                break;
        }

        g_dbgRoutine = (DebugRoutines)displayDebugMenuAndGetOption(
                                        "1. Test Navigate\n"
                                        "2. Test PID\n"
                                        "3. Run Speed PID calibration\n"
                                        "4. Run Heading PID calibration\n"
                                        "5. Run Distance PID calibration\n", DEBUG_MAX);
    }
}

static uint32_t displayDebugMenuAndGetOption(const char* pOptionsString, uint32_t maxOption)
{
    g_OLED.clearScreen();
    g_OLED.printf("Debug Menu in GDB");
    g_OLED.refreshAndBlock();
    while (true)
    {
        uint32_t choice = 0;
        char     buffer[16];

        printf("\n\nDebug Menu\n");
        printf("%s", pOptionsString);
        printf("\nSelect: ");
        fgets(buffer, sizeof(buffer), stdin);
        choice = strtoul(buffer, NULL, 10);
        if (choice < 1 || choice >= maxOption)
        {
            printf("Invalid selection\n");
        }
        else
        {
            return choice;
        }
    }
}

static void testNavigateRoutine()
{
    float leftWheelDiameter = LEFT_WHEEL_DIAMETER;
    float rightWheelDiameter = RIGHT_WHEEL_DIAMETER;
    float wheelbase = WHEEL_BASELINE;

    while (true)
    {
        printf("\n\n"
               "Update odometry settings.\n"
               "Just press <Return> to keep current setting.\n");

        leftWheelDiameter = getFloatOption("Left Wheel Diameter in mm", leftWheelDiameter);
        rightWheelDiameter = getFloatOption("Right Wheel Diameter in mm", rightWheelDiameter);
        wheelbase = getFloatOption("Wheel base in mm", wheelbase);

        // Enter main action loop.
        const float    runSpeed_mps = 0.25f;
        const uint32_t delayms = 1000 / MOTOR_PID_FREQUENCY;
        const uint32_t delay_us = delayms * 1000;
        uint32_t count = 0;

        // Setup the fixed heading text on the OLED.
        g_OLED.clearScreen();
        g_OLED.printf("  Pos X:\n");
        g_OLED.printf("  Pos Y:\n");
        g_OLED.printf("Heading:\n");
        g_OLED.printf("  Ticks:\n");
        g_OLED.printf(" Max mA:\n");

        Navigate::Position waypoints[] =
        {
            { .x = 0.0f, .y = 500.0f, .heading = NAN }
        };

        g_navigate.reset();
        g_navigate.setParameters(leftWheelDiameter, rightWheelDiameter, wheelbase);
        g_navigate.setWaypoints(waypoints, ARRAY_SIZE(waypoints));
        g_navigate.setWaypointVelocity(runSpeed_mps);
        g_drive.enable();
        uint32_t prevTime = micros();
        uint32_t prevDumpTime = micros();
        while (true)
        {
            // Delay specified amount of time between iterations of the loop.
            sleep_us(&prevTime, delay_us);
            if (g_dbg)
            {
                return;
            }

            g_navigate.update();

            DifferentialDrive::DriveStats stats = g_drive.getStats();
            // Check to see if either motor controller has detected a fault (over voltage, over current, over temp, etc).
            if (stats.faultDetected != DriveBits::NEITHER)
            {
                hangOnError("Motor fault detected!");
            }
            // Check for motor currents measured by ADC that are too high (software over current detection).
            if (stats.overcurrentDetected != DriveBits::NEITHER)
            {
                hangOnError("Motor over current detected!");
            }
            Navigate::Position position = g_navigate.getCurrentPosition();

            // Display position in mm, accumulated encoder ticks, and max current seen by each motor.
            g_OLED.setCursor(8*6, 0*8);  g_OLED.printf("%7.1f", position.x);
            g_OLED.setCursor(8*6, 1*8);  g_OLED.printf("%7.1f", position.y);
            g_OLED.setCursor(8*6, 2*8);  g_OLED.printf("%6.1f", RADIAN_TO_DEGREE*position.heading);
            g_OLED.setCursor(8*6, 3*8);  g_OLED.printf("%5ld,%5ld", stats.encoderCount.left, stats.encoderCount.right);
            g_OLED.setCursor(8*6, 4*8);  g_OLED.printf("%4ld,%4ld\n", stats.maxCurrent_mA.left, stats.maxCurrent_mA.right);
            g_OLED.refresh();

            if (g_navigate.hasReachedFinalWaypoint())
            {
                g_OLED.refreshAndBlock();
                g_navigate.reset();
                break;
            }

            // Dump frame count once every 10 seconds.
            count++;
            uint32_t currTime = micros();
            uint32_t elapsedTime = currTime - prevDumpTime;
            if (elapsedTime >= 10*1000000)
            {
                printf("%lu\n", count);
                prevDumpTime = currTime;
                count = 0;
            }
        }
    }
}

static float getFloatOption(const char* pDescription, float defaultVal)
{
    char input[32];

    printf("%s (%f): ", pDescription, defaultVal);
    fgets(input, sizeof(input), stdin);
    if (input[0] == '\n')
    {
        return defaultVal;
    }
    return strtof(input, NULL);
}

static void testPidRoutine()
{
    // Enter main action loop.
    const uint32_t delayms = 1000 / MOTOR_PID_FREQUENCY;
    const uint32_t delay_us = delayms * 1000;
    const int32_t velocityLimit = 25;
    const int32_t velocityStep = 5;
    const uint32_t velocityDelayms = 2000;
    int32_t velocity = 0;
    int32_t velocityInc = velocityStep;
    uint32_t velocityIter = 0;
    uint32_t count = 0;

    // Setup the fixed heading text on the OLED.
    g_OLED.clearScreen();
    g_OLED.printf("   Set:\n");
    g_OLED.printf(" Ticks:\n");
    g_OLED.printf(" Power:\n");
    g_OLED.printf("    mA:\n");
    g_OLED.printf("Max mA:\n");
    g_OLED.printf("   mAh:\n");

    uint32_t prevTime = micros();
    uint32_t prevDumpTime = prevTime;
    while (true)
    {
        // Delay specified amount of time between iterations of the loop.
        sleep_us(&prevTime, delay_us);
        if (g_dbg)
        {
            return;
        }

        // Every so often the motor velocity should be changed so that it oscillates back and forth between the set
        // positive and negative limits.
        if (++velocityIter >= velocityDelayms/delayms)
        {
            // Update motor velocity.
            velocity += velocityInc;
            if (abs(velocity) > velocityLimit)
            {
                velocityInc = -velocityInc;
                velocity += 2*velocityInc;
            }

            // Toggle the LED to show progress.
            nrf_gpio_pin_toggle(BLINK_LED_PIN);

            velocityIter = 0;
        }

        DriveFloatValues driveVelocity = { .left = (float)velocity, .right = (float)velocity };
        g_drive.setVelocity(driveVelocity);

        DifferentialDrive::DriveStats stats = g_drive.getStats();
        // Check to see if either motor controller has detected a fault (over voltage, over current, over temp, etc).
        if (stats.faultDetected != DriveBits::NEITHER)
        {
            hangOnError("Motor fault detected!");
        }
        // Check for motor currents measured by ADC that are too high (software over current detection).
        if (stats.overcurrentDetected != DriveBits::NEITHER)
        {
            hangOnError("Motor over current detected!");
        }

        // Display PWM rate, encoder rate, motor current.
        g_OLED.setCursor(7*6, 0*8);
            g_OLED.printf("%4ld", velocity);
        g_OLED.setCursor(7*6, 1*8);
            g_OLED.printf("%4ld,%4ld", stats.velocityActual.left, stats.velocityActual.right);
        g_OLED.setCursor(7*6, 2*8);
            g_OLED.printf("%4ld,%4ld", stats.power.left, stats.power.right);
        g_OLED.setCursor(7*6, 3*8);
            g_OLED.printf("%4ld,%4ld", stats.current_mA.left, stats.current_mA.right);
        g_OLED.setCursor(7*6, 4*8);
            g_OLED.printf("%4ld,%4ld\n", stats.maxCurrent_mA.left, stats.maxCurrent_mA.right);
        g_OLED.setCursor(7*6, 5*8);
            g_OLED.printf("%4ld\n", (int32_t)(stats.capacityUsed_mAh+0.5f));
        g_OLED.refresh();

        // Dump frame count once every 10 seconds.
        count++;
        uint32_t currTime = micros();
        uint32_t elapsedTime = currTime - prevDumpTime;
        if (elapsedTime >= 10*1000000)
        {
            printf("%lu\n", count);
            prevDumpTime = currTime;
            count = 0;
        }
    }
}

static void sleep_us(uint32_t* pPrevTime, uint32_t delay)
{
    // Delay specified amount of time.
    uint32_t prevTime = *pPrevTime;
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
    } while (elapsedTime < delay);
    *pPrevTime = currTime;
}

static void testPidSpeedCalibrateRoutine()
{
    // Can try the right motor as well and compare.
    #define PID_CALIBRATE_MOTOR left

    // Parameters that can be tweaked for this calibration.
    const int32_t  initialPower = 20;
    const int32_t  bumpPower = 50;
    const uint32_t initialSampleCount = 200;
    const uint32_t bumpSampleCount = 200;
    const uint32_t delayms = 1000 / MOTOR_PID_FREQUENCY;
    const uint32_t delay_us = delayms * 1000;

    // Let the user know that we are running the PID calibration routine.
    g_OLED.clearScreen();
    g_OLED.printf("Speed Calibration");
    g_OLED.refreshAndBlock();

    // Count down for 10 seconds before actually running the test.
    uint32_t prevTime = micros();
    for (int i = 10 ; i > 0 ; i--)
    {
        g_OLED.setCursor(0, 2*8);
            g_OLED.printf("%2d", i);
        g_OLED.refresh();
        sleep_us(&prevTime, 1000000);
    }
    g_OLED.setCursor(0, 2*8);
        g_OLED.printf("Running...");
    g_OLED.refreshAndBlock();

    // Enter main action loop.
    int32_t power = initialPower;
    struct SamplePoints
    {
        uint32_t usec;
        int32_t  ticks;
    } samples[initialSampleCount+bumpSampleCount];
    prevTime = micros();
    for (size_t i = 0 ; i < ARRAY_SIZE(samples) ; i++)
    {
        sleep_us(&prevTime, delay_us);
        if (g_dbg)
        {
            return;
        }

        // Step up motor power at the desired sample interval.
        if (i >= initialSampleCount)
        {
            power = bumpPower;
        }

        DriveValues drivePower = { .left = power, .right = power };
        g_drive.setPower(drivePower);

        DifferentialDrive::DriveStats stats = g_drive.getStats();
        // Check to see if either motor controller has detected a fault (over voltage, over current, over temp, etc).
        if (stats.faultDetected != DriveBits::NEITHER)
        {
            hangOnError("Motor fault detected!");
        }
        // Check for motor currents measured by ADC that are too high (software over current detection).
        if (stats.overcurrentDetected != DriveBits::NEITHER)
        {
            hangOnError("Motor over current detected!");
        }

        // Record this sample to RAM for later dumping.
        samples[i].usec = prevTime;
        samples[i].ticks = stats.velocityActual.PID_CALIBRATE_MOTOR;
    }
    g_drive.disable();

    g_OLED.setCursor(0, 2*8);
        g_OLED.printf("Saving calibration data... ");
    g_OLED.refreshAndBlock();

    // Output the results to a file where GDB is running on the host.
    FILE* pFile = fopen("speed_calibrate.csv", "w");
    if (pFile == NULL)
    {
        hangOnError("error: Failed to open speed_calibrate.csv");
        return;
    }
    uint32_t baseTime = samples[0].usec;
    fprintf(pFile, "usec,power,ticks\n");
    for (size_t i = 0 ; i < ARRAY_SIZE(samples) ; i++)
    {
        int32_t power = initialPower;
        if (i >= initialSampleCount)
        {
            power = bumpPower;
        }

        fprintf(pFile, "%lu,%ld,%ld\n", samples[i].usec-baseTime, power, samples[i].ticks);
    }
    fclose(pFile);

    // Calculate the model parameters from the sampled data.
    // Average of PV value before CO is changed.
    float sum = 0.0f;
    for (size_t i = initialSampleCount-10 ; i < initialSampleCount ; i++)
    {
        sum += (float)samples[i].ticks;
    }
    float lowPV = sum / 10.0f;
    // Average of PV value after CO is changed, the last few samples.
    sum = 0.0f;
    for (size_t i = ARRAY_SIZE(samples)-10 ; i < ARRAY_SIZE(samples) ; i++)
    {
        sum += (float)samples[i].ticks;
    }
    float highPV = sum / 10.0f;
    // Can now calculate deltaPV.
    float deltaPV = highPV - lowPV;
    // deltaCO is difference between high and low power settings.
    float deltaCO = bumpPower - initialPower;
    float Kp = deltaPV / deltaCO;
    // Calculate PV value at which 63% of the deltaPV has occurred.
    float limitPV = lowPV + 0.63f * deltaPV;
    // Search the samples after the bump to find when PV starts changing to find dead time and when it reaches 63% of
    // deltaPV.
    bool foundDeadTime = false;
    uint32_t startTime = samples[initialSampleCount].usec;
    uint32_t deadTime = 0;
    uint32_t Tp = 0;
    for (size_t i = initialSampleCount ; i < ARRAY_SIZE(samples) ; i++)
    {
        if (!foundDeadTime && samples[i].ticks > lowPV + 1.0)
        {
            foundDeadTime = true;
            deadTime = samples[i].usec - startTime;
        }
        else if (foundDeadTime && samples[i].ticks >= limitPV)
        {
            Tp = samples[i].usec - startTime - deadTime;
            break;
        }
    }

    printf("\nCalculated PID Model Parameters\n");
    printf("ΔPV=%f\n", deltaPV);
    printf("ΔCO=%f\n", deltaCO);
    printf("Kp=%f\n", Kp);
    printf("Tp=%lu\n", Tp);
    printf("ϴp=%lu\n", deadTime);
    float Tc;
    if (Tp > 8*deadTime)
    {
        Tc = Tp;
    }
    else
    {
        Tc = 8*deadTime;
    }
    float KcAggressive = (1.0f / Kp) * ((Tp + 0.5f*deadTime) / (Tc/10.0f + 0.5f*deadTime));
    float KcModerate = (1.0f / Kp) * ((Tp + 0.5f*deadTime) / (Tc + 0.5f*deadTime));
    float KcConservative = (1.0f / Kp) * ((Tp + 0.5f*deadTime) / (Tc*10.0f + 0.5f*deadTime));
    float Ti = Tp + 0.5f*deadTime;
    float Td = (Tp * (float)deadTime) / (2.0f*Tp + deadTime);
    printf("Aggressive Kc=%f\n", KcAggressive);
    printf("Moderate Kc=%f\n", KcModerate);
    printf("Conservative Kc=%f\n", KcConservative);
    printf("Ti=%f\n", Ti/1000000.0f);
    printf("Td=%f\n", Td/1000000.0f);
}

static void testPidHeadingCalibrateRoutine()
{
    // Parameters that can be tweaked for this calibration.
    const float    testVelocity_mps = 0.25f;
    const float    maxAngle = M_PI / 4.0f;
    const float    minAngle = -maxAngle;
    const size_t   sampleCount = 1000;
    const uint32_t delayms = 1000 / MOTOR_PID_FREQUENCY;
    const uint32_t delay_us = delayms * 1000;

    // Let the user know that we are running the PID calibration routine.
    g_OLED.clearScreen();
    g_OLED.printf("Heading Calibration");
    g_OLED.refreshAndBlock();

    // Count down for 10 seconds before actually running the test.
    uint32_t prevTime = micros();
    for (int i = 10 ; i > 0 ; i--)
    {
        g_OLED.setCursor(0, 2*8);
            g_OLED.printf("%2d", i);
        g_OLED.refresh();
        sleep_us(&prevTime, 1000000);
    }
    g_OLED.setCursor(0, 2*8);
        g_OLED.printf("Running...");
    g_OLED.refreshAndBlock();

    // Enter main action loop.
    float velocity = testVelocity_mps;
    enum Direction
    {
        CLOCKWISE,
        COUNTERCLOCKWISE
    } direction = CLOCKWISE;
    struct SamplePoints
    {
        uint32_t usec;
        float    heading;
        float    velocity;
    } samples[sampleCount];
    g_drive.enable();
    prevTime = micros();
    for (size_t i = 0 ; i < ARRAY_SIZE(samples) ; i++)
    {
        sleep_us(&prevTime, delay_us);
        if (g_dbg)
        {
            return;
        }

        // Update navigation module.
        DriveFloatValues velocities =
        {
            .left = velocity,
            .right = -velocity
        };
        g_navigate.drive(velocities);
        Navigate::Position position = g_navigate.getCurrentPosition();

        // Record this sample to RAM for later dumping.
        samples[i].usec = prevTime;
        samples[i].heading = position.heading;
        samples[i].velocity = velocity;

        DifferentialDrive::DriveStats stats = g_drive.getStats();
        // Check to see if either motor controller has detected a fault (over voltage, over current, over temp, etc).
        if (stats.faultDetected != DriveBits::NEITHER)
        {
            hangOnError("Motor fault detected!");
        }
        // Check for motor currents measured by ADC that are too high (software over current detection).
        if (stats.overcurrentDetected != DriveBits::NEITHER)
        {
            hangOnError("Motor over current detected!");
        }

        // Keep rotating until the desired angle limit has been hit or exceeded.
        switch (direction)
        {
        case CLOCKWISE:
            if (position.heading >= maxAngle)
            {
                direction = COUNTERCLOCKWISE;
                velocity = -testVelocity_mps;
            }
            break;
        case COUNTERCLOCKWISE:
            if (position.heading <= minAngle)
            {
                direction = CLOCKWISE;
                velocity = testVelocity_mps;
            }
            break;
        }
    }
    g_drive.disable();

    g_OLED.setCursor(0, 2*8);
        g_OLED.printf("Saving samples... ");
    g_OLED.refreshAndBlock();

    // Output the results to a file where GDB is running on the host.
    FILE* pFile = fopen("heading_calibrate.csv", "w");
    if (pFile == NULL)
    {
        hangOnError("error: Failed to open heading_calibrate.csv");
        return;
    }
    uint32_t baseTime = samples[0].usec;
    fprintf(pFile, "usec,velocity,heading\n");
    for (size_t i = 0 ; i < ARRAY_SIZE(samples) ; i++)
    {
        fprintf(pFile, "%lu,%f,%f\n", samples[i].usec-baseTime, samples[i].velocity, samples[i].heading);
    }
    fclose(pFile);

    enum SlopeState
    {
        SLOPE1_START_SEARCH,
        SLOPE1_END_SEARCH,
        SLOPE2_START_SEARCH,
        SLOPE2_END_SEARCH
    };
    SlopeState slopeState = SLOPE1_START_SEARCH;
    struct Stats
    {
        float      startTime;
        float      startAngle;
        float      sum;
        uint32_t   count;
    };
    Stats slope1 = {0.0f, 0.0f, 0.0f, 0};
    Stats slope2 = {0.0f, 0.0f, 0.0f, 0};
    Stats deadTime = {0.0f, 0.0f, 0.0f, 0};
    float minSpeed = 3.0f * M_PI;
    float maxSpeed = -3.0f * M_PI;

    // Set previous angle to a value low enough that first element will be detected as start of slope1.
    float prevAngle = -100.0f;
    int32_t prevSpeed = 100;
    for (size_t i = 0 ; i < ARRAY_SIZE(samples) ; i++)
    {
        uint32_t intTime = samples[i].usec;
        float    angle = samples[i].heading;
        float    speed = samples[i].velocity;

        float time = (float)intTime / 1000000.0f;

        // Track the min and maximum speed used during test.
        if (speed < minSpeed)
            minSpeed = speed;
        if (speed > maxSpeed)
            maxSpeed = speed;

        // Find the ends of slope1 & slope2.
        switch (slopeState)
        {
        case SLOPE1_START_SEARCH:
            if (angle > prevAngle)
            {
                slope1.startTime = time;
                slope1.startAngle = angle;
                if (deadTime.startTime != 0.0f)
                {
                    deadTime.sum += time - deadTime.startTime;
                    deadTime.count++;
                }
                slopeState = SLOPE1_END_SEARCH;
            }
            break;
        case SLOPE1_END_SEARCH:
            if (prevSpeed > speed)
            {
                slope1.sum += (angle - slope1.startAngle) / (time - slope1.startTime);
                slope1.count++;
                deadTime.startTime = time;
                slopeState = SLOPE2_START_SEARCH;
            }
            break;
        case SLOPE2_START_SEARCH:
            if (angle < prevAngle)
            {
                slope2.startTime = time;
                slope2.startAngle = angle;
                if (deadTime.startTime != 0.0f)
                {
                    deadTime.sum += time - deadTime.startTime;
                    deadTime.count++;
                }
                slopeState = SLOPE2_END_SEARCH;
            }
            break;
        case SLOPE2_END_SEARCH:
            if (prevSpeed < speed)
            {
                slope2.sum += (angle - slope2.startAngle) / (time - slope2.startTime);
                slope2.count++;
                deadTime.startTime = time;
                slopeState = SLOPE1_START_SEARCH;
            }
            break;
        }
        prevAngle = angle;
        prevSpeed = speed;
    }

    float slope1Val = slope1.sum / (float)slope1.count;
    float slope2Val = slope2.sum / (float)slope2.count;
    float deadTimeVal = deadTime.sum / (float)deadTime.count;
    float CO1 = maxSpeed;
    float CO2 = minSpeed;
    float Kp_star = (slope2Val - slope1Val) / (float)(CO2 - CO1);
    float Tc = 3.0f * deadTimeVal;
    float Kc = (1.0f / Kp_star) * ((2 * Tc + deadTimeVal) / powf(Tc + deadTimeVal, 2.0f));
    float Ti = 2 * Tc + deadTimeVal;

    printf("\n\nCalculated PID Model Parameters\n");
    printf("slope1 = %f\n", slope1Val);
    printf("slope2 = %f\n", slope2Val);
    printf("CO1 = %f\n", CO1);
    printf("CO2 = %f\n", CO2);
    printf("Kp* = %f\n", Kp_star);
    printf("Өp = %f\n", deadTimeVal);
    printf("Tc = %f\n", Tc);
    printf("Kc = %f\n", Kc);
    printf("Ti = %f\n", Ti);
}

static void testPidDistanceCalibrateRoutine()
{
    // Parameters that can be tweaked for this calibration.
    const float    testVelocity_mps = 0.25f;
    const float    maxDistance = M_PI * LEFT_WHEEL_DIAMETER * 0.5;
    const float    minDistance = -maxDistance;
    const size_t   sampleCount = 2000;
    const uint32_t delayms = 1000 / MOTOR_PID_FREQUENCY;
    const uint32_t delay_us = delayms * 1000;

    // Let the user know that we are running the PID calibration routine.
    g_OLED.clearScreen();
    g_OLED.printf("Distance Calibration");
    g_OLED.refreshAndBlock();

    // Count down for 10 seconds before actually running the test.
    uint32_t prevTime = micros();
    for (int i = 10 ; i > 0 ; i--)
    {
        g_OLED.setCursor(0, 2*8);
            g_OLED.printf("%2d", i);
        g_OLED.refresh();
        sleep_us(&prevTime, 1000000);
    }
    g_OLED.setCursor(0, 2*8);
        g_OLED.printf("Running...");
    g_OLED.refreshAndBlock();

    // Enter main action loop.
    float velocity = testVelocity_mps;
    enum Direction
    {
        FORWARD,
        REVERSE
    } direction = FORWARD;
    struct SamplePoints
    {
        uint32_t usec;
        float    distance;
        float    velocity;
    } samples[sampleCount];
    g_drive.enable();
    prevTime = micros();
    for (size_t i = 0 ; i < ARRAY_SIZE(samples) ; i++)
    {
        sleep_us(&prevTime, delay_us);
        if (g_dbg)
        {
            return;
        }

        // Update navigation module.
        DriveFloatValues velocities =
        {
            .left = velocity,
            .right = velocity
        };
        g_navigate.drive(velocities);
        Navigate::Position position = g_navigate.getCurrentPosition();

        // Record this sample to RAM for later dumping.
        samples[i].usec = prevTime;
        samples[i].distance = position.y;
        samples[i].velocity = velocity;

        DifferentialDrive::DriveStats stats = g_drive.getStats();
        // Check to see if either motor controller has detected a fault (over voltage, over current, over temp, etc).
        if (stats.faultDetected != DriveBits::NEITHER)
        {
            hangOnError("Motor fault detected!");
        }
        // Check for motor currents measured by ADC that are too high (software over current detection).
        if (stats.overcurrentDetected != DriveBits::NEITHER)
        {
            hangOnError("Motor over current detected!");
        }

        // Keep rotating until the desired angle limit has been hit or exceeded.
        switch (direction)
        {
        case FORWARD:
            if (position.y >= maxDistance)
            {
                direction = REVERSE;
                velocity = -testVelocity_mps;
            }
            break;
        case REVERSE:
            if (position.y <= minDistance)
            {
                direction = FORWARD;
                velocity = testVelocity_mps;
            }
            break;
        }
    }
    g_drive.disable();

    g_OLED.setCursor(0, 2*8);
        g_OLED.printf("Saving samples... ");
    g_OLED.refreshAndBlock();

    // Output the results to a file where GDB is running on the host.
    FILE* pFile = fopen("distance_calibrate.csv", "w");
    if (pFile == NULL)
    {
        hangOnError("error: Failed to open distance_calibrate.csv");
        return;
    }
    uint32_t baseTime = samples[0].usec;
    fprintf(pFile, "usec,speed,distance\n");
    for (size_t i = 0 ; i < ARRAY_SIZE(samples) ; i++)
    {
        fprintf(pFile, "%lu,%f,%f\n", samples[i].usec-baseTime, samples[i].velocity, samples[i].distance);
    }
    fclose(pFile);

    enum SlopeState
    {
        SLOPE1_START_SEARCH,
        SLOPE1_END_SEARCH,
        SLOPE2_START_SEARCH,
        SLOPE2_END_SEARCH
    };
    SlopeState slopeState = SLOPE1_START_SEARCH;
    struct Stats
    {
        float      startTime;
        float      startDistance;
        float      sum;
        uint32_t   count;
    };
    Stats slope1 = {0.0f, 0.0f, 0.0f, 0};
    Stats slope2 = {0.0f, 0.0f, 0.0f, 0};
    Stats deadTime = {0.0f, 0.0f, 0.0f, 0};
    float minSpeed = 3.0f * M_PI;
    float maxSpeed = -3.0f * M_PI;

    // Set previous angle to a value low enough that first element will be detected as start of slope1.
    float prevDistance = -100.0f;
    int32_t prevSpeed = 100;
    for (size_t i = 0 ; i < ARRAY_SIZE(samples) ; i++)
    {
        uint32_t intTime = samples[i].usec;
        float    distance = samples[i].distance;
        float    speed = samples[i].velocity;

        float time = (float)intTime / 1000000.0f;

        // Track the min and maximum speed used during test.
        if (speed < minSpeed)
            minSpeed = speed;
        if (speed > maxSpeed)
            maxSpeed = speed;

        // Find the ends of slope1 & slope2.
        switch (slopeState)
        {
        case SLOPE1_START_SEARCH:
            if (distance > prevDistance)
            {
                slope1.startTime = time;
                slope1.startDistance = distance;
                if (deadTime.startTime != 0.0f)
                {
                    deadTime.sum += time - deadTime.startTime;
                    deadTime.count++;
                }
                slopeState = SLOPE1_END_SEARCH;
            }
            break;
        case SLOPE1_END_SEARCH:
            if (prevSpeed > speed)
            {
                slope1.sum += (distance - slope1.startDistance) / (time - slope1.startTime);
                slope1.count++;
                deadTime.startTime = time;
                slopeState = SLOPE2_START_SEARCH;
            }
            break;
        case SLOPE2_START_SEARCH:
            if (distance < prevDistance)
            {
                slope2.startTime = time;
                slope2.startDistance = distance;
                if (deadTime.startTime != 0.0f)
                {
                    deadTime.sum += time - deadTime.startTime;
                    deadTime.count++;
                }
                slopeState = SLOPE2_END_SEARCH;
            }
            break;
        case SLOPE2_END_SEARCH:
            if (prevSpeed < speed)
            {
                slope2.sum += (distance - slope2.startDistance) / (time - slope2.startTime);
                slope2.count++;
                deadTime.startTime = time;
                slopeState = SLOPE1_START_SEARCH;
            }
            break;
        }
        prevDistance = distance;
        prevSpeed = speed;
    }

    float slope1Val = slope1.sum / (float)slope1.count;
    float slope2Val = slope2.sum / (float)slope2.count;
    float deadTimeVal = deadTime.sum / (float)deadTime.count;
    float CO1 = maxSpeed;
    float CO2 = minSpeed;
    float Kp_star = (slope2Val - slope1Val) / (float)(CO2 - CO1);
    float Tc = 3.0f * deadTimeVal;
    float Kc = (1.0f / Kp_star) * ((2 * Tc + deadTimeVal) / powf(Tc + deadTimeVal, 2.0f));
    float Ti = 2 * Tc + deadTimeVal;

    printf("\n\nCalculated PID Model Parameters\n");
    printf("slope1 = %f\n", slope1Val);
    printf("slope2 = %f\n", slope2Val);
    printf("CO1 = %f\n", CO1);
    printf("CO2 = %f\n", CO2);
    printf("Kp* = %f\n", Kp_star);
    printf("Өp = %f\n", deadTimeVal);
    printf("Tc = %f\n", Tc);
    printf("Kc = %f\n", Kc);
    printf("Ti = %f\n", Ti);
}

static void hangOnError(const char* pErrorMessage)
{
    printf("%s\r\n", pErrorMessage);

    g_OLED.clearScreen();
    g_OLED.printf("%s", pErrorMessage);
    g_OLED.refreshAndBlock();

    g_drive.disable();
    while (true)
    {
        // Loop here forever if a motor fault has been detected.
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
    #else
        if ((NRF_UICR->NFCPINS & UICR_NFCPINS_PROTECT_Msk) != (UICR_NFCPINS_PROTECT_NFC << UICR_NFCPINS_PROTECT_Pos))
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
    #else
        if (((NRF_UICR->PSELRESET[0] & UICR_PSELRESET_CONNECT_Msk) == (UICR_PSELRESET_CONNECT_Connected << UICR_PSELRESET_CONNECT_Pos)) ||
            ((NRF_UICR->PSELRESET[1] & UICR_PSELRESET_CONNECT_Msk) == (UICR_PSELRESET_CONNECT_Connected << UICR_PSELRESET_CONNECT_Pos)))
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
// Remember whether the motors were enabled or not when entering the debugger.
static bool g_motorsWereEnabled = false;

static void enteringDebuggerHook(void* pvContext)
{
    if (g_mriDisableHooks)
    {
        return;
    }

    // Stop the motors when entering the debugger so that the robot doesn't run off.
    g_motorsWereEnabled = g_drive.isEnabled();
    if (g_motorsWereEnabled)
    {
        g_drive.disable();
    }
}

static void leavingDebuggerHook(void* pvContext)
{
    if (g_mriDisableHooks)
    {
        return;
    }

    // Can re-enable the motors now.
    if (g_motorsWereEnabled)
    {
        g_drive.enable();
    }
}


// Required to get C++ code to build.
extern "C" void __cxa_pure_virtual()
{
    ASSERT ( 0 );
    abort();
}
