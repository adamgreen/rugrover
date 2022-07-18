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
// Driver for the closed loop control of RugRover's differential drive.
#include "DifferentialDrive.h"
#include <string.h>


DifferentialDrive::DifferentialDrive(uint8_t leftEnablePin, uint8_t leftPwm1Pin, uint8_t leftPwm2Pin,
                    uint8_t leftDiagPin, uint8_t leftOcmPin, bool leftReverse,
                    uint8_t leftEncoderAPin, uint8_t leftEncoderBPin,
                    uint8_t rightEnablePin, uint8_t rightPwm1Pin, uint8_t rightPwm2Pin,
                    uint8_t rightDiagPin, uint8_t rightOcmPin, bool rightReverse,
                    uint8_t rightEncoderAPin, uint8_t rightEncoderBPin,
                    float pidKc, float pidTi, float pidTd, uint32_t maxMotorPercentage,
                    SAADCScanner* pADC,
                    nrf_drv_pwm_t* pPWM, uint32_t frequency,
                    uint32_t maxCurrent_mA)
: m_leftEncoder(leftEncoderAPin, leftEncoderBPin),
  m_rightEncoder(rightEncoderAPin, rightEncoderBPin),
  m_leftPID(pidKc, pidTi, pidTd, 0.0f, -(float)maxMotorPercentage, (float)maxMotorPercentage, 0.010f),
  m_rightPID(pidKc, pidTi, pidTd, 0.0f, -(float)maxMotorPercentage, (float)maxMotorPercentage, 0.010f),
  m_motors(leftEnablePin, leftPwm1Pin, leftPwm2Pin, leftDiagPin, leftOcmPin, leftReverse,
           rightEnablePin, rightPwm1Pin, rightPwm2Pin, rightDiagPin, rightOcmPin, rightReverse,
           pADC, pPWM, frequency, maxCurrent_mA)
{
    memset(&m_prevTicks, 0, sizeof(m_prevTicks));
    memset(&m_maxCurrents_mA, 0, sizeof(m_maxCurrents_mA));
    memset(&m_prevVelocities, 0, sizeof(m_prevVelocities));
    m_leftReverse = leftReverse;
    m_rightReverse = rightReverse;
}

void DifferentialDrive::updateMotors()
{
    // The encoder counts by default go negative for forward motion so negate them so that they match reversing that I
    // do on the motors.
    int32_t leftTicks = -m_leftEncoder.getCount();
    int32_t rightTicks = -m_rightEncoder.getCount();
    if (m_leftReverse)
    {
        leftTicks = -leftTicks;
    }
    if (m_rightReverse)
    {
        rightTicks = -rightTicks;
    }
    int32_t leftDiff = leftTicks - m_prevTicks.left;
    int32_t rightDiff = rightTicks - m_prevTicks.right;
    m_prevTicks.left = leftTicks;
    m_prevTicks.right = rightTicks;

    float leftVelocity = leftDiff;
    float rightVelocity = rightDiff;

    int32_t leftPower = (int32_t)(m_leftPID.compute(leftVelocity) + 0.5f);
    int32_t rightPower = (int32_t)(m_rightPID.compute(rightVelocity) + 0.5f);
    m_motors.setPower(leftPower, rightPower);

    m_prevVelocities.left = leftVelocity;
    m_prevVelocities.right = rightVelocity;
}

DifferentialDrive::DriveStats DifferentialDrive::getStats()
{
    DriveStats stats;

    stats.velocityRequested.left = m_leftPID.getSetPoint();
    stats.velocityRequested.right = m_rightPID.getSetPoint();
    stats.velocityActual = m_prevVelocities;

    stats.encoderCount.left = m_leftEncoder.getCount();
    stats.encoderCount.right = m_rightEncoder.getCount();
    stats.power.left = m_leftPID.getControlOutput();
    stats.power.right = m_rightPID.getControlOutput();

    stats.current_mA = m_motors.getCurrentReadings();
    if (stats.current_mA.left > m_maxCurrents_mA.left)
    {
        m_maxCurrents_mA.left = stats.current_mA.left;
    }
    if (stats.current_mA.right > m_maxCurrents_mA.right)
    {
        m_maxCurrents_mA.right = stats.current_mA.right;
    }
    stats.maxCurrent_mA = m_maxCurrents_mA;

    stats.faultDetected = m_motors.haveMotorsEncounteredFault();
    stats.overcurrentDetected = m_motors.haveMotorsDetectedCurrentOverload();

    // The way the code is written, if left motor is in auto PID mode then so is the right motor.
    stats.autoMode = m_leftPID.isAutomaticModeEnabled();

    return stats;
}
