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
                    uint32_t encoderTicksPerRevolution,
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
    m_prevSampleTime = 0;
    m_encoderTicksPerRevolution = (float)encoderTicksPerRevolution;
}

void DifferentialDrive::updateMotors()
{
    // The encoder counts by default go negative for forward motion so negate them so that they match reversing that I
    // do on the motors.
    uint32_t currentSampleTime = micros();
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

    int32_t elapsedTime_us = currentSampleTime - m_prevSampleTime;
    m_prevSampleTime = currentSampleTime;

    float leftVelocity = ((float)leftDiff * 60.0f * 1000000.0f) / ((float)elapsedTime_us * m_encoderTicksPerRevolution);
    float rightVelocity = ((float)rightDiff * 60.0f * 1000000.0f) / ((float)elapsedTime_us * m_encoderTicksPerRevolution);

    float elapsedTime = (float)elapsedTime_us / 1000000.0f;
    m_leftPID.setSampleTime(elapsedTime);
    m_rightPID.setSampleTime(elapsedTime);

    int32_t leftPower = m_leftPID.compute(leftVelocity);
    int32_t rightPower = m_rightPID.compute(rightVelocity);
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

    // UNDONE: Motor driver could return Values and Bits for current readings and faults.
    DualTB9051FTGDrivers::CurrentReadings currents = m_motors.getCurrentReadings();
    stats.current_mA.left = currents.leftCurrent_mA;
    stats.current_mA.right = currents.rightCurrent_mA;

    if (currents.leftCurrent_mA > m_maxCurrents_mA.left)
    {
        m_maxCurrents_mA.left = currents.leftCurrent_mA;
    }
    if (currents.rightCurrent_mA > m_maxCurrents_mA.right)
    {
        m_maxCurrents_mA.right = currents.rightCurrent_mA;
    }
    stats.maxCurrent_mA = m_maxCurrents_mA;

    stats.faultDetected = NEITHER;
    if (m_motors.hasLeftMotorEncounteredFault())
    {
        stats.faultDetected = (Bits)(stats.faultDetected | LEFT);
    }
    if (m_motors.hasRightMotorEncounteredFault())
    {
        stats.faultDetected = (Bits)(stats.faultDetected | RIGHT);
    }

    stats.overcurrentDetected = NEITHER;
    if (m_motors.hasLeftMotorDetectedCurrentOverload())
    {
        stats.overcurrentDetected = (Bits)(stats.overcurrentDetected | LEFT);
    }
    if (m_motors.hasRightMotorDetectedCurrentOverload())
    {
        stats.overcurrentDetected = (Bits)(stats.overcurrentDetected | RIGHT);
    }

    // The way the code is written, if left motor is in auto PID mode then so is the right motor.
    stats.autoMode = m_leftPID.isAutomaticModeEnabled();

    return stats;
}
