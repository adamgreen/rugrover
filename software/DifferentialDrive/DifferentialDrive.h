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
#ifndef DIFFERENTIAL_DRIVE_H_
#define DIFFERENTIAL_DRIVE_H_

#include <stdint.h>
#include <nrf_drv_pwm.h>
#include "SAADCScanner/SAADCScanner.h"
#include "QuadratureDecoder/QuadratureDecoderHardware.h"
#include "QuadratureDecoder/QuadratureDecoderSoftware.h"
#include "DualTB9051FTGDrivers/DualTB9051FTGDrivers.h"
#include "GlobalTimer/GlobalTimer.h"
#include "PID/PID.h"

class DifferentialDrive
{
    public:
        DifferentialDrive(uint8_t leftEnablePin, uint8_t leftPwm1Pin, uint8_t leftPwm2Pin,
                          uint8_t leftDiagPin, uint8_t leftOcmPin, bool leftReverse,
                          uint8_t leftEncoderAPin, uint8_t leftEncoderBPin,
                          uint8_t rightEnablePin, uint8_t rightPwm1Pin, uint8_t rightPwm2Pin,
                          uint8_t rightDiagPin, uint8_t rightOcmPin, bool rightReverse,
                          uint8_t rightEncoderAPin, uint8_t rightEncoderBPin,
                          float pidKc, float pidTi, float pidTd, int32_t maxMotorPercentage,
                          SAADCScanner* pADC,
                          nrf_drv_pwm_t* pPWM, uint32_t pwmFrequency_Hz, uint32_t pidFrequency_Hz,
                          uint32_t maxCurrent_mA);

        struct DriveStats
        {
            DriveFloatValues    velocityRequested;
            DriveValues         velocityActual;
            DriveValues         encoderCount;
            DriveValues         power;
            DriveValues         current_mA;
            DriveValues         maxCurrent_mA;
            float               capacityUsed_mAh;
            DriveBits           faultDetected;
            DriveBits           overcurrentDetected;
            bool                autoMode;
        };

        bool init()
        {
            if (m_initWasSuccessful)
            {
                return true;
            }
            m_initWasSuccessful = true;
            m_initWasSuccessful &= m_leftEncoder.init();
            m_initWasSuccessful &= m_rightEncoder.init();
            m_initWasSuccessful &= m_motors.init();

            return m_initWasSuccessful;
        }

        bool isInit()
        {
            return m_initWasSuccessful;
        }

        void enable()
        {
            m_motors.enable(true);
            m_isEnabled = true;
        }

        void disable()
        {
            m_motors.enable(false);
            m_isEnabled = false;
        }

        bool isEnabled()
        {
            return m_isEnabled;
        }

        void setVelocity(DriveFloatValues& ticks)
        {
            m_leftPID.updateSetPoint(ticks.left);
            m_rightPID.updateSetPoint(ticks.right);

            updateMotors();
        }

        void setPower(DriveValues& percentage)
        {
            m_leftPID.setOutputManually(percentage.left);
            m_rightPID.setOutputManually(percentage.right);

            updateMotors();
        }

        DriveStats getStats();
        uint32_t getPidFrequency()
        {
            return m_pidFrequency_Hz;
        }

    protected:
        void updateMotors();
        void constrainLimits(DriveLimitValues& limits);

        DriveValues                 m_prevTicks;
        DriveValues                 m_maxCurrents_mA;
        DriveValues                 m_prevVelocities;
        DriveLimits                 m_maxMotorLimits;
        uint32_t                    m_pidFrequency_Hz;
        float                       m_interval;
        float                       m_capacityUsed_mAh;
        QuadratureDecoderHardware   m_leftEncoder;
        QuadratureDecoderSoftware   m_rightEncoder;
        PID                         m_leftPID;
        PID                         m_rightPID;
        DualTB9051FTGDrivers        m_motors;
        bool                        m_leftReverse;
        bool                        m_rightReverse;
        bool                        m_initWasSuccessful;
        bool                        m_isEnabled;
};

#endif // DIFFERENTIAL_DRIVE_H_
