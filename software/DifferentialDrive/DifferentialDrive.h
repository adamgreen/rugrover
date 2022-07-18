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
                          float pidKc, float pidTi, float pidTd, uint32_t maxMotorPercentage,
                          SAADCScanner* pADC,
                          nrf_drv_pwm_t* pPWM, uint32_t frequency,
                          uint32_t maxCurrent_mA);

        struct DriveStats
        {
            DriveFloatValues    velocityRequested;
            DriveValues         velocityActual;
            DriveValues         encoderCount;
            DriveValues         power;
            DriveValues         current_mA;
            DriveValues         maxCurrent_mA;
            DriveBits           faultDetected;
            DriveBits           overcurrentDetected;
            bool                autoMode;
        };

        bool init()
        {
            bool initWasSuccessful = true;
            initWasSuccessful &= m_leftEncoder.init();
            initWasSuccessful &= m_rightEncoder.init();
            initWasSuccessful &= m_motors.init();

            return initWasSuccessful;
        }

        void enable()
        {
            m_motors.enable(true);
        }

        void disable()
        {
            m_motors.enable(false);
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

    protected:
        void updateMotors();

        DriveValues                 m_prevTicks;
        DriveValues                 m_maxCurrents_mA;
        DriveValues                 m_prevVelocities;
        QuadratureDecoderHardware   m_leftEncoder;
        QuadratureDecoderSoftware   m_rightEncoder;
        PID                         m_leftPID;
        PID                         m_rightPID;
        DualTB9051FTGDrivers        m_motors;
        bool                        m_leftReverse;
        bool                        m_rightReverse;
};

#endif // DIFFERENTIAL_DRIVE_H_
