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
// Driver to control 2 Toshiba TB9051FTG 5A motor drivers.
// The one class controls two motor drivers so that it can own the single PWM instance that will be used for both
// motors.
#ifndef DUAL_TB9051FTG_DRIVERS_
#define DUAL_TB9051FTG_DRIVERS_

#include <nrf_drv_pwm.h>
#include "SAADCScanner/SAADCScanner.h"

class DualTB9051FTGDrivers
{
    public:
        struct CurrentReadings
        {
            int32_t leftCurrent_mA;
            int32_t rightCurrent_mA;
        };

        DualTB9051FTGDrivers(uint8_t leftEnablePin, uint8_t leftPwm1Pin, uint8_t leftPwm2Pin,
                             uint8_t leftDiagPin, uint8_t leftOcmPin, bool leftReverse,
                             uint8_t rightEnablePin, uint8_t rightPwm1Pin, uint8_t rightPwm2Pin,
                             uint8_t rightDiagPin, uint8_t rightOcmPin, bool rightReverse,
                             SAADCScanner* pADC,
                             nrf_drv_pwm_t* pPWM, uint32_t frequency,
                             uint32_t maxCurrent_mA, uint8_t irqPriority = _PRIO_APP_LOWEST);

        bool init();

        // The left and right parameters should be provided power values between -100 and 100.
        void setPower(int32_t left, int32_t right)
        {
            m_leftMotor.setPower(left);
            m_rightMotor.setPower(right);
        }

        void enable(bool enable)
        {
            m_leftMotor.enable(enable);
            m_rightMotor.enable(enable);
        }

        CurrentReadings getCurrentReadings()
        {
            CurrentReadings currentReadings;

            currentReadings.leftCurrent_mA = m_leftMotor.getCurrentReading();
            currentReadings.rightCurrent_mA = m_rightMotor.getCurrentReading();
            return currentReadings;
        }

        // Motor driver faults are caused by Vcc over/undervoltage, Vbat undervoltage, over current, over temperature.
        bool hasLeftMotorEncounteredFault()
        {
            return m_leftMotor.hasEncounteredFault();
        }
        bool hasRightMotorEncounteredFault()
        {
            return m_rightMotor.hasEncounteredFault();
        }
        bool hasEitherMotorEncounteredFault()
        {
            return hasLeftMotorEncounteredFault() || hasRightMotorEncounteredFault();
        }
        bool hasLeftMotorDetectedCurrentOverload()
        {
            return m_leftMotor.hasDetectedCurrentOverload();
        }
        bool hasRightMotorDetectedCurrentOverload()
        {
            return m_rightMotor.hasDetectedCurrentOverload();
        }
        bool hasEitherMotorDetectedCurrentOverload()
        {
            return hasLeftMotorDetectedCurrentOverload() || hasRightMotorDetectedCurrentOverload();
        }

    protected:
        class MotorDriver : public ISAADCScannerNotification
        {
            public:
                MotorDriver(DualTB9051FTGDrivers* pMotors, uint8_t enablePin, uint8_t diagPin, uint8_t ocmPin,
                            uint32_t maxCurrent_mA, bool reverse);

                struct DutyCyclePointers
                {
                    uint16_t* pPwm1;
                    uint16_t* pPwm2;
                };
                struct DutyCycleSequences
                {
                    DutyCyclePointers dutyCyclePointers[2];
                };

                void init(DutyCycleSequences* pDutyCycleSequences, uint16_t counterTop);
                bool initAdcToMeasureMotorCurrents(SAADCScanner* pADC);

                void setPower(int32_t power);
                void enable(bool enable);

                void eventHandler(nrf_drv_pwm_evt_type_t eventType);

                bool hasEncounteredFault();
                bool hasDetectedCurrentOverload();
                int32_t getCurrentReading();

            protected:
                virtual void notifyLimitExceeded(bool lowLimitExceeded, bool highLimitExceeded);

                // Duty cycles passed into PWM peripheral are composed of 15-bit count and 1-bit for pulse polarity.
                uint16_t extractPolarity(uint16_t dutyCycle)
                {
                    return (dutyCycle >> 15) & 1;
                }
                uint16_t extractMagnitude(uint16_t dutyCycle)
                {
                    return (dutyCycle & 0x7FFF);
                }
                uint16_t buildDutyCycle(bool polarity, uint16_t magnitude)
                {
                    uint16_t dutyCycle = magnitude;
                    if (polarity)
                    {
                        dutyCycle |= (1 << 15);
                    }
                    return dutyCycle;
                }

                void waitForLastDutyCycleUpdateToComplete()
                {
                    while (m_updatedDutyCycle)
                    {
                    }
                }

                DualTB9051FTGDrivers*   m_pMotors;
                SAADCScanner::Channel*  m_pAdcChannel;
                uint32_t                m_maxCurrent_mA;
                DutyCycleSequences      m_dutyCycleSequences;
                volatile uint16_t       m_currDutyCycle;
                uint16_t                m_counterTop;
                volatile bool           m_updatedDutyCycle;
                bool                    m_enabled;
                bool                    m_reverse;
                bool                    m_diagAsserted;
                bool                    m_currentOverloadDetected;
                uint8_t                 m_enablePin;
                uint8_t                 m_diagPin;
                uint8_t                 m_ocmPin;
        };

        static void staticEventHandler(nrf_drv_pwm_evt_type_t eventType)
        {
            m_pThis->m_leftMotor.eventHandler(eventType);
            m_pThis->m_rightMotor.eventHandler(eventType);
        }
        static DualTB9051FTGDrivers* m_pThis;

        SAADCScanner*   m_pADC;
        nrf_drv_pwm_t*  m_pPWM;
        uint32_t        m_frequency;
        MotorDriver     m_leftMotor;
        MotorDriver     m_rightMotor;
        uint16_t        m_counterTop;
        nrf_pwm_values_individual_t m_dutyCycles[2];
        uint8_t         m_irqPriority;
        uint8_t         m_leftPwm1Pin;
        uint8_t         m_leftPwm2Pin;
        uint8_t         m_rightPwm1Pin;
        uint8_t         m_rightPwm2Pin;
};

#endif // DUAL_TB9051FTG_DRIVERS_