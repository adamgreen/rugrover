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
#include <string.h>
#include <app_util_platform.h>
#include <nrf_gpio.h>
#include "DualTB9051FTGDrivers.h"


// Macros to convert between ADC readings and mA current values for the motor driver.
// Input analog range of (0-4095) to (0-3.3V)
#define ADC_TO_V(X) ((X)*33/40950)
// Input voltage range of (0-3.3V) to mA
//      I = OCMV / 220ohm * 1/0.00223
#define V_TO_mA(X) ((X)*2038)
// Input analog range of (0-4095) to mA
#define ADC_TO_mA(X) ((X)*67254/40950)
// Input motor current in mA to ADC range of (0-4095)
#define mA_TO_ADC(X) ((X)*40950/67254)



DualTB9051FTGDrivers* DualTB9051FTGDrivers::m_pThis = NULL;



DualTB9051FTGDrivers::DualTB9051FTGDrivers(uint8_t leftEnablePin, uint8_t leftPwm1Pin, uint8_t leftPwm2Pin,
                                           uint8_t leftDiagPin, uint8_t leftOcmPin, bool leftInvert,
                                           uint8_t rightEnablePin, uint8_t rightPwm1Pin, uint8_t rightPwm2Pin,
                                           uint8_t rightDiagPin, uint8_t rightOcmPin, bool rightInvert,
                                           SAADCScanner* pADC,
                                           nrf_drv_pwm_t* pPWM, uint32_t frequency,
                                           uint32_t maxCurrent_mA, uint8_t irqPriority /* = _PRIO_APP_LOWEST */)
: m_leftMotor(this, leftEnablePin, leftDiagPin, leftOcmPin, maxCurrent_mA, leftInvert),
  m_rightMotor(this, rightEnablePin, rightDiagPin, rightOcmPin, maxCurrent_mA, rightInvert)
{
    // Only support one instance of this class.
    ASSERT ( m_pThis == NULL );
    m_pThis = this;

    // Must provided pointer to PWM object used with the motor drivers.
    ASSERT ( pPWM );

    // PWM frequency for these motor drivers can't exceed 20kHz.
    ASSERT ( frequency > 0 && frequency <= 20000 );

    m_pADC = pADC;
    m_pPWM = pPWM;
    m_leftPwm1Pin = leftPwm1Pin;
    m_leftPwm2Pin = leftPwm2Pin;
    m_rightPwm1Pin = rightPwm1Pin;
    m_rightPwm2Pin = rightPwm2Pin;
    m_frequency = frequency;
    m_counterTop = 0;
    memset(&m_dutyCycles, 0, sizeof(m_dutyCycles));
    m_irqPriority = irqPriority;
}

bool DualTB9051FTGDrivers::init()
{
    bool validFreq = false;
    nrf_pwm_clk_t baseClock = NRF_PWM_CLK_125kHz;
    uint16_t counterTop = 3;
    for (uint32_t div = 0, freq=16000000 ; freq >= 125000 ; freq /= 2, div++)
    {
        uint32_t top = freq/m_frequency;
        if (top <= 0x7FFF)
        {
            baseClock = (nrf_pwm_clk_t)div;
            counterTop = (uint16_t)top;
            validFreq = true;
            break;
        }
    }
    if (!validFreq)
    {
        return false;
    }

    nrf_drv_pwm_config_t config =
    {
        .output_pins = {m_leftPwm1Pin, m_leftPwm2Pin, m_rightPwm1Pin, m_rightPwm2Pin},
        .irq_priority = m_irqPriority,
        .base_clock = baseClock,
        .count_mode = NRF_PWM_MODE_UP,
        .top_value = counterTop,
        .load_mode = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode = NRF_PWM_STEP_AUTO
    };
    ret_code_t result = nrf_drv_pwm_init(m_pPWM, &config, staticEventHandler);
    if (result != NRF_SUCCESS)
    {
        return false;
    }
    m_counterTop = counterTop;

    MotorDriver::DutyCycleSequences leftDutyCycles =
    { .dutyCyclePointers =
        {{.pPwm1 = &m_dutyCycles[0].channel_0,
          .pPwm2 = &m_dutyCycles[0].channel_1},
         {.pPwm1 = &m_dutyCycles[1].channel_0,
          .pPwm2 = &m_dutyCycles[1].channel_1}}
    };
    MotorDriver::DutyCycleSequences rightDutyCycles =
    { .dutyCyclePointers =
        {{.pPwm1 = &m_dutyCycles[0].channel_2,
          .pPwm2 = &m_dutyCycles[0].channel_3},
         {.pPwm1 = &m_dutyCycles[1].channel_2,
          .pPwm2 = &m_dutyCycles[1].channel_3}}
    };
    m_leftMotor.init(&leftDutyCycles, m_counterTop);
    m_rightMotor.init(&rightDutyCycles, m_counterTop);

    bool adcResult = m_leftMotor.initAdcToMeasureMotorCurrents(m_pADC);
    if (!adcResult)
    {
        return false;
    }
    adcResult = m_rightMotor.initAdcToMeasureMotorCurrents(m_pADC);
    if (!adcResult)
    {
        return false;
    }
    m_pADC->startScanning();

    nrf_pwm_sequence_t pwmSequence0 =
    {
        .values = { .p_individual = &m_dutyCycles[0] },
        .length = NRF_PWM_VALUES_LENGTH(m_dutyCycles[0]),
        .repeats = 0,
        .end_delay = 0
    };
    nrf_pwm_sequence_t pwmSequence1 =
    {
        .values = { .p_individual = &m_dutyCycles[1] },
        .length = NRF_PWM_VALUES_LENGTH(m_dutyCycles[1]),
        .repeats = 0,
        .end_delay = 0
    };
    uint32_t playbackFlags = NRF_DRV_PWM_FLAG_LOOP | NRF_DRV_PWM_FLAG_SIGNAL_END_SEQ0 | NRF_DRV_PWM_FLAG_SIGNAL_END_SEQ1;
    nrf_drv_pwm_complex_playback(m_pPWM, &pwmSequence0, &pwmSequence1, 1, playbackFlags);

    return true;
}



DualTB9051FTGDrivers::MotorDriver::MotorDriver(DualTB9051FTGDrivers* pMotors, uint8_t enablePin, uint8_t diagPin,
                                               uint8_t ocmPin, uint32_t maxCurrent_mA, bool reverse)
{
    m_pMotors = pMotors;
    m_currDutyCycle = 0;
    m_reverse = reverse;
    m_enablePin = enablePin;
    m_diagPin = diagPin;
    m_ocmPin = ocmPin;
    m_maxCurrent_mA = maxCurrent_mA;
    m_diagAsserted = false;
    m_currentOverloadDetected = false;
}

void DualTB9051FTGDrivers::MotorDriver::init(DutyCycleSequences* pDutyCycleSequences, uint16_t counterTop)
{
    m_dutyCycleSequences = *pDutyCycleSequences;
    m_counterTop = counterTop;

    nrf_gpio_pin_clear(m_enablePin);
    nrf_gpio_cfg_output(m_enablePin);
    nrf_gpio_cfg_input(m_diagPin, NRF_GPIO_PIN_NOPULL);

    // Get the duty cycle sequences initialized correctly for 0% power.
    m_currDutyCycle = 0;
    eventHandler(NRF_DRV_PWM_EVT_END_SEQ0);
    eventHandler(NRF_DRV_PWM_EVT_END_SEQ1);
}

bool DualTB9051FTGDrivers::MotorDriver::initAdcToMeasureMotorCurrents(SAADCScanner* pADC)
{
    // User doesn't care about motor currents if they don't pass in ADC scanner object.
    if (pADC == NULL)
    {
        return true;
    }

    m_pAdcChannel = pADC->addChannel(m_ocmPin,
                                     NRF_SAADC_RESISTOR_DISABLED,
                                     NRF_SAADC_GAIN1_4,
                                     NRF_SAADC_REFERENCE_VDD4,
                                     NRF_SAADC_ACQTIME_3US,
                                     SAADC_LOWER_LIMIT_DISABLED,
                                     mA_TO_ADC(m_maxCurrent_mA),
                                     this);
    if (m_pAdcChannel == NULL)
    {
        return false;
    }

    return true;
}

void DualTB9051FTGDrivers::MotorDriver::enable(bool enable)
{
    if (enable)
    {
        m_diagAsserted = false;
        m_currentOverloadDetected = false;
        nrf_gpio_pin_set(m_enablePin);
    }
    else
    {
        setPower(0);
        nrf_gpio_pin_clear(m_enablePin);
    }
    m_enabled = enable;
}

void DualTB9051FTGDrivers::MotorDriver::setPower(int32_t power)
{
    ASSERT ( power >= -100 && power <= 100 );

    if (m_reverse)
    {
        power = -power;
    }

    uint16_t currDutyCycle = m_currDutyCycle;
    uint16_t currMagnitude = extractMagnitude(currDutyCycle);
    bool     currPolarity = extractPolarity(currDutyCycle);

    uint32_t newMagnitude = abs(power) * m_counterTop / 100;
    bool     newPolarity = power < 0;

    if (currPolarity == newPolarity && currMagnitude == newMagnitude)
    {
        // Nothing has changed so just return.
        return;
    }

    waitForLastDutyCycleUpdateToComplete();
    if (newPolarity != currPolarity && currMagnitude != 0)
    {
        // Can't immediately change polarity. Run it through a zero cycle first.
        newMagnitude = 0;
        newPolarity = currPolarity;
    }
    m_currDutyCycle = buildDutyCycle(newPolarity, newMagnitude);
    m_updatedDutyCycle = true;
}

bool DualTB9051FTGDrivers::MotorDriver::hasEncounteredFault()
{
    return m_diagAsserted;
}

bool DualTB9051FTGDrivers::MotorDriver::hasDetectedCurrentOverload()
{
    return m_currentOverloadDetected;
}

int32_t DualTB9051FTGDrivers::MotorDriver::getCurrentReading()
{
    if (m_pAdcChannel == NULL)
    {
        return 0;
    }
    SAADCScanner::Channel::Reading reading = m_pAdcChannel->read();
    return ADC_TO_mA((int32_t)reading.max);
}

DriveLimits DualTB9051FTGDrivers::MotorDriver::getPowerLimits()
{
    DriveLimits limits;

    uint16_t currDutyCycle = m_currDutyCycle;
    uint16_t currMagnitude = extractMagnitude(currDutyCycle);
    bool     currPolarity = extractPolarity(currDutyCycle);
    if (m_reverse)
    {
        currPolarity = !currPolarity;
    }

    if (currMagnitude == 0)
    {
        limits.min = -100;
        limits.max = 100;
    }
    else if (currPolarity)
    {
        limits.min = -100;
        limits.max = 0;
    }
    else
    {
        limits.min = 0;
        limits.max = 100;
    }
    return limits;
}

void DualTB9051FTGDrivers::MotorDriver::notifyLimitExceeded(bool lowLimitExceeded, bool highLimitExceeded)
{
    ASSERT ( !lowLimitExceeded && highLimitExceeded );
    m_currentOverloadDetected = true;
    // Disable both motors if an overcurrent situation is discovered in either of them.
    m_pMotors->enable(false);
}

void DualTB9051FTGDrivers::MotorDriver::eventHandler(nrf_drv_pwm_evt_type_t eventType)
{
    DutyCyclePointers* pDutyCycles = NULL;

    if (m_enabled && nrf_gpio_pin_read(m_diagPin) == 0)
    {
        m_diagAsserted = true;
        m_pMotors->enable(false);
    }

    switch (eventType)
    {
        case NRF_DRV_PWM_EVT_FINISHED:
            break;
        case NRF_DRV_PWM_EVT_END_SEQ0:
            pDutyCycles = &m_dutyCycleSequences.dutyCyclePointers[0];
            break;
        case NRF_DRV_PWM_EVT_END_SEQ1:
            pDutyCycles = &m_dutyCycleSequences.dutyCyclePointers[1];
            break;
        case NRF_DRV_PWM_EVT_STOPPED:
            break;
    }

    if (pDutyCycles != NULL)
    {
        uint16_t pwm1DutyCycle = m_currDutyCycle;
        uint16_t pwm2DutyCycle = buildDutyCycle(extractPolarity(pwm1DutyCycle), 0);
        *pDutyCycles->pPwm1 = pwm1DutyCycle;
        *pDutyCycles->pPwm2 = pwm2DutyCycle;
        m_updatedDutyCycle = false;
    }
}
