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
/* This driver uses the I2CAsync driver to communicate with the NXP FXOS8700CQ accelerometers/magnetometers and the
   NXP FXAS21002C gyros.
*/
#include <string.h>
#include <nrf_delay.h>
#include "AdafruitPrecision9DoF.h"



static AdafruitPrecision9DoF* g_pThis = NULL;

AdafruitPrecision9DoF::AdafruitPrecision9DoF(I2CAsync* pI2CAsync, uint8_t int1Pin, int32_t sampleRate_Hz)
:   m_accelMag(sampleRate_Hz, pI2CAsync),
    m_gyro(sampleRate_Hz, pI2CAsync)
{
    m_failedIsrIo = 0;
    m_currentSample = 0;
    m_lastSample = 0;
    m_int1Pin = int1Pin;
    m_ioIndex = 0;
    m_sampleRate_Hz = sampleRate_Hz;
}

bool AdafruitPrecision9DoF::init()
{
    if (!m_accelMag.init() || !m_gyro.init())
    {
        return false;
    }

    // Wait for interrupt pin (first sample is ready) to go low before setting up falling edge interrupt.
    while (nrf_gpio_pin_read(m_int1Pin) == 1)
    {
    }
    bool result = installInterruptOnFallingEdge(m_int1Pin);
    if (!result)
    {
        return result;
    }

    // Issue reads for first sample and block.
    interruptHandler();
    waitForFirstIoToComplete();

    // Make sure that interrupt pin is no longer asserted after reading first sample.
    ASSERT ( nrf_gpio_pin_read(m_int1Pin) == 1 );

    return true;
}

bool AdafruitPrecision9DoF::installInterruptOnFallingEdge(uint8_t pin)
{
    nrf_drv_gpiote_in_config_t config =
    {
        .sense = NRF_GPIOTE_POLARITY_HITOLO,
        .pull = NRF_GPIO_PIN_PULLUP,
        .is_watcher = false,
        .hi_accuracy = true
    };

    // Only support one instance for the GPIO interrupt handler.
    ASSERT ( g_pThis == NULL );
    if (g_pThis != NULL)
    {
        return false;
    }
    g_pThis = this;

    ASSERT ( nrf_drv_gpiote_is_init() );
    uint32_t result = nrf_drv_gpiote_in_init(pin, &config, staticInterruptHandler);
    if (result != NRF_SUCCESS)
    {
        return false;
    }
    nrf_drv_gpiote_in_event_enable(pin, true);

    return true;
}

void  AdafruitPrecision9DoF::staticInterruptHandler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    g_pThis->interruptHandler();
}

void AdafruitPrecision9DoF::interruptHandler()
{
    m_accelMag.getVectors(&m_sensorValues.accel, &m_sensorValues.mag, this);
    m_gyro.getVector(&m_sensorValues.gyro, &m_sensorValues.gyroTemperature, this);
}

void AdafruitPrecision9DoF::waitForFirstIoToComplete()
{
    while (m_currentSample != 1)
    {
        // Block until first interruptHandler() issued I2C transactions have completed.
    }
}

// II2CNotification method.
void AdafruitPrecision9DoF::notify(bool wasSuccessful)
{
    if (!wasSuccessful)
    {
        nrf_atomic_u32_add(&m_failedIsrIo, 1);
    }
    uint32_t newIoIndex = nrf_atomic_u32_add(&m_ioIndex, 1);
    if ((newIoIndex % 2) == 0)
    {
        // Every second transfer that completes means that the latest samples from accelerometer/magnetometer and
        // gyros have been received.
        m_currentSample++;
    }
}

SensorValues AdafruitPrecision9DoF::getRawSensorValues()
{
    uint32_t     currentSample;
    SensorValues sensorValues;

    // Wait for next sample to become available.
    do
    {
        currentSample = m_currentSample;
    } while (currentSample == m_lastSample);
    m_lastSample = currentSample;

    // UNDONE: Use interrupt priority for this.
    __disable_irq();
        memcpy(&sensorValues, &m_sensorValues, sizeof(sensorValues));
    __enable_irq();

    return sensorValues;
}
