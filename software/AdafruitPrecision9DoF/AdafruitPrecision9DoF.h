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
#ifndef ADAFRUIT_PRECISION_9DOF_H_
#define ADAFRUIT_PRECISION_9DOF_H_

#include <nrf_atomic.h>
#include <nrf_drv_gpiote.h>
#include "I2CAsync/I2CAsync.h"
#include "FXOS8700CQ.h"
#include "FXAS21002C.h"
#include "Vector.h"

struct SensorValues
{
    uint32_t        samplePeriod_us;
    Vector<int16_t> accel;
    Vector<int16_t> mag;
    Vector<int16_t> gyro;
    int16_t         gyroTemperature;
};



class AdafruitPrecision9DoF : protected II2CNotification
{
public:
    AdafruitPrecision9DoF(I2CAsync* pI2CAsync, uint8_t int1Pin, int32_t sampleRate_Hz);

    bool init();

    bool            wouldBlock() { return m_lastSample == m_currentSample; }
    SensorValues    getRawSensorValues();
    bool            didIoFail()
    {
        uint32_t failedIsrIo = m_failedIsrIo;
        nrf_atomic_u32_sub(&m_failedIsrIo, failedIsrIo);
        return failedIsrIo > 0;
    }

protected:
    // II2CNotification method.
    virtual void notify(bool wasSuccessful);

    bool         installInterruptOnFallingEdge(uint8_t pin);
    static void  staticInterruptHandler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
    void         interruptHandler();
    void         waitForFirstIoToComplete();

    I2CAsync*               m_pI2CAsync;
    FXOS8700CQ              m_accelMag;
    FXAS21002C              m_gyro;
    volatile uint32_t       m_currentSample;
    volatile uint32_t       m_failedIsrIo;
    uint32_t                m_lastSample;
    uint32_t                m_lastSampleTime_us;
    volatile uint32_t       m_ioIndex;
    int32_t                 m_sampleRate_Hz;
    SensorValues            m_sensorValuesDirty;
    SensorValues            m_sensorValues;
    uint8_t                 m_int1Pin;
};

#endif /* ADAFRUIT_PRECISION_9DOF_H_ */
