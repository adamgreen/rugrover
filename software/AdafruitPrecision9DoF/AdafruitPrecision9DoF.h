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
#ifndef ADAFRUIT_PRECISION_9DOF_H_
#define ADAFRUIT_PRECISION_9DOF_H_

#include <nrf_drv_gpiote.h>
#include "FXOS8700CQ.h"
#include "FXAS21002C.h"
#include "Vector.h"

struct SensorValues
{
    Vector<int16_t> accel;
    Vector<int16_t> mag;
    Vector<int16_t> gyro;
    int16_t         gyroTemperature;
};



class AdafruitPrecision9DoF
{
public:
    AdafruitPrecision9DoF(nrf_drv_twi_t const * pTwiInstance, uint8_t int1Pin, int32_t sampleRate_Hz);

    bool init();

    bool                   wouldBlock() { return m_lastSample == m_currentSample; }
    SensorValues           getRawSensorValues();
    int   didIoFail() { return m_failedIsrIo > 0; }

protected:
    bool         installInterruptOnFallingEdge(uint8_t pin);
    static void  staticInterruptHandler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
    void         interruptHandler();

    FXOS8700CQ              m_accelMag;
    FXAS21002C              m_gyro;
    volatile uint32_t       m_currentSample;
    volatile uint32_t       m_failedIsrIo;
    uint32_t                m_lastSample;
    int32_t                 m_sampleRate_Hz;
    SensorValues            m_sensorValues;
    uint8_t                 m_int1Pin;
};

#endif /* ADAFRUIT_PRECISION_9DOF_H_ */
