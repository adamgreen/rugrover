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
#ifndef FXOS8700CQ_H_
#define FXOS8700CQ_H_

#include "SensorBase.h"
#include "Vector.h"


class FXOS8700CQ : protected SensorBase
{
public:
    FXOS8700CQ(int32_t sampleRate_Hz, I2CAsync* pI2CAsync, int address = 0x1F);
    bool init();

    // There will be 1 I2C transfer completed to pNotify interface if supplied.
    void getVectors(Vector<int16_t>* pAccelVector, Vector<int16_t>* pMagVector, II2CNotification* pNotify);

protected:
    // Method called in when a I2C operation is completed.
    virtual void opCompleted(bool wasSuccessful, uint32_t index);

    void reset();
    bool initMagnetometer();
    bool initAccelerometer(int32_t sampleRateHz);

    // Need enough space for 3-axis of magnetometer and accelerometer readings (16-bits per axis).
    Vector<int16_t>* m_pAccelVector;
    Vector<int16_t>* m_pMagVector;
    uint8_t          m_buffer[sizeof(int16_t)*(3+3)];
};

#endif /* FXOS8700CQ_H_ */
