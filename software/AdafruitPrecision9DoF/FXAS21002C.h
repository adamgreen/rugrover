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
// Driver for the FXAS21002C 3DoF gyro.
#ifndef FXAS21002C_H_
#define FXAS21002C_H_

#include "Vector.h"
#include "SensorBase.h"


class FXAS21002C : protected SensorBase
{
public:
    FXAS21002C(int32_t sampleRate_Hz, I2CAsync* pI2CAsync, int address = 0x21);
    bool init();

    // There will be 2 I2C transfers completed to pNotify interface if supplied.
    void getVector(Vector<int16_t>* pVector, int16_t* pTemperature, II2CNotification* pNotify);

protected:
    // Method called in when a I2C operation is completed.
    virtual void opCompleted(bool wasSuccessful, uint32_t index);

    bool initGyro();

    Vector<int16_t>* m_pVector;
    int16_t*         m_pTemperature;
    char             m_bigEndianData[sizeof(int16_t)*3];
    int8_t           m_temp;
};

#endif /* FXAS21002C_H_ */
