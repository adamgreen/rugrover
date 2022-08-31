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
#ifndef FXAS21002C_H_
#define FXAS21002C_H_

#include "Vector.h"
#include "SensorBase.h"


class FXAS21002C : public SensorBase
{
public:
    FXAS21002C(int32_t sampleRate_Hz, nrf_drv_twi_t const * pTwiInstance, int address = 0x21);
    bool init();

    bool getVector(Vector<int16_t>* pVector, int16_t* pTemperature);

protected:
    bool initGyro();
};

#endif /* FXAS21002C_H_ */
