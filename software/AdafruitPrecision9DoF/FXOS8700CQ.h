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


class FXOS8700CQ : public SensorBase
{
public:
    FXOS8700CQ(int32_t sampleRate_Hz, nrf_drv_twi_t const * pTwiInstance, int address = 0x1F);
    bool init();

    bool getVectors(Vector<int16_t>* pAccelVector, Vector<int16_t>* pMagVector);

protected:
    void reset();
    bool initMagnetometer();
    bool initAccelerometer(int32_t sampleRateHz);
};

#endif /* FXOS8700CQ_H_ */
