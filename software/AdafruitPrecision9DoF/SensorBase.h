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
#ifndef SENSOR_BASE_H_
#define SENSOR_BASE_H_

#include <nrf_drv_twi.h>


class SensorBase
{
public:
    SensorBase(int32_t sampleRate_Hz, nrf_drv_twi_t const * pTwiInstance, uint8_t address);

protected:
    bool writeRegister(uint8_t registerAddress, uint8_t value);
    bool readRegister(uint8_t registerAddress, uint8_t* pBuffer);
    bool readRegisters(uint8_t registerAddress, void* pBuffer, size_t bufferSize);

    nrf_drv_twi_t const *   m_pI2C;
    int32_t                 m_sampleRate_Hz;
    uint8_t                 m_address;
};

#endif /* SENSOR_BASE_H_ */
