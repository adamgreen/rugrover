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

#include "I2CAsync/I2CAsync.h"


class II2CNotification
{
    public:
        virtual void notify(bool wasSuccessful) = 0;
};


class SensorBase : protected ITWIMNotification
{
public:
    SensorBase(int32_t sampleRate_Hz, I2CAsync* pI2CAsync, uint8_t address);

protected:
    // ITWIMNotification methods.
    virtual void notify(nrf_drv_twi_evt_t const * pEvent);

    // Method called in sub-class when a I2C operation is completed.
    virtual void opCompleted(bool wasSuccessful, uint32_t index) = 0;

    bool writeRegister(uint8_t registerAddress, uint8_t value, ITWIMNotification* pNotify = NULL);
    bool readRegister(uint8_t registerAddress, uint8_t* pBuffer, ITWIMNotification* pNotify = NULL);
    bool readRegisters(uint8_t registerAddress, void* pBuffer, size_t bufferSize, ITWIMNotification* pNotify = NULL);

    I2CAsync*           m_pI2C;
    II2CNotification*   m_pNotify;
    int32_t             m_sampleRate_Hz;
    volatile uint32_t   m_ioIndex;
    volatile uint32_t   m_errorCount;
    uint8_t             m_address;
};

#endif /* SENSOR_BASE_H_ */
