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
// Base class of I2C functionality shared by the FXOS8700CQ and FXAS21002C sensors.
#include <nrf_atomic.h>
#include "SensorBase.h"


SensorBase::SensorBase(int32_t sampleRate_Hz, I2CAsync* pI2CAsync, uint8_t address)
{
    m_pI2C = pI2CAsync;
    m_pNotify = NULL;
    m_address = address;
    m_ioIndex = 0;
    m_sampleRate_Hz = sampleRate_Hz;
    m_errorCount = 0;
}

bool SensorBase::writeRegister(uint8_t registerAddress, uint8_t value, ITWIMNotification* pNotify /*= NULL*/)
{
    return m_pI2C->writeRegister(m_address, registerAddress, value, pNotify);
}

bool SensorBase::readRegister(uint8_t registerAddress, uint8_t* pBuffer, ITWIMNotification* pNotify /*= NULL*/)
{
    return readRegisters(registerAddress, pBuffer, sizeof(*pBuffer), pNotify);
}

bool SensorBase::readRegisters(uint8_t registerAddress, void* pBuffer, size_t bufferSize, ITWIMNotification* pNotify /*= NULL*/)
{
    return m_pI2C->readRegisters(m_address, registerAddress, pBuffer, bufferSize, pNotify);
}

// ITWIMNotification methods.
void SensorBase::notify(nrf_drv_twi_evt_t const * pEvent)
{
    bool opWasSuccessful = true;

    // Code assumes that the overall operation was successful unless one of the I2C operations gets an error.
    // NOTE: pEvent == NULL is the way that getVectors() calls this function when it fails to queue up an I/O.
    if (pEvent == NULL || pEvent->type != NRF_DRV_TWI_EVT_DONE)
    {
        nrf_atomic_u32_add(&m_errorCount, 1);
        opWasSuccessful = false;
    }

    // Update I2C transfer count.
    uint32_t newIndex = nrf_atomic_u32_add(&m_ioIndex, 1);

    // Let sub-class do whatever is appropriate for the I/O that just completed.
    opCompleted(opWasSuccessful, newIndex);
}
