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
#include "SensorBase.h"


SensorBase::SensorBase(int32_t sampleRate_Hz, nrf_drv_twi_t const * pTwiInstance, uint8_t address)
{
    m_pI2C = pTwiInstance;
    m_address = address;
    m_sampleRate_Hz = sampleRate_Hz;
}

bool SensorBase::writeRegister(uint8_t registerAddress, uint8_t value)
{
    uint8_t dataToSend[2] = { registerAddress, value };

    ret_code_t result = nrf_drv_twi_tx(m_pI2C, m_address, dataToSend, sizeof(dataToSend), false);
    return result == NRF_SUCCESS;
}

bool SensorBase::readRegister(uint8_t registerAddress, uint8_t* pBuffer)
{
    return readRegisters(registerAddress, pBuffer, sizeof(*pBuffer));
}

bool SensorBase::readRegisters(uint8_t registerAddress, void* pBuffer, size_t bufferSize)
{
    ret_code_t result = nrf_drv_twi_tx(m_pI2C, m_address, &registerAddress, sizeof(registerAddress), true);
    if (result != NRF_SUCCESS)
    {
        return false;
    }

    result = nrf_drv_twi_rx(m_pI2C, m_address, (uint8_t*)pBuffer, bufferSize);
    return result == NRF_SUCCESS;
}
