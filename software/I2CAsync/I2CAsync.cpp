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
// Driver to make it easier to use the nRF52's TWI peripheral asynchronously from multiple ISR priorities to multiple
// I2C devices.
#include <nrf_assert.h>
#include <nrf_atomic.h>
#include "I2CAsync.h"


I2CAsync::I2CAsync(nrf_drv_twi_t const * pTwiInstance, uint8_t sclPin, uint8_t sdaPin,
                   nrf_twi_frequency_t frequency /* = NRF_TWI_FREQ_400K */, uint8_t irqPriority /* = _PRIO_APP_LOWEST */)
{
    m_pI2C = pTwiInstance;
    m_sclPin = sclPin;
    m_sdaPin = sdaPin;
    m_frequency = frequency;
    m_inFlight = 0;
    m_irqPriority = irqPriority;
    m_isInit = false;
}

bool I2CAsync::init()
{
    nrf_drv_twi_config_t i2cConfig =
    {
        .scl = m_sclPin,
        .sda = m_sdaPin,
        .frequency = m_frequency,
        .interrupt_priority = m_irqPriority,
        .clear_bus_init = false,
        .hold_bus_uninit = false
    };

    // Don't try to init this object twice.
    ASSERT ( !m_isInit );

    ret_code_t result = nrf_drv_twi_init(m_pI2C, &i2cConfig, staticIsrHandler, this);
    if (result == NRF_SUCCESS)
    {
        m_isInit = true;
        return true;
    }
    return false;
}

void I2CAsync::staticIsrHandler(nrf_drv_twi_evt_t const * pEvent, void* pvContext)
{
    I2CAsync* pThis = (I2CAsync*)pvContext;
    pThis->isrHandler(pEvent);
}

void I2CAsync::isrHandler(nrf_drv_twi_evt_t const * pEvent)
{
    // Transfer just completed so release the mutex.
    m_inFlight = 0;

    // UNDONE: Should communicate errors back to blocking callers.
    if (m_currentEntry.pNotify != NULL)
    {
        m_currentEntry.pNotify->notify(pEvent);
    }
    if (m_currentEntry.pIsCompleted)
    {
        *m_currentEntry.pIsCompleted = true;
    }

    popEntryFromQueueAndBeginI2CTransfer();
}

void I2CAsync::popEntryFromQueueAndBeginI2CTransfer()
{
    // Attempt to acquire the mutex which only allows one I2C transaction to be in flight.
    uint32_t valueIfFree = 0;
    uint32_t valueIfInUse = 1;
    bool acquired = __atomic_compare_exchange_n(&m_inFlight, &valueIfFree, valueIfInUse, false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST);
    if (!acquired)
    {
        // There is already an entry in flight so can't start another. Then next entry will be started when current 1
        // completes.
        return;
    }

    bool haveAnotherEntry = m_queue.read(m_currentEntry);
    if (!haveAnotherEntry)
    {
        // No more entries in queue to send so release the mutex and return.
        m_inFlight = 0;
        return;
    }

    ret_code_t result = NRF_ERROR_INVALID_PARAM;
    switch (m_currentEntry.type)
    {
        case REGISTER_WRITE:
        {
            // Send registerAddress and data fields together in a single Tx transfer.
            result = nrf_drv_twi_tx(m_pI2C, m_currentEntry.i2cAddress, &m_currentEntry.registerAddress, 1+1, false);
            break;
        }
        case REGISTER_READ:
        {
            nrf_drv_twi_xfer_desc_t transferDesc =
            {
                .type = NRF_DRV_TWI_XFER_TXRX,
                .address = m_currentEntry.i2cAddress,
                .primary_length = 1,
                .secondary_length = (uint8_t)m_currentEntry.bufferSize,
                .p_primary_buf = &m_currentEntry.registerAddress,
                .p_secondary_buf = (uint8_t*)m_currentEntry.pBuffer
            };
            result = nrf_drv_twi_xfer(m_pI2C, &transferDesc, 0);
            break;
        }
        default:
            ASSERT ( false );
            break;
    }
    APP_ERROR_CHECK(result);
    if (result != NRF_SUCCESS)
    {
        // Failed to issue I2C transfer so release the mutex.
        m_inFlight = 0;
    }
}

void I2CAsync::enable()
{
    // Must be initialized before enabling.
    ASSERT ( m_isInit );

    nrf_drv_twi_enable(m_pI2C);
}

bool I2CAsync::writeRegister(uint8_t i2cAddress, uint8_t registerAddress, uint8_t value,
                              ITWIMNotification* pNotify /* = NULL */)
{
    Entry entry =
    {
        .pIsCompleted = NULL,
        .pBuffer = NULL,
        .pNotify = pNotify,
        .bufferSize = 0,
        .type = REGISTER_WRITE,
        .i2cAddress = i2cAddress,
        .registerAddress = registerAddress,
        .data = value
    };
    return queueUpEntry(entry);
}

bool I2CAsync::readRegisters(uint8_t i2cAddress, uint8_t registerAddress, void* pBuffer, size_t bufferSize,
                             ITWIMNotification* pNotify /* = NULL */)
{
    Entry entry =
    {
        .pIsCompleted = NULL,
        .pBuffer = pBuffer,
        .pNotify = pNotify,
        .bufferSize = bufferSize,
        .type = REGISTER_READ,
        .i2cAddress = i2cAddress,
        .registerAddress = registerAddress
    };
    return queueUpEntry(entry);
}

bool I2CAsync::queueUpEntry(Entry& entry)
{
    // TWIM driver doesn't support transfers longer than what will fit in 8-bit variable.
    ASSERT ( entry.bufferSize <= 0xFF );

    volatile bool isCompleted = false;
    if (entry.pNotify == NULL)
    {
        entry.pIsCompleted = &isCompleted;
    }

    bool result = m_queue.write(entry);
    if (!result)
    {
        // Probably need to increase the size of the circular buffer if this fails.
        ASSERT ( result );
        return false;
    }
    popEntryFromQueueAndBeginI2CTransfer();

    // Make it a blocking call if pNotify is NULL.
    while (entry.pNotify == NULL && !isCompleted)
    {
    }
    return true;
}
