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
#ifndef I2C_ASYNC_H_
#define I2C_ASYNC_H_

#include <stdint.h>
#include <app_util_platform.h>
#include <nrf_drv_twi.h>
#include "CircularBuffer/CircularBuffer.h"

// The maximum size of the circular queue to use for buffering up I2C requests from all of the clients.
// Must be a power of 2.
#define I2C_ASYNC_BUFFER_SIZE    (1 << 3)


class ITWIMNotification
{
    public:
        virtual void notify(nrf_drv_twi_evt_t const * pEvent) = 0;
};


class I2CAsync
{
    public:
        I2CAsync(nrf_drv_twi_t const * pTwiInstance, uint8_t sclPin, uint8_t sdaPin,
                 nrf_twi_frequency_t frequency = NRF_TWI_FREQ_400K, uint8_t irqPriority = _PRIO_APP_LOWEST);

        bool init();
        void enable();

        bool writeRegister(uint8_t i2cAddress, uint8_t registerAddress, uint8_t value,
                            ITWIMNotification* pNotify = NULL);
        bool readRegisters(uint8_t i2cAddress, uint8_t registerAddress, void* pBuffer, size_t bufferSize,
                           ITWIMNotification* pNotify = NULL);


        void disableInterrupt()
        {
            m_origPriority = __get_BASEPRI();
            __set_BASEPRI(m_irqPriority << (8-__NVIC_PRIO_BITS));
        }
        void enableInterrupt()
        {
            __set_BASEPRI(m_origPriority);
        }

    protected:
        enum EntryType
        {
            REGISTER_WRITE,
            REGISTER_READ
        };
        struct Entry
        {
            volatile bool*    pIsCompleted;
            volatile bool*    pWasSuccessful;
            void*             pBuffer;
            ITWIMNotification* pNotify;
            size_t            bufferSize;
            EntryType         type;
            uint8_t           i2cAddress;
            // registerAddress and data must be together like this in the structure since for writes they are sent
            // together as a 2-byte array.
            uint8_t           registerAddress;
            uint8_t           data;
        };

        static void staticIsrHandler(nrf_drv_twi_evt_t const * pEvent, void* pContext);
        void isrHandler(nrf_drv_twi_evt_t const * pEvent);
        void popEntryFromQueueAndBeginI2CTransfer();
        bool queueUpEntry(Entry& entry);

        nrf_drv_twi_t const*        m_pI2C;
        volatile uint32_t           m_inFlight;
        uint32_t                    m_irqPriority;
        uint32_t                    m_origPriority;
        Entry                       m_currentEntry;
        CircularBuffer<Entry, I2C_ASYNC_BUFFER_SIZE>    m_queue;
        nrf_twi_frequency_t         m_frequency;
        bool                        m_isInit;
        uint8_t                     m_sclPin;
        uint8_t                     m_sdaPin;
};

#endif // I2C_ASYNC_H_
