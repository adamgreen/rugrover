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
#include <nrf_atomic.h>
#include "QuadratureDecoderSoftware.h"


QuadratureDecoderSoftware* QuadratureDecoderSoftware::m_instances[4];
const int32_t QuadratureDecoderSoftware::m_stateTable[4][4] = { { 0, -1,  1,  0},
                                                                { 1,  0,  0, -1},
                                                                {-1,  0,  0,  1},
                                                                { 0,  1, -1,  0} };


QuadratureDecoderSoftware::QuadratureDecoderSoftware(uint8_t aPin, uint8_t bPin)
{
    m_aPin = aPin;
    m_bPin = bPin;
    m_pinBitmask = (1 << aPin) | (1 << bPin);
    m_count = 0;
    m_prevEncoder = 0;
    m_pGpioReg = NULL;
}

bool QuadratureDecoderSoftware::init()
{
    nrf_drv_gpiote_in_config_t config =
    {
        .sense = NRF_GPIOTE_POLARITY_TOGGLE,
        .pull = NRF_GPIO_PIN_NOPULL,
        .is_watcher = false,
        .hi_accuracy = true
    };
    uint32_t result;

    ASSERT ( m_bPin == m_aPin + 1 );
    ASSERT ( nrf_drv_gpiote_is_init() );

    result = nrf_drv_gpiote_in_init(m_aPin, &config, isrHandler);
    if (result != NRF_SUCCESS)
    {
        return false;
    }
    result = nrf_drv_gpiote_in_init(m_bPin, &config, isrHandler);
    if (result != NRF_SUCCESS)
    {
        return false;
    }

    uint32_t pinNumber = m_aPin;
    m_pGpioReg = nrf_gpio_pin_port_decode(&pinNumber);
    m_prevEncoder = (m_pGpioReg->IN >> m_aPin) & 0x3;

    bool addSuccessful = addInstance(this);
    ASSERT ( addSuccessful );

    nrf_drv_gpiote_in_event_enable(m_aPin, true);
    nrf_drv_gpiote_in_event_enable(m_bPin, true);

    return addSuccessful;
}

bool QuadratureDecoderSoftware::addInstance(QuadratureDecoderSoftware* pInstance)
{
    for (size_t i = 0 ; i < sizeof(m_instances)/sizeof(m_instances[0]) ; i++)
    {
        if (m_instances[i] == NULL)
        {
            m_instances[i] = pInstance;
            return true;
        }
    }

    // Only support 4 instances.
    return false;
}

void QuadratureDecoderSoftware::isrHandler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    uint32_t pinBitmask = 1 << pin;

    for (size_t i = 0 ; i < sizeof(m_instances)/sizeof(m_instances[0]) ; i++)
    {
        if (pinBitmask & m_instances[i]->m_pinBitmask)
        {
            m_instances[i]->pinChanged();
            return;
        }
    }
}

void QuadratureDecoderSoftware::pinChanged()
{
    // Fetch the current state of the encoder pins.
    uint32_t portValue = m_pGpioReg->IN;
    uint32_t currEncoder = (portValue >> m_aPin) & 0x3;

    // Look up delta based on current and previous state.
    int32_t encoderDelta = m_stateTable[m_prevEncoder][currEncoder];
    nrf_atomic_u32_add(&m_count, encoderDelta);

    // Remember current encoder state.
    m_prevEncoder = currEncoder;
 }

int32_t QuadratureDecoderSoftware::getCount()
{
    return m_count;
}
