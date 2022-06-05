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
// Driver to interface with quadrature encoders using the QDEC peripheral on the nRF52 microcontroller.
#include <app_util_platform.h>
#include "QuadratureDecoderHardware.h"


QuadratureDecoderHardware::QuadratureDecoderHardware(uint8_t aPin, uint8_t bPin)
{
    m_aPin = aPin;
    m_bPin = bPin;
    m_count = 0;
}

bool QuadratureDecoderHardware::init()
{
    nrf_drv_qdec_config_t config =
    {
        .reportper = NRF_QDEC_REPORTPER_DISABLED,
        .sampleper = NRF_QDEC_SAMPLEPER_128us,
        .psela = m_aPin,
        .pselb = m_bPin,
        .pselled = 0xFFFFFFFF,
        .ledpre = 0,
        .ledpol = NRF_QDEC_LEPOL_ACTIVE_LOW,
        .dbfen = false,
        .sample_inten = false,
        .interrupt_priority = _PRIO_APP_LOWEST
    };

    uint32_t result = nrf_drv_qdec_init(&config, NULL);
    if (result != NRF_SUCCESS)
    {
        return false;
    }
    nrf_drv_qdec_enable();

    return true;
}

int32_t QuadratureDecoderHardware::getCount()
{
    int16_t tickDelta;
    int16_t errorCount;
    nrf_drv_qdec_accumulators_read(&tickDelta, &errorCount);
    m_count += tickDelta;

    return m_count;
}
