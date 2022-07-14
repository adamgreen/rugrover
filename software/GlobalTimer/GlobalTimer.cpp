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
// Driver to create a global 1MHz timer that can be queried to determine number of microseconds since reset.
// Mimics Arduino's micros() functionality.
#include "GlobalTimer.h"
#include <app_util_platform.h>
#include <nrf_drv_timer.h>

class GlobalTimer
{
    public:
        GlobalTimer(nrf_drv_timer_t const * pTimerInstance);

        uint32_t micros()
        {
            return nrf_drv_timer_capture(m_pTimerInstance, NRF_TIMER_CC_CHANNEL0);
        }

    protected:
        static void dummyCallback(nrf_timer_event_t eventType, void* pvContext);

        nrf_drv_timer_t const*  m_pTimerInstance;
};

// User TIMER2 for the global 1MHz timer.
static nrf_drv_timer_t g_timerInstance = NRF_DRV_TIMER_INSTANCE(2);
static GlobalTimer g_globalTimer(&g_timerInstance);



GlobalTimer::GlobalTimer(nrf_drv_timer_t const * pTimerInstance)
{
    nrf_drv_timer_config_t config =
    {
        .frequency = NRF_TIMER_FREQ_1MHz,
        .mode = NRF_TIMER_MODE_TIMER,
        .bit_width = NRF_TIMER_BIT_WIDTH_32,
        .interrupt_priority = _PRIO_APP_LOWEST,
        .p_context = NULL
    };

    uint32_t result = nrf_drv_timer_init(pTimerInstance, &config, dummyCallback);
    APP_ERROR_CHECK(result);
    nrf_drv_timer_enable(pTimerInstance);

    m_pTimerInstance = pTimerInstance;
}

void GlobalTimer::dummyCallback(nrf_timer_event_t eventType, void* pvContext)
{
    // Needs to be passed into init but we aren't using interrupts.
}

uint32_t micros()
{
    return g_globalTimer.micros();
}
