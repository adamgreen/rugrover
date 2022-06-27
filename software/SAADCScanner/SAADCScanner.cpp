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
// Driver to use the SAADC peripheral to continously scan the configured analog pins and track the latest stats
// (min, max, mean, latest) of analog measurements made since the last read request.
#include "SAADCScanner.h"

SAADCScanner* SAADCScanner::m_pThis = NULL;

SAADCScanner::SAADCScanner(nrf_saadc_resolution_t resolution /* = NRF_SAADC_RESOLUTION_12BIT */,
                           uint8_t irqPriority /* = _PRIO_APP_LOWEST*/)
{
    // Can only instantiate one SAADCScanner object.
    ASSERT ( m_pThis == NULL );

    m_usedChannelCount = 0;
    memset(m_buffer, 0, sizeof(m_buffer));
    m_resolution = resolution;
    m_irqPriority = irqPriority;
    m_pThis = this;

    for (size_t i = 0 ; i < sizeof(m_channels)/sizeof(m_channels[0]) ; i++)
    {
        m_channels[i].m_index = i;
    }
}

bool SAADCScanner::init()
{
    nrf_drv_saadc_config_t config =
    {
        .resolution = m_resolution,
        .oversample = NRF_SAADC_OVERSAMPLE_DISABLED,
        .interrupt_priority = m_irqPriority,
        .low_power_mode = false
    };

    uint32_t result = nrf_drv_saadc_init(&config, eventHandler);
    if (result != NRF_SUCCESS)
    {
        return false;
    }

    return true;
}

SAADCScanner::Channel* SAADCScanner::addChannel(uint8_t pin,
                                                nrf_saadc_resistor_t pullUpDown /* = NRF_SAADC_RESISTOR_DISABLED */,
                                                nrf_saadc_gain_t gain /* =NRF_SAADC_GAIN1_4 */,
                                                nrf_saadc_reference_t reference /* =NRF_SAADC_REFERENCE_VDD4 */,
                                                nrf_saadc_acqtime_t acquisitionTime /* =NRF_SAADC_ACQTIME_3US */,
                                                int16_t lowLimit /* =NRF_DRV_SAADC_LIMITH_DISABLED */,
                                                int16_t highLimit /* =NRF_DRV_SAADC_LIMITL_DISABLED */,
                                                ISAADCScannerNotification* pNotification /* = NULL */)
{
    Channel* pChannel = findFreeChannel();
    if (pChannel == NULL)
    {
        return NULL;
    }

    bool result = pChannel->init(pin, pullUpDown, gain, reference, acquisitionTime, lowLimit, highLimit, pNotification, m_irqPriority);
    if (!result)
    {
        return NULL;
    }

    return pChannel;
}

SAADCScanner::Channel* SAADCScanner::findFreeChannel()
{
    if (m_usedChannelCount >= ARRAY_SIZE(m_channels))
    {
        return NULL;
    }
    return &m_channels[m_usedChannelCount++];
}

bool SAADCScanner::startScanning()
{
    uint32_t result;

    result = nrf_drv_saadc_buffer_convert(m_buffer, m_usedChannelCount);
    if (result != NRF_SUCCESS)
    {
        return false;
    }
    result = nrf_drv_saadc_sample();
    if (result != NRF_SUCCESS)
    {
        return false;
    }

    return true;
}

void SAADCScanner::eventHandler(nrf_drv_saadc_evt_t const * pEvent)
{
    bool result;

    switch (pEvent->type)
    {
        case NRF_DRV_SAADC_EVT_DONE:
            ASSERT ( pEvent->data.done.size == m_pThis->m_usedChannelCount );
            for (size_t i = 0 ; i < pEvent->data.done.size ; i++)
            {
                m_pThis->m_channels[i].update(pEvent->data.done.p_buffer[i]);
            }
            result = m_pThis->startScanning();
            ASSERT ( result );
            break;

        case NRF_DRV_SAADC_EVT_LIMIT:
            ASSERT ( pEvent->data.limit.channel < m_pThis->m_usedChannelCount );
            m_pThis->m_channels[pEvent->data.limit.channel].limitExceeded(pEvent->data.limit.limit_type);
            break;

        case NRF_DRV_SAADC_EVT_CALIBRATEDONE:
            break;
    }
}



SAADCScanner::Channel::Channel()
{
    clear();
    m_pNotification = NULL;
    m_irqPriority = 0;
    m_origPriority = 0;
}

bool SAADCScanner::Channel::init(uint8_t pin,
                   nrf_saadc_resistor_t pullUpDown,
                   nrf_saadc_gain_t gain,
                   nrf_saadc_reference_t reference,
                   nrf_saadc_acqtime_t acquisitionTime,
                   int16_t lowLimit,
                   int16_t highLimit,
                   ISAADCScannerNotification* pNotification,
                   uint8_t irqPriority)
{
    nrf_saadc_input_t analogPin = NRF_SAADC_INPUT_DISABLED;
    switch (pin)
    {
        case 2:
            analogPin = NRF_SAADC_INPUT_AIN0;
            break;
        case 3:
            analogPin = NRF_SAADC_INPUT_AIN1;
            break;
        case 4:
            analogPin = NRF_SAADC_INPUT_AIN2;
            break;
        case 5:
            analogPin = NRF_SAADC_INPUT_AIN3;
            break;
        case 28:
            analogPin = NRF_SAADC_INPUT_AIN4;
            break;
        case 29:
            analogPin = NRF_SAADC_INPUT_AIN5;
            break;
        case 30:
            analogPin = NRF_SAADC_INPUT_AIN6;
            break;
        case 31:
            analogPin = NRF_SAADC_INPUT_AIN7;
            break;
        default:
            // Not a valid Analog In pin.
            ASSERT ( false );
            return false;
    }
    m_irqPriority = irqPriority;
    m_pNotification = pNotification;

    nrf_saadc_channel_config_t config =
    {
        .resistor_p = pullUpDown,
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
        .gain = gain,
        .reference = reference,
        .acq_time = acquisitionTime,
        .mode = NRF_SAADC_MODE_SINGLE_ENDED,
        .burst = NRF_SAADC_BURST_DISABLED,
        .pin_p = analogPin,
        .pin_n = NRF_SAADC_INPUT_DISABLED
    };
    uint32_t result = nrf_drv_saadc_channel_init(m_index, &config);
    if (result != NRF_SUCCESS)
    {
        return false;
    }
    nrf_drv_saadc_limits_set(m_index, lowLimit, highLimit);

    return true;
}

void SAADCScanner::Channel::update(int16_t latestSample)
{
    m_count++;
    m_sum += latestSample;
    m_latest = latestSample;
    if (latestSample < m_min)
    {
        m_min = latestSample;
    }
    if (latestSample > m_max)
    {
        m_max = latestSample;
    }
}

void SAADCScanner::Channel::limitExceeded(nrf_saadc_limit_t limitHit)
{
    switch (limitHit)
    {
        case NRF_SAADC_LIMIT_LOW:
            m_lowLimitExceeded = true;
            break;
        case NRF_SAADC_LIMIT_HIGH:
            m_highLimitExceeded = true;
            break;
    }
    if (m_pNotification != NULL)
    {
        m_pNotification->notifyLimitExceeded(m_lowLimitExceeded, m_highLimitExceeded);
    }
}
