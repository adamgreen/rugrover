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
// Driver for the FXAS21002C 3DoF gyro.
#include <nrf_assert.h>
#include <nrf_atomic.h>
#include <nrf_delay.h>
#include "FXAS21002C.h"


/* I2C Register Addresses */
#define STATUS          0x00
#define OUT_X_MSB       0x01
#define OUT_X_LSB       0x02
#define OUT_Y_MSB       0x03
#define OUT_Y_LSB       0x04
#define OUT_Z_MSB       0x05
#define OUT_Z_LSB       0x06
#define DR_STATUS       0x07
#define F_STATUS        0x08
#define F_SETUP         0x09
#define F_EVENT         0x0A
#define INT_SRC_FLAG    0x0B
#define WHO_AM_I        0x0C
#define CTRL_REG0       0x0D
#define RT_CFG          0x0E
#define RT_SRC          0x0F
#define RT_THS          0x10
#define RT_COUNT        0x11
#define TEMP            0x12
#define CTRL_REG1       0x13
#define CTRL_REG2       0x14
#define CTRL_REG3       0x15

/* CTRL_REG0 bits */
/*  Low pass filter cutoff frequency selection, varies by ODR setting. */
#define BW_SHIFT            6
#define BW_HIGHEST_FREQ     (0 << BW_SHIFT)
#define BW_MEDIUM_FREQ      (1 << BW_SHIFT)
#define BW_LOWEST_FREQ      (3 << BW_SHIFT)
/*  SPI Interface Mode */
#define SPIW_3_WIRE         (1 << 5)
/*  High pass filter cutoff frequency selection, varies by ODR setting. */
#define SEL_SHIFT           3
#define SEL_HIGHEST         (0 << SEL_SHIFT)
#define SEL_LOWEST          (3 << SEL_SHIFT)
/*  High pass filter enabled when set to 1. */
#define HPF_EN              (1 << 2)
/*  Full range selection: 250/500/1000/2000 deg/s */
#define FS_2000_DPS         0
#define FS_1000_DPS         1
#define FS_500_DPS          2
#define FS_250_DPS          3

/* CTRL_REG1 bits */
/*  Software reset started when set to 1. */
#define RST                 (1 << 6)
/*  Self test enabled when set to 1. */
#define ST                  (1 << 5)
/*  Output data rate. */
#define DR_SHIFT            2
#define DR_800_HZ           (0 << DR_SHIFT)
#define DR_400_HZ           (1 << DR_SHIFT)
#define DR_200_HZ           (2 << DR_SHIFT)
#define DR_100_HZ           (3 << DR_SHIFT)
#define DR_50_HZ            (4 << DR_SHIFT)
#define DR_25_HZ            (5 << DR_SHIFT)
#define DR_12_5_HZ          (6 << DR_SHIFT)
/*  Active mode enabled when set to 1. */
#define ACTIVE              (1 << 1)
/*  Ready mode enabled when set to 1. */
#define READY               (1 << 0)


FXAS21002C::FXAS21002C(int32_t sampleRate_Hz, I2CAsync* pI2CAsync, int address /* = 0x21 */)
: SensorBase(sampleRate_Hz, pI2CAsync, address)
{
    m_pVector = NULL;
    m_pTemperature = NULL;
}

bool FXAS21002C::init()
{
    return initGyro();
}

bool FXAS21002C::initGyro()
{
    bool result = false;

    // Determine register setting appropriate for requested sample rate.
    uint8_t rate = 0;
    switch (m_sampleRate_Hz)
    {
        case 400:
            rate = DR_400_HZ;
            break;
        case 200:
            rate = DR_200_HZ;
            break;
        case 100:
            rate = DR_100_HZ;
            break;
        default:
            // Invalid rate requested.
            return false;
    }

    // Reset to make sure that device is in standby mode.
    // NOTE: Ignore any I2C error as a reset will truncate the ACK.
    writeRegister(CTRL_REG1, RST);

    // Wait a bit for reset to occur.
    nrf_delay_ms(1);

    // Configure for highest low pass filter frequency setting, disable high pass filter, and
    // set full scale resolution to 250 degrees/s for highest resolution.
    result = writeRegister(CTRL_REG0, BW_HIGHEST_FREQ | FS_250_DPS);
    if (!result)
    {
        return result;
    }

    // Set sampling rate to desired frequency.
    // Also switches device back into active mode.
    result = writeRegister(CTRL_REG1, rate | ACTIVE);
    if (!result)
    {
        return result;
    }

    // Takes 60ms to enter active mode.
    nrf_delay_ms(60);

    return true;
}

void FXAS21002C::getVector(Vector3<int16_t>* pVector, int16_t* pTemperature, II2CNotification* pNotify)
{
    // Can only be called in async mode.
    ASSERT ( pNotify != NULL );

    // Setup for members to track this async request.
    m_pNotify = pNotify;
    m_pVector = pVector;
    m_pTemperature = pTemperature;

    // Read the latest gyro x, y, z measurements.
    bool result = readRegisters(OUT_X_MSB, m_bigEndianData, sizeof(m_bigEndianData), this);
    if (!result)
    {
        notify(NULL);
    }

    // Read the latest temperature measurement.
    result = readRegister(TEMP, (uint8_t*)&m_temp, this);
    if (!result)
    {
        notify(NULL);
    }
}

void FXAS21002C::opCompleted(bool wasSuccessful, uint32_t index)
{
    // Don't populate data to caller's buffers if the operation failed.
    if (wasSuccessful)
    {
        switch (index % 2)
        {
            case 1:
                // Data returned is big endian so byte swap.
                m_pVector->x = (m_bigEndianData[0] << 8) | m_bigEndianData[1];
                m_pVector->y = (m_bigEndianData[2] << 8) | m_bigEndianData[3];
                m_pVector->z = (m_bigEndianData[4] << 8) | m_bigEndianData[5];
                break;
            case 0:
                // Just needs a simple copy for this 1-byte transfer.
                *m_pTemperature = m_temp;
                break;
        }
    }

    // Ping caller's callback if the overall transfer is now completed.
    if ((index % 2) == 0)
    {
        uint32_t errorCount = m_errorCount;
        nrf_atomic_u32_sub(&m_errorCount, errorCount);
        ASSERT ( errorCount <= 2 );

        m_pNotify->notify(errorCount == 0);
    }
}
