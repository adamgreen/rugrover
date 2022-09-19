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
// Driver for the FXOS8700CQ 3DoF accelerometer/magnetometer.
#include <nrf_assert.h>
#include <nrf_delay.h>
#include "FXOS8700CQ.h"


/* I2C Registers Addresses */
#define STATUS              0x00
#define OUT_X_MSB           0x01
#define OUT_X_LSB           0x02
#define OUT_Y_MSB           0x03
#define OUT_Y_LSB           0x04
#define OUT_Z_MSB           0x05
#define OUT_Z_LSB           0x06
#define F_SETUP             0x09
#define TRIG_CFG            0x0A
#define SYSMOD              0x0B
#define INT_SOURCE          0x0C
#define WHO_AM_I            0x0D
#define XYZ_DATA_CFG        0x0E
#define HP_FILTER_CUTOFF    0x0F
#define PL_STATUS           0x10
#define PL_CFG              0x11
#define PL_COUNT            0x12
#define PL_BF_ZCOMP         0x13
#define PL_THS_REG          0x14
#define A_FFMT_CFG          0x15
#define A_FFMT_SRC          0x16
#define A_FFMT_THS          0x17
#define A_FFMT_COUNT        0x18
#define TRANSIENT_CFG       0x1D
#define TRANSIENT_SRC       0x1E
#define TRANSIENT_THS       0x1F
#define TRANSIENT_COUNT     0x20
#define PULSE_CFG           0x21
#define PULSE_SRC           0x22
#define PULSE_THSX          0x23
#define PULSE_THSY          0x24
#define PULSE_THSZ          0x25
#define PULSE_TMLT          0x26
#define PULSE_LTCY          0x27
#define PULSE_WIND          0x28
#define ASLP_COUNT          0x29
#define CTRL_REG1           0x2A
#define CTRL_REG2           0x2B
#define CTRL_REG3           0x2C
#define CTRL_REG4           0x2D
#define CTRL_REG5           0x2E
#define OFF_X               0x2F
#define OFF_Y               0x30
#define OFF_Z               0x31
#define M_DR_STATUS         0x32
#define M_OUT_X_MSB         0x33
#define M_OUT_X_LSB         0x34
#define M_OUT_Y_MSB         0x35
#define M_OUT_Y_LSB         0x36
#define M_OUT_Z_MSB         0x37
#define M_OUT_Z_LSB         0x38
#define CMP_X_MSB           0x39
#define CMP_X_LSB           0x3A
#define CMP_Y_MSB           0x3B
#define CMP_Y_LSB           0x3C
#define CMP_Z_MSB           0x3D
#define CMP_Z_LSB           0x3E
#define M_OFF_X_MSB         0x3F
#define M_OFF_X_LSB         0x40
#define M_OFF_Y_MSB         0x41
#define M_OFF_Y_LSB         0x42
#define M_OFF_Z_MSB         0x43
#define M_OFF_Z_LSB         0x44
#define MAX_X_MSB           0x45
#define MAX_X_LSB           0x46
#define MAX_Y_MSB           0x47
#define MAX_Y_LSB           0x48
#define MAX_Z_MSB           0x49
#define MAX_Z_LSB           0x4A
#define MIN_X_MSB           0x4B
#define MIN_X_LSB           0x4C
#define MIN_Y_MSB           0x4D
#define MIN_Y_LSB           0x4E
#define MIN_Z_MSB           0x4F
#define MIN_Z_LSB           0x50
#define TEMP                0x51
#define M_THS_CFG           0x52
#define M_THS_SRC           0x53
#define M_THS_X_MSB         0x54
#define M_THS_X_LSB         0x55
#define M_THS_Y_MSB         0x56
#define M_THS_Y_LSB         0x57
#define M_THS_Z_MSB         0x58
#define M_THS_Z_LSB         0x59
#define M_THS_COUNT         0x5A
#define M_CTRL_REG1         0x5B
#define M_CTRL_REG2         0x5C
#define M_CTRL_REG3         0x5D
#define M_INT_SRC           0x5E
#define A_VECM_CFG          0x5F
#define A_VECM_THS_MSB      0x60
#define A_VECM_THS_LSB      0x61
#define A_VECM_CNT          0x62
#define A_VECM_INITX_MSB    0x63
#define A_VECM_INITX_LSB    0x64
#define A_VECM_INITY_MSB    0x65
#define A_VECM_INITY_LSB    0x66
#define A_VECM_INITZ_MSB    0x67
#define A_VECM_INITZ_LSB    0x68
#define M_VECM_CFG          0x69
#define M_VECM_THS_MSB      0x6A
#define M_VECM_THS_LSB      0x6B
#define M_VECM_CNT          0x6C
#define M_VECM_INITX_MSB    0x6D
#define M_VECM_INITX_LSB    0x6E
#define M_VECM_INITY_MSB    0x6F
#define M_VECM_INITY_LSB    0x70
#define M_VECM_INITZ_MSB    0x71
#define M_VECM_INITZ_LSB    0x72
#define A_FFMT_THS_X_MSB    0x73
#define A_FFMT_THS_X_LSB    0x74
#define A_FFMT_THS_Y_MSB    0x75
#define A_FFMT_THS_Y_LSB    0x76
#define A_FFMT_THS_Z_MSB    0x77
#define A_FFMT_THS_Z_LSB    0x78

/* XYZ_DATA_CFG bits */
/*  High Pass Filter enable when set to 1. */
#define HPF_OUT             (1 << 4)
/*  Full Scale setting 2/4/8g. */
#define FS_2G               0x0
#define FS_4G               0x1
#define FS_8G               0x2

/* CTRL_REG1 bits */
/*  Auto wakeup sample rate. */
#define ASLP_RATE_SHIFT     6
#define ASLP_RATE_50HZ      (0x0 << ASLP_RATE_SHIFT)
#define ASLP_RATE_12_5HZ    (0x1 << ASLP_RATE_SHIFT)
#define ASLP_RATE_6_25HZ    (0x2 << ASLP_RATE_SHIFT)
#define ASLP_RATE_1_56HZ    (0x3 << ASLP_RATE_SHIFT)
/*  Output Data Rate. */
#define DR_SHIFT            3
#define DR_800_HZ           (0x0 << DR_SHIFT)
#define DR_400_HZ           (0x1 << DR_SHIFT)
#define DR_200_HZ           (0x2 << DR_SHIFT)
#define DR_100_HZ           (0x3 << DR_SHIFT)
#define DR_50_HZ            (0x4 << DR_SHIFT)
#define DR_12_5_HZ          (0x5 << DR_SHIFT)
#define DR_6_25_HZ          (0x6 << DR_SHIFT)
#define DR_1_5625_HZ        (0x7 << DR_SHIFT)
/*  Low Noise enable when set to 1. */
#define LNOISE              (1 << 2)
/*  Fast Read Mode will only read 8-bits per axis when set to 1. */
#define F_READ              (1 << 1)
/*  Active mode enabled when set to 1. */
#define ACTIVE              (1 << 0)

/* CTRL_REG2 bits */
/*  Self test enabled when set to 1. */
#define ST                          (1 << 7)
/*  Reset device when set to 1. */
#define RST                         (1 << 6)
/*  Sleep mode sampling rate selection. */
#define SMODS_SHIFT                 3
#define SMODS_NORMAL                (0 << SMODS_SHIFT)
#define SMODS_LOW_NOISE_LOW_POWER   (1 << SMODS_SHIFT)
#define SMODS_HIGH_RESOLUTION       (2 << SMODS_SHIFT)
#define SMODS_LOW_POWER             (3 << SMODS_SHIFT)
/*  Auto Sleep mode enabled when set to 1. */
#define SLPE                        (1 << 2)
/*  Active mode sampling rate selection. */
#define MODS_SHIFT                  0
#define MODS_NORMAL                 (0 << MODS_SHIFT)
#define MODS_LOW_NOISE_LOW_POWER    (1 << MODS_SHIFT)
#define MODS_HIGH_RESOLUTION        (2 << MODS_SHIFT)
#define MODS_LOW_POWER              (3 << MODS_SHIFT)

/* CTRL_REG3 bits */
#define FIFO_GATE                   (1 << 7)
#define WAKE_TRANS                  (1 << 6)
#define WAKE_LNDPRT                 (1 << 5)
#define WAKE_PULSE                  (1 << 4)
#define WAKE_FFMT                   (1 << 3)
#define WAKE_A_VECM                 (1 << 2)
#define IPOL                        (1 << 1)
#define PP_OD                       (1 << 0)

/* CTRL_REG4 bits */
#define INT_EN_ASLP                 (1 << 7)
#define INT_EN_FIFO                 (1 << 6)
#define INT_EN_TRANS                (1 << 5)
#define INT_EN_LNDPRT               (1 << 4)
#define INT_EN_PULSE                (1 << 3)
#define INT_EN_FFMT                 (1 << 2)
#define INT_EN_A_VECM               (1 << 1)
/*  Interrupt on data ready enabled when set to 1. */
#define INT_EN_DRDY                 (1 << 0)

/* CTRL_REG5 bits */
#define INT_CFG_ASLP                (1 << 7)
#define INT_CFG_FIFO                (1 << 6)
#define INT_CFG_TRANS               (1 << 5)
#define INT_CFG_LNDPRT              (1 << 4)
#define INT_CFG_PULSE               (1 << 3)
#define INT_CFG_FFMT                (1 << 2)
#define INT_CFG_A_VECM              (1 << 1)
/*  Data-ready interrupt is routed to INT1 (if set to 1) or INT2 (if set to 0). */
#define INT_CFG_DRDY                (1 << 0)

/* M_CTRL_REG1 bits */
/*  Magnetic hard-iron offset auto-calibration enabled when set to 1. */
#define M_ACAL              (1 << 7)
/*  One-shot magnetic reset is enabled when set to 1. */
#define M_RST               (1 << 6)
/*  One-shot triggered magnetic measurement enabled when set to 1. */
#define M_OST               (1 << 5)
/* Oversample ratio (OSR) for magnetometer data. */
#define M_OS_SHIFT          2
#define M_OS_LOWEST         (0 << M_OS_SHIFT)
#define M_OS_HIGHEST        (7 << M_OS_SHIFT)
/* Magnetometer/Accelerometer/Hybrid mode selection. */
#define M_HMS_ACCEL         0
#define M_HSM_MAG           1
#define M_HSM_HYBRID        3

/* M_CTRL_REG2 bits */
/*  Hybrid auto-increment mode enabled when set to 1. */
#define HYB_AUTOINC_MODE    (1 << 5)
/*  Magnetic measurement max/min detection function disabled when set to 1. */
#define M_MAXIMUM_DIS       (1 << 4)
/*  Magnetic measurement min/max detection function disable using the magnetic threshold event trigger when set to 1. */
#define M_MAXIMUM_DIS_THS   (1 << 3)
/*  Magnetic measurement min/max detection function reset when set to 1. */
#define M_MAXIMUM_RST       (1 << 2)
/*  Magnetic sensor reset (degaussing) frequency. */
#define M_RST_CNT_1         0
#define M_RST_CNT_16        1
#define M_RST_CNT_512       2
#define M_RST_CNT_DISABLED  3


FXOS8700CQ::FXOS8700CQ(int32_t sampleRate_Hz, I2CAsync* pI2CAsync, int address /* = 0x1F */)
: SensorBase(sampleRate_Hz, pI2CAsync, address)
{
    m_pAccelVector = NULL;
    m_pMagVector = NULL;
}

bool FXOS8700CQ::init()
{
    bool result = false;

    reset();
    result = initMagnetometer();
    if (!result)
    {
        return false;
    }
    result = initAccelerometer(m_sampleRate_Hz);
    return result;
}

void FXOS8700CQ::reset()
{
    // First reset the device to make sure that it is in standby mode.
    // NOTE: Ignore any I2C error as a reset will truncate the ACK.
    writeRegister(CTRL_REG2, RST);

    // Can take 1 ms for reset to occur.
    nrf_delay_ms(1);
}

bool FXOS8700CQ::initMagnetometer()
{
    // Set magnetometer to highest oversample rate and activate hybrid mode to read both accelerometer and
    // magnetometer together.
    bool result = writeRegister(M_CTRL_REG1, M_OS_HIGHEST | M_HSM_HYBRID);
    return result;
}

bool FXOS8700CQ::initAccelerometer(int32_t sampleRateHz)
{
    bool result = false;

    // Determine sample rate register setting appropriate for requested sample rate.
    // Sampling frequency needs to be 2x requested rate in hybrid mode.
    uint8_t rate = 0;
    switch (sampleRateHz)
    {
        case 400:
            rate = DR_800_HZ;
            break;
        case 200:
            rate = DR_400_HZ;
            break;
        case 100:
            rate = DR_200_HZ;
            break;
        default:
            // Invalid rate requested.
            return false;
    }

    // Set to +/-2g full scale.
    result = writeRegister(XYZ_DATA_CFG, FS_2G);
    if (!result)
    {
        return result;
    }
    // Configure for highest resolution, oversampling mode while in active mode.
    result = writeRegister(CTRL_REG2, MODS_HIGH_RESOLUTION);
    if (!result)
    {
        return result;
    }
    // UNDONE: Once not using the nRF52 Devkit where I am sharing the interrupt pin with the buttons on the board then
    //         I don't need to do this and don't need to enable pull-up in the GPIOTE configuration.
    // Set interrupt pins to be open-drain instead of push-pull.
    result = writeRegister(CTRL_REG3, PP_OD);
    if (!result)
    {
        return result;
    }
    // Enable data ready interrupt on INT2 pin. Will go low when data is ready.
    result = writeRegister(CTRL_REG4, INT_EN_DRDY);
    if (!result)
    {
        return result;
    }
    // Enable low noise mode for the accelerometer and set the sampling rate.
    // Also switches device back into active mode.
    result = writeRegister(CTRL_REG1, rate | LNOISE | ACTIVE);
    if (!result)
    {
        return result;
    }

    return true;
}

void FXOS8700CQ::getVectors(Vector3<int16_t>* pAccelVector, Vector3<int16_t>* pMagVector, II2CNotification* pNotify)
{
    // Can only be called in async mode.
    ASSERT ( pNotify != NULL );

    // Setup for members to track this async request.
    m_pNotify = pNotify;
    m_pAccelVector = pAccelVector;
    m_pMagVector = pMagVector;

    // Start burst read at M_OUT_X_MSB and read all 6-axis values.
    bool result = readRegisters(M_OUT_X_MSB, m_buffer, sizeof(m_buffer), this);
    if (!result)
    {
        notify(NULL);
    }
}

void FXOS8700CQ::opCompleted(bool wasSuccessful, uint32_t index)
{
    // Don't populate caller's buffers if the I2C transfer failed.
    if (wasSuccessful)
    {
        // Convert big endian data read from sensor to little endian.
        m_pMagVector->x = ((uint16_t)m_buffer[0] << 8) | m_buffer[1];
        m_pMagVector->y = ((uint16_t)m_buffer[2] << 8) | m_buffer[3];
        m_pMagVector->z = ((uint16_t)m_buffer[4] << 8) | m_buffer[5];
        m_pAccelVector->x = ((uint16_t)m_buffer[6] << 8) | m_buffer[7];
        m_pAccelVector->y = ((uint16_t)m_buffer[8] << 8) | m_buffer[9];
        m_pAccelVector->z = ((uint16_t)m_buffer[10] << 8) | m_buffer[11];

        // Sign extend 14-bit signed values into 16-bit values.
        const int shift14to16 = 16-14;
        m_pAccelVector->x = (int16_t)(m_pAccelVector->x << shift14to16) >> shift14to16;
        m_pAccelVector->y = (int16_t)(m_pAccelVector->y << shift14to16) >> shift14to16;
        m_pAccelVector->z = (int16_t)(m_pAccelVector->z << shift14to16) >> shift14to16;
    }

    // Ping caller's callback if the overall transfer is now completed.
    m_pNotify->notify(wasSuccessful);
}
