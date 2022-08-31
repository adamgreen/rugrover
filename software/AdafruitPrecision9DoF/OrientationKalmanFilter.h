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
#ifndef ORIENTATION_KALMAN_FILTER_H_
#define ORIENTATION_KALMAN_FILTER_H_

#include "Quaternion.h"
#include "Vector.h"
#include "Matrix4x4.h"
#include "AdafruitPrecision9DoF.h"

struct SensorCalibratedValues
{
    Vector<float> accel;
    Vector<float> mag;
    Vector<float> gyro;
};

struct SensorCalibration
{
    float             initialVariance;
    float             gyroVariance;
    float             accelMagVariance;
    Vector<float>     gyroCoefficientA;
    Vector<float>     gyroCoefficientB;
    Vector<float>     gyroScale;
    Vector<float>     declinationCorrection;
    Vector<float>     mountingCorrection;
    Vector<int16_t>   accelMin;
    Vector<int16_t>   accelMax;
    Vector<int16_t>   magMin;
    Vector<int16_t>   magMax;
    Vector<int16_t>   accelSwizzle;
    Vector<int16_t>   magSwizzle;
    Vector<int16_t>   gyroSwizzle;
};


class OrientationKalmanFilter
{
public:
    OrientationKalmanFilter(int32_t sampleRate_Hz, const SensorCalibration* pCalibration = NULL);

    void calibrate(const SensorCalibration* pCalibration);

    SensorCalibratedValues calibrateSensorValues(const SensorValues* pRawValues);
    Quaternion             getOrientation(SensorCalibratedValues* pCalibratedValues, int sampleTime_us);
    float                  getHeading(Quaternion* pOrientation);
    float                  getYaw(Quaternion* pOrientation);
    float                  getPitch(Quaternion* pOrientation);
    float                  getRoll(Quaternion* pOrientation);

    void  reset() { m_resetRequested = true; }

protected:
    void         resetKalmanFilter(SensorCalibratedValues* pCalibratedValues);
    Quaternion   getOrientationFromAccelerometerMagnetometerMeasurements(SensorCalibratedValues* pCalibratedValues);
    static float angleFromDegreeMinuteSecond(Vector<float>* pAngle);

    SensorCalibration       m_calibration;
    SensorCalibratedValues  m_midpoints;
    SensorCalibratedValues  m_scales;
    Quaternion              m_currentOrientation;
    Matrix4x4               m_kalmanP;
    Matrix4x4               m_kalmanQ;
    Matrix4x4               m_kalmanR;
    float                   m_gyroTimeScaleFactor;
    float                   m_declinationCorrection;
    float                   m_mountingCorrection;
    int32_t                 m_sampleRate_Hz;
    bool                    m_resetRequested;
};

#endif /* ORIENTATION_KALMAN_FILTER_H_ */
