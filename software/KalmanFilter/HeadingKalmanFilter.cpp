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
/* Implements a Kalman filter which takes the wheel odometry based heading change deltas and yaw gyro rate measurements
   to generate a heading.

  * The code for this filter was based on what I learned from reading "Kalman Filter for Beginners with MATLAB Examples"
    by Phil Kim.
*/
#include <math.h>
#include "HeadingKalmanFilter.h"
#include <LinearAlgebra/Matrix3x2.h>


HeadingKalmanFilter::HeadingKalmanFilter(const HeadingCalibration* pCalibration /* = NULL */) :
  m_kalmanA(1.0f, 1.0f/100.0f, -1.0/100.0f,
            0.0f, 1.0f,        0.0f,
            0.0f, 0.0f,        1.0f),
  m_kalmanH(0.0f, 1.0f/100.0f, -1.0f/100.0f,
            0.0f, 1.0f,        0.0f)

{
    m_resetRequested = true;
    calibrate(pCalibration);
}

void HeadingKalmanFilter::calibrate(const HeadingCalibration* pCalibration)
{
    if (!pCalibration)
    {
        return;
    }
    m_calibration = *pCalibration;

    // Convert gyro scale to radians and pre-divide to improve performance at runtime.
    m_calibration.gyroScale = ((1.0f / m_calibration.gyroScale) * (float)M_PI) / 180.0f;

    // System model covariance matrices which don't change are placed on diagonal.
    m_kalmanQ.clear();
    for (int i = 0 ; i < 3 ; i++)
    {
        m_kalmanQ.m_data[i][i] = m_calibration.modelVariances[i];
    }
    m_kalmanR.clear();
    for (int i = 0 ; i < 2 ; i++)
    {
        m_kalmanR.m_data[i][i] = m_calibration.sensorVariances[i];
    }
}

HeadingModelState HeadingKalmanFilter::update(float odometryHeadingDelta_rad, float rawGyroRate, float samplePeriod_sec)
{
    float gyroRate_rad_per_sec = rawGyroRate * m_calibration.gyroScale;

    if (m_resetRequested)
    {
        resetKalmanFilter(gyroRate_rad_per_sec);
        return m_currentHeading;
    }

    // Calculate Kalman prediction for x and error.
    // xPredicted = A * prevXEstimate
    HeadingModelState xPredicted = applySystemModel(m_currentHeading, samplePeriod_sec);

    // PPredicted = A * prevPEstimate * Atranspose + Q
    Matrix3x3 PPredicted = m_kalmanA.multiply(m_kalmanP).multiplyTransposed(m_kalmanA).addDiagonal(m_kalmanQ);

    // Update H matrix components which includes the sample period.
    m_kalmanH.m_01 = samplePeriod_sec;
    m_kalmanH.m_02 = -samplePeriod_sec;

    // Calculate the Kalman gain.
    // K = PPredicted * Htranspose * I/(H * PPredicted * Htranspose + R)
    Matrix2x2 temp1 = m_kalmanH.multiply(PPredicted).multiplyTransposed(m_kalmanH).addDiagonal(m_kalmanR).inverse();
    Matrix3x2 K = PPredicted.multiplyTransposed(m_kalmanH).multiply(temp1);

    // Prepare the z vector from current sensor measurements.
    Vector2<float> z(odometryHeadingDelta_rad, gyroRate_rad_per_sec);

    // Calculate the Kalman estimates.
    // xEstimate = xPredicted + K*(z - H * xPredicted)
    m_currentHeading = xPredicted.add(K.multiply(z.subtract(m_kalmanH.multiply(xPredicted))));
    m_currentHeading.m_heading = constrainAngle(m_currentHeading.m_heading);
    // P = PPredicted - K * H * PPredicted
    m_kalmanP = PPredicted.subtract(K.multiply(m_kalmanH).multiply(PPredicted));

    return m_currentHeading;
}

void HeadingKalmanFilter::resetKalmanFilter(float gyroRate)
{
    // The robot must be stationary and pointed at the direction considered to be 0 when reset is called. This
    // means the current heading will be 0, the turning rate (with error) will be 0 as well, and the current gyro
    // reading is just the current drift (bias) being reported by the gyro as anything other than 0 is the bias.
    m_currentHeading = { 0.0f, 0.0f, gyroRate };

    m_kalmanP.clear();
    for (int i = 0 ; i < 3 ; i++)
    {
        m_kalmanP.m_data[i][i] = m_calibration.initialModelVariances[i];
    }

    m_resetRequested = false;
}
