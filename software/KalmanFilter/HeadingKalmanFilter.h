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
#ifndef HEADING_KALMAN_FILTER_H_
#define HEADING_KALMAN_FILTER_H_

#include <nrf.h>
#include <LinearAlgebra/Vector2.h>
#include <LinearAlgebra/Vector3.h>
#include <LinearAlgebra/Matrix2x2.h>
#include <LinearAlgebra/Matrix2x3.h>
#include <LinearAlgebra/Matrix3x3.h>
#include <PID/PID.h>

// Give more descriptive names of system state variables to x, y, z components of Vector3 class.
// They are in radians for heading and radians/second for the others.
#define m_heading               x
#define m_angleRateWithError    y
#define m_angleRateError        z

typedef Vector3<float> HeadingModelState;

struct HeadingCalibration
{
    float           gyroScale;
    Vector3<float>  initialModelVariances;
    Vector3<float>  modelVariances;
    Vector2<float>  sensorVariances;
};

class HeadingKalmanFilter
{
public:
    HeadingKalmanFilter(const HeadingCalibration* pCalibration = NULL);

    void calibrate(const HeadingCalibration* pCalibration);
    void reset() { m_resetRequested = true; }

    HeadingModelState update(float odometryHeadingDelta_rad, float rawGyroRate, float samplePeriod_sec);

    float getHeading() { return m_currentHeading.m_heading; }
    float getGyroRate() { return m_currentHeading.m_angleRateWithError; }
    float getGyroDrift() { return m_currentHeading.m_angleRateError; }

    Matrix3x3 getErrorMatrix() { return m_kalmanP; }

    // Normally not called by higher level code but useful for measuring variances for Q matrix.
    HeadingModelState applySystemModel(const HeadingModelState& currState, float samplePeriod_sec)
    {
        // Update the time based components of the A matrix with the latest interval.
        m_kalmanA.m_01 = samplePeriod_sec;
        m_kalmanA.m_02 = -samplePeriod_sec;

        HeadingModelState newState = m_kalmanA.multiply(currState);
        newState.m_heading = constrainAngle(newState.m_heading);

        return newState;
    }

protected:
    void resetKalmanFilter(float gyroRate);

    HeadingCalibration      m_calibration;
    Vector3<float>          m_currentHeading;
    Matrix3x3               m_kalmanA;
    Matrix2x3               m_kalmanH;
    Matrix3x3               m_kalmanP;
    Matrix3x3               m_kalmanQ;
    Matrix2x2               m_kalmanR;
    bool                    m_resetRequested;
};

#endif /* HEADING_KALMAN_FILTER_H_ */
