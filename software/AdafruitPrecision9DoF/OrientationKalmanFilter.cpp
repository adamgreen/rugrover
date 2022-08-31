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
#include <math.h>
#include <PID/PID.h>
#include "OrientationKalmanFilter.h"


OrientationKalmanFilter::OrientationKalmanFilter(int32_t sampleRate_Hz, const SensorCalibration* pCalibration /* = NULL */)
{
    m_resetRequested = true;
    m_sampleRate_Hz = sampleRate_Hz;

    // The 0.5f is part of the math that relates gyro rates to rotation derivative and is just pre-calculated here
    // along with the time delta to improve runtime performance.
    m_gyroTimeScaleFactor = (1.0f / sampleRate_Hz) * 0.5f;

    calibrate(pCalibration);
}

void OrientationKalmanFilter::calibrate(const SensorCalibration* pCalibration)
{
    if (!pCalibration)
        return;
    m_calibration = *pCalibration;

    m_midpoints.accel.x = (m_calibration.accelMin.x + m_calibration.accelMax.x) / 2.0f;
    m_midpoints.accel.y = (m_calibration.accelMin.y + m_calibration.accelMax.y) / 2.0f;
    m_midpoints.accel.z = (m_calibration.accelMin.z + m_calibration.accelMax.z) / 2.0f;
    m_midpoints.mag.x = (m_calibration.magMin.x + m_calibration.magMax.x) / 2.0f;
    m_midpoints.mag.y = (m_calibration.magMin.y + m_calibration.magMax.y) / 2.0f;
    m_midpoints.mag.z = (m_calibration.magMin.z + m_calibration.magMax.z) / 2.0f;

    m_scales.accel.x = (m_calibration.accelMax.x - m_calibration.accelMin.x) / 2.0f;
    m_scales.accel.y = (m_calibration.accelMax.y - m_calibration.accelMin.y) / 2.0f;
    m_scales.accel.z = (m_calibration.accelMax.z - m_calibration.accelMin.z) / 2.0f,
    m_scales.mag.x = (m_calibration.magMax.x - m_calibration.magMin.x) / 2.0f;
    m_scales.mag.y = (m_calibration.magMax.y - m_calibration.magMin.y) / 2.0f;
    m_scales.mag.z = (m_calibration.magMax.z - m_calibration.magMin.z) / 2.0f;

    // Convert gyro scale to radians and pre-divide to improve performance at runtime.
    m_calibration.gyroScale.x = ((1.0f / m_calibration.gyroScale.x) * (float)M_PI) / 180.0f;
    m_calibration.gyroScale.y = ((1.0f / m_calibration.gyroScale.y) * (float)M_PI) / 180.0f;
    m_calibration.gyroScale.z = ((1.0f / m_calibration.gyroScale.z) * (float)M_PI) / 180.0f;

    // System model covariance matrices which don't change.
    float gyroVariance = m_calibration.gyroVariance;
    float accelMagVariance = m_calibration.accelMagVariance;
    m_kalmanQ.clear();
    m_kalmanR.clear();
    for (int i = 0 ; i < 4 ; i++)
    {
        m_kalmanQ.m_data[i][i] = gyroVariance;
        m_kalmanR.m_data[i][i] = accelMagVariance;
    }

    m_declinationCorrection = angleFromDegreeMinuteSecond(&m_calibration.declinationCorrection);
    m_mountingCorrection = angleFromDegreeMinuteSecond(&m_calibration.mountingCorrection);
}

float OrientationKalmanFilter::angleFromDegreeMinuteSecond(Vector<float>* pAngle)
{
    float angleInDegrees;

    if (pAngle->x >= 0.0f)
        angleInDegrees = pAngle->x + pAngle->y / 60.0f + pAngle->z / 3600.0f;
    else
        angleInDegrees = pAngle->x - pAngle->y / 60.0f - pAngle->z / 3600.0f;

    return (angleInDegrees * (float)M_PI) / 180.0f;
}

SensorCalibratedValues OrientationKalmanFilter::calibrateSensorValues(const SensorValues* pRawValues)
{
    SensorCalibratedValues calibratedValues;

    calibratedValues.accel.x = (pRawValues->accel.x - m_midpoints.accel.x) / m_scales.accel.x;
    calibratedValues.accel.y = (pRawValues->accel.y - m_midpoints.accel.y) / m_scales.accel.y;
    calibratedValues.accel.z = (pRawValues->accel.z - m_midpoints.accel.z) / m_scales.accel.z;

    calibratedValues.mag.x = (pRawValues->mag.x - m_midpoints.mag.x) / m_scales.mag.x;
    calibratedValues.mag.y = (pRawValues->mag.y - m_midpoints.mag.y) / m_scales.mag.y;
    calibratedValues.mag.z = (pRawValues->mag.z - m_midpoints.mag.z) / m_scales.mag.z;

    calibratedValues.gyro.x = pRawValues->gyro.x - (pRawValues->gyroTemperature * m_calibration.gyroCoefficientA.x + m_calibration.gyroCoefficientB.x);
    calibratedValues.gyro.y = pRawValues->gyro.y - (pRawValues->gyroTemperature * m_calibration.gyroCoefficientA.y + m_calibration.gyroCoefficientB.y);
    calibratedValues.gyro.z = pRawValues->gyro.z - (pRawValues->gyroTemperature * m_calibration.gyroCoefficientA.z + m_calibration.gyroCoefficientB.z);

    calibratedValues.gyro.x *= m_calibration.gyroScale.x;
    calibratedValues.gyro.y *= m_calibration.gyroScale.y;
    calibratedValues.gyro.z *= m_calibration.gyroScale.z;

    return calibratedValues;
}

Quaternion OrientationKalmanFilter::getOrientation(SensorCalibratedValues* pCalibratedValues, int sampleTime_us)
{
    if (m_resetRequested)
        resetKalmanFilter(pCalibratedValues);

    // The /2.0f is part of the math that relates gyro rates to rotation derivative. The rest of division accounts for
    // sample time being in us instead of seconds.
    m_gyroTimeScaleFactor = sampleTime_us / (2.0f * 1000000.0f);

    // Swizzle the axis so that gyro's axis match overall sensor setup.
    Vector<float> gyro = Vector<float>::createFromSwizzledSource(m_calibration.gyroSwizzle, pCalibratedValues->gyro);

    // Construct matrix which applies gyro rates (derivatives) to quaternion.
    // This will be the A matrix for the system model.
    // A = I + 0.5 * dt * |      0 -gyro.x -gyro.y -gyro.z |
    //                    | gyro.x       0  gyro.z -gyro.y |
    //                    | gyro.y -gyro.z       0  gyro.x |
    //                    | gyro.z  gyro.y -gyro.x       0 |
    gyro = gyro.multiply(m_gyroTimeScaleFactor);
    Matrix4x4 A(  1.0f, -gyro.x, -gyro.y, -gyro.z,
                gyro.x,    1.0f,  gyro.z, -gyro.y,
                gyro.y, -gyro.z,    1.0f,  gyro.x,
                gyro.z,  gyro.y, -gyro.x,  1.0f );

    // Calculate Kalman prediction for x and error.
    // xPredicted = A * prevXEstimate
    Quaternion xPredicted = A.multiply(m_currentOrientation);
    xPredicted.normalize();

    // PPredicted = A * prevPEstimate * Atranspose + Q
    Matrix4x4 temp1 = A.multiply(m_kalmanP);
    Matrix4x4 temp2 = temp1.multiplyTransposed(A);
    Matrix4x4 PPredicted = temp2.addDiagonal(m_kalmanQ);

    // Calculate the Kalman gain.
    // Simplified a bit since the H matrix is the identity matrix.
    // K = PPredicted * I/(PPredicted + R)
    temp1 = PPredicted.add(m_kalmanR);
    temp2 = temp1.inverse();
    Matrix4x4 K = PPredicted.multiply(temp2);

    // Fetch the accelerometer/magnetometer measurements as a quaternion.
    Quaternion z = getOrientationFromAccelerometerMagnetometerMeasurements(pCalibratedValues);
    // Flip the quaternion (q == -q for quaternions) if the angle between prediction and measurement is obtuse. Each
    // unique orientation can have two distinct quaternion representations. This code detects if the measurement is
    // using the other representation and if it is then it flips it to the use the matching representation.
    if (z.dotProduct(xPredicted) < 0.0f)
    {
        z.flip();
    }

    // Calculate the Kalman estimates.
    // Again, simplified a bit since H is the identity matrix.
    // xEstimate = xPredicted + K*(z - xPredicted)
    // P = PPredicted - K*PPredicted
    Quaternion diff = z.subtract(xPredicted);
    Quaternion correction = K.multiply(diff);
    m_currentOrientation = xPredicted.add(correction);
    m_currentOrientation.normalize();

    temp1 = K.multiply(PPredicted);
    m_kalmanP = PPredicted.subtract(temp1);

    return m_currentOrientation;
}

void OrientationKalmanFilter::resetKalmanFilter(SensorCalibratedValues* pCalibratedValues)
{
    m_kalmanP.clear();
    for (int i = 0 ; i < 4 ; i++)
        m_kalmanP.m_data[i][i] = m_calibration.initialVariance;

    m_currentOrientation = getOrientationFromAccelerometerMagnetometerMeasurements(pCalibratedValues);
    m_resetRequested = false;
}

Quaternion OrientationKalmanFilter::getOrientationFromAccelerometerMagnetometerMeasurements(SensorCalibratedValues* pCalibratedValues)
{
    // Setup gravity (down) and north vectors.
    // NOTE: The fields are swizzled to make the axis on the device match the axis on the screen.
    Vector<float> down = Vector<float>::createFromSwizzledSource(m_calibration.accelSwizzle, pCalibratedValues->accel);
    Vector<float> north = Vector<float>::createFromSwizzledSource(m_calibration.magSwizzle, pCalibratedValues->mag);

    // Project the north vector onto the earth surface plane, for which gravity is the surface normal.
    //  north.dotProduct(downNormalized) = north.magnitude * cos(theta)  NOTE: downNormalized.magnitude = 1.0f
    //   The result of this dot product is the length of the north vector when projected onto the gravity vector since
    //   the magnitude of the north vector is the hypotenuse of a right angle triangle and the unit gravity vector
    //   is the side adjacent to angle theta (the angle between gravity and north vectors).
    //  northProjectedToGravityNormal = downNormalized.multiply(north.dotProduct(downNormalized)
    //   Multiply the unit gravity vector by the magnitude previously calculated to get the vector representing the
    //   north vector after it has been projected onto the gravity vector.
    //  north = north.subtract(northProjectedToGravityNormal)
    //   Follow this projected vector down the surface normal to the plane representing the earth's surface.
    down.normalize();
    Vector<float> northProjectedToGravityNormal = down.multiply(north.dotProduct(down));
    north = north.subtract(northProjectedToGravityNormal);
    north.normalize();

    // To create a rotation matrix, we need all 3 basis vectors so calculate the vector which
    // is orthogonal to both the down and north vectors (ie. the normalized cross product).
    Vector<float> west = north.crossProduct(down);
    west.normalize();
    Quaternion rotationQuaternion = Quaternion::createFromBasisVectors(north, down, west);

    return rotationQuaternion;
}

float OrientationKalmanFilter::getHeading(Quaternion* pOrientation)
{
    // Correct compass heading for declination at location where the robot is being run.
    // Also account for how the IMU is mounted to the robot. The yaw reading is negated to
    // flip the rotations around the y-axis so that they increase as you progress from North
    // to East.
    return constrainAngle(-getYaw(pOrientation) + m_declinationCorrection + m_mountingCorrection);
}

float OrientationKalmanFilter::getYaw(Quaternion* pOrientation)
{
    float w = pOrientation->w;
    float x = pOrientation->x;
    float y = pOrientation->y;
    float z = pOrientation->z;

    return atan2f(2.0f*(x*z+y*w), 1.0f-2.0f*(x*x+y*y));
}

float OrientationKalmanFilter::getPitch(Quaternion* pOrientation)
{
    float w = pOrientation->w;
    float x = pOrientation->x;
    float y = pOrientation->y;
    float z = pOrientation->z;

    return asinf(-2.0f*(y*z-x*w));
}

float OrientationKalmanFilter::getRoll(Quaternion* pOrientation)
{
    float w = pOrientation->w;
    float x = pOrientation->x;
    float y = pOrientation->y;
    float z = pOrientation->z;

    return atan2f(2.0f*(x*y+z*w), 1.0f-2.0f*(x*x+z*z));
}
