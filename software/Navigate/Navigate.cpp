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
// Class to navigate the RugRover using the DifferentialDrive driver.
#include <Navigate/Navigate.h>
#include <string.h>
#include <nrf_assert.h>


Navigate::Navigate(DifferentialDrive* pDrive, uint32_t pidFrequency_Hz, float ticksPerRotation,
                  float leftWheelDiameter_mm, float rightWheelDiameter_mm, float wheelBaseline_mm,
                  float distanceThreshold_mm, float angleThreshold_radians,
                  float headingPidKc, float headingPidTi, float headingPidTd,
                  float distancePidKc, float distancePidTi, float distancePidTd)
: m_headingPID(headingPidKc, headingPidTi, headingPidTd, 0.0f, -0.5f, 0.5f, 1.0f/pidFrequency_Hz),
  m_distancePID(distancePidKc, distancePidTi, distancePidTd, 0.0f, -0.5f, 0.5f, 1.0f/pidFrequency_Hz)
{
    ASSERT ( pDrive != NULL );

    m_pDrive = pDrive;
    m_pidFrequency_Hz = pidFrequency_Hz;
    m_ticksPerRotation = ticksPerRotation;
    m_distanceThreshold = distanceThreshold_mm;
    m_angleThreshold = angleThreshold_radians;
    m_pWaypoints = NULL;
    m_waypointCount = 0;
    setParameters(leftWheelDiameter_mm, rightWheelDiameter_mm, wheelBaseline_mm);
    reset();
    memset(&m_prevTicks, 0, sizeof(m_prevTicks));
}

void Navigate::setParameters(float leftWheelDiameter_mm, float rightWheelDiameter_mm, float wheelBaseline_mm)
{
    m_wheelDiameters_mm.left = leftWheelDiameter_mm;
    m_wheelDiameters_mm.right = rightWheelDiameter_mm;
    m_speedConversions.left = calculateMetersPerSecondToTicksPerSample(leftWheelDiameter_mm);
    m_speedConversions.right = calculateMetersPerSecondToTicksPerSample(rightWheelDiameter_mm);
    m_distanceConversions.left = calculateTicksToMM(leftWheelDiameter_mm);
    m_distanceConversions.right = calculateTicksToMM(rightWheelDiameter_mm);
    m_wheelBaseline_mm = wheelBaseline_mm;
}

void Navigate::reset()
{
    memset(&m_currentPosition, 0, sizeof(m_currentPosition));
    m_waypointIndex = 0;
    m_waypointState = ROTATE_TO_NEXT;
    m_distancePID.reset();
    m_headingPID.reset();
}

void Navigate::update()
{
    if (!m_pDrive->isEnabled() || m_waypointIndex >= m_waypointCount)
    {
        return;
    }

    updateCurrentPosition();

    PositionDelta delta;
    DriveFloatValues velocities =
    {
        .left = 0.0f,
        .right = 0.0f
    };
    switch (m_waypointState)
    {
        case ROTATE_TO_NEXT:
        {
            delta = deltaToNextWaypoint();
            if (fabsf(constrainAngle(delta.angle-m_currentPosition.heading)) < m_angleThreshold)
            {
                m_waypointState = DRIVE_TO_NEXT;
                break;
            }

            m_headingPID.updateSetPoint(delta.angle);
            float controlOutput = m_headingPID.compute(m_currentPosition.heading);
            velocities.left = controlOutput;
            velocities.right = -controlOutput;
            break;
        }
        case DRIVE_TO_NEXT:
        {
            const float ninetyDegrees = 90.0f * DEGREE_TO_RADIAN;
            delta = deltaToNextWaypoint();
            m_headingPID.updateSetPoint(delta.angle);
            float anglePortion = m_headingPID.compute(m_currentPosition.heading);
            m_distancePID.updateSetPoint(delta.distance);
            float distancePortion = m_distancePID.compute(0);
            float angleDelta = constrainAngle(delta.angle-m_currentPosition.heading);
            if (delta.distance < m_distanceThreshold || fabsf(angleDelta) > ninetyDegrees)
            {
                m_waypointState = ROTATE_TO_HEADING;
                break;
            }
            velocities.left = distancePortion + anglePortion;
            velocities.right = distancePortion - anglePortion;
            break;
        }
        case ROTATE_TO_HEADING:
        {
            float desiredHeading = m_pWaypoints[m_waypointIndex].heading;
            float currentHeading = m_currentPosition.heading;
            float angleDelta = isnan(desiredHeading) ? 0.0f : constrainAngle(desiredHeading - currentHeading);
            if (fabsf(angleDelta) < m_angleThreshold)
            {
                m_waypointIndex++;
                m_waypointState = ROTATE_TO_NEXT;
                break;
            }

            m_headingPID.updateSetPoint(desiredHeading);
            float controlOutput = m_headingPID.compute(currentHeading);
            velocities.left = controlOutput;
            velocities.right = -controlOutput;
            break;
        }
    }

    velocities.left = roundMagnitudeUpTo1(velocities.left * m_speedConversions.left);
    velocities.right = roundMagnitudeUpTo1(velocities.right * m_speedConversions.right);
    m_pDrive->setVelocity(velocities);
}

float Navigate::roundMagnitudeUpTo1(float velocity)
{
    if (velocity > 0.0f && velocity < 1.0f)
    {
        velocity = 1.0f;
    }
    else if (velocity < 0.0f && velocity > -1.0f)
    {
        velocity = -1.0f;
    }

    return velocity;
}

void Navigate::drive(DriveFloatValues& velocities_mps)
{
    if (!m_pDrive->isEnabled())
    {
        return;
    }

    updateCurrentPosition();
    DriveFloatValues velocities =
    {
        .left = velocities_mps.left * m_speedConversions.left,
        .right = velocities_mps.right * m_speedConversions.right
    };
    m_pDrive->setVelocity(velocities);
}

void Navigate::updateCurrentPosition()
{
    DifferentialDrive::DriveStats stats = m_pDrive->getStats();
    int32_t leftDiff = stats.encoderCount.left - m_prevTicks.left;
    int32_t rightDiff = stats.encoderCount.right - m_prevTicks.right;

    DriveFloatValues wheelDistances;
    wheelDistances.left = (float)leftDiff * m_distanceConversions.left;
    wheelDistances.right = (float)rightDiff * m_distanceConversions.right;

    float linearDelta = (wheelDistances.left + wheelDistances.right) / 2.0f;
    float angleDelta = (wheelDistances.left - wheelDistances.right) / m_wheelBaseline_mm;

    m_currentPosition.heading = constrainAngle(m_currentPosition.heading + angleDelta);
    m_currentPosition.x += linearDelta * sinf(m_currentPosition.heading);
    m_currentPosition.y += linearDelta * cosf(m_currentPosition.heading);

    m_prevTicks = stats.encoderCount;
}

Navigate::PositionDelta Navigate::deltaToNextWaypoint()
{
    const Position* pWaypoint = &m_pWaypoints[m_waypointIndex];

    float diffY = pWaypoint->y - m_currentPosition.y;
    float diffX = pWaypoint->x - m_currentPosition.x;

    PositionDelta delta =
    {
        .distance = sqrtf(diffX*diffX + diffY*diffY),
        .angle = atan2f(diffX, diffY)
    };
    return delta;
}

void Navigate::setWaypointVelocities(float driveVelocity_mps, float turnVelocity_mps)
{
    m_distancePID.setControlLimits(-driveVelocity_mps, driveVelocity_mps);
    m_headingPID.setControlLimits(-turnVelocity_mps, turnVelocity_mps);
}

float Navigate::calculateMetersPerSecondToTicksPerSample(float wheelDiameter_mm)
{
    // m/s * 1000mm/m * 1rev/circum_mm * ticks/rev * 1s/100sample = ticks/sample
    return (1000.0f * m_ticksPerRotation) / ((float)M_PI * wheelDiameter_mm * (float)m_pidFrequency_Hz);
}

float Navigate::calculateTicksToMM(float wheelDiameter_mm)
{
    // ticks * circumference_mm/rev * 1rev/ticks
    return ((float)M_PI * wheelDiameter_mm) / m_ticksPerRotation;
}
