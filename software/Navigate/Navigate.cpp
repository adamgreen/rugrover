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


struct LogEntry
{
    int8_t leftTickDelta;
    int8_t rightTickDelta;
    int8_t leftMotorPower;
    int8_t rightMotorPower;
};


Navigate::Navigate(DifferentialDrive* pDrive, uint32_t pidFrequency_Hz, float ticksPerRotation,
                  float leftWheelDiameter_mm, float rightWheelDiameter_mm, float wheelbase_mm,
                  float distanceThreshold_mm, float angleThreshold_radians, float headingRatio,
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
    m_headingRatio = headingRatio;
    m_pWaypoints = NULL;
    m_waypointCount = 0;
    m_pLog = NULL;
    m_pLogCurr = NULL;
    m_logSize = 0;
    setParameters(leftWheelDiameter_mm, rightWheelDiameter_mm, wheelbase_mm);
    reset();
    memset(&m_prevTicks, 0, sizeof(m_prevTicks));
}

void Navigate::setParameters(float leftWheelDiameter_mm, float rightWheelDiameter_mm, float wheelbase_mm)
{
    m_wheelDiameters_mm.left = leftWheelDiameter_mm;
    m_wheelDiameters_mm.right = rightWheelDiameter_mm;
    m_speedConversions.left = calculateMetersPerSecondToTicksPerSample(leftWheelDiameter_mm);
    m_speedConversions.right = calculateMetersPerSecondToTicksPerSample(rightWheelDiameter_mm);
    m_distanceConversions.left = calculateTicksToMM(leftWheelDiameter_mm);
    m_distanceConversions.right = calculateTicksToMM(rightWheelDiameter_mm);
    m_wheelbase_mm = wheelbase_mm;
}

void Navigate::reset()
{
    memset(&m_currentPosition, 0, sizeof(m_currentPosition));
    m_waypointIndex = 0;
    m_waypointState = ROTATE_TO_NEXT;
    m_distancePID.reset();
    m_headingPID.reset();
    if (m_pLog != NULL)
    {
        setLogBuffer(m_pLog, m_logSize);
    }
}

void Navigate::update()
{
    if (!m_pDrive->isEnabled())
    {
        return;
    }

    updateCurrentPosition();

    DriveFloatValues velocities =
    {
        .left = 0.0f,
        .right = 0.0f
    };

    if (m_waypointIndex < m_waypointCount)
    {
        PositionDelta delta;
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

                m_distancePID.updateSetPoint(delta.distance);
                float distancePortion = m_distancePID.compute(0);

                float headingLimit = distancePortion * m_headingRatio;
                m_headingPID.setControlLimits(-headingLimit, headingLimit);
                m_headingPID.updateSetPoint(delta.angle);
                float anglePortion = m_headingPID.compute(m_currentPosition.heading);

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
    DriveValues wheelDiffs =
    {
        .left = stats.encoderCount.left - m_prevTicks.left,
        .right = stats.encoderCount.right - m_prevTicks.right
    };
    PositionDelta delta = calculateMovementAmount(wheelDiffs);
    m_currentPosition.heading = constrainAngle(m_currentPosition.heading + delta.angle);
    m_currentPosition.x += delta.distance * sinf(m_currentPosition.heading);
    m_currentPosition.y += delta.distance * cosf(m_currentPosition.heading);

    m_prevTicks = stats.encoderCount;

    // Log the particulars for sample to the log buffer if one has been provided.
    if (m_pLog != NULL)
    {
        LogEntry* pCurr = (LogEntry*)m_pLogCurr;
        pCurr->leftTickDelta = wheelDiffs.left;
        pCurr->rightTickDelta = wheelDiffs.right;
        pCurr->leftMotorPower = stats.power.left;
        pCurr->rightMotorPower = stats.power.right;
        pCurr++;
        if ((int8_t*)pCurr >= m_pLog + m_logSize)
        {
            pCurr = (LogEntry*)m_pLog;
        }
        m_pLogCurr = (int8_t*)pCurr;
    }
}

Navigate::PositionDelta Navigate::calculateMovementAmount(DriveValues& wheelDiffs)
{
    DriveFloatValues wheelDistances;
    wheelDistances.left = (float)wheelDiffs.left * m_distanceConversions.left;
    wheelDistances.right = (float)wheelDiffs.right * m_distanceConversions.right;

    PositionDelta delta;
    delta.distance = (wheelDistances.left + wheelDistances.right) / 2.0f;
    delta.angle = (wheelDistances.left - wheelDistances.right) / m_wheelbase_mm;

    return delta;
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

void Navigate::setLogBuffer(void* pLog, size_t logSize)
{
    ASSERT ( pLog && (logSize % sizeof(LogEntry)) == 0 );
    m_pLog = m_pLogCurr = (int8_t*)pLog;
    m_logSize = logSize;
    memset(m_pLog, 127, logSize);
}

bool Navigate::dumpLog(const char* pFilename)
{
    if (m_pLog == NULL)
    {
        return false;
    }

    FILE* pFile = fopen(pFilename, "w");
    if (pFile == NULL)
    {
        fprintf(stderr, "error: Failed to open %s\n", pFilename);
        return false;
    }

    fprintf(pFile, "Time(ms),Left Wheel(delta ticks),Right Wheel(delta ticks),Left Motor(%%),Right Motor(%%),"
                   "Left Wheel(ticks),Right Wheel(ticks),X(mm),Y(mm),Heading(degrees)\n");
    LogEntry* pStop = (LogEntry*)m_pLogCurr;
    LogEntry* pCurr = (LogEntry*)m_pLogCurr;
    LogEntry* pLast = (LogEntry*)(m_pLog + m_logSize - sizeof(LogEntry));
    uint32_t msec = micros() / 1000;
    DriveValues ticks = m_prevTicks;
    Position position = m_currentPosition;

    do
    {
        // Decrement log entry pointer, wrapping around to end of log as needed.
        pCurr--;
        if ((int8_t*)pCurr < m_pLog)
        {
            pCurr = pLast;
        }

        if (pCurr->leftMotorPower == 127)
        {
            // Motor power can't be 127%. This is the fill value which means this entry hasn't been filled yet so stop.
            break;
        }
        fprintf(pFile, "%lu,%d,%d,%d,%d,%ld,%ld,%f,%f,%f\n",
                msec,
                pCurr->leftTickDelta, pCurr->rightTickDelta,
                pCurr->leftMotorPower, pCurr->rightMotorPower,
                ticks.left, ticks.right,
                position.x, position.y, position.heading * RADIAN_TO_DEGREE);

        // We are dumping the log backwards, so rewind the time, tick count, position, etc.
        msec -= 1000 / m_pidFrequency_Hz;
        ticks.left -= pCurr->leftTickDelta;
        ticks.right -= pCurr->rightTickDelta;
        DriveValues wheelDiffs =
        {
            .left = pCurr->leftTickDelta,
            .right = pCurr->rightTickDelta
        };
        PositionDelta delta = calculateMovementAmount(wheelDiffs);
        position.x -= delta.distance * sinf(position.heading);
        position.y -= delta.distance * cosf(position.heading);
        position.heading = constrainAngle(position.heading - delta.angle);
    } while (pCurr != pStop);

    fclose(pFile);
    return true;
}
