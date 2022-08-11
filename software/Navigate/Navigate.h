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
#ifndef NAVIGATE_H_
#define NAVIGATE_H_

#include <stdint.h>
#include <math.h>
#include <DifferentialDrive/DifferentialDrive.h>


// Macros to convert between radians and degrees.
#define RADIAN_TO_DEGREE (180.0f/(float)M_PI)
#define DEGREE_TO_RADIAN ((float)M_PI/180.0f)


class Navigate
{
    public:
        Navigate(DifferentialDrive* pDrive, float ticksPerRotation,
                 float leftWheelDiameter_mm, float rightWheelDiameter_mm, float wheelbase_mm,
                 float distanceThreshold_mm, float angleThreshold_radians, float headingRatio,
                 float headingPidKc, float headingPidTi, float headingPidTd,
                 float distancePidKc, float distancePidTi, float distancePidTd);

        void setParameters(float leftWheelDiameter_mm, float rightWheelDiameter_mm, float wheelbase_mm);
        void reset();

        struct Position
        {
            // Coordinates of this waypoint in mm.
            float x;
            float y;
            // The angle the robot should be turned to face once waypoint is reached. Can be NAN if no turn is desired.
            float heading;
        };

        void setWaypoints(const Position* pWaypoints, size_t waypointCount)
        {
            m_pWaypoints = pWaypoints;
            m_waypointCount = waypointCount;
            m_waypointIndex = 0;
            m_waypointState = ROTATE_TO_NEXT;
        }

        void setWaypointVelocities(float driveVelocity_mps, float turnVelocity_mps);

        void update();
        void drive(DriveFloatValues& velocities_mps);

        Position getCurrentPosition()
        {
            return m_currentPosition;
        }

        bool hasReachedFinalWaypoint()
        {
            return m_waypointIndex >= m_waypointCount;
        }

        void setLogBuffer(void* pLog, size_t logSize);
        bool dumpLog(const char* pFilename);

    protected:
        struct PositionDelta
        {
            float distance;
            float angle;
        };

        float roundMagnitudeUpTo1(float velocity);
        PositionDelta calculateMovementAmount(DriveValues& wheelDiffs);
        PositionDelta deltaToNextWaypoint();
        float calculateMetersPerSecondToTicksPerSample(float wheelDiameter_mm);
        float calculateTicksToMM(float wheelDiameter_mm);
        void updateCurrentPosition();

        enum WaypointState
        {
            ROTATE_TO_NEXT,
            DRIVE_TO_NEXT,
            ROTATE_TO_HEADING
        };

        DifferentialDrive*  m_pDrive;
        const Position*     m_pWaypoints;
        size_t              m_waypointCount;
        size_t              m_waypointIndex;
        WaypointState       m_waypointState;
        int8_t*             m_pLog;
        int8_t*             m_pLogCurr;
        size_t              m_logSize;
        float               m_ticksPerRotation;
        DriveFloatValues    m_wheelDiameters_mm;
        DriveFloatValues    m_speedConversions;
        DriveFloatValues    m_distanceConversions;
        float               m_wheelbase_mm;
        float               m_distanceThreshold;
        float               m_angleThreshold;
        float               m_headingRatio;
        float               m_turnVelocity_mps;
        Position            m_currentPosition;
        DriveValues         m_prevTicks;
        AnglePID            m_headingPID;
        PID                 m_distancePID;
};

#endif // NAVIGATE_H_
