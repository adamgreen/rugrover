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
class HeadingSensorCalibration
{
  public int         rate;
  public float       initialVariance;
  public float       gyroVariance;
  public float       accelMagVariance;
  public IntVector   accelMin;
  public IntVector   accelMax;
  public IntVector   magMin;
  public IntVector   magMax;
  public FloatVector gyroCoefficientA;
  public FloatVector gyroCoefficientB;
  public FloatVector gyroScale;
  public IntVector   declinationCorrection;
  public IntVector   mountingCorrection;
  public IntVector   accelSwizzle;
  public IntVector   magSwizzle;
  public IntVector   gyroSwizzle;
};

class HeadingSensor
{
  HeadingSensor(Client port)
  {
    // Clear out any data that we might be in the middle of.
    m_port = port;
    m_port.clear();
    m_port.readStringUntil('\n');
  }

  boolean update()
  {
    if (m_port.available() == 0)
    {
      return false;
    }

    String line = m_port.readStringUntil('\n');
    if (line == null)
      return false;

    String[] tokens = splitTokens(line, ",\n");
    if (tokens.length == 5)
    {
      int nextToken = 0;

      // Extract and store orientation quaternion calculated by embedded device.
      float w = float(tokens[nextToken++]);
      float x = float(tokens[nextToken++]);
      float y = float(tokens[nextToken++]);
      float z = float(tokens[nextToken++]);
      m_embeddedQuaternion = quaternion(w, x, y, z);
      m_headingAngle = float(tokens[nextToken++]);

      return true;
    }

    return false;
  }

  float[] getEmbeddedQuaternion()
  {
    return m_embeddedQuaternion;
  }

  float getHeading(float[] q)
  {
    return m_headingAngle;
  }

  float constrainAngle(float angle)
  {
    if (angle < -PI)
    {
      return angle + TWO_PI;
    }
    else if (angle > PI)
    {
      return angle - TWO_PI;
    }

    return angle;
  }

  float getYaw(float[] q)
  {
    float w = q[0];
    float x = q[1];
    float y = q[2];
    float z = q[3];

    return atan2(2*(x*z+y*w), 1-2*(x*x+y*y));
  }

  float getPitch(float[] q)
  {
    float w = q[0];
    float x = q[1];
    float y = q[2];
    float z = q[3];

    return asin(-2*(y*z-x*w));
  }

  float getRoll(float[] q)
  {
    float w = q[0];
    float x = q[1];
    float y = q[2];
    float z = q[3];

    return atan2(2*(x*y+z*w), 1-2*(x*x+z*z));
  }

  Client  m_port;
  float[] m_embeddedQuaternion = new float[4];
  float   m_headingAngle;
};
