/*  Copyright (C) 2014  Adam Green (https://github.com/adamgreen)

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
  public IntVector   accelMin;
  public IntVector   accelMax;
  public IntVector   magMin;
  public IntVector   magMax;
  public FloatVector gyroCoefficientA;
  public FloatVector gyroCoefficientB;
  public FloatVector gyroScale;
};

class HeadingSensor
{
  HeadingSensor(Client port, Heading sampleCounts)
  {
    m_averages = new MovingAverage[10];
    m_averages[0] = new MovingAverage(sampleCounts.m_accelX);
    m_averages[1] = new MovingAverage(sampleCounts.m_accelY);
    m_averages[2] = new MovingAverage(sampleCounts.m_accelZ);
    m_averages[3] = new MovingAverage(sampleCounts.m_magX);
    m_averages[4] = new MovingAverage(sampleCounts.m_magY);
    m_averages[5] = new MovingAverage(sampleCounts.m_magZ);
    m_averages[6] = new MovingAverage(sampleCounts.m_gyroX);
    m_averages[7] = new MovingAverage(sampleCounts.m_gyroY);
    m_averages[8] = new MovingAverage(sampleCounts.m_gyroZ);
    m_averages[9] = new MovingAverage(sampleCounts.m_gyroTemperature);

    // Clear out any data that we might be in the middle of.
    m_port = port;
    m_port.clear();
    String dummy = m_port.readStringUntil('\n');
  }

  boolean update()
  {
    if (m_port.available() == 0)
    {
      return false;
    }

    String line = m_port.readStringUntil('\n');
    if (line == null)
    {
      return false;
    }

    String[] tokens = splitTokens(line, ",\n");
    if (tokens.length == 11)
    {
      m_currentRaw.m_accelX = int(tokens[0]);
      m_currentRaw.m_accelY = int(tokens[1]);
      m_currentRaw.m_accelZ = int(tokens[2]);
      m_currentRaw.m_magX = int(tokens[3]);
      m_currentRaw.m_magY = int(tokens[4]);
      m_currentRaw.m_magZ = int(tokens[5]);
      m_currentRaw.m_gyroX = int(tokens[6]);
      m_currentRaw.m_gyroY = int(tokens[7]);
      m_currentRaw.m_gyroZ = int(tokens[8]);
      m_currentRaw.m_gyroTemperature = int(tokens[9]);

      for (int i = 0 ; i < m_averages.length ; i++)
        m_averages[i].update(int(tokens[i]));

      m_max = m_max.max(m_currentRaw);
      m_min = m_min.min(m_currentRaw);

      return true;
    }

    return false;
  }

  Heading getCurrentRaw()
  {
    return m_currentRaw;
  }

  Heading getCurrentMovingAverage()
  {
    return new Heading(m_averages[0].getAverage(),
                       m_averages[1].getAverage(),
                       m_averages[2].getAverage(),
                       m_averages[3].getAverage(),
                       m_averages[4].getAverage(),
                       m_averages[5].getAverage(),
                       m_averages[6].getAverage(),
                       m_averages[7].getAverage(),
                       m_averages[8].getAverage(),
                       m_averages[9].getAverage());
  }

  Heading getMin()
  {
    return m_min;
  }

  Heading getMax()
  {
    return m_max;
  }

  Client  m_port;
  Heading m_currentRaw = new Heading();
  Heading m_min = new Heading(0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF,
                              0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF,
                              0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF);
  Heading m_max = new Heading(0x80000000, 0x80000000, 0x80000000,
                              0x80000000, 0x80000000, 0x80000000,
                              0x80000000, 0x80000000, 0x80000000, 0x80000000);
  MovingAverage[] m_averages;
};
