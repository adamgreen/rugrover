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
import processing.net.*; 

HeadingSensorCalibration g_calibration;
HeadingSensor            g_headingSensor;
PMatrix3D                g_rotationMatrix;
int                      g_samples = 0;
float[]                  g_currentRotation = {0.0f, 0.0f, 0.0f};
int                      g_gyroXBias = 0;
int                      g_gyroYBias = 0;
int                      g_gyroZBias = 0;
int                      g_fontHeight;
PFont                    g_font;

// Globals used for calculating and dumps statistics (mean and variance) for all sensors.
final int     g_samplesForStats = 6000;
DoubleHeading g_samplesSum;
DoubleHeading g_samplesSquaredSum;
int           g_statSamples;
PrintWriter   g_statsFile;
PrintWriter   g_gyroFile;
int           g_lastSample;
int           g_lastDumpCount;
int           g_lastDumpTime_ms;

void setup() 
{
  size(1024, 180);
  fill(255, 0, 0);

  g_font = loadFont("Monaco-24.vlw");
  textFont(g_font);
  g_fontHeight = int(textAscent() + textDescent() + 0.5f);
  
  Client port = new Client(this, "localhost", 3334);

  Heading filterWidths = new Heading(16, 16, 16, 16, 16, 16, 16, 16, 16, 16);
  g_headingSensor = new HeadingSensor(port, filterWidths);
  
  // Intialize variables used for calculating mean and variance.
  g_samplesSum = new DoubleHeading(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  g_samplesSquaredSum = new DoubleHeading(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  g_statSamples = 0;
  g_statsFile = createWriter("stats.csv");
  g_gyroFile = createWriter("gyro.csv");
  g_statsFile.println("accelX,accelY,accelZ,magX,magY,magZ,gryoX,gyroY,gyroZ,gyroTemperature");
  g_gyroFile.println("temperature,x,y,z");
  g_statsFile.flush(); g_gyroFile.flush();
  
  g_lastSample = 0;
  g_lastDumpCount = 0;
  g_lastDumpTime_ms = millis();
}

void draw()
{
  // Setup 3d transformations other than rotation which will come next.
  background(25);
  fill(255);

  Heading heading = g_headingSensor.getCurrentMovingAverage();

  String gyroInfo   = "            Gyro: " + nfp(heading.m_gyroX, 5) + ", " + nfp(heading.m_gyroY, 5) + ", " + nfp(heading.m_gyroZ, 5) + ", " + nfp(heading.m_gyroTemperature, 5); 
  String gyroAccum  = gyroAccumInfo();
  String accelInfo  = accelerometerInfo();
  String magMinInfo = magnetometerMin();
  String magMaxInfo = magnetometerMax();
  String sampleRate = sampleRateText();

  int line = 1;
  text(gyroInfo, 10, g_fontHeight * line++);
  text(gyroAccum, 10, g_fontHeight * line++);
  text(magMinInfo, 10, g_fontHeight * line++);
  text(magMaxInfo, 10, g_fontHeight * line++);
  text(accelInfo, 10, g_fontHeight * line++);
  text(sampleRate, 10, g_fontHeight * line++);
}

String gyroAccumInfo()
{
  return "        Gyro Sum: " + nfp(g_currentRotation[0], 10, 2) + ", " + 
                                nfp(g_currentRotation[1], 10, 2) + ", " + 
                                nfp(g_currentRotation[2], 10, 2);
}

String accelerometerInfo()
{
  Heading heading = g_headingSensor.getCurrentMovingAverage();
  return "   Accelerometer: " + nfp(heading.m_accelX, 5) + ", " + nfp(heading.m_accelY, 5) + ", " + nfp(heading.m_accelZ, 5);
}

String magnetometerMin()
{
  Heading minHeading = g_headingSensor.getMin();
  return "Magnetometer Min: " + nfp(minHeading.m_magX, 5) + ", " + nfp(minHeading.m_magY, 5) + ", " + nfp(minHeading.m_magZ, 5);
}

String magnetometerMax()
{
  Heading maxHeading = g_headingSensor.getMax();
  return "Magnetometer Max: " + nfp(maxHeading.m_magX, 5) + ", " + nfp(maxHeading.m_magY, 5) + ", " + nfp(maxHeading.m_magZ, 5);
}

String sampleRateText()
{
  return "     Sample Rate: " + nf(g_lastDumpCount, 0);
}

void updateStats()
{
  Heading currentRaw = g_headingSensor.getCurrentRaw();
  g_samplesSum.add(currentRaw);
  g_samplesSquaredSum.addSquared(currentRaw);
  g_statSamples++;
}

void dumpStatsToFiles()
{
  if (g_statSamples < g_samplesForStats)
    return;
  
  DoubleHeading mean = new DoubleHeading(g_samplesSum.m_accelX / g_statSamples,
                                       g_samplesSum.m_accelY / g_statSamples,
                                       g_samplesSum.m_accelZ / g_statSamples,
                                       g_samplesSum.m_magX / g_statSamples,
                                       g_samplesSum.m_magY / g_statSamples,
                                       g_samplesSum.m_magZ / g_statSamples,
                                       g_samplesSum.m_gyroX / g_statSamples,
                                       g_samplesSum.m_gyroY / g_statSamples,
                                       g_samplesSum.m_gyroZ / g_statSamples,
                                       g_samplesSum.m_gyroTemperature / g_statSamples);
  DoubleHeading variance = new DoubleHeading((g_samplesSquaredSum.m_accelX - ((g_samplesSum.m_accelX * g_samplesSum.m_accelX) / g_statSamples)) / (g_statSamples - 1),
                                           (g_samplesSquaredSum.m_accelY - ((g_samplesSum.m_accelY * g_samplesSum.m_accelY) / g_statSamples)) / (g_statSamples - 1),
                                           (g_samplesSquaredSum.m_accelZ - ((g_samplesSum.m_accelZ * g_samplesSum.m_accelZ) / g_statSamples)) / (g_statSamples - 1),
                                           (g_samplesSquaredSum.m_magX - ((g_samplesSum.m_magX * g_samplesSum.m_magX) / g_statSamples)) / (g_statSamples - 1),
                                           (g_samplesSquaredSum.m_magY - ((g_samplesSum.m_magY * g_samplesSum.m_magY) / g_statSamples)) / (g_statSamples - 1),
                                           (g_samplesSquaredSum.m_magZ - ((g_samplesSum.m_magZ * g_samplesSum.m_magZ) / g_statSamples)) / (g_statSamples - 1),
                                           (g_samplesSquaredSum.m_gyroX - ((g_samplesSum.m_gyroX * g_samplesSum.m_gyroX) / g_statSamples)) / (g_statSamples - 1),
                                           (g_samplesSquaredSum.m_gyroY - ((g_samplesSum.m_gyroY * g_samplesSum.m_gyroY) / g_statSamples)) / (g_statSamples - 1),
                                           (g_samplesSquaredSum.m_gyroZ - ((g_samplesSum.m_gyroZ * g_samplesSum.m_gyroZ) / g_statSamples)) / (g_statSamples - 1),
                                           (g_samplesSquaredSum.m_gyroTemperature - ((g_samplesSum.m_gyroTemperature * g_samplesSum.m_gyroTemperature) / g_statSamples)) / (g_statSamples - 1));

  
  g_statsFile.print("Mean: ");
  mean.print(g_statsFile);
  g_statsFile.println();
  
  g_statsFile.print("Variance: ");
  variance.print(g_statsFile);
  g_statsFile.println();
  
  g_gyroFile.println(mean.m_gyroTemperature + "," + mean.m_gyroX + "," + mean.m_gyroY + "," + mean.m_gyroZ);

  g_statsFile.flush();
  g_gyroFile.flush();

  g_samplesSum = new DoubleHeading(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  g_samplesSquaredSum = new DoubleHeading(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  g_statSamples = 0;
}

void integrateGyroMeasurements()
{
  // Integrate gryo readings and dump if user has made such a request.
//  final float timeScale = (1.0f / 198.4f);
  final float timeScale = (1.0f / 99.3f);
  final int   gyroThreshold = 4;
  Heading heading = g_headingSensor.getCurrentRaw();
  int gyroX = heading.m_gyroX - g_gyroXBias;
  int gyroY = heading.m_gyroY - g_gyroYBias;
  int gyroZ = heading.m_gyroZ - g_gyroZBias;
  if (abs(gyroX) < gyroThreshold)
    gyroX = 0;
  if (abs(gyroY) < gyroThreshold)
    gyroY = 0;
  if (abs(gyroZ) < gyroThreshold)
    gyroZ = 0;
  g_currentRotation[0] += float(gyroX) * timeScale;
  g_currentRotation[1] += float(gyroY) * timeScale;
  g_currentRotation[2] += float(gyroZ) * timeScale;
}

void keyPressed()
{
  char lowerKey = Character.toLowerCase(key);
  
  switch(lowerKey)
  {
  case 'a':
    println(accelerometerInfo());
    break;
  case 'm':
    println(magnetometerMin());
    println(magnetometerMax());
    break;
  case 'g':
    println(gyroAccumInfo());
    Heading heading = g_headingSensor.getCurrentRaw();
    g_gyroXBias = heading.m_gyroX;
    g_gyroYBias = heading.m_gyroY;
    g_gyroZBias = heading.m_gyroZ;
    g_currentRotation[0] = 0.0f;
    g_currentRotation[1] = 0.0f;
    g_currentRotation[2] = 0.0f;
    break;
  }
}

void clientEvent(Client client)
{
  while (g_headingSensor.update())
  {
    g_samples++;
    int currentTime_ms = millis();
    int elapsedTime_ms = currentTime_ms - g_lastDumpTime_ms;
    if (elapsedTime_ms >= 1000)
    {
      g_lastDumpCount = g_samples - g_lastSample;
      g_lastSample = g_samples;
      g_lastDumpTime_ms = currentTime_ms;
    }
    
    updateStats();
    dumpStatsToFiles();
    integrateGyroMeasurements();
  }
}
