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

Client                   g_port;
HeadingSensor            g_headingSensor;
boolean                  g_zeroRotation = false;
float                    g_cameraAngle = 0.0f;
int                      g_samples = 0;
int                      g_lastSampleCount;
float[]                  g_rotationQuaternion = {1.0f, 0.0f, 0.0f, 0.0f};
int                      g_fontHeight;
PFont                    g_font;
PMatrix3D                g_kalmanP;


void setup()
{
  size(1024, 768, P3D);
  fill(255, 0, 0);

  g_font = loadFont("Monaco-24.vlw");
  textFont(g_font);
  g_fontHeight = int(textAscent() + textDescent() + 0.5f);

  Client port = new Client(this, "localhost", 3334);
  g_headingSensor = new HeadingSensor(port);
  g_lastSampleCount = millis();

  g_port = port;
}

void draw()
{
  background(100);

  int elapsedTime = millis() - g_lastSampleCount;
  if (elapsedTime > 10000)
  {
    println(g_samples / (elapsedTime / 1000));
    g_lastSampleCount = millis();
    g_samples = 0;
  }

  // Convert rotation quaternion into a 4x4 rotation matrix to be used for rendering.
  PMatrix3D rotationMatrix = quaternionToMatrix(g_rotationQuaternion);

  float headingAngle = g_headingSensor.getHeading(g_rotationQuaternion);

  // If the user has pressed the space key, then move the camera to face the device front.
  if (g_zeroRotation)
  {
    PVector cam = new PVector(0, (height / 2.0) / tan(radians(30.0)));
    g_cameraAngle = -g_headingSensor.getYaw(g_rotationQuaternion);
    cam.rotate(g_cameraAngle);
    camera(cam.x + width/2.0, height/2.0, cam.y, width/2.0, height/2.0, 0, 0, 1, 0);
    g_zeroRotation = false;
  }

  // Rotate the rendered box using the calculated rotation matrix.
  pushMatrix();
    translate(width / 2, height / 2, 0);
    scale(5.0f, 5.0f, 5.0f);
    applyMatrix(rotationMatrix);
    drawBox();
  popMatrix();

  // Rotate the compass image accordingly.
  pushMatrix();
    rotate2DPlaneToFaceCamera();
    translate(width - 150, height - 150, 0);
    drawCompass(headingAngle);
  popMatrix();

  // Display rotation source to user.
  fill(255);
  rotate2DPlaneToFaceCamera();
}

void rotate2DPlaneToFaceCamera()
{
  translate(width/2.0f, height/2.0f, 0.0f);
  rotateY(-g_cameraAngle);
  translate(-width/2.0f, -height/2.0f, 0.0f);
}

void drawBox()
{
  // Draw four sides of box with different colours on each.
  stroke(160);
  fill(82, 10, 242);
  beginShape(QUADS);
    vertex(-25, -10, 50);
    vertex(-25, 10, 50);
    vertex(25, 10, 50);
    vertex(25, -10, 50);
  endShape();

  fill(255);
  beginShape(QUADS);
    vertex(-25, -10, -50);
    vertex(-25, -10, 50);
    vertex(25, -10, 50);
    vertex(25, -10, -50);
  endShape();

  fill(0);
  beginShape(QUADS);
    vertex(-25, 10, -50);
    vertex(-25, 10, 50);
    vertex(25, 10, 50);
    vertex(25, 10, -50);
  endShape();

  fill(126, 209, 13);
  beginShape(QUADS);
    vertex(-25, -10, -50);
    vertex(-25, 10, -50);
    vertex(25, 10, -50);
    vertex(25, -10, -50);
  endShape();

  fill(209, 6, 10);
  beginShape(QUADS);
    vertex(-25, 10, -50);
    vertex(-25, 10, 50);
    vertex(-25, -10, 50);
    vertex(-25, -10, -50);
  endShape();

  fill(237, 255, 0);
  beginShape(QUADS);
    vertex(25, 10, -50);
    vertex(25, 10, 50);
    vertex(25, -10, 50);
    vertex(25, -10, -50);
  endShape();
}

void drawCompass(float angle)
{
  // Adjust heading angle so that the rendered needle points up when robot is facing north (angle == 0).
  angle = g_headingSensor.constrainAngle(angle - PI);

  rotateX(radians(-90));

  noStroke();
  fill(255, 0, 0);
  drawCylinder(100, 100, 10, 64);

  fill(0, 0, 255);
  rotateY(angle);
  translate(2.5, 0, 50);
  box(5, 10, 100);
}

void drawCylinder(float topRadius, float bottomRadius, float tall, int sides)
{
  float angle = 0;
  float angleIncrement = TWO_PI / sides;
  beginShape(QUAD_STRIP);
  for (int i = 0; i < sides + 1; ++i) {
    vertex(topRadius*cos(angle), 0, topRadius*sin(angle));
    vertex(bottomRadius*cos(angle), tall, bottomRadius*sin(angle));
    angle += angleIncrement;
  }
  endShape();

  // If it is not a cone, draw the circular top cap
  if (topRadius != 0) {
    angle = 0;
    beginShape(TRIANGLE_FAN);

    // Center point
    vertex(0, 0, 0);
    for (int i = 0; i < sides + 1; i++) {
      vertex(topRadius * cos(angle), 0, topRadius * sin(angle));
      angle += angleIncrement;
    }
    endShape();
  }

  // If it is not a cone, draw the circular bottom cap
  if (bottomRadius != 0) {
    angle = 0;
    beginShape(TRIANGLE_FAN);

    // Center point
    vertex(0, tall, 0);
    for (int i = 0; i < sides + 1; i++) {
      vertex(bottomRadius * cos(angle), tall, bottomRadius * sin(angle));
      angle += angleIncrement;
    }
    endShape();
  }
}

void clientEvent(Client port)
{
  if (g_port != port)
    return;
  if (g_headingSensor == null)
    return;
  if (!g_headingSensor.update())
    return;
  g_samples++;

  // Using embedded device's Kalman filter.
  g_rotationQuaternion = g_headingSensor.getEmbeddedQuaternion();
}

void keyPressed()
{
  char lowerKey = Character.toLowerCase(key);

  switch(lowerKey)
  {
  case ' ':
    g_zeroRotation = true;
    break;
  }
}
