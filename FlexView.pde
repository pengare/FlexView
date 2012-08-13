/*************************************************************************************
* Test Sketch for Razor AHRS v1.4.0
* 9 Degree of Measurement Attitude and Heading Reference System
* for Sparkfun "9DOF Razor IMU" and "9DOF Sensor Stick"
*
* Released under GNU GPL (General Public License) v3.0
* Copyright (C) 2011 Quality & Usability Lab, Deutsche Telekom Laboratories, TU Berlin
* Written by Peter Bartz (peter-bartz@gmx.de)
*
* Infos, updates, bug reports and feedback:
*     http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs
*************************************************************************************/

/*
  NOTE: There seems to be a bug with the serial library in the latest Processing
  versions 1.5 and 1.5.1: "WARNING: RXTX Version mismatch ...". The previous version
  1.2.1 works fine and is still available on the Processing download page.
*/

import processing.opengl.*;
import processing.serial.*;

//OpenGL import
import processing.opengl.*;
import javax.media.opengl.*;
import javax.media.opengl.glu.*;
import com.sun.opengl.util.*; 

//OpenGL object
GL gl;
// IF THE SKETCH CRASHES OR HANGS ON STARTUP, MAKE SURE YOU ARE USING THE RIGHT SERIAL PORT:
// 1. Have a look at the Processing console output of this sketch.
// 2. Look for the serial port list and find the port you need (it's the same as in Arduino).
// 3. Set your port number here:
final static int SERIAL_PORT_NUM = 4;
// 4. Try again.

final static int SERIAL_PORT_BAUD_RATE = 57600;

float yaw = 0.0f;
float pitch = 0.0f;
float roll = 0.0f;
float yawOffset = 0.0f;

PFont font;
Serial serial;



boolean synched = false;

//For camera
int zoomLevel = 900;
int rotateVal = 0;
vec3 camPos;
vec3 camTarget;
float camFov;
float camNearCutPlane;

//For the textured sphere
PImage bg;
PImage texmap;
PImage texMoon;
PImage texSun;

int sDetail = 35;  // Sphere detail setting
float rotationX = 0;
float rotationY = 0;
float velocityX = 0;
float velocityY = 0;
float globeRadius = 450;
float globeRadiusBig = 600;
float globeRadiusSmall = 70;
float pushBack = 50;

float[] cx, cz, sphereX, sphereY, sphereZ;
float sinLUT[];
float cosLUT[];
float SINCOS_PRECISION = 0.5;
int SINCOS_LENGTH = int(360.0 / SINCOS_PRECISION);

//For small sphere animation
float smallSphereAngle = 0;


//For bend sensor
final static int SERIAL_PORT_NUM_BEND_SENSOR = 6;
final int LINE_FEED = 10;
Serial serialBend;
int prevBendVal = -100; //minus value for checking if stable
int currBendVal = 0;
boolean bBendSensorInit = false;
boolean bSensorStable = false;


void drawArrow(float headWidthFactor, float headLengthFactor) {
  float headWidth = headWidthFactor * 200.0f;
  float headLength = headLengthFactor * 200.0f;
  
  pushMatrix();
  
  // Draw base
  translate(0, 0, -100);
  box(100, 100, 200);
  
  // Draw pointer
  translate(-headWidth/2, -50, -100);
  beginShape(QUAD_STRIP);
    vertex(0, 0 ,0);
    vertex(0, 100, 0);
    vertex(headWidth, 0 ,0);
    vertex(headWidth, 100, 0);
    vertex(headWidth/2, 0, -headLength);
    vertex(headWidth/2, 100, -headLength);
    vertex(0, 0 ,0);
    vertex(0, 100, 0);
  endShape();
  beginShape(TRIANGLES);
    vertex(0, 0, 0);
    vertex(headWidth, 0, 0);
    vertex(headWidth/2, 0, -headLength);
    vertex(0, 100, 0);
    vertex(headWidth, 100, 0);
    vertex(headWidth/2, 100, -headLength);
  endShape();
  
  popMatrix();
}

void drawBoard() {
  pushMatrix();

  rotateY(-radians(yaw - yawOffset));
  rotateX(-radians(pitch));
  rotateZ(radians(roll)); 

  // Board body
  fill(255, 0, 0);
  box(250, 20, 400);
  
  // Forward-arrow
  pushMatrix();
  translate(0, 0, -200);
  scale(0.5f, 0.2f, 0.25f);
  fill(0, 255, 0);
  drawArrow(1.0f, 2.0f);
  popMatrix();
    
  popMatrix();
}

// Skip incoming serial stream data until token is found
boolean readToken(Serial serial, String token) {
  // Wait until enough bytes are available
  if (serial.available() < token.length())
    return false;
  
  // Check if incoming bytes match token
  for (int i = 0; i < token.length(); i++) {
    if (serial.read() != token.charAt(i))
      return false;
  }
  
  return true;
}

void initScene()
{
  //init camera
  camPos = new vec3(0, 0, 900);
  camTarget = new vec3(0, 0, 0);
  camFov = PI/3.0;
  camNearCutPlane = 1;
  
  //ortho(-width/2, width/2, -height/2, height/2, -10000, 10000);
  perspective(camFov, float(width)/float(height), 
            camNearCutPlane, 10000);
  //camPos = new vec3(0, 0, 30);
  //camTarget = new vec3(0, 0, 0);
}

// Global setup
void setup() {
  // Setup graphics
  size(1024, 768, OPENGL);  
  
//  gl = ((PGraphicsOpenGL)g).gl;
//  gl.glEnable(GL.GL_CULL_FACE);
  //Load texture
  texmap = loadImage("world32k.jpg");   
  texMoon = loadImage("moon.jpg");
  texSun = loadImage("world32k.jpg"); 
  
  //Init scene
  initScene();
  
  initializeSphere(sDetail);
  smooth();
  noStroke();
  frameRate(50);
  
  // Load font
  font = loadFont("Univers-66.vlw");
  textFont(font);
  
//  // Setup serial port I/O
//  println("AVAILABLE SERIAL PORTS:");
//  println(Serial.list());
//  String portName = Serial.list()[SERIAL_PORT_NUM];
//  println();
//  println("HAVE A LOOK AT THE LIST ABOVE AND SET THE RIGHT SERIAL PORT NUMBER IN THE CODE!");
//  println("  -> Using port " + SERIAL_PORT_NUM + ": " + portName);
//  serial = new Serial(this, portName, SERIAL_PORT_BAUD_RATE);
//  
//  
//  //Setup serial comm for bend sensor
//  serialBend = new Serial(this, Serial.list()[6], 9600);
//  serialBend.bufferUntil(LINE_FEED);
}

void setupRazor() {
  println("Trying to setup and synch Razor...");
  
  // On Mac OSX and Linux (Windows too?) the board will do a reset when we connect, which is really bad.
  // See "Automatic (Software) Reset" on http://www.arduino.cc/en/Main/ArduinoBoardProMini
  // So we have to wait until the bootloader is finished and the Razor firmware can receive commands.
  // To prevent this, disconnect/cut/unplug the DTR line going to the board. This also has the advantage,
  // that the angles you receive are stable right from the beginning. 
  delay(3000);  // 3 seconds should be enough
  
  // Set Razor output parameters
  serial.write("#ob");  // Turn on binary output
  serial.write("#o1");  // Turn on continuous streaming output
  serial.write("#oe0"); // Disable error message output
  
  // Synch with Razor
  serial.clear();  // Clear input buffer up to here
  serial.write("#s00");  // Request synch token
}

float readFloat(Serial s) {
  // Convert from little endian (Razor) to big endian (Java) and interpret as float
  return Float.intBitsToFloat(s.read() + (s.read() << 8) + (s.read() << 16) + (s.read() << 24));
}

void drawBkStars()
{
  pushMatrix();
  translate(0, 0, -500);
  //box(50);
  popMatrix();
}

void drawInfoText()
{
//  pushMatrix();
//  translate(width/2, height/2, 0);
//  fill(255);
//  box(50);
//  popMatrix();
  textFont(font, 20);
  fill(255);
  textAlign(LEFT);

  // Output info text
  text("Point FTDI connector towards screen and press 'a' to align", 10, 25);

  // Output angles
  pushMatrix();
  translate(10, height - 10);
  textAlign(LEFT);
  text("Yaw: " + ((int) yaw), 0, 0);
  text("Pitch: " + ((int) pitch), 150, 0);
  text("Roll: " + ((int) roll), 300, 0);
  popMatrix();
}

void draw() {
   // Reset scene
  background(0);
  lights();

//  // Sync with Razor 
//  if (!synched) {
//    textAlign(CENTER);
//    fill(255);
//    text("Connecting to Razor...", width/2, height/2, -200);
//    
//    if (frameCount == 2)
//      setupRazor();  // Set ouput params and request synch token
//    else if (frameCount > 2)
//      synched = readToken(serial, "#SYNCH00\r\n");  // Look for synch token
//    return;
//  }
//  
//  // Read angles from serial port
//  while (serial.available() >= 12) {
//    yaw = readFloat(serial);
//    pitch = readFloat(serial);
//    roll = readFloat(serial);
//  }
  
  // Draw info text, which is not influence by the interactivity
  camera(width/2, height/2, 700, width/2, height/2, 0, 0, 1, 0);
  drawInfoText();
  
  // Camera setup
  //camera(width/2,height/2,zoomLevel,width/2,height/2,0,0,1,0);
  //camera(0, 0, 500, 0, 0, 0, 0, 1, 0);
  camera(camPos.x, camPos.y, camPos.z, camTarget.x, camTarget.y, camTarget.z, 0, 1, 0);
  
  pushMatrix();
  rotateY(-radians(yaw - yawOffset));
  rotateX(-radians(pitch));
  rotateZ(radians(roll)); 

//  stroke(255, 0, 0);
//  line(0, 0, 0, 10000, 0, 0);
//  
//  stroke(0, 255, 0);
//  line(0, 0, 0, 0, 10000, 0);
//  
//  stroke(0, 0, 255);
//  line(0,0,0, 0, 0, 10000);
  
  // Draw background star
  drawBkStars();
  //translate(width/2, height/2, -350);
  //drawBoard();
  
  //Draw the big sphere
  globeRadius = globeRadiusBig;
  renderGlobe(globeRadiusBig, texSun);
  
  //Draw the track of small sphere
  pushMatrix();
  //translate(width/2.0, height/2.0, pushBack);
  stroke(0, 255, 0);
  float prevPosX = 400, prevPosZ = 0;
  int dashLineIndicator = 0;
  for(int segment = 0; segment <= 200; ++segment)
  {
    float currPosX = 400 * cos(2*3.14 / 50.0 * segment);
    float currPosZ = 400 * sin(2*3.14 / 50.0 * segment);
    dashLineIndicator ^= 1;
    if(dashLineIndicator == 1)
      line(prevPosX, 0, prevPosZ, currPosX, 0, currPosZ);
    prevPosX = currPosX;
    prevPosZ = currPosZ;
  }
  noStroke();
  popMatrix();
  
  
  //Draw the small sphere
  translate(400 * cos(smallSphereAngle), 0, 400 * sin(smallSphereAngle));
  globeRadius = globeRadiusSmall;
  
  smallSphereAngle += 0.05;
  if(smallSphereAngle > 2 * 3.14)
  {
    smallSphereAngle = 0;
  }
  
  renderGlobe(globeRadiusSmall, texMoon);
  
  popMatrix();
}

void keyPressed() {
  switch (key) {
    case '0':  // Turn Razor's continuous output stream off
      serial.write("#o0");
      break;
    case '1':  // Turn Razor's continuous output stream on
      serial.write("#o1");
      break;
    case 'f':  // Request one single yaw/pitch/roll frame from Razor (use when continuous streaming is off)
      serial.write("#f");
      break;
    case 'a':  // Align screen with Razor
      yawOffset = yaw; 
      break;  
    case 'w':
      //camPos.z -= 100;
      camNearCutPlane += 50;
        perspective(camFov, float(width)/float(height), 
            camNearCutPlane, 10000);
      break;
    case 's':
      //camPos.z += 100;
      camNearCutPlane -= 50;
        perspective(camFov, float(width)/float(height), 
            camNearCutPlane, 10000);
      break;
  }
}


//For sphere
void initializeSphere(int res)
{
  sinLUT = new float[SINCOS_LENGTH];
  cosLUT = new float[SINCOS_LENGTH];

  for (int i = 0; i < SINCOS_LENGTH; i++) {
    sinLUT[i] = (float) Math.sin(i * DEG_TO_RAD * SINCOS_PRECISION);
    cosLUT[i] = (float) Math.cos(i * DEG_TO_RAD * SINCOS_PRECISION);
  }

  float delta = (float)SINCOS_LENGTH/res;
  float[] cx = new float[res];
  float[] cz = new float[res];

  // Calc unit circle in XZ plane
  for (int i = 0; i < res; i++) {
    cx[i] = -cosLUT[(int) (i*delta) % SINCOS_LENGTH];
    cz[i] = sinLUT[(int) (i*delta) % SINCOS_LENGTH];
  }

  // Computing vertexlist vertexlist starts at south pole
  int vertCount = res * (res-1) + 2;
  int currVert = 0;

  // Re-init arrays to store vertices
  sphereX = new float[vertCount];
  sphereY = new float[vertCount];
  sphereZ = new float[vertCount];
  float angle_step = (SINCOS_LENGTH*0.5f)/res;
  float angle = angle_step;

  // Step along Y axis
  for (int i = 1; i < res; i++) {
    float curradius = sinLUT[(int) angle % SINCOS_LENGTH];
    float currY = -cosLUT[(int) angle % SINCOS_LENGTH];
    for (int j = 0; j < res; j++) {
      sphereX[currVert] = cx[j] * curradius;
      sphereY[currVert] = currY;
      sphereZ[currVert++] = cz[j] * curradius;
    }
    angle += angle_step;
  }
  sDetail = res;
}

// Generic routine to draw textured sphere
void texturedSphere(float r, PImage t) 
{
  int v1, v11, v2;
  //r = (r + 240 ) * 0.33;
  r = r * 0.33;
  beginShape(TRIANGLE_STRIP);
  texture(t);
  float iu=(float)(t.width-1)/(sDetail);
  float iv=(float)(t.height-1)/(sDetail);
  float u=0, v=iv;
  for (int i = 0; i < sDetail; i++) {
    vertex(0, -r, 0, u, 0);
    vertex(sphereX[i]*r, sphereY[i]*r, sphereZ[i]*r, u, v);
    u+=iu;
  }
  vertex(0, -r, 0, u, 0);
  vertex(sphereX[0]*r, sphereY[0]*r, sphereZ[0]*r, u, v);
  endShape();   

  // Middle rings
  int voff = 0;
  for (int i = 2; i < sDetail; i++) {
    v1=v11=voff;
    voff += sDetail;
    v2=voff;
    u=0;
    beginShape(TRIANGLE_STRIP);
    texture(t);
    for (int j = 0; j < sDetail; j++) {
      vertex(sphereX[v1]*r, sphereY[v1]*r, sphereZ[v1++]*r, u, v);
      vertex(sphereX[v2]*r, sphereY[v2]*r, sphereZ[v2++]*r, u, v+iv);
      u+=iu;
    }

    // Close each ring
    v1=v11;
    v2=voff;
    vertex(sphereX[v1]*r, sphereY[v1]*r, sphereZ[v1]*r, u, v);
    vertex(sphereX[v2]*r, sphereY[v2]*r, sphereZ[v2]*r, u, v+iv);
    endShape();
    v+=iv;
  }
  u=0;

  // Add the northern cap
  beginShape(TRIANGLE_STRIP);
  texture(t);
  for (int i = 0; i < sDetail; i++) {
    v2 = voff + i;
    vertex(sphereX[v2]*r, sphereY[v2]*r, sphereZ[v2]*r, u, v);
    vertex(0, r, 0, u, v+iv);    
    u+=iu;
  }
  vertex(sphereX[voff]*r, sphereY[voff]*r, sphereZ[voff]*r, u, v);
  endShape();
}


void renderGlobe(float r, PImage t) {
  pushMatrix();
  //translate(width/2.0, height/2.0, pushBack);
  pushMatrix();
  noFill();
  stroke(255, 200);
  strokeWeight(2);
  smooth();
  popMatrix();
  lights();    
  pushMatrix();
  //translate(0, 0, zoomLevel);
  //rotateX( radians(-rotationX) );  
  //rotateY( radians(270 - rotationY) );

//This roataion is along with self coordinate
//  rotateY(radians(yaw - yawOffset));
//  rotateX(radians(pitch));
//  rotateZ(-radians(roll)); 
  fill(200);
  noStroke();
  textureMode(IMAGE);  
  texturedSphere(r, t);
  popMatrix();  
  popMatrix();
  rotationX += velocityX;
  //rotationY += velocityY;
  rotationY = rotateVal;
  velocityX *= 0.95;
  velocityY *= 0.95;

  // Implements mouse control (interaction will be inverse when sphere is  upside down)
  if (mousePressed) {
    velocityX += (mouseY-pmouseY) * 0.01;
    velocityY -= (mouseX-pmouseX) * 0.01;
  }
}


////////////
void serialEvent(Serial port)
{
  //read from bend sensor and send via serial

  if (serialBend.available() > 0)
  {
    String val = serialBend.readStringUntil(LINE_FEED);
    parseArduinoOutput(val);
    if (!bSensorStable)
    {
      checkSensorStable();
    }
    else
    {
      if (!bBendSensorInit)
      { 
        prevBendVal = currBendVal;
        bBendSensorInit = true;
      }
      else
      {
        //      String val = myPort.readStringUntil(LINE_FEED);
        //      currBendVal = int(trim(val));
        if (abs(currBendVal - prevBendVal) > 10)
        {
          camPos.z -= (currBendVal - prevBendVal);
          println(""+currBendVal+",,,"+prevBendVal);
          //zoomLevel = currBendVal;
        }
        //prevBendVal = currBendVal;
      }

//      if (!bRotateSensorInit)
//      {
//        prevRotateVal = currRotateVal;
//        bRotateSensorInit = true;
//      }
//      else
//      {
//        if (abs(currRotateVal - prevRotateVal) > 20)
//        {
//          rotateVal += (currRotateVal - prevRotateVal)/2;
//          prevRotateVal = currRotateVal;
//        }
//      }
    }
  }
}

void parseArduinoOutput(String val)
{
  final int[] data = int(split(trim(val), ','));
  if (data.length == 2)
  {
    currBendVal = data[0];
    //currRotateVal = data[1];
  }
}

void checkSensorStable()
{
    if(abs(currBendVal-prevBendVal) > 10)
    {
      bSensorStable = true;
    }
}



