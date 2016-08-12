/*   CPV Feed-forward tracking
 *   Test Sketch for indoor optics lab testing
 *   
 *   Michael Lipski
 *   AOPL
 *   Summer 2016
 *   
 *   Allows user to input solar azimuth and zenith in degrees and calculates the displacement in mm along X and Y from the origin of the lens system.
 *   These mm displacement values get converted to number of microsteps and are sent as absolute move commands to the Zaber X-LRM200A linear stages.
 */

#include <translate.h>

#include <zaberx.h>

#include <DebugMacros.h>
#include <LSM303CTypes.h>
#include <SparkFunIMU.h>
#include <SparkFunLSM303C.h>

#include <SoftwareSerial.h>

#ifndef PI
#define PI 3.14159265358979
#endif

int rsRX = 4;
int rsTX = 5;

// Variables involved in finding the panel pitch from 3-axis accelerometer readings
const byte interrupt1 = 2;     // Uno can support external interrupts on pins 2 and 3
volatile boolean setPitch = false;   // boolean for setting panel tilt by reading accelerometer data; off by default
int averaging = 100;

// Acceleration components from 3-axis accelerometer
double accelX;
double accelY;
double accelZ;

double azimuth;
double zenith;

polar coord; // zenith and azimuth in a struct
polar coordP;
vector cart;
vector cartP;

//Enter array tilt and heading
double heading = 0 * (PI/180);
double tilt = 0 * (PI/180);

double radius;
double zaber[2] = {0, 0};
const unsigned long offsetX = 3880000;    //tracking the starting and current absolute positions of the stages
const unsigned long offsetY = 1200000;
unsigned long posX = 0;
unsigned long posY = 0;

// Define common command numbers
int axisX = 1;
int axisY = 2;
String Home = "home";
String MoveAbs = "move abs";
String MoveRel = "move rel";
String Stop = "stop";
String SetMaxspeed = "set maxspeed";
String GetPos = "get pos";
String moveAbsX = "/1 move abs ";
String moveAbsY = "/2 move abs ";
String moveRelX = "/1 move rel ";
String moveRelY = "/2 move rel ";
String comm;

//Period of feedback iterations
const int interval = 5000;

unsigned long previousMillis = 0;
unsigned long currentMillis = 0;

//Create an object for the 6DOF IMU
LSM303C imu;

// Create a software serial port to communicate with the Zaber stages
SoftwareSerial rs232(rsRX, rsTX);   

void setup() 
{
  // Start the Arduino hardware serial port at 9600 baud
  Serial.begin(9600);
  delay(100);
  Serial.println("CPV Feed-forward test sketch");

  //Start software serial connection with Zaber stages
  rs232.begin(115200);
  delay(2000);
  rs232.println("/renumber");
  delay(1000);
  zMove(axisX, offsetX);
  zMove(axisY, offsetY);
  Serial.println("Positioning stages...");
  //delay(15000);
  Serial.println("Ready.");
  Serial.println("Enter azimuth and elevation, separated by a space:");  
}

void loop()
{
  if(Serial.available() > 0)
  {
    //  Read azimuth and elevation from serial terminal
    comm = Serial.readStringUntil(' ');
    azimuth = comm.toFloat();
    comm = Serial.readStringUntil('\n');
    zenith = comm.toFloat();
    
    coord.az = azimuth * (PI/180);
    coord.ze = zenith * (PI/180);
    
    //  Finding solar coordinates w.r.t. panel normal
    cart = sph2rect(coord);

    cartP.x = (cos(heading) * cart.x) - (sin(heading) * cos(tilt) * cart.y) - (sin(heading) * sin(tilt) * cart.z);
    cartP.y = (1)*(sin(heading) * cart.x) + (cos(heading) * cos(tilt) * cart.y) + (cos(heading) * sin(tilt) * cart.z);
    cartP.z = (cos(tilt) * cart.z) - (sin(tilt) * cart.y);

    coordP = rect2sph(cartP);
    
    if(coordP.az < 0)
    {
      coordP.az += (2*PI);
    }
    else if(coordP.az > (2*PI))
    {
      coordP.az -= (2*PI);
    }
    
    //  Determining zaber stage coordinates   
    radius = interp1(sin(coordP.ze));
    zaber[0] = (-1) * radius * sin(coordP.az);
    zaber[1] = (-1) * radius * cos(coordP.az);
    
    Serial.print("Azimuth: ");
    Serial.print(azimuth);
    Serial.print("\tZenith: ");
    Serial.print(zenith);
    Serial.print("\tAzimuth*: ");
    Serial.print(coordP.az * (180/PI));
    Serial.print("\tZenith*: ");
    Serial.print(coordP.ze * (180/PI));
    Serial.print("\tX: ");
    Serial.print(zaber[0]);
    Serial.print("\tY: ");
    Serial.println(zaber[1]);
    //zMove(axisX, mm(zaber[0]));
    //zMove(axisY, mm(zaber[1]));    
  }

  if(setPitch == true)
  {
    accelX = 0;
    accelY = 0;
    accelZ = 0;

    for(int i = 0; i < averaging; i++)
    {
      accelX += imu.readAccelX();
      accelY += imu.readAccelY();
      accelZ += imu.readAccelZ();

      delay(10);
    }
    
    accelX /= averaging;
    accelY /= averaging;
    accelZ /= averaging;
    
    tilt = atan2(accelX, sqrt((accelY * accelY) + (accelZ * accelZ)));
    
    setPitch = false;
  }
}

void findPitch()
{
  setPitch = true;
}

void zMove(int axis, long pos)
{
  String command;
  if(axis == 1)
  {
    posX = offsetX + pos;
    command = moveAbsX + posX;    
  }
  else if(axis == 2)
  {
    posY = pos;
    command = moveAbsY + posY;
  }  
  rs232.println(command);
}

void zMoveRel(int axis, long dist)
{
  String command;
  if(axis == 1)
  {
    posX += dist;
    command = moveRelX + posX;    
  }
  else if(axis == 2)
  {
    posY += dist;
    command = moveRelY + posY;
  }  
  rs232.println(command);
}
