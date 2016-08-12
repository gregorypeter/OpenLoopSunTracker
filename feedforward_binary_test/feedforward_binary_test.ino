/*   CPV Feed-forward tracking
 *   Test Sketch for indoor optics lab testing
 *   Using f20 lens data
 *   
 *   Michael Lipski
 *   AOPL
 *   Summer 2016
 *   
 *   Allows user to input solar azimuth and zenith in degrees and calculates the displacement in mm along X and Y from the origin of the lens system.
 *   These mm displacement values get converted to number of microsteps and are sent as absolute move commands to the Zaber X-LRM200A linear stages.
 */

#include <zaberx.h>

#include <translate.h>

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

double azimuth;
double zenith;

polar coord; // zenith and azimuth in a struct
polar coordP;
vector cart;
vector cartP;

//Enter array tilt and heading
double heading = 0 * (PI/180);
double tilt = 0 * (PI/180);

// Variables involved in finding the panel pitch from 3-axis accelerometer readings
const byte interrupt1 = 2;     // Uno can support external interrupts on pins 2 and 3
volatile boolean setPitch = false;   // boolean for setting panel tilt by reading accelerometer data; off by default
int averaging = 100;

// Acceleration components from 3-axis accelerometer
double accelX;
double accelY;
double accelZ;

// Variables for Zaber binary communication
byte command[6];
byte reply[6];

float outData;
long replyData;

double radius;
double zaber[2] = {0, 0};

const unsigned long offsetX = 2148185;    //tracking the starting and current absolute positions of the stages
const unsigned long offsetY = 2104209;

unsigned long posX = 0;
unsigned long posY = 0;

int axisX = 1;
int axisY = 2;

// Define common command numbers
int homer = 1;      // home the stage
int renumber = 2;   // renumber all devices in the chain
int moveAbs = 20;   // move absolute
int moveRel = 21;   // move relative
int stopMove = 23;  // Stop
int speedSet = 42;    // Speed to target = 0.00219727(V) degrees/sec (assuming 64 microstep resolution)
int getPos = 60;      // Query the device for its position
int storePos = 16;    // Position can be stored in registers 0 to 15
int returnPos = 17;   // returns the value (in microsteps) of the position stored in the indicated register
int move2Pos = 18;    // move to the position stored in the indicated register
int reset = 0;        // akin to toggling device power

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

  /*
  // Initializing LSM303C 6DOF IMU
  if (imu.begin() != IMU_SUCCESS)
  {
    Serial.println("Failed setup.");
    while(1);
  }
  */

  // Enable external interrupt on pin specified by interrupt1 in order to find panel pitch
  pinMode(interrupt1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interrupt1), findPitch, FALLING);

  //Start software serial connection with Zaber stages
  rs232.begin(9600);
  delay(2000);
  Serial.println("Positioning stages...");
  
  /*
  // Positioning stages to origin coinciding with lens axis of symmetry
  posX = sendCommand(axisX, moveAbs, offsetX);
  posY = sendCommand(axisY, moveAbs, offsetY);
  delay(4000);
  while((sendCommand(axisX, getPos, 0) != offsetX) && (sendCommand(axisY, getPos, 0) != offsetY))
  {
    delay(1000);
  }
  */
  
  Serial.println("Ready.");
  Serial.println("Enter azimuth and zenith angles, separated by a space:");  
}

void loop()
{
  if(Serial.available() > 0)
  {
    //  Read azimuth and zenith from serial terminal
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
    posX = sendCommand(axisX, moveAbs, mm(zaber[0]) + offsetX);
    posY = sendCommand(axisY, moveAbs, mm(zaber[1]) + offsetY);    
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

long sendCommand(int device, int com, long data)
{
   unsigned long data2;
   unsigned long temp;
   unsigned long repData;
   long replyNeg;
   byte dumper[1];
   
   // Building the six command bytes
   command[0] = byte(device);
   command[1] = byte(com);
   if(data < 0)
   {
     data2 = data + quad;
   }
   else
   {
     data2 = data;
   }
   temp = data2 / cubed;
   command[5] = byte(temp);
   data2 -= (cubed * temp);
   temp = data2 / squared;
   command[4] = byte(temp);
   data2 -= (squared * temp);
   temp = data2 / 256;
   command[3] = byte(temp);
   data2 -= (256 * data2);
   command[2] = byte(data2);
   
   // Clearing serial buffer
   while(rs232.available() > 0)
   {
     rs232.readBytes(dumper, 1);
   }
   
   // Sending command to stage(s)
   rs232.write(command, 6);

   delay(20);
   
   // Reading device reply
   if(rs232.available() > 0)
   {
     rs232.readBytes(reply, 6);
   }
   
   repData = (cubed * reply[5]) + (squared * reply[4]) + (256 * reply[3]) + reply[2];

   if(reply[4] == 1)
   {
     repData += 65536;
   }
   
   if(reply[5] > 127)
   {
     replyNeg = repData - quad;
   }
   
   // Printing full reply bytes as well as reply data in decimal 
   Serial.print(reply[0]);
   Serial.print(' ');
   Serial.print(reply[1]);
   Serial.print(' ');
   Serial.print(reply[2]);
   Serial.print(' ');
   Serial.print(reply[3]);
   Serial.print(' ');
   Serial.print(reply[4]);
   Serial.print(' ');
   Serial.println(reply[5]);
   Serial.print("\tData: ");
   if(reply[5] > 127)
   {
     Serial.println(replyNeg);
     return replyNeg;
   }
   else
   {
     Serial.println(repData);  
     return repData;
   }    
}
