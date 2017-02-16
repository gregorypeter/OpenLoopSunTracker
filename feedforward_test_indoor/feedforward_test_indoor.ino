/*   CPV Feed-forward tracking - Indoor test sketch
 *    Using Zaber Binary protocol
 *    Using new lens data
 *   
 *   Michael Lipski
 *   AOPL
 *   Summer 2016
 *   
 *   Feed-forward tracking for CPV test setup.  Sketch accepts user input for zenith angle through serial interface; input angle controls Zaber rotational
 *   stage with light source attached to it.
 */

#include <zaberx.h>

#include "RTClib.h"

#include <Sun_position_algorithms.h>
#include <translate.h>

#include <SoftwareSerial.h>

#include <Wire.h>

//#include <Adafruit_RGBLCDShield.h>
//#include <utility/Adafruit_MCP23017.h>

///////////////////////// OPEN-LOOP TRACKING VARIABLES /////////////////////////////////////////

// Enter array tilt and heading //
double heading = 180 * (PI/180);
double tilt = 0 * (PI/180);

double zenith = 0;
double azimuth = 0;

// NREL solar position calculation variables
sunpos SunPos;
polar coord; // zenith and azimuth in a struct
polar coordP;
vector cart;
vector cartP;

/*
// Variables involved in finding the panel pitch from 3-axis accelerometer readings
const byte interrupt1 = 2;     // Uno can support external interrupts on pins 2 and 3
volatile boolean setPitch = false;   // boolean for setting panel tilt by reading accelerometer data; off by default
int averaging = 100;

// Acceleration components from 3-axis accelerometer
double accelX;
double accelY;
double accelZ;
*/

//////////////////////// ZABER STAGE VARIABLES ///////////////////////////////////////////////////////////
byte command[6];
byte reply[6];

long replyData;

double radius;    
double zaber[2] = {0, 0};   // [x,y] for the stages (in mm)

const unsigned long offsetX = 2119000;    //tracking the starting and current absolute positions of the stages
const unsigned long offsetY = 2095000;

unsigned long posX = 0;
unsigned long posY = 0;

long delX;
long delY;

float posXum = 0;
float posYum = 0;

int portA = 1;
int portB = 2;

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

String serialComm;

///////////////////////////// PIN DECLARATIONS ///////////////////////////////////////////////////////

// GPS uses software serial by default, with RX = pin 2 and TX = pin 3.  Mega 2560 does not support 
// software serial RX on pin 2, so add a jumper wire from pin 2 on GPS shield to RX pin used

const int rsRX = 2;     // RS-232 shield pins for controlling linear stages
const int rsTX = 3;

const int rsRXb = 4;    // RS-232 shield pins for controlling rotational stage
const int rsTXb = 5;

//////////////////////////// OBJECT DECLARATIONS /////////////////////////////////////////////////

RTC_DS1307 rtc;

// Adafruit RGB LCD Shield
//Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

//Create an object for the 6DOF IMU
//LSM303C imu;

// Create a software serial port to communicate with the Zaber linear stages
SoftwareSerial rs232(rsRX, rsTX);   

// Create a software serial port to communicate with the Zaber rotational stage
SoftwareSerial rs232b(rsRXb, rsTXb);   

////////////////////////////// CODE ///////////////////////////////////////////////////////////////

void setup() 
{
  // Start the Arduino hardware serial port at 9600 baud
  Serial.begin(57600);

  // Enable external interrupt on pin specified by interrupt1 in order to find panel pitch
  //pinMode(interrupt1, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(interrupt1), findPitch, FALLING);

  // Begin communication with LCD
  //lcd.begin(16, 2);     // (columns, rows)
  //lcd.setBacklight(0x5);
  
  // Sets the stages to use binary protocol
  rs232.begin(115200);
  delay(1000);  
  rs232.println("/tools setcomm 9600 1");
  delay(500);
  Serial.println(rs232.readStringUntil('\n'));
  delay(100);
  rs232.end();
  delay(200);

  // Start software serial connection with Zaber stages
  rs232.begin(9600);
  rs232b.begin(9600);
  delay(2000);
  replyData = sendCommand(portA, 0, 42, 34402);       // Set speed to 1 mm/s
  replyData = sendCommand(portB, 0, 42, 2276);        // Set speed to 5 deg/s 
}

void loop()
{ 
  // Serial commands
  if(Serial.available() > 0)
  {
    serialComm = Serial.readStringUntil('\n');
    if(serialComm == "zenith")
    {
      // Get zenith angle from user
      Serial.print("Enter zenith angle: ");
      serialComm == Serial.readStringUntil('\n');
      zenith = serialComm.toFloat();

      // Send zenith angle to rotational stage
      sendCommand(portB, 0, moveAbs, stepsD(zenith));
      
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
      radius = interp2(sin(coordP.ze));
      zaber[0] = (-1) * radius * sin(coordP.az);
      zaber[1] = (-1) * radius * cos(coordP.az);
    }
    if(serialComm == "getpos")
    {
      posX = sendCommand(portA, axisX, getPos, 0);
      delX = posX - offsetX;
      posXum = delX * umResolution;
      
      posY = sendCommand(portA, axisY, getPos, 0);
      delY = posY - offsetY;
      posYum = delY * umResolution;

      Serial.print(posX);
      Serial.print('\t');
      Serial.println(posY);
          
      Serial.print("X: ");
      Serial.print(delX);
      Serial.print(" uSteps, ");
      Serial.print (posXum);   
      Serial.print(" um \tY: ");
      Serial.print(delY);
      Serial.print(" uSteps, ");
      Serial.print(posYum);
      Serial.println(" um");
    }
    else if(serialComm == "getcoords")
    {
      Serial.print("Az: ");
      Serial.print(coord.az * 180/PI);
      Serial.print("\tZe: ");
      Serial.print(coord.ze * 180/PI);
      Serial.print("\tAz*: ");
      Serial.print(coordP.az * 180/PI);
      Serial.print("\tZe*: ");
      Serial.print(coordP.ze * 180/PI);
      Serial.print("\tX: ");
      Serial.print(mm(zaber[0]));
      Serial.print(" uSteps, ");
      Serial.print(zaber[0] * 1000);      
      Serial.print(" um \tY: ");
      Serial.print(mm(zaber[1]));
      Serial.print(" uSteps, ");
      Serial.print(zaber[1] * 1000);
      Serial.println(" um");
    }
    else if(serialComm == "goto")
    {
      sendCommand(portA, axisX, moveAbs, mm(zaber[0]) + offsetX);
      sendCommand(portA, axisY, moveAbs, mm(zaber[1]) + offsetY);
    }
    else if(serialComm = "origin")
    {
      sendCommand(portA, axisX, moveAbs, offsetX);
      sendCommand(portA, axisY, moveAbs, offsetY);
    }
  }
}

long sendCommand(int port, int device, int com, long data)
{
   unsigned long data2;
   unsigned long temp;
   unsigned long repData;
   long replyNeg;
   float replyFloat;
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

   if(port == 1)
   {
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
   }
   else if(port == 2)
   {
     // Clearing serial buffer
     while(rs232b.available() > 0)
     {
       rs232b.readBytes(dumper, 1);
     }
     
     // Sending command to stage(s)
     rs232b.write(command, 6);
  
     delay(20);
     
     // Reading device reply
     if(rs232b.available() > 0)
     {
       rs232b.readBytes(reply, 6);
     }
   }
   
   replyFloat = (cubed * float(reply[5])) + (squared * float(reply[4])) + (256 * float(reply[3])) + float(reply[2]); 
   repData = long(replyFloat);
   
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
   Serial.print("\tData:");
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
