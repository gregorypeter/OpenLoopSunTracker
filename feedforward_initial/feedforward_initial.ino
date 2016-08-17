/*   CPV Feed-forward tracking
 *   
 *   Michael Lipski
 *   AOPL
 *   Summer 2016
 *   
 *   Feed-forward tracking for CPV test setup.  Reads data from GPS unit, calculates position of the sun, and converts solar position to mm
 *   displacement along X and Y of the Zaber linear stages.
 */

#include <zaberx.h>

#include <TinyGPS++.h>

#include <DebugMacros.h>
#include <LSM303CTypes.h>
#include <SparkFunIMU.h>
#include <SparkFunLSM303C.h>

#include <Sun_position_algorithms.h>
#include <translate.h>

#include <SoftwareSerial.h>

// GPS uses software serial by default, with RX = pin 2 and TX = pin 3.  Mega 2560 does not support software serial RX on pin 2, so add a jumper wire from pin 2 on GPS shield to RX pin used
int RXPin = 2;
int TXPin = 3;
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

sunpos SunPos;
polar coord; // zenith and azimuth in a struct
polar coordP;
vector cart;
vector cartP;

// Enter array tilt and heading
double heading = 180 * (PI/180);
double tilt = 0 * (PI/180);

double radius;
double zaberOld[2] = {0, 0};    // [x,y] for the stages (in mm)
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

int GPSBaud = 4800;

// Create a TinyGPS++ object called "gps"
TinyGPSPlus gps;

//Create an object for the 6DOF IMU
LSM303C imu;

// Create a software serial port called "gpsSerial"
SoftwareSerial gpsSerial(RXPin, TXPin);  

// Create a software serial port to communicate with the Zaber stages
SoftwareSerial rs232(rsRX, rsTX);   

void setup() 
{
  // Start the Arduino hardware serial port at 9600 baud
  Serial.begin(9600);

  // Enable external interrupt on pin specified by interrupt1 in order to find panel pitch
  pinMode(interrupt1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interrupt1), findPitch, FALLING);

  // Start the software serial port at the GPS's default baud
  gpsSerial.begin(GPSBaud);

  //Start software serial connection with Zaber stages
  rs232.begin(115200);
  delay(2000);
  rs232.println("/renumber");
  delay(1000);
  rs232.println("/home");
  delay(10000);
  zMove(axisX, offsetX);
  zMove(axisY, offsetY);
}

void loop()
{
  currentMillis = millis();
  if(currentMillis - previousMillis >= interval)
  {
    if(gpsSerial.available() > 0)
    {
      if(gps.encode(gpsSerial.read()))
      {
        SunPos.UT = gps.time.hour() + double(gps.time.minute())/60.0 + double(gps.time.second())/3600.0 + double(gps.time.centisecond())/360000; // UT in hours [decimal]
        SunPos.Day = gps.date.day(); // day [integer]
        SunPos.Month = gps.date.month(); // month [integer]
        SunPos.Year = gps.date.year(); // year [integer]
        SunPos.Dt = 96.4 + 0.567*double(gps.date.year()-2061); // Terrestial time - UT
        SunPos.Longitude = gps.location.lng() * (2*PI/360.0); // State College Longitude and Latitude [radians]      
        SunPos.Latitude = gps.location.lat() * (2*PI/360.0);
        SunPos.Pressure = 1.0; // Pressure [atm]
        //SunPos.Temperature = imu.readTempC(); // Temperature [C], pulled from LSM303C 6DOF sensor     
        SunPos.Temperature = 20.0;
        
        SunPos.Algorithm5();
  
        coord.ze = SunPos.Zenith;
        coord.az = SunPos.Azimuth + PI;
  
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
        zaberOld[0] = zaber[0];
        zaberOld[1] = zaber[1];
        if((coordP.ze < 90) && (coordP.ze > 0))
        {
          radius = interp1(sin(coordP.ze));
          zaber[0] = (-1) * radius * sin(coordP.az);
          zaber[1] = (-1) * radius * cos(coordP.az);
        }
        zMove(axisX, mm(zaber[0]));
        zMove(axisY, mm(zaber[1]));
      }
    }
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
