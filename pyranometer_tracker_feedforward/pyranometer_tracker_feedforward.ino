/*   Pyranometer tracker using feed-forward only
 *      
 *   Michael Lipski
 *   AOPL
 *   Summer 2016
 *   
 *   Feed-forward tracking for pyranometer mount pulls data from GPS unit, calculates solar position, then sends move commands to Zaber T-series rotational stages.
 */

#include <zaberx.h>

#include <TinyGPS++.h>

#include <Sun_position_algorithms.h>
#include <translate.h>

#include <SoftwareSerial.h>

//   solar tracking variables
sunpos SunPos;
float phi;    // azimuthal angle
float theta;  // elevation angle

//    Zaber rotational stage variables
byte command[6];
char reply[6];

int azimuth = 1;    // Device ID of azimuth stage
int zenith = 2;     // Device ID of elevation stage

long azimPos = 0;   // Variable which tracks the absolute position of the azimuth stage (in microsteps)
long zeniPos = 0;   // Variable which tracks the absolute position of the elevation stage (in microsteps)

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

const unsigned int intervalLong = 60000;    // Period of feed-forward iterations

unsigned long currentMillis = 0;
unsigned long longMillis = 0;

int GPSBaud = 4800;

//On Mega, RX must be one of the following: pin 10-15, 50-53, A8-A15
int RXPin = 2;
int TXPin = 3;
int rsRX = 4;
int rsTX = 5;

// Create a TinyGPS++ object called "gps"
TinyGPSPlus gps;

// Create a software serial port called "gpsSerial"
SoftwareSerial gpsSerial(RXPin, TXPin);  

SoftwareSerial rs232(rsRX, rsTX);   //RX, TX

void setup() 
{
  //  Open serial connection with computer
  Serial.begin(9600);
  
  // Start the software serial port at the GPS's default baud
  gpsSerial.begin(GPSBaud);

  //  Start software serial port with Zaber rotational stage
  rs232.begin(9600);
  
  delay(1000);
  sendCommand(0, renumber, 0);
  delay(1000);
  sendCommand(0, speedSet, 4551);   // Set speed to 10 degrees/sec
  delay(1000);
  //sendCommand(0, homer, 0);
  //delay(5000);
}

void loop()
{
  currentMillis = millis();

  //  Feed-forward tracking
  if(currentMillis - longMillis >= intervalLong)
  {
    while (gpsSerial.available() > 0)
    {
      if(gps.encode(gpsSerial.read()))
      {
        longMillis = currentMillis;
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

        theta = SunPos.Zenith * (180/PI);
        phi = SunPos.Azimuth * (180/PI) + 180;

        sendCommand(azimuth, moveAbs, phi);
        sendCommand(zenith, moveAbs, theta);
      }
    }
  }
}

void sendCommand(int device, int com, long data)
{
   unsigned int temp;
   long replyData;
   
   // Building the six command bytes
   command[0] = byte(device);
   command[1] = byte(com);
   if(data < 0)
   {
     data +=  quad;
   }
   temp = data / cubed;
   command[5] = byte(temp);
   data -= (cubed * temp);
   temp = data / squared;
   command[4] = byte(temp);
   data -= (squared * temp);
   temp = data / 256;
   command[3] = byte(temp);
   data -= (256 * data);
   command[2] = byte(data);
   
   // Sending command to stage(s)
   rs232.write(command, 6);

   // Updating position of stage
   if(com == moveAbs)
   {
     if(device == azimuth)
     {
       azimPos = data;
     }
     else if(device == zenith)
     {
       zeniPos = data;
     }
   }
   else if(com == moveRel)
   {
     if(device == azimuth)
     {
       azimPos += data;
     }
     else if(device == zenith)
     {
       zeniPos += data;
     }
   }
   else if(com == homer)
   {
     azimPos = 0;
     zeniPos = 0;
   }
   
   // Reading device reply
   if(rs232.available() > 0)
   {
     rs232.readBytes(reply, 6);
   }

   replyData = (cubed * reply[5]) + (squared * reply[4]) + (256 * reply[3]) + reply[2];
   if(reply[5] > 127)
   {
     replyData -= quad;
   }

   /*
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
   Serial.print(reply[5]);
   Serial.print("\tData:");
   Serial.println(replyData);  
   */ 
}
