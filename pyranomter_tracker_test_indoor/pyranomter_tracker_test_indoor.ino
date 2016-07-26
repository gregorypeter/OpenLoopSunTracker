/*   Test sketch for pyranometer tracker
 *      
 *   Michael Lipski
 *   AOPL
 *   Summer 2016
 *   
 *   Allows user to input azimuth and zenith angles in degrees via the serial console.  These angles get converted into microsteps and sent to the Zaber
 *   T-series rotational stages controlling the pyranometer.
 */

#include <zaberx.h>

#include <SoftwareSerial.h>

#ifndef PI
#define PI 3.14159265358979
#endif

//    Zaber rotational stage variables
byte command[6];
char reply[6];

float phi;    // azimuthal angle
float theta;  // elevation angle

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

String comm;

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
  delay(200);
  Serial.println("\tPyranometer tracker test sketch");
  Serial.println("\tMake sure stages are homed");
  Serial.println("--------------------------------------------------------");

  //  Start software serial port with Zaber rotational stage
  rs232.begin(9600);
  
  delay(1000);
  sendCommand(0, renumber, 0);
  delay(500);
  sendCommand(0, speedSet, 4551);   // Set speed to 10 degrees/sec
  delay(500);
  //sendCommand(0, homer, 0);
  //delay(5000);
  Serial.println("Enter azimuth and zenith angles, separated by a space:");
}

void loop()
{
  if(Serial.available() > 0)
  {
    comm = Serial.readStringUntil(' ');       
    phi = comm.toFloat();
    comm = Serial.readStringUntil('\n');
    theta = 90 - comm.toFloat();
    Serial.print(phi);
    Serial.print(' ');
    Serial.println(theta);
        
    sendCommand(azimuth, moveAbs, stepsD(phi));
    sendCommand(zenith, moveAbs, stepsD(theta));
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
