/*   Sample code for testing the conversion from solar coordinates to planar coordinates in the array of the concentrator cell
 *    Using f20 lens data
 * 
 *   Michael Lipski
 *   AOPL
 *   Summer 2016
 * 
 *   Gets data from GPS, calculates solar position, then transforms local horizontal solar coordinates into XY coordinates
 *   in the plane of the concentrator.
 */

#include <TinyGPS++.h>

#include <Sun_position_algorithms.h>
#include <translate.h>

#include <SoftwareSerial.h>

//GPS uses software serial by default, with RX = pin 2 and TX = pin 3.  Mega 2560 does not support software serial RX on pin 2, so add a jumper wire from pin 2 on GPS shield to RX pin used
int RXPin = 2;
int TXPin = 3;

sunpos SunPos;
polar coord; // zenith and azimuth in a struct
polar coordP;
vector cart;
vector cartP;

double radius;
double zaberOld[2] = {0, 0};    // [x,y] for the stages (in mm)
double zaber[2] = {0, 0};

//Enter array tilt and heading
double heading = 180 * (PI/180);
double tilt = 0 * (PI/180);

int GPSBaud = 4800;

double p[] = {0,  0.0235, 0.0470, 0.0705, 0.0940, 0.117,  0.141,  0.164,  0.188,  0.211,  0.235,  0.258,  0.282,  0.305,  
          0.329,  0.352,  0.376,  0.399,  0.423,  0.446,  0.470,  0.493,  0.517,  0.540,  0.564,  0.587,  0.611,  0.634,  
          0.658,  0.681,  0.705,  0.728,  0.752,  0.775,  0.799,  0.822,  0.846,  0.869,  0.893,  0.916,  0.940};
          
double q[] = {0,  0.132,  0.264,  0.397,  0.529,  0.662,  0.795,  0.928,  1.06, 1.20, 1.33, 1.46, 1.60, 1.74, 1.87, 2.01, 
              2.15, 2.29, 2.42, 2.57, 2.71, 2.85, 2.99, 3.14, 3.28, 3.43, 3.58, 3.73, 3.89, 4.05, 4.21, 4.37, 4.53, 4.69, 
              4.86, 5.04, 5.21, 5.39, 5.57, 5.76, 5.96};

// Create a TinyGPS++ object called "gps"
TinyGPSPlus gps;

// Create a software serial port called "gpsSerial"
SoftwareSerial gpsSerial(RXPin, TXPin);

void setup() 
{
  // Start the Arduino hardware serial port at 9600 baud
  Serial.begin(9600);

  // Start the software serial port at the GPS's default baud
  gpsSerial.begin(GPSBaud);
  delay(2000);
  Serial.println("Solar position routine begin");

}

void loop()
{
   while (gpsSerial.available() > 0)
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
      
      /*
      if((coord.az > PI) && (coordP.az < 0))
      {
        coordP.az += (2* PI);
      }
      else if((coord.az < PI) && (coordP.az < 0))
      {
        coordP.az += PI;
      }
      else if((coord.az > PI) && (coordP.az > 0))
      {
        coordP.az += PI;
      }
      */      

      //  Determining zaber stage coordinates
      zaberOld[0] = zaber[0];
      zaberOld[1] = zaber[1];
      if((coordP.ze < 90) && (coordP.ze > 0))
      {
        radius = interp1(sin(coordP.ze));
        zaber[0] = (-1) * radius * sin(coordP.az);
        zaber[1] = (-1) * radius * cos(coordP.az);
      }      
    
      // prints out timestamp and zenith and azimuth angles in degrees    
      
      Serial.print(gps.date.month());
      Serial.print(F("/"));
      Serial.print(gps.date.day());
      Serial.print(F("/"));
      Serial.print(gps.date.year());   
      Serial.print(F(" "));
      if (gps.time.isValid())
      {
        if (gps.time.hour() < 10) Serial.print(F("0"));
        Serial.print(gps.time.hour());
        Serial.print(F(":"));
        if (gps.time.minute() < 10) Serial.print(F("0"));
        Serial.print(gps.time.minute());
        Serial.print(F(":"));
        if (gps.time.second() < 10) Serial.print(F("0"));
        Serial.print(gps.time.second());
        Serial.print(F("."));
        if (gps.time.centisecond() < 10) Serial.print(F("0"));
        Serial.print(gps.time.centisecond());
      }
      else
      {
        Serial.print(F("INVALID"));
      }
      Serial.print("\t");
      Serial.print(gps.location.lat(), 6);
      Serial.print(F(","));
      Serial.print(gps.location.lng(), 6);
      Serial.print("\tZenith: ");
      Serial.print(coord.ze * (180/PI));
      Serial.print(" Azimuth: ");
      Serial.print(coord.az * (180/PI));    
      Serial.print("\tZenith*:");
      Serial.print(coordP.ze * 180/PI);
      Serial.print("\tAzimuth*:");
      Serial.print(coordP.az * 180/PI);  
      Serial.print("\tX:");
      Serial.print(zaber[0]);
      Serial.print("\tY:");
      Serial.println(zaber[1]);
      delay(500);
    }    
  } 
}

double interp1(double input)
{
  double out;
  int i = 0;
  while(input >= p[i])
  {
    i++;
  }
  out = q[i-1] + ((input - p[i-1])/(p[i] - p[i-1])) * (q[i] - q[i-1]);
  return out;
}

