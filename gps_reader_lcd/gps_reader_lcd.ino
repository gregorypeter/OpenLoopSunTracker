/*  GPS data reader for an EM 506 gps module
 *   Output to Hitachi HD44780 LCD
 *   
 *   Michael Lipski
 *   AOPL
 *   Summer 2016
 * 
 *  Reads date, time, lat and long from gps and outputs to LCD screen.
 */
#include <SoftwareSerial.h>

#include <TinyGPS++.h>

#include <Wire.h>
#include "RTClib.h"
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>

int GPSBaud = 4800;

int RXPin = 10;
int TXPin = 3;

// Create a TinyGPS++ object called "gps"
TinyGPSPlus gps;

// Create a software serial port called "gpsSerial"
SoftwareSerial gpsSerial(RXPin, TXPin);  

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

void setup()
{
  // begin communication with LCD
  lcd.begin(16, 2);     // (columns, rows)
  lcd.setBacklight(0x5);

  // Start the software serial port at the GPS's default baud
  gpsSerial.begin(GPSBaud);
}

void loop() 
{
    if (gpsSerial.available() > 0)
    {
      if (gps.encode(gpsSerial.read()))
      {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(gps.date.month());
          lcd.print('/');
          lcd.print(gps.date.day());
          lcd.print('/');
          lcd.print(gps.date.year());
          lcd.print(' ');
          lcd.print(gps.time.hour());
          lcd.print(':');
          lcd.print(gps.time.minute());
          lcd.setCursor(0, 1);
          lcd.print(gps.location.lat());
          lcd.setCursor(8, 1);
          lcd.print(gps.location.lng());
      }
    }
    delay(500);
}
