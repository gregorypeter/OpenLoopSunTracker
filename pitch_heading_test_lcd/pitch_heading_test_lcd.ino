/*   LSM303C 6DOF IMU (6 Degree-of-freedom inertial measurement unit) Test Sketch
 *    Using Hitachi HD44780 chipset LCD screen
 *      
 *   Michael Lipski
 *   AOPL
 *   Summer 2016
 *   
 *   Reads X, Y, and Z components of acceleration and magnetic field and, assuming the board is stationary and is subject to only the earth's magnetic
 *   field, determines the pitch and heading of the board.  Outputs data to LCD screen instead of serial monitor.
 */
#include <LiquidCrystal.h>

#include <DebugMacros.h>
#include <LSM303CTypes.h>
#include <SparkFunIMU.h>
#include <SparkFunLSM303C.h>

double accelX = 0;  //accelerometer components
double accelY = 0;  
double accelZ = 0;  

double magX = 0;  //magnetic field strength in x direction
double magY = 0;  //magnetic field strength in y direction
double magZ = 0;  //magnetic field strength in z direction
double xHor = 0;  //x component of projection of magnetic field vector onto XY plane
double yHor = 0;  //y component of projection of magnetic field vector onto XY plane

double pitch = 0;   //calculated values of pitch, roll, and yaw
double roll = 0;
double heading = 0;

int iter8 = 50;

// Create an object to control LCD
LiquidCrystal lcd(8, 9, 10, 11, 12, 13);      // (RS, enable, D4, D5, D6, D7); R/W tied to ground for write only

// Create an object to control LSM303C 6DOF IMU
LSM303C imu;                // Uno: A4 is SDA, A5 is SCL

void setup() 
{
  lcd.begin(16, 2);    // (characters per line, # of lines);
  delay(1000);
}

void loop() 
{
  //Determining tilt from accelerometer readings
  accelX = 0;
  accelY = 0;
  accelZ = 0;

  for(int i = 0; i < iter8; i++)
  {
    accelX += imu.readAccelX();
    accelY += imu.readAccelY();
    accelZ += imu.readAccelZ();
  }

  accelX /= iter8;
  accelY /= iter8;
  accelZ /= iter8;

  pitch = atan2(accelX , sqrt(pow(accelY , 2) + pow(accelZ , 2)));
  roll = atan2(accelY , sqrt(pow(accelX, 2) + pow(accelZ, 2)));
  
  //Determining heading from X & Y magnetometer readings
  magX = 0;
  magY = 0;
  magZ = 0;

  for(int i = 0; i < iter8; i++)
  {
    magX += imu.readMagX();
    magY += imu.readMagY();
    magZ += imu.readMagZ();
  }

  magX /= iter8;
  magY /= iter8;
  magZ /= iter8;

  yHor = magY*cos(roll) + magZ*sin(roll);
  xHor = magX*cos(pitch) + magZ*sin(pitch)*sin(roll) + magZ*sin(pitch)*cos(roll);

  heading = (-atan2(yHor , xHor) * (180/PI)) + 180;

  lcd.clear();
  lcd.print(" P: ");
  lcd.print(pitch);
  lcd.setCursor(0, 1);
  lcd.print(" H: ");
  lcd.print(heading);
}
