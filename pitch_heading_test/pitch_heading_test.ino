/*   LSM303C 6DOF IMU (6 Degree-of-freedom inertial measurement unit) Test Sketch
 *      
 *   Michael Lipski
 *   AOPL
 *   Summer 2016
 *   
 *   Reads X, Y, and Z components of acceleration and magnetic field and, assuming the board is stationary and is subject to only the earth's magnetic
 *   field, determines the pitch and heading of the board.    
 */

#include <Wire.h>

#include <translate.h>

#include <DebugMacros.h>
#include <LSM303CTypes.h>
#include <SparkFunIMU.h>
#include <SparkFunLSM303C.h>

const int iter8 = 50;

double accelX = 0;  //accelerometer components
double accelY = 0;  
double accelZ = 0; 
double accelM = 0; //magnitude of acceleration vector

double magX = 0;  //magnetic field strength in x direction
double magY = 0;  //magnetic field strength in y direction
double magZ = 0;  //magnetic field strength in z direction
double magM = 0;  //magnitude of magnetic vector

double xHor = 0;  //x component of projection of magnetic field vector onto XY plane
double yHor = 0;  //y component of projection of magnetic field vector onto XY plane

double pitch = 0;   //calculated values of pitch, roll, and yaw
double roll = 0;
double heading = 0;

double emY = -0.38444;   //Y and Z components of unit vector in direction of earth's magnetic field
double emZ = -0.92315;
double emYi = 1/emY;
double trans[3][3];

polar coord;
polar coordP;
vector v_i;
vector v_f;

//Create an object to control LSM303C 6DOF IMU
LSM303C imu;            // Uno: A4 is SDA, A5 is SCL

void setup() 
{
  Serial.begin(9600);

  /*
  Wire.begin();
  delay(100);
  Wire.beginTransmission(0x1D);
  Wire.write(byte(0x20));
  Wire.write(byte(0x57));
  Wire.endTransmission();
  delay(100);
  */
  
  
  if (imu.begin() != IMU_SUCCESS)
  {
    Serial.println("Failed setup.");
    while(1);
  }
  delay(1000);
}

void loop() 
{
  //Determining tilt from accelerometer readings   
  
  accelX = 0;
  accelY = 0;
  accelZ = 0;
  magX = 0;
  magY = 0;
  magZ = 0;

  for(int i = 0; i < iter8; i++)
  {
    accelX += imu.readAccelX();
    accelY += imu.readAccelY();
    accelZ += imu.readAccelZ();
    magX += imu.readMagX();
    magY += imu.readMagY();
    magZ += imu.readMagZ();
    
    delay(9);
  }

  accelX /= iter8;
  accelY /= iter8;
  accelZ /= iter8;
  magX /= iter8;
  magY /= iter8;
  magZ /= iter8;

  /*
  accelM = sqrt(accelX*accelX + accelY*accelY + accelZ*accelZ);   
  magM = sqrt(magX*magX + magY*magY + magZ*magZ);  

  trans[0][2] = (-1) * (accelX/accelM);
  trans[1][2] = (-1) * (accelY/accelM);
  trans[2][2] = (-1) * (accelZ/accelM);
  trans[0][1] = ((magX/magM) + (emZ * trans[0][2])) * emYi;
  trans[1][1] = ((magY/magM) + (emZ * trans[1][2])) * emYi;
  trans[2][1] = ((magX/magM) + (emZ * trans[2][2])) * emYi;
  trans[0][0] = (trans[1][1] * trans[2][2]) - (trans[2][1] * trans[1][2]);
  trans[1][0] = (trans[2][1] * trans[0][2]) - (trans[0][1] * trans[2][2]);
  trans[2][0] = (trans[0][1] * trans[1][2]) - (trans[1][1] * trans[0][2]);

  v_i.x = 0;
  v_i.y = 1;
  v_i.z = 0;
  
  v_f.x = (trans[0][0] * v_i.x) + (trans[0][1] * v_i.y) + (trans[0][2] * v_i.z);
  v_f.y = (trans[1][0] * v_i.x) + (trans[1][1] * v_i.y) + (trans[1][2] * v_i.z);
  v_f.z = (trans[2][0] * v_i.x) + (trans[2][1] * v_i.y) + (trans[2][2] * v_i.z);

  coordP = rect2sph(v_f);

  heading = (-1)*coordP.az * (180/PI);
  if(heading < 0)
    heading += 360;
  */
  
  pitch = atan2(accelX , sqrt((accelY * accelY) + (accelZ * accelZ)));
  roll = atan2(accelY , sqrt((accelX * accelX) + (accelZ * accelZ))); 

  yHor = magY*cos(roll) + magZ*sin(roll);
  xHor = magX*cos(pitch) + magZ*sin(pitch)*sin(roll) + magZ*sin(pitch)*cos(roll);

  heading = (-atan2(yHor , xHor) * (180/PI)) + 180;

  /*
  Serial.print("X: ");
  Serial.print(magX);
  Serial.print("\tY: ");
  Serial.print(magY);
  Serial.print("\tZ: ");
  Serial.println(magZ);
  */

  /*
  Serial.print("AccelX: ");
  Serial.print(accelX);
  Serial.print("\tAccelY: ");
  Serial.print(accelY);
  Serial.print("\tAccelZ: ");
  Serial.print(accelZ);
  */

  /*
  Serial.print("AccelX: ");
  Serial.print(averageX);
  Serial.print("\tAccelY: ");
  Serial.print(averageY);
  Serial.print("\tAccelZ: ");
  Serial.print(averageZ);
  */
  
  Serial.print("\tPitch:");
  Serial.print(pitch * (180/PI));
  Serial.print("\tRoll:");
  Serial.print(roll * (180/PI));
  Serial.print("\tHeading:");
  Serial.println(heading);  
  
}
