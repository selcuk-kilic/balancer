#include "ahrs.h"
#include "CurieTimerOne.h"
#include "constants.h"

Madgwick filter;
Reporter reporter;
unsigned long t0 = 0;
unsigned long t1 = 0;
float pitch = 0;
float roll = 0;
#define M_PI 3.14159265359	    

void complementaryFilter(float ax, float ay, float az, float gx, float gy, float gz, float dt)
{
    float pitchAcc, rollAcc;               
 
    // Integrate the gyroscope data -> int(angularSpeed) = angle
    pitch += gx * dt; // Angle around the X-axis
    roll -= gy * dt;    // Angle around the Y-axis
 
    // Compensate for drift with accelerometer data if !bullshit
    // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
    int forceMagnitudeApprox = abs(ax) + abs(ay) + abs(az);
    if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
    {
	// Turning around the X axis results in a vector on the Y-axis
        pitchAcc = atan2f(ay, az) * 180 / M_PI;
        pitch = pitch * 0.98 + pitchAcc * 0.02;
 
	// Turning around the Y axis results in a vector on the X-axis
        rollAcc = atan2f(ax, az) * 180 / M_PI;
        roll = roll * 0.98 + rollAcc * 0.02;
    }
}

void AHRS::filterUpdateIsr() {
  t0 = micros();
  Serial.print("AHRSsure: ");
  Serial.print(t0-t1);
  Serial.print(" ");
  int aix, aiy, aiz;
  int gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;
  
  CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);
  ax = convertRawAcceleration(aix);
  ay = convertRawAcceleration(aiy);
  az = convertRawAcceleration(aiz);
  gx = convertRawGyro(gix);
  gy = convertRawGyro(giy);
  gz = convertRawGyro(giz);

  filter.updateIMU(gx, gy, gz, ax, ay, az);
  //complementaryFilter(ax,ay,az,gx,gy,gz,0.01);

  //filter.getYaw();
  t1 = micros();
  Serial.print(t1-t0);
  Serial.println();
  //reporter(0,roll,pitch);//
  reporter(filter.getYaw(),filter.getPitch(),filter.getRoll());
}

float AHRS::convertRawAcceleration(int aRaw) {
  return ((float)aRaw / 32768.0) * ACCELEROMETER_RANGE;
}

float AHRS::convertRawGyro(int gRaw) {
  return ((float)gRaw / 32768.0) * GYROSCOPE_RANGE;
}


/*
 suureler:
 100Hz: 36340 13660
 50Hz:  46340 13660
 25Hz:  66340 13660
 */
void AHRS::setup(Reporter sendResult) {
  // start the IMU and filter
  CurieIMU.begin();
  CurieIMU.setGyroRate(GYROSCOPE_RATE);
  CurieIMU.setAccelerometerRate(ACCELEROMETER_RATE);
  CurieIMU.setAccelerometerRange(ACCELEROMETER_RANGE);
  CurieIMU.setGyroRange(GYROSCOPE_RANGE);

  reporter = sendResult;

  filter.begin(IMU_FILTER_FREQ_HZ);

  const int oneSecInUsec = 1000000;   // A second in mirco second unit.
  int _time = oneSecInUsec / IMU_FILTER_FREQ_HZ;
  CurieTimerOne.start(_time, &filterUpdateIsr);  // set timer and callback
}


