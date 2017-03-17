
#include "constants.h"
#include "Drive.h"
#include "PID.h"
#include <CurieIMU.h>
#include "CurieTimerOne.h"

Drive drive;
PID pid;
float last_roll=0;

float find_roll(int ax, int ay, int az, 
            float gx, float last_roll, float dt)
{
  float RADIANS_TO_DEGREES = 180/3.14159;
  float accel_angle = atan(ay/sqrt(ax*ax + az*az))*RADIANS_TO_DEGREES;
  
  float gyro_angle = gx*dt + last_roll;
  
  // Apply the complementary filter to figure out the change in angle - choice of alpha is
  // estimated now.  Alpha depends on the sampling rate...
  float alpha = 0.96;
  return alpha*gyro_angle + (1.0 - alpha)*accel_angle;
}

float convertRawAcceleration(int aRaw) {
  return ((float)aRaw / 32768.0) * ACCELEROMETER_RANGE;
}

float convertRawGyro(int gRaw) {
  return ((float)gRaw / 32768.0) * GYROSCOPE_RANGE;
}

void TIMERISR() {
  
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

  const float roll = find_roll(aix,aiy,aiz,gx,last_roll,TIMER_PERIOD_SEC);
  float Power = pid.Update(roll, TIMER_PERIOD_SEC);

  drive.bothMotors((Power>0), (unsigned char) abs(Power),(Power>0), (unsigned char) abs(Power) );

  last_roll = roll;
}

void setup() {
  Serial.begin(9600);
  CurieIMU.begin();
  CurieIMU.setGyroRate(GYROSCOPE_RATE);
  CurieIMU.setAccelerometerRate(ACCELEROMETER_RATE);
  CurieIMU.setAccelerometerRange(ACCELEROMETER_RANGE);
  CurieIMU.setGyroRange(GYROSCOPE_RANGE);
  CurieTimerOne.start((int)(TIMER_PERIOD_SEC*1000000), &TIMERISR);  // set timer and callback
  drive.setup();
  pid.Kp = 8;
  pid.Ki = 0.0;
  pid.Kd = 2.5;
  //derivativeFiltration = config_reader.GetFloat("pid", "angle_derivativeFiltration", 0.1f);
  pid.lowLimit = -120.0;
  pid.highLimit = 120.0;
}

void loop() {
  delay(10000);
}
