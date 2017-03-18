
#include "constants.h"
#include "Drive.h"
#include "PID.h"
#include <CurieIMU.h>
#include "CurieTimerOne.h"

#include <BlynkSimpleCurieBLE.h> 
#include <CurieBLE.h> 
// You should get Auth Token in the Blynk App. 
// Go to the Project Settings (nut icon). 
char auth[] = "9da31fec1bbe40d987ecb5d7d461f144"; 
BLEPeripheral  blePeripheral; 
int kp_us=0,kp_alt=0;
int kd_us=0,kd_alt=0;

Drive drive;
PID pid;
float last_roll=0;
int roll_init_counter=0;

float find_roll(int ax, int ay, int az, 
            float gx, float last_roll, float dt)
{
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
  //ax = convertRawAcceleration(aix);
  //ay = convertRawAcceleration(aiy);
  //az = convertRawAcceleration(aiz);
  gx = convertRawGyro(gix);
  //gy = convertRawGyro(giy);
  //gz = convertRawGyro(giz);

  if(roll_init_counter>0)
  {
    last_roll += atan(aiy/sqrt(aix*aix + aiz*aiz))*RADIANS_TO_DEGREES;
    digitalWrite(13, HIGH);
    if(roll_init_counter==1)
    {
      last_roll /= 50;
      digitalWrite(13, LOW);
    }
    roll_init_counter--;
    return;
  }

  const float roll = find_roll(aix,aiy,aiz,gx,last_roll,TIMER_PERIOD_SEC);

  pid.Kp = kp_us + ((float)kp_alt)/100;
  pid.Kd = kd_us + ((float)kd_alt)/100;
  Serial.print(pid.Kp);
  Serial.print("  ");
  Serial.println(pid.Kd);
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
  pid.lowLimit = -255.0;
  pid.highLimit = 255.0;
  // Setup the LED pin
  pinMode(13, OUTPUT);
  roll_init_counter = 50;
  
   blePeripheral.setLocalName("Blynk Hello"); 
   blePeripheral.setDeviceName("Blynk Hello"); 
   blePeripheral.setAppearance(384); 
   Blynk.begin(blePeripheral, auth); 
   blePeripheral.begin(); 
}

BLYNK_WRITE(V0) {
 int value = param.asInt();
 kp_us = value;
} 

BLYNK_WRITE(V1) {
 int value = param.asInt();
 kp_alt = value;
} 

BLYNK_WRITE(V2) {
 int value = param.asInt();
 kd_us = value;
} 

BLYNK_WRITE(V3) {
 int value = param.asInt();
 kd_alt = value;
} 

void loop() { 
 Blynk.run(); 
 blePeripheral.poll(); 
} 