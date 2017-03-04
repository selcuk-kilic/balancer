#include "ahrs.h"
#include "Drive.h"
#include "PID.h"

AHRS ahrs;
Drive drive;
PID pid;

void getOrientation(float yaw, float pitch, float roll) {
  Serial.print("Orientation: ");
  Serial.print(yaw);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.println(roll);  

  float Power = pid.Update(3*roll, 0.4);
    
  drive.bothMotors((Power>0), (unsigned char) abs(Power),(Power>0), (unsigned char) abs(Power) );
}

void setup() {
  Serial.begin(9600);
  ahrs.setup(&getOrientation);
  drive.setup();
    pid.Kp = 2.7;
    pid.Ki = 0.0;
    pid.Kd = 0.3;
    //derivativeFiltration = config_reader.GetFloat("pid", "angle_derivativeFiltration", 0.1f);
    pid.lowLimit = -120.0;
    pid.highLimit = 120.0;
}

void loop() {
  delay(10000);
}
