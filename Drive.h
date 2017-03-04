#ifndef Drive_h
#define Drive_h


class Drive {

public:
  void setup();

  void motorA(bool direction, unsigned char speed);
  void motorB(bool direction, unsigned char speed);
  void bothMotors(bool directionA, unsigned char speedA,bool directionB, unsigned char speedB);
  
};


#endif //Drive.h
