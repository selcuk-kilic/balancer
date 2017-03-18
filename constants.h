#ifndef constants_h
#define constants_h


const float TIMER_PERIOD_SEC = 0.04;
const unsigned short ACCELEROMETER_RATE = 400;
const unsigned short GYROSCOPE_RATE = 400;
const unsigned short ACCELEROMETER_RANGE = 2;
const unsigned short GYROSCOPE_RANGE = 250;

#define MOTOR_A_PIN_EN 9
#define MOTOR_A_PIN_P 10
#define MOTOR_A_PIN_N 8

#define MOTOR_B_PIN_EN 3
#define MOTOR_B_PIN_P 2
#define MOTOR_B_PIN_N 4

const float ACCELEROMETER_SENSITIVITY = 32768.0/ACCELEROMETER_RANGE;
const float GYROSCOPE_SENSITIVITY = 32768.0/GYROSCOPE_RANGE;
const float RADIANS_TO_DEGREES = 180/3.14159;

#endif //constants_h
