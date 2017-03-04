
#include <Arduino.h>
#include "Drive.h"
#include "constants.h"

void Drive::setup() {
  // set all the motor control pins to outputs
  pinMode(MOTOR_A_PIN_EN, OUTPUT);
  pinMode(MOTOR_A_PIN_P, OUTPUT);
  pinMode(MOTOR_A_PIN_N, OUTPUT);
  pinMode(MOTOR_B_PIN_EN, OUTPUT);
  pinMode(MOTOR_B_PIN_P, OUTPUT);
  pinMode(MOTOR_B_PIN_N, OUTPUT);
}

void Drive::motorA(bool direction, unsigned char speed) {
  if(direction){
    digitalWrite(MOTOR_A_PIN_P, HIGH);
    digitalWrite(MOTOR_A_PIN_N, LOW);
  }
  else {
    digitalWrite(MOTOR_A_PIN_N, HIGH);
    digitalWrite(MOTOR_A_PIN_P, LOW);
  }
  analogWrite(MOTOR_A_PIN_EN, (speed>>1) ); 
}

void Drive::motorB(bool direction, unsigned char speed) {
  if(direction){
    digitalWrite(MOTOR_B_PIN_P, HIGH);
    digitalWrite(MOTOR_B_PIN_N, LOW);
  }
  else {
    digitalWrite(MOTOR_B_PIN_N, HIGH);
    digitalWrite(MOTOR_B_PIN_P, LOW);
  }
  analogWrite(MOTOR_B_PIN_EN, (speed>>1) );   
}

void Drive::bothMotors(bool directionA, unsigned char speedA,bool directionB, unsigned char speedB) {
  motorA(directionA,speedA);
  motorB(directionB,speedB);
}

