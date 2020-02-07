#include "Arduino.h"
#include "AccelStepper.h"

int dir_pin = 2;
int step_pin = 3;
int enable_pin = 4;

AccelStepper motor(1, step_pin, dir_pin);

void setup() {
  Serial.begin(9600);

  motor.setMaxSpeed(1000);
  motor.setAcceleration(1000);
  motor.moveTo(500);
}

void loop() {
  if (motor.distanceToGo() == 0){
    //Serial.print("Moving to ");
    //Serial.println(-motor.currentPosition());
    motor.moveTo(-motor.currentPosition());
  }
  motor.run();
}
