#include "Arduino.h"
#include "AccelStepper.h"

int dir_pin = 8;
int step_pin = 7;
int enable_pin = 12;

AccelStepper motor(1, step_pin, dir_pin);

void setup() {
  Serial.begin(9600);

  motor.setMaxSpeed(2200);
  motor.setAcceleration(500);
  motor.moveTo(7000);
}

void loop() {
  if (motor.distanceToGo() == 0){
    motor.moveTo(-motor.currentPosition());
  }
  motor.run();
}
