/*
  Device testing code for Jack Gooday's
  Laser Guide Star Facility (LGSF) shutter
  To be deployed to a seperate micro which
  controls the CMD pin (acts as an imaginary
  facility control system)
*/

#include "Arduino.h"

int command_pin = 9;
int max_open_time = 5000;  // Max open time of the shutter (ms)
int max_close_time = 2000; // Max close time of the shutter (ms)
int completed_cycles = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("setup finished");
}

void loop() {
  // Open the shutter
  //digitalWrite(command_pin, HIGH);
  analogWrite(command_pin, 255);
  delay(max_open_time);
  //digitalWrite(command_pin, LOW);
  analogWrite(command_pin, 0);
  delay(max_close_time);
  completed_cycles++;
  Serial.println(completed_cycles);
}
