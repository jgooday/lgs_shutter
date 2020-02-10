/*
  Device testing code for Jack Gooday's
  Laser Guide Star Facility (LGSF) shutter
  To be deployed to a seperate micro which
  controls the CMD pin (acts as an imaginary
  facility control system)
*/

#include "Arduino.h"

void update_state();

int command_pin = 9;
int fault_pin = 3;         // INT1
int max_open_time = 5000;  // Max open time of the shutter (ms)
int max_close_time = 1000; // Max close time of the shutter (ms)

unsigned long start_time;
unsigned long current_time;
bool CMD;
bool FAULT;
int completed_cycles = 0;

void setup() {
  // Start serial communication
  Serial.begin(9600);

  // Set pin modes, and connect an interrupt to fault
  // This will call update_state whenever fault changes
  pinMode(command_pin, OUTPUT);
  pinMode(fault_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(fault_pin), update_state, CHANGE);

  // Print headers for data
  Serial.println("TIME, CMD, FAULT, CYCLE");

  // Set start time
  start_time = millis();
}

void loop() {
  // Open the shutter
  CMD = true;             // Change signal
  update_state();         // Write signal, among other things
  delay(max_open_time);   // Wait for it to open

  // Close the shutter
  CMD = false;
  update_state();
  delay(max_close_time);

  completed_cycles++;
}

void update_state() {
  // Read the fault pin
  FAULT = digitalRead(fault_pin);

  // Write the command pin
  // analogWrite used as it gives a higher
  // voltage for some reason
  if (CMD) {
    analogWrite(command_pin, 255);
  } else {
    analogWrite(command_pin, 0);
  }

  // Update the current time
  current_time = millis() - start_time;

  // Write state to console
  Serial.print(current_time);
  Serial.print(", ");
  Serial.print(CMD);
  Serial.print(", ");
  Serial.print(FAULT);
  Serial.print(", ");
  Serial.println(completed_cycles);
}
