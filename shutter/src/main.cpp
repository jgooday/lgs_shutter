/*
  Microcontroller code for Jack Gooday's
  Laser Guide Star Facility (LGSF) shutter
*/

#include "Arduino.h"
#include "AccelStepper.h"

// Functions
void update_operation();
void read_state();
void print_state();
void enable_motor();
void disable_motor();
void motor_forwards();
void motor_reverse();
void activate_magnet();
void deactivate_magnet();

// Output pins
int dir_pin = 2;
int step_pin = 3;
int enable_pin = 4;
int magnet_pin = 5;
int fault_indicator_pin = 6;
int enable_indicator_pin = 9;

// Pin mapping for limit switches
// CMD -> PC0
// SC -> PC1
// MC -> PC2
// SO -> PC4

// Motor object definition
AccelStepper motor(1, step_pin, dir_pin);
int motor_max_speed = 1000;
int motor_max_accel = 1000;

// System state
bool SC;
bool MC;
bool SO;
bool CMD;
bool motor_enabled;
bool FAULT;

volatile int value = 0;

void setup() {
  Serial.begin(9600);

  cli();
  PCICR |= 0b00000010;  // Enables Ports C Pin Change Interrupts
  PCMSK1 |= 0b00001111; // PCINT8, 9, 10, 11 (PC0, 1, 2, 3)
  sei();

  // Output pins
  pinMode(magnet_pin, OUTPUT);
  pinMode(fault_indicator_pin, OUTPUT);
  pinMode(enable_indicator_pin, OUTPUT);

  // Set up motor
  motor.setMaxSpeed(motor_max_speed);
  motor.setAcceleration(motor_max_accel);
  disable_motor();

  // Initialise operative state
  update_operation();
}

void loop() {
  if (motor_enabled){
    motor.run();
  }
}

ISR(PCINT1_vect) {
  // Handles all interrupts on port C
  // Directs straight to update the operational
  // state of the device
  update_operation();
}

void update_operation() {
  // Reads the state of all inputs, then runs
  // the logic derived using the truth table
  // to operate the motor and magnet
  //
  // Note: as there is currently no MO limit
  // switch, it's been left out of logic statements

  // Read state of inputs
  read_state();

  print_state();

  // Motor operation
  if ((!CMD & !MC) || (SO & !MC) || (CMD & !SO)) {// & !MO)) {
    // Command CLOSE & motor isn't closed
    // Shutter open & motor isn't closed
    // Command OPEN & shutter [and motor] not open
    //  -> The motor should move
    enable_motor();

    if (CMD & !SO) {// & !MO) {
      // Command OPEN & shutter [and motor] not open
      //  -> The motor should move forwards
      motor_forwards();
    } else {
      // Command CLOSE & motor isn't closed OR
      // Shutter open & motor isn't closed
      //  -> The motor should move backwards
      motor_reverse();
    }

  } else {
    // The motor should be disabled
    disable_motor();
  }

  // Magnet operation
  if (CMD) {
    activate_magnet();
  } else {
    deactivate_magnet();
  }

  // Fault detection
  FAULT = !(                    // Note inversion
    (!CMD & !SO & SC & MC) ||   // command CLOSE, not SO, SC and MC
    (CMD & SO & !SC & MC)       // command OPEN, SO and MC, not SC
  );                            //  -> No fault (hence inversion)

  digitalWrite(enable_indicator_pin, motor_enabled);
  digitalWrite(fault_indicator_pin, FAULT);

}

void read_state(){
  // Reads the state of inputs/limit switches
  // Refer to limit switch mapping at start of code
  CMD = PINC & _BV(PINC0);
  SC = PINC & _BV(PINC1);
  MC = PINC & _BV(PINC2);
  SO = PINC & _BV(PINC3);
}

void print_state(){
  Serial.print(CMD);
  Serial.print(", ");
  Serial.print(SC);
  Serial.print(", ");
  Serial.print(MC);
  Serial.print(", ");
  Serial.println(SO);
}

void enable_motor(){
  // Enables the motor (logical low to driver)
  analogWrite(enable_pin, 0);
  motor_enabled = true;
}

void disable_motor(){
  // Disables the motor (logical high to driver)
  analogWrite(enable_pin, 255);
  motor_enabled = false;
}

void motor_forwards(){
  // Reset the position of the motor, then set it
  // a new large target
  motor.setCurrentPosition(0);
  motor.moveTo(-50000);
}

void motor_reverse(){
  // Reset the position of the motor, then set it
  // a new large (negative) target
  motor.setCurrentPosition(0);
  motor.moveTo(50000);
}

void activate_magnet(){
  digitalWrite(magnet_pin, HIGH);
}

void deactivate_magnet(){
  digitalWrite(magnet_pin, LOW);
}
