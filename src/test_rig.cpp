/*
  Microcontroller code for Jack Gooday's
  Laser Guide Star Facility (LGSF) shutter
*/

#include "Arduino.h"
#include "AccelStepper.h"

// Functions
void update_operation();
void read_state();
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
int motor_max_speed = 1500;
int motor_max_accel = 1500;

// System state
bool SC;
bool MC;
bool SO;
bool CMD;
bool motor_enabled;
bool FAULT;

void setup() {
    Serial.begin(9600);

    ////////////////////////////////////////////////////////////////
    // Interrupts

    // Configure inputs using the Data Direction Register D (DDRD)
    // DDRD: [PD7][PD6]...[PD2][PD1] <- a 1 represents output, 0 input
    // Therefore, clear bits that you want as input
    DDRC &= ~_BV(DDC0); // pin PC0 (arduino A0)
    DDRC &= ~_BV(DDC1); // pin PC1 (arduino A1)
    DDRC &= ~_BV(DDC2); // pin PC2 (arduino A2)
    DDRC &= ~_BV(DDC3); // pin PC3 (arduino A3)

    // Enable pull-up resistors using the Port D Data Register (PORTD)
    PORTC |= _BV(PORTC0);
    PORTC |= _BV(PORTC1);
    PORTC |= _BV(PORTC2);
    PORTC |= _BV(PORTC3);

    // Enable pin change interrupts using Pin Change Mask Register 2 (PCMSK2)
    PCMSK2 |= _BV(PCINT8); // PCINT on PC0
    PCMSK2 |= _BV(PCINT9); // PCINT on PC1
    PCMSK2 |= _BV(PCINT10); // PCINT on PC2
    PCMSK2 |= _BV(PCINT11); // PCINT on PC3

    // Enable pin change interrupt 2 using the Pin Change Interrrupt Control Register (PCICR)
    PCICR |= _BV(PCIE2);

    // Enable interrupts
    sei();

    ////////////////////////////////////////////////////////////////

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
    //if (motor.distanceToGo() == 0)
      // Have the motor continuously move
      //motor.moveTo(motor.currentPosition()*2);
    motor.run();
  }
}

ISR(PCINT2_vect) {
  // Handles all interrupts on port D
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

  Serial.println("end update");

}

void read_state(){
  // Reads the state of inputs/limit switches
  SC = PIND & _BV(PIND2);
  MC = PIND & _BV(PIND3);
  SO = PIND & _BV(PIND4);
  CMD = PIND & _BV(PIND5);
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
