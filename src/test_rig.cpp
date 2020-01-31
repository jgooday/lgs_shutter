#include "Arduino.h"
#include "AccelStepper.h"

void enable_motor();
void disable_motor();

int dir_pin = 8;
int step_pin = 9;
int enable_pin = 10;

AccelStepper motor(1, step_pin, dir_pin);
bool motor_enabled = false;

void setup() {
    Serial.begin(9600);

    ////////////////////////////////////////////////////////////////
    // Interrupts

    // Configure inputs using the Data Direction Register D (DDRD)
    // DDRD: [PD7][PD6]...[PD2][PD1] <- a 1 represents output, 0 input
    // Therefore, clear bits that you want as input
    DDRD &= ~_BV(DDD2); // pin PD2 (arduino 2)
    DDRD &= ~_BV(DDD3); // pin PD3 (arduino 3)
    DDRD &= ~_BV(DDD4); // pin PD4 (arduino 4)

    // Enable pull-up resistors using the Port D Data Register (PORTD)
    PORTD |= _BV(PORTD2);
    PORTD |= _BV(PORTD3);
    PORTD |= _BV(PORTD4);

    // Enable pin change interrupts using Pin Change Mask Register 2 (PCMSK2)
    PCMSK2 |= _BV(PCINT18); // PCINT on PD2
    PCMSK2 |= _BV(PCINT19); // PCINT on PD3
    PCMSK2 |= _BV(PCINT20); // PCINT on PD4

    // Enable pin change interrupt 2 using the Pin Change Interrrupt Control Register (PCICR)
    PCICR |= _BV(PCIE2);

    // Configure PB5 as an output using the Port B Data Direction Register (DDRB)
    DDRB |= _BV(DDB5);

    // Enable interrupts
    sei();

    ////////////////////////////////////////////////////////////////

    motor.setMaxSpeed(2100);
    motor.setAcceleration(1500);
    motor.moveTo(5000);
    enable_motor();

}

void loop() {
  if (motor_enabled){
    if (motor.distanceToGo() == 0)
      motor.moveTo(-motor.currentPosition());
    motor.run();
  }
}

ISR(PCINT2_vect) {
  // Handles interrupts on PORTD. Checks each pin,
  // starting from PD2 through to PD4. If it finds
  // one that is on, it turns on an LED

  // Read PD2 using the Port D Pin Input Register (PIND)
  if (PIND & _BV(PIND2)) {
      // PD2 is high
      // Set PB5 low using the Port B Data Register (PORTB)
      PORTB &= ~_BV(PORTB5);
  } else if (PIND & _BV(PIND3)) {
      // PD3 is high
      // Set PB5 low using the Port B Data Register (PORTB)
      PORTB &= ~_BV(PORTB5);
  } else if (PIND & _BV(PIND4)) {
        // PD4 is high
        // Enable the motor
        disable_motor();
  } else {
      // PD2 and PD3 low
      // Set PB5 high using the Port B Data Register (PORTB)
      PORTB |= _BV(PORTB5);
  }

}

void enable_motor(){
  // Enable is logical low
  analogWrite(enable_pin, 0);
  motor_enabled = true;
}

void disable_motor(){
  // Enable is logical low
  analogWrite(enable_pin, 255);
  motor_enabled = false;
}








//#include "Arduino.h"
//#include "AccelStepper.h"
//
//void open_motor();
//void close_motor();
//void refresh_state();
//void print_state();
//void stop_motor();
//
//int dir_pin = 10;
//int step_pin = 11;
//int enable_pin = 12;
//int so_pin = 3;
//int mc_pin = 4;
//int sc_pin = 5;
//
//AccelStepper motor(1, 11, 10);
//
//volatile bool SO = false;
//volatile bool SC = false;
//volatile bool MC = false;
//volatile bool CMD = true; //false for close, must raise high to open
//
//int drive_freq = 980*1.04;
//int half_period_us = 500000/drive_freq;  //milliseconds delay between high and //low states
//
//void setup() {
//  // Begin console interface
//  Serial.begin(9600);
//
//  // Set pin modes
//  pinMode(dir_pin, OUTPUT);
//  pinMode(step_pin, OUTPUT);
//  pinMode(enable_pin, OUTPUT);
//  pinMode(so_pin, INPUT);
//  pinMode(sc_pin, INPUT);
//  pinMode(mc_pin, INPUT);
//
//  // Set initial state for direction pin
//  //analogWrite(dir_pin, 255);
//  analogWrite(enable_pin, 255); //255 is disabled
//
//  //attachInterrupt(digitalPinToInterrupt(mc_pin), refresh_state, CHANGE);
//  //attachInterrupt(digitalPinToInterrupt(sc_pin), refresh_state, CHANGE);
//  //attachInterrupt(digitalPinToInterrupt(so_pin), refresh_state, CHANGE);
//
//  motor.setMaxSpeed(4000);
//  motor.setSpeed(0);
//
//  Serial.println("Setup complete");
//  delay(1000);
//  Serial.println("Entering loop");
//}
//
//void loop() {
//
//  // Refresh state
//  refresh_state();
//  //print_state();
//
//  // Decide what to do
//  if (CMD) {
//    // Command OPEN
//    if (!SO) {
//      // Shutter is not open. Open it!
//      open_motor();
//    } else if (!MC) {
//      // Shutter is open, but motor is not closed
//      // Return the motor
//      close_motor();
//    } else {
//      //Serial.println("STABLE OPEN");
//      //analogWrite(enable_pin, 255);
//      stop_motor();
//    }
//  } else {
//    // Command CLOSE
//    Serial.println("Deactivate EM");
//  }
//
//  motor.runSpeed();
//}
//
//void open_motor(){
//  Serial.println("Opening shutter");
//  //analogWrite(dir_pin, 255);
//  //analogWrite(enable_pin, 0);
//  //step_motor();
//  //analogWrite(enable_pin, 0);
//  motor.setSpeed(-1000);
//}
//
//void close_motor(){
//  Serial.println("Returning motor");
//  //analogWrite(dir_pin, 0);
//  //analogWrite(enable_pin, 0);
//  //step_motor();
//  //analogWrite(enable_pin, 0);
//  motor.setSpeed(1000);
//}
//
//void stop_motor(){
//  //Serial.println("Stop motor");
//  //refresh_state();
//  analogWrite(enable_pin, 255);
//  motor.stop();
//}
//
//void refresh_state(){
//  SO = digitalRead(so_pin);
//  SC = digitalRead(sc_pin);
//  MC = digitalRead(mc_pin);
//}
//
//void print_state(){
//  Serial.print("State (CMD, SO SC, MC): ");
//  Serial.print(CMD);
//  Serial.print(" ");
//  Serial.print(SO);
//  Serial.print(" ");
//  Serial.print(SC);
//  Serial.print(" ");
//  Serial.println(MC);
//}



//void step_motor(){
//  //Serial.println("Opening shutter");
//  delayMicroseconds(half_period_us);
//  analogWrite(step_pin, 0);
//  delayMicroseconds(half_period_us);
//  analogWrite(step_pin, 255);
//``}
