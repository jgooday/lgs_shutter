#include "Arduino.h"
#include "AccelStepper.h"

void enable_motor();
void disable_motor();

int dir_pin = 13;
int step_pin = 7;
int enable_pin = 12;
int mc_pin = 8;
int sc_pin = 3;

bool motor_enabled = false;

//AccelStepper motor(1, step_pin, dir_pin);

volatile uint8_t portbhistory = 0xFF;     // default is high because the pull-up

void setup() {
  Serial.begin(9600);

  // Enable pin change interrupt on the PCINT18 pin using Pin Change Mask Register 2 (PCMSK2)
  PCMSK2 |= _BV(PCINT18);

  // Enable pin change interrupt 2 using the Pin Change Interrrupt Control Register (PCICR)
  PCICR |= _BV(PCIE2);

  //motor.setMaxSpeed(2100);
  //motor.setAcceleration(1500);
  //motor.moveTo(5000);

  analogWrite(8, 255);

}

void loop() {
  //if (motor.distanceToGo() == 0){
  //  motor.moveTo(-motor.currentPosition());
  //}
  //motor.run();
  //Serial.println(motor_enabled);
}


ISR(PCINT2_vect)
{
  //digitalWrite()
    //// Read PD2 using the Port D Pin Input Register (PIND)
    //if (PIND & _BV(PIND2))
    //{
    //    // PD2 is high, so button is released

    //    // Set PB5 low using the Port B Data Register (PORTB)
    //    PORTB &= ~_BV(PORTB5);
    //}
    //else
    //{
    //    // PD2 is low, so button is pressed

    //    // Set PB5 high using the Port B Data Register (PORTB)
    //    PORTB |= _BV(PORTB5);
    //}
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
