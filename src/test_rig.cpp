#include "Arduino.h"
#include "AccelStepper.h"

void enable_motor();
void disable_motor();

// Motor driver pins
int dir_pin = 8;
int step_pin = 9;
int enable_pin = 10;

// Pin mapping for limit switches
// SC -> PD2
// MC -> PD3
// SO -> PD4

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
