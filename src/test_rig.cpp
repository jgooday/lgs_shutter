#include "Arduino.h"

ISR(PCINT2_vect) {
    // Read PD2 using the Port D Pin Input Register (PIND)
    if (PIND & _BV(PIND2)) {
        // PD2 is high
        // Set PB5 low using the Port B Data Register (PORTB)
        PORTB &= ~_BV(PORTB5);
    } else if (PIND & _BV(PIND3)) {
        // PD3 is high
        // Set PB5 low using the Port B Data Register (PORTB)
        PORTB &= ~_BV(PORTB5);
    } else {
        // PD2 and PD3 low
        // Set PB5 high using the Port B Data Register (PORTB)
        PORTB |= _BV(PORTB5);
    }
}

void setup() {
    Serial.begin(9600);

    // Configure inputs using the Data Direction Register D (DDRD)
    // DDRD: [PD7][PD6]...[PD2][PD1] <- a 1 represents output, 0 input
    // Therefore, clear bits that you want as input
    DDRD &= ~_BV(DDD2); // pin PD2 (arduino 2)
    DDRD &= ~_BV(DDD3); // pin PD3 (arduino 3)

    // Enable the pull-up resistor on PD2 using the Port D Data Register (PORTD)
    PORTD |= _BV(PORTD2);
    PORTD |= _BV(PORTD3);

    // Enable pin change interrupt on the PCINT18 pin using Pin Change Mask Register 2 (PCMSK2)
    PCMSK2 |= _BV(PCINT18); // PCINT on PD2
    PCMSK2 |= _BV(PCINT19); // PCINT on PD3

    // Enable pin change interrupt 2 using the Pin Change Interrrupt Control Register (PCICR)
    PCICR |= _BV(PCIE2);

    // Configure PB5 as an output using the Port B Data Direction Register (DDRB)
    DDRB |= _BV(DDB5);

    // Enable interrupts
    sei();

}

void loop() {
  //
}
