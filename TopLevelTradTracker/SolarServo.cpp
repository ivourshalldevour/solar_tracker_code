#include "SolarServo.hpp"

extern volatile int8_t current_angle;   // stores current position of declination servo.

// ISR for generating servo pulses on pins 5&6 using timer1
ISR(TIMER1_COMPA_vect) {
    //cli();  // make sure no other interrupts can override this one.
    TCCR1B = TCCR1B & 0b11111000;   // stop counter. It will be cleared when starting timer next time.
    PORTD = PORTD & 0b10011111;     // Set pins 5&6 to output 0
    //sei();
}


// setting up timer1 to use as Pulse generator for servos on pins 5 & 6.
void setupServoTimer() {
    // Set pins 5 & 6 as outputs.
    DDRD = DDRD | 0b01100000; 
    
    cli();
    TCCR1A = 0; // COM1A1/COM1A0 both zero. Therefore OC1A pin disconnected.

    // Must be in CTC mode OCR1A is TOP (mode 4 in table 16-4)
    TCCR1B = (1 << WGM12);

    // In TIMSK1 register
    TIMSK1 = (1<<OCIE1A);   // enable OCR1A output compare to generate interrupt
    sei();

    // no need to modify TIFR1 register at any point (even in ISR)
        // OCF1A is set when TCNT1 reaches OCR1A.
        // since we are triggering an interrupt, OCF1A is automatically cleared.
}

void commandServo(byte servo_num, int8_t angle) {
    // generating one-shot pulse
    cli(); // already done by default by ATMEGA328P processor
    OCR1A = 24000 + angle*177;  // signed integer multiplication
    TCNT1 = 0;  // clear the counter
    if(servo_num==1) {
        PORTD = PORTD | 0b01000000; // Output a HIGH on pin 6.
    }
    else {
        PORTD = PORTD | 0b00100000; // Output a HIGH in pin 5.
    }
    TCCR1B = TCCR1B | (1<<CS10);    // start counter with prescaler of 1.
    sei(); // already done by default by ATMEGA328P processor
}

void slowServo(byte servo_num, int8_t target_angle) {
    // Determine direction and steps
    int8_t step = (target_angle > current_angle) ? 1 : -1;
    for (int8_t angle = current_angle; angle != target_angle; angle += step) {
        // Generate one-shot pulse
        cli(); // Disable interrupts
        OCR1A = 24000 + angle * 177; // Set pulse width based on angle
        TCNT1 = 0; // Clear the counter
        
        // Set the appropriate pin high
        if (servo_num == 1) {
            PORTD |= 0b01000000; // Output a HIGH on pin 6
        } else {
            PORTD |= 0b00100000; // Output a HIGH on pin 5
        }
        
        TCCR1B |= (1 << CS10); // Start the timer
        sei(); // Enable interrupts

        // Wait for the pulse to complete
        delay(80);
    }
    // once reached target angle
    current_angle = target_angle;
}

// IDEA!!!
    // Use OCR1A to generate pulses for declination servo
    // Use OCR1B to generate pulses for hour angle servo.
    // Two sepparate interrupts. One is triggered by OCR1B. Set hour angle pin to output 0.
    // The other interrupt is triggered by OCR1A. Set declination pin to output 0.
    // Unfortunately this means we have two interrupts doign ALMOST the exact same thing.
    // Perhaps it uses less memory to use just one interrupt that sets both pins to zero.


