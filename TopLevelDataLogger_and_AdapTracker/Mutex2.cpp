#include <Arduino.h>

/*
    This code file is only to be used by Arduino 2 (the adaptive tracker.)
*/

/* OLD startMutex() has a timeout.
byte startMutex() {
    // Try to broadcast
    byte time_start = millis() / 1000;
    while(1) {     // only attempts to do this for one second
        if( (millis()/1000) == time_start) {
            if((PINB & (1<<PINB1)) | (PINC & (1<<PINC3))) { // while mutex1 or 3 is HIGH
                continue;
            }
            else {  // if mutex 1&3 are LOW
                PORTB = PORTB | (1<<PORTB0);    // output HIGH on mutex2
                break;
            }
        }
        else return 1;  // timed out while trying to arbitrate
    }
    // check mutex1&3 one more time (incase another arduino transmits at the same time)
    delay(5);   // Arduino2 has 2nd priority hence the delay.
    if( (PINB & (1<<PINB1)) | (PINC & (1<<PINC3)) ) { // if they are HIGH
        PORTC = PORTB & 0b11111110; // PB0 (mutex2) set to LOW
        return 1;   // failed to arbitrate
    }
    else return 0;  // succesfully arbitrated for mutex.
}
*/

byte startMutex() {
    // Try to broadcast
    while(1) {     // keeps trying to do this forever
        if((PINB & (1<<PINB1)) | (PINC & (1<<PINC3))) { // while mutex1 or 3 is HIGH
            continue;
        }
        else {  // if mutex 1&3 are LOW
            PORTB = PORTB | (1<<PORTB0);    // output HIGH on mutex2
            break;
        }
    }
    // check mutex1&3 one more time (incase another arduino transmits at the same time)
    delay(5);   // Arduino2 has 2nd priority hence the delay.
    if( (PINB & (1<<PINB1)) | (PINC & (1<<PINC3)) ) { // if they are HIGH
        PORTC = PORTB & 0b11111110; // PB0 (mutex2) set to LOW
        return 1;   // failed to arbitrate
    }
    else return 0;  // succesfully arbitrated for mutex.
}

void endMutex() {
    // output LOW on pin PC3
    PORTB &=  !(1<<PORTB1);
}



