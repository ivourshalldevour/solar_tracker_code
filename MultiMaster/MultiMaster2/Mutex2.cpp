#include <Arduino.h>

/*
    This code file is only to be used by Arduino 1 (the traditional tracker.)
*/

byte startMutex() {
    // Try to broadcast
    byte time_start = millis() / 1000;
    while(1) {     // only attempts to do this for one second
        if( (millis()/1000) == time_start) {
            if((PINB & (1<<PINB0)) | (PINC & (1<<PINC3))) { // while mutex2 or 3 is HIGH
                continue;
            }
            else {  // if mutex 2&3 are LOW
                PORTB = PORTB | (1<<PORTB1);    // output HIGH on mutex1
                break;
            }
        }
        else return 1;  // timed out while trying to arbitrate
    }
    // check mutex1&2 one more time (incase another arduino transmits at the same time)
    delay(5);   // Arduino2 has 2nd priority hence the delay.
    if(PINB & (1<<PINB1) & (1<<PORTB0)) { // if they are HIGH
        PORTC = PORTC & 0b11110111; // PC3 set to LOW
        return 1;   // failed to arbitrate
    }
    else return 0;  // succesfully arbitrated for mutex.
}

void endMutex() {
    // output LOW on pin PC3
    PORTB &=  !(1<<PORTB1);
}



