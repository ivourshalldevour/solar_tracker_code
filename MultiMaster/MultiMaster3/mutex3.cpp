#include <Arduino.h>

/*
    This code file is only to be used by Arduino 3 (the datalogger).
*/

byte startMutex() {
    // Try to broadcast
    byte time_start = millis() / 1000;
    while(1) {     // only attempts to do this for one second
        if( (millis()/1000) == time_start) {
            if(PINB & ((1<<PINB1) | (1<<PORTB0))) { // while mutex1&2 are HIGH
                continue;
            }
            else {  // if mutex 1&2 are LOW
                PORTC = PORTC | (1<<PORTC3);    // output HIGH on mutex3
                break;
            }
        }
        else return 1;  // timed out while trying to arbitrate
    }
    // check mutex1&2 one more time (incase another arduino transmits at the same time)
    delay(5);  // Arduino3 has 3rd (last) priority hence the delay.
    if(PINB & (1<<PINB1) & (1<<PORTB0)) { // if they are HIGH
        PORTC = PORTC & 0b11110111; // PC3 set to LOW
        return 1;   // failed to arbitrate
    }
    else return 0;  // succesfully arbitrated for mutex.
}

void endMutex() {
    // output LOW on pin PC3
    PORTC = PORTC & 0b11110111;
}



