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
    // Arduino 1 has top priority amongst the I2C masters.
    // Therefore mutex2&3 aren't checked again.
    return 0;   // success!
}

void endMutex() {
    // output LOW on pin PC3
    PORTB &=  !(1<<PORTB1);
}



