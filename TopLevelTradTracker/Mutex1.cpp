#include <Arduino.h>

/*
    This code file is only to be used by Arduino 1 (the traditional tracker.)
*/

/* OLD startMutex() has a timeout
byte startMutex() {
    // Try to broadcast
    byte time_start = millis() / 1000;
    while(1) {     // only attempts to do this for one second
        if( (millis()/1000) == time_start) {
            if((PINB & (1<<PINB0)) | (PINC & (1<<PINC3))) { // while mutex2 or 3 is HIGH
                Serial.println("Mutex2or3 high.");
                continue;
            }
            else {  // if mutex 2&3 are LOW
                PORTB = PORTB | (1<<PORTB1);    // output HIGH on mutex1
                Serial.println("Output high on mutex1.");
                break;
            }
        }
        else return 1;  // timed out while trying to arbitrate
    }
    // Arduino 1 has top priority amongst the I2C masters.
    // Therefore mutex2&3 aren't checked again.
    return 0;   // success!
}
*/

// this version of startMutex has no timeout period.
byte startMutex() {
    // Try to broadcast
    while(1) {  // will keep trying forever.
        if((PINB & (1<<PINB0)) | (PINC & (1<<PINC3))) { // while mutex2 or 3 is HIGH
            Serial.println("Mutex2or3 high.");
        }
        else {  // if mutex 2&3 are LOW
            PORTB = PORTB | (1<<PORTB1);    // output HIGH on mutex1
            Serial.println("Output high on mutex1.");
            break;
        }
    }
    // Arduino 1 has top priority amongst the I2C masters.
    // Therefore mutex2&3 aren't checked again.
    return 0;   // success!
}

void endMutex() {
    // output LOW on pin PC3
    PORTB &=  !(1<<PORTB1);
}



