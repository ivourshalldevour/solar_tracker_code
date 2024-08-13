#include <Wire.h>
#include "mutex.hpp"

/*
    This code is for Arduino 3 (datalogger). It outputs on mutex3 (pinPC3/A3)
    and listens on mutex1 (pinPB1/9) and mutex2 (pinPB0/8). To simulate an I2C
    transmission it will instead broadcast to serial port. This code is meant
    to test multi-master I2C communication arbitered via 3 extra GPIO pins.
*/
void setup() {
    Serial.begin(9600);
    // Set pins PB1 and PB0 as inputs. (without pullups)
    DDRB = DDRB & (!((1<<DDB1) /*| (1<<DDB0)*/));

    // Set pin PC3 as output.
    PORTC = PORTC & !(1<<PORTC3);   // set to output LOW
    DDRC = DDRC | (1<<DDC3);   // then set as output (avoids accidental HIGH)
}


void loop() {
    byte status = 0;
    status = startMutex();
    if(status) {
        Serial.println("Failed to arbitrate.");
    }
    else{
        Serial.println("I2C comms going ahead...");
        delay(1000);
        endMutex();
    }
    delay(2000);
}
