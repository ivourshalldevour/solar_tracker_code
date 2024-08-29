#include <Wire.h>
#include <hd44780.h>                       // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header

#include "Mutex2.hpp"

hd44780_i2Cexp lcd; // global for the hd44780 LCD library


/*
    This code is for Arduino 2 (adaptive tracker). It outputs on mutex2
    (pinPB0/8) and listens on mutex2 (pinPB2/9) and mutex3 (pinPC3/A3). It
    will print to an LCD to test I2C communication. This code is meant to
    test multi-master I2C communication arbitered via 3 extra GPIO pins.
*/
void setup() {
    DDRB = DDRB & !(1<<DDB1);   // set mutex1 (PB1) as input.
    DDRC = DDRC & !(1<<DDC3);   // set mutex3 (PC3) as input.

    // set mutex 2 (PB0) as output.
    PORTB = PORTB & !(1<<PORTB0);   // set to output LOW
    DDRB = DDRB | (1<<DDB0);   // then set as output (avoids accidental HIGH)

    Serial.begin(9600);
    Wire.begin();
    byte status = lcd.begin(16, 2); // initialising lcd (also turns on backlight)
    if(status) {
        hd44780::fatalError(status);
    } // if there was an error, blink the lcd backlight.

}

void loop() {


}
