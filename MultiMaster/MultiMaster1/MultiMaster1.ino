#include <Wire.h>
#include <hd44780.h>                       // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header

#include "Mutex1.hpp"

hd44780_I2Cexp lcd; // global for the hd44780 LCD library.

/*
    This code is for Arduino 1 (traditional tracker). It outputs on mutex1
    (pinPB1/9) and listens on mutex2 (pinPB0/8) and mutex3 (pinPC3/A3). It
    will print to and LCD to test the I2C communication. This code is meant to
    test multi-master I2C communication arbitered via 3 extra GPIO pins.
*/
void setup() {
    DDRB = DDRB & !(1<<DDB0);   // set mutex2 (PB0) as input.
    DDRC = DDRC & !(1<<DDC3);   // set mutex3 (PC3) as input.

    // set mutex 1 (PB1) as output.
    PORTB = PORTB & !(1<<PORTB1);   // set to output LOW
    DDRB = DDRB | (1<<DDB1);   // then set as output (avoids accidental HIGH)

    Serial.begin(9600);
    Wire.begin();
    byte status = lcd.begin(16, 2); // initialising lcd (also turns on backlight)
    if(status) {
        hd44780::fatalError(status);
    } // if there was an error, blink the lcd backlight.
}

void loop() {
    if(startMutex()) {
        Serial.println("Failed to arbitrate.");
    }
    else{
        lcd.backlight();
        lcd.print("I2C going for it");
    }
    delay(40);
    endMutex();
    lcd.clear();
    lcd.noBacklight();
    delay(2000);
}
