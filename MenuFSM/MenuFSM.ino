#include <Arduino.h>
#include <Wire.h>
#include <hd44780.h>                       // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header
#include <EEPROM.h>

#include "ButtonInput.hpp"  // uses CSB buttons
#include "RtcMethods.hpp"
#include "MenuFSM.hpp"

#define LCD_COLS 16
#define LCD_ROWS 2

hd44780_I2Cexp lcd(0x27); // declare lcd object: auto locate & auto config expander chip
    // we don't give it a specific I2C address because the library auto finds it.

//float latitude = -33.832;       // might make double in future (since trig functions use double)
//float longitude = 151.124;


void setup() {
    Serial.begin(9600); // open the serial port at 9600 bps:
    Wire.begin();       // join I2C bus as master

    // set required button pins (CSB) as inputs
    PORTB = PORTB & 0b11111110; // setting PB0 (pin8) as input without pullup.
    DDRB  = DDRB  & 0b11111110;
    PORTC = PORTC & 0b01101111; // setting PC7 & PC4 (pin7 and pin4)as inputs without pullups.
    DDRC  = DDRC  & 0b01101111;

    int status;
    status = lcd.begin(LCD_COLS, LCD_ROWS); // initialising lcd (also turns on backlight)
    if(status) {
        hd44780::fatalError(status);
    } // if there was an error, blink the lcd backlight.

    while(rtcCheckClock(RTC_ADDRESS)) {
        Serial.println("Bad clock.");
    }
}


void loop() {
    menuFSM();
}