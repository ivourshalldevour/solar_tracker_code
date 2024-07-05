#include <Wire.h>
#include <hd44780.h>                       // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header

#include "ButtonInput.hpp"  // uses CSB buttons
#include "RtcMethods.hpp"
#include "MenuFSM.hpp"
//#include "Lcd.h"

#define LCD_COLS 16
#define LCD_ROWS 2

//hd44780_I2Cexp lcd(0x27); // declare lcd object: auto locate & auto config expander chip
    // we don't give it a specific I2C address because the library auto finds it.

float latitude = -33.832;       // might make double in future (since trig functions use double)
float longitude = 151.124;


void setup() {
    int status;

    Serial.begin(9600); // open the serial port at 9600 bps:
    Wire.begin();       // join I2C bus as master

    status = lcd.begin(LCD_COLS, LCD_ROWS); // initialising lcd (also turns on backlight)
    if(status) {
        hd44780::fatalError(status);
    } // if there was an error, blink the lcd backlight.
}


void loop() {
    menuFSM();
}

