#include <Wire.h>
#include <hd44780.h>                       // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header

#include "secondPage.h"

hd44780_I2Cexp lcd; // declare lcd object: auto locate & auto config expander chip

void setup() {
    Wire.begin();
    lcd.begin(16,2);
    //lcd.print("Hello World");
    badFunc();
}

void loop() {
    
}
