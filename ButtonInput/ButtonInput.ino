/*
    This code prints to an LCD via I2C how many
    times each button was pressed:
        Cycle   (c, on pin 8)
        Select  (s, on pin 7)
        Back    (b, on pin 4)
    The buttons are assumed to have external pull-up resistors.
    No de-bouncing is done in software.
    Ivo Urbanczyk 25 Jun 2024
*/

#include <Wire.h>
#include <hd44780.h>                       // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header

#include "ButtonInput.hpp"

#define LCD_COLS 16
#define LCD_ROWS 2

hd44780_I2Cexp lcd; // declare lcd object: auto locate & auto config expander chip
    // we don't give it a specific I2C address because the library auto finds it.

byte c_count = 0;  // no. times this button was pressed.
byte s_count = 0;
byte b_count = 0;

void setup() {
    int status;

    // set required button pins as inputs
    PORTB = PORTB & 0b11111110; // setting PB0 (pin8) as input without pullup.
    DDRB  = DDRB  & 0b11111110;
    PORTC = PORTC & 0b01101111; // setting PC7 & PC4 (pin7 and pin4)as inputs without pullups.
    DDRC  = DDRC  & 0b01101111;

    Serial.begin(9600); // open the serial port at 9600 bps:
    Wire.begin();       // join I2C bus as master

    status = lcd.begin(LCD_COLS, LCD_ROWS); // initialising lcd (also turns on backlight)
    if(status) {
        hd44780::fatalError(status);
    } // if there was an error, blink the lcd backlight.
    lcd.print("Button Pressed:");
    lcd.setCursor(0,1);
    lcd.print("c:   s:   b:   ");
}

void loop() {
    Button button = readButtons();
    switch(button) {
        case cycle: {
            lcd.setCursor(2,1);
            c_count++;
            lcd.print(c_count, DEC);
        } break;
        case select: {
            lcd.setCursor(7,1);
            s_count++;
            lcd.print(s_count, DEC);
        } break;
        case back: {
            lcd.setCursor(12,1);
            b_count++;
            lcd.print(b_count, DEC);
        }
    }
}

