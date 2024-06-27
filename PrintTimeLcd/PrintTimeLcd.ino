#include <Wire.h>
#include <hd44780.h>                       // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header

#include "RtcReadout.hpp"

#define LCD_COLS 16
#define LCD_ROWS 2

hd44780_I2Cexp lcd; // declare lcd object: auto locate & auto config expander chip
    // we don't give it a specific I2C address because the library auto finds it.

void setup() {
    int status;

    Serial.begin(9600); // open the serial port at 9600 bps:
    Wire.begin();       // join I2C bus as master

    status = lcd.begin(LCD_COLS, LCD_ROWS); // initialising lcd (also turns on backlight)
    if(status) {
        hd44780::fatalError(status);
    } // if there was an error, blink the lcd backlight.
    lcd.print("Time (24hrs)");

     // checking if RTC is good. (Must clear it's OS flag).
    while(rtcCheckClock(RTC_ADDRESS)) {
        Serial.println("Bad clock.");
    }
}

void loop() {
    int status;
    unsigned int secs;
    static unsigned int lastsecs = 69;  // initializing to non-zero value
    byte rtc_time[7];
    char string[17];    // only 16 chars in each lcd row (+1 for null character)

    secs = millis() / 1000;
    // see if 1 second has passed
    if(secs != lastsecs) {
        lastsecs = secs;    // keep track of the last second value.

        // get time from rtc
        Wire.beginTransmission(RTC_ADDRESS);    // Get the slave's attention, tell it we're sending a command byte
        Wire.write(0x3);                        // The command byte, sets pointer to register with address of 0x3
        Wire.requestFrom(RTC_ADDRESS,7);        // Tell slave we need to read 7bytes begining at the current register
        int i = 0;
        while(Wire.available() != 0 && (i < 7)) {   // making sure that there are still bytes to read from I2C
            rtc_time[i] = Wire.read();              // read that byte into 'rtc_time' array
            i++;
        }
        Wire.endTransmission();
        rtcConvertTime(rtc_time);

        // print to lcd
        sprintf(string, "%02d:%02d:%02d ", rtc_time[2], rtc_time[1], rtc_time[0]);
        status = lcd.setCursor(0,1); // column 0, row 1 (second row)
        if(status) // non zero status means it was unsuccesful
		{
			hd44780::fatalError(status); // does not return
		}
        lcd.print(string);
    }
}
