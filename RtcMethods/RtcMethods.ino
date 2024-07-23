#include <Wire.h>
#include <stdio.h>

#include "RtcMethods.hpp"

#define RTC_ADDRESS 0b1101000   // for PCF8523 chip

byte rtc_interrupt;

void setup() {
    Serial.begin(9600); // open the serial port at 9600 bps:
    Wire.begin();       // join I2C bus as master

    // check clock integrity. (Must clear it's OS flag).
    while(rtcCheckClock(RTC_ADDRESS)) {
        Serial.println("Bad clock.");
    }
}

void loop() {
    byte rtc_time[7];

    rtcGetTime(rtc_time, RTC_ADDRESS);
    rtcConvertTime(rtc_time);   // converts from BCD encoding to normal binary.

    char string[100];
    sprintf(string, "Year:%d Month:%d Weekday:%d Day:%d hms: %02d:%02d:%02d", rtc_time[6], rtc_time[5], rtc_time[4], rtc_time[3], rtc_time[2], rtc_time[1], rtc_time[0]);
    Serial.println(string);
    delay(2000);
}



