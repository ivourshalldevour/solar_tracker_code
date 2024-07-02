#include <Wire.h>
#include <stdio.h>

#include "RtcMethods.hpp"

#define RTC_ADDRESS 0b1101000   // for PCF8523 chip

void setup() {
    Serial.begin(9600); // open the serial port at 9600 bps:
    Wire.begin();       // join I2C bus as master

    /*
    // check if RTC is in battery switchover mode.
    // if not,  put it in battery switchover mode.
    Wire.beginTransmission(RTC_ADDRESS);
    Wire.write(0x2); // address of control register 3
    Wire.requestFrom(RTC_ADDRESS,1); // read only 1 byte (only control register 3)
    byte ctrl_reg3 = Wire.read();
    Wire.endTransmission();
    Serial.println(ctrl_reg3, BIN);
    if((ctrl_reg3 & 0b11100000) != 0) { // if not in battery switchover
        Serial.println("changing ctrl_reg3.");
        Wire.beginTransmission(RTC_ADDRESS);
        Wire.write(0x2);    //address
        Wire.write(ctrl_reg3 & 0b00011111);       // value at that address
        // ctrl_reg3[7:5] becomes 0b101    battery switchover enabled
        Wire.endTransmission();
    }
    Wire.beginTransmission(RTC_ADDRESS);
    Wire.write(0x2); // address of control register 3
    Wire.requestFrom(RTC_ADDRESS,1); // read only 1 byte (only control register 3)
    ctrl_reg3 = Wire.read();
    Wire.endTransmission();
    Serial.println(ctrl_reg3, BIN);
    */

    // check clock integrity. (Must clear it's OS flag).
    while(rtcCheckClock(RTC_ADDRESS)) {
        Serial.println("Bad clock.");
    }

    // writing a new time to the rtc
    //byte time[7] = {30, 12, 18, 02, 2, 07, 24};
    //rtcWriteTime(time, RTC_ADDRESS);
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



