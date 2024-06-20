#include <Wire.h>
#include <stdio.h>

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
    while(RtcCheckClock(RTC_ADDRESS)) {
        Serial.println("Bad clock.");
    }
}

void loop() {
    byte rtc_time_raw[7];   // some values in the RTC are in BCD
    byte rtc_time[7];       // this is the cleaned up version with normal binary encoding (can be used directly)

    Wire.beginTransmission(RTC_ADDRESS);    // Get the slave's attention, tell it we're sending a command byte
    Wire.write(0x3);                        // The command byte, sets pointer to register with address of 0x3
    Wire.requestFrom(RTC_ADDRESS,7);        // Tell slave we need to read 7bytes begining at the current register
    int i = 0;
    while(Wire.available() != 0 && (i < 7)) {   // making sure that there are still bytes to read from I2C
        rtc_time_raw[i] = Wire.read();          // read that byte into 'rtc_time_raw' array
        i++;
    }
    Wire.endTransmission();                 // "Hang up the line" so others can use it (can have multiple slaves & masters connected)

    RtcConvertDate(rtc_time_raw, rtc_time);
    char string[100];
    sprintf(string, "Year:%d Month:%d Weekday:%d Day:%d hms: %d:%d:%d", rtc_time[6], rtc_time[5], rtc_time[4], rtc_time[3], rtc_time[2], rtc_time[1], rtc_time[0]);
    Serial.println(string);
    delay(2000);
}

byte RtcCheckClock(int address) {
    // Assumes the PCF8523 chip is being used.
    // returns 1 if the OS flag is set. Therefore clock integrity cannot be guaranteed.
        // also attempts to clear the flag if this is the case
    // returns 0 if the flag was 0.
    Wire.beginTransmission(address);
    Wire.write(0x3);                // set to seconds register
    Wire.requestFrom(address,1);    // read the seconds register
    byte sec_reg = Wire.read();
    Wire.endTransmission();
    if(sec_reg & 0b10000000) {    // check if OS flag (bit 7) is set.
        Wire.beginTransmission(address);
        Wire.write(0x3);
        Wire.write(sec_reg & 0b01111111);   // attempt to clear the flag.
        Wire.endTransmission();
        return 1;                           // the OS flag was set.
    }
    else {
        return 0; // OS flag was not set.
    }
    Wire.endTransmission();
}

void RtcConvertDate(byte *time_raw, byte *time) {
    /*
    A function that takes raw time data from the PCF8523 RTC chip and converts it
    from BCD into normal binary encoding.
    Returns 1 if clock integrity is not guaranteed.

    Input:  pointer to a 7 element byte (a.k.a. unit8_t) array    in BCD
    Output: pointer to a 7 element byte array                     in binary

    Assumes OS flag is cleared. (therefore RTC clock integrity is guaranteed).
        Call RtcCheckClock() to clear this flag.
    Assumes hours are in 24hour format (can be changed in PCF8523 chip)
    Assumes time[0]=sec, time[1]=min, 2=hrs, 3=days, 4=weekdays,5=months,6=yrs
    */

    // convert seconds
    time[0] = (time_raw[0] >> 4)*10 + (time_raw[0] & 0b00001111);

    // convert minutes
    time[1] = (time_raw[1] >> 4)*10 + (time_raw[1] & 0b00001111);

    // convert hours
    time[2] = (time_raw[2] >> 4)*10 + (time_raw[2] & 0b00001111);

    // convert days
    time[3] = (time_raw[3] >> 4)*10 + (time_raw[3] & 0b00001111);

    // no conversion needed for weekday
    time[4] = time_raw[4];      // Sunday is 0, monday is 1 etc.

    // convert months
    time[5] = (time_raw[5] >> 4)*10 + (time_raw[5] & 0b00001111);

    //convert years
    time[6] = (time_raw[6] >> 4)*10 + (time_raw[6] & 0b00001111);
}
