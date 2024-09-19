#ifndef RTC_READOUT_H
#define RTC_READOUT_H

#include <Arduino.h>
#include <Wire.h>

#define RTC_ADDRESS 0b1101000   // for PCF8523 chip

// ISR to wakeup the arduino when RTC alarm (or countdown timer) goes off.
ISR(EXT_INT0);

byte rtcCheckClock(int address);


/*
    Function that sets up the countdown Timer A and B on the PCF8523 real time
    clock to generate interrupts on its INT1 pin. This is an active LOW
    interrupt. The countdown timer B is set to count down the number of
    minutes given by intervalB. While timer A is set to count down the number
    of minutes given by intervalA
    Input:
        - intervalB: number of minutes between each interrupt from Timer B
        - intervalA: number of minutes between each interrupt from Timer A
        - If the interval is given as 0, that timer is disabled. 
    Output:
        - Writes to I2C to setup the RTCs registers to start the timer.
    Assumes:
        - The interrupt needs to be cleared via I2C in a different function
        to disable the interrupt.
*/
void rtcSetupCountdown(byte intervalA, byte intervalB, int rtc_address);

int julianDay(byte* time);


/*
    A function that gets the date and time from a PCF8523 Real Time Clock
    via I2C. It does not do any data conversion. Just gets the raw time.
    Inputs:
        - rtc_address      the I2C address of the Real Time Clock.
        - time     pointer to 7 element array to store time in.
    Outputs:
        - time is written with the retrieved date and time.
    Assumes:
        time[0]=sec, time[1]=min, 2=hrs, 3=days, 4=weekdays,5=months,6=yrs  (bascially little endian)
*/
void rtcGetTime(byte time[7], int rtc_address);


/*
    A function that takes raw time data from the PCF8523 RTC chip and converts it
    from BCD into normal binary encoding. Should be called after rtcGetTime().
    Returns 1 if clock integrity is not guaranteed.

    Input:  pointer to a 7 element byte (a.k.a. unit8_t) array    in BCD
    Output: edits that array directly.

    Assumes:
     - rtc's OS flag is cleared. (therefore RTC clock integrity is guaranteed).
        Call rtcCheckClock() to clear this flag.
     - hours are in 24hour format (can be changed in PCF8523 chip)
     - time[0]=sec, time[1]=min, 2=hrs, 3=days, 4=weekdays,5=months,6=yrs
     - for weekdays (sunday is 0, monday is 1, etc.)
*/
void rtcConvertTime(byte* time);


/* 
    Writes the local time (ss:mm:hh)  to an RTC on the
    PCF8523 chip. Using I2C.

    Inputs:
     - 3 element byte array (time[]) in BCD encoding
     - the I2C address for the Real Time Clock(RTC).
     Assumes:
         - each time[] element is in BCD encoding.
         - time[0]=sec, time[1]=min, 2=hrs  (bascially little endian)
         - Already joined I2C bus as master.  Wire.begin(); is done.
*/
void rtcWriteTime(byte time[3], int rtc_address);


/* 
    Writes the date (dd/mm/yy) to an RTC on the
    PCF8523 chip. Using I2C.

    Inputs:
     - 3 element byte array (time[]) in BCD encoding.
     - the I2C address for the Real Time Clock(RTC).
     Assumes:
         - each time[] element is in BCD encoding.
         - time[0]=day, time[1]=month, 2=year  (bascially little endian)
         - Weekday number (sunday, monday, etc) is not changed.
         - Already joined I2C bus as master.  Wire.begin(); is done.
*/
void rtcWriteDate(byte time[3], int rtc_address);

#endif
