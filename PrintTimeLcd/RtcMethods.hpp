#ifndef RTC_READOUT_H
#define RTC_READOUT_H

#include <Arduino.h>
#include <Wire.h>

#define RTC_ADDRESS 0b1101000   // for PCF8523 chip


byte rtcCheckClock(int address);

void rtcGetTime(byte time[7], int rtc_address);

void rtcConvertTime(byte *time);

void rtcWriteTime(byte time[3], int rtc_address);

void rtcWriteDate(byte time[3], int rtc_address);

#endif
