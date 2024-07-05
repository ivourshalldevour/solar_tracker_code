#ifndef MENU_FSM_H
#define MENU_FSM_H

#include <Arduino.h>
#include <Wire.h>
#include <hd44780.h>                       // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header
#include "ButtonInput.hpp"
#include "RtcMethods.hpp"
//#include "Lcd.h"


typedef enum {start, gps, local_time, date, lat, lon, digit_selector} FSM_State;

extern float latitude;
extern float longitude;

void menuFSM();
void digitSelector(byte digit_format);
void readLcdDigits(char digits[8], byte digit_format);
byte getMaxDigitCol(byte digit_format);
void menuPrintTime(byte time[7], byte format);
void convertDigitsToTime(byte time[3], byte digits[8], byte format);
char* floatToLcd(float x, char p[8], byte format);

#endif