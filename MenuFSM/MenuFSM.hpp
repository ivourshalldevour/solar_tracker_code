#ifndef MENU_FSM_H
#define MENU_FSM_H

#include <Arduino.h>

#define LAT_EEPROM_ADDRESS 0    // spaced by 4 bytes since latitude and longitude are
#define LON_EEPROM_ADDRESS 4    // floats which take up 4bytes each.

// typedef enum state; is not defined here in the headder file so that it
// cannot be accessed outside the MenuFSM code.

// This function uses 3 statics (current_state, next_state, digit_format)
// Might change later to be passed as arguments, to save memory.
void menuFSM();

void digitSelector(byte digit_format);
void readLcdDigits(char digits[8], byte digit_format);
byte getMaxDigitCol(byte digit_format);
void menuPrintTime(byte time[7], byte format);
void convertDigitsToTime(byte time[3], byte digits[8], byte format);
char* floatToLcd(float x, char p[8], byte format);

#endif
