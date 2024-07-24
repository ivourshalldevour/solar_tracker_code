#include "MenuFSM.hpp"
#include "ButtonInput.hpp"
#include "RtcMethods.hpp"

#include <Arduino.h>
#include <Wire.h>
#include <hd44780.h>                       // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header
#include <EEPROM.h>

extern hd44780_I2Cexp lcd; // defined in main .ino file.
typedef enum {start, gps, local_time, date, lat, lon, digit_selector} FSM_State;

void menuFSM() {
    /*
        This function uses busy waiting to read the button states (cycle,
        select, and back). All other microcontroller functions will be stopped
        once menuFSM is entered. The latitude, longitude, date and time are
        are edited via the LCD.

        Date and time values are stored on the RTC, so menuFSM writes directly
        to the RTC using I2C, once the values are edited. Latitude and
        longitude are stored in EEPROM.
    */
    Serial.println("Entered MenuFSM.");

    FSM_State current_state = start;
    FSM_State next_state = start; 

    // used for the digitSelector() function to know how to print to lcd.
    // also is convenient to determine what the state immediately prior was.
    byte digit_format;    // 0=lat 1=lon, 2=hh:mm:ss, 3=yy/mthmth/dd

    for(;;) {   // this loop is what does the busy waiting, and why program gets stuck in MenuFSM.

        Button button = readButtons(); // returns which button was pressed (assumes only 1 is pressed at a time).

        // Lcd is only cleared in the next state logic. This because we only want
        // the lcd to be cleared when a state transition occurs, and not every loop
        // iteration. Otherwise screen would flicker. (Maybe not, but i think so.)

        // next state logic
        switch(current_state) {
            case start:
                switch(button) {
                    // if button == cycle     do nothing
                    case select: {
                        next_state = gps;
                        lcd.clear();
                    } break;
                    case back: {
                        lcd.clear();
                        return; // exit menuFSM function.
                    }
                }
                break;
            case gps:
                switch(button) {
                    case cycle: {
                        next_state = local_time;
                        lcd.clear();
                    } break;
                    case select: { 
                        next_state = lat;
                        lcd.clear();
                    } break;
                    case back: {
                        next_state = start;
                        lcd.clear();
                    }
                }
                break;
            case local_time:
                switch(button) {
                    case cycle: {
                        next_state = date; 
                        lcd.clear();
                    } break;
                    case select: {
                        next_state = digit_selector;
                        digit_format = 2;
                    } break;
                    case back: {
                        next_state = start;
                        lcd.clear();
                    }
                }
                break;
            case date:
                switch(button) {
                    case cycle: { 
                        next_state = gps;
                        lcd.clear();
                    } break;
                    case select: {
                        next_state = digit_selector;
                        digit_format = 3;
                    } break;
                    case back: {
                        next_state = start;
                        lcd.clear();
                    }
                }
                break;
            case lat:
                switch(button) {
                    case cycle: {
                        next_state = lon;
                        lcd.clear();
                    } break;
                    case select: {
                        next_state = digit_selector;
                        digit_format = 0;
                    } break;
                    case back: {
                        next_state = gps;
                        lcd.clear();
                    }
                }
                break;
            case lon:
                switch(button) {
                    case cycle: {
                        next_state = lat;
                        lcd.clear();
                    } break;
                    case select: {
                        next_state = digit_selector;
                        digit_format = 1;
                    } break;
                    case back: {
                        next_state = gps;
                        lcd.clear();
                    }
                }
            // digit_selector case is actually handled in output logic.
        }
        current_state = next_state; 
        // end of next state logic

        // output logic
        switch(current_state) {
            case start: {
                lcd.home();     // these homing lines are required because the LCD is constantly being bombarded with write commands. Even when no state change occurs or no buttons are pressed.
                lcd.write("Main Menu");
            } break;
            case gps: {
                lcd.home();
                lcd.write("GPS Coordinates");
            } break;
            case local_time: {
                lcd.home();
                lcd.write("Local Time");
                // get time and print it on second row
                // this could be updated each second (looks cool)
                // or could just display time once, at entry to this state.
                byte time[7];
                rtcGetTime(time, RTC_ADDRESS);
                menuPrintTime(time, 0); // 0 for printing local time. 
            } break;
            case date: {
                lcd.home();
                lcd.write("Date (dd/mm/yy)");
                // get date and print it on second row
                byte time[7];
                rtcGetTime(time, RTC_ADDRESS);
                menuPrintTime(time, 1); // 1 for printing date.
            } break;
            case lat: {
                lcd.home();
                lcd.write("Latitude");
                lcd.setCursor(0,1);
                char str[8];
                float latitude;
                EEPROM.get(LAT_EEPROM_ADDRESS, latitude);
                floatToLcd(latitude, str, 0);    // 0 for latitude
                lcd.write(str);
            } break;
            case lon: {
                lcd.home();
                lcd.write("Longitude");
                lcd.setCursor(0,1); // second row
                char str[8];
                float longitude;
                EEPROM.get(LON_EEPROM_ADDRESS, longitude);
                floatToLcd(longitude, str, 1);    // 1 for longitude
                lcd.write(str);
            } break;
            case digit_selector: {
                digitSelector(digit_format);
                // next state depends on from where digit_selector was entered.
                // must check digit_format to know previous state.
                if(digit_format == 0) {
                    next_state = lat;
                    lcd.clear();
                }
                else if(digit_format == 1) {
                    next_state = lon;
                    lcd.clear();
                }
                else if(digit_format == 2) {
                    next_state = local_time;
                    lcd.clear();
                }
                else if(digit_format == 3) {      // technically doesn't need to be else if() could just be else
                    next_state = date;
                    lcd.clear();
                }
                else {
                    lcd.clear();
                    lcd.write("Inval digi_forma");
                }
            }
        }
    }
}


void digitSelector(byte digit_format) {
    /*
        Used internally by menuFSM(). This is basically its own nested FSM
        that implements changing(selecting) the digits displayed on the LCD
        so that lat, lon, date and time values can be edited. Mainly does
        conversions (string -> float      string -> date/time 
        date/time -> string   and   float -> string ). The RTC(for date/time)
        and EEPROM(for lat/lon) are written from within digitSelector().
        Inputs:
        - digit_format=0    latitude
        - digit_format=1    longitude
        - digit_format=2    local_time
        - digit_format=3    date
    */
    char digits[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    // read characters on bottom row of LCD into array.
    readLcdDigits(digits, digit_format);
    lcd.setCursor(0,1); // start of second row
    lcd.blink();    // blink the cursor.

    // all of this needs to be encapsulated within a loop
        // this is because the digitSelector() is its own nested FSM.
        // do not exit digitSelector() until digit selection is done.
    byte col = 0;
    byte condition = 1;
    while(condition) {
        // skip these characters
        if( (digits[col]=='/') || (digits[col]=='.') || (digits[col]==':') ) {
            col++;
            lcd.setCursor(col,1);
            continue;
        }
        // get which button was pressed.
        Button button = readButtons();

        // edit the digits array
        switch(button) {
            case cycle: {
                if (digits[col] == ' ') {   // if there is padding space, change to zero.
                    digits[col] = '0';
                }
                else if(digits[col] == '+') { // alternate the sign bit if there is one.
                    digits[col] = '-';
                }
                else if(digits[col] == '-') {
                    digits[col] = '+';
                }
                else if(digits[col]=='9') { // wrap around back to '0'
                    digits[col] = '0';
                }
                else {
                    (digits[col])++;     // increment to the next ASCII character.
                }
                lcd.write(digits[col]);     // this auto-increments the cursor position
                lcd.setCursor(col,1);       // must set cursor back
            } break;
            case select: {
                col++;  // move to next character
                lcd.setCursor(col, 1);
                if( !(col < getMaxDigitCol(digit_format)) ) {   // check if we are done.
                    condition = 0;  // stop the while loop
                }
            } break;
            case back: {
                lcd.noBlink();
                return;  // exit the digitSelector();
            }
        }
    }
    // last digit has been selected.
    // therefore, turn off blinking cursor.
    lcd.noBlink();
    
    // convert the BCD digit array into whatever format we need
        // if lat or lon {convert to float}
        // if date or time {convert to integers} the RtcReadout.hpp time format
    if(digit_format==0) {   // latitude
        EEPROM.put(LAT_EEPROM_ADDRESS, atof(digits));
    }
    else if(digit_format==1) {  // longitude
        EEPROM.put(LON_EEPROM_ADDRESS, atof(digits));
    }
    else if(digit_format==2) {  // local time
        byte time[3];
        convertDigitsToTime(time, digits, 0);   // 0 for local time
        rtcWriteTime(time, RTC_ADDRESS);
    }
    else if(digit_format==3) {  // date
        byte time[3];
        convertDigitsToTime(time, digits, 1);   // 1 for date.
        rtcWriteDate(time, RTC_ADDRESS);
    }
    return;
}


void readLcdDigits(char digits[8], byte digit_format) {
    /* 
    A function that is only used internally by digitSelector(). It reads chars
    on bottom row of LCD and stores them in the digits[] array.
    Inputs:
        - digit_format=0    latitude
        - digit_format=1    longitude
        - digit_format=2    local_time
        - digit_format=3    date

    Assumes:
        - LCD has already been initiallised.
        - Only reads bottom row of 2x16 LCD.
    */

    //int i = 0;  // actual digit place (0s 10s 100s ss mm hh etc.) (different to char position).
    byte c;
    byte max_col = getMaxDigitCol(digit_format);
    for(byte col=0; col<max_col; col++) {
        lcd.setCursor(col, 1);
        c = lcd.read();
        digits[col] = c;  // dont convert from ASCII to integers yet. 
        /*
        // this is useful for converion from ascii to BCD encoding
        if( (c=='-') || (c=='+') ) {  // leave the sign character '+' or '-' in digits[i]
            digits[i] = c;
            i++;
            continue;
        }
        c = c - '0'; // converting from ASCII to integer value
        if( (c <= 9) && (c >= 0) ) {    // ignores ',' '.' '/' '-' and other characters.
            digits[i] = c;              // all '0' '1' '2' etc. ASCII characters should return 0-9 value when '0' character is subtracted from them.
            i++;
        }
        if(i > 5) break; // don't try to write too many values to an array with only 6 elements.
        */
    }
}


byte getMaxDigitCol(byte digit_format) {
    /*
    Function that returns the number of columns to be able to select digits for
    depending on the digit_format.
    In simpler terms; it returns the number of characters on the bottom row 
    that can be edited for that digit_format.
    */
    if((digit_format==0) || (digit_format==1)) {   // latitude or longitude
        return 7;
    }
    else return 8;  // date or local time.
}


void menuPrintTime(byte time[7], byte format) {
    /*
    Prints to the HD44780 LCD with I2C. Uses time values encoded in BCD format
    directly as read from the RTC.
    Inputs:
        - format=0  print local time and add in ':' characters.
        - format=1  print date and add in '/' characters.
    Assumes:
        - prints on second row only.
    */
    lcd.setCursor(0,1); // second row
    
    byte upper_nibble = 0b11110000;
    byte lower_nibble = 0b00001111;

    if(format) {  // if format == 1   print date
        lcd.write(((time[3] & upper_nibble) >> 4) + '0');   // printing days
        lcd.write((time[3] & lower_nibble) + '0');
        lcd.write('/');

        lcd.write(((time[5] & upper_nibble) >> 4) + '0');  // printing month
        lcd.write((time[5] & lower_nibble) + '0');
        lcd.write('/');

        lcd.write(((time[6] & upper_nibble) >> 4) + '0');  // printing year
        lcd.write((time[6] & lower_nibble) + '0');
    }
    else {  // else format is 0, so print local time.
        lcd.write(((time[2] & upper_nibble) >> 4) + '0');   // printing hours
        lcd.write((time[2] & lower_nibble) + '0');
        lcd.write(':');

        lcd.write(((time[1] & upper_nibble) >> 4) + '0');  // printing minutes
        lcd.write((time[1] & lower_nibble) + '0');
        lcd.write(':');

        lcd.write(((time[0] & upper_nibble) >> 4) + '0');  // printing seconds      bitmask can be 0b01110000 to avoid reading OS flag
        lcd.write((time[0] & lower_nibble) + '0');
    }
}


void convertDigitsToTime(byte time[3], byte digits[8], byte format) {
    /*
    A function that converts the ASCII digits on an LCD to the BCD format
    required by the RTC.
    Inputs:
        - time[3]   an array that will be passed to the rtcWriteTime() function.
        - digits[8] the ASCII characters that are currently displayed on the LCD.
        - format    0 if converting local time, 1 if converting date.
    */
    byte i = 0; // index digits[]
    byte j;     // index time[]

    if(format) j = 0;   // converting date
    else       j = 2;   // converting local time

    /*
    Need to flip direction in which time[] is filled in depending on
    whether date or local time is being converted. This is because in ASCII
    format date is (basically) little endian (dd/mm/yy), while local time is
    big endian (hh:mm:ss). time[] must be in little endian in the real time
    clock's BCD format. So we have to reverse the direction in which time[] is
    indexed.This is why j is either decremented or incremented. The digits[]
    are always read from left to right. (in increasing order of addresses).
    */

    while(i < 7) {
        time[j] = ((digits[i] - '0') << 4) + (digits[i+1] - '0');
        if(format) j++; // converting date
        else       j--; // converting local time.
        i = i + 3;  // this skips the '/' and ':' characters.
    }
    return;
}


char* floatToLcd(float x, char p[8], byte format) {
    /*
    Only to be used to convert lat or lon floats to string. Got this function
    off stack overflow. I modified it so that it adds zeros at front for
    padding and only displays 3 decimal places. The + and - signs are also
    only added if latitude is being input.

    Inputs:
        - x is the float number to convert.
        - p is the char[] array into which the string will go.
        - if format=0 will produce string in latitude format (with +/- sign)
        - if format=1 will produce string in longitude format (no sign at front).
    Assumes:
        - p is uninitialised.
        - p is 8 elements in size.
    */

    p[7] = 0;   // add null character at end
    uint32_t decimals;  // store the fractional part of x
    uint16_t units;      // store the integers part of x
    if(x < 0) {     // take care of negative numbers
        decimals = (int)(x * -1000) % 1000; // make 100 for 2 decimals etc.
        units = (int)(-1 * x);
    } else {    // positive numbers
        decimals = (int)(x * 1000) % 1000;
        units = (int)x;
    }

    p[6] = (decimals % 10) + '0';
    decimals /= 10;
    p[5] = (decimals % 10) + '0';
    decimals /= 10;
    p[4] = (decimals % 10) + '0';
    p[3] = '.';
    if(!format) {    // if in latitude format
        p[2] = (units % 10) + '0';
        units /= 10;
        p[1] = (units % 10) + '0';
        if(x < 0) p[0] = '-';
        else p[0] = '+';
    }
    else {      // in longitude mode.
        p[2] = (units % 10) + '0';
        units /= 10;
        p[1] = (units % 10) + '0';
        units /= 10;
        p[0] = (units % 10) + '0';
    }

    /*  // the stack overflow code
    char *s = p + 8; // go to end of buffer     (buffer is 8 elements in size)
    uint16_t decimals;  // variable to store the decimals
    int units;  // variable to store the units (part to left of decimal place)
    if(x < 0) { // take care of negative numbers
        decimals = (int)(x * -1000) % 1000; // make 100 for 2 decimals etc.
        units = (int)(-1 * x);
    } else { // positive numbers
        decimals = (int)(x * 1000) % 1000;
        units = (int)x;
    }

    // repeat for as many decimal places as you need
    *--s = (decimals % 10) + '0';
    decimals /= 10;                 // only 3 decimals places needed
    *--s = (decimals % 10) + '0';
    decimals /= 10;
    *--s = (decimals % 10) + '0';
    *--s = '.';     // add decimal point

    while(units > 0) {
        *--s = (units % 10) + '0';
        units /= 10;
    }
    if(x < 0) *--s = '-';   // - sign for negative numbers.
    return s;     // might not need this
    */
}