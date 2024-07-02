#include <Wire.h>
#include <hd44780.h>                       // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header

#include "ButtonInput.hpp"  // uses CSB buttons
#include "RtcMethods.hpp"

#define LCD_COLS 16
#define LCD_ROWS 2

hd44780_I2Cexp lcd; // declare lcd object: auto locate & auto config expander chip
    // we don't give it a specific I2C address because the library auto finds it.

typedef enum {start, gps, local_time, date, lat, lon, digit_selector} FSM_State;

FSM_State current_state = start;
FSM_State next_state = start;

float latitude = 33.832;       // might make double in future (since trig functions use double)
float longitude = 151.124;

void setup() {
    int status;

    Serial.begin(9600); // open the serial port at 9600 bps:
    Wire.begin();       // join I2C bus as master

    status = lcd.begin(LCD_COLS, LCD_ROWS); // initialising lcd (also turns on backlight)
    if(status) {
        hd44780::fatalError(status);
    } // if there was an error, blink the lcd backlight.
}

void loop() {
    // used for the digitSelector() function to know how to print to lcd.
    // also is convenient to determine what the state immediately prior was.
    static byte digit_format;    // 0=lat 1=lon, 2=hh:mm:ss, 3=yy/mthmth/dd
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
                }
                // if button == back        go back to sleep mode?? (future implementation)
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
            rtcConvertTime(time);
            char str[9];
            sprintf(str, "%02d:%02d:%02d", time[2], time[1], time[0]);  //   time[0]=sec, time[1]=min, 2=hrs, 3=days, 4=weekdays,5=months,6=yrs
            lcd.setCursor(0,1); // second row
            lcd.write(str);
        } break;
        case date: {
            lcd.home();
            lcd.write("Date (dd/mm/yy)");
            // get date and print it on second row
            byte time[7];
            rtcGetTime(time, RTC_ADDRESS);
            rtcConvertTime(time);
            char str[9];
            // if this sprintf() function uses too much memory. I could use several sepparate lcd.print(time[i], DEC) functions. And lcd.print("/") in between.
            // however, then i won't be able to pad with zeros. sprintf() can pad with zeros.
            sprintf(str, "%02d/%02d/%02d", time[3], time[5], time[6]);   //   time[0]=sec, time[1]=min, 2=hrs, 3=days, 4=weekdays,5=months,6=yrs
            lcd.setCursor(0,1); // second row
            lcd.write(str);
        } break;
        case lat: {
            lcd.home();
            lcd.write("Latitude");
            char str[8];
            dtostrf(latitude, 7, 3, str);
            if(latitude > 0) {  // must add in '+' sepparately.
                str[0] = '+';
            }
            lcd.setCursor(0,1); // second row
            lcd.write(str);
        } break;
        case lon: {
            lcd.home();
            lcd.write("Longitude");
            lcd.setCursor(0,1); // second row
            lcd.print(longitude, DEC);
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
                lcd.print("Inval digi_forma");
            }
        }
    }
}


void digitSelector(byte digit_format) {
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
        latitude = atof(digits);
    }
    else if(digit_format==1) {  // longitude
        longitude = atof(digits);
    }
    else if(digit_format==2) {  // local time

    }
    else if(digit_format==3) {  // date

    }
    return;
}

void readLcdDigits(char digits[8], byte digit_format) {
    /* 
    A function that is only used internally by digitSelector(). It reads chars
    on bottom row of LCD and stores them in an array.

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
    In simpler terms; it returns the number of characters  on the bottom row 
    that can be edited for that digit_format.
    */
    if((digit_format==0) || (digit_format==1)) {   // latitude or longitude
        return 7;
    }
    else return 8;  // date or local time.
}

