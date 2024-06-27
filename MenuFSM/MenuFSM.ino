#include <Wire.h>
#include <hd44780.h>                       // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header

#include "ButtonInput.hpp"  // uses CSB buttons
#include "RtcReadout.hpp"

#define LCD_COLS 16
#define LCD_ROWS 2

hd44780_I2Cexp lcd; // declare lcd object: auto locate & auto config expander chip
    // we don't give it a specific I2C address because the library auto finds it.

typedef enum {start, gps, local_time, date, lat, lon, digit_selector} FSM_State;

FSM_State current_state = start;
FSM_State next_state = start;

float latitude = -33.832;       // might make double in future (since trig functions use double)
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
                    lcd.clear();
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
                    lcd.clear();
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
                    lcd.clear();
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
                    lcd.clear();
                } break;
                case back: {
                    next_state = gps;
                    lcd.clear();
                }
            }
            break;
        case digit_selector:
            switch(button) {
                // cycle is handled by digitSelector() in output logic.
                // select is handled by digitSelector() in output logic.
                case back: {
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
    current_state = next_state; 
    // end of next state logic

    
    // output logic
    switch(current_state) {
        case start: {
            lcd.home();
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
            // however, then ii won't be able to pad with zeros. sprintf() can pad with zeros.
            sprintf(str, "%02d/%02d/%02d", time[3], time[5], time[6]);   //   time[0]=sec, time[1]=min, 2=hrs, 3=days, 4=weekdays,5=months,6=yrs
            lcd.setCursor(0,1); // second row
            lcd.write(str);
        } break;
        case lat: {
            lcd.home();
            lcd.write("Latitude");
            lcd.setCursor(0,1); // second row
            lcd.print(latitude, DEC);
        } break;
        case lon: {
            lcd.home();
            lcd.write("Longitude");
            lcd.setCursor(0,1); // second row
            lcd.print(longitude, DEC);
        } break;
        case digit_selector: {
            lcd.home();
            lcd.write("digit_selector");
            digitSelector(digit_format);
        }
    }
}


void digitSelector(byte digit_format) {

    // all of this needs to be encapsulated within a for loop
        // this is because the digitSelector() is its own nested FSM.
        // do not exit digitSelector() until digit selection is done.

    // for each digit_format case
        // read all current digits on bottom row.
        // store each digit into an array (this is essentially BCD encoding)
    // turn on blinking cursor
    // for each digit:
        // if cycle {increment digit and display}
        // if select {move to next digit} (basically continue;)
        // if back {exit digitSelector()}
    // once last digit selected (above for loop done)
    // turn off blinking cursor
    // convert the BCD digit array into whatever format we need
        // if lat or lon {convert to float}
        // if date or time {convert to integers} the RtcReadout.hpp time format
}

