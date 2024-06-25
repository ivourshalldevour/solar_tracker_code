#include <Wire.h>
#include <hd44780.h>                       // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header

#define LCD_COLS 16
#define LCD_ROWS 2

hd44780_I2Cexp lcd; // declare lcd object: auto locate & auto config expander chip
    // we don't give it a specific I2C address because the library auto finds it.

typedef enum {none,cycle, select, back} Button;
typedef enum {start, gps, local_time, date, lat, lon, hh, mm, ss, yy, mthmth, dd} FSM_State;

FSM_State current_state = start;
FSM_state next_state = start;


void setup() {
    int status;

    Serial.begin(9600); // open the serial port at 9600 bps:
    Wire.begin();       // join I2C bus as master

    status = lcd.begin(LCD_COLS, LCD_ROWS); // initialising lcd (also turns on backlight)
    if(status) {
        hd44780::fatalError(status);
    } // if there was an error, blink the lcd backlight.

    lcd.print("Main Menu"); // top hierarchy
}

void loop() {
    // read pins 8 7 and 4.
    // assuming that more than 1 cannot be pressed at the same time.
    Button button = none;
    if(digitalRead(8) == 1)
        button = cycle;
    else if(digitalRead(7) == 1)
        button = select;
    else if(digitalRead(4) == 1)
        button = back;
    else
        button = none;

    // next state logic
    switch(current_state) {
        case start:
            /*
            lcd.clear();                // should i combine next state logic and output logic ????? 
            lcd.print("Main Menu");
            */
            switch(button) {
                case cycle: next_state = gps;
            }
            // if button == select      do nothing
            // if button == back        do nothing
            break;
        case gps:

            switch(button) {
                case cycle: next_state = local_time; break;
                case select: next_state = lat; break;
                case back: next_state = start;
            }
            break;
        case local_time:
            switch(button) {
                case cycle: next_state = date; break;
                case select: next_state = hh; break;
                case back: next_state = start;
            }
            break;
        case date:
            switch(button) {
                case cycle: next_state = gps; break;
                case select: next_state = yy; break;
                case back: next_state = start;
            }
            break;
        case lat:
            switch(button) {
                case cycle: next_state = lon; break;
                case select: next_state = // enter digit counter
                case back: next_state = gps;
            }
            break;
        case lon:
            switch(button) {
                case cycle: next_state = lat; break;
                case select: next_state = // enter digit counter
                case back: next_state = gps;
            }
            break;
        case hh:
            switch(button) {
                case cycle: next_state = mm; break;
                case select: next_state = // enter digit counter
                case back: next_state = local_time;
            }
            break;
        case mm:
            switch(button) {
                case cycle: next_state = ss; break;
                case select: next_state = // enter digit counter
                case back: next_state = local_time;
            }
            break;
        case ss:
            switch(button) {
                case cycle: next_state = hh; break;
                case select: next_state = // enter digit counter
                case back: next_state = local_time;
            }
            break;
        case yy:
            switch(button) {
                case cycle: next_state = mthmth; break;
                case select: next_state = // enter digit counter
                case back: next_state = date;
            }
            break;
        case mthmth:
            switch(button) {
                case cycle: next_state = dd; break;
                case select: next_state = // enter digit counter
                case back: next_state = date;
            }
            break;
        case dd:
            switch(button) {
                case cycle: next_state = yy; break;
                case select: next_state = // enter digit counter
                case back: next_state = date;
            }
            break;
    }

    current_state = next_state; 
    
    // output state logic

}
