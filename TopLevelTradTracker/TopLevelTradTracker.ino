#include <Wire.h>
#include <SD.h>
#include <stdio.h>
#include <hd44780.h>                       // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header
#include <EEPROM.h>

#include "RtcMethods.hpp"
#include "SolarServo.hpp"
#include "ButtonInput.hpp"
#include "MenuFSM.hpp"


#define LCD_COLS 16
#define LCD_ROWS 2


    /*  GLOBALS */

hd44780_I2Cexp lcd; // declare lcd object: auto locate & auto config expander chip
    // we don't give it a specific I2C address because the library auto finds it.

//  Flag set by ISRs, and cleared by main program loop.
byte rtc_interrupt = 0;
byte keyboard_interrupt = 0;    // used to enter MenuFSM function.


    /* Setup the microcontroller    */

void setup() {
    Serial.begin(9600); // open the serial port at 9600 bps:
    Wire.begin();       // join I2C bus as master

    // Set these pins as inputs for the Cycle, Select, Back buttons.
    PORTB = PORTB & 0b11111110; // setting PB0 (pin8) as input without pullup.
    DDRB  = DDRB  & 0b11111110;
    PORTC = PORTC & 0b01101111; // setting PC7 & PC4 (pin7 and pin4)as inputs without pullups.
    DDRC  = DDRC  & 0b01101111;

    // Set INT0 pin as external interrupt from the RTC.
    //DDRD = ... set PD2 as 0 to be input (already done by default).
    PORTD = PORTD | 0b00000100; // enabling pull-up resistor on PD2.
    EICRA = 1 << ISC01;     // setting interrupt to be triggered on falling edge.
    EIMSK = 1 << INT0;   // enabling external interrupt on pin INT0

    // initialise the I2C LCD module.
    int status;
    status = lcd.begin(LCD_COLS, LCD_ROWS); // initialising lcd (also turns on backlight)
    if(status) {
        hd44780::fatalError(status);
    } // if there was an error, blink the lcd backlight.

    // check that the RTC has good clock integrity
    while(rtcCheckClock(RTC_ADDRESS)) {
        Serial.println("Bad clock.");
    }

    // set the RTC to generate alarm and countdown timer interrupts
        // Write to Tmr_CLKOUT_ctrl register
            // set permanent active interrupt (TAM bit)
            // set permanent active interrupt (TBM bit)
            // disable CLKOUT generation (COF bits)
            // disable timer A (TAC bits)
            // enable timer B (TBC bit)

    setupServoTimer();  // also configures pins 5&6 as outputs.
    rtc_interrupt=1;
}



    /* Main program loop */

void loop() {
    
    // read RTC's Control_2 register
    // if AF==1
        // clear AF & CTBF
        // disable alarm timer.
        // enable timer B
    // if CTBF==1
        // clear AF & CTBF

    if(rtc_interrupt==1) {
        /*
        // read RTC's Control_2 register
        Wire.beginTransmission(RTC_ADDRESS);
        Wire.write(0x1);   // set to address of Control_2 register
        Wire.requestFrom(RTC_ADDRESS, 1);   // read 1 byte
        byte rtc_control_2 = Wire.read();
        Wire.endTransmission();
        
        // Clear AF and CTBF in RTC registers
        Wire.beginTransmission(RTC_ADDRESS);
        Wire.write(0x1);   // set to address of Control_2 register
        Wire.write(rtc_control_2 & 0b11010111);
        Wire.endTransmission();
        */
        
        // get date and time from RTC.
        byte time[7];
        rtcGetTime(time, RTC_ADDRESS);
        rtcConvertTime(time);

        // Calculate position of sun (LHA and declination).
        // Using algorithm on PVeducation website. https://www.pveducation.org/pvcdrom/properties-of-sunlight/solar-time

        // Calculate Local Hour Angle (LHA).
        float longitude;
        EEPROM.get(LON_EEPROM_ADDRESS, longitude);
        int julian_day = julianDay(time);
        float B = (360.0/365.0)*(julian_day-81)*(PI/180.0);
        float eqn_time = 9.87*sin(2*B) - 7.53*cos(B) - 1.5*sin(B);
        Serial.print("eqn_time: "); Serial.println(eqn_time);
        float LT = (float)((float)time[2]+(float)time[1]*(1/60.0));     // local time (non-daylight saving)
        Serial.print("LT: "); Serial.println(LT);
        float LST = LT + (1/60.0)*(4*(longitude-150.0) + eqn_time);   // Local solar time (accounts for eqn of time.)
        Serial.print("LST: "); Serial.println(LST);
        float LHA = 15.0*(LST-12.0);
        Serial.print("LHA: "); Serial.println(LHA);

        // Calculate Declination
        float declination = -23.45*cos( (360.0/365.0)*(julian_day-10)*(PI/180.0) );
        Serial.print("DEC: "); Serial.println(declination);

        // calculate elevation angle (to check if above horizon)
        float elev = asin(sin(declination)*);
        // if sun above horizon.
            // move servos.
            // goto sleep.
        // if sun below horizon
            // stop timer B (set TBC=0 in Tmr_CLKOUT_ctrl register).
            // calculate sunrise time.
            // set alarm for this sunrise time.
            // goto sleep.
    }

    else if(keyboard_interrupt==1) {    // for now technically impossible to enter since we dont have an ISR to set this flag.
        keyboard_interrupt = 0; // clear flag.
        //menuFSM();
    }
    rtc_interrupt=0;
}


