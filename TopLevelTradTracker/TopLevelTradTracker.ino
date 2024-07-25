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
byte rtc_interrupt = 0;         // used to trigger tracker movements.
byte keyboard_interrupt = 0;    // used to enter MenuFSM function.


    /* Setup the microcontroller    */

void setup() {
    Serial.begin(9600); // open the serial port at 9600 bps:
    Wire.begin();       // join I2C bus as master

    // Set these pins as inputs for the Cycle, Select, Back buttons.
    PORTC = PORTC & 0b01100111; // setting PC7, PC4, PC3 (pin7, pin4, pin3)as inputs without pullups.
    DDRC  = DDRC  & 0b01100111;

    // Enable pin change interrupts on the Cycle, Select, Back buttons.
    PCICR = PCICR | (1 << PCIE2);   // enabling interrupts from pins PD7-PD0.
    PCMSK2 = PCMSK2 | (1<<PCINT23) | (1<<PCINT20) | (1<<PCINT19);   // Masking so only PD7 PD4 & PD3 cause interrupts.
    // these pin change interrupts call PCINT2_vect.

    // Set pin 2 (INT0) to be external interrupt from RTC
    DDRD = DDRD & 0b11111011;   // set pin to to input.
    PORTD = PORTD | 0b00000100; // enabling pull-up resistor on pin 2.
    EICRA = 1 << ISC01;         // setting interrupt to be triggered on falling edge.
    EIMSK = 1 << INT0;          // enabling external interrupt on pin 2

    // initialise the I2C LCD module.
    int status;
    status = lcd.begin(LCD_COLS, LCD_ROWS); // initialising lcd (also turns on backlight)
    if(status) {
        hd44780::fatalError(status);
    } // if there was an error, blink the lcd backlight.
    lcd.off();  // turn off pixels and backlight.

    // check that the RTC has good clock integrity
    while(rtcCheckClock(RTC_ADDRESS)) {
        Serial.println("Bad clock.");
    }

    setupServoTimer();  // also configures pins 5&6 as outputs.
    rtcSetupCountdown(1, RTC_ADDRESS);  // configure rtc to generate interrupts every 1 minute.
}



    /* Main program loop */

void loop() {
    if(rtc_interrupt==1) {
        rtc_interrupt = 0;

        // clear the RTC's CTBF flag.
        Wire.beginTransmission(RTC_ADDRESS);
        Wire.write(0x1);    // Control_2 register address
        Wire.write(0b00000001); // clear all flags & set CTBIE (enables timer B).
        Wire.endTransmission();
        
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
        //float elev = asin(sin(declination)*);
        // if sun above horizon.
            // move servos.
            // goto sleep.
        // if sun below horizon
            // stop timer B (set TBC=0 in Tmr_CLKOUT_ctrl register).
            // calculate sunrise time.
            // set alarm for this sunrise time.
            // goto sleep.
    }
    else if(keyboard_interrupt==1) {
        keyboard_interrupt = 0; // clear flag.
        PCICR = PCICR & (!(1<<PCIE2));  // disable pin change interrupts from any of the CSB buttons.
        menuFSM();
        PCIFR = PCIFR | (1<<PCIF2);   // clear PCINT2 flag by writing one to it.
        PCICR = PCICR | (1<<PCIE2); // enable pin change interrupts from the CSB buttons again.
    }
}


