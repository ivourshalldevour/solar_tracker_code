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
#include "Mutex1.hpp"


#define LCD_COLS 16
#define LCD_ROWS 2


    /*  GLOBALS */

hd44780_I2Cexp lcd(0x27); // declare lcd object with address

//  Flag set by ISRs, and cleared by main program loop.
byte volatile rtc_interrupt = 0;         // used to trigger tracker movements.
byte volatile keyboard_interrupt = 0;    // used to enter MenuFSM function.


    /* Setup the microcontroller    */

void setup() {
    Serial.begin(115200); // open the serial port at 115200 bps:
    Wire.begin();       // join I2C bus as master

    // Set these pins as inputs for the Cycle, Select, Back buttons.
    PORTC = PORTC & 0b01100111; // setting PC7, PC4, PC3 (pin7, pin4, pin3)as inputs without pullups.
    DDRC  = DDRC  & 0b01100111;

    // Set the mutex pins as inputs and outputs
    DDRB = DDRB & !(1<<DDB0);   // set mutex2 (PB0) as input.
    DDRC = DDRC & !(1<<DDC3);   // set mutex3 (PC3) as input.
    // set mutex 1 (PB1) as output.
    PORTB = PORTB & !(1<<PORTB1);   // set to output LOW
    DDRB = DDRB | (1<<DDB1);   // then set as output (avoids accidental HIGH)

    // Enable pin change interrupts on the Cycle, Select, Back buttons.
    PCICR = PCICR | (1 << PCIE2);   // enabling interrupts from pins PD7-PD0.
    PCMSK2 = PCMSK2 | (1<<PCINT23) | (1<<PCINT20) | (1<<PCINT19);   // Masking so only PD7 PD4 & PD3 cause interrupts.
    // these pin change interrupts call PCINT2_vect.

    // Set pin 2 (INT0) to be external interrupt from RTC
    DDRD = DDRD & 0b11111011;   // set pin to to input.
    PORTD = PORTD | 0b00000100; // enabling pull-up resistor on pin 2.
    EICRA = 1 << ISC01;         // setting interrupt to be triggered on falling edge.
    EIMSK = 1 << INT0;          // enabling external interrupt on pin 2

    startMutex();   // start i2c comms.

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
    rtcSetupCountdown(1, 1, RTC_ADDRESS);   // also clears CTAF and CTBF
    endMutex(); // end i2c comms.
}



    /* Main program loop */

void loop() {
    if(rtc_interrupt==1) {
        rtc_interrupt = 0;

        // Reading RTC's Control_2 register to determine which Timer caused the interrupt.
        byte status = startMutex();   // start i2c
        Serial.println(status,BIN);
        if (status) {Serial.println("startMutex failed...");return;}   // arbitration was not successful.
        Serial.println("startMutex() success!");

        Wire.beginTransmission(RTC_ADDRESS);
        Wire.write(0x1);    // Control_2 register address
        Wire.endTransmission(false);    // false so I2C line is not released.
        Wire.requestFrom(RTC_ADDRESS, 1);   // read 1 byte.
        byte ctrl_reg2 = Wire.read();
        Wire.endTransmission();
        if( !(ctrl_reg2 & 0b00100000) ) { // if CTBF not raised
            // ignore this interrupt
            // caused by something else 
            // maybe Timer A.
            endMutex(); // finish i2c comms
            return; // leave the main loop() function.
        }
        // CTBF was raised!
        // so now we clear CTBF.
        Wire.beginTransmission(RTC_ADDRESS);
        Wire.write(0x1);    // Control_2 reg address
        Wire.write(ctrl_reg2 & 0b11011111);
        Wire.endTransmission();

        // get date and time from RTC.
        byte time[7];
        rtcGetTime(time, RTC_ADDRESS);
        rtcConvertTime(time);
        endMutex(); // finish i2c comms

        // Calculate position of sun (LHA and declination).
        // Using algorithm on PVeducation website. https://www.pveducation.org/pvcdrom/properties-of-sunlight/solar-time

        // Calculate Local Hour Angle (LHA).
        float longitude;
        EEPROM.get(LON_EEPROM_ADDRESS, longitude);
        Serial.print("LON: "); Serial.println(longitude);
        int julian_day = julianDay(time);
        Serial.print("julian_day: "); Serial.println(julian_day);       // julian day

        float B = (360.0/365.0)*(julian_day-81)*(PI/180.0);
        float eqn_time = 9.87*sin(2*B) - 7.53*cos(B) - 1.5*sin(B);
        Serial.print("eqn_time: "); Serial.println(eqn_time);           // eqn of time

        float LT = (float)((float)time[2]+(float)time[1]*(1/60.0));     // local time (non-daylight saving)
        Serial.print("LT: "); Serial.println(LT);

        float LST = LT + (1/60.0)*(4*(longitude-150.0) + eqn_time);     // Local solar time (accounts for eqn of time.)
        Serial.print("LST: "); Serial.println(LST);

        float LHA = 15.0*(LST-12.0);                                    // LHA
        Serial.print("LHA: "); Serial.println(LHA);

        // Calculate declination angle
        float declination = -23.45*cos( (360.0/365.0)*(julian_day+10)*(PI/180.0) );
        Serial.print("DEC: "); Serial.println(declination);

        // calculate elevation angle (+ve above horizon, ranges -90 to +90)
        float latitude;
        latitude = EEPROM.get(LAT_EEPROM_ADDRESS, latitude);
        Serial.print("LAT: "); Serial.println(latitude);
        float elev = asin(sin(declination*(PI/180.0)) * sin(latitude*(PI/180.0)) + cos(declination*(PI/180.0)) * cos(latitude*(PI/180.0)) * cos(LHA*(PI/180.0)) );
        Serial.print("elev: "); Serial.println(elev*(180.0/PI));
        if(elev < 0 ) { // if sun below horizon
            // stop timer B (set TBC=0 in Tmr_CLKOUT_ctrl register).
            // calculate sunrise time.
            // set alarm for this sunrise time.
            // goto sleep.
            Serial.println("Sun below horizon.");
            return;
        }

        // calculate azimuth angle  (0 at South. +90 at West, -90 at East, ranges -180 to +180)
        float azi = atan2(sin(LHA*(PI/180.0)), cos(LHA*(PI/180.0))*sin(latitude*(PI/180.0)) - tan(declination*(PI/180.0))*cos(latitude*(PI/180.0)) );
        Serial.print("azi: "); Serial.println(azi*(180.0/PI));

        // calculate solar tracker axes angles
        // these formulas won't be valid if sun is below horizon
        float theta0 = (180.0/PI)*atan( cos(azi)*(1/tan(elev)) );
        float theta1 = (180.0/PI)*sin(azi)*cos(elev);
        Serial.print("theta0: "); Serial.println((int8_t)theta0);
        Serial.print("theta1: "); Serial.println((int8_t)theta1);

        // moving servos
        commandServo(0, (int8_t)theta0);
        delay(20);
        commandServo(1, (int8_t)theta1);
    }
    else if(keyboard_interrupt==1) {
        keyboard_interrupt = 0; // clear flag.
        PCICR = PCICR & (!(1<<PCIE2));  // disable pin change interrupts from any of the CSB buttons.
        byte status = startMutex();   // need this to arbitrate the multi-master i2c port.
        if(status) {return;}    // arbitration was not successful.
        menuFSM();
        endMutex();

        PCIFR = PCIFR | (1<<PCIF2);   // clear PCINT2 flag by writing one to it.
        PCICR = PCICR | (1<<PCIE2); // enable pin change interrupts from the CSB buttons again.
    }
}


