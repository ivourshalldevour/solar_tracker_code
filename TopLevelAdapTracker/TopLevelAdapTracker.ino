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
#include "Mutex2.hpp"


#define LCD_COLS 16
#define LCD_ROWS 4



    /*  GLOBALS */

hd44780_I2Cexp lcd(0x26); // declare lcd object with address

//  Flag set by ISRs, and cleared by main program loop.
byte rtc_interrupt = 0;         // used to trigger tracker movements.
byte keyboard_interrupt = 0;    // used to enter MenuFSM function.



    /* 
        Setup the microcontroller    
            The RTC is not written by Arduino 2. All RTC setup is done
            by Arduino 1.
    */


void setup() {
    Serial.begin(115200); // open the serial port at 9600 bps:
    Wire.begin();       // join I2C bus as master

    // Set these pins as inputs for the Cycle, Select, Back buttons.
    PORTC = PORTC & 0b01100111; // setting PC7, PC4, PC3 (pin7, pin4, pin3)as inputs without pullups.
    DDRC  = DDRC  & 0b01100111;

    // Set mutex pins as inputs and outputs.
    DDRB = DDRB & !(1<<DDB1);   // set mutex1 (PB1) as input.
    DDRC = DDRC & !(1<<DDC3);   // set mutex3 (PC3) as input.
    // set mutex 2 (PB0) as output.
    PORTB = PORTB & !(1<<PORTB0);   // set to output LOW
    DDRB = DDRB | (1<<DDB0);   // then set as output (avoids accidental HIGH)

     // Enable pin change interrupts on the Cycle, Select, Back buttons.
    PCICR = PCICR | (1 << PCIE2);   // enabling interrupts from pins PD7-PD0.
    PCMSK2 = PCMSK2 | (1<<PCINT23) | (1<<PCINT20) | (1<<PCINT19);   // Masking so only PD7 PD4 & PD3 cause interrupts.
    // these pin change interrupts call PCINT2_vect.

    // Set pin 2 (INT0) to be external interrupt from RTC
    DDRD = DDRD & 0b11111011;   // set pin to to input.
    PORTD = PORTD | 0b00000100; // enabling pull-up resistor on pin 2.
    EICRA = 1 << ISC01;         // setting interrupt to be triggered on falling edge.
    EIMSK = 1 << INT0;          // enabling external interrupt on pin 2

    startMutex();

    // initialise the I2C LCD module.
    int status;
    status = lcd.begin(LCD_COLS, LCD_ROWS); // initialising lcd (also turns on backlight)
    if(status) {
        hd44780::fatalError(status);
    } // if there was an error, blink the lcd backlight.
    lcd.off();  // turn off pixels and backlight.
    endMutex(); // end i2c comms.

    setupServoTimer();  // also configures pins 5&6 as outputs.
}

void loop() {
// ignoring RtcInterrupt in the Adaptive tracker for now.

/*if(rtc_interrupt==1) {
        rtc_interrupt = 0;
        // Do stuff.
}
else*/ if(keyboard_interrupt==1) {
        keyboard_interrupt = 0; // clear flag.
        PCICR = PCICR & (!(1<<PCIE2));  // disable pin change interrupts from any of the CSB buttons.
        byte status = startMutex();   // need this to arbitrate the multi-master i2c port.
        Serial.println(status, BIN);
        menuFSM();
        endMutex();
        PCIFR = PCIFR | (1<<PCIF2);   // clear PCINT2 flag by writing one to it.
        PCICR = PCICR | (1<<PCIE2); // enable pin change interrupts from the CSB buttons again.
    }
}
