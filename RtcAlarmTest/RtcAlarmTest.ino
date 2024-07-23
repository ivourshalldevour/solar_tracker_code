#include <Wire.h>
#include <stdio.h>

#include "RtcMethods.hpp"



    /*  GLOBALS */

//  Flag set by ISRs, and cleared by main program loop.
volatile byte rtc_interrupt = 0;
volatile byte keyboard_interrupt = 0;    // used to enter MenuFSM function.



    /* Setup the microcontroller    */

void setup() {
    Serial.begin(9600); // open the serial port at 9600 bps:
    Wire.begin();       // join I2C bus as master

    // Set pin 2 (INT0) to be external interrupt from RTC
    DDRD = DDRD & 0b11111011;   // set pin to to input.
    PORTD = PORTD | 0b00000100; // enabling pull-up resistor on pin 2.
    EICRA = 1 << ISC01;     // setting interrupt to be triggered on falling edge.
    EIMSK = 1 << INT0;   // enabling external interrupt on pin 2

    // check that the RTC has good clock integrity
    while(rtcCheckClock(RTC_ADDRESS)) {
        Serial.println("Bad clock.");
    }

    // set the RTC to generate countdown timer B interrupts every 30 seconds.
        // Write to Control_2 register
            Wire.beginTransmission(RTC_ADDRESS);
            Wire.write(0x1);    // address of Control_2
            Wire.write(0b00000001);   // Set CTBIE=1 to enable countdown timer B interrupts. Also clear all flags.
            Wire.endTransmission();
        // Write to Tmr_B_Freq_ctrl register & load value into T_B.
            Wire.beginTransmission(RTC_ADDRESS);
            Wire.write(0x12);   // address of Tmr_B_Freq_ctrl
            Wire.write(0b00000010);
            // TBW don't care. Arduino interrupts set to trigger on falling edge not level, so pulse width doesn't matter.
            // TBQ = 0b010 for counting seconds.
            Wire.write(5);   // Load timer value into T_B. Interrupts will be generated every 5 seconds.
            Wire.endTransmission();
        // Write to Tmr_CLKOUT_ctrl register
            Wire.beginTransmission(RTC_ADDRESS);
            Wire.write(0x0F);   // address of Tmr_CLKOUT_ctrl
            Wire.write(0b00111001);
            // set permanent active interrupt (TAM bit)
            // set permanent active interrupt (TBM bit)
            // disable CLKOUT generation (COF bits)
            // disable timer A (TAC bits)
            // enable timer B (TBC bit)
            Wire.endTransmission();
    
    // Notes:
    /* 
        - To turn off timer B. Clear bit TBC in Tmr_CLKOUT_ctrl register.
        - Interrupts can be disabled (timer can still run) with CTBIE in register Control_2
        - Only load T_B register when timer B is disabled. Avoids corrupting the value loaded into the timer, since this value is set asynchronously to the RTC clock.
        - Loading T_B with 0 effectively stops timer B.
        - When timer B reaches 1. CTBF in Control_2 is set. INT1 is permanent LOW. Flag CTBF must be cleared to clear interrupt back to HIGH.
    */
}

void loop() {
    if(rtc_interrupt==1) {
        rtc_interrupt = 0;  // clear flag

        // clear the RTC's CTBF flag.
        Wire.beginTransmission(RTC_ADDRESS);
        Wire.write(0x1);    // Control_2 register address
        Wire.write(0b00000001); // clear all flags & set CTBIE.
        Wire.endTransmission();

        // Now, read time and print to serial.
        byte time[7] {0,0,0,0,0,0,0};
        rtcGetTime(time, RTC_ADDRESS);
        rtcConvertTime(time);
        Serial.println("hh:mm:ss");
        Serial.print(time[2]);
        Serial.print(":");
        Serial.print(time[1]);
        Serial.print(":");
        Serial.println(time[0]);
    }
}

