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

    rtcSetupCountdown(1, RTC_ADDRESS);
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

