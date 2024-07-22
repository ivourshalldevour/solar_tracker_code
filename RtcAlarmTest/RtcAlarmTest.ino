#include <Wire.h>
#include <stdio.h>

#include "RtcMethods.hpp"



    /*  GLOBALS */

//  Flag set by ISRs, and cleared by main program loop.
byte rtc_interrupt = 0;
byte keyboard_interrupt = 0;    // used to enter MenuFSM function.



    /* Setup the microcontroller    */

void setup() {
    Serial.begin(9600); // open the serial port at 9600 bps:
    Wire.begin();       // join I2C bus as master


}

void loop() {


}
