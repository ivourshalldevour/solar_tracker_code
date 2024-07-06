#include <SD.h>
#include <Wire.h>
#include "RtcMethods.hpp"

File root;

void setup() {
    Serial.begin(9600);
    Wire.begin();

    // clears the OS flag on the rtc    
    while(rtcCheckClock(RTC_ADDRESS)) {
        Serial.println("Bad clock");
    }

    Serial.print("Initializing SD card...");
    if (!SD.begin(10)) {  // 10 is Chip Select pin.
        Serial.println("Initialisation failed");
        //while(true);    // stay here
    }
    //Serial.println("initialization done.");


    unsigned int secs;
    unsigned int last_secs = 1;
    unsigned int raw_analog = 0;
    byte i = 0;
    // do this once every second
    while(true) {
        secs = millis() / 1000;
        if(secs != last_secs) {
            last_secs = secs;   // keep track of last value.
            
            // this is executed each second.
            raw_analog = analogRead(A0); 
            Serial.println(raw_analog, DEC);
            i++;
        }
        else if(i > 30) {
            break;
        }
    }
    Serial.println("Done.");
}

void loop() {
}
