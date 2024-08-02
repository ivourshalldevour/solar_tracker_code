#include <SD.h>
#include <Wire.h>
#include "PowerMeter.hpp"

byte rtc_interrupt = 0;

void setup() {
    Serial.begin(9600);
    Wire.begin();

    // clears the OS flag on the rtc    
    //while(rtcCheckClock(RTC_ADDRESS)) {
    //    Serial.println("Bad clock");
    //}

    setupPowerMeter(0x40);  // i2c address 0x40

    Serial.print("Initializing SD card...");
    if (!SD.begin(10)) {  // 10 is Chip Select pin.
        Serial.println("Initialisation failed");
        while(true);    // stay here
    }
    Serial.println("Done.");

    File file = SD.open("test.txt", FILE_WRITE);    // FILE_WRITE enables both read and write access.
    file.println("panel_num,time(s),voltage(mV),current(mA),power(mW)");    // write out column headings


    unsigned int secs;
    unsigned int last_secs = 1;
    unsigned int raw_analog = 0;
    int measurements[3] = {0,0,0};

    byte i = 1;
    // do this once every second
    while(true) {
        secs = millis() / 1000;
        if(secs != last_secs) {
            last_secs = secs;   // keep track of last value.
            
            // this is executed once each second. for 30s max
            readPower(measurements, 0x40);
            file.print(1); file.print(',');                 // panel num
            file.print(i); file.print(',');                 // time (s)
            file.print(measurements[0]); file.print(',');   // bus voltage
            file.print(measurements[1]); file.print(',');   // current
            file.println(measurements[2]);    // power
                    
            Serial.println(i);
            i++;
        }
        else if(i > 30) {
            break;
        }
    }

    file.close();   // close the file.
    Serial.println("Done logging.");
}

void loop() {
}
