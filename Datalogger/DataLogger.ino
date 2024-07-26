#include <SD.h>
#include <Wire.h>

byte rtc_interrupt = 0;

void setup() {
    Serial.begin(9600);
    Wire.begin();

    // clears the OS flag on the rtc    
    //while(rtcCheckClock(RTC_ADDRESS)) {
    //    Serial.println("Bad clock");
    //}

    Serial.print("Initializing SD card...");
    if (!SD.begin(10)) {  // 10 is Chip Select pin.
        Serial.println("Initialisation failed");
        while(true);    // stay here
    }
    Serial.println("Done.");

    File myfile = SD.open("test.txt", FILE_WRITE);    // FILE WRITE enables both read and write access.
    


    unsigned int secs;
    unsigned int last_secs = 1;
    unsigned int raw_analog = 0;
    byte i = 1;
    // do this once every second
    while(true) {
        secs = millis() / 1000;
        if(secs != last_secs) {
            last_secs = secs;   // keep track of last value.
            
            // this is executed once each second. for 30s max
            myfile.print(i);
            if((i%5)==0) myfile.print('\n');
            else myfile.print(',');
            Serial.println(i);
            i++;
        }
        else if(i > 30) {
            break;
        }
    }

    myfile.close();   // close the file.
}

void loop() {
}
