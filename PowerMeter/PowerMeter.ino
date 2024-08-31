#include <Wire.h>
#include "PowerMeter.hpp"

void setup() {
    Serial.begin(9600);
    Wire.begin();   // join i2c as master

    // setup the INA219 current meter chip
    setupPowerMeter(0x45);

    /*
        Notes:
        - Internal registers of the chip have an 8 bit address even though they
        store a 16bit value.
        - The INA219 doesn't have an auto incrementing register pointer like the PCF8523 has.
    */
}

void loop() {
    // read from INA219 power meter every 1 second.
    unsigned static int secs = 0;
    unsigned static int lastsecs = 0;
    secs = millis() / 1000;
    if(secs != lastsecs) {
        lastsecs = secs;

        // measurements[0] is for Bus voltage
        // measurements[1] is for current
        // measurements[2] is for power
        int measurements[3] = {0, 0, 0};
        
        byte status = readPower(measurements, (byte)0x45);
        if(status) {
            Serial.println("Error: Power and current overflowed.");
            return; // skip to next iteration of loop()
        }
        
        Serial.print("V_Bus   (mV): ");
        Serial.println(measurements[0]);
        Serial.print("Current (mA): ");
        Serial.println(measurements[1]);
        Serial.print("Power   (mW): ");
        Serial.println(measurements[2]);
    }
}
