#include <Wire.h>
#include "PowerMeter.hpp"

void setup() {
    Serial.begin(9600);
    Wire.begin();   // join i2c as master

    // setup the INA219 current meter chip
    Wire.beginTransmission(0x40);
    Wire.write(5);      // calibration register address
    Wire.write(16);     // MSByte of calibration value.
    Wire.write(0);      // LSByte of calibration value. Gives a current LSBit of 0.0001Amps
    Wire.endTransmission();
    Wire.beginTransmission(0x40);
    Wire.write(0);      // configuration register address
    Wire.write(0b00111111); // this is to set PGA gain to /8, 128 samples to average for each
    Wire.write(0b11111111); // reading (both shunt and bus voltage), and set mode to measure both shunt and bus V continuously.
    Wire.endTransmission();

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
        
        byte status = readPower(measurements, (byte)0x40);
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
