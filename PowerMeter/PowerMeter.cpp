#include <Arduino.h>
#include <Wire.h>
#include "PowerMeter.hpp"


byte readPower(int* measurements, byte address) {
        // read Bus voltage
        Wire.beginTransmission(address);
        Wire.write(2);  // bus voltage address is 2
        Wire.endTransmission(false);    // doesn't do a stop condition does a re-start.
        Wire.requestFrom(address,2);   // reading 2 bytes, because we are reading a 16bit register.
        byte MSB = Wire.read();
        byte LSB = Wire.read();
        Wire.endTransmission();
        // now check OVF flag in the Bus voltage register
        if((LSB & 0b1) == 1) {
            return 1; // OVF is set. Current and power registers are overflowed.
        }
        measurements[0] = (MSB<<5) | (LSB>>3);   // remove 3 lowest bytes (they are flags).
        measurements[0] = measurements[0] * 4;  // convert to mV

        // read power register
        Wire.beginTransmission(address);
        Wire.write(3); 
        Wire.endTransmission(false);
        Wire.requestFrom(address,2);
        MSB = Wire.read();
        LSB = Wire.read();
        Wire.endTransmission();
        measurements[2] = (MSB<<8) | LSB; 
        measurements[2] = measurements[2] * 2;  // convert to mW
        
        // read current register
        Wire.beginTransmission(address);
        Wire.write(4);  
        Wire.endTransmission(false);
        Wire.requestFrom(address,2);
        MSB = Wire.read();
        LSB = Wire.read();
        Wire.endTransmission();
        measurements[1] = (MSB<<8) | LSB;
        measurements[1] = measurements[1] / 10; // convert to mA

        return 0;   // 0 for success!
}
