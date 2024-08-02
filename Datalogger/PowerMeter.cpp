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


void setupPowerMeter(byte address) {
    Wire.beginTransmission(address);
    Wire.write(5);      // calibration register address
    Wire.write(16);     // MSByte of calibration value. It is 4096 in decimal (a 16bit value)
    Wire.write(0);      // LSByte of calibration value. Gives a current LSBit of 0.0001Amps
    Wire.endTransmission();
    Wire.beginTransmission(address);
    Wire.write(0);      // configuration register address
    Wire.write(0b00111111); // this is to set PGA gain to /8, 128 samples to average for each
    Wire.write(0b11111111); // reading (both shunt and bus voltage), and set mode to measure both shunt and bus V continuously.
    Wire.endTransmission();
}
