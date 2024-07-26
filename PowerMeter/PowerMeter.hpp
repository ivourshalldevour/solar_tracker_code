#ifndef POWER_METER_H
#define POWER_METER_H

/*
    Uses I2C to read the measured Bus voltage, current and power from an
    INA219 power meter chip. It does all the necesary conversions and checks
    the OVF flag.
    Inputs:
    - measurements is a pointer to a 3 element array of int
    - i2c_address is the address of the INA219 chip.
    Outputs:
    - returns 0 if OVF is LOW and conversions successful.
    - returns 1 if OVF is 1. Therefore, current and power registers are overflowed. Measurements is not changed.
    Assumes:
    - The CNVR flag is not checked.
    - The INA219 has already been setup appropriately.
    - measurements[0] is bus voltage in mV
    - measurements[1] is current in mA
    - measurements[2] is power in mW
*/
byte readPower(int* measurements, byte i2c_address);

#endif