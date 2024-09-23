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
    - returns 1 if OVF is 1. Therefore, current and power registers are
      overflowed. Measurements is not changed.
    Assumes:
    - The CNVR flag is not checked.
    - The INA219 has already been setup appropriately.
    - measurements[0] is bus voltage in mV
    - measurements[1] is current in mA
    - measurements[2] is power in mW
*/
byte readPVI(int* measurements, byte i2c_address);

/*
    Does the same as readMeasurements() but only returns power instead
    of current and voltage aswell.
*/
byte readPower(int* power, byte i2c_address);


/*
    Sets up the INA219 current meter using I2C. Must be given an I2C address
    to which to write. The INA219 is configured to average 128 12bit samples
    together for each voltage measurement. It measures voltage and current
    continously. Uses a current calibration value of 4096 which gives a
    current LSBit of 0.1mA. This causes a power LSbit of 2mW. The bus voltage
    LSBit is always 4mV.
*/
void setupPowerMeter(byte i2c_address);

#endif