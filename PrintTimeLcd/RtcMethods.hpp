#ifndef RTC_READOUT_H
#define RTC_READOUT_H

#define RTC_ADDRESS 0b1101000   // for PCF8523 chip

byte rtcCheckClock(int address) {
    // Assumes the PCF8523 chip is being used.
    // returns 1 if the OS flag is set. Therefore clock integrity cannot be guaranteed.
        // also attempts to clear the flag if this is the case
    // returns 0 if the flag was 0.
    Wire.beginTransmission(address);
    Wire.write(0x3);                // set to seconds register
    Wire.requestFrom(address,1);    // read the seconds register
    byte sec_reg = Wire.read();
    Wire.endTransmission();
    if(sec_reg & 0b10000000) {    // check if OS flag (bit 7) is set.
        Wire.beginTransmission(address);
        Wire.write(0x3);
        Wire.write(sec_reg & 0b01111111);   // attempt to clear the flag.
        Wire.endTransmission();
        return 1;                           // the OS flag was set.
    }
    else {
        return 0; // OS flag was not set.
    }
    Wire.endTransmission();
}

void rtcGetTime(byte time[7], int rtc_address) {
    /*
        A function that gets the date and time from a PCF8523 Real Time Clock
        via I2C. It does not do any data conversion. Just gets the raw time.
        Inputs:
         - rtc_address      the I2C address of the Real Time Clock.
         - time     pointer to 7 element array to store time in.
        Outputs:
         - time is written with the retrieved date and time.
        Assumes:
            time[0]=sec, time[1]=min, 2=hrs, 3=days, 4=weekdays,5=months,6=yrs  (bascially little endian)
    */

    Wire.beginTransmission(rtc_address);    // Get the slave's attention, tell it we're sending a command byte
    Wire.write(0x3);                        // The command byte, sets pointer to register with address of 0x3
    Wire.requestFrom(RTC_ADDRESS,7);        // Tell slave we need to read 7bytes begining at the current register
    int i = 0;
    while(Wire.available() != 0 && (i < 7)) {   // making sure that there are still bytes to read from I2C
        time[i] = Wire.read();          // read that byte into 'rtc_time' array
        i++;
    }
    Wire.endTransmission();                 // "Hang up the line" so others can use it (can have multiple slaves & masters connected)
}

void rtcConvertTime(byte *time) {
    /*
    A function that takes raw time data from the PCF8523 RTC chip and converts it
    from BCD into normal binary encoding. Should be called after rtcGetTime().
    Returns 1 if clock integrity is not guaranteed.

    Input:  pointer to a 7 element byte (a.k.a. unit8_t) array    in BCD
    Output: edits that array directly.

    Assumes:
     - rtc's OS flag is cleared. (therefore RTC clock integrity is guaranteed).
        Call rtcCheckClock() to clear this flag.
     - hours are in 24hour format (can be changed in PCF8523 chip)
     - time[0]=sec, time[1]=min, 2=hrs, 3=days, 4=weekdays,5=months,6=yrs
     - for weekdays (sunday is 0, monday is 1, etc.)
    */

    byte low_nibble = 0b00001111;     // maybe change to byte (doesnt have to be const)
    // convert seconds
    time[0] = (time[0] >> 4)*10 + (time[0] & low_nibble);

    // convert minutes
    time[1] = (time[1] >> 4)*10 + (time[1] & low_nibble);

    // convert hours
    time[2] = (time[2] >> 4)*10 + (time[2] & low_nibble);

    // convert days
    time[3] = (time[3] >> 4)*10 + (time[3] & low_nibble);

    // no conversion needed for weekday
    time[4] = time[4];      // Sunday is 0, monday is 1 etc.

    // convert months
    time[5] = (time[5] >> 4)*10 + (time[5] & low_nibble);

    //convert years
    time[6] = (time[6] >> 4)*10 + (time[6] & low_nibble);
}

void rtcWriteTime(byte time[3], int rtc_address) {
    /* 
    Writes the local time (ss:mm:hh)  to an RTC on the
    PCF8523 chip. Using I2C.

    Inputs:
     - 3 element byte array (time[]) in BCD encoding
     - the I2C address for the Real Time Clock(RTC).
     Assumes:
         - each time[] element is in BCD encoding.
         - time[0]=sec, time[1]=min, 2=hrs  (bascially little endian)
         - Already joined I2C bus as master.  Wire.begin(); is done.
    */

    // write the bcd array to RTC chip.
    Wire.beginTransmission(rtc_address);
    Wire.write(0x3);    // set to seconds register.
    for(byte i=0; i<3; i++) {
        Wire.write(time[i]);
    }
    Wire.endTransmission();
}

void rtcWriteDate(byte time[3], int rtc_address) {
    /* 
    Writes the date (dd/mm/yy) to an RTC on the
    PCF8523 chip. Using I2C.

    Inputs:
     - 3 element byte array (time[]) in BCD encoding.
     - the I2C address for the Real Time Clock(RTC).
     Assumes:
         - each time[] element is in BCD encoding.
         - time[0]=day, time[1]=month, 2=year  (bascially little endian)
         - Already joined I2C bus as master.  Wire.begin(); is done.
    */
    
    // write the bcd array to RTC chip.
    Wire.beginTransmission(rtc_address);
    Wire.write(0x6);    // set to days register
    Wire.write(time[0]); // write in day number
    Wire.endTransmission();
    Wire.beginTransmission(rtc_address);
    Wire.write(0x8);    // set to months register   (must skip weekdays register)
    Wire.write(time[1]);     // write in month number
    Wire.write(time[2]);     // write in year number
}

#endif
