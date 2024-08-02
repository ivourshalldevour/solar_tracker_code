#include "RtcMethods.hpp"

/*
    // Code to check if RTC is in battery switchover mode.
    // If not,  put it in battery switchover mode.
    Wire.beginTransmission(RTC_ADDRESS);
    Wire.write(0x2); // address of control register 3
    Wire.requestFrom(RTC_ADDRESS,1); // read only 1 byte (only control register 3)
    byte ctrl_reg3 = Wire.read();
    Wire.endTransmission();
    Serial.println(ctrl_reg3, BIN);
    if((ctrl_reg3 & 0b11100000) != 0) { // if not in battery switchover
        Serial.println("changing ctrl_reg3.");
        Wire.beginTransmission(RTC_ADDRESS);
        Wire.write(0x2);    //address
        Wire.write(ctrl_reg3 & 0b00011111);       // value at that address
        // ctrl_reg3[7:5] becomes 0b101    battery switchover enabled
        Wire.endTransmission();
    }
    Wire.beginTransmission(RTC_ADDRESS);
    Wire.write(0x2); // address of control register 3
    Wire.requestFrom(RTC_ADDRESS,1); // read only 1 byte (only control register 3)
    ctrl_reg3 = Wire.read();
    Wire.endTransmission();
    Serial.println(ctrl_reg3, BIN);
*/


// defined in main .ino file
extern byte rtc_interrupt;


ISR(INT0_vect) {
    rtc_interrupt = 1;
}


byte rtcCheckClock(int address) {
    // Assumes the PCF8523 chip is being used.
    // returns 1 if the OS flag is set. Therefore clock integrity cannot be guaranteed.
        // also attempts to clear the flag if this is the case
    // returns 0 if the flag was 0.
    Wire.beginTransmission(address);
    Wire.write(0x3);                // set to seconds register
    Wire.endTransmission(false);    // false so line is not released.
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
}


void rtcSetupCountdown(byte interval, int rtc_address) {
    /*
        Function that setups up the countdown timer B on the PCF8523 real time
        clock to generate interrupts on its INT1 pin. This is an active LOW
        interrupt. The countdown timer B is set to count down the number of
        minutes given by interval.
        Input:
            - interval: number of minutes between each interrupt signal.
        Output:
            - Writes to I2C to setup the RTCs registers to start the timer.
        Assumes:
            - The interrupt needs to be cleared via I2C in a different function
              to disable the interrupt.
    */

    // Write to Control_2 register
        Wire.beginTransmission(RTC_ADDRESS);
        Wire.write(0x1);    // address of Control_2
        Wire.write(0b00000001);   // Set CTBIE=1 to enable countdown timer B interrupts. Also clear all flags.
        Wire.endTransmission();
    // Write to Tmr_B_Freq_ctrl register & load value into T_B.
        Wire.beginTransmission(RTC_ADDRESS);
        Wire.write(0x12);   // address of Tmr_B_Freq_ctrl
        Wire.write(0b00000011);
        // TBW don't care. Arduino interrupts set to trigger on falling edge not level, so pulse width doesn't matter.
        // TBQ = 0b011 for counting minutes.
        Wire.write(interval);   // Load number of minutes into T_B.
        Wire.endTransmission();
    // Write to Tmr_CLKOUT_ctrl register
        Wire.beginTransmission(RTC_ADDRESS);
        Wire.write(0x0F);   // address of Tmr_CLKOUT_ctrl
        Wire.write(0b00111001);
        // set permanent active interrupt (TAM bit)
        // set permanent active interrupt (TBM bit)
        // disable CLKOUT generation (COF bits)
        // disable timer A (TAC bits)
        // enable timer B (TBC bit)
        Wire.endTransmission();
}


int julianDay(byte* time) {
    // check if current year is leap year
    byte leap_year = 0;
    if((time[6]%4) == 0) {
        leap_year = 1;
    }

    // Adjust february to have 29 days if leap year.
    byte days_in_month[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if(leap_year) {
        days_in_month[1]++;
    }
    
    // Calculate days since beginning of year.
    int julian_day = time[3];   // days
    for(byte i=0; i < (time[5]-1); i++) {
        julian_day = julian_day + (int)days_in_month[i];
    }
    return julian_day;
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
    Wire.endTransmission(false);    // false so line is not released.
    Wire.requestFrom(rtc_address,7);        // Tell slave we need to read 7bytes begining at the current register
    int i = 0;
    while(Wire.available() != 0 && (i < 7)) {   // making sure that there are still bytes to read from I2C
        time[i] = Wire.read();          // read that byte into 'rtc_time' array
        i++;
    }
    Wire.endTransmission();                 // "Hang up the line" so others can use it (can have multiple slaves & masters connected)
}


void rtcConvertTime(byte* time) {
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
         - Weekday number (sunday, monday, etc) is not changed.
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
    Wire.endTransmission();
}

