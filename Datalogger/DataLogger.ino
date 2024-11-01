#include <SD.h>
#include <Wire.h>

#include "RtcMethods.hpp"
#include "PowerMeter.hpp"
#include "Mutex3.hpp"

byte rtc_interrupt = 0;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Set pin 2 (INT0) to be external interrupt from RTC
    DDRD = DDRD & 0b11111011;   // set pin to to input.
    PORTD = PORTD | 0b00000100; // enabling pull-up resistor on pin 2.
    EICRA = 1 << ISC01;         // setting interrupt to be triggered on falling edge.
    EIMSK = 1 << INT0;          // enabling external interrupt on pin 2

    // Set mutex pins as inputs and outputs.
    DDRB = DDRB & (!((1<<DDB1) | (1<<DDB0)));   // Set pins PB1 and PB0 as inputs. (without pullups)
    // Set pin PC3 as output.
    PORTC = PORTC & !(1<<PORTC3);   // set to output LOW
    DDRC = DDRC | (1<<DDC3);   // then set as output (avoids accidental HIGH)

    startMutex();
    setupPowerMeter(0x41);  // i2c addresses
    setupPowerMeter(0x44);
    setupPowerMeter(0x45);
    endMutex();

    // SD.begin(10);
    Serial.print("Initializing SD card...");
    if (!SD.begin(10)) {  // 10 is Chip Select pin.
        Serial.println("Initialisation failed");
        while(true);    // stay here
    }
    Serial.println("Done.");

    //File file = SD.open("test.txt", FILE_WRITE);    // FILE_WRITE enables both read and write access.
    //file.println("panel_num,time(s),voltage(mV),current(mA),power(mW)");    // write out column headings


    unsigned int secs;
    unsigned int last_secs = 1;
    int measurements[3] = {0,0,0};
    char str[9] = {0,0,0,0,0,0,0,0,0};
}

void loop() {
    if(rtc_interrupt) {
        rtc_interrupt = 0;
        // Read RTC's control_2 register to determine if Timer A or B timed-out
        byte status = startMutex();   // start i2c comms.
        if(status) {return;}    // arbitration failed.

        Wire.beginTransmission(RTC_ADDRESS);
        Wire.write(0x1);    // Control_2 register address
        Wire.endTransmission(false);    // false so I2C line is not released.
        Wire.requestFrom(RTC_ADDRESS, 1);   // read 1 byte.
        byte ctrl_reg2 = Wire.read();
        Wire.endTransmission();
        if( !(ctrl_reg2 & 0b01000000) ) { // if CTAF not raised
            // ignore this interrupt
            // caused by something else 
            // maybe Timer B.
            endMutex(); // finish i2c comms
            return; // leave the main loop() function.
        }
        // CTAF was raised!
        // so now we clear CTAF.
        Wire.beginTransmission(RTC_ADDRESS);
        Wire.write(0x1);    // Control_2 reg address
        Wire.write(ctrl_reg2 & 0b10111111);
        Wire.endTransmission();
        
        Serial.println("Logging...");

        int measurements[3] = {0, 0, 0};
        char str[9];

        // get date and time from RTC.
        byte time[7];
        rtcGetTime(time, RTC_ADDRESS);
        rtcConvertTime(time);

        // Now read the power level.
        readPower(measurements, 0x41);

        // open the logging file
        File file = SD.open("test.txt", FILE_WRITE);    // might need to be FILE_APPEND
        file.print(1); file.print(',');                 // panel num
        sprintf(str, "%02d:%02d:%02d", time[2], time[1], time[0]);  
        file.print(str); file.print(',');               // print time (hh:mm:ss)
        sprintf(str, "%02d/%02d/%02d", time[3], time[5], time[6]);  
        file.print(str); file.print(',');           // print date (dd/mm/yy)
        file.print(measurements[0]); file.print(',');   // bus voltage (mV)
        file.print(measurements[1]); file.print(',');   // current  (mA)
        file.println(measurements[2]);                  // power (mW)
        file.close();

        // Repeat for other power meters.
        readPower(measurements, 0x44);
        file = SD.open("test.txt", FILE_WRITE);    // might need to be FILE_APPEND
        file.print(2); file.print(',');                 // panel num
        sprintf(str, "%02d:%02d:%02d", time[2], time[1], time[0]);  
        file.print(str); file.print(',');               // print time (hh:mm:ss)
        sprintf(str, "%02d/%02d/%02d", time[3], time[5], time[6]);  
        file.print(str); file.print(',');           // print date (dd/mm/yy)
        file.print(measurements[0]); file.print(',');   // bus voltage (mV)
        file.print(measurements[1]); file.print(',');   // current  (mA)
        file.println(measurements[2]);                  // power (mW)
        file.close();

        readPower(measurements, 0x45);
        file = SD.open("test.txt", FILE_WRITE);    // might need to be FILE_APPEND
        file.print(3); file.print(',');                 // panel num
        sprintf(str, "%02d:%02d:%02d", time[2], time[1], time[0]);  
        file.print(str); file.print(',');               // print time (hh:mm:ss)
        sprintf(str, "%02d/%02d/%02d", time[3], time[5], time[6]);  
        file.print(str); file.print(',');           // print date (dd/mm/yy)
        file.print(measurements[0]); file.print(',');   // bus voltage (mV)
        file.print(measurements[1]); file.print(',');   // current  (mA)
        file.println(measurements[2]);                  // power (mW)
        file.close();

        endMutex(); // finish i2c comms
    }
}
