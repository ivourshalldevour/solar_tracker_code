#include <Wire.h>
#include <SD.h>
// #include <stdio.h>
#include <hd44780.h>                       // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header
#include <EEPROM.h>

#include "RtcMethods.hpp"
#include "SolarServo.hpp"
#include "ButtonInput.hpp"
#include "MenuFSM.hpp"
#include "Mutex2.hpp"
#include "PowerMeter.hpp"


#define LCD_COLS 16
#define LCD_ROWS 4



    /*  GLOBALS */

hd44780_I2Cexp lcd(0x26); // declare lcd object with address

//  Flag set by ISRs, and cleared by main program loop.
byte rtc_interrupt = 0;         // used to trigger tracker movements.
byte keyboard_interrupt = 0;    // used to enter MenuFSM function.
volatile int8_t current_angle = 0;  // stores current angle of declination servo 0.



    /* 
        Setup the microcontroller    
            The RTC is not written by Arduino 2. All RTC setup is done
            by Arduino 1.
    */


void setup() {
    Serial.begin(115200); // open the serial port at 115200 bps:
    Wire.begin();       // join I2C bus as master

    // Set these pins as inputs for the Cycle, Select, Back buttons.
    PORTC = PORTC & 0b01100111; // setting PC7, PC4, PC3 (pin7, pin4, pin3)as inputs without pullups.
    DDRC  = DDRC  & 0b01100111;

    // Set mutex pins as inputs and outputs.
    DDRB = DDRB & !(1<<DDB1);   // set mutex1 (PB1) as input.
    DDRC = DDRC & !(1<<DDC3);   // set mutex3 (PC3) as input.
    // set mutex 2 (PB0) as output.
    PORTB = PORTB & !(1<<PORTB0);   // set to output LOW
    DDRB = DDRB | (1<<DDB0);   // then set as output (avoids accidental HIGH)

     // Enable pin change interrupts on the Cycle, Select, Back buttons.
    PCICR = PCICR | (1 << PCIE2);   // enabling interrupts from pins PD7-PD0.
    PCMSK2 = PCMSK2 | (1<<PCINT23) | (1<<PCINT20) | (1<<PCINT19);   // Masking so only PD7 PD4 & PD3 cause interrupts.
    // these pin change interrupts call PCINT2_vect.

    // Set pin 2 (INT0) to be external interrupt from RTC
    DDRD = DDRD & 0b11111011;   // set pin to to input.
    PORTD = PORTD | 0b00000100; // enabling pull-up resistor on pin 2.
    EICRA = 1 << ISC01;         // setting interrupt to be triggered on falling edge.
    EIMSK = 1 << INT0;          // enabling external interrupt on pin 2

    startMutex();

    setupPowerMeter(0x41);  // i2c addresses
    setupPowerMeter(0x44);
    setupPowerMeter(0x45);

    // initialise the I2C LCD module.
    int status;
    status = lcd.begin(LCD_COLS, LCD_ROWS); // initialising lcd (also turns on backlight)
    if(status) {
        hd44780::fatalError(status);
    } // if there was an error, blink the lcd backlight.
    lcd.off();  // turn off pixels and backlight.

    endMutex(); // end i2c comms.


    // SD.begin(10);
    Serial.print("Initializing SD card...");
    if (!SD.begin(10)) {  // 10 is Chip Select pin.
        Serial.println("Initialisation failed");
        while(true);    // stay here
    }
    Serial.println("Done.");


    setupServoTimer();  // also configures pins 5&6 as outputs.
    // Move servos to home (0,0)
    commandServo(0, 0);
    delay(10);
    commandServo(1, 0);
    delay(5000);    // Wait 5 seconds so I can physically align the
                    // mechanism if there was any slip during homing.
    current_angle = 0;
}

void loop() {
// ignoring RtcInterrupt in the Adaptive tracker for now.
// remember to limit adaptive tracker's declination servo (servo 0) to only +-80 degrees. So it doesnt hit post.

if(rtc_interrupt==1) {
        rtc_interrupt = 0;
        Serial.println("Ard2: RTC_INT triggered");

        // start i2c comms.
        byte status = startMutex();
        if(status) {return;}    // arbitration failed.

        // Read RTC's control_2 register to determine if Timer A or B timed-out
        Wire.beginTransmission(RTC_ADDRESS);
        Wire.write(0x1);    // Control_2 register address
        Wire.endTransmission(false);    // false so I2C line is not released.
        Wire.requestFrom(RTC_ADDRESS, 1);   // read 1 byte.
        byte ctrl_reg2 = Wire.read();
        Wire.endTransmission();

        // CTAF triggers both datalogging and adaptive tracker movements in one.
        if( ctrl_reg2 & 0b01000000 ) {  // CTAF was raised!
            Serial.println("Ard2: CTAF was raised!");
            // clear CTAF.
            Wire.beginTransmission(RTC_ADDRESS);
            Wire.write(0x1);    // Control_2 reg address
            Wire.write(ctrl_reg2 & 0b10111111);
            Wire.endTransmission();


            ////////       Begin tracker movements      //////


            // kept outside of calculation scope
            float direct_theta0 = 0;    // angles to point directly at sun
            float direct_theta1 = 0;
            float max_theta0 = 0;       // angles at which max power was measured
            float max_theta1 = 0;
            int direct_power = 0;       // power when pointed directly at sun

            {   // scoping bracket to remove intermediate variables from stack once calculations are done.

            // Move to direct position of sun


            // get date and time from RTC.
            byte time[7];
            rtcGetTime(time, RTC_ADDRESS);
            rtcConvertTime(time);

            // Calculate position of sun (LHA and declination).
            // Using algorithm on PVeducation website. https://www.pveducation.org/pvcdrom/properties-of-sunlight/solar-time

            // Calculate Local Hour Angle (LHA).
            float longitude;
            EEPROM.get(LON_EEPROM_ADDRESS, longitude);
            int julian_day = julianDay(time);

            float B = (360.0/365.0)*(julian_day-81)*(PI/180.0);
            float eqn_time = 9.87*sin(2*B) - 7.53*cos(B) - 1.5*sin(B);
            float LT = (float)((float)time[2]+(float)time[1]*(1/60.0));     // local time (non-daylight saving)
            float LST = LT + (1/60.0)*(4*(longitude-150.0) + eqn_time);     // Local solar time (accounts for eqn of time.)
            float LHA = 15.0*(LST-12.0);                                    // LHA

            // Calculate declination angle
            float declination = -23.45*cos( (360.0/365.0)*(julian_day+10)*(PI/180.0) );

            // calculate elevation angle (+ve above horizon, ranges -90 to +90)
            float latitude;
            latitude = EEPROM.get(LAT_EEPROM_ADDRESS, latitude);
            float elev = asin(sin(declination*(PI/180.0)) * sin(latitude*(PI/180.0)) + cos(declination*(PI/180.0)) * cos(latitude*(PI/180.0)) * cos(LHA*(PI/180.0)) );
            Serial.print("elev: "); Serial.println(elev*(180.0/PI));
            if(elev < 0 ) { // if sun below horizon
                // stop timer B (set TBC=0 in Tmr_CLKOUT_ctrl register).
                // calculate sunrise time.
                // set alarm for this sunrise time.
                // goto sleep.
                Serial.println("Not logging. Sun below horizon.");
                return; // don't log data if sun below horizon.
            }

            // calculate azimuth angle  (0 at South. +90 at West, -90 at East, ranges -180 to +180)
            float azi = atan2(sin(LHA*(PI/180.0)), cos(LHA*(PI/180.0))*sin(latitude*(PI/180.0)) - tan(declination*(PI/180.0))*cos(latitude*(PI/180.0)) );
            Serial.print("azi: "); Serial.println(azi*(180.0/PI));

            // calculate solar tracker axes angles
            // these formulas won't be valid if sun is below horizon
            direct_theta0 = (180.0/PI)*atan( cos(azi)*(1/tan(elev)) );
            direct_theta1 = (180.0/PI)*sin(azi)*cos(elev);
            Serial.print("direct_theta0: "); Serial.println((int8_t)direct_theta0);
            Serial.print("direct_theta1: "); Serial.println((int8_t)direct_theta1);
            }   // scoping bracket to remove calculation intermediate steps from stack


            // moving servos
            slowServo(0, (int8_t)direct_theta0);   // moves slowly to avoid slip in mechanism
            delay(20);
            commandServo(1, (int8_t)direct_theta1);

            // measure power at the actual location of the sun
            readPower(direct_power, 0x44);
            Serial.print("Direct power:"); Serial.println(direct_power);

            // now search power through a cross shape
            {   // scoping bracket to remove intermediate measurements from stack once done searching.
            int theta_powers[7];  // power measurements while moving through an axis

            // first measure through theta0
            byte i = 0; // indexes measurements
            for(byte angle=-90; angle<90; angle+=30) {
                slowServo(0, angle);
                readPower(&(theta_powers[i]), 0x44);
                i++;
            }
            // find which theta0 had max power
            byte max_i = 0; // index at which max power occured
            { // stack scoping bracket
            int max_power = 0;  // 
            for(i=0; i<7; i++) {
                if(theta_powers[i] > max_power) {
                    max_power = theta_powers[i];
                    max_i = i;
                }
            }
            }
            max_theta0 = (int)-90 + (int)(max_i  *(int)30);

            // move to max_theta0
            slowServo(0, max_theta0);

            // now measure through theta1
            i = 0; // indexes measurements
            for(byte angle=-90; angle<90; angle+=30) {
                commandServo(1, angle);
                delay(500);
                readPower(&(theta_powers[i]), 0x44);
                i++;
            }

            // find which theta1 had max power
            max_i = 0; // index at which max power occured
            { // stack scoping bracket
            int max_power = 0;  // 
            for(i=0; i<7; i++) {
                if(theta_powers[i] > max_power) {
                    max_power = theta_powers[i];
                    max_i = i;
                }
            }
            if(max_power > direct_power) {  // if this searched location has more power than direct position
                max_theta1 =  (int)-90 + (int)(max_i  *(int)30);
                Serial.println("Max power found at cross-searched position.");
            }
            else { // if direct position has more power
                max_theta0 = direct_theta0;
                max_theta1 = direct_theta1;
                Serial.println("Max power found at direct position.");
            }
            }
            

            }   // stack scoping bracket to remove all temporary variables within search routine.

            // move to position of max power
            slowServo(0, max_theta0);
            commandServo(1, max_theta1);
            delay(500);
            Serial.println("Movements done.");

            //////      Tracker movements done      ///////////





            ///////      Now proceed to datalogging      //////

            // No data will be recorded if sun below horizon as above.
            int log_measurements[3] = {0, 0, 0};
            char str[9];

            // get date and time from RTC.
            byte time[7];
            rtcGetTime(time, RTC_ADDRESS);
            rtcConvertTime(time);

            // Now read the power level.
            readPVI(log_measurements, 0x41);

            // open the logging file
            File file = SD.open("test.txt", FILE_WRITE);    // might need to be FILE_APPEND
            file.print(1); file.print(',');                 // panel num
            sprintf(str, "%02d:%02d:%02d", time[2], time[1], time[0]);  
            file.print(str); file.print(',');               // print time (hh:mm:ss)
            sprintf(str, "%02d/%02d/%02d", time[3], time[5], time[6]);  
            file.print(str); file.print(',');           // print date (dd/mm/yy)
            file.print(log_measurements[0]); file.print(',');   // bus voltage (mV)
            file.print(log_measurements[1]); file.print(',');   // current  (mA)
            file.print(log_measurements[2]); file.print(',');   // power (mW)
            file.println(','); // the missing values for theta0 and theta1
            file.close();

            // Repeat for other power meters.
            readPVI(log_measurements, 0x44);
            file = SD.open("test.txt", FILE_WRITE);    // might need to be FILE_APPEND
            file.print(2); file.print(',');                 // panel num
            sprintf(str, "%02d:%02d:%02d", time[2], time[1], time[0]);  
            file.print(str); file.print(',');               // print time (hh:mm:ss)
            sprintf(str, "%02d/%02d/%02d", time[3], time[5], time[6]);  
            file.print(str); file.print(',');           // print date (dd/mm/yy)
            file.print(log_measurements[0]); file.print(',');   // bus voltage (mV)
            file.print(log_measurements[1]); file.print(',');   // current  (mA)
            file.print(log_measurements[2]); file.print(',');   // power (mW)
            file.print(max_theta0); file.print(",");
            file.println(max_theta1);  // adaptive tracker angles.
            file.close();

            readPVI(log_measurements, 0x45);
            file = SD.open("test.txt", FILE_WRITE);    // might need to be FILE_APPEND
            file.print(3); file.print(',');                 // panel num
            sprintf(str, "%02d:%02d:%02d", time[2], time[1], time[0]);  
            file.print(str); file.print(',');               // print time (hh:mm:ss)
            sprintf(str, "%02d/%02d/%02d", time[3], time[5], time[6]);  
            file.print(str); file.print(',');           // print date (dd/mm/yy)
            file.print(log_measurements[0]); file.print(',');   // bus voltage (mV)
            file.print(log_measurements[1]); file.print(',');   // current  (mA)
            file.print(log_measurements[2]); file.print(','); // power (mW)
            file.println(','); // the missing values for theta0 and theta1
            file.close();
        }
        Serial.println("Logging done.");

        endMutex(); // finish i2c comms
}


else if(keyboard_interrupt==1) {
        keyboard_interrupt = 0; // clear flag.
        PCICR = PCICR & (!(1<<PCIE2));  // disable pin change interrupts from any of the CSB buttons.
        byte status = startMutex();   // need this to arbitrate the multi-master i2c port.
        menuFSM();
        endMutex();
        PCIFR = PCIFR | (1<<PCIF2);   // clear PCINT2 flag by writing one to it.
        PCICR = PCICR | (1<<PCIE2); // enable pin change interrupts from the CSB buttons again.
    }
}
