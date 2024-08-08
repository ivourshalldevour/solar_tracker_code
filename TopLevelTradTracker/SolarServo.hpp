#ifndef SOLAR_SERVO_H
#define SOLAR_SERVO_H

#include <Arduino.h>

// ISR to generate pulse for servo on pins 5&6 using timer1
ISR(TIMER1_COMPA_vect);

/*
    Sets up the 16bit timer1 on the arduino to be ready for the commandServo()
    function.
*/
void setupServoTimer();


/*
    Uses timer1 to generate one-shot pulses (only one made for each function
    call) that control the RC servos on pins 5&6 of the arduino. Requires an
    interrupt service routine to run at the end of each pulse, so isn't
    entirely hardware timer solution. The pulses are between 0.5ms and 2.5ms
    long, which corresponds to +90deg (clockwise) and -90deg (anticlockwise)
    respectively.
*/
// servo_num=0 is declination servo on pin 5
// servo_num=1 is hour angle servo pn pin 6
// Angle is +-90 degrees. 0 is neutral (PV panel horizontal).
void commandServo(byte servo_num, int8_t angle);

#endif