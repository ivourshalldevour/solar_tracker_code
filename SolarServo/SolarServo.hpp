#ifndef SOLAR_SERVO_H
#define SOLAR_SERVO_H

#include <Arduino.h>

// ISR to generate pulse for servo on pins 5&6 using timer1
ISR(TIMER1_COMPA_vect);

void setupServoTimer();

// servo_num=0 is declination servo on pin 5
// servo_num=1 is hour angle servo pn pin 6
// Angle is +-90 degrees. 0 is neutral (PV panel horizontal).
void commandServo(byte servo_num, int8_t angle);

#endif