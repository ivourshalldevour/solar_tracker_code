#include "SolarServo.hpp"


void setup() {
    Serial.begin(9600);

    setupServoTimer();

    // clock prescaler 1, 16bit counter timer
    // 65535 / 16,000,000Hz = 0.004095s = 4.095ms
    // RC servos only works on pulses of width 0.5ms to 2.5ms (1.5ms is neutral)
    // 0.0005sx16Mhz = 8000 counter value
    // 0.0025sx16Mhz = 40000 counter value
    // Therefore we must manipulate the output compare (OCR1A) register only within these values.

    /* Notes:
        // 9500 in OCR1A (0.593ms pulse) is 90 right
        // 40000 in OCR1A (2.5ms pulse) is 90 left.
        // 
    */
}

void loop() {
    commandServo(1, 45);
    delay(2000);
    commandServo(1, 90);    
    delay(2000);
}


