#include "SolarServo.hpp"


void setup() {
    Serial.begin(115200);

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
    /*
    commandServo(0, 0);
    delay(10);
    commandServo(1, 0);
    Serial.println("0 degrees.");
    delay(4000);
    commandServo(0, 0);
    delay(10);
    commandServo(1, 00);
    Serial.println("00 degrees.");
    delay(4000);
    */

    // Slow servo movements
    slowServo(0, 0, -80);
    delay(10);
    commandServo(1, 90);
    delay(4000);

    slowServo(0, -80, 0);
    delay(10);
    commandServo(1, 0);
    delay(4000);
}


