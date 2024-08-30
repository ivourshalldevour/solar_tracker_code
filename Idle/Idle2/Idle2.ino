/*
    This code is only to be uploaded to Arduino 2.
    This code's only purpose is to pull-down the Mutex lines.
    Otherwise it does nothing. That's the point, this code is only for 
    debugging purposes, so we can isolate the problem.
*/

void setup() {
    DDRB = DDRB & !(1<<DDB1);   // set mutex1 (PB1) as input.
    DDRC = DDRC & !(1<<DDC3);   // set mutex3 (PC3) as input.

    // set mutex 2 (PB0) as output.
    PORTB = PORTB & !(1<<PORTB0);   // set to output LOW
    DDRB = DDRB | (1<<DDB0);   // then set as output (avoids accidental HIGH)
}

void loop() {
    // does nothing.
}
