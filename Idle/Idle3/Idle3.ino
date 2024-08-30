/*
    This code is only to be uploaded to Arduino 3.
    This code's only purpose is to pull-down the Mutex lines.
    Otherwise it does nothing. That's the point, this code is only for 
    debugging purposes, so we can isolate the problem.
*/

void setup() {
    // Set pins PB1 and PB0 as inputs. (without pullups)
    DDRB = DDRB & (!((1<<DDB1) | (1<<DDB0)));

    // Set pin PC3 as output.
    PORTC = PORTC & !(1<<PORTC3);   // set to output LOW
    DDRC = DDRC | (1<<DDC3);   // then set as output (avoids accidental HIGH)
}

void loop() {
    // Do nothing
}
