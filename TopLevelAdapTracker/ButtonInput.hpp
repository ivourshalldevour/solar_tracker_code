#ifndef BUTTON_INPUT_H
#define BUTTON_INPUT_H

#include <Arduino.h>

typedef enum {none, cycle, select, back} Button;

/*
    A function that performs rising edge detection on the three input buttons:
        Cycle   (c)
        Select  (s)
        Back    (b)
    Returns the Button enum type. No debouncing is done in software.
    Assumes:
        - only one button is pressed at a time.
        - input pins are already configured as inputs with external pullups.
        - it is called from within a loop.
*/
Button readButtons();

ISR(PCINT2_vect);

#endif