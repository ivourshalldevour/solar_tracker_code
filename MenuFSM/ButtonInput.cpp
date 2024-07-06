#include "ButtonInput.hpp"

Button readButtons() {
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
    byte c_current = digitalRead(8);
    byte s_current = digitalRead(7);
    byte b_current = digitalRead(4);

    Button pressed;

    static byte c_prev = 0;
    static byte s_prev = 0;
    static byte b_prev = 0;

    // this is used to detect rising edge.
    if(c_current==1 && c_prev==0)
        pressed = cycle;
    else if(s_current==1 && s_prev==0)
        pressed = select;
    else if(b_current==1 && b_prev==0)
        pressed = back;
    else
        pressed = none;

    c_prev = c_current;
    s_prev = s_current;
    b_prev = b_current;

    return pressed;
}