#include "ButtonInput.hpp"

// defined in main .ino file
extern volatile byte keyboard_interrupt;

Button readButtons() {
    byte c_current = digitalRead(3);
    byte s_current = digitalRead(7);
    byte b_current = digitalRead(4);

    Button pressed;

    static byte c_prev = c_current;
    static byte s_prev = s_current;
    static byte b_prev = b_current;

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


ISR(PCINT2_vect) {
    if((PIND & 0b10011000) != 0) {
        keyboard_interrupt = 1;    // rising edge on any of the CSB buttons
    }
    else keyboard_interrupt = 0;    // falling edge (ignore this interrupt)
}

