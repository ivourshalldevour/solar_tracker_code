#ifndef BUTTON_INPUT_H
#define BUTTON_INPUT_H

#include <Arduino.h>

typedef enum {none, cycle, select, back} Button;

Button readButtons();

ISR(PCINT2_vect);

#endif