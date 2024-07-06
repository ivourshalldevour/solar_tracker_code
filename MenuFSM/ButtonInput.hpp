#ifndef BUTTON_INPUT_H
#define BUTTON_INPUT_H

#include <Arduino.h>

// defined in header file. Therefore, accessable outside the ButtonInput code.
typedef enum {none, cycle, select, back} Button;

Button readButtons();

#endif