#include "secondPage.h"

extern hd44780_I2Cexp lcd; 

void badFunc(void) {
    // error: 'lcd' was not declared in this scope
    lcd.print("Hello World");
}