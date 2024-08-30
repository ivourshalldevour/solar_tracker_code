#ifndef MUTEX_H
#define MUTEX_H

    // This code file is only to be used by Arduino 2 (adaptive tracker).

/*
    Call this before starting any i2c communication.
    This function will keep busy waiting until the line is free. But
    only for a maximum of 1 second. After this it times out and returns
    a 1 to show arbitration was not successful.
*/
byte startMutex();

/*
    Call this once you have finished i2c communication to release the line.
    Can be called even if startMutex was not called.
    (or if it wasn't successful).
*/
void endMutex();


#endif MUTEX_H