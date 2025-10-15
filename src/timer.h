#ifndef TIMER_H
#define TIMER_H

#include <stdint.h> // Required for the uint32_t type

// Initializes the 1ms system timer.
// Must be called once in your main setup.
void timer_init(void);

// Returns the number of milliseconds that have passed since the program started.
// This function is safe to call from anywhere in your code.
uint32_t millis(void);

#endif // TIMER_H