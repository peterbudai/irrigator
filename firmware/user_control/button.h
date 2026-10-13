/********************************************************************************

Header file for pushbutton input detection.

********************************************************************************/

#pragma once

#include <stdint.h>

// ------------------------------------------------------------- 
// Global state
// -------------------------------------------------------------

// Stores whether a valid button press has been detected.
// Must be reset manually after the button press is handled. If this isn't done,
// subsequent button presses may remain undetected.
extern volatile uint8_t button_pressed;

// ------------------------------------------------------------- 
// Functions
// -------------------------------------------------------------

// Init button control ports and timers.
void buttonInit(void);
// Timer tick callback for periodic button sensing.
void buttonTick(void);