/********************************************************************************

Header file for pushbutton input detection.

********************************************************************************/

#pragma once

#include <stdbool.h>

// ------------------------------------------------------------- 
// Global state
// -------------------------------------------------------------

// Stores whether a valid button press has been detected.
// Must be reset manually after the button press is handled. If this isn't done,
// subsequent button presses may remain undetected.
extern volatile bool button_pressed;

// ------------------------------------------------------------- 
// Functions
// -------------------------------------------------------------

// Init LED control ports and timers.
void buttonInit(void);
// Timer tick callback for periodic button sensing.
void buttonTick(void);