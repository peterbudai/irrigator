/********************************************************************************

Header file for LED output control.

********************************************************************************/

#pragma once

#include <stdint.h>

// ------------------------------------------------------------- 
// Constants
// -------------------------------------------------------------

// Valid LED states
#define LED_MASK 0x03
#define LED_STEADY_OFF 0x00
#define LED_BLINK_SLOW 0x01
#define LED_BLINK_FAST 0x02
#define LED_STEADY_ON 0x03

// ------------------------------------------------------------- 
// Global state
// -------------------------------------------------------------

// Status of the two LEDs - which mode do they operate in.
// Valid values are the constants above.
extern volatile uint8_t led_state[2];

// ------------------------------------------------------------- 
// Functions
// -------------------------------------------------------------

// Init LED control ports and timers.
void ledInit(void);
// LED timer tick callback.
void ledTick(void);