/********************************************************************************

Pushbutton input detection implementation.
The pushbutton is assumed to be connected to a digital GPIO pin, with an
external pull-up resistor. So the button push is an active-low event.

No hardware debounce is required, the button sensing is not edge-triggered,
but checking the input level periodically.

********************************************************************************/

#include "button.h"

#include <stdint.h>
#include <avr/io.h>

// ------------------------------------------------------------- 
// Internal constants
// -------------------------------------------------------------

#define BUTTON_PORT PB1       // Button is connected to PORTB1
#define BUTTON_PRESSED 0      // Button is active-low
#define BUTTON_RELEASED 1     // Button is active-low
#define BUTTON_DEBOUNCE 100   // Debounce time in milliseconds

// ------------------------------------------------------------- 
// State variables
// -------------------------------------------------------------

// Operational state of the two LEDs.
volatile bool button_pressed;
// Last stable state of the pushbutton (1: released, 0: pushed)
volatile uint8_t button_state;
// Number of milliseconds elapsed since changing from stable state.
volatile uint8_t button_timer;

// Init button port and timers.
void buttonInit(void) {
    // Set initial state
    button_pressed = false;
    button_state = BUTTON_RELEASED;
    button_timer = 0;

    // Configure button sense port as input with internal pull-up disabled
    DDRB &= ~_BV(BUTTON_PORT);
    PORTB &= ~_BV(BUTTON_PORT);
}

// Timer tick callback for periodic button sensing and debouncing.
// This function is called once every millisecond, from the timer interrupt routine.
void buttonTick(void) {
    // Sense current button status
    uint8_t sense = PINB & _BV(BUTTON_PORT);

    // Detect change from last known stable state
    if (sense == button_state) {
        // Reset debounce timer if no change, or we returned (bounced) to the stable state.
        button_timer = 0;
        return;
    }

    // On change detected, wait for debounce time
    if (++button_timer < BUTTON_DEBOUNCE) {
        return;
    }

    // Valid change detected, switch to the new stable state
    button_state = sense;
    if (button_state == BUTTON_PRESSED) {
        // If button press event detected, save it
        button_pressed = true;
    }
}