/********************************************************************************

LED output control implementation.

This code assumes a dual-color LED (or any two LEDs, basically), connected to
two GPIO pins, with active-high operations. It supports four different light
patterns for each LED, from steady off, through blink, up to steady on. This
way, we can signal various states via the combination of colors and light/blink
patterns.

The two LEDs' light patterns are designed so they are not overlapping:

  Pattern   |                       Time (s)
    name    | 0         1         2         3         4         5
------------+-----------------------------------------------------
STEADY_ON   | PPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPP
BLINK_FAST  | RRRRRBBBBBRRRRRBBBBBRRRRRBBBBBRRRRRBBBBBRRRRRBBBBB
BLINK_SLOW  | R.........B.......................................
STEADY_OFF  | ..................................................

Pattern repeats after a period of 5 seconds. Letters show the color visible
at each 1/10th second time slice (R: red, B: blue, P: purple).

********************************************************************************/

#include "led.h"

#include <avr/io.h>

// ------------------------------------------------------------- 
// Internal constants
// -------------------------------------------------------------

#define LED_1 PB3             // LED 1 (red) is connected to PORTB3
#define LED_2 PB4             // LED 2 (blue) is connected to PORTB4

// ------------------------------------------------------------- 
// State variables
// -------------------------------------------------------------

// Operational state of the two LEDs.
volatile uint8_t led_state[2];
// LED timer counter.
// Incremented in every 1/10th seconds.
volatile uint8_t led_timer;

// Init LED control ports and timers.
void ledInit(void) {
    // Set initial state
    led_state[0] = LED_STEADY_OFF;
    led_state[1] = LED_STEADY_OFF;
    led_timer = 0;

    // LED control ports PB3 and PB4 as outputs (initially LOW)
    DDRB |= _BV(LED_2) | _BV(LED_1);
    PORTB &= ~(_BV(LED_2) | _BV(LED_1));
}

// LED timer tick callback.
// This function is called once every 1/10th second, from the timer interrupt routine.
void ledTick(void) {
    // Function is called once every 100 milliseconds, so this counter
    // covers exactly 50x0.1=5 seconds, this is the period of the LED
    // light patterns.
    if (++led_timer >= 50) {
        led_timer = 0;
    };

    // Light pattern for the first LED
    switch (led_state[0]) {
        case LED_STEADY_ON:
            PORTB |= _BV(LED_1);                // 0..4999 ON
            break;
        case LED_BLINK_FAST:
            // Binary search, to minimize the number of comparisons
            if (led_timer < 25) {
                if (led_timer < 10) {
                    if (led_timer < 5) {
                        PORTB |= _BV(LED_1);    // 0..499 ON
                    } else {
                        PORTB &= ~_BV(LED_1);   // 500..999 OFF
                    }
                } else {
                    if (led_timer < 15) {
                        PORTB |= _BV(LED_1);    // 1000..1499 ON
                    } else if (led_timer < 20) {
                        PORTB &= ~_BV(LED_1);   // 1500..1999 OFF
                    } else {
                        PORTB |= _BV(LED_1);    // 2000..2499 ON
                    }
                }
            } else {
                if (led_timer < 40) {
                    if (led_timer < 30) {
                        PORTB &= ~_BV(LED_1);   // 2500..2999 OFF
                    } else if (led_timer < 35) {
                        PORTB |= _BV(LED_1);    // 3000..3499 ON
                    } else {
                        PORTB &= ~_BV(LED_1);   // 3500..3999 OFF
                    }
                } else {
                    if (led_timer < 45) {
                        PORTB |= _BV(LED_1);    // 4000..4499 ON
                    } else {
                        PORTB &= ~_BV(LED_1);   // 4500..4999 OFF
                    }
                }
            }
            break;
        case LED_BLINK_SLOW:
            if (led_timer < 1) {
                PORTB |= _BV(LED_1);            // 0..99 ON
            } else {
                PORTB &= ~_BV(LED_1);           // 100..4999 OFF
            }
            break;
        default:
            PORTB &= ~_BV(LED_1);               // 0..4999 OFF
            break;
    }

    // Light pattern for the second LED
    switch(led_state[1]) {
        case LED_STEADY_ON:
            PORTB |= _BV(LED_2);                // 0..4999 ON
            break;
        case LED_BLINK_FAST:
            // Binary search, to minimize the number of comparisons
            if (led_timer < 25) {
                if (led_timer < 10) {
                    if (led_timer < 5) {
                        PORTB &= ~_BV(LED_2);   // 0..499 OFF
                    } else {
                        PORTB |= _BV(LED_2);    // 500..999 ON
                    }
                } else {
                    if (led_timer < 15) {
                        PORTB &= ~_BV(LED_2);   // 1000..1499 OFF
                    } else if (led_timer < 20) {
                        PORTB |= _BV(LED_2);    // 1500..1999 ON
                    } else {
                        PORTB &= ~_BV(LED_2);   // 2000..2499 OFF
                    }
                }
            } else {
                if (led_timer < 40) {
                    if (led_timer < 30) {
                        PORTB |= _BV(LED_2);    // 2500..2999 ON
                    } else if (led_timer < 35) {
                        PORTB &= ~_BV(LED_2);   // 3000..3499 OFF
                    } else {
                        PORTB |= _BV(LED_2);    // 3500..3999 ON
                    }
                } else {
                    if (led_timer < 45) {
                        PORTB &= ~_BV(LED_2);   // 4000..4499 OFF
                    } else {
                        PORTB |= _BV(LED_2);    // 4500..4999 ON
                    }
                }
            }
            break;
        case LED_BLINK_SLOW:
            if (led_timer < 10) {                // 0..999 OFF
                PORTB &= ~_BV(LED_2);
            } else if (led_timer < 11) {        // 1000..1099 ON
                PORTB |= _BV(LED_2);
            } else {
                PORTB &= ~_BV(LED_2);           // 1100..4999 OFF
            }
            break;
        case LED_STEADY_OFF:
        default:
            PORTB &= ~_BV(LED_2);               // 0..4999 OFF
            break;
    }
}