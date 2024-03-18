#include "led.h"

#include <avr/io.h>

#define LED_1 PB3             // LED 1 (red) is connected to PORTB3
#define LED_2 PB4             // LED 2 (blue) is connected to PORTB4

volatile uint8_t led_state[2];
volatile uint8_t led_counter;

void ledInit(void) {
    // Set initial state
    led_state[0] = LED_STEADY_ON;
    led_state[1] = LED_STEADY_ON;
    led_counter = 0;

    // LED control ports PB3 and PB4 as outputs (initially LOW)
    DDRB |= _BV(LED_2) | _BV(LED_1);
    PORTB &= ~(_BV(LED_2) | _BV(LED_1));
}

void ledTick(void) {
    if(++led_counter >= 50) {
        led_counter = 0;
    };

    switch(led_state[0]) {
        case LED_STEADY_ON:
            PORTB |= _BV(LED_1);
            break;
        case LED_BLINK_FAST:
            if(led_counter < 25) {
                if(led_counter < 10) {
                    if(led_counter < 5) {
                        PORTB |= _BV(LED_1);    // 0..499 ON
                    } else {
                        PORTB &= ~_BV(LED_1);   // 500..999 OFF
                    }
                } else {
                    if(led_counter < 15) {
                        PORTB |= _BV(LED_1);    // 1000..1499 ON
                    } else if(led_counter < 20) {
                        PORTB &= ~_BV(LED_1);   // 1500..1999 OFF
                    } else {
                        PORTB |= _BV(LED_1);    // 2000..2499 ON
                    }
                }
            } else {
                if(led_counter < 40) {
                    if(led_counter < 30) {
                        PORTB &= ~_BV(LED_1);   // 2500..2999 OFF
                    } else if(led_counter < 35) {
                        PORTB |= _BV(LED_1);    // 3000..3499 ON
                    } else {
                        PORTB &= ~_BV(LED_1);   // 3500..3999 OFF
                    }
                } else {
                    if(led_counter < 45) {
                        PORTB |= _BV(LED_1);    // 4000..4499 ON
                    } else {
                        PORTB &= ~_BV(LED_1);   // 4500..4999 OFF
                    }
                }
            }
            break;
        case LED_BLINK_SLOW:
            if(led_counter < 1) {
                PORTB |= _BV(LED_1);
            } else {
                PORTB &= ~_BV(LED_1);
            }
            break;
        default:
            PORTB &= ~_BV(LED_1);
            break;
    }

    switch(led_state[1]) {
        case LED_STEADY_ON:
            PORTB |= _BV(LED_2);
            break;
        case LED_BLINK_FAST:
            if(led_counter < 25) {
                if(led_counter < 10) {
                    if(led_counter < 5) {
                        PORTB &= ~_BV(LED_2);   // 0..499 OFF
                    } else {
                        PORTB |= _BV(LED_2);    // 500..999 ON
                    }
                } else {
                    if(led_counter < 15) {
                        PORTB &= ~_BV(LED_2);   // 1000..1499 OFF
                    } else if(led_counter < 20) {
                        PORTB |= _BV(LED_2);    // 1500..1999 ON
                    } else {
                        PORTB &= ~_BV(LED_2);   // 2000..2499 OFF
                    }
                }
            } else {
                if(led_counter < 40) {
                    if(led_counter < 30) {
                        PORTB |= _BV(LED_2);    // 2500..2999 ON
                    } else if(led_counter < 35) {
                        PORTB &= ~_BV(LED_2);   // 3000..3499 OFF
                    } else {
                        PORTB |= _BV(LED_2);    // 3500..3999 ON
                    }
                } else {
                    if(led_counter < 45) {
                        PORTB &= ~_BV(LED_2);   // 4000..4499 OFF
                    } else {
                        PORTB |= _BV(LED_2);    // 4500..4999 ON
                    }
                }
            }
            break;
        case LED_BLINK_SLOW:
            if(led_counter < 10) {
                PORTB &= ~_BV(LED_2);
            } else if (led_counter < 11) {
                PORTB |= _BV(LED_2);
            } else {
                PORTB &= ~_BV(LED_2);
            }
            break;
        default:
            PORTB &= ~_BV(LED_2);
            break;
    }
}