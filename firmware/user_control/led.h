#pragma once

#include <stdint.h>

// LED state constants
#define LED_MASK 0x03
#define LED_STEADY_OFF 0x00
#define LED_BLINK_SLOW 0x01
#define LED_BLINK_FAST 0x02
#define LED_STEADY_ON 0x03

extern volatile uint8_t led_state[2];

void ledInit(void);
void ledTick(void);