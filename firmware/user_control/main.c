#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include "led.h"
#include "usiTwiSlave.h"

// ------------------------------------------------------------- 
// Constants
// -------------------------------------------------------------

// This will be the I2C address of this slave controller
#define I2C_ADDR 0x0B

// Command byte structure
#define CMD_SET_LED1 0x10
#define CMD_SET_LED2 0x20
#define CMD_ACK_BTN 0x80

// ------------------------------------------------------------- 
// Global data
// -------------------------------------------------------------

volatile uint8_t timer_ms;

// Handle I2C data sent from master. Accepts a single command byte.
//
// Command byte structure:
// Bit 7 6 5 4 3 2 1 0
//     | | \+/ \+/ \+/
//     | |  |   |   +--- LED 1 state
//     | |  |   +------- LED 2 state
//     | |  +----------- LED select bits
//     | +-------------- Unused (ignored, should be 0)
//     +---------------- Reset button status (1: reset, 0: keep)
static void onReceive(uint8_t amount) {
    while(amount--) {
        // Read next command byte from I2C input buffer
        uint8_t command = usiTwiReceiveByte();

        // Execute command
        if(command & CMD_SET_LED1) {
            led_state[0] = (command >> 0) & (uint8_t)LED_MASK;
        }
        if(command & CMD_SET_LED2) {
            led_state[1] = (command >> 2) & (uint8_t)LED_MASK;
        }
        // TODO: button
    }
    wdt_reset();
}

// Handle I2C data read by master. Returns relay status in a single byte.
//
// Status byte structure:
// Bit 7 6 5 4 3 2 1 0
//     | | \+/ \+/ \+/
//     | |  |   |   +--- LED 1 state
//     | |  |   +------- LED 2 state
//     | |  +----------- Unused (set to 0)
//     | +-------------- Button release detected
//     +---------------- Button press detected
static void onRequest(void) {
    uint8_t state = /* TODO: button */ (led_state[1] << 2) | (led_state[0] << 0);
    usiTwiTransmitByte(state);
    wdt_reset();
}

// Handle timer tick at every millisecond
//
ISR(TIM0_COMPA_vect) {
    if(++timer_ms >= 100) {
        timer_ms = 0;
        ledTick();
    }
}

// Main function
int main(void) {
    // -------------------------------------------------------------
    // Init all peripherials
    // -------------------------------------------------------------

    // Init LED control ports PB4 and PB3
    ledInit();

    // I2C inputs on pins PB2 and PB0
    usiTwiSlaveInit(I2C_ADDR, onReceive, onRequest);

    // Button sense port PB1 as input with internal pull-up disabled (external pullup provided)
    DDRB &= ~_BV(PB1);
    PORTB &= ~_BV(PB1);

    // -------------------------------------------------------------
    // Init timer
    // -------------------------------------------------------------

    // -------------------------------------------------------------
    // Main loop - wait for external events and handle them
    // -------------------------------------------------------------
    
    // Enable sleep and interrupts
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    sei();

    // Reset if no I2C command received for 8 seconds
    // Safety measure: if I2C master or bus hangs, better reinit
    wdt_enable(WDTO_8S);
    for (;;) {
        sleep_cpu();
    }
}
