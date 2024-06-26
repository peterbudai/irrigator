/********************************************************************************

Firmware main file for relay control module.

This firmware turns an ATTiny2/4/85 microcontroller into an I2C slave that
can receive commands to control two independent relays via two GPIO pins,
using active-high digital signals.

********************************************************************************/

#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include "usiTwiSlave.h"

// ------------------------------------------------------------- 
// Constants
// -------------------------------------------------------------

// This will be the I2C address of this slave controller
#define I2C_ADDR 0x0A

// Command byte constants
#define CMD_MASK 0xF0
#define CMD_OFF 0x10
#define CMD_ON 0x20
#define CMD_SET 0x30

#define RELAY_MASK 0x03
#define RELAY_1 PB3             // Relay 1 is connected to PORTB3
#define RELAY_2 PB4             // Relay 2 is connected to PORTB4

// ------------------------------------------------------------- 
// Callback functions for I2C communication
// -------------------------------------------------------------

// Handle I2C data sent from master. Accepts a single command byte.
//
// Command byte structure:
// Bit 7 6 5 4 3 2 1 0
//     \--+--/ \+/ | |
//        |     |  | +-- Relay 1 select/state
//        |     |  +---- Relay 2 select/state
//        |     +------- Unused (ignored, should be 0)
//        +------------- Command
//
// This function gets called from the USI interrupt handler.
static void onReceive(uint8_t amount) {
    while(amount--) {
        // Read next command byte from I2C input buffer
        uint8_t command = usiTwiReceiveByte();

        // Execute command
        switch(command & CMD_MASK) {
            case CMD_OFF:
                // Turn the specified relays off
                PORTB &= ~((command & RELAY_MASK) << RELAY_1);
                break;
            case CMD_ON:
                // Turn the specified relays on
                PORTB |= (command & RELAY_MASK) << RELAY_1;
                break;
            case CMD_SET:
                // Set the relays to the specified state (on: 1, off: 0)
                PORTB = (PORTB & ~(_BV(RELAY_2) | _BV(RELAY_1))) | ((command & RELAY_MASK) << RELAY_1);
                break;
            default:
                // Unknown or ping command, do nothing
                break;
        }
    }
    wdt_reset();
}

// Handle I2C data read by master. Returns relay status in a single byte.
//
// Status byte structure:
// Bit 7 6 5 4 3 2 1 0
//     \----+----/ | |
//          |      | +-- Relay 1 state (on: 1, off: 0)
//          |      +---- Relay 2 state (on: 1, off: 0)
//          +----------- Unused (set to 0)
//
// This function gets called from the USI interrupt handler.
static void onRequest(void) {
    uint8_t state = (PORTB & (_BV(RELAY_2) | _BV(RELAY_1))) >> RELAY_1;
    usiTwiTransmitByte(state);
    wdt_reset();
}

// -------------------------------------------------------------
// Initialization and main event loop
// -------------------------------------------------------------

// The one and only main function.
// This is called upon a reset condition and never returns.
int main(void) {
    //
    // Init all peripherials
    //

    // Relay control ports PB3 and PB4 as outputs (initially LOW)
    DDRB |= _BV(RELAY_2) | _BV(RELAY_1);
    PORTB &= ~(_BV(RELAY_2) | _BV(RELAY_1));

    // I2C inputs on pins PB2 and PB0
    usiTwiSlaveInit(I2C_ADDR, onReceive, onRequest);

    // Unused pin PB1 as input with internal pull-up enabled
    DDRB &= ~_BV(PB1);
    PORTB |= _BV(PB1);

    //
    // Main loop - wait for external events and handle them
    //
    
    // Enable sleep and interrupts
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    sei();

    // Reset if no I2C communication occured for 8 seconds
    // Safety measure: if I2C master or bus hangs, better turn off relays and reinit
    wdt_enable(WDTO_8S);
    for (;;) {
        sleep_cpu();
    }
}
