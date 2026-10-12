/********************************************************************************

Firmware main file for relay control module.

This firmware turns an ATTiny2/4/85 microcontroller into an I2C slave that
can receive commands to control two independent relays via two GPIO pins,
using active-high digital signals.

********************************************************************************/

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include "usiTwiSlave.h"

// ------------------------------------------------------------- 
// Constants
// -------------------------------------------------------------

// This will be the I2C address of this slave controller
#define I2C_ADDR 0x0A
// How many relays are controlled via I2C registers
#define RELAY_COUNT 2

typedef struct {
    // Desired state of the relay (1: on, 0: off)
    uint8_t state;
    // Bitmask for controlling the relay pin on PORTB
    uint8_t mask;
} relay_t;

// Relay configuration
static relay_t relays[RELAY_COUNT] = {
    {0, _BV(PB3)},              // Relay 0 is connected to PORTB3
    {0, _BV(PB4)},              // Relay 1 is connected to PORTB4
};

// Which I2C register is currently addressed (set by the last register-select byte)
static uint8_t relay_select = 0;

// ------------------------------------------------------------- 
// Callback functions for I2C communication
// -------------------------------------------------------------

// Handle I2C data sent from master: register-select (1 byte) or register
// write (2 bytes: register, value). STOP-triggered flushing means amount is
// normally 1 or 2; any further bytes are drained (not applied) so a stray
// extra byte can't permanently misalign the next message.
//
// This function gets called from the USI interrupt handler.
static void onReceive(uint8_t amount) {
    // First byte is always the register select
    uint8_t reg = usiTwiReceiveByte();
    amount--;

    bool valid_reg = reg < RELAY_COUNT;
    if(valid_reg) {
        relay_select = reg;
    }

    if(amount) {
        // A second byte present in the same batch means it's a write
        uint8_t value = usiTwiReceiveByte();
        amount--;

        // Drop the value entirely if the register select was invalid,
        // instead of misapplying it to the previously selected register
        if(valid_reg && value <= 1) {
            relays[relay_select].state = value;

            if(value) {
                PORTB |= relays[relay_select].mask;
            } else {
                PORTB &= ~relays[relay_select].mask;
            }
        }
    }

    // Drain any unexpected extra bytes so the buffer can't desync
    while(amount--) {
        usiTwiReceiveByte();
    }

    wdt_reset();
}

// Handle I2C data read by master. Returns the state of the currently
// selected relay register.
//
// This function gets called from the USI interrupt handler.
static void onRequest(void) {
    usiTwiTransmitByte(relays[relay_select].state);
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
    for(uint8_t i = 0; i < RELAY_COUNT; i++) {
        DDRB |= relays[i].mask;
        PORTB &= ~relays[i].mask;
    }

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
        if (usiTwiTransactionActive()) {
            // USI has no Stop interrupt, so Stop must be polled; can't sleep
            // through it or a pending write would never get flushed
            usiTwiCheckStop();
        } else {
            sleep_cpu();
        }
    }
}
