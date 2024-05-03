/********************************************************************************

Firmware main file for user interface control module.

This firmware turns an ATTiny2/4/85 microcontroller into an I2C slave that
can receive commands to drive a dual-color LED. The LED can signal various
states via the combination of colors and light/blink patterns. The module
can also read a state of an active-low pushbutton and report it back to the
I2C master.

********************************************************************************/

#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include "button.h"
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
// Callback functions for I2C communication
// -------------------------------------------------------------

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
//
// This function gets called from the USI interrupt handler.
static void onReceive(uint8_t amount) {
    while (amount--) {
        // Read next command byte from I2C input buffer
        uint8_t command = usiTwiReceiveByte();

        // Execute command
        if (command & CMD_SET_LED1) {
            led_state[0] = (command >> 0) & (uint8_t)LED_MASK;
        }
        if (command & CMD_SET_LED2) {
            led_state[1] = (command >> 2) & (uint8_t)LED_MASK;
        }
        if (command & CMD_ACK_BTN) {
            button_pressed = false;
        }
    }
    wdt_reset();
}

// Handle I2C data read by master. Returns relay status in a single byte.
//
// Status byte structure:
// Bit 7 6 5 4 3 2 1 0
//     | \-+-/ \+/ \+/
//     |   |    |   +--- LED 1 state
//     |   |    +------- LED 2 state
//     |   +------------ Unused (set to 0)
//     +---------------- Button press detected
//
// This function gets called from the USI interrupt handler.
static void onRequest(void) {
    uint8_t state = (button_pressed ? (1 << 7) : 0) | (led_state[1] << 2) | (led_state[0] << 0);
    usiTwiTransmitByte(state);
    wdt_reset();
}

// -------------------------------------------------------------
// Initialization and main event loop
// -------------------------------------------------------------

// Counter for elapsed milliseconds.
// Increased by one via a timer interrupt once every millisecond.
volatile uint8_t timer_ms;

// Handle timer tick interrupt.
// Called once at every millisecond.
ISR(TIM0_COMPA_vect) {
    // LED code only needs to run once in every 1/10th second.
    if (++timer_ms >= 100) {
        timer_ms = 0;
        ledTick();
    }

    // Periodic button sensing and debouncing
    buttonTick();
}

// The main function.
// This is called upon a reset condition and never returns.
int main(void) {
    //
    // Init all peripherials
    //

    // Init LED control ports PB4, PB3 and button sense ports PB1
    ledInit();
    buttonInit();

    // I2C inputs on pins PB2 and PB0
    usiTwiSlaveInit(I2C_ADDR, onReceive, onRequest);

    //
    // Init timer
    //

    // Start at 0
    TCNT0 = 0;
    // 1000 Hz (8000000/((124+1)*64))
    OCR0A = 124;
    // CTC
    TCCR0A = (1 << WGM01);
    // Prescaler 64
    TCCR0B = (1 << CS01) | (1 << CS00);
    // Output Compare Match A Interrupt Enable
    TIMSK = (1 << OCIE0A);

    //
    // Main loop - wait for external events and handle them
    //
    
    // Enable sleep and interrupts
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    sei();

    // Reset if no I2C communication occured for 8 seconds
    // Safety measure: if I2C master or bus hangs, better reinit to a known state
    wdt_enable(WDTO_8S);
    for (;;) {
        sleep_cpu();
    }
}
