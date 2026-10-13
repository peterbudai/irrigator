/********************************************************************************

Firmware main file for user interface control module.

This firmware turns an ATTiny2/4/85 microcontroller into an I2C slave that
can receive commands to drive a dual-color LED. The LED can signal various
states via the combination of colors and light/blink patterns. The module
can also read a state of an active-low pushbutton and report it back to the
I2C master.

********************************************************************************/

#include <stdint.h>
#include <stdbool.h>
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

// Last I2C register exposed by this module
#define REGISTER_MAX 2

// Which I2C register is currently addressed (set by the last register-select byte)
volatile uint8_t register_select = 0;

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

    if(reg <= REGISTER_MAX) {
        register_select = reg;
    }

    if(amount) {
        // A second byte present in the same batch means it's a write
        uint8_t value = usiTwiReceiveByte();
        amount--;

        if(reg < REGISTER_MAX && value <= LED_STEADY_ON) {
            // Set LED state for the addressed LED register
            // Values must be one of the defined LED states, otherwise ignored
            led_state[register_select] = value;
        }

        if(reg == REGISTER_MAX && value == 0) {
            // Writing 0 acknowledges (clears) a detected press;
            // any other value is ignored since a press can't be forced
            button_pressed = value;
        }
        // Drop the value entirely if the register select was invalid,
        // instead of misapplying it to the previously selected register
    }

    // Drain any unexpected extra bytes so the buffer can't desync
    while(amount--) {
        usiTwiReceiveByte();
    }

    wdt_reset();
}

// Handle I2C data read by master. Returns the state of the currently
// selected register.
//
// This function gets called from the USI interrupt handler.
static void onRequest(void) {
    uint8_t value = 0;

    if(register_select < REGISTER_MAX) {
        value = led_state[register_select];
    } else {
        value = button_pressed;
    }

    usiTwiTransmitByte(value);
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
        if (usiTwiTransactionActive()) {
            // USI has no Stop interrupt, so Stop must be polled; can't sleep
            // through it or a pending write would never get flushed
            usiTwiCheckStop();
        } else {
            sleep_cpu();
        }
    }
}
