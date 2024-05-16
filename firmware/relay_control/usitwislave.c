/*	See LICENSE for Copyright etc. */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include "usitwislave.h"

#if	defined(__AVR_ATtiny2313__)
#	define DDR_USI				DDRB
#	define PORT_USI				PORTB
#	define PIN_USI				PINB
#	define PORT_USI_SDA			PB5
#	define PORT_USI_SCL			PB7
#	define PIN_USI_SDA			PINB5
#	define PIN_USI_SCL			PINB7
#	define USI_OVERFLOW_VECTOR	USI_OVERFLOW_vect
#endif

#if	defined(__AVR_ATtiny24__) | \
	defined(__AVR_ATtiny44__) | \
	defined(__AVR_ATtiny84__)
#	define DDR_USI				DDRA
#	define PORT_USI				PORTA
#	define PIN_USI				PINA
#	define PORT_USI_SDA			PA6
#	define PORT_USI_SCL			PA4
#	define PIN_USI_SDA			PINA6
#	define PIN_USI_SCL			PINA4
#	define USI_OVERFLOW_VECTOR	USI_OVF_vect
#endif

#if	defined(__AVR_ATtiny25__) | \
	defined(__AVR_ATtiny45__) | \
	defined(__AVR_ATtiny85__)
#	define DDR_USI				DDRB
#	define PORT_USI				PORTB
#	define PIN_USI				PINB
#	define PORT_USI_SDA			PB0
#	define PORT_USI_SCL			PB2
#	define PIN_USI_SDA			PINB0
#	define PIN_USI_SCL			PINB2
#	define USI_OVERFLOW_VECTOR	USI_OVF_vect
#endif

#if defined(__AVR_ATtiny26__)
#	define DDR_USI				DDRB
#	define PORT_USI				PORTB
#	define PIN_USI				PINB
#	define PORT_USI_SDA			PB0
#	define PORT_USI_SCL			PB2
#	define PIN_USI_SDA			PINB0
#	define PIN_USI_SCL			PINB2
#	define USI_OVERFLOW_VECTOR	USI_OVF_vect
#endif

#if	defined(__AVR_ATtiny261__) | \
	defined(__AVR_ATtiny261a__) | \
	defined(__AVR_ATtiny461__) | \
	defined(__AVR_ATtiny461a__) | \
	defined(__AVR_ATtiny861__) | \
	defined(__AVR_ATtiny861a__)
#	if defined(USI_ON_PORT_A)
#		define DDR_USI				DDRA
#		define PORT_USI				PORTA
#		define PIN_USI				PINA
#		define PORT_USI_SDA			PA0
#		define PORT_USI_SCL			PA2
#		define PIN_USI_SDA			PINA0
#		define PIN_USI_SCL			PINA2
#	else
#		define DDR_USI				DDRB
#		define PORT_USI				PORTB
#		define PIN_USI				PINB
#		define PORT_USI_SDA			PB0
#		define PORT_USI_SCL			PB2
#		define PIN_USI_SDA			PINB0
#		define PIN_USI_SCL			PINB2
#	endif
#	define USI_OVERFLOW_VECTOR		USI_OVF_vect
#endif

#if	defined(__AVR_ATmega165__) | \
	defined(__AVR_ATmega325__) | \
	defined(__AVR_ATmega3250__) | \
	defined(__AVR_ATmega645__) | \
	defined(__AVR_ATmega6450__) | \
	defined(__AVR_ATmega329__) | \
	defined(__AVR_ATmega3290__)
#	define DDR_USI				DDRE
#	define PORT_USI				PORTE
#	define PIN_USI				PINE
#	define PORT_USI_SDA			PE5
#	define PORT_USI_SCL			PE4
#	define PIN_USI_SDA			PINE5
#	define PIN_USI_SCL			PINE4
#	define USI_OVERFLOW_VECTOR	USI_OVERFLOW_vect
#endif

#if defined(__AVR_ATmega169__)
#	define DDR_USI				DDRE
#	define PORT_USI				PORTE
#	define PIN_USI				PINE
#	define PORT_USI_SDA			PE5
#	define PORT_USI_SCL			PE4
#	define PIN_USI_SDA			PINE5
#	define PIN_USI_SCL			PINE4
#	define USI_OVERFLOW_VECTOR	USI_OVERFLOW_vect
#endif

enum
{
	OF_STATE_CHECK_ADDRESS,
	OF_STATE_SEND_DATA,
	OF_STATE_REQUEST_ACK,
	OF_STATE_CHECK_ACK,
	OF_STATE_RECEIVE_DATA,
	OF_STATE_STORE_DATA_AND_SEND_ACK
} overflow_state_t;

enum
{
	SS_STATE_BEFORE_START,
	SS_STATE_AFTER_START,
	SS_STATE_ADDRESS_SELECTED,
	SS_STATE_ADDRESS_NOT_SELECTED,
	SS_STATE_DATA_PROCESSED
} startstop_state_t;

static void (*idle_callback)(void);
static void	(*data_callback)(uint8_t input_buffer_length, const uint8_t *input_buffer,
						uint8_t *output_buffer_length, uint8_t *output_buffer);

static uint8_t of_state;
static uint8_t ss_state;

static uint8_t	slave_address;

static uint8_t	input_buffer[USI_TWI_BUFFER_SIZE];
static uint8_t	input_buffer_length;
static uint8_t	output_buffer[USI_TWI_BUFFER_SIZE];
static uint8_t	output_buffer_length;
static uint8_t	output_buffer_current;

#if defined(CONFIG_USI_TWI_STATS)
static bool	stats_enabled;
static uint16_t	start_conditions_count;
static uint16_t	stop_conditions_count;
static uint16_t	error_conditions_count;
static uint16_t	overflow_conditions_count;
static uint16_t	local_frames_count;
static uint16_t	idle_call_count;
#endif

static void set_sda_to_input(void)
{
	DDR_USI &= ~_BV(PORT_USI_SDA);
}

static void set_sda_to_output(void)
{
	DDR_USI |= _BV(PORT_USI_SDA);
}

static inline void set_scl_to_input(void)
{
	DDR_USI &= ~_BV(PORT_USI_SCL);
}

static inline void set_scl_to_output(void)
{
	DDR_USI |= _BV(PORT_USI_SCL);
}

static inline void set_sda_low(void)
{
	PORT_USI &= ~_BV(PORT_USI_SDA);
}

static inline void set_sda_high(void)
{
	PORT_USI |= _BV(PORT_USI_SDA);
}

static inline void set_scl_low(void)
{
	PORT_USI &= ~_BV(PORT_USI_SCL);
}

static inline void set_scl_high(void)
{
	PORT_USI |= _BV(PORT_USI_SCL);
}

static inline void twi_reset_state(void)
{
	USISR =
		(1		<< USISIF)	|		// clear start condition flag
		(1		<< USIOIF)	|		// clear overflow condition flag
		(0		<< USIPF)	|		// !clear stop condition flag
		(1		<< USIDC)	|		// clear arbitration error flag
		(0x00	<< USICNT0);		// set counter to "8" bits

	USICR =
		(1 << USISIE) |									// enable start condition interrupt
		(0 << USIOIE) |									// !enable overflow interrupt
		(1 << USIWM1) | (0 << USIWM0) |					// set usi in two-wire mode, disable bit counter overflow hold
		(1 << USICS1) | (0 << USICS0) | (0 << USICLK) |	// shift register clock source = external, positive edge, 4-bit counter source = external, both edges
		(0 << USITC);									// don't toggle clock-port pin
}

static void twi_reset(void)
{
	// make sure no sda/scl remains pulled up or down

	set_sda_to_input();		//	deactivate internal pullup on sda/scl
	set_sda_low();
	set_scl_to_input();
	set_scl_low();

	set_sda_to_output();	//	release (set high) on sda/scl
	set_sda_high();
	set_sda_to_input();
	set_sda_high();
	set_scl_to_output();
	set_scl_high();

	twi_reset_state();
}

static inline void twi_init(void)
{
#if defined(USIPP)
#if  defined(USI_ON_PORT_A)
	USIPP |= _BV(USIPOS);
#else
	USIPP &= ~_BV(USIPOS);
# endif
#endif

	twi_reset();
}

ISR(USI_START_vect)
{
	set_sda_to_input();

	// wait for SCL to go low to ensure the start condition has completed (the
	// start detector will hold SCL low) - if a stop condition arises then leave
	// the interrupt to prevent waiting forever - don't use USISR to test for stop
	// condition as in Application Note AVR312 because the stop condition Flag is
	// going to be set from the last TWI sequence

	while(!(PIN_USI & _BV(PIN_USI_SDA)) &&
			(PIN_USI & _BV(PIN_USI_SCL)))

	// possible combinations
	//	sda = low	scl = low		break	start condition
	// 	sda = low	scl = high		loop
	//	sda = high	scl = low		break	stop condition
	//	sda = high	scl = high		break	stop condition

	if((PIN_USI & _BV(PIN_USI_SDA)))	// stop condition
	{
		twi_reset();

#if defined(CONFIG_USI_TWI_STATS)
		if(stats_enabled)
			error_conditions_count++;
#endif
		return;
	}

#if defined(CONFIG_USI_TWI_STATS)
	if(stats_enabled)
		start_conditions_count++;
#endif

	of_state = OF_STATE_CHECK_ADDRESS;
	ss_state = SS_STATE_AFTER_START;

	USIDR = 0xff;

	USICR =
		(1 << USISIE) |									// enable start condition interrupt
		(1 << USIOIE) |									// enable overflow interrupt
		(1 << USIWM1) | (1 << USIWM0) |					// set usi in two-wire mode, enable bit counter overflow hold
		(1 << USICS1) | (0 << USICS0) | (0 << USICLK) |	// shift register clock source = external, positive edge, 4-bit counter source = external, both edges
		(0 << USITC);									// don't toggle clock-port pin

	USISR =
		(1		<< USISIF)	|		// clear start condition flag
		(1		<< USIOIF)	|		// clear overflow condition flag
		(0		<< USIPF)	|		// !clear stop condition flag
		(1		<< USIDC)	|		// clear arbitration error flag
		(0x00	<< USICNT0);		// set counter to "8" bits
}

ISR(USI_OVERFLOW_VECTOR)
{
	// bit shift register overflow condition occured
	// scl forced low until overflow condition is cleared!

	uint8_t data		= USIDR;
	uint8_t set_counter = 0x00;		// send 8 bits (16 edges)

#if defined(CONFIG_USI_TWI_STATS)
	if(stats_enabled)
		overflow_conditions_count++;
#endif

again:
	switch(of_state)
	{
		// start condition occured and succeed
		// check address, if not OK, reset usi
		// note: not using general call address

		case(OF_STATE_CHECK_ADDRESS):
		{
			uint8_t address;
			uint8_t direction;

			direction	= data & 0x01;
			address		= (data & 0xfe) >> 1;

			if(address == slave_address)
			{
				ss_state = SS_STATE_ADDRESS_SELECTED;

				if(direction)					// read request from master
					of_state = OF_STATE_SEND_DATA;
				else							// write request from master
					of_state = OF_STATE_RECEIVE_DATA;

				USIDR		= 0x00;
				set_counter = 0x0e;				// send 1 bit (2 edges)
				set_sda_to_output();			// initiate send ack
			}
			else
			{
				USIDR		= 0x00;
				set_counter = 0x00;
				twi_reset_state();
				ss_state = SS_STATE_ADDRESS_NOT_SELECTED;
			}

			break;
		}

		// process read request from master

		case(OF_STATE_SEND_DATA):
		{
			ss_state = SS_STATE_DATA_PROCESSED;
			of_state = OF_STATE_REQUEST_ACK;

			if(output_buffer_current < output_buffer_length)
				USIDR = output_buffer[output_buffer_current++];
			else
				USIDR = 0x00;					// no more data, but cannot send "nothing" or "nak"

			set_counter = 0x00;
			set_sda_to_output();				// initiate send data

			break;
		}

		// data sent to master, request ack (or nack) from master

		case(OF_STATE_REQUEST_ACK):
		{
			of_state = OF_STATE_CHECK_ACK;

			USIDR		= 0x00;
			set_counter = 0x0e;					//	receive 1 bit (2 edges)
			set_sda_to_input();					//	initiate receive ack

			break;
		}

		// ack/nack from master received

		case(OF_STATE_CHECK_ACK):
		{
			if(data)	// if NACK, the master does not want more data
			{
				of_state = OF_STATE_CHECK_ADDRESS;
				set_counter = 0x00;
				twi_reset();
			}
			else
			{
				of_state = OF_STATE_SEND_DATA;
				goto again;	// from here we just drop straight into state_send_data
			}				// don't wait for another overflow interrupt

			break;
		}

		// process write request from master

		case(OF_STATE_RECEIVE_DATA):
		{
			ss_state = SS_STATE_DATA_PROCESSED;

			of_state = OF_STATE_STORE_DATA_AND_SEND_ACK;

			set_counter = 0x00;					// receive 1 bit (2 edges)
			set_sda_to_input();					// initiate receive data

			break;
		}

		// data received from master, store it and wait for more data

		case(OF_STATE_STORE_DATA_AND_SEND_ACK):
		{
			of_state = OF_STATE_RECEIVE_DATA;

			if(input_buffer_length < (USI_TWI_BUFFER_SIZE - 1))
				input_buffer[input_buffer_length++] = data;

			USIDR		= 0x00;
			set_counter = 0x0e;					// send 1 bit (2 edges)
			set_sda_to_output();				// initiate send ack

			break;
		}
	}

	USISR =
		(0				<< USISIF)	|		// don't clear start condition flag
		(1				<< USIOIF)	|		// clear overflow condition flag
		(0				<< USIPF)	|		// don't clear stop condition flag
		(1				<< USIDC)	|		// clear arbitration error flag
		(set_counter	<< USICNT0);		// set counter to 8 or 1 bits
}

void usi_twi_slave(uint8_t slave_address_in, bool use_sleep,
	usi_twi_data_callback_t data_callback_in, usi_twi_idle_callback_t idle_callback_in)
{
	bool call_datacallback = false;

	slave_address			= slave_address_in;
	data_callback			= data_callback_in;
	idle_callback			= idle_callback_in;

	input_buffer_length		= 0;
	output_buffer_length	= 0;
	output_buffer_current	= 0;
	ss_state				= SS_STATE_BEFORE_START;

	if(use_sleep)
		set_sleep_mode(SLEEP_MODE_IDLE);

	twi_init();

	sei();

	for(;;)
	{
		if(idle_callback)
		{
			idle_callback();

#if defined(CONFIG_USI_TWI_STATS)
			if(stats_enabled)
				idle_call_count++;
#endif
		}

		if(use_sleep && (ss_state == SS_STATE_BEFORE_START))
			sleep_mode();

		if(USISR & _BV(USIPF))
		{
			cli();

#if defined(CONFIG_USI_TWI_STATS)
			if(stats_enabled)
				stop_conditions_count++;
#endif

			USISR |= _BV(USIPF);	// clear stop condition flag

			switch(ss_state)
			{
				case(SS_STATE_AFTER_START):
				{
					twi_reset();
					break;
				}

				case(SS_STATE_DATA_PROCESSED):
				{
#if defined(CONFIG_USI_TWI_STATS)
					if(stats_enabled)
						local_frames_count++;
#endif
					call_datacallback = true;

					break;
				}
			}

			ss_state = SS_STATE_BEFORE_START;

			sei();
		}

		if(call_datacallback)
		{
			output_buffer_length	= 0;
			output_buffer_current	= 0;
			data_callback(input_buffer_length, input_buffer, &output_buffer_length, output_buffer);
			input_buffer_length		= 0;
			call_datacallback		= false;
		}
	}
}

#if defined(CONFIG_USI_TWI_STATS)

void usi_twi_enable_stats(bool onoff)
{
	stats_enabled				= onoff;
	start_conditions_count		= 0;
	stop_conditions_count		= 0;
	error_conditions_count		= 0;
	overflow_conditions_count	= 0;
	local_frames_count			= 0;
	idle_call_count				= 0;
}

uint16_t usi_twi_stats_start_conditions(void)
{
	return(start_conditions_count);
}

uint16_t usi_twi_stats_stop_conditions(void)
{
	return(stop_conditions_count);
}

uint16_t usi_twi_stats_error_conditions(void)
{
	return(error_conditions_count);
}

uint16_t usi_twi_stats_overflow_conditions(void)
{
	return(overflow_conditions_count);
}

uint16_t usi_twi_stats_local_frames(void)
{
	return(local_frames_count);
}

uint16_t usi_twi_stats_idle_calls(void)
{
	return(idle_call_count);
}

#endif