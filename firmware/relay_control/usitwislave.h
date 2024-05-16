#pragma once

#include <stdbool.h>
#include <stdint.h>

#define USI_TWI_BUFFER_SIZE 32

typedef void (*usi_twi_data_callback_t)(uint8_t input_buffer_length, const uint8_t *input_buffer,
										uint8_t *output_buffer_length, uint8_t *output_buffer);
typedef void (*usi_twi_idle_callback_t)(void);

void usi_twi_slave(uint8_t slave_address, bool use_sleep,
				   usi_twi_data_callback_t data_callback, usi_twi_idle_callback_t idle_callback);

#if defined(CONFIG_USI_TWI_STATS)
void		usi_twi_enable_stats(bool onoff);
uint16_t	usi_twi_stats_start_conditions(void);
uint16_t	usi_twi_stats_stop_conditions(void);
uint16_t	usi_twi_stats_error_conditions(void);
uint16_t	usi_twi_stats_overflow_conditions(void);
uint16_t	usi_twi_stats_local_frames(void);
uint16_t	usi_twi_stats_idle_calls(void);
#endif