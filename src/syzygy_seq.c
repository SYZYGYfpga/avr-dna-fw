/*
 * syzygy_seq.c
 *
 * Author : Opal Kelly Inc.
 * 
 * Functions related to supply sequencing present in the official SYZYGY firmware.
 *
 ********************************************************************************
 * Copyright (c) 2017 Opal Kelly Incorporated
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ********************************************************************************
 *
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include "syzygy_seq.h"
#include "syzygy_helpers.h"

const uint8_t sequencer_adc_mux[] = {SQ0_ADC_MUX, SQ1_ADC_MUX, SQ2_ADC_MUX};

// Helper struct to keep track of the registers for each of the enable pins
const avr_pin_struct enable_pins[] = {
	{.ddr = DDR_EN0, .pin = PIN_EN0, .port = PORT_EN0},
	{.ddr = DDR_EN1, .pin = PIN_EN1, .port = PORT_EN1},
	{.ddr = DDR_EN2, .pin = PIN_EN2, .port = PORT_EN2}
};

// Global variable to store the current time in ms, tracked by Timer 1 interrupt
volatile uint32_t current_time_ms;

// Initialize the timer-related registers
// This sets up the timer to use the system clock divided by 1024 as this allows for
// the best coverage of our range.
// All PWM and external input features are disabled as they are not used by this design.
void init_seq_timer()
{
	// Set output comparison register A to 125, results in a timer interrupt every 1ms
	OCR1AL = TIMER_COUNT_MS;
	
	// Enable timer interrupt for output comparison register A
	TIMSK1 = (0 << ICIE1) | (0 << OCIE1B) |
	         (1 << OCIE1A) | (0 << TOIE1);

	// Turn off all waveform generation and comparison functions
	TCCR1A = (0 << COM1A0) | (0 << COM1A1) |
	         (0 << COM1B0) | (0 << COM1B1) |
	         (0 << WGM10)  | (0 << WGM11);
	
	// Select ClkI/O / 64 as the timer clock (see page 108-109 of attiny44 datasheet)
	// Clear and reset timer when it hits OCR1A
	TCCR1B = (0 << ICNC1) | (0 << ICES1) |
	         (0 << WGM13) | (1 << WGM12) |
	         (0 << CS12)  | (1 << CS11)  | (1 << CS10);
}

// Read out the sequencer configuration from the reserved EEPROM space
void read_seq_config(sequence_config *configuration)
{
	uint8_t i;
	uint8_t delay_val;
	uint16_t delay_time;
	
	for (i = 0; i < 3; i++) {
		configuration->threshold[i] = read_eeprom(FW_BYTES_EEPROM - FW_RESERVED_BYTES + i);
	}
	
	for (i = 0; i < 3; i++) {
		delay_val = read_eeprom(FW_BYTES_EEPROM - FW_RESERVED_BYTES + i + 3);

		// Delay value calculation:
		// The delay time is stored as a single 8-bit value in the EEPROM.
		// This value is compressed in a semi-exponential fashion as so:
		// -Delay values from 0 to 50 represent 0 to 0.5 seconds in 10ms steps
		// -Delay values from 51 to 55 represent 0.6 to 1.0 seconds in 100ms steps
		// -Delay values from 56 to 58 represent 1.5 to 2.5 seconds in 500ms steps
		if (delay_val <= 50) {
			delay_time = (uint16_t)delay_val * 10;
		} else if (delay_val > 50 && delay_val < 56) {
			delay_time = (((uint16_t)delay_val - 51) * 100) + (600 * TIMER_COUNT_MS);
		} else if (delay_val >= 56 && delay_val < 59) {
			delay_time = (((uint16_t)delay_val - 56) * 500) + (1500 * TIMER_COUNT_MS);
		} else {
			delay_time = 2500; // use the max delay value
		}

		configuration->delay[i] = delay_time;
	}
	
	for (i = 0; i < 3; i++) {
		configuration->en_config[i] = read_eeprom(FW_BYTES_EEPROM - FW_RESERVED_BYTES + i + 6);
	}
}

// Initialize the enable outputs of the sequencer.
// This takes into account whether the output is active high or low.
void init_seq_outputs(uint8_t *en_config)
{
	uint8_t i;
	uint8_t active_high;
	
	for (i = 0; i < 3; i++) {
		if (!(en_config[i] & (1 << EN_ENABLED_BIT))) { // check if the output is enabled
			active_high = (en_config[i] & (1 << EN_ACTIVE_HIGH_BIT)) ? 0 : 1;
			
			if (active_high == 1) {
				// Active high enable, set the pin low for now
				PORT_EN &= ~(1 << enable_pins[i].port);
			} else {
				// Active low enable, set the pin high for now
				PORT_EN |= (1 << enable_pins[i].port);
			}
			DDR_EN  |= (1 << enable_pins[i].ddr);
		}
	}
}

// Set the output to its "active" state (whether high or low)
void set_seq_enable(uint8_t en_num, uint8_t en_config)
{
	uint8_t active_high;
	
	active_high = (en_config & (1 << EN_ACTIVE_HIGH_BIT)) ? 0 : 1;
	
	if(!(en_config & (1 << EN_ENABLED_BIT))) { // check if the output is enabled at all
		if (active_high == 1) {
			// Active high enable, set the pin high
			PORT_EN |= (1 << enable_pins[en_num].port);
		} else {
			// Active low enable, set the pin low
			PORT_EN &= ~(1 << enable_pins[en_num].port);
		}
	}
}

// Read the ADC value corresponding to the desired threshold and return
// 1 if the threshold has been crossed, 0 otherwise.
uint8_t test_threshold(uint8_t threshold_to_update, uint8_t threshold_val)
{
	uint16_t adc_reading = 0;
	
	set_adc_mux(sequencer_adc_mux[threshold_to_update]);
	
	START_ADC;

	adc_reading = read_adc();
	
	if ((adc_reading >> 2) >= threshold_val) {
		return 1;
	}
	
	return 0;
}

// Check if a delay time has been reached.
// Returns 1 if the delay has passed, 0 otherwise.
uint8_t check_delay(uint32_t channel_timer_count, uint16_t delay_time)
{
	uint32_t current_time_rel;
	
	current_time_rel = current_time_ms - channel_timer_count;
	
	if (current_time_rel > delay_time) {
		return 1;
	}
	
	return 0;
}

uint32_t current_seq_time_ms()
{
	return current_time_ms;
}

// Update the current time every millisecond
ISR(TIM1_COMPA_vect)
{
	current_time_ms++;
}
