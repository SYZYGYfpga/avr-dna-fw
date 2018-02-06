/*
 * main.c
 *
 * Author : Opal Kelly Inc.
 * 
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
#include "USI_TWI_Slave.h"
#include "syzygy_helpers.h"
#include "syzygy_seq.h"
#include "syzygy_dna_fw.h"

// The main loop executes as follows:
// - Initialize ADC and power sequencer timer hardware
// - Initialize the power sequencer enable outputs
// - Read the analog voltage on the R_GA pin
// - Convert the R_GA voltage to an I2C address
// - Initialize the I2C handler with the I2C address
// - Initialize the dna_fw timer used to handle I2C transactions
// - Run through the power sequencing code
// - Loop forever, doing nothing
int main(void)
{
	uint16_t adc_val = 0;
	uint8_t i2c_addr = 0;
	uint8_t i;
	sequence_config current_seq_config;
	 // Track the delay time for each enable channel
	uint32_t channel_timer_count[3];
	// The following variables are used to store bitfields in which
	// each bit corresponds to a true (1) or false (0) value for each
	// enable channel or adc input.
	uint8_t channel_threshold_passed = 0;
	uint8_t used_analog_inputs = 0;
	uint8_t crossed_thresholds = 0;
	uint8_t channel_delay_done = 0;
	uint8_t used_enables = 0;
	uint8_t channel_enable_done = 0;

	init_adc();
	
	init_seq_timer();
	
	read_seq_config(&current_seq_config);
	
	// Set sequencer pins to output in the inactive state
	init_seq_outputs(current_seq_config.en_config);
	
	// Get RGA ADC reading
	adc_val = read_adc();
	
	// Determine our I2C address if the ADC reading is valid
	if(adc_val > 0) {
		i2c_addr = adc_to_addr(adc_val);
	}
	
	// Setup I2C if we have a valid address
	if(i2c_addr > 0) {
		USI_TWI_Slave_Initialise(i2c_addr);
	}

	init_i2c_timer();
	
	// Set global interrupt enable (required for I2C communication)
	sei();
	
	// Populate the used_ bitfields based on the sequencer configuration
	for(i = 0; i < 3; i++) {
		used_analog_inputs |= current_seq_config.en_config[i] & 0x7;
		used_enables |= ((current_seq_config.en_config[i] & (1 << EN_ENABLED_BIT)) ? 0 : 1) << i;
	}
	
	// Power sequencing loop
	while ((channel_enable_done & used_enables) != used_enables) {
		// Update the necessary power supply readings for each analog input
		for(i = 0; i < 3; i++) {
			if ((used_analog_inputs & ~crossed_thresholds) & (1 << i)) {
				crossed_thresholds |= test_threshold(i, current_seq_config.threshold[i]) << i;
			}
		}
			
		// Iterate over each sequencer enable channel and check if it's ready to output
		for(i = 0; i < 3; i++) {
			if (channel_delay_done & (1 << i)) {
				// The delay is complete, activate sequencer enable
				set_seq_enable(i, current_seq_config.en_config[i]);
				channel_enable_done |= 1 << i;
			} else if (channel_threshold_passed & (1 << i)) {
				// All necessary thresholds have passed, check if the channel
				// delay has completed
				channel_delay_done |= check_delay(channel_timer_count[i], current_seq_config.delay[i]) << i;
			} else if ((current_seq_config.en_config[i] & crossed_thresholds) == (current_seq_config.en_config[i] & 0x7)) {
				// We've crossed all necessary thresholds for this channel,
				// record the current timer count to allow for delay measurement
				channel_timer_count[i] = current_seq_time_ms();
				channel_threshold_passed |= 1 << i;
			}
		}
	}
	
	// Main application loop (runs after power sequencing completes
	while (1) {
		
	}
}

