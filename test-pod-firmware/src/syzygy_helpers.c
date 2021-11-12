/*
 * syzygy_helpers.c
 *
 * Author : Opal Kelly Inc.
 * 
 * This file contains some helper functions for handling various functionality on
 * the AVR.
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

#include <stdint.h>
#include <avr/io.h>
#include "syzygy_helpers.h"
#include "board_specific.h"

void init_adc()
{
	// Default to RGA mux setting
	ADMUX = RGA_ADC_MUX;

	// Configure ADC Control Register B
	ADCSRB = (0 << BIN) | (0 << ACME) // disable Bipolar Input Mode and ACME
	       | (0 << ADLAR)             // disable ADC Left Adjust Result
	       | (0 << ADTS2) | (0 << ADTS1) | (0 << ADTS0); // set free running mode
	
	// Configure ADC Control Register A
	ADCSRA = (1 << ADEN) | (0 << ADSC) // Enable ADC, do not start a conversion yet
	       | (0 << ADATE)              // disable auto trigger
	       | (0 << ADIF) | (0 << ADIE) // disable ADC interrupt
	       | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Divide ADC clock by 128
		   
	START_ADC;
	
	// Disable digital portion of GA pin since this is an analog-only input
	DIDR0 = ADC_DIDR;
}

// Take an ADC reading from the currently configured ADC pin (based on the
// current ADC MUX setting). Returns as a 16-bit right adjusted value.
// Note: this function blocks until the ADC reading is complete.
uint16_t read_adc()
{
	uint16_t i;
	uint8_t low_byte, high_byte;
	
	// Wait for conversion to complete
	for (i = 0; i < ADC_TIMEOUT_CYCLES; i++){
		if(!(ADCSRA & (1 << ADSC))) {
			low_byte = ADCL;
			high_byte = ADCH;
			
			return (high_byte << 8) | low_byte; // conversion is completed
		}
	}
	
	// The conversion never finished, something is wrong
	return -1;
}

// Write to the internal EEPROM
void write_eeprom (uint8_t addr, uint8_t data)
{
	while (EECR & (1 << EEPE)); // wait for previous transaction to finish
	EECR = (0 << EEPM1) | (0 << EEPM0); // clear EEPROM program enable bits
	EEARL = addr; // set EEPROM address
	EEDR = data;
	EECR |= (1 << EEMPE);
	EECR |= (1 << EEPE);
}

// Read from the internal EEPROM
uint8_t read_eeprom (uint8_t addr)
{
	while (EECR & (1 << EEPE)); // wait for previous transaction to finish
	EEARL = addr; // set EEPROM address
	EECR |= (1 << EERE); // Begin read (completes in a single cycle)
	return EEDR;
}

// Write to reserved EEPROM space (used for firmware specific functionality)
void write_reserved (uint8_t addr, uint8_t data)
{
	write_eeprom(addr + FW_BYTES_EEPROM - FW_RESERVED_BYTES, data);
}

// Read from reserved EEPROM space (used for firmware specific functionality)
uint8_t read_reserved (uint8_t addr)
{
	return read_eeprom(addr + FW_BYTES_EEPROM - FW_RESERVED_BYTES);
}