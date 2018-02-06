/*
 * syzygy_dna_fw.c
 *
 * Author : Opal Kelly Inc.
 * 
 * This file contains functions and constants related to communicating SYZYGY DNA
 * information over I2C.
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
#include <avr/boot.h>
#include <avr/pgmspace.h>
#include "syzygy_dna_fw.h"
#include "syzygy_helpers.h"
#include "board_specific.h"
#include "USI_TWI_Slave.h"

// We want to poll I2C at 20 kHz to capture up to 100 kHz I2C traffic
// The timer clock runs at 1MHz (clk_io / 8), so a count of 50 provides
// 20 kHz polling.
#define TIMER_DNA_COUNT 50

#define GA_VALID_BUFFER_MV 50  // allow for +/-50mV from each GA value to account for noise, etc.
#define DNA_FLASH_OFFSET 0xC00 // Byte offset of DNA storage in flash

// Voltages in mV of each geographical address resistance measurement
// Defined in SYZYGY Specification v1.0
// This assumes a 3.3V reference and a 10k pull-up on the peripheral.
const uint16_t adc_val_ga[] = {3147, 2944, 2740, 2538, 2341, 2135, 1926, 1734,
                               1535, 1341, 1137, 933, 738, 541, 342, 153};

// Geographical address corresponding to each potential resistor reading
// Defined in the SYZYGY specification v1.0
const uint8_t geo_addr[16] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
                              0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F};

// Registers:
// {FW major version, FW minor version, DNA major version, DNA minor version
//  EEPROM size (high), EEPROM size (low)}
const uint8_t syzygy_reg[] = {0x01, 0x01, 0x01, 0x00, FW_DNA_BYTES >> 8, FW_DNA_BYTES & 0xFF};
#define SYZYGY_REG_SIZE 6 // The syzygy_reg setup contains 6 bytes

// Some (global) variables for storing the sub_address state between runs of
// the I2C polling timer interrupt
volatile uint16_t sub_addr_read = 0;
volatile uint16_t sub_addr_write = 0;

// Initialize the I2C timer on timer/counter0, this is used to periodically
// poll the USI_TWI interface.
void init_i2c_timer()
{
	// Set max value for I2C timer
	OCR0A = TIMER_DNA_COUNT;

	// Enable I2C timer interrupt
	TIMSK0 = (0 << OCIE0B) | (1 << OCIE0A) | (0 << TOIE0);

	// Set timer to reset when it hits OCRA
	TCCR0A = (0 << COM0A1) | (0 << COM0A0) |
	         (0 << COM0B1) | (0 << COM0B0) |
	         (1 << WGM01)  | (0 << WGM00);

	// Set timer to use clkIO / 8
	TCCR0B = (0 << FOC0A)  | (0 << FOC0B) |
	         (0 << WGM02)  | (0 << CS02)  |
	         (1 << CS01)   | (0 << CS00);
}

// We don't have any registers to write to so this does nothing
uint8_t write_reg (uint8_t addr, uint8_t data)
{
	return 0;
}

// Read from the registers, if the register does not exist, read 0xFF
uint8_t read_reg (uint8_t addr)
{
	if (addr < SYZYGY_REG_SIZE) {
		return syzygy_reg[addr];
	}
	
	return 0xFF;
}

// Translate an adc reading from the RGA pin to a SYZYGY I2C address
uint8_t adc_to_addr (uint32_t adc_in)
{
	uint8_t i = 0;
	
	// Convert ADC reading to mV
	adc_in = adc_in * ADC_MV;
	
	adc_in = adc_in >> ADC_BITS;
	
	for (i = 0; i < 16; i++){
		if ((adc_in > (adc_val_ga[i] - GA_VALID_BUFFER_MV))
		 && (adc_in < (adc_val_ga[i] + GA_VALID_BUFFER_MV))) {
			return geo_addr[i];
		}
	}
	
	return 0; // no valid address found, error
}

// Function used to handle writing to the flash memory.
// For this Firmware writes must be word (2-byte) aligned to function properly.
// A single byte write will result in undefined behavior.
void handle_dna_flash_write()
{
	uint16_t dna_addr = (sub_addr_write & 0x7FFF) + DNA_FLASH_OFFSET;
	uint16_t dna_data = 0xFF;
	uint8_t temp_data;
	uint8_t data_count = USI_TWI_Data_In_Receive_Buffer();
	uint8_t data_count_odd = data_count % 2;
	uint8_t i;
	
	// Disable the USI during flash operations to prevent it from entering
	// an undesired state while the flash is being written.
	USICR = 0;
	DDRA &= ~(1 << PORTA4); // Set SCL to input
	USISR = 0xF0; // Clear all flags and reset overflow counter
	
	clear_flash_buffer();
	
	// Fill the temporary flash buffer
	for (i = 0; i < data_count; i++) {
		temp_data = USI_TWI_Receive_Byte();
		
		if (dna_addr & 1) {
			// on an odd-byte, write to the temporary flash buffer
			dna_data |= temp_data << 8;
			write_flash_buffer(dna_addr, dna_data);
		} else {
			dna_data = temp_data;
		}
		
		dna_addr++;
	}
	
	// If there was an odd amount of data and the address pointed to
	// the lower byte of a word, write this value to memory to flush it
	if (data_count_odd && ((sub_addr_write & 0x1) == 0)) {
		// Keep the high byte at 0xFF to allow writing to it later
		dna_data |= 0xFF00;
		write_flash_buffer(dna_addr, dna_data);
	}
	
	dna_addr = (sub_addr_write & 0x7FFF) + DNA_FLASH_OFFSET;
	
	if ((dna_addr & FLASH_PAGE_MASK) == dna_addr) {
		erase_flash_page(dna_addr);
	}
	
	write_flash_page(dna_addr);
	
	// Re-enable the USI interface now that we're finished with the flash
	USICR = (1<<USISIE)|(0<<USIOIE)| // Enable Start Condition Interrupt. Disable Overflow Interrupt.
	        (1<<USIWM1)|(0<<USIWM0)| // Set USI in Two-wire mode. No USI Counter overflow hold.
	        (1<<USICS1)|(0<<USICS0)|(0<<USICLK)| // Shift Register Clock Source = External, positive edge
	        (0<<USITC);
	USISR = 0xF0; // Clear all flags and reset overflow counter
	DDRA |= (1 << PORTA4); // Set SCL to output again
}

// Simple function to re-map the provided DNA address to the flash memory space
uint8_t read_dna_flash(uint16_t addr)
{
	addr &= 0x07FF;
	
	return read_flash(addr + DNA_FLASH_OFFSET);
}

// Handle the I2C timer interrupt. This reads data transmitted from the I2C
// master and performs the necessary tasks. This must be run at least once per
// I2C byte transfer to function properly.
ISR(TIM0_COMPA_vect)
{
	// Is this a start of a new transaction?
	// If so, wait until we have two bytes for the sub_address
	if (USI_TWI_Is_New_Tx() == 1) {
		
		if (USI_TWI_Data_In_Receive_Buffer() < 2){
			return;
		}
		
		// Clear the new_tx flag so that this is only performed once per transfer
		USI_TWI_Clear_New_Tx();
		
		// Set the sub_address variables
		sub_addr_write = ((uint16_t) USI_TWI_Receive_Byte()) << 8;
		sub_addr_write |= ((uint16_t) USI_TWI_Receive_Byte());
		sub_addr_read = sub_addr_write;
		
		if (USI_TWI_Data_In_Receive_Buffer() == 0){
			Flush_TWI_Buffers();
		}
	}

	if (USI_TWI_Data_In_Receive_Buffer() > 0) {
		// If there's still data left we have a write
		if (sub_addr_write >= 0x9000) {
			write_reserved(sub_addr_write, USI_TWI_Receive_Byte());
			sub_addr_write++;
		} else if (sub_addr_write >= 0x8000) {
			// Allow data to buffer in RAM until a stop condition occurs since writes take awhile
			if (USISR & (1 << USIPF)) {
				handle_dna_flash_write();
			}
		} else {
			write_reg(sub_addr_write, USI_TWI_Receive_Byte());
			sub_addr_write++;
		}
	} else if (USI_TWI_Data_In_Transmit_Buffer() < TWI_TX_BUFFER_SIZE - 1) {
		// Assume read, fill the transmit buffer
		if (sub_addr_read >= 0x9000) {
			USI_TWI_Transmit_Byte(read_reserved(sub_addr_read));
			sub_addr_read++;
		} else if (sub_addr_read >= 0x8000 && sub_addr_read < 0x8000 + FW_DNA_BYTES) {
			USI_TWI_Transmit_Byte(read_dna_flash(sub_addr_read));
			sub_addr_read++;
		} else {
			USI_TWI_Transmit_Byte(read_reg(sub_addr_read));
			sub_addr_read++;
		}
	}
}
