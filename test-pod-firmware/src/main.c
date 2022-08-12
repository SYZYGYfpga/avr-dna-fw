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

// Sense voltage dividers set to output 20% of the input voltage

// Assume +/- 5%
#define HIGH_THRESH_3v3 690 // 3450 mV before resistor divider
#define LOW_THRESH_3v3  624 // 3120 mV before resistor divider
// Assume +/- 10%
#define HIGH_THRESH_5v  1096 // 5480 mV before resistor divider
#define LOW_THRESH_5v   897  // 4485 mV before resistor divider
// Assume +/- 10%
#define HIGH_THRESH_VIO 396 // 1980 mV before resistor divider
#define LOW_THRESH_VIO  324 // 1620 mV before resistor divider

// Number of ADC readings to average
#define ADC_READ_AVERAGES 10


#define TEST_MODE_1_PIN 0x04
#define TEST_MODE_2_PIN 0x02
#define TEST_MODE_3_PIN 0x01

// Configure the pins used by test pods
void config_test_mode_pins()
{
	DDRB = 7;
	DDRA &= ~(1 << DDA7);
}

// Average a set of ADC readings and adjust to mV
uint32_t average_adc_readings(uint16_t* data, uint32_t size)
{
	uint32_t adc_reading = 0;
	for (uint32_t i = 0; i < size; i++) {
		adc_reading += (uint32_t) data[i];
	}
	adc_reading /= size;
	adc_reading = adc_reading * ADC_MV;
	adc_reading = adc_reading >> ADC_BITS;
	
	return adc_reading;
}


// Perform test pod checks and communicate with FPGA
// Communication is as follows:
// If (TEST_MODE_0 high)
//    TEST_MODE_1 = 3.3V_good
//    TEST_MODE_2 = VIO_good
//    TEST_MODE_3 = 5V_good
// else
//    TEST_MODE_1 = !3.3V_good
//    TEST_MODE_2 = !VIO_good
//    TEST_MODE_3 = !5V_good
void test_pod_check()
{
	static uint8_t status_field = 0;
	static uint8_t averageIndex = 0;
	static uint16_t averageBufferSQ0[ADC_READ_AVERAGES];
	static uint16_t averageBufferSQ1[ADC_READ_AVERAGES];
	static uint16_t averageBufferSQ2[ADC_READ_AVERAGES];
	
	uint32_t adc_reading;
	
	// Read 5V ADC and check
	set_adc_mux(SQ0_ADC_MUX);
	START_ADC;
	if (averageIndex < ADC_READ_AVERAGES) {
		averageBufferSQ0[averageIndex] = read_adc();
	}
	else {
		adc_reading = average_adc_readings(averageBufferSQ0, ADC_READ_AVERAGES);
		
		if ((adc_reading < HIGH_THRESH_5v) && (adc_reading > LOW_THRESH_5v)) {
			status_field |= TEST_MODE_3_PIN;
		}
		else
		{
			status_field &= ~TEST_MODE_3_PIN;
		}
	}
	
	// Read VIO ADC and check
	set_adc_mux(SQ1_ADC_MUX);
	START_ADC;
	if (averageIndex < ADC_READ_AVERAGES) {
		averageBufferSQ1[averageIndex] = read_adc();
	}
	else {
		adc_reading = average_adc_readings(averageBufferSQ1, ADC_READ_AVERAGES);
		
		if ((adc_reading < HIGH_THRESH_VIO) && (adc_reading > LOW_THRESH_VIO)) {
			status_field |= TEST_MODE_2_PIN;
		}
		else
		{
			status_field &= ~TEST_MODE_2_PIN;
		}
	}
	
	// Read 3.3V ADC and check
	set_adc_mux(SQ2_ADC_MUX);
	START_ADC;
	if (averageIndex < ADC_READ_AVERAGES) {
		averageBufferSQ2[averageIndex] = read_adc();
	}
	else {
		adc_reading = average_adc_readings(averageBufferSQ2, ADC_READ_AVERAGES);
		
		if ((adc_reading < HIGH_THRESH_3v3) && (adc_reading > LOW_THRESH_3v3)) {
			status_field |= TEST_MODE_1_PIN;
		}
		else
		{
			status_field &= ~TEST_MODE_1_PIN;
		}
	}
	
	// Increment current average index
	if (averageIndex >= ADC_READ_AVERAGES) {
		averageIndex = 0;
	}
	else {
		averageIndex++;
	}
	
	// Check Test Mode 0 pin and return status
	if (PINA & (1 << PINA7)) {
		PORTB = status_field & 0x7;
	} else {
		PORTB = (~status_field) & 0x7;
	}
}

int main(void)
{
	uint16_t adc_val = 0;
	uint8_t i2c_addr = 0;
	
	config_test_mode_pins();

	init_adc();
	
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
	
	// Main application loop (runs after power sequencing completes
	while (1) 
	{
		// Test pod specific functionality
		test_pod_check();
	}
}

