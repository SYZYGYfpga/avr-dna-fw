/*
 * syzygy_helpers.h
 *
 * Author : Opal Kelly Inc.
 * 
 * This file describes some helper functions for handling various functionality on
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

#define ADC_MV 3300 // maximum value of the ADC in mV
#define ADC_BITS 10 // with a 10-bit ADC referenced to 3.3V we have ~3mV per bit

#define FLASH_PAGE_WORDS 32 // number of words in a flash page
#define FLASH_PAGE_MASK 0xFC0 // Used to mask off the page of a flash address

#define ADC_TIMEOUT_CYCLES 1000

// Start an ADC reading
#define START_ADC (ADCSRA |= (1 << ADSC))
// Set the ADC MUX value, only uses VCC as a reference
#define set_adc_mux(mux_setting) ADMUX = mux_setting

// A small structure that can be used to better organize sets of pin registers
typedef struct {
	uint8_t ddr;
	uint8_t pin;
	uint8_t port;
} avr_pin_struct;

// Function prototypes:
// ADC related
void init_adc();
uint16_t read_adc();

// EEPROM related
void write_eeprom (uint8_t addr, uint8_t data);
uint8_t read_eeprom (uint8_t addr);
void write_reserved (uint8_t addr, uint8_t data);
uint8_t read_reserved (uint8_t addr);

// Flash related
#define clear_flash_buffer() SPMCSR = (1 << CTPB) | (0 << RFLB) | (0 << PGWRT) | \
                                      (0 << PGERS) | (0 << SPMEN)
#define erase_flash_page(addr) boot_page_erase_safe(addr)
#define write_flash_page(addr) boot_page_write_safe(addr)
#define write_flash_buffer(addr, data) boot_page_fill_safe(addr, data)
#define read_flash(addr) pgm_read_byte_near(addr)
