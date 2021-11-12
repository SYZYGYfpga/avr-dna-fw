/*
 * board_specific.h
 *
 * Author : Opal Kelly Inc.
 * 
 * Board/implementation specific definitions.
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

#define F_CPU 8000000

// Pinout:
// Sequencer Enable Output Pins
#define PORT_EN PORTB
#define DDR_EN  DDRB
#define PIN_EN  PINB
#define PORT_EN0 PORTB0
#define PORT_EN1 PORTB1
#define PORT_EN2 PORTB2
#define DDR_EN0  DDB0
#define DDR_EN1  DDB1
#define DDR_EN2  DDB2
#define PIN_EN0  PINB0
#define PIN_EN1  PINB1
#define PIN_EN2  PINB2

// RGA and Sequencer ADC MUX values
#define RGA_ADC_MUX 0x0
#define SQ0_ADC_MUX 0x1
#define SQ1_ADC_MUX 0x2
#define SQ2_ADC_MUX 0x3

// ADC DIDR values
#define RGA_ADC_DIDR ADC0D
#define SQ0_ADC_DIDR ADC1D
#define SQ1_ADC_DIDR ADC2D
#define SQ2_ADC_DIDR ADC3D

#define ADC_DIDR ((1 << RGA_ADC_DIDR) | (1 << SQ0_ADC_DIDR) | \
                  (1 << SQ1_ADC_DIDR) | (1 << SQ2_ADC_DIDR))


#define FW_BYTES_EEPROM 256 // Total space available in the EEPROM
#define FW_RESERVED_BYTES 9 // EEPROM Space reserved for non-DNA functionality
#define FW_DNA_BYTES 1024 // DNA storage available
