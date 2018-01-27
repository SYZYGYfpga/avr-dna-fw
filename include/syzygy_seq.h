/*
 * syzygy_seq.h
 *
 * Author : Opal Kelly Inc.
 * 
 * This file describes functions and constants necessary for supply sequencing
 * operation.
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

#include "board_specific.h"

#define TIMER_COUNT_MS ((F_CPU / 1000) / 64) // required timer count for 1ms of time

// threshold[i] stores the power good threshold for sequencer input i
// delay[i] stores the delay from power good to enable i set
// en_config[i] bit [j] is set if enable i depends on threshold j
// en_config[i] bit 3 sets if the output is active high (0 = active high, 1 = active low)
// en_config[i] bit 4 sets if the output is disabled (0 = enabled, 1 = disabled)
typedef struct {
	uint8_t threshold[3];
	uint16_t delay[3];
	uint8_t en_config[3];
} sequence_config;

#define EN_ACTIVE_HIGH_BIT 3
#define EN_ENABLED_BIT 4

void init_seq_timer();
void read_seq_config(sequence_config *configuration);
void init_seq_outputs(uint8_t *en_config);
uint8_t test_threshold(uint8_t threshold_to_update, uint8_t threshold_val);
uint8_t check_delay(uint32_t channel_timer_count, uint16_t delay_time);
uint32_t current_seq_time_ms();
void set_seq_enable(uint8_t enable_num, uint8_t enable_config);