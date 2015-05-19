/**
 * vuart.h is a header for full-duplex software UART
 *
 * Copyright (C) 2015 Masood Behabadi <masood@dentcat.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef VUART_H_
#define VUART_H_

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

/*
 * Sets baud rate for UART
 */
#define VUART_BAUDRATE 38400
/*
 * Sets the size of data in each UART frame
 */
#define VUART_CHAR_SIZE 8
/*
 * Sets number of stop bits
 */
#define VUART_STOP_BITS 1

/*
 * Bit positions definition in state variable
 */
#define VUART_DATA_EMPTY 0
#define VUART_RECEIVE_COMPLETE 1
#define VUART_TRANSMIT_COMPLETE 2

/*
 * Non-zero reading means new data is received and ready to be read
 */
#define vuart_is_rx_complete() vuart_state & (1 << VUART_RECEIVE_COMPLETE)
/*
 * Non-zero reading means previous data has been completely shifted out
 */
#define vuart_is_tx_complete() vuart_state & (1 << VUART_TRANSMIT_COMPLETE)
/*
 * Non-zero reading means the buffer is empty and ready for new data
 */
#define vuart_is_data_empty() vuart_state & (1 << VUART_DATA_EMPTY)

/*
 * Holds current receive and transmit state
 */
volatile uint8_t vuart_state;

uint8_t vuart_read_data();

void vuart_write_data(uint8_t data);

void vuart_init();

#endif /* VUART_H_ */
