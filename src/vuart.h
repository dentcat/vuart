/**
 * vuart.h is a header for full-duplex software UART
 *
 * Copyright (C) 2013 Masood Behabadi <masood@dentcat.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
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
