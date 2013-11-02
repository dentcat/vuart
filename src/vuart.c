/*
 * vuart.c implements a full-duplex software UART
 *
 * This implementation requires an external and a timer interrupt.
 * INT0 and TIMER0 is used by default.
 *
 * If you are using anything other than 8Mhz internal oscillator, you
 * need to adjust ONE_BIT_COUNT and ONE_PLUS_BIT_COUNT as well as
 * timer's prescaler if necessary.
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

#include "vuart.h"

#if defined(__AVR_ATmega168P__)  \
    || defined(__AVR_ATtiny48__) \
    || defined(__AVR_ATtiny88__)

#define EXTERNAL_INTERRUPT              INT0_vect
#define RX_TIMER_INTERRUPT              TIMER0_COMPA_vect
#define TX_TIMER_INTERRUPT              TIMER0_COMPB_vect

#define enable_rx_timer_interrupt()     TIMSK0 |= (1 << OCIE0A)
#define enable_tx_timer_interrupt()     TIMSK0 |= (1 << OCIE0B)
#define enable_external_interrupt()     EIMSK  |= (1 << INT0)
#define disable_rx_timer_interrupt()    TIMSK0 &= ~(1 << OCIE0A)
#define disable_tx_timer_interrupt()    TIMSK0 &= ~(1 << OCIE0B)
#define disable_external_interrupt()    EIMSK  &= ~(1 << INT0)
#define reset_rx_interrupt_flag()       TIFR0  |= (1 << OCF0A)
#define reset_tx_interrupt_flag()       TIFR0  |= (1 << OCF0B)
#define reset_external_interrupt_flag() EIFR   |= (1 << INTF0)

#define configure_external_interrupt()  \
    EICRA = (EICRA & ~(1 << ISC01 | 1 << ISC01)) | (1 << ISC01)

#if defined(__AVR_ATmega168P__)
#define configure_timer()               TCCR0A = 0
#else
#define configure_timer()               TCCR0B = 0
#endif

#define configure_timer_prescale()      TCCR0B = (1 << CS01)

#define configure_rx_pin()              DDRD &= ~(1 << PD2)
#define configure_rx_pullup()           PORTD |= (1 << PD2)
#define configure_tx_pin()              DDRD |= (1 << PD1)

#define set_tx_pin()                    PORTD |= (1 << PD1)
#define clear_tx_pin()                  PORTD &= ~(1 << PD1)
#define read_rx_pin()                   PIND & (1 << PD2)

#define COUNTER                         TCNT0
#define RX_NEXT_COUNT                   OCR0A
#define TX_NEXT_COUNT                   OCR0B

#elif defined(__AVR_ATtiny13A__)

#define EXTERNAL_INTERRUPT              INT0_vect
#define RX_TIMER_INTERRUPT              TIM0_COMPA_vect
#define TX_TIMER_INTERRUPT              TIM0_COMPB_vect

#define enable_rx_timer_interrupt()     TIMSK0 |= (1 << OCIE0A)
#define enable_tx_timer_interrupt()     TIMSK0 |= (1 << OCIE0B)
#define enable_external_interrupt()     GIMSK |= (1 << INT0)
#define disable_rx_timer_interrupt()    TIMSK0 &= ~(1 << OCIE0A)
#define disable_tx_timer_interrupt()    TIMSK0 &= ~(1 << OCIE0B)
#define disable_external_interrupt()    GIMSK &= ~(1 << INT0)

#define configure_external_interrupt()  \
    MCUCR = (MCUCR & ~(1 << ISC01 | 1 << ISC01)) | (1 << ISC01)

#define configure_timer()               TCCR0A = 0
#define configure_timer_prescale()      TCCR0B = (1 << CS01)

#define configure_rx_pin()              DDRB &= ~(1 << PB1)
#define configure_rx_pullup()           PORTB |= (1 << PB1)
#define configure_tx_pin()              DDRB |= (1 << PB0)

#define set_tx_pin()                    PORTB |= (1 << PB0)
#define clear_tx_pin()                  PORTB &= ~(1 << PB0)
#define read_rx_pin()                   PINB & (1 << PB1)

#define COUNTER                         TCNT0
#define RX_NEXT_COUNT                   OCR0A
#define TX_NEXT_COUNT                   OCR0B

#else

#error "Virtual UART port mapping not defined for this MCU"

#endif

#if VUART_BAUDRATE == 38400

#define ONE_BIT_COUNT       26
#define ONE_PLUS_BIT_COUNT  32

#else

#error "VUART_BAUDRATE not defined!"

#endif

static volatile uint8_t tx_data;
static volatile uint8_t tx_bit_count;

static volatile uint8_t rx_data;
static volatile uint8_t rx_data_complete;
static volatile uint8_t rx_bit_count;

/*
 * EXTERNAL_INTERRUPT initiates receiving data
 *
 * It disable itself and schedules timer interrupt for receiving data.
 */
ISR(EXTERNAL_INTERRUPT)
{
    RX_NEXT_COUNT = COUNTER + ONE_PLUS_BIT_COUNT;  // set next timer

    reset_rx_interrupt_flag();
    enable_rx_timer_interrupt();

    disable_external_interrupt();

    rx_data = 0;
    rx_bit_count = 0;
}

/*
 * RX_TIMER_INTERRUPT reads bits on RX_PIN and stores them in receive buffer
 *
 * After store all the bits into rx_data it disable itself and re-enable
 * external interrupt to listen for a new data frame.
 */
ISR(RX_TIMER_INTERRUPT)
{
    RX_NEXT_COUNT += ONE_BIT_COUNT;

    if (read_rx_pin())
        rx_data |= (1 << rx_bit_count);

    ++rx_bit_count;

    if (rx_bit_count == VUART_CHAR_SIZE + VUART_STOP_BITS) {
        rx_data_complete = rx_data;
        vuart_state |= (1 << VUART_RECEIVE_COMPLETE);

        disable_rx_timer_interrupt();
        reset_external_interrupt_flag();
        enable_external_interrupt();
    }
}

/*
 * TX_TIMER_INTERRUPT transmit bits in tx_data in correct intervals
 *
 * After transmitting all the bits, sets flags and disable itself
 */
ISR(TX_TIMER_INTERRUPT)
{
    TX_NEXT_COUNT += ONE_BIT_COUNT;

    if (tx_bit_count < VUART_CHAR_SIZE) {
        if (tx_data & 0x01)
            set_tx_pin();
        else
            clear_tx_pin();

        tx_data >>= 1;
    } else
        set_tx_pin();

    tx_bit_count++;

    if (tx_bit_count > VUART_CHAR_SIZE + VUART_STOP_BITS) {
        vuart_state |= (1 << VUART_TRANSMIT_COMPLETE | 1 << VUART_DATA_EMPTY);
        disable_tx_timer_interrupt();
    }
}

/*
 * vuart_init() configures and enables external and timer interrupts
 */
void vuart_init()
{
    // configuring external interrupt
    configure_external_interrupt();
    enable_external_interrupt();

    disable_rx_timer_interrupt();

    configure_rx_pin();

    disable_tx_timer_interrupt();

    configure_tx_pin();
    set_tx_pin();

    vuart_state = 1 << VUART_DATA_EMPTY;

    // configuring timer
    configure_timer();
    configure_timer_prescale();

    sei();
}

/*
 * vuart_read_dat() reads and return data in receive buffer
 *
 * If receive buffer is empty, it blocks and wait until data become available.
 * If non-blocking behaviour is needed, check vuart_is_rx_complete()
 * before calling this function
 */
uint8_t vuart_read_data()
{
    while(!(vuart_state & (1 << VUART_RECEIVE_COMPLETE)));

    vuart_state &= ~(1 << VUART_RECEIVE_COMPLETE);
    return rx_data_complete;
}

/**
 * It writes data to transmit buffer waiting to be sent
 *
 * If transmit buffer is not empty, it blocks and waits until it can transmit
 * new data
 */
void vuart_write_data(uint8_t data)
{
    while(!(vuart_state & (1 << VUART_DATA_EMPTY)));

    vuart_state &= ~(1 << VUART_DATA_EMPTY);
    tx_data = data;
    tx_bit_count = 0;

    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        clear_tx_pin(); // start bit
        TX_NEXT_COUNT = COUNTER + ONE_BIT_COUNT;

        reset_tx_interrupt_flag();
        enable_tx_timer_interrupt();
    }
}
