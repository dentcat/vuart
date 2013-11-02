UART for AVR MCUs
=================

**vuart** is a full duplex UART implementation for AVR Microcontrollers that 
lack the hardware support (eg. ATtiny).

## Getting it to work

The code is tuned for baudrate of 38400 on internal 8Mhz clock. In order
to make it work with a different setting, you need to fine-tune
`ONE_BIT_COUNT` and `ONE_PLUS_BIT_COUNT`.

## License

Copyright (C) 2013 Masood Behabadi

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
