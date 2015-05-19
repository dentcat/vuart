UART for AVR MCUs
=================

**vuart** is a full duplex UART implementation for AVR Microcontrollers that 
lack the hardware support (eg. ATtiny).

## Getting it to work

The code is tuned for baudrate of 38400 on internal 8Mhz clock. In order
to make it work with a different setting, you need to fine-tune
`ONE_BIT_COUNT` and `ONE_PLUS_BIT_COUNT`.

## License

Copyright (C) 2015 Masood Behabadi

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
implied.
See the License for the specific language governing permissions and
limitations under the License.
