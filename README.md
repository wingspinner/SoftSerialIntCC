# SoftSerialIntCC
Timer Capture/Compare Driven Software Serial Libray for STM32Duino

* Simultaneous Multi-instance Full-duplex Software Serial Library for STM32Duino
* Using Timer Capture/Compare Hardware Assist
* 
/******************************************************************************   
Features:
- Member functions compatible with Hardware Serial and Arduino NewSoftSerial Libs
- Fully interrupt driven and uses timer capture and compare - no delay routines.
- Tolerates other interrupts up to 1 bit time (minus interrupt latencies)
- Circular buffers on both send and receive.
- Simultaneous send/receive on multiple ports
- Up to 115,200 baud. 57,600 baud full duplex
- Supports baud rates down to 300 baud with minimal system overhead
- Supports up to 4 ports (one timer used per port) without modification.
- Easily modified for more ports or different timers on chips with more timers.
- Extensions for transmit enable/disable
- Can send/receive simultaneously with other ports/instantiatious. Tested reliable
  up to 57,600 depending on system interrupt load.

Notes:

Restrictions
- Must use correct pins assigned to the capture/compare input/outputs of the timer
  you select. As currently defined:
  TX pin = Timer channel 1
  RX pin = Timer channel 2
  Of course, this can be changed by modifing the definitions under "Timer Channel
  Definitions" in this file.

Performance
- Multiple port simultaneous send or receive and ability to communicate full duplex
  (receiving a charachter and transmitting at the same time) will be dependent on
  baud rate and other system interrupt load (including interrupts disabled)- you'll
  need to experiment.
- Can be used similar to delay based libraries (NewSoftSerial for instance) which only
  enable one port at a time at half duplex to guarantee reliable operation at high 
  baud rates and system interrupt loading.

Improvements
- No doubt the code can be improved upon. If you improve the code please give back by
  providing soure code for modifications you've made!
- Specific improvements that come to mind and I'd like to explore are:
  o Replacing the STM32/Maple timer interrupt handlers with something more streamlined
    and lower latency and overhead.
  o A lower overhead method to implement C++ ISR's while supporting multiple
    instantians and the STM32 timer shared interrupts.
  o Code tuning to save cycles in the C++ ISR's such as using bit-banding
  o Automatically select pins based on timer selection.
  
License
* Copyright 2015 Ron Curry, InSyte Technologies
* 
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use, copy,
* modify, merge, publish, distribute, sublicense, and/or sell copies
* of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
* BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
* ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*****************************************************************************/
