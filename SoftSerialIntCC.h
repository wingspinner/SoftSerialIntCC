#ifndef SoftSerialInt_h
#define SoftSerialInt_h

/******************************************************************************
* SoftSerialIntCC.cpp
* Simultaneous Multi-instance Full-duplex Software Serial Library for STM32Duino
* Using Timer Capture/Compare Hardware Assist
* 
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

/********************************************************************************
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

Acknowledgements
ISRs heavily borrow from Paul Stoffregen's AltSoftSerial originally written for
the Teensi

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
  Definitions" in the .cc file.

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
 *****************************************************************************/


#include <libmaple/libmaple.h>
#include <Arduino.h>

#include <HardwareTimer.h>
#include <ext_interrupts.h>

/******************************************************************************
* Definitions
******************************************************************************/
#define DEBUG_DELAY 0
#define DEBUG_PIN 17
#define DEBUG_PIN1 18

#define  SSI_RX_BUFF_SIZE 64
#define  SSI_TX_BUFF_SIZE 64
#define SS_MAX_RX_BUFF (SSI_RX_BUFF_SIZE - 1)  // RX buffer size
#define SS_MAX_TX_BUFF (SSI_TX_BUFF_SIZE - 1)   // TX buffer size
#define _SSI_VERSION   1.2  // Library Version

/******************************************************************************
* Class Definition
******************************************************************************/
class SoftSerialInt : public Stream
{
  private:

    // Per object data
    uint8_t                   transmitPin;
    exti_num                  gpioBit;
    uint8_t                   receivePin;
    HardwareTimer             timerSerial;
    timer_dev                 *timerSerialDEV;
    uint8_t                   rxtxTimer;
    bool                      activeRX;
    bool                      activeTX;

    volatile uint32_t                  *rxPolBit;

    #if DEBUG_DELAY
    volatile uint8_t          overFlowTail;
    volatile uint8_t          overFlowHead;    
    #endif
    
    uint16_t                  bitPeriod;
    uint16_t                  startBitPeriod;
    uint16_t                  rxStopTicks;
    volatile uint8_t          bufferOverflow;

    volatile int8_t           rxBitCount;
    volatile uint8_t          rxBit;
    volatile uint8_t          rxByte;
    volatile uint16_t         rxTarget;
    volatile uint16_t         receiveBufferWrite;
    volatile uint16_t         receiveBufferRead;
    volatile uint8_t          receiveBuffer[SS_MAX_RX_BUFF]; 
    
    volatile int8_t           txBitCount;
    volatile uint8_t          txBit;
    volatile uint8_t          txByte;
    volatile uint16_t         transmitBufferWrite;
    volatile uint16_t         transmitBufferRead;
    volatile uint8_t          transmitBuffer[SS_MAX_TX_BUFF];
    
    // Static Data
    static SoftSerialInt      *interruptObject1; // This looks inefficient but it reduces
    static SoftSerialInt      *interruptObject2; // interrupt latency a small amount
    static SoftSerialInt      *interruptObject3;
    static SoftSerialInt      *interruptObject4;
    static voidFuncPtr        handleRXTimeoutInterruptP[4];
    static voidFuncPtr        handleRXBitInterruptP[4];
    static voidFuncPtr        handleTXBitInterruptP[4];

    
    // Static Methods
    // Better way to do this?
    static inline void handleRXBitInterrupt1()  __attribute__((__always_inline__));
    static inline void handleRXTimeoutInterrupt1() __attribute__((__always_inline__));
    static inline void handleTXBitInterrupt1()  __attribute__((__always_inline__));

    static inline void handleRXBitInterrupt2()  __attribute__((__always_inline__));
    static inline void handleRXTimeoutInterrupt2() __attribute__((__always_inline__));
    static inline void handleTXBitInterrupt2()  __attribute__((__always_inline__));

    static inline void handleRXBitInterrupt3()  __attribute__((__always_inline__));
    static inline void handleRXTimeoutInterrupt3() __attribute__((__always_inline__));
    static inline void handleTXBitInterrupt3()  __attribute__((__always_inline__));

    static inline void handleRXBitInterrupt4()  __attribute__((__always_inline__));
    static inline void handleRXTimeoutInterrupt4() __attribute__((__always_inline__));
    static inline void handleTXBitInterrupt4()  __attribute__((__always_inline__));

    // Private Methods
    inline void     noTXInterrupts()            __attribute__((__always_inline__));
    inline void     txInterrupts()              __attribute__((__always_inline__));
    inline uint16_t isTXInterruptEnabled()      __attribute__((__always_inline__));
    inline void     txInterruptsClr()           __attribute__((__always_inline__));
    inline void     noRXTimeoutInterrupts()     __attribute__((__always_inline__));
    inline void     rxTimeoutInterrupts()       __attribute__((__always_inline__));
    inline void     rxTimeoutInterruptsClr()       __attribute__((__always_inline__));
    inline void     noRXInterrupts()            __attribute__((__always_inline__));
    inline void     rxInterrupts()              __attribute__((__always_inline__));
    inline void     rxInterruptsClr()           __attribute__((__always_inline__));

    inline void     rxTimeout(void)         __attribute__((__always_inline__));
    inline void     rxNextBit(void)             __attribute__((__always_inline__));
    inline void     txNextBit(void)             __attribute__((__always_inline__));
    void            setInterruptObject(uint8_t timerNumber);

  public:
    // Public Methods
    SoftSerialInt(int receivePinT, int transmitPinT, uint8_t rxtxTimerT /*, bool inverseLogic*/ );
       
    ~SoftSerialInt();
    
    static int      library_version() { return _SSI_VERSION; }
    void            begin(uint32_t tBaud);
    bool            listen();
    bool            isListening() { return activeRX; }
    bool            stopListening();
    bool            talk();
    bool            isTalkingT() { return activeTX; }
    bool            stopTalking();
    void            end();
    bool            overflow() { bool ret = bufferOverflow; if (ret) bufferOverflow = false; return ret; }

    virtual int     peek();
    virtual size_t  write(uint8_t byte);
    virtual int     read();
    virtual int     available();
    virtual void    flush();
    operator bool() { return true; }

    // Debug use only
    #if DEBUG_DELAY
    uint16_t        getBitPeriod() { return bitPeriod; }
    void            setBitPeriod(uint16_t period) { bitPeriod = period; }
    uint16_t        getBitCentering() { return startBitPeriod; }
    void            setBitCentering(uint16_t period) { startBitPeriod = period; }
    uint8_t         getOverFlowTail() { return overFlowTail; }
    uint8_t         getOverFlowHead() { return overFlowHead; }
    uint8_t         getTXHead(){ return transmitBufferRead; }
    uint8_t         getTXTail(){ return transmitBufferWrite; }
    uint8_t         getRXHead(){ return receiveBufferRead; }
    uint8_t         getRXTail(){ return receiveBufferWrite; }
    uint8_t         getrxtxTimer(){ return rxtxTimer; }
    #endif
    
    using Print::write;
  
};

#endif
