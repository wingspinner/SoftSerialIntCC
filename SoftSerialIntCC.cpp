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

Acknowledgements
ISRs heavily borrow from Paul Stoffregen's AltSoftSerial originally written for
the Teensi

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
 *****************************************************************************/

/******************************************************************************
* Versions
* 1.2 - Release
* 
******************************************************************************/

 
 #include <Arduino.h>
#include <HardwareTimer.h>
#include <libmaple/timer.h>
#include <ext_interrupts.h>
#include "SSSTimerCapture.h"
#include "SoftSerialIntCC.h"

/******************************************************************************
* Timer Channel Definitions
******************************************************************************/
#define TIMER_MAX_COUNT   0xffff
#define TX_TIMER_CHANNEL  TIMER_CH1
#define TX_TIMER_MASK     TIMER_DIER_CC1IE_BIT
#define TX_TIMER_PENDING  TIMER_SR_CC1IF_BIT
#define TX_CCR            CCR1

#define RX_TIMER_CHANNEL  TIMER_CH2
#define RX_TIMER_MASK     TIMER_DIER_CC2IE_BIT
#define RX_TIMER_PENDING  TIMER_SR_CC2IF_BIT
#define RX_CCR            CCR2

#define RX_TIMER_CHANNEL_B  TIMER_CH3
#define RX_TIMER_MASK_B     TIMER_DIER_CC3IE_BIT
#define RX_TIMER_PENDING_B  TIMER_SR_CC3IF_BIT
#define RX_CCR_B            CCR3
/******************************************************************************
* ISR Related Statics
******************************************************************************/
SoftSerialInt           *SoftSerialInt::interruptObject1;
SoftSerialInt           *SoftSerialInt::interruptObject2;
SoftSerialInt           *SoftSerialInt::interruptObject3;
SoftSerialInt           *SoftSerialInt::interruptObject4;


voidFuncPtr             SoftSerialInt::handleRXTimeoutInterruptP[4] = { 
  handleRXTimeoutInterrupt1, handleRXTimeoutInterrupt2, handleRXTimeoutInterrupt3, handleRXTimeoutInterrupt4
};

voidFuncPtr             SoftSerialInt::handleRXBitInterruptP[4] = {
  handleRXBitInterrupt1, handleRXBitInterrupt2, handleRXBitInterrupt3, handleRXBitInterrupt4
};

voidFuncPtr             SoftSerialInt::handleTXBitInterruptP[4] = {
  handleTXBitInterrupt1, handleTXBitInterrupt2, handleTXBitInterrupt3, handleTXBitInterrupt4
};


/******************************************************************************
* Convenience functions to disable/enable tx and rx interrupts
******************************************************************************/
// Mask transmit interrupt
inline void SoftSerialInt::noTXInterrupts() {
  *bb_perip(&(timerSerialDEV->regs).gen->DIER, TX_TIMER_MASK) = 0;
}


// Enable transmit interrupt
// Note: Purposely does not clear pending interrupt
inline void SoftSerialInt::txInterrupts() {
  *bb_perip(&(timerSerialDEV->regs).gen->DIER, TX_TIMER_MASK) = 1;
}


// Test if transmit interrupt is enabled
inline uint16_t SoftSerialInt::isTXInterruptEnabled() {
  return (*bb_perip(&(timerSerialDEV->regs).gen->DIER, TX_TIMER_MASK));  
}


// Clear pending interrupt and enable receive interrupt
// Note: Clears pending interrupt
inline void SoftSerialInt::txInterruptsClr() {
  *bb_perip(&(timerSerialDEV->regs).gen->SR, TX_TIMER_PENDING) = 0; // Clear int pending
  *bb_perip(&(timerSerialDEV->regs).gen->DIER, TX_TIMER_MASK) = 1;
}


// Mask receive capture interrupt
inline void SoftSerialInt::noRXInterrupts() {
  *bb_perip(&(timerSerialDEV->regs).gen->DIER, RX_TIMER_MASK) = 0;
}


// Enable receive capture interrupt
// Note: Purposely does not clear pending interrupt
inline void SoftSerialInt::rxInterrupts() {
  *bb_perip(&(timerSerialDEV->regs).gen->DIER, RX_TIMER_MASK) = 1;
}


// Clear pending interrupt and enable receive capture interrupt
// Note: Clears pending interrupt
inline void SoftSerialInt::rxInterruptsClr() {
  *bb_perip(&(timerSerialDEV->regs).gen->SR, RX_TIMER_PENDING) = 0; // Clear int pending
  *bb_perip(&(timerSerialDEV->regs).gen->DIER, RX_TIMER_MASK) = 1;

}


// Mask receive timeout interrupt
inline void SoftSerialInt::noRXTimeoutInterrupts() {
  *bb_perip(&(timerSerialDEV->regs).gen->DIER, RX_TIMER_MASK_B) = 0;
}


// Enable receive timeout interrupt
// Note: Purposely does not clear pending interrupt
inline void SoftSerialInt::rxTimeoutInterrupts() {
  *bb_perip(&(timerSerialDEV->regs).gen->DIER, RX_TIMER_MASK_B) = 1;
}


// Clear pending interrupt and enable receive timeout interrupt
// Note: Clears pending interrupt
inline void SoftSerialInt::rxTimeoutInterruptsClr() {
  *bb_perip(&(timerSerialDEV->regs).gen->SR, RX_TIMER_PENDING_B) = 0; // Clear int pending
  *bb_perip(&(timerSerialDEV->regs).gen->DIER, RX_TIMER_MASK_B) = 1;

}


/******************************************************************************
* Specialized functions to set interrupt priorities and assign object ointers
* These are needed due to the gyrations required to support per instance ISRs
******************************************************************************/
// Set Interrupt Priority for EXTInt line
void setEXTIntPriority(uint8_t pin, uint8_t priority) {

  switch((exti_num)(PIN_MAP[pin].gpio_bit)) {
    case EXTI0:
      nvic_irq_set_priority(NVIC_EXTI0, priority);
      break;
    case EXTI1:
      nvic_irq_set_priority(NVIC_EXTI1, priority);
      break;
    case EXTI2:
      nvic_irq_set_priority(NVIC_EXTI2, priority);
      break;
    case EXTI3:
      nvic_irq_set_priority(NVIC_EXTI3, priority);
      break;
    case EXTI4:
      nvic_irq_set_priority(NVIC_EXTI4, priority);
      break;
    case EXTI5:
    case EXTI6:
    case EXTI7:
    case EXTI8:
    case EXTI9:
      nvic_irq_set_priority(NVIC_EXTI_9_5, priority);
      break;
    case EXTI10:
    case EXTI11:
    case EXTI12:
    case EXTI13:
    case EXTI14:
    case EXTI15:
      nvic_irq_set_priority(NVIC_EXTI_15_10, priority);
      break;
  }
  
}


// Set Interrupt Priority for Timer Interrupts
void setTimerIntPriority(uint8_t timerNumber, uint8_t priority) {

  switch(timerNumber) {
    case 1:
      nvic_irq_set_priority(NVIC_TIMER1_UP, priority);
      nvic_irq_set_priority(NVIC_TIMER1_CC, priority);
      break;
    case 2:
      nvic_irq_set_priority(NVIC_TIMER2, priority);
      break;
    case 3:
      nvic_irq_set_priority(NVIC_TIMER3, priority);
      break;
    case 4:
      nvic_irq_set_priority(NVIC_TIMER4, priority);
      break;
  }
  
}


// Set the correct interruptObject for this instance
void SoftSerialInt::setInterruptObject(uint8_t timerNumber) {

  switch (timerNumber) {
    case 1:
      interruptObject1 = this;
      break;
    case 2:
      interruptObject2 = this;
      break;
    case 3:
      interruptObject3 = this;
      break;
    case 4:
      interruptObject4 = this;
      break;
  }
}


/******************************************************************************
* Constructor / Destructor
******************************************************************************/
// Constructor
SoftSerialInt::SoftSerialInt(int receivePinT = 15, int transmitPinT = 16, uint8_t rxtxTimerT = 1) :
  receivePin(receivePinT),
  transmitPin(transmitPinT),
  timerSerial(rxtxTimerT),
  rxtxTimer(rxtxTimerT)
{
  // Assign pointer to the hardware registers
  timerSerialDEV = timerSerial.c_dev();

  // Translate transmit pin number to external interrupt number
  gpioBit = (exti_num)(PIN_MAP[transmitPin].gpio_bit);

  // Setup ISR pointer for this instance and timer (one timer per instance)
  // This is a workaround for c++
  setInterruptObject(rxtxTimer);

  // Set up some pointers that help reduce ISR cycles
  rxPolBit = bb_perip(&(timerSerialDEV->regs).gen->CCER, 4 * (RX_TIMER_CHANNEL - 1) + 1);

}


// Destructor
SoftSerialInt::~SoftSerialInt() {
  end();
}


/******************************************************************************
* TX and RX Interrupt Service Routines
******************************************************************************/
// Transmits next bit. Called by timer ch1 compare interrupt
void SoftSerialInt::txNextBit(void) {
  uint8_t iByte, iBit;
  uint16_t captureTarget;

  #if DEBUG_DELAY
  digitalWrite(DEBUG_PIN1,1);
  digitalWrite(DEBUG_PIN1,0);
  #endif

  iByte = txByte;
  captureTarget = (uint16_t)((timerSerialDEV->regs).gen->TX_CCR);
  
  // State 0 through 9 send bits
  while (txBitCount < 9) {
    captureTarget += bitPeriod;
    iBit = iByte & 1;
    iByte >>= 1;
    txBitCount++;
    if (iBit != txBit) {
      if (iBit) {
        timer_oc_set_mode(timerSerialDEV, TX_TIMER_CHANNEL, TIMER_OC_MODE_ACTIVE_ON_MATCH, 0); // CONFIG_MATCH_SET()
      } else {
        timer_oc_set_mode(timerSerialDEV, TX_TIMER_CHANNEL, TIMER_OC_MODE_INACTIVE_ON_MATCH, 0);        
      }
      timer_set_compare(timerSerialDEV, TX_TIMER_CHANNEL, captureTarget);
      txBit = iBit;
      txByte = iByte;
      return;   
    }
  }

  if (txBitCount == 9) {
    txBitCount = 10;
        timer_oc_set_mode(timerSerialDEV, TX_TIMER_CHANNEL, TIMER_OC_MODE_ACTIVE_ON_MATCH, 0);        
    timer_set_compare(timerSerialDEV, TX_TIMER_CHANNEL, captureTarget + bitPeriod);
    return;
  }

  if ((transmitBufferRead == transmitBufferWrite)) {
      
      // Buffer empty so shutdown delay/timer until "write" puts data in
      txBitCount = 0;
      timer_oc_set_mode(timerSerialDEV, TX_TIMER_CHANNEL, TIMER_OC_MODE_FORCE_ACTIVE, 0); // CONFIG_MATCH_SET()
      noTXInterrupts();   
    } else {
      txBitCount = 1;
      transmitBufferRead = (transmitBufferRead == SS_MAX_TX_BUFF ) ? 0 : transmitBufferRead + 1;
      txByte = transmitBuffer[transmitBufferRead];
      txBit = 0;      
        timer_oc_set_mode(timerSerialDEV, TX_TIMER_CHANNEL, TIMER_OC_MODE_INACTIVE_ON_MATCH, 0);        
      timer_set_compare(timerSerialDEV, TX_TIMER_CHANNEL, captureTarget + bitPeriod);      
    }
  
}


// Receive Timeout ISR
inline void SoftSerialInt::rxTimeout(void){

  uint8_t iBit;

  noRXTimeoutInterrupts();
  timer_cc_set_pol(timerSerialDEV, RX_TIMER_CHANNEL, 1); // Config falling edge

  iBit = rxBit ^ 0x80;
  while (rxBitCount < 9) {
    rxByte = (rxByte >> 1) | iBit;
    rxBitCount++;
  }
  uint16_t next = (receiveBufferWrite == SS_MAX_RX_BUFF ) ? 0 : receiveBufferWrite + 1;
  if (next != receiveBufferRead) {
    receiveBuffer[next] = rxByte;
    receiveBufferWrite = next;
  }
  timer_cc_set_pol(timerSerialDEV, RX_TIMER_CHANNEL, 1);
  rxBit = 0;
  rxBitCount = 0;
}


// Receive next bit interrupt
inline void SoftSerialInt::rxNextBit(void) {
  uint8_t iBit;
  uint16_t iCapture,iTarget;
  int16_t iOffset;

  iCapture = (uint16_t)((timerSerialDEV->regs).gen->RX_CCR);
  iBit = rxBit;
  if (iBit) {
    *rxPolBit = 1; //    timer_cc_set_pol(timerSerialDEV, RX_TIMER_CHANNEL, 1); // Config falling edge
    rxBit = 0;
  } else {
    *rxPolBit = 0; //    timer_cc_set_pol(timerSerialDEV, RX_TIMER_CHANNEL, 0);
    rxBit=0x80;    
  }

  if (rxBitCount == 0) {
    if (!iBit) {
      timerSerial.setCompare(RX_TIMER_CHANNEL_B, iCapture + rxStopTicks); 
      rxTimeoutInterruptsClr();
      rxTarget = iCapture + startBitPeriod;
      rxBitCount = 1;
    }
  } else {
    iTarget = rxTarget;
    while (1) {

      #if DEBUG_DELAY
      digitalWrite(DEBUG_PIN,1);
      digitalWrite(DEBUG_PIN,0);
      #endif
    
      iOffset = iCapture - iTarget;
      if (iOffset < 0) break;
      rxByte = (rxByte >> 1) | rxBit;
      iTarget += bitPeriod;
      rxBitCount++;
      if (rxBitCount >= 9) {
        noRXTimeoutInterrupts();
        uint16_t next = (receiveBufferWrite == SS_MAX_RX_BUFF ) ? 0 : receiveBufferWrite + 1;
        if (next != receiveBufferRead) {
          receiveBuffer[next] = rxByte;
          receiveBufferWrite = next;
        }
        *rxPolBit = 1; //    timer_cc_set_pol(timerSerialDEV, RX_TIMER_CHANNEL, 1); // Config falling edge
        rxBit = 0;
        rxBitCount = 0;
        return;    
      }
    }
    rxTarget = iTarget;   
  }
}


/******************************************************************************
* Begin - Instance setup
******************************************************************************/
void SoftSerialInt::begin(uint32_t tBaud) {
  
  digitalWrite(transmitPin, 1);
  pinMode(receivePin, INPUT_PULLUP);
  pinMode(transmitPin, PWM);

  #if DEBUG_DELAY
  pinMode(DEBUG_PIN, OUTPUT);
  digitalWrite(DEBUG_PIN, 0);
  pinMode(DEBUG_PIN1, OUTPUT);
  digitalWrite(DEBUG_PIN1, 0);
  #endif

  // Initialize the timer
  noInterrupts();
  timerSerial.pause();

  if (tBaud > 2400) {
    bitPeriod = (uint16_t)((uint32_t)((CYCLES_PER_MICROSECOND * 1000000) + tBaud / 2) / tBaud);
    startBitPeriod = bitPeriod + (bitPeriod / 2) - 300;
    timerSerial.setPrescaleFactor(1);
  } else if (tBaud > 300) {
    bitPeriod = (uint16_t)(((uint32_t)(CYCLES_PER_MICROSECOND * 1000000) / 16) / tBaud);
    startBitPeriod = bitPeriod + (bitPeriod / 2);
    timerSerial.setPrescaleFactor(16); 
  } else {
    bitPeriod = (uint16_t)(((uint32_t)(CYCLES_PER_MICROSECOND * 1000000) / 16) / tBaud) / 2;
    bitPeriod -= 600;
    startBitPeriod = bitPeriod + (bitPeriod / 2);
    timerSerial.setPrescaleFactor(16);     
  }

  rxStopTicks = bitPeriod * 37 / 4;

  timerSerial.setOverflow(TIMER_MAX_COUNT); 

  // Set transmit bit timer channel and interrupt  
  timer_cc_disable(timerSerialDEV, TX_TIMER_CHANNEL);
  timerSerial.setMode(TX_TIMER_CHANNEL, TIMER_OUTPUT_COMPARE);

  // State tx machine start state, attach bit interrupt, and mask it for now
  transmitBufferRead = transmitBufferWrite = 0;
  txBitCount = 0;  
  timerSerial.attachInterrupt(TX_TIMER_CHANNEL, handleTXBitInterruptP[rxtxTimer - 1]);
  noTXInterrupts();
  
  // Set receive bit timer channel and interrupt 
  timer_cc_disable(timerSerialDEV, RX_TIMER_CHANNEL);
  timer_ic_set_mode(timerSerialDEV, RX_TIMER_CHANNEL, TIMER_IC_MODE_TI1, TIMER_IC_PRESCALER_NONE,
                      TIMER_IC_FILTER_NONE);        
  *rxPolBit = 1; //    timer_cc_set_pol(timerSerialDEV, RX_TIMER_CHANNEL, 1); // Config falling edge

  // Set rx State machine start state, attach the bit interrupt and mask it for now
  receiveBufferRead = receiveBufferWrite = 0;
  rxBitCount = 0;
  timerSerial.attachInterrupt(RX_TIMER_CHANNEL, handleRXBitInterruptP[rxtxTimer - 1]);
  noRXInterrupts();
      
  // Set timeout bit receive timer channel and interrupt and mask for now
  timer_cc_disable(timerSerialDEV, RX_TIMER_CHANNEL_B);
  timerSerial.setMode(RX_TIMER_CHANNEL_B, TIMER_OUTPUT_COMPARE);
  timerSerial.attachInterrupt(RX_TIMER_CHANNEL_B, handleRXTimeoutInterruptP[rxtxTimer - 1]);
  noRXTimeoutInterrupts();
  
  bufferOverflow = false;
  receiveBufferRead = receiveBufferWrite = 0;
  transmitBufferRead = transmitBufferWrite = 0;

  // Make the timer we are using high priority
  setTimerIntPriority(rxtxTimer, 0);

  // Load the timer values and start it
  timerSerial.refresh();     
  timerSerial.resume();

  // Turn everything on
  activeRX = true;
  timer_cc_enable(timerSerialDEV, RX_TIMER_CHANNEL);
  timer_cc_enable(timerSerialDEV, RX_TIMER_CHANNEL_B);

  activeTX = true;
  timer_cc_enable(timerSerialDEV, TX_TIMER_CHANNEL);

  // Turn on interrupts. TX interrupts turn on at first write.
  rxInterruptsClr();
  interrupts();
}


/******************************************************************************
* RX Related Public Methods
******************************************************************************/
// Sets current instance listening. Transmit is always enabled
// If his instance was already activeRX does nothing and returns false 
bool SoftSerialInt::listen() {

  // If receive not activeRX then re-init and set activeRX
  if (!activeRX) {

    // Reset receieve buffer and mark activeRX
    bufferOverflow = false;
    receiveBufferRead = receiveBufferWrite = 0;
    rxBitCount = 0;
    activeRX = true;

    // Turn the receive interrupts
    *rxPolBit = 1; //    timer_cc_set_pol(timerSerialDEV, RX_TIMER_CHANNEL, 1); // Config falling edge
    timer_cc_enable(timerSerialDEV, RX_TIMER_CHANNEL);
    rxInterruptsClr();
      
    return true;
  }
  return false;
}


// Stop Listening - Shuts down only RX - Use end() to stop both rx and tx
// Returns true if was listening when called
// This instance will stop all RX interrupts after current in-process
// byte is finished receiving (if any).
// If no in-process receive byte it stops immediately
bool SoftSerialInt::stopListening() {

  if (activeRX) {
    
    // Wait until in idle state
    while (rxBitCount);
    timer_cc_disable(timerSerialDEV, RX_TIMER_CHANNEL);
    noRXInterrupts();
    activeRX = false;
    return true;

  } else
    return false;
    
}


// Completely shutsdown this instance
// Not an RX related method but needs to be after stopListening
void SoftSerialInt::end() {

  stopListening();
  timerSerial.pause();
  timerSerial.detachInterrupt(RX_TIMER_CHANNEL_B);
  timerSerial.detachInterrupt(RX_TIMER_CHANNEL);
  timerSerial.detachInterrupt(TX_TIMER_CHANNEL);
  timerSerial.setMode(TX_TIMER_CHANNEL, TIMER_DISABLED); 
  timerSerial.setMode(RX_TIMER_CHANNEL, TIMER_DISABLED); 
  timerSerial.setMode(RX_TIMER_CHANNEL_B, TIMER_DISABLED); 
  digitalWrite(transmitPin, 1);
}


// Returns number of bytes in the RX buffer
int SoftSerialInt::available() { 
  int i;

  if (!activeRX)
    return 0;

//  noRXInterrupts();
//  i = (receiveBufferWrite + SS_MAX_RX_BUFF - receiveBufferRead) % SS_MAX_RX_BUFF;
//  rxInterrupts();

    if (receiveBufferWrite >= receiveBufferRead)
      i = receiveBufferWrite - receiveBufferRead;
    else
      i = SS_MAX_RX_BUFF + receiveBufferWrite - receiveBufferRead; 

  return i;
}


// Blocking read to be compatible with HardwareSerial
// Blocks until byte is available in buffer
// Returns -1 if instance is not activeRX
int SoftSerialInt::read() { 
  
  if (!activeRX)
    return -1;
    
  // Wait if buffer is empty
  if (receiveBufferRead == receiveBufferWrite)
    return -1;

  receiveBufferRead = (receiveBufferRead == SS_MAX_RX_BUFF ) ? 0 : receiveBufferRead + 1;
  uint8_t inData = receiveBuffer[receiveBufferRead];  

  return inData;
}


// Flush the receive buffer
void SoftSerialInt::flush() {
  
//  noRXInterrupts();
  receiveBufferRead = receiveBufferWrite = 0;
//  rxInterrupts();

}


// Return the next item in the receive buffer but leave in buffer
int SoftSerialInt::peek() {

  if (!activeRX)
    return -1;

  // If buffer is empty return false
  if (receiveBufferRead == receiveBufferWrite)
    return -1;

  // Otherwise read the byte at head of buffer but don't delete
  return receiveBuffer[receiveBufferRead];

}


/******************************************************************************
* TX Related Public Method(s)
******************************************************************************/
// Sets current instance enabled for sending
// If his instance was already activeRX does nothing and returns false 
bool SoftSerialInt::talk() {

  // If transmit not active then re-init and set activeTX
  if (!activeTX) {

    // Reset transmit buffer and mark active
    transmitBufferRead = transmitBufferWrite = 0;
    txBitCount = 0;  
    timer_cc_enable(timerSerialDEV, TX_TIMER_CHANNEL);
    activeTX = true;

    // Turn transmit interrupts on
    txInterruptsClr();
      
    return true;
  }
  return false;
}


// Stop Sending - Shuts down only TX - Use end() to stop both rx and tx
// or "stopListening" for rx
// Returns true if sending already enabled when called
// This instance will stop sending at end of current byte immediately 
bool SoftSerialInt::stopTalking() {

  if (activeTX) {

    // Wait until in-progess byte sent if any
    while (txBitCount);
    activeTX = false;
    noTXInterrupts();
    digitalWrite(transmitPin, 1);
    timer_cc_disable(timerSerialDEV, TX_TIMER_CHANNEL);
    return true;

  } else
    return false;

}


// Virtual write
// Saves tx byte in buffer and restarts transmit delay timer
// 1 bit time latency prior to transmit start if buffer was empty
size_t SoftSerialInt::write(uint8_t b) {
  uint8_t head;

  if (activeTX) {
      // Blocks if buffer full
      bool i;
      head = (transmitBufferWrite == SS_MAX_TX_BUFF) ? 0 : transmitBufferWrite + 1;
      while ( head == transmitBufferRead);

      noInterrupts();
//      if (txBitCount) {
      if (isTXInterruptEnabled()) {
      // Save new data in buffer and bump the write pointer
      transmitBuffer[head] = b;
      transmitBufferWrite = head;
      } else {
        txBitCount = 1;
        txByte = b;
        txBit = 0;
        timer_oc_set_mode(timerSerialDEV, TX_TIMER_CHANNEL, TIMER_OC_MODE_INACTIVE_ON_MATCH, 0);        
        timer_set_compare(timerSerialDEV, TX_TIMER_CHANNEL, (uint16_t)((timerSerialDEV->regs).gen->TX_CCR) + 16);        
        txInterruptsClr();
      }
      interrupts();
    
      return 1;
  } else {
    return 0;
  }
  
}

/******************************************************************************
* 
* Intermediate Level Interrupt Service Routines
* One ISR for each interrupt times 4 to support 4 instantiations of the class
* on up to 4 different timers. 
* 
* This is to work around the fact that static data and
* static member functions become part of the base class and are common to all
* instantiations and ISRs must be static in order to derive a std C type pointer to
* them which is required by the NVIC and hardware timer interrupt code. If there is 
* a better way to do this I'd welcome the opportunity to learn about it.
* 
* These are at the bottom of the file just to get them out of the way.
******************************************************************************/

inline void SoftSerialInt::handleRXBitInterrupt1() {

//  if (interruptObject1)
//  { 
//    noInterrupts(); 
    interruptObject1->rxNextBit(); 
//    interrupts();
//  }
}


inline void SoftSerialInt::handleRXTimeoutInterrupt1() {
  if (interruptObject1)
//  {
//    noInterrupts(); 
    interruptObject1->rxTimeout();
//    interrupts();
//  }
}


inline void SoftSerialInt::handleTXBitInterrupt1() {
//  if (interruptObject1)
//  {
//    noInterrupts(); 
    interruptObject1->txNextBit();
//    interrupts();
//  }
}

inline void SoftSerialInt::handleRXBitInterrupt2() {
//  if (interruptObject2)
//  {
//    noInterrupts(); 
    interruptObject2->rxNextBit();
//    interrupts();
//  }
}


inline void SoftSerialInt::handleRXTimeoutInterrupt2() {
//  if (interruptObject2)
//  {
//    noInterrupts(); 
    interruptObject2->rxTimeout();
//    interrupts();
//  }
}


inline void SoftSerialInt::handleTXBitInterrupt2() {
//  if (interruptObject2)
//  {
//    noInterrupts(); 
    interruptObject2->txNextBit();
//    interrupts();
//  }
}

inline void SoftSerialInt::handleRXBitInterrupt3() {
//  if (interruptObject3)
//  {
//    noInterrupts(); 
    interruptObject3->rxNextBit();
//    interrupts();
//  }
}


inline void SoftSerialInt::handleRXTimeoutInterrupt3() {
//  if (interruptObject3)
//  {
//    noInterrupts(); 
    interruptObject3->rxTimeout();
//    interrupts();
//  }
}


inline void SoftSerialInt::handleTXBitInterrupt3() {
//  if (interruptObject3)
//  {
//    noInterrupts(); 
    interruptObject3->txNextBit();
//    interrupts();
//  }
}

inline void SoftSerialInt::handleRXBitInterrupt4() {
//  if (interruptObject4)
//  {
//    noInterrupts(); 
    interruptObject4->rxNextBit();
//    interrupts();
//  }
}


inline void SoftSerialInt::handleRXTimeoutInterrupt4() {
//  if (interruptObject4)
//  {
//    noInterrupts(); 
    interruptObject4->rxTimeout();
//    interrupts();
//  }
}


inline void SoftSerialInt::handleTXBitInterrupt4() {
//  if (interruptObject4)
//  {
//    noInterrupts(); 
    interruptObject4->txNextBit();
//    interrupts();
//  }

}

