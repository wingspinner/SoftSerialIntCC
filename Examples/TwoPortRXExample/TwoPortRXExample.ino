#include <SoftSerialIntCC.h>

SoftSerialIntCC ss(10, 11, 2);
SoftSerialIntCC ss2(4, 5, 3);

/* This sample shows how to process received data
   on two different "soft" serial ports without risk
   of dropping characters due to interrupt latencies.  
   Here we listen on the first port (ss) until we receive a
   '?' character.  Then we begin listening on the other soft port.

   This method is really just using one serial port at a time
   in other words, you can't receive on both ports or transmit
   at the same time. This example is included to show compatibility
   with the Arduino NewSoftSerial library however it doesn't
   exploit the capability of SoftSerialIntCC to simultaneously
   service two ports at some baud rates (57.6k and lower).

   This method may
   be useful in systems with high interrupt loading, many ports,
   or just to be conservative. SoftSerialIntCC has
   been tested with two ports enabled at the same time and
   appears to operate without errors up to 57600 baud. It can
   tolerate up to a little less than one bit time of interrupt
   latency therefore it may be possible to have more than two
   ports enabled simultaneously at lower baud rates. 

   Note that to disable a port you must add a "stopListening" since
   SoftSerialIntCC allows individual control of each
   port and does not automatically disable the other ports when
   a "listen" is executed on one port.
*/

void setup()
{
  // Start the HW serial port
  Serial.begin(57600);

  // Start each soft serial port
  ss.begin(4800);
  ss2.begin(4800);

  // By default, the most recently "begun" port is listening.
  // We want to listen on ss, so let's explicitly select it.
  ss.listen();
  ss2.stopListening(); // NOTE: This needs to be add for SoftSerialIntCC
  
  // Simply wait for a ? character to come down the pipe
  Serial.println("Data from the first port: ");
  char c = 0;
  do
    if (ss.available())
    {
      c = (char)ss.read();
      Serial.print(c);
    }
  while (c != '?');

  // Now listen on the second port
  ss2.listen();
  ss.stopListening; // NOTE: This needs to be add for SoftSerialIntCC

  Serial.println("Data from the second port: ");
}

void loop()
{
  if (ss2.available())
  {
    char c = (char)ss2.read();
    Serial.print(c);
  }
}
  
