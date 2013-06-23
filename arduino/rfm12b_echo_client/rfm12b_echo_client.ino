/* rfm12b-linux: linux kernel driver for the rfm12(b) RF module by HopeRF
*  Copyright (C) 2013 Georg Kaindl
*  
*  This file is part of rfm12b-linux.
*  
*  rfm12b-linux is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, either version 2 of the License, or
*  (at your option) any later version.
*  
*  rfm12b-linux is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*  
*  You should have received a copy of the GNU General Public License
*  along with rfm12b-linux.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
   DOCUMENTATION:

   This is an Arduino sketch that acts as a "beacon" over an RFM12B
   module to test communication with the rfm12b-linux driver. It is meant
   to run on a JeeNode <http://jeelabs.net/projects/hardware/wiki/JeeNode>, but
   can easily be ported to another Arduino-compatible board. It requires the
   JeeLib <https://github.com/jcw/jeelib>.

   The sketch periodically sends a packet via the RFM12B module, but also
   listens to incoming packets. Thus, it can serve as a "beacon" especially
   meant to work in conjunction with the rfm12b_echo.c linux example: While
   the linux example echoes any packet it receives, this Arduino sketch will
   send and receive such packets.
   
   The sketch is meant to be used to two LEDs connected to the JeeNode's port
   1: The first LED (anode at '+', cathode at 'A') will blink whenever the
   JeeNode sends out a packet, the second LED (anode at 'D', cathode at 'G')
   will blink whenever the JeeNode receives a packet. If the rfm12b_echo.c
   linux example is running, in range an nobody else is sending packets, you
   should see both LEDs light up nearly simultaneously. The sketch is also
   useful to test the range and reliability of your RFM12B network.

   Check the options at the top of the sketch to make sure they match your
   linux driver configuration.
*/

#include <JeeLib.h>

// must match the ID set in the linux driver (either as compile-time option or via ioctl()).
#define GROUP_ID   211
// must match the ID set in the linux driver (either as compile-time option or via ioctl()).
#define BAND_ID    RF12_868MHZ
// not relevant for the linux driver, but still...
#define NODE_ID    15
// amount of time in milliseconds a LED shines (won't block the sketch)
#define LED_DELAY 100
// amount of time to wait between sending packets
#define SEND_DELAY 1000
// length of each packet (must be <= 66)
#define PACKET_LEN 10

/**************************/

Port leds (1);
MilliTimer sendTimer;
char payload[PACKET_LEN];
byte needToSend;
int pPos;

static void recvLed(int state)
{
   leds.mode(OUTPUT);
   leds.digiWrite(state);
}

static void sendLed(int state)
{
   leds.mode2(OUTPUT);
   leds.digiWrite2(!state);
}

static void printBuffer(byte* buf, int len)
{
   for (byte i = 0; i < len; ++i) {
      Serial.print(buf[i]);
      Serial.print(' ');    
   }
    
   Serial.println();
}

void setup()
{
   Serial.begin(57600);
   rf12_initialize(NODE_ID, BAND_ID, GROUP_ID);
}

void loop()
{
   static unsigned long lastMillis;
   static int sendLedMillis;
   static int recvLedMillis;
   
   unsigned long now = millis();
   if (lastMillis != 0) {
      recvLed((recvLedMillis > 0));
      sendLed((sendLedMillis > 0));
      
      if (recvLedMillis > 0) recvLedMillis -= (now - lastMillis);
      if (sendLedMillis > 0) sendLedMillis -= (now - lastMillis);
   }
   lastMillis = now;
  
   if (rf12_recvDone() && rf12_crc == 0) {   
      Serial.print("RECV ");
      printBuffer((byte*)rf12_data, rf12_len);
      recvLed(1);
      recvLedMillis = LED_DELAY;
   }
  
   if (sendTimer.poll(SEND_DELAY))
      needToSend = 1;
   
   if (needToSend && rf12_canSend()) {
      needToSend = 0;
     
      for (int i=0; i<PACKET_LEN; i++) {
         payload[i] = (pPos + i) % 255;
      }
     
      pPos = (pPos + 1) % 255;
     
      sendLed(1);
      sendLedMillis = LED_DELAY;
     
      rf12_sendStart(0, payload, PACKET_LEN);
      rf12_sendWait(1);
      rf12_recvDone();
     
      Serial.print("SEND ");
      printBuffer((byte*)payload, PACKET_LEN);
   }
}
