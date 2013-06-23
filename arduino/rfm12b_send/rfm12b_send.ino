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
   
   This is an Arduino sketch that indefinitely sends packets over an RFM12B
   module to test receiving packets with the rfm12b-linux driver. It is meant
   to run on a JeeNode <http://jeelabs.net/projects/hardware/wiki/JeeNode>, but
   can easily be ported to another Arduino-compatible board. It requires the
   JeeLib <https://github.com/jcw/jeelib>.
   
   The sketch runs an infinite loop sending 10-byte data packets over the RFM12B
   module. You can connect an LED on JeeNode port 1 (anode to '+', cathode to
   'A'), which will blink whenever a packet is sent. The data packet to be sent
   will also be written to the serial port.
   
   Check the options at the top of the sketch to make sure they match your
   linux driver configuration.
*/

#include <JeeLib.h>

// must match the ID set in the linux driver
#define GROUP_ID   211
// must match the ID set in the linux driver
#define BAND_ID    RF12_868MHZ
// not relevant for the linux driver unless in Jee-compatible mode, but still...
#define NODE_ID    15
// amount of time in milliseconds the sketch is blocked to let the LED shine
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
   if (sendTimer.poll(SEND_DELAY))
      needToSend = 1;
   
   if (needToSend && (rf12_recvDone() || rf12_canSend())) {
      needToSend = 0;
     
      for (int i=0; i<PACKET_LEN; i++) {
         payload[i] = (pPos + i) % 255;
      }
        
      pPos = (pPos + 1) % 255;
   
      sendLed(1);
      rf12_sendStart(0, payload, PACKET_LEN);
      rf12_sendWait(1);
      delay(LED_DELAY);
      sendLed(0);  
        
      Serial.print("SEND ");
      printBuffer((byte*)payload, PACKET_LEN);
   }
}


