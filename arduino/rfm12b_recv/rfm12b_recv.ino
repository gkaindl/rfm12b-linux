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

   This is an Arduino sketch that indefinitely receives packets over an RFM12B
   module to test sending packets with the rfm12b-linux driver. It is meant
   to run on a JeeNode <http://jeelabs.net/projects/hardware/wiki/JeeNode>, but
   can easily be ported to another Arduino-compatible board. It requires the
   JeeLib <https://github.com/jcw/jeelib>.

   The sketch listens for packets on the RFM12B module and prints them to serial.
   You can connect an LED on JeeNode port 1 (anode to 'D', cathode to 'G'),
   which will blink whenever a packet is received.
   
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
// amount of time in milliseconds the sketch is blocked to let the LED shine
#define LED_DELAY  100

/**************************/

Port leds (1);

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
   if (rf12_recvDone() && rf12_crc == 0) {
      recvLed(1);
  
      Serial.print("RECV ");
      printBuffer((byte*)rf12_data, rf12_len);
    
      delay(LED_DELAY);

      recvLed(0);  
   }
}
