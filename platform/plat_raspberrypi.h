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

#if !defined(__RFM12_PLAT_RASPBERRYPI_H__)
#define __RFM12_PLAT_RASPBERRYPI_H__

#include <linux/ioport.h>
#include <asm/io.h>

/*
   This file handles pinmuxing on Raspberry Pi.
   
   If you want to change the default hardware settings for RPi, this
   is the place to look.
*/


// the amount of RFM12 modules connected to the raspberry pi
// per default, we have settings for one board. if you want
// more boards, you need to add the necessary settings below
// as well.
#define NUM_RFM12_BOARDS         1

/*
*  default config for raspberry pi (one RFM12 module)
*
*  Raspberry Pi          RFM12B
*  ---------------------------------
*  P1/17                 VDD (+3.3V)
*  P1/19                 SDI
*  P1/21                 SDO
*  P1/22                 nIRQ
*  P1/23                 SCK
*  P1/25                 GND
*  P1/26                 nSEL
*
*  make sure the SPI driver is loaded before loading this
*  module!
*
*  modprobe spi-bcm2708
*/

#if RFM12B_BOARD==1        // RPI
#define GPIO_MEM_BASE      0x20200000
#elif RFM12B_BOARD==4      // RPI2
#define GPIO_MEM_BASE      0x3F200000
#endif

struct spi_rfm12_board_config board_configs[NUM_RFM12_BOARDS] = {
   {
      .irq_pin      = 25, // gpio 25
      .spi_bus      = 0,  // spi port on P1 header
      .spi_cs       = 1   // CS 1
   }
};

static int
spi_rfm12_cleanup_pinmux_settings(void);
static int
spi_rfm12_cleanup_irq_pin_settings(rfm12_module_type_t module_type);

static int
spi_rfm12_cleanup_pinmux_settings(void)
{
   return 0;
}

static int
spi_rfm12_cleanup_irq_pin_settings(rfm12_module_type_t module_type)
{
   return 0;
}

#endif // __RFM12_PLAT_RASPBERRYPI_H__
