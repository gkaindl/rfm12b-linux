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
   This file contains declarations for platform-specific setup and
   management code.
   
   It probably doesn't make as much sense now anymore as it did when I
   started to structure the code like this, but originally, I also supported
   a rare x86-compatible board called "Bifferboard", which required a massive
   hack to have an IRQ-usable pin (hijacking a serial-related IRQ) and some
   tricks for fast bit-banging SPI, so it was completely different from a
   "normal" SPI-master driver. Hence the extra abstraction...
*/

#if !defined(__PLATFORM_H__)
#define __PLATFORM_H__

static int
platform_module_init(void);

static int
platform_module_cleanup(void);

static void*
platform_irq_identifier_for_spi_device(u16 spi_bus, u16 spi_cs);

static int
platform_irq_init(void* identifier, rfm12_module_type_t  module_type,
   void* rfm12_data);

static int
platform_irq_handled(void* identifier);

static int
platform_irq_cleanup(void* identifier);

struct spi_rfm12_board_config {
   u16 irq_pin;
   u16 spi_bus;
   u16 spi_cs;
};

struct rfm12_data;

static void
rfmXX_handle_interrupt(struct rfm12_data* rfm12);

#endif
