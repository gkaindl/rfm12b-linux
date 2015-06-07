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

#if !defined(__RFM12_PLAT_BEAGLEBONE_H__)
#define __RFM12_PLAT_BEAGLEBONE_H__

#include <linux/ioport.h>
#include <asm/io.h>

/*
   This file handles pinmuxing on Beaglebone and Beaglebone Black.
   
   If you want to change the default hardware settings for BBB/BB, this
   is the place to look.
*/

// the amount of RFM12 modules connected to the beaglebone
// per default, we have settings for one board. if you want
// more boards, you need to add the necessary settings below
// as well.
#define NUM_RFM12_BOARDS         1

/*
*  default config for beaglebone (one RFM12 module)
*
*  Beaglebone            RFM12B
*  ---------------------------------
*  P9/1                  GND
*  P9/3                  VDD (+3.3V)
*  P9/27                 nIRQ
*  P9/29                 SDO
*  P9/30                 SDI
*  P9/31                 SCK
*  P9/42                 nSEL
*/

struct spi_rfm12_board_config board_configs[NUM_RFM12_BOARDS] = {
   {
      .irq_pin      = 115,   // gpio3_19
      
      // on pre-device-tree kernels, SPI1 is exported as spi bus 2,
      // but on newer 3.8+ device-tree kernels, it is spi bus 1.
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)
      .spi_bus      = 2,   // spi port on beaglebone header
#else
      .spi_bus      = 1,   // spi port on beaglebone header
#endif
      .spi_cs         = 1      // CS 1
   }
};

struct am33xx_pinmux_settings {
   u32 pin_addr;
   u32 settings;
};

#define AM33XX_CONTROL_BASE      0x44e10000

// these pinmux settings will be applied when the module is loaded.
// if you want to connect multiple boards, add the definitions
// and settings for the required pins here.
static struct am33xx_pinmux_settings pinmux_settings[] = {
   {
      // beaglebone pin p9/27 (mcasp0_fsr)
      // set as GPIO3_19 (mode 7), input, pull-state defined per-board type
      .pin_addr = AM33XX_CONTROL_BASE + 0x9a4,
      .settings = 0x7 | (0 << 3) | (1 << 5) // TODO!!! PULLUP ENABLED, FUCK!!!
   },
   {
      // beaglebone pin p9/42 (ecap0_in_pwm0_out)
      // set as SPI1_CS1 (mode 2), PULLUP ENABLED, INPUT ENABLED
      .pin_addr = AM33XX_CONTROL_BASE + 0x964,
      .settings = 0x2 | (2 << 3) | (1 << 5)
   },
   {
      // beaglebone pin p9/29 (mcasp0_fsx)
      // set as SPI1_D0 (mode 3), PULLDOWN DISABLED, INPUT ENABLED
      .pin_addr = AM33XX_CONTROL_BASE + 0x994,
      .settings = 0x3 | (0 << 3) | (1 << 5)
   },
   {
      // beaglebone pin p9/30 (mcasp0_axr0)
      // set as SPI1_D1 (mode 3), PULLDOWN DISABLED, INPUT ENABLED
      .pin_addr = AM33XX_CONTROL_BASE + 0x998,
      .settings = 0x3 | (0 << 3) | (1 << 5)
   },
   {
      // beaglebone pin p9/31 (mcasp0_aclkx)
      // set as SPI1_SCLK (mode 3), PULLDOWN DISABLED, INPUT ENABLED
      .pin_addr = AM33XX_CONTROL_BASE + 0x990,
      .settings = 0x3 | (0 << 3) | (1 << 5)
   },
#if RFM12B_BOARD==3
   //  on Beaglebone Black, this pad is also connected to pin 42,
   //  so set it to input mode in order not ot have it interfere with
   //  the CS line.
   //  mode GPIO3_18 (mode 7), PULLDOWN DISABLED, INPUT ENABLED
   {
      .pin_addr = AM33XX_CONTROL_BASE + 0x9a0,
      .settings = 0x7 | (1 << 5)
   },
#endif
   { 0, 0}
};

static int
spi_rfm12_init_pinmux_settings(void);
static int
spi_rfm12_init_irq_pin_settings(rfm12_module_type_t module_type);
static int
spi_rfm12_cleanup_pinmux_settings(void);
static int
spi_rfm12_cleanup_irq_pin_settings(rfm12_module_type_t module_type);

static int
spi_rfm12_init_pinmux_settings(void)
{   
   void* addr = NULL;
   // first pinmux_setting is the IRQ pin, which is treated specially
   struct am33xx_pinmux_settings* pin_conf = &pinmux_settings[1];
   
   while (0 != pin_conf->pin_addr) {      
      addr = ioremap(pin_conf->pin_addr, 4);
      
      if (NULL == addr) {
         printk(KERN_ALERT RFM12B_DRV_NAME
            " : unable to ioremap memory address 0x%x during pin muxing.\n",
            pin_conf->pin_addr
         );
         
         (void)spi_rfm12_cleanup_pinmux_settings();
         
         return -EBUSY;
      }
      
      iowrite32(pin_conf->settings, addr);
      iounmap(addr);
      
      pin_conf++;
   }
   
   return 0;
}

static int
spi_rfm12_init_irq_pin_settings(rfm12_module_type_t module_type)
{
   void* addr = NULL;
   struct am33xx_pinmux_settings irq_conf = pinmux_settings[0];
   
   addr = ioremap(irq_conf.pin_addr, 4);
      
   if (NULL == addr) {
      printk(KERN_ALERT RFM12B_DRV_NAME
         " : unable to ioremap memory address 0x%x during irq pin muxing.\n",
         irq_conf.pin_addr
      );
      
      (void)spi_rfm12_cleanup_irq_pin_settings(module_type);
      
      return -EBUSY;
   }
   
   switch (module_type) {
      case RFM12_TYPE_RF12:
         irq_conf.settings |= (2 << 3); // pullup enabled
         break;
      case RFM12_TYPE_RF69:
         irq_conf.settings |= (1 << 3); // no pull
         break;
      default:
         break;
   }
   
   iowrite32(irq_conf.settings, addr);
   iounmap(addr);
   
   return 0;
}

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

#endif // __RFM12_PLAT_BEAGLEBONE_H__
