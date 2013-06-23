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
   This is the implementation of platform.h for normal SPI-master drivers,
   using the kernel's SPI-master interface. See platform.h for extra info.
   
   We don't use thr SPI driver's IRQ functionality, but handle it ourselves.
*/

#if !defined(__RFM12_PLAT_SPI_H__)
#define __RFM12_PLAT_SPI_H__

#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/spi/spi.h>

struct spi_rfm12_active_board {
   u16 irq;
   void* irq_data;
   struct spi_device* spi_device;
   int idx;
   struct {
      u8 gpio_claimed:1;
      u8 irq_claimed:1;
      u8 irq_enabled:1;
   } state;
};

static struct spi_rfm12_active_board active_boards[NUM_RFM12_BOARDS];

static int
spi_rfm12_init_pinmux_settings(void);
static int
spi_rfm12_cleanup_pinmux_settings(void);

static irqreturn_t
spi_rfm12_irq_handler(int irq, void* dev_id);

static int
spi_rfm12_setup_irq_pins(void);
static int
spi_rfm12_cleanup_irq_pins(void);

static int
spi_rfm12_register_spi_devices(void);
static int
spi_rfm12_deregister_spi_devices(void);

static irqreturn_t
spi_rfm12_irq_handler(int irq, void* dev_id)
{
   struct spi_rfm12_active_board* brd = (struct spi_rfm12_active_board*)dev_id;
   
   if (NULL != brd->irq_data) {   
      if (brd->state.irq_enabled) {
         brd->state.irq_enabled = 0;
         disable_irq_nosync(brd->irq);
      }
      
      rfm12_handle_interrupt((struct rfm12_data*)brd->irq_data);
   }
   
   return IRQ_HANDLED;
}

static int
platform_irq_handled(void* identifier)
{
   struct spi_rfm12_active_board* brd = (struct spi_rfm12_active_board*)identifier;
   struct spi_rfm12_board_config* cfg = &board_configs[brd->idx];
      
   if (0 == brd->state.irq_enabled) {
      if (0 == gpio_get_value(cfg->irq_pin))
         rfm12_handle_interrupt((struct rfm12_data*)brd->irq_data);
      else {
         brd->state.irq_enabled = 1;
         enable_irq(brd->irq);
      }
   }

   return 0;
}

static int
spi_rfm12_setup_irq_pins(void)
{
   int err, i;
      
   for (i=0; i<NUM_RFM12_BOARDS; i++) {
      err = gpio_request_one(board_configs[i].irq_pin, GPIOF_IN,
            RFM12B_DRV_NAME " irq pin");
      if (0 != err) {
         printk(KERN_ALERT RFM12B_DRV_NAME
            " : unable to obtain GPIO pin %u.\n",
            board_configs[i].irq_pin
         );
         
         goto errReturn;
      }
      
      active_boards[i].state.gpio_claimed = 1;
   
      err = gpio_to_irq(board_configs[i].irq_pin);
      if (err < 0) {
         printk(
            KERN_ALERT RFM12B_DRV_NAME
            " : unable to obtain IRQ for GPIO pin %u: %i.\n",
            board_configs[i].irq_pin, err
         );
         
         goto gpioErrReturn;
      }
      
      active_boards[i].irq = (u16)err;
   }
   
   err = 0;
   return err;

gpioErrReturn:
   while (i >= 0) {
      gpio_free(board_configs[i].irq_pin);
      active_boards[i].state.gpio_claimed = 0;
      i--;
   }

errReturn:   
   return err;
}

static int
spi_rfm12_cleanup_irq_pins(void)
{
   int i;
   
   for (i=0; i<NUM_RFM12_BOARDS; i++) {
      (void)platform_irq_cleanup(&active_boards[i]);
      
      if (active_boards[i].state.gpio_claimed) {
         gpio_free(board_configs[i].irq_pin);
         active_boards[i].state.gpio_claimed = 0;
      }
   }
   
   return 0;
}

static int
platform_module_init(void)
{
   int err, i;
      
   for (i=0; i<NUM_RFM12_BOARDS; i++) {
      active_boards[i].idx = i;
   }
   
   err = spi_rfm12_init_pinmux_settings();
   if (0 != err) goto muxFailed;
   
   err = spi_rfm12_setup_irq_pins();
   if (0 != err) goto irqFailed;
   
   err = spi_rfm12_register_spi_devices();
   if (0 != err) goto spiFailed;
   
   return err;

spiFailed:
   spi_rfm12_cleanup_irq_pins();
irqFailed:
   spi_rfm12_cleanup_pinmux_settings();
muxFailed:
   return err;
}

static int
platform_module_cleanup(void)
{
   (void)spi_rfm12_cleanup_pinmux_settings();
   (void)spi_rfm12_cleanup_irq_pins();
   (void)spi_rfm12_deregister_spi_devices();
   
   return 0;
}

static void*
platform_irq_identifier_for_spi_device(u16 spi_bus, u16 spi_cs)
{
   int i;
   
   for (i=0; i<NUM_RFM12_BOARDS; i++) {
      if (spi_bus == board_configs[i].spi_bus &&
         spi_cs == board_configs[i].spi_cs)
         return &active_boards[i];
   }
   
   return NULL;
}

static int
platform_irq_init(void* identifier, void* rfm12_data)
{
   int err;
   struct spi_rfm12_active_board* brd = (struct spi_rfm12_active_board*)identifier;
   struct spi_rfm12_board_config* cfg = &board_configs[brd->idx];

   if (brd->state.irq_claimed)
      return -EBUSY;

   err = request_any_context_irq(
      brd->irq,
      spi_rfm12_irq_handler,
      IRQF_TRIGGER_FALLING | IRQF_DISABLED,
      RFM12B_DRV_NAME,
      (void*)brd
   );

   if (0 <= err) {
      brd->state.irq_claimed = 1;
      brd->state.irq_enabled = 1;
      brd->irq_data = rfm12_data;      
   } else
      printk(
         KERN_ALERT RFM12B_DRV_NAME
         " : unable to activate IRQ %u: %i.\n",
         brd->irq, err
      );

   if (0 == gpio_get_value(cfg->irq_pin))
      spi_rfm12_irq_handler(brd->irq, (void*)brd);

   return err;   
}

static int
platform_irq_cleanup(void* identifier)
{
   int err = 0;   
   struct spi_rfm12_active_board* brd = (struct spi_rfm12_active_board*)identifier;
   
   if (brd->state.irq_claimed) {
      if (brd->state.irq_enabled) {
         disable_irq(brd->irq);
         brd->state.irq_enabled = 0;
      }
      
      free_irq(brd->irq, (void*)brd);
      brd->state.irq_claimed = 0;      
      brd->irq_data = NULL;
   }
   
   return err;
}

static int
spi_rfm12_register_spi_devices(void)
{
   int i, err = 0;
   struct spi_master* spi_master;
   struct spi_device* spi_device;
   struct device* sdev;
   char buf[128];
   
   for (i=0; i<NUM_RFM12_BOARDS; i++) {
      spi_master = spi_busnum_to_master(board_configs[i].spi_bus);
      if (NULL == spi_master) {
         err = -ENODEV;
         printk(
            KERN_ALERT RFM12B_DRV_NAME
               " : no spi_master found for busnum %u.\n",
               board_configs[i].spi_bus
         );
         
         goto errReturn;
      }
      
      spi_device = spi_alloc_device(spi_master);
      if (NULL == spi_device) {
         printk(
            KERN_ALERT RFM12B_DRV_NAME
               " : spi_alloc_device() failed.\n"
         );
         err = -ENOMEM;
         goto errReturn;
      }
      
      spi_device->chip_select = board_configs[i].spi_cs;
      
      snprintf(
         buf,
         sizeof(buf),
         "%s.%u", 
         dev_name(&spi_device->master->dev),
         spi_device->chip_select
      );
      
      // if a driver is already registered for our chipselect, try
      // to unregister it, and retry. fail if unregistering didn't
      // work for some reason.
      sdev = bus_find_device_by_name(spi_device->dev.bus, NULL, buf);
      if (NULL != sdev) {
         spi_unregister_device((struct spi_device*)sdev);
         spi_dev_put((struct spi_device*)sdev);
      }
      
      sdev = bus_find_device_by_name(spi_device->dev.bus, NULL, buf);
      if (NULL != sdev) {
         spi_dev_put(spi_device);

         printk(
            KERN_ALERT RFM12B_DRV_NAME
               " : driver [%s] already registered for [%s], can't "
               "unregister it\n",
               (sdev->driver && sdev->driver->name) ?
                  sdev->driver->name : "unknown",
               buf
         );

         err = -EBUSY;
         goto errReturn;
      }
      
      spi_device->max_speed_hz = RFM12B_SPI_MAX_HZ;
      spi_device->mode = RFM12B_SPI_MODE;
      spi_device->bits_per_word = RFM12B_SPI_BITS;
      spi_device->chip_select = board_configs[i].spi_cs;
      spi_device->irq = -1; /* we do our own interrupt handling */
      spi_device->controller_state = NULL;
      spi_device->controller_data = NULL;
      strlcpy(spi_device->modalias, RFM12B_DRV_NAME, SPI_NAME_SIZE);
      
      err = spi_add_device(spi_device);
      if (0 != err) {
         spi_dev_put(spi_device);
         
         printk(
            KERN_ALERT RFM12B_DRV_NAME
               " : failed to register SPI device: %i\n",
               err
         );
      } else
         active_boards[i].spi_device = spi_device;
      
      put_device(&spi_master->dev);
      spi_master = NULL;
   }
   
   return err;
   
errReturn:
   if (NULL != spi_master)
      put_device(&spi_master->dev);

   (void)spi_rfm12_deregister_spi_devices();

   return err;
}

static int
spi_rfm12_deregister_spi_devices(void)
{
   int i;
   
   for (i=0; i<NUM_RFM12_BOARDS; i++) {
      if (NULL != active_boards[i].spi_device) {
         spi_unregister_device(active_boards[i].spi_device);
         spi_dev_put(active_boards[i].spi_device);
         
         active_boards[i].spi_device = NULL;
      }
   }
   
   return 0;
}

#endif // __RFM12_PLAT_SPI_H__
