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

#if !defined(__RFM12B_CONFIG_H__)
#define __RFM12B_CONFIG_H__

/*
  The plaform you are building for. This is important, change it to
  whatever board you are using. You can further edit hardware-specific
  configuration in the following files.
  
  IMPORTANT: If you don't set this to the board you actually want to
             build for, you'll get a segfault or oops when you try to
             insert the module!
  
  BOARD             NUMBER          HARDWARE-SPECIFICS
  
  Raspberry Pi      1               platform/plat_raspberrypi.h
  Beaglebone        2               platform/plat_beaglebone.h
  Beaglebone Black  3               platform/plat_beaglebone.h
*/
#define RFM12B_BOARD        0

/*
  The name of the driver within the kernel (e.g. shows up in logs, etc...)
*/
#define RFM12B_DRV_NAME     "rfm12b"

/*
  The textual part of the device name in /dev. E.g., with the default
  setting, the device name for an RFM12b module on SPI bus 2, CS 1 would
  be /dev/rfmb12.2.1
*/
#define RFM12B_DEV_NAME     "rfm12b"

/*
  The default group ID to use for each RFM12B board. You can change the
  group ID for a particular board via ioctl() when you open the device.
  You can also pass group_id=X when loading the module.
  
  Each RFM12B needs to have the same group ID to talk to each other.
  Change this to whatever you want, (needs to be between 0 and 255).
*/
#define RFM12B_DEFAULT_GROUP_ID   211

/*
  The default frequency band to use for each RFM12B board. You can change
  the band for a particular board via ioctl() when you open the device.
  You can also pass band_id=X when loading the module.
  
  You should only use the band for which your board was built. This should
  be written at the bottom of the RFM12B board.
  
  The settings are as follows
     1 ... 433mhz
     2 ... 868mhz
*/
#define RFM12B_DEFAULT_BAND_ID   2

/*
  The default bit rate to use for each RFM12B board. You can change the
  bit rate for a particular board via ioctl() when you open the device.
  You can also pass bit_rate=X when loading the module.
  
  The bit rate determines how fast the board sends and receives data.
  All boards need to be configured to the same rate to communicate.
  
  Please refer to the datasheet for how to calculate this byte. The
  default setting is ~ 49.2kpbs, which is also what jeelib uses.
  <https://github.com/jcw/jeelib>
*/
#define RFM12B_DEFAULT_BIT_RATE  0x06

/*
  Whether Jee-compatible mode is enabled by default or not. You can
  still enable Jee-compatible mode for a board either via ioctl() or
  by passing a parameter to the module when loading it.
  
  Jee-compatible mode differs from normal operation by these traits
  
  1) The first two bytes of a packet (header and length) are passed
     to user-space via read(), so you can inspect them.
  2) When you pass data to send via write(), the first two bytes of
     the data are the Jee header and length bytes. If the length byte
     does not match the actual packet length, it will be automatically
     fixed by the driver.
  3) The driver can automatically send Jee ACKs for packets that
     request them (see RFM12B_DEFAULT_JEE_AUTOACK).
*/
#define RFM12B_DEFAULT_JEE_ID    0

/*
  Whether the driver should automatically send ACKs for packets that
  request them when in Jee-compatible mode. You can change this per
  board via ioctl() or change the default by passing a module parameter
  on load.
  
  If Jee-compatible mode is enabled (see RFM12B_DEFAULT_JEE_ID), the
  driver can automatically send ACKs for packets that request them, so
  you don't have to worry about this in your user-space program.
*/
#define RFM12B_DEFAULT_JEE_AUTOACK  1

/*
  SPI settings that the driver will use. You shouldn't need to change
  anything, though you can experiment with higher SPI frequencies -
  however, note that the RFM12b's datasheet mentions that 2.5MHz is
  the highest freq. for receiving, so experiment at your own risk!
*/

#define RFM12B_SPI_MAX_HZ    2500000
#define RFM12B_SPI_MODE      0
#define RFM12B_SPI_BITS      8

/*
  The major and number of minors for registering the rfm12 SPI driver.
  You shouldn't need to change this, unless you get an error trying to
  load the module about the SPI major already being taken. If so, change
  to a different integer.
*/

#define RFM12B_SPI_MAJOR        'r'
#define RFM12B_NUM_SPI_MINORS   32

/*
  The RFM12b has a FFOV (rx register overflow) and an RGUR (tx register
  underflow) field in the status word. Those should be set if there was
  an overflow of data while receiving (e.g. you're not reading data fast
  enough) or if you don't supply bits to be sent fast enough, respectively.
  
  However, I'm not sure if I understand those fields correctly, as disregarding
  them seems to improve reliability a lot – I need to investigate this further.
  Thus, you can enable/disable the behavior to regard a send/recv as failed
  when these bits are set in the status register.
  
  Note that even if the "drop packet" behavior is disabled, you are still
  protected against corrupted data by the internal CRC16 checksum.
  
  Tip: Leave these at defaults, unless you're actively investigating this.
*/
#define RFM12B_DROP_PACKET_ON_FFOV       1
#define RFM12B_RETRY_SEND_ON_RGUR        0

/****************************** DON'T EDIT BELOW **************************/

#if RFM12B_BOARD<=0 || RFM12B_BOARD>3
#error Please specify your board. (RFM12B_BOARD in rfm12b_config.h).
#else
#define MODULE_BOARD_CONFIGURED 1
#endif

#if defined(MODULE_BOARD_CONFIGURED)
#include <linux/version.h>

#if BUILD_MODULE
#include "platform/platform.h"

#if RFM12B_BOARD==1
#include "platform/plat_raspberrypi.h"

#elif RFM12B_BOARD==2 || RFM12B_BOARD==3
#include "platform/plat_beaglebone.h"
#endif

#include "platform/plat_spi.h"
#endif // BUILD_MODULE

#if RFM12B_BOARD==1
#define RF12_TESTS_DEV      "/dev/" RFM12B_DEV_NAME ".0.1"
#elif RFM12B_BOARD==2 || RFM12B_BOARD==3
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)
#define RF12_TESTS_DEV      "/dev/" RFM12B_DEV_NAME ".2.1"
#else
#define RF12_TESTS_DEV      "/dev/" RFM12B_DEV_NAME ".1.1"
#endif
#endif

#endif // MODULE_BOARD_CONFIGURED

#endif // __RFM12B_CONFIG_H__
